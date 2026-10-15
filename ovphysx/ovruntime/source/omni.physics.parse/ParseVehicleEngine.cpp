// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-DRIVETRAIN-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleEngineAPI reader. Reads moi / peakTorque / max- and
// idleRotationSpeed / torqueCurve (3-entry default; max 8 authored)
// / dampingRate{Full,ZeroEngaged,ZeroDisengaged} with kgms-scale
// defaults when sentinel -1 is authored. Walker pre-reads the
// torque-curve VtArray<GfVec2f> (IPhysicsSource has no float[2] array
// accessor) and passes it via EngineInfo.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-1 AC-2 AC-3
PreparedEngine prepareEngine(ParseContext& ctx, ObjectKey key, const EngineInfo& info)
{
    PreparedEngine input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleEngine:dampingRateFullThrottle", VehicleAttributeType::floating },
        { "physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged", VehicleAttributeType::floating },
        { "physxVehicleEngine:dampingRateZeroThrottleClutchEngaged", VehicleAttributeType::floating },
        { "physxVehicleEngine:idleRotationSpeed", VehicleAttributeType::floating },
        { "physxVehicleEngine:maxRotationSpeed", VehicleAttributeType::floating },
        { "physxVehicleEngine:moi", VehicleAttributeType::floating },
        { "physxVehicleEngine:peakTorque", VehicleAttributeType::floating }
    });
    return input;
}

void prepareEngineStorage(const PreparedEngine& input, EngineDesc& output)
{
    (void)input; (void)output;
}

bool fillEngine(const PreparedEngine& input, EngineDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const EngineInfo& info = input.info;


    const std::string& ownerName = src.sourceKeyToString(key);

    EngineDesc* desc = &output;
    desc->key = key;

    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleEngine:moi"), v))
        {
            if (!(v >= FLT_MIN) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"moi\" out of range.", ownerName.c_str());
                return false;
            }
            desc->moi = v;
        }
        else
        {
            desc->moi = 1.0f * info.kgmsScale;
        }
    }

    {
        float v;
        if (!src.getAttribute(key, src.attribute("physxVehicleEngine:peakTorque"), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"peakTorque\" defined.", ownerName.c_str());
            return false;
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"peakTorque\" out of range.", ownerName.c_str());
                return false;
            }
            desc->peakTorque = v;
        }
        else
        {
            desc->peakTorque = 500.0f * info.kgmsScale;
        }
    }

    auto readRequiredNonNeg = [&](const char* attrName, float& out) -> bool {
        float v;
        if (!src.getAttribute(key, src.attribute(attrName), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = v;
        return true;
    };

    if (!readRequiredNonNeg("physxVehicleEngine:maxRotationSpeed",  desc->maxRotationSpeed))  return false;
    if (!readRequiredNonNeg("physxVehicleEngine:idleRotationSpeed", desc->idleRotationSpeed)) return false;

    // torqueCurve: walker pre-read. When authored, expect 1..8 entries
    // with monotonically-increasing first dimension in [0, 1]. Default
    // (when info.hasTorqueCurve=false) is (0,0.8)(0.33,1)(1,0.8).
    if (info.hasTorqueCurve)
    {
        uint32_t cnt = info.torqueCurvePointCount;
        if (cnt == 0)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" needs to have at least 1 entry.",
                           ownerName.c_str());
            return false;
        }
        if (cnt > EngineDesc::maxNumberOfTorqueCurvePoints)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has more than %d entries. Entries will be culled.",
                           ownerName.c_str(), EngineDesc::maxNumberOfTorqueCurvePoints);
            cnt = EngineDesc::maxNumberOfTorqueCurvePoints;
        }
        float xVal = 0.0f;
        for (uint32_t i = 0; i < cnt; ++i)
        {
            if (xVal > info.torqueCurve[i].x)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                               "expects increasing values in [0, 1].", ownerName.c_str());
                return false;
            }
            desc->torqueCurve[i] = info.torqueCurve[i];
            xVal = info.torqueCurve[i].x;
        }
        if (xVal > 1.0f)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                           "expects increasing values in [0, 1].", ownerName.c_str());
            return false;
        }
        desc->torqueCurvePointCount = cnt;
    }
    else
    {
        desc->torqueCurve[0] = { 0.0f, 0.8f };
        desc->torqueCurve[1] = { 0.33f, 1.0f };
        desc->torqueCurve[2] = { 1.0f, 0.8f };
        desc->torqueCurvePointCount = 3;
    }

    auto readDampingWithKgmsFallback = [&](const char* attrName, float defaultMul, float& out) -> bool {
        float v;
        if (!src.getAttribute(key, src.attribute(attrName), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                               ownerName.c_str(), attrName);
                return false;
            }
            out = v;
        }
        else
        {
            out = defaultMul * info.kgmsScale;
        }
        return true;
    };

    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateFullThrottle",            0.15f, desc->dampingRateFullThrottle)) return false;
    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged", 2.0f,  desc->dampingRateZeroThrottleClutchEngaged)) return false;
    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged", 0.35f, desc->dampingRateZeroThrottleClutchDisengaged)) return false;

    return true;
}

void finishEngineStorage(EngineDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<EngineDesc> parseEngine(ParseContext& ctx, ObjectKey key, const EngineInfo& info)
{
    const PreparedEngine input = prepareEngine(ctx, key, info);
    DescPtr<EngineDesc> output = allocateDesc<EngineDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareEngineStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillEngine(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishEngineStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
