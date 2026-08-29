// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-1 AC-2 AC-3
DescPtr<EngineDesc> parseEngine(ParseContext& ctx, ObjectKey key, const EngineInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<EngineDesc> desc = allocateDesc<EngineDesc>(ctx.descriptorAllocator());
    desc->key = key;

    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicleEngine:moi"), v))
        {
            if (!(v >= FLT_MIN) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"moi\" out of range.", ownerName.c_str());
                return {};
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
        if (!src.getAttribute(key, src.internToken("physxVehicleEngine:peakTorque"), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"peakTorque\" defined.", ownerName.c_str());
            return {};
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"peakTorque\" out of range.", ownerName.c_str());
                return {};
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
        if (!src.getAttribute(key, src.internToken(attrName), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = v;
        return true;
    };

    if (!readRequiredNonNeg("physxVehicleEngine:maxRotationSpeed",  desc->maxRotationSpeed))  return {};
    if (!readRequiredNonNeg("physxVehicleEngine:idleRotationSpeed", desc->idleRotationSpeed)) return {};

    // torqueCurve: walker pre-read. When authored, expect 1..8 entries
    // with monotonically-increasing first dimension in [0, 1]. Default
    // (when info.hasTorqueCurve=false) is (0,0.8)(0.33,1)(1,0.8).
    if (info.hasTorqueCurve)
    {
        uint32_t cnt = info.torqueCurvePointCount;
        if (cnt == 0)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueCurve\" of engine \"%s\" needs to have at least 1 entry.",
                           ownerName.c_str());
            return {};
        }
        if (cnt > EngineDesc::maxNumberOfTorqueCurvePoints)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has more than %d entries. Entries will be culled.",
                           ownerName.c_str(), EngineDesc::maxNumberOfTorqueCurvePoints);
            cnt = EngineDesc::maxNumberOfTorqueCurvePoints;
        }
        float xVal = 0.0f;
        for (uint32_t i = 0; i < cnt; ++i)
        {
            if (xVal > info.torqueCurve[i].x)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                               "expects increasing values in [0, 1].", ownerName.c_str());
                return {};
            }
            desc->torqueCurve[i] = info.torqueCurve[i];
            xVal = info.torqueCurve[i].x;
        }
        if (xVal > 1.0f)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                           "expects increasing values in [0, 1].", ownerName.c_str());
            return {};
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
        if (!src.getAttribute(key, src.internToken(attrName), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
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

    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateFullThrottle",            0.15f, desc->dampingRateFullThrottle)) return {};
    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged", 2.0f,  desc->dampingRateZeroThrottleClutchEngaged)) return {};
    if (!readDampingWithKgmsFallback("physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged", 0.35f, desc->dampingRateZeroThrottleClutchDisengaged)) return {};

    return desc;
}

} // namespace omni::physics::parse
