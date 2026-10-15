// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-COMPONENTS-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleSuspensionAPI reader. Per-component parser for the
// shareable suspension descriptor (spring strength/damper + travel
// distance / max compression+droop fallback + sprung mass). The
// deprecated camberAtRest / camberAtMaxCompression / camberAtMaxDroop
// angles are read with half-pi range checks.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <cmath>

#ifndef M_PI_2
#  define M_PI_2 1.57079632679489661923
#endif

namespace omni::physics::parse
{

namespace
{

bool readNonNegFloatRequired(PreparedVehicleReader& src, ObjectKey key, const char* attrName,
                             const std::string& owner, float& out, VehicleFillStatus& status)
{
    float v;
    if (!src.getAttribute(key, src.attribute(attrName), v))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       owner.c_str(), attrName);
        return false;
    }
    if (!(v >= 0.0f) || !(v < FLT_MAX))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                       owner.c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

bool readCamberAngle(PreparedVehicleReader& src, ObjectKey key, const char* attrName,
                     const std::string& owner, float& out, VehicleFillStatus& status)
{
    float v;
    if (!src.getAttribute(key, src.attribute(attrName), v))
    {
        out = 0.0f;
        return true;
    }
    VEHICLE_LOG_WARN(status, "Usd Physics: suspension \"%s\": attribute \"%s\" is deprecated. "
                  "Please use PhysxVehicleSuspensionComplianceAPI instead.", owner.c_str(), attrName);
    const float lower = -static_cast<float>(M_PI_2);
    const float upper = static_cast<float>(M_PI_2) + FLT_MIN;
    if (!(v >= lower) || !(v < upper))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.", owner.c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
PreparedSuspension prepareSuspension(ParseContext& ctx, ObjectKey key)
{
    PreparedSuspension input;
    input.key = key;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleSuspension:camberAtMaxCompression", VehicleAttributeType::floating },
        { "physxVehicleSuspension:camberAtMaxDroop", VehicleAttributeType::floating },
        { "physxVehicleSuspension:camberAtRest", VehicleAttributeType::floating },
        { "physxVehicleSuspension:maxCompression", VehicleAttributeType::floating },
        { "physxVehicleSuspension:maxDroop", VehicleAttributeType::floating },
        { "physxVehicleSuspension:springDamperRate", VehicleAttributeType::floating },
        { "physxVehicleSuspension:springStrength", VehicleAttributeType::floating },
        { "physxVehicleSuspension:sprungMass", VehicleAttributeType::floating },
        { "physxVehicleSuspension:travelDistance", VehicleAttributeType::floating }
    });
    return input;
}

void prepareSuspensionStorage(const PreparedSuspension& input, SuspensionDesc& output)
{
    (void)input; (void)output;
}

bool fillSuspension(const PreparedSuspension& input, SuspensionDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;


    const std::string& ownerName = src.sourceKeyToString(key);

    SuspensionDesc* desc = &output;
    desc->key = key;
    desc->travelDistance = 0.0f;
    desc->maxCompression = 0.0f;
    desc->maxDroop = 0.0f;
    desc->camberAtRest = 0.0f;
    desc->camberAtMaxCompression = 0.0f;
    desc->camberAtMaxDroop = 0.0f;

    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:springStrength", ownerName, desc->springStrength, status)) return false;
    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:springDamperRate", ownerName, desc->springDamperRate, status)) return false;

    // travelDistance preferred; deprecated maxCompression + maxDroop fallback.
    {
        float td;
        if (src.getAttribute(key, src.attribute("physxVehicleSuspension:travelDistance"), td))
        {
            if (!(td > 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"travelDistance\" needs to be positive.",
                               ownerName.c_str());
                return false;
            }
            desc->travelDistance = td;
            desc->maxDroop = 0.0f;
            desc->maxCompression = 0.0f;
        }
        else
        {
            desc->travelDistance = -1.0f;
            float mc;
            if (!src.getAttribute(key, src.attribute("physxVehicleSuspension:maxCompression"), mc))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"maxCompression\" defined.",
                               ownerName.c_str());
                return false;
            }
            VEHICLE_LOG_WARN(status, "Usd Physics: suspension \"%s\": attribute \"maxCompression\" is deprecated. "
                          "Please use travelDistance instead.", ownerName.c_str());
            if (!(mc >= 0.0f) || !(mc < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"maxCompression\" out of range.",
                               ownerName.c_str());
                return false;
            }
            desc->maxCompression = mc;
            float md;
            if (!src.getAttribute(key, src.attribute("physxVehicleSuspension:maxDroop"), md))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"maxDroop\" defined.",
                               ownerName.c_str());
                return false;
            }
            VEHICLE_LOG_WARN(status, "Usd Physics: suspension \"%s\": attribute \"maxDroop\" is deprecated. "
                          "Please use travelDistance instead.", ownerName.c_str());
            desc->maxDroop = md;  // negative values legal — auto-compute marker
        }
    }

    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:sprungMass", ownerName, desc->sprungMass, status)) return false;

    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtRest", ownerName, desc->camberAtRest, status)) return false;
    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtMaxCompression", ownerName, desc->camberAtMaxCompression, status)) return false;
    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtMaxDroop", ownerName, desc->camberAtMaxDroop, status)) return false;

    return true;
}

void finishSuspensionStorage(SuspensionDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<SuspensionDesc> parseSuspension(ParseContext& ctx, ObjectKey key)
{
    const PreparedSuspension input = prepareSuspension(ctx, key);
    DescPtr<SuspensionDesc> output = allocateDesc<SuspensionDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareSuspensionStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillSuspension(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishSuspensionStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
