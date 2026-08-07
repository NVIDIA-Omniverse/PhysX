// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

bool readNonNegFloatRequired(IPhysicsSource& src, ObjectKey key, const char* attrName,
                             const std::string& owner, float& out)
{
    float v;
    if (!src.getAttribute(key, src.internToken(attrName), v))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       owner.c_str(), attrName);
        return false;
    }
    if (!(v >= 0.0f) || !(v < FLT_MAX))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                       owner.c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

bool readCamberAngle(IPhysicsSource& src, ObjectKey key, const char* attrName,
                     const std::string& owner, float& out)
{
    float v;
    if (!src.getAttribute(key, src.internToken(attrName), v))
    {
        out = 0.0f;
        return true;
    }
    CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"%s\" is deprecated. "
                  "Please use PhysxVehicleSuspensionComplianceAPI instead.", owner.c_str(), attrName);
    const float lower = -static_cast<float>(M_PI_2);
    const float upper = static_cast<float>(M_PI_2) + FLT_MIN;
    if (!(v >= lower) || !(v < upper))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.", owner.c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
DescPtr<SuspensionDesc> parseSuspension(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<SuspensionDesc> desc = allocateDesc<SuspensionDesc>(ctx.descriptorAllocator());
    desc->key = key;
    desc->travelDistance = 0.0f;
    desc->maxCompression = 0.0f;
    desc->maxDroop = 0.0f;
    desc->camberAtRest = 0.0f;
    desc->camberAtMaxCompression = 0.0f;
    desc->camberAtMaxDroop = 0.0f;

    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:springStrength", ownerName, desc->springStrength)) return {};
    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:springDamperRate", ownerName, desc->springDamperRate)) return {};

    // travelDistance preferred; deprecated maxCompression + maxDroop fallback.
    {
        float td;
        if (src.getAttribute(key, src.internToken("physxVehicleSuspension:travelDistance"), td))
        {
            if (!(td > 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"travelDistance\" needs to be positive.",
                               ownerName.c_str());
                return {};
            }
            desc->travelDistance = td;
            desc->maxDroop = 0.0f;
            desc->maxCompression = 0.0f;
        }
        else
        {
            desc->travelDistance = -1.0f;
            float mc;
            if (!src.getAttribute(key, src.internToken("physxVehicleSuspension:maxCompression"), mc))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"maxCompression\" defined.",
                               ownerName.c_str());
                return {};
            }
            CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"maxCompression\" is deprecated. "
                          "Please use travelDistance instead.", ownerName.c_str());
            if (!(mc >= 0.0f) || !(mc < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"maxCompression\" out of range.",
                               ownerName.c_str());
                return {};
            }
            desc->maxCompression = mc;
            float md;
            if (!src.getAttribute(key, src.internToken("physxVehicleSuspension:maxDroop"), md))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"maxDroop\" defined.",
                               ownerName.c_str());
                return {};
            }
            CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"maxDroop\" is deprecated. "
                          "Please use travelDistance instead.", ownerName.c_str());
            desc->maxDroop = md;  // negative values legal — auto-compute marker
        }
    }

    if (!readNonNegFloatRequired(src, key, "physxVehicleSuspension:sprungMass", ownerName, desc->sprungMass)) return {};

    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtRest", ownerName, desc->camberAtRest)) return {};
    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtMaxCompression", ownerName, desc->camberAtMaxCompression)) return {};
    if (!readCamberAngle(src, key, "physxVehicleSuspension:camberAtMaxDroop", ownerName, desc->camberAtMaxDroop)) return {};

    return desc;
}

} // namespace omni::physics::parse
