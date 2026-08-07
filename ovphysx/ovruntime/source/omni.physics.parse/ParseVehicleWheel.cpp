// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-VEH-COMPONENTS-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleWheelAPI reader. Per-component parser for the
// shareable wheel descriptor (mass / moi / radius / width /
// dampingRate + deprecated maxBrakeTorque / maxHandBrakeTorque /
// maxSteerAngle / toeAngle). Returns nullptr on validation failure.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cmath>
#include <cfloat>

#ifndef M_PI_2
#  define M_PI_2 1.57079632679489661923
#endif

namespace omni::physics::parse
{

namespace
{

bool readNonNegFloat(IPhysicsSource& src, ObjectKey key, const char* attrName,
                     std::string_view ownerName, float& out)
{
    float v;
    if (!src.getAttribute(key, src.internToken(attrName), v))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       std::string(ownerName).c_str(), attrName);
        return false;
    }
    if (!(v >= 0.0f) || !(v < FLT_MAX))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [0, FLT_MAX).",
                       std::string(ownerName).c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

bool readAngleHalfPi(IPhysicsSource& src, ObjectKey key, const char* attrName,
                     std::string_view ownerName, float& out)
{
    float v;
    if (!src.getAttribute(key, src.internToken(attrName), v))
        return false;
    const float lower = static_cast<float>(-M_PI_2) + FLT_MIN;
    const float upper = static_cast<float>(M_PI_2);
    if (!(v >= lower) || !(v < upper))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                       std::string(ownerName).c_str(), attrName);
        return false;
    }
    out = v;
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
DescPtr<WheelDesc> parseWheel(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<WheelDesc> desc = allocateDesc<WheelDesc>(ctx.descriptorAllocator());
    desc->key = key;
    desc->maxBrakeTorque = 0.0f;
    desc->maxHandBrakeTorque = 0.0f;
    desc->maxSteerAngle = 0.0f;
    desc->toeAngle = 0.0f;

    if (!readNonNegFloat(src, key, "physxVehicleWheel:radius", ownerName, desc->radius)) return {};
    if (!readNonNegFloat(src, key, "physxVehicleWheel:width", ownerName, desc->width)) return {};
    if (!readNonNegFloat(src, key, "physxVehicleWheel:mass", ownerName, desc->mass)) return {};
    if (!readNonNegFloat(src, key, "physxVehicleWheel:moi", ownerName, desc->moi)) return {};
    if (!readNonNegFloat(src, key, "physxVehicleWheel:dampingRate", ownerName, desc->dampingRate)) return {};

    // Deprecated attributes — read only when authored, emit deprecation
    // warning + range check.
    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicleWheel:maxBrakeTorque"), v))
        {
            CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxBrakeTorque\" is deprecated. "
                          "Please use PhysxVehicleBrakesAPI instead.", ownerName.c_str());
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"maxBrakeTorque\" out of range.",
                               ownerName.c_str());
                return {};
            }
            desc->maxBrakeTorque = v;
        }
    }
    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicleWheel:maxHandBrakeTorque"), v))
        {
            CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxHandBrakeTorque\" is deprecated. "
                          "Please use PhysxVehicleBrakesAPI instead.", ownerName.c_str());
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"maxHandBrakeTorque\" out of range.",
                               ownerName.c_str());
                return {};
            }
            desc->maxHandBrakeTorque = v;
        }
    }
    if (src.hasAuthoredAttribute(key, src.internToken("physxVehicleWheel:maxSteerAngle")))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxSteerAngle\" is deprecated. "
                      "Please use PhysxVehicleSteeringAPI instead.", ownerName.c_str());
        if (!readAngleHalfPi(src, key, "physxVehicleWheel:maxSteerAngle", ownerName, desc->maxSteerAngle))
            return {};
    }
    if (src.hasAuthoredAttribute(key, src.internToken("physxVehicleWheel:toeAngle")))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"toeAngle\" is deprecated. "
                      "Please use PhysxVehicleSuspensionComplianceAPI instead.", ownerName.c_str());
        if (!readAngleHalfPi(src, key, "physxVehicleWheel:toeAngle", ownerName, desc->toeAngle))
            return {};
    }

    return desc;
}

} // namespace omni::physics::parse
