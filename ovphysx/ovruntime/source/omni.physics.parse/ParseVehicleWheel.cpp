// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
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

bool readNonNegFloat(PreparedVehicleReader& src, ObjectKey key, const char* attrName,
                     std::string_view ownerName, float& out, VehicleFillStatus& status)
{
    float v;
    if (!src.getAttribute(key, src.attribute(attrName), v))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       ownerName.data(), attrName);
        return false;
    }
    if (!(v >= 0.0f) || !(v < FLT_MAX))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [0, FLT_MAX).",
                       ownerName.data(), attrName);
        return false;
    }
    out = v;
    return true;
}

// Reads a deprecated half-pi angle. `false` means the attribute HAS a value and that value
// is out of range -- the one condition that invalidates the wheel. A read that yields no
// value leaves `out` at its default and is NOT an error: the callers gate on
// hasAuthoredAttribute, and a backend that answers resolved columns cannot narrow that
// question (ADR-0020), so "gate says authored" does not imply "there is a value here".
// Treating the missing value as fatal dropped the entire WheelDesc, which then failed the
// wheel attachment's required-reference check and invalidated the whole vehicle -- an
// unauthored deprecated attribute silently deleting the vehicle that does not use it.
// `outPresent` reports whether a value was actually read, so the caller only emits the
// deprecation warning when the user really did author one.
bool readAngleHalfPi(PreparedVehicleReader& src, ObjectKey key, const char* attrName,
                     std::string_view ownerName, float& out, bool& outPresent, VehicleFillStatus& status)
{
    outPresent = false;
    float v;
    if (!src.getAttribute(key, src.attribute(attrName), v))
        return true;
    const float lower = static_cast<float>(-M_PI_2) + FLT_MIN;
    const float upper = static_cast<float>(M_PI_2);
    if (!(v >= lower) || !(v < upper))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                       ownerName.data(), attrName);
        return false;
    }
    out = v;
    outPresent = true;
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
PreparedWheel prepareWheel(ParseContext& ctx, ObjectKey key)
{
    PreparedWheel input;
    input.key = key;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleWheel:dampingRate", VehicleAttributeType::floating },
        { "physxVehicleWheel:mass", VehicleAttributeType::floating },
        { "physxVehicleWheel:maxBrakeTorque", VehicleAttributeType::floating },
        { "physxVehicleWheel:maxHandBrakeTorque", VehicleAttributeType::floating },
        { "physxVehicleWheel:maxSteerAngle", VehicleAttributeType::floating, true },
        { "physxVehicleWheel:moi", VehicleAttributeType::floating },
        { "physxVehicleWheel:radius", VehicleAttributeType::floating },
        { "physxVehicleWheel:toeAngle", VehicleAttributeType::floating, true },
        { "physxVehicleWheel:width", VehicleAttributeType::floating }
    });
    return input;
}

void prepareWheelStorage(const PreparedWheel& input, WheelDesc& output)
{
    (void)input; (void)output;
}

bool fillWheel(const PreparedWheel& input, WheelDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;


    const std::string& ownerName = src.sourceKeyToString(key);

    WheelDesc* desc = &output;
    desc->key = key;
    desc->maxBrakeTorque = 0.0f;
    desc->maxHandBrakeTorque = 0.0f;
    desc->maxSteerAngle = 0.0f;
    desc->toeAngle = 0.0f;

    if (!readNonNegFloat(src, key, "physxVehicleWheel:radius", ownerName, desc->radius, status)) return false;
    if (!readNonNegFloat(src, key, "physxVehicleWheel:width", ownerName, desc->width, status)) return false;
    if (!readNonNegFloat(src, key, "physxVehicleWheel:mass", ownerName, desc->mass, status)) return false;
    if (!readNonNegFloat(src, key, "physxVehicleWheel:moi", ownerName, desc->moi, status)) return false;
    if (!readNonNegFloat(src, key, "physxVehicleWheel:dampingRate", ownerName, desc->dampingRate, status)) return false;

    // Deprecated attributes — read only when authored, emit deprecation
    // warning + range check.
    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleWheel:maxBrakeTorque"), v))
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: wheel \"%s\": attribute \"maxBrakeTorque\" is deprecated. "
                          "Please use PhysxVehicleBrakesAPI instead.", ownerName.c_str());
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"maxBrakeTorque\" out of range.",
                               ownerName.c_str());
                return false;
            }
            desc->maxBrakeTorque = v;
        }
    }
    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleWheel:maxHandBrakeTorque"), v))
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: wheel \"%s\": attribute \"maxHandBrakeTorque\" is deprecated. "
                          "Please use PhysxVehicleBrakesAPI instead.", ownerName.c_str());
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"maxHandBrakeTorque\" out of range.",
                               ownerName.c_str());
                return false;
            }
            desc->maxHandBrakeTorque = v;
        }
    }
    if (src.hasAuthoredAttribute(key, src.attribute("physxVehicleWheel:maxSteerAngle")))
    {
        bool present = false;
        if (!readAngleHalfPi(src, key, "physxVehicleWheel:maxSteerAngle", ownerName, desc->maxSteerAngle, present, status))
            return false;
        if (present)
            VEHICLE_LOG_WARN(status, "Usd Physics: wheel \"%s\": attribute \"maxSteerAngle\" is deprecated. "
                          "Please use PhysxVehicleSteeringAPI instead.", ownerName.c_str());
    }
    if (src.hasAuthoredAttribute(key, src.attribute("physxVehicleWheel:toeAngle")))
    {
        bool present = false;
        if (!readAngleHalfPi(src, key, "physxVehicleWheel:toeAngle", ownerName, desc->toeAngle, present, status))
            return false;
        if (present)
            VEHICLE_LOG_WARN(status, "Usd Physics: wheel \"%s\": attribute \"toeAngle\" is deprecated. "
                          "Please use PhysxVehicleSuspensionComplianceAPI instead.", ownerName.c_str());
    }

    return true;
}

void finishWheelStorage(WheelDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<WheelDesc> parseWheel(ParseContext& ctx, ObjectKey key)
{
    const PreparedWheel input = prepareWheel(ctx, key);
    DescPtr<WheelDesc> output = allocateDesc<WheelDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareWheelStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillWheel(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishWheelStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
