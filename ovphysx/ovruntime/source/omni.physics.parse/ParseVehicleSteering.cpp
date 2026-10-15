// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleSteeringAPI (single-apply) and
// PhysxVehicleAckermannSteeringAPI (single-apply) readers.
//
// The two APIs are mutually exclusive: if PhysxVehicleSteeringAPI is
// present, basic-steering parser fires; otherwise if
// AckermannSteeringAPI is applied, Ackermann parser fires.
//
// Walker pre-reads wheels (VtArray<int>) + angleMultipliers
// (VtArray<float>) for the basic variant; the Ackermann variant is
// all-scalar.
//
// NonlinearCmdResponse pointer is set to nullptr in both variants;
// the consumer adapter wires it from the matching multi-apply emit.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <cmath>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
PreparedSteeringBasic prepareSteeringBasic(ParseContext& ctx, ObjectKey key, const SteeringBasicInfo& info)
{
    PreparedSteeringBasic input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleSteering:maxSteerAngle", VehicleAttributeType::floating }
    });
    return input;
}

void prepareSteeringBasicStorage(const PreparedSteeringBasic& input, SteeringBasicDesc& output)
{
    output.wheels.resize(input.info.wheels.size());
    output.angleMultipliers.resize(input.info.hasAngleMultipliers ? input.info.angleMultipliers.size() : 0);
}

bool fillSteeringBasic(const PreparedSteeringBasic& input, SteeringBasicDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const SteeringBasicInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.wheels)::value_type> wheelsOut(output.wheels, vehicle_output_slot::wheels, status);
    VehicleOutputVector<typename decltype(output.angleMultipliers)::value_type> angleMultipliersOut(output.angleMultipliers, vehicle_output_slot::angleMultipliers, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    SteeringBasicDesc* desc = &output;
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit

    if (!src.getAttribute(key, src.attribute("physxVehicleSteering:maxSteerAngle"), desc->maxSteerAngle))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"maxSteerAngle\" defined.", ownerName.c_str());
        return false;
    }

    wheelsOut.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheels\" of steering system \"%s\" can not hold negative values.",
                           ownerName.c_str());
            return false;
        }
        wheelsOut.push_back(w);
    }

    if (info.hasAngleMultipliers)
    {
        if (info.angleMultipliers.size() != wheelsOut.size())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"angleMultipliers\" of steering system \"%s\" needs to have the same "
                           "number of entries as the \"wheels\" attribute or else should not be defined at all.",
                           ownerName.c_str());
            return false;
        }
        angleMultipliersOut.reserve(info.angleMultipliers.size());
        for (float m : info.angleMultipliers)
        {
            const float maxAngle = desc->maxSteerAngle * m;
            if (std::fabs(maxAngle) > static_cast<float>(M_PI))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"angleMultipliers\": maxSteerAngle * angleMultiplier "
                               "has to in range [-pi, pi] for all steered wheels.", ownerName.c_str());
                return false;
            }
            angleMultipliersOut.push_back(m);
        }
    }

    return true;
}

void finishSteeringBasicStorage(SteeringBasicDesc& output, const VehicleFillStatus& status)
{
    output.wheels.resize(status.written[vehicle_output_slot::wheels]);
    output.angleMultipliers.resize(status.written[vehicle_output_slot::angleMultipliers]);
}

DescPtr<SteeringBasicDesc> parseSteeringBasic(ParseContext& ctx, ObjectKey key, const SteeringBasicInfo& info)
{
    const PreparedSteeringBasic input = prepareSteeringBasic(ctx, key, info);
    DescPtr<SteeringBasicDesc> output = allocateDesc<SteeringBasicDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareSteeringBasicStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillSteeringBasic(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishSteeringBasicStorage(*output, status);
    return output;
}


// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
PreparedSteeringAckermann prepareSteeringAckermann(ParseContext& ctx, ObjectKey key)
{
    PreparedSteeringAckermann input;
    input.key = key;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleAckermannSteering:maxSteerAngle", VehicleAttributeType::floating },
        { "physxVehicleAckermannSteering:strength", VehicleAttributeType::floating },
        { "physxVehicleAckermannSteering:trackWidth", VehicleAttributeType::floating },
        { "physxVehicleAckermannSteering:wheel0", VehicleAttributeType::integer },
        { "physxVehicleAckermannSteering:wheel1", VehicleAttributeType::integer },
        { "physxVehicleAckermannSteering:wheelBase", VehicleAttributeType::floating }
    });
    return input;
}

void prepareSteeringAckermannStorage(const PreparedSteeringAckermann& input, SteeringAckermannDesc& output)
{
    (void)input; (void)output;
}

bool fillSteeringAckermann(const PreparedSteeringAckermann& input, SteeringAckermannDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;


    const std::string& ownerName = src.sourceKeyToString(key);

    SteeringAckermannDesc* desc = &output;
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit

    auto readRequiredFloat = [&](const char* attrName, float& out) -> bool {
        if (!src.getAttribute(key, src.attribute(attrName), out))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        return true;
    };

    auto readRequiredInt = [&](const char* attrName, int& out) -> bool {
        int64_t iv;
        if (!src.getAttribute(key, src.attribute(attrName), iv))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = static_cast<int>(iv);
        return true;
    };

    if (!readRequiredFloat("physxVehicleAckermannSteering:maxSteerAngle", desc->maxSteerAngle)) return false;

    if (!readRequiredInt("physxVehicleAckermannSteering:wheel0", desc->wheel0)) return false;
    if (desc->wheel0 < 0)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"wheel0\" needs to be non-negative.", ownerName.c_str());
        return false;
    }

    if (!readRequiredInt("physxVehicleAckermannSteering:wheel1", desc->wheel1)) return false;
    if (desc->wheel1 < 0)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"wheel1\" needs to be non-negative.", ownerName.c_str());
        return false;
    }

    if (desc->wheel0 == desc->wheel1)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attributes \"wheel0\" and \"wheel1\" can not hold the "
                       "same value.", ownerName.c_str());
        return false;
    }

    if (!readRequiredFloat("physxVehicleAckermannSteering:wheelBase", desc->wheelBase)) return false;
    if (!(desc->wheelBase > 0.0f))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"wheelBase\" needs to be positive.", ownerName.c_str());
        return false;
    }

    if (!readRequiredFloat("physxVehicleAckermannSteering:trackWidth", desc->trackWidth)) return false;
    if (!(desc->trackWidth > 0.0f))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"trackWidth\" needs to be positive.", ownerName.c_str());
        return false;
    }

    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleAckermannSteering:strength"), v))
        {
            if (!(v >= 0.0f) || !(v < 1.0f + FLT_EPSILON))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"strength\" out of range.", ownerName.c_str());
                return false;
            }
            desc->strength = v;
        }
        else
        {
            desc->strength = 1.0f;
        }
    }

    return true;
}

void finishSteeringAckermannStorage(SteeringAckermannDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<SteeringAckermannDesc> parseSteeringAckermann(ParseContext& ctx, ObjectKey key)
{
    const PreparedSteeringAckermann input = prepareSteeringAckermann(ctx, key);
    DescPtr<SteeringAckermannDesc> output = allocateDesc<SteeringAckermannDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareSteeringAckermannStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillSteeringAckermann(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishSteeringAckermannStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
