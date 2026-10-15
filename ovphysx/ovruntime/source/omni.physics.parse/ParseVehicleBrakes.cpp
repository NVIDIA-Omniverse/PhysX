// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleBrakesAPI (multi-apply per brake instance) reader.
// Walker drives multi-apply iteration via `forEachMultiApplyInstance`
// and calls this once per applied instance (only the `brakes0` /
// `brakes1` instance tokens are honored; walker filters).
//
// Walker pre-reads wheels (VtArray<int>) + torqueMultipliers
// (VtArray<float>) since IPhysicsSource has no array accessor.
// NonlinearCmdResponse pointer is set to nullptr; the consumer
// adapter wires it from the matching multi-apply emit.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <string>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
PreparedBrakes prepareBrakes(ParseContext& ctx, ObjectKey key, std::string_view instanceName, uint8_t brakesIndex, const BrakesInfo& info)
{
    PreparedBrakes input;
    input.key = key;
    input.instanceName = instanceName;
    input.brakesIndex = brakesIndex;
    input.info = info;
    input.instanceToken = ctx.source().internToken(instanceName);
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleBrakes:maxBrakeTorque", VehicleAttributeType::floating }
    }, instanceName);
    return input;
}

void prepareBrakesStorage(const PreparedBrakes& input, BrakesDesc& output)
{
    output.wheels.resize(input.info.wheels.size());
    output.torqueMultipliers.resize(input.info.hasTorqueMultipliers ? input.info.torqueMultipliers.size() : 0);
}

bool fillBrakes(const PreparedBrakes& input, BrakesDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const std::string& instanceName = input.instanceName;
    [[maybe_unused]] uint8_t brakesIndex = input.brakesIndex;
    [[maybe_unused]] const BrakesInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.wheels)::value_type> wheelsOut(output.wheels, vehicle_output_slot::wheels, status);
    VehicleOutputVector<typename decltype(output.torqueMultipliers)::value_type> torqueMultipliersOut(output.torqueMultipliers, vehicle_output_slot::torqueMultipliers, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    BrakesDesc* desc = &output;
    desc->brakesIndex = brakesIndex;
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit



    float mbt;
    if (!src.getAttribute(key, src.attribute("physxVehicleBrakes:maxBrakeTorque"), mbt))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"maxBrakeTorque\" defined.", ownerName.c_str());
        return false;
    }
    if (!(mbt >= 0.0f) || !(mbt < FLT_MAX))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"maxBrakeTorque\" out of range.", ownerName.c_str());
        return false;
    }
    desc->maxBrakeTorque = mbt;

    wheelsOut.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheels\" of braking system \"%s\" can not hold negative values.",
                           instanceName.data());
            return false;
        }
        wheelsOut.push_back(w);
    }

    if (info.hasTorqueMultipliers)
    {
        if (info.torqueMultipliers.size() != wheelsOut.size())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" needs to have the same "
                           "number of entries as the \"wheels\" attribute or else should not be defined at all.",
                           instanceName.data());
            return false;
        }
        torqueMultipliersOut.reserve(info.torqueMultipliers.size());
        for (float m : info.torqueMultipliers)
        {
            if (m < 0.0f)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" can not hold "
                               "negative values.", instanceName.data());
                return false;
            }
            torqueMultipliersOut.push_back(m);
        }
    }

    return true;
}

void finishBrakesStorage(BrakesDesc& output, const VehicleFillStatus& status)
{
    output.wheels.resize(status.written[vehicle_output_slot::wheels]);
    output.torqueMultipliers.resize(status.written[vehicle_output_slot::torqueMultipliers]);
}

DescPtr<BrakesDesc> parseBrakes(ParseContext& ctx, ObjectKey key, std::string_view instanceName, uint8_t brakesIndex, const BrakesInfo& info)
{
    const PreparedBrakes input = prepareBrakes(ctx, key, instanceName, brakesIndex, info);
    DescPtr<BrakesDesc> output = allocateDesc<BrakesDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareBrakesStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillBrakes(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishBrakesStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
