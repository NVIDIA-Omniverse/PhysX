// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-DRIVE-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleDriveBasicAPI reader. Single scalar (peakTorque) with
// kgms-scale default at sentinel -1. NonlinearCmdResponse is multi-
// apply (per command); the consumer adapter wires the pointer from
// the matching multi-apply emit.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

PreparedDriveBasic prepareDriveBasic(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    PreparedDriveBasic input;
    input.key = key;
    input.kgmsScale = kgmsScale;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleDriveBasic:peakTorque", VehicleAttributeType::floating }
    });
    return input;
}

void prepareDriveBasicStorage(const PreparedDriveBasic& input, DriveBasicDesc& output)
{
    (void)input; (void)output;
}

bool fillDriveBasic(const PreparedDriveBasic& input, DriveBasicDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] float kgmsScale = input.kgmsScale;


    const std::string& ownerName = src.sourceKeyToString(key);

    DriveBasicDesc* desc = &output;
    desc->key = key;
    desc->nonlinearCmdResponse = nullptr;  // consumer adapter wires this from multi-apply NonlinearCmdResponseAPI

    float peakTorque;
    if (!src.getAttribute(key, src.attribute("physxVehicleDriveBasic:peakTorque"), peakTorque))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"peakTorque\" defined.", ownerName.c_str());
        return false;
    }
    if (peakTorque != -1.0f)
    {
        if (!(peakTorque >= 0.0f) || !(peakTorque < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"peakTorque\" out of range.", ownerName.c_str());
            return false;
        }
        desc->peakTorque = peakTorque;
    }
    else
    {
        desc->peakTorque = 1000.0f * kgmsScale;
    }

    return true;
}

void finishDriveBasicStorage(DriveBasicDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<DriveBasicDesc> parseDriveBasic(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    const PreparedDriveBasic input = prepareDriveBasic(ctx, key, kgmsScale);
    DescPtr<DriveBasicDesc> output = allocateDesc<DriveBasicDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareDriveBasicStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillDriveBasic(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishDriveBasicStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
