// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-ROOT-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleDriveStandardAPI reader. All of DriveStandardDesc's
// fields are cross-reference pointers into the tracker's owned
// component descriptors (Engine / Gears / AutoGearBox / Clutch /
// NonlinearCmdResponse). The walker pre-resolves the four rel-or-API
// cross-refs to ObjectKey paths via DriveStandardInfo; the consumer
// adapter resolves them to engine-side descriptor pointers at
// processScannedDescs time. Parse-lib here just allocates and
// returns; no scalar reads.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
PreparedDriveStandard prepareDriveStandard(ParseContext& ctx, ObjectKey key, const DriveStandardInfo& info)
{
    PreparedDriveStandard input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    return input;
}

void prepareDriveStandardStorage(const PreparedDriveStandard& input, DriveStandardDesc& output)
{
    (void)input; (void)output;
}

bool fillDriveStandard(const PreparedDriveStandard& input, DriveStandardDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const DriveStandardInfo& info = input.info;

    DriveStandardDesc* desc = &output;
    desc->engine = nullptr;
    desc->gears  = nullptr;
    desc->autoGearBox = nullptr;
    desc->clutch = nullptr;
    desc->engineId = kInvalidObjectId;
    return true;
}

void finishDriveStandardStorage(DriveStandardDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<DriveStandardDesc> parseDriveStandard(ParseContext& ctx, ObjectKey key, const DriveStandardInfo& info)
{
    const PreparedDriveStandard input = prepareDriveStandard(ctx, key, info);
    DescPtr<DriveStandardDesc> output = allocateDesc<DriveStandardDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareDriveStandardStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillDriveStandard(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishDriveStandardStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
