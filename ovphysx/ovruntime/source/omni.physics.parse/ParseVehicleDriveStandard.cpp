// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
DescPtr<DriveStandardDesc> parseDriveStandard(ParseContext& ctx, ObjectKey /*key*/,
                                              const DriveStandardInfo& /*info*/)
{
    DescPtr<DriveStandardDesc> desc = allocateDesc<DriveStandardDesc>(ctx.descriptorAllocator());
    desc->engine = nullptr;
    desc->gears  = nullptr;
    desc->autoGearBox = nullptr;
    desc->clutch = nullptr;
    desc->engineId = kInvalidObjectId;
    return desc;
}

} // namespace omni::physics::parse
