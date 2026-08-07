// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

DescPtr<DriveBasicDesc> parseDriveBasic(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<DriveBasicDesc> desc = allocateDesc<DriveBasicDesc>(ctx.descriptorAllocator());
    desc->key = key;
    desc->nonlinearCmdResponse = nullptr;  // consumer adapter wires this from multi-apply NonlinearCmdResponseAPI

    float peakTorque;
    if (!src.getAttribute(key, src.internToken("physxVehicleDriveBasic:peakTorque"), peakTorque))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"peakTorque\" defined.", ownerName.c_str());
        return {};
    }
    if (peakTorque != -1.0f)
    {
        if (!(peakTorque >= 0.0f) || !(peakTorque < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"peakTorque\" out of range.", ownerName.c_str());
            return {};
        }
        desc->peakTorque = peakTorque;
    }
    else
    {
        desc->peakTorque = 1000.0f * kgmsScale;
    }

    return desc;
}

} // namespace omni::physics::parse
