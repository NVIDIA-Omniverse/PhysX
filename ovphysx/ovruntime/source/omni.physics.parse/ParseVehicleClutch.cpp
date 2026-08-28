// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-VEH-DRIVETRAIN-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleClutchAPI reader. Single scalar field (strength); the
// default is 10.0f * kgmsScale. kgmsScale is walker-resolved.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-1 AC-2 AC-3
DescPtr<ClutchDesc> parseClutch(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<ClutchDesc> desc = allocateDesc<ClutchDesc>(ctx.descriptorAllocator());

    float strength;
    if (src.getAttribute(key, src.internToken("physxVehicleClutch:strength"), strength))
    {
        if (!(strength >= FLT_MIN) || !(strength < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"strength\" out of range.", ownerName.c_str());
            return {};
        }
        desc->strength = strength;
    }
    else
    {
        desc->strength = 10.0f * kgmsScale;
    }

    return desc;
}

} // namespace omni::physics::parse
