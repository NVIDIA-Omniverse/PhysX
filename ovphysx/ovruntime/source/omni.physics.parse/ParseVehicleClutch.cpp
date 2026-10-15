// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-DRIVETRAIN-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleClutchAPI reader. Single scalar field (strength); the
// default is 10.0f * kgmsScale. kgmsScale is walker-resolved.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-1 AC-2 AC-3
PreparedClutch prepareClutch(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    PreparedClutch input;
    input.key = key;
    input.kgmsScale = kgmsScale;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleClutch:strength", VehicleAttributeType::floating }
    });
    return input;
}

void prepareClutchStorage(const PreparedClutch& input, ClutchDesc& output)
{
    (void)input; (void)output;
}

bool fillClutch(const PreparedClutch& input, ClutchDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] float kgmsScale = input.kgmsScale;


    const std::string& ownerName = src.sourceKeyToString(key);

    ClutchDesc* desc = &output;

    float strength;
    if (src.getAttribute(key, src.attribute("physxVehicleClutch:strength"), strength))
    {
        if (!(strength >= FLT_MIN) || !(strength < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"strength\" out of range.", ownerName.c_str());
            return false;
        }
        desc->strength = strength;
    }
    else
    {
        desc->strength = 10.0f * kgmsScale;
    }

    return true;
}

void finishClutchStorage(ClutchDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<ClutchDesc> parseClutch(ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    const PreparedClutch input = prepareClutch(ctx, key, kgmsScale);
    DescPtr<ClutchDesc> output = allocateDesc<ClutchDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareClutchStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillClutch(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishClutchStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
