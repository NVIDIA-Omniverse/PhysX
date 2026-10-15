// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-DRIVETRAIN-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleGearsAPI reader. ratios is a VtArray<float>: walker
// pre-reads it (IPhysicsSource has no float-array accessor) and
// passes via GearsInfo. ratioScale + switchTime are scalar reads
// done in the parse-lib.

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
PreparedGears prepareGears(ParseContext& ctx, ObjectKey key, const GearsInfo& info)
{
    PreparedGears input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleGears:ratioScale", VehicleAttributeType::floating },
        { "physxVehicleGears:switchTime", VehicleAttributeType::floating }
    });
    return input;
}

void prepareGearsStorage(const PreparedGears& input, GearsDesc& output)
{
    output.ratios.resize(input.info.hasRatios ? input.info.ratios.size() : 6);
}

bool fillGears(const PreparedGears& input, GearsDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const GearsInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.ratios)::value_type> ratiosOut(output.ratios, vehicle_output_slot::ratios, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    GearsDesc* desc = &output;

    // ratios: walker pre-read. Max one reverse + (maxNumberOfGears - 2)
    // forward gears. First entry is reverse (must be negative).
    // Subsequent entries are forward gears (must be positive, strictly
    // descending). Default (when info.hasRatios=false) is the 6-entry
    // built-in default.
    constexpr uint32_t maxForwardGearCount = GearsDesc::maxNumberOfGears - 2;
    if (info.hasRatios)
    {
        if (info.ratios.empty())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"ratios\" of gears \"%s\" needs to have at least 1 entry.",
                           ownerName.c_str());
            return false;
        }
        if (info.ratios.size() > (maxForwardGearCount + 1))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"ratios\" of gears \"%s\" only supports one reverse and %d forward gears.",
                           ownerName.c_str(), maxForwardGearCount);
            return false;
        }
        ratiosOut.reserve(info.ratios.size());
        if (info.ratios[0] < 0.0f)
            ratiosOut.push_back(info.ratios[0]);
        else
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: first entry for "
                           "reverse gear ratio needs to be negative.", ownerName.c_str());
            return false;
        }
        float previousRatio = FLT_MAX;
        for (size_t i = 1; i < info.ratios.size(); ++i)
        {
            const float r = info.ratios[i];
            if (r > 0.0f && r < previousRatio)
            {
                ratiosOut.push_back(r);
                previousRatio = r;
            }
            else
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: entries for "
                               "forward gear ratios need to be positive and form a descending sequence.", ownerName.c_str());
                return false;
            }
        }
    }
    else
    {
        ratiosOut = { -4.0f, 4.0f, 2.0f, 1.5f, 1.1f, 1.0f };
    }

    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleGears:ratioScale"), v))
        {
            if (!(v >= FLT_MIN) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"ratioScale\" out of range.", ownerName.c_str());
                return false;
            }
            desc->ratioScale = v;
        }
        else
        {
            desc->ratioScale = 4.0f;
        }
    }

    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicleGears:switchTime"), v))
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"switchTime\" out of range.", ownerName.c_str());
                return false;
            }
            desc->switchTime = v;
        }
        else
        {
            desc->switchTime = 0.5f;
        }
    }

    return true;
}

void finishGearsStorage(GearsDesc& output, const VehicleFillStatus& status)
{
    output.ratios.resize(status.written[vehicle_output_slot::ratios]);
}

DescPtr<GearsDesc> parseGears(ParseContext& ctx, ObjectKey key, const GearsInfo& info)
{
    const PreparedGears input = prepareGears(ctx, key, info);
    DescPtr<GearsDesc> output = allocateDesc<GearsDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareGearsStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillGears(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishGearsStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
