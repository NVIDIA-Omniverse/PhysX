// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-1 AC-2 AC-3
DescPtr<GearsDesc> parseGears(ParseContext& ctx, ObjectKey key, const GearsInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<GearsDesc> desc = allocateDesc<GearsDesc>(ctx.descriptorAllocator());

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
            CARB_LOG_ERROR("Usd Physics: attribute \"ratios\" of gears \"%s\" needs to have at least 1 entry.",
                           ownerName.c_str());
            return {};
        }
        if (info.ratios.size() > (maxForwardGearCount + 1))
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"ratios\" of gears \"%s\" only supports one reverse and %d forward gears.",
                           ownerName.c_str(), maxForwardGearCount);
            return {};
        }
        desc->ratios.reserve(info.ratios.size());
        if (info.ratios[0] < 0.0f)
            desc->ratios.push_back(info.ratios[0]);
        else
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: first entry for "
                           "reverse gear ratio needs to be negative.", ownerName.c_str());
            return {};
        }
        float previousRatio = FLT_MAX;
        for (size_t i = 1; i < info.ratios.size(); ++i)
        {
            const float r = info.ratios[i];
            if (r > 0.0f && r < previousRatio)
            {
                desc->ratios.push_back(r);
                previousRatio = r;
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: entries for "
                               "forward gear ratios need to be positive and form a descending sequence.", ownerName.c_str());
                return {};
            }
        }
    }
    else
    {
        desc->ratios = { -4.0f, 4.0f, 2.0f, 1.5f, 1.1f, 1.0f };
    }

    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicleGears:ratioScale"), v))
        {
            if (!(v >= FLT_MIN) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"ratioScale\" out of range.", ownerName.c_str());
                return {};
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
        if (src.getAttribute(key, src.internToken("physxVehicleGears:switchTime"), v))
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"switchTime\" out of range.", ownerName.c_str());
                return {};
            }
            desc->switchTime = v;
        }
        else
        {
            desc->switchTime = 0.5f;
        }
    }

    return desc;
}

} // namespace omni::physics::parse
