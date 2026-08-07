// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-VEH-DRIVE-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleAutoGearBoxAPI reader. Walker pre-reads upRatios + down-
// Ratios (both VtArray<float>); parse-lib stores them verbatim. The
// expected-ratio-count validation against the parent DriveStandard's
// gears.ratios.size() can't run here because the parent rel isn't
// resolved at parse-lib time — that validation is deferred to vehicle
// create. Each entry must be non-negative; signature checks live in
// the parse-lib.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

bool copyNonNegative(const std::vector<float>& src, std::vector<float>& dst,
                     const char* attrName, const std::string& owner)
{
    dst.reserve(src.size());
    for (float v : src)
    {
        if (v < 0.0f)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"%s\" of autoGearBox \"%s\" has invalid values: entries must "
                           "not be negative.", attrName, owner.c_str());
            return false;
        }
        dst.push_back(v);
    }
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-1 AC-2 AC-3
DescPtr<AutoGearBoxDesc> parseAutoGearBox(ParseContext& ctx, ObjectKey key, const AutoGearBoxInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<AutoGearBoxDesc> desc = allocateDesc<AutoGearBoxDesc>(ctx.descriptorAllocator());

    if (info.hasUpRatios)
    {
        if (!copyNonNegative(info.upRatios, desc->upRatios, "upRatios", ownerName))
            return {};
    }
    // The parent's gears.ratios.size() isn't visible here, so we can't
    // size a per-slot 0.65f default; leave the array empty when not
    // authored. Vehicle-create-side fills if it cares.

    if (info.hasDownRatios)
    {
        if (!copyNonNegative(info.downRatios, desc->downRatios, "downRatios", ownerName))
            return {};
    }

    float latency;
    if (src.getAttribute(key, src.internToken("physxVehicleAutoGearBox:latency"), latency))
    {
        if (!(latency >= 0.0f) || !(latency < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"latency\" out of range.", ownerName.c_str());
            return {};
        }
        desc->latency = latency;
    }
    else
    {
        desc->latency = 2.0f;
    }

    return desc;
}

} // namespace omni::physics::parse
