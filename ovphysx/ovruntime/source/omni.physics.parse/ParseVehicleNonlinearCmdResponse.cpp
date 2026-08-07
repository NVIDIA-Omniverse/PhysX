// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-VEH-ROOT-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleNonlinearCommandResponseAPI (multi-apply per command
// instance) reader. Walker pre-reads all three arrays
// (commandValues / speedResponsesPerCommandValue / speedResponses)
// because IPhysicsSource has no array accessor.
//
// Validation rules:
//   - commandValues: max 8 entries, in [0,1], strictly increasing.
//   - speedResponsesPerCommandValue: same length as commandValues,
//     each entry in [0, speedResponses.size()), strictly increasing.
//   - speedResponses: max 64 entries, per-graph strictly-increasing
//     speed; response values clamped to [0,1].

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
DescPtr<NonlinearCmdResponseDesc> parseNonlinearCmdResponse(
    ParseContext& ctx, ObjectKey ownerKey,
    std::string_view instanceName, const NonlinearCmdResponseInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(ownerKey));
    (void)instanceName;  // unused — kept for API symmetry with other parsers

    DescPtr<NonlinearCmdResponseDesc> desc = allocateDesc<NonlinearCmdResponseDesc>(ctx.descriptorAllocator());

    // commandValues
    if (info.hasCommandValues)
    {
        if (info.commandValues.size() > NonlinearCmdResponseDesc::maxNumberOfCommandValues)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                           "has more than the maximum allowed number of %d entries.",
                           ownerName.c_str(), NonlinearCmdResponseDesc::maxNumberOfCommandValues);
            return {};
        }
        float lastCommandValue = -1.0f;
        desc->commandValues.reserve(info.commandValues.size());
        for (float v : info.commandValues)
        {
            if (v < 0.0f || v > 1.0f)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                               "expects values in range [0, 1].", ownerName.c_str());
                return {};
            }
            if (v <= lastCommandValue)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                               "expects strictly increasing entries.", ownerName.c_str());
                return {};
            }
            desc->commandValues.push_back(v);
            lastCommandValue = v;
        }
    }

    // speedResponses count check (fires before the per-cv index walk).
    if (info.hasSpeedResponses &&
        info.speedResponses.size() > NonlinearCmdResponseDesc::maxNumberOfSpeedResponses)
    {
        CARB_LOG_ERROR("Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                       "has more than the maximum allowed number of %d entries.",
                       ownerName.c_str(), NonlinearCmdResponseDesc::maxNumberOfSpeedResponses);
        return {};
    }

    // speedResponsesPerCommandValue: length must match commandValues
    if (info.hasSpeedResponsesPerCommandValue)
    {
        if (info.speedResponsesPerCommandValue.size() != info.commandValues.size())
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                           "needs to have the same number of entries as the \"commandValues\" attribute.",
                           ownerName.c_str());
            return {};
        }
    }

    // Walk speedResponsesPerCommandValue: each entry in
    // [0, speedResponses.size()), strictly increasing.
    {
        int lastIndex = -1;
        desc->speedResponsesPerCommandValue.reserve(info.speedResponsesPerCommandValue.size());
        const int srSize = static_cast<int>(info.speedResponses.size());
        for (int idx : info.speedResponsesPerCommandValue)
        {
            if (idx < 0 || (idx + 1) > srSize)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                               "expects values in range [0, %d).", ownerName.c_str(), srSize);
                return {};
            }
            if (idx <= lastIndex)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                               "expects strictly increasing entries.", ownerName.c_str());
                return {};
            }
            desc->speedResponsesPerCommandValue.push_back(idx);
            lastIndex = idx;
        }
    }

    // Walk speedResponses: response value (y) in [0,1]; per-graph
    // strictly-increasing speed (x). Graph boundaries are at the
    // indices in `speedResponsesPerCommandValue`.
    {
        float lastSpeedValue = -FLT_MAX;
        desc->speedResponses.reserve(info.speedResponses.size());
        int speedGraphIndex = 1;
        int entryIndex = 0;
        for (const carb::Float2& sp : info.speedResponses)
        {
            if (sp.y < 0.0f || sp.y > 1.0f)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                               "expects response values in range [0, 1].", ownerName.c_str());
                return {};
            }
            if (sp.x <= lastSpeedValue)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                               "expects strictly increasing speed entries.", ownerName.c_str());
                return {};
            }
            desc->speedResponses.push_back(sp);
            ++entryIndex;

            if (static_cast<size_t>(speedGraphIndex) >= info.speedResponsesPerCommandValue.size() ||
                entryIndex < info.speedResponsesPerCommandValue[speedGraphIndex])
            {
                lastSpeedValue = sp.x;
            }
            else
            {
                lastSpeedValue = -FLT_MAX;
                ++speedGraphIndex;
            }
        }
    }

    return desc;
}

} // namespace omni::physics::parse
