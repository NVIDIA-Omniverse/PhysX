// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
PreparedNonlinearCmdResponse prepareNonlinearCmdResponse(ParseContext& ctx, ObjectKey ownerKey, std::string_view instanceName, const NonlinearCmdResponseInfo& info)
{
    PreparedNonlinearCmdResponse input;
    input.ownerKey = ownerKey;
    input.instanceName = instanceName;
    input.info = info;
    input.instanceToken = ctx.source().internToken(instanceName);
    input.reads = prepareVehicleReads(ctx.source(), ownerKey, {

    }, instanceName);
    return input;
}

void prepareNonlinearCmdResponseStorage(const PreparedNonlinearCmdResponse& input, NonlinearCmdResponseDesc& output)
{
    output.commandValues.resize(input.info.hasCommandValues ? input.info.commandValues.size() : 0);
    output.speedResponsesPerCommandValue.resize(input.info.hasSpeedResponsesPerCommandValue ? input.info.speedResponsesPerCommandValue.size() : 0);
    output.speedResponses.resize(input.info.hasSpeedResponses ? input.info.speedResponses.size() : 0);
}

bool fillNonlinearCmdResponse(const PreparedNonlinearCmdResponse& input, NonlinearCmdResponseDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey ownerKey = input.ownerKey;
    [[maybe_unused]] const std::string& instanceName = input.instanceName;
    [[maybe_unused]] const NonlinearCmdResponseInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.commandValues)::value_type> commandValuesOut(output.commandValues, vehicle_output_slot::commandValues, status);
    VehicleOutputVector<typename decltype(output.speedResponsesPerCommandValue)::value_type> speedResponsesPerCommandValueOut(output.speedResponsesPerCommandValue, vehicle_output_slot::speedResponsesPerCommandValue, status);
    VehicleOutputVector<typename decltype(output.speedResponses)::value_type> speedResponsesOut(output.speedResponses, vehicle_output_slot::speedResponses, status);


    const std::string& ownerName = src.sourceKeyToString(ownerKey);
    (void)instanceName;  // unused — kept for API symmetry with other parsers

    NonlinearCmdResponseDesc* desc = &output;

    // commandValues
    if (info.hasCommandValues)
    {
        if (info.commandValues.size() > NonlinearCmdResponseDesc::maxNumberOfCommandValues)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                           "has more than the maximum allowed number of %d entries.",
                           ownerName.c_str(), NonlinearCmdResponseDesc::maxNumberOfCommandValues);
            return false;
        }
        float lastCommandValue = -1.0f;
        commandValuesOut.reserve(info.commandValues.size());
        for (float v : info.commandValues)
        {
            if (v < 0.0f || v > 1.0f)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                               "expects values in range [0, 1].", ownerName.c_str());
                return false;
            }
            if (v <= lastCommandValue)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "
                               "expects strictly increasing entries.", ownerName.c_str());
                return false;
            }
            commandValuesOut.push_back(v);
            lastCommandValue = v;
        }
    }

    // speedResponses count check (fires before the per-cv index walk).
    if (info.hasSpeedResponses &&
        info.speedResponses.size() > NonlinearCmdResponseDesc::maxNumberOfSpeedResponses)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                       "has more than the maximum allowed number of %d entries.",
                       ownerName.c_str(), NonlinearCmdResponseDesc::maxNumberOfSpeedResponses);
        return false;
    }

    // speedResponsesPerCommandValue: length must match commandValues
    if (info.hasSpeedResponsesPerCommandValue)
    {
        if (info.speedResponsesPerCommandValue.size() != info.commandValues.size())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                           "needs to have the same number of entries as the \"commandValues\" attribute.",
                           ownerName.c_str());
            return false;
        }
    }

    // Walk speedResponsesPerCommandValue: each entry in
    // [0, speedResponses.size()), strictly increasing.
    {
        int lastIndex = -1;
        speedResponsesPerCommandValueOut.reserve(info.speedResponsesPerCommandValue.size());
        const int srSize = static_cast<int>(info.speedResponses.size());
        for (int idx : info.speedResponsesPerCommandValue)
        {
            if (idx < 0 || (idx + 1) > srSize)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                               "expects values in range [0, %d).", ownerName.c_str(), srSize);
                return false;
            }
            if (idx <= lastIndex)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "
                               "expects strictly increasing entries.", ownerName.c_str());
                return false;
            }
            speedResponsesPerCommandValueOut.push_back(idx);
            lastIndex = idx;
        }
    }

    // Walk speedResponses: response value (y) in [0,1]; per-graph
    // strictly-increasing speed (x). Graph boundaries are at the
    // indices in `speedResponsesPerCommandValue`.
    {
        float lastSpeedValue = -FLT_MAX;
        speedResponsesOut.reserve(info.speedResponses.size());
        int speedGraphIndex = 1;
        int entryIndex = 0;
        for (const carb::Float2& sp : info.speedResponses)
        {
            if (sp.y < 0.0f || sp.y > 1.0f)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                               "expects response values in range [0, 1].", ownerName.c_str());
                return false;
            }
            if (sp.x <= lastSpeedValue)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "
                               "expects strictly increasing speed entries.", ownerName.c_str());
                return false;
            }
            speedResponsesOut.push_back(sp);
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

    return true;
}

void finishNonlinearCmdResponseStorage(NonlinearCmdResponseDesc& output, const VehicleFillStatus& status)
{
    output.commandValues.resize(status.written[vehicle_output_slot::commandValues]);
    output.speedResponsesPerCommandValue.resize(status.written[vehicle_output_slot::speedResponsesPerCommandValue]);
    output.speedResponses.resize(status.written[vehicle_output_slot::speedResponses]);
}

DescPtr<NonlinearCmdResponseDesc> parseNonlinearCmdResponse(ParseContext& ctx, ObjectKey ownerKey, std::string_view instanceName, const NonlinearCmdResponseInfo& info)
{
    const PreparedNonlinearCmdResponse input = prepareNonlinearCmdResponse(ctx, ownerKey, instanceName, info);
    DescPtr<NonlinearCmdResponseDesc> output = allocateDesc<NonlinearCmdResponseDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareNonlinearCmdResponseStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillNonlinearCmdResponse(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishNonlinearCmdResponseStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
