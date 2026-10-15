// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

bool copyNonNegative(const std::vector<float>& src, VehicleOutputVector<float>& dst,
                     const char* attrName, const std::string& owner, VehicleFillStatus& status)
{
    dst.reserve(src.size());
    for (float v : src)
    {
        if (v < 0.0f)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"%s\" of autoGearBox \"%s\" has invalid values: entries must "
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
PreparedAutoGearBox prepareAutoGearBox(ParseContext& ctx, ObjectKey key, const AutoGearBoxInfo& info)
{
    PreparedAutoGearBox input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleAutoGearBox:latency", VehicleAttributeType::floating }
    });
    return input;
}

void prepareAutoGearBoxStorage(const PreparedAutoGearBox& input, AutoGearBoxDesc& output)
{
    output.upRatios.resize(input.info.hasUpRatios ? input.info.upRatios.size() : 0);
    output.downRatios.resize(input.info.hasDownRatios ? input.info.downRatios.size() : 0);
}

bool fillAutoGearBox(const PreparedAutoGearBox& input, AutoGearBoxDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const AutoGearBoxInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.upRatios)::value_type> upRatiosOut(output.upRatios, vehicle_output_slot::upRatios, status);
    VehicleOutputVector<typename decltype(output.downRatios)::value_type> downRatiosOut(output.downRatios, vehicle_output_slot::downRatios, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    AutoGearBoxDesc* desc = &output;

    if (info.hasUpRatios)
    {
        if (!copyNonNegative(info.upRatios, upRatiosOut, "upRatios", ownerName, status))
            return false;
    }
    // The parent's gears.ratios.size() isn't visible here, so we can't
    // size a per-slot 0.65f default; leave the array empty when not
    // authored. Vehicle-create-side fills if it cares.

    if (info.hasDownRatios)
    {
        if (!copyNonNegative(info.downRatios, downRatiosOut, "downRatios", ownerName, status))
            return false;
    }

    float latency;
    if (src.getAttribute(key, src.attribute("physxVehicleAutoGearBox:latency"), latency))
    {
        if (!(latency >= 0.0f) || !(latency < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"latency\" out of range.", ownerName.c_str());
            return false;
        }
        desc->latency = latency;
    }
    else
    {
        desc->latency = 2.0f;
    }

    return true;
}

void finishAutoGearBoxStorage(AutoGearBoxDesc& output, const VehicleFillStatus& status)
{
    output.upRatios.resize(status.written[vehicle_output_slot::upRatios]);
    output.downRatios.resize(status.written[vehicle_output_slot::downRatios]);
}

DescPtr<AutoGearBoxDesc> parseAutoGearBox(ParseContext& ctx, ObjectKey key, const AutoGearBoxInfo& info)
{
    const PreparedAutoGearBox input = prepareAutoGearBox(ctx, key, info);
    DescPtr<AutoGearBoxDesc> output = allocateDesc<AutoGearBoxDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareAutoGearBoxStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillAutoGearBox(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishAutoGearBoxStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
