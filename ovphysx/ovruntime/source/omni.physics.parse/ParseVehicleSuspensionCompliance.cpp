// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleSuspensionComplianceAPI reader. Per-prim parser for
// suspension compliance carrying four optional VtArray fields:
//   - wheelToeAngle           : VtArray<GfVec2f>, max 3, normalized jounce x
//   - wheelCamberAngle        : VtArray<GfVec2f>, max 3, normalized jounce x
//   - suspensionForceAppPoint : VtArray<GfVec4f>, max 3
//   - tireForceAppPoint       : VtArray<GfVec4f>, max 3
//
// Walker pre-reads the arrays since IPhysicsSource has no array
// accessor. Validation rules:
//   - Each list size <= 3.
//   - For angle lists: jounce (x) in [0, 1], monotonically increasing;
//     |angle (y)| <= pi/2.
//   - For point lists: jounce (x) in [0, 1], monotonically increasing;
//     no value range check on y/z/w.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <cmath>

#ifndef M_PI_2
#  define M_PI_2 1.57079632679489661923
#endif

namespace omni::physics::parse
{

namespace
{

bool copyAngles(const std::vector<carb::Float2>& src, VehicleOutputVector<carb::Float2>& dst,
                const std::string& owner, const char* attrName, VehicleFillStatus& status)
{
    if (src.size() > 3)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                       "entries is 3.", owner.c_str(), attrName);
        return false;
    }
    float previousJounce = -1.0f;
    for (const carb::Float2& v : src)
    {
        if (v.x < 0.0f || v.x > 1.0f)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s (first axis)\" needs to be in range [0, 1].",
                           owner.c_str(), attrName);
            return false;
        }
        if (std::fabs(v.y) > static_cast<float>(M_PI_2))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s (second axis)\" needs to be in range [-pi, pi].",
                           owner.c_str(), attrName);
            return false;
        }
        if (v.x <= previousJounce)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
                           "have to be monotonically increasing.", owner.c_str(), attrName);
            return false;
        }
        dst.push_back(v);
        previousJounce = v.x;
    }
    return true;
}

bool copyPoints(const std::vector<carb::Float4>& src, VehicleOutputVector<carb::Float4>& dst,
                const std::string& owner, const char* attrName, VehicleFillStatus& status)
{
    if (src.size() > 3)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                       "entries is 3.", owner.c_str(), attrName);
        return false;
    }
    float previousJounce = -1.0f;
    for (const carb::Float4& v : src)
    {
        if (v.x < 0.0f || v.x > 1.0f)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s (first axis)\" needs to be in range [0, 1].",
                           owner.c_str(), attrName);
            return false;
        }
        if (v.x <= previousJounce)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
                           "have to be monotonically increasing.", owner.c_str(), attrName);
            return false;
        }
        dst.push_back(v);
        previousJounce = v.x;
    }
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
PreparedSuspensionCompliance prepareSuspensionCompliance(ParseContext& ctx, ObjectKey key, const SuspensionComplianceInfo& info)
{
    PreparedSuspensionCompliance input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    return input;
}

void prepareSuspensionComplianceStorage(const PreparedSuspensionCompliance& input, SuspensionComplianceDesc& output)
{
    output.wheelToeAngleList.resize(std::min(input.info.wheelToeAngles.size(), size_t(3)));
    output.wheelCamberAngleList.resize(std::min(input.info.wheelCamberAngles.size(), size_t(3)));
    output.suspensionForceAppPointList.resize(std::min(input.info.suspensionForceAppPoints.size(), size_t(3)));
    output.tireForceAppPointList.resize(std::min(input.info.tireForceAppPoints.size(), size_t(3)));
}

bool fillSuspensionCompliance(const PreparedSuspensionCompliance& input, SuspensionComplianceDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const SuspensionComplianceInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.wheelToeAngleList)::value_type> wheelToeAngleListOut(output.wheelToeAngleList, vehicle_output_slot::wheelToeAngleList, status);
    VehicleOutputVector<typename decltype(output.wheelCamberAngleList)::value_type> wheelCamberAngleListOut(output.wheelCamberAngleList, vehicle_output_slot::wheelCamberAngleList, status);
    VehicleOutputVector<typename decltype(output.suspensionForceAppPointList)::value_type> suspensionForceAppPointListOut(output.suspensionForceAppPointList, vehicle_output_slot::suspensionForceAppPointList, status);
    VehicleOutputVector<typename decltype(output.tireForceAppPointList)::value_type> tireForceAppPointListOut(output.tireForceAppPointList, vehicle_output_slot::tireForceAppPointList, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    SuspensionComplianceDesc* desc = &output;

    if (!copyAngles(info.wheelToeAngles,    wheelToeAngleListOut,    ownerName, "wheelToeAngle", status)) return false;
    if (!copyAngles(info.wheelCamberAngles, wheelCamberAngleListOut, ownerName, "wheelCamberAngle", status)) return false;
    if (!copyPoints(info.suspensionForceAppPoints, suspensionForceAppPointListOut, ownerName, "suspensionForceAppPoint", status)) return false;
    if (!copyPoints(info.tireForceAppPoints,       tireForceAppPointListOut,       ownerName, "tireForceAppPoint", status)) return false;

    return true;
}

void finishSuspensionComplianceStorage(SuspensionComplianceDesc& output, const VehicleFillStatus& status)
{
    output.wheelToeAngleList.resize(status.written[vehicle_output_slot::wheelToeAngleList]);
    output.wheelCamberAngleList.resize(status.written[vehicle_output_slot::wheelCamberAngleList]);
    output.suspensionForceAppPointList.resize(status.written[vehicle_output_slot::suspensionForceAppPointList]);
    output.tireForceAppPointList.resize(status.written[vehicle_output_slot::tireForceAppPointList]);
}

DescPtr<SuspensionComplianceDesc> parseSuspensionCompliance(ParseContext& ctx, ObjectKey key, const SuspensionComplianceInfo& info)
{
    const PreparedSuspensionCompliance input = prepareSuspensionCompliance(ctx, key, info);
    DescPtr<SuspensionComplianceDesc> output = allocateDesc<SuspensionComplianceDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareSuspensionComplianceStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillSuspensionCompliance(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishSuspensionComplianceStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
