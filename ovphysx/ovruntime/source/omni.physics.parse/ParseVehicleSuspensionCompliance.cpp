// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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

bool copyAngles(const std::vector<carb::Float2>& src, std::vector<carb::Float2>& dst,
                const std::string& owner, const char* attrName)
{
    if (src.size() > 3)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                       "entries is 3.", owner.c_str(), attrName);
        return false;
    }
    float previousJounce = -1.0f;
    for (const carb::Float2& v : src)
    {
        if (v.x < 0.0f || v.x > 1.0f)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s (first axis)\" needs to be in range [0, 1].",
                           owner.c_str(), attrName);
            return false;
        }
        if (std::fabs(v.y) > static_cast<float>(M_PI_2))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s (second axis)\" needs to be in range [-pi, pi].",
                           owner.c_str(), attrName);
            return false;
        }
        if (v.x <= previousJounce)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
                           "have to be monotonically increasing.", owner.c_str(), attrName);
            return false;
        }
        dst.push_back(v);
        previousJounce = v.x;
    }
    return true;
}

bool copyPoints(const std::vector<carb::Float4>& src, std::vector<carb::Float4>& dst,
                const std::string& owner, const char* attrName)
{
    if (src.size() > 3)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                       "entries is 3.", owner.c_str(), attrName);
        return false;
    }
    float previousJounce = -1.0f;
    for (const carb::Float4& v : src)
    {
        if (v.x < 0.0f || v.x > 1.0f)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s (first axis)\" needs to be in range [0, 1].",
                           owner.c_str(), attrName);
            return false;
        }
        if (v.x <= previousJounce)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
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
DescPtr<SuspensionComplianceDesc> parseSuspensionCompliance(
    ParseContext& ctx, ObjectKey key, const SuspensionComplianceInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<SuspensionComplianceDesc> desc = allocateDesc<SuspensionComplianceDesc>(ctx.descriptorAllocator());

    if (!copyAngles(info.wheelToeAngles,    desc->wheelToeAngleList,    ownerName, "wheelToeAngle")) return {};
    if (!copyAngles(info.wheelCamberAngles, desc->wheelCamberAngleList, ownerName, "wheelCamberAngle")) return {};
    if (!copyPoints(info.suspensionForceAppPoints, desc->suspensionForceAppPointList, ownerName, "suspensionForceAppPoint")) return {};
    if (!copyPoints(info.tireForceAppPoints,       desc->tireForceAppPointList,       ownerName, "tireForceAppPoint")) return {};

    return desc;
}

} // namespace omni::physics::parse
