// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-TENDON-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

// Fixed tendon reader. Reads PhysxTendonAxisRootAPI + PhysxTendonAxisAPI
// multi-apply instances on a single-DOF joint (revolute or prismatic)
// and emits one PhysxTendonAxisDesc per axis instance + one
// PhysxTendonFixedDesc per root. Revolute joints get a deg→rad gearing
// conversion. Hierarchy resolution + engine-object creation stay
// consumer-side via the `createFixedTendons` helper.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <cmath>
#include <string>
#include <unordered_set>

namespace omni::physics::parse
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Read a clamped float; clamps ±inf to bounds, out-of-range to nearest.
void readClampedFloat(IPhysicsSource& src, ObjectKey key, const std::string& attrName,
                      float minV, float maxV, float& out)
{
    float val;
    if (!src.getAttribute(key, src.internToken(attrName), val))
        return;
    if (std::isinf(val))
        val = (val < 0.0f) ? minV : maxV;
    if (val < minV)      val = minV;
    else if (val > maxV) val = maxV;
    out = val;
}

void readBool(IPhysicsSource& src, ObjectKey key, const std::string& attrName, bool& out)
{
    src.getAttribute(key, src.internToken(attrName), out);
}

// Reads `physxTendon:<inst>:gearing` (float[], first element) and
// `physxTendon:<inst>:forceCoefficient` (float[], first element).
// For revolute joints, gearing is converted deg→rad (or clamped to
// ±FLT_MAX past ±180°); for prismatic, kept verbatim. Axes vector
// holds the single resolved JointAxis (eRotX or eTransX).
//
// `link0` / `link1` come from the walker (joint body0/body1 rel targets).
void fillAxisFromApi(IPhysicsSource& src, ObjectKey jointKey, const std::string& instance,
                    const FixedTendonParseInfo& info, PhysxTendonAxisDesc& desc,
                    std::string_view ownerPath)
{
    const std::string base = std::string("physxTendon:") + instance + ":";

    // forceCoefficient: float[] schema; only the first element is
    // meaningful. UsdSource surfaces single-element VtFloatArray as
    // Float at the boundary so this scalar read works.
    src.getAttribute(jointKey, src.internToken(base + "forceCoefficient"), desc.forceCoefficients[0]);

    // gearing: same shape, but per-joint-type deg→rad conversion.
    float gearing = 1.0f;
    const bool gearingAuthored = src.getAttribute(jointKey, src.internToken(base + "gearing"), gearing);

    if (info.jointType == eJointRevolute)
    {
        // User authors coefficient in deg→tendonLength; multiply by rad→deg
        // so the runtime conversion of joint angle from rad consumes the
        // same coefficient. Past ±180° → ±FLT_MAX.
        if (!gearingAuthored)
            gearing = 1.0f;
        const float degLimit = kPi; // 180° in rad
        if (gearing > degLimit) gearing = FLT_MAX;
        else if (gearing < -degLimit) gearing = -FLT_MAX;
        else                          gearing *= kRadToDeg;

        desc.gearings[0] = gearing;
        desc.axes[0]     = eRotX;
    }
    else if (info.jointType == eJointPrismatic)
    {
        desc.gearings[0] = gearingAuthored ? gearing : 1.0f;
        desc.axes[0]     = eTransX;
    }
    else
    {
        CARB_LOG_WARN(
            "Tendon axis at %s applied to unsupported joint type. Only revolute and prismatic are currently supported.",
            std::string(ownerPath).c_str());
        // Still push the desc into the map but with unset axis.
        desc.gearings[0] = gearingAuthored ? gearing : 1.0f;
    }

    desc.link0         = info.body0;
    desc.link1         = info.body1;
    desc.instanceToken = src.internToken(instance);
    desc.jointKey     = jointKey;
}

// Fill PhysxTendonFixedDesc from PhysxTendonAxisRootAPI:<inst>.
void fillFixedTendonFromApi(IPhysicsSource& src, ObjectKey jointKey, const std::string& instance,
                            PhysxTendonFixedDesc& desc, std::string_view ownerPath)
{
    const std::string base = std::string("physxTendon:") + instance + ":";

    readClampedFloat(src, jointKey, base + "stiffness",      0.0f, FLT_MAX, desc.stiffness);
    readClampedFloat(src, jointKey, base + "limitStiffness", 0.0f, FLT_MAX, desc.limitStiffness);
    readClampedFloat(src, jointKey, base + "damping",        0.0f, FLT_MAX, desc.damping);
    readClampedFloat(src, jointKey, base + "offset",      -FLT_MAX, FLT_MAX, desc.offset);
    desc.isEnabled = true;
    readBool       (src, jointKey, base + "tendonEnabled",                  desc.isEnabled);

    desc.restLength = -FLT_MAX;
    desc.lowLimit   = -FLT_MAX;
    desc.highLimit  =  FLT_MAX;
    readClampedFloat(src, jointKey, base + "restLength", -FLT_MAX, FLT_MAX, desc.restLength);
    readClampedFloat(src, jointKey, base + "lowerLimit", -FLT_MAX, FLT_MAX, desc.lowLimit);
    readClampedFloat(src, jointKey, base + "upperLimit", desc.lowLimit, FLT_MAX, desc.highLimit);

    if (desc.limitStiffness != 0.0f && desc.lowLimit == -FLT_MAX && desc.highLimit == FLT_MAX)
    {
        CARB_LOG_WARN(
            "The fixed tendon at %s has a positive limit stiffness but no limits set!",
            std::string(ownerPath).c_str());
    }

    desc.instanceToken = src.internToken(instance);
    desc.jointKey     = jointKey;
    desc.rootAxis      = nullptr; // consumer wires
}

} // namespace

// @implements REQ-PARSE-TENDON-002
// @covers AC-1 AC-2 AC-3 AC-4
void parseFixedTendons(ParseContext& ctx, ObjectKey jointKey,
                       const FixedTendonParseInfo& info,
                       std::vector<DescPtr<PhysxTendonAxisDesc>>& outAxes,
                       std::vector<DescPtr<PhysxTendonFixedDesc>>& outTendons)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerPathStr(src.sourceKeyToString(jointKey));

    // Dedup across the two multi-apply bases. `PhysxTendonAxisRootAPI`
    // auto-applies `PhysxTendonAxisAPI` with the same instance name
    // (per the schema's `apiSchemas` metadata), so the Axis-iter sees
    // the auto-applied entry and would log a spurious "duplicate" error
    // / mis-classify the root as intermediate. Track Root-claimed
    // instances separately and silently skip them in the Axis pass.
    std::unordered_set<std::string> claimedByRoot;
    std::unordered_set<std::string> instancesSeen;
    auto claim = [&](std::string_view instance) -> bool {
        if (instancesSeen.insert(std::string(instance)).second)
            return true;
        CARB_LOG_ERROR(
            "More than one tendon axis instance with name %s was applied at joint %s.",
            std::string(instance).c_str(), ownerPathStr.c_str());
        return false;
    };

    // Roots first (each emits an axis AND a tendon descriptor sharing
    // the same jointKey + instanceToken).
    src.forEachMultiApplyInstance(jointKey, "PhysxTendonAxisRootAPI",
        [&](std::string_view inst) {
            if (!claim(inst)) return;
            claimedByRoot.insert(std::string(inst));
            const std::string instance(inst);

            DescPtr<PhysxTendonAxisDesc> axis = allocateDesc<PhysxTendonAxisDesc>(ctx.descriptorAllocator());
            fillAxisFromApi(src, jointKey, instance, info, *axis, ownerPathStr);

            DescPtr<PhysxTendonFixedDesc> tendon = allocateDesc<PhysxTendonFixedDesc>(ctx.descriptorAllocator());
            fillFixedTendonFromApi(src, jointKey, instance, *tendon, ownerPathStr);

            outAxes.push_back(std::move(axis));
            outTendons.push_back(std::move(tendon));
        });

    // Intermediate axes. Silently skip Root-claimed instances; that
    // overlap is the auto-apply, not a user-authored duplicate.
    src.forEachMultiApplyInstance(jointKey, "PhysxTendonAxisAPI",
        [&](std::string_view inst) {
            if (claimedByRoot.count(std::string(inst)))
                return;
            if (!claim(inst)) return;
            const std::string instance(inst);

            DescPtr<PhysxTendonAxisDesc> axis = allocateDesc<PhysxTendonAxisDesc>(ctx.descriptorAllocator());
            fillAxisFromApi(src, jointKey, instance, info, *axis, ownerPathStr);
            outAxes.push_back(std::move(axis));
        });
}

} // namespace omni::physics::parse
