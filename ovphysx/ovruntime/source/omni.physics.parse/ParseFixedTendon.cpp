// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedTendon.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>

namespace omni::physics::parse
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
// Largest |gearing| whose deg->rad conversion below still fits in a float; past it the
// multiply would overflow to +-inf, so the value saturates to +-FLT_MAX instead. This is
// legacy `parseAxes`' `GfDegreesToRadians(FLT_MAX)` bound, and the same one the runtime
// gearing updater applies (`degToRad(FLT_MAX)`, PhysXTendonPropertiesUpdate.cpp), so an
// authored gearing and a gearing set later at runtime agree.
constexpr float kMaxRevoluteGearing = static_cast<float>(static_cast<double>(FLT_MAX) * (3.14159265358979323846 / 180.0));

PreparedFixedTendon prepareInstance(ParseContext& ctx, ObjectKey key, const std::string& instance,
                                    const FixedTendonParseInfo& info, bool root, const std::string& owner)
{
    IPhysicsSource& src = ctx.source();
    PreparedFixedTendon input;
    input.key = key;
    input.instance = src.internToken(instance);
    input.info = info;
    input.root = root;
    const std::string base = "physxTendon:" + instance + ":";
    input.values.gearing = prepareAttribute<float>(src, key, src.internToken(base + "gearing"));
    input.values.forceCoefficient = prepareAttribute<float>(src, key, src.internToken(base + "forceCoefficient"));
    if (info.jointType != eJointRevolute && info.jointType != eJointPrismatic)
        CARB_LOG_WARN("Tendon axis at %s applied to unsupported joint type. Only revolute and prismatic are currently supported.", owner.c_str());
    else if (info.jointType == eJointRevolute && std::fabs(input.values.gearing.valueOr(1.0f)) > kMaxRevoluteGearing)
        CARB_LOG_WARN("The tendon axis %s at %s has a gearing of %g, too large to convert from degrees; it is clamped to +-FLT_MAX.",
                      instance.c_str(), owner.c_str(), double(input.values.gearing.valueOr(1.0f)));
    if (root)
    {
        input.values.stiffness = prepareAttribute<float>(src, key, src.internToken(base + "stiffness"));
        input.values.limitStiffness = prepareAttribute<float>(src, key, src.internToken(base + "limitStiffness"));
        input.values.damping = prepareAttribute<float>(src, key, src.internToken(base + "damping"));
        input.values.offset = prepareAttribute<float>(src, key, src.internToken(base + "offset"));
        input.values.restLength = prepareAttribute<float>(src, key, src.internToken(base + "restLength"));
        input.values.lowerLimit = prepareAttribute<float>(src, key, src.internToken(base + "lowerLimit"));
        input.values.upperLimit = prepareAttribute<float>(src, key, src.internToken(base + "upperLimit"));
        input.values.enabled = prepareAttribute<bool>(src, key, src.internToken(base + "tendonEnabled"));
        // Validate diagnostics on the coordinator in original instance order.
        float stiffness = 0.0f, low = -FLT_MAX, high = FLT_MAX;
        fillTendonFloat(input.values.limitStiffness, 0.0f, FLT_MAX, stiffness);
        fillTendonFloat(input.values.lowerLimit, -FLT_MAX, FLT_MAX, low);
        fillTendonFloat(input.values.upperLimit, low, FLT_MAX, high);
        if (stiffness != 0.0f && low == -FLT_MAX && high == FLT_MAX)
            CARB_LOG_WARN("The fixed tendon at %s has a positive limit stiffness but no limits set!", owner.c_str());
    }
    input.prepared = true;
    return input;
}

} // namespace

// @implements REQ-PARSE-TENDON-002
// @covers AC-1 AC-2 AC-3 AC-4
void prepareFixedTendons(ParseContext& ctx, ObjectKey jointKey,
                       const FixedTendonParseInfo& info,
                       std::vector<PreparedFixedTendon>& inputs)
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

            inputs.push_back(prepareInstance(ctx, jointKey, instance, info, true, ownerPathStr));
        });

    // Intermediate axes. Silently skip Root-claimed instances; that
    // overlap is the auto-apply, not a user-authored duplicate.
    src.forEachMultiApplyInstance(jointKey, "PhysxTendonAxisAPI",
        [&](std::string_view inst) {
            if (claimedByRoot.count(std::string(inst)))
                return;
            if (!claim(inst)) return;
            const std::string instance(inst);

            inputs.push_back(prepareInstance(ctx, jointKey, instance, info, false, ownerPathStr));
        });
}

bool fillFixedTendonAxis(const PreparedFixedTendon& input, PhysxTendonAxisDesc& output) noexcept
{
    if (!input.prepared || output.gearings.size() != 1 || output.forceCoefficients.size() != 1 || output.axes.size() != 1) return false;
    input.values.forceCoefficient.read(output.forceCoefficients[0]);
    float gearing = input.values.gearing.valueOr(1.0f);
    if (input.info.jointType == eJointRevolute)
    {
        if (gearing > kMaxRevoluteGearing) gearing = FLT_MAX;
        else if (gearing < -kMaxRevoluteGearing) gearing = -FLT_MAX;
        else gearing *= kRadToDeg;
        output.axes[0] = eRotX;
    }
    else if (input.info.jointType == eJointPrismatic) output.axes[0] = eTransX;
    output.gearings[0] = gearing;
    output.link0 = input.info.body0;
    output.link1 = input.info.body1;
    output.instanceToken = input.instance;
    output.jointKey = input.key;
    return true;
}
bool fillFixedTendonRoot(const PreparedFixedTendon& input, PhysxTendonFixedDesc& output) noexcept
{
    if (!input.prepared || !input.root) return false;
    fillTendonFloat(input.values.stiffness, 0.0f, FLT_MAX, output.stiffness);
    fillTendonFloat(input.values.limitStiffness, 0.0f, FLT_MAX, output.limitStiffness);
    fillTendonFloat(input.values.damping, 0.0f, FLT_MAX, output.damping);
    fillTendonFloat(input.values.offset, -FLT_MAX, FLT_MAX, output.offset);
    output.isEnabled = input.values.enabled.valueOr(true);
    output.restLength = -FLT_MAX;
    output.lowLimit = -FLT_MAX;
    output.highLimit = FLT_MAX;
    fillTendonFloat(input.values.restLength, -FLT_MAX, FLT_MAX, output.restLength);
    fillTendonFloat(input.values.lowerLimit, -FLT_MAX, FLT_MAX, output.lowLimit);
    fillTendonFloat(input.values.upperLimit, output.lowLimit, FLT_MAX, output.highLimit);
    output.instanceToken = input.instance;
    output.jointKey = input.key;
    output.rootAxis = nullptr;
    return true;
}
void parseFixedTendons(ParseContext& ctx, ObjectKey key, const FixedTendonParseInfo& info,
                       std::vector<DescPtr<PhysxTendonAxisDesc>>& axes, std::vector<DescPtr<PhysxTendonFixedDesc>>& tendons)
{
    std::vector<PreparedFixedTendon> inputs;
    prepareFixedTendons(ctx, key, info, inputs);
    const size_t roots = size_t(std::count_if(inputs.begin(), inputs.end(), [](const PreparedFixedTendon& input) { return input.root; }));
    if (!reserveDescriptors<PhysxTendonAxisDesc>(ctx.descriptorAllocator(), inputs.size()) ||
        !reserveDescriptors<PhysxTendonFixedDesc>(ctx.descriptorAllocator(), roots)) throw std::bad_alloc();
    for (const PreparedFixedTendon& input : inputs)
    {
        DescPtr<PhysxTendonAxisDesc> axis = allocateDesc<PhysxTendonAxisDesc>(ctx.descriptorAllocator());
        if (axis && fillFixedTendonAxis(input, *axis)) axes.push_back(std::move(axis));
        if (input.root)
        {
            DescPtr<PhysxTendonFixedDesc> tendon = allocateDesc<PhysxTendonFixedDesc>(ctx.descriptorAllocator());
            if (tendon && fillFixedTendonRoot(input, *tendon)) tendons.push_back(std::move(tendon));
        }
    }
}

} // namespace omni::physics::parse
