// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-TENDON-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

// Spatial tendon reader. Emits a flat list of attachments
// (root / intermediate / leaf) per multi-apply instance applied to the
// link prim. Same attribute names and clamp ranges as the schema doc;
// `localPos` is baked by the link prim's world-space scale. Hierarchy
// reconstruction (parent ObjectKey → instance children) stays
// consumer-side.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedTendon.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <string>
#include <unordered_set>

namespace omni::physics::parse
{

namespace
{

// Resolve the single target of `physxTendon:<instance>:parentLink`.
// Returns invalid ObjectKey when the relationship is missing or empty.
// Logs a warning when more than one target is set.
ObjectKey readParentLink(IPhysicsSource& src, ObjectKey key, const std::string& base,
                         std::string_view ownerPath)
{
    std::vector<ObjectKey> targets;
    src.getRelationshipTargets(key, src.internToken(base + "parentLink"), targets);
    if (targets.empty())
        return ObjectKey{};
    if (targets.size() > 1)
    {
        CARB_LOG_WARN(
            "Attachment at %s has relationship to multiple parents! Will use parent at 0 position of array.",
            std::string(ownerPath).c_str());
    }
    return targets[0];
}

TokenId readParentAttachmentToken(IPhysicsSource& src, ObjectKey key, const std::string& base)
{
    TokenId t;
    if (src.getAttribute(key, src.internToken(base + "parentAttachment"), t))
        return t;
    return TokenId{};
}

PreparedSpatialTendon prepareInstance(ParseContext& ctx, ObjectKey key, const std::string& instance,
                                      const SpatialTendonParseInfo& info, ObjectType type)
{
    IPhysicsSource& src = ctx.source();
    PreparedSpatialTendon input;
    input.key = key;
    input.instance = src.internToken(instance);
    input.type = type;
    input.scale = info.linkWorldScale;
    const std::string base = "physxTendon:" + instance + ":";
    if (type != eTendonAttachmentRoot)
        input.values.gearing = prepareAttribute<float>(src, key, src.internToken(base + "gearing"));
    input.localPos = prepareAttribute<carb::Float3>(src, key, src.internToken(base + "localPos"));
    input.parentToken = readParentAttachmentToken(src, key, base);
    input.parent = readParentLink(src, key, base, src.sourceKeyToString(key));
    if (type == eTendonAttachmentRoot)
    {
        input.values.enabled = prepareAttribute<bool>(src, key, src.internToken(base + "tendonEnabled"));
        input.values.stiffness = prepareAttribute<float>(src, key, src.internToken(base + "stiffness"));
        input.values.limitStiffness = prepareAttribute<float>(src, key, src.internToken(base + "limitStiffness"));
        input.values.damping = prepareAttribute<float>(src, key, src.internToken(base + "damping"));
        input.values.offset = prepareAttribute<float>(src, key, src.internToken(base + "offset"));
    }
    else if (type == eTendonAttachmentLeaf)
    {
        input.values.restLength = prepareAttribute<float>(src, key, src.internToken(base + "restLength"));
        input.values.lowerLimit = prepareAttribute<float>(src, key, src.internToken(base + "lowerLimit"));
        input.values.upperLimit = prepareAttribute<float>(src, key, src.internToken(base + "upperLimit"));
    }
    return input;
}

} // namespace

// @implements REQ-PARSE-TENDON-001
// @covers AC-1 AC-2 AC-3 AC-4
void prepareSpatialTendons(ParseContext& ctx, ObjectKey linkKey,
                         const SpatialTendonParseInfo& info,
                         std::vector<PreparedSpatialTendon>& inputs)
{
    IPhysicsSource& src = ctx.source();

    // Dedup across the three multi-apply bases. Two separate sets:
    //
    // - `claimedByRootOrLeaf`: instance names already classified as Root
    //   or Leaf. PhysxTendonAttachmentRootAPI / LeafAPI both auto-apply
    //   PhysxTendonAttachmentAPI with the same instance name (per the
    //   schema's `apiSchemas` metadata), so the Attachment-iter sees
    //   the auto-applied entry and would mis-classify the prim as an
    //   intermediate. The Attachment iter silently skips these.
    //
    // - `instancesSeen`: surfaces the genuine user-error case where
    //   both Root and Leaf are applied with the same instance name on
    //   the same prim.
    std::unordered_set<std::string> claimedByRootOrLeaf;
    std::unordered_set<std::string> instancesSeen;
    auto claim = [&](std::string_view instance) -> bool {
        if (instancesSeen.insert(std::string(instance)).second)
            return true;
        CARB_LOG_ERROR(
            "More than one tendon attachment instance with name %s was applied at prim %s.",
            std::string(instance).c_str(),
            std::string(src.sourceKeyToString(linkKey)).c_str());
        return false;
    };

    // Root first.
    src.forEachMultiApplyInstance(linkKey, "PhysxTendonAttachmentRootAPI",
        [&](std::string_view inst) {
            if (!claim(inst)) return;
            claimedByRootOrLeaf.insert(std::string(inst));
            const std::string instance(inst);
            const std::string base = std::string("physxTendon:") + instance + ":";

            inputs.push_back(prepareInstance(ctx, linkKey, instance, info, eTendonAttachmentRoot));
        });

    // Leaf next. Run before the Attachment iter so it can silently
    // ignore the auto-applied AttachmentAPI:<leaf-instance> entries.
    src.forEachMultiApplyInstance(linkKey, "PhysxTendonAttachmentLeafAPI",
        [&](std::string_view inst) {
            if (!claim(inst)) return;
            claimedByRootOrLeaf.insert(std::string(inst));
            const std::string instance(inst);
            const std::string base = std::string("physxTendon:") + instance + ":";

            inputs.push_back(prepareInstance(ctx, linkKey, instance, info, eTendonAttachmentLeaf));
        });

    // Intermediate attachments last. Silently skip instances already
    // claimed by Root/Leaf — RootAPI/LeafAPI auto-apply AttachmentAPI
    // with the same instance name; that is NOT a user duplicate.
    src.forEachMultiApplyInstance(linkKey, "PhysxTendonAttachmentAPI",
        [&](std::string_view inst) {
            if (claimedByRootOrLeaf.count(std::string(inst)))
                return;
            if (!claim(inst)) return;
            const std::string instance(inst);

            inputs.push_back(prepareInstance(ctx, linkKey, instance, info, eTendonAttachment));
        });
}

bool fillSpatialTendon(const PreparedSpatialTendon& input, PhysxTendonAttachmentDesc& output) noexcept
{
    if (output.type != input.type || (input.type != eTendonAttachment && input.type != eTendonAttachmentRoot && input.type != eTendonAttachmentLeaf)) return false;
    if (input.type == eTendonAttachmentRoot) output.gearing = 1.0f;
    else fillTendonFloat(input.values.gearing, -FLT_MAX, FLT_MAX, output.gearing);
    const carb::Float3 position = input.localPos.valueOr({ 0.0f, 0.0f, 0.0f });
    output.localPos = { position.x * input.scale.x, position.y * input.scale.y, position.z * input.scale.z };
    output.linkKey = input.key;
    output.instanceToken = input.instance;
    output.parentToken = input.parentToken;
    output.parentKey = input.parent;
    if (input.type == eTendonAttachmentRoot)
    {
        PhysxTendonSpatialDesc& root = static_cast<PhysxTendonSpatialDesc&>(output);
        root.isEnabled = input.values.enabled.valueOr(true);
        fillTendonFloat(input.values.stiffness, 0.0f, FLT_MAX, root.stiffness);
        fillTendonFloat(input.values.limitStiffness, 0.0f, FLT_MAX, root.limitStiffness);
        fillTendonFloat(input.values.damping, 0.0f, FLT_MAX, root.damping);
        fillTendonFloat(input.values.offset, -FLT_MAX, FLT_MAX, root.offset);
    }
    else if (input.type == eTendonAttachmentLeaf)
    {
        PhysxTendonAttachmentLeafDesc& leaf = static_cast<PhysxTendonAttachmentLeafDesc&>(output);
        fillTendonFloat(input.values.restLength, -FLT_MAX, FLT_MAX, leaf.restLength);
        fillTendonFloat(input.values.lowerLimit, -FLT_MAX, FLT_MAX, leaf.lowLimit);
        fillTendonFloat(input.values.upperLimit, leaf.lowLimit, FLT_MAX, leaf.highLimit);
    }
    return true;
}
void parseSpatialTendons(ParseContext& ctx, ObjectKey key, const SpatialTendonParseInfo& info,
                         std::vector<DescPtr<PhysxTendonAttachmentDesc>>& out)
{
    std::vector<PreparedSpatialTendon> inputs;
    prepareSpatialTendons(ctx, key, info, inputs);
    size_t roots = 0, leaves = 0;
    for (const PreparedSpatialTendon& input : inputs)
    {
        roots += input.type == eTendonAttachmentRoot;
        leaves += input.type == eTendonAttachmentLeaf;
    }
    if (!reserveDescriptors<PhysxTendonSpatialDesc>(ctx.descriptorAllocator(), roots) ||
        !reserveDescriptors<PhysxTendonAttachmentLeafDesc>(ctx.descriptorAllocator(), leaves) ||
        !reserveDescriptors<PhysxTendonAttachmentDesc>(ctx.descriptorAllocator(), inputs.size() - roots - leaves)) throw std::bad_alloc();
    for (const PreparedSpatialTendon& input : inputs)
    {
        DescPtr<PhysxTendonAttachmentDesc> desc;
        if (input.type == eTendonAttachmentRoot) desc = allocateDesc<PhysxTendonSpatialDesc, PhysxTendonAttachmentDesc>(ctx.descriptorAllocator());
        else if (input.type == eTendonAttachmentLeaf) desc = allocateDesc<PhysxTendonAttachmentLeafDesc, PhysxTendonAttachmentDesc>(ctx.descriptorAllocator());
        else desc = allocateDesc<PhysxTendonAttachmentDesc>(ctx.descriptorAllocator());
        if (desc && fillSpatialTendon(input, *desc)) out.push_back(std::move(desc));
    }
}

} // namespace omni::physics::parse
