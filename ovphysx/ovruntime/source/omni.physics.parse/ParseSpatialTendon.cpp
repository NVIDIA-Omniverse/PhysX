// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <string>
#include <unordered_set>

namespace omni::physics::parse
{

namespace
{

// Read a clamped float from `linkKey.attr`. Returns the unchanged `out`
// when the attribute is unauthored or wrong-kind; clamps ±inf to the
// bounds and out-of-range values to the nearest bound.
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

// Read `physxTendon:<instance>:localPos`. The authored localPos is
// multiplied by the link prim's world-space scale (per-axis); the
// walker pre-resolves that scale and passes it in `info.linkWorldScale`.
carb::Float3 readLocalPos(IPhysicsSource& src, ObjectKey key, const std::string& base,
                          const carb::Float3& scale)
{
    carb::Float3 result{ 0.0f, 0.0f, 0.0f };
    src.getAttribute(key, src.internToken(base + "localPos"), result);
    result.x *= scale.x;
    result.y *= scale.y;
    result.z *= scale.z;
    return result;
}

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

// Apply attachment-common fields (gearing / localPos / link / parent /
// instanceToken) to `desc`. Used by both the root and the
// intermediate/leaf paths.
void fillAttachmentCommon(IPhysicsSource& src, ObjectKey linkKey, const std::string& instance,
                         const SpatialTendonParseInfo& info,
                         PhysxTendonAttachmentDesc& desc, bool isRoot)
{
    const std::string base = std::string("physxTendon:") + instance + ":";

    if (!isRoot)
    {
        // Root attachments force gearing = 1.0 after common fill.
        readClampedFloat(src, linkKey, base + "gearing", -FLT_MAX, FLT_MAX, desc.gearing);
    }
    desc.localPos      = readLocalPos(src, linkKey, base, info.linkWorldScale);
    desc.linkKey      = linkKey;
    desc.instanceToken = src.internToken(instance);
    desc.parentToken   = readParentAttachmentToken(src, linkKey, base);
    desc.parentKey    = readParentLink(src, linkKey, base, src.sourceKeyToString(linkKey));
}

} // namespace

// @implements REQ-PARSE-TENDON-001
// @covers AC-1 AC-2 AC-3 AC-4
void parseSpatialTendons(ParseContext& ctx, ObjectKey linkKey,
                         const SpatialTendonParseInfo& info,
                         std::vector<DescPtr<PhysxTendonAttachmentDesc>>& out)
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

            DescPtr<PhysxTendonSpatialDesc> root =
                allocateDesc<PhysxTendonSpatialDesc>(ctx.descriptorAllocator());
            fillAttachmentCommon(src, linkKey, instance, info, *root, /*isRoot=*/true);
            root->gearing = 1.0f; // root attachments always have gearing 1.0
            root->isEnabled = true;
            readBool       (src, linkKey, base + "tendonEnabled",                   root->isEnabled);
            readClampedFloat(src, linkKey, base + "stiffness",      0.0f, FLT_MAX, root->stiffness);
            readClampedFloat(src, linkKey, base + "limitStiffness", 0.0f, FLT_MAX, root->limitStiffness);
            readClampedFloat(src, linkKey, base + "damping",        0.0f, FLT_MAX, root->damping);
            readClampedFloat(src, linkKey, base + "offset",      -FLT_MAX, FLT_MAX, root->offset);
            out.push_back(descPtrCast<PhysxTendonAttachmentDesc>(std::move(root)));
        });

    // Leaf next. Run before the Attachment iter so it can silently
    // ignore the auto-applied AttachmentAPI:<leaf-instance> entries.
    src.forEachMultiApplyInstance(linkKey, "PhysxTendonAttachmentLeafAPI",
        [&](std::string_view inst) {
            if (!claim(inst)) return;
            claimedByRootOrLeaf.insert(std::string(inst));
            const std::string instance(inst);
            const std::string base = std::string("physxTendon:") + instance + ":";

            DescPtr<PhysxTendonAttachmentLeafDesc> leaf =
                allocateDesc<PhysxTendonAttachmentLeafDesc>(ctx.descriptorAllocator());
            fillAttachmentCommon(src, linkKey, instance, info, *leaf, /*isRoot=*/false);
            readClampedFloat(src, linkKey, base + "restLength",  -FLT_MAX, FLT_MAX, leaf->restLength);
            readClampedFloat(src, linkKey, base + "lowerLimit",  -FLT_MAX, FLT_MAX, leaf->lowLimit);
            readClampedFloat(src, linkKey, base + "upperLimit", leaf->lowLimit, FLT_MAX, leaf->highLimit);
            out.push_back(descPtrCast<PhysxTendonAttachmentDesc>(std::move(leaf)));
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

            DescPtr<PhysxTendonAttachmentDesc> desc =
                allocateDesc<PhysxTendonAttachmentDesc>(ctx.descriptorAllocator());
            fillAttachmentCommon(src, linkKey, instance, info, *desc, /*isRoot=*/false);
            out.push_back(std::move(desc));
        });
}

} // namespace omni::physics::parse
