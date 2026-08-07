// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-ATTACH-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

// Attachment + element-collision-filter readers:
//   - bool attachmentEnabled / filterEnabled
//   - single-target src0 / src1 relationships (return nullptr if
//     either rel has != 1 target)
//   - float damping, stiffness (attachment only)

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

namespace omni::physics::parse
{

namespace
{

// Read a single-target relationship. Returns false on != 1 target.
bool readSingleTargetRel(IPhysicsSource& src,
                         ObjectKey key,
                         TokenId rel,
                         ObjectKey& out)
{
    std::vector<ObjectKey> targets;
    src.getRelationshipTargets(key, rel, targets);
    if (targets.size() != 1)
        return false;
    out = targets[0];
    return true;
}

bool readBoolAttr(const IPhysicsSource& src, ObjectKey key, TokenId attr, bool defaultVal)
{
    bool v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
}

float readFloatAttr(const IPhysicsSource& src, ObjectKey key, TokenId attr, float defaultVal)
{
    float v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
}

} // anonymous

DescPtr<PhysxDeformableAttachmentDesc> parseAttachment(ParseContext& ctx, ObjectKey key, ObjectType type)
{
    KnownTokens tok;
    tok.intern(ctx.source());

    DescPtr<PhysxDeformableAttachmentDesc> desc =
        allocateDesc<PhysxDeformableAttachmentDesc>(ctx.descriptorAllocator());
    desc->type = type;
    desc->enabled = readBoolAttr(ctx.source(), key, tok.omniphysicsAttachmentEnabled, true);

    if (!readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc0, desc->src0) ||
        !readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc1, desc->src1))
        return {};

    desc->damping   = readFloatAttr(ctx.source(), key, tok.omniphysicsDamping,   0.0f);
    desc->stiffness = readFloatAttr(ctx.source(), key, tok.omniphysicsStiffness, 0.0f);
    return desc;
}

DescPtr<PhysxDeformableCollisionFilterDesc> parseElementCollisionFilter(ParseContext& ctx, ObjectKey key)
{
    KnownTokens tok;
    tok.intern(ctx.source());

    DescPtr<PhysxDeformableCollisionFilterDesc> desc =
        allocateDesc<PhysxDeformableCollisionFilterDesc>(ctx.descriptorAllocator());
    desc->enabled = readBoolAttr(ctx.source(), key, tok.omniphysicsFilterEnabled, true);

    if (!readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc0, desc->src0) ||
        !readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc1, desc->src1))
        return {};

    return desc;
}

} // namespace omni::physics::parse
