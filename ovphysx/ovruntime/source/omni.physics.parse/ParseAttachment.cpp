// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedAttachment.h>
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

} // anonymous

PreparedAttachment prepareAttachment(ParseContext& ctx, ObjectKey key, ObjectType type)
{
    const KnownTokens& tok = ctx.knownTokens();
    PreparedAttachment input;
    input.key = key;
    input.type = type;
    input.enabled = prepareAttribute<bool>(ctx.source(), key, tok.omniphysicsAttachmentEnabled);
    input.valid = readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc0, input.src0) &&
                  readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc1, input.src1);
    if (input.valid)
    {
        input.damping = prepareAttribute<float>(ctx.source(), key, tok.omniphysicsDamping);
        input.stiffness = prepareAttribute<float>(ctx.source(), key, tok.omniphysicsStiffness);
    }
    input.prepared = true;
    return input;
}

PreparedAttachment prepareElementCollisionFilter(ParseContext& ctx, ObjectKey key)
{
    const KnownTokens& tok = ctx.knownTokens();
    PreparedAttachment input;
    input.key = key;
    input.enabled = prepareAttribute<bool>(ctx.source(), key, tok.omniphysicsFilterEnabled);
    input.valid = readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc0, input.src0) &&
                  readSingleTargetRel(ctx.source(), key, tok.omniphysicsSrc1, input.src1);
    input.prepared = true;
    return input;
}

bool fillAttachment(const PreparedAttachment& input, PhysxDeformableAttachmentDesc& output) noexcept
{
    if (!input.prepared || !input.valid) return false;
    output.type = input.type;
    output.enabled = input.enabled.valueOr(true);
    output.src0 = input.src0;
    output.src1 = input.src1;
    output.damping = input.damping.valueOr(0.0f);
    output.stiffness = input.stiffness.valueOr(0.0f);
    return true;
}

bool fillElementCollisionFilter(const PreparedAttachment& input, PhysxDeformableCollisionFilterDesc& output) noexcept
{
    if (!input.prepared || !input.valid) return false;
    output.enabled = input.enabled.valueOr(true);
    output.src0 = input.src0;
    output.src1 = input.src1;
    return true;
}

DescPtr<PhysxDeformableAttachmentDesc> parseAttachment(ParseContext& ctx, ObjectKey key, ObjectType type)
{
    const PreparedAttachment input = prepareAttachment(ctx, key, type);
    if (!input.valid) return {};
    DescPtr<PhysxDeformableAttachmentDesc> output = allocateDesc<PhysxDeformableAttachmentDesc>(ctx.descriptorAllocator());
    if (output) fillAttachment(input, *output);
    return output;
}
DescPtr<PhysxDeformableCollisionFilterDesc> parseElementCollisionFilter(ParseContext& ctx, ObjectKey key)
{
    const PreparedAttachment input = prepareElementCollisionFilter(ctx, key);
    if (!input.valid) return {};
    DescPtr<PhysxDeformableCollisionFilterDesc> output = allocateDesc<PhysxDeformableCollisionFilterDesc>(ctx.descriptorAllocator());
    if (output) fillElementCollisionFilter(input, *output);
    return output;
}

} // namespace omni::physics::parse
