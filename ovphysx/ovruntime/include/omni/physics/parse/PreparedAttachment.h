// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once
#include "PreparedAttribute.h"
#include "Descriptors.h"
namespace omni::physics::parse
{
class ParseContext;
struct PreparedAttachment
{
    bool prepared = false, valid = false;
    ObjectKey key, src0, src1;
    ObjectType type = eUndefined;
    PreparedAttribute<bool> enabled;
    PreparedAttribute<float> damping, stiffness;
};
PreparedAttachment prepareAttachment(ParseContext& context, ObjectKey key, ObjectType type);
PreparedAttachment prepareElementCollisionFilter(ParseContext& context, ObjectKey key);
bool fillAttachment(const PreparedAttachment& input, PhysxDeformableAttachmentDesc& output) noexcept;
bool fillElementCollisionFilter(const PreparedAttachment& input, PhysxDeformableCollisionFilterDesc& output) noexcept;
}
