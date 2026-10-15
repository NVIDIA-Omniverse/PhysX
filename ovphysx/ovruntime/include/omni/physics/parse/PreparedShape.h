// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "PreparedCollision.h"

namespace omni::physics::parse
{

struct PreparedShapeCommon
{
    bool prepared = false;
    bool ownersResolved = true;
    SourceUnits units;
    ShapeInfo info;
    PreparedCollisionExt collision;
    std::vector<ObjectId> sceneIds;
};

PreparedShapeCommon prepareShapeCommon(ParseContext& context, ObjectKey key, const ShapeInfo& info);
// sceneIds must already have its final size. No allocation, lookup or logging
// occurs here; unresolved owners and incomplete storage return a fixed status.
bool fillShapeCommon(const PreparedShapeCommon& input, PhysxShapeDesc& output) noexcept;

} // namespace omni::physics::parse
