// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-8
 */

#include <carb/logging/Log.h>

#include "VoxelMap.h"
#include "PhysXTools.h"

#include "usdLoad/LoadUsd.h"

using namespace carb;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

bool setVoxelRange(long int stageId, omni::physics::parse::ObjectKey key, const int sx, const int sy, const int sz, const int ex, const int ey, const int ez, const int type, const int subType, const int update)
{
    // Source-backed type/schema dispatch keyed by ObjectKey: "Xform" is the registered USD
    // prim-type name for UsdGeomXform, "InfiniteVoxelMapAPI" the applied-schema name -- same
    // pattern as LoadStage.cpp's voxel-map scan. The argument checks still report the usual
    // errors; a well-formed call then fails because the voxel map itself is unsupported.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("setVoxelRange failed due to a missing attache stage.");
        return false;
    }

    const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
    if (!src || !src->exists(key) || !src->isA(key, src->internToken("Xform")))
    {
        CARB_LOG_ERROR("setVoxelRange input prim is not an Xform.");
        return false;
    }

    if (!src->hasSchema(key, src->internToken("InfiniteVoxelMapAPI")))
    {
        CARB_LOG_ERROR("setVoxelRange input prim does not have an InfiniteVoxelMapAPI applied.");
        return false;
    }

    CARB_UNUSED(sx, sy, sz, ex, ey, ez, type, subType, update);
    CARB_LOG_ERROR("setVoxelRange: InfiniteVoxelMapAPI voxel maps are not supported by the USD-free runtime.");
    return false;
}
