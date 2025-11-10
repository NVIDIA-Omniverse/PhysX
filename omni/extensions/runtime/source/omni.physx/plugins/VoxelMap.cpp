// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>

#include "VoxelMap.h"
#include "PhysXTools.h"

#include "usdLoad/LoadUsd.h"

using namespace pxr;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

bool setVoxelRange(long int stageId, const pxr::SdfPath& path, const int sx, const int sy, const int sz, const int ex, const int ey, const int ez, const int type, const int subType, const int update)
{
    // try and find USD stage from Id
    UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        CARB_LOG_ERROR("setVoxelRange was unable to find a USD stage.");
        return false;
    }

    UsdPrim inputPrim = stage->GetPrimAtPath(path);
    if (!inputPrim || !inputPrim.IsA<UsdGeomXform>())
    {
        CARB_LOG_ERROR("setVoxelRange input prim is not an Xform.");
        return false;
    }

    // check for InfiniteVoxelMapAPI presence
    const TfTokenVector& appliedSchemas = inputPrim.GetPrimTypeInfo().GetAppliedAPISchemas(); 
    static auto isVoxelSchema = [](const TfToken& token) { return token == gInfiniteVoxelMapAPI; };
    if (std::none_of(appliedSchemas.begin(), appliedSchemas.end(), isVoxelSchema))
    {
        CARB_LOG_ERROR("setVoxelRange input prim does not have an InfiniteVoxelMapAPI applied.");
        return false;
    }

    // try to update voxels
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("setVoxelRange failed due to a missing attache stage.");
        return false;
    }

    ObjectId id = attachedStage->getObjectDatabase()->findEntry(path, eInfiniteVoxelMap);
    if (id == kInvalidObjectId)
    {
        CARB_LOG_ERROR("setVoxelRange failed due to a parsing error.");
        return false;
    }

    void* ptr = OmniPhysX::getInstance().getInternalPhysXDatabase().getInternalTypedRecord(ePTInfiniteVoxelMap, id);
    CARB_ASSERT(ptr);

    InternalInfiniteVoxelMap* infiniteVoxelMap = reinterpret_cast<InternalInfiniteVoxelMap*>(ptr);
    infiniteVoxelMap->mInfiniteVoxelMap.setUpdateVoxel(sx, sy, sz, ex, ey, ez, type, subType, update);
    return true;
}
