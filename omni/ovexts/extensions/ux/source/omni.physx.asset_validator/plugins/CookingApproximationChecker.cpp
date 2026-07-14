// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>

#include <common/utilities/Utilities.h> // intToPath
#include <common/foundation/TypeCast.h>
#include <carb/settings/ISettings.h>

using namespace physx;
using namespace omni::physx;


bool convexGPUCompatibilityChecker(uint64_t stageId, uint64_t primId)
{
    IPhysxCooking* iPhysxCooking = carb::getCachedInterface<IPhysxCooking>();
    IPhysxCookingPrivate* iPhysxCookingPrivate = carb::getCachedInterface<IPhysxCookingPrivate>();
    IPhysx* iPhysx = carb::getCachedInterface<IPhysx>();
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
    {
        CARB_LOG_ERROR("convexGPUCompatibilityChecker: Invalid stageId \"%lld\" passed", stageId);
        return false;
    }
    PXR_NS::SdfPath primToCheck(intToPath(primId));
    if (primToCheck.IsEmpty())
    {
        CARB_LOG_ERROR("convexGPUCompatibilityChecker: Invalid primId \"%lld\" passed", primId);
        return false;
    }

    // Check if primToCheck has a Convex Hull approximation
    PXR_NS::UsdPrim mesh = stage->GetPrimAtPath(primToCheck);
    if (!mesh || !mesh.IsA<PXR_NS::UsdGeomMesh>())
    {
        CARB_LOG_WARN("convexGPUCompatibilityChecker: Prim \"%s\" is not a UsdGeomMesh", primToCheck.GetText());
        return true;
    }
    const PXR_NS::UsdPhysicsMeshCollisionAPI colMeshAPI = PXR_NS::UsdPhysicsMeshCollisionAPI::Get(stage, primToCheck);
    if (!colMeshAPI)
    {
        CARB_LOG_ERROR(
            "convexGPUCompatibilityChecker: Prim \"%s\" has no Convex Hull approximation", primToCheck.GetText());
        return false;
    }
    // Acquire cooking statistics
    PhysxCookingStatistics cookingStats = iPhysxCookingPrivate->getCookingStatistics();

    PXR_NS::TfToken approximation;
    colMeshAPI.GetApproximationAttr().Get(&approximation);
    if (approximation == PXR_NS::UsdPhysicsTokens->convexHull)
    {
        ConvexMeshCookingParams params;
        iPhysxCooking->precookMesh(stageId, primId, params, nullptr); // No callback, blocking call
    }
    else if (approximation == PXR_NS::UsdPhysicsTokens->convexDecomposition)
    {
        ConvexDecompositionCookingParams params;
        iPhysxCooking->precookMesh(stageId, primId, params, nullptr); // No callback, blocking call
    }
    else
    {
        CARB_LOG_ERROR(
            "convexGPUCompatibilityChecker: Prim \"%s\" has an unsupported approximation type", primToCheck.GetText());
        return false;
    }

    // get cooking statistics again
    PhysxCookingStatistics cookingStats2 = iPhysxCookingPrivate->getCookingStatistics();
    if (cookingStats2.totalWarningsFailedGPUCompatibility - cookingStats.totalWarningsFailedGPUCompatibility > 0)
    {
        CARB_LOG_WARN(
            "convexGPUCompatibilityChecker: Convex hull for \"%s\" is not GPU compatible", primToCheck.GetText());
        return false;
    }

    return true;
}
