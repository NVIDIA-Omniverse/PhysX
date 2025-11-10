// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>

#include <common/utilities/Utilities.h> // intToPath
#include <common/foundation/TypeCast.h>

using namespace physx;
using namespace omni::physx;

// TODO: getGfMatrix4d is copy pasted from InternalScene.cpp and there is another copy in PhysicsSchemaHelper.cpp
// Should we consolidate all of them into TypeCast.h or some other common place?
struct Transform
{
    pxr::GfVec3f position;
    pxr::GfQuatf orientation;
    pxr::GfVec3f scale;
};

static PXR_NS::GfMatrix4d getGfMatrix4d(const Transform& transform)
{
    PXR_NS::GfMatrix4d mat;
    PXR_NS::GfMatrix4d rotMat;
    PXR_NS::GfMatrix4d scaleMat;

    scaleMat.SetScale(transform.scale);
    rotMat.SetRotate(transform.orientation);
    mat = scaleMat * rotMat;
    mat.SetTranslateOnly(transform.position);

    return mat;
}

bool applyJointStates(PxArticulationReducedCoordinate& articulation, pxr::UsdStageRefPtr stage, bool applyModification)
{
    pxr::UsdGeomXformCache xformCache;

    PxU32 numLinks = articulation.getNbLinks();
    for (PxU32 idx = 0; idx < numLinks; ++idx)
    {
        PxArticulationLink* link = nullptr;
        articulation.getLinks(&link, 1, idx);
        PxTransform linkTransform = link->getGlobalPose();
    }
    IPhysx* iPhysx = carb::getCachedInterface<IPhysx>();
    if (articulation.getScene())
    {
        articulation.updateKinematic(PxArticulationKinematicFlag::ePOSITION);
    }
    bool isCoherent = true;
    for (PxU32 idx = 0; idx < numLinks; ++idx)
    {
        PxArticulationLink* link = nullptr;
        articulation.getLinks(&link, 1, idx);
        PxTransform linkTransform = link->getGlobalPose();
        usdparser::ObjectId linkId = (usdparser::ObjectId)link->userData;
        pxr::SdfPath linkPath = iPhysx->getPhysXObjectUsdPath(linkId);
        pxr::UsdPrim prim = stage->GetPrimAtPath(linkPath);
        if (prim)
        {
            const pxr::GfMatrix4d sourceMatrix = xformCache.GetLocalToWorldTransform(prim);

            PxTransform pose;
            PxMeshScale scale;
            omni::physx::toPhysX(pose, scale.scale, sourceMatrix);

            Transform transform;
            transform.position = toVec3f(linkTransform.p);
            transform.scale = toVec3f(scale.scale); // Are we scale rotation?
            transform.orientation =
                pxr::GfQuatf(linkTransform.q.w, { linkTransform.q.x, linkTransform.q.y, linkTransform.q.z });
            const pxr::GfMatrix4d wantedTransform = getGfMatrix4d(transform);

            const pxr::GfMatrix4d existingTransform = xformCache.GetLocalToWorldTransform(prim);
            if (!pxr::GfIsClose(existingTransform, wantedTransform, 1e-6))
            {
                isCoherent = false;
                if (applyModification)
                {
                    CARB_LOG_INFO("JointStateChecker: Modify link \"%s\" transform to make it coherent with Joint State",
                                  linkPath.GetText());
                    omni::usd::UsdUtils::setLocalTransformFromWorldTransformMatrix(prim, wantedTransform);
                }
                else
                {
                    CARB_LOG_WARN("JointStateChecker: Link \"%s\" transform is not coherent with Joint State",
                                  linkPath.GetText());
                }
            }
        }
    }
    return isCoherent;
}

template <typename Lambda>
void forEachScene(uint64_t stageId, Lambda&& lambda)
{
    auto* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    auto* iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    IPhysx* iPhysx = carb::getCachedInterface<IPhysx>();

    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);
    PxScene* scene = nullptr;
    if (usdrtStage)
    {
        for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsScene")))
        {
            const omni::fabric::PathC pathC(usdrtPath);
            const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
            if (!usdPath.IsEmpty())
            {
                scene = static_cast<PxScene*>(iPhysx->getPhysXPtr(usdPath, omni::physx::ePTScene));
                if (scene != nullptr)
                {
                    lambda(scene);
                }
                else
                {
                    CARB_LOG_WARN("JointStateChecker: null PxScene* for path \"%s\"", usdPath.GetText());
                }
            }
        }
    }
    else
    {
        CARB_LOG_ERROR("JointStateChecker: Invalid USDRT stage for stageId \"%lld\"", stageId);
    }
}

bool jointStateChecker(uint64_t stageId, uint64_t primId, bool applyModification)
{
    IPhysxSimulation* iPhysxSimulation = carb::getCachedInterface<IPhysxSimulation>();
    IPhysx* iPhysx = carb::getCachedInterface<IPhysx>();
    pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
    {
        CARB_LOG_ERROR("JointStateChecker: Invalid stageId \"%lld\" passed", stageId);
        return false;
    }
    pxr::SdfPath rootArticulationPath(intToPath(primId));
    if (rootArticulationPath.IsEmpty())
    {
        CARB_LOG_ERROR("JointStateChecker: Invalid primId \"%lld\" passed", primId);
        return false;
    }

    // We need to find all articulations living "under a given ArticulationRootAPI".
    // We can't use IPhysx::getObjectId(path, ePTArticulation) as it will only return the first articulation.
    bool articulationFound = false;
    bool isCoherent = true;
    auto fixArticulation = [&](::physx::PxScene* scene) {
        const PxU32 nbArticulations = scene->getNbArticulations();
        PxArticulationReducedCoordinate* articulation = nullptr;
        for (PxU32 i = 0; i < nbArticulations; i++)
        {
            scene->getArticulations(&articulation, 1, i);
            pxr::SdfPath articulationPath = iPhysx->getPhysXObjectUsdPath((usdparser::ObjectId)articulation->userData);
            if (articulationPath.HasPrefix(rootArticulationPath))
            {
                if (applyModification)
                {
                    CARB_LOG_INFO("JointStateChecker: Fix articulation \"%s\"", articulationPath.GetText());
                }
                else
                {
                    CARB_LOG_INFO("JointStateChecker: Inspect articulation \"%s\"", articulationPath.GetText());
                }
                if (!applyJointStates(*articulation, stage, applyModification))
                {
                    isCoherent = false;
                }
                articulationFound = true;
            }
        }
    };
    forEachScene(stageId, fixArticulation);
    if (!articulationFound)
    {
        CARB_LOG_ERROR("JointStateChecker: \"%s\" has no articulations", rootArticulationPath.GetString().data());
        return false;
    }
    return isCoherent;
}
