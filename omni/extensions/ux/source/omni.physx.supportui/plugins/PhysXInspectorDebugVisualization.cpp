// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
#include "PhysXInspector.h"
#include "PhysXInspectorModel.h"
#include "PhysXInspectorDebugVisualization.h"
#include <common/foundation/TypeCast.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>
#include "PhysXInspectorOverlay.h"
// clang-format on

using namespace physx;

void PhysXInspectorDebugVisualization::onStartup(PhysXInspector* inspector)
{
    mInspector = inspector;
    mDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();

    omni::kit::StageUpdatePtr stageUpdate =  omni::kit::getStageUpdate();
    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.displayName = "PhysxInspector Debug visualization";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysicsUI;
    desc.userData = this;
    desc.onPrimAdd = nullptr;
    desc.onPrimOrPropertyChange = nullptr;
    desc.onPrimRemove = nullptr;
    desc.onUpdate = [](float currentTime, float elapsedTime, const omni::kit::StageUpdateSettings* settings,
                       void* userData) { ((PhysXInspectorDebugVisualization*)userData)->updateDebugVisualization(); };
    desc.onStop = nullptr;
    desc.onResume = nullptr;
    desc.onPause = nullptr;
    if (stageUpdate)
        mStageUpdateNode = stageUpdate->createStageUpdateNode(desc);
}

void PhysXInspectorDebugVisualization::onShutdown()
{
    omni::kit::StageUpdatePtr stageUpdate =  omni::kit::getStageUpdate();
    if (mStageUpdateNode && stageUpdate)
    {
        stageUpdate->destroyStageUpdateNode(mStageUpdateNode);
        mStageUpdateNode = nullptr;
    }
}

void PhysXInspectorDebugVisualization::checkIfSomeBodyIsHovered()
{
    mIsSomeBodyHovered = false;
    for (auto it = mOverlays.begin(); it != mOverlays.end();)
    {
        auto ptr = it->lock();
        if (ptr)
        {
            if (ptr->isAnyBodyHovered())
            {
                mIsSomeBodyHovered = true;
            }
            ++it;
        }
        else
        {
            it = mOverlays.erase(it);
        }
    }
}
bool PhysXInspectorDebugVisualization::isBodyHovered(::physx::PxRigidBody* body) const
{
    for (auto it = mOverlays.begin(); it != mOverlays.end();)
    {
        auto ptr = it->lock();
        if (ptr)
        {
            if (ptr->isBodyHovered(body))
            {
                return true;
            }
        }
        ++it;
    }
    return false;
}

void PhysXInspectorDebugVisualization::registerOverlay(std::shared_ptr<omni::ui::scene::PhysXInspectorOverlayImpl> overlay)
{
    mOverlays.push_back(overlay);
}

void PhysXInspectorDebugVisualization::updateDebugVisualization()
{
    checkIfSomeBodyIsHovered();
    for (auto it : mInspector->mInspectorModels)
    {
        auto ptr = it.lock();
        if (ptr)
        {
            if (ptr->getShowMassesAndInertiaModel()->getValueAsBool())
            {
                drawViewportForModel(ptr.get());
            }
        }
    }
}

void PhysXInspectorDebugVisualization::drawViewportForModel(PhysXInspectorModelImpl* model)
{
    if (model->mSelectionIsArticulation)
    {
        PxArticulationReducedCoordinate* articulation = mInspector->getArticulationAt(model->mSelectedPrimPath);
        if (articulation && articulation->getNbLinks() > 0)
        {
            PxU32 numLinks = articulation->getNbLinks();
            for (PxU32 idx = 0; idx < numLinks; ++idx)
            {
                PxArticulationLink* link;
                articulation->getLinks(&link, 1, idx);
                drawInertiaCubeFor(link);
            }
        }
    }
    else
    {
        PxScene* scene = mInspector->getSceneAt(mInspector->mSelectedPhysicsScenePath.GetText());
        if (scene)
        {
            PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
            for (PxU32 idx = 0; idx < nbActors; idx++)
            {
                PxActor* pxActor = nullptr;
                scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &pxActor, 1, idx);
                if (!model->isIncludedInCurrentSelection(pxActor->userData))
                    continue;
                drawInertiaCubeFor((PxRigidBody*)pxActor);
            }
            const PxU32 nbArticulations = scene->getNbArticulations();
            PxArticulationReducedCoordinate* articulation = nullptr;
            for (PxU32 i = 0; i < nbArticulations; i++)
            {
                scene->getArticulations(&articulation, 1, i);
                if (!model->isIncludedInCurrentSelection(articulation->userData))
                    continue;
                PxU32 numLinks = articulation->getNbLinks();
                for (PxU32 idx = 0; idx < numLinks; ++idx)
                {
                    PxArticulationLink* link;
                    articulation->getLinks(&link, 1, idx);
                    drawInertiaCubeFor(link);
                }
            }
        }
    }
}


void PhysXInspectorDebugVisualization::drawInertiaCubeFor(::physx::PxRigidBody* body)
{
    // Logic for this code is taken from PhysX NpArticulationLink::visualize / NpRigidDynamic::visualize
    auto invertDiagInertia = [](const PxVec3& m) -> PxVec3 {
        return PxVec3(m.x == 0.0f ? 0.0f : 1.0f / m.x, m.y == 0.0f ? 0.0f : 1.0f / m.y, m.z == 0.0f ? 0.0f : 1.0f / m.z);
    };
    auto getDimsFromBodyInertia = [](const PxVec3& inertiaMoments, PxReal mass) -> PxVec3 {
        const PxVec3 inertia = inertiaMoments * (6.0f / mass);
        return PxVec3(PxSqrt(PxAbs(-inertia.x + inertia.y + inertia.z)),
                      PxSqrt(PxAbs(+inertia.x - inertia.y + inertia.z)),
                      PxSqrt(PxAbs(+inertia.x + inertia.y - inertia.z)));
    };
    bool isHovered = false;
    if (mIsSomeBodyHovered)
    {
        isHovered = isBodyHovered(body);
    }
    uint32_t boxColor = isHovered ? 0xff00ff00 : 0xffffffff;
    float lineWidth = isHovered ? 5.0f : 2.0f;
    auto dims = getDimsFromBodyInertia(invertDiagInertia(body->getMassSpaceInvInertiaTensor()), body->getMass());
    auto transform = body->getGlobalPose() * body->getCMassLocalPose();
    const auto extents = dims;
    carb::Float3 boxSize = omni::physx::toFloat3(extents);
    carb::Float3 boxPos = omni::physx::toFloat3(transform.p);
    carb::Float4 boxRotation = omni::physx::toFloat4(transform.q);
    mDebugDraw->drawBox(boxPos, boxRotation, boxSize, boxColor, lineWidth);
}
