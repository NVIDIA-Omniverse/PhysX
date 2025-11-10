// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/IPhysxStageUpdate.h>

#include <common/utilities/Utilities.h>

#include <PhysXSettings.h>

#include "usdLoad/LoadUsd.h"
#include "OmniPhysX.h"
#include "PhysXUpdate.h"
#include "Raycast.h"
#include "PhysXStageUpdate.h"
#include "PhysXPropertyQuery.h"

using namespace pxr;

namespace omni
{
namespace physx
{
void onPhysXFabricAttach(long int stageId)
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    if (OmniPhysX::getInstance().getStageUpdate().isAttached())
    {
        usdparser::UsdLoad::getUsdLoad()->fabricAttach(stageId);
        usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(stageId, true);
    }
        
}

void onPhysXAttach(long int stageId)
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    OmniPhysX::getInstance().setSimulationAttachedStage(false);
    OmniPhysX::getInstance().getStageUpdate().attach(true);
    OmniPhysX::getInstance().getStageUpdate().setAttachedStage(stageId);
    OmniPhysX::getInstance().physXAttach(stageId, false);
    OmniPhysX::getInstance().sendSimulationEvent(eAttachedToStage);
}

void onPhysXDetach()
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    OmniPhysX::getInstance().setSimulationAttachedStage(false);
    OmniPhysX::getInstance().getStageUpdate().attach(false);
    OmniPhysX::getInstance().getStageUpdate().setAttachedStage(0);
    OmniPhysX::getInstance().physXDetach();
    OmniPhysX::getInstance().sendSimulationEvent(eDetachedFromStage);
}

void physXPause()
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    if (OmniPhysX::getInstance().getStageUpdate().isAttached())
    {
        OmniPhysX::getInstance().setSimulationRunning(false);
        OmniPhysX::getInstance().sendSimulationEvent(SimulationEvent::ePaused);
    }
}

void physXReset()
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    if (OmniPhysX::getInstance().getStageUpdate().isAttached())
    {
        OmniPhysX::getInstance().resetSimulation();

        long stageId = OmniPhysX::getInstance().getStageId();
        usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(stageId, true);

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        if (omniPhysX.hasTempPhysicsScene())
        {
            UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
            ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
            stage->RemovePrim(omniPhysX.getTempPhysicsScenePath());
            omniPhysX.setHasTempPhysicsScene(false);
        }

        // A.B. Makes sure physics is created, do we need this?
        omniPhysX.getPhysXSetup().getPhysics();
        // reset the current simulation timestamp offset
        omniPhysX.setCurrentTimestampOffset(omniPhysX.getSimulationTimestamp());
    }
}

void physXResume(float currentTime)
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    if (OmniPhysX::getInstance().getStageUpdate().isAttached())
    {
        // CUDA synchronous launch check, setting could have changed
        // Allows runtime CUDA error debugging
        OmniPhysX::getInstance().getPhysXSetup().setupGPU();

        // Make sure physics is created before we start to do anything
        OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const bool wasSimulationStopped = omniPhysX.setSimulationStarted(true);

        // create the raycast command buffer
        omniPhysX.getRaycastManager().clearCommandBuffer();

        getPhysXUsdPhysicsInterface().setExposePrimNames(omniPhysX.getISettings()->getAsBool(kSettingExposePrimPathNames));
        omniPhysX.setSimulationStarted(true);
        if (omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(pxr::SdfPath(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene)));
        else
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(pxr::SdfPath());
        if (wasSimulationStopped)
            getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(false);

        usdparser::UsdLoad::getUsdLoad()->update(currentTime);

        if (wasSimulationStopped)
        {
            getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(true);
            // now the initial load is done and notifications should be sent
        }

        if (wasSimulationStopped)
        {
            const long stageId = OmniPhysX::getInstance().getStageId();
            usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(stageId, false);
        }

        omniPhysX.setOutputVelocitiesLocalSpace(omniPhysX.getISettings()->getAsBool(kSettingOutputVelocitiesLocalSpace));

        omniPhysX.sendSimulationEvent(SimulationEvent::eResumed);
        omniPhysX.setSimulationRunning(true);
        omniPhysX.getPhysXSetup().resetPhysXErrorCounter();

        if (omniPhysX.getPhysXStats())
        {
            omniPhysX.getPhysXStats()->setStage(omniPhysX.getStage());
        }
    }
}

void physXStageUpdate(float currentTime, float elapsedSecs, bool enableUpdate)
{
    if (!OmniPhysX::getInstanceCheck())
        return;

    const PhysXStageUpdate& su = OmniPhysX::getInstance().getStageUpdate();
    if (su.isAttached())
    {
        physXUpdate(currentTime, elapsedSecs, enableUpdate);
    }
    if (su.isPropertyQueryAttached())
    {
        OmniPhysX::getInstance().getPropertyQueryManager().updateQueuedRequests();
    }
}

void PhysXStageUpdate::attachStageUpdate()
{
    mAttached = true;
    if (mAttachedStage)
    {
        onPhysXAttach((long)mAttachedStage);
    }
}

void PhysXStageUpdate::detachStageUpdate()
{
    mAttached = false;
}

}
}
void fillInterface(omni::physx::IPhysxStageUpdate& iface)
{
    iface.onAttach = omni::physx::onPhysXAttach;
    iface.onFabricAttach = omni::physx::onPhysXFabricAttach;
    iface.onDetach = omni::physx::onPhysXDetach;
    iface.onPause = omni::physx::physXPause;
    iface.onReset = omni::physx::physXReset;
    iface.onResume = omni::physx::physXResume;
    iface.onUpdate = omni::physx::physXStageUpdate;
    iface.handleRaycast = omni::physx::handleRaycast;
}
