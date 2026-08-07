// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/IPhysxStageUpdate.h>

#include <optional>

#include <common/utilities/Utilities.h>

#include <PhysXSettings.h>

#include "usdLoad/LoadUsd.h"
#include "OmniPhysX.h"
#include "PhysXUpdate.h"
#include "Raycast.h"
#include "PhysXStageUpdate.h"
#include "PhysXPropertyQuery.h"

using namespace PXR_NS;

namespace omni
{
namespace physx
{
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

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        if (omniPhysX.hasTempPhysicsScene())
        {
            UsdStageWeakPtr stage = usdparser::UsdLoad::getUsdLoad()->getActiveStage();
            ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
            stage->RemovePrim(omniPhysX.getTempPhysicsScenePath());
            omniPhysX.setHasTempPhysicsScene(false);
        }

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
        // Refresh CUDA synchronous-launch state if a context was already lazily
        // created (at the first GPU scene attach). Never create one here -- that
        // would open a CUDA context on a CPU-only resume.
        {
            PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
            if (physxSetup.getCudaContextManager())
                physxSetup.setupGPU();
        }

        // Make sure physics is created before we start to do anything
        OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const bool wasSimulationStopped = omniPhysX.setSimulationStarted(true);

        omniPhysX.getRaycastManager().clearCommandBuffer();

        getPhysXUsdPhysicsInterface().setExposePrimNames(omniPhysX.getISettings()->getAsBool(kSettingExposePrimPathNames));
        omniPhysX.setSimulationStarted(true);
        if (omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(PXR_NS::SdfPath(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene)));
        else
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(PXR_NS::SdfPath());
        {
            // On the initial attach (simulation was stopped) suppress
            // initial-population notifications; the scope restores both gates
            // even if update() throws.
            std::optional<InitialStagePopulationScope> populationScope;
            if (wasSimulationStopped)
                populationScope.emplace(getPhysXUsdPhysicsInterface());

            usdparser::UsdLoad::getUsdLoad()->update(currentTime);
        }

        if (wasSimulationStopped)
        {
            getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(true);
            // now the initial load is done and notifications should be sent
        }

        omniPhysX.setOutputVelocitiesLocalSpace(omniPhysX.getISettings()->getAsBool(kSettingOutputVelocitiesLocalSpace));

        omniPhysX.sendSimulationEvent(SimulationEvent::eResumed);
        omniPhysX.setSimulationRunning(true);
        omniPhysX.getPhysXSetup().resetPhysXErrorCounter();
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
    iface.onDetach = omni::physx::onPhysXDetach;
    iface.onPause = omni::physx::physXPause;
    iface.onReset = omni::physx::physXReset;
    iface.onResume = omni::physx::physXResume;
    iface.onUpdate = omni::physx::physXStageUpdate;
    iface.handleRaycast = omni::physx::handleRaycast;
}
