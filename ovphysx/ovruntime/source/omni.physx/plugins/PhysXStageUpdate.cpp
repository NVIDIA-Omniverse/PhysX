// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2 AC-3 AC-4

#include "usdLoad/LoadUsd.h"
#include "usdLoad/PrimUpdate.h"
#include "OmniPhysX.h"
#include "PhysXUpdate.h"
#include "Raycast.h"
#include "PhysXStageUpdate.h"
#include "PhysXPropertyQuery.h"
#include "RuntimeExecution.h"
#include <private/omni/physx/IPhysxStageUpdate.h>
#include <optional>

namespace omni::physx
{
AttachHandle onPhysXAttach(long stageId)
{
    RuntimeExecutionScope execution(kNoAttach);
    if (!execution || !OmniPhysX::getInstanceCheck())
        return kNoAttach;
    AttachHandle attach = kNoAttach;
    try
    {
        attach = OmniPhysX::getInstance().physXAttach(stageId, false);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Stage-update attachment failed");
        return kNoAttach;
    }
    if (attach != kNoAttach)
        OmniPhysX::getInstance().sendSimulationEvent(eAttachedToStage, attach);
    return attach;
}

RuntimeResult onPhysXDetach(AttachHandle attach)
{
    if (attach == kNoAttach || attach == kActiveAttach)
        return RuntimeResult::eInvalidAttachment;
    RuntimeExecutionScope configuration(kNoAttach);
    if (!configuration)
        return RuntimeResult::eBusy;
    RuntimeExecutionScope execution(attach, true);
    if (!execution)
        return RuntimeResult::eBusy;
    if (!OmniPhysX::getInstanceCheck() || !OmniPhysX::getInstance().physXDetach(attach))
        return RuntimeResult::eInvalidAttachment;
    OmniPhysX::getInstance().sendSimulationEvent(eDetachedFromStage, attach);
    return RuntimeResult::eSuccess;
}

RuntimeResult physXPause(AttachHandle attach)
{
    RuntimeExecutionScope execution(attach);
    if (!execution)
        return RuntimeResult::eBusy;
    usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    owner->setSimulationRunning(false);
    OmniPhysX::getInstance().sendSimulationEvent(SimulationEvent::ePaused, attach);
    return RuntimeResult::eSuccess;
}

RuntimeResult physXReset(AttachHandle attach)
{
    RuntimeExecutionScope execution(attach, true);
    if (!execution)
        return RuntimeResult::eBusy;
    usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    OmniPhysX::getInstance().resetSimulation(*owner);
    owner->resetSimulationStepCount();
    if (owner->hasTempPhysicsScene())
    {
        owner->removeDefaultPhysicsScenePlaceholder(owner->getTempPhysicsSceneKey());
        owner->setTempPhysicsSceneKey({});
    }
    return RuntimeResult::eSuccess;
}

RuntimeResult physXResume(AttachHandle attach, float currentTime)
{
    RuntimeExecutionScope execution(attach);
    if (!execution)
        return RuntimeResult::eBusy;
    usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    OmniPhysX& runtime = OmniPhysX::getInstance();
    const bool wasStopped = !owner->hasSimulationStarted();
    owner->getRaycastManager().clearCommandBuffer();
    owner->setSimulationStarted(true);
    {
        std::optional<InitialStagePopulationScope> population;
        if (wasStopped)
            population.emplace(getPhysXUsdPhysicsInterface(), *owner);
        usdparser::processUpdates(*owner, currentTime);
    }
    if (wasStopped)
        getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(*owner, true);
    owner->setSimulationRunning(true);
    runtime.sendSimulationEvent(SimulationEvent::eResumed, attach);
    runtime.getPhysXSetup().resetPhysXErrorCounter();
    return RuntimeResult::eSuccess;
}

RuntimeResult physXStageUpdate(AttachHandle attach, float currentTime, float elapsedSecs, bool enableUpdate)
{
    RuntimeExecutionScope execution(attach);
    if (!execution)
        return RuntimeResult::eBusy;
    usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    physXUpdate(*owner, currentTime, elapsedSecs, enableUpdate && owner->isSimulationRunning());
    // The adapter is synchronous under the same runtime admission contract.
    waitForSimulationCompletion(*owner, false);
    return RuntimeResult::eSuccess;
}

RuntimeResult updatePropertyQueries()
{
    RuntimeExecutionScope execution(kNoAttach);
    if (!execution)
        return RuntimeResult::eBusy;
    OmniPhysX::getInstance().getPropertyQueryManager().updateQueuedRequests();
    return RuntimeResult::eSuccess;
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
    iface.updatePropertyQueries = omni::physx::updatePropertyQueries;
}
