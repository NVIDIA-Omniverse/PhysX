// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1

/**
 * @implements REQ-SIM-GPU-ERROR-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-ATTACH-SERVICES-001
 * @covers AC-3
 *
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include "PhysXUpdate.h"
#include "RuntimeExecution.h"
#include "OmniPhysX.h"

#include "internal/InternalScene.h"
#include "internal/InternalParticle.h"
#include "internal/InternalDeformable.h"
#include "usdInterface/UsdInterface.h"
#include "Raycast.h"
#include "Trigger.h"
#include "CookingDataAsync.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXTools.h"
#include "usdLoad/LoadUsd.h"
#include "PhysXScene.h"
#include "ContactReport.h"
#include "PhysXDefines.h"
#include "PhysXSimulationCallbacks.h"
#include "ScopedNoticeLock.h"

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/RuntimeError.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#if USE_PHYSX_GPU
#include <cudamanager/PxCudaContext.h>
#endif

#include "utils/Profile.h"

#include <optional>
#include <stdexcept>


using namespace ::physx;
using namespace carb;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;
using namespace cookingdataasync;


namespace omni
{
namespace physx
{

// A scene filter for the *Scene-suffixed entry points: nullopt means "no specific scene was
// requested" (the original empty-SdfPath sentinel); a present-but-possibly-invalid ObjectKey
// means "a specific scene was requested" (the original non-empty SdfPath), which must be kept
// distinct from "no filter" even when that key fails to resolve to any known object -- an
// unresolvable requested scene must skip every scene, not silently fall back to updating all of
// them.
using SceneFilter = std::optional<omni::physics::parse::ObjectKey>;

// Returns true if a scene must be skipped from a simulation update. If no scene filter is
// present, a simulation scene is only skipped if marked 'Disabled', otherwise if a specific
// scene was requested, we will only process that specific one.
static inline bool checkSkipScene(SceneFilter sceneFilter, const PhysXScene* sc)
{
    if (!sceneFilter.has_value())
    {
        // update all scenes in the simulation if not specifically disabled
        if (sc->getUpdateType() == eDisabled)
            return true;
    }
    else
    {
        // update only the scene that was specifically requested
        if (sc->getSceneSdfPath() != *sceneFilter)
            return true;
    }
    return false;
}

// Resolves a public-ABI `uint64_t scenePath` (see IPhysx.h: "Scene USD path encoded as
// uint64_t") into the SceneFilter the rest of this file operates on. `scenePath` IS the
// scene's `ObjectKey::handle` directly (ADR-0018, a breaking change) -- no SdfPath-bit
// decode, no attach lookup. 0 stays the "no specific scene" sentinel.
static SceneFilter resolveSceneKey(uint64_t scenePath)
{
    if (scenePath == 0)
        return std::nullopt;
    return omni::physics::parse::ObjectKey{ scenePath };
}

// Zero explicitly selects the attachment-wide operation. An unknown nonzero
// scene key must not run update hooks or consume completion from another scene.
static bool hasSelectedScene(const AttachedStage& attachedStage, uint64_t scenePath)
{
    if (scenePath == 0)
        return true;
    for (AttachedStage::SceneMap::const_reference entry : attachedStage.getPhysXScenes())
    {
        if (entry.second->getSceneSdfPath().handle == scenePath)
            return true;
    }
    return false;
}

void waitForSimulationCompletion(const AttachedStage& attachedStage, bool doPostWork)
{
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        sc->waitForCompletion(doPostWork);
    }
}

static bool physxCheckResultsInternal(AttachedStage& attachedStage, SceneFilter sceneFilter)
{
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    bool allScenesAreCompletedOrSkipped = true;
    bool selectedSceneFound = !sceneFilter.has_value();
    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (checkSkipScene(sceneFilter, sc))
            continue;

        selectedSceneFound = true;
        if (!sc->isComplete())
        {
            allScenesAreCompletedOrSkipped = false;
            break;
        }
    }
    return selectedSceneFound && allScenesAreCompletedOrSkipped;
}

bool physxCheckResults(AttachedStage& attachedStage)
{
    return physxCheckResultsInternal(attachedStage, std::nullopt);
}

bool physxCheckResultsScene(AttachedStage& attachedStage, uint64_t scenePath)
{
    return physxCheckResultsInternal(attachedStage, resolveSceneKey(scenePath));
}

// Updates a specific physX simulation scene or, if sceneFilter is invalid, all the scenes in the simulation.
// Note: if a specific physX simulation scene is specified, it will be updated *even if disabled* (disabled only applies
// to the omniphysx update loop, not if the user wants to step a specific scene singularly).
static void physXUpdateNonRenderInternal(AttachedStage& attachedStage, SceneFilter sceneFilter, float elapsedSecs, float currentTime, bool forceAsync)
{
    CARB_PROFILE_ZONE(0, "PhysXUpdateNonRender");

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    bool needStepping = false;
    float currentTimeShift = 0.0f;
    // Move waitForCompletion here, gUsdLoad->update we apply kinematics actors poses, simulation must be complete
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        if (checkSkipScene(sceneFilter, sc))
            continue;

        const bool asyncSimRender = forceAsync || sc->getUpdateType() == eAsynchronous;
        if (sc->getScene())
        {
            if (asyncSimRender)
            {
                CARB_PROFILE_ZONE(0, "Wait for async");
                sc->waitForCompletion();
            }
            else if (elapsedSecs > 0.0f)
            {
                float timestepsPerSecond = float(sc->getTimeStepsPerSeconds());
                const PxReal fixedTimeStep = 1.0f / timestepsPerSecond;

                sc->computeSubstepping(elapsedSecs, fixedTimeStep, timestepsPerSecond);
                const uint32_t currentNumSubSteps = sc->getCurrentStep();
                if (currentNumSubSteps > 1 && currentTimeShift == 0.0f)
                {
                    // We want to end on currentTime, therefore we have to start before the current time
                    currentTimeShift = -1.0f * (currentNumSubSteps - 1) * sc->getCurrentTimeStep();
                }
                if(currentNumSubSteps > 0)
                {
                    needStepping = true;
                }
            }
        }
    }

    {
        CARB_PROFILE_ZONE(0, "USDUpdate");
        processUpdates(attachedStage, currentTime + currentTimeShift);

        omniPhysX.getErrorEventStream()->pump();
    }

    {
        CARB_PROFILE_ZONE(0, "FinishSetup");
        if (PhysXUsdPhysicsInterface* iface = attachedStage.getPhysXPhysicsInterface())
            iface->finishSetup(attachedStage);
        attachedStage.getInternalPhysXDatabase().updateDirtyMassActors();
    }

    {
        CARB_PROFILE_ZONE(0, "ProcessDeformableAttachmentAndCollisionFilterShapeEvents");
        getPhysXUsdPhysicsInterface().processDeformableAttachmentShapeEvents(attachedStage);
        getPhysXUsdPhysicsInterface().processDeformableCollisionFilterShapeEvents(attachedStage);
    }

    {
        CARB_PROFILE_ZONE(0, "PhysicsUpdate::debugDraw");
        attachedStage.getInternalPhysXDatabase().debugDraw();
    }

    if (physxScenes.empty())
        return;

    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        if (checkSkipScene(sceneFilter, sc))
            continue;

        {
            CARB_PROFILE_ZONE(0, "UpdateDeformableAttachmentsAndCollisionFilters");

            std::vector<InternalDeformableAttachment*>& attachmentList = sc->getInternalScene()->mDeformableAttachments;
            for (size_t i = 0; i < attachmentList.size(); ++i)
            {
                attachmentList[i]->update();
            }

            std::vector<InternalDeformableCollisionFilter*>& collisionFilterList = sc->getInternalScene()->mDeformableCollisionFilters;
            for (size_t i = 0; i < collisionFilterList.size(); ++i)
            {
                collisionFilterList[i]->update();
            }
        }

        PxScene* scene = sc->getScene();
        if (scene)
        {
            // setup debug render scale
            if (attachedStage.isDebugVisualizationEnabled())
            {
                const float gizmoScale = attachedStage.getCachedSettings().viewportGizmoScale;
                scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, gizmoScale * attachedStage.getVisualizationScale());
            }

            if (elapsedSecs > 0.0f)
            {
                const bool asyncSimRender = forceAsync || (sc->getUpdateType() == eAsynchronous);
                if (asyncSimRender)
                {
                    sc->updateMirroredBodies();
                }
                else if (sc->isNewScene())
                {
                    // set stepping again, we have new scenes
                    float timestepsPerSecond = float(sc->getTimeStepsPerSeconds());
                    const PxReal fixedTimeStep = 1.0f / timestepsPerSecond;

                    sc->computeSubstepping(elapsedSecs, fixedTimeStep, timestepsPerSecond);
                    const uint32_t currentNumSubSteps = sc->getCurrentStep();
                    if (currentNumSubSteps > 1 && currentTimeShift == 0.0f)
                    {
                        // We want to end on currentTime, therefore we have to start before the current time
                        currentTimeShift = -1.0f * (currentNumSubSteps - 1) * sc->getCurrentTimeStep();
                    }
                    if(currentNumSubSteps > 0)
                    {
                        needStepping = true;
                    }
                }
            }
        }

        if (sceneFilter.has_value())
            break; // We already updated the scene simulation we were interested in
    }

    uint32_t updateStepIndex = 0;
    bool simulationHappened = false;
    if (!needStepping)
    {
        if (elapsedSecs > 0.0f)
        {
            CARB_PROFILE_ZONE(0, "UpdateRaycast");
            attachedStage.getRaycastManager().onUpdateRaycasts(elapsedSecs);
        }
        if (forceAsync) // for direct simulate call (forceAsync == true) we need to fire the pre-step event
        {
            CARB_PROFILE_ZONE(0, "pre-step update subscription update");
            omniPhysX.fireOnStepEventSubscriptions(attachedStage.getAttachHandle(), elapsedSecs, true);
        }
    }
    while (needStepping)
    {

        bool sendStepUpdate = false;

        float currentTimeStep = 0.0f;
        for (AttachedStage::SceneMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;

            if (!checkSkipScene(sceneFilter, sc) && sc->getCurrentStep())
            {
                currentTimeStep = sc->getCurrentTimeStep();
                sendStepUpdate = true;
                break;
            }
        }

        if (sendStepUpdate)
        {
            {
                CARB_PROFILE_ZONE(0, "pre-step update subscription update");
                omniPhysX.fireOnStepEventSubscriptions(attachedStage.getAttachHandle(), currentTimeStep, true);
            }
        }

        bool newStep = false;
        for (AttachedStage::SceneMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;

            if (checkSkipScene(sceneFilter, sc))
                continue;

            if (sc->getCurrentStep())
            {
                // Only run this once per step. If per scene stepping is later added, onUpdateRaycasts should be updated to function per scene.
                if(!newStep)
                {
                    CARB_PROFILE_ZONE(0, "UpdateRaycast");
                    attachedStage.getRaycastManager().onUpdateRaycasts(sc->getCurrentTimeStep());
                }

                sc->step();
                newStep = true;
                simulationHappened = true;
            }

            if (sceneFilter.has_value())
                break; // We already updated the scene simulation we were interested in
        }

        if (newStep)
        {
            updateStepIndex++;
            // wait for all to finish
            for (AttachedStage::SceneMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                if (checkSkipScene(sceneFilter, sc))
                    continue;

                if (sc->getCurrentStep())
                {
                    sc->waitForCompletion();
                }

                if (sceneFilter.has_value())
                    break; // We already updated the scene simulation we were interested in
            }

            float currentTimeStep = 0.0f;
            uint32_t currentStep = 0;
            for (AttachedStage::SceneMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                if (checkSkipScene(sceneFilter, sc))
                    continue;

                if (sc->getCurrentStep())
                {
                    sendStepUpdate = true;
                    currentStep = sc->getCurrentStep();
                    // A.B. TODO these notifications would have to be send per scene, same for USD updates
                    if (currentTimeStep > 0.0f && currentTimeStep != sc->getCurrentTimeStep())
                    {
                        CARB_LOG_ERROR_ONCE("Physics scenes stepping is not the same, step subscription will be send with later step, per scene step is not yet supported.");
                    }
                    currentTimeStep = sc->getCurrentTimeStep();
                    sc->updateMirroredBodies();
                    sc->decreaseCurrentStep();
                }

                if (sceneFilter.has_value())
                    break; // We already updated the scene simulation we were interested in
            }

            if (sendStepUpdate)
            {
                {
                    CARB_PROFILE_ZONE(0, "post-step update subscription update");
                    omniPhysX.fireOnStepEventSubscriptions(attachedStage.getAttachHandle(), currentTimeStep, false);
                }

                // we dont need to run the update for the last step, no simulation happens anymore
                if (currentStep != 1)
                {
                    CARB_PROFILE_ZONE(0, "USDUpdateStep");
                    processUpdates(attachedStage, currentTime + currentTimeShift + updateStepIndex * currentTimeStep);
                }
            }
        }
        else
        {
            needStepping = false;
        }
    }

    // Simulation complete synchronous call
    if (simulationHappened)
    {
        CARB_PROFILE_ZONE(0, "SimulationComplete");
        omniPhysX.fireStatusEventSubscriptions(attachedStage.getAttachHandle(), eSimulationComplete);
    }
}

void physXUpdateNonRender(AttachedStage& attachedStage, float elapsedSecs, float currentTime)
{
    physXUpdateNonRenderInternal(attachedStage, std::nullopt, elapsedSecs, currentTime, false);
}

void physXUpdateSceneNonRender(AttachedStage& attachedStage, uint64_t scenePath, float elapsedSecs, float currentTime)
{
    physXUpdateNonRenderInternal(attachedStage, resolveSceneKey(scenePath), elapsedSecs, currentTime, false);
}

void physXUpdateUsd(AttachedStage& attachedStage)
{
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (!sc->isReadbackSuppressed())
        {
            sc->getInternalScene()->updateSimulationOutputs(true, false, false, false);
        }
    }
}

static void physxSimulateSceneInternal(AttachedStage& attachedStage, SceneFilter sceneFilter, float elapsedSecs, float currentTime)
{
    // sync USD changes
    // update raycast etc
    attachedStage.profileUpdateStart = OmniPhysX::getInstance().isOmniPhysXProfilingEnabled() ?
        Time::getCurrentCounterValue() : 0;
    physXUpdateNonRenderInternal(attachedStage, sceneFilter, elapsedSecs, currentTime, true);

    // dispatch the async work, no stepping
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        if (checkSkipScene(sceneFilter, sc))
            continue;

        attachedStage.execution.pendingOwner = attachedStage.getAttachHandle();
        attachedStage.execution.pendingScenes.push_back(sc->getSceneSdfPath().handle);
        physXUpdateNonRenderDispatch(sc, elapsedSecs, currentTime, true, true);
    }
}

void physxSimulate(AttachedStage& attachedStage, float elapsedSecs, float currentTime)
{
    physxSimulateSceneInternal(attachedStage, std::nullopt, elapsedSecs, currentTime);
}

void physxSimulateScene(AttachedStage& attachedStage, uint64_t scenePath, float elapsedSecs, float currentTime)
{
    if (!hasSelectedScene(attachedStage, scenePath))
        return;
    physxSimulateSceneInternal(attachedStage, resolveSceneKey(scenePath), elapsedSecs, currentTime);
}

static void physxFetchResultsInternal(AttachedStage& attachedStage, SceneFilter sceneFilter)
{
    {
        CARB_PROFILE_ZONE(0, "fetchResults::waitForCompletion");
        for (AttachedStage::SceneMap::const_reference entry : attachedStage.getPhysXScenes())
        {
            if (!sceneFilter.has_value() || entry.second->getSceneSdfPath() == *sceneFilter)
                entry.second->waitForCompletion(true);
        }
    }

    if (attachedStage.profileUpdateStart != 0)
    {
        const auto elapsed = Time::getCurrentCounterValue() - attachedStage.profileUpdateStart;
        const float timeInMs = Time::getCounterFrequency().toTensOfNanos(elapsed) / (100.0f * 1000.0f);
        std::lock_guard<carb::tasking::MutexWrapper> lock(OmniPhysX::getInstance().profileMutex);
        OmniPhysX::getInstance().getOmniPhysXProfileStats().push_back({"PhysX Update", timeInMs});
    }

    const OmniCachedSettings& cachedSettings = attachedStage.getCachedSettings();
    const bool updateToUsd = cachedSettings.updateToUsd;
    const bool updateVelocitiesToUsd = cachedSettings.updateVelocitiesToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    {
        CARB_PROFILE_ZONE(0, "fetchResults::updateRenderTransforms");
        PHYSICS_PROFILE("Physics Update Transforms");
        const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();

        for (AttachedStage::SceneMap::const_reference ref : physxScenes)
        {
            const PhysXScene* sc = ref.second;

            if (checkSkipScene(sceneFilter, sc))
                continue;

#if USE_PHYSX_GPU
            PxScene* scene = sc->getScene();
            if ((scene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
                scene->getBroadPhaseType() == PxBroadPhaseType::eGPU)
            {
                PxCudaContextManager* manager = scene->getCudaContextManager();
                if (manager)
                {
                    const PxCUresult cudaError = manager->getCudaContext()->getLastError();
                    if (cudaError != 0)
                    {
                        // Worker-thread diagnostics do not reach the caller's RuntimeErrorScope.
                        OVX_RUNTIME_ERROR("GPU simulation cannot continue: CUDA error %d.", static_cast<int>(cudaError));
                        continue;
                    }
                }
            }
#endif

            if (!sc->isReadbackSuppressed())
            {
                sc->getInternalScene()->updateSimulationOutputs(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd);
            }
        }

        {
            CARB_PROFILE_ZONE(0, "fetchResults::transformationUpdateCallback");
            SimulationCallbacks* cb = &attachedStage.getSimulationCallbacks();
            if (cb && cb->getTransformUpdateFn())
            {
                RuntimeCallbackScope callbackScope;
                TransformUpdateFn updateFn = cb->getTransformUpdateFn();
                updateFn(0.0f, 0.0f, cb->getUserData());
            }
        }
    }

    {
        CARB_PROFILE_ZONE(0, "fetchResults::fireProfileStatsSubscription");
        OmniPhysX::getInstance().fireProfileStatsSubscription();
    }
}

void physxFetchResultsScene(AttachedStage& attachedStage, uint64_t scenePath)
{
    if (!hasSelectedScene(attachedStage, scenePath))
        return;
    physxFetchResultsInternal(attachedStage, resolveSceneKey(scenePath));
}

void physxFetchResults(AttachedStage& attachedStage)
{
    physxFetchResultsInternal(attachedStage, std::nullopt);
}

void physXUpdateNonRenderDispatch(PhysXScene* sc, float elapsedSecs, float currentTime, bool forceAsync, bool noStepping)
{
    const bool asyncSimRender = forceAsync || sc->getUpdateType() == eAsynchronous;

    if (asyncSimRender && elapsedSecs > 0.0f)
    {
        PxScene* scene = sc->getScene();

        PxU32 steps = 1;
        float timeStep = elapsedSecs;
        if (!noStepping)
        {
            float timestepsPerSecond = float(sc->getTimeStepsPerSeconds());
            const PxReal fixedTimeStep = 1.0f / timestepsPerSecond;
            timeStep = fixedTimeStep;

            sc->computeSubstepping(elapsedSecs, fixedTimeStep, timestepsPerSecond);
        }
        else
        {
            sc->setSubstepping(1, elapsedSecs);
        }

        if (scene)
        {
            CARB_PROFILE_ZONE(0, "simulate/fetch results");
            sc->getAttachedStage().setAsyncUSDUpdate(true);
            sc->launch(forceAsync);
        }
    }
}

void updateCooking()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    CookingDataAsync* cookingDataAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync)
    {
        cookingDataAsync->pump();
    }

    // We still need to process the event stream even if the simulation itself isn't active
    {
        CARB_PROFILE_ZONE(0, "ErrorEventStreamPump");
        omniPhysX.getErrorEventStream()->pump();
    }
}

void physXUpdate(AttachedStage& attachedStage, float currentTime, float elapsedSecs, bool enableUpdate)
{
    if (elapsedSecs <= 0.0f)
    {
        CARB_LOG_VERBOSE("PhysicsUpdate: provided elapsed time was less or equat to zero, skipping update.");
        return;
    }

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if(!omniPhysX.getPhysXSetup().hasBeenInitiallyCreated())
    {
        CARB_LOG_ERROR_ONCE("PhysicsUpdate: Update called before initialization or after shutdown");
        return;
    }

    CookingDataAsync* cookingDataAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();

    // Give up a time slice to any asynchronous mesh cooking tasks.
    if (cookingDataAsync)
    {
        uint32_t taskCount = cookingDataAsync->pump(attachedStage);
        if (taskCount)
        {
            enableUpdate = false;
        }
    }

    if (!enableUpdate)
    {
        // We still need to process the event stream even if the simulation itself isn't active
        {
            CARB_PROFILE_ZONE(0, "ErrorEventStreamPump");
            omniPhysX.getErrorEventStream()->pump();
        }

        return;
    }

    CARB_PROFILE_ZONE(0, "PhysXUpdate - update of PhysX");

    {
        PHYSICS_PROFILE("PhysX Update");
        physXUpdateNonRender(attachedStage, elapsedSecs, currentTime);
    }
    const AttachedStage::SceneMap& physxScenes = attachedStage.getPhysXScenes();
    const OmniCachedSettings& cachedSettings = attachedStage.getCachedSettings();
    const bool updateToUsd = cachedSettings.updateToUsd;
    const bool updateVelocitiesToUsd = cachedSettings.updateVelocitiesToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    {
        PHYSICS_PROFILE("Physics Update Transforms");
        for (AttachedStage::SceneMap::const_reference ref : physxScenes)
        {
            const PhysXScene* sc = ref.second;
            {
                {
                    CARB_PROFILE_ZONE(0, "updateRenderTransforms");
                    if (!sc->isReadbackSuppressed())
                    {
                        sc->getInternalScene()->updateSimulationOutputs(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd);
                    }
                }
            }
        }

        // update transformation callback
        {
            CARB_PROFILE_ZONE(0, "transformationUpdateCallback");
            SimulationCallbacks* cb = &attachedStage.getSimulationCallbacks();
            if (cb && cb->getTransformUpdateFn())
            {
                RuntimeCallbackScope callbackScope;
                TransformUpdateFn updateFn = cb->getTransformUpdateFn();
                updateFn(elapsedSecs, currentTime, cb->getUserData());
            }
        }
    }

    for (AttachedStage::SceneMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        {
            CARB_PROFILE_ZONE(0, "updateNonRenderDispatch");
            physXUpdateNonRenderDispatch(sc, elapsedSecs, currentTime);
        }
    }

    OmniPhysX::getInstance().fireProfileStatsSubscription();
}

void OmniPhysX::fireOnStepEventSubscriptions(AttachHandle attach, float timeStep, bool preStep)
{
    auto owner = UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!owner)
        return;

    RuntimeCallbackScope callbackScope(attach, preStep);
    StepEventMap snapshot;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
        snapshot = preStep ? owner->preStepSubscriptions : owner->postStepSubscriptions;
    }
    for (const StepEventMap::value_type& entry : snapshot)
    {
        if (entry.second.attach != attach)
            continue;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
            const StepEventMap& subscriptions = preStep ? owner->preStepSubscriptions : owner->postStepSubscriptions;
            if (subscriptions.find(entry.first) == subscriptions.end())
                continue;
        }
        try { entry.second.stepFn(timeStep, entry.second.userData); }
        catch (const std::exception& e) { CARB_LOG_ERROR("Physics step callback threw: %s", e.what()); }
        catch (...) { CARB_LOG_ERROR("Physics step callback threw an unknown exception"); }
    }
}

RuntimeResult physXUpdateNonRender(AttachHandle attach, float elapsedSecs, float currentTime)
{
    RuntimeExecutionScope operation(attach);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    physXUpdateNonRender(*owner, elapsedSecs, currentTime);
    return RuntimeResult::eSuccess;
}

RuntimeResult physXUpdateSceneNonRender(AttachHandle attach, uint64_t scenePath, float elapsedSecs, float currentTime)
{
    RuntimeExecutionScope operation(attach);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    if (!hasSelectedScene(*owner, scenePath))
        return RuntimeResult::eInvalidScene;
    physXUpdateSceneNonRender(*owner, scenePath, elapsedSecs, currentTime);
    return RuntimeResult::eSuccess;
}

namespace
{
carb::tasking::TaskStorageKey gCallbackStorage = carb::tasking::kInvalidTaskStorageKey;
thread_local RuntimeCallbackState* gNativeCallbackState = nullptr;

RuntimeCallbackState* callbackState()
{
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    if (tasking->getTaskContext() != carb::tasking::kInvalidTaskContext)
        return static_cast<RuntimeCallbackState*>(tasking->getTaskStorage(gCallbackStorage));
    return gNativeCallbackState;
}

void setCallbackState(RuntimeCallbackState* state)
{
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    if (tasking->getTaskContext() != carb::tasking::kInvalidTaskContext)
    {
        if (!tasking->setTaskStorage(gCallbackStorage, state))
            std::terminate(); // Continuing without callback protection is unsafe.
    }
    else
        gNativeCallbackState = state;
}
}

void initializeRuntimeCallbacks()
{
    CARB_ASSERT(gCallbackStorage == carb::tasking::kInvalidTaskStorageKey);
    gCallbackStorage = carb::getCachedInterface<carb::tasking::ITasking>()->allocTaskStorage(nullptr);
    if (gCallbackStorage == carb::tasking::kInvalidTaskStorageKey)
        throw std::runtime_error("Unable to allocate runtime callback task storage");
}

void shutdownRuntimeCallbacks()
{
    // Runtime shutdown drains all owners and their callbacks before releasing this slot.
    carb::getCachedInterface<carb::tasking::ITasking>()->freeTaskStorage(gCallbackStorage);
    gCallbackStorage = carb::tasking::kInvalidTaskStorageKey;
}

RuntimeCallbackScope::RuntimeCallbackScope(AttachHandle attach, bool preStep)
    : RuntimeCallbackScope(attach, preStep, nullptr)
{
}

RuntimeCallbackScope::RuntimeCallbackScope(AttachHandle attach, bool preStep, const usdparser::AttachedStage* customOwner) : mPrevious(callbackState())
{
    mState.depth = mPrevious ? mPrevious->depth + 1 : 1;
    mState.preStepOwner = preStep ? attach : omni::physics::kNoAttach;
    mState.customOwner = customOwner;
    setCallbackState(&mState);
}

RuntimeCallbackScope::~RuntimeCallbackScope()
{
    setCallbackState(mPrevious);
}

AttachHandle preStepCallbackOwner()
{
    const RuntimeCallbackState* state = callbackState();
    return state ? state->preStepOwner : omni::physics::kNoAttach;
}

uint32_t getRuntimeCallbackDepth()
{
    const RuntimeCallbackState* state = callbackState();
    return state ? state->depth : 0;
}

const usdparser::AttachedStage* customCreationOwner()
{
    const RuntimeCallbackState* state = callbackState();
    return state ? state->customOwner : nullptr;
}

RuntimeExecutionState& getRuntimeExecutionState()
{
    static RuntimeExecutionState state;
    return state;
}

std::atomic<uint32_t>& runtimeOperationCount()
{
    static std::atomic<uint32_t> count{0};
    return count;
}

std::shared_mutex& runtimeAuthoringMutex()
{
    static std::shared_mutex mutex;
    return mutex;
}

RuntimeAuthoringScope::RuntimeAuthoringScope() : mLock(runtimeAuthoringMutex(), std::defer_lock)
{
    if (!getRuntimeCallbackDepth() && mLock.try_lock())
        runtimeOperationCount().fetch_add(1, std::memory_order_relaxed);
}

RuntimeAuthoringScope::~RuntimeAuthoringScope()
{
    if (mLock.owns_lock())
        runtimeOperationCount().fetch_sub(1, std::memory_order_relaxed);
}

RuntimeExecutionScope::RuntimeExecutionScope(AttachHandle attach, bool allowPendingOwner, bool preStepForce)
{
    if (getRuntimeCallbackDepth() != 0)
    {
        // The outer pre-step dispatch retains the owner and serializes its scene.
        // Only force application opts into this narrowly defined internal call.
        mAllowed = preStepForce && attach != omni::physics::kNoAttach && preStepCallbackOwner() == attach;
        return;
    }
    if (attach == omni::physics::kNoAttach)
        mState = &getRuntimeExecutionState();
    else
    {
        mOwner = UsdLoad::getUsdLoad()->acquireAttachment(attach);
        mState = mOwner ? &mOwner->execution : &mInvalidOwnerState;
    }
    mOwnsEntry = !mState->entered.test_and_set(std::memory_order_acquire);
    if (mOwnsEntry)
    {
        runtimeOperationCount().fetch_add(1, std::memory_order_relaxed);
        mAllowed = mState->pendingOwner == omni::physics::kNoAttach || allowPendingOwner;
        // A lease obtained just before retirement must not resurrect the owner.
        if (mOwner && UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach) != mOwner.get())
            mAllowed = false;
    }
}

RuntimeExecutionScope::~RuntimeExecutionScope()
{
    if (mOwnsEntry)
    {
        mState->entered.clear(std::memory_order_release);
        runtimeOperationCount().fetch_sub(1, std::memory_order_relaxed);
    }
}

void retireRuntimeExecution(AttachHandle attach)
{
    if (auto owner = UsdLoad::getUsdLoad()->acquireAttachment(attach))
    {
        owner->execution.pendingScenes.clear();
        owner->execution.pendingOwner = omni::physics::kNoAttach;
    }
}

RuntimeResult physxSimulate(AttachHandle attach, float elapsedSecs, float currentTime)
{
    RuntimeExecutionScope operation(attach);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!attachedStage)
        return RuntimeResult::eInvalidAttachment;
    operation.begin(attach);
    physxSimulate(*attachedStage, elapsedSecs, currentTime);
    return RuntimeResult::eSuccess;
}

RuntimeResult physxSimulateScene(AttachHandle attach, uint64_t scenePath, float elapsedSecs, float currentTime)
{
    if (scenePath == 0)
        return physxSimulate(attach, elapsedSecs, currentTime);
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!attachedStage)
        return RuntimeResult::eInvalidAttachment;
    if (!hasSelectedScene(*attachedStage, scenePath))
        return RuntimeResult::eInvalidScene;
    if (operation.hasPendingScene(scenePath))
        return RuntimeResult::eBusy;
    operation.begin(attach);
    physxSimulateScene(*attachedStage, scenePath, elapsedSecs, currentTime);
    return RuntimeResult::eSuccess;
}

RuntimeResult physxFetchResults(AttachHandle attach)
{
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!attachedStage)
        return RuntimeResult::eInvalidAttachment;
    physxFetchResults(*attachedStage);
    operation.complete();
    return RuntimeResult::eSuccess;
}

RuntimeResult physxFetchResultsScene(AttachHandle attach, uint64_t scenePath)
{
    if (scenePath == 0)
        return physxFetchResults(attach);
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!attachedStage)
        return RuntimeResult::eInvalidAttachment;
    if (!hasSelectedScene(*attachedStage, scenePath))
        return RuntimeResult::eInvalidScene;
    physxFetchResultsScene(*attachedStage, scenePath);
    operation.complete(scenePath);
    return RuntimeResult::eSuccess;
}

bool physxCheckResults(AttachHandle attach)
{
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return false;
    if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach))
        return physxCheckResults(*attachedStage);
    return false;
}

bool physxCheckResultsScene(AttachHandle attach, uint64_t scenePath)
{
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return false;
    if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach))
        return physxCheckResultsScene(*attachedStage, scenePath);
    return false;
}

RuntimeResult physXUpdateUsd(AttachHandle attach)
{
    RuntimeExecutionScope operation(attach);
    if (!operation)
        return RuntimeResult::eBusy;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!attachedStage)
        return RuntimeResult::eInvalidAttachment;
    physXUpdateUsd(*attachedStage);
    return RuntimeResult::eSuccess;
}



} // namespace physx
} // namespace omni
