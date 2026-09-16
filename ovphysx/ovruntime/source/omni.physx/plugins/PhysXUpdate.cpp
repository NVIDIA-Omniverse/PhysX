// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXUpdate.h"
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
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>

#include "utils/Profile.h"

#include <optional>


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

void waitForSimulationCompletion(bool doPostWork)
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        sc->waitForCompletion(doPostWork);
    }
}

static bool physxCheckResultsInternal(SceneFilter sceneFilter)
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    bool allScenesAreCompletedOrSkipped = true;
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (checkSkipScene(sceneFilter, sc))
            continue;

        if (!sc->isComplete())
        {
            allScenesAreCompletedOrSkipped = false;
            break;
        }
    }
    return allScenesAreCompletedOrSkipped;
}

bool physxCheckResults()
{
    return physxCheckResultsInternal(std::nullopt);
}

bool physxCheckResultsScene(uint64_t scenePath)
{
    return physxCheckResultsInternal(resolveSceneKey(scenePath));
}

// Updates a specific physX simulation scene or, if sceneFilter is invalid, all the scenes in the simulation.
// Note: if a specific physX simulation scene is specified, it will be updated *even if disabled* (disabled only applies
// to the omniphysx update loop, not if the user wants to step a specific scene singularly).
static void physXUpdateNonRenderInternal(SceneFilter sceneFilter, float elapsedSecs, float currentTime, bool forceAsync)
{
    CARB_PROFILE_ZONE(0, "PhysXUpdateNonRender");

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    bool needStepping = false;
    float currentTimeShift = 0.0f;
    // Move waitForCompletion here, gUsdLoad->update we apply kinematics actors poses, simulation must be complete
    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
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
        UsdLoad::getUsdLoad()->update(currentTime + currentTimeShift);

        omniPhysX.getErrorEventStream()->pump();
    }

    {
        CARB_PROFILE_ZONE(0, "FinishSetup");
        const AttachedStage* stage = UsdLoad::getUsdLoad()->getAttachedStage(0);
        if (stage)
        {
            getPhysXUsdPhysicsInterface().finishSetup(*stage);
        }
        OmniPhysX::getInstance().getInternalPhysXDatabase().updateDirtyMassActors();
    }

    {
        CARB_PROFILE_ZONE(0, "ProcessDeformableAttachmentAndCollisionFilterShapeEvents");
        getPhysXUsdPhysicsInterface().processDeformableAttachmentShapeEvents();
        getPhysXUsdPhysicsInterface().processDeformableCollisionFilterShapeEvents();
    }

    {
        CARB_PROFILE_ZONE(0, "PhysicsUpdate::debugDraw");
        omniPhysX.getInternalPhysXDatabase().debugDraw();
    }

    if (physxScenes.empty())
        return;

    for (PhysXScenesMap::const_reference ref : physxScenes)
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
            if (OmniPhysX::getInstance().isDebugVisualizationDirty())
            {
                const float gizmoScale = omniPhysX.getCachedSettings().viewportGizmoScale;
                scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, gizmoScale * OmniPhysX::getInstance().getVisualizationScale());
                OmniPhysX::getInstance().setDebugVisualizationDirty(false);
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
            omniPhysX.getRaycastManager().onUpdateRaycasts(elapsedSecs);
        }
        if (forceAsync) // for direct simulate call (forceAsync == true) we need to fire the pre-step event
        {
            CARB_PROFILE_ZONE(0, "pre-step update subscription update");
            omniPhysX.fireOnStepEventSubscriptions(elapsedSecs, true);
        }
    }
    while (needStepping)
    {

        bool sendStepUpdate = false;

        float currentTimeStep = 0.0f;
        for (PhysXScenesMap::const_reference ref : physxScenes)
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
                omniPhysX.fireOnStepEventSubscriptions(currentTimeStep, true);
            }
        }

        bool newStep = false;
        for (PhysXScenesMap::const_reference ref : physxScenes)
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
                    omniPhysX.getRaycastManager().onUpdateRaycasts(sc->getCurrentTimeStep());
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
            for (PhysXScenesMap::const_reference ref : physxScenes)
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
            for (PhysXScenesMap::const_reference ref : physxScenes)
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
                    omniPhysX.fireOnStepEventSubscriptions(currentTimeStep, false);
                }

                // we dont need to run the update for the last step, no simulation happens anymore
                if (currentStep != 1)
                {
                    CARB_PROFILE_ZONE(0, "USDUpdateStep");
                    UsdLoad::getUsdLoad()->update(currentTime + currentTimeShift + updateStepIndex * currentTimeStep);
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
        omniPhysX.fireStatusEventSubscriptions(eSimulationComplete);
    }
}

void physXUpdateNonRender(float elapsedSecs, float currentTime)
{
    physXUpdateNonRenderInternal(std::nullopt, elapsedSecs, currentTime, false);
}

void physXUpdateSceneNonRender(uint64_t scenePath, float elapsedSecs, float currentTime)
{
    physXUpdateNonRenderInternal(resolveSceneKey(scenePath), elapsedSecs, currentTime, false);
}

void physXUpdateUsd()
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (!sc->isReadbackSuppressed())
        {
            sc->getInternalScene()->updateSimulationOutputs(true, false, false, false);
        }
    }
}

static void physxSimulateSceneInternal(SceneFilter sceneFilter, float elapsedSecs, float currentTime)
{
    // sync USD changes
    // update raycast etc
    PHYSICS_CROSS_THREAD_PROFILE_START("PhysX Update");
    physXUpdateNonRenderInternal(sceneFilter, elapsedSecs, currentTime, true);

    // dispatch the async work, no stepping
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        if (checkSkipScene(sceneFilter, sc))
            continue;

        physXUpdateNonRenderDispatch(sc, elapsedSecs, currentTime, true, true);
    }
}

void physxSimulate(float elapsedSecs, float currentTime)
{
    physxSimulateSceneInternal(std::nullopt, elapsedSecs, currentTime);
}

void physxSimulateScene(uint64_t scenePath, float elapsedSecs, float currentTime)
{
    physxSimulateSceneInternal(resolveSceneKey(scenePath), elapsedSecs, currentTime);
}

static void physxFetchResultsInternal(SceneFilter sceneFilter)
{
    {
        CARB_PROFILE_ZONE(0, "fetchResults::waitForCompletion");
        waitForSimulationCompletion(true);
    }

    PHYSICS_CROSS_THREAD_PROFILE_END("PhysX Update");

    const OmniCachedSettings& cachedSettings = OmniPhysX::getInstance().getCachedSettings();
    const bool updateToUsd = cachedSettings.updateToUsd;
    const bool updateVelocitiesToUsd = cachedSettings.updateVelocitiesToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    {
        CARB_PROFILE_ZONE(0, "fetchResults::updateRenderTransforms");
        PHYSICS_PROFILE("Physics Update Transforms");
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            const PhysXScene* sc = ref.second;

            if (checkSkipScene(sceneFilter, sc))
                continue;

            if (!sc->isReadbackSuppressed())
            {
                sc->getInternalScene()->updateSimulationOutputs(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd);
            }
        }

        {
            CARB_PROFILE_ZONE(0, "fetchResults::transformationUpdateCallback");
            SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
            if (cb && cb->getTransformUpdateFn())
            {
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

void physxFetchResultsScene(uint64_t scenePath)
{
    physxFetchResultsInternal(resolveSceneKey(scenePath));
}

void physxFetchResults()
{
    physxFetchResultsInternal(std::nullopt);
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
            UsdLoad::getUsdLoad()->setAsyncUSDUpdate(true);
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

void physXUpdate(float currentTime, float elapsedSecs, bool enableUpdate)
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
        uint32_t taskCount = cookingDataAsync->pump();
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
        physXUpdateNonRender(elapsedSecs, currentTime);
    }
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    OmniCachedSettings& cachedSettings = OmniPhysX::getInstance().getCachedSettings();
    const bool updateToUsd = cachedSettings.updateToUsd;
    const bool updateVelocitiesToUsd = cachedSettings.updateVelocitiesToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    {
        PHYSICS_PROFILE("Physics Update Transforms");
        for (PhysXScenesMap::const_reference ref : physxScenes)
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
            SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
            if (cb && cb->getTransformUpdateFn())
            {
                TransformUpdateFn updateFn = cb->getTransformUpdateFn();
                updateFn(elapsedSecs, currentTime, cb->getUserData());
            }
        }
    }

    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        {
            CARB_PROFILE_ZONE(0, "updateNonRenderDispatch");
            physXUpdateNonRenderDispatch(sc, elapsedSecs, currentTime);
        }
    }

    OmniPhysX::getInstance().fireProfileStatsSubscription();
}

} // namespace physx
} // namespace omni
