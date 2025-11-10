// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXUpdate.h"
#include "OmniPhysX.h"

#include "internal/InternalScene.h"
#include "internal/InternalParticle.h"
#include "internal/InternalDeformableDeprecated.h"
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
#include "particles/FabricParticles.h"
#include "ScopedNoticeLock.h"

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysx.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>

#include "utils/Profile.h"


using namespace ::physx;
using namespace carb;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;
using namespace cookingdataasync;
using namespace pxr;


namespace omni
{
namespace physx
{

// Returns true if a scene must be skipped from a simulation update. If the default (empty)
// SdfPath is passed, a simulation scene is only skipped if marked 'Disabled', otherwise
// if a specific scene was requested (i.e. SdfPath is not empty), we will only process that specific one.
static inline bool checkSkipScene(const pxr::SdfPath& scenePath, const PhysXScene* sc)
{
    if (scenePath.IsEmpty())
    {
        // update all scenes in the simulation if not specifically disabled
        if (sc->getUpdateType() == Disabled)
            return true;
    }
    else
    {
        // update only the scene that was specifically requested
        if (sc->getSceneSdfPath() != scenePath)
            return true;
    }
    return false;
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

static bool physxCheckResultsInternal(const pxr::SdfPath& scenePath)
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    bool allScenesAreCompletedOrSkipped = true;
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (checkSkipScene(scenePath, sc))
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
    return physxCheckResultsInternal(pxr::SdfPath());
}

bool physxCheckResultsScene(uint64_t scenePath)
{
    return physxCheckResultsInternal(intToSdfPath(scenePath));
}

// Updates a specific physX simulation scene or, if scenePath is empty, all the scenes in the simulation.
// Note: if a specific physX simulation scene is specified, it will be updated *even if disabled* (disabled only applies
// to the omniphysx update loop, not if the user wants to step a specific scene singularly).
static void physXUpdateNonRenderInternal(const pxr::SdfPath& scenePath, float elapsedSecs, float currentTime, bool forceAsync)
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

        if (checkSkipScene(scenePath, sc))
            continue;

        const bool asyncSimRender = forceAsync || sc->getUpdateType() == Asynchronous;
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
        CARB_PROFILE_ZONE(0, "DeformableAttachmentUpdateDeprecated");
        getPhysXUsdPhysicsInterface().finalizeDeformableRigidAttachmentsDeprecated();
        getPhysXUsdPhysicsInterface().updateDeformableAttachmentsDeprecated();
    }
    
    {
        CARB_PROFILE_ZONE(0, "ProcessDeformableAttachmentAndCollisionFilterShapeEvents");
        getPhysXUsdPhysicsInterface().processDeformableAttachmentShapeEvents();
        getPhysXUsdPhysicsInterface().processDeformableCollisionFilterShapeEvents();
    }

    if (physxScenes.empty())
        return;

    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        if (checkSkipScene(scenePath, sc))
            continue;

        {
            CARB_PROFILE_ZONE(0, "DeformableAttachmentRefreshDeprecated");
            std::vector<InternalAttachmentDeprecated*>& attachements = sc->getInternalScene()->mAttachmentsDeprecated;
            for (size_t i = 0; i < attachements.size(); ++i)
            {
                attachements[i]->refreshAttachment();
            }
        }
        
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
            if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
            {
                const float gizmoScale = omniPhysX.getCachedSettings().viewportGizmoScale;
                scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, gizmoScale * OmniPhysX::getInstance().getVisualizationScale());
            }

            if (elapsedSecs > 0.0f)
            {
                const bool asyncSimRender = forceAsync || (sc->getUpdateType() == Asynchronous);
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

        if (!scenePath.IsEmpty())
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

            if (!checkSkipScene(scenePath, sc) && sc->getCurrentStep())
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

            if (checkSkipScene(scenePath, sc))
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

            if (!scenePath.IsEmpty())
                break; // We already updated the scene simulation we were interested in
        }

        if (newStep)
        {
            updateStepIndex++;
            // wait for all to finish
            for (PhysXScenesMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                if (checkSkipScene(scenePath, sc))
                    continue;

                if (sc->getCurrentStep())
                {
                    sc->waitForCompletion();
                }

                if (!scenePath.IsEmpty())
                    break; // We already updated the scene simulation we were interested in
            }

            float currentTimeStep = 0.0f;
            uint32_t currentStep = 0;
            for (PhysXScenesMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                if (checkSkipScene(scenePath, sc))
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

                if (!scenePath.IsEmpty())
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
    physXUpdateNonRenderInternal(pxr::SdfPath(), elapsedSecs, currentTime, false);
}

void physXUpdateSceneNonRender(uint64_t scenePath, float elapsedSecs, float currentTime)
{
    physXUpdateNonRenderInternal(intToSdfPath(scenePath), elapsedSecs, currentTime, false);
}

void physXUpdateUsd()
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;
        if (!sc->isReadbackSuppressed())
        {
            sc->getInternalScene()->updateSimulationOutputs(true, false, false, false, false);
        }
    }
}

static void physxSimulateSceneInternal(const pxr::SdfPath& scenePath, float elapsedSecs, float currentTime)
{
    // sync USD changes
    // update raycast etc
    PHYSICS_CROSS_THREAD_PROFILE_START("PhysX Update");
    physXUpdateNonRenderInternal(scenePath, elapsedSecs, currentTime, true);

    // dispatch the async work, no stepping
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        if (checkSkipScene(scenePath, sc))
            continue;

        physXUpdateNonRenderDispatch(sc, elapsedSecs, currentTime, true, true);
    }
}

void physxSimulate(float elapsedSecs, float currentTime)
{
    physxSimulateSceneInternal(pxr::SdfPath(), elapsedSecs, currentTime);
}

void physxSimulateScene(uint64_t scenePath, float elapsedSecs, float currentTime)
{
    physxSimulateSceneInternal(intToSdfPath(scenePath), elapsedSecs, currentTime);
}

static void physxFetchResultsInternal(const pxr::SdfPath& scenePath)
{
    {
        CARB_PROFILE_ZONE(0, "fetchResults::waitForCompletion");
        waitForSimulationCompletion(true);
    }

    PHYSICS_CROSS_THREAD_PROFILE_END("PhysX Update");

    const OmniCachedSettings& cachedSettings = OmniPhysX::getInstance().getCachedSettings();
    const bool updateToUsd = cachedSettings.updateToUsd;
    const bool updateVelocitiesToUsd = cachedSettings.updateVelocitiesToUsd;
    const bool updateResidualsToUsd = cachedSettings.updateResidualsToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    {
        CARB_PROFILE_ZONE(0, "fetchResults::updateRenderTransforms");
        PHYSICS_PROFILE("Physics Update Transforms");
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            const PhysXScene* sc = ref.second;

            if (checkSkipScene(scenePath, sc))
                continue;

            if (!sc->isReadbackSuppressed())
            {
                sc->getInternalScene()->updateSimulationOutputs(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd, updateResidualsToUsd);
            }
        }
    }

    {
        OmniPhysX::getInstance().getInternalPhysXDatabase().updateSimulationOutputs(updateResidualsToUsd);
    }


#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    FabricParticles* fabricParticles = OmniPhysX::getInstance().getFabricParticles();
    if(fabricParticles) // can be nullptr in memory stages
        fabricParticles->update();
#endif

    OmniPhysX::getInstance().fireProfileStatsSubscription();
}

void physxFetchResultsScene(uint64_t scenePath)
{
    physxFetchResultsInternal(intToSdfPath(scenePath));
}

void physxFetchResults()
{
    physxFetchResultsInternal(pxr::SdfPath());
}

void physXUpdateNonRenderDispatch(PhysXScene* sc, float elapsedSecs, float currentTime, bool forceAsync, bool noStepping)
{
    const bool asyncSimRender = forceAsync || sc->getUpdateType() == Asynchronous;

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
    const bool updateResidualsToUsd = cachedSettings.updateResidualsToUsd;
    const bool outputVelocitiesLocalSpace = cachedSettings.outputVelocitiesLocalSpace;
    const bool updateParticlesToUsd = cachedSettings.updateParticlesToUsd;

    // This flag is set in OmniPhysX::subscribeToSettingsChangeEvents but we re-init local mesh cache
    // here to wait for the cooking async pump to be finished completely
    if(cachedSettings.delayedInitLocalMeshCache)
    {
        cookingdataasync::CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if(cookingDataAsync)
        {
            cookingDataAsync->setLocalMeshCacheEnabled(cachedSettings.localMeshCacheEnabled);
            cookingDataAsync->setLocalMeshCacheSize(cachedSettings.localMeshCacheSize);
        }
        cachedSettings.delayedInitLocalMeshCache = false;
    }

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
                        sc->getInternalScene()->updateSimulationOutputs(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd, updateResidualsToUsd);
                    }
                }
            }
        }
        {
            CARB_PROFILE_ZONE(0, "updateJointResiduals");
            OmniPhysX::getInstance().getInternalPhysXDatabase().updateSimulationOutputs(updateResidualsToUsd);
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

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    omniPhysX.getFabricParticles()->update();
#endif

    OmniPhysX::getInstance().fireProfileStatsSubscription();
}

} // namespace physx
} // namespace omni
