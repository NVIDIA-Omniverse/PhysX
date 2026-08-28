// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"

#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXSimulationCallbacks.h"
#include "Trigger.h"
#include "PhysXCustomJoint.h"
#include "PhysXCustomGeometry.h"
#include "PhysXPropertyQuery.h"
#include "PhysXSettings.h"
#include "PhysXUpdate.h"
#include "PhysXDebugVisualization.h"
#include "CookingDataAsync.h"
#include "CookingDataAsync.h"
#include "Raycast.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Scene.h"
#include "PhysXFoundation.h"

#include <omni/physics/usd/CustomTokens.h>

#include "particles/PhysXParticlePost.h"

#include <common/utilities/MemoryMacros.h>

using namespace PXR_NS;
using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

static OmniPhysX* gOmniPhysXInstance;
bool OmniPhysX::mWasStarted = false;

OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)
OMNI_LOG_ADD_CHANNEL(kRoboticsLogChannel, "omni.physx.logging.robotics", "Physics Robotics")
OMNI_LOG_DECLARE_CHANNEL(kSceneMultiGPULogChannel)
OMNI_LOG_ADD_CHANNEL(kSceneMultiGPULogChannel, "omni.physx.logging.scenemultigpy", "Physics MultiGPU")

static constexpr char kViewportGizmoScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/scale";

///////////////////////////////////////////////////////////////////////////////

void OmniPhysX::sendSimulationEvent(SimulationEvent type)
{
    using namespace carb::events;
    {
        carb::events::IEventPtr event = carb::stealObject(
            mSimulationEventStreamV2->createEventPtr(static_cast<EventType>(type), kGlobalSenderId));
        mSimulationEventStreamV2->dispatch(event.get());
    }
}

void OmniPhysX::releasePhysXScenes()
{
    //update postprocess before any particle sets are released
    omni::physx::particles::notifyPhysXRelease();
    releaseInternalPhysXDatabase();
    mPhysXSetup.releasePhysXScenes();
    createInternalPhysXDatabase();
    mCustomJointManager->clear();
    mCustomGeometryManager->clear();
    for (ReplicatorMap::reference ref : mReplicatorMap)
    {
        ref.second.clear();
    }

    mProfileStats.clear();
    mCrossProfileStats.clear();
}

void OmniPhysX::releaseInternalPhysXDatabase()
{
    SAFE_DELETE_SINGLE(mInternalPhysXDatabase);
}

void OmniPhysX::createInternalPhysXDatabase()
{
    if (!mInternalPhysXDatabase)
        mInternalPhysXDatabase = ICE_NEW(InternalPhysXDatabase);
}

void OmniPhysX::physXAttachSession(PXR_NS::UsdStageWeakPtr stage)
{
    createInternalPhysXDatabase();

    SimulationCallbacks::getSimulationCallbacks()->reset();

    // Ensure the PxPhysics singleton. Pass the stage explicitly: the AttachedStage
    // is not registered with UsdLoad until the attach() call below, so
    // getActiveStage() would be empty here on first physics creation. For a
    // stageless (ovstage) attach the handle is empty and PhysX uses default
    // tolerances (RESIDUAL: route tolerances through the source units — part of the
    // remaining null-stage decoupling sweep).
    // A.B. TODO tolerances are per-stage and should move into individual scenes.
    if (stage)
        getPhysXSetup().getPhysics(stage);
    else
        getPhysXSetup().getPhysics();

    mCurrentTimestampOffset = mSimulationTimestamp;
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));
    const char* forceSingleScene = mISettings->getStringBuffer(kSettingForceParseOnlySingleScene);
    getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(
        forceSingleScene ? PXR_NS::SdfPath(forceSingleScene) : PXR_NS::SdfPath());
}

void OmniPhysX::physXAttach(long int stageId, bool loadPhysics)
{
    UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    physXAttachSession(stage);

    PhysXReplicator* replicator = getReplicator(stageId);
    if (replicator && loadPhysics)
    {
        replicator->attach(stageId, &getPhysXUsdPhysicsInterface(), true);
    }
    else
    {
        UsdLoad::getUsdLoad()->attach(loadPhysics, stageId, &getPhysXUsdPhysicsInterface());
    }
}

bool OmniPhysX::physXAttachOvstage(const void* ovstageAttachPayload,
                                  uint64_t readOrdinal,
                                  PXR_NS::UsdStageWeakPtr backingStage,
                                  uint64_t effectiveBackingStageId)
{
    // Same session prologue as the USD attach, with no stage; the only divergence
    // is the stageless attachOvstage() entry (which installs the ovstage backends).
    physXAttachSession(PXR_NS::UsdStageWeakPtr{});

    // Use the same preclassified local id for replicator selection and stage
    // registration. Re-querying the payload here would happen after the session
    // prologue and could disagree with the StageCache residency decision.
    PhysXReplicator* replicator =
        effectiveBackingStageId ? getReplicator(effectiveBackingStageId) : nullptr;

    const bool attached = UsdLoad::getUsdLoad()->attachOvstage(ovstageAttachPayload,
                                                               readOrdinal,
                                                               backingStage,
                                                               effectiveBackingStageId,
                                                               &getPhysXUsdPhysicsInterface(),
                                                               /*loadPhysics=*/replicator == nullptr);
    if (!attached)
        return false;

    if (replicator)
        replicator->attach(effectiveBackingStageId, &getPhysXUsdPhysicsInterface(), /*attachStage=*/false);

    return true;
}

bool OmniPhysX::physXUpdateFromOvStage(uint64_t fromOrdinal, uint64_t toOrdinal)
{
    // Route to the single active AttachedStage (ovstage attaches stageless, so it
    // resolves through getActiveAttachedStage()). Its change feed pulls the delta
    // for [fromOrdinal, toOrdinal] and drives the incremental-update callbacks.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    return attachedStage ? attachedStage->updateFromOvStage(fromOrdinal, toOrdinal) : false;
}

void OmniPhysX::physXDetach()
{
    // Drain queued property-query requests before tearing down the stage. Pending
    // requests hold UsdStageWeakPtrs whose targets get destroyed here; firing their
    // queryFinishedCallback now (with eERROR_RUNTIME) prevents a later
    // updateQueuedRequests tick from dereferencing an expired/null weak ptr.
    // This is the common detach path - reached from both physxSimulationDetach() and
    // onPhysXDetach(), so it covers direct API and Kit stage-update teardowns.
    getPropertyQueryManager().cancelAllPendingRequests(PhysxPropertyQueryResult::eERROR_RUNTIME);

    {
        std::unique_lock<carb::tasking::MutexWrapper> simStartedLock(mSimParamMutex);
        if (mHasSimulationStarted)
        {
            mHasSimulationStarted = false;
            sendSimulationEvent(SimulationEvent::eStopped);
        }
    }

    UsdLoad::getUsdLoad()->detach(UsdLoad::getUsdLoad()->getActiveStageId());

    releasePhysXScenes();

    SimulationCallbacks::getSimulationCallbacks()->reset();
}

void OmniPhysX::resetSimulation()
{
    OmniPhysX& omniPhysX = *this;

    const bool outputVelocitiesLocalSpace = omniPhysX.getISettings()->getAsBool(kSettingOutputVelocitiesLocalSpace);
    const bool useUsdUpdate = omniPhysX.getISettings()->getAsBool(kSettingUpdateToUsd);
    const bool useUsdVelocitiesUpdate = omniPhysX.getISettings()->getAsBool(kSettingUpdateVelocitiesToUsd);

    waitForSimulationCompletion(false);

    UsdLoad::getUsdLoad()->blockUSDUpdate(true);

    cookingdataasync::CookingDataAsync* cookingDataAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync)
    {
        cookingDataAsync->blockUSDUpdate(true);
    }

    omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    if (!omniPhysX.getCachedSettings().disableResetOnStop && omniPhysX.getISettings()->getAsBool(kSettingResetOnStop))
        db.resetStartProperties(useUsdUpdate, useUsdVelocitiesUpdate, outputVelocitiesLocalSpace);

    if (cookingDataAsync)
    {
        cookingDataAsync->blockUSDUpdate(false); // needs to be called before releasePhysicsObjects tears the cooking down
    }

    getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(false);  // do not send these notifications when the simulation is to end
    UsdLoad::getUsdLoad()->releasePhysicsObjects(UsdLoad::getUsdLoad()->getActiveStageId());
    UsdLoad::getUsdLoad()->blockUSDUpdate(false);

    omniPhysX.sendSimulationEvent(SimulationEvent::eStopped);
    omniPhysX.setSimulationStarted(false);
    omniPhysX.setSimulationRunning(false);
}

static void readPersistentSettings()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::settings::ISettings* iSettings = omniPhysX.getISettings();
    omniPhysX.setGpuPipelineOverride(iSettings->getAsInt(kSettingOverrideGPU));
    omniPhysX.getPhysXSetup().setThreadCount(iSettings->getAsInt(kSettingNumThreads));
    omniPhysX.getPhysXSetup().setMaxNumberOfPhysXErrors(iSettings->getAsInt(kSettingMaxNumberOfPhysXErrors));

}

void enableLogChannel(const omni::log::LogChannelData& channel, bool enable)
{
    auto log = omniGetLogWithoutAcquire();
    if (enable)
    {
        log->setChannelEnabled(channel, true, omni::log::SettingBehavior::eInherit);
    }
    else
    {
        log->setChannelEnabled(channel, false, omni::log::SettingBehavior::eOverride);
    }
}

void OmniPhysX::onStartup()
{
    gOmniPhysXInstance = this;
    mWasStarted = false;

    // Add physics systems
    carb::Framework* framework = carb::getFramework();

    mIDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();

    mITasking = carb::getCachedInterface<carb::tasking::ITasking>();

    mISettings = carb::getCachedInterface<carb::settings::ISettings>();
    PhysXSettings::getInstance().setDefaults();
    readPersistentSettings();

    mSimulationEventStreamV2 = carb::events::getCachedEventsInterface()->createEventStream();
    mErrorEventStream = carb::events::getCachedEventsInterface()->createEventStream();

    mTriggerManager = new TriggerManager();
    mCustomJointManager = new PhysXCustomJointManager();
    mCustomGeometryManager = new PhysXCustomGeometryManager();
    mPropertyQueryManager = new PhysXPropertyQueryManager();
    mRaycastManager = new RaycastManager();

    // Skip loading the CUDA driver library on CPU-only machines so isCudaLibPresent()
    // correctly returns false when the foundation's device probe forced CPU mode.
    {
        const omni::physx::IPhysxFoundation& physxFoundation = omni::physx::foundation::getInterface();
        const bool alreadyCpuMode = physxFoundation.isCpuMode && physxFoundation.isCpuMode();
        if (!alreadyCpuMode)
        {
#if CARB_PLATFORM_WINDOWS
            mCudaHandle = carb::extras::loadLibrary("nvcuda.dll");
#else
            mCudaHandle = dlopen("libcuda.so", RTLD_LAZY | RTLD_GLOBAL);
#endif
            CARB_LOG_INFO("\nomni.physx handle on CUDA lib is %p\n", mCudaHandle);
        }
    }

    mPhysXSetup.createPhysics();
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));

    // Registers custom tokens with the parse-lib's process-wide registry
    // (`omni::physics::usd::register*Token`) so they're visible to every
    // subsequent `scanStage` call.
    static const TfToken oldConvexPrim("ConvexMesh");
    static const TfToken planePrim("Plane");
    static const TfToken gearJointPrim("PhysxPhysicsGearJoint");
    static const TfToken rackJointPrim("PhysxPhysicsRackAndPinionJoint");
    static const TfToken physicsJointInstancerPrim("PhysxPhysicsJointInstancer");
    static const TfToken meshMergingCollisionAPI("PhysxMeshMergeCollisionAPI");
    omni::physics::usd::registerCustomShapeToken(oldConvexPrim);
    omni::physics::usd::registerCustomShapeToken(planePrim);
    omni::physics::usd::registerCustomShapeToken(meshMergingCollisionAPI);
    omni::physics::usd::registerCustomJointToken(gearJointPrim);
    omni::physics::usd::registerCustomJointToken(rackJointPrim);
    omni::physics::usd::registerCustomPhysicsInstancerToken(physicsJointInstancerPrim);

    auto uniqueSubscriptionIdGenerator = []() -> omni::physx::SubscriptionId {
        static omni::physx::SubscriptionId nextId = 0;
        return nextId++;
    };
    // Having a shared id generator for both pre-step and post-step simulation event registries allows us to
    // simplify the API so the user can unsubscribe(id) instead of having to specify unsubscribe(id, pre_map)
    mPreStepSubscriptions.setIdGenerator(uniqueSubscriptionIdGenerator);
    mPostStepSubscriptions.setIdGenerator(uniqueSubscriptionIdGenerator);

    enableLogChannel(kRoboticsLogChannel, mISettings->getAsBool(kSettingLogRobotics));
    enableLogChannel(kSceneMultiGPULogChannel, mISettings->getAsBool(kSettingLogSceneMultiGPU));
    subscribeToSettingsChangeEvents();

    createInternalPhysXDatabase();
    mWasStarted = true;

    createInternalPhysXDatabase();
}

void OmniPhysX::onShutdown()
{
    mWasStarted = false;
    unsubscribeFromSettingsChangeEvents();
    if (UsdLoad::getUsdLoad()->getActiveAttachedStage())
    {
        CARB_LOG_WARN("USD stage detach not called, holding a loose ptr to a stage!");
        physXDetach();
    }

    releasePhysXScenes();

    clearDebugVisualizationData();
    releaseMeshCache();

    mPhysXSetup.releasePhysics();

    getTriggerManager()->release();

    mPostStepSubscriptions.clear();
    mPreStepSubscriptions.clear();
    mSimulationSubscriptions.clear();

    UsdLoad::releaseUsdLoad();

    delete mTriggerManager;
    delete mCustomJointManager;
    delete mCustomGeometryManager;
    delete mPropertyQueryManager;
    delete mRaycastManager;

    mReplicatorMap.clear();

    mSimulationEventStreamV2 = nullptr;
    mErrorEventStream = nullptr;

    mIDictionary = nullptr;
    mITasking = nullptr;
    mISettings = nullptr;

    if (mCudaHandle)
    {
        carb::extras::unloadLibrary(mCudaHandle);
        mCudaHandle = nullptr;
    }

    gOmniPhysXInstance = nullptr;
}

OmniPhysX& OmniPhysX::getInstance()
{
    if (!gOmniPhysXInstance)
    {
        gOmniPhysXInstance = ICE_NEW(OmniPhysX);
    }
    return *gOmniPhysXInstance;
}

OmniPhysX* OmniPhysX::getInstanceCheck()
{
    return gOmniPhysXInstance;
}

void OmniPhysX::createOmniPhysXInstance()
{
    if (!gOmniPhysXInstance)
    {
        gOmniPhysXInstance = ICE_NEW(OmniPhysX);
    }
}

void OmniPhysX::subscribeToSettingsChangeEvents()
{
    carb::dictionary::SubscriptionId* subID;
    auto localMeshCacheChangedLambda = [](const carb::dictionary::Item* changedItem,
                                          carb::dictionary::ChangeEventType eventType, void* userData) {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
        // We must delay local mesh creation to make sure that its async pump has finished processing tasks
    };

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingLogRobotics,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            enableLogChannel(kRoboticsLogChannel, dict->getAsBool(changedItem));
        },
        nullptr);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingLogSceneMultiGPU,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            enableLogChannel(kSceneMultiGPULogChannel, dict->getAsBool(changedItem));
        },
        nullptr);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kViewportGizmoScalePath,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            // The change-callback fires only on an explicit set, so honor the value verbatim --
            // including exactly 0, which deliberately turns debug viz off (eSCALE =
            // viewportGizmoScale * visualizationScale). The "unset -> 1.0" default that keeps
            // debug viz visible in headless / minimal hosts is applied once at init below; it
            // is NOT re-applied here so an explicit 0 is distinguishable from an absent setting.
            const float gizmoScaleSetting = dict->getAsFloat(changedItem);
            omniPhysX.mCachedSettings.viewportGizmoScale = gizmoScaleSetting;
            omniPhysX.setDebugVisualizationDirty(true);
        },
        nullptr);
    // Default an UNSET viewport gizmo scale to 1.0 so debug-viz eSCALE (= viewportGizmoScale *
    // visualizationScale) is non-zero in headless / minimal hosts that never seed the viewport
    // gizmo scale; an explicitly-set value -- including 0 -- is honored (distinguishing "unset"
    // from "deliberately zeroed"). This cached value feeds every eSCALE consumer, including the
    // per-step debug-viz refresh in PhysXUpdate.cpp.
    const bool gizmoScaleSet = mISettings->isAccessibleAs(carb::dictionary::ItemType::eFloat, kViewportGizmoScalePath);
    const float gizmoScaleInit = mISettings->getAsFloat(kViewportGizmoScalePath);
    mCachedSettings.viewportGizmoScale = gizmoScaleSet ? gizmoScaleInit : 1.0f;
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingDisplayParticles,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.visualizationDisplayParticles = dict->getAsInt(changedItem);
        },
        nullptr);
    mCachedSettings.visualizationDisplayParticles = mISettings->getAsInt(kSettingDisplayParticles);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingUpdateToUsd,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.updateToUsd = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.updateToUsd = mISettings->getAsBool(kSettingUpdateToUsd);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingUpdateToUsdUsingXformCommonAPI,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.updateToUsdUsingXformCommonAPI = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.updateToUsdUsingXformCommonAPI = mISettings->getAsBool(kSettingUpdateToUsdUsingXformCommonAPI);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingUpdateVelocitiesToUsd,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.updateVelocitiesToUsd = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.updateVelocitiesToUsd = mISettings->getAsBool(kSettingUpdateVelocitiesToUsd);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingOutputVelocitiesLocalSpace,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.outputVelocitiesLocalSpace = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.outputVelocitiesLocalSpace = mISettings->getAsBool(kSettingOutputVelocitiesLocalSpace);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingUpdateParticlesToUsd,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.updateParticlesToUsd = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.updateParticlesToUsd = mISettings->getAsBool(kSettingUpdateParticlesToUsd);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingMinFrameRate,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.minFrameRate = dict->getAsInt(changedItem);
        },
        nullptr);
    mCachedSettings.minFrameRate = mISettings->getAsInt(kSettingMinFrameRate);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingJointBodyTransformCheckTolerance,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.jointBodyTransformCheckTolerance = dict->getAsFloat(changedItem);
        },
    nullptr);
    mCachedSettings.jointBodyTransformCheckTolerance = mISettings->getAsFloat(kSettingJointBodyTransformCheckTolerance);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingSimulateEmptyScene,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.simulateEmptyScene = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.simulateEmptyScene = mISettings->getAsBool(kSettingSimulateEmptyScene);
    mSubscribedSettings.push_back(subID);


    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingSynchronousKernelLaunches,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.enableSynchronousKernelLaunches = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.enableSynchronousKernelLaunches = mISettings->getAsBool(kSettingSynchronousKernelLaunches);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingDisableContactProcessing,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.disableContactProcessing = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.disableContactProcessing = mISettings->getAsBool(kSettingDisableContactProcessing);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingDefaultSimulator,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.defaultSimulator = dict->get<std::string>(changedItem);
            UsdLoad::getUsdLoad()->changeDefaultSimulator(omniPhysX.mCachedSettings.defaultSimulator);
        },
        nullptr);
    mCachedSettings.defaultSimulator = mISettings->getStringBuffer(kSettingDefaultSimulator);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingEnableExtendedJointAngles,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.enableExtendedJointAngles = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.enableExtendedJointAngles = mISettings->getAsBool(kSettingEnableExtendedJointAngles);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingCollisionApproximateCones,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.approximateCones = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.approximateCones = mISettings->getAsBool(kSettingCollisionApproximateCones);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingCollisionApproximateCylinders,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.approximateCylinders = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.approximateCylinders = mISettings->getAsBool(kSettingCollisionApproximateCylinders);
    mSubscribedSettings.push_back(subID);

}

void OmniPhysX::unsubscribeFromSettingsChangeEvents()
{
    for (auto subID : mSubscribedSettings)
    {
        mISettings->unsubscribeToChangeEvents(subID);
    }
    mSubscribedSettings.clear();
}

bool OmniPhysX::registerReplicator(uint64_t id, const IReplicatorCallback& callback)
{
    AttachedStage* stage = UsdLoad::getUsdLoad()->getAttachedStage(id);
    if (stage)
    {
        stage->setReplicatorStage(true);
    }

    // insert_or_assign, not insert: the latest registrant's callbacks must win. A plain
    // insert is a silent no-op when the stage is already registered, so a caller's
    // callbacks would be dropped while the call still reported success. Safe to replace:
    // PhysXReplicator holds no PhysX state between register and replicate.
    const bool inserted = mReplicatorMap.insert_or_assign(id, PhysXReplicator(callback)).second;
    if (!inserted)
    {
        CARB_LOG_INFO("registerReplicator: replaced existing replicator registration for stage %llu",
                      static_cast<unsigned long long>(id));
    }
    return true;
}

void OmniPhysX::unregisterReplicator(uint64_t id)
{
    ReplicatorMap::iterator fit = mReplicatorMap.find(id);
    if (fit != mReplicatorMap.end())
    {
        fit->second.clear();
        mReplicatorMap.erase(fit);

        AttachedStage* stage = UsdLoad::getUsdLoad()->getAttachedStage(id);
        if (stage)
        {
            stage->setReplicatorStage(false);
        }
    }
}
