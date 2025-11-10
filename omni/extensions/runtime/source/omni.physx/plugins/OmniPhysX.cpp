// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

#include "particles/PhysXParticlePost.h"
#include "particles/FabricParticles.h"

#include <common/utilities/MemoryMacros.h>

using namespace pxr;
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
    if (mPhysXStats)
    {
        mPhysXStats->setStage(nullptr);
    }
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

void OmniPhysX::physXAttach(long int stageId, bool loadPhysics)
{
    UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    mIStageReaderWriter = carb::getFramework()->tryAcquireInterface<omni::fabric::IStageReaderWriter>();
    createInternalPhysXDatabase();

    mStage = stage;
    mStageId = stageId;

    SimulationCallbacks::getSimulationCallbacks()->reset();

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    SAFE_DELETE_SINGLE(mFabricParticles);
    mFabricParticles = new FabricParticles;
    mFabricParticles->attachStage(stageId);
#endif

    if (mPhysXStats)
    {
        mPhysXStats->setStage(mStage);
    }

    OmniPhysX::getInstance().getPhysXSetup().getPhysics();
    // A.B. TODO this is wrong and needs to go into individual scenes
    mCurrentTimestampOffset = mSimulationTimestamp;
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));
    if(mISettings->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(pxr::SdfPath(mISettings->getStringBuffer(kSettingForceParseOnlySingleScene)));
    else
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(pxr::SdfPath());
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

void OmniPhysX::physXDetach()
{
    {
        std::unique_lock<carb::tasking::MutexWrapper> simStartedLock(mSimParamMutex);
        if (mHasSimulationStarted)
        {
            mHasSimulationStarted = false;
            sendSimulationEvent(SimulationEvent::eStopped);
        }
    }

    UsdLoad::getUsdLoad()->detach(getStageId());

    releasePhysXScenes();

    SimulationCallbacks::getSimulationCallbacks()->reset();

    mStage = nullptr;
    mStageId = 0u;

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    if (mFabricParticles)
    {
        mFabricParticles->detachStage();
        SAFE_DELETE_SINGLE(mFabricParticles);
    }
#endif
}

void OmniPhysX::resetSimulation()
{
    OmniPhysX& omniPhysX = *this;

    const bool outputVelocitiesLocalSpace = omniPhysX.getISettings()->getAsBool(kSettingOutputVelocitiesLocalSpace);
    const bool useUsdUpdate = omniPhysX.getISettings()->getAsBool(kSettingUpdateToUsd);
    const bool useUsdVelocitiesUpdate = omniPhysX.getISettings()->getAsBool(kSettingUpdateVelocitiesToUsd);

    waitForSimulationCompletion(false);

    UsdLoad::getUsdLoad()->blockUSDUpdate(true);
    const bool changeTrackingPaused = UsdLoad::getUsdLoad()->isChangeTrackingPaused(omniPhysX.getStageId());
    UsdLoad::getUsdLoad()->pauseChangeTracking(omniPhysX.getStageId(), true);

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
    UsdLoad::getUsdLoad()->releasePhysicsObjects(omniPhysX.getStageId());
    UsdLoad::getUsdLoad()->pauseChangeTracking(omniPhysX.getStageId(), changeTrackingPaused);
    UsdLoad::getUsdLoad()->blockUSDUpdate(false);

    omniPhysX.sendSimulationEvent(SimulationEvent::eStopped);
    omniPhysX.setSimulationStarted(false);
    omniPhysX.setSimulationRunning(false);

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    FabricParticles* fabricParticles = omniPhysX.getFabricParticles();
    if (fabricParticles)
    {
        fabricParticles->reset();
    }
#endif
}

static void readPersistentSettings()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::settings::ISettings* iSettings = omniPhysX.getISettings();
    omniPhysX.setGpuPipelineOverride(iSettings->getAsInt(kSettingOverrideGPU));
    omniPhysX.getPhysXSetup().setThreadCount(iSettings->getAsInt(kSettingNumThreads));
    omniPhysX.getPhysXSetup().setMaxNumberOfPhysXErrors(iSettings->getAsInt(kSettingMaxNumberOfPhysXErrors));
    cookingdataasync::CookingDataAsync* cookingDataAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
    if(cookingDataAsync)
    {
        cookingDataAsync->setLocalMeshCacheEnabled(iSettings->getAsBool(kSettingUseLocalMeshCache));
        cookingDataAsync->setLocalMeshCacheSize(iSettings->getAsInt(kSettingLocalMeshCacheSizeMB));
    }
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
    mIStageReaderWriter = framework->tryAcquireInterface<omni::fabric::IStageReaderWriter>();

    mIDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();

    mITasking = carb::getCachedInterface<carb::tasking::ITasking>();

    mIConvexDecomposition = carb::getCachedInterface<omni::convexdecomposition::ConvexDecomposition>();
    CARB_ASSERT(mIConvexDecomposition);

    mIUsdPhysicsParse = carb::getCachedInterface<omni::physics::usdparser::IUsdPhysicsParse>();
    CARB_ASSERT(mIUsdPhysicsParse);

    // set default physics settings
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

    // load cuda if available
    #if CARB_PLATFORM_WINDOWS
    mCudaHandle = carb::extras::loadLibrary("nvcuda.dll");
#else
    mCudaHandle = dlopen("libcuda.so", RTLD_LAZY | RTLD_GLOBAL);
#endif
    CARB_LOG_INFO("\nomni.physx handle on CUDA lib is %p\n", mCudaHandle);

    // creates physics
    mPhysXSetup.createPhysics();
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));

    // custom types definitions to UsdPhysics
    omni::physics::schema::IUsdPhysics* usdPhysics = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
    static const TfToken oldConvexPrim("ConvexMesh");
    static const TfToken planePrim("Plane");
    static const TfToken gearJointPrim("PhysxPhysicsGearJoint");
    static const TfToken rackJointPrim("PhysxPhysicsRackAndPinionJoint");
    static const TfToken physicsJointInstancerPrim("PhysxPhysicsJointInstancer");
    static const TfToken meshMergingCollisionAPI("PhysxMeshMergeCollisionAPI");
    usdPhysics->addCustomShapeToken(oldConvexPrim);
    usdPhysics->addCustomShapeToken(planePrim);
    usdPhysics->addCustomShapeToken(meshMergingCollisionAPI);
    usdPhysics->addCustomJointToken(gearJointPrim);
    usdPhysics->addCustomJointToken(rackJointPrim);
    usdPhysics->addCustomPhysicsInstancerToken(physicsJointInstancerPrim);

    // Static storage lambda, this will survive
    auto uniqueSubscriptionIdGenerator = []() -> omni::physx::SubscriptionId {
        static omni::physx::SubscriptionId nextId = 0;
        return nextId++;
    };
    // Having a shared id generator for both pre-step and post-step simulation event registries allows us to
    // simplify the API so the user can unsubscribe(id) instead of having to specify unsubscribe(id, pre_map)
    mPreStepSubscriptions.setIdGenerator(uniqueSubscriptionIdGenerator);
    mPostStepSubscriptions.setIdGenerator(uniqueSubscriptionIdGenerator);

    mPhysXStats = new PhysXStats();
    if (mStage)
    {
        mPhysXStats->setStage(mStage);
    }

    enableLogChannel(kRoboticsLogChannel, mISettings->getAsBool(kSettingLogRobotics));
    enableLogChannel(kSceneMultiGPULogChannel, mISettings->getAsBool(kSettingLogSceneMultiGPU));
    subscribeToSettingsChangeEvents();
    mWasStarted = true;

    createInternalPhysXDatabase();
}

void OmniPhysX::onShutdown()
{
    mWasStarted = false;
    unsubscribeFromSettingsChangeEvents();
    if (mStage)
    {
        CARB_LOG_WARN("USD stage detach not called, holding a loose ptr to a stage!");
        physXDetach();
    }

    if (mPhysXStats)
    {
        delete mPhysXStats;
        mPhysXStats = nullptr;
    }
    mUploadPhysXStatsToCarb = false;

    releasePhysXScenes();

    mIConvexDecomposition = nullptr;
    mIUsdPhysicsParse = nullptr;

    clearDebugVisualizationData();
    releaseMeshCache();

    mPhysXSetup.releasePhysics();

    getTriggerManager()->release();

    mPostStepSubscriptions.clear();
    mPreStepSubscriptions.clear();
    mSimulationSubscriptions.clear();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    delete usdLoad;

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
    mIStageReaderWriter = nullptr;
    mISettings = nullptr;

    mStage = nullptr;
    mStage = 0u;

    if (mCudaHandle)
    {
        carb::extras::unloadLibrary(mCudaHandle);
        mCudaHandle = nullptr;
    }

    gOmniPhysXInstance = nullptr;
}

OmniPhysX& OmniPhysX::getInstance()
{
    return *gOmniPhysXInstance;
}

const OmniPhysX* OmniPhysX::getInstanceCheck()
{
    return gOmniPhysXInstance;
}

void OmniPhysX::createOmniPhysXInstance()
{
    gOmniPhysXInstance = ICE_NEW(OmniPhysX);
}

void OmniPhysX::subscribeToSettingsChangeEvents()
{
    carb::dictionary::SubscriptionId* subID;
    auto localMeshCacheChangedLambda = [](const carb::dictionary::Item* changedItem,
                                          carb::dictionary::ChangeEventType eventType, void* userData) {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
        // We must delay local mesh creation to make sure that its async pump has finished processing tasks
        cachedSettings.localMeshCacheEnabled = omniPhysX.getISettings()->getAsBool(kSettingUseLocalMeshCache);
        cachedSettings.localMeshCacheSize = omniPhysX.getISettings()->getAsInt(kSettingLocalMeshCacheSizeMB);
        cachedSettings.delayedInitLocalMeshCache = true;
    };
    // set local mesh cache change callbacks
    subID = mISettings->subscribeToNodeChangeEvents(kSettingUseLocalMeshCache, localMeshCacheChangedLambda, nullptr);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(kSettingLocalMeshCacheSizeMB, localMeshCacheChangedLambda, nullptr);
    mSubscribedSettings.push_back(subID);

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
            omniPhysX.mCachedSettings.viewportGizmoScale = dict->getAsFloat(changedItem);
        },
        nullptr);
    mCachedSettings.viewportGizmoScale = mISettings->getAsFloat(kViewportGizmoScalePath);
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
    kSettingUpdateResidualsToUsd,
    [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        omniPhysX.mCachedSettings.updateResidualsToUsd = dict->getAsBool(changedItem);
    },
    nullptr);
    mCachedSettings.updateResidualsToUsd = mISettings->getAsBool(kSettingUpdateResidualsToUsd);
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
        kSettingDisableSleeping,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.disableSleeping = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.disableSleeping = mISettings->getAsBool(kSettingDisableSleeping);
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
            omniPhysX.mCachedSettings.defaultSimulator = dict->getStringBuffer(changedItem);
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

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingFabricEnabled,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            omniPhysX.mCachedSettings.fabricEnabled = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.fabricEnabled = mISettings->getAsBool(kSettingFabricEnabled);
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

    mReplicatorMap.insert(std::make_pair(id, PhysXReplicator(callback)));
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
