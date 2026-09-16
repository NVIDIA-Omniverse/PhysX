// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-SIM-DEFAULT-001
 * @covers AC-2
 *
 * @implements REQ-SIM-OVSTAGE-ATTACH-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-REPLICATE-002
 * @covers AC-1 AC-2 AC-3
 */

// This file is pxr-free. OmniPhysX::physXAttach -- the only USD-specific member left here --
// is defined in the pxr-free usdBridge/RuntimeBridge.cpp (ADR-0027).
#include <omni/physics/parse/CustomTokens.h>

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
#include "usdInterface/UsdInterface.h"

#include "particles/PhysXParticlePost.h"

#include <common/utilities/MemoryMacros.h>

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

void OmniPhysX::physXAttachSession()
{
    createInternalPhysXDatabase();

    SimulationCallbacks::getSimulationCallbacks()->reset();

    mCurrentTimestampOffset = mSimulationTimestamp;
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));
    // Kit-inspector-only debug filter, unreachable from ovphysx's own ovstage attach path
    // -- see setForceParseOnlySingleScene's declaration comment.
    {
        const char* forceSingleScene = mISettings->getStringBuffer(kSettingForceParseOnlySingleScene);
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(forceSingleScene ? forceSingleScene : std::string());
    }
}

// OmniPhysX::physXAttach is defined in usdBridge/RuntimeBridge.cpp: its body resolves stageId
// through the reparse seam. Both call sites (loadTargetStage/loadTargetStage_Id in PhysX.cpp)
// are benchmark-harness-only; with no seam it is an error-logging stub.

bool OmniPhysX::physXAttachOvstage(const void* ovstageAttachPayload,
                                  uint64_t readOrdinal,
                                  AttachOvstageBackingStageHandle backingStage,
                                  uint64_t effectiveBackingStageId)
{
    // Same session prologue as the USD attach, with no stage; the only divergence
    // is the stageless attachOvstage() entry (which installs the ovstage backends).
    physXAttachSession();

    // Same pending-registration rule as physXAttach(): the only key a pre-attach registrant can
    // have used is kActiveAttach. This drops the effectiveBackingStageId != 0 guard that stage-id
    // keying forced -- a stageless attach has no stage id to key on (0 meant "no stage", so no
    // replicator could ever be found for it), but it does get an attach handle like any other
    // attach, so a registration now applies to it too.
    ReplicatorMap::iterator pendingReplicator = mReplicatorMap.find(kActiveAttach);
    PhysXReplicator* replicator = pendingReplicator != mReplicatorMap.end() ? &pendingReplicator->second : nullptr;

    const bool attached = UsdLoad::getUsdLoad()->attachOvstage(ovstageAttachPayload,
                                                               readOrdinal,
                                                               effectiveBackingStageId,
                                                               &getPhysXUsdPhysicsInterface(),
                                                               /*loadPhysics=*/replicator == nullptr,
                                                               backingStage);
    if (!attached)
        return false;

    if (replicator &&
        !replicator->attach(effectiveBackingStageId, &getPhysXUsdPhysicsInterface(), /*attachStage=*/false))
    {
        return false;
    }

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

    // Clear any leftover mouse-interaction command (e.g. a repeating push) from the
    // process-global raycast buffer. It is otherwise cleared only on the Kit stage-update
    // resume path (PhysXStageUpdate), so on a direct-API detach a stale ePush leaks into the
    // next attach and applies a recurring impulse to whatever body its stale raycast hits.
    getRaycastManager().clearCommandBuffer();

    {
        std::unique_lock<carb::tasking::MutexWrapper> simStartedLock(mSimParamMutex);
        if (mHasSimulationStarted)
        {
            mHasSimulationStarted = false;
            sendSimulationEvent(SimulationEvent::eStopped);
        }
    }

    // A replicator registration that applied to this attach (found under either spelling --
    // kActiveAttach or this attach's own handle, see findReplicatorEntry) must not survive the
    // attach it was applied to. Otherwise a later, unrelated attach that skips re-registering
    // would inherit this attach's stale replicationAttachFn/exclude-paths/loadPhysics override --
    // the same hazard class already closed for the clone()/tensor endpoint (see getReplicator()).
    // Must run before detach() below invalidates the active AttachedStage, since
    // unregisterReplicator() resolves the handle to reset AttachedStage::setReplicatorStage()
    // too.
    //
    // Gated on isReplicatorStage(): that flag is set only once a replicator actually attaches
    // to this stage (UsdLoad::attachReplicatorFinish / OmniPhysX::registerReplicator on an already-live
    // handle), so a registration still pending under kActiveAttach for an attach that never
    // completed -- e.g. a transactional attachOvstage() rollback that tears down a partial attach
    // before any replicator applied -- is untouched here and remains available to the next attach
    // attempt. Without this gate the rollback's detach() would erase that pending registration via
    // the kActiveAttach fallback in findReplicatorEntry(), silently dropping the caller's replicator
    // registration for good.
    const AttachHandle activeAttachHandle = UsdLoad::getUsdLoad()->getActiveAttachHandle();
    if (activeAttachHandle != UsdLoad::kNoAttachHandle)
    {
        AttachedStage* activeStage = UsdLoad::getUsdLoad()->resolveAttach(activeAttachHandle);
        if (activeStage && activeStage->isReplicatorStage())
        {
            if (getReplicator(activeAttachHandle))
            {
                CARB_LOG_WARN(
                    "physXDetach: discarding the replicator registration for attach %llu -- it does not "
                    "survive this detach, register again before the next attach if replication should "
                    "still apply.",
                    static_cast<unsigned long long>(activeAttachHandle));
            }
            unregisterReplicator(activeAttachHandle);
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

    // resetStartProperties() authors to USD, and the change notices sent when its SdfChangeBlock
    // closes can re-enter physics and tear down or re-create the PhysX SDK together with the async
    // cooking singleton, so the pointer fetched above may be dangling by now. Re-fetch it.
    // blockUSDUpdate() is a counter guarded against underflow, so unbalancing the pair against a
    // freshly created singleton is harmless in release; the guard is preceded by a CARB_ASSERT, so
    // an assert-enabled build reports the unbalanced release rather than misbehaving.
    cookingDataAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
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

    // Registers custom tokens with the parse-lib's process-wide, source-agnostic registry
    // (omni/physics/parse/CustomTokens.h) so they are visible to every subsequent scan.
    // These six are also pre-seeded by the registry's own constructor, making this an
    // idempotent no-op -- kept as the runtime's own public registration surface.
    using omni::physics::parse::CustomTokenKind;
    omni::physics::parse::registerCustomToken(CustomTokenKind::eShape, "ConvexMesh");
    omni::physics::parse::registerCustomToken(CustomTokenKind::eShape, "Plane");
    omni::physics::parse::registerCustomToken(CustomTokenKind::eShape, "PhysxMeshMergeCollisionAPI");
    omni::physics::parse::registerCustomToken(CustomTokenKind::eJoint, "PhysxPhysicsGearJoint");
    omni::physics::parse::registerCustomToken(CustomTokenKind::eJoint, "PhysxPhysicsRackAndPinionJoint");
    omni::physics::parse::registerCustomToken(CustomTokenKind::ePhysicsInstancer, "PhysxPhysicsJointInstancer");

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

namespace omni::physx::ovx
{
// Defined in OvxPhysicsRead.cpp. Drains the read-column pool before CUDA/foundation teardown.
void ovxDrainColumnPools();
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

    // Free the read-column pool's device/pinned buffers and drop its manager references while the CUDA
    // context managers are still alive -- releasePhysics() below destroys them. Without this the pool's
    // only drain is the next read's stale sweep, so a detach + shutdown with no later read leaks them
    // across CUDA teardown.
    ovx::ovxDrainColumnPools();

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
    const char* defaultSimulator = mISettings->getStringBuffer(kSettingDefaultSimulator);
    mCachedSettings.defaultSimulator = defaultSimulator ? defaultSimulator : "";
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

PhysXReplicator* OmniPhysX::getReplicator(AttachHandle attachHandle)
{
    ReplicatorMap::iterator fit = findReplicatorEntry(attachHandle);
    return fit != mReplicatorMap.end() ? &fit->second : nullptr;
}

ReplicatorMap::iterator OmniPhysX::findReplicatorEntry(AttachHandle attachHandle)
{
    ReplicatorMap::iterator fit = mReplicatorMap.find(attachHandle);
    if (fit != mReplicatorMap.end())
        return fit;

    // The registration key is whichever spelling of the attach the registrant had at the time:
    // kActiveAttach when it registered before the attach existed (the classic replicator flow),
    // the real handle when it registered after (the clone() entry point). Both name the same
    // attach whenever there is exactly one, so accept either spelling for the other rather than
    // making the lookup depend on when the caller happened to register.
    const AttachHandle activeHandle = UsdLoad::getUsdLoad()->getActiveAttachHandle();
    if (activeHandle == UsdLoad::kNoAttachHandle)
        return mReplicatorMap.end();

    if (attachHandle == kActiveAttach)
        return mReplicatorMap.find(activeHandle);
    if (attachHandle == activeHandle)
        return mReplicatorMap.find(kActiveAttach);

    return mReplicatorMap.end();
}

bool OmniPhysX::registerReplicator(AttachHandle attachHandle, const IReplicatorCallback& callback)
{
    // An unresolvable handle is not by itself an error here: registering *before* attaching is the
    // documented flow, and no handle exists then. But kActiveAttach is the only value that can name
    // an attach that does not exist yet, so anything else that resolves to nothing is a caller
    // still passing a stage id, or a handle whose attach is gone. Such a registration would sit in
    // the map forever and never fire, so reject it loudly instead (ADR-0016 Decision 3).
    AttachedStage* stage = UsdLoad::getUsdLoad()->resolveAttach(attachHandle);
    if (!stage && attachHandle != kActiveAttach)
    {
        CARB_LOG_ERROR(
            "registerReplicator: %llu does not name a live attach. Pass the handle from "
            "IPhysxSimulation::getAttachHandle(), or kActiveAttach for the lone active attach "
            "(also the only usable value when registering before attaching).",
            static_cast<unsigned long long>(attachHandle));
        return false;
    }

    if (stage)
    {
        stage->setReplicatorStage(true);
    }

    // A prior registration may already exist under the alias spelling of this same
    // attach (kActiveAttach vs. its concrete handle) -- e.g. registered pre-attach
    // under kActiveAttach, now being replaced under the concrete handle obtained
    // from getAttachHandle(). The insert_or_assign below only ever touches the exact
    // key it is given, so without this the aliased entry would survive alongside the
    // new one: unregisterReplicator() on detach resolves and erases only the entry
    // actually keyed by attachHandle, leaving the old callback/userData in the map to
    // wrongly apply to the next, unrelated attach. Erase it first so both spellings
    // never coexist as separate entries.
    ReplicatorMap::iterator aliasedEntry = findReplicatorEntry(attachHandle);
    if (aliasedEntry != mReplicatorMap.end() && aliasedEntry->first != attachHandle)
    {
        mReplicatorMap.erase(aliasedEntry);
    }

    // insert_or_assign, not insert: the latest registrant's callbacks must win. A plain
    // insert is a silent no-op when the attach is already registered, so a caller's
    // callbacks would be dropped while the call still reported success. Safe to replace:
    // PhysXReplicator holds no PhysX state between register and replicate.
    const bool inserted = mReplicatorMap.insert_or_assign(attachHandle, PhysXReplicator(callback)).second;
    if (!inserted)
    {
        CARB_LOG_INFO("registerReplicator: replaced existing replicator registration for attach %llu",
                      static_cast<unsigned long long>(attachHandle));
    }
    return true;
}

void OmniPhysX::unregisterReplicator(AttachHandle attachHandle)
{
    // Same either-spelling rule as the lookup: a registration made pre-attach under kActiveAttach
    // must still be removable by a caller that has since obtained the real handle, and vice versa.
    ReplicatorMap::iterator fit = findReplicatorEntry(attachHandle);
    if (fit != mReplicatorMap.end())
    {
        fit->second.clear();
        mReplicatorMap.erase(fit);

        AttachedStage* stage = UsdLoad::getUsdLoad()->resolveAttach(attachHandle);
        if (stage)
        {
            stage->setReplicatorStage(false);
        }
    }
}
