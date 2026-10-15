// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-4

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-3 AC-4

// @implements REQ-OMNIPVD-LATE-001
// @covers AC-9

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
#include "ScopedNoticeLock.h"
#include <stdexcept>
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
#include "RuntimeExecution.h"
#include "PhysXDebugVisualization.h"
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

void OmniPhysX::sendSimulationEvent(SimulationEvent type, AttachHandle attach)
{
    using namespace carb::events;
    {
        carb::events::IEventPtr event = carb::stealObject(
            mSimulationEventStreamV2->createEventPtr(static_cast<EventType>(type), kGlobalSenderId));
        getIDictionary()->makeInt64AtPath(event->payload, "attachHandle", static_cast<int64_t>(attach));
        RuntimeCallbackScope callbackScope;
        mSimulationEventStreamV2->dispatch(event.get());
    }
}

void OmniPhysX::releasePhysXScenes()
{
    //update postprocess before any particle sets are released
    omni::physx::particles::notifyAllPhysXRelease();
    mPhysXSetup.releasePhysXScenes();
    mCustomJointManager->clear();
    mCustomGeometryManager->clear();
    for (ReplicatorMap::reference ref : mReplicatorMap)
    {
        ref.second->clear();
    }

    mProfileStats.clear();
    mCrossProfileStats.clear();
}



// OmniPhysX::physXAttach is defined in usdBridge/RuntimeBridge.cpp: its body resolves stageId
// through the reparse seam. Both call sites (loadTargetStage/loadTargetStage_Id in PhysX.cpp)
// are benchmark-harness-only; with no seam it is an error-logging stub.

AttachHandle OmniPhysX::physXAttachOvstage(const void* ovstageAttachPayload,
                                  uint64_t readOrdinal,
                                  AttachOvstageBackingStageHandle backingStage,
                                  uint64_t effectiveBackingStageId,
                                  const PhysxAttachOptions& options,
                                  bool startConfiguredRecording)
{
    // Candidates snapshot these parser defaults during construction. Do not
    // create an active database or change a live attachment's clocks here.
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));
    const char* forceSingleScene = mISettings->getStringBuffer(kSettingForceParseOnlySingleScene);
    getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(forceSingleScene ? forceSingleScene : std::string());

    return UsdLoad::getUsdLoad()->attachOvstage(ovstageAttachPayload, readOrdinal,
        effectiveBackingStageId, &getPhysXUsdPhysicsInterface(), /*loadPhysics=*/true,
        backingStage, &options, startConfiguredRecording);
}

bool OmniPhysX::physXUpdateFromOvStage(AttachHandle attach, uint64_t fromOrdinal, uint64_t toOrdinal)
{
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    return attachedStage ? attachedStage->updateFromOvStage(fromOrdinal, toOrdinal) : false;
}

bool OmniPhysX::physXDetach(AttachHandle attach)
{
    UsdLoad* loader = UsdLoad::getUsdLoad();
    AttachedStage* owner = loader->getAttachedStageByHandle(attach);
    if (!owner)
        return false;
    waitForSimulationCompletion(*owner, false);
    getPropertyQueryManager().cancelStageRequests(owner->getStageId(), PhysxPropertyQueryResult::eERROR_RUNTIME);
    unregisterReplicator(attach);
    const bool stopped = owner->hasSimulationStarted();
    if (!loader->retireAttachment(attach))
        return false;
    if (stopped)
        sendSimulationEvent(SimulationEvent::eStopped, attach);
    return true;
}

void OmniPhysX::resetSimulation(AttachedStage& owner)
{
    OmniPhysX& omniPhysX = *this;

    const OmniCachedSettings& settings = owner.getCachedSettings();
    const bool outputVelocitiesLocalSpace = settings.outputVelocitiesLocalSpace;
    const bool useUsdUpdate = settings.updateToUsd;
    const bool useUsdVelocitiesUpdate = settings.updateVelocitiesToUsd;

    waitForSimulationCompletion(owner, false);
    retireRuntimeExecution(owner.getAttachHandle());

    AttachedStage* attachedStage = &owner;
    ScopedNoticeBlock blockNotices(*attachedStage);

    omni::physx::internal::InternalPhysXDatabase& db = owner.getInternalPhysXDatabase();
    if (!settings.disableResetOnStop && settings.resetOnStop)
        db.resetStartProperties(useUsdUpdate, useUsdVelocitiesUpdate, outputVelocitiesLocalSpace);

    getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(*attachedStage, false);  // do not send these notifications when the simulation is to end
    UsdLoad::getUsdLoad()->releasePhysicsObjects(owner.getAttachHandle());

    omniPhysX.sendSimulationEvent(SimulationEvent::eStopped, attachedStage->getAttachHandle());
    owner.setSimulationStarted(false);
    owner.setSimulationRunning(false);
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

// Process-lifetime identities, including across runtime shutdown/reinitialize.
// Never recycle IDs: a removed subscription must not alias a later observer.
static SubscriptionId nextRuntimeEventId()
{
    static std::atomic<SubscriptionId> next{ 0 };
    SubscriptionId value = next.load(std::memory_order_relaxed);
    while (value < kInvalidSubscriptionId)
    {
        if (next.compare_exchange_weak(value, value + 1, std::memory_order_relaxed))
            return value;
    }
    return kInvalidSubscriptionId;
}

SubscriptionId OmniPhysX::addOnStepEventSubscription(AttachHandle attach, std::function<void(float, void*)> onUpdate,
                                                    void* userData, bool preStep, int order)
{
    auto owner = UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!owner)
        return kInvalidSubscriptionId;

    if (!onUpdate)
        return kInvalidSubscriptionId;
    const SubscriptionId id = nextRuntimeEventId();
    if (id != kInvalidSubscriptionId)
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
        StepEventMap& subscriptions = preStep ? owner->preStepSubscriptions : owner->postStepSubscriptions;
        subscriptions.emplace(std::make_pair(order, id), StepEventSub{ attach, onUpdate, userData });
    }
    return id;
}

void OmniPhysX::removeOnStepEventSubscription(SubscriptionId id)
{
    for (const auto& entry : UsdLoad::getUsdLoad()->getAttachedStages())
    {
        const auto& owner = entry.second;
    std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
    for (StepEventMap* subscriptions : { &owner->preStepSubscriptions, &owner->postStepSubscriptions })
    {
        for (auto it = subscriptions->begin(); it != subscriptions->end(); ++it)
        {
            if (it->first.second == id)
            {
                subscriptions->erase(it);
                break;
            }
        }
    }
    }
}

SubscriptionId OmniPhysX::addStatusEventSubscription(AttachHandle attach, std::function<void(SimulationStatusEvent, void*)> onEvent,
                                                    void* userData)
{
    auto owner = UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!owner)
        return kInvalidSubscriptionId;

    if (!onEvent)
        return kInvalidSubscriptionId;
    const SubscriptionId id = nextRuntimeEventId();
    if (id != kInvalidSubscriptionId)
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
        owner->simulationSubscriptions.emplace(id, SimulationEventSub{ attach, onEvent, userData });
    }
    return id;
}

void OmniPhysX::removeStatusEventSubscription(SubscriptionId id)
{
    for (const auto& entry : UsdLoad::getUsdLoad()->getAttachedStages())
    {
        const auto& owner = entry.second;
    std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
    owner->simulationSubscriptions.erase(id);
    }
}

void OmniPhysX::fireStatusEventSubscriptions(AttachHandle attach, SimulationStatusEvent eventStatus) const
{
    auto owner = UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!owner)
        return ;

    RuntimeCallbackScope callbackScope;
    SimulationEventMap snapshot;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
        snapshot = owner->simulationSubscriptions;
    }
    for (const SimulationEventMap::value_type& entry : snapshot)
    {
        if (entry.second.attach != attach)
            continue;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
            if (owner->simulationSubscriptions.find(entry.first) == owner->simulationSubscriptions.end())
                continue;
        }
        try { entry.second.eventFn(eventStatus, entry.second.userData); }
        catch (const std::exception& e) { CARB_LOG_ERROR("Physics status callback threw: %s", e.what()); }
        catch (...) { CARB_LOG_ERROR("Physics status callback threw an unknown exception"); }
    }
}

void OmniPhysX::retireEventSubscriptions(usdparser::AttachedStage& attachedStage)
{
    auto* owner = &attachedStage;
    const auto attach = owner->getAttachHandle();

    getTriggerManager()->retireSubscriptions(attach);
    std::lock_guard<carb::tasking::MutexWrapper> lock(owner->eventMutex);
    for (StepEventMap* subscriptions : { &owner->preStepSubscriptions, &owner->postStepSubscriptions })
    {
        for (auto it = subscriptions->begin(); it != subscriptions->end();)
            it = it->second.attach == attach ? subscriptions->erase(it) : std::next(it);
    }
    for (auto it = owner->simulationSubscriptions.begin(); it != owner->simulationSubscriptions.end();)
        it = it->second.attach == attach ? owner->simulationSubscriptions.erase(it) : std::next(it);
}

SubscriptionId OmniPhysX::addProfileStatsSubscription(ProfileStatsNotificationFn onEvent, void* userData)
{
    if (!onEvent)
        return kInvalidSubscriptionId;
    const SubscriptionId id = nextRuntimeEventId();
    if (id != kInvalidSubscriptionId)
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mEventSubscriptionsMutex);
        mProfileStatsSubscriptions.emplace(id, std::make_pair(onEvent, userData));
    }
    return id;
}

void OmniPhysX::removeProfileStatsSubscription(SubscriptionId id)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mEventSubscriptionsMutex);
    mProfileStatsSubscriptions.erase(id);
}

void OmniPhysX::fireProfileStatsSubscription()
{
    if (!mEnableProfile)
        return;
    RuntimeCallbackScope callbackScope;
    ProfileEventMap snapshot;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mEventSubscriptionsMutex);
        snapshot = mProfileStatsSubscriptions;
    }
    if (snapshot.empty())
        return;
    ProfileStatsVector stats;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(profileMutex);
        stats.swap(mProfileStats);
    }
    if (stats.empty())
        return;
    for (const ProfileEventMap::value_type& entry : snapshot)
    {
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mEventSubscriptionsMutex);
            if (mProfileStatsSubscriptions.find(entry.first) == mProfileStatsSubscriptions.end())
                continue;
        }
        try { entry.second.first(stats, entry.second.second); }
        catch (const std::exception& e) { CARB_LOG_ERROR("Physics profile callback threw: %s", e.what()); }
        catch (...) { CARB_LOG_ERROR("Physics profile callback threw an unknown exception"); }
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
    mUsdPhysicsInterface = std::make_shared<PhysXUsdPhysicsInterface>();

    mISettings = carb::getCachedInterface<carb::settings::ISettings>();
    PhysXSettings::getInstance().setDefaults();
    readPersistentSettings();

    mSimulationEventStreamV2 = carb::events::getCachedEventsInterface()->createEventStream();
    mErrorEventStream = carb::events::getCachedEventsInterface()->createEventStream();

    mTriggerManager = new TriggerManager();
    mCustomJointManager = new PhysXCustomJointManager();
    mCustomGeometryManager = new PhysXCustomGeometryManager();
    mPropertyQueryManager = new PhysXPropertyQueryManager();

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

    enableLogChannel(kRoboticsLogChannel, mISettings->getAsBool(kSettingLogRobotics));
    enableLogChannel(kSceneMultiGPULogChannel, mISettings->getAsBool(kSettingLogSceneMultiGPU));
    subscribeToSettingsChangeEvents();

    mWasStarted = true;

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
    while (!UsdLoad::getUsdLoad()->getAttachedStages().empty())
    {
        const AttachHandle attach = UsdLoad::getUsdLoad()->getAttachedStages().begin()->first;
        if (!physXDetach(attach))
            throw std::runtime_error("Could not retire attachment during runtime shutdown");
    }
    releasePhysXScenes();

    releaseMeshCache();

    // Free the read-column pool's device/pinned buffers and drop its manager references while the CUDA
    // context managers are still alive -- releasePhysics() below destroys them. Without this the pool's
    // only drain is the next read's stale sweep, so a detach + shutdown with no later read leaks them
    // across CUDA teardown.
    ovx::ovxDrainColumnPools();

    mPhysXSetup.releasePhysics();

    getTriggerManager()->release();


    UsdLoad::releaseUsdLoad();

    delete mTriggerManager;
    delete mCustomJointManager;
    delete mCustomGeometryManager;
    delete mPropertyQueryManager;

    mReplicatorMap.clear();

    mSimulationEventStreamV2 = nullptr;
    mErrorEventStream = nullptr;

    mUsdPhysicsInterface.reset();
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
        kSettingOverrideGPU,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType, void*) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            OmniPhysX::getInstance().getCachedSettings().gpuPipelineOverride = dict->getAsInt(changedItem);
        },
        nullptr);
    mCachedSettings.gpuPipelineOverride = mISettings->getAsInt(kSettingOverrideGPU);
    mSubscribedSettings.push_back(subID);
    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingSuppressReadback,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType, void*) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            OmniPhysX::getInstance().getCachedSettings().suppressReadback = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.suppressReadback = mISettings->getAsBool(kSettingSuppressReadback);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingReplicatorEnvIdsOnAttach,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType, void*) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            OmniPhysX::getInstance().getCachedSettings().replicatorEnvIdsOnAttach = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.replicatorEnvIdsOnAttach = mISettings->getAsBool(kSettingReplicatorEnvIdsOnAttach);
    mSubscribedSettings.push_back(subID);

    subID = mISettings->subscribeToNodeChangeEvents(
        kSettingResetOnStop,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void*) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            OmniPhysX::getInstance().getCachedSettings().resetOnStop = dict->getAsBool(changedItem);
        },
        nullptr);
    mCachedSettings.resetOnStop = mISettings->getAsBool(kSettingResetOnStop);
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
            omniPhysX.mCachedSettings.physxDefaultSimulator = dict->get<std::string>(changedItem) == "PhysX";
        },
        nullptr);
    const char* defaultSimulator = mISettings->getStringBuffer(kSettingDefaultSimulator);
    mCachedSettings.physxDefaultSimulator = defaultSimulator && std::strcmp(defaultSimulator, "PhysX") == 0;
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

std::shared_ptr<PhysXReplicator> OmniPhysX::getReplicator(AttachHandle attachHandle)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mReplicatorMutex);
    auto fit = mReplicatorMap.find(attachHandle);
    return fit != mReplicatorMap.end() ? fit->second : nullptr;
}

bool OmniPhysX::registerReplicator(AttachHandle attachHandle, const IReplicatorCallback& callback)
{
    AttachedStage* stage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!stage)
        return false;
    std::lock_guard<carb::tasking::MutexWrapper> lock(mReplicatorMutex);
    mReplicatorMap.insert_or_assign(attachHandle, std::make_shared<PhysXReplicator>(callback));
    return true;
}

void OmniPhysX::unregisterReplicator(AttachHandle attachHandle)
{
    ReplicatorMap::node_type retired;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mReplicatorMutex);
        retired = mReplicatorMap.extract(attachHandle);
        if (!retired.empty())
            retired.mapped()->cancel();
    }
}
