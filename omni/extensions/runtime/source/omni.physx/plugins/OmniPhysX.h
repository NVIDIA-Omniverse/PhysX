// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once


#include <carb/dictionary/IDictionary.h>
#include <carb/tasking/ITasking.h>
#include <carb/settings/ISettings.h>
#include <carb/events/EventsUtils.h>
#include <carb/events/IEvents.h>
#include <carb/extras/Library.h>

#include <omni/convexdecomposition/ConvexDecomposition.h>
#include <private/omni/physics/IUsdPhysicsParse.h>

#include <omni/fabric/IFabric.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>

#include "Setup.h"
#include "PhysXReplicator.h"
#include "internal/InternalPhysXDatabase.h"
#include "PhysXStageUpdate.h"
#include "PhysXStats.h"

#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{
class FabricBatchData;
}

class ContactReport;
class TriggerManager;
class FabricParticles;
class PhysXCustomJointManager;
class PhysXCustomGeometryManager;
class PhysXPropertyQueryManager;
class RaycastManager;

struct StepEventSub
{
    bool valid;
    omni::physx::OnPhysicsStepEventFn stepFn;
    void* userData;
};

template <typename OrderingType>
using StepEventSubscriptionRegistry = EventSubscriptionRegistry<StepEventSub, OrderingType>;
using SimulationEventSubscriptionRegistry =
    EventSubscriptionRegistry<std::pair<omni::physx::OnPhysicsSimulationEventFn, void*>>;

using ProfileStatsSubscriptionRegistry =
    EventSubscriptionRegistry<std::pair<omni::physx::ProfileStatsNotificationFn, void*>>;

using ProfileStatsVector = std::vector<PhysicsProfileStats>;
using CrossThreadProfileMap = std::unordered_map<std::string, uint64_t>;

// OM-45822 caching these settings avoids slowing down performance of the runtime update loop (visible in OmniGym sim)
struct OmniCachedSettings
{
    float viewportGizmoScale = 1.0f;
    int visualizationDisplayParticles = 0;
    bool updateToUsd = true;
    bool updateVelocitiesToUsd = true;
    bool outputVelocitiesLocalSpace = false;
    bool updateParticlesToUsd = true;
    bool updateResidualsToUsd = true;
    bool delayedInitLocalMeshCache = false;
    bool localMeshCacheEnabled = false;
    int localMeshCacheSize = 0;
    int minFrameRate = 30;
    float jointBodyTransformCheckTolerance = 0.001f;
    bool simulateEmptyScene = false;
    bool disableSleeping = false;
    bool enableSynchronousKernelLaunches = false;
    bool disableContactProcessing = false;
    bool disableResetOnStop = false;
    std::string defaultSimulator = "";
    bool approximateCones = false;
    bool approximateCylinders = false;
    bool fabricEnabled = false;
    bool enableExtendedJointAngles = false;
};

struct AddStepEvent
{
    omni::physx::SubscriptionId id;
    OnPhysicsStepEventFn onUpdate;
    void* userData;
    bool preStep;
    int order;
};

// shift by one if someone uses an Id from outside
static const ::physx::PxU32 kOmniPhysXTypeId = ::physx::PxConcreteType::eFIRST_USER_EXTENSION << 1;
static const int kTypeIdBatchSize = 8;

class OmniPhysX : public Allocateable
{
public:
    ///////////////////////////////////////////////////////////////////////////////////////
    // Extension startup
    void onStartup();

    // Extension shutdown
    void onShutdown();
    ///////////////////////////////////////////////////////////////////////////////////////

    // Get the global instance
    static OmniPhysX& getInstance();

    // Get the global instance check function
    static const OmniPhysX* getInstanceCheck();

    static void createOmniPhysXInstance();

    static const ::physx::PxCookingParams getStageDefaultCookingParams()
    {
        return OmniPhysX::getInstance().getPhysXSetup().getStageDefaultCookingParams(OmniPhysX::getInstance().getStage());
    }

    static bool isStarted()
    {
        return mWasStarted;
    }
    static bool mWasStarted;

    ///////////////////////////////////////////////////////////////////////////////////////
    // Attach to a stage, main entry point
    void physXAttach(long int stageId, bool loadPhysics);

    // Deatch from a stage, remove all physics
    void physXDetach();

    void resetSimulation();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Check if IPhysxSimulation attach was used
    bool isSimulationAttachedStage() const
    {
        return mSimulationAttachStage;
    }
    void setSimulationAttachedStage(bool val)
    {
        mSimulationAttachStage = val;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get PhysX Globals
    const PhysXSetup& getPhysXSetup() const
    {
        return mPhysXSetup;
    }
    PhysXSetup& getPhysXSetup()
    {
        return mPhysXSetup;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get Internal PhysX database, contains records for PhysX and internal pointers
    const internal::InternalPhysXDatabase& getInternalPhysXDatabase() const
    {
        return *mInternalPhysXDatabase;
    }
    internal::InternalPhysXDatabase& getInternalPhysXDatabase()
    {
        return *mInternalPhysXDatabase;
    }

    static const internal::InternalPhysXDatabase* getInternalPhysXDatabaseCheck()
    {
        return getInstanceCheck() ? getInstance().mInternalPhysXDatabase : nullptr;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    // Create/release for PhysX and InternalPhysXDatabase
    void releasePhysXScenes();
    void createInternalPhysXDatabase();
    void releaseInternalPhysXDatabase();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get dictionary interface
    carb::dictionary::IDictionary* getIDictionary() const
    {
        return mIDictionary;
    }

    // Get fabric interface
    omni::fabric::IStageReaderWriter* getIStageReaderWriter() const
    {
        return mIStageReaderWriter;
    }

    // Get tasking interface
    carb::tasking::ITasking* getITasking() const
    {
        return mITasking;
    }

    // Get settings interface
    carb::settings::ISettings* getISettings() const
    {
        return mISettings;
    }

    // Get convex decomposition interface
    omni::convexdecomposition::ConvexDecomposition* getIConvexDecomposition() const
    {
        return mIConvexDecomposition;
    }

    // Get usd physics parse interface
    omni::physics::usdparser::IUsdPhysicsParse* getIUsdPhysicsParse() const
    {
        return mIUsdPhysicsParse;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get EventStreams
    carb::events::IEventStreamPtr getSimulationEventStreamV2() const
    {
        return mSimulationEventStreamV2;
    }
    carb::events::IEventStreamPtr getErrorEventStream() const
    {
        return mErrorEventStream;
    }

    // Send simulation event
    void sendSimulationEvent(SimulationEvent type);

    template <typename... ValuesT>
    carb::events::IEventPtr createSimulationEventV2(SimulationEvent type, ValuesT... values)
    {
        using namespace carb::events;
        carb::events::IEventPtr event = carb::stealObject(
            mSimulationEventStreamV2->createEventPtr(static_cast<EventType>(type), kGlobalSenderId, values...));

        return event;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Return the current simulation layer, if set
    pxr::SdfLayerRefPtr getSimulationLayer() const
    {
        return mSimulationLayer;
    }
    // Set new simulation layer
    void setSimulationLayer(pxr::SdfLayerRefPtr layer)
    {
        mSimulationLayer = layer;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // simulation overrides
    int32_t getGpuPipelineOverride() const
    {
        return mGpuPipelineOverride;
    }
    void setGpuPipelineOverride(int val)
    {
        mGpuPipelineOverride = val;
    }
    int32_t getSolverTypeOverride() const
    {
        return mSolverTypeOverride;
    }
    void setSolverTypeOverride(int val)
    {
        mSolverTypeOverride = val;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // profiling
    bool isPVDProfileEnabled() const
    {
        return mEnablePVDProfile;
    }
    void setPvdProfileEnabled(bool val)
    {
        mEnablePVDProfile = val;
    }
    bool isOmniPhysXProfilingEnabled() const
    {
        return mEnableProfile;
    }
    void setOmniPhysXProfilingEnabled(bool val)
    {
        mEnableProfile = val;
    }
    const ProfileStatsVector& getOmniPhysXProfileStats() const
    {
        return mProfileStats;
    }
    ProfileStatsVector& getOmniPhysXProfileStats()
    {
        return mProfileStats;
    }

    CrossThreadProfileMap& getOmniPhysXCrossThreadProfileMap()
    {
        return mCrossProfileStats;
    }

    const CrossThreadProfileMap& getOmniPhysXCrossThreadProfileMap() const
    {
        return mCrossProfileStats;
    }

    SubscriptionId addProfileStatsSubscription(ProfileStatsNotificationFn onEvent, void* userData)
    {
        if (mEventSubscriptionLock)
        {
            CARB_LOG_ERROR("Subscription cannot be changed during the event call.");
            return kInvalidSubscriptionId;
        }
        else
        {
            return mProfileStatsSubscriptions.addEvent(std::make_pair(onEvent, userData));
        }
    }

    void removeProfileStatsSubscription(SubscriptionId subscriptionId)
    {
        if (mEventSubscriptionLock)
        {
            CARB_LOG_ERROR("Subscription cannot be changed during the event call.");
        }
        else
        {
            mProfileStatsSubscriptions.removeEvent(subscriptionId);
        }
    }

    void fireProfileStatsSubscription()
    {
        if (mEnableProfile)
        {
            mEventSubscriptionLock = true;
            if (!mProfileStatsSubscriptions.map.empty() && !mProfileStats.empty())
            {
                ProfileStatsSubscriptionRegistry::EventMap::const_iterator it = mProfileStatsSubscriptions.map.begin();
                ProfileStatsSubscriptionRegistry::EventMap::const_iterator itEnd = mProfileStatsSubscriptions.map.end();
                while (it != itEnd)
                {
                    it->second.first(mProfileStats, it->second.second);
                    it++;
                }
                mProfileStats.clear();
                mCrossProfileStats.clear();
            }
            mEventSubscriptionLock = false;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // debug vis
    bool isDebugVisualizationEnabled() const
    {
        return mEnableVisualization;
    }
    void setDebugVisualizationEnabled(bool val)
    {
        mEnableVisualization = val;
    }
    bool isNormalsVisualizationEnabled() const
    {
        return mEnableNormalsVisualization;
    }
    void setNormalsVisualizationEnabled(bool val)
    {
        mEnableNormalsVisualization = val;
    }
    float getVisualizationScale() const
    {
        return mVisualizationScale;
    }
    void setVisualizationScale(float val)
    {
        mVisualizationScale = val;
    }
    uint32_t getVisualizationBitMask() const
    {
        return mVisualizationBitMask;
    }
    void setVisualizationBitMask(uint32_t val)
    {
        mVisualizationBitMask = val;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Simulation state
    bool isSimulationRunning() const
    {
        return mRunning;
    }
    void setSimulationRunning(bool val)
    {
        mRunning = val;
    }
    bool hasSimulationStarted() const
    {
        bool retVal = false;
        {
            std::unique_lock<carb::tasking::MutexWrapper> simStartedLock(mSimParamMutex);
            retVal = mHasSimulationStarted;
        }
        return retVal;
    }
    // sets simulation started value and returns previous value
    bool setSimulationStarted(bool val)
    {
        bool wasSimulationStopped;
        {
            std::unique_lock<carb::tasking::MutexWrapper> simStartedLock(mSimParamMutex);
            wasSimulationStopped = !mHasSimulationStarted;
            mHasSimulationStarted = val;
        }
        return wasSimulationStopped;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Velocities output space
    bool isOutputVelocitiesLocalSpace() const
    {
        return mOutputVelocitiesLocalSpace;
    }
    void setOutputVelocitiesLocalSpace(bool val)
    {
        mOutputVelocitiesLocalSpace = val;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get event subscriptions

    // This function uses default arguments for legacy API uses: if not specified, it defaults to post-step not-ordered
    // events insertion (which was the default before)
    SubscriptionId addOnStepEventSubscription(OnPhysicsStepEventFn onUpdate,
                                              void* userData,
                                              bool preStep = false,
                                              int order = 0)
    {
        if (mEventSubscriptionLock)
        {
            auto& subscriptionsMap = preStep ? mPreStepSubscriptions : mPostStepSubscriptions;
            omni::physx::SubscriptionId id = subscriptionsMap.getNextId();
            mAddStepSubscriptions.push_back({ id, onUpdate, userData, preStep, order });
            return id;
        }
        else
        {
            auto& subscriptionsMap = preStep ? mPreStepSubscriptions : mPostStepSubscriptions;
            return subscriptionsMap.addEvent({ true, onUpdate, userData }, order);
        }
    }

    void removeOnStepEventSubscription(SubscriptionId subscriptionId)
    {
        if (mEventSubscriptionLock)
        {
            StepEventSubscriptionRegistry<OrderedRegistryTag>::EventMap::iterator fit =
                std::find_if(mPreStepSubscriptions.getMap().begin(), mPreStepSubscriptions.getMap().end(),
                             [&](const auto& pair) { return pair.second.first == subscriptionId; });

            if (fit != mPreStepSubscriptions.getMap().end())
            {
                fit->second.second.valid = false;
                mInvalidStepSubscriptions.push_back(subscriptionId);
            }
            else
            {
                fit = std::find_if(mPostStepSubscriptions.getMap().begin(), mPostStepSubscriptions.getMap().end(),
                                   [&](const auto& pair) { return pair.second.first == subscriptionId; });

                if (fit != mPostStepSubscriptions.getMap().end())
                {
                    fit->second.second.valid = false;
                    mInvalidStepSubscriptions.push_back(subscriptionId);
                }
            }
        }
        else
        {
            // These maps have a shared id generator so ids are unique, this allows us to remove events without having
            // to specify if it's in one map or another
            mPreStepSubscriptions.removeEvent(subscriptionId);
            mPostStepSubscriptions.removeEvent(subscriptionId);
        }
    }

    void fireOnStepEventSubscriptions(float timeStep, bool preStep)
    {
        mEventSubscriptionLock = true;
        const auto& subscriptionsMap = preStep ? mPreStepSubscriptions : mPostStepSubscriptions;
        for (const auto& [key, value] : subscriptionsMap.getMap())
        {
            const StepEventSub& ev = value.second;
            if (ev.stepFn && ev.valid)
                ev.stepFn(timeStep, ev.userData);
        }
        mEventSubscriptionLock = false;
        if (!mInvalidStepSubscriptions.empty())
        {
            for (const SubscriptionId id : mInvalidStepSubscriptions)
            {
                removeOnStepEventSubscription(id);
            }
            mInvalidStepSubscriptions.clear();
        }
        if (!mAddStepSubscriptions.empty())
        {
            for (const AddStepEvent& sub : mAddStepSubscriptions)
            {
                auto& subscriptionsMap = sub.preStep ? mPreStepSubscriptions : mPostStepSubscriptions;
                subscriptionsMap.getMap().insert({ sub.order, { sub.id, { true, sub.onUpdate, sub.userData } } });
            }
            mAddStepSubscriptions.clear();
        }
    }

    void fireStatusEventSubscriptions(SimulationStatusEvent eventStatus) const
    {
        mEventSubscriptionLock = true;
        if (!mSimulationSubscriptions.map.empty())
        {
            SimulationEventSubscriptionRegistry::EventMap::const_iterator it = mSimulationSubscriptions.map.begin();
            SimulationEventSubscriptionRegistry::EventMap::const_iterator itEnd = mSimulationSubscriptions.map.end();
            while (it != itEnd)
            {
                it->second.first(eventStatus, it->second.second);
                it++;
            }
        }
        mEventSubscriptionLock = false;
    }

    SubscriptionId addStatusEventSubscription(OnPhysicsSimulationEventFn onEvent, void* userData)
    {
        if (mEventSubscriptionLock)
        {
            CARB_LOG_ERROR("Subscription cannot be changed during the event call.");
            return kInvalidSubscriptionId;
        }
        else
        {
            return mSimulationSubscriptions.addEvent(std::make_pair(onEvent, userData));
        }
    }

    void removeStatusEventSubscription(SubscriptionId subscriptionId)
    {
        if (mEventSubscriptionLock)
        {
            CARB_LOG_ERROR("Subscription cannot be changed during the event call.");
        }
        else
        {
            mSimulationSubscriptions.removeEvent(subscriptionId);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Usd stage
    pxr::UsdStageWeakPtr getStage() const
    {
        return mStage; // Current stage
    }
    long getStageId() const
    {
        return mStageId;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Temp physics scene
    bool hasTempPhysicsScene() const
    {
        return mHasTempPhysicsScene;
    }
    void setHasTempPhysicsScene(bool val)
    {
        mHasTempPhysicsScene = val;
    }
    const pxr::SdfPath& getTempPhysicsScenePath() const
    {
        return mTempPhysicsScenePath;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // FabricBatchData
    const usdparser::FabricBatchData* getFabricBatchData() const
    {
        return mFabricBatchData;
    }
    usdparser::FabricBatchData* getFabricBatchData()
    {
        return mFabricBatchData;
    }
    void setFabricBatchData(usdparser::FabricBatchData* fc)
    {
        mFabricBatchData = fc;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Trigger report
    const TriggerManager* getTriggerManager() const
    {
        return mTriggerManager;
    }
    TriggerManager* getTriggerManager()
    {
        return mTriggerManager;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Raycast manager
    const RaycastManager& getRaycastManager() const
    {
        return *mRaycastManager;
    }
    RaycastManager& getRaycastManager()
    {
        return *mRaycastManager;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Custom joint manager
    const PhysXCustomJointManager& getCustomJointManager() const
    {
        return *mCustomJointManager;
    }
    PhysXCustomJointManager& getCustomJointManager()
    {
        return *mCustomJointManager;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Custom geometry manager
    const PhysXCustomGeometryManager& getCustomGeometryManager() const
    {
        return *mCustomGeometryManager;
    }
    PhysXCustomGeometryManager& getCustomGeometryManager()
    {
        return *mCustomGeometryManager;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    //
    const PhysXPropertyQueryManager& getPropertyQueryManager() const
    {
        return *mPropertyQueryManager;
    }
    PhysXPropertyQueryManager& getPropertyQueryManager()
    {
        return *mPropertyQueryManager;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Type id
    ::physx::PxU32 getFreeTypeId()
    {
        if (mFreeTypeId.empty())
        {
            for (int i = 0; i < kTypeIdBatchSize; i++)
            {
                const ::physx::PxU32 id = ::physx::PxU32(mTypeIds.size());
                mTypeIds.push_back(kOmniPhysXTypeId + id);
                mFreeTypeId.push_back(kOmniPhysXTypeId + id);
            }
        }
        ::physx::PxU32 typeId = mFreeTypeId.back();
        mFreeTypeId.pop_back();
        return typeId;
    }
    void pushBackFreeTypeId(::physx::PxU32 id)
    {
        mFreeTypeId.push_back(id);
    }

    uint64_t getSimulationTimestamp() const
    {
        return mSimulationTimestamp;
    }
    uint64_t getSimulationStepCount() const
    {
        return mSimulationTimestamp - mCurrentTimestampOffset;
    }
    void setCurrentTimestampOffset(uint64_t val)
    {
        mCurrentTimestampOffset = val;
    }
    void increateSimulationTimestamp()
    {
        mSimulationTimestamp++;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Get Settings Structure

    OmniCachedSettings& getCachedSettings()
    {
        return mCachedSettings;
    }

    const OmniCachedSettings& getCachedSettings() const
    {
        return mCachedSettings;
    }

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    FabricParticles* getFabricParticles() const
    {
        return mFabricParticles;
    }
#endif

    ///////////////////////////////////////////////////////////////////////////////////////
    // Replicator
    bool registerReplicator(uint64_t stageId, const IReplicatorCallback& callback);
    void unregisterReplicator(uint64_t stageId);
    PhysXReplicator* getReplicator(uint64_t stageId)
    {
        ReplicatorMap::iterator fit = mReplicatorMap.find(stageId);
        if (fit != mReplicatorMap.end())
        {
            return &fit->second;
        }
        return nullptr;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // CUDA
    bool isCudaLibPresent()
    {
        return mCudaHandle != nullptr;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // StageUpdate
    PhysXStageUpdate& getStageUpdate()
    {
        return mStageUpdate;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // PhysXStats
    bool isUploadPhysXStatsToCarbEnabled() const
    {
        return mUploadPhysXStatsToCarb;
    }
    void enableUploadPhysXStatsToCarb(bool val)
    {
        mUploadPhysXStatsToCarb = val;
    }
    PhysXStats* getPhysXStats() const
    {
        return mPhysXStats;
    }

private:
    // methods
    void subscribeToSettingsChangeEvents();
    void unsubscribeFromSettingsChangeEvents();

    // interfaces
    omni::fabric::IStageReaderWriter* mIStageReaderWriter{ nullptr };
    carb::dictionary::IDictionary* mIDictionary{ nullptr };
    carb::tasking::ITasking* mITasking{ nullptr };
    carb::settings::ISettings* mISettings{ nullptr };
    omni::convexdecomposition::ConvexDecomposition* mIConvexDecomposition{ nullptr };
    omni::physics::usdparser::IUsdPhysicsParse* mIUsdPhysicsParse{ nullptr };

    // PhysX globals
    PhysXSetup mPhysXSetup;
    internal::InternalPhysXDatabase* mInternalPhysXDatabase{ nullptr }; // database with PhysX <-> internal objects
                                                                        // records

    // event streams
    carb::events::IEventStreamPtr mSimulationEventStreamV2;
    carb::events::IEventStreamPtr mErrorEventStream;

    pxr::SdfLayerRefPtr mSimulationLayer{ nullptr };

    // simulation overrides
    int32_t mGpuPipelineOverride{ -1 }; // -1: use setting from schema, 0: force CPU, 1: force GPU
    int32_t mSolverTypeOverride{ -1 }; // -1: use setting from schema, 0: force PGS, 1: force TGS

    // profiling
    bool mEnablePVDProfile{ 0 };
    bool mEnableProfile{ true }; // do own top level profiling for now
    ProfileStatsVector mProfileStats;
    CrossThreadProfileMap mCrossProfileStats;
    ProfileStatsSubscriptionRegistry mProfileStatsSubscriptions;

    // debug vis
    bool mEnableVisualization{ false };
    bool mEnableNormalsVisualization{ false };
    float mVisualizationScale{ 1.0f };
    uint32_t mVisualizationBitMask{ 0 };

    // Simulation state
    mutable carb::tasking::MutexWrapper mSimParamMutex; // A.B. Do we really need this mutex?
    bool mHasSimulationStarted{ false };
    bool mRunning{ false };
    uint64_t mSimulationTimestamp{ 0 };
    uint64_t mCurrentTimestampOffset{ 0 };

    // velocity output state
    bool mOutputVelocitiesLocalSpace{ false };

    // Event subscriptions
    StepEventSubscriptionRegistry<OrderedRegistryTag> mPreStepSubscriptions;
    StepEventSubscriptionRegistry<OrderedRegistryTag> mPostStepSubscriptions;
    SimulationEventSubscriptionRegistry mSimulationSubscriptions;
    mutable bool mEventSubscriptionLock{ false };
    std::vector<SubscriptionId> mInvalidStepSubscriptions;
    std::vector<AddStepEvent> mAddStepSubscriptions;

    // Usd stage
    pxr::UsdStageRefPtr mStage{ nullptr }; // Current stage
    long mStageId{ 0u };
    bool mSimulationAttachStage{ false }; // whether IPhysxSimulation interface was used

    // Temp physics scene
    bool mHasTempPhysicsScene{ false };
    pxr::SdfPath mTempPhysicsScenePath{ "/PhysicsScene_16e12ee3daea" };

    // FabricBatchData
    usdparser::FabricBatchData* mFabricBatchData{ nullptr };

    // ISettings subscriptions to remove ourselves from when exiting
    OmniCachedSettings mCachedSettings;
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;

    TriggerManager* mTriggerManager{ nullptr };

    RaycastManager* mRaycastManager{ nullptr };

    carb::extras::LibraryHandle mCudaHandle{ nullptr };

    // Stage update
    PhysXStageUpdate mStageUpdate;

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    FabricParticles* mFabricParticles{ nullptr };
#endif
    PhysXCustomJointManager* mCustomJointManager{ nullptr };
    PhysXCustomGeometryManager* mCustomGeometryManager{ nullptr };
    PhysXPropertyQueryManager* mPropertyQueryManager{ nullptr };

    // type id
    std::vector<::physx::PxU32> mTypeIds;
    std::vector<::physx::PxU32> mFreeTypeId;

    // replicator
    ReplicatorMap mReplicatorMap;

    // statistics
    bool mUploadPhysXStatsToCarb{ false }; // whether or not to upload physX stats into carb::stats
    PhysXStats* mPhysXStats{ nullptr };
};

} // namespace physx
} // namespace omni
