// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-4

#pragma once
#include <carb/tasking/TaskingUtils.h>
#include "AttachmentEvents.h"
#include "CachedSettings.h"

/**
 * @implements REQ-ATTACH-LIFETIME-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-4
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-1 AC-2
 */

#include <carb/dictionary/IDictionary.h>
#include <carb/tasking/ITasking.h>
#include <carb/settings/ISettings.h>
#include <carb/events/EventsUtils.h>
#include <carb/events/IEvents.h>
#include <carb/extras/Library.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>

#include "Setup.h"
#include "RuntimeExecution.h"
#include "PhysXReplicator.h"
#include "internal/InternalPhysXDatabase.h"
#include "PhysXStageUpdate.h"
#include "usdLoad/AttachedStage.h" // AttachedStageUsdHandle (AttachOvstageBackingStageHandle below)

#include <cstddef>
#include <map>
#include <mutex>
#include <vector>

namespace omni::physx::tensors { class SimulationBackend; }
namespace omni
{
namespace physx
{

class PhysXUsdPhysicsInterface;
class ContactReport;
class TriggerManager;
class PhysXCustomJointManager;
class PhysXCustomGeometryManager;
class PhysXPropertyQueryManager;
class RaycastManager;
struct PhysXRuntimeInterfaces;

using ProfileEventMap = std::map<SubscriptionId, std::pair<ProfileStatsNotificationFn, void*>>;

using ProfileStatsVector = std::vector<PhysicsProfileStats>;
using CrossThreadProfileMap = std::unordered_map<std::string, uint64_t>;

// OmniPhysX::mSimulationLayer's handle type: an opaque, pointer-sized owner of an SdfLayer
// reference. The real TfRefPtr<SdfLayer> lives inside that storage; the special members route
// through the installed op table (omni::physics::parse::simulationLayerHandleOps(), ADR-0027
// seam #3, RuntimeBridge.cpp) so this header and OvruntimePhysX name no pxr type.
//
// @implements REQ-BUILD-BRIDGE-001
// @covers AC-4
// Empty-is-normal contract (ADR-0027): a default-constructed handle holds no layer, which is the
// only state an ovstage / USD-free attach ever produces. With no op table installed (production)
// the handle is a zeroed POD; with the USD library's table installed it is refcount-correct over
// a real SdfLayerRefPtr.
class SimulationLayerHandle
{
public:
    SimulationLayerHandle() noexcept;
    SimulationLayerHandle(std::nullptr_t) noexcept;
    SimulationLayerHandle(const SimulationLayerHandle& other) noexcept;
    SimulationLayerHandle& operator=(const SimulationLayerHandle& other) noexcept;
    ~SimulationLayerHandle();

    explicit operator bool() const noexcept;

    // Non-owning peek at the raw SdfLayer*, for the IPhysicsDataWrite hand-off
    // (beginFrameWrite / prepareTransformWrite): this handle's own reference keeps the layer
    // alive for the duration of the call, and a USD sink takes its own real reference from it.
    void* rawLayer() const noexcept;

    // Adopts `layer` (a raw SdfLayer*), taking a reference through the op table's adoptRaw
    // hook. Only the stage-lifecycle seam produces such a pointer.
    static SimulationLayerHandle fromRawLayer(void* layer) noexcept;

    // Raw storage address; only the pxr-free bridge and the stage-lifecycle seam use it to
    // construct an SdfLayerRefPtr into the opaque storage (ADR-0027), never naming a pxr type.
    void* storage() noexcept
    {
        return mStorage;
    }
    const void* storage() const noexcept
    {
        return mStorage;
    }

private:
    alignas(void*) unsigned char mStorage[sizeof(void*)];
};
static_assert(sizeof(SimulationLayerHandle) == sizeof(void*),
              "SimulationLayerHandle must stay pointer-sized: TfRefPtr<SdfLayer> is one raw pointer");

// physXAttachOvstage's backingStage parameter type is exactly usdLoad/AttachedStage.h's
// AttachedStageUsdHandle -- the value is forwarded straight into UsdLoad::attachOvstage().
// Reusing the one alias (rather than declaring a layout-identical twin) avoids needing a
// conversion shim at the call site.
using AttachOvstageBackingStageHandle = ::omni::physx::usdparser::AttachedStageUsdHandle;

// OM-45822 caching these settings avoids slowing down performance of the runtime update loop (visible in OmniGym sim)


// shift by one if someone uses an Id from outside
static const ::physx::PxU32 kOmniPhysXTypeId = ::physx::PxConcreteType::eFIRST_USER_EXTENSION << 1;
static const int kTypeIdBatchSize = 8;

class OmniPhysX : public Allocateable
{
public:
    std::shared_ptr<tensors::SimulationBackend> tensorBackend;
    carb::tasking::RecursiveMutexWrapper outputCacheMutex;
    carb::tasking::MutexWrapper samplerIndexMutex;

    ///////////////////////////////////////////////////////////////////////////////////////
    void onStartup();

    void onShutdown();

    PhysXUsdPhysicsInterface& getUsdPhysicsInterface() { return *mUsdPhysicsInterface; }
    ///////////////////////////////////////////////////////////////////////////////////////

    static OmniPhysX& getInstance();

    static OmniPhysX* getInstanceCheck();

    static void createOmniPhysXInstance();

    static bool isStarted()
    {
        return mWasStarted;
    }
    static bool mWasStarted;

    ///////////////////////////////////////////////////////////////////////////////////////
    // Attach to a stage, main entry point
    AttachHandle physXAttach(long int stageId, bool loadPhysics, const PhysxAttachOptions& options = {});
    // Stageless attach for a consumer-provided ovstage source (ADR-0002 M2c-E).
    // Returns false when the lower attach fails. The backing stage/id are
    // classified before this method starts the session. `backingStage` (a Kit-
    // hosted USD stage ovstage optionally co-attaches) is always empty without USD:
    // there is no UsdUtilsStageCache to resolve one from, and
    // physxSimulationAttachOvstage (PhysX.cpp) always passes
    // `effectiveBackingStageId == 0` there.
    AttachHandle physXAttachOvstage(const void* ovstageAttachPayload,
                            uint64_t readOrdinal,
                            AttachOvstageBackingStageHandle backingStage,
                            uint64_t effectiveBackingStageId,
                            const PhysxAttachOptions& options,
                            bool startConfiguredRecording = true);
    // Pull + apply ovstage change deltas over an explicit ordinal range (ADR-0003 M3).
    bool physXUpdateFromOvStage(omni::physics::AttachHandle attach, uint64_t fromOrdinal, uint64_t toOrdinal);

private:
    // Shared session prologue for both attach paths (USD + ovstage): create the
    // internal database, reset sim callbacks, and configure the parse interface.
    // Ensuring the PxPhysics singleton is not this method's job: physXAttach() does it
    // itself, before calling in, since it is the only caller with a real stage to derive
    // tolerances from.

public:

    // Deatch from a stage, remove all physics
    bool physXDetach(AttachHandle attach);

    void resetSimulation(usdparser::AttachedStage& owner);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Check if IPhysxSimulation attach was used



    ///////////////////////////////////////////////////////////////////////////////////////
    const PhysXSetup& getPhysXSetup() const
    {
        return mPhysXSetup;
    }
    PhysXSetup& getPhysXSetup()
    {
        return mPhysXSetup;
    }

    PhysXRuntimeInterfaces& getRuntimeInterfaces();
    const PhysXRuntimeInterfaces& getRuntimeInterfaces() const;
    void releaseRuntimeInterfaces();

    bool isPhysxRuntimeStarted() const
    {
        return mPhysxRuntimeStarted;
    }

    void setPhysxRuntimeStarted(bool started)
    {
        mPhysxRuntimeStarted = started;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Create/release for PhysX and InternalPhysXDatabase
    void releasePhysXScenes();

    ///////////////////////////////////////////////////////////////////////////////////////
    carb::dictionary::IDictionary* getIDictionary() const
    {
        return mIDictionary;
    }

    carb::tasking::ITasking* getITasking() const
    {
        return mITasking;
    }

    carb::settings::ISettings* getISettings() const
    {
        return mISettings;
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

    void sendSimulationEvent(SimulationEvent type, AttachHandle attach = omni::physics::kNoAttach);

    template <typename... ValuesT>
    carb::events::IEventPtr createSimulationEventV2(SimulationEvent type, ValuesT... values)
    {
        using namespace carb::events;
        carb::events::IEventPtr event = carb::stealObject(
            mSimulationEventStreamV2->createEventPtr(static_cast<EventType>(type), kGlobalSenderId, values...));

        return event;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Return the current simulation layer, if set. Kit/USD-authoring-only (an anonymous
    // sublayer for scrubbing simulation-time overrides back out of the edited stage):
    // confirmed zero real ovstage/ovphysx callers -- InternalScene.cpp's own comment
    // already documents mSimulationLayer is always null under ovstage; not reachable from
    // ovphysx's own attach path to begin with. See SimulationLayerHandle's comment above --
    // with no op table installed this is dead storage, so a plain pass-through is enough.
    SimulationLayerHandle getSimulationLayer() const
    {
        return mSimulationLayer;
    }
    void setSimulationLayer(SimulationLayerHandle layer)
    {
        mSimulationLayer = layer;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // simulation overrides
    int32_t getGpuPipelineOverride() const
    {
        return mCachedSettings.gpuPipelineOverride;
    }
    void setGpuPipelineOverride(int val)
    {
        mCachedSettings.gpuPipelineOverride = val;
    }
    int32_t getSolverTypeOverride() const
    {
        return mCachedSettings.solverTypeOverride;
    }
    void setSolverTypeOverride(int val)
    {
        mCachedSettings.solverTypeOverride = val;
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
    mutable carb::tasking::MutexWrapper profileMutex;
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

    SubscriptionId addProfileStatsSubscription(ProfileStatsNotificationFn onEvent, void* userData);
    void removeProfileStatsSubscription(SubscriptionId subscriptionId);
    void fireProfileStatsSubscription();

    ///////////////////////////////////////////////////////////////////////////////////////
    // debug vis


    ///////////////////////////////////////////////////////////////////////////////////////
    // Simulation state



    // sets simulation started value and returns previous value


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

    SubscriptionId addOnStepEventSubscription(AttachHandle attach, std::function<void(float, void*)> onUpdate,
                                              void* userData, bool preStep, int order);
    void removeOnStepEventSubscription(SubscriptionId subscriptionId);
    void fireOnStepEventSubscriptions(AttachHandle attach, float timeStep, bool preStep);
    void fireStatusEventSubscriptions(AttachHandle attach, SimulationStatusEvent eventStatus) const;
    SubscriptionId addStatusEventSubscription(AttachHandle attach, std::function<void(SimulationStatusEvent, void*)> onEvent, void* userData);
    void removeStatusEventSubscription(SubscriptionId subscriptionId);
    void retireEventSubscriptions(usdparser::AttachedStage& attachedStage);

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
        std::lock_guard<carb::tasking::MutexWrapper> lock(mTypeIdMutex);
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
        std::lock_guard<carb::tasking::MutexWrapper> lock(mTypeIdMutex);
        mFreeTypeId.push_back(id);
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

    ///////////////////////////////////////////////////////////////////////////////////////
    // Replicator registrations require a concrete live attachment (ADR-0042).
    bool registerReplicator(AttachHandle attachHandle, const IReplicatorCallback& callback);
    void unregisterReplicator(AttachHandle attachHandle);
    // Registrations are resolved only by the exact attachment handle.
    std::shared_ptr<PhysXReplicator> getReplicator(AttachHandle attachHandle);

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

private:
    // methods
    void subscribeToSettingsChangeEvents();
    void unsubscribeFromSettingsChangeEvents();

    // interfaces
    carb::dictionary::IDictionary* mIDictionary{ nullptr };
    carb::tasking::ITasking* mITasking{ nullptr };
    carb::settings::ISettings* mISettings{ nullptr };

    // PhysX globals
    std::shared_ptr<PhysXUsdPhysicsInterface> mUsdPhysicsInterface;
    PhysXSetup mPhysXSetup;

    // event streams
    carb::events::IEventStreamPtr mSimulationEventStreamV2;
    carb::events::IEventStreamPtr mErrorEventStream;

    SimulationLayerHandle mSimulationLayer;

    // simulation overrides

    // profiling
    std::atomic<bool> mEnablePVDProfile{ false };
    std::atomic<bool> mEnableProfile{ true }; // do own top level profiling for now
    ProfileStatsVector mProfileStats;
    CrossThreadProfileMap mCrossProfileStats;
    ProfileEventMap mProfileStatsSubscriptions;

    // debug vis


    // Simulation state

    // velocity output state
    bool mOutputVelocitiesLocalSpace{ false };

    // Event subscriptions



    mutable carb::tasking::MutexWrapper mEventSubscriptionsMutex;


    // ISettings subscriptions to remove ourselves from when exiting
    OmniCachedSettings mCachedSettings;
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;

    TriggerManager* mTriggerManager{ nullptr };



    carb::extras::LibraryHandle mCudaHandle{ nullptr };

    // Stage update
    PhysXStageUpdate mStageUpdate;

    PhysXCustomJointManager* mCustomJointManager{ nullptr };
    PhysXCustomGeometryManager* mCustomGeometryManager{ nullptr };
    PhysXPropertyQueryManager* mPropertyQueryManager{ nullptr };

    // type id
    std::vector<::physx::PxU32> mTypeIds;
    carb::tasking::MutexWrapper mTypeIdMutex;
    std::vector<::physx::PxU32> mFreeTypeId;

    // replicator, keyed by attach handle
    ReplicatorMap mReplicatorMap;
    carb::tasking::MutexWrapper mReplicatorMutex;



    // Function-table copies for the static runtime accessors that replaced
    // Carbonite acquire/publication for the internal PhysX runtime.
    PhysXRuntimeInterfaces* mRuntimeInterfaces{ nullptr };
    bool mPhysxRuntimeStarted{ false };
};

} // namespace physx
} // namespace omni
