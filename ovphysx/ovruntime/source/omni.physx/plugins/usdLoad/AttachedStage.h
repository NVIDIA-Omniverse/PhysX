// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-FEED-006
 * @covers AC-5
 */

// @implements REQ-PARSE-BATCH-001
// @covers AC-3

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

// @implements REQ-ATTACH-IO-001
// @covers AC-3

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-6 AC-7 AC-9 AC-10

/**
 * @implements REQ-ATTACH-BACKENDS-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

/**
 * @implements REQ-SIM-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-30
 *
 * @implements REQ-SIM-SCENEQUERY-001
 * @covers AC-3
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-4
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-1 AC-3 AC-6
 *
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-7
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-1 AC-6
 */

/**
 * @implements REQ-ATTACH-VIEWS-001
 * @covers AC-2
 */

// @implements REQ-NATIVE-INTERACTION-001
// @covers AC-2
#pragma once
#include <carb/tasking/TaskingUtils.h>
#include "RuntimeExecution.h"
#include "AttachmentEvents.h"

#include <stdexcept>
#include <omni/physics/AttachHandle.h>
#include <omni/physx/AttachOptions.h>
#include "../CachedSettings.h"
#include <cstddef>
#include "utils/Pair.h"
#include "Trigger.h"
#include "PhysXSimulationCallbacks.h"
#include <omni/physics/ovstage/OvstageParseBackend.h>
#include <omni/physics/parse/ScanBackend.h>

#include <private/omni/physx/PhysxUsd.h>
#include <usdInterface/UsdInterface.h>
#include "PrimUpdate.h" // PropertyChangeMap/KeyChangeMap/PrimUpdateMap
// ObjectDb (mObjectDatabase) plus the ObjectKey-keyed CollisionGroupsMap /
// DeformableAttachment*HistoryMap aliases used below.
#include "LoadTools.h"

#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IPhysicsSource.h> // IPhysicsSource + SourceUnits (cached by value)
#include <omni/physics/parse/KnownTokens.h> // KnownTokens (cached by value, see mKnownTokens)

#include <carb/Types.h>
#include <carb/logging/Log.h> // CARB_LOG_WARN in getEnvIdFromToken

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// AttachedStage holds the parse source trio only through the backend-neutral
// interfaces (ADR-0005) — it never names a concrete backend type. Consumers
// that need USD specifics down-cast the abstraction on demand (asUsdSource /
// asUsdDataWrite), which yields null under a non-USD backend.
namespace physx { class PxCpuDispatcher; class PxCudaContextManager; }
namespace omni::sampling { struct SurfaceSampler; }
namespace omni::physx::native { class Drag; }
namespace omni::physx { class CustomPhysXJoint; class CustomPhysXGeometryCallback; }
namespace omni::physx::particles
{
class PhysxParticleFactory;
class ParticlePostprocess;
struct PostprocessRef
{
    uint32_t refCount = 0;
    ParticlePostprocess* postprocess = nullptr;
};
using PostprocessRefMap = std::unordered_map<omni::physics::parse::ObjectKey, PostprocessRef,
    omni::physics::parse::ObjectKey::Hash>;
}
namespace omni { namespace physics { namespace parse { class DescriptorBatchAllocator; struct ScanOptions; class IPhysicsSource; class IPhysicsDataWrite; class IChangeFeed; struct AttachTarget; } } }

namespace omni
{
namespace physx
{
void retireCollisionRepresentationTasks(omni::physics::AttachHandle attach);
class PhysXScene;
class DebugVisualizationCache;
class RaycastManager;
using FilteredPairsSet = std::unordered_map<Pair<uint32_t>, uint32_t, PairHash>;
using CollisionGroupsPairsSet = std::unordered_set<Pair<uint32_t>, PairHash>;

namespace usdparser
{
// Compound (prim, attribute) key for mTimeSampledAttributes; the backend-agnostic
// equivalent of a USD SdfPath property path.
struct TimeSampledPropertyKey
{
    omni::physics::parse::ObjectKey primKey;
    omni::physics::parse::TokenId attr;
    bool operator==(const TimeSampledPropertyKey& o) const
    {
        return primKey == o.primKey && attr == o.attr;
    }
    struct Hash
    {
        size_t operator()(const TimeSampledPropertyKey& k) const
        {
            return omni::physics::parse::ObjectKey::Hash{}(k.primKey) ^
                   (omni::physics::parse::TokenId::Hash{}(k.attr) << 1);
        }
    };
};
using TimeSampleMap = std::unordered_map<TimeSampledPropertyKey, OnUpdateObjectFn, TimeSampledPropertyKey::Hash>;

// Tracks animated kinematic bodies by their ObjectKey.
using AnimatedKinematicBodySet = std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;
using TokenIdEnvIdMap =
    std::unordered_map<omni::physics::parse::TokenId, uint32_t, omni::physics::parse::TokenId::Hash>;

struct GeneratedDeformableAttachmentData
{
    enum class Kind
    {
        eVtxTet,
        eVtxXform,
    };

    Kind kind = Kind::eVtxTet;
    bool enabled = false;
    std::vector<int32_t> vtxIndicesSrc0;
    std::vector<int32_t> tetIndicesSrc1;
    std::vector<carb::Float3> tetCoordsSrc1;
    std::vector<carb::Float3> localPositionsSrc1;
};

struct GeneratedDeformableCollisionFilterData
{
    bool enabled = false;
    std::vector<uint32_t> groupElemCounts0;
    std::vector<uint32_t> groupElemIndices0;
    std::vector<uint32_t> groupElemCounts1;
    std::vector<uint32_t> groupElemIndices1;
};

using GeneratedDeformableAttachmentDataMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                                                GeneratedDeformableAttachmentData,
                                                                omni::physics::parse::ObjectKey::Hash>;
using GeneratedDeformableCollisionFilterDataMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                                                     GeneratedDeformableCollisionFilterData,
                                                                     omni::physics::parse::ObjectKey::Hash>;

// In-memory stand-in for one Attachment / ElementCollisionFilter sub-prim of a
// PhysxAutoDeformableAttachmentAPI prim. Used when the attach cannot author sub-prims
// (no live USD stage, i.e. ovstage): the child never exists in the source, its key is
// minted with keyFor("<autoAttachment>/<childName>"), and its payload lives in the
// generated-data maps above.
struct GeneratedAutoAttachmentChild
{
    omni::physics::parse::ObjectKey key;
    omni::physics::parse::ObjectType type = omni::physics::parse::ObjectType::eUndefined; // eAttachment* or eDeformableCollisionFilter
    omni::physics::parse::ObjectKey src0;
    omni::physics::parse::ObjectKey src1;
};

struct GeneratedAutoAttachmentLayout
{
    std::vector<GeneratedAutoAttachmentChild> children;
    // Input CRC of the last generated payload; replaces the physxAutoDeformableAttachment:inputCrc
    // attribute the USD arm stores on the prim.
    std::vector<uint8_t> inputCrc;
};

using GeneratedAutoAttachmentLayoutMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                                            GeneratedAutoAttachmentLayout,
                                                            omni::physics::parse::ObjectKey::Hash>;

// One array attribute produced by the auto-deformable cook (ADR-0022).
//
// `bytes` is the packed element payload in exactly the layout
// IPhysicsSource::resolveBuffer hands out, so a reader can wrap it in a
// synthetic BufferHandle and reuse the existing fillArray dispatch rather than
// growing a second type ladder. Scalars are carried as one-element arrays and
// the cooking CRC markers as eUInt8 blobs, which is why this is bytes + type +
// count rather than a VtArray variant.
struct CookedArray
{
    std::vector<uint8_t> bytes;
    omni::physics::parse::BufferElemType type = omni::physics::parse::BufferElemType::eFloat;
    uint32_t elemCount = 0;
    // Distinguishes "the write-back deliberately recorded this as empty" (e.g. the
    // velocities reset) from "nothing was ever recorded here" -- both look like an
    // empty `bytes` vector, but only the former may be served as an explicit empty
    // result; the latter must fall through to source. See getArrayValue (PhysXTools.h).
    bool recorded = false;
};

using CookedArrayMap =
    std::unordered_map<omni::physics::parse::TokenId, CookedArray, omni::physics::parse::TokenId::Hash>;
using CookedGeometryMap =
    std::unordered_map<omni::physics::parse::ObjectKey, CookedArrayMap, omni::physics::parse::ObjectKey::Hash>;

enum class ChangeSource
{
    eUsd,
    eUnknwon,
};

class AttachedStage;
class ChangeSourceBlock
{
public:
    ChangeSourceBlock(AttachedStage& attachedStage, ChangeSource source);
    ~ChangeSourceBlock();

    ChangeSourceBlock(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock& operator=(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock(ChangeSourceBlock&&) = default;
    ChangeSourceBlock& operator=(ChangeSourceBlock&&) = default;

private:
    AttachedStage& mAttachedStage;
    ChangeSource mPrevSource;
};

// AttachedStage::mStage's handle type: an opaque, fixed-layout owner of a UsdStageWeakPtr.
// The special members route through the installed op table
// (omni::physics::parse::attachedStageUsdHandleOps(), ADR-0027 seam #3, AttachedStageBridge.cpp)
// so this header names no pxr type. UsdStageWeakPtr (TfWeakPtr<UsdStage>) is not
// pointer-sized -- raw pointer plus a TfRefPtr<Tf_Remnant>, 16 bytes -- hence the two-pointer
// storage. USD-linked callers reinterpret storage() as the UsdStageWeakPtr it is.
class AttachedStageUsdHandle
{
public:
    AttachedStageUsdHandle() noexcept;
    AttachedStageUsdHandle(std::nullptr_t) noexcept;
    AttachedStageUsdHandle(const AttachedStageUsdHandle& other) noexcept;
    AttachedStageUsdHandle& operator=(const AttachedStageUsdHandle& other) noexcept;
    ~AttachedStageUsdHandle();

    explicit operator bool() const noexcept;

    // Raw storage address; only the USD bridge's conversion helpers and IParseBackend's
    // `nativeStage` payload may read it, as the UsdStageWeakPtr* it is.
    void* storage() noexcept
    {
        return mStorage;
    }
    const void* storage() const noexcept
    {
        return mStorage;
    }

private:
    alignas(void*) unsigned char mStorage[2 * sizeof(void*)];
};
static_assert(sizeof(AttachedStageUsdHandle) == 2 * sizeof(void*) &&
                  alignof(AttachedStageUsdHandle) == alignof(void*),
              "AttachedStageUsdHandle must stay a two-pointer-sized POD matching UsdStageWeakPtr");

class AttachedStage
{
public:
    RuntimeExecutionState execution;
    // Access under this attachment's execution admission or destruction callback.
    std::unordered_map<uint64_t, std::weak_ptr<native::Drag>> interactionDrags;
    mutable std::unordered_map<omni::physics::parse::ObjectKey, CustomPhysXJoint*, omni::physics::parse::ObjectKey::Hash> customJoints;
    mutable std::unordered_multimap<omni::physics::parse::ObjectKey, CustomPhysXGeometryCallback*, omni::physics::parse::ObjectKey::Hash> customGeometries;
    std::shared_ptr<::physx::PxCpuDispatcher> cpuDispatcher;
    std::vector<std::shared_ptr<::physx::PxCudaContextManager>> cudaContextManagers;
    size_t nextCudaContext = 0;
    std::unordered_map<uint64_t, std::shared_ptr<omni::sampling::SurfaceSampler>> surfaceSamplers;
    mutable std::unordered_map<omni::physics::parse::ObjectKey, particles::PhysxParticleFactory*,
        omni::physics::parse::ObjectKey::Hash> particleFactories;
    mutable particles::PostprocessRefMap particlePostprocess;
    mutable carb::tasking::MutexWrapper eventMutex;
    StepEventMap preStepSubscriptions;
    StepEventMap postStepSubscriptions;
    SimulationEventMap simulationSubscriptions;
    mutable std::atomic<uint64_t> recordVersion{1};
    mutable std::atomic<uint64_t> kinematicVersion{1};

    using WeakLifetime = std::weak_ptr<AttachedStage*>;
    WeakLifetime getWeakLifetime() const { return mLifetime; }
    static AttachedStage* resolveLifetime(const WeakLifetime& lifetime)
    {
        const std::shared_ptr<AttachedStage*> state = lifetime.lock();
        return state ? *state : nullptr;
    }

    const std::weak_ptr<bool>& getCookingObserverDriver() const { return mCookingObserverDriver; }
    void setCookingObserverDriver(const std::shared_ptr<bool>& driver) { mCookingObserverDriver = driver; }

    void setCookingRetirement(std::function<void()> retire) const { mRetireCooking = std::move(retire); }
    void retireCooking() const
    {
        omni::physx::retireCollisionRepresentationTasks(getAttachHandle());
        std::function<void()> retire = std::move(mRetireCooking);
        if (retire)
            retire();
    }

    struct NoticeBlockState
    {
        std::atomic_int32_t source{ 0 };
        std::atomic_int32_t cooking{ 0 };
    };
    const std::shared_ptr<NoticeBlockState>& getNoticeBlockState() const { return mNoticeBlockState; }
    bool sourceUpdatesBlocked() const { return mNoticeBlockState->source.load() != 0; }
    bool cookingUpdatesBlocked() const { return mNoticeBlockState->cooking.load() != 0; }

    using MeshKeyCache = std::unordered_map<omni::physics::parse::ObjectKey, MeshKey,
        omni::physics::parse::ObjectKey::Hash>;
    struct CookingRefreshState
    {
        using KeySet = std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;
        KeySet prims;
        KeySet transforms;
        KeySet schemas;
        KeySet parents;
        std::unordered_set<omni::physics::parse::TokenId, omni::physics::parse::TokenId::Hash> collisionTokens;
        omni::physics::parse::KnownTokens tokens;
        bool tokensInitialized = false;
    };
    MeshKeyCache& getMeshKeyCache() const { return mMeshKeyCache; }
    CookingRefreshState& getCookingRefreshState() const { return mCookingRefreshState; }
    // Observers read this count without traversing another owner's mutable sets.
    mutable std::atomic<uint32_t> pendingCookingRefreshCount{0};
    uint64_t profileUpdateStart = 0;

    AttachedStage();
    AttachedStage(AttachedStageUsdHandle stage, PhysXUsdPhysicsInterface* iface);
    ~AttachedStage();

    // Reuse descriptor blocks across scans; each returned scan retains the pool.
    omni::physics::parse::ScanOptions descriptorScanOptions();

    PhysXUsdPhysicsInterface* getPhysXPhysicsInterface()
    {
        return mPhysicsInterface;
    }

    const PhysXUsdPhysicsInterface* getPhysXPhysicsInterface() const
    {
        return mPhysicsInterface;
    }

    // A USD caller turns the handle into a real UsdStageWeakPtr with usdStageOf().
    AttachedStageUsdHandle getStage()
    {
        return mStage;
    }

    AttachedStageUsdHandle getStage() const
    {
        return mStage;
    }

    // The UsdUtilsStageCache id of the USD stage this attach reads (USD backend)
    // or mirrors (an ovstage attach with a resident backing stage); 0 when there
    // is no backing USD stage. This is also the key the attach is registered
    // under in UsdLoad.
    //
    // The id is owned by the parse source, not by this class: it is a property
    // of what the source reads, the source already has to know it, and keeping a
    // second copy here would give it a second lifetime to drift out of. Reading
    // it through the backend-neutral IPhysicsSource means neither the USD nor the
    // ovstage spelling leaks into this consumer. The source snapshots the id when
    // it is built — the same cadence this member was recomputed at — so the value
    // is stable for as long as the source lives.
    long getStageId() const
    {
        return mSource ? static_cast<long>(mSource->residentUsdStageId()) : 0;
    }

    // True when this attach is driven by a consumer-provided external source
    // (ovstage) rather than by a USD stage. The distinction is not "has no
    // stage": an external attach may still carry a resident backing stage, and
    // callers that branch on the *source kind* — serial vs parallel collection
    // inversion, source-only side-effect gathering, particle-sampling reads —
    // must not infer it from stage identity.
    bool hasExternalSource() const
    {
        return mExternalAttachPayload != nullptr;
    }

    // Physical ovstage identity, independent of dictionary keys or backing USD metadata.
    const void* getOvstageInstance() const
    {
        return hasExternalSource() ? mOwnedAttachPayload.instance : nullptr;
    }

    // Identity of this *attach*, minted by UsdLoad and unique for the process
    // lifetime. Distinct from getStageId(): a stageless attach has stage id 0 but
    // still gets a nonzero handle, and reattaching the same USD stage yields the
    // same stage id but a new handle. This is what identifies an attach to
    // consumers that must not silently outlive it (ADR-0013).
    uint64_t getAttachHandle() const
    {
        return mAttachHandle;
    }

    void setAttachHandle(uint64_t handle)
    {
        mAttachHandle = handle;
    }


    struct ObjectNotificationState
    {
        bool enabled = false;
        bool initialPopulation = false;
    };
    ObjectNotificationState& objectNotificationState() { return mObjectNotificationState; }
    const ObjectNotificationState& objectNotificationState() const { return mObjectNotificationState; }
    // The handle comes from the reparse seam (IUsdReparse::resolveStageToHandle).
    void setStage(AttachedStageUsdHandle stage)
    {
        mStage = stage;
        // Returning to the USD path drops every trace of a previous external
        // attach, including its backing-stage input: the stage id now comes from
        // the UsdSource rebuildSource() is about to build.
        mExternalAttachPayload = nullptr;
        mExternalReadOrdinal = 0;
        mExternalBackingStageId = 0;
        rebuildSource();
    }

    // Copy the ovstage descriptor into stable attachment-owned storage and build
    // private parse/scan backends. The caller retains the referenced instance and
    // dictionary through detach; the descriptor itself may be released after this call.
    // `backingStage` is optional developer-adapter metadata, not attach identity.
    //
    // `backingStageId` is the caller's already-classified stage-cache id for
    // `backingStage`. It is an *input* to the attach rather than a redundant copy
    // of getStageId(): the external backend needs the id to build its source, and
    // at that moment the source — the thing that owns the id afterwards — does not
    // exist yet. The caller classifies residency once, before any session state is
    // mutated, so that a nonresident or out-of-range candidate reported by another
    // USD runtime is rejected up front rather than turning into a local stage
    // lookup mid-attach. Must be the id of `backingStage` (0 for an empty handle);
    // after the attach, getStageId() reads it back through the source.
    void setOvstageSource(const void* attachPayload,
                          AttachedStageUsdHandle backingStage = AttachedStageUsdHandle{},
                          uint64_t readOrdinal = 1,
                          uint64_t backingStageId = 0);

    // The backend-opaque attach handle for the current source: the live USD
    // stage for a USD attach, or the consumer-provided payload otherwise. Fed to
    // both the parse backend (createSource) and the scan dispatch
    // (scanStage) -- the per-attachment dispatch lives here, not
    // at call sites. Valid while this AttachedStage lives.
    omni::physics::parse::AttachTarget attachTarget() const;

    // Dense object IDs are local to this attachment's owned record database.
    internal::InternalPhysXDatabase& getInternalPhysXDatabase() const;
    void releaseInternalPhysXDatabase();
    void releaseNativePhysics();

    FilteredPairsSet& getFilteredPairs() const
    {
        return mFilteredPairs;
    }
    FilteredPairsSet& getFilteredPairs()
    {
        return mFilteredPairs;
    }
    void addFilteredPair(const Pair<uint32_t>& pair)
    {
        FilteredPairsSet::iterator fit = mFilteredPairs.find(pair);
        if (fit != mFilteredPairs.end())
        {
            fit->second++;
        }
        else
        {
            mFilteredPairs[pair] = 1;
        }
    }
    void removeFilteredPair(const Pair<uint32_t>& pair)
    {
        FilteredPairsSet::iterator fit = mFilteredPairs.find(pair);
        if (fit != mFilteredPairs.end())
        {
            if (fit->second <= 1)
            {
                mFilteredPairs.erase(fit);
            }
            else
            {
                fit->second--;
            }
        }
    }
    CollisionGroupsPairsSet& getCollisionGroupFilteredPairs() const
    {
        return mCollisionsGroupFilteredPairs;
    }
    CollisionGroupsPairsSet& getCollisionGroupFilteredPairs()
    {
        return mCollisionsGroupFilteredPairs;
    }
    struct ParseOptions
    {
        bool exposePrimNames = true;
        std::string forceSingleScenePath;
        ObjectId forcedSceneId = kInvalidObjectId;
    };
    ParseOptions& getParseOptions() const { return mParseOptions; }
    void setAttachOptions(const PhysxAttachOptions& options);
    bool isDebugVisualizationEnabled() const
    {
        return mEnableVisualization;
    }
    void setDebugVisualizationEnabled(bool val)
    {
        mEnableVisualization = val;
        mDebugVisualizationDirty = true;
    }
    bool isNormalsVisualizationEnabled() const
    {
        return mEnableNormalsVisualization;
    }
    bool isDebugVisualizationDirty() const
    {
        return mEnableVisualization && mDebugVisualizationDirty;
    }
    void setDebugVisualizationDirty(bool val)
    {
        mDebugVisualizationDirty = val;
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
        mDebugVisualizationDirty = true;
    }
    uint64_t getVisualizationBitMask() const
    {
        return mVisualizationBitMask;
    }
    void setVisualizationBitMask(uint64_t val)
    {
        mVisualizationBitMask = val;
    }

    DebugVisualizationCache& getDebugVisualizationCache() const;

    const OmniCachedSettings& getCachedSettings() const { return mCachedSettings; }
    void setContactProcessingDisabled(bool disabled) { mCachedSettings.disableContactProcessing = disabled; }
    const PhysxSharedSettings& getSharedSettings() const { return mSharedSettings; }
    const std::vector<std::string>& getExcludedPaths() const { return mExcludedPaths; }

    bool isSimulationRunning() const { return mSimulationRunning; }
    void setSimulationRunning(bool running) { mSimulationRunning = running; }
    bool hasSimulationStarted() const { return mSimulationStarted; }
    void setSimulationStarted(bool started) { mSimulationStarted = started; }
    void recordSimulationStep() { ++mSimulationSteps; }
    void recordManualStep() { ++mManualSteps; }
    uint64_t simulationTimestamp() const { return mSimulationSteps.load() + mManualSteps.load(); }
    uint64_t simulationStepCount() const { return simulationTimestamp() - mSimulationStepOffset; }
    void resetSimulationStepCount() { mSimulationStepOffset = simulationTimestamp(); }

    void setAsyncUSDUpdate(bool value) { mAsyncUpdate.store(value); }
    bool getAsyncUSDUpdate() const { return mAsyncUpdate.load(); }

    SimulationCallbacks& getSimulationCallbacks() const { return mSimulationCallbacks; }
    RaycastManager& getRaycastManager() const;

    AttachmentTriggerState& getTriggerState() const { return mTriggerState; }

    bool hasTempPhysicsScene() const { return mTempPhysicsSceneKey.valid(); }
    omni::physics::parse::ObjectKey getTempPhysicsSceneKey() const { return mTempPhysicsSceneKey; }
    void setTempPhysicsSceneKey(omni::physics::parse::ObjectKey key) { mTempPhysicsSceneKey = key; }

    struct PendingPhysicsSetup
    {
        bool dirty = true;
        std::vector<ObjectId> articulations;
        std::vector<internal::InternalPbdParticleSystem*> particleSystems;
    };
    PendingPhysicsSetup& getPendingPhysicsSetup() const { return mPendingPhysicsSetup; }

    using SceneMap = std::unordered_map<size_t, PhysXScene*>;
    const SceneMap& getPhysXScenes() const { return mScenes; }
    PhysXScene* getPhysXScene(size_t id) const
    {
        if (id == 0 || id == kInvalidObjectId)
            return mDefaultPhysXScene;
        const auto it = mScenes.find(id);
        return it == mScenes.end() ? nullptr : it->second;
    }
    void registerPhysXScene(size_t id, PhysXScene* scene)
    {
        if (!mScenes.emplace(id, scene).second)
            throw std::logic_error("duplicate scene ID in attachment");
        if (!mDefaultPhysXScene)
            mDefaultPhysXScene = scene;
    }
    void unregisterPhysXScene(PhysXScene* scene)
    {
        for (auto it = mScenes.begin(); it != mScenes.end(); ++it)
        {
            if (it->second == scene)
            {
                mScenes.erase(it);
                if (mDefaultPhysXScene == scene)
                    mDefaultPhysXScene = mScenes.empty() ? nullptr : mScenes.begin()->second;
                return;
            }
        }
    }
    PhysXScene* getDefaultPhysXScene() const { return mDefaultPhysXScene; }
    void setDefaultPhysXScene(PhysXScene* scene) { mDefaultPhysXScene = scene; }


    omni::physics::parse::IScanBackend* getScanBackend() const { return mScanBackend.get(); }

    omni::physics::parse::ScannedStage scanStage(
        const std::vector<std::string>& roots, const std::vector<std::string>& excludes,
        const omni::physics::parse::ScanOptions& options,
        omni::physics::parse::IDescriptorAllocator& allocator) const;


    // ------------------------------------------------------------------
    // Path / ObjectKey resolution. AttachedStage owns a UsdSource keyed
    // off the current stage. Callers consuming parse-side descriptors
    // resolve their ObjectKey fields back to SdfPath here when they
    // genuinely need a USD path (USD authoring, change
    // notice handlers, ObjectDb lookups). This is the boundary between
    // the source-agnostic parse library and the USD-coupled runtime.
    // ------------------------------------------------------------------

    // Backend-neutral read access to the attached source — the only source
    // accessor. It exposes the source-agnostic IPhysicsSource contract, so code
    // routed through it does not depend on the backend being USD. Code that
    // genuinely needs USD specifics down-casts on demand via
    // omni::physics::usd::asUsdSource(getSource()) (null under a non-USD
    // backend); getStage() remains the USD-stage escape hatch.
    omni::physics::parse::IPhysicsSource* getSource();
    const omni::physics::parse::IPhysicsSource* getSource() const;

    // Stage units (metersPerUnit / kilogramsPerUnit / upAxis) for this attach,
    // read once from the source in rebuildSource() and cached. This is the single
    // omni.physx-side units accessor — code must read units through here (or the
    // source) rather than reaching to a UsdStage directly, so it works under both
    // the USD and ovstage backends. Returns defaults (1.0 / 1.0 / Z-up) when no
    // source is bound.
    const omni::physics::parse::SourceUnits& getSourceUnits() const
    {
        return mUnits;
    }

    // Well-known token batch for this attach's source, bound exactly once per
    // rebuildSource() (mirrors getSourceUnits() above). Attach-scoped hot paths
    // read tokens from here instead of calling KnownTokens::intern() themselves --
    // intern() has no memoization and unconditionally re-interns its whole
    // vocabulary on every call (REQ-SIM-SCENEQUERY-001). When the source caches its
    // own batch (IPhysicsSource::knownTokens()) this IS that batch, not a copy
    // (REQ-LOAD-TOKENS-001 AC-5).
    const omni::physics::parse::KnownTokens& getKnownTokens() const
    {
        return mKnownTokensRef ? *mKnownTokensRef : mKnownTokens;
    }

    // Source-agnostic sink for physics simulation output (poses/velocities/
    // arrays written back out). Built by the active parse backend in
    // rebuildSource(), so it shares the stage's lifetime. Consumers needing the
    // USD-specific sink API down-cast via asUsdDataWrite() on demand.
    omni::physics::parse::IPhysicsDataWrite* getDataWrite()
    {
        return mDataWrite.get();
    }

    const omni::physics::parse::IPhysicsDataWrite* getDataWrite() const
    {
        return mDataWrite.get();
    }

    // Sink for scene-description side effects that must reach a resident backing USD stage
    // even when the active source wires no write sink of its own (ovstage, by design).
    // Null when neither exists.
    //
    // NOT a general substitute for getDataWrite(): the per-frame simulation write-back stays
    // gated on that one, because an ovstage consumer owns its own output authoring.
    omni::physics::parse::IPhysicsDataWrite* getAuthoringDataWrite()
    {
        return mDataWrite ? mDataWrite.get() : mBackingStageDataWrite.get();
    }

    // Author the fallback default-PhysicsScene placeholder through the active write sink,
    // or straight into the resident backing stage's session layer when there is no sink.
    // Returns false, with nothing authored, when neither exists.
    bool createDefaultPhysicsScenePlaceholder(omni::physics::parse::ObjectKey sceneKey);

    // Remove a placeholder authored by createDefaultPhysicsScenePlaceholder(), mirroring
    // its two paths. No-op when neither exists.
    void removeDefaultPhysicsScenePlaceholder(omni::physics::parse::ObjectKey sceneKey);

    // Source-vended push feed of runtime change deltas (ADR-0003). Shares the
    // stage's lifetime (rebuilt with the source); consumers register interests on
    // it. Returns null when no source is attached.
    omni::physics::parse::IChangeFeed* getChangeFeed()
    {
        return mChangeFeed.get();
    }

    // Pull the change delta for an explicit producer-supplied version range and
    // apply it (the ovstage path; engine entry IPhysxSimulation::updateFromOvStage).
    // Delegates to the feed's drainRange, which fires the registered onSourceChange
    // / onSourceGroupComplete callbacks. Returns false if there is no feed or the
    // unread suffix could not be served. A range whose end is at or below the
    // consumed-ordinal cursor returns true as a no-op.
    bool updateFromOvStage(uint64_t fromOrdinal, uint64_t toOrdinal)
    {
        if (!mChangeFeed)
            return false;

        // Attach already parsed every change through mExternalReadOrdinal. Treat
        // an entirely consumed range as a successful no-op, and trim an
        // overlapping range so the feed sees only changes newer than the current
        // cursor. This prevents a redundant replay of the attach ordinal from
        // recreating the initial population while preserving notifications for
        // population authored after attach.
        if (toOrdinal <= mExternalReadOrdinal)
            return true;

        const uint64_t unreadFromOrdinal = std::max(fromOrdinal, mExternalReadOrdinal + 1);
        const bool ok = mChangeFeed->drainRange(unreadFromOrdinal, toOrdinal);
        if (ok)
        {
            // Advance the read cursor only after a successful drain: a failed
            // range must not leave state claiming those ordinals were consumed.
            mExternalReadOrdinal = toOrdinal;
            flushBufferedChanges(*this, 0.0f);
        }
        // A false holds the cursor uniformly, whichever kind drainRange meant: a partial-commit HOLD
        // (correct -- retry the same range next drain) or an UNSERVABLE range (from below retained history,
        // or a read failure). Those want OPPOSITE recoveries -- retry vs re-attach -- but drainRange returns
        // one bool (see its doc), so an unservable range stalls here until a re-attach is triggered out of
        // band. Distinguishing them in the result is the deferred re-attach work (ADR-0003 / ADR-0006 OQs).
        return ok;
    }

    // Resolution is string-typed (keyFor / textFor / textViewFor); there is no pxr-typed
    // accessor, which is what lets this header name no pxr type at all.

    // Existence-independent interning: mints a key for any syntactically valid path, even
    // one naming no live object (see TestObjectKeyMinting.cpp). Needed for SYNTHETIC
    // identities never authored into the backing IPhysicsSource, e.g. PhysXReplicator's
    // runtime clone targets.
    omni::physics::parse::ObjectKey keyFor(std::string_view path) const;

    // Convenience for diagnostics: the path string of a key without building
    // an SdfPath. Returns "" (never null) when the key/source is invalid, so
    // it is safe to feed straight into %s log formatting.
    const char* textFor(omni::physics::parse::ObjectKey key) const;

    // string_view sibling of textFor(), for callers that need the length
    // alongside the text (e.g. building an omni::span<const char>) -- avoids
    // a second lookup plus a strlen() re-derivation of a length the source's
    // interned std::string storage already carries.
    std::string_view textViewFor(omni::physics::parse::ObjectKey key) const;

    // Is `ancestor` equal to, or a namespace ancestor of, `node`? Walks the
    // source's parent chain rather than converting to SdfPath and calling
    // HasPrefix (ADR-0019 decision 2). Mirrors
    // omni.physics.parse/ArticulationGraph.cpp's isAncestorOrSelf. False
    // when there is no active source.
    bool isAncestorOrSelf(omni::physics::parse::ObjectKey ancestor, omni::physics::parse::ObjectKey node) const
    {
        ancestor = canonicalCarrierKey(ancestor);
        node = canonicalCarrierKey(node);
        if (ancestor == node)
            return true;
        const omni::physics::parse::IPhysicsSource* src = getSource();
        if (!src)
            return false;
        for (omni::physics::parse::ObjectKey k = src->canonicalKey(src->getParent(node)); k.valid();
             k = src->canonicalKey(src->getParent(k)))
        {
            if (k == ancestor)
                return true;
        }
        return false;
    }

    bool isPhysXDefaultSimulator() const
    {
        return mCachedSettings.physxDefaultSimulator;
    }

    void setDisableResetOnStop(bool disable)
    {
        mCachedSettings.disableResetOnStop = disable;
    }

    // Release engine objects and clear per-attach state. Reset paths prebuild a
    // replacement ObjectDb; owner teardown passes false to avoid rebuilding an
    // object that will immediately be destroyed.
    void releasePhysicsObjects(bool rebuildObjectDatabase = true);

    ObjectDb* getObjectDatabase()
    {
        return mObjectDatabase;
    }

    const ObjectDb* getObjectDatabase() const
    {
        return mObjectDatabase;
    }

    AnimatedKinematicBodySet& getAnimatedKinematicBodies()
    {
        return mAnimatedKinematicBodies;
    }

    const AnimatedKinematicBodySet& getAnimatedKinematicBodies() const
    {
        return mAnimatedKinematicBodies;
    }

    void removeAnimatedKinematicBody(omni::physics::parse::ObjectKey key)
    {
        mAnimatedKinematicBodies.erase(key);
    }

    PrimUpdateMap& getPrimUpdateMap()
    {
        return mPrimUpdateMap;
    }

    const PrimUpdateMap& getPrimUpdateMap() const
    {
        return mPrimUpdateMap;
    }

    PrimChangeMap& getPrimChangeMap()
    {
        return mPrimChangeMap;
    }

    const PrimChangeMap& getPrimChangeMap() const
    {
        return mPrimChangeMap;
    }

void bufferRequestRigidBodyMassUpdate(omni::physics::parse::ObjectKey key)
{
    mRigidBodyMassUpdateMap.insert(key);
}

    TimeSampleMap& getTimeSampleMap()
    {
        return mTimeSampledAttributes;
    }

    const TimeSampleMap& getTimeSampleMap() const
    {
        return mTimeSampledAttributes;
    }

    void registerTimeSampledAttribute(omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId attr, OnUpdateObjectFn onUpdate)
    {
        mTimeSampledAttributes[TimeSampledPropertyKey{ primKey, attr }] = onUpdate;
    }

    void unregisterTimeSampledAttribute(omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId attr)
    {
        mTimeSampledAttributes.erase(TimeSampledPropertyKey{ primKey, attr });
    }

    void registerStageSpecificAttribute(ChangeParams& changeParam);

    void clearStageSpecificAttributes();

    // Thin forward to ObjectDb::findOrCreateEntry(ObjectKey, ...) (LoadTools.h).
    void registerObjectId(omni::physics::parse::ObjectKey key, const ObjectCategory& category, const ObjectId& newEntryId);

    void updateRigidBodyMass();

    const ObjectIdMap* getObjectIds(omni::physics::parse::ObjectKey key) const;

    // True iff `key` still names a live object owned by this attach. "Live" is broader
    // than "authored in the parse source": a PhysX-replicator clone (and other
    // physics-only objects) is registered in the ObjectDb with no USD prim or ovstage
    // row, so IPhysicsSource::exists() alone reports it as gone (ADR-0019). Checks the
    // ObjectDb first (in-memory, and the only index that sees clone-only objects), then
    // the parse source, then the InternalPhysXDatabase for runtime-created objects such
    // as D6 joints. Shared by the two public ObjectKey->path resolvers -- objectKeyToPath
    // and IRigidContactView::getOtherActorPathsFromIds -- so they cannot disagree.
    bool isKeyLive(omni::physics::parse::ObjectKey key) const;

    CollisionGroupsMap& getCollisionGroupMap()
    {
        return mCollisionGroupsMap;
    }

    const CollisionGroupsMap& getCollisionGroupMap() const
    {
        return mCollisionGroupsMap;
    }

    std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps()
    {
        return mAdditionalCollisionGroupMaps;
    }

    const std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps() const
    {
        return mAdditionalCollisionGroupMaps;
    }

    DeformableAttachmentHistoryMap& getDeformableAttachmentHistoryMap()
    {
        return mDeformableAttachmentHistoryMap;
    }

    DeformableCollisionFilterHistoryMap& getDeformableCollisionFilterHistoryMap()
    {
        return mDeformableCollisionFilterHistoryMap;
    }

    void setGeneratedDeformableAttachmentData(omni::physics::parse::ObjectKey key,
                                              const GeneratedDeformableAttachmentData& data)
    {
        mGeneratedDeformableAttachmentData[canonicalCarrierKey(key)] = data;
    }

    const GeneratedDeformableAttachmentData* getGeneratedDeformableAttachmentData(
        omni::physics::parse::ObjectKey key) const
    {
        auto it = mGeneratedDeformableAttachmentData.find(canonicalCarrierKey(key));
        return it != mGeneratedDeformableAttachmentData.end() ? &it->second : nullptr;
    }

    void setGeneratedDeformableCollisionFilterData(omni::physics::parse::ObjectKey key,
                                                   const GeneratedDeformableCollisionFilterData& data)
    {
        mGeneratedDeformableCollisionFilterData[canonicalCarrierKey(key)] = data;
    }

    const GeneratedDeformableCollisionFilterData* getGeneratedDeformableCollisionFilterData(
        omni::physics::parse::ObjectKey key) const
    {
        auto it = mGeneratedDeformableCollisionFilterData.find(canonicalCarrierKey(key));
        return it != mGeneratedDeformableCollisionFilterData.end() ? &it->second : nullptr;
    }

    void clearGeneratedDeformableAttachmentData()
    {
        mGeneratedDeformableAttachmentData.clear();
        mGeneratedDeformableCollisionFilterData.clear();
        mGeneratedAutoAttachmentLayouts.clear();
        mGeneratedAutoAttachmentParents.clear();
    }

    // ------------------------------------------------------------------
    // In-memory auto-attachment layouts (the ovstage stand-in for authored sub-prims).

    // Replaces any previous layout for `autoAttachmentKey`.
    void setGeneratedAutoAttachmentLayout(omni::physics::parse::ObjectKey autoAttachmentKey,
                                          GeneratedAutoAttachmentLayout layout)
    {
        const omni::physics::parse::ObjectKey autoKey = canonicalCarrierKey(autoAttachmentKey);
        clearGeneratedAutoAttachmentLayout(autoKey);
        for (GeneratedAutoAttachmentChild& child : layout.children)
        {
            child.key = canonicalCarrierKey(child.key);
            mGeneratedAutoAttachmentParents[child.key] = autoKey;
        }
        mGeneratedAutoAttachmentLayouts[autoKey] = std::move(layout);
    }

    const GeneratedAutoAttachmentLayout* getGeneratedAutoAttachmentLayout(
        omni::physics::parse::ObjectKey autoAttachmentKey) const
    {
        auto it = mGeneratedAutoAttachmentLayouts.find(canonicalCarrierKey(autoAttachmentKey));
        return it != mGeneratedAutoAttachmentLayouts.end() ? &it->second : nullptr;
    }

    GeneratedAutoAttachmentLayout* getGeneratedAutoAttachmentLayout(omni::physics::parse::ObjectKey autoAttachmentKey)
    {
        auto it = mGeneratedAutoAttachmentLayouts.find(canonicalCarrierKey(autoAttachmentKey));
        return it != mGeneratedAutoAttachmentLayouts.end() ? &it->second : nullptr;
    }

    // The generated child a key stands for, or null when the key is a real source object.
    const GeneratedAutoAttachmentChild* findGeneratedAutoAttachmentChild(omni::physics::parse::ObjectKey childKey) const
    {
        const omni::physics::parse::ObjectKey key = canonicalCarrierKey(childKey);
        auto parentIt = mGeneratedAutoAttachmentParents.find(key);
        if (parentIt == mGeneratedAutoAttachmentParents.end())
            return nullptr;
        auto layoutIt = mGeneratedAutoAttachmentLayouts.find(parentIt->second);
        if (layoutIt == mGeneratedAutoAttachmentLayouts.end())
            return nullptr;
        for (const GeneratedAutoAttachmentChild& child : layoutIt->second.children)
        {
            if (child.key == key)
                return &child;
        }
        return nullptr;
    }

    void clearGeneratedAutoAttachmentLayout(omni::physics::parse::ObjectKey autoAttachmentKey)
    {
        auto it = mGeneratedAutoAttachmentLayouts.find(canonicalCarrierKey(autoAttachmentKey));
        if (it == mGeneratedAutoAttachmentLayouts.end())
            return;
        for (const GeneratedAutoAttachmentChild& child : it->second.children)
        {
            mGeneratedAutoAttachmentParents.erase(child.key);
            // The payloads are keyed by the children, so they go with the layout.
            mGeneratedDeformableAttachmentData.erase(child.key);
            mGeneratedDeformableCollisionFilterData.erase(child.key);
        }
        mGeneratedAutoAttachmentLayouts.erase(it);
    }

    const GeneratedAutoAttachmentLayoutMap& getGeneratedAutoAttachmentLayouts() const
    {
        return mGeneratedAutoAttachmentLayouts;
    }

    // Drops the layouts of every auto-attachment prim at or below `parentKey` (prim removal).
    void clearGeneratedAutoAttachmentLayoutsUnderPath(omni::physics::parse::ObjectKey parentKey)
    {
        std::vector<omni::physics::parse::ObjectKey> removed;
        for (const auto& entry : mGeneratedAutoAttachmentLayouts)
        {
            if (isAncestorOrSelf(parentKey, entry.first))
                removed.push_back(entry.first);
        }
        for (omni::physics::parse::ObjectKey key : removed)
            clearGeneratedAutoAttachmentLayout(key);
    }

    // `parentKey`-rooted ancestry test, not a pathFor()+HasPrefix() bridge
    // (ADR-0019 decision 2): walks the source's parent chain via
    // isAncestorOrSelf() instead of converting each entry's key back to a
    // path.
    void clearGeneratedDeformableAttachmentDataUnderPath(omni::physics::parse::ObjectKey parentKey)
    {
        for (auto it = mGeneratedDeformableAttachmentData.begin(); it != mGeneratedDeformableAttachmentData.end();)
        {
            it = isAncestorOrSelf(parentKey, it->first) ? mGeneratedDeformableAttachmentData.erase(it) : std::next(it);
        }
        for (auto it = mGeneratedDeformableCollisionFilterData.begin(); it != mGeneratedDeformableCollisionFilterData.end();)
        {
            it = isAncestorOrSelf(parentKey, it->first) ? mGeneratedDeformableCollisionFilterData.erase(it) : std::next(it);
        }
    }

    // ------------------------------------------------------------------
    // Cooked-geometry carrier (ADR-0022).
    //
    // Cooked sim/collision geometry is runtime SCRATCH: the auto-deformable
    // cook derives it from authored input, and the runtime's own correctness
    // may not ride on reading it back out of the scene description. It is
    // published through getDataWrite() when a sink exists (a USD attach still
    // gets the layer edit it always got), and recorded here unconditionally so
    // the later, independent re-reads work on a backend that vends no sink.
    //
    // Deliberately the same shape, owner, key and lifetime hooks as
    // mGeneratedDeformableAttachmentData above, which already solves this
    // problem for auto-generated attachment data.
    //
    // Written only by the deformable cooking write-back and read only through
    // omni::physx::internal::getArrayValue / getValue, which gate the lookup on
    // there being no live write sink — the carrier substitutes for a missing
    // sink, it never shadows a working one.
    // ------------------------------------------------------------------
    void setCookedArray(omni::physics::parse::ObjectKey key,
                        omni::physics::parse::TokenId attr,
                        const void* bytes,
                        size_t byteCount,
                        omni::physics::parse::BufferElemType type,
                        uint32_t elemCount)
    {
        CookedArray& entry = mCookedGeometry[canonicalCarrierKey(key)][attr];
        if (bytes && byteCount)
        {
            const uint8_t* first = static_cast<const uint8_t*>(bytes);
            entry.bytes.assign(first, first + byteCount);
        }
        else
        {
            entry.bytes.clear();
        }
        entry.type = type;
        entry.elemCount = elemCount;
        entry.recorded = true;
    }

    const CookedArray* getCookedArray(omni::physics::parse::ObjectKey key,
                                      omni::physics::parse::TokenId attr) const
    {
        const auto it = mCookedGeometry.find(canonicalCarrierKey(key));
        if (it == mCookedGeometry.end())
            return nullptr;
        const auto attrIt = it->second.find(attr);
        return attrIt != it->second.end() ? &attrIt->second : nullptr;
    }

    // Drop one recorded attribute. The write-back's counterpart to the USD
    // sink's removeAttribute: a stale entry from an earlier cook must not
    // survive a cook that decided the attribute should not exist.
    void clearCookedArray(omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr)
    {
        const auto it = mCookedGeometry.find(canonicalCarrierKey(key));
        if (it == mCookedGeometry.end())
            return;
        it->second.erase(attr);
        if (it->second.empty())
            mCookedGeometry.erase(it);
    }

    // False when nothing has ever been cooked on this attach — the read hook's
    // fast path, so a runtime that never auto-cooks pays one branch per read.
    bool hasCookedGeometry() const
    {
        return !mCookedGeometry.empty();
    }

    void clearCookedGeometry()
    {
        mCookedGeometry.clear();
    }

    // `parentKey`-rooted ancestry test; see clearGeneratedDeformableAttachmentDataUnderPath above.
    void clearCookedGeometryUnderPath(omni::physics::parse::ObjectKey parentKey)
    {
        for (auto it = mCookedGeometry.begin(); it != mCookedGeometry.end();)
        {
            it = isAncestorOrSelf(parentKey, it->first) ? mCookedGeometry.erase(it) : std::next(it);
        }
    }



    void addReplicatorMemoryBlock(void* memory)
    {
        mReplicatorMemory.push_back(memory);
    }

    void freeReplicatorMemory()
    {
        for (void* mem : mReplicatorMemory)
        {
            free(mem);
        }

        mReplicatorMemory.clear();
    }

    void setUseReplicatorEnvIds(bool val)
    {
        mUseReplicatorEnvIds = val;
    }

    bool isUsingReplicatorEnvIds() const
    {
        return mUseReplicatorEnvIds;
    }

    // Running base for explicit replicator env-ids. Each clone() batch on this attach numbers its
    // copies 1..N; the base offsets later batches by the count already assigned, so a second clone
    // does not reuse the first batch's env-ids (which under GPU broadphase would merge two distinct
    // environments into one for collision filtering). Reset with the stage.
    uint32_t getReplicatorEnvIdBase() const
    {
        return mReplicatorEnvIdBase;
    }

    void advanceReplicatorEnvIdBase(uint32_t count)
    {
        mReplicatorEnvIdBase += count;
    }

    // Raise the base to at least `atLeast` — used after a batch with caller-supplied env ids so a
    // later positional batch cannot alias an explicitly-placed environment.
    void raiseReplicatorEnvIdBase(uint32_t atLeast)
    {
        if (mReplicatorEnvIdBase < atLeast)
            mReplicatorEnvIdBase = atLeast;
    }

    // Target paths of successful IPhysxSimulation::cloneEnvironments calls on this attach.
    // Runtime clones author no USD prim, so a USD walk cannot see them; this record is what
    // lets a later clone reject a reused or nested (ancestor/descendant) target, which would
    // stack duplicate live actors on one path. Lives and dies with the attach, like the clones.
    //
    // Plain std::string-keyed (not SdfPath, not ObjectKey), same precedent and for the same
    // reason as PrimHierarchyStorage (common/utilities/PrimHierarchyStorage.h): these targets
    // are SYNTHETIC identities never authored into the backing IPhysicsSource, so an
    // ObjectKey-keyed set could not be enumerated/prefix-compared without a redundant
    // side-table, and this is pure path-prefix hierarchy arithmetic (ancestor/descendant
    // overlap), not USD-specific. Works for every source.
    const std::set<std::string>& getRuntimeCloneTargets() const
    {
        return mRuntimeCloneTargets;
    }

    void addRuntimeCloneTarget(const std::string& path)
    {
        mRuntimeCloneTargets.insert(path);
    }

    void setChangeSource(ChangeSource source)
    {
        mChangeSource = source;
    }

    ChangeSource getChangeSource() const
    {
        return mChangeSource;
    }

    ChangeSourceBlock getChangeSourceBlock(ChangeSource source)
    {
        return ChangeSourceBlock(*this, source);
    }

    // Find-or-mint-and-share-the-counter, keyed by TokenId.
    uint32_t registerEnvIdFromToken(omni::physics::parse::TokenId token)
    {
        TokenIdEnvIdMap::const_iterator fit = mTokenIdEnvIdMap.find(token);
        if (fit == mTokenIdEnvIdMap.end())
        {
            const uint32_t envIdInt = mEnvIdCounter++;
            mTokenIdEnvIdMap[token] = envIdInt;
            return envIdInt;
        }
        else
        {
            return fit->second;
        }
    }

    uint32_t getEnvIdFromToken(omni::physics::parse::TokenId token) const
    {
        TokenIdEnvIdMap::const_iterator fit = mTokenIdEnvIdMap.find(token);
        if (fit != mTokenIdEnvIdMap.end())
        {
            return fit->second;
        }
        else
        {
            CARB_LOG_WARN("EnvId not found for given scene partition token id: %llu",
                          (unsigned long long)token.id);
            return 0;
        }
    }

private:
    ObjectNotificationState mObjectNotificationState;
    // Normalize `key` to the source's canonical handle before it touches a
    // carrier map keyed by ObjectKey (mCookedGeometry, and the analogous
    // mGeneratedDeformable* maps). Some backends (ovstage) can mint distinct
    // ObjectKeys for the same logical object depending on the lookup used to
    // reach it (enumerate/get_paths vs intern_path -- see IPhysicsSource::
    // canonicalKey); without this, a write and a later read/clear that reached
    // the object through different lookups would land in different map slots.
    // Identity on every other backend (canonicalKey defaults to identity), so
    // this is a provable no-op there.
    omni::physics::parse::ObjectKey canonicalCarrierKey(omni::physics::parse::ObjectKey key) const
    {
        if (!key.valid())
            return key;
        const omni::physics::parse::IPhysicsSource* src = getSource();
        return src ? src->canonicalKey(key) : key;
    }

    bool mEnableVisualization{ false };
    bool mEnableNormalsVisualization{ false };
    bool mDebugVisualizationDirty{ true };
    float mVisualizationScale{ 1.0f };
    uint64_t mVisualizationBitMask{ 0ul };
    mutable std::unique_ptr<DebugVisualizationCache> mDebugVisualizationCache;
    mutable internal::InternalPhysXDatabase* mInternalPhysXDatabase{ nullptr };
    mutable FilteredPairsSet mFilteredPairs;
    mutable CollisionGroupsPairsSet mCollisionsGroupFilteredPairs;
    bool mSimulationRunning = false;
    bool mSimulationStarted = false;
    std::atomic_uint64_t mSimulationSteps{ 0 };
    std::atomic_uint64_t mManualSteps{ 0 };
    uint64_t mSimulationStepOffset = 0;
    std::atomic_bool mAsyncUpdate{ false };
    mutable SimulationCallbacks mSimulationCallbacks{ *this };
    mutable std::unique_ptr<RaycastManager> mRaycastManager;
    mutable AttachmentTriggerState mTriggerState;
    mutable ParseOptions mParseOptions;
    OmniCachedSettings mCachedSettings;
    PhysxSharedSettings mSharedSettings;
    std::vector<std::string> mExcludedPaths;
    omni::physics::parse::ObjectKey mTempPhysicsSceneKey;
    mutable PendingPhysicsSetup mPendingPhysicsSetup;
    SceneMap mScenes;
    PhysXScene* mDefaultPhysXScene{ nullptr };
    std::shared_ptr<AttachedStage*> mLifetime;
    std::weak_ptr<bool> mCookingObserverDriver;
    mutable std::function<void()> mRetireCooking;
    std::shared_ptr<NoticeBlockState> mNoticeBlockState = std::make_shared<NoticeBlockState>();
    mutable MeshKeyCache mMeshKeyCache;
    mutable CookingRefreshState mCookingRefreshState;
    PhysXUsdPhysicsInterface* mPhysicsInterface;
    AttachedStageUsdHandle mStage;
    uint64_t mAttachHandle = 0;
    ObjectDb* mObjectDatabase;
    PrimUpdateMap mPrimUpdateMap;
    PrimChangeMap mPrimChangeMap;
    TimeSampleMap mTimeSampledAttributes;
    AnimatedKinematicBodySet mAnimatedKinematicBodies;
    std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> mRigidBodyMassUpdateMap;
    CollisionGroupsMap mCollisionGroupsMap;
    std::vector<CollisionGroupsMap> mAdditionalCollisionGroupMaps;
    DeformableAttachmentHistoryMap mDeformableAttachmentHistoryMap;
    DeformableCollisionFilterHistoryMap mDeformableCollisionFilterHistoryMap;
    GeneratedDeformableAttachmentDataMap mGeneratedDeformableAttachmentData;
    GeneratedDeformableCollisionFilterDataMap mGeneratedDeformableCollisionFilterData;
    GeneratedAutoAttachmentLayoutMap mGeneratedAutoAttachmentLayouts;
    // child key -> auto-attachment key, the reverse index of mGeneratedAutoAttachmentLayouts.
    std::unordered_map<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>
        mGeneratedAutoAttachmentParents;
    // Cooked geometry produced by the auto-deformable cook (ADR-0022). Cleared
    // whenever the source trio is rebuilt and on releasePhysicsObjects, so it
    // never outlives its source's token/key vocabulary.
    CookedGeometryMap mCookedGeometry;

    bool mUseReplicatorEnvIds;
    uint32_t mEnvIdCounter;
    uint32_t mReplicatorEnvIdBase;
    TokenIdEnvIdMap mTokenIdEnvIdMap;
    std::set<std::string> mRuntimeCloneTargets;

    std::vector<void*> mReplicatorMemory;
    ChangeSource mChangeSource{ ChangeSource::eUnknwon };

    // Active parse source trio (ADR-0005), (re)built by the registered backend
    // in rebuildSource() when setStage is called. Owned and exposed purely
    // through the backend-neutral interfaces — no concrete backend type leaks
    // into this consumer.
    // Declared before the source bundle so backends and payload outlive all readers.
    omni::physics::ovstage::OvstageAttach mOwnedAttachPayload;
    std::unique_ptr<omni::physics::parse::IParseBackend> mParseBackend;
    std::unique_ptr<omni::physics::parse::IScanBackend> mScanBackend;
    // Protect publication/reset across coordinator entry points; never used by fill workers.
    std::mutex mDescriptorPoolMutex;
    std::shared_ptr<omni::physics::parse::DescriptorBatchAllocator> mDescriptorPool;
    std::unique_ptr<omni::physics::parse::IPhysicsSource> mSource;
    // Stage units cached from mSource in rebuildSource() — see getSourceUnits().
    omni::physics::parse::SourceUnits mUnits;
    // Well-known token batch bound once against mSource in rebuildSource() via
    // bindKnownTokens() — see getKnownTokens(). KnownTokens::intern()
    // unconditionally re-interns its whole ~659-field vocabulary on every call, so
    // callers must read the bound batch instead of interning their own per
    // query/update (REQ-SIM-SCENEQUERY-001). mKnownTokensRef points at the
    // source's own batch when it caches one, else at mKnownTokens (the owned
    // fallback for sources that do not); it is re-resolved whenever mSource is
    // replaced and nulled whenever mSource is dropped, so it never outlives the
    // source it points into.
    omni::physics::parse::KnownTokens mKnownTokens;
    const omni::physics::parse::KnownTokens* mKnownTokensRef = nullptr;
    void bindKnownTokens();
    std::unique_ptr<omni::physics::parse::IPhysicsDataWrite> mDataWrite;
    // getAuthoringDataWrite()'s fallback; only ever set on an external-source attach that
    // retained a resident backing USD stage. Holds a raw pointer into mSource, so it is
    // rebuilt/cleared together with it.
    std::unique_ptr<omni::physics::parse::IPhysicsDataWrite> mBackingStageDataWrite;
    // Runtime change feed (ADR-0003), vended by the source and rebuilt in
    // lockstep with it. Owns this stage's change listener and drives the
    // onSourceChange / onSourceGroupComplete consumer callbacks.
    std::unique_ptr<omni::physics::parse::IChangeFeed> mChangeFeed;

    // Consumer-provided, backend-opaque AttachTarget payload (ADR-0002 M2c) —
    // non-null selects an external parse/scan backend over the USD stage. Set
    // via setOvstageSource(); consumer-owned (must outlive the attach). Kept as
    // const void* so this header needs no ovstage (or any backend) dependency.
    const void* mExternalAttachPayload = nullptr;
    // Optional snapshot override carried through AttachTarget for runtime ovstage
    // re-scans. 0 means the backend should use its payload's attach-time ordinal.
    uint64_t mExternalReadOrdinal = 0;
    // The external attach's *input parameter* for its resident backing stage id,
    // as classified by the caller of setOvstageSource(). This is not a second
    // copy of the stage id — it exists only because the external backend must be
    // told the id in order to build the source, and the source is what owns the
    // id from then on (getStageId() reads it back through IPhysicsSource, never
    // from here). Meaningless, and cleared, on the USD path.
    uint64_t mExternalBackingStageId = 0;

    void rebuildSource();

    // Installs the prim-hierarchy liveness predicate on `db`. Sole definition of that
    // predicate: it is source-independent (pxr-free spelling), so it lives here rather than
    // in a USD-only sliver, and every ObjectDb this class owns -- both ctors and
    // releasePhysicsObjects()'s replacement -- is initialised through it.
    void initPrimHierarchyStorage(ObjectDb& db);

    // --- USD-reaching slivers (ADR-0027) -----------------------------------------------
    // Defined in the pxr-free usdBridge/AttachedStageBridge.cpp; USD is reached only through
    // the installed seams, so each is a no-op when none is installed.

    // The ctor's `setStage(stage)`. No-op without the handle-ops seam (handle stays empty).
    void initUsdStageBinding(AttachedStageUsdHandle stage);

    // The first intern of the change params the common ctor staged; only a USD-backed attach
    // can do it at ctor time (its source already exists). No-op for ovstage attaches, which
    // intern from rebuildSource() instead. Staging the params and installing the
    // prim-hierarchy predicate are source-independent and stay in the common ctor
    // (REQ-BUILD-UNIBUILD-001 AC-7).
    void initUsdChangeRegistrations();

    // rebuildSource()'s `if (mStage)` arm. Returns false (nothing built, caller falls
    // through to the no-source teardown branch) when there is no live USD stage.
    bool rebuildUsdSource();

    // attachTarget()'s non-external arm: points the target at the live USD stage handle.
    void fillUsdAttachTarget(omni::physics::parse::AttachTarget& target) const;

    // The resident-backing-stage fallback of createDefault/removeDefaultPhysicsScenePlaceholder,
    // used when the active source wires no write sink (ovstage, by design).
    bool createDefaultPhysicsSceneOnStage(omni::physics::parse::ObjectKey sceneKey);
    void removeDefaultPhysicsSceneOnStage(omni::physics::parse::ObjectKey sceneKey);

    // Builds getAuthoringDataWrite()'s fallback sink over the resident backing USD stage,
    // resolving keys/tokens through the (non-USD) active source. Null when there is no
    // backing stage or no source.
    std::unique_ptr<omni::physics::parse::IPhysicsDataWrite> makeBackingStageDataWrite();
};

} // namespace usdparser
} // namespace physx
} // namespace omni
