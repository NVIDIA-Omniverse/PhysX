// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-6 AC-8
 */

#include "tensors/CommonTypes.h" // RigidBodyEntry / ArticulationEntry (cached superset entries)
#include "tensors/PhysicsTypes.h"
#include "tensors/cpu/CpuSimulationData.h"
#include "tensors/gpu/GpuSimulationData.h"

#include <omni/physx/IPhysx.h>
#include <omni/physics/tensors/TensorApi.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationView;
}
}
}

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}
}
}

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::ISimulationView;

class GpuSimulationView;
class CpuSimulationView;

// The PhysX simulation backend. Provided to consumers through the static
// PhysX runtime TensorApi table (createSimulationView/reset/resetStage). There
// is no backend registry; PhysX is the only backend.
class SimulationBackend
{
public:
    SimulationBackend();
    ~SimulationBackend();

    //
    // public API
    //

    ISimulationView* createSimulationView(omni::physics::tensors::AttachHandle attachHandle);

    // Stageless entry (ADR-0008): build a simulation view directly over a PxScene, with no resident
    // UsdStageCache stage, so an ovstage-only attach can still source bulk reads from the backend.
    // The view carries a null UsdStage and never dereferences it on the rigid/articulation read
    // path. Returns nullptr on a null scene / GPU-data init failure.
    ISimulationView* createSimulationViewForScene(::physx::PxScene* scene);

    void onSceneDestroyed(const ::physx::PxScene* scene);

    void shutdown();

    void reset();

    void resetStage(omni::physics::tensors::AttachHandle attachHandle);

    //
    // utilities
    //

    // total number of physics simulation steps since the application started, always increasing
    int64_t getTimestamp() const;

    // number of simulation steps in the active simulation, or 0 if there is no active simulation
    int64_t getStepCount() const;

    // stage update events
    void prePhysicsUpdate();

    // manual step counting
    void incrementStepCount()
    {
        ++mManualStepCount;
    }

    void removeSimulationView(ISimulationView* view);

    // Drop this scene's cached simulation data so the next createSimulationViewForScene rebuilds it.
    // For staleness the entry's own validation cannot see -- notably a PxScene* recycled to a
    // different scene with identical topology.
    void invalidateSceneCache(::physx::PxScene* scene);

    // The cached simulation view for a scene, built once and BACKEND-OWNED -- the caller must not
    // release it. Ask it for the superset rigid/articulation views; they are its children.
    // `outGeneration` identifies the build, so a consumer caching anything derived from those views
    // can tell when they were rebuilt underneath it.
    //
    // GPU-typed because it is GPU-only: returns null unless the scene has eENABLE_DIRECT_GPU_API.
    // Saying so in the signature keeps callers from reaching for a static_cast.
    GpuSimulationView* acquireSceneView(::physx::PxScene* scene, uint64_t* outGeneration = nullptr);

    // The CPU counterpart, with the same ownership and generation contract. Separate entry point
    // rather than one returning a base pointer, for the reason above: the caller needs the concrete
    // type to reach supersetRigidView(). Returns null when the scene runs the GPU pipeline.
    CpuSimulationView* acquireCpuSceneView(::physx::PxScene* scene, uint64_t* outGeneration = nullptr);

    // Conservative O(1) "might this scene hold a rigid dynamic with no DirectGPU row?", for callers
    // deciding whether to run a per-body actor-flag scan. True when the answer is unknown -- no GPU
    // data for the scene yet -- so a false is always something that saw the whole scene.
    bool sceneMayHaveDisabledRigidDynamics(::physx::PxScene* scene);

private:
    ::physx::PxScene* findPhysicsScene(const usdparser::AttachedStage& attachedStage) const;

    // Lock ordering: mStageDataMutex must be held to access any of the maps/sets
    // below. Views must NOT call back into removeSimulationView() during invalidate()
    // (would deadlock -- mStageDataMutex is non-recursive). Required call order when
    // both registry and backend locks are needed: registryMutex -> mStageDataMutex.
    std::mutex mStageDataMutex;

    // Keyed by attach handle, not stage id: two attaches of the same USD stage are
    // different attaches, and a stageless attach has no usable stage id at all.
    using AttachHandle = omni::physics::tensors::AttachHandle;

    // Identity of one cached scene: the attach that owns it AND which of that attach's scenes it
    // is. Attach alone is not an identity -- one attach can hold several PhysicsScene prims, and
    // keying on it alone makes them share a single entry, so reading them in turn evicts and
    // rebuilds every device buffer on every read. The PxScene* is not an identity on its own
    // either, because addresses are recycled across attach cycles; it only separates the scenes an
    // attach currently holds. What rejects a recycled address is unchanged: sceneCacheEntryValid's
    // hasScene/topology checks and gpuSimDataShapeMatches.
    struct SceneKey
    {
        AttachHandle attach = omni::physics::kNoAttach;
        const ::physx::PxScene* scene = nullptr;

        bool operator==(const SceneKey& o) const
        {
            return attach == o.attach && scene == o.scene;
        }
    };

    struct SceneKeyHash
    {
        size_t operator()(const SceneKey& k) const
        {
            return std::hash<AttachHandle>()(k.attach) * 31u + std::hash<const void*>()(k.scene);
        }
    };

    // Per attach, NOT per scene: CpuSimulationData holds no scene and nothing sized from one --
    // contact buckets and dirty-force trackers are attach-scoped. Keying it per scene would put two
    // instances behind one attach and double every contact-bucket registration.
    std::unordered_map<AttachHandle, CpuSimulationDataPtr>      mCpuSimDataByAttach;
    std::unordered_map<SceneKey, GpuSimulationDataPtr, SceneKeyHash> mGpuSimDataByScene;
    // Views indexed per attach so resetStage() can invalidate only that attach's
    // views; the flattened set of all live views is the union of these vectors.
    std::unordered_map<AttachHandle, std::vector<ISimulationView*>> mViewsByAttach;

    // Scene-scoped cache for the stageless (ovstage) entry: the simulation view for a scene, and
    // the state validating that it still describes it. The simulation data itself is NOT owned here
    // -- it lives in mGpuSimDataByScene and this holds a shared_ptr to it -- but building it and
    // the superset views on top of it dominates the read they serve, which is what this cache
    // exists to avoid (ADR-0008).
    //
    // A PxScene* is recycled across attach cycles, so the pointer alone is NOT an identity: a stale
    // entry handed to a different scene at the same address produces a getArticulationData
    // null-deref on an articulation-link read. Every hit therefore revalidates the invariants the
    // simulation data was sized and indexed by; any mismatch rebuilds.

    // Every quantity GpuSimulationData::init() sizes a buffer from. There is no resize path, so a
    // change in any of these means the cached data is the wrong shape.
    struct SceneTopology
    {
        ::physx::PxU32 numRds = 0;
        ::physx::PxU32 numArtis = 0;
        ::physx::PxU32 maxLinks = 0;
        ::physx::PxU32 maxDofs = 0;
        ::physx::PxU32 maxFixedTendons = 0;
        ::physx::PxU32 maxSpatialTendons = 0;

        bool operator==(const SceneTopology& o) const
        {
            return numRds == o.numRds && numArtis == o.numArtis && maxLinks == o.maxLinks &&
                   maxDofs == o.maxDofs && maxFixedTendons == o.maxFixedTendons &&
                   maxSpatialTendons == o.maxSpatialTendons;
        }
    };

    struct SceneCacheEntry
    {
        CpuSimulationDataPtr cpuData;
        GpuSimulationDataPtr gpuData;
        ::physx::PxScene* scene = nullptr;
        ::physx::PxCudaContextManager* ctxMgr = nullptr;
        bool gpuPipeline = false;
        SceneTopology topology;

        // The database's object-lifetime epoch when `topology` was measured. While it is unchanged,
        // no PhysX object has been created or removed, so the topology cannot have moved and the
        // scene walk that measures it can be skipped -- see sceneCacheEntryValid.
        uint64_t dbEpoch = 0;

        // Bumped every time this entry is (re)built, i.e. whenever the cached views stop describing
        // the scene they were built for. A consumer that derives its own per-scene data from the
        // superset view can cache it alongside this number and recompute only when it changes.
        // Never reused: it counts rebuilds, not entries.
        uint64_t generation = 0;

        // Superset rigid-body view over EVERY rigid dynamic and articulation link in the scene,
        // built once and reused; a read selects its subset with a per-read record list rather than
        // constructing a view over the requested set. One view serves both standalone bodies and
        // links, since a PxArticulationLink is a PxRigidBody the rigid-body view already handles.
        //
        // Backend-owned. Released only via takeSceneCacheViews(), never while mStageDataMutex is
        // held: ~BaseSimulationView calls back into removeSimulationView(), which takes that same
        // non-recursive lock.
        ISimulationView* supersetView = nullptr;
    };
    std::unordered_map<SceneKey, SceneCacheEntry, SceneKeyHash> mSceneCache;

    // Views whose scene was destroyed. Released by the next acquire rather than in the destruction
    // notification itself, where the PxScene is still being torn down.
    std::vector<ISimulationView*> mDeferredViewReleases;

    // Detach an entry's cached views so the caller can release them after dropping the lock.
    static void takeSceneCacheViews(SceneCacheEntry& entry, std::vector<ISimulationView*>& out);
    // Detach every view the backend owns -- the cached superset views and any already detached by a
    // destruction notification. Both teardown paths, reset() and shutdown(), go through here. Call
    // with mStageDataMutex held; the caller releases `out` after dropping it.
    void takeOwnedViews(std::vector<ISimulationView*>& out);

    // True when `entry` still describes `scene` as it is now. Cheap: PxScene's counts are O(1)
    // accessors and the per-articulation loop only runs when articulations are present.
    static bool sceneCacheEntryValid(const SceneCacheEntry& entry, ::physx::PxScene* scene);

    // Recompute the topology GpuSimulationData::init() sizes its buffers from.
    static void sceneTopology(::physx::PxScene* scene, SceneTopology& t);

    // True when `data`'s buffers are still the right shape for `scene`. Compares against the sizes
    // the data actually holds rather than a remembered copy of them, so the two cannot drift.
    static bool gpuSimDataShapeMatches(const GpuSimulationData& data, ::physx::PxScene* scene);

    // The attach that owns `scene`. The backend keys by attach; the ovstage read enumerates scenes
    // out of the object database, which carries no attach column, so the resolution lives here
    // rather than at every read call site.
    AttachHandle attachOwning(const ::physx::PxScene* scene) const;

    // The AttachedStage behind that attach, or null when there is none.
    usdparser::AttachedStage* attachedStageFor(const ::physx::PxScene* scene) const;

    // The scene's GPU simulation data, rebuilt when the scene no longer matches the topology its
    // buffers were sized from. Every path that hands this data to a view must go through here.
    // Caller must hold mStageDataMutex. Returns null if the data could not be built.
    GpuSimulationDataPtr ensureGpuSimDataLocked(AttachHandle attachHandle, ::physx::PxScene* scene);

    // The scene's cache entry, validated and rebuilt if stale. Caller must hold mStageDataMutex,
    // and must release anything appended to `pendingRelease` only after dropping it. Returns null
    // if the simulation data could not be built. Separate from view construction so the read path
    // can reach a validated entry without creating a view it does not need.
    SceneCacheEntry* ensureSceneCacheEntryLocked(::physx::PxScene* scene,
                                                 std::vector<ISimulationView*>& pendingRelease);

    uint64_t mManualStepCount = 0;

    omni::physx::SubscriptionId mObjectChangeSubscriptionId = omni::physx::kInvalidSubscriptionId;
};

} // namespace tensors
} // namespace physx
} // namespace omni
