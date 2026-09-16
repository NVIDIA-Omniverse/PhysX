// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-3 AC-7
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-6 AC-8 AC-9
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-TENSOR-SCENE-001
 * @covers AC-1 AC-2
 */

#include "internal/Internal.h" // recordLifetimeEpoch
#include "tensors/GlobalsAreBad.h"
#include "tensors/SimulationBackend.h"
#include "tensors/cpu/CpuSimulationView.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/CudaCommon.h"

#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadTools.h"
#include "usdLoad/LoadUsd.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxSimulation.h>

#include <algorithm>
#include <atomic>
#include <limits>

// so we can determine the physx device ordinal
#include <carb/events/IEvents.h>
#include <common/utilities/PhysXErrorCallback.h>
static CarbPhysXErrorCallback gErrorCallback;

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

namespace
{
// Backend-level destruction hook. Distinct from the per-view subscriptions, which only invalidate
// the view that owns them: this one exists so the per-stage data is dropped even when no view is
// alive to observe the deletion.
void onPhysXObjectDestroyedCallback(omni::physics::parse::ObjectKey key,
                                    usdparser::ObjectId objectId,
                                    omni::physx::PhysXType type,
                                    void* userData)
{
    if (!userData || !g_physx)
        return;

    SimulationBackend* backend = static_cast<SimulationBackend*>(userData);
    if (type == omni::physx::ePTScene)
    {
        backend->onSceneDestroyed(static_cast<PxScene*>(g_physx->getPhysXPtrFast(objectId)));
    }

}
} // namespace

SimulationBackend::SimulationBackend()
{
    if (g_physx)
    {
        omni::physx::IPhysicsObjectChangeCallback callback;
        callback.objectDestructionNotifyFn = onPhysXObjectDestroyedCallback;
        callback.userData = this;
        mObjectChangeSubscriptionId = g_physx->subscribeObjectChangeNotifications(callback);
    }

    if (mObjectChangeSubscriptionId == omni::physx::kInvalidSubscriptionId)
    {
        CARB_LOG_ERROR(
            "Failed to subscribe to physics object change notifications; simulation data for a replaced physics scene will not be rebuilt");
    }
}

void SimulationBackend::shutdown()
{
    if (g_physx && mObjectChangeSubscriptionId != omni::physx::kInvalidSubscriptionId)
    {
        g_physx->unsubscribeObjectChangeNotifications(mObjectChangeSubscriptionId);
    }
    mObjectChangeSubscriptionId = omni::physx::kInvalidSubscriptionId;

    // ~SimulationBackend calls only this, never reset(), so for a backend torn down without a reset
    // this is the last chance to free the views it owns. Safe to run twice: takeOwnedViews() clears
    // what it detaches, so a shutdown() after a reset() finds nothing.
    std::vector<ISimulationView*> pendingRelease;
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);
        takeOwnedViews(pendingRelease);
    }
    // Outside the lock: ~BaseSimulationView calls back into removeSimulationView().
    for (ISimulationView* v : pendingRelease)
        v->release(true);
}

SimulationBackend::~SimulationBackend()
{
    shutdown();
}

void SimulationBackend::onSceneDestroyed(const PxScene* scene)
{
    if (!scene)
        return;

    std::lock_guard<std::mutex> lock(mStageDataMutex);

    for (auto it = mGpuSimDataByScene.begin(); it != mGpuSimDataByScene.end();)
    {
        if (it->first.scene == scene)
        {
            it = mGpuSimDataByScene.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // The CPU side has no scene to match on (CpuSimulationData holds none), so it is reclaimed by
    // resetStage on the attach axis rather than here.
    //
    // The scene cache is this backend's own: drop the entry describing the dead scene, detaching
    // its superset view for the next acquire to release -- releasing here would re-enter
    // removeSimulationView under this lock.
    for (std::unordered_map<SceneKey, SceneCacheEntry, SceneKeyHash>::iterator it = mSceneCache.begin();
         it != mSceneCache.end();)
    {
        if (it->first.scene == scene)
        {
            takeSceneCacheViews(it->second, mDeferredViewReleases);
            it = mSceneCache.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

ISimulationView* SimulationBackend::createSimulationView(AttachHandle attachHandle)
{
    if (!g_physx)
    {
        CARB_LOG_ERROR("Failed to create simulation view: physics interface is not available");
        return nullptr;
    }

    if (attachHandle == omni::physics::tensors::kActiveAttach)
    {
        if (!g_physxSimulation)
        {
            CARB_LOG_ERROR("Failed to resolve the active attach: PhysX simulation interface is not available");
            return nullptr;
        }
        attachHandle = g_physxSimulation->getAttachHandle();
    }

    // ADR-0013: the handle identifies an attach, not a USD stage. Resolve it to the
    // AttachedStage that owns the physics objects; a null USD stage behind it is legal
    // (a stageless ovstage attach), an unresolvable handle is not.
    usdparser::AttachedStage* attachedStage =
        usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("Failed to create simulation view: attach handle %llu does not identify a live attach",
                       static_cast<unsigned long long>(attachHandle));
        return nullptr;
    }

    PxScene* scene = findPhysicsScene(*attachedStage);
    if (!scene)
    {
        CARB_LOG_ERROR("Failed to create simulation view: no active physics scene found");
        return nullptr;
    }

    bool useGpuPipeline = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    std::lock_guard<std::mutex> lock(mStageDataMutex);

    if (useGpuPipeline)
    {
        // Through the same helper the cached path uses. Reusing the entry whenever the SCENE
        // pointer matched was not enough: init() sizes every device buffer from the scene's counts
        // and there is no resize path, so a scene that GREW since the entry was built hands the next
        // view buffers too small for the counts it passes to the DirectGPU getters. That writes out
        // of bounds, poisons the CUDA context, and surfaces as an unrelated CUDA 700 later.
        const GpuSimulationDataPtr gpuData = ensureGpuSimDataLocked(attachHandle, scene);
        if (!gpuData)
        {
            CARB_LOG_ERROR("Failed to initialize GPU simulation data");
            return nullptr;
        }
        GpuSimulationView* simView = new GpuSimulationView(attachedStage, scene, gpuData);
        mViewsByAttach[attachHandle].push_back(simView);
        return simView;
    }
    else
    {
        auto& cpuData = mCpuSimDataByAttach[attachHandle];
        if (!cpuData)
        {
            cpuData = std::make_shared<CpuSimulationData>(*this, attachHandle);
        }
        CpuSimulationView* simView = new CpuSimulationView(attachedStage, scene, cpuData);
        mViewsByAttach[attachHandle].push_back(simView);
        return simView;
    }
}

// Topology the simulation data is sized and indexed by. Recomputed on every cache hit and compared
// against what the entry was built with -- see SceneCacheEntry.
// Must mirror GpuSimulationData::init() exactly, including its 0xffffffff inbound-dof guard: a
// divergence here either never matches (the cache silently never hits) or matches when the buffers
// were sized differently (the crash this validation exists to prevent).
// TODO: this walk runs on every cache lookup, i.e. every read; profile it on an articulation-heavy
// scene. Rigid-only scenes early-out after two O(1) counts.
void SimulationBackend::sceneTopology(::physx::PxScene* scene, SceneTopology& t)
{
    t = SceneTopology{};
    t.numRds = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    t.numArtis = scene->getNbArticulations();
    if (t.numArtis == 0)
        return;

    std::vector<PxArticulationReducedCoordinate*> artis(t.numArtis);
    scene->getArticulations(artis.data(), t.numArtis);
    for (PxArticulationReducedCoordinate* arti : artis)
    {
        if (!arti)
            continue;
        // getDofs() is the joint DOF total, excluding a floating base's six -- no per-link walk needed.
        const PxU32 numLinks = arti->getNbLinks();
        const PxU32 numDofs = arti->getDofs(); // 0xffffffff if the articulation is not in a scene

        t.maxLinks = PxMax(t.maxLinks, numLinks);
        if (numDofs != 0xffffffff)
            t.maxDofs = PxMax(t.maxDofs, numDofs);
        t.maxFixedTendons = PxMax(t.maxFixedTendons, arti->getNbFixedTendons());
        t.maxSpatialTendons = PxMax(t.maxSpatialTendons, arti->getNbSpatialTendons());
    }
}

bool SimulationBackend::gpuSimDataShapeMatches(const GpuSimulationData& data, ::physx::PxScene* scene)
{
    SceneTopology now;
    sceneTopology(scene, now);
    return data.mNumRds == now.numRds && data.mNumArtis == now.numArtis && data.mMaxLinks == now.maxLinks &&
           data.mMaxDofs == now.maxDofs && data.mMaxFixedTendons == now.maxFixedTendons &&
           data.mMaxSpatialTendons == now.maxSpatialTendons;
}

// The AttachedStage that owns `scene`. Null for a scene with no live attach, which the view
// constructors accept -- a stageless attach has no UsdStage behind it and never dereferences one.
usdparser::AttachedStage* SimulationBackend::attachedStageFor(const ::physx::PxScene* scene) const
{
    const AttachHandle handle = attachOwning(scene);
    return handle == omni::physics::kNoAttach ?
               nullptr :
               usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(handle);
}

// The attach that owns `scene`; every cache key carries one (ADR-0013). ovruntime holds one attach
// at a time, so this is the active attach -- a multi-attach process needs a reverse index. That
// limit is on the ATTACH axis only: an attach with several scenes is keyed correctly, because
// SceneKey pairs this handle with the scene itself.
AttachHandle SimulationBackend::attachOwning(const ::physx::PxScene* scene) const
{
    if (!scene)
        return omni::physics::kNoAttach;
    const usdparser::AttachedStage* attached = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    return attached ? attached->getAttachHandle() : omni::physics::kNoAttach;
}

GpuSimulationDataPtr SimulationBackend::ensureGpuSimDataLocked(AttachHandle attachHandle, ::physx::PxScene* scene)
{
    // init() sizes every device buffer from the scene's counts and there is no resize path, so data
    // built for a smaller scene must not be reused for a grown one: a view passes its own larger
    // count to the DirectGPU getters and writes out of bounds. Nothing reports it at the call site --
    // it poisons the CUDA context and surfaces as a later, unrelated CUDA 700.
    // Keyed by attach AND scene, so an attach holding several scenes keeps one entry per scene:
    // keying on attach alone made two scenes alternate through a single entry and rebuilt every
    // device buffer on each read. The entry is dropped when the scene it describes grew.
    const SceneKey key{ attachHandle, scene };
    const std::unordered_map<SceneKey, GpuSimulationDataPtr, SceneKeyHash>::iterator it =
        mGpuSimDataByScene.find(key);
    if (it != mGpuSimDataByScene.end())
    {
        if (it->second && gpuSimDataShapeMatches(*it->second, scene))
            return it->second;
        // Dropping the map's reference does not free the data under a view that still holds one;
        // that view keeps the buffers it was built against until it releases.
        mGpuSimDataByScene.erase(it);
    }

    GpuSimulationDataPtr data = std::make_shared<GpuSimulationData>(*this, attachHandle);
    if (!data->init(scene))
        return nullptr; // nothing was inserted, so there is nothing to roll back
    mGpuSimDataByScene[key] = data;
    return data;
}

bool SimulationBackend::sceneCacheEntryValid(const SceneCacheEntry& entry, ::physx::PxScene* scene)
{
    if (entry.scene != scene)
        return false;
    if (entry.gpuPipeline != scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return false;
    if (entry.ctxMgr != scene->getCudaContextManager())
        return false;

    // A cached view whose scene was deleted has been invalidated in place (REQ-TENSOR-VIEW-001):
    // its simulation data is torn down and its scene pointer nulled, so hasScene() stops matching.
    // Handing that view back crashes on first use, and the checks above cannot see it -- a later
    // scene reusing the address passes all of them. This is the authoritative test, and unlike the
    // destruction notification it does not depend on the subscription having been established.
    if (entry.supersetView && !static_cast<BaseSimulationView*>(entry.supersetView)->hasScene(scene))
        return false;

    // The view invalidates itself when an actor, articulation, link or shape it holds is destroyed
    // (BaseSimulationView's destruction subscription, membership-checked per type). That is the
    // mechanism ADR-0008 Decision 3 names, and it is what catches a removal the topology probe
    // below cannot see -- a destroy and create in one step nets to the same counts.
    if (entry.supersetView && !entry.supersetView->getValid())
        return false;

    // The simulation data has no resize path: every buffer is sized in init() from these, and
    // mActor2RdIndexMap is built from the actor set they describe. A recycled PxScene* shows up
    // here as a topology mismatch -- a rigid-only entry reused for a scene with articulations
    // leaves mLinkOrRootTransformsDev unallocated.
    //
    // Skipped while the database still reports the epoch this topology was measured under: nothing
    // enters or leaves the scene without creating or removing a record, so an unchanged epoch means
    // an unchanged actor and articulation set. The O(articulations) walk otherwise runs on EVERY
    // acquire. The epoch is the one fact that comes from outside the snapshot -- a cached structural
    // property (homogeneity, prim paths, metatype set) describes the scene as it WAS.
    if (entry.dbEpoch != 0 && entry.dbEpoch == internal::recordLifetimeEpoch())
        return true;

    SceneTopology now;
    sceneTopology(scene, now);
    return entry.topology == now;
}

void SimulationBackend::takeOwnedViews(std::vector<ISimulationView*>& out)
{
    // The backend owns the cached superset views, so they are detached here and destroyed after the
    // lock is dropped.
    for (auto& [key, entry] : mSceneCache)
        takeSceneCacheViews(entry, out);
    mSceneCache.clear();

    // Views detached by a destruction notification and not yet drained. The only other drain is the
    // next acquire, so without this a scene destroyed and then never read again leaves its superset
    // views -- and the device allocations they own -- alive for the process lifetime.
    out.insert(out.end(), mDeferredViewReleases.begin(), mDeferredViewReleases.end());
    mDeferredViewReleases.clear();
}

void SimulationBackend::takeSceneCacheViews(SceneCacheEntry& entry, std::vector<ISimulationView*>& out)
{
    if (entry.supersetView)
        out.push_back(entry.supersetView);
    entry.supersetView = nullptr; // the superset rigid/articulation views are its children
}

void SimulationBackend::invalidateSceneCache(::physx::PxScene* scene)
{
    std::vector<ISimulationView*> pendingRelease;
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);
        const SceneKey key{ attachOwning(scene), scene };
        const std::unordered_map<SceneKey, SceneCacheEntry, SceneKeyHash>::iterator it = mSceneCache.find(key);
        if (it != mSceneCache.end())
        {
            takeSceneCacheViews(it->second, pendingRelease);
            mSceneCache.erase(it);
        }
        // The simulation data too, not just the entry. What goes stale is the data's actor map and
        // the sizes its buffers were built from; rebuilding the entry around the SAME data cannot
        // repair that, and the caller would retry against an identically stale view forever.
        // Dropping the shared_ptr does not free it under a view that still holds one.
        //
        // Only this scene's. The attach's CpuSimulationData is deliberately kept: nothing in it is
        // derived from a scene, so it cannot go stale for a scene reason, and dropping it here
        // would leave a sibling scene's cached entry holding the old instance while the next
        // acquire built a second one -- two contact-bucket registries behind one attach.
        mGpuSimDataByScene.erase(key);
    }
    // Outside the lock: releasing a view destroys it, and ~BaseSimulationView calls back into
    // removeSimulationView(), which takes mStageDataMutex.
    for (ISimulationView* v : pendingRelease)
        v->release(true);
}


SimulationBackend::SceneCacheEntry* SimulationBackend::ensureSceneCacheEntryLocked(
    ::physx::PxScene* scene, std::vector<ISimulationView*>& pendingRelease)
{
    // Guarded here rather than at each caller: everything below dereferences `scene`.
    if (!scene)
        return nullptr;

    const AttachHandle attachHandle = attachOwning(scene);
    const SceneKey key{ attachHandle, scene };

    // Views detached by onSceneDestroyed: their scene is gone, so releasing them is safe now and
    // the caller does it after dropping the lock.
    if (!mDeferredViewReleases.empty())
    {
        pendingRelease.insert(pendingRelease.end(), mDeferredViewReleases.begin(), mDeferredViewReleases.end());
        mDeferredViewReleases.clear();
    }

    const bool useGpuPipeline = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    SceneCacheEntry& entry = mSceneCache[key];
    if (!sceneCacheEntryValid(entry, scene))
    {
        // Stale or first use: drop whatever was held and rebuild. Dropping the shared_ptr does not
        // free the data out from under a view that still holds it -- the last reference wins. The
        // entry's cached superset view is detached for release by the caller, not leaked.
        takeSceneCacheViews(entry, pendingRelease);
        entry = SceneCacheEntry{};
        // A rebuild invalidates everything a consumer derived from the previous entry.
        // Process-wide, not per-backend: a consumer caching against this number can outlive the
        // backend that issued it, and a counter restarting at zero with a recycled scene address
        // hands that consumer a match for data describing a scene that no longer exists.
        static std::atomic<uint64_t> sGeneration{ 0 };
        entry.generation = ++sGeneration;
        entry.scene = scene;
        entry.ctxMgr = scene->getCudaContextManager();
        entry.gpuPipeline = useGpuPipeline;
        // Sampled before the walk: taking it after would seal in an epoch that a mutation racing
        // the walk had already invalidated, so the entry would claim to describe a scene it never
        // measured. Sampling first can only cost an extra rebuild.
        entry.dbEpoch = internal::recordLifetimeEpoch();
        sceneTopology(scene, entry.topology);

        // This scene's simulation data, NOT a copy of it: every consumer of the scene shares the
        // one instance. It carries the attach handle, which is nonzero for every live attach
        // (ADR-0013), so the data can resolve its own attach -- that is what makes the authored
        // GPU-contact capacity reachable on the ovstage path.
        if (useGpuPipeline)
        {
            GpuSimulationDataPtr gpuData = ensureGpuSimDataLocked(attachHandle, scene);
            if (!gpuData)
            {
                CARB_LOG_ERROR("Failed to initialize GPU simulation data for stageless scene view");
                mSceneCache.erase(key);
                return nullptr;
            }
            entry.gpuData = gpuData;
        }
        else
        {
            CpuSimulationDataPtr& cpuData = mCpuSimDataByAttach[attachHandle];
            if (!cpuData)
                cpuData = std::make_shared<CpuSimulationData>(*this, attachHandle);
            entry.cpuData = cpuData;
        }
    }
    // No eviction policy, because there is nothing to bound: an entry holds the superset view and
    // its row maps, the scene's simulation data is shared rather than copied, and onSceneDestroyed
    // drops the entry when the scene goes. The live scene count IS the size.
    // Returns the entry just validated or built, not a fresh lookup: mSceneCache[key] would
    // re-insert a default-constructed entry if any path above had erased it, and hand the caller a
    // non-null pointer to it -- straight through the `if (!entryPtr)` guard.
    return &entry;
}

ISimulationView* SimulationBackend::createSimulationViewForScene(::physx::PxScene* scene)
{
    if (!scene)
    {
        CARB_LOG_ERROR("createSimulationViewForScene: null scene");
        return nullptr;
    }

    // The view is per-call; the simulation data behind it is cached per scene and shared by
    // shared_ptr. Views are tracked under mViewsByAttach so reset()/resetStage() can still reach them.

    const bool useGpuPipeline = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    std::vector<ISimulationView*> pendingRelease;
    ISimulationView* result = [&]() -> ISimulationView*
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);

        SceneCacheEntry* entryPtr = ensureSceneCacheEntryLocked(scene, pendingRelease);
        if (!entryPtr)
            return nullptr; // anything detached is still released below
        SceneCacheEntry& entry = *entryPtr;

        if (useGpuPipeline)
        {
            // Null UsdStage: the rigid/articulation read path never dereferences it, and the
            // pattern matchers (which would) are bypassed -- entries come from an actor set.
            GpuSimulationView* simView = new GpuSimulationView(attachedStageFor(scene), scene, entry.gpuData);
            mViewsByAttach[attachOwning(scene)].push_back(simView);
            return simView;
        }
        CpuSimulationView* simView = new CpuSimulationView(attachedStageFor(scene), scene, entry.cpuData);
        mViewsByAttach[attachOwning(scene)].push_back(simView);
        return simView;
    }();

    // Outside the lock: ~BaseSimulationView calls back into removeSimulationView().
    for (ISimulationView* v : pendingRelease)
        v->release(true);
    return result;
}

CpuSimulationView* SimulationBackend::acquireCpuSceneView(PxScene* scene, uint64_t* outGeneration)
{
    if (outGeneration)
        *outGeneration = 0;
    if (!scene || scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return nullptr;

    std::vector<ISimulationView*> pendingRelease;
    CpuSimulationView* result = [&]() -> CpuSimulationView*
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);
        SceneCacheEntry* entryPtr = ensureSceneCacheEntryLocked(scene, pendingRelease);
        if (!entryPtr)
            return nullptr;
        SceneCacheEntry& entry = *entryPtr;

        if (!entry.supersetView)
        {
            CpuSimulationView* sv = new CpuSimulationView(attachedStageFor(scene), scene, entry.cpuData,
                                                         /*notifyWhenSimStopped=*/true);
            mViewsByAttach[attachOwning(scene)].push_back(sv);
            entry.supersetView = sv;
        }

        if (outGeneration)
            *outGeneration = entry.generation;
        // Safe: the pipeline gate above is what admits an entry here, and entry.supersetView is only
        // ever built by the matching branch.
        return static_cast<CpuSimulationView*>(entry.supersetView);
    }();

    for (ISimulationView* v : pendingRelease)
        v->release(true);
    return result;
}

GpuSimulationView* SimulationBackend::acquireSceneView(PxScene* scene, uint64_t* outGeneration)
{
    if (outGeneration)
        *outGeneration = 0;
    if (!scene || !scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return nullptr;

    std::vector<ISimulationView*> pendingRelease;
    GpuSimulationView* result = [&]() -> GpuSimulationView*
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);
        // Straight to the validated entry: constructing a throwaway view here would cost a
        // subscribeObjectChangeNotifications / unsubscribe round-trip on every read.
        SceneCacheEntry* entryPtr = ensureSceneCacheEntryLocked(scene, pendingRelease);
        if (!entryPtr)
            return nullptr;
        SceneCacheEntry& entry = *entryPtr;

        if (!entry.supersetView)
        {
            GpuSimulationView* sv = new GpuSimulationView(attachedStageFor(scene), scene, entry.gpuData,
                                                         /*notifyWhenSimStopped=*/true);
            mViewsByAttach[attachOwning(scene)].push_back(sv);
            entry.supersetView = sv;
        }

        if (outGeneration)
            *outGeneration = entry.generation;
        // Safe: the GPU gate above is what admits an entry here at all, and entry.supersetView is
        // only ever built by the matching branch.
        return static_cast<GpuSimulationView*>(entry.supersetView);
    }();

    for (ISimulationView* v : pendingRelease)
        v->release(true);
    return result;
}

bool SimulationBackend::sceneMayHaveDisabledRigidDynamics(PxScene* scene)
{
    if (!scene)
        return false;

    std::lock_guard<std::mutex> lock(mStageDataMutex);
    const std::unordered_map<SceneKey, GpuSimulationDataPtr, SceneKeyHash>::const_iterator it =
        mGpuSimDataByScene.find(SceneKey{ attachOwning(scene), scene });
    if (it == mGpuSimDataByScene.end() || !it->second)
        return true; // unknown scene -- say "maybe" so the caller scans rather than trusts us
    return it->second->mMayHaveDisabledRd;
}

void SimulationBackend::removeSimulationView(ISimulationView* view)
{
    std::lock_guard<std::mutex> lock(mStageDataMutex);

    // Remove from the per-attach index. Linear scan is acceptable: views per attach
    // are few and removeSimulationView is not on the hot path.
    for (auto& [handle, views] : mViewsByAttach)
    {
        auto it = std::find(views.begin(), views.end(), view);
        if (it != views.end())
        {
            views.erase(it);
            return;
        }
    }
}

void SimulationBackend::reset()
{
    std::vector<ISimulationView*> pendingRelease;
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);

        // The backend owns the cached superset views and the device allocations under them, so a
        // reset that clears the per-attach data without this leaves them alive for the process
        // lifetime -- and pointing at simulation data that has just been dropped.
        takeOwnedViews(pendingRelease);

        mCpuSimDataByAttach.clear();
        mGpuSimDataByScene.clear();

        for (auto& [handle, views] : mViewsByAttach)
            for (auto* view : views)
                view->invalidate();
        mViewsByAttach.clear();

        mManualStepCount = 0;
    }
    // Outside the lock: ~BaseSimulationView calls back into removeSimulationView().
    for (ISimulationView* v : pendingRelease)
        v->release(true);
}

void SimulationBackend::resetStage(AttachHandle attachHandle)
{
    if (attachHandle == omni::physics::tensors::kNoAttach)
        return;

    if (attachHandle == omni::physics::tensors::kActiveAttach)
    {
        if (!g_physxSimulation)
            return;
        attachHandle = g_physxSimulation->getAttachHandle();
        if (attachHandle == omni::physics::tensors::kNoAttach)
            return;
    }

    std::vector<ISimulationView*> pendingRelease;
    {
        std::lock_guard<std::mutex> lock(mStageDataMutex);

        // Every one of this attach's cached superset views has to go with the data it reads --
        // all of them, since the attach may hold several scenes. Without this a Play/Stop cycle
        // leaves an entry pointing at simulation data this function is about to erase, and the next
        // acquire for a recycled attach handle finds it. Detached under the lock, released below it.
        for (std::unordered_map<SceneKey, SceneCacheEntry, SceneKeyHash>::iterator cacheIt = mSceneCache.begin();
             cacheIt != mSceneCache.end();)
        {
            if (cacheIt->first.attach == attachHandle)
            {
                takeSceneCacheViews(cacheIt->second, pendingRelease);
                cacheIt = mSceneCache.erase(cacheIt);
            }
            else
            {
                ++cacheIt;
            }
        }

        // Invalidate and remove all views for this attach before releasing the data.
        // Views hold shared_ptrs to the data AND a borrowed AttachedStage, so this must
        // run before the attach is torn down.
        auto viewIt = mViewsByAttach.find(attachHandle);
        if (viewIt != mViewsByAttach.end())
        {
            for (auto* view : viewIt->second)
                view->invalidate();
            mViewsByAttach.erase(viewIt);
        }

        mCpuSimDataByAttach.erase(attachHandle);
        for (std::unordered_map<SceneKey, GpuSimulationDataPtr, SceneKeyHash>::iterator it = mGpuSimDataByScene.begin();
             it != mGpuSimDataByScene.end();)
        {
            if (it->first.attach == attachHandle)
            {
                it = mGpuSimDataByScene.erase(it);
            }
            else
            {
                ++it;
            }
        }
        // mManualStepCount intentionally NOT cleared here: it is a process-wide
        // accumulator for steps driven outside the Kit event loop (incremented via
        // BaseSimulationView::incrementStepCount). It offsets getTimestamp()/getStepCount()
        // for ALL attaches and must remain monotonic across per-attach resets.
    }
    // Outside the lock: ~BaseSimulationView calls back into removeSimulationView().
    for (ISimulationView* v : pendingRelease)
        v->release(true);
}

PxScene* SimulationBackend::findPhysicsScene(const usdparser::AttachedStage& attachedStage) const
{
    // try our private backdoor first...
    if (g_physxPrivate)
    {
        return g_physxPrivate->getPhysXScene();
    }

    if (!g_physx)
    {
        return nullptr;
    }

    // Plain strings + the pxr-free keyFor(std::string_view) overload (byte-identical
    // behavior to keyFor(SdfPath) under a real USD backend -- see AttachedStage.h), so this
    // tier is pxr-free.
    static const std::vector<std::string> quickPaths
    {
        "/physicsScene",
        "/World/physicsScene",
        "/World/PhysicsScene", // Isaac Sim authoring convention (capital P)
    };

    // check low-hanging fruit before full blown search
    for (const std::string& path : quickPaths)
    {
        PxScene* scene = static_cast<PxScene*>(BaseSimulationView::resolvePhysXPtr(&attachedStage, attachedStage.keyFor(path), omni::physx::ePTScene));
        if (scene)
        {
            return scene;
        }
    }

    // Ask the internal object DB which paths carry a scene, instead of traversing the
    // whole stage looking for UsdPhysicsScene prims. It answers under any parse backend
    // (including a stageless attach) and only visits paths that actually produced a
    // scene, so it is also cheaper than the traversal it replaces.
    const usdparser::ObjectDb* objectDb = attachedStage.getObjectDatabase();
    if (!objectDb)
    {
        return nullptr;
    }

    const usdparser::ObjectCategory sceneCategory(usdparser::eScene);
    // getKeyMap() is ObjectDb's ObjectKey-keyed enumeration, populated via
    // findOrCreateEntry (see LoadTools.h). Iteration order is unspecified (backed by an
    // unordered_map), so sort by display text (AttachedStage::textViewFor) so that
    // when more than one scene is attached and none is at a quick path, the
    // lexicographically smallest scene path wins, deterministically, under either
    // backend.
    std::vector<omni::physics::parse::ObjectKey> sceneKeys;
    for (const auto& [key, entries] : objectDb->getKeyMap())
    {
        if (entries.find(sceneCategory) != entries.end())
        {
            sceneKeys.push_back(key);
        }
    }
    std::sort(sceneKeys.begin(), sceneKeys.end(),
             [&attachedStage](omni::physics::parse::ObjectKey a, omni::physics::parse::ObjectKey b)
             {
                 return attachedStage.textViewFor(a) < attachedStage.textViewFor(b);
             });

    for (omni::physics::parse::ObjectKey key : sceneKeys)
    {
        PxScene* scene = static_cast<PxScene*>(BaseSimulationView::resolvePhysXPtr(&attachedStage, key, omni::physx::ePTScene));
        if (scene)
        {
            return scene;
        }
    }

    return nullptr;
}

void SimulationBackend::prePhysicsUpdate()
{
    // TODO: make it a setting
    bool enableAutoFlush = false;
    if (!enableAutoFlush)
        return;

    std::lock_guard<std::mutex> lock(mStageDataMutex);
    for (auto& kv : mGpuSimDataByScene)
    {
        if (kv.second)
            kv.second->flush();
    }
}

int64_t SimulationBackend::getTimestamp() const
{
    if (g_physxSimulation)
    {
        return static_cast<int64_t>(g_physxSimulation->getSimulationTimestamp() + mManualStepCount);
    }
    return 0;
}

int64_t SimulationBackend::getStepCount() const
{
    if (g_physxSimulation)
    {
        return static_cast<int64_t>(g_physxSimulation->getSimulationStepCount() + mManualStepCount);
    }
    return 0;
}

}
}
}
