// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-1, AC-2, AC-5, AC-6, AC-7, AC-8
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-1, AC-2, AC-5, AC-9
 *
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-1, AC-2, AC-3, AC-4b, AC-4c, AC-4d, AC-5, AC-6
 *
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-5, AC-6, AC-7, AC-8, AC-9
 *
 * @implements REQ-READ-TENDON-001
 * @covers AC-1, AC-2, AC-3, AC-5, AC-6, AC-7
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-1, AC-2, AC-3, AC-4, AC-6, AC-8, AC-9, AC-10, AC-11, AC-13, AC-15, AC-16, AC-17, AC-19
 *
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-2, AC-5, AC-6, AC-7
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-1, AC-2, AC-3, AC-4, AC-5, AC-6, AC-7, AC-8, AC-9
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-1, AC-2, AC-3, AC-4, AC-5, AC-6, AC-9
 *
 * @implements REQ-READ-MATERIAL-001
 * @covers AC-1, AC-2, AC-3, AC-4, AC-5, AC-6, AC-7
 *
 * @implements REQ-READ-POOL-001
 * @covers AC-1, AC-3, AC-4, AC-5, AC-6
 */

#include <omni/physx/IOvxPhysicsRead.h>

#include "OvxPhysicsShared.h" // object enumeration shared with the write

#include <OvstageOutput.h> // buildPathList (no-string list); private to omni.physics.ovstage


#include "OmniPhysX.h"
#include "PhysXScene.h"

#include <omni/physx/IPhysxSettings.h> // kSettingOvstageReadPoolMaxMB
#include <carb/settings/ISettings.h>
#include "PhysXTools.h" // getWorldTransform, PhysXType
#include "internal/Internal.h"
#include "internal/InternalPhysXDatabase.h"
#include "internal/InternalActor.h"
#include "internal/InternalScene.h"
#include "internal/InternalDeformable.h"
#include "internal/InternalParticle.h"

#include <cudamanager/PxCudaContextManager.h>
#include <cudamanager/PxCudaContext.h>
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

// ADR-0008: source rigid output from the tensor backend's bulk PxDirectGPUAPI
// reads (device-resident, correct under suppressReadback) via a stageless, actor-set view.
#include "tensors/SimulationBackend.h"
#include "tensors/GlobalsAreBad.h"
#include "tensors/OvStageRowsVersion.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/GpuArticulationView.h"
#include "tensors/gpu/GpuRigidBodyView.h" // fused ovstage column read (fill + gather, no host block)
#include "tensors/cpu/CpuRigidBodyView.h" // host counterpart of the fused column gather
#include "tensors/cpu/CpuSimulationView.h"
#include "tensors/base/BasePointInstancerView.h" // the one reframe, shared with the kernel
#include "tensors/gpu/GpuPointInstancerView.h" // device reframe for point-instancer columns
#include "tensors/cpu/CpuPointInstancerView.h" // host reframe, the write's scatter target
#include "tensors/gpu/GpuPointSetReadView.h" // device gather for deformable sim-mesh columns
#include "tensors/cpu/CpuArticulationView.h"
#include "tensors/gpu/CudaKernels.h" // fetchArtiDofAttributeOvStage (joint DOF gather)
#include "tensors/base/BaseSimulationView.h"
#include "tensors/base/BaseVehicleView.h" // ovstage vehicle wheel transforms (host-only)
#include "tensors/VehicleWheelOvStageRecord.h"
#include "tensors/CommonTypes.h"
#include <omni/physics/tensors/TensorDesc.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/IArticulationView.h>

#include <memory>

#include <omni/physx/IPhysx.h> // PhysXType (ePTActor / ePTLink / ...)

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace ::physx;
using namespace omni::physx;          // OmniPhysX, OvxAttr, kOvx* scopes, ePT* enum
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;
using namespace omni::physx::ovx; // scanRigidRecords / bucketForScene / allPhysicsScenes / ...

namespace
{


// ----------------------------------------------------------------------------
// Session storage
// ----------------------------------------------------------------------------


// DEVICE columns only. They are sub-allocated from one buffer and each start is handed out as a
// DLTensor data pointer, which dlpack.h requires to be 256-byte aligned; padding every column up to
// that keeps each start aligned.
//
// Host columns get no such treatment and are not covered by this: they point into ordinary
// std::vector<float/int32_t/uint8_t> storage in GroupStore, whose alignment is whatever the
// allocator gives.
//
// That is deliberate, not an oversight. dlpack.h states the 256-byte requirement and then says of
// it: "multiple libraries (CuPy, PyTorch, TensorFlow, TVM, perhaps others) do not adhere to this
// 256 byte alignment requirement on CPU/CUDA/ROCm, and always use byte_offset=0 ... it is
// recommended to not rely on the data pointer being correctly aligned." Over-aligning host storage
// would put an over-aligned allocator on every host column to honour a guarantee the spec itself
// tells consumers not to depend on, and that every major consumer already ignores.
//
// So the contract is scoped to the device path rather than the storage widened to meet it. Do not
// restate this invariant as if it held for both.
constexpr size_t kColumnAlignFloats = 64; // 256 bytes
inline size_t alignColumnFloats(size_t floats)
{
    return ((floats + kColumnAlignFloats - 1) / kColumnAlignFloats) * kColumnAlignFloats;
}

// One typed column group, with the scratch the DLTensor aliases living here so it
// outlives the fetch. tensor_count is always 1 (a fixed group stacks its prims in
// one tensor; an array group is emitted one-per-prim, so one prim → one tensor).
struct GroupStore
{
    std::vector<float>    floats;  // SoA scratch: rows * comp (host path, float32 columns)
    std::vector<int32_t> ints; // Host scratch for int32 columns such as shapeCount.
    // Byte-wide host scratch for a column whose element type is not float32 -- today the two actor flags, which
    // the tensor API reports as uint8. Exactly one of `floats`, `ints` and `bytes` is filled per group.
    std::vector<uint8_t>  bytes;
    // Element type of the emitted tensor; lanes are patched from the column width at finalize.
    // Defaults to float32, so a builder that does not care says nothing.
    DLDataType            dtype{ kDLFloat, 32, 1 };
    void*                 deviceData = nullptr; // device SoA buffer (GPU path); owned by the ReadSession
    int deviceOrdinal = -1; // -1 = host scratch; >=0 = kDLCUDA device ordinal
    // CUDA context deviceData was allocated in; selects the completion event this group publishes.
    PxCudaContextManager* ctxMgr = nullptr;
    // One entry per tensor. A fixed group stacks every prim into one tensor and so has exactly one;
    // an ARRAY group is one tensor per prim (ovstage's shape -- see docs/ovstage_integration.md), so
    // a per-prim attribute belongs in ONE group carrying N of them, not N groups carrying one each.
    //
    // shapes is parallel to tensors and separate from it because each DLTensor::shape must point at
    // storage that outlives the fetch; patched to point into this vector when the group is handed
    // over.
    std::vector<int64_t>  shapes;  // [rows] per tensor
    std::vector<DLTensor> tensors;
    std::vector<uint32_t> indexMap; // array-slot scatter (instancer active subset); empty = identity
    ovx_primpath_list_t   list = OVX_INVALID_PRIMPATH_LIST;
    uint32_t              primCount = 0; // prims this group covers (fixed: rows; array: 1)
    ovx_token_t           attribute = OVX_INVALID_TOKEN;
    bool                  isArray = false;
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
};

// A per-CUDA-context pool of released scratch buffers -- output columns (device
// memory) and pinned host staging -- reused across reads instead of freed and re-allocated every time.
// A buffer ENTERS at ovxReleaseRead, after this session's completion event has been waited (the exact
// point it was freed before), so no consumer can still hold it, and LEAVES at the next read whose
// request its size covers.
//
// Both kinds sit under one context entry with ONE manager reference: a pooled pointer is only freeable
// through the manager that produced it, so the pool holds a reference on each manager it has any buffer
// for -- the pointer never outlives the manager, and pinning it also stops its address being recycled
// under a still-pooled entry. The reference drops only when the context's pool is flushed, which
// happens once per read for every pooled context no longer backed by a live scene
// (poolFlushStaleContexts), never on a plain acquire, so a warm read/release cycle keeps exactly one
// reference. A detached or destroyed context is reclaimed on the next read; at process exit the pool
// goes with the other globals.
struct PooledColumn
{
    uintptr_t ptr = 0; // CUdeviceptr for device buffers, host void* for pinned -- both held as uintptr_t
    size_t bytes = 0;
};

struct ContextPool
{
    std::vector<PooledColumn> deviceFree; // kDLCUDA output columns, freed with memFree
    std::vector<PooledColumn> pinnedFree; // pinned host staging, freed with memFreeHost
    size_t pooledBytes = 0;               // device + pinned bytes currently retained; bounded by the budget below
    bool refHeld = false;                 // one manager reference, held while this context has pooled anything
};

std::mutex g_poolMutex;
std::unordered_map<::physx::PxCudaContextManager*, ContextPool> g_pool;

// Set while the runtime tears the pool down for shutdown (ovxDrainColumnPools): a session that returns
// a buffer during teardown then FREES it rather than re-pooling into a pool that is about to be
// destroyed and whose managers releasePhysics() is about to release. Reset on the next read.
bool g_poolTeardown = false;

// The memory the pool may RETAIN per context, from /physics/ovstageReadPoolMaxMB (<= 0 DISABLES the
// pool, default 256 MiB). With the pool enabled a read allocates the columns it needs and the pool
// normally hands them back on the next read instead of freeing them -- a large win for the columns read
// every step -- but retaining them costs device memory. Once a context's pooled buffers reach this
// budget a further released buffer is freed rather than kept, which is no worse than before the pool
// existed. A value of 0 or less turns retention off entirely: every column is freed on release and
// allocated fresh on the next read, exactly as before the pool existed. Read per release (a cached
// settings lookup) so a runtime change takes effect immediately.
//
// Returns the retention budget in bytes, or 0 meaning DISABLED -- the release paths read 0 as "retain
// nothing" and free rather than pool.
size_t poolMaxBudgetBytes()
{
    carb::settings::ISettings* const settings = OmniPhysX::getInstance().getISettings();
    const int32_t maxMiB = settings ? settings->getAsInt(kSettingOvstageReadPoolMaxMB) : 256;
    // 0 or negative -> 0 bytes, which the release paths read as "pool disabled" (free, do not retain).
    return maxMiB > 0 ? (static_cast<size_t>(maxMiB) << 20) : 0;
}

// Best-fit over one free list -- the smallest buffer that still covers `bytes`, so a run of same-size
// reads recycles tightly rather than handing a large buffer to a small request and forcing the next
// large one to allocate. 0 on a miss (the caller allocates). On a hit `capacityOut` is the buffer's
// ACTUAL size, which the caller must carry forward so it re-enters the pool at its true capacity rather
// than shrinking to whatever request last borrowed it. Caller holds g_poolMutex.
uintptr_t poolAcquireFrom(std::vector<PooledColumn>& freeList, size_t bytes, size_t& capacityOut)
{
    size_t best = freeList.size();
    for (size_t i = 0; i < freeList.size(); ++i)
        if (freeList[i].bytes >= bytes && (best == freeList.size() || freeList[i].bytes < freeList[best].bytes))
            best = i;
    if (best == freeList.size())
        return 0;
    const uintptr_t ptr = freeList[best].ptr;
    capacityOut = freeList[best].bytes;
    freeList[best] = freeList.back();
    freeList.pop_back();
    // The reference is NOT dropped when this empties the list: buffers now checked out to a live read
    // will be returned, and the manager must stay alive until they are. Only flush drops it.
    return ptr;
}

uintptr_t poolAcquireColumn(::physx::PxCudaContextManager* mgr, size_t bytes, size_t& capacityOut)
{
    if (!mgr || bytes == 0)
        return 0;
    std::lock_guard<std::mutex> lock(g_poolMutex);
    std::unordered_map<::physx::PxCudaContextManager*, ContextPool>::iterator it = g_pool.find(mgr);
    if (it == g_pool.end())
        return 0;
    const uintptr_t ptr = poolAcquireFrom(it->second.deviceFree, bytes, capacityOut);
    if (ptr)
        it->second.pooledBytes -= capacityOut; // no longer retained
    return ptr;
}

uintptr_t poolAcquirePinned(::physx::PxCudaContextManager* mgr, size_t bytes, size_t& capacityOut)
{
    if (!mgr || bytes == 0)
        return 0;
    std::lock_guard<std::mutex> lock(g_poolMutex);
    std::unordered_map<::physx::PxCudaContextManager*, ContextPool>::iterator it = g_pool.find(mgr);
    if (it == g_pool.end())
        return 0;
    const uintptr_t ptr = poolAcquireFrom(it->second.pinnedFree, bytes, capacityOut);
    if (ptr)
        it->second.pooledBytes -= capacityOut; // no longer retained
    return ptr;
}

// Take the pool's single manager reference on this context's first pooled buffer. Caller holds
// g_poolMutex and the ContextPool reference it passes.
void poolTakeReference(::physx::PxCudaContextManager* mgr, ContextPool& pool)
{
    if (!pool.refHeld)
    {
        mgr->acquireReference(); // the pool now owes this manager a free; keep it alive until flush
        pool.refHeld = true;
    }
}

// Enforce the retention budget on the ALREADY-retained set, not just on admission. The budget is
// read per release, so a budget LOWERED at runtime (256 MiB -> 64 MiB) must trim what earlier releases
// retained under the higher one -- the admit check alone is only a growth limit, which would leave the
// old bytes retained until the context is swept. Frees returned buffers only (the free lists), never a
// checked-out one, so it is safe at any time. Evicts arbitrarily within each list: this enforces the
// bound, it does NOT choose cold vs hot blocks to keep a phase-changed working set warm -- that is a
// separate eviction policy, deliberately out of scope. Caller holds g_poolMutex.
void poolTrimToBudget(ContextPool& pool, ::physx::PxCudaContext* cu, size_t budget)
{
    while (pool.pooledBytes > budget && !pool.deviceFree.empty())
    {
        const PooledColumn c = pool.deviceFree.back();
        pool.deviceFree.pop_back();
        pool.pooledBytes -= c.bytes;
        if (c.ptr)
            cu->memFree(static_cast<CUdeviceptr>(c.ptr));
    }
    while (pool.pooledBytes > budget && !pool.pinnedFree.empty())
    {
        const PooledColumn c = pool.pinnedFree.back();
        pool.pinnedFree.pop_back();
        pool.pooledBytes -= c.bytes;
        if (c.ptr)
            cu->memFreeHost(reinterpret_cast<void*>(c.ptr));
    }
}

// Return a device column to the pool, or free it outright when the pool is full or the context is
// unknown. The caller has already waited on the completion event, so the buffer is idle.
void poolReleaseColumn(::physx::PxCudaContextManager* mgr, ::physx::PxCudaContext* cu, uintptr_t ptr, size_t bytes)
{
    if (!ptr)
        return;
    const size_t budget = poolMaxBudgetBytes();
    if (!mgr || !cu || bytes == 0)
    {
        // Context unusable: free outright -- pooling and trimming both need a live context. A DISABLED
        // pool (budget 0) is deliberately NOT short-circuited here: it flows to the normal path so the
        // trim below empties the already-retained lists ("0 retains nothing"), not just this buffer.
        if (cu)
            cu->memFree(static_cast<CUdeviceptr>(ptr));
        return;
    }
    std::lock_guard<std::mutex> lock(g_poolMutex);
    if (g_poolTeardown)
    {
        cu->memFree(static_cast<CUdeviceptr>(ptr)); // pool tearing down for shutdown: free, do not re-pool
        return;
    }
    // find(), not operator[]: a rejected (over-budget or disabled) return must NOT default-insert an
    // empty entry. An entry with refHeld == false is a manager the pool holds a raw pointer to but
    // never took a reference on, so a later stale sweep would dereference it after the manager was
    // destroyed. The insert happens only on the retain path below, together with the reference.
    const std::unordered_map<::physx::PxCudaContextManager*, ContextPool>::iterator it = g_pool.find(mgr);
    // Trim the retained set to the current budget before deciding this release; the admit check below
    // only limits growth. With budget 0 (pool disabled at runtime) this empties the free lists, so a
    // disable frees what earlier releases retained under a higher budget, not just this buffer -- the
    // manager reference is dropped later by the stale sweep, which is the only point that can prove no
    // checked-out buffer still needs the context alive.
    if (it != g_pool.end() && it->second.pooledBytes > budget)
        poolTrimToBudget(it->second, cu, budget);
    const size_t held = (it != g_pool.end()) ? it->second.pooledBytes : 0;
    if (held + bytes > budget)
    {
        cu->memFree(static_cast<CUdeviceptr>(ptr)); // over the byte budget: do not retain, no entry
        return;
    }
    ContextPool& pool = g_pool[mgr];
    poolTakeReference(mgr, pool);
    pool.deviceFree.push_back({ ptr, bytes });
    pool.pooledBytes += bytes;
}

// The pinned-host counterpart: same rules, freed with memFreeHost rather than memFree.
void poolReleasePinned(::physx::PxCudaContextManager* mgr, ::physx::PxCudaContext* cu, uintptr_t ptr, size_t bytes)
{
    if (!ptr)
        return;
    const size_t budget = poolMaxBudgetBytes();
    if (!mgr || !cu || bytes == 0)
    {
        // Context unusable: free outright. A DISABLED pool (budget 0) flows to the normal path so the
        // trim below empties the already-retained lists -- see poolReleaseColumn.
        if (cu)
            cu->memFreeHost(reinterpret_cast<void*>(ptr));
        return;
    }
    std::lock_guard<std::mutex> lock(g_poolMutex);
    if (g_poolTeardown)
    {
        cu->memFreeHost(reinterpret_cast<void*>(ptr)); // pool tearing down for shutdown: free, do not re-pool
        return;
    }
    // find(), not operator[]: see poolReleaseColumn -- a rejected return must not leave an empty,
    // reference-less entry for the stale sweep to dereference after the manager is gone.
    const std::unordered_map<::physx::PxCudaContextManager*, ContextPool>::iterator it = g_pool.find(mgr);
    if (it != g_pool.end() && it->second.pooledBytes > budget)
        poolTrimToBudget(it->second, cu, budget); // enforce a possibly-lowered budget (see poolReleaseColumn)
    const size_t held = (it != g_pool.end()) ? it->second.pooledBytes : 0;
    if (held + bytes > budget)
    {
        cu->memFreeHost(reinterpret_cast<void*>(ptr)); // over the byte budget: do not retain, no entry
        return;
    }
    ContextPool& pool = g_pool[mgr];
    poolTakeReference(mgr, pool);
    pool.pinnedFree.push_back({ ptr, bytes });
    pool.pooledBytes += bytes;
}

// A context's pooled buffers, lifted out of the pool so they can be freed after the pool lock is
// dropped. memFree/memFreeHost need the manager's OWN CUDA context current, so they run under that
// manager's CUDA lock; taking that lock under the pool lock would invert the order the read paths use
// (CUDA lock first). Move the free lists out under the pool lock, release it, then free.
struct ReclaimedPool
{
    ::physx::PxCudaContextManager* mgr = nullptr;
    std::vector<PooledColumn> deviceBuffers;
    std::vector<PooledColumn> pinnedBuffers;
    bool refHeld = false;
};

// Free every buffer in each reclaimed pool under its manager's CUDA lock, then drop the reference the
// pool held on that manager. Safe at any time: the pool holds only released buffers, so nothing a
// consumer still holds is freed.
void freeReclaimedPools(std::vector<ReclaimedPool>& reclaimed)
{
    for (ReclaimedPool& r : reclaimed)
    {
        // Dereference the manager only if the pool owned a reference on it. An entry with
        // refHeld == false never pinned the manager (a reference is taken with the first retained
        // buffer, before any buffer is stored), so it may already be destroyed -- getCudaContext()
        // on it would be a use-after-free -- and it holds no buffers to free anyway.
        if (r.refHeld && r.mgr && r.mgr->getCudaContext())
        {
            PxScopedCudaLock _lock(*r.mgr);
            PxCudaContext* cu = r.mgr->getCudaContext();
            for (const PooledColumn& c : r.deviceBuffers)
                if (c.ptr)
                    cu->memFree(static_cast<CUdeviceptr>(c.ptr));
            for (const PooledColumn& c : r.pinnedBuffers)
                if (c.ptr)
                    cu->memFreeHost(reinterpret_cast<void*>(c.ptr));
        }
        if (r.refHeld && r.mgr)
            r.mgr->release(); // matches the acquireReference in poolTakeReference
    }
}

// Reclaim every pooled context NOT in `live` -- one with no live scene keeping it will never serve
// another read, so its pooled memory and the reference pinning its manager are released. An empty
// `live` flushes all. Called once per read with the live context set, so a detached or destroyed
// context is reclaimed on the next read rather than pinned to process exit.
void poolFlushStaleContexts(const std::unordered_set<::physx::PxCudaContextManager*>& live)
{
    std::vector<ReclaimedPool> reclaimed;
    {
        std::lock_guard<std::mutex> lock(g_poolMutex);
        for (std::unordered_map<::physx::PxCudaContextManager*, ContextPool>::iterator it = g_pool.begin();
             it != g_pool.end();)
        {
            if (!it->first || live.find(it->first) == live.end())
            {
                reclaimed.push_back({ it->first, std::move(it->second.deviceFree),
                                  std::move(it->second.pinnedFree), it->second.refHeld });
                it = g_pool.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    freeReclaimedPools(reclaimed);
}

// Device columns owned by a session, bucketed by the CUDA context that allocated them. A pointer
// and an event are only meaningful in their own context, so both the free and the completion wait
// must go through the manager the allocation came from.
struct DeviceContextAllocs
{
    // A REFERENCE, not an observer. A borrowed device column can outlive the attach that produced
    // it -- the Python frontend deliberately lets a Warp array keep the read session alive past
    // detach_ovstage() -- and a later attach may build a new PxCudaContextManager when the CUDA
    // launch settings or device count changed. Releasing through the old pointer then locks,
    // synchronises and frees through a destroyed manager.
    //
    // PxCudaContextManager is reference counted for exactly this ("holding a reference guarantees
    // it stays alive", PxCudaContextManager.h): the session acquires one when the context first
    // enters it and releases it in ovxReleaseRead after the frees, so the manager cannot be
    // destroyed while this session still owes it a free.
    PxCudaContextManager* ctxMgr = nullptr;
    // Device columns (kDLCUDA); returned to the per-context pool in ovxReleaseRead. Each carries its
    // byte size so release can hand it back to a size-keyed free list. Held as
    // uintptr_t to keep the CUDA driver types out of the session storage.
    std::vector<PooledColumn> buffers;
    // Pinned host staging buffers: sources for the read's async HtoD copies -- the
    // reframe matrices and the pending-particle patch. Session-owned so a queued copy still has its
    // source when the function that filled it returns; returned to the pinned side of the same pool at
    // ovxReleaseRead, past the SAME completion wait that orders the copies, so no streamSynchronize is
    // needed on the read path.
    std::vector<PooledColumn> pinned;
    // Completion event for this context's device work, recorded on its null stream once every column
    // has been gathered and handed to the consumer via ovstage_read_group_t::data.cuda_sync
    // (ADR-0008). Non-zero only once recorded. Destroyed in ovxReleaseRead.
    uintptr_t event = 0;
    // No event AND no successful drain: nothing in the process can say whether this context's
    // gathers finished. Distinct from `event == 0` alone, which also covers the benign case where
    // the event could not be recorded but the blocking fallback drained successfully -- there the
    // columns ARE complete and the buffers ARE safe to free.
    //
    // Tracked explicitly because both of the places that care read it in the negative: fetch must
    // not hand out a column whose completion is unknown, and release must not free a destination a
    // gather may still be writing into. `event == 0` cannot distinguish those from "drained fine".
    bool completionUnknown = false;
};

struct ReadSession
{
    // The attach this read was built against; see QueryState::attach. Release destroys prim lists
    // THROUGH `dict`, so a stale session must drop its entries rather than dereference it.
    uint64_t attach = 0;
    ovx_path_dictionary_t* dict = nullptr;
    std::vector<GroupStore> groups;
    size_t cursor = 0;

    // How many live groups hold each prim list. Groups over one prim set share a list, so a list is destroyed
    // when the last of them is released -- this counts them, so a release is O(1) rather than a scan of `groups`.
    std::unordered_map<ovx_primpath_list_t, uint32_t> listRefs;

    // A backend build or gather failed, so the groups below are an INCOMPLETE answer to the query. The drain
    // still hands back everything that did succeed and reports kOvxReadStatusError in place of
    // kOvxReadStatusEndOfIteration, so a caller cannot mistake a short set for the whole one.
    //
    // Set ONLY for backend failure. Three things are answers rather than failures and must not set it: an
    // attribute this object type does not produce, the pre-step unavailable state, and a device-only type on a
    // sim with no CUDA context (structural, not transient). Conflating them makes the status useless for
    // deciding whether to retry.
    bool backendFailed = false;
    // One bucket per context the read touched. A session spans every scene, and under
    // /physics/sceneMultiGPUMode scenes are handed round-robin context managers, so a single read
    // can own buffers on several devices.
    std::vector<DeviceContextAllocs> deviceAllocs;

    // Acquire an output-column buffer of `bytes` for `mgr`'s context, from the per-context pool when
    // one fits and by allocating otherwise. The returned buffer is owned by the
    // session and returned to the pool at release. Returns 0 on allocation failure.
    uintptr_t acquireColumn(PxCudaContextManager* mgr, PxCudaContext* cu, size_t bytes)
    {
        size_t capacity = bytes;
        uintptr_t buf = poolAcquireColumn(mgr, bytes, capacity);
        if (!buf)
        {
            CUdeviceptr fresh = 0;
            if (!cu || cu->memAlloc(&fresh, bytes) != 0 || !fresh)
                return 0;
            buf = static_cast<uintptr_t>(fresh);
            capacity = bytes;
        }
        // Recorded at the buffer's true capacity, not the request, so release re-pools it there.
        deviceAllocsFor(mgr).buffers.push_back({ buf, capacity });
        return buf;
    }

    // Acquire a pinned host staging buffer of at least `bytes` for `mgr`'s context, from the pinned
    // side of the per-context pool when one fits and by allocating (memHostAlloc) otherwise. Pinned so
    // the HtoD copy that reads it is genuinely async, and session-owned so it
    // survives to release, where it is returned to the pool past the completion wait. Returns null on
    // failure. `outBytes` is the buffer's true capacity, which the caller may use to bound its writes.
    void* acquirePinned(PxCudaContextManager* mgr, PxCudaContext* cu, size_t bytes, size_t& outBytes)
    {
        size_t capacity = bytes;
        uintptr_t buf = poolAcquirePinned(mgr, bytes, capacity);
        if (!buf)
        {
            void* fresh = nullptr;
            if (!cu || cu->memHostAlloc(&fresh, bytes, 0) != 0 || !fresh)
                return nullptr;
            buf = reinterpret_cast<uintptr_t>(fresh);
            capacity = bytes;
        }
        deviceAllocsFor(mgr).pinned.push_back({ buf, capacity });
        outBytes = capacity;
        return reinterpret_cast<void*>(buf);
    }

    // The one place a context manager enters a session, so the one place the reference is taken.
    DeviceContextAllocs& deviceAllocsFor(PxCudaContextManager* mgr)
    {
        for (DeviceContextAllocs& a : deviceAllocs)
            if (a.ctxMgr == mgr)
                return a;
        deviceAllocs.emplace_back();
        deviceAllocs.back().ctxMgr = mgr;
        if (mgr)
            mgr->acquireReference(); // matched in ovxReleaseRead, after this context's frees
        return deviceAllocs.back();
    }
};

struct QueryState
{
    uint32_t type = 0;
    uint32_t scope = 0;
    // The attach this query was created against (ADR-0013's monotonic, never-reused handle).
    //
    // `dict` below is BORROWED. Handles here are process-global and outlive any attach, so without
    // this the sequence attach A -> create query -> detach A -> attach B leaves a query that
    // enumerates B's scene while interning through A's dictionary -- two attaches' identity spaces
    // in one answer, reported as success. Every entry point checks this before dereferencing `dict`;
    // see attachStillOwns().
    //
    // Whether that pointer is merely wrong or actually dangling depends on the stage: OvstageSource
    // holds `mDict` NON-owning, so it dies with the scene owner rather than with the attach. Same
    // guard either way -- the cross-attach answer is wrong before anything is freed.
    uint64_t attach = 0;
    ovx_path_dictionary_t* dict = nullptr; // shared source dictionary (for ovxQueryDictionary)
    // Backing storage for ovxFetchQueryResult's returned attribute-token array
    // (kept alive until ovxReleaseQuery).
    std::vector<ovx_token_t> resultAttrs;
};

// The source's shared path dictionary. buildPathList sets *outDict from the
// OvstageSource even for a zero-length list, so this gets the dict without
// building one. Returns null for a non-ovstage backend.
ovx_path_dictionary_t* sourceDictionary(IPhysicsSource& source)
{
    ovx_path_dictionary_t* dict = nullptr;
    omni::physics::ovstage::buildPathList(source, nullptr, 0, &dict);
    return dict;
}

// The same liveness purge for the caches keyed directly on PxScene*. Same reasoning as above:
// a scene that has gone away is never looked up again, so nothing else would free its entry.
// Templated rather than copied per cache -- several use this ownership shape, and the loop is the
// whole content.
template <typename Map>
void purgeDeadSceneEntries(Map& cache, const std::vector<PxScene*>& scenes)
{
    for (typename Map::iterator it = cache.begin(); it != cache.end();)
        it = (std::find(scenes.begin(), scenes.end(), it->first) != scenes.end()) ?
                 std::next(it) :
                 cache.erase(it);
}

std::mutex g_mutex;
uint64_t g_nextQuery = 1;
uint64_t g_nextRead = 1;
std::unordered_map<uint64_t, QueryState> g_queries;
std::unordered_map<uint64_t, ReadSession> g_reads;

// ----------------------------------------------------------------------------
// Common helpers
// ----------------------------------------------------------------------------

// Whether a read or query handle minted against `attach` may still be used.
//
// The test is LIVENESS, not activeness: is that attach still attached? Not "is it the one
// getActiveAttachedStage() would pick". UsdLoad holds mAttachedStages as a MAP, so several attaches
// can be live at once -- one per ovphysx instance in a multi-instance process -- and
// getActiveAttachedStage() returns the single entry only when there is exactly one, falling back to
// getAttachedStage(0) otherwise. Comparing against that would refuse a handle whose own stage is
// still attached and whose dictionary is still valid, purely because a different instance's attach
// sorted first. Rejecting a live handle is a worse failure than the staleness this guards.
//
// ADR-0013 mints these monotonic and never reused, which is what makes the lookup decisive: a
// handle that resolves to nothing is detached, not merely inactive, and kNoAttachHandle cannot be
// confused with "some attach". getAttachedStageByHandle is the one place that reads a handle back,
// so this asks it rather than keeping a second notion of what is attached.
bool attachStillOwns(uint64_t attach, const char* who)
{
    UsdLoad* const usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad && attach != UsdLoad::kNoAttachHandle && usdLoad->getAttachedStageByHandle(attach))
        return true;
    CARB_LOG_ERROR("%s: this handle belongs to attach %llu, which is no longer attached. The handle is "
                   "stale -- release it; reading or releasing through it would use that attach's path "
                   "dictionary, which its stage owner may already have destroyed.",
                   who, (unsigned long long)attach);
    return false;
}

// The authored USD role of a column, which a consumer reads to know what the numbers MEAN -- and which the
// write path carries back unchanged, so two columns of the same USD type must not disagree here.
//
// Roled names are matched explicitly and NONE is the DEFAULT, not an enumeration: an attribute with a genuine
// role that nobody adds a branch for is emitted roleless and silently. A default rather than a table field
// because this serves BOTH the table-driven types and the hardcoded ones (vehicle wheels, deformable and
// particle points/velocities). The forcing function is TestOvstageOutputReadback's "every advertised attribute
// states a role", which walks discovery and fails on any attribute it has no expectation for.
ovstage_attribute_semantic_t semanticForName(const std::string& name)
{
    // Positions, in whatever frame. The frame is not part of the role: instancer `positions` are
    // instancer-local and tagged POINT already, so a local-frame centre of mass is no different.
    if (name == omni::physx::OvxAttr::kPosition || name == omni::physx::OvxAttr::kPoints ||
        name == omni::physx::OvxAttr::kPositions || name == omni::physx::OvxAttr::kCenterOfMassPosition ||
        name == omni::physx::OvxAttr::kCenterOfMassWorld || name == omni::physx::OvxAttr::kCenterOfMassLocal ||
        name == omni::physx::OvxAttr::kRootPosition ||
        name == omni::physx::OvxAttr::kRestPoints)
        return OVSTAGE_SEMANTIC_POINT;

    if (name == omni::physx::OvxAttr::kOrientation || name == omni::physx::OvxAttr::kOrientations ||
        name == omni::physx::OvxAttr::kCenterOfMassOrientation || name == omni::physx::OvxAttr::kRootOrientation)
        return OVSTAGE_SEMANTIC_QUATERNION;

    // Velocities and accelerations alike: `velocities` and `accelerations` are both vector3f[] on
    // UsdGeomPointInstancer, which is where those two array names come from, so tagging one and not
    // the other would have the same USD role reported two ways over the same instancer.
    if (name == omni::physx::OvxAttr::kLinearVelocity || name == omni::physx::OvxAttr::kAngularVelocity ||
        name == omni::physx::OvxAttr::kVelocities || name == omni::physx::OvxAttr::kAngularVelocities ||
        name == omni::physx::OvxAttr::kLinearAcceleration || name == omni::physx::OvxAttr::kAngularAcceleration ||
        name == omni::physx::OvxAttr::kAccelerations || name == omni::physx::OvxAttr::kAngularAccelerations ||
        name == omni::physx::OvxAttr::kRootLinearVelocity || name == omni::physx::OvxAttr::kRootAngularVelocity)
        return OVSTAGE_SEMANTIC_VECTOR;

    // 3x3 tensors, 9 lanes column-major.
    if (name == omni::physx::OvxAttr::kInertia || name == omni::physx::OvxAttr::kInverseInertia)
        return OVSTAGE_SEMANTIC_MATRIX;

    // Everything else genuinely has no USD role: mass / inverseMass, the per-shape offsets and material
    // coefficients and the per-axis joint and tendon values are plain scalars; disableGravity /
    // disableSimulation are flags; shapeCount is a count.
    return OVSTAGE_SEMANTIC_NONE;
}

ovx_token_t internToken(ovx_path_dictionary_t* dict, const std::string& name)
{
    ovx_token_t tok = OVX_INVALID_TOKEN;
    if (dict)
        ovx_path_dictionary_intern_token(dict, ovx_string_t{ name.c_str(), name.size() }, &tok);
    return tok;
}

// Finish a group: build its prim list, intern its attribute token, and stamp the
// DLTensor. `keys` are the prims the group covers (fixed: one per row; array: the
// single owning prim). Returns false (and adds nothing) if the list cannot be built
// (non-ovstage backend) — keeping the read ovstage-only.
bool finalizeGroup(ReadSession& s,
                   IPhysicsSource& source,
                   GroupStore&& g,
                   const std::string& attrName,
                   const std::vector<ObjectKey>& keys,
                   int64_t rows,
                   int64_t comp,
                   const std::vector<ovx_primpath_t>* canonicalHandles = nullptr,
                   ovx_primpath_list_t* sharedList = nullptr)
{
    // DLDataType::lanes is uint16_t, and `comp` is the column's full width. A legal articulation can
    // exceed it: a 64-link / 189-DOF one gives a fixed-base Jacobian width of 378 * 189 = 71,442
    // (74,880 floating-base), which wraps to a small number while the allocation and the gather use
    // the real width -- so a consumer computes its byte count and shape from metadata describing a
    // column that is not the one it was handed. Refuse to publish it rather than truncate.
    //
    // The right long-term answer is a shape with real dimensions rather than a width folded into
    // lanes; until then this is the boundary that keeps the wrong number from reaching a caller.
    // Every call site propagates a false return.
    if (comp <= 0 || comp > static_cast<int64_t>(UINT16_MAX))
    {
        CARB_LOG_ERROR("ovxReadAttributes: '%s' has a column width of %lld, which does not fit DLPack's "
                       "16-bit dtype.lanes; the group is omitted rather than published with a wrapped "
                       "width. This articulation topology is too wide for the current group shape.",
                       attrName.c_str(), static_cast<long long>(comp));
        return false;
    }

    ovx_path_dictionary_t* dict = nullptr;
    // Canonicalising a key costs a lock plus a cache lookup and building the list walks every prim, so a read
    // emitting several groups over ONE prim set does both once and shares the result. Releasing a group then
    // drops that group's claim rather than destroying the list -- see ovxReleaseGroup.
    if (sharedList && *sharedList != OVX_INVALID_PRIMPATH_LIST)
    {
        g.list = *sharedList;
        dict = s.dict;
    }
    else if (canonicalHandles && canonicalHandles->size() == keys.size())
        g.list = omni::physics::ovstage::buildPathListFromHandles(
            source, canonicalHandles->data(), canonicalHandles->size(), &dict);
    else
        g.list = omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (sharedList)
        *sharedList = g.list;
    if (g.list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        CARB_LOG_ERROR("ovxReadAttributes: failed to build ovx_primpath_list_t for '%s' (%zu prims) — "
                       "the active backend is not ovstage (this read is ovstage-only).",
                       attrName.c_str(), keys.size());
        return false;
    }
    s.dict = dict;
    g.primCount = static_cast<uint32_t>(keys.size());
    g.attribute = internToken(dict, attrName);
    g.semantic = semanticForName(attrName);
    g.shapes.assign(1, rows);
    g.tensors.assign(1, DLTensor{});
    DLTensor& t0 = g.tensors[0];

    if (g.deviceOrdinal >= 0)
    {
        // Device-resident column (ADR-0008): the data lives on the simulation GPU. The read
        // session records one completion event per CUDA context and publishes that event through
        // data.cuda_sync when the group is fetched.
        t0.data = g.deviceData;
        t0.device = DLDevice{ kDLCUDA, g.deviceOrdinal };
    }
    else
    {
        // Whichever scratch this column's element type filled.
        if (!g.bytes.empty())
            t0.data = static_cast<void*>(g.bytes.data());
        else if (!g.ints.empty())
            t0.data = static_cast<void*>(g.ints.data());
        else
            t0.data = static_cast<void*>(g.floats.data());
        t0.device = DLDevice{ kDLCPU, 0 };
    }
    t0.ndim = 1;
    t0.dtype = g.dtype;
    t0.dtype.lanes = static_cast<uint16_t>(comp);
    t0.shape = nullptr;   // patched to point at the stored shape in fetch (stable address)
    t0.strides = nullptr; // contiguous row-major SoA
    t0.byte_offset = 0;

    if (g.list != OVX_INVALID_PRIMPATH_LIST)
        ++s.listRefs[g.list];
    s.groups.push_back(std::move(g));
    return true;
}

// The inverse of finalizeGroup, for a builder that has to withdraw groups it already published -- a
// cross-scene failure, where earlier scenes finalized before a later one failed.
//
// Shrinking s.groups alone is not enough: finalizeGroup bumped listRefs for each of those groups, and
// ovxReleaseGroup relies on listRefs counting groups actually present in s.groups.
void rollbackGroups(ReadSession& s, size_t firstGroup)
{
    for (size_t i = firstGroup; i < s.groups.size(); ++i)
    {
        const ovx_primpath_list_t list = s.groups[i].list;
        if (list == OVX_INVALID_PRIMPATH_LIST)
            continue;
        const std::unordered_map<ovx_primpath_list_t, uint32_t>::iterator ref = s.listRefs.find(list);
        if (ref == s.listRefs.end())
            continue;
        if (--ref->second == 0)
        {
            s.listRefs.erase(ref);
            if (s.dict)
                ovx_path_dictionary_destroy_path_list(s.dict, list);
        }
    }
    s.groups.resize(firstGroup);
}

// An array group over MANY prims: one tensor per prim in ONE group, which is ovstage's array shape
// and what a per-prim attribute should emit. `g` arrives with its tensors and shapes already built
// (the caller knows where each prim's values live); this attaches the prim list, the attribute
// token and the semantic exactly as finalizeGroup does, and hands the whole set over as a single
// group.
bool finalizeArrayGroup(ReadSession& s,
                        IPhysicsSource& source,
                        GroupStore&& g,
                        const std::string& attrName,
                        const std::vector<ObjectKey>& keys,
                        const std::vector<ovx_primpath_t>* canonicalHandles = nullptr,
                        ovx_primpath_list_t* sharedList = nullptr)
{
    ovx_path_dictionary_t* dict = nullptr;
    // Same bargain as finalizeGroup's: several array groups over ONE prim set build the list once and share
    // it, and releasing a group drops its claim rather than destroying the list. A deformable read emits one
    // array group per attribute over the same bodies, so the walk over every prim is not repeated per column.
    if (sharedList && *sharedList != OVX_INVALID_PRIMPATH_LIST)
    {
        g.list = *sharedList;
        dict = s.dict;
    }
    else if (canonicalHandles && canonicalHandles->size() == keys.size())
        g.list = omni::physics::ovstage::buildPathListFromHandles(
            source, canonicalHandles->data(), canonicalHandles->size(), &dict);
    else
        g.list = omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (sharedList)
        *sharedList = g.list;
    if (g.list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        CARB_LOG_ERROR("ovxReadAttributes: failed to build ovx_primpath_list_t for '%s' (%zu prims) — "
                       "the active backend is not ovstage (this read is ovstage-only).",
                       attrName.c_str(), keys.size());
        return false;
    }
    s.dict = dict;
    g.primCount = static_cast<uint32_t>(keys.size());
    g.attribute = internToken(dict, attrName);
    g.semantic = semanticForName(attrName);
    g.isArray = true;

    ++s.listRefs[g.list];
    s.groups.push_back(std::move(g));
    return true;
}

// ----------------------------------------------------------------------------
// Rigid bodies (standalone + point-instancer instances)
// ----------------------------------------------------------------------------

struct InstancerAccum
{
    ObjectKey instancerKey;
    uint32_t maxIndex = 0;
    bool hasActive = false; // ACTIVE scope: instancer is emitted iff >=1 instance moved last step

    // Backend path only (buildInstancerBackendGroups), on either device. `rows` stays empty there:
    // the reframe that fills it is exactly the work the view does, so the enumeration never computes
    // it. What is collected instead is the instance list the view is built from, and the frame its
    // local poses are expressed in.
    std::vector<omni::physx::tensors::PointInstance> instances;
    ::physx::PxMat44d worldInverse{ ::physx::PxIdentity };

    // The scene this instancer's bodies live in. A view belongs to ONE scene -- a body's DirectGPU
    // index is only meaningful in the scene that owns it -- so the backend builder buckets on this
    // and builds a view per scene. Set from the first instanced body seen for this instancer.
    PxScene* scene = nullptr;
    // Set when a later body of the SAME instancer turns up in a different scene. That would need one
    // instancer's slots filled from two views, which no column gather can express, so the read
    // reports it rather than serving the subset it can reach.
    bool multiScene = false;
};

// One scene's active set. The flag is per scene, so this is the honest unit: a scene without
// eENABLE_ACTIVE_ACTORS reports `available = false` rather than an empty set that a caller could
// mistake for "nothing moved".
//
// Stays HERE while ActiveActorSet and the cross-scene collectActiveActors live in
// OvxPhysicsShared: only the read has a caller holding a scene to ask. Move it across when the
// write grows one -- the shared file is for what BOTH directions must agree on, not a dumping
// ground for anything articulation-shaped.
ActiveActorSet collectActiveActorsForScene(PxScene* scene)
{
    ActiveActorSet out;
    if (!scene || !(scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS))
        return out;

    // The owning InternalScene, because the released-actor predicate lives there and the list
    // cannot be walked safely without it.
    InternalScene* const internalScene = internalSceneOf(scene);
    if (!internalScene)
        return out; // no owner to ask: report nothing rather than dereference the list

    out.sizeTo(OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords().size());
    markActiveActorsOfScene(internalScene, out);
    return out;
}

// ----------------------------------------------------------------------------
// Point-instancer enumeration
// ----------------------------------------------------------------------------

// How much of each instance the caller actually needs. The walk that decides WHICH instancers and
// instances there are is identical for both, so no caller can disagree with another about the shape
// of a read.
//
// There is no host-values mode. Instancer columns are gathered by BasePointInstancerView on either
// device, so nothing needs a pose pulled back and reframed on the host -- that was the legacy path,
// and it is gone.
enum class InstancerCollect
{
    // Descriptions: the instance list a tensor view is built from. No pose query and no reframe --
    // that is the work the view does instead.
    eDescriptions,
    // Nothing per instance at all: just which instancers exist. The QUERY path only wants the keys.
    eKeysOnly,
};

// The empty set, for callers that enumerate under a scope where "active" does not apply.
const ActiveActorSet kNoActiveActors;

void enumerateInstancers(AttachedStage& as,
                         uint32_t scope,
                         const ActiveActorSet& activeSet,
                         std::vector<InstancerAccum>& instancers,
                         const std::vector<PxRigidDynamic*>* preScanned = nullptr,
                         InstancerCollect collect = InstancerCollect::eDescriptions)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    std::unordered_map<uint64_t, size_t> instancerSlot;   // instancerKey.handle -> index in instancers
    std::map<uint64_t, ::physx::PxMat44d> instancerWorldInv; // cached per instancer

    const std::vector<InternalDatabase::Record>& records = db.getRecords();

    // The instanced dynamics this walk describes. Standalone bodies are enumerated elsewhere and
    // deliberately excluded here. Taken from the caller's shared scan when it has one, so the read
    // does not walk every record a second time just to find these.
    std::vector<PxRigidDynamic*> instancedOwned;
    if (!preScanned)
    {
        for (const InternalDatabase::Record& rec : records)
        {
            if (rec.mType != omni::physx::ePTActor || !rec.mPtr || !rec.mInternalPtr)
                continue;
            PxRigidDynamic* d = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>();
            InternalActor* ia = reinterpret_cast<InternalActor*>(rec.mInternalPtr);
            if (d && ia && ia->mInstanceIndex != kInvalidUint32_t)
                instancedOwned.push_back(d);
        }
    }
    const std::vector<PxRigidDynamic*>& instancedDynamics = preScanned ? *preScanned : instancedOwned;

    // No point-instancer instances anywhere means no instancer groups, so the walk over every
    // database record below can only come back empty.
    if (instancedDynamics.empty())
        return;

    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != omni::physx::ePTActor || !rec.mPtr || !rec.mInternalPtr)
            continue;
        PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(rec.mPtr);
        PxRigidDynamic* dyn = actor->is<PxRigidDynamic>();
        if (!dyn)
            continue; // rigid bodies = PxRigidDynamic (dynamic or kinematic)

        // Per-body "moved last step" decision, shared by standalone bodies and
        // point-instancer instances. Prefer the engine's active-actor set when the caller supplied
        // one; else approximate with isSleeping() -- a kinematic body reports active only while it
        // is being moved.
        bool bodyActive = true;
        if (scope == omni::physx::kOvxActive)
        {
            bodyActive = activeSet.sceneReports(dyn->getScene())
                             ? activeSet.isActive(ri)
                             : !((dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) || dyn->isSleeping());
        }

        InternalActor* ia = reinterpret_cast<InternalActor*>(rec.mInternalPtr);

        // Standalone bodies are sourced from the backend view (buildStandaloneRigid); skip here.
        if (!ia || ia->mInstanceIndex == kInvalidUint32_t)
            continue;

        const uint64_t ih = ia->mInstanceKey.handle;

        // The instancer's own world transform, inverted -- the frame every local pose is in. Cached per
        // instancer WITHIN this read only: it is a stage query, so a moving instancer changes it with nothing
        // in the object database to notice, and a value kept across reads would go quietly stale.
        //
        // Left identity under eKeysOnly, which is only safe because that mode's caller reads nothing but the
        // key. Anything that starts reading InstancerAccum::worldInverse has to resolve it here too.
        ::physx::PxMat44d worldInverse(::physx::PxIdentity);
        if (collect != InstancerCollect::eKeysOnly)
        {
            std::map<uint64_t, ::physx::PxMat44d>::iterator invIt = instancerWorldInv.find(ih);
            if (invIt == instancerWorldInv.end())
                invIt = instancerWorldInv
                            .emplace(ih, omni::physx::affineInverse(getWorldTransform(
                                             as, ia->mInstanceKey, omni::physics::parse::ReadTime::defaultTime())))
                            .first;
            worldInverse = invIt->second;
        }

        auto devSlotIt = instancerSlot.find(ih);
        if (devSlotIt == instancerSlot.end())
        {
            devSlotIt = instancerSlot.emplace(ih, instancers.size()).first;
            InstancerAccum acc;
            acc.instancerKey = ia->mInstanceKey;
            acc.worldInverse = worldInverse;
            acc.scene = dyn->getScene();
            instancers.push_back(std::move(acc));
        }
        else
        {
            InstancerAccum& seen = instancers[devSlotIt->second];
            if (seen.scene && dyn->getScene() && seen.scene != dyn->getScene())
                seen.multiScene = true;
        }
        InstancerAccum& devAcc = instancers[devSlotIt->second];
        if (collect == InstancerCollect::eDescriptions)
        {
            omni::physx::tensors::PointInstance pi;
            pi.body = dyn;
            pi.index = ia->mInstanceIndex;
            pi.protoInverse = ia->mProtoTransformInverse;
            devAcc.instances.push_back(pi);
        }
        devAcc.maxIndex = std::max(devAcc.maxIndex, ia->mInstanceIndex);
        devAcc.hasActive = devAcc.hasActive || bodyActive;
    }

    // ACTIVE scope: keep an instancer only if at least one of its instances moved
    // last step. The retained ones still carry their full instance set (above), so
    // each is emitted as a dense, contract-compliant full array — no intra-tensor
    // scatter (see buildRigidBodyGroups).
    if (scope == omni::physx::kOvxActive)
        instancers.erase(std::remove_if(instancers.begin(), instancers.end(),
                                        [](const InstancerAccum& a) { return !a.hasActive; }),
                         instancers.end());
}


// The rigid-body output attributes, one row each. A row names the column width, the array-valued variant, and
// the view method that reads it -- so adding an attribute is a row here and a thin method on each view, with
// no dispatch to extend anywhere. `components` is the single source of truth for the width: the allocation
// below and the view's write are both sized from it.
struct RigidAttributeRow
{
    const char* token;
    const char* arrayToken;
    int components;
    // Element type of the column. float32 for everything with a DirectGPU read behind it; uint8 for
    // the two actor flags, which the tensor API reports as bytes.
    omni::physics::tensors::TensorDataType dtype;
    // True when the attribute has NO device source, so its column is host-resident even on a DirectGPU scene:
    // these are simulation *inputs* PhysX never writes back (mass, inertia, centre of mass, the actor flags),
    // so there is no device copy to read and nothing for readback suppression to make stale.
    bool hostOnly;
    // One value per SHAPE of the body rather than per body, so `components` above is 0 and the real width --
    // the widest body in this read -- is resolved at emit from the view, as one padded fixed group.
    bool perShape;
    bool (omni::physx::tensors::GpuRigidBodyView::*gpuRead)(
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t) const;
    bool (omni::physx::tensors::CpuRigidBodyView::*cpuRead)(
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t) const;

    // Which OvStageShapeProperty a per-shape row IS, so the emit hands the whole per-shape set to one backend
    // call instead of resolving the shape's material once per shape PER column.
    //
    // Read only when `perShape` is true; eNone is the "not applicable" answer for every other row. Both
    // switches in OvStageShapeProperty.h handle eNone explicitly -- -Wno-switch is set tree-wide, so nothing
    // would report a sixth property.
    omni::physx::tensors::OvStageShapeProperty shapeProperty =
        omni::physx::tensors::OvStageShapeProperty::eNone;
};

// A backend that structurally cannot serve an attribute names one of these instead of a stub, so a row still
// has to say something for each backend: a FORGOTTEN implementation stays a compile error while a DELIBERATE
// omission is legible.
constexpr decltype(RigidAttributeRow::gpuRead) kRigidNotOnGpu = nullptr;
constexpr decltype(RigidAttributeRow::cpuRead) kRigidNotOnCpu = nullptr;

constexpr omni::physics::tensors::TensorDataType kF32 = omni::physics::tensors::TensorDataType::eFloat32;
constexpr omni::physics::tensors::TensorDataType kU8 = omni::physics::tensors::TensorDataType::eUint8;
constexpr omni::physics::tensors::TensorDataType kI32 = omni::physics::tensors::TensorDataType::eInt32;

constexpr RigidAttributeRow kRigidAttributes[] = {
    { omni::physx::OvxAttr::kPosition, omni::physx::OvxAttr::kPositions, 3, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getPositionsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getPositionsOvStage },
    { omni::physx::OvxAttr::kOrientation, omni::physx::OvxAttr::kOrientations, 4, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getOrientationsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getOrientationsOvStage },
    { omni::physx::OvxAttr::kLinearVelocity, omni::physx::OvxAttr::kVelocities, 3, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getLinearVelocitiesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getLinearVelocitiesOvStage },
    { omni::physx::OvxAttr::kAngularVelocity, omni::physx::OvxAttr::kAngularVelocities, 3, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getAngularVelocitiesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getAngularVelocitiesOvStage },
    // Array tokens follow UsdGeomPointInstancer, which is where `velocities` rather than
    // `linearVelocities` comes from: it spells the linear one `accelerations`. It has no angular
    // acceleration at all, so that one keeps the `angular` prefix its velocity sibling has.
    { omni::physx::OvxAttr::kLinearAcceleration, omni::physx::OvxAttr::kAccelerations, 3, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getLinearAccelerationsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getLinearAccelerationsOvStage },
    { omni::physx::OvxAttr::kAngularAcceleration, omni::physx::OvxAttr::kAngularAccelerations, 3, kF32, false, false,
      &omni::physx::tensors::GpuRigidBodyView::getAngularAccelerationsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::getAngularAccelerationsOvStage },

    // The body PROPERTIES. One implementation on BaseRigidBodyView serves both backends -- naming it
    // twice here is the same function reached through each view -- because these have no device
    // source to differ over. `hostOnly` is what carries that to the emit; see the field's comment.
    //
    // No array token: a point instancer authors no per-instance mass or inertia, so there is no
    // instancer attribute for these to publish under. The array fill warns rather than inventing one.
    { omni::physx::OvxAttr::kMass, nullptr, 1, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getMassesOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getMassesOvStage },
    { omni::physx::OvxAttr::kInverseMass, nullptr, 1, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getInvMassesOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getInvMassesOvStage },
    { omni::physx::OvxAttr::kInertia, nullptr, 9, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getInertiasOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getInertiasOvStage },
    { omni::physx::OvxAttr::kInverseInertia, nullptr, 9, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getInvInertiasOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getInvInertiasOvStage },
    // COM_POSE splits into position and orientation exactly as the world pose does, one attribute
    // per column. Both are in the body's LOCAL frame (getCMassLocalPose), unlike position/orientation
    // which are world -- the attribute names cannot carry that, so it is stated here and in the docs.
    { omni::physx::OvxAttr::kCenterOfMassPosition, nullptr, 3, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getComPositionsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getComPositionsOvStage },
    { omni::physx::OvxAttr::kCenterOfMassOrientation, nullptr, 4, kF32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getComOrientationsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getComOrientationsOvStage },
    { omni::physx::OvxAttr::kDisableGravity, nullptr, 1, kU8, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getDisableGravitiesOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getDisableGravitiesOvStage },
    { omni::physx::OvxAttr::kDisableSimulation, nullptr, 1, kU8, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getDisableSimulationsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getDisableSimulationsOvStage },
    // Per-shape. Split per value rather than one interleaved friction triple, matching how the pose
    // splits into position and orientation: one attribute is one column.
    { omni::physx::OvxAttr::kStaticFriction, nullptr, 0, kF32, true, true,
      &omni::physx::tensors::BaseRigidBodyView::getStaticFrictionsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getStaticFrictionsOvStage,
      omni::physx::tensors::OvStageShapeProperty::eStaticFriction },
    { omni::physx::OvxAttr::kDynamicFriction, nullptr, 0, kF32, true, true,
      &omni::physx::tensors::BaseRigidBodyView::getDynamicFrictionsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getDynamicFrictionsOvStage,
      omni::physx::tensors::OvStageShapeProperty::eDynamicFriction },
    { omni::physx::OvxAttr::kRestitution, nullptr, 0, kF32, true, true,
      &omni::physx::tensors::BaseRigidBodyView::getRestitutionsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getRestitutionsOvStage,
      omni::physx::tensors::OvStageShapeProperty::eRestitution },
    { omni::physx::OvxAttr::kContactOffset, nullptr, 0, kF32, true, true,
      &omni::physx::tensors::BaseRigidBodyView::getContactOffsetsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getContactOffsetsOvStage,
      omni::physx::tensors::OvStageShapeProperty::eContactOffset },
    { omni::physx::OvxAttr::kRestOffset, nullptr, 0, kF32, true, true,
      &omni::physx::tensors::BaseRigidBodyView::getRestOffsetsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getRestOffsetsOvStage,
      omni::physx::tensors::OvStageShapeProperty::eRestOffset },
    // Per BODY, not per shape: how many of a per-shape row's entries are real.
    { omni::physx::OvxAttr::kShapeCount, nullptr, 1, kI32, true, false,
      &omni::physx::tensors::BaseRigidBodyView::getShapeCountsOvStage,
      &omni::physx::tensors::BaseRigidBodyView::getShapeCountsOvStage },
};

// Whole-articulation output attributes. State and COM have backend-specific sources; shape
// properties are authored inputs and therefore use the same host reader on both backends. Keeping
// them in a table separate from kRigidAttributes is load-bearing: these rows index articulations,
// not bodies or links, even where the public attribute spelling is deliberately shared.
using ArticulationGpuRead = bool (omni::physx::tensors::GpuArticulationView::*)(
    const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t) const;
using ArticulationCpuRead = bool (omni::physx::tensors::CpuArticulationView::*)(
    const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t) const;
using ArticulationHostRead = bool (omni::physx::tensors::BaseArticulationView::*)(
    const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32) const;

struct ArticulationRootAttributeRow
{
    const char* token;
    // Lane count, or 0 for a perShape row whose real width is resolved at emit -- as on
    // RigidAttributeRow. No arrayToken: an articulation is never point-instanced, so there is no
    // array form to name.
    int components;
    // Element type of the column: float32 for the state and shape columns, int32 for shapeCount.
    omni::physics::tensors::TensorDataType dtype;
    // True when the attribute has no device source, so the column is host-resident even on a
    // DirectGPU scene and `hostRead` below serves it through the base view.
    bool hostOnly;
    // One value per SHAPE of the articulation rather than per articulation, flattened link-major.
    // `components` is 0 for these; the width is the widest selected articulation in this read.
    bool perShape;
    // True when the column is emitted one group per cohort of structurally identical articulations
    // rather than one group over the whole scene partition (REQ-READ-INVDYN-001 AC-2), and
    // `inverseDynamicsColumn` rather than `components` gives its width. Read only when perCohort is true.
    //
    // For five of the six this is forced: their width is a function of each articulation's topology,
    // so one group cannot express two cohorts. `jacobianShape` is the exception -- its width is a
    // constant 2 -- and it is per-cohort by CHOICE, to stay on the same partitioning as the jacobian
    // it describes rather than making a caller join two differently-partitioned groups.
    bool perCohort;
    omni::physx::tensors::BaseArticulationView::InverseDynamicsColumn inverseDynamicsColumn;
    ArticulationGpuRead gpuRead;
    ArticulationCpuRead cpuRead;
    // The host-only reads live on the BASE view and take no rowsToken -- there is no device copy to keep in
    // step -- so their signature differs and they need this third slot rather than one base-class pointer in
    // both backend slots.
    ArticulationHostRead hostRead;

    // Which root-state quantity this row IS, or eNone. On the row rather than in a token list beside the
    // table, whose failure mode is silent: rename a token there and the column is still SERVED, it just stops
    // sharing its fetch. Defaulted, so only the four root-state rows say so.
    omni::physx::tensors::BaseArticulationView::RootStateQuantity rootState =
        omni::physx::tensors::BaseArticulationView::RootStateQuantity::eNone;
};

// The articulation equivalents of kRigidNotOnGpu / kRigidNotOnCpu above, and for the same reason: a
// row must say something for every slot, so a forgotten read stays a compile error while a
// deliberate omission reads as one. Three slots means three names.
constexpr decltype(ArticulationRootAttributeRow::gpuRead) kArtiRootNotOnGpu = nullptr;
constexpr decltype(ArticulationRootAttributeRow::cpuRead) kArtiRootNotOnCpu = nullptr;
constexpr decltype(ArticulationRootAttributeRow::hostRead) kArtiRootNotOnHost = nullptr;

using InvDynCol = omni::physx::tensors::BaseArticulationView::InverseDynamicsColumn;
using RSQ = omni::physx::tensors::BaseArticulationView::RootStateQuantity;

constexpr ArticulationRootAttributeRow kArticulationRootAttributes[] = {
    { omni::physx::OvxAttr::kRootPosition, 3, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getPositionsOvStage,
      &omni::physx::tensors::CpuArticulationView::getPositionsOvStage, kArtiRootNotOnHost, RSQ::ePosition },
    { omni::physx::OvxAttr::kRootOrientation, 4, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getOrientationsOvStage,
      &omni::physx::tensors::CpuArticulationView::getOrientationsOvStage, kArtiRootNotOnHost, RSQ::eOrientation },
    { omni::physx::OvxAttr::kRootLinearVelocity, 3, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getLinearVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::getLinearVelocitiesOvStage, kArtiRootNotOnHost,
      RSQ::eLinearVelocity },
    { omni::physx::OvxAttr::kRootAngularVelocity, 3, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getAngularVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::getAngularVelocitiesOvStage, kArtiRootNotOnHost,
      RSQ::eAngularVelocity },
    { omni::physx::OvxAttr::kCenterOfMassWorld, 3, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getMassCentersWorldOvStage,
      &omni::physx::tensors::CpuArticulationView::getMassCentersWorldOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kCenterOfMassLocal, 3, kF32, false, false, false, InvDynCol::eNone,
      &omni::physx::tensors::GpuArticulationView::getMassCentersLocalOvStage,
      &omni::physx::tensors::CpuArticulationView::getMassCentersLocalOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kStaticFriction, 0, kF32, true, true, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getStaticFrictionsOvStage },
    { omni::physx::OvxAttr::kDynamicFriction, 0, kF32, true, true, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getDynamicFrictionsOvStage },
    { omni::physx::OvxAttr::kRestitution, 0, kF32, true, true, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getRestitutionsOvStage },
    { omni::physx::OvxAttr::kContactOffset, 0, kF32, true, true, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getContactOffsetsOvStage },
    { omni::physx::OvxAttr::kRestOffset, 0, kF32, true, true, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getRestOffsetsOvStage },
    { omni::physx::OvxAttr::kShapeCount, 1, kI32, true, false, false, InvDynCol::eNone, kArtiRootNotOnGpu, kArtiRootNotOnCpu,
      &omni::physx::tensors::BaseArticulationView::getShapeCountsOvStage },
    // Inverse dynamics columns. Width is a function of the articulation's topology, so `components` is 0 --
    // it is not the width -- and each is emitted one group per cohort (REQ-READ-INVDYN-001).
    { omni::physx::OvxAttr::kJacobian, 0, kF32, false, false, true, InvDynCol::eJacobian,
      &omni::physx::tensors::GpuArticulationView::getJacobiansOvStage,
      &omni::physx::tensors::CpuArticulationView::getJacobiansOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kMassMatrix, 0, kF32, false, false, true, InvDynCol::eMassMatrix,
      &omni::physx::tensors::GpuArticulationView::getMassMatricesOvStage,
      &omni::physx::tensors::CpuArticulationView::getMassMatricesOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kCoriolisForce, 0, kF32, false, false, true, InvDynCol::eGeneralizedForce,
      &omni::physx::tensors::GpuArticulationView::getCoriolisForcesOvStage,
      &omni::physx::tensors::CpuArticulationView::getCoriolisForcesOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kGravityForce, 0, kF32, false, false, true, InvDynCol::eGeneralizedForce,
      &omni::physx::tensors::GpuArticulationView::getGravityForcesOvStage,
      &omni::physx::tensors::CpuArticulationView::getGravityForcesOvStage, kArtiRootNotOnHost },
    { omni::physx::OvxAttr::kCentroidalMomentum, 0, kF32, false, false, true, InvDynCol::eCentroidalMomentum,
      &omni::physx::tensors::GpuArticulationView::getCentroidalMomentaOvStage,
      &omni::physx::tensors::CpuArticulationView::getCentroidalMomentaOvStage, kArtiRootNotOnHost },
    // Host-resident: derived from the metatype, so there is nothing for a device path to read.
    // perCohort despite a width that never varies, so it shares the jacobian's partitioning rather
    // than sitting scene-wide beside it: a caller pairs the two groups by prim list, the same way it
    // identifies any cohort. The rows of one such group are identical to each other by construction.
    { omni::physx::OvxAttr::kJacobianShape, 2, kI32, true, false, true, InvDynCol::eJacobianShape, kArtiRootNotOnGpu,
      kArtiRootNotOnCpu, &omni::physx::tensors::BaseArticulationView::getJacobianShapesOvStage },
};

const ArticulationRootAttributeRow* findArticulationRootAttribute(std::string_view token)
{
    for (const ArticulationRootAttributeRow& row : kArticulationRootAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}


// The joint-DOF output attributes, one row each -- same contract as the rigid table above: a row
// names the view method that reads it, so adding an attribute is a row plus a thin method on each
// articulation view. Both backends take the same arguments, so one row serves both.
struct JointAttributeRow
{
    const char* token;
    bool (omni::physx::tensors::GpuArticulationView::*gpuRead)(
        const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32,
        const omni::physics::tensors::TensorDesc*) const;
    bool (omni::physx::tensors::CpuArticulationView::*cpuRead)(
        const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32,
        const omni::physics::tensors::TensorDesc*) const;

    // Which DOF state quantity this column IS, so the emit hands the whole cache-backed set to one refresh
    // instead of running one per column.
    //
    // Host-side only: it selects the CPU cache flags. eNone is the "not a state column" answer and
    // jointProjectedForce is the row that takes it -- derived from the link incoming joint forces rather than
    // read out of the articulation cache. Defaulted, so only the five state rows say so.
    omni::physx::tensors::BaseArticulationView::DofStateQuantity stateQuantity =
        omni::physx::tensors::BaseArticulationView::DofStateQuantity::eNone;
};

constexpr decltype(JointAttributeRow::gpuRead) kJointNotOnGpu = nullptr;
constexpr decltype(JointAttributeRow::cpuRead) kJointNotOnCpu = nullptr;

constexpr JointAttributeRow kJointAttributes[] = {
    { omni::physx::OvxAttr::kJointPosition, &omni::physx::tensors::GpuArticulationView::getDofPositionsOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofPositionsOvStage,
      omni::physx::tensors::BaseArticulationView::DofStateQuantity::ePosition },
    { omni::physx::OvxAttr::kJointVelocity, &omni::physx::tensors::GpuArticulationView::getDofVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofVelocitiesOvStage,
      omni::physx::tensors::BaseArticulationView::DofStateQuantity::eVelocity },
    // The per-step CONTROL values, read from the same DOF buffers as position/velocity and folded the
    // same way: degrees on an angular axis, body-order sign. The view method fixes the fold.
    { omni::physx::OvxAttr::kJointPositionTarget,
      &omni::physx::tensors::GpuArticulationView::getDofPositionTargetsOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofPositionTargetsOvStage,
      omni::physx::tensors::BaseArticulationView::DofStateQuantity::ePositionTarget },
    { omni::physx::OvxAttr::kJointVelocityTarget,
      &omni::physx::tensors::GpuArticulationView::getDofVelocityTargetsOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofVelocityTargetsOvStage,
      omni::physx::tensors::BaseArticulationView::DofStateQuantity::eVelocityTarget },
    // An effort, not a coordinate: newton-metres on an angular axis too, so the body-order sign folds
    // but the degree conversion does NOT. dofStateScalePolicy is where that pairing is decided, once
    // per quantity, so the row names the quantity and nothing here restates the fold.
    { omni::physx::OvxAttr::kJointActuationForce,
      &omni::physx::tensors::GpuArticulationView::getDofActuationForcesOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofActuationForcesOvStage,
      omni::physx::tensors::BaseArticulationView::DofStateQuantity::eActuationForce },
    // Derived from the link incoming joint force rather than read from a DOF buffer. Both backends
    // resolve the joint frame and body order while projecting, so no fold is applied on top.
    { omni::physx::OvxAttr::kJointProjectedForce,
      &omni::physx::tensors::GpuArticulationView::getDofProjectedForcesOvStage,
      &omni::physx::tensors::CpuArticulationView::getDofProjectedForcesOvStage },
};

const JointAttributeRow* findJointAttribute(std::string_view token)
{
    for (const JointAttributeRow& row : kJointAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// The joint-DOF PROPERTY attributes: authored drive/limit/friction values PhysX never writes back. A property
// enum rather than a method-pointer pair, because there is no device source to differ over -- the tensor paths
// behind these refuse a device tensor (REQ-TENSOR-CPU-ONLY-001), so one `getDofPropertiesOvStage` on
// BaseArticulationView serves both backends and takes the whole requested SET in one call, fetching each PhysX
// struct once per DOF rather than once per column. These columns are HOST-resident even on a DirectGPU scene,
// so one read can hand back a kDLCUDA jointPosition beside a kDLCPU jointStiffness.
// Width comes from dofPropertyComponents/dofPropertyIsByte.
struct JointPropertyAttributeRow
{
    const char* token;
    omni::physx::tensors::DofProperty prop;
};

constexpr JointPropertyAttributeRow kJointPropertyAttributes[] = {
    { omni::physx::OvxAttr::kJointStiffness, omni::physx::tensors::DofProperty::eStiffness },
    { omni::physx::OvxAttr::kJointDamping, omni::physx::tensors::DofProperty::eDamping },
    // (lower, upper) as ONE interval, so one column of 2 lanes. The friction and drive-model triples
    // go the other way: separately-named quantities that merely share a PhysX struct.
    { omni::physx::OvxAttr::kJointLimit, omni::physx::tensors::DofProperty::eLimit },
    { omni::physx::OvxAttr::kJointMaxVelocity, omni::physx::tensors::DofProperty::eMaxVelocity },
    { omni::physx::OvxAttr::kJointMaxForce, omni::physx::tensors::DofProperty::eMaxForce },
    { omni::physx::OvxAttr::kJointArmature, omni::physx::tensors::DofProperty::eArmature },
    { omni::physx::OvxAttr::kJointStaticFriction, omni::physx::tensors::DofProperty::eStaticFriction },
    { omni::physx::OvxAttr::kJointDynamicFriction, omni::physx::tensors::DofProperty::eDynamicFriction },
    { omni::physx::OvxAttr::kJointViscousFriction, omni::physx::tensors::DofProperty::eViscousFriction },
    { omni::physx::OvxAttr::kJointSpeedEffortGradient, omni::physx::tensors::DofProperty::eSpeedEffortGradient },
    { omni::physx::OvxAttr::kJointMaxActuatorVelocity, omni::physx::tensors::DofProperty::eMaxActuatorVelocity },
    { omni::physx::OvxAttr::kJointVelocityDependentResistance,
      omni::physx::tensors::DofProperty::eVelocityDependentResistance },
    { omni::physx::OvxAttr::kJointDriveType, omni::physx::tensors::DofProperty::eDriveType },
};

const JointPropertyAttributeRow* findJointPropertyAttribute(std::string_view token)
{
    for (const JointPropertyAttributeRow& row : kJointPropertyAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// nextRowsVersion lives in OvxPhysicsShared.h: the read and the write resolve the same rows and
// share one cache entry, so they must share its version counter too -- two counters would hand out
// the same value for different lists.
DLDataType dlTypeOf(omni::physics::tensors::TensorDataType dtype)
{
    switch (dtype)
    {
    case omni::physics::tensors::TensorDataType::eFloat32: return DLDataType{ kDLFloat, 32, 1 };
    case omni::physics::tensors::TensorDataType::eUint8:   return DLDataType{ kDLUInt, 8, 1 };
    case omni::physics::tensors::TensorDataType::eInt8:    return DLDataType{ kDLInt, 8, 1 };
    case omni::physics::tensors::TensorDataType::eInt32:   return DLDataType{ kDLInt, 32, 1 };
    default:
        CARB_LOG_ERROR_ONCE("ovxReadAttributes: no DLPack mapping for tensor dtype %d; "
                            "reporting float32, which will misdescribe the column.",
                            static_cast<int>(dtype));
        CARB_ASSERT(false);
        return DLDataType{ kDLFloat, 32, 1 };
    }
}

// Size the group's host scratch for `count` elements of `dtype` and return where to write them.
// The declared element type selects its matching C++ object storage. Width alone is insufficient:
// writing int32 values through a float vector violates C++ object lifetime and aliasing rules even
// though both elements occupy four bytes.
void* allocHostColumn(GroupStore& g, omni::physics::tensors::TensorDataType dtype, size_t count)
{
    switch (dtype)
    {
    case omni::physics::tensors::TensorDataType::eUint8:
    case omni::physics::tensors::TensorDataType::eInt8:
        g.bytes.assign(count, 0);
        return g.bytes.data();
    case omni::physics::tensors::TensorDataType::eInt32:
        g.ints.assign(count, 0);
        return g.ints.data();
    case omni::physics::tensors::TensorDataType::eFloat32:
        g.floats.assign(count, 0.0f);
        return g.floats.data();
    default:
        break;
    }
    CARB_LOG_ERROR_ONCE("ovxReadAttributes: no host column storage for tensor dtype %d.", static_cast<int>(dtype));
    CARB_ASSERT(false);
    return nullptr;
}

// Null for a name this type does not produce -- a normal answer, not a failure.
const RigidAttributeRow* findRigidAttribute(std::string_view token)
{
    for (const RigidAttributeRow& row : kRigidAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// False for a name this type does not produce, and false for one with no point-instancer form: the
// only caller is the instancer array fill, and a row without an arrayToken has nothing to publish
// under there. Returning true with a null name would crash on the std::string assignment.
bool rigidAttrInfo(const std::string& name, int& comp, std::string& arrayName)
{
    const RigidAttributeRow* row = findRigidAttribute(name);
    if (!row || !row->arrayToken)
        return false;
    comp = row->components;
    arrayName = row->arrayToken;
    return true;
}

// ADR-0008 core: given a body set (all of one RigidBodyType) + their keys, source
// pose/velocity from the tensor backend's bulk PxDirectGPUAPI read on the GPU and emit
// device-resident (kDLCUDA) columns for `names`. Shared by standalone rigid bodies and
// articulation links (a PxArticulationLink is a PxRigidBody the rigid-body view handles).
// Returns false if the device path is unavailable (caller does not emit stale host data).



// Defined below, next to sceneRunsDirectGpu. Declared here because the category emitters above it need
// the same per-scene readiness gate the articulation root and joint paths already apply.
bool directGpuSceneStepped(PxScene* scene);

// True when this scene cannot serve a DirectGPU read yet, which ADR-0008 Decision 10 makes a clean
// OMISSION rather than an error: the step-first precondition is not a failure.
//
// Per SCENE, deliberately. The backend's own readiness check is a process-wide step count
// (SimulationBackend::getStepCount), so with IPhysxSimulation::simulateScene stepping one scene at a
// time, stepping scene A satisfies it for unstepped scene B -- B then reaches a DirectGPU call PhysX
// refuses, and that refusal became backendFailed, failing the WHOLE read including A's good data.
// Gating here, before any allocation or backend work, keeps the unready partition out instead.
bool directGpuPartitionUnready(PxScene* scene)
{
    return scene && !directGpuSceneStepped(scene);
}

// Reclaim the output-column pool for any context no longer backed by a live scene -- a detach or a
// scene teardown since the last read. Runs once per read, before it allocates:
// O(scenes), and frees nothing in steady state where every context is still live. The empty-pool
// early-out keeps the common CPU / first-read path off the database walk.
void flushStaleColumnPools()
{
    {
        std::lock_guard<std::mutex> lock(g_poolMutex);
        g_poolTeardown = false; // a read means the runtime is live again -- re-enable pooling
        if (g_pool.empty())
            return;
    }
    std::unordered_set<PxCudaContextManager*> live;
    for (PxScene* scene : allPhysicsScenes())
        if (PxCudaContextManager* mgr = scene->getCudaContextManager())
            live.insert(mgr);
    poolFlushStaleContexts(live);
}

// Source pose/velocity for `bodies` (all one RigidBodyType) from the tensor backend's rigid-body view and emit
// one column per requested attribute (ADR-0008). ONE path for CPU and GPU -- the output residence falls out of
// the view: on a DirectGPU scene the columns are device-resident (kDLCUDA, correct under suppressReadback via
// the bulk PxDirectGPUAPI read); on a CPU scene they are host columns (the CPU view reads getGlobalPose()).
// Shared by standalone rigid bodies and articulation links, which the rigid-body view handles alike.
bool emitRigidBodyColumns(ReadSession& s,
                          IPhysicsSource& source,
                          PxScene* scene,
                          const std::vector<PxRigidBody*>& bodies,
                          const std::vector<ObjectKey>& keys,
                          omni::physx::tensors::RigidBodyType type,
                          const std::vector<std::string>& names,
                          uint32_t scope,
                          // The cache entry `bodies` and `keys` point into, when the caller served
                          // them from one. Passed rather than re-looked-up: reaching the same entry
                          // through operator[] can rehash the map and dangle the caller's vectors,
                          // a use-after-free rather than a wrong value. With the entry in hand this
                          // path performs no map operation at all.
                          RigidReadCacheEntry* preresolved = nullptr)
{

    if (bodies.empty())
        return true; // nothing to emit -- not a failure
    if (bodies.size() != keys.size())
    {
        CARB_LOG_ERROR("ovphysx read: rigid-body and key lists disagree (%zu bodies, %zu keys).",
                       bodies.size(), keys.size());
        s.backendFailed = true;
        return false;
    }

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    const int matchedCount = static_cast<int>(bodies.size());
    const char* kindName =
        (type == omni::physx::tensors::RigidBodyType::eArticulationLink) ? "articulation link" : "rigid body";
    // Fail-closed: on any backend build/read failure the whole set is skipped rather than emitting
    // stale host data. Warn once so a non-empty set silently yielding no column is diagnosable
    // instead of looking like "no matching objects".
    auto warnNoColumn = [&](const char* reason)
    {
        CARB_LOG_WARN_ONCE("ovphysx read: %d %s object(s) not sourced from the tensor backend (%s); "
                           "their output columns are omitted.",
                           matchedCount, kindName, reason);
        // Every backend failure funnels through here, so the session is flagged here rather than at each
        // `return false` -- callers discard those booleans.
        s.backendFailed = true;
    };

    // Read only what was asked for: skip the whole view build when no requested attribute maps to
    // this type.
    bool anyRigidAttr = false;
    for (const std::string& name : names)
        anyRigidAttr = anyRigidAttr || findRigidAttribute(name) != nullptr;
    if (!anyRigidAttr)
        return true; // no rigid-body attribute requested -- nothing to emit, not a failure

    // Before the backend is touched: an unstepped DirectGPU scene omits its partition rather than
    // failing the read. Not warnNoColumn -- that sets backendFailed, and a step-first precondition
    // is not a backend failure.
    if (directGpuPartitionUnready(scene))
        return true;

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        warnNoColumn("no tensor backend");
        return false;
    }


    if (gpu)
    {
        // DirectGPU has no state row for eDISABLE_SIMULATION rigid dynamics, so they are omitted from
        // every output column while staying in query discovery and in the backend's stable superset
        // map. CPU deliberately does not take this filter, which is why it is applied inside this
        // branch: getGlobalPose() and host properties remain available for disabled actors there.
        //
        // Gated on the scene's disable hint rather than run unconditionally: the filter's own
        // detection is a per-matched-body actor-flag read, one cache miss each, and on a scene with
        // nothing disabled it can only ever conclude "nothing to drop". The hint is conservative,
        // so a true still runs the scan; only a whole-scene "none disabled" skips it.
        const EnabledRigidBodies enabled(bodies, keys,
                                         /*apply=*/backend->sceneMayHaveDisabledRigidDynamics(scene));
        const bool anyDisabled = enabled.anyDisabled();
        const std::vector<PxRigidBody*>& outputBodies = enabled.bodies();
        const std::vector<ObjectKey>& outputKeys = enabled.keys();
        const int n = static_cast<int>(outputBodies.size());

        // disableSimulation must stay READABLE for a disabled body, mirroring the write's exemption
        // (OvxPhysicsWrite.cpp): it is the control a caller reads to see the disable and writes 0 to
        // undo it. The filter above drops disabled rows from every column, so when any body is disabled
        // and this read asks for disableSimulation, that ONE column emits over the UNFILTERED body set
        // below -- its host getter reads getActorFlags off the superset's retained sentinel row, so the
        // disabled body reports 1. Every other attribute stays filtered: a disabled body has no
        // DirectGPU state and is correctly absent from those.
        bool wantsDisableSim = false;
        for (const std::string& nm : names)
            if (std::string_view(nm) == omni::physx::OvxAttr::kDisableSimulation)
            {
                wantsDisableSim = true;
                break;
            }
        const bool disableSimUnfiltered = anyDisabled && wantsDisableSim;
        std::vector<PxU32> disableSimRowsAll; // superset rows for the unfiltered body set

        // Device-resident path: bulk read into device scratch, strided device->device extract.
        // The scene's own manager, not the process default: under multi-GPU the backend view runs in
        // this scene's context, so columns must be allocated there to be addressable by its kernels.
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            warnNoColumn("no CUDA context");
            return false;
        }
        PxCudaContext* cu = ctxMgr->getCudaContext();
        PxScopedCudaLock _lock(*ctxMgr);

        // Cached superset view over every rigid dynamic and link in the scene, reused across reads; the
        // requested subset is selected at gather time by a record list, so nothing is built per read.
        //
        // One retry, covering both the view and the row mapping: a body missing from the row map means the
        // cached view does not describe this actor set -- it predates the body, or the scene's address was
        // reused by a different scene -- so the cache is dropped and rebuilt. Giving up on a miss instead
        // would emit no columns at all for that read.
        int devOrd = 0;
        omni::physx::tensors::GpuRigidBodyView* grbv = nullptr;
        std::vector<PxU32> rowsOwned;
        bool rowsResolved = false;
        bool rowsFromCache = false;
        uint64_t generation = 0;
        // Only kOvxAll is stable across steps; the active set is recomputed each step.
        // The structural cache retains the full matched census so a later re-enable is visible even
        // though recordLifetimeEpoch does not move. Do not overwrite it with a temporarily filtered
        // enabled set; rebuild the cheap row/handle selection while any body is disabled.
        const bool cacheable = (scope == kOvxAll && !anyDisabled);
        const RigidReadCacheKey cacheKey{ scene, static_cast<int>(type), scope };
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
            omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(scene, &generation);
            grbv = sv ? sv->supersetRigidView(&rowMap) : nullptr;
            if (!grbv || !rowMap)
            {
                grbv = nullptr;
                // The superset could not be built over this scene's data, which the view cannot tell
                // apart from a scene holding no rigid bodies. Drop the entry so the next attempt
                // rebuilds; a genuinely empty scene simply fails again.
                backend->invalidateSceneCache(scene);
                continue;
            }
            devOrd = sv->getDeviceOrdinal();

            // Rows taken under this generation still address this view: the backend takes a fresh
            // generation whenever the superset view is rebuilt, including when an actor it holds is
            // destroyed -- the removal the topology check alone cannot see.
            if (cacheable)
            {
                const RigidReadCacheEntry* hit = preresolved;
                if (!hit)
                {
                    const std::unordered_map<RigidReadCacheKey, RigidReadCacheEntry,
                                             RigidReadCacheKeyHash>::const_iterator found =
                        g_rigidReadCache.find(cacheKey);
                    hit = (found != g_rigidReadCache.end()) ? &found->second : nullptr;
                }
                if (hit && hit->generation == generation &&
                    hit->rows.size() == outputBodies.size() && hit->keys == outputKeys)
                {
                    rowsResolved = true;
                    rowsFromCache = true;
                    break;
                }
            }

            rowsOwned.clear();
            rowsOwned.reserve(outputBodies.size());
            bool allResolved = true;
            for (PxRigidBody* b : outputBodies)
            {
                const std::unordered_map<const PxRigidBody*, PxU32>::const_iterator it = rowMap->find(b);
                if (it == rowMap->end())
                {
                    allResolved = false;
                    break;
                }
                rowsOwned.push_back(it->second);
            }
            // The disableSimulation exemption needs the row for EVERY matched body, disabled included;
            // the superset keeps a sentinel row for each, so this resolves against the same row map.
            if (allResolved && disableSimUnfiltered)
            {
                disableSimRowsAll.clear();
                disableSimRowsAll.reserve(bodies.size());
                for (PxRigidBody* b : bodies)
                {
                    const std::unordered_map<const PxRigidBody*, PxU32>::const_iterator it = rowMap->find(b);
                    if (it == rowMap->end())
                    {
                        allResolved = false;
                        break;
                    }
                    disableSimRowsAll.push_back(it->second);
                }
            }
            if (allResolved)
            {
                rowsResolved = true;
                break;
            }

            grbv = nullptr;
            backend->invalidateSceneCache(scene);
        }
        if (!grbv || !rowsResolved)
        {
            warnNoColumn("superset rigid-body view unavailable");
            return false;
        }
        if (!grbv->refreshDisabledRowsOvStage())
        {
            warnNoColumn("DirectGPU row refresh failed");
            return false;
        }
        // When every matched body is disabled the filtered set is empty, but disableSimulation must
        // still emit over the unfiltered set below so a caller can read and clear the disable. Only
        // short-circuit when that column is not in play; other attributes have no enabled rows and are
        // skipped per-column in the emit.
        if (outputBodies.empty() && !disableSimUnfiltered)
            return true;

        // One canonicalisation for the whole emit -- every attribute below covers the SAME prim
        // set, and canonicalising costs a lock plus a cache lookup per prim.
        std::vector<ovx_primpath_t> canonicalOwned; // only filled on the uncacheable path
        const std::vector<ovx_primpath_t>* canonicalHandles = nullptr;
        const std::vector<PxU32>* rowsPtr = &rowsOwned;
        uint64_t rowsVersion = 0;
        if (cacheable)
        {
            // operator[] only when the caller had no entry: with one, inserting here is exactly
            // the rehash that would dangle its vectors.
            RigidReadCacheEntry& ce = preresolved ? *preresolved : g_rigidReadCache[cacheKey];
            // Refreshed together, because they are all valid for exactly one generation: the rows
            // address the superset view it names, and it pins the dictionary the handles were
            // interned into. Generation is not on its own enough to serve them, which is why the
            // hit above also compares the key list: an unchanged generation does NOT prove an
            // unchanged body set, only that the superset view was not rebuilt.
            if (!rowsFromCache)
            {
                ce.generation = generation;
                ce.dbEpoch = omni::physx::internal::recordLifetimeEpoch();
                ce.keys = outputKeys;
                ce.handles.clear();
                omni::physics::ovstage::canonicalisePathHandles(source, outputKeys.data(), outputKeys.size(),
                                                                ce.handles, nullptr);
                ce.rows = std::move(rowsOwned);
                ce.rowsVersion = nextRowsVersion(); // a new list, so any device copy is stale
            }
            rowsVersion = ce.rowsVersion;
            rowsPtr = &ce.rows;
            // Point AT the entry rather than copying its handles out. Safe because nothing erases from the
            // map between here and the read that uses it: the purge runs once at the top of the read, and
            // this scene is live by construction.
            canonicalHandles = &ce.handles;
        }
        else
        {
            omni::physics::ovstage::canonicalisePathHandles(source, outputKeys.data(), outputKeys.size(),
                                                            canonicalOwned, nullptr);
            canonicalHandles = &canonicalOwned;
            // kOvxActive -- and kOvxAll while a disabled body is filtered out -- rebuilds its rows
            // every read by design, so there is nothing to compare against and no basis for reusing
            // the device copy. A fresh version forces the upload.
            rowsVersion = nextRowsVersion();
        }
        const std::vector<PxU32>& rows = *rowsPtr;

        // The unfiltered disableSimulation column (see above) covers a DIFFERENT prim set than the
        // shared filtered one, so it canonicalises its own handles and builds its own prim list.
        std::vector<ovx_primpath_t> disableSimHandlesAll;
        ovx_primpath_list_t disableSimPrimList = OVX_INVALID_PRIMPATH_LIST;
        if (disableSimUnfiltered)
            omni::physics::ovstage::canonicalisePathHandles(source, keys.data(), keys.size(),
                                                            disableSimHandlesAll, nullptr);

        // Held by the view, not allocated and uploaded per read: the view owns the buffer -- so the session
        // does not track it -- and re-uploads when this token changes. The token IS the row list's version,
        // so it changes exactly when the list does.
        const uint64_t rowsToken = rowsVersion;
        bool ok = true;

        // One allocation for every column this read emits, sub-allocated below, since a device allocation
        // costs about as much as the gather kernels themselves. The columns stay individually addressable --
        // each group points at its own span -- and the session frees the single base pointer.
        CUdeviceptr columnBase = 0;
        if (ok)
        {
            size_t totalFloats = 0;
            for (const std::string& name : names)
            {
                const RigidAttributeRow* row = findRigidAttribute(name);
                // hostOnly columns are not in this allocation: they have no device source, so they
                // are filled into the group's own host scratch further down.
                if (row && row->gpuRead && !row->hostOnly)
                    totalFloats += alignColumnFloats(size_t(n) * row->components);
            }
            if (totalFloats == 0)
            {
                ok = true; // no requested attribute produces a column for this type
            }
            else
            {
                // Pool-or-allocate; acquireColumn also registers it for release.
                columnBase = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, totalFloats * sizeof(float)));
                if (!columnBase)
                {
                    warnNoColumn("column allocation failed");
                    ok = false;
                }
            }
        }

        // Every column below covers the SAME prims, so they share one list.
        ovx_primpath_list_t sharedPrimList = OVX_INVALID_PRIMPATH_LIST;
        size_t columnCursor = 0;
        // Once for the emit, not once per attribute: it is an O(rows) walk and every per-shape column
        // in this read shares the answer. -1 until a per-shape attribute asks, so a read without one
        // never walks at all.
        int perShapeWidth = -1;
        // The per-shape columns are host work even here: all five are hostOnly and their gpuRead and cpuRead
        // are the SAME BaseRigidBodyView method, so this branch takes the same set-form divert as the host
        // one and resolves each shape's material once for the whole set instead of once per column.
        constexpr size_t kNoShapeSlotDev = static_cast<size_t>(-1);
        std::vector<size_t> devShapeSlotForName(names.size(), kNoShapeSlotDev);
        std::vector<GroupStore> devShapeGroups;
        if (ok)
        {
            std::vector<const RigidAttributeRow*> shapeRows;
            std::vector<size_t> shapeNameIdx; // position in `names`, so duplicates get their own slot
            for (size_t idx = 0; idx < names.size(); ++idx)
            {
                const RigidAttributeRow* row = findRigidAttribute(names[idx]);
                if (row && row->gpuRead && row->hostOnly && row->perShape)
                {
                    shapeRows.push_back(row);
                    shapeNameIdx.push_back(idx);
                }
            }
            if (shapeRows.size() > 1)
            {
                perShapeWidth = static_cast<int>(grbv->maxShapesForRows(rows.data(), static_cast<uint32_t>(n)));
                if (perShapeWidth > 0)
                {
                    const size_t numShapeCols = shapeRows.size();
                    devShapeGroups.resize(numShapeCols);
                    std::vector<omni::physics::tensors::TensorDesc> descs(numShapeCols);
                    std::vector<omni::physx::tensors::BaseRigidBodyView::ShapePropertyColumn> columns(numShapeCols);
                    for (size_t k = 0; k < numShapeCols; ++k)
                    {
                        devShapeGroups[k].isArray = false;
                        devShapeGroups[k].dtype = dlTypeOf(shapeRows[k]->dtype);
                        descs[k].device = -1; // host, as the per-column path below also emits
                        descs[k].dtype = shapeRows[k]->dtype;
                        descs[k].numDims = 2;
                        descs[k].dims[0] = n;
                        descs[k].dims[1] = perShapeWidth;
                        descs[k].data = allocHostColumn(devShapeGroups[k], shapeRows[k]->dtype,
                                                        size_t(n) * perShapeWidth);
                        columns[k].property = shapeRows[k]->shapeProperty;
                        columns[k].dst = &descs[k];
                    }
                    if (grbv->getShapePropertyColumnsOvStage(columns.data(), static_cast<PxU32>(numShapeCols),
                                                             rows.data(), static_cast<uint32_t>(n)))
                    {
                        for (size_t k = 0; k < numShapeCols; ++k)
                            devShapeSlotForName[shapeNameIdx[k]] = k;
                    }
                    else
                    {
                        ok = false;
                    }
                }
            }
        }

        // Not gated on columnBase: a read asking only for host-only properties allocates no device
        // memory at all, and gating on it would emit nothing for such a read. A device-backed row can
        // only exist here when the sizing pass found one, which is exactly when columnBase is set.
        if (ok)
        {
            size_t devNameSlot = 0;
            for (const std::string& name : names)
            {
                const size_t nameIdx = devNameSlot++; // before any `continue`, so it tracks `names`
                const RigidAttributeRow* row = findRigidAttribute(name);
                // No column when this type does not produce the attribute, or when this backend
                // structurally cannot serve it. Both are "not applicable here", not failures, so
                // neither emits a group -- the sizing pass above skips them identically.
                if (!row || !row->gpuRead)
                    continue;
                const int comp = row->components;

                // Filled by the set call above; emitted here so group order still follows `names`.
                if (devShapeSlotForName[nameIdx] != kNoShapeSlotDev)
                {
                    // A false return means NO group was appended (the prim list could not be
                    // built). Ignoring it ends the read successfully with the column missing.
                    if (!finalizeGroup(s, source, std::move(devShapeGroups[devShapeSlotForName[nameIdx]]), name,
                                       outputKeys, static_cast<int64_t>(n), perShapeWidth, canonicalHandles,
                                       &sharedPrimList))
                    {
                        ok = false;
                        break;
                    }
                    continue;
                }

                // A property column on a DirectGPU scene: read on the host into the group's own
                // scratch and emit kDLCPU. Mixed residency within one read is deliberate and is what
                // the attribute actually is -- there is no device copy of an authored input to read,
                // so uploading one would invent a device residency rather than preserve it. The
                // group's dtype comes off the row, which is how the uint8 flags stay bytes.
                if (row->hostOnly)
                {
                    // A per-shape column's width is this read's widest body, taken from the same view
                    // and row list the values come from -- not the view's mMaxShapes, which spans the
                    // whole scene and would let an unrelated many-shaped body widen every read.
                    if (row->perShape && perShapeWidth < 0)
                        perShapeWidth = static_cast<int>(
                            grbv->maxShapesForRows(rows.data(), static_cast<uint32_t>(n)));
                    const int width = row->perShape ? perShapeWidth : comp;
                    if (width == 0)
                    {
                        // No body in this read has a shape, so the column has no width to publish.
                        CARB_LOG_WARN_ONCE("ovxReadAttributes: '%s' skipped -- no body in the read has a "
                                           "collision shape.", name.c_str());
                        continue;
                    }
                    // disableSimulation on a scene with a disabled body emits over the unfiltered set so
                    // its disabled row (value 1) is present; every other host-only column stays filtered.
                    // It is never per-shape, so the width above is unaffected.
                    const bool useAll = disableSimUnfiltered &&
                                        std::string_view(name) == omni::physx::OvxAttr::kDisableSimulation;
                    // No enabled rows and not the unfiltered disableSimulation column: this host-only
                    // property (mass, inertia, centerOfMass, disableGravity, ...) has no rows to gather,
                    // and its host getter rejects a null column, so emit nothing for it -- mirroring the
                    // device-column skip below. Per-shape columns already fell out at width == 0 above.
                    if (!useAll && n == 0)
                        continue;
                    const int colN = useAll ? static_cast<int>(bodies.size()) : n;
                    const PxU32* const colRows = useAll ? disableSimRowsAll.data() : rows.data();
                    const std::vector<ObjectKey>& colKeys = useAll ? keys : outputKeys;
                    const std::vector<ovx_primpath_t>* const colHandles =
                        useAll ? &disableSimHandlesAll : canonicalHandles;
                    ovx_primpath_list_t* const colPrimList = useAll ? &disableSimPrimList : &sharedPrimList;

                    GroupStore hg;
                    hg.isArray = false;
                    hg.dtype = dlTypeOf(row->dtype);
                    omni::physics::tensors::TensorDesc htd;
                    htd.device = -1;
                    htd.dtype = row->dtype;
                    htd.numDims = 2; htd.dims[0] = colN; htd.dims[1] = width;
                    htd.data = allocHostColumn(hg, row->dtype, size_t(colN) * width);
                    if (!(grbv->*row->gpuRead)(&htd, colRows, static_cast<uint32_t>(colN), rowsToken))
                    {
                        ok = false;
                        break;
                    }
                    if (!finalizeGroup(s, source, std::move(hg), name, colKeys, static_cast<int64_t>(colN), width,
                                       colHandles, colPrimList))
                    {
                        ok = false;
                        break;
                    }
                    continue;
                }

                // No enabled rows (every matched body is disabled): this device attribute has no
                // column to gather, and getPoseColumnOvStage rejects a null one, so emit nothing for
                // it. disableSimulation above already covered the disabled set over the unfiltered rows.
                if (n == 0)
                    continue;

                // A failure past this point stops the read rather than skipping the column: the
                // `continue` above means "this attribute does not apply to this type", so silently
                // reusing it here would make a gather failure indistinguishable from a caller
                // asking for an attribute the type never produces.
                const CUdeviceptr colBuf = columnBase + columnCursor * sizeof(float);
                columnCursor += alignColumnFloats(size_t(n) * comp);

                omni::physics::tensors::TensorDesc td;
                td.device = devOrd; td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
                td.numDims = 2; td.dims[0] = n; td.dims[1] = comp; td.data = reinterpret_cast<void*>(colBuf);

                // `rows` maps destination slot -> superset row, so the cached view serves this
                // read's subset without being rebuilt for it.
                if (!(grbv->*row->gpuRead)(&td, rows.data(), static_cast<uint32_t>(n), rowsToken))
                {
                    ok = false;
                    break;
                }
                GroupStore g;
                g.isArray = false;
                g.deviceData = reinterpret_cast<void*>(colBuf);
                g.deviceOrdinal = devOrd;
                g.ctxMgr = ctxMgr;
                if (!finalizeGroup(s, source, std::move(g), name, outputKeys, static_cast<int64_t>(n), comp,
                                   canonicalHandles, &sharedPrimList))
                {
                    ok = false;
                    break;
                }
            }
        }
        // No synchronize: everything above is ordered on this context's null stream and the session
        // records a completion event per context once all groups are built, handed to the consumer
        // through data.cuda_sync. Nothing here frees device memory, so there is nothing to wait for.
        if (!ok)
            warnNoColumn("device read failed");
        // The superset view is backend-owned and outlives this read; nothing to release here.
        return ok;
    }
    else
    {
        const int n = static_cast<int>(bodies.size());
        // Host path: each column is written straight from the actors into the group's storage, with no
        // intermediate and no second strided pass, and only the component actually requested is queried off
        // the body. Served by the backend's cached superset view exactly as the device branch is, so no view
        // is constructed per read.
        omni::physx::tensors::CpuRigidBodyView* crbv = nullptr;
        std::vector<PxU32> rowsOwned;
        bool rowsResolved = false;
        bool rowsFromCache = false;
        uint64_t generation = 0;
        const bool cacheable = (scope == kOvxAll);
        const RigidReadCacheKey cacheKey{ scene, static_cast<int>(type), scope };
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
            omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(scene, &generation);
            crbv = sv ? sv->supersetRigidView(&rowMap) : nullptr;
            if (!crbv || !rowMap)
            {
                crbv = nullptr;
                backend->invalidateSceneCache(scene);
                continue;
            }

            if (cacheable)
            {
                const RigidReadCacheEntry* hit = preresolved;
                if (!hit)
                {
                    const std::unordered_map<RigidReadCacheKey, RigidReadCacheEntry,
                                             RigidReadCacheKeyHash>::const_iterator found =
                        g_rigidReadCache.find(cacheKey);
                    hit = (found != g_rigidReadCache.end()) ? &found->second : nullptr;
                }
                if (hit && hit->generation == generation &&
                    hit->rows.size() == bodies.size() && hit->keys == keys)
                {
                    rowsResolved = true;
                    rowsFromCache = true;
                    break;
                }
            }

            rowsOwned.clear();
            rowsOwned.reserve(bodies.size());
            bool allResolved = true;
            for (PxRigidBody* b : bodies)
            {
                const std::unordered_map<const PxRigidBody*, PxU32>::const_iterator it = rowMap->find(b);
                if (it == rowMap->end())
                {
                    allResolved = false;
                    break;
                }
                rowsOwned.push_back(it->second);
            }
            if (allResolved)
            {
                rowsResolved = true;
                break;
            }

            crbv = nullptr;
            backend->invalidateSceneCache(scene);
        }
        if (!crbv || !rowsResolved)
        {
            warnNoColumn("superset rigid-body view unavailable");
            return false;
        }

        // Same once-per-emit canonicalisation as the device branch, cached against the same generation.
        std::vector<ovx_primpath_t> canonicalOwned; // only filled on the uncacheable path
        const std::vector<ovx_primpath_t>* canonicalHandles = nullptr;
        const std::vector<PxU32>* rowsPtr = &rowsOwned;
        if (cacheable)
        {
            // operator[] only when the caller had no entry: with one, inserting here is exactly
            // the rehash that would dangle its vectors.
            RigidReadCacheEntry& ce = preresolved ? *preresolved : g_rigidReadCache[cacheKey];
            if (!rowsFromCache)
            {
                ce.generation = generation;
                ce.dbEpoch = omni::physx::internal::recordLifetimeEpoch();
                ce.keys = keys;
                ce.handles.clear();
                omni::physics::ovstage::canonicalisePathHandles(source, keys.data(), keys.size(),
                                                                ce.handles, nullptr);
                ce.rows = std::move(rowsOwned);
                // Stamped on the host path too, though nothing here reads it: RigidReadCacheKey has
                // no device component, so this entry is the one a later device read looks up. A new
                // list built here has to invalidate the device copy that read would otherwise reuse.
                ce.rowsVersion = nextRowsVersion();
            }
            rowsPtr = &ce.rows;
            canonicalHandles = &ce.handles;
        }
        else
        {
            omni::physics::ovstage::canonicalisePathHandles(source, keys.data(), keys.size(),
                                                            canonicalOwned, nullptr);
            canonicalHandles = &canonicalOwned;
        }
        const std::vector<PxU32>& rows = *rowsPtr;

        bool ok = true;
        // Every column below covers the SAME prims, so they share one list -- as the device branch does --
        // rather than each attribute interning its own copy of an identical path list.
        ovx_primpath_list_t sharedPrimList = OVX_INVALID_PRIMPATH_LIST;
        int perShapeWidth = -1; // as on the device branch: one O(rows) walk shared by every per-shape column

        // Per-shape columns are FILLED here, in one backend call, and emitted by the loop below in its own
        // order -- so this changes what the read pays, not what it publishes or in which sequence. They share
        // one walk of every body's shapes, and three of the five (staticFriction, dynamicFriction,
        // restitution) also share the shape->material resolve. Only worth it above one column, since a single
        // per-shape column already makes exactly one walk.
        //
        // Indexed by POSITION in `names`, not keyed by row: `names` is the caller's list verbatim, so two
        // occurrences of one attribute are a reachable input from the public C API and resolve to the SAME
        // row pointer. Positions are unique, so duplicates get their own slot and every group is moved once.
        constexpr size_t kNoShapeSlot = static_cast<size_t>(-1);
        std::vector<size_t> shapeSlotForName(names.size(), kNoShapeSlot);
        std::vector<GroupStore> shapeGroups;
        {
            std::vector<const RigidAttributeRow*> shapeRows;
            std::vector<size_t> shapeNameIdx; // parallel: which position in `names` each column came from
            for (size_t idx = 0; idx < names.size(); ++idx)
            {
                const RigidAttributeRow* row = findRigidAttribute(names[idx]);
                if (row && row->cpuRead && row->perShape)
                {
                    shapeRows.push_back(row);
                    shapeNameIdx.push_back(idx);
                }
            }
            if (shapeRows.size() > 1)
            {
                perShapeWidth = static_cast<int>(crbv->maxShapesForRows(rows.data(), static_cast<uint32_t>(n)));
                if (perShapeWidth > 0)
                {
                    const size_t numShapeCols = shapeRows.size();
                    shapeGroups.resize(numShapeCols);
                    std::vector<omni::physics::tensors::TensorDesc> descs(numShapeCols);
                    std::vector<omni::physx::tensors::BaseRigidBodyView::ShapePropertyColumn> columns(numShapeCols);
                    for (size_t k = 0; k < numShapeCols; ++k)
                    {
                        shapeGroups[k].isArray = false;
                        shapeGroups[k].dtype = dlTypeOf(shapeRows[k]->dtype);
                        descs[k].device = -1;
                        descs[k].dtype = shapeRows[k]->dtype;
                        descs[k].numDims = 2;
                        descs[k].dims[0] = n;
                        descs[k].dims[1] = perShapeWidth;
                        descs[k].data = allocHostColumn(shapeGroups[k], shapeRows[k]->dtype,
                                                        size_t(n) * perShapeWidth);
                        columns[k].property = shapeRows[k]->shapeProperty;
                        columns[k].dst = &descs[k];
                    }
                    if (crbv->getShapePropertyColumnsOvStage(columns.data(), static_cast<PxU32>(numShapeCols),
                                                             rows.data(), static_cast<uint32_t>(n)))
                    {
                        for (size_t k = 0; k < numShapeCols; ++k)
                            shapeSlotForName[shapeNameIdx[k]] = k;
                    }
                    else
                    {
                        ok = false;
                    }
                }
            }
        }

        size_t nameSlot = 0;
        for (const std::string& name : names)
        {
            const size_t nameIdx = nameSlot++; // advanced before any `continue`, so it tracks `names`
            if (!ok)
                break;
            const RigidAttributeRow* row = findRigidAttribute(name);
            if (!row || !row->cpuRead)
                continue;
            const int comp = row->components;

            // Filled by the set call above; emitted here so the group order still follows `names`.
            if (shapeSlotForName[nameIdx] != kNoShapeSlot)
            {
                if (!finalizeGroup(s, source, std::move(shapeGroups[shapeSlotForName[nameIdx]]), name, keys,
                                   static_cast<int64_t>(n), perShapeWidth, canonicalHandles, &sharedPrimList))
                {
                    ok = false;
                    break;
                }
                continue;
            }

            // Same per-shape width resolution as the device branch, from this branch's view, and
            // likewise computed once for the emit rather than once per column.
            if (row->perShape && perShapeWidth < 0)
                perShapeWidth =
                    static_cast<int>(crbv->maxShapesForRows(rows.data(), static_cast<uint32_t>(n)));
            const int width = row->perShape ? perShapeWidth : comp;
            if (width == 0)
            {
                CARB_LOG_WARN_ONCE("ovxReadAttributes: '%s' skipped -- no body in the read has a "
                                   "collision shape.", name.c_str());
                continue;
            }

            GroupStore g;
            g.isArray = false;
            g.dtype = dlTypeOf(row->dtype);

            omni::physics::tensors::TensorDesc td;
            td.device = -1; td.dtype = row->dtype;
            td.numDims = 2; td.dims[0] = n; td.dims[1] = width;
            td.data = allocHostColumn(g, row->dtype, size_t(n) * width);

            // Indexed through the superset: the view covers the whole scene, the rows select this
            // read's subset. rowsToken is unused on the host -- there is no device copy to keep
            // current -- and exists so both backends share one signature.
            if (!(crbv->*row->cpuRead)(&td, rows.data(), static_cast<uint32_t>(n), 0))
            {
                ok = false;
                break;
            }
            if (!finalizeGroup(s, source, std::move(g), name, keys, static_cast<int64_t>(n), width,
                               canonicalHandles, &sharedPrimList))
            {
                ok = false;
                break;
            }
        }
        if (!ok)
            warnNoColumn("host read failed");
        // The superset view is backend-owned and outlives this read; nothing to release here.
        return ok;
    }
}

// `activeSet` is supplied rather than resolved here because the only caller runs this ONCE PER
// SCENE, and resolving inside made the whole set a per-scene rebuild -- O(scenes x records) for a
// value that does not depend on the scene. It is still consulted only under kOvxActive, so a caller
// passing an empty set gets exactly what an ALL scope got before.
void enumerateStandaloneRigid(uint32_t scope,
                              PxScene* scene,
                              const ActiveActorSet& activeSet,
                              std::vector<PxRigidBody*>& bodies,
                              std::vector<ObjectKey>& keys)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const std::vector<InternalDatabase::Record>& records = db.getRecords();
    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != ePTActor || !rec.mPtr || !rec.mInternalPtr)
            continue;
        PxRigidDynamic* dyn = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>();
        if (!dyn)
            continue;
        if (dyn->getScene() != scene)
            continue; // read each body through its owning scene's view (per-scene GPU indices)
        InternalActor* ia = reinterpret_cast<InternalActor*>(rec.mInternalPtr);
        if (ia && ia->mInstanceIndex != kInvalidUint32_t)
            continue; // point-instancer instance
        if (scope == kOvxActive)
        {
            const bool active = activeSet.sceneReports(dyn->getScene())
                ? activeSet.isActive(ri)
                : !((dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) || dyn->isSleeping());
            if (!active)
                continue;
        }
        bodies.push_back(dyn);
        keys.push_back(rec.mKey);
    }
}

// The record walk, run at most once per read and only when some scene needs it.
// Standalone bodies for one scene, from the cache when the backend reports the generation they were
// resolved under, and from the walk otherwise.
bool buildStandaloneRigid(ReadSession& s, IPhysicsSource& source, PxScene* scene,
                          uint32_t scope, const std::vector<std::string>& names,
                          LazyRigidScan& lazy)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    const RigidReadCacheKey cacheKey{ scene, static_cast<int>(omni::physx::tensors::RigidBodyType::eRigidDynamic),
                                      scope };

    // The body set is derived from the object database and from nothing else, so it is unchanged exactly
    // while no object has been created or retired -- which the epoch answers on its own, with no view to
    // acquire. The rows and handles still need the generation, checked by the emit below, which has to
    // acquire a view anyway in order to read.
    if (backend && scope == omni::physx::kOvxAll)
    {
        const std::unordered_map<RigidReadCacheKey, RigidReadCacheEntry, RigidReadCacheKeyHash>::iterator
            epochHit = g_rigidReadCache.find(cacheKey);
        if (epochHit != g_rigidReadCache.end() &&
            epochHit->second.dbEpoch == omni::physx::internal::recordLifetimeEpoch() &&
            !epochHit->second.bodies.empty() &&
            epochHit->second.bodies.size() == epochHit->second.keys.size())
        {
            // The entry goes WITH the vectors that point into it: handed over, the emit performs no
            // map operation, so nothing can rehash underneath them while they are in use.
            RigidReadCacheEntry& ce = epochHit->second;
            return emitRigidBodyColumns(s, source, scene, ce.bodies, ce.keys,
                                        omni::physx::tensors::RigidBodyType::eRigidDynamic, names, scope,
                                        &ce);
        }
    }

    const RigidSceneBucket* bucket = bucketForScene(lazy.get(), scene);
    if (!bucket)
        return true; // this scene owns no standalone bodies; not an error

    // Stored before the emit, which sets the generation these belong to once it has refreshed the
    // rows and handles alongside them.
    if (scope == omni::physx::kOvxAll)
        g_rigidReadCache[cacheKey].bodies = bucket->bodies;

    return emitRigidBodyColumns(s, source, scene, bucket->bodies, bucket->keys,
                                omni::physx::tensors::RigidBodyType::eRigidDynamic, names, scope);
}

// True when a scene runs DirectGPU, which is the only shape the device instancer path serves. Whether the
// instanced bodies SHARE that scene is answered off the record walk instead, which already has every actor in
// cache when it can answer for free.
// The readiness test without the memo. directGpuSceneReady caches the same answer on an
// articulation entry; the instancer path has no such entry and needs to ask directly.
bool directGpuSceneStepped(PxScene* scene)
{
    return !scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) || scene->getTimestamp() > 1;
}

bool sceneRunsDirectGpu(PxScene* const scene)
{
    // The context manager is the extra this path needs beyond the flag, since it allocates and gathers on the
    // scene's own context.
    return scene && (scene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
           scene->getCudaContextManager() != nullptr;
}


// The link counterpart of LazyRigidScan: the record walk, run at most once per read and only when
// some scene needs it -- and, for kOvxAll, at most once per structural change.
//
// The walk produces link POINTERS and keys, which are derived from the object database and from
// nothing else, so they are unchanged exactly while no object has been created or retired. The
// database's lifetime epoch says that directly, so a stepping loop walks once and then never again.
// Deliberately not the view generation: that answers "was the superset view rebuilt", which is the
// wrong question for a set of pointers the view had no part in producing.
//
// kOvxActive is never cached, for the same reason the rigid read does not cache it: the active set
// is recomputed every step, so a walk skipped on an unchanged epoch would serve the previous step's
// membership.
struct CachedLinkScan
{
    uint64_t dbEpoch = 0; // 0 = nothing cached
    RigidRecordScan data;

    // What buildLinkIncomingJointForce derives from `data`, kept here so one epoch governs the scan and
    // everything read off it. Two validators, because the two halves go stale for different reasons -- the
    // same split g_articulationReadCache documents:
    //
    //   dbEpoch     covers the enumeration below. It is a filter over the object database, so it is
    //               unchanged exactly while no object has been created or removed.
    //   generation  covers `recs`. A record names a SUPERSET ROW, valid only for the view build that
    //               numbered the rows, and the backend takes a fresh generation when it rebuilds.
    //
    // Caching the records on the epoch alone would survive a view rebuild and silently return another
    // selection's values. `enumValid` is separate from `dbEpoch != 0` because a scene with no articulation
    // links caches an empty, valid enumeration.
    bool enumValid = false;
    std::vector<PxArticulationReducedCoordinate*> artis;
    std::vector<uint32_t> localArtiIdx;
    std::vector<uint32_t> linkIdx;
    std::vector<ObjectKey> linkKeys;
    std::vector<ovx_primpath_t> linkHandles;

    uint64_t generation = 0; // 0 = no records cached
    // Content version of `recs`, minted on each rebuild; the device record upload keys on this (see
    // buildLinkIncomingJointForce), so it re-uploads on any content change, not only a generation move.
    uint64_t recsVersion = 0;
    std::vector<omni::physx::tensors::ArticulationLinkOvStageRecord> recs;
};
std::unordered_map<const PxScene*, CachedLinkScan> g_linkScanCache;

struct LazyLinkScan
{
    uint32_t scope;
    bool ran = false;
    RigidRecordScan data;

    const RigidRecordScan& get()
    {
        if (!ran)
        {
            // A local, not a member: `ran` already makes this body run at most once per holder, and
            // nothing outside consumes the set. LazyRigidScan holds its equivalent because the
            // instancer fallback reads it too, which is a reason this holder does not have.
            const ActiveActorSet activeSet =
                (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};
            scanLinkRecords(scope, activeSet, data);
            ran = true;
        }
        return data;
    }
};

// What a link advertises on top of a rigid body's set. A named list, not a token comparison at its one
// consumer: attrNamesForType appends this and the `wanted` check below accepts it, so per AC-2 of
// REQ-READ-ATTRS-001 discovery cannot drift from what is served.
constexpr const char* kArticulationLinkOnlyAttributes[] = { omni::physx::OvxAttr::kLinkIncomingJointForce };

// The one link attribute a rigid-body view cannot serve: the value lives in the ARTICULATION view (the
// articulation cache on CPU, the link incoming-joint-force buffer on GPU). Defined below where that
// view binding already lives, declared here because the link path calls it.
bool buildLinkIncomingJointForce(ReadSession& s,
                                 IPhysicsSource& source,
                                 PxScene* scene,
                                 const std::vector<PxRigidBody*>& bodies,
                                 const std::vector<ObjectKey>& keys,
                                 const std::vector<std::string>& names,
                                 CachedLinkScan* cache);

// Articulation links (a PxArticulationLink is a PxRigidBody the rigid-body view handles),
// honoring ACTIVE scope. Sourced from the tensor backend view (device-resident on GPU, host on CPU).
bool buildArticulationLinks(ReadSession& s, IPhysicsSource& source, PxScene* scene,
                            uint32_t scope, const std::vector<std::string>& names, LazyLinkScan& lazy)
{
    const RigidSceneBucket* bucket = nullptr;
    CachedLinkScan* linkCache = nullptr;
    if (scope == omni::physx::kOvxAll)
    {
        CachedLinkScan& lc = g_linkScanCache[scene];
        const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();
        if (lc.dbEpoch != dbEpochNow)
        {
            lc.data = lazy.get(); // this read's single walk, kept for the reads that follow
            lc.dbEpoch = dbEpochNow;
            // Everything derived from the scan dies with it: a new epoch means links may have been
            // created or removed, so the enumeration, the handles interned from it and the records
            // built off it are all gone. Clearing the handles is not optional -- they are rebuilt on
            // a SIZE mismatch, so an epoch that changed which links exist without changing how many
            // would otherwise reuse the previous set's paths.
            lc.enumValid = false;
            lc.generation = 0;
            lc.linkHandles.clear();
        }
        bucket = bucketForScene(lc.data, scene);
        linkCache = &lc;
    }
    else
    {
        bucket = bucketForScene(lazy.get(), scene);
    }
    if (!bucket)
        return true; // this scene owns no links; not an error

    const bool rigidOk = emitRigidBodyColumns(s, source, scene, bucket->bodies, bucket->keys,
                                              omni::physx::tensors::RigidBodyType::eArticulationLink, names, scope);
    // Over the SAME prim list and the same scope, so a read asking for pose and linkIncomingJointForce
    // together gets both over one link set. Not folded into the rigid emit above because the source
    // is a different view; a read asking for neither pays one string comparison per requested name.
    const bool artiOk =
        buildLinkIncomingJointForce(s, source, scene, bucket->bodies, bucket->keys, names, linkCache);
    return rigidOk && artiOk;
}

// ----------------------------------------------------------------------------
// Point-instancer groups, sourced from the device
// ----------------------------------------------------------------------------

// The instancer view for one scene, cached across reads exactly as the rigid read caches its
// superset view: keyed by the object-lifetime epoch (the instance SET changed) and by the backend
// generation (the parent simulation view was rebuilt, which released this as its child). The
// pointer is never dereferenced when either has moved, so a released view is dropped rather than
// followed.
struct InstancerViewCacheEntry
{
    uint64_t dbEpoch = 0;
    uint64_t generation = 0;
    // Whether this entry has been built for (dbEpoch, generation). Separate from `view` because a
    // scene can legitimately build to NO instancers: `view` stays null there, so keying the lookup
    // on the pointer alone reads a cached empty answer as a miss and re-enumerates on every read.
    bool built = false;
    // Base pointer, not the GPU type: one cache serves both backends. A CPU scene reads
    // instancers through CpuPointInstancerView, and everything this builder asks of the view --
    // check(), release(), setWorldInverses(), getInstancerColumnsOvStage() -- is on the base, whose
    // destructor is virtual so release() deletes the right one.
    omni::physx::tensors::BasePointInstancerView* view = nullptr;
    // What the emit needs about each instancer, in the order the view was built in. Held here so a cache HIT
    // does not have to enumerate at all -- rebuilding these means walking the object database and
    // materialising one description per instance.
    //
    // Safe to hold across reads because all of it is fixed while the object-lifetime epoch is: the instancer
    // set, each instancer's key, and the highest instance index it publishes all change only by creating or
    // retiring a record. The instancer's world TRANSFORM is not in here, and must not be -- see AC-6.
    std::vector<ObjectKey> keys;
    std::vector<uint32_t> maxIndex;
    // Instancer key handle -> its slot in the two vectors above. One entry per INSTANCER, not per
    // instance, so it is small. It exists so an ACTIVE read can turn the actors PhysX reports as
    // moved into "which instancers may publish" without re-enumerating -- see
    // computeActiveInstancerMask.
    std::unordered_map<uint64_t, size_t> slotForInstancer;
};
std::unordered_map<const PxScene*, InstancerViewCacheEntry> g_instancerViewCache;

// The HOST instancer view, cached per scene for the write. Separate from g_instancerViewCache above,
// which holds a GpuPointInstancerView and serves the device read.
//
// This is the first caller of CpuSimulationView::createPointInstancerViewFromEntries -- the host
// READ reframes instances directly and never built one. That is why the write gets its own cache
// rather than sharing the read's: there is no existing host view to share.
struct HostInstancerViewCacheEntry
{
    uint64_t dbEpoch = 0;
    uint64_t generation = 0;
    omni::physx::tensors::CpuPointInstancerView* view = nullptr;
    std::vector<ObjectKey> keys;
    std::vector<uint32_t> arrayLengths;
};
std::unordered_map<const PxScene*, HostInstancerViewCacheEntry> g_hostInstancerViewCache;

} // namespace (reopened below)

namespace omni::physx::ovx
{
// Deterministic teardown: free EVERY pooled output-column buffer and drop every manager reference,
// ahead of the runtime's CUDA/foundation shutdown. Called from OmniPhysX::onShutdown before
// releasePhysics(), while the context managers are still alive -- the pool's own references keep them
// so. The pool's only other drain is the stale sweep at the next read (flushStaleColumnPools), so a
// final detach + shutdown with no subsequent read would otherwise leave device/pinned buffers and
// manager references alive across CUDA teardown. Draining on plain detach is deliberately NOT done:
// the CUDA context is reused across a Play/Stop/Play cycle, so that would throw away the warm pool the
// next read reuses; the next read's stale sweep already retires a genuinely dead context.
void ovxDrainColumnPools()
{
    {
        std::lock_guard<std::mutex> lock(g_poolMutex);
        g_poolTeardown = true; // a late session return during teardown frees rather than re-pools
    }
    poolFlushStaleContexts({}); // empty live set: reclaim every pooled context
}

// Declared in OvxPhysicsShared.h. Lives here because everything it touches does: enumerateInstancers,
// the Gf world-transform query, and the entry type the view is built from.
bool instancerWriteTargets(::physx::PxScene* scene, const uint32_t scope, InstancerWriteTargets& out)
{
    out.view = nullptr;
    out.keys.clear();
    out.arrayLengths.clear();

    ActiveContext ctx;
    if (!getActiveContext("instancerWriteTargets", ctx))
        return false;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    // The generation comes from the view that matches the SCENE's pipeline. Acquiring the CPU one
    // unconditionally -- which an earlier version did -- returns null on a DirectGPU scene and failed
    // the whole write session with nothing logged, since the device branch below never ran.
    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    uint64_t generation = 0;
    omni::physx::tensors::CpuSimulationView* sv = nullptr;
    if (gpu)
    {
        if (!backend->acquireSceneView(scene, &generation))
            return false;
    }
    else
    {
        sv = backend->acquireCpuSceneView(scene, &generation);
        if (!sv)
            return false;
    }

    // A DirectGPU scene is served by the view the READ already caches -- built by the same
    // enumeration, so the write inherits its records, its ranges and its instancer ordering rather
    // than building a parallel set that could disagree.
    if (gpu)
    {
        InstancerViewCacheEntry& dev = g_instancerViewCache[scene];
        const uint64_t dbEpochNowDev = omni::physx::internal::recordLifetimeEpoch();
        if (dev.dbEpoch != dbEpochNowDev || dev.generation != generation || !dev.view)
            return true; // no device view built yet -- read an instancer column first; not a failure
        std::vector<::physx::PxMat44d> worldInverses;
        worldInverses.reserve(dev.keys.size());
        for (const ObjectKey& key : dev.keys)
            worldInverses.push_back(
                omni::physx::affineInverse(getWorldTransform(
                    *ctx.stage, key, omni::physics::parse::ReadTime::defaultTime())));
        if (!dev.view->setWorldInverses(worldInverses.data(), PxU32(dev.keys.size())))
        {
            CARB_LOG_ERROR("instancerWriteTargets: the device instancer view refused its world "
                           "inverses; no instancer group emitted.");
            return false;
        }
        out.view = dev.view;
        out.keys = dev.keys;
        out.arrayLengths.reserve(dev.maxIndex.size());
        for (uint32_t m : dev.maxIndex)
            out.arrayLengths.push_back(m + 1);
        return true;
    }

    HostInstancerViewCacheEntry& ce = g_hostInstancerViewCache[scene];
    const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();
    if (ce.dbEpoch != dbEpochNow || ce.generation != generation || !ce.view)
    {
        std::vector<InstancerAccum> instancers;
        // eDescriptions, not eValues: the view needs the instance LIST, and the pose readback and
        // reframe eValues pays for is exactly the work the write is about to redo in reverse.
        // kNoActiveActors: this enumerates under the caller's scope for the LIST of instancers, and
        // the active filter is a per-step fact applied at emit, not at enumeration -- the same
        // reason the read's own instancer view is built with kOvxAll.
        enumerateInstancers(*ctx.stage, scope, kNoActiveActors, instancers, nullptr,
                            InstancerCollect::eDescriptions);
        ce.keys.clear();
        ce.arrayLengths.clear();
        ce.view = nullptr;
        if (!instancers.empty())
        {
            std::vector<omni::physx::tensors::PointInstancerEntry> entries;
            entries.reserve(instancers.size());
            for (const InstancerAccum& acc : instancers)
            {
                omni::physx::tensors::PointInstancerEntry e;
                e.worldInverse = acc.worldInverse;
                e.arrayLength = acc.maxIndex + 1;
                e.instances = acc.instances;
                entries.push_back(std::move(e));
                ce.keys.push_back(acc.instancerKey);
                ce.arrayLengths.push_back(acc.maxIndex + 1);
            }
            ce.view = sv->createPointInstancerViewFromEntries(entries);
            if (!ce.view)
            {
                ce.keys.clear();
                ce.arrayLengths.clear();
                return false;
            }
        }
        ce.dbEpoch = dbEpochNow;
        ce.generation = generation;
    }

    // Resolved every call and never cached, for the reason the read's AC-6 gives: the instancer's
    // world transform is a STAGE query, and a moving instancer changes it while creating and
    // retiring nothing, so the epoch guarding everything above cannot see it.
    if (ce.view)
    {
        std::vector<::physx::PxMat44d> worldInverses;
        worldInverses.reserve(ce.keys.size());
        for (const ObjectKey& key : ce.keys)
            worldInverses.push_back(
                omni::physx::affineInverse(getWorldTransform(
                    *ctx.stage, key, omni::physics::parse::ReadTime::defaultTime())));
        if (!ce.view->setWorldInverses(worldInverses.data(), PxU32(ce.keys.size())))
            return false;
    }

    out.view = ce.view;
    out.keys = ce.keys;
    out.arrayLengths = ce.arrayLengths;
    return true;
}
} // namespace omni::physx::ovx

namespace
{

// Which of the view's four columns an attribute name reads, if any.
bool instancerColumnForName(const std::string& name, omni::physx::tensors::BasePointInstancerView::InstancerColumn& out)
{
    using Column = omni::physx::tensors::BasePointInstancerView::InstancerColumn;
    if (name == omni::physx::OvxAttr::kPosition)
        out = Column::eLocalPosition;
    else if (name == omni::physx::OvxAttr::kOrientation)
        out = Column::eLocalOrientation;
    else if (name == omni::physx::OvxAttr::kLinearVelocity)
        out = Column::eLinearVelocity;
    else if (name == omni::physx::OvxAttr::kAngularVelocity)
        out = Column::eAngularVelocity;
    else if (name == omni::physx::OvxAttr::kLinearAcceleration)
        out = Column::eLinearAcceleration;
    else if (name == omni::physx::OvxAttr::kAngularAcceleration)
        out = Column::eAngularAcceleration;
    else
        return false;
    return true;
}

// Which instancers an ACTIVE read may publish, decided without re-enumerating: "at least one of my instances
// moved last step" is a per-STEP fact, so it cannot be cached beside the view, but both ways of learning it
// are reachable from what the cache already holds.
//
// With eENABLE_ACTIVE_ACTORS the cost is O(actors that moved): each resolves to its instancer through the
// same InternalActor::mInstanceKey the enumeration keyed on. Without the flag there is no such list, so every
// instance has to be asked and the early exit per instancer keeps that off a full scan.
//
// Which branch runs is a property of the SCENE and the split is total: PhysXScene clears
// eENABLE_ACTIVE_ACTORS on every scene that suppresses readback, and suppressing readback is what turns
// DirectGPU on. A DirectGPU scene therefore ALWAYS takes the fallback, and the active-actor branch is
// reachable only from a CPU-backend read -- so a test written against a DirectGPU scene covers half of this.
//
// Both branches agree on a kinematic instance: PhysX does not report one that has not been moved, and the
// sleep-state branch tests eKINEMATIC explicitly, which is the rule the enumeration applies too.
//
// Returns false only when it cannot decide, which fails the read rather than publishing a mask that might
// be wrong.
bool computeActiveInstancerMask(PxScene* scene, const InstancerViewCacheEntry& ce, std::vector<bool>& emit)
{
    emit.assign(ce.keys.size(), false);
    if (!scene || !ce.view)
        return false;

    if (scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS)
    {
        const std::vector<InternalDatabase::Record>& records =
            OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords();
        // Through the shared safe walk, not an open-coded loop: this one used to null-check and
        // then read userData, which is the released-actor dereference the walk exists to prevent.
        // The bounds check below cannot stand in for it -- it rejects a garbage INDEX, but the read
        // that produced the index has already happened.
        forEachLiveActiveActor(internalSceneOf(scene),
                               [&records, &ce, &emit](const PxActor* actor)
                               {
                                   // userData is the record index, the same decoding
                                   // collectActiveActorsForScene does.
                                   const size_t recordIndex = reinterpret_cast<size_t>(actor->userData);
                                   if (recordIndex >= records.size())
                                       return;
                                   const InternalDatabase::Record& rec = records[recordIndex];
                                   if (rec.mType != omni::physx::ePTActor || !rec.mInternalPtr)
                                       return;
                                   const InternalActor* ia =
                                       reinterpret_cast<const InternalActor*>(rec.mInternalPtr);
                                   if (ia->mInstanceIndex == kInvalidUint32_t)
                                       return; // a standalone body or an articulation link
                                   std::unordered_map<uint64_t, size_t>::const_iterator it =
                                       ce.slotForInstancer.find(ia->mInstanceKey.handle);
                                   if (it != ce.slotForInstancer.end())
                                       emit[it->second] = true;
                               });
        return true;
    }

    // No active list on this scene, so ask each body its own sleep state -- over the instances the
    // view already holds, which is why this needs no walk either.
    const std::vector<omni::physx::tensors::PointInstancerEntry>& entries = ce.view->getEntries();
    if (entries.size() != ce.keys.size())
        return false; // the two went out of step; declining is the safe answer
    for (size_t i = 0; i < entries.size(); ++i)
    {
        for (const omni::physx::tensors::PointInstance& pi : entries[i].instances)
        {
            PxRigidDynamic* const d = pi.body;
            if (d && !((d->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) || d->isSleeping()))
            {
                emit[i] = true;
                break;
            }
        }
    }
    return true;
}

// Emit every instancer group for one scene.
//
// The shape is the same one buildStandaloneRigid uses: ONE allocation for the whole read, sub-allocated per
// column and then per instancer inside it, with the session owning the single base pointer. What differs is
// that a column here is not contiguous over its prims -- each instancer publishes its FULL instance array
// placed by index, so the column carries holes, and `offsets` tells the kernel where each instancer's
// sub-array starts.
//
// Returns false if the view cannot serve this read; the caller fails the read (there is no host path).
// Returns true when it served the read. A false return has TWO meanings, and the caller must tell
// them apart: `applicable` stays true when the view was selected and could not deliver (a real failure,
// which fails the read), and is set false only when there was nothing here for it to serve -- no requested
// attribute has an instancer array form, which is the same answer for every scene.
bool buildInstancerBackendGroups(ReadSession& s,
                                 IPhysicsSource& source,
                                 AttachedStage& as,
                                 PxScene* const scene,
                                 uint32_t scope,
                                 const std::vector<std::string>& names,
                                 LazyRigidScan& lazy,
                                 uint64_t dbEpochNow,
                                 bool& applicable)
{
    applicable = true;
    if (!scene)
        return false;

    // One builder, both backends. Exactly three things differ: which simulation view the instancer view is
    // created from, where the column is allocated, and whether the group carries a device pointer or host
    // floats. Enumeration, caching, the world-inverse resolve, the column layout and the gather-then-emit
    // ordering are identical.
    const bool gpu = sceneRunsDirectGpu(scene);

    omni::physx::tensors::SimulationBackend* const backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    // Device path only. A CPU scene may have no CUDA context manager at all, so taking the lock
    // there is not merely wasteful -- there is nothing to lock.
    PxCudaContextManager* ctxMgr = nullptr;
    PxCudaContext* cu = nullptr;
    std::unique_ptr<PxScopedCudaLock> cudaLock;
    if (gpu)
    {
        ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
            return false;
        cu = ctxMgr->getCudaContext();
        cudaLock = std::make_unique<PxScopedCudaLock>(*ctxMgr);
    }

    uint64_t generation = 0;
    omni::physx::tensors::GpuSimulationView* gpuSv = nullptr;
    omni::physx::tensors::CpuSimulationView* cpuSv = nullptr;
    int devOrd = -1;
    if (gpu)
    {
        gpuSv = backend->acquireSceneView(scene, &generation);
        if (!gpuSv)
            return false;
        devOrd = gpuSv->getDeviceOrdinal();
        if (devOrd < 0)
            return false;
    }
    else
    {
        cpuSv = backend->acquireCpuSceneView(scene, &generation);
        if (!cpuSv)
            return false;
    }

    // Retiring the cached entry means two different things, and collapsing them leaks.
    //
    // A changed GENERATION says the parent GpuSimulationView was rebuilt, which released its children on the
    // way down. The pointer is already dead, so it is dropped and must not be touched.
    //
    // A changed EPOCH says only that the object set moved. The parent is still alive and still holds this view
    // in mPointInstancerViews, so dropping the pointer abandons it with its device records rather than freeing
    // it. recordLifetimeEpoch is process-global, bumped by any create or retire anywhere on the stage, so a
    // workload that authors materials or joints while reading instancers would strand one per bump.
    //
    // Gated on `built`, not on `view`: an entry that built to no instancers holds a null view and must still
    // retire on the same two triggers, or it would answer empty for a scene that has since gained instancers.
    InstancerViewCacheEntry& ce = g_instancerViewCache[scene];
    if (ce.built && ce.generation != generation)
    {
        ce.view = nullptr;
        ce.built = false;
    }
    else if (ce.built && ce.dbEpoch != dbEpochNow)
    {
        if (ce.view)
            ce.view->release(); // deregisters through ~BasePointInstancerView -> _onChildRelease
        ce.view = nullptr;
        ce.built = false;
    }

    if (!ce.built)
    {
        // Cache miss: enumerate, the expensive part, and keep everything the emit will want again so the
        // next read does not repeat it. This is the only place on this path that materialises the walk.
        //
        // Enumerated at kOvxAll REGARDLESS of the scope being read, which is what makes the entry cacheable
        // across scopes: enumerateInstancers ends with a whole-instancer erase that runs in every collect
        // mode, so an ACTIVE enumeration would build a view holding only the instancers that moved on the
        // step it happened to run. The scope filter is applied at emit instead, per read, because it is a
        // per-step fact.
        std::vector<InstancerAccum> instancers;
        enumerateInstancers(as, kOvxAll, kNoActiveActors, instancers, &lazy.get().instanced,
                            InstancerCollect::eDescriptions);

        std::vector<omni::physx::tensors::PointInstancerEntry> entries;
        entries.reserve(instancers.size());
        ce.keys.clear();
        ce.maxIndex.clear();
        ce.slotForInstancer.clear();
        for (const InstancerAccum& acc : instancers)
        {
            // The enumeration walks the whole object database, so filter to THIS scene: the view is
            // created from this scene's simulation view, and an instancer from another scene has no
            // valid index in it. A stage with instancers in several scenes gets one view per scene,
            // built by one call to this function each.
            if (acc.scene != scene)
                continue;
            if (acc.multiScene)
            {
                // One instancer whose own instances straddle scenes. No single view can gather it,
                // and emitting the reachable subset would report a short instance array as complete.
                CARB_LOG_ERROR("ovxReadAttributes: a point instancer has instances in more than one physics "
                               "scene, which cannot be gathered through a per-scene view; its columns are "
                               "omitted rather than served in part.");
                ce.keys.clear();
                ce.maxIndex.clear();
                ce.slotForInstancer.clear();
                return false;
            }
            ce.slotForInstancer.emplace(acc.instancerKey.handle, ce.keys.size());
            omni::physx::tensors::PointInstancerEntry e;
            e.worldInverse = acc.worldInverse;
            e.arrayLength = acc.maxIndex + 1;
            e.instances = acc.instances;
            entries.push_back(std::move(e));
            ce.keys.push_back(acc.instancerKey);
            ce.maxIndex.push_back(acc.maxIndex);
        }
        // Filtered to nothing: this scene holds no instancers. A complete answer for it, and the
        // caller moves on to the next scene.
        //
        // Marked built, not just stamped: `view` stays null here, so an entry identified by the pointer
        // would read back as a miss and pay the full enumeration again on every read to re-learn that
        // this scene has nothing -- silently, because the answer is correct either way.
        if (entries.empty())
        {
            ce.dbEpoch = dbEpochNow;
            ce.generation = generation;
            ce.built = true;
            return true;
        }

        ce.view = gpu ? static_cast<omni::physx::tensors::BasePointInstancerView*>(
                            gpuSv->createPointInstancerViewFromEntries(entries)) :
                        static_cast<omni::physx::tensors::BasePointInstancerView*>(
                            cpuSv->createPointInstancerViewFromEntries(entries));
        if (!ce.view)
        {
            ce.keys.clear();
            ce.maxIndex.clear();
            ce.slotForInstancer.clear();
            return false;
        }
        ce.dbEpoch = dbEpochNow;
        ce.generation = generation;
        ce.built = true;
    }

    // No instancers in THIS scene is a complete answer, not a decline: with several scenes on the
    // stage the caller asks each one in turn, and most hold none.
    const size_t numInstancers = ce.keys.size();
    if (numInstancers == 0)
        return true;
    if (gpu)
    {
        omni::physx::tensors::GpuPointInstancerView* gpuView =
            static_cast<omni::physx::tensors::GpuPointInstancerView*>(ce.view);
        if (!gpuView->refreshDisabledRowsOvStage())
        {
            // Silence here would drop every instancer group for this scene with no trace of why.
            CARB_LOG_WARN_ONCE("ovxReadAttributes: point-instancer DirectGPU row refresh failed; "
                               "this scene's instancer columns are omitted.");
            return false;
        }
    }

    // The scope filter, applied here rather than in the enumeration that built the view. Computed BEFORE the
    // world-inverse queries and the column allocation, because the common ACTIVE case on a settled scene is
    // that nothing moved -- and then this path is one empty loop rather than a gather whose groups are all
    // discarded.
    std::vector<bool> emit;
    if (scope == kOvxActive)
    {
        if (!computeActiveInstancerMask(scene, ce, emit))
            return false;
        if (std::find(emit.begin(), emit.end(), true) == emit.end())
        {
            // Served, and the answer is no instancer groups. Returning here skips the per-attribute
            // "no point-instancer array form" warning the plan loop below emits, so a read of, say,
            // mass warns at ALL scope and not at ACTIVE on a settled scene. That is a log
            // difference only -- both publish nothing for instanced bodies either way.
            return true;
        }
    }
    else
    {
        emit.assign(numInstancers, true);
    }

    // Resolved every read and never cached (AC-6): this is a stage query, and a moving instancer
    // changes it while creating and retiring nothing, so the epoch guarding everything else above
    // cannot see it. One query per INSTANCER, not per instance -- which is why it is affordable to
    // redo unconditionally.
    std::vector<::physx::PxMat44d> worldInverses;
    worldInverses.reserve(numInstancers);
    for (const ObjectKey& key : ce.keys)
        worldInverses.push_back(
            omni::physx::affineInverse(getWorldTransform(as, key, omni::physics::parse::ReadTime::defaultTime())));
    if (!ce.view->setWorldInverses(worldInverses.data(), PxU32(numInstancers)))
        return false;

    // Size every column first: one allocation for the read, as in buildStandaloneRigid. Each
    // instancer's sub-array is aligned so its group can be handed out as a DLTensor of its own.
    struct ColumnPlan
    {
        std::string arrayName;
        omni::physx::tensors::BasePointInstancerView::InstancerColumn column;
        int comp = 0;
        size_t base = 0; // float offset of this column's region within the allocation
        std::vector<PxU32> offsets; // per instancer, floats from `base`
    };
    std::vector<ColumnPlan> plans;
    size_t totalFloats = 0;
    for (const std::string& name : names)
    {
        int comp = 0;
        std::string arrayName;
        if (!rigidAttrInfo(name, comp, arrayName))
        {
            // Two reasons, told apart because they mean different things to a caller: an unknown name
            // is a caller mistake, a known one with no instancer form is a property of the attribute.
            if (findRigidAttribute(name))
                CARB_LOG_WARN_ONCE("ovxReadAttributes: '%s' has no point-instancer array form; "
                                   "instanced bodies are omitted from it (standalone bodies are not).",
                                   name.c_str());
            else
                CARB_LOG_WARN("ovxReadAttributes: '%s' is not a rigid-body output attribute — skipped.",
                              name.c_str());
            continue;
        }
        // Not conditional any more: rigidAttrInfo above admits exactly the six attributes with an
        // instancer array form -- position, orientation, linear/angular velocity, linear/angular
        // acceleration -- and InstancerColumn covers all six. The branch that used to decline here
        // for "acceleration today" became unreachable when acceleration was added to the view.
        ColumnPlan plan;
        const bool mapped = instancerColumnForName(name, plan.column);
        CARB_ASSERT(mapped);
        CARB_UNUSED(mapped);

        // The kernel scatters at a stride FIXED per column -- 4 for the orientation quaternion, 3 for a vec3
        // -- while `comp` comes off kRigidAttributes, and nothing makes the two agree: widening a row in that
        // table would leave the kernel writing at the old stride into sub-arrays sized for the new one,
        // overwriting neighbouring instances silently. Declining fails the read rather than emitting
        // a column the kernel would scatter at the wrong stride.
        const int kernelStride =
            (plan.column == omni::physx::tensors::BasePointInstancerView::InstancerColumn::eLocalOrientation) ? 4 : 3;
        if (comp != kernelStride)
        {
            CARB_LOG_ERROR("ovxReadAttributes: '%s' is %d wide but the instancer kernel scatters at %d; "
                           "the instancer columns are omitted rather than scattered at the wrong stride.",
                           name.c_str(), comp, kernelStride);
            return false;
        }

        plan.arrayName = arrayName;
        plan.comp = comp;
        plan.base = totalFloats;
        // Padded on the DEVICE only. alignColumnFloats exists because a sub-array start is handed out as a
        // DLTensor data pointer and dlpack wants those 256-byte aligned, which is true of the device branch,
        // where the group aliases into `columnBase`. The host branch copies its slice into the group's own
        // vector, so nothing points into `hostColumn` and its layout carries no alignment requirement --
        // worth the branch, since here the padding is per instancer per column rather than once per column.
        plan.offsets.reserve(numInstancers);
        size_t cursor = 0;
        for (size_t i = 0; i < numInstancers; ++i)
        {
            // The +1 in size_t, NOT in PxU32. `size_t(ce.maxIndex[i] + 1)` incremented before the
            // widening, so a maxIndex of UINT32_MAX wrapped to a zero-size slot and every later
            // instancer was handed this one's region to scatter into.
            const size_t rows = size_t(ce.maxIndex[i]) + 1;
            if (rows > SIZE_MAX / size_t(comp))
            {
                CARB_LOG_ERROR("ovxReadAttributes: instancer '%s' needs %zu rows x %d components, which "
                               "overflows the column size; the instancer columns are omitted rather than "
                               "sized from a wrapped product.",
                               arrayName.c_str(), rows, comp);
                return false;
            }
            const size_t slot = rows * size_t(comp);
            const size_t step = gpu ? alignColumnFloats(slot) : slot;
            if (step < slot || step > SIZE_MAX - cursor)
            {
                CARB_LOG_ERROR("ovxReadAttributes: the instancer column cursor overflows while placing '%s'.",
                               arrayName.c_str());
                return false;
            }
            // The offset is handed to the kernel as a PxU32 while the ALLOCATION uses the full size_t
            // total, so a cursor past UINT32_MAX would wrap and scatter this instancer into an earlier
            // instancer's region -- silently, since the allocation is big enough for both.
            if (cursor > UINT32_MAX)
            {
                CARB_LOG_ERROR("ovxReadAttributes: instancer column offset %zu exceeds the 32-bit offset the "
                               "scatter kernel takes; the instancer columns are omitted rather than wrapped.",
                               cursor);
                return false;
            }
            plan.offsets.push_back(PxU32(cursor));
            cursor += step;
        }
        if (cursor > SIZE_MAX - totalFloats)
        {
            CARB_LOG_ERROR("ovxReadAttributes: the total instancer column size overflows; the instancer "
                           "columns are omitted.");
            return false;
        }
        totalFloats += cursor;
        plans.push_back(std::move(plan));
    }
    if (plans.empty())
    {
        // The ONE not-applicable exit: no requested attribute has an instancer array form. The host
        // path takes over and warns per attribute, which is correct on either device.
        applicable = false;
        return false;
    }

    // Zeroed on both backends because the columns have HOLES: an instance-array slot with no live body is
    // never written. The view contract states it -- "slots with no instance are left as the caller zero-filled
    // them" -- so the zero is the caller's job on either path.
    CUdeviceptr columnBase = 0;
    std::vector<float> hostColumn;
    if (gpu)
    {
        // Checked before the product is formed: totalFloats is a float COUNT and the allocation takes
        // bytes, so a count that fits can still overflow the byte product and request a buffer far
        // smaller than the offsets computed above address into.
        if (totalFloats > SIZE_MAX / sizeof(float))
        {
            CARB_LOG_ERROR("ovxReadAttributes: the instancer column byte size overflows; the instancer "
                           "columns are omitted rather than allocated from a wrapped product.");
            return false;
        }
        // Pool-or-allocate; acquireColumn also registers it for release.
        columnBase = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, totalFloats * sizeof(float)));
        if (!columnBase)
            return false;
        // Stream-ordered on the null stream, so it precedes the gathers without a host sync.
        //
        // The RESULT matters and used to be discarded. This zero is not cosmetic: the kernel scatters
        // per instance, so the padding between one instancer's sub-array and the next is never written
        // by any gather. Those holes ARE the zero. A failed fill followed by a successful gather
        // publishes whatever the allocator last held in them, indistinguishable from real output.
        if (cu->memsetD32Async(columnBase, 0u, totalFloats, 0) != 0)
        {
            CARB_LOG_ERROR("ovxReadAttributes: could not zero the instancer column; the read is failed "
                           "rather than published with holes holding uninitialized device memory.");
            return false;
        }
    }
    else
    {
        // One buffer for every column and instancer, because the view fills the whole thing in one
        // call keyed by `offsets`. Packed, not padded like the device layout -- see the offsets
        // loop. The per-instancer groups copy their slice out of it below: a host GroupStore owns
        // its floats and cannot alias a shared buffer the way a device group aliases `columnBase`.
        hostColumn.assign(totalFloats, 0.0f);
    }

    // Every gather first, THEN every group. Not a style choice: emitting as we go would leave the
    // groups from earlier columns published when a later one fails, and the caller's fallback would
    // then emit the host path's groups for the SAME prims and attributes on top of them. Failing
    // before anything is published is what makes that fallback safe.
    for (const ColumnPlan& plan : plans)
    {
        void* const dst = gpu ? reinterpret_cast<void*>(columnBase + plan.base * sizeof(float)) :
                                static_cast<void*>(hostColumn.data() + plan.base);
        if (!ce.view->getInstancerColumnsOvStage(plan.column, dst, plan.offsets.data(), PxU32(numInstancers)))
            return false;
    }

    // One interned prim list per INSTANCER, reused across every column of it. Each instancer is its own
    // single-prim set, so the sharing runs down the plan loop rather than across it -- holding the lists in a
    // vector keyed by instancer gets that without reordering the groups a caller sees.
    std::vector<ovx_primpath_list_t> instancerLists(numInstancers, OVX_INVALID_PRIMPATH_LIST);

    for (const ColumnPlan& plan : plans)
    {
        for (size_t i = 0; i < numInstancers; ++i)
        {
            if (!emit[i])
                continue; // ACTIVE scope: no instance of this instancer moved last step
            const size_t rows = size_t(ce.maxIndex[i]) + 1;
            GroupStore g;
            g.isArray = true;
            if (gpu)
            {
                g.deviceData = reinterpret_cast<void*>(columnBase +
                                                       (plan.base + plan.offsets[i]) * sizeof(float));
                g.deviceOrdinal = devOrd;
                g.ctxMgr = ctxMgr;
            }
            else
            {
                // Copy rather than alias: a host group OWNS its floats. The slice is exactly this
                // instancer's run -- the host layout is packed, so consecutive offsets abut and
                // `rows * comp` is the whole of it.
                const float* const src = hostColumn.data() + plan.base + plan.offsets[i];
                g.floats.assign(src, src + rows * size_t(plan.comp));
            }
            std::vector<ObjectKey> keys{ ce.keys[i] };
            // A false return appended no group; reporting success here would end the read with this
            // instancer's column simply absent.
            if (!finalizeGroup(s, source, std::move(g), plan.arrayName, keys, static_cast<int64_t>(rows),
                               plan.comp, /*canonicalHandles*/ nullptr, &instancerLists[i]))
                return false;
        }
    }
    return true;
}

void buildRigidBodyGroups(ReadSession& s,
                          IPhysicsSource& source,
                          AttachedStage& as,
                          uint32_t scope,
                          const std::vector<std::string>& names)
{
    std::vector<PxScene*> scenes;
    scenes = allPhysicsScenes();

    // Once per read, with the live set in hand: entries for scenes that have gone away are never
    // looked up again, so this is what frees them.
    purgeDeadRigidReadCacheEntries(scenes);

    // Keyed by PxScene*, like the cache purged above, and for the same reason: a scene that has gone away is
    // never looked up again, so nothing else would drop its entry -- and a reused address would otherwise find
    // a view built for a different scene.
    purgeDeadSceneEntries(g_instancerViewCache, scenes);

    // Standalone bodies: sourced from the tensor backend view (device-resident on GPU, host on CPU), once per
    // scene so each body is read through its owning scene's view. These calls are what drive the database
    // walk, so a read whose scenes all still report the generation they were resolved under never touches the
    // database at all.
    LazyRigidScan lazy{ scope };
    for (PxScene* scene : scenes)
        buildStandaloneRigid(s, source, scene, scope, names, lazy);

    // What the object database holds by way of instanced dynamics, as of an epoch. Never pointers to
    // the bodies themselves -- enumerateInstancers dereferences whatever list it is handed and a
    // stale one holds retired actors -- only the two facts a read needs before it can decide
    // anything: whether there are any, and which scene owns them.
    //
    // The epoch is what makes both reusable: no object has been created or retired, so an instancer cannot
    // have appeared, disappeared or moved between scenes. Caching the SCENE, not just the emptiness, is what
    // lets a warm read skip the O(records) walk entirely, as buildStandaloneRigid does above.
    //
    // Process-global, and correct only because scanRigidRecords walks the WHOLE object database
    // rather than one attach or one scene: what is cached is a statement about all of it. If that
    // scan ever becomes per-scene or per-attach, this silently stops enumerating instancers for
    // whatever it did not cover, so it would have to become per-scene state at the same time.
    struct InstancedSetAtEpoch
    {
        uint64_t dbEpoch = 0; // 0 = never observed
        bool any = false; // the database holds at least one instanced dynamic
        // Every DISTINCT scene holding instanced dynamics, not one-scene-or-give-up. A view belongs
        // to a scene, so a stage with instancers in several gets one view each and this read asks
        // them in turn -- which is what retired the host fallback that used to cover the case.
        std::vector<PxScene*> scenes;
    };
    // Holding a PxScene* under the lifetime epoch alone is safe because a scene is itself an ePTScene record,
    // so creating or destroying one moves that epoch and address reuse cannot slip past it. The liveness probe
    // below covers a scene that somehow left `scenes` without a record changing.
    static InstancedSetAtEpoch sInstancedSet;

    const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();

    if (sInstancedSet.dbEpoch != dbEpochNow)
    {
        const RigidRecordScan& scan = lazy.get();
        sInstancedSet.dbEpoch = dbEpochNow;
        sInstancedSet.any = !scan.instanced.empty();
        sInstancedSet.scenes.clear();
        for (PxRigidDynamic* d : scan.instanced)
        {
            PxScene* const sc = d ? d->getScene() : nullptr;
            if (sc && std::find(sInstancedSet.scenes.begin(), sInstancedSet.scenes.end(), sc) ==
                          sInstancedSet.scenes.end())
                sInstancedSet.scenes.push_back(sc);
        }
    }

    // Nothing instanced: the loop below would do nothing anyway, and the "no array form" warning
    // would fire about instanced bodies that do not exist. A property read on a scene of standalone
    // bodies must say nothing at all.
    if (!sInstancedSet.any)
        return;

    // One backend call per scene holding instancers, each serving that scene's own view. There is no
    // host fallback behind this any more: BasePointInstancerView covers all six attributes with an
    // instancer array form on either device, so a failure here is a failure, not a reason to reframe
    // from CPU accessors a DirectGPU scene does not keep current.
    //
    // Scene order is the order instanced bodies were first seen in the record walk, so a single-scene
    // stage -- every stage that reached the backend before -- emits exactly the group order it did.
    for (PxScene* const instancedScene : sInstancedSet.scenes)
    {
        // A scene that went away without any record changing would leave a live epoch naming a dead
        // pointer. Cheap to rule out -- one linear probe over a list with one entry per PhysicsScene.
        if (std::find(scenes.begin(), scenes.end(), instancedScene) == scenes.end())
            continue;

        bool applicable = true;
        if (buildInstancerBackendGroups(s, source, as, instancedScene, scope, names, lazy, dbEpochNow,
                                        applicable))
            continue;

        if (!applicable)
        {
            // Nothing here for it to serve: no requested attribute has an instancer array form. The
            // per-attribute warning was already emitted by the builder, and no other scene can have a
            // different answer, since the question is about the attribute names alone.
            return;
        }

        // Pre-step is NOT a failure, it is the documented step-first precondition (ADR-0008
        // Decision 10) -- PxDirectGPUAPI has no readable state until the first step completes, and
        // the articulation path already treats exactly this condition as unavailability rather than
        // as a backend failure. Reporting it as an error here made the same scene answer two
        // different ways depending on which type was asked for. Emit nothing and say nothing; the
        // caller steps once and reads again.
        //
        // `continue`, not `return`: readiness is a property of THIS scene. Scene order follows record
        // order, so returning let one unstepped partition silently drop every ready partition behind
        // it -- and the caller cannot tell that from a stage that genuinely has no instancers there.
        // The two returns around this one stay returns because their conclusions really are global:
        // `applicable` is decided by the attribute names alone, and a backend failure means the read
        // is incomplete however the other scenes fare.
        if (!directGpuSceneStepped(instancedScene))
            continue;

        // Anything else: selected and could not deliver. Reporting beats falling back -- the read is
        // one answer, and a partial one is indistinguishable at the API from a complete one.
        CARB_LOG_ERROR_ONCE("ovxReadAttributes: the point-instancer read could not be served from the "
                            "backend view for a scene; the instancer columns are omitted rather than "
                            "published in part.");
        s.backendFailed = true;
        return;
    }
}

// ----------------------------------------------------------------------------
// Articulation links (link body transforms — a fixed group, never instanced)
// ----------------------------------------------------------------------------

void buildArticulationLinkGroups(ReadSession& s,
                                 IPhysicsSource& source,
                                 uint32_t scope,
                                 const std::vector<std::string>& names)
{
    // Link pose/velocity is sourced from the tensor backend view (device-resident on GPU, host on
    // CPU) via buildArticulationLinks, once per scene; there is no separate host reader.
    const std::vector<PxScene*> scenes = allPhysicsScenes();

    // A link read caches rows and handles per scene exactly as a rigid-body read does -- the emit is the same
    // function, keyed only by type -- but a workload that never asks for rigid bodies never reaches the rigid
    // path that frees them.
    purgeDeadRigidReadCacheEntries(scenes);

    // Same reason, for the scan this path keeps across reads: keyed by PxScene*, and a scene that has gone
    // away is never looked up again, so nothing else would free it.
    purgeDeadSceneEntries(g_linkScanCache, scenes);

    // The database walk is driven by these calls, so it runs once for the whole read rather than
    // once per scene.
    LazyLinkScan lazy{ scope };
    for (PxScene* scene : scenes)
        buildArticulationLinks(s, source, scene, scope, names, lazy);
}

// ----------------------------------------------------------------------------
// Articulation joint state (per-axis scalar — an array group per joint prim)
// ----------------------------------------------------------------------------

// The backend's cached superset articulation view for one scene, plus this read's articulations mapped onto
// its superset rows.
//
// Root, joint and tendon reads need exactly this, and the device and host sides differ only in which acquire
// call and which view type. Everything else -- the two-attempt retry, what counts as unusable, and why an
// articulation the view does not describe invalidates rather than fails -- is identical, so it lives here
// once rather than in a branch per device per caller.
struct ArticulationViewBinding
{
    omni::physx::tensors::GpuArticulationView* gpuAv = nullptr; // set iff gpu
    omni::physx::tensors::CpuArticulationView* cpuAv = nullptr; // set iff !gpu
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    int devOrd = -1;         // host columns keep -1; only read under `if (gpu)`
    uint64_t generation = 0; // the view build whose rows localToRow names
    // Translates this read's articulation index into the superset row the cached view and the
    // device scratch are indexed by.
    std::vector<uint32_t> localToRow;
    // Usually points to localToRow. A caller with a generation-validated cached mapping can lend it
    // here instead, avoiding both the per-articulation hash lookups and a copy on the hot path.
    const std::vector<uint32_t>* resolvedLocalToRow = nullptr;
    bool usedCachedRows = false;
};

bool articulationShapeTopologyMatches(const omni::physx::tensors::ArticulationEntry& entry,
                                      PxArticulationReducedCoordinate* articulation,
                                      std::vector<PxArticulationLink*>& createdLinks,
                                      std::vector<PxArticulationLink*>& orderedLinks, std::vector<PxShape*>& shapes)
{
    if (!articulation || entry.arti != articulation)
        return false;

    const PxU32 linkCount = articulation->getNbLinks();
    if (entry.numLinks != linkCount || entry.links.size() != linkCount)
        return false;

    createdLinks.resize(linkCount);
    if (linkCount > 0 && articulation->getLinks(createdLinks.data(), linkCount) != linkCount)
        return false;

    // ArticulationEntry stores links in simulation-cache order, while getLinks() returns creation
    // order. Reproduce buildArticulationEntry's ordering before comparing the snapshots so a
    // branched articulation is not falsely treated as stale.
    orderedLinks.assign(linkCount, nullptr);
    for (PxArticulationLink* link : createdLinks)
    {
        if (!link)
            return false;
        const PxU32 linkIndex = link->getLinkIndex();
        if (linkIndex >= linkCount || orderedLinks[linkIndex])
            return false;
        orderedLinks[linkIndex] = link;
    }

    size_t shapeOffset = 0;
    for (PxU32 linkIndex = 0; linkIndex < linkCount; ++linkIndex)
    {
        PxArticulationLink* const link = orderedLinks[linkIndex];
        if (!link || entry.links[linkIndex] != link)
            return false;

        const PxU32 shapeCount = link->getNbShapes();
        if (shapeOffset + shapeCount > entry.shapes.size())
            return false;
        shapes.resize(shapeCount);
        if (shapeCount > 0 && link->getShapes(shapes.data(), shapeCount) != shapeCount)
            return false;
        for (PxU32 shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex)
            if (entry.shapes[shapeOffset + shapeIndex] != shapes[shapeIndex])
                return false;
        shapeOffset += shapeCount;
    }

    return entry.numShapes == shapeOffset && entry.shapes.size() == shapeOffset;
}

bool articulationShapeTopologiesMatch(const std::vector<omni::physx::tensors::ArticulationEntry>& entries,
                                      const std::vector<PxArticulationReducedCoordinate*>& articulations,
                                      const std::vector<uint32_t>& rows)
{
    if (articulations.size() != rows.size())
        return false;
    // Reuse three scratch vectors across the scene-wide validation. A structural epoch is rare,
    // but an 8k-articulation scene must still not perform several heap allocations per root.
    std::vector<PxArticulationLink*> createdLinks;
    std::vector<PxArticulationLink*> orderedLinks;
    std::vector<PxShape*> shapes;
    for (size_t index = 0; index < articulations.size(); ++index)
    {
        const uint32_t row = rows[index];
        if (row >= entries.size() ||
            !articulationShapeTopologyMatches(entries[row], articulations[index], createdLinks, orderedLinks, shapes))
            return false;
    }
    return true;
}

// `skipRowsForGeneration` is the caller's own cached view generation, or 0 if it has none. When it
// matches the view this resolves to, the caller already holds everything derived from the mapping and
// will not read it, so building it is pure waste -- one hash lookup per articulation, per read. The
// caller states the condition rather than this function guessing it, because the three callers guard
// on different things (the tendon path also requires its enumeration to be current). Ignored when
// `requireShapeTopology` is set, since that check reads the mapping here.
bool acquireArticulationBinding(omni::physx::tensors::SimulationBackend* backend, PxScene* scene, bool gpu,
                                const std::vector<PxArticulationReducedCoordinate*>& artis,
                                ArticulationViewBinding& out, const std::vector<uint32_t>* cachedRows = nullptr,
                                uint64_t cachedGeneration = 0, bool requireShapeTopology = false,
                                uint64_t skipRowsForGeneration = 0)
{
    // Every failure below unwinds identically: drop what this attempt resolved, invalidate the
    // scene entry so the next one rebuilds, and retry.
    const auto dropAndInvalidate = [&]()
    {
        out.gpuAv = nullptr;
        out.cpuAv = nullptr;
        out.entries = nullptr;
        backend->invalidateSceneCache(scene);
    };

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
        out.gpuAv = nullptr;
        out.cpuAv = nullptr;
        out.entries = nullptr;
        out.resolvedLocalToRow = nullptr;
        out.usedCachedRows = false;
        if (gpu)
        {
            omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(scene, &out.generation);
            out.gpuAv = sv ? sv->supersetArticulationView(&artiRowMap, &out.entries) : nullptr;
            if (out.gpuAv)
                out.devOrd = sv->getDeviceOrdinal();
        }
        else
        {
            omni::physx::tensors::CpuSimulationView* csv = backend->acquireCpuSceneView(scene, &out.generation);
            out.cpuAv = csv ? csv->supersetArticulationView(&artiRowMap, &out.entries) : nullptr;
            out.devOrd = -1;
        }

        const bool haveView = gpu ? (out.gpuAv != nullptr) : (out.cpuAv != nullptr);
        if (!haveView || !artiRowMap || !out.entries)
        {
            // The superset could not be built over this scene's articulations, which the view
            // cannot tell apart from a scene holding none. Drop the entry so the next attempt
            // rebuilds; a genuinely empty scene simply fails again.
            dropAndInvalidate();
            continue;
        }

        if (cachedRows && cachedGeneration != 0 && cachedGeneration == out.generation &&
            cachedRows->size() == artis.size())
        {
            out.resolvedLocalToRow = cachedRows;
            out.usedCachedRows = true;
        }
        else if (!requireShapeTopology && skipRowsForGeneration != 0 && skipRowsForGeneration == out.generation)
        {
            // The caller is warm for this view and will not read the mapping, so it is not resolved.
            //
            // `resolvedLocalToRow` is left null and that is the whole protection: every caller that
            // opts in reads the mapping THROUGH it, so asking to skip and then reading anyway
            // dereferences null at the point of misuse. Clearing `localToRow` is not enough on its
            // own -- an empty vector indexes out of range silently, which is the failure this is
            // meant to make loud.
            out.localToRow.clear();
            out.resolvedLocalToRow = nullptr;
            return true;
        }
        else
        {
            out.localToRow.clear();
            out.localToRow.reserve(artis.size());
            bool allResolved = true;
            for (PxArticulationReducedCoordinate* arti : artis)
            {
                const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
                    artiRowMap->find(arti);
                if (it == artiRowMap->end())
                {
                    allResolved = false;
                    break;
                }
                out.localToRow.push_back(it->second);
            }
            if (!allResolved)
            {
                // An articulation the cached view does not describe: stale for this read, rebuild.
                dropAndInvalidate();
                continue;
            }
            out.resolvedLocalToRow = &out.localToRow;
        }

        // The two branches that arrive here -- reused cache and fresh resolve -- both carry a
        // resolved mapping, so the shape-topology check is written once rather than per branch: it
        // depends only on the mapping, not on where it came from. The skip branch above returns
        // early instead, because it has no mapping to check and cannot be reached with
        // `requireShapeTopology` set.
        if (!requireShapeTopology || articulationShapeTopologiesMatch(*out.entries, artis, *out.resolvedLocalToRow))
            return true;

        dropAndInvalidate();
    }
    return false;
}

// One queried joint and the articulation it belongs to, in this read's articulation ordering.
// JointRec and JointSlice live in OvxPhysicsShared.h: the write indexes joint state through the
// same records, so one definition is what keeps the two directions naming the same joints.
// ArticulationRootRec and ArticulationReadCacheEntry live in OvxPhysicsShared.h: the write indexes
// the same articulations, joints and DOF records through them, so one definition is what keeps the
// two directions enumerating the same objects in the same order. The STORAGE and the structural walk
// stay here, because the walk is this file's machinery -- only the type and the two accessors below
// are shared.

bool directGpuSceneReady(PxScene* scene, ArticulationReadCacheEntry& cache)
{
    if (!scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return true;
    if (!cache.directGpuReady && scene->getTimestamp() > 1)
        cache.directGpuReady = true;
    return cache.directGpuReady;
}

bool articulationSceneInsertionPending(
    const std::vector<PxArticulationReducedCoordinate*>& articulations)
{
    // Ovstage publishes authoritative database records before buffered attach work inserts the
    // corresponding SDK articulations into their PxScene. A scene-wide tensor view cannot contain
    // those rows yet. This is a normal lifecycle state: discovery remains valid, while the read
    // emits no groups until flushChanges() or the first simulation update completes the insertion.
    return std::any_of(articulations.begin(), articulations.end(),
                       [](PxArticulationReducedCoordinate* articulation)
                       { return articulation && articulation->getScene() == nullptr; });
}

struct ArticulationStructuralRef
{
    PxScene* scene = nullptr;
    uint32_t index = 0;
};

// One cache, guarded by the public API's g_mutex, owns every product of the articulation structural
// walk. Keeping the DB-order references beside the per-scene buckets lets discovery preserve the
// authoritative order without walking the database a second time.
struct ArticulationReadCache
{
    uint64_t dbEpoch = 0;
    std::unordered_map<const PxScene*, ArticulationReadCacheEntry> byScene;
    std::vector<ArticulationStructuralRef> rootsInDatabaseOrder;
    std::vector<ArticulationStructuralRef> jointsInDatabaseOrder;
};
ArticulationReadCache g_articulationReadCache;

// Forward-declared so the shared accessors below can sit beside the cache they serve rather than
// after the ~190-line walk that fills it.
bool refreshArticulationStructuralCache(const std::vector<PxScene*>& scenes);

} // namespace (reopened below)

namespace omni::physx::ovx
{
// Declared in OvxPhysicsShared.h so the WRITE can fill and read this cache without a prior read
// having run. The walk itself is unchanged and still lives in this file.
//
// NOT independently locked: both entry points are called under the public API mutex of whichever
// direction is running, and ovphysx.h already requires callers to serialize simulation, stage
// mutation and binding creation across instances. This adds no concurrency the API does not offer.
bool refreshArticulationCache(const std::vector<::physx::PxScene*>& scenes)
{
    return refreshArticulationStructuralCache(scenes);
}

ArticulationReadCacheEntry* articulationCacheEntry(const ::physx::PxScene* scene)
{
    const std::unordered_map<const PxScene*, ArticulationReadCacheEntry>::iterator it =
        g_articulationReadCache.byScene.find(scene);
    return it == g_articulationReadCache.byScene.end() ? nullptr : &it->second;
}
} // namespace omni::physx::ovx

namespace
{

// Refresh roots, link membership and joints together under one lifetime epoch. The scan builds into
// a temporary cache and publishes it only after the epoch is sampled again, so no consumer can see
// a half-filled cache stamped as current.
bool refreshArticulationStructuralCache(const std::vector<PxScene*>& scenes)
{
    const uint64_t currentEpoch = omni::physx::internal::recordLifetimeEpoch();
    bool allScenesPresent = g_articulationReadCache.byScene.size() == scenes.size();
    for (PxScene* scene : scenes)
        allScenesPresent = allScenesPresent && g_articulationReadCache.byScene.count(scene) != 0;
    if (g_articulationReadCache.dbEpoch == currentEpoch && allScenesPresent)
        return true;

    const std::unordered_set<const PxScene*> liveScenes(scenes.begin(), scenes.end());

    struct RootLocation
    {
        PxScene* scene;
        uint32_t rootIndex;
    };
    struct ViewLocation
    {
        PxScene* scene;
        uint32_t artiIndex;
    };
    struct PendingLink
    {
        PxScene* scene;
        PxArticulationReducedCoordinate* arti;
        size_t recordIndex;
    };
    struct PendingJoint
    {
        PxScene* scene;
        PxArticulationReducedCoordinate* arti;
        PxArticulationJointReducedCoordinate* joint;
        InternalJoint* internalJoint;
        ObjectKey key;
    };

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint64_t epochBefore = omni::physx::internal::recordLifetimeEpoch();
        ArticulationReadCache next;
        next.byScene.reserve(scenes.size());
        for (PxScene* scene : scenes)
            next.byScene.emplace(scene, ArticulationReadCacheEntry{});

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        const size_t sceneDivisor = std::max<size_t>(size_t(1), scenes.size());
        // Roots and joints are only a fraction of the database (links, shapes and materials make up
        // most records). Reserve a bounded fraction per scene: enough to avoid geometric growth in
        // articulation-heavy workloads without pinning four full-database-sized vectors.
        const size_t perSceneReserve = records.size() / (sceneDivisor * 4) + 1;
        for (std::pair<const PxScene* const, ArticulationReadCacheEntry>& item : next.byScene)
        {
            item.second.artis.reserve(perSceneReserve);
            item.second.rootArtis.reserve(perSceneReserve);
            item.second.rootKeys.reserve(perSceneReserve);
            item.second.roots.reserve(perSceneReserve);
            item.second.joints.reserve(perSceneReserve);
        }

        std::unordered_map<PxArticulationReducedCoordinate*, RootLocation> rootLocations;
        std::unordered_map<PxArticulationReducedCoordinate*, ViewLocation> viewLocations;
        rootLocations.reserve(records.size() / 8 + 1);
        viewLocations.reserve(records.size() / 8 + 1);
        std::vector<PendingLink> pendingLinks;
        std::vector<PendingJoint> pendingJoints;
        pendingLinks.reserve(records.size() / 4 + 1);
        pendingJoints.reserve(records.size() / 8 + 1);
        next.rootsInDatabaseOrder.reserve(records.size() / 8 + 1);
        next.jointsInDatabaseOrder.reserve(records.size() / 8 + 1);

        for (size_t recordIndex = 0; recordIndex < records.size(); ++recordIndex)
        {
            const InternalDatabase::Record& rec = records[recordIndex];
            if (rec.mType == omni::physx::ePTArticulation && rec.mPtr)
            {
                PxArticulationReducedCoordinate* arti = reinterpret_cast<PxArticulationReducedCoordinate*>(rec.mPtr);
                // The SDK articulation is not inserted into its PxScene until the first update,
                // but its database record already has the authoritative owner. Discovery must see
                // that pre-step object even though a DirectGPU read deliberately emits no groups.
                InternalArticulation* internalArticulation = reinterpret_cast<InternalArticulation*>(rec.mInternalPtr);
                PxScene* scene = internalArticulation && internalArticulation->mPhysxScene ?
                                     internalArticulation->mPhysxScene->getScene() :
                                     arti->getScene();
                if (!scene || liveScenes.count(scene) == 0)
                    continue;
                ArticulationReadCacheEntry& entry = next.byScene[scene];
                const uint32_t rootIndex = static_cast<uint32_t>(entry.roots.size());
                const std::pair<std::unordered_map<PxArticulationReducedCoordinate*, RootLocation>::iterator, bool> inserted =
                    rootLocations.emplace(arti, RootLocation{ scene, rootIndex });
                if (!inserted.second)
                {
                    entry.duplicateRootPointers = true;
                    continue;
                }
                entry.rootArtis.push_back(arti);
                entry.rootKeys.push_back(rec.mKey);
                entry.roots.emplace_back();
                next.rootsInDatabaseOrder.push_back({ scene, rootIndex });
            }
            else if (rec.mType == omni::physx::ePTLink && rec.mPtr)
            {
                PxArticulationLink* link = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>();
                if (!link)
                    continue;
                PxArticulationReducedCoordinate* arti =
                    &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
                pendingLinks.push_back({ link->getScene(), arti, recordIndex });
            }
            else if (rec.mType == omni::physx::ePTLinkJoint && rec.mPtr && rec.mInternalPtr)
            {
                PxArticulationJointReducedCoordinate* joint =
                    reinterpret_cast<PxArticulationJointReducedCoordinate*>(rec.mPtr);
                PxArticulationLink& child = joint->getChildArticulationLink();
                PxScene* scene = child.getScene();
                PxArticulationReducedCoordinate* arti =
                    &static_cast<PxArticulationReducedCoordinate&>(child.getArticulation());
                pendingJoints.push_back(
                    { scene, arti, joint, reinterpret_cast<InternalJoint*>(rec.mInternalPtr), rec.mKey });
            }
        }

        for (const PendingLink& link : pendingLinks)
        {
            const std::unordered_map<PxArticulationReducedCoordinate*, RootLocation>::const_iterator found =
                rootLocations.find(link.arti);
            if (found == rootLocations.end() || (link.scene && found->second.scene != link.scene))
                continue;
            ArticulationReadCacheEntry& entry = next.byScene[found->second.scene];
            entry.roots[found->second.rootIndex].linkRecordIndices.push_back(link.recordIndex);
        }

        for (const PendingJoint& joint : pendingJoints)
        {
            const std::unordered_map<PxArticulationReducedCoordinate*, RootLocation>::const_iterator rootFound =
                rootLocations.find(joint.arti);
            PxScene* scene = joint.scene;
            if (rootFound != rootLocations.end())
            {
                if (scene && scene != rootFound->second.scene)
                    continue;
                scene = rootFound->second.scene;
            }
            if (!scene || liveScenes.count(scene) == 0)
                continue;

            std::unordered_map<PxArticulationReducedCoordinate*, ViewLocation>::iterator found =
                viewLocations.find(joint.arti);
            if (found == viewLocations.end())
            {
                ArticulationReadCacheEntry& entry = next.byScene[scene];
                const uint32_t artiIndex = static_cast<uint32_t>(entry.artis.size());
                entry.artis.push_back(joint.arti);
                found = viewLocations.emplace(joint.arti, ViewLocation{ scene, artiIndex }).first;
                if (rootFound == rootLocations.end())
                {
                    // A legacy/inconsistent database: the joint is served off its own articulation, but
                    // there is no root record to publish a whole-articulation row from.
                    CARB_LOG_WARN_ONCE(
                        "ovphysx read: an articulation joint has no attached "
                        "ePTArticulation root record; joint output is preserved, but no "
                        "whole-articulation row is fabricated.");
                }
            }
            if (found->second.scene != scene)
                continue;
            ArticulationReadCacheEntry& entry = next.byScene[scene];
            const uint32_t jointIndex = static_cast<uint32_t>(entry.joints.size());
            entry.joints.push_back({ joint.joint, joint.internalJoint, joint.key, found->second.artiIndex });
            next.jointsInDatabaseOrder.push_back({ scene, jointIndex });
        }

        if (omni::physx::internal::recordLifetimeEpoch() != epochBefore)
            continue;

        for (std::pair<const PxScene* const, ArticulationReadCacheEntry>& item : next.byScene)
            item.second.dbEpoch = epochBefore;
        next.dbEpoch = epochBefore;
        g_articulationReadCache = std::move(next);
        return true;
    }

    g_articulationReadCache = ArticulationReadCache{};
    CARB_LOG_WARN_ONCE("ovphysx read: articulation structure changed during both cache-build attempts.");
    return false;
}

bool articulationRootInScope(const ArticulationReadCacheEntry& entry, size_t rootIndex, uint32_t scope,
                             const ActiveActorSet& activeActors, const PxScene* owningScene)
{
    if (scope != omni::physx::kOvxActive)
        return true;
    if (rootIndex >= entry.roots.size() || rootIndex >= entry.rootArtis.size())
        return false;
    if (activeActors.sceneReports(owningScene))
    {
        for (size_t linkRecordIndex : entry.roots[rootIndex].linkRecordIndices)
            if (activeActors.isActive(linkRecordIndex))
                return true;
        return false;
    }
    PxArticulationReducedCoordinate* const articulation = entry.rootArtis[rootIndex];
    if (!articulation)
        return false;
    if (articulation->getScene())
        return !articulation->isSleeping();

    // A freshly attached DirectGPU scene exposes its database records before PhysX inserts the
    // articulations on the first step. Calling isSleeping() in that state is invalid. DirectGPU
    // disables sleeping, so its documented ACTIVE == ALL rule is already unambiguous; an
    // uninserted sleep-enabled articulation has no solver-active state yet and is omitted.
    return owningScene && owningScene->getFlags().isSet(PxSceneFlag::eDISABLE_SLEEPING);
}

// One cohort of structurally identical articulations: the rows an inverse dynamics group covers, and the
// identity it publishes. Ordered by the database order of the cohort's first row, which is the order
// REQ-READ-ARTICULATION-001 AC-2 already gives the rows themselves (REQ-READ-INVDYN-001 AC-3).
struct ArticulationCohort
{
    const omni::physx::tensors::ArticulationMetatype* metatype = nullptr;
    std::vector<PxU32> rows;
    std::vector<ObjectKey> keys;
    std::vector<ovx_primpath_t> handles;
    // Cohorts cover different prim sets, so each interns its own list rather than sharing the
    // whole-scene one the non-cohort columns share.
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
};

struct PendingArticulationRootGroup
{
    GroupStore group;
    const char* token = nullptr;
    int width = 0;
    // Negative for a column covering the whole scene partition; otherwise the index of the cohort
    // it covers, whose keys and count replace the scene's at finalize. An index rather than a
    // pointer so the slot it names stays writable without casting the constness back off.
    int cohortIndex = -1;
};

void failArticulationRootBackend(ReadSession& session, int rootCount, const char* reason)
{
    CARB_LOG_WARN_ONCE(
        "ovphysx read: %d articulation root(s) not sourced from the tensor "
        "backend (%s); their columns are omitted.",
        rootCount, reason);
    session.backendFailed = true;
}

bool buildArticulationRoot(ReadSession& s, IPhysicsSource& source, PxScene* scene, ArticulationReadCacheEntry& cache,
                           uint32_t scope, const std::vector<const ArticulationRootAttributeRow*>& requested)
{
    const size_t rootCount = cache.rootArtis.size();
    const int rootCountForLog = static_cast<int>(rootCount);

    if (cache.duplicateRootPointers || cache.rootKeys.size() != rootCount || cache.roots.size() != rootCount)
    {
        failArticulationRootBackend(s, rootCountForLog, "duplicate or inconsistent ePTArticulation records");
        return false;
    }
    if (rootCount == 0)
        return true;

    std::vector<PxArticulationReducedCoordinate*> selectedArtis;
    std::vector<ObjectKey> selectedKeys;
    const std::vector<PxArticulationReducedCoordinate*>* artis = &cache.rootArtis;
    const std::vector<ObjectKey>* keys = &cache.rootKeys;
    if (scope == omni::physx::kOvxActive)
    {
        const ActiveActorSet activeActors = collectActiveActorsForScene(scene);
        selectedArtis.reserve(rootCount);
        selectedKeys.reserve(rootCount);
        for (size_t rootIndex = 0; rootIndex < rootCount; ++rootIndex)
        {
            if (!articulationRootInScope(cache, rootIndex, scope, activeActors, scene))
                continue;
            selectedArtis.push_back(cache.rootArtis[rootIndex]);
            selectedKeys.push_back(cache.rootKeys[rootIndex]);
        }
        artis = &selectedArtis;
        keys = &selectedKeys;
    }
    if (artis->empty())
        return true;

    if (articulationSceneInsertionPending(*artis))
        return true;

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    // DirectGPU has no readable state before the first simulation step. This is the documented
    // step-first unavailable state, not a backend failure: omit the whole articulation set so a
    // host-only property requested alongside live state cannot look like a complete mixed read.
    if (gpu && !directGpuSceneReady(scene, cache))
        return true;

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        failArticulationRootBackend(s, rootCountForLog, "no tensor backend");
        return false;
    }

    PxCudaContextManager* ctxMgr = nullptr;
    PxCudaContext* cu = nullptr;
    std::optional<PxScopedCudaLock> cudaLock;
    if (gpu)
    {
        ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            failArticulationRootBackend(s, rootCountForLog, "no CUDA context");
            return false;
        }
        cu = ctxMgr->getCudaContext();
        cudaLock.emplace(*ctxMgr);
    }

    const bool cacheable = scope == omni::physx::kOvxAll;
    const bool requestsShapeTopology = std::any_of(
        requested.begin(), requested.end(), [](const ArticulationRootAttributeRow* row) { return row && row->hostOnly; });
    // Shape topology belongs to the scene's structural epoch, not to one ALL/ACTIVE membership.
    // Validate every root once after an object-database mutation so a backend view whose aggregate
    // maxima happened not to change is still rebuilt, then keep both scopes off this O(links +
    // shapes) path until the next structural epoch.
    const bool requireShapeTopology = requestsShapeTopology && cache.rootShapeTopologyEpoch != cache.dbEpoch;
    const std::vector<uint32_t>* cachedRows = (cacheable && cache.rootGeneration != 0) ? &cache.rootRows : nullptr;
    ArticulationViewBinding binding;
    bool haveBinding = false;
    if (requireShapeTopology)
    {
        const std::vector<uint32_t>* validationRows = cache.rootGeneration != 0 ? &cache.rootRows : nullptr;
        ArticulationViewBinding validationBinding;
        if (!acquireArticulationBinding(
                backend, scene, gpu, cache.rootArtis, validationBinding, validationRows, cache.rootGeneration, true) ||
            !validationBinding.resolvedLocalToRow)
        {
            failArticulationRootBackend(s, rootCountForLog, "superset articulation shape topology unavailable");
            return false;
        }
        cache.rootShapeTopologyEpoch = cache.dbEpoch;
        if (cacheable)
        {
            binding = std::move(validationBinding);
            if (!binding.usedCachedRows)
                binding.resolvedLocalToRow = &binding.localToRow;
            haveBinding = true;
        }
    }
    if ((!haveBinding &&
         !acquireArticulationBinding(backend, scene, gpu, *artis, binding, cachedRows, cache.rootGeneration)) ||
        !binding.resolvedLocalToRow)
    {
        failArticulationRootBackend(s, rootCountForLog, "superset articulation view unavailable");
        return false;
    }

    if (binding.entries->size() != cache.rootArtis.size())
    {
        CARB_LOG_WARN_ONCE(
            "ovphysx read: the scene-wide articulation view contains %zu row(s), while "
            "the database exposes %zu root record(s); backend-only rows are ignored.",
            binding.entries->size(), cache.rootArtis.size());
    }

    const std::vector<uint32_t>* rows = binding.resolvedLocalToRow;
    if (!binding.usedCachedRows)
    {
        std::unordered_set<uint32_t> uniqueRows;
        uniqueRows.reserve(rows->size());
        bool validRows = rows->size() == artis->size();
        for (uint32_t row : *rows)
        {
            validRows = validRows && row < binding.entries->size() && uniqueRows.insert(row).second;
            if (!validRows)
                break;
        }
        if (!validRows)
        {
            failArticulationRootBackend(s, rootCountForLog, "missing, duplicate, or out-of-range articulation view row");
            return false;
        }
    }

    uint64_t rowsVersion = 0;
    std::vector<ovx_primpath_t> selectedHandles;
    const std::vector<ovx_primpath_t>* canonicalHandles = nullptr;
    if (cacheable)
    {
        if (!binding.usedCachedRows)
        {
            cache.rootRows = *rows;
            cache.rootGeneration = binding.generation;
            cache.rootRowsVersion = nextRowsVersion();
            cache.rootHandles.clear();
            omni::physics::ovstage::canonicalisePathHandles(
                source, cache.rootKeys.data(), cache.rootKeys.size(), cache.rootHandles, nullptr);
        }
        else if (cache.rootHandles.size() != cache.rootKeys.size())
        {
            cache.rootHandles.clear();
            omni::physics::ovstage::canonicalisePathHandles(
                source, cache.rootKeys.data(), cache.rootKeys.size(), cache.rootHandles, nullptr);
        }
        rows = &cache.rootRows;
        rowsVersion = cache.rootRowsVersion;
        if (cache.rootHandles.size() == cache.rootKeys.size())
            canonicalHandles = &cache.rootHandles;
    }
    else
    {
        // ACTIVE membership is current-step state. Canonicalise its current keys once for all
        // attributes, but neither retain the selection nor reuse its device-upload identity.
        omni::physics::ovstage::canonicalisePathHandles(source, keys->data(), keys->size(), selectedHandles, nullptr);
        if (selectedHandles.size() == keys->size())
            canonicalHandles = &selectedHandles;
        rowsVersion = nextRowsVersion();
    }
    const PxU32 outputCount = static_cast<PxU32>(artis->size());
    omni::physx::tensors::GpuArticulationView* gpuView = binding.gpuAv;
    omni::physx::tensors::CpuArticulationView* cpuView = binding.cpuAv;
    omni::physx::tensors::BaseArticulationView* baseView =
        gpu ? static_cast<omni::physx::tensors::BaseArticulationView*>(gpuView) :
              static_cast<omni::physx::tensors::BaseArticulationView*>(cpuView);

    // Cohorts, built once and only when something asks for a per-cohort column. Membership is the interned
    // metatype pointer, which is already the identity isHomogeneous() compares. First appearance wins the
    // ordering, and `rows` is in database order, so cohort order is too (REQ-READ-INVDYN-001 AC-3).
    //
    // Derived per read and never cached, which satisfies REQ-READ-INVDYN-001 AC-9 with no invalidation of
    // its own: a structural change alters `rows` or the interned metatypes, and the next read partitions
    // from those.
    std::vector<ArticulationCohort> cohorts;
    const bool wantCohorts = std::any_of(requested.begin(), requested.end(),
                                         [](const ArticulationRootAttributeRow* row) { return row && row->perCohort; });
    if (wantCohorts)
    {
        for (PxU32 i = 0; i < outputCount; ++i)
        {
            const PxU32 viewRow = (*rows)[i];
            const omni::physx::tensors::ArticulationMetatype* const metatype = baseView->getMetatype(viewRow);
            std::vector<ArticulationCohort>::iterator found = cohorts.begin();
            for (; found != cohorts.end(); ++found)
                if (found->metatype == metatype)
                    break;
            if (found == cohorts.end())
            {
                cohorts.push_back(ArticulationCohort{});
                found = cohorts.end() - 1;
                found->metatype = metatype;
            }
            found->rows.push_back(viewRow);
            found->keys.push_back((*keys)[i]);
            if (canonicalHandles)
                found->handles.push_back((*canonicalHandles)[i]);
        }
    }

    CUdeviceptr columnBase = 0;
    size_t totalFloats = 0;
    if (gpu)
    {
        for (const ArticulationRootAttributeRow* row : requested)
        {
            if (!row || row->hostOnly)
                continue;
            if (!row->perCohort)
            {
                const size_t wide = size_t(outputCount) * size_t(row->components);
                const size_t padded = alignColumnFloats(wide);
                if (padded < wide || padded > SIZE_MAX - totalFloats)
                {
                    failArticulationRootBackend(s, rootCountForLog, "column sizing overflows");
                    return false;
                }
                totalFloats += padded;
                continue;
            }
            // A per-cohort column is one sub-allocation per cohort, each at that cohort's own width.
            for (const ArticulationCohort& cohort : cohorts)
            {
                const PxU32 cohortWidth =
                    baseView->inverseDynamicsColumnWidthForRow(cohort.rows.front(), row->inverseDynamicsColumn);
                // Zero is structurally unservable and is skipped, matching the emit pass.
                if (cohortWidth == 0)
                    continue;
                // Too wide for DLPack's uint16_t dtype.lanes. The finalize-time guard already
                // refuses this and fails the read; refusing HERE keeps the same outcome without
                // first sizing, allocating and gathering a column that cannot be handed out.
                //
                // Fails the read rather than skipping the cohort: the established contract is an
                // ERROR, not a quiet omission, and a skip here would also diverge from the emit
                // pass -- which the assert below relies on agreeing with this one.
                if (cohortWidth > UINT16_MAX)
                {
                    CARB_LOG_ERROR("ovxReadAttributes: '%s' has a cohort width of %u, which does not fit "
                                   "DLPack's 16-bit dtype.lanes; the read is refused during sizing rather "
                                   "than after allocating and gathering it.",
                                   row->token, cohortWidth);
                    failArticulationRootBackend(s, rootCountForLog, "cohort column too wide for DLPack lanes");
                    return false;
                }
                const size_t wide = cohort.rows.size() * size_t(cohortWidth);
                if (cohortWidth != 0 && cohort.rows.size() > SIZE_MAX / size_t(cohortWidth))
                {
                    failArticulationRootBackend(s, rootCountForLog, "cohort column sizing overflows");
                    return false;
                }
                const size_t padded = alignColumnFloats(wide);
                if (padded < wide || padded > SIZE_MAX - totalFloats)
                {
                    failArticulationRootBackend(s, rootCountForLog, "cohort column sizing overflows");
                    return false;
                }
                totalFloats += padded;
            }
        }
        if (totalFloats > SIZE_MAX / sizeof(float))
        {
            failArticulationRootBackend(s, rootCountForLog, "column byte size overflows");
            return false;
        }
        if (totalFloats > 0)
        {
            // Pool-or-allocate; acquireColumn also registers it for release.
            columnBase = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, totalFloats * sizeof(float)));
            if (!columnBase)
            {
                failArticulationRootBackend(s, rootCountForLog, "column allocation failed");
                return false;
            }
        }
    }
    std::vector<PendingArticulationRootGroup> pending;
    pending.reserve(requested.size());
    size_t columnCursor = 0;
    int perShapeWidth = -1;

    // Where one group's values will go: the pending record and a destination tensor, allocated but
    // not yet filled. Separate from the read below because the root-state columns are allocated
    // several at a time and then filled by ONE call, so allocation cannot be welded to a
    // one-row-one-call dispatch.
    const auto beginGroup = [&](const ArticulationRootAttributeRow* row, PxU32 groupCount, int width,
                                int cohortIndex, PendingArticulationRootGroup& item,
                                omni::physics::tensors::TensorDesc& tensor) -> bool
    {
        item.token = row->token;
        item.width = width;
        item.cohortIndex = cohortIndex;
        item.group.dtype = dlTypeOf(row->dtype);
        tensor.dtype = row->dtype;
        tensor.numDims = 2;
        tensor.dims[0] = groupCount;
        tensor.dims[1] = width;

        // A host-only column and a CPU-backend column allocate the same way and differ only in which
        // view reads them; only a device-backed column carves out of the shared device buffer.
        if (row->hostOnly || !gpu)
        {
            tensor.device = -1;
            tensor.data = allocHostColumn(item.group, row->dtype, size_t(groupCount) * size_t(width));
            return tensor.data != nullptr;
        }
        const CUdeviceptr column = columnBase + columnCursor * sizeof(float);
        columnCursor += alignColumnFloats(size_t(groupCount) * size_t(width));
        tensor.device = binding.devOrd;
        tensor.data = reinterpret_cast<void*>(column);
        item.group.deviceData = reinterpret_cast<void*>(column);
        item.group.deviceOrdinal = binding.devOrd;
        item.group.ctxMgr = ctxMgr;
        return true;
    };

    // One group: `groupRows`/`groupKeys` are the whole scene partition for an ordinary column, or one
    // cohort's subset for a per-cohort one. Everything below is identical either way, which is why it
    // is written once.
    const auto emitGroup = [&](const ArticulationRootAttributeRow* row, const PxU32* groupRows, PxU32 groupCount,
                               int width, int cohortIndex) -> bool
    {
        // The device selection cache keys on (token, count), and its contract is that the token changes
        // whenever the rows do. Cohorts of one read share `rowsVersion` and can share a row COUNT, so passing
        // it unmodified would hand the second cohort the first one's uploaded indices, silently. The cohort's
        // ordinal separates them and is stable while the selection is, so a warm read still hits the cache.
        //
        // Disjoint bit fields rather than a hash, so non-collision is a property of the layout. Both bounds
        // are asserted because a wrap would not fail here -- it would quietly serve another cohort's rows.
        uint64_t selectionToken = rowsVersion;
        if (cohortIndex >= 0)
        {
            const uint64_t ordinal = uint64_t(cohortIndex) + 1u;
            CARB_ASSERT(ordinal < (uint64_t(1) << 24) && rowsVersion < (uint64_t(1) << 40));
            selectionToken = rowsVersion ^ (ordinal << 40);
        }

        PendingArticulationRootGroup item;
        omni::physics::tensors::TensorDesc tensor;
        if (!beginGroup(row, groupCount, width, cohortIndex, item, tensor))
        {
            failArticulationRootBackend(s, rootCountForLog, "column allocation failed");
            return false;
        }

        const bool got = row->hostOnly ? (baseView->*row->hostRead)(&tensor, groupRows, groupCount) :
                         gpu           ? (gpuView->*row->gpuRead)(&tensor, groupRows, groupCount, selectionToken) :
                                         (cpuView->*row->cpuRead)(&tensor, groupRows, groupCount, 0);
        if (!got)
        {
            failArticulationRootBackend(s, rootCountForLog, gpu ? "device read failed" : "host read failed");
            return false;
        }
        pending.push_back(std::move(item));
        return true;
    };

    // Unserved by the backend this read is taking is "not applicable", not a failure -- the same rule the
    // joint and rigid paths apply. Kept apart from `got` so a null slot and a genuinely failed read cannot
    // report as the same thing. On the device side a non-f32 row is unserved for the same reason: there is
    // no device gather for it, not a broken one.
    const auto served = [&](const ArticulationRootAttributeRow* row) -> bool
    {
        return row->hostOnly ? row->hostRead != kArtiRootNotOnHost :
               gpu           ? (row->gpuRead != kArtiRootNotOnGpu && row->dtype == kF32) :
                               row->cpuRead != kArtiRootNotOnCpu;
    };

    // The ROOT-STATE columns, taken as a SET before the per-column loop below: two host fetches serve all
    // four (eROOT_TRANSFORM fills both pose columns, eROOT_VELOCITIES both velocity ones), and on the device
    // three do, since only the pose pair shares a read type. Only worth diverting above one column, since a
    // single-column read is already one fetch either way.

    // Rows, not their tokens: std::find over `const char*` is POINTER equality, which works only because both
    // sides come out of kArticulationRootAttributes, and would stop de-duplicating in silence the day a token
    // arrives from elsewhere.
    std::vector<const ArticulationRootAttributeRow*> rootStateServed;
    {
        std::vector<const ArticulationRootAttributeRow*> rootStateRows;
        for (const ArticulationRootAttributeRow* row : requested)
        {
            // The rows the reader hands to ONE gather instead of calling per column. Every other row has
            // rootState == eNone and keeps the one-row-one-call path.
            if (!row || row->perCohort || !served(row) || row->rootState == RSQ::eNone)
                continue;
            rootStateRows.push_back(row);
        }

        if (rootStateRows.size() > 1)
        {
            const size_t numRootState = rootStateRows.size();
            std::vector<PendingArticulationRootGroup> items(numRootState);
            std::vector<omni::physics::tensors::TensorDesc> descs(numRootState);
            std::vector<omni::physx::tensors::BaseArticulationView::RootStateColumn> columns(numRootState);
            bool allocated = true;
            for (size_t i = 0; i < numRootState && allocated; ++i)
            {
                // Never perShape or perCohort, so the width is the row's own lane count.
                allocated =
                    beginGroup(rootStateRows[i], outputCount, rootStateRows[i]->components, -1, items[i], descs[i]);
                columns[i].quantity = rootStateRows[i]->rootState;
                columns[i].dst = &descs[i];
            }
            if (!allocated)
            {
                failArticulationRootBackend(s, rootCountForLog, "column allocation failed");
                return false;
            }

            // `rowsVersion` unmodified: these columns are never per-cohort, so they read the whole
            // scene partition -- the row set the bare token already names.
            const bool got =
                gpu ? gpuView->getRootStateColumnsOvStage(columns.data(), static_cast<PxU32>(numRootState),
                                                          rows->data(), outputCount, rowsVersion) :
                      cpuView->getRootStateColumnsOvStage(columns.data(), static_cast<PxU32>(numRootState),
                                                          rows->data(), outputCount);
            if (!got)
            {
                failArticulationRootBackend(s, rootCountForLog, gpu ? "device read failed" : "host read failed");
                return false;
            }
            for (size_t i = 0; i < numRootState; ++i)
            {
                rootStateServed.push_back(rootStateRows[i]);
                pending.push_back(std::move(items[i]));
            }
        }
    }

    // Emitted by row set -- every scene-wide column, then one block per cohort -- because the device
    // selection cache holds exactly one uploaded row list, so each switch between row sets costs an upload.
    // Only cohort ORDER is constrained (REQ-READ-INVDYN-001 AC-3), which the outer loop below preserves;
    // nothing constrains the order attributes appear in.
    for (const ArticulationRootAttributeRow* row : requested)
    {
        if (!row || row->perCohort || !served(row))
            continue;
        if (std::find(rootStateServed.begin(), rootStateServed.end(), row) != rootStateServed.end())
            continue; // already filled, as part of the set above

        if (row->perShape && perShapeWidth < 0)
            perShapeWidth = static_cast<int>(baseView->maxShapesForRows(rows->data(), outputCount));
        const int width = row->perShape ? perShapeWidth : row->components;
        if (width == 0)
        {
            CARB_LOG_WARN_ONCE(
                "ovxReadAttributes: '%s' skipped -- no articulation in the read has a "
                "collision shape.",
                row->token);
            continue;
        }
        if (!emitGroup(row, rows->data(), outputCount, width, -1))
            return false;
    }

    for (size_t cohortIndex = 0; cohortIndex < cohorts.size(); ++cohortIndex)
    {
        const ArticulationCohort& cohort = cohorts[cohortIndex];
        for (const ArticulationRootAttributeRow* row : requested)
        {
            if (!row || !row->perCohort || !served(row))
                continue;

            // Zero means this cohort structurally cannot serve the column -- today, a fixed-base
            // articulation asked for centroidal momentum. Omitted, not failed, and the other
            // cohorts still emit theirs (REQ-READ-INVDYN-001 AC-5).
            const PxU32 cohortWidth =
                baseView->inverseDynamicsColumnWidthForRow(cohort.rows.front(), row->inverseDynamicsColumn);
            if (cohortWidth == 0)
                continue;
            // No width check here. On GPU the sizing pass above has already refused an
            // unrepresentable cohort and returned; on CPU there is no sizing pass, and the
            // finalize-time guard is what refuses it -- which costs nothing there, since the CPU
            // path allocates no device column and runs no gather.
            if (!emitGroup(row, cohort.rows.data(), static_cast<PxU32>(cohort.rows.size()),
                           static_cast<int>(cohortWidth), static_cast<int>(cohortIndex)))
                return false;
        }
    }
    // The sizing pass and the emit pass compute the same widths by the same rules, separately, over a device
    // allocation that sums over cohorts. A rule that disagreed between the two would overrun the allocation
    // and corrupt a later column instead of failing.
    CARB_ASSERT(columnCursor <= totalFloats);

    // Publish only after every requested gather succeeded, so a failed DirectGPU call cannot leave
    // a prefix of apparently valid articulation groups in the read.
    //
    // Two sharing scopes, because two prim sets are in play. The whole-scene columns all cover the
    // same prims and share one interned list; a cohort covers a subset, so it shares only with the
    // other columns of its own cohort. Handing a cohort the scene-wide list would publish the wrong
    // prim identity against the right values.
    ovx_primpath_list_t sharedPrimList = OVX_INVALID_PRIMPATH_LIST;
    for (PendingArticulationRootGroup& item : pending)
    {
        ArticulationCohort* const cohort = item.cohortIndex >= 0 ? &cohorts[item.cohortIndex] : nullptr;
        const std::vector<ObjectKey>& groupKeys = cohort ? cohort->keys : *keys;
        const int64_t groupCount = cohort ? static_cast<int64_t>(cohort->rows.size()) :
                                            static_cast<int64_t>(outputCount);
        const std::vector<ovx_primpath_t>* groupHandles = canonicalHandles;
        if (cohort)
            groupHandles = cohort->handles.size() == cohort->keys.size() ? &cohort->handles : nullptr;
        ovx_primpath_list_t* const listSlot = cohort ? &cohort->list : &sharedPrimList;

        if (!finalizeGroup(s, source, std::move(item.group), item.token, groupKeys, groupCount, item.width,
                           groupHandles, listSlot))
        {
            failArticulationRootBackend(s, rootCountForLog, "path-list construction failed");
            return false;
        }
    }
    return true;
}

void buildArticulationRootGroups(
    ReadSession& s, IPhysicsSource& source, uint32_t scope, const std::vector<std::string>& names)
{
    std::vector<const ArticulationRootAttributeRow*> requested;
    requested.reserve(names.size());
    for (const std::string& name : names)
    {
        const ArticulationRootAttributeRow* row = findArticulationRootAttribute(name);
        if (row)
            requested.push_back(row);
    }
    if (requested.empty())
        return;

    const std::vector<PxScene*> scenes = allPhysicsScenes();
    purgeDeadSceneEntries(g_articulationReadCache.byScene, scenes);
    if (!refreshArticulationStructuralCache(scenes))
    {
        s.backendFailed = true;
        return;
    }
    const size_t firstRootGroup = s.groups.size();
    for (PxScene* scene : scenes)
    {
        const std::unordered_map<const PxScene*, ArticulationReadCacheEntry>::iterator found =
            g_articulationReadCache.byScene.find(scene);
        if (found != g_articulationReadCache.byScene.end() &&
            !buildArticulationRoot(s, source, scene, found->second, scope, requested))
        {
            // No partial answer across scenes: withdraw the groups earlier scenes already
            // published. Device buffers stay session-owned and are retired by ovxReleaseRead; the
            // path lists go with their groups. None of it is reachable -- the read ends terminally.
            rollbackGroups(s, firstRootGroup);
            return;
        }
    }
}

void warnArticulationJointBackend(int jointCount, const char* reason)
{
    CARB_LOG_WARN_ONCE(
        "ovphysx read: %d articulation joint(s) not sourced from the tensor "
        "backend (%s); their joint-state columns are omitted.",
        jointCount, reason);
}

// Articulation joint-state read, sourced from the tensor backend's articulation DOF read (ADR-0008). On GPU it
// is device-resident (correct under suppressReadback, where the per-joint getArticulationJointPosition /
// Velocity is stale); on CPU it reads the articulation cache. Both go through the per-attribute reads named in
// kJointAttributes, which gather the (joint-prim, enabled-axis) subset directly into the output, with no
// intermediate dense [numArti x maxDofs] buffer. One array group per joint. Returns false if the backend path
// is unavailable.
bool buildJointState(ReadSession& s, IPhysicsSource& source, PxScene* scene, const std::vector<std::string>& names)
{

    std::vector<std::string> jointNames;
    for (const std::string& name : names)
        if (findJointAttribute(name) || findJointPropertyAttribute(name))
            jointNames.push_back(name);
    if (jointNames.empty())
        return true;

    // The wrapper refreshes every scene from one epoch-gated database walk; roots determine the articulation
    // ordering, and each joint's viewArtiIdx indexes that walk's per-scene database order.
    const std::unordered_map<const PxScene*, ArticulationReadCacheEntry>::iterator cacheIt =
        g_articulationReadCache.byScene.find(scene);
    if (cacheIt == g_articulationReadCache.byScene.end())
        return true;
    ArticulationReadCacheEntry& jc = cacheIt->second;
    const std::vector<JointRec>& joints = jc.joints;
    const std::vector<PxArticulationReducedCoordinate*>& artis = jc.artis;
    if (joints.empty())
        return true;

    if (articulationSceneInsertionPending(artis))
        return true;

    // Fail-closed like the rigid/link path: warn once so a non-empty joint set that yields no
    // column (backend view unavailable) is diagnosable rather than silently empty.
    const int numJoints = static_cast<int>(joints.size());

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    // A newly attached scene has timestamp 1 before its first simulated frame. Keep discovery
    // available, but do not acquire a DirectGPU view whose state buffers PhysX has not produced.
    //
    // This withholds the whole joint set, INCLUDING the property columns, which need no device buffer
    // and would read correctly. Deliberate, and the same choice buildArticulationRoot makes: a read
    // returning properties but not state would be a partial result no caller can distinguish from a
    // complete one. Revisiting it is a contract change for every articulation type at once.
    if (gpu && !directGpuSceneReady(scene, jc))
        return true;

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        warnArticulationJointBackend(numJoints, "no tensor backend");
        return false;
    }

    // The GPU path holds the CUDA context across view creation + gather; the CPU path needs neither.
    PxCudaContextManager* ctxMgr = nullptr;
    PxCudaContext* cu = nullptr;
    std::optional<PxScopedCudaLock> cudaLock;
    if (gpu)
    {
        // The scene's own manager, not the process default: under multi-GPU the backend view runs in
        // this scene's context, so columns must be allocated there to be addressable by its kernels.
        ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            warnArticulationJointBackend(numJoints, "no CUDA context");
            return false;
        }
        cu = ctxMgr->getCudaContext();
        cudaLock.emplace(*ctxMgr);
    }

    // 2. Articulation view: the backend's cached superset over every articulation in the scene,
    //    reused across reads, on both devices. The record list below indexes it by superset row, so
    //    nothing is constructed per read and nothing is torn down after one.
    ArticulationViewBinding binding;
    if (!acquireArticulationBinding(backend, scene, gpu, artis, binding, nullptr, 0, false, jc.generation))
    {
        warnArticulationJointBackend(numJoints, "superset articulation view unavailable");
        return false;
    }
    omni::physx::tensors::GpuArticulationView* gpuAv = binding.gpuAv;
    omni::physx::tensors::CpuArticulationView* cpuAv = binding.cpuAv;
    const int devOrd = binding.devOrd;
    const uint64_t viewGeneration = binding.generation;
    // Pointer, not a reference: this read asked acquireArticulationBinding to skip resolving the
    // mapping when it is warm, and the skip leaves this null. Dereferenced only inside the branch
    // that rebuilds the records, which is exactly when the skip cannot have fired.
    const std::vector<uint32_t>* const localToRowPtr = binding.resolvedLocalToRow;
    const std::vector<omni::physx::tensors::ArticulationEntry>& entries = *binding.entries;
    // Records, slices and canonical handles come from ensureJointRecords -- the SAME shared builder
    // the write calls into this same ArticulationReadCacheEntry. The degrees detection, the
    // body0IsParent sign and the angScale/invAngScale fold therefore have one producer, so a read
    // and a write on the same generation cannot disagree about a joint's units the way two open-coded
    // copies eventually would. enumCached is true: reaching here means buildJointState just resolved
    // the joint set, so the enumeration behind these superset rows is current.
    ensureJointRecords(jc, source, joints, entries, *localToRowPtr, viewGeneration, /*enumCached=*/true);
    const std::vector<omni::physx::tensors::ArticulationDofOvStageRecord>& recs = jc.recs;
    const std::vector<JointSlice>& slices = jc.slices;
    if (recs.empty())
        return true; // no unlocked DOF on any queried joint -- nothing to emit
    const uint32_t numOut = static_cast<uint32_t>(recs.size());
    const std::vector<ObjectKey>& sliceKeys = jc.sliceKeys;
    const std::vector<ovx_primpath_t>& sliceHandles = jc.sliceHandles;
    // Only usable if every key canonicalised; otherwise the group falls back to building its list
    // from the raw keys.
    const bool haveHandles = sliceHandles.size() == slices.size();


    // GPU: upload the shared record list once; every attribute's gather reads the same device buffer.
    // Only a DEVICE-sourced attribute needs it -- a read asking for nothing but properties gathers
    // entirely on the host, and uploading a record per DOF for it would be the one cost of the GPU
    // path paid by a read that never enters it.
    const bool anyDeviceColumn =
        gpu && std::any_of(jointNames.begin(), jointNames.end(),
                           [](const std::string& name) { return findJointAttribute(name) != nullptr; });
    CUdeviceptr recBuf = 0;
    if (anyDeviceColumn)
    {
        // View-owned and cached: a warm read re-uses the upload instead of re-allocating and
        // re-copying bytes the recsCached gate just declined to rebuild. The token is the record
        // CONTENT version, not the view generation: the generation does not move when an epoch change
        // rebuilds the records under a same-shape view, so keying on it served a stale device buffer.
        // A new view has a null buffer, so it re-uploads regardless. NOT registered for session
        // release: the view owns it and frees it in its destructor.
        recBuf = reinterpret_cast<CUdeviceptr>(gpuAv->ovStageDofRecordsDevice(recs.data(), numOut, jc.recsVersion));
        if (!recBuf)
            return false;
    }

    // The PROPERTY columns, gathered together before the emit loop rather than one at a time inside
    //     it. Each is host-resident and comes out of a per-(joint, axis) PhysX struct, and seven of the
    //     thirteen come out of the SAME struct, so one call means one walk of the record list and at most
    //     four fetches per DOF. The emit loop below then finds the values ready.
    //
    // Keyed by POSITION in `jointNames`, not by name: `names` is the caller's list verbatim, and ADR-0007
    // serves a duplicated attribute twice rather than collapsing it. Under a name key the second occurrence
    // would find the buffer the first one moved out -- a published group with no values.
    std::vector<std::vector<float>> propFloats(jointNames.size());
    std::vector<std::vector<uint8_t>> propBytes(jointNames.size());
    {
        std::vector<omni::physx::tensors::BaseArticulationView::DofPropertyColumn> propColumns;
        std::vector<omni::physics::tensors::TensorDesc> propDescs;
        propColumns.reserve(jointNames.size());
        propDescs.reserve(jointNames.size());
        for (size_t nameIdx = 0; nameIdx < jointNames.size(); ++nameIdx)
        {
            const std::string& name = jointNames[nameIdx];
            if (findJointAttribute(name))
                continue;
            const JointPropertyAttributeRow* propRow = findJointPropertyAttribute(name);
            if (!propRow)
                continue;
            const uint32_t comp = omni::physx::tensors::dofPropertyComponents(propRow->prop);
            const size_t elems = size_t(numOut) * comp;
            omni::physics::tensors::TensorDesc td;
            td.device = -1;
            td.numDims = 1;
            td.dims[0] = static_cast<int64_t>(elems);
            if (omni::physx::tensors::dofPropertyIsByte(propRow->prop))
            {
                std::vector<uint8_t>& buf = propBytes[nameIdx];
                buf.resize(elems);
                td.dtype = omni::physics::tensors::TensorDataType::eUint8;
                td.data = buf.data();
            }
            else
            {
                std::vector<float>& buf = propFloats[nameIdx];
                buf.resize(elems);
                td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
                td.data = buf.data();
            }
            propDescs.push_back(td);
            propColumns.push_back({ propRow->prop, nullptr });
        }
        // The descs are pointed at only after the vector has stopped growing: a push_back that
        // reallocates would leave every previously stored pointer dangling.
        for (size_t c = 0; c < propColumns.size(); ++c)
            propColumns[c].dst = &propDescs[c];

        if (!propColumns.empty())
        {
            const omni::physx::tensors::BaseArticulationView* baseAv =
                gpu ? static_cast<const omni::physx::tensors::BaseArticulationView*>(gpuAv) :
                      static_cast<const omni::physx::tensors::BaseArticulationView*>(cpuAv);
            if (!baseAv->getDofPropertiesOvStage(propColumns.data(),
                                                 static_cast<PxU32>(propColumns.size()), recs.data(), numOut))
            {
                s.backendFailed = true;
                return false;
            }
        }
    }

    // 3a.3. The DOF STATE columns, gathered before the loop: every cache-backed one needs
    //     copyInternalStateToCache first, and PxArticulationCacheFlags is a flag SET, so one pass with the
    //     union fills all of them instead of one refresh pass per column.
    //
    //     CPU only, and not an omission: each DirectGPU state column is its own PxArticulationGPUAPIReadType
    //     filling its own destination, so there is no shared fetch to collapse and the device branch below
    //     reads one column at a time.
    std::vector<std::vector<float>> stateFloats(jointNames.size());
    if (!gpu)
    {
        std::vector<omni::physx::tensors::BaseArticulationView::DofStateColumn> stateColumns;
        std::vector<omni::physics::tensors::TensorDesc> stateDescs;
        stateColumns.reserve(jointNames.size());
        stateDescs.reserve(jointNames.size());
        for (size_t nameIdx = 0; nameIdx < jointNames.size(); ++nameIdx)
        {
            const JointAttributeRow* const row = findJointAttribute(jointNames[nameIdx]);
            if (!row || !row->cpuRead ||
                row->stateQuantity == omni::physx::tensors::BaseArticulationView::DofStateQuantity::eNone)
                continue;
            std::vector<float>& buf = stateFloats[nameIdx];
            buf.resize(numOut);
            omni::physics::tensors::TensorDesc td;
            td.device = -1;
            td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
            td.numDims = 1;
            td.dims[0] = numOut;
            td.data = buf.data();
            stateDescs.push_back(td);
            stateColumns.push_back({ row->stateQuantity, nullptr });
        }
        // Pointed at only once the vector has stopped growing, as above: a reallocating push_back
        // would leave every desc pointer already stored dangling.
        for (size_t c = 0; c < stateColumns.size(); ++c)
            stateColumns[c].dst = &stateDescs[c];

        if (!stateColumns.empty() &&
            !cpuAv->getDofStateColumnsOvStage(stateColumns.data(), static_cast<PxU32>(stateColumns.size()),
                                              recs.data(), numOut))
        {
            s.backendFailed = true;
            return false;
        }
    }

    bool ok = true;
    for (size_t nameIdx = 0; nameIdx < jointNames.size(); ++nameIdx)
    {
        const std::string& name = jointNames[nameIdx];
        // 3b. One-pass ovstage gather into a flat column: device buffer for a device-sourced
        //     attribute on a DirectGPU scene (shared records), host buffer otherwise (records
        //     passed directly).
        const JointAttributeRow* row = findJointAttribute(name);
        const JointPropertyAttributeRow* propRow = row ? nullptr : findJointPropertyAttribute(name);
        // As on the rigid path: unserved by this backend is "not applicable", not a failure.
        if (row && (gpu ? row->gpuRead == nullptr : row->cpuRead == nullptr))
            continue;
        if (!row && !propRow)
            continue;
        // A PROPERTY column is host-resident on both devices (see kJointPropertyAttributes), so the
        // device branch is "device-sourced attribute AND DirectGPU scene", not just the latter.
        const bool deviceColumn = gpu && row != nullptr;
        const uint32_t comp = propRow ? omni::physx::tensors::dofPropertyComponents(propRow->prop) : 1u;
        const bool isByte = propRow && omni::physx::tensors::dofPropertyIsByte(propRow->prop);
        const size_t elems = size_t(numOut) * comp;
        CUdeviceptr outBuf = 0;         // GPU: session-owned device column (groups point into it)
        std::vector<float> hostOut;     // host float column (the group takes ownership below)
        std::vector<uint8_t> hostBytes; // host uint8 column (jointDriveType)
        bool got = false;
        if (deviceColumn)
        {
            // Pool-or-allocate; acquireColumn also registers it for release.
            outBuf = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, size_t(numOut) * sizeof(float)));
            if (!outBuf)
            {
                ok = false;
                break;
            }
            omni::physics::tensors::TensorDesc td;
            td.device = devOrd; td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
            td.numDims = 1; td.dims[0] = numOut; td.data = reinterpret_cast<void*>(outBuf);
            omni::physx::tensors::ArticulationDofOvStageRecord* recsDev =
                reinterpret_cast<omni::physx::tensors::ArticulationDofOvStageRecord*>(recBuf);
            got = (gpuAv->*row->gpuRead)(recsDev, numOut, &td);
            if (!got)
            {
                ok = false;
                break;
            }
        }
        else
        {
            if (propRow)
            {
                // Gathered already, in the single batched walk above -- taken, not re-read. Moved
                // rather than copied: the store is dead after this loop and the buffer is O(dofs).
                if (isByte)
                    hostBytes = std::move(propBytes[nameIdx]);
                else
                    hostOut = std::move(propFloats[nameIdx]);
                got = true;
            }
            else if (row->stateQuantity != omni::physx::tensors::BaseArticulationView::DofStateQuantity::eNone)
            {
                // Filled by the one refresh pass above; taken here so group order still follows the
                // caller's list. The ROW says whether this is a state column, the same way the branch
                // above asks propRow -- asking the buffer instead would be a second way to decide a
                // fact the table already carries. `row` is non-null here: propRow is set only when
                // row is null, so the branch above took that case.
                hostOut = std::move(stateFloats[nameIdx]);
                got = true;
            }
            else
            {
                // jointProjectedForce: derived rather than read off the cache, so it has no state
                // quantity and no share of the refresh.
                omni::physics::tensors::TensorDesc td;
                td.device = -1;
                td.numDims = 1;
                td.dims[0] = static_cast<int64_t>(elems);
                hostOut.resize(elems);
                td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
                td.data = hostOut.data();
                got = (cpuAv->*row->cpuRead)(recs.data(), numOut, &td);
            }
            if (!got)
            {
                ok = false;
                break;
            }
        }

        // 3c. ONE array group for the whole attribute: one tensor per joint, which is ovstage's array shape,
        // so a prim list, a fetch, a release and a consumer wait are paid once per attribute rather than once
        // per joint. The values are already contiguous -- the gather wrote one flat buffer and each joint owns
        // a slice of it -- so the per-joint tensors point at their offsets and nothing is copied here.
        GroupStore g;
        g.isArray = true;
        g.deviceOrdinal = deviceColumn ? devOrd : -1;
        g.ctxMgr = deviceColumn ? ctxMgr : nullptr;
        // The group owns the host values its tensors point into. Moved rather than copied: the
        // column is one float (or byte) per DOF of the read and nothing else refers to it.
        if (!deviceColumn)
        {
            if (isByte)
                g.bytes = std::move(hostBytes);
            else
                g.floats = std::move(hostOut);
        }
        g.dtype = isByte ? DLDataType{ kDLUInt, 8, 1 } : DLDataType{ kDLFloat, 32, 1 };
        g.shapes.resize(slices.size());
        g.tensors.assign(slices.size(), DLTensor{});
        for (size_t si = 0; si < slices.size(); ++si)
        {
            const JointSlice& sl = slices[si];
            // The row count is the joint's ENABLED-AXIS count either way; a multi-component attribute
            // carries its width in dtype.lanes rather than multiplying it into the shape.
            g.shapes[si] = static_cast<int64_t>(sl.count);
            DLTensor& t = g.tensors[si];
            if (deviceColumn)
                t.data = reinterpret_cast<void*>(outBuf + size_t(sl.offset) * sizeof(float));
            else if (isByte)
                t.data = static_cast<void*>(g.bytes.data() + size_t(sl.offset) * comp);
            else
                t.data = static_cast<void*>(g.floats.data() + size_t(sl.offset) * comp);
            t.device = deviceColumn ? DLDevice{ kDLCUDA, devOrd } : DLDevice{ kDLCPU, 0 };
            t.ndim = 1;
            t.dtype = g.dtype;
            t.dtype.lanes = static_cast<uint16_t>(comp);
            t.shape = nullptr; // patched to &g.shapes[si] at fetch, where the address is stable
            t.strides = nullptr;
            t.byte_offset = 0;
        }
        if (!finalizeArrayGroup(s, source, std::move(g), name, sliceKeys, haveHandles ? &sliceHandles : nullptr))
        {
            ok = false;
            break;
        }
    }
    // Both superset views are backend-owned and outlive the read; nothing to release here.
    return ok;
}

// ----------------------------------------------------------------------------
// Articulation link incoming joint force (one row per link prim — a fixed group)
// ----------------------------------------------------------------------------

// `cache` is null under kOvxActive and non-null under kOvxAll -- the caller owns that decision because
// it owns the scan. Never cache ACTIVE: its membership is recomputed every step without the epoch
// moving, so a walk skipped on an unchanged epoch serves the previous step's set.
bool buildLinkIncomingJointForce(ReadSession& s,
                                 IPhysicsSource& source,
                                 PxScene* scene,
                                 const std::vector<PxRigidBody*>& bodies,
                                 const std::vector<ObjectKey>& keys,
                                 const std::vector<std::string>& names,
                                 CachedLinkScan* cache)
{
    const bool wanted = std::any_of(names.begin(), names.end(),
                                    [](const std::string& name)
                                    {
                                        return std::any_of(std::begin(kArticulationLinkOnlyAttributes),
                                                           std::end(kArticulationLinkOnlyAttributes),
                                                           [&name](const char* token) { return name == token; });
                                    });
    if (!wanted || bodies.empty())
        return true;

    // Same per-scene gate as the pose/velocity path, for the same reason: this category reached the
    // backend ungated and turned an unstepped scene into a whole-read failure.
    if (directGpuPartitionUnready(scene))
        return true;

    // Defined before the walk so the precondition below can report through it too: every failure exit funnels
    // through here. `numLinks` is the queried body count until the walk narrows it to the links actually
    // found, and the lambda reads it by reference at call time.
    int numLinks = static_cast<int>(bodies.size());
    auto warnNoColumn = [&](const char* reason)
    {
        CARB_LOG_WARN_ONCE("ovphysx read: %d articulation link(s) not sourced from the tensor backend "
                           "(%s); their linkIncomingJointForce column is omitted.",
                           numLinks, reason);
        // This function's bool is ANDed into a value buildArticulationLinkGroups discards, so without
        // the flag a dropped column reports as a successful read of a scene with no links.
        s.backendFailed = true;
    };

    // A broken precondition, not "nothing to do": the walk below indexes `keys` by the same i as `bodies`, so
    // a short key list would read past its end. Refused and flagged rather than reported as a successful
    // empty read.
    if (bodies.size() != keys.size())
    {
        warnNoColumn("link and key lists disagree");
        return false;
    }

    // 1. The queried links, and the articulations they belong to in this read's ordering. Walked
    //    from the prim list the link path already produced rather than from the object database
    //    again: the caller's `bodies` IS that list, scope filtering included.
    //
    //    Built ONCE per lifetime epoch when the caller supplies a cache, since this walk dominates the read.
    //    The scan it walks is cached on the same epoch one level up and invalidated with it, so the two
    //    cannot disagree about which links exist.
    //
    //    The four vectors bind to the cache when there is one and to scratch when there is not, so
    //    the walk below fills them in place either way and there is one spelling downstream.
    std::vector<PxArticulationReducedCoordinate*> scratchArtis;
    std::vector<uint32_t> scratchLocalArtiIdx;
    std::vector<uint32_t> scratchLinkIdx;
    std::vector<ObjectKey> scratchLinkKeys;
    std::vector<PxArticulationReducedCoordinate*>& artis = cache ? cache->artis : scratchArtis;
    std::vector<uint32_t>& localArtiIdx = cache ? cache->localArtiIdx : scratchLocalArtiIdx;
    std::vector<uint32_t>& linkIdx = cache ? cache->linkIdx : scratchLinkIdx;
    std::vector<ObjectKey>& linkKeys = cache ? cache->linkKeys : scratchLinkKeys;

    if (!cache || !cache->enumValid)
    {
        std::unordered_map<PxArticulationReducedCoordinate*, uint32_t> artiIndex;
        artis.clear();
        localArtiIdx.clear();
        linkIdx.clear();
        linkKeys.clear();
        localArtiIdx.reserve(bodies.size());
        linkIdx.reserve(bodies.size());
        linkKeys.reserve(bodies.size());
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            PxArticulationLink* link = bodies[i] ? bodies[i]->is<PxArticulationLink>() : nullptr;
            if (!link)
                continue;
            PxArticulationReducedCoordinate* arti =
                &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
            if (arti->getScene() != scene)
                continue; // read each articulation through its owning scene's view
            const std::unordered_map<PxArticulationReducedCoordinate*, uint32_t>::iterator it = artiIndex.find(arti);
            uint32_t vi;
            if (it == artiIndex.end())
            {
                vi = static_cast<uint32_t>(artis.size());
                artiIndex[arti] = vi;
                artis.push_back(arti);
            }
            else
            {
                vi = it->second;
            }
            localArtiIdx.push_back(vi);
            linkIdx.push_back(link->getLinkIndex());
            linkKeys.push_back(keys[i]);
        }
        if (cache)
            cache->enumValid = true;
    }

    // Genuinely nothing to do: a body that is not an articulation link, or belongs to another scene,
    // is skipped above, so an empty set here is a scene with no links to serve rather than a failure.
    numLinks = static_cast<int>(linkKeys.size());    if (linkKeys.empty())
        return true;

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        warnNoColumn("no tensor backend");
        return false;
    }

    PxCudaContextManager* ctxMgr = nullptr;
    PxCudaContext* cu = nullptr;
    std::unique_ptr<PxScopedCudaLock> cudaLock;
    if (gpu)
    {
        ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            warnNoColumn("no CUDA context");
            return false;
        }
        cu = ctxMgr->getCudaContext();
        cudaLock = std::make_unique<PxScopedCudaLock>(*ctxMgr);
    }

    // 2. The same cached superset articulation view the joint read uses, through the same helper.
    ArticulationViewBinding binding;
    if (!acquireArticulationBinding(backend, scene, gpu, artis, binding, nullptr, 0, false,
                                    cache ? cache->generation : 0))
    {
        warnNoColumn("superset articulation view unavailable");
        return false;
    }
    const int devOrd = binding.devOrd;

    // 3. One record per link prim, naming its superset row and its index within the articulation.
    //    Gated on the view GENERATION, not the epoch that gates the walk above: a record names a
    //    superset row, so it survives only the view build that numbered the rows. Caching it on the
    //    epoch would let it outlive a view rebuild and address rows that no longer mean the same
    //    thing -- which fails by returning another selection's values rather than by failing.
    std::vector<omni::physx::tensors::ArticulationLinkOvStageRecord> scratchRecs;
    std::vector<omni::physx::tensors::ArticulationLinkOvStageRecord>& recs = cache ? cache->recs : scratchRecs;
    if (!cache || cache->generation != binding.generation || binding.generation == 0)
    {
        recs.assign(linkKeys.size(), omni::physx::tensors::ArticulationLinkOvStageRecord{});
        for (size_t i = 0; i < linkKeys.size(); ++i)
        {
            recs[i].viewArtiIdx = (*binding.resolvedLocalToRow)[localArtiIdx[i]];
            recs[i].physxLinkIdx = linkIdx[i];
        }
        if (cache)
        {
            cache->generation = binding.generation;
            cache->recsVersion = ovx::mintRecordContentVersion();
        }
    }
    const uint32_t numOut = static_cast<uint32_t>(recs.size());
    const uint32_t comp = 6;

    // Canonicalised once per epoch alongside the enumeration, for the same reason: it is a function of the
    // prim list, which the epoch governs.
    std::vector<ovx_primpath_t> scratchHandles;
    std::vector<ovx_primpath_t>& handles = cache ? cache->linkHandles : scratchHandles;
    if (handles.size() != linkKeys.size())
        omni::physics::ovstage::canonicalisePathHandles(source, linkKeys.data(), linkKeys.size(), handles, nullptr);
    const bool haveHandles = handles.size() == linkKeys.size();

    GroupStore g;
    g.deviceOrdinal = gpu ? devOrd : -1;
    g.ctxMgr = gpu ? ctxMgr : nullptr;
    g.dtype = DLDataType{ kDLFloat, 32, 1 };

    omni::physics::tensors::TensorDesc td;
    td.numDims = 1;
    td.dims[0] = static_cast<int64_t>(size_t(numOut) * comp);
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    bool got = false;
    if (gpu)
    {
        // View-owned and cached, like the DOF and tendon record lists (ovStage*RecordsDevice): the
        // record buffer is a pure function of the recs content -- re-uploaded only when the token below
        // changes -- and freed by the view's destructor, NOT session-owned. A per-read memAlloc
        // registered for session release here instead returned an undersized record block to the
        // output-column pool the larger column acquire could never reuse, so it accumulated until it
        // exhausted the budget and forced the hot output back to alloc/free churn.
        const uint64_t recordToken = cache ? cache->recsVersion : ovx::mintRecordContentVersion();
        const CUdeviceptr recBuf = reinterpret_cast<CUdeviceptr>(
            binding.gpuAv->ovStageLinkForceRecordsDevice(recs.data(), numOut, recordToken));
        if (!recBuf)
        {
            warnNoColumn("record buffer unavailable");
            return false;
        }
        // Pool-or-allocate; acquireColumn also registers it for release.
        CUdeviceptr outBuf =
            static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, size_t(numOut) * comp * sizeof(float)));
        if (!outBuf)
        {
            warnNoColumn("device allocation failed");
            return false;
        }
        td.device = devOrd;
        td.data = reinterpret_cast<void*>(outBuf);
        got = binding.gpuAv->getLinkIncomingJointForcesOvStage(
            reinterpret_cast<omni::physx::tensors::ArticulationLinkOvStageRecord*>(recBuf), numOut, &td);
        if (!got)
        {
            warnNoColumn("device gather failed");
            return false;
        }
        g.deviceData = reinterpret_cast<void*>(outBuf);
    }
    else
    {
        g.floats.assign(size_t(numOut) * comp, 0.0f);
        td.device = -1;
        td.data = g.floats.data();
        got = binding.cpuAv->getLinkIncomingJointForcesOvStage(recs.data(), numOut, &td);
        if (!got)
        {
            warnNoColumn("host gather failed");
            return false;
        }
    }

    // A FIXED group -- one row per link prim, `comp` lanes wide -- not an array group: a link has
    // exactly one incoming joint. Same shape as the rigid-body columns over the same prim list.
    if (!finalizeGroup(s, source, std::move(g), omni::physx::OvxAttr::kLinkIncomingJointForce, linkKeys, numOut, comp,
                       haveHandles ? &handles : nullptr))
    {
        warnNoColumn("group emission failed");
        return false;
    }
    return true;
}

void buildJointStateGroups(ReadSession& s,
                           IPhysicsSource& source,
                           const std::vector<std::string>& names)
{
    // Joint state is sourced from the tensor backend's DOF read (device-resident on GPU, host on CPU)
    // via buildJointState, once per scene; there is no separate host per-axis reader.
    //
    // BOTH joint tables, matching the gate buildJointState applies. A properties-only read is
    // legitimate, and returning here would hand it an empty result rather than a refusal.
    const bool anyJointAttribute =
        std::any_of(names.begin(), names.end(),
                    [](const std::string& name)
                    { return findJointAttribute(name) != nullptr || findJointPropertyAttribute(name) != nullptr; });
    if (!anyJointAttribute)
        return;

    const std::vector<PxScene*> scenes = allPhysicsScenes();
    purgeDeadSceneEntries(g_articulationReadCache.byScene, scenes);
    if (!refreshArticulationStructuralCache(scenes))
    {
        s.backendFailed = true;
        return;
    }
    const size_t firstJointGroup = s.groups.size();
    for (PxScene* scene : scenes)
    {
        if (!buildJointState(s, source, scene, names))
        {
            rollbackGroups(s, firstJointGroup);
            s.backendFailed = true;
            return;
        }
    }
}

// ----------------------------------------------------------------------------
// Articulation tendons (fixed / spatial — one row per tendon prim, a fixed group)
// ----------------------------------------------------------------------------

// The tendon output attributes. One table for both kinds, because they share every property a
// spatial tendon has; `fixedOnly` marks the two the schema puts on the fixed tendon alone.
//
// A row names a PROPERTY rather than a view method, unlike kRigidAttributes and kJointAttributes.
// Those tables name a method per backend so that forgetting one is a compile error, which is worth
// it where each attribute has its own source. Here every property comes out of one DirectGPU struct
// at a different float offset (and, on the host, one switch over the same enum), so there is no
// per-attribute source to forget: a row is the offset and nothing else.
struct TendonAttributeRow
{
    const char* token;
    omni::physx::tensors::TendonProperty prop;
    bool fixedOnly;
};

constexpr TendonAttributeRow kTendonAttributes[] = {
    { omni::physx::OvxAttr::kTendonStiffness, omni::physx::tensors::TendonProperty::eStiffness, false },
    { omni::physx::OvxAttr::kTendonDamping, omni::physx::tensors::TendonProperty::eDamping, false },
    { omni::physx::OvxAttr::kTendonLimitStiffness, omni::physx::tensors::TendonProperty::eLimitStiffness, false },
    { omni::physx::OvxAttr::kTendonOffset, omni::physx::tensors::TendonProperty::eOffset, false },
    { omni::physx::OvxAttr::kTendonLimit, omni::physx::tensors::TendonProperty::eLimit, true },
    { omni::physx::OvxAttr::kTendonRestLength, omni::physx::tensors::TendonProperty::eRestLength, true },
};

const TendonAttributeRow* findTendonAttribute(std::string_view token, bool fixed)
{
    for (const TendonAttributeRow& row : kTendonAttributes)
        if (token == row.token && (fixed || !row.fixedOnly))
            return &row;
    return nullptr;
}

// One queried tendon: the prim that roots it, and the (articulation, tendon) pair to read it from.
// The articulation POINTER is deliberately not here -- viewArtiIdx indexes `artis`, which holds it
// once for the scene rather than once per tendon.
//
// The prim is the ROOT of the tendon -- the joint carrying PhysxTendonAxisRootAPI, or the link
// carrying PhysxTendonAttachmentRootAPI. A tendon's other axes and attachments are database records
// too, but they are parts of the tendon rather than tendons, so only roots become rows.
// TendonRec and TendonReadCacheEntry live in OvxPhysicsShared.h: the write derives the same tendon
// set through them, so one definition is what keeps the two directions naming the same tendons. The
// STORAGE and the walk stay here, as with the articulation cache.
std::unordered_map<const PxScene*, TendonReadCacheEntry> g_fixedTendonReadCache;
std::unordered_map<const PxScene*, TendonReadCacheEntry> g_spatialTendonReadCache;

// Forward declaration: the shared accessor below sits beside the caches it serves, ahead of the walk.
void enumerateTendons(PxScene* scene, bool fixed, std::vector<TendonRec>& out,
                      std::vector<::physx::PxArticulationReducedCoordinate*>& artis);

} // namespace (reopened below)

namespace omni::physx::ovx
{
// Declared in OvxPhysicsShared.h. Same epoch gate the read's own path uses, so a write that runs
// first pays the walk and the read that follows gets the hit, or the reverse -- either way it
// happens once per structural change rather than once per direction.
TendonReadCacheEntry& tendonCacheEntry(::physx::PxScene* scene, bool fixed)
{
    TendonReadCacheEntry& tc = fixed ? g_fixedTendonReadCache[scene] : g_spatialTendonReadCache[scene];
    const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();
    // No !tendons.empty() guard: dbEpoch IS the "nothing cached yet" sentinel, since a fresh entry
    // holds 0 and the epoch counter starts at 1. Adding one would make an EMPTY result uncacheable.
    if (tc.dbEpoch != dbEpochNow)
    {
        tc.tendons.clear();
        tc.artis.clear();
        enumerateTendons(scene, fixed, tc.tendons, tc.artis);
        tc.dbEpoch = dbEpochNow;
        tc.generation = 0; // the records describe the previous tendon set; retire them
    }
    return tc;
}
} // namespace omni::physx::ovx

namespace
{

// Collect the scene's tendons of one kind, in database order, each with the index PhysX gives it
// within its articulation.
//
// Driven from the database rather than by walking articulations, so a tendon's PRIM comes for free:
// the record that created it is keyed by the root prim's path, and UsdInterface stores that
// record's id in the PhysX object's userData. Walking articulations instead would find the tendons
// but not the prims, and would then need the userData hop anyway.
//
// NOT bucketed by scene, unlike scanRigidRecords / scanLinkRecords: this filters on getScene() and is called
// once per scene, so S scenes cost S walks where those cost one. Deliberate -- the epoch cache upstream means
// this runs on a structural change rather than per read, and at one scene the two are identical. Revisit if a
// workload appears that runs many scenes AND mutates them often.
void enumerateTendons(PxScene* scene,
                      bool fixed,
                      std::vector<TendonRec>& out,
                      std::vector<::physx::PxArticulationReducedCoordinate*>& artis)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    std::unordered_map<::physx::PxArticulationReducedCoordinate*, uint32_t> artiIndex;
    // Resolving a tendon's index means asking its articulation for its tendon list, so the list is
    // fetched once per articulation rather than once per tendon.
    std::unordered_map<const ::physx::PxArticulationReducedCoordinate*,
                       std::unordered_map<const void*, uint32_t>> tendonIndexByArti;

    const omni::physx::PhysXType wantType = fixed ? omni::physx::ePTFixedTendonAxis : omni::physx::ePTTendonAttachment;
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != wantType || !rec.mPtr)
            continue;

        // Roots only: a non-root axis or attachment is part of a tendon, not one of its own.
        const void* tendonPtr = nullptr;
        ::physx::PxArticulationReducedCoordinate* arti = nullptr;
        if (fixed)
        {
            ::physx::PxArticulationTendonJoint* tj =
                reinterpret_cast<::physx::PxArticulationTendonJoint*>(rec.mPtr);
            if (tj->getParent())
                continue;
            ::physx::PxArticulationFixedTendon* tendon = tj->getTendon();
            if (!tendon)
                continue;
            tendonPtr = tendon;
            arti = tendon->getArticulation();
        }
        else
        {
            ::physx::PxArticulationAttachment* at = reinterpret_cast<::physx::PxArticulationAttachment*>(rec.mPtr);
            if (at->getParent())
                continue;
            ::physx::PxArticulationSpatialTendon* tendon = at->getTendon();
            if (!tendon)
                continue;
            tendonPtr = tendon;
            arti = tendon->getArticulation();
        }
        if (!arti || arti->getScene() != scene)
            continue; // read each articulation through its owning scene's view

        std::unordered_map<const ::physx::PxArticulationReducedCoordinate*,
                           std::unordered_map<const void*, uint32_t>>::iterator idxIt =
            tendonIndexByArti.find(arti);
        if (idxIt == tendonIndexByArti.end())
        {
            std::unordered_map<const void*, uint32_t> byPtr;
            if (fixed)
            {
                const uint32_t n = arti->getNbFixedTendons();
                std::vector<::physx::PxArticulationFixedTendon*> buf(n);
                if (n)
                    arti->getFixedTendons(buf.data(), n);
                for (uint32_t i = 0; i < n; ++i)
                    byPtr[buf[i]] = i;
            }
            else
            {
                const uint32_t n = arti->getNbSpatialTendons();
                std::vector<::physx::PxArticulationSpatialTendon*> buf(n);
                if (n)
                    arti->getSpatialTendons(buf.data(), n);
                for (uint32_t i = 0; i < n; ++i)
                    byPtr[buf[i]] = i;
            }
            idxIt = tendonIndexByArti.emplace(arti, std::move(byPtr)).first;
        }
        const std::unordered_map<const void*, uint32_t>::const_iterator hit = idxIt->second.find(tendonPtr);
        if (hit == idxIt->second.end())
            continue; // a tendon its own articulation does not list: nothing addressable to read

        std::unordered_map<::physx::PxArticulationReducedCoordinate*, uint32_t>::iterator it = artiIndex.find(arti);
        uint32_t vi;
        if (it == artiIndex.end())
        {
            vi = static_cast<uint32_t>(artis.size());
            artiIndex[arti] = vi;
            artis.push_back(arti);
        }
        else
        {
            vi = it->second;
        }
        out.push_back({ rec.mKey, hit->second, vi });
    }
}

// Emit one fixed group per requested attribute: one row per tendon prim, every queried tendon of
// the scene stacked into one tensor.
//
// A fixed group rather than the joint read's array-per-prim shape because a tendon IS a prim here
// -- the root prim -- so one row per prim is the honest mapping and the cheaper one. The schema
// does allow several roots on one prim (multi-apply, distinct instance names), which shows up as
// that prim's key repeating across consecutive rows; the rows are still correct and correctly
// ordered, they just cannot be told apart by key alone.
bool buildTendonState(ReadSession& s,
                      IPhysicsSource& source,
                      PxScene* scene,
                      bool fixed,
                      const std::vector<std::string>& names)
{
    std::vector<std::string> wanted;
    for (const std::string& name : names)
        if (findTendonAttribute(name, fixed))
            wanted.push_back(name);
    if (wanted.empty())
        return true;

    // Before the enumeration cache is even consulted: an unready scene should leave no trace, and a
    // walk recorded against this epoch would be cached under a readiness that has not happened yet.
    if (directGpuPartitionUnready(scene))
        return true;

    // operator[], deliberately: the inserted entry is what caches the answer, INCLUDING an empty
    // one. Looking up first and skipping the insert on a miss would re-walk the database on every
    // read of a tendonless scene; looking up first and inserting anyway just pays two lookups. The
    // insert is bounded and freed -- one entry per live scene per kind, purged by
    // purgeDeadSceneEntries above -- and it happens only for a scene whose read actually asked for
    // a tendon attribute, since the early-out above returns first.
    // The shared accessor holds the epoch gate now, so the write derives this set the same way.
    const uint64_t dbEpochBefore = (fixed ? g_fixedTendonReadCache[scene] : g_spatialTendonReadCache[scene]).dbEpoch;
    TendonReadCacheEntry& tc = ovx::tendonCacheEntry(scene, fixed);
    const bool enumCached = tc.dbEpoch == dbEpochBefore;
    // Held by reference, and enumerated in place on a miss. A hit is the whole point of the cache,
    // and copying the vectors out would put an O(tendons) copy back on exactly the path the cache
    // exists to make free.
    const std::vector<TendonRec>& tendons = tc.tendons;
    const std::vector<::physx::PxArticulationReducedCoordinate*>& artis = tc.artis;
    if (tendons.empty())
        return true;

    const int numTendons = static_cast<int>(tendons.size());
    const char* kindName = fixed ? "fixed" : "spatial";
    auto warnNoTendonColumn = [&](const char* reason)
    {
        CARB_LOG_WARN_ONCE("ovphysx read: %d %s tendon(s) not sourced from the tensor backend (%s); "
                           "their property columns are omitted.",
                           numTendons, kindName, reason);
    };

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        warnNoTendonColumn("no tensor backend");
        return false;
    }

    PxCudaContextManager* ctxMgr = nullptr;
    PxCudaContext* cu = nullptr;
    std::unique_ptr<PxScopedCudaLock> cudaLock;
    if (gpu)
    {
        ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            warnNoTendonColumn("no CUDA context");
            return false;
        }
        cu = ctxMgr->getCudaContext();
        cudaLock = std::make_unique<PxScopedCudaLock>(*ctxMgr);
    }

    // The same cached superset view the joint path reads, resolved by the same helper: the records
    // below name superset rows, so they are addressable only for the view build that numbered them.
    ArticulationViewBinding binding;
    if (!acquireArticulationBinding(backend, scene, gpu, artis, binding, nullptr, 0, false,
                                    enumCached ? tc.generation : 0))
    {
        warnNoTendonColumn("superset articulation view unavailable");
        return false;
    }
    omni::physx::tensors::GpuArticulationView* gpuAv = binding.gpuAv;
    omni::physx::tensors::CpuArticulationView* cpuAv = binding.cpuAv;
    const int devOrd = binding.devOrd;
    const uint64_t viewGeneration = binding.generation;
    // Pointer, not a reference -- see buildJointState: the warm-read skip leaves this null, and it is
    // dereferenced only where the records are rebuilt.
    const std::vector<uint32_t>* const localToRowPtr = binding.resolvedLocalToRow;

    // Records and canonical prim handles: attribute-independent, so derived once and cached with
    // the view build they name.
    // Same reasoning as enumCached: generation is the sentinel (a fresh entry holds 0, which
    // viewGeneration != 0 already excludes), and recs cannot be empty here anyway -- the tendon set
    // is non-empty by the early-out above and this builds one record per tendon.
    const bool recsCached = enumCached && tc.generation == viewGeneration && viewGeneration != 0;
    if (!recsCached)
    {
        tc.recs.clear();
        tc.keys.clear();
        tc.handles.clear();
        tc.recs.reserve(tendons.size());
        tc.keys.reserve(tendons.size());
        const std::vector<uint32_t>& localToRow = *localToRowPtr;
        for (const TendonRec& tr : tendons)
        {
            omni::physx::tensors::ArticulationTendonOvStageRecord r;
            r.viewArtiIdx = localToRow[tr.viewArtiIdx];
            r.tendonIdx = tr.tendonIdx;
            tc.recs.push_back(r);
            tc.keys.push_back(tr.key);
        }
        omni::physics::ovstage::canonicalisePathHandles(source, tc.keys.data(), tc.keys.size(), tc.handles,
                                                        nullptr);
        // Last, deliberately: everything above has to have completed for the entry to be claimable,
        // and generation is what claims it. A build that bailed part-way leaves it retired.
        tc.generation = viewGeneration;
        // A fresh content version for the rebuilt records; the device upload keys on this, not on the
        // generation, so it re-uploads whenever the record content changes (see TendonReadCacheEntry).
        tc.recsVersion = ovx::mintRecordContentVersion();
    }
    // By reference, as above: on a hit this is the entire body of work the cache saves.
    const std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord>& recs = tc.recs;
    const std::vector<ObjectKey>& keys = tc.keys;
    const std::vector<ovx_primpath_t>& handles = tc.handles;
    const uint32_t numOut = static_cast<uint32_t>(recs.size());
    const std::vector<ovx_primpath_t>* canonicalHandles = handles.size() == keys.size() ? &handles : nullptr;

    // GPU: the record list is the same for every attribute, so it is uploaded once -- view-owned and
    // cached across reads, not re-allocated and re-copied every read. FIXED and SPATIAL upload to
    // SEPARATE view buffers (the `fixed` flag), so an equal-count read of the two kinds cannot alias.
    // The token is the record CONTENT version (tc.recsVersion), which turns over on every tc.recs
    // rebuild -- including an epoch-driven one the view generation would miss. NOT session-owned: the
    // view frees it in its destructor, so it is not registered for session release.
    CUdeviceptr recBuf = 0;
    if (gpu)
    {
        recBuf = reinterpret_cast<CUdeviceptr>(
            gpuAv->ovStageTendonRecordsDevice(recs.data(), numOut, tc.recsVersion, fixed));
        if (!recBuf)
            return false;
    }

    // Every attribute covers the same prim set, so the list is built once and shared -- releasing a
    // group drops its claim rather than destroying the list (see ovxReleaseGroup).
    ovx_primpath_list_t sharedList = OVX_INVALID_PRIMPATH_LIST;
    bool ok = true;
    for (const std::string& name : wanted)
    {
        const TendonAttributeRow* row = findTendonAttribute(name, fixed);
        if (!row)
            continue;
        const uint32_t comp = omni::physx::tensors::tendonPropertyComponents(row->prop);
        const size_t floats = size_t(numOut) * comp;

        GroupStore g;
        g.isArray = false;
        g.deviceOrdinal = gpu ? devOrd : -1;
        g.ctxMgr = gpu ? ctxMgr : nullptr;

        bool got = false;
        if (gpu)
        {
            // Pool-or-allocate; acquireColumn also registers the buffer for release.
            // Registered up front on purpose: getTendonPropertyOvStage reports failure via
            // cudaGetLastError AFTER launching its gather against this buffer, so a false return does
            // not prove no kernel is writing it. Owning it from allocation means release frees it past
            // the completion eventSynchronize either way, never inline under a live kernel.
            CUdeviceptr outBuf = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, floats * sizeof(float)));
            if (!outBuf)
            {
                ok = false;
                break;
            }
            omni::physics::tensors::TensorDesc td;
            td.device = devOrd;
            td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
            td.numDims = 1;
            td.dims[0] = static_cast<int64_t>(floats);
            td.data = reinterpret_cast<void*>(outBuf);
            omni::physx::tensors::ArticulationTendonOvStageRecord* recsDev =
                reinterpret_cast<omni::physx::tensors::ArticulationTendonOvStageRecord*>(recBuf);
            got = fixed ? gpuAv->getFixedTendonPropertiesOvStage(recsDev, numOut, row->prop, &td) :
                          gpuAv->getSpatialTendonPropertiesOvStage(recsDev, numOut, row->prop, &td);
            if (!got)
            {
                ok = false;
                break;
            }
            g.deviceData = reinterpret_cast<void*>(outBuf);
        }
        else
        {
            g.floats.resize(floats);
            omni::physics::tensors::TensorDesc td;
            td.device = -1;
            td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
            td.numDims = 1;
            td.dims[0] = static_cast<int64_t>(floats);
            td.data = g.floats.data();
            got = fixed ? cpuAv->getFixedTendonPropertiesOvStage(recs.data(), numOut, row->prop, &td) :
                          cpuAv->getSpatialTendonPropertiesOvStage(recs.data(), numOut, row->prop, &td);
            if (!got)
            {
                ok = false;
                break;
            }
        }

        if (!finalizeGroup(s, source, std::move(g), name, keys, static_cast<int64_t>(numOut),
                           static_cast<int64_t>(comp), canonicalHandles, &sharedList))
        {
            ok = false;
            break;
        }
    }

    return ok;
}

void buildTendonGroups(ReadSession& s,
                       IPhysicsSource& source,
                       bool fixed,
                       const std::vector<std::string>& names)
{
    const std::vector<PxScene*> scenes = allPhysicsScenes();
    // Both kinds, not just the one being read: a workload that only ever reads fixed tendons would
    // otherwise never reach the call that frees the spatial cache's dead entries.
    purgeDeadSceneEntries(g_fixedTendonReadCache, scenes);
    purgeDeadSceneEntries(g_spatialTendonReadCache, scenes);

    // Flagged HERE rather than at each failure inside buildTendonState, and for a reason worth
    // stating: of its five `return false` exits only three go through warnNoTendonColumn -- the
    // other two are a device memAlloc and a memcpyHtoD, which fail under memory pressure with no
    // warning at all. Flagging at the group level covers all five, which is also what
    // buildJointStateGroups does with warnArticulationJointBackend.
    //
    // Without this the read reported kOvxReadStatusEndOfIteration on a backend failure -- success,
    // with the tendon columns simply absent -- which is exactly the short-answer-as-success the
    // "a read that cannot produce a column fails" contract exists to prevent.
    const size_t firstTendonGroup = s.groups.size();
    for (PxScene* scene : scenes)
    {
        if (!buildTendonState(s, source, scene, fixed, names))
        {
            // No partial answer across scenes: withdraw what earlier scenes published. Device
            // buffers stay session-owned and are retired by ovxReleaseRead.
            rollbackGroups(s, firstTendonGroup);
            s.backendFailed = true;
            return;
        }
    }
}

// ----------------------------------------------------------------------------
// Vehicle wheels (per-wheel world transform — a fixed group)
// ----------------------------------------------------------------------------

// The attribute set this type serves. One list, read by the group builder's filter and by
// attrNamesForType, so discovery cannot advertise a set the read would then refuse -- the rule
// REQ-READ-ATTRS-001 AC-2 states for the types that have a full attribute table. Two entries do not
// justify a table, but they do justify not writing them out twice.
constexpr const char* kVehicleWheelAttrs[] = { omni::physx::OvxAttr::kPosition,
                                              omni::physx::OvxAttr::kOrientation };

bool isVehicleWheelAttribute(const std::string& name)
{
    for (const char* attr : kVehicleWheelAttrs)
        if (name == attr)
            return true;
    return false;
}

// Which of a vehicle's wheels become rows, and the prim each maps to. One definition, shared by the
// read (which numbers rows off the vehicle view's entries) and by discovery (which needs only the
// keys), so the two cannot disagree about what a wheel row is.
template <typename Fn>
void forEachVehicleWheelRow(InternalVehicle& veh, Fn&& fn)
{
    const size_t numEntries = veh.mWheelTransformManagementEntries.size();
    for (size_t j = 0; j < numEntries; ++j)
    {
        // A detached wheel has no pose to report: removeWheelAttachment also deletes the PhysX
        // shapes mapped to it and disables the wheel in the vehicle SDK.
        if (j >= veh.mWheelAttachments.size() || !veh.mWheelAttachments[j])
            continue;
        fn(static_cast<uint32_t>(j), veh.mWheelTransformManagementEntries[j].wheelRootKey);
    }
}

// kOvxActive for a vehicle is a property of its chassis actor: the wheels are not solver bodies, so
// the only thing that can be reported as moving or not is the body they hang off.
bool vehicleActorInScope(PxRigidDynamic& actor, uint32_t scope, const ActiveActorSet& activeSet)
{
    if (scope != omni::physx::kOvxActive)
        return true;
    if (activeSet.sceneReports(actor.getScene()))
        return activeSet.isActive(reinterpret_cast<size_t>(actor.userData)); // its own record index
    return !actor.isSleeping();
}

// What a vehicle-wheel read derives before it can gather, cached per scene so a steady stepping loop
// derives it once instead of once per read.
//
// Two validators, like the joint and tendon caches, but for different reasons than theirs:
//
//   setEpoch    is InternalScene::mVehicleSetEpoch, and covers the records, the keys and the
//               interned handles alike -- the rows index mVehicles, and that epoch is what moves
//               when mVehicles is renumbered or a wheel is retired. It has to be that epoch and not
//               the object-lifetime one the other caches use, because enable/disable and
//               wheel-attachment removal both do so without touching the object database.
//   generation  covers what setEpoch structurally cannot: it is PER SCENE and restarts at 1 for a
//               fresh InternalScene, while this cache is keyed by PxScene* -- an address the backend
//               documents as recyclable. Detach, reattach onto the same address with the same
//               vehicle count, and the epochs match while the records name the previous stage's
//               prims. The backend's generation is process-wide and monotonic precisely so a
//               consumer can rule that out.
// VehicleReadCacheEntry lives in OvxPhysicsShared.h: the write scatters wheel controls through the
// same records, so one definition keeps the two directions naming the same wheels. The storage and
// the walk stay here.
std::unordered_map<const PxScene*, VehicleReadCacheEntry> g_vehicleReadCache;

} // namespace (reopened below)

namespace omni::physx::ovx
{
// Declared in OvxPhysicsShared.h. Read-only: the walk that fills this needs a CpuSimulationView and
// the InternalScene, which this file holds at that point and the write does not.
VehicleReadCacheEntry* vehicleCacheEntry(const ::physx::PxScene* scene)
{
    const std::unordered_map<const PxScene*, VehicleReadCacheEntry>::iterator it =
        g_vehicleReadCache.find(scene);
    return it == g_vehicleReadCache.end() ? nullptr : &it->second;
}
} // namespace omni::physx::ovx

namespace
{

// Emit one fixed group per requested attribute: one row per wheel prim, every wheel of the scene
// stacked into one tensor, sourced from the scene's vehicle view.
void buildVehicleWheelState(ReadSession& s,
                            IPhysicsSource& source,
                            PxScene* scene,
                            InternalScene& internalScene,
                            uint32_t scope,
                            const ActiveActorSet& activeSet,
                            const std::vector<std::string>& wanted)
{
    // Every backend failure below funnels through here, which is why the session is flagged here
    // rather than at each early return: nothing reads this function's outcome, so an unflagged
    // failure would let ovxFetchReadNext report end-of-iteration -- telling the caller it saw
    // everything while the columns are missing. Same reasoning as emitRigidBodyColumns' warnNoColumn.
    auto warnNoVehicleColumn = [&](const char* reason)
    {
        CARB_LOG_WARN_ONCE("ovphysx read: vehicle wheels not sourced from the tensor backend (%s); "
                           "their transform columns are omitted.",
                           reason);
        s.backendFailed = true;
    };

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    // The CPU scene view unconditionally, with no device branch anywhere below: a vehicle cannot be
    // attached to a DirectGPU scene at all -- its suspension and sticky-tire constraints are custom
    // PxConstraints with a CPU solver-prep function, which PhysX refuses there -- so the scene this
    // runs on is never one acquireSceneView would serve.
    uint64_t generation = 0;
    omni::physx::tensors::CpuSimulationView* sv =
        backend ? backend->acquireCpuSceneView(scene, &generation) : nullptr;
    if (!sv)
    {
        warnNoVehicleColumn("no host scene view");
        return;
    }
    omni::physx::tensors::BaseVehicleView* vv = sv->vehicleView(internalScene);
    if (!vv)
        return; // no vehicles in this scene -- not a failure

    VehicleReadCacheEntry& vc = g_vehicleReadCache[scene];
    if (vc.setEpoch != vv->getBuiltEpoch() || vc.generation != generation)
    {
        // Which rows exist is decided ONCE, here, and never on the composed transform's isValid():
        // validity is a per-step property, so dropping a row for it would silently renumber the caller's
        // prim list mid-run. An invalid transform reports its value, which InternalScene::
        // updateVehicleTransforms warns about rather than hides.
        vc.recs.clear();
        vc.keys.clear();
        vc.handles.clear();
        const std::vector<omni::physx::tensors::VehicleEntry>& entries = vv->getEntries();
        for (uint32_t i = 0; i < entries.size(); ++i)
        {
            forEachVehicleWheelRow(*entries[i].vehicle,
                                   [&](uint32_t wheelIdx, const ObjectKey& key)
                                   {
                                       omni::physx::tensors::VehicleWheelOvStageRecord r;
                                       r.viewVehicleIdx = i;
                                       r.wheelIdx = wheelIdx;
                                       vc.recs.push_back(r);
                                       vc.keys.push_back(key);
                                   });
        }
        omni::physics::ovstage::canonicalisePathHandles(source, vc.keys.data(), vc.keys.size(), vc.handles,
                                                        nullptr);
        // Last, deliberately: both have to be claimed together, and a build that bailed part-way
        // leaves the entry retired rather than half-valid.
        vc.setEpoch = vv->getBuiltEpoch();
        vc.generation = generation;
    }

    // kOvxActive is a per-step property, so it cannot be cached alongside the rows. Filter the
    // cached rows rather than re-deriving them: the handles come out by gather, which keeps the
    // expensive half -- interning a key -- on the cache-miss path even for an active-only read.
    const bool filtered = (scope == omni::physx::kOvxActive);
    std::vector<omni::physx::tensors::VehicleWheelOvStageRecord> subRecs;
    std::vector<ObjectKey> subKeys;
    std::vector<ovx_primpath_t> subHandles;
    if (filtered)
    {
        const std::vector<omni::physx::tensors::VehicleEntry>& entries = vv->getEntries();
        std::vector<bool> inScope(entries.size(), false);
        for (size_t i = 0; i < entries.size(); ++i)
            inScope[i] = vehicleActorInScope(*entries[i].actor, scope, activeSet);
        const bool haveHandles = vc.handles.size() == vc.keys.size();
        for (size_t i = 0; i < vc.recs.size(); ++i)
        {
            if (!inScope[vc.recs[i].viewVehicleIdx])
                continue;
            subRecs.push_back(vc.recs[i]);
            subKeys.push_back(vc.keys[i]);
            if (haveHandles)
                subHandles.push_back(vc.handles[i]);
        }
    }
    const std::vector<omni::physx::tensors::VehicleWheelOvStageRecord>& recs = filtered ? subRecs : vc.recs;
    const std::vector<ObjectKey>& keys = filtered ? subKeys : vc.keys;
    const std::vector<ovx_primpath_t>& handles = filtered ? subHandles : vc.handles;
    const uint32_t numOut = static_cast<uint32_t>(recs.size());
    if (numOut == 0)
        return;
    const std::vector<ovx_primpath_t>* canonicalHandles = handles.size() == keys.size() ? &handles : nullptr;

    // Both attributes cover the same prim set, so the list is built once and shared -- releasing a
    // group drops its claim rather than destroying the list (see ovxReleaseGroup).
    ovx_primpath_list_t sharedList = OVX_INVALID_PRIMPATH_LIST;

    // Both columns fall out of ONE composition per wheel -- getGlobalPose, getCMassLocalPose and either the
    // shape's local pose or the vehicle SDK's, yielding a single PxTransform carrying the position AND the
    // orientation. Filled here in one call; emitted by the loop below in `wanted` order. Only above one
    // column: a single-column read already composes exactly once.
    std::vector<GroupStore> wheelGroups;
    if (wanted.size() > 1)
    {
        wheelGroups.resize(wanted.size());
        std::vector<omni::physics::tensors::TensorDesc> descs(wanted.size());
        std::vector<omni::physx::tensors::BaseVehicleView::WheelTransformColumn> columns(wanted.size());
        for (size_t k = 0; k < wanted.size(); ++k)
        {
            const omni::physx::tensors::VehicleWheelComponent component =
                (wanted[k] == omni::physx::OvxAttr::kOrientation) ?
                    omni::physx::tensors::VehicleWheelComponent::eOrientation :
                    omni::physx::tensors::VehicleWheelComponent::ePosition;
            const size_t floats = size_t(numOut) * omni::physx::tensors::vehicleWheelComponents(component);
            wheelGroups[k].isArray = false;
            wheelGroups[k].floats.resize(floats);
            descs[k].device = -1;
            descs[k].dtype = omni::physics::tensors::TensorDataType::eFloat32;
            descs[k].numDims = 1;
            descs[k].dims[0] = static_cast<int64_t>(floats);
            descs[k].data = wheelGroups[k].floats.data();
            columns[k].component = component;
            columns[k].dst = &descs[k];
        }
        if (!vv->getWheelTransformColumnsOvStage(internalScene, recs.data(), numOut, columns.data(),
                                                 static_cast<uint32_t>(wanted.size())))
        {
            warnNoVehicleColumn("wheel transform gather failed");
            return;
        }
    }

    size_t wheelSlot = 0;
    for (const std::string& name : wanted)
    {
        const size_t slot = wheelSlot++;
        const omni::physx::tensors::VehicleWheelComponent component =
            (name == omni::physx::OvxAttr::kOrientation) ? omni::physx::tensors::VehicleWheelComponent::eOrientation :
                                                           omni::physx::tensors::VehicleWheelComponent::ePosition;
        const uint32_t comp = omni::physx::tensors::vehicleWheelComponents(component);
        const size_t floats = size_t(numOut) * comp;

        // Filled by the set call above; emitted here so group order still follows `wanted`.
        if (!wheelGroups.empty())
        {
            if (!finalizeGroup(s, source, std::move(wheelGroups[slot]), name, keys,
                               static_cast<int64_t>(numOut), static_cast<int64_t>(comp), canonicalHandles,
                               &sharedList))
            {
                warnNoVehicleColumn("group emit failed");
                return;
            }
            continue;
        }

        GroupStore g;
        g.isArray = false;
        g.floats.resize(floats);
        omni::physics::tensors::TensorDesc td;
        td.device = -1;
        td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
        td.numDims = 1;
        td.dims[0] = static_cast<int64_t>(floats);
        td.data = g.floats.data();
        if (!vv->getWheelTransformsOvStage(internalScene, recs.data(), numOut, component, &td))
        {
            warnNoVehicleColumn("wheel transform gather failed");
            return;
        }

        if (!finalizeGroup(s, source, std::move(g), name, keys, static_cast<int64_t>(numOut),
                           static_cast<int64_t>(comp), canonicalHandles, &sharedList))
        {
            warnNoVehicleColumn("group emit failed");
            return;
        }
    }
}

void buildVehicleWheelGroups(ReadSession& s,
                             IPhysicsSource& source,
                             uint32_t scope,
                             const std::vector<std::string>& names)
{
    // Filtered here rather than per scene, so an unsupported attribute is reported once per read
    // instead of once per scene.
    std::vector<std::string> wanted;
    for (const std::string& name : names)
    {
        if (isVehicleWheelAttribute(name))
            wanted.push_back(name);
        else
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not a vehicle-wheel output attribute "
                          "(only position / orientation) — skipped.", name.c_str());
    }
    if (wanted.empty())
        return;

    // The pairs, then the projection -- not allPhysicsScenes(), which would walk the database again.
    const std::vector<ScenePair> scenes = allPhysicsScenesWithInternal();
    purgeDeadSceneEntries(g_vehicleReadCache, scenesOf(scenes));

    // Once for the whole read, not once per scene: collectActiveActors walks the entire record
    // database and pulls getActiveActors() from every scene, so it is process-wide work that does
    // not vary by the scene being emitted. Same discipline as the rigid and articulation paths.
    const ActiveActorSet activeSet =
        (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};

    for (const ScenePair& sp : scenes)
        buildVehicleWheelState(s, source, sp.first, *sp.second, scope, activeSet, wanted);
}

// ----------------------------------------------------------------------------
// Deformable bodies (sim-mesh points / velocities — ONE array group per attribute)
// ----------------------------------------------------------------------------

// One kernel launch per attribute over every body, writing a device column the groups point into -- the shape
// the joint read uses (see finalizeArrayGroup's other call site) and the one REQ-READ-DEVICE-001 AC-2
// describes.
//
// The view it gathers through is the read's OWN (GpuPointSetReadView), not the tensor binding's
// GpuVolumeDeformableBodyView: the binding's view sets PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC on every
// body it covers, and a read may not change how the solver behaves.

// The authored topology one deformable publishes: rest positions and element indices. Collected
// once per view build and held for the view's lifetime, because none of it can change without an
// object being created or retired -- which is what retires the cache entry.
//
// It is HOST data and stays host. Unlike points and velocities there is no device copy in the
// runtime to gather from: these are source-authored arrays, so the read emits kDLCPU columns and a
// consumer branches on device_type. That is REQ-READ-ATTRS-001 AC-8's rule, applied to the same
// situation it was written for -- an attribute whose source has no device copy is not a device
// column the read declined to produce.
// Int32 per SIMULATION element: a tetrahedral sim mesh names four nodes, a triangular one three. A function
// rather than a field on the struct below, since it is decided entirely by which deformable kind is read.
uint32_t simElementLanes(bool isVolume)
{
    return isVolume ? 4u : 3u;
}

struct DeformableTopology
{
    std::vector<float> restPoints; // 3 floats per node
    std::vector<int32_t> simIndices; // simElementLanes() int32 per element
    std::vector<int32_t> collIndices; // 4 int32 per tet; empty for surface bodies
};

// Which topology column is being talked about. File scope rather than local to the emit, because
// the collector below is keyed on it too.
enum class DeformableTopo
{
    eRestPoints,
    eSimIndices,
    eCollIndices
};

// Copy one array-valued attribute out of the source. Source-agnostic on purpose: this goes through
// IPhysicsSource rather than USD, so the read keeps working on any backend that can answer, and the
// buffer is released immediately rather than accumulating over reads (the interface's own note).
template <typename T>
bool copyArrayAttribute(IPhysicsSource& source,
                        ObjectKey key,
                        const char* attrName,
                        omni::physics::parse::BufferElemType expected,
                        uint32_t lanes,
                        std::vector<T>& out)
{
    using namespace omni::physics::parse;
    out.clear();
    if (!key.valid())
        return false;
    const TokenId token = source.internToken(attrName);
    const BufferHandle handle = source.getArrayAttribute(key, token, ReadTime{});
    if (!handle.valid() || handle.type != expected || handle.elemCount == 0)
    {
        source.releaseBuffer(handle);
        return false;
    }
    size_t byteCount = 0;
    const void* data = source.resolveBuffer(handle, byteCount);
    const size_t wanted = size_t(handle.elemCount) * lanes * sizeof(T);
    if (!data || byteCount < wanted)
    {
        source.releaseBuffer(handle);
        return false;
    }
    out.resize(size_t(handle.elemCount) * lanes);
    std::memcpy(out.data(), data, wanted);
    source.releaseBuffer(handle);
    return true;
}

// Gather one body's topology. A body whose source cannot answer keeps empty vectors and is simply
// omitted from those columns -- the same warn-and-omit shape REQ-READ-ATTRS-001 AC-9 uses for an
// attribute a particular object genuinely has no value for, rather than publishing zeros that look
// measured.
// ONE part, so a read pays only for what it asked for: collecting all three with the view would pull every
// authored rest-point and index array in the scene for a pure `points` read, and hold three vectors per body
// resident until the lifetime epoch moved.
void collectDeformableTopologyPart(IPhysicsSource& source,
                                   InternalDeformableBody* b,
                                   bool isVolume,
                                   DeformableTopo which,
                                   DeformableTopology& out)
{
    using namespace omni::physics::parse;
    switch (which)
    {
    case DeformableTopo::eRestPoints:
        copyArrayAttribute<float>(source, b->mSimMeshKey, "omniphysics:restShapePoints", BufferElemType::eVec3, 3,
                                  out.restPoints);
        return;

    case DeformableTopo::eSimIndices:
        if (isVolume)
        {
            copyArrayAttribute<int32_t>(source, b->mSimMeshKey, "tetVertexIndices", BufferElemType::eInt4, 4,
                                        out.simIndices);
        }
        else
        {
            // A surface body's simulation elements are the mesh's faceVertexIndices -- a FLAT int
            // array, three per triangle, not an int3 array. The parse path already refuses a surface
            // deformable whose faceVertexIndices disagree with restTriVtxIndices, so the two are
            // interchangeable here and this reads the one that is always present.
            copyArrayAttribute<int32_t>(source, b->mSimMeshKey, "faceVertexIndices", BufferElemType::eInt32, 1,
                                        out.simIndices);
        }
        return;

    case DeformableTopo::eCollIndices:
        // Only a volume body has a collision mesh distinct from its simulation mesh, which is why
        // this is the one attribute the surface type does not advertise.
        if (isVolume)
        {
            InternalVolumeDeformableBody* vol = static_cast<InternalVolumeDeformableBody*>(b);
            copyArrayAttribute<int32_t>(source, vol->mCollMeshKey, "tetVertexIndices", BufferElemType::eInt4, 4,
                                        out.collIndices);
        }
        return;
    }
}

// Per scene and per deformable kind: the read's device view plus what the emit needs about each
// body. Keyed by the object-lifetime epoch alone -- unlike the instancer cache there is no
// generation to track, because this view is not a child of any simulation view, so nothing but a
// created or retired object can invalidate it.
struct DeformableReadCacheEntry
{
    uint64_t dbEpoch = 0;
    std::unique_ptr<omni::physx::tensors::GpuPointSetReadView> view;
    std::vector<ObjectKey> keys; // one per view row, in the view's order
    std::vector<InternalDeformableBody*> bodies; // ditto: the reframe matrix comes from here
    std::vector<PxDeformableBody*> pxBodies; // ditto, for isSleeping()
    std::vector<DeformableTopology> topology; // ditto: authored rest points and element indices

    // Which topology columns `topology` actually holds. Filled on first genuine request rather than
    // with the view, so a `points`-only read never touches the source arrays -- and cleared with
    // everything else when the object-lifetime epoch moves, which is the one thing that can retire
    // the arrays behind them.
    bool haveRestPoints = false;
    bool haveSimIndices = false;
    bool haveCollIndices = false;
};
std::unordered_map<const PxScene*, DeformableReadCacheEntry> g_volumeDeformableCache;
std::unordered_map<const PxScene*, DeformableReadCacheEntry> g_surfaceDeformableCache;

// PxMat44d::front() and PointSetTransform::m share the same column-major layout (both are
// PxMat44T's own), so this is an element-wise narrow rather than a relayout. Kept as a named
// function anyway: it is the one place the two are asserted to agree, and a transpose slipped in
// here would be invisible everywhere else.
void toPointSetTransform(const ::physx::PxMat44d& m, omni::physx::tensors::PointSetTransform& out)
{
    const double* src = m.front();
    for (int i = 0; i < 16; ++i)
        out.m[i] = float(src[i]);
}

// Upload a point-set view's per-prim reframe matrices without blocking the host: stage
// `count` of them into a pinned, session-owned buffer and issue the async copy. Falls back to the
// synchronous upload when a pinned buffer cannot be acquired, so a pool/alloc failure degrades to
// correct-but-blocking rather than failing the read. Shared by the deformable and particle paths.
bool uploadReframeMatrices(ReadSession& s,
                           PxCudaContextManager* mgr,
                           PxCudaContext* cu,
                           omni::physx::tensors::GpuPointSetReadView& view,
                           const omni::physx::tensors::PointSetTransform* xf,
                           PxU32 count)
{
    const size_t bytes = size_t(count) * sizeof(omni::physx::tensors::PointSetTransform);
    size_t cap = 0;
    void* const pinned = bytes ? s.acquirePinned(mgr, cu, bytes, cap) : nullptr;
    if (pinned)
    {
        std::memcpy(pinned, xf, bytes);
        return view.setReframeMatricesAsync(
            static_cast<const omni::physx::tensors::PointSetTransform*>(pinned), count);
    }
    return view.setReframeMatrices(xf, count);
}

// Build the view's rows for one scene. A body with no device buffer or no vertices is skipped
// HERE rather than at emit, so the view's rows, `keys` and `bodies` stay index-aligned.
void collectDeformableEntries(InternalScene* sc,
                              bool isVolume,
                              std::vector<omni::physx::tensors::PointSetReadEntry>& entries,
                              DeformableReadCacheEntry& ce)
{
    // Built from empty rather than appended to, so this function's result depends on the scene
    // alone and not on what the caller happened to leave behind.
    entries.clear();
    ce.keys.clear();
    ce.bodies.clear();
    ce.pxBodies.clear();
    ce.topology.clear();

    auto add = [&](InternalDeformableBody* b, PxDeformableBody* px, const PxVec4* pos, const PxVec4* vel)
    {
        if (!b || !px || b->mNumSimMeshVertices == 0 || !pos || !vel)
            return;
        omni::physx::tensors::PointSetReadEntry e;
        e.positions = pos;
        e.velocities = vel;
        e.numPoints = b->mNumSimMeshVertices;
        entries.push_back(e);
        ce.keys.push_back(b->mSimMeshKey);
        ce.bodies.push_back(b);
        ce.pxBodies.push_back(px);
        ce.topology.emplace_back(); // left empty; filled per column, on demand
    };

    if (isVolume)
    {
        for (InternalVolumeDeformableBody* b : sc->mVolumeDeformableBodies)
        {
            PxDeformableVolume* dv = b ? b->mDeformableVolume : nullptr;
            if (!dv)
                continue;
            add(b, dv, reinterpret_cast<const PxVec4*>(dv->getSimPositionInvMassBufferD()),
                reinterpret_cast<const PxVec4*>(dv->getSimVelocityBufferD()));
        }
    }
    else
    {
        for (InternalSurfaceDeformableBody* b : sc->mSurfaceDeformableBodies)
        {
            PxDeformableSurface* ds = b ? b->mDeformableSurface : nullptr;
            if (!ds)
                continue;
            add(b, ds, reinterpret_cast<const PxVec4*>(ds->getPositionInvMassBufferD()),
                reinterpret_cast<const PxVec4*>(ds->getVelocityBufferD()));
        }
    }
}

} // namespace (reopened below)

namespace omni::physx::ovx
{
// The deformable counterpart of particleWriteTargets (ADR-0012). Declared in
// OvxPhysicsShared.h; it lives here because collectDeformableEntries does, and the write must inherit
// that function's admission rules whole rather than restate them -- a body with no sim-mesh
// vertices, or no PhysX object, is skipped identically, and the ORDER matches, which is what lets a
// caller read a column and write it straight back.
bool deformableWriteTargets(::physx::PxScene* scene, const bool isVolume, DeformableWriteTargets& out)
{
    out.keys.clear();
    out.bodies.clear();
    out.internals.clear();
    out.counts.clear();

    ActiveContext ctx;
    if (!getActiveContext("deformableWriteTargets", ctx))
        return false;

    InternalScene* sc = nullptr;
    for (const ScenePair& pair : allPhysicsScenesWithInternal())
    {
        if (pair.first == scene)
        {
            sc = pair.second;
            break;
        }
    }
    if (!sc)
        return true; // not a live physics scene: nothing to offer, not a failure

    std::vector<omni::physx::tensors::PointSetReadEntry> entries;
    DeformableReadCacheEntry scratch;
    collectDeformableEntries(sc, isVolume, entries, scratch);

    out.keys = std::move(scratch.keys);
    out.bodies = std::move(scratch.pxBodies);
    out.internals = std::move(scratch.bodies);
    out.counts.reserve(entries.size());
    for (const omni::physx::tensors::PointSetReadEntry& e : entries)
        out.counts.push_back(uint32_t(e.numPoints));
    return true;
}
} // namespace omni::physx::ovx

namespace
{

// Why a deformable gather failed, as a closed set rather than a free string, so the log suppression
// below can be per CAUSE.
enum class PointSetFailure
{
    eViewUnbuildable,
    eTransformUpload,
    eColumnAlloc,
    eColumnZero,
    eGatherLaunch,
    ePendingPatch,
    eCount
};

const char* pointSetFailureText(PointSetFailure which)
{
    switch (which)
    {
    // Worded for the SHAPE, not for either object type: both failDeformableBackend and failParticleBackend
    // emit these verbatim, and the caller's own message already names the type ("volume deformable
    // column(s)", "particle column(s)"). A particle set has no sim mesh, so nothing here may mention one.
    case PointSetFailure::eViewUnbuildable: return "the device view could not be built over the scene's point sets";
    case PointSetFailure::eTransformUpload: return "the reframe matrices could not be uploaded";
    case PointSetFailure::eColumnAlloc:     return "the destination device column could not be allocated";
    case PointSetFailure::eColumnZero:      return "the destination device column could not be zeroed, so regions no gather writes would publish uninitialized memory";
    case PointSetFailure::eGatherLaunch:    return "the device gather kernel could not be launched";
    case PointSetFailure::ePendingPatch:    return "the authored pre-upload values could not be copied into the column";
    case PointSetFailure::eCount:           break;
    }
    return "unknown";
}

// A deformable column that could not be gathered has to SAY so: ReadSession::backendFailed is what turns the
// drain's closing kOvxReadStatusEndOfIteration into kOvxReadStatusError.
//
// Suppression is per CAUSE, not per call site. `CARB_LOG_ERROR_ONCE` latches on a static at its own expansion
// point, and this helper has exactly one, so whichever cause fired first would be the only one ever printed.
// Suppression is still needed: a read runs per frame, so an unguarded line repeats for as long as the sim
// does. ERROR rather than WARN, matching the kOvxReadStatusError the caller then sees.
//
// Only backend failure comes through at all. The two structural answers on this path -- a sim with no CUDA
// context, and a scene holding no deformable of this kind -- deliberately do not, for the reason spelled out
// at ReadSession::backendFailed.
void failDeformableBackend(ReadSession& session, bool isVolume, PointSetFailure which)
{
    static bool logged[2][size_t(PointSetFailure::eCount)] = {};
    bool& seen = logged[isVolume ? 1 : 0][size_t(which)];
    if (!seen)
    {
        seen = true;
        CARB_LOG_ERROR("ovphysx read: %s deformable column(s) could not be gathered (%s); they are omitted.",
                       isVolume ? "volume" : "surface", pointSetFailureText(which));
    }
    session.backendFailed = true;
}

void buildDeformableGroups(ReadSession& s,
                           IPhysicsSource& source,
                           uint32_t type,
                           uint32_t scope,
                           const std::vector<std::string>& names)
{
    bool wantPoints = false, wantVel = false;
    bool wantRestPoints = false, wantSimIndices = false, wantCollIndices = false;
    for (const std::string& nm : names)
    {
        if (nm == omni::physx::OvxAttr::kPoints) wantPoints = true;
        else if (nm == omni::physx::OvxAttr::kVelocities) wantVel = true;
        else if (nm == omni::physx::OvxAttr::kRestPoints) wantRestPoints = true;
        else if (nm == omni::physx::OvxAttr::kSimElementIndices) wantSimIndices = true;
        else if (nm == omni::physx::OvxAttr::kCollisionElementIndices) wantCollIndices = true;
        else CARB_LOG_WARN("ovxReadAttributes: '%s' is not a deformable output attribute "
                           "for this type — skipped.", nm.c_str());
    }
    if (!wantPoints && !wantVel && !wantRestPoints && !wantSimIndices && !wantCollIndices)
        return;

    const bool isVolume = (type == omni::physx::kOvxDeformableVolume);
    if (wantCollIndices && !isVolume)
    {
        // A surface deformable has no collision mesh distinct from its simulation mesh, so this is
        // an attribute the type does not have rather than one the read failed to produce. Refused
        // by name (AC-4) instead of falling through to a neighbouring column.
        CARB_LOG_WARN("ovxReadAttributes: '%s' is a volume-deformable attribute; surface deformables "
                      "have no separate collision mesh — skipped.", omni::physx::OvxAttr::kCollisionElementIndices);
        wantCollIndices = false;
    }
    std::unordered_map<const PxScene*, DeformableReadCacheEntry>& cache =
        isVolume ? g_volumeDeformableCache : g_surfaceDeformableCache;

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();
    std::vector<PxScene*> live;

    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        InternalScene* sc = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
        PxScene* scene = sc->getScene();
        if (!scene)
            continue;
        live.push_back(scene);

        // Per scene rather than once for the read: the deformable buffers below belong to this
        // scene's context, which under multi-GPU is not the process-default manager.
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            // Not a failure: deformables are device-only by design (ADR-0008 Decision 9), so a sim with no
            // CUDA context cannot produce this column at all -- structural, not transient, and nothing a
            // caller could retry.
            CARB_LOG_WARN("ovxReadAttributes: deformable read needs a CUDA context — none available.");
            continue;
        }
        PxCudaContext* cu = ctxMgr->getCudaContext();
        PxScopedCudaLock _lock(*ctxMgr);

        // NOT needed by the gather below. The gather reads PhysX's own device sim buffers, which the
        // SOLVER writes; the copy stream carries the stepper's D2H copies into host staging buffers
        // this path never touches. So nothing here races it, and this is not what orders the read.
        //
        // Kept, but only for a read that asks for a SIMULATED column: it is a host-blocking
        // streamSynchronize, and a topology-only read (restPoints / simElementIndices /
        // collisionElementIndices) issues no device work at all.
        //
        // Keeping it here at all is belt-and-braces rather than load-bearing: the read is not the
        // only thing that clears mDeformableCopyStreamDirty. InternalScene::updateDeformableTransforms
        // and PhysXScene's own sync points clear it too, so gating this cannot strand the flag.
        if (wantPoints || wantVel)
            sc->syncDeformableCopyStream(ctxMgr);

        DeformableReadCacheEntry& ce = cache[scene];
        // Keyed on the epoch alone: the row vectors below are only safe to leave behind while the view is
        // null because every path that resets the view also clears them.
        if (ce.dbEpoch != dbEpochNow)
        {
            ce.view.reset();
            ce.keys.clear();
            ce.bodies.clear();
            ce.pxBodies.clear();
            ce.topology.clear();
            ce.haveRestPoints = ce.haveSimIndices = ce.haveCollIndices = false;
        }
        if (!ce.view)
        {
            std::vector<omni::physx::tensors::PointSetReadEntry> entries;
            collectDeformableEntries(sc, isVolume, entries, ce);
            if (!entries.empty())
                ce.view.reset(new omni::physx::tensors::GpuPointSetReadView(ctxMgr, entries));
            if (!ce.view || !ce.view->isUsable())
            {
                // Dropped rather than kept: a view that cannot serve answers every column with
                // "nothing to do", which would publish zero-filled groups under a successful read.
                //
                // Two different things land here and only one of them is a failure. A scene holding
                // no deformable of this kind produces no entries at all -- that is an ANSWER, and
                // flagging it would make every rigid-only scene report a failed read. A NON-EMPTY
                // entry set that yields an unusable view is a genuine backend failure.
                if (!entries.empty())
                    failDeformableBackend(s, isVolume, PointSetFailure::eViewUnbuildable);
                ce.view.reset();
                ce.keys.clear();
                ce.bodies.clear();
                ce.pxBodies.clear();
                ce.topology.clear();
                ce.haveRestPoints = ce.haveSimIndices = ce.haveCollIndices = false;
                continue;
            }
            ce.dbEpoch = dbEpochNow;
        }

        const PxU32 numBodies = ce.view->getCount();
        const PxU32 columnFloats = ce.view->columnFloats();
        if (numBodies == 0 || columnFloats == 0)
            continue;

        // The device half runs only when a SIMULATED column was asked for, and NOTHING inside it may
        // leave this scene's iteration. A topology-only read (restPoints / element indices) needs no
        // device buffer and no gather at all, and a read that asks for BOTH must still be served its
        // host columns when the device side fails -- so every failure below records itself, clears
        // `deviceOk` and falls through to the publish loop instead of taking a `continue`. That loop
        // tests `deviceOk` per device column, so the host ones are still emitted.
        //
        // Dropping host columns the caller explicitly asked for because an unrelated device step failed is
        // the bug this shape exists to prevent.
        CUdeviceptr columnBase = 0;
        uint32_t pointsColumn = 0, velocityColumn = 0;
        int devOrd = -1;
        bool deviceOk = (wantPoints || wantVel);
        const uint32_t numColumns = (wantPoints ? 1u : 0u) + (wantVel ? 1u : 0u);
        const size_t totalFloats = size_t(columnFloats) * numColumns;
        if (deviceOk && wantPoints)
        {
            // Resolved every read and never cached: moving the sim-mesh prim changes this matrix
            // while creating and retiring nothing, so the epoch guarding the view above cannot see
            // it. Same reason the instancer path re-resolves its world inverses -- and one entry per
            // BODY, not per vertex, so redoing it unconditionally is affordable.
            std::vector<omni::physx::tensors::PointSetTransform> xf(numBodies);
            for (PxU32 i = 0; i < numBodies; ++i)
                toPointSetTransform(ce.bodies[i]->mWorldToSimMesh, xf[i]);
            if (!uploadReframeMatrices(s, ctxMgr, cu, *ce.view, xf.data(), numBodies))
            {
                failDeformableBackend(s, isVolume, PointSetFailure::eTransformUpload);
                deviceOk = false;
            }
        }
        if (deviceOk)
        {
            // Pool-or-allocate; acquireColumn also registers the buffer for release.
            columnBase = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, totalFloats * sizeof(float)));
            if (!columnBase)
            {
                failDeformableBackend(s, isVolume, PointSetFailure::eColumnAlloc);
                deviceOk = false;
            }
        }
        if (deviceOk)
        {
            // Zeroed defensively, not because a contract says so. The column is compact and every float in
            // it is written by the gather, so on the success path this clears nothing that survives; what it
            // covers is a body whose `src` buffer PhysX has not published yet, which the kernel skips per
            // record -- handing a consumer whatever the allocator last held is the worse default.
            //
            // So the result is checked: precisely because the regions this protects are the ones no
            // gather writes, a failed fill is invisible downstream. The gather can still succeed and the
            // skipped records then read as valid output.
            if (cu->memsetD32Async(columnBase, 0u, totalFloats, 0) != 0)
            {
                failDeformableBackend(s, isVolume, PointSetFailure::eColumnZero);
                deviceOk = false;
            }
        }

        if (deviceOk)
        {

            // Every gather first, THEN every group -- the same ordering the instancer path uses, and
            // for the same reason: a group published before a later column fails would be a partial
            // read reporting success.
            uint32_t columnIdx = 0;
            pointsColumn = wantPoints ? columnIdx++ : 0u;
            velocityColumn = wantVel ? columnIdx++ : 0u;
            auto columnPtr = [&](uint32_t idx)
            { return reinterpret_cast<void*>(columnBase + size_t(idx) * columnFloats * sizeof(float)); };

            using Column = omni::physx::tensors::GpuPointSetReadView::Column;
            if (wantPoints)
                deviceOk = ce.view->getColumnOvStage(Column::ePoints, columnPtr(pointsColumn));
            if (deviceOk && wantVel)
                deviceOk = ce.view->getColumnOvStage(Column::eVelocities, columnPtr(velocityColumn));
            if (!deviceOk)
                failDeformableBackend(s, isVolume, PointSetFailure::eGatherLaunch);

            // NO host sync here, deliberately. ovxReadAttributes records one completion event per
            // CUDA context after buildGroups returns and publishes it on every group that names that
            // context through g.ctxMgr -- which these do. The consumer waits on data.cuda_sync;
            // blocking the host here would do that waiting on its behalf, which is exactly the
            // handoff ADR-0008 and REQ-READ-DEVICE-001 AC-1 replaced.
            devOrd = static_cast<int>(ctxMgr->getDevice());
        }


        // ONE array group per attribute, carrying one tensor per prim -- not one group per prim.
        //
        // The participating prims are resolved ONCE, here, and both columns reuse them: the same key vector
        // and, through `sharedList`, the same ovx prim list. Rebuilding either per column would repeat a walk
        // over every body for data that cannot differ between columns.
        //
        // This is also where ACTIVE scope is applied. The gather above covered every body, because
        // the view is built over the whole scene and cached and a cached view must not depend on the
        // active set; a sleeping body simply gets no tensor and no key.
        std::vector<ObjectKey> keys;
        std::vector<PxU32> rows;
        keys.reserve(numBodies);
        rows.reserve(numBodies);
        for (PxU32 i = 0; i < numBodies; ++i)
        {
            if (scope == omni::physx::kOvxActive && ce.pxBodies[i]->isSleeping())
                continue;
            keys.push_back(ce.keys[i]);
            rows.push_back(i);
        }
        if (keys.empty())
            continue;

        ovx_primpath_list_t sharedList = OVX_INVALID_PRIMPATH_LIST;
        auto emit = [&](const char* name, uint32_t column)
        {
            GroupStore g;
            g.isArray = true;
            g.deviceOrdinal = devOrd;
            g.ctxMgr = ctxMgr;
            g.shapes.reserve(rows.size());
            g.tensors.reserve(rows.size());
            const CUdeviceptr base = columnBase + size_t(column) * columnFloats * sizeof(float);
            for (size_t r = 0; r < rows.size(); ++r)
            {
                const PxU32 i = rows[r];
                g.shapes.push_back(static_cast<int64_t>(ce.view->pointCountFor(i)));
                DLTensor t{};
                t.data = reinterpret_cast<void*>(base + size_t(ce.view->setOffsetFloats(i)) * sizeof(float));
                t.device = DLDevice{ kDLCUDA, devOrd };
                t.ndim = 1;
                t.dtype = DLDataType{ kDLFloat, 32, 3 };
                t.shape = nullptr; // patched to &g.shapes[r] at fetch, where the address is stable
                t.strides = nullptr;
                t.byte_offset = 0;
                g.tensors.push_back(t);
            }
            if (!finalizeArrayGroup(s, source, std::move(g), name, keys, nullptr, &sharedList))
            {
                // Publishing is where the prim list is built, so a failure here drops the whole
                // column. The drain would still reach END_OF_ITERATION, handing the caller a short
                // result that reads as a complete one -- so fail the read instead of omitting it.
                CARB_LOG_ERROR("ovphysx read: could not publish the %s column", name);
                s.backendFailed = true;
            }
        };

        // Topology columns: rest points and element indices. Host, not device, and one array group
        // per attribute exactly like the simulated pair -- the only differences are where the values
        // come from (source-authored arrays, cached with the view) and that a body which has none
        // simply contributes no tensor. Both kinds are published by the one request-ordered loop
        // below, which is what lets a host column sit BETWEEN two device columns when that is what
        // the caller asked for.
        //
        // The per-read cost is a memcpy out of that cache into group storage, not a re-read of the
        // source. The copy is what keeps REQ-READ-DEVICE-001 AC-6's ownership rule: a group must stay
        // valid until IT is released, and the cache behind it is dropped the moment the object
        // lifetime epoch moves.
        // Filled here, once, and then cached for as long as the view is. The flag is per column, so
        // a read that asks only for `simElementIndices` never pulls rest points.
        //
        // `have` is set even when a body answered with NOTHING, so a source that could not serve the
        // array is cached as an empty column until the lifetime epoch moves. That is deliberate:
        // these are authored inputs, so "no rest points on this prim" is a property of the asset and
        // not a transient condition to retry every frame -- and the body is omitted from the group
        // either way (AC-9), so retrying would change nothing but the cost.
        auto ensureTopology = [&](bool& have, DeformableTopo which)
        {
            if (have)
                return;
            for (size_t i = 0; i < ce.topology.size(); ++i)
                collectDeformableTopologyPart(source, ce.bodies[i], isVolume, which, ce.topology[i]);
            have = true;
        };

        using Topo = DeformableTopo;
        auto emitTopology = [&](const char* name, uint32_t lanes, Topo which, bool& have)
        {
            ensureTopology(have, which);
            const bool isInt = (which != Topo::eRestPoints);
            auto countOf = [&](const DeformableTopology& t) -> size_t
            {
                switch (which)
                {
                case Topo::eRestPoints:  return t.restPoints.size();
                case Topo::eSimIndices:  return t.simIndices.size();
                default:                 return t.collIndices.size();
                }
            };
            auto intData = [&](const DeformableTopology& t) -> const int32_t*
            { return which == Topo::eSimIndices ? t.simIndices.data() : t.collIndices.data(); };

            GroupStore g;
            g.isArray = true;
            g.dtype = isInt ? DLDataType{ kDLInt, 32, 1 } : DLDataType{ kDLFloat, 32, 1 };

            // How many values this body contributes, or 0 for "no tensor". Consulted by both loops
            // below so the sizing pass and the fill pass cannot disagree about who is in the group.
            auto contributes = [&](const DeformableTopology& t) -> size_t
            {
                const size_t count = countOf(t);
                if (count == 0)
                    return 0; // AC-9: no source for this body, so no tensor -- never a row of zeros
                if (count % lanes != 0)
                {
                    // No shape describes such an array: `count / lanes` publishes fewer values than
                    // it holds and says nothing about the remainder. Omitted by name instead, the
                    // way an attribute the type does not have is.
                    //
                    // Defensive on three of the four columns, where the source is typed and the
                    // length is a multiple of the lane count by construction -- rest points come
                    // back eVec3, both tet index arrays eInt4. The one that can genuinely reach it
                    // is a SURFACE body's simElementIndices: they are read from `faceVertexIndices`
                    // as a FLAT int array and grouped in threes, so anything but a triangle mesh
                    // lands here. The parse path is expected to have refused that already, which
                    // makes this the assertion that it did.
                    CARB_LOG_WARN_ONCE("ovxReadAttributes: '%s' holds %zu values, which is not a whole number of "
                                       "%u-value elements — that body is omitted from the group.",
                                       name, count, lanes);
                    return 0;
                }
                return count;
            };

            std::vector<ObjectKey> present;
            present.reserve(rows.size());
            size_t total = 0;
            for (const PxU32 i : rows)
            {
                const size_t count = contributes(ce.topology[i]);
                if (count == 0)
                    continue;
                present.push_back(ce.keys[i]);
                total += count;
            }
            if (present.empty())
                return;

            if (isInt)
                g.ints.resize(total);
            else
                g.floats.resize(total);

            size_t cursor = 0;
            g.shapes.reserve(present.size());
            g.tensors.reserve(present.size());
            for (const PxU32 i : rows)
            {
                const DeformableTopology& t = ce.topology[i];
                const size_t count = contributes(t);
                if (count == 0)
                    continue;
                void* dst = nullptr;
                if (isInt)
                {
                    std::memcpy(g.ints.data() + cursor, intData(t), count * sizeof(int32_t));
                    dst = g.ints.data() + cursor;
                }
                else
                {
                    std::memcpy(g.floats.data() + cursor, t.restPoints.data(), count * sizeof(float));
                    dst = g.floats.data() + cursor;
                }
                g.shapes.push_back(static_cast<int64_t>(count / lanes));
                DLTensor dl{};
                dl.data = dst;
                dl.device = DLDevice{ kDLCPU, 0 };
                dl.ndim = 1;
                dl.dtype = g.dtype;
                dl.dtype.lanes = static_cast<uint16_t>(lanes);
                dl.shape = nullptr; // patched at fetch, where the address is stable
                dl.strides = nullptr;
                dl.byte_offset = 0;
                g.tensors.push_back(dl);
                cursor += count;
            }
            // NOT sharing `sharedList`: the prim set here can be a SUBSET of the simulated columns'
            // (a body with no authored rest points contributes to `points` but not to `restPoints`),
            // and a shared list must describe exactly the prims its group carries.
            if (!finalizeArrayGroup(s, source, std::move(g), name, present))
            {
                CARB_LOG_ERROR("ovphysx read: could not publish the %s column", name);
                s.backendFailed = true;
            }
        };

        // Emission walks the REQUEST rather than a fixed sequence, so the caller's order and any
        // repeated name survive into the result (AC-19). The wantX flags decide what gets GATHERED --
        // asking twice must not gather twice -- and this loop decides what gets PUBLISHED, which is
        // the half a caller can observe. The joint path has always indexed its name list positionally
        // (buildJointState); one API must not answer two ways depending on the object type queried.
        //
        // AC-9: a column the gather could not produce is OMITTED, never published zero-filled. The
        // status the caller gets already says the read came back short, and `deviceOk` is what keeps
        // the two answers consistent with each other.
        for (const std::string& nm : names)
        {
            // No trailing else: an unrecognised name was warned about once before the scene loop, and
            // warning again here would repeat it per scene.
            if (nm == omni::physx::OvxAttr::kPoints)
            {
                if (deviceOk)
                    emit(nm.c_str(), pointsColumn);
            }
            else if (nm == omni::physx::OvxAttr::kVelocities)
            {
                if (deviceOk)
                    emit(nm.c_str(), velocityColumn);
            }
            else if (nm == omni::physx::OvxAttr::kRestPoints)
            {
                emitTopology(nm.c_str(), 3, Topo::eRestPoints, ce.haveRestPoints);
            }
            else if (nm == omni::physx::OvxAttr::kSimElementIndices)
            {
                emitTopology(nm.c_str(), simElementLanes(isVolume), Topo::eSimIndices, ce.haveSimIndices);
            }
            else if (nm == omni::physx::OvxAttr::kCollisionElementIndices && wantCollIndices)
            {
                // Gated on the flag, not re-tested here: `wantCollIndices` carries the surface-
                // deformable suppression decided (and warned about) once above.
                emitTopology(nm.c_str(), 4, Topo::eCollIndices, ce.haveCollIndices);
            }
        }
    }

    purgeDeadSceneEntries(cache, live);
}

// ----------------------------------------------------------------------------
// Deformable materials (one fixed row per material prim)
// ----------------------------------------------------------------------------

// Read straight off `PxDeformableMaterial`, not through the tensor binding's `DeformableMaterialView`: the
// database record already carries everything needed -- `mPtr` IS the PhysX material and `mKey` IS the prim it
// was authored on -- so a view would add a build, an entry list and a lifetime to manage for data reachable in
// one dereference. Constructing a binding view is also not free of side effects, and a read may not have any.
//
// Host columns, and not as a deferral. Material properties are simulation INPUTS -- the caller
// authors them and PhysX never writes them back -- so there is no device copy for a device column to
// come from. That is REQ-TENSOR-CPU-ONLY-001 as REQ-READ-ATTRS-001 AC-8 applies it to the rigid-body
// properties, and the same reasoning lands here unchanged.

// Which getter serves an attribute, and whether the attribute exists on a volume material at all.
// Split by KIND rather than padded: PhysX puts thickness and the two bending terms on
// PxDeformableSurfaceMaterial only, and the tensor binding reports 0.0 for them on a volume material.
// This read omits the row instead -- REQ-READ-ATTRS-001 AC-9's rule that an object with no source
// for an attribute gets no value rather than a zero that reads as measured. A caller asking for
// `bendingStiffness` therefore gets a group covering the surface materials only, which is the same
// per-attribute prim subset the topology columns already produce.
struct MaterialAttributeRow
{
    const char* name;
    bool surfaceOnly;
    float (*read)(const PxDeformableMaterial&);
};

const MaterialAttributeRow kMaterialAttributes[] = {
    { omni::physx::OvxAttr::kDeformableDynamicFriction, false,
      [](const PxDeformableMaterial& m) { return m.getDynamicFriction(); } },
    { omni::physx::OvxAttr::kDeformableYoungsModulus, false,
      [](const PxDeformableMaterial& m) { return m.getYoungsModulus(); } },
    // PhysX spells this getPoissons(); the attribute keeps the schema's name (poissonsRatio) because
    // that is what a caller authored and what the tensor API reports.
    { omni::physx::OvxAttr::kDeformablePoissonsRatio, false,
      [](const PxDeformableMaterial& m) { return m.getPoissons(); } },
    { omni::physx::OvxAttr::kDeformableElasticityDamping, false,
      [](const PxDeformableMaterial& m) { return m.getElasticityDamping(); } },
    // The three surface-only rows downcast. That is checked rather than trusted where the row set is
    // built below -- `isSurface` comes from PxBase::is<>(), PhysX's own concrete type, not from the
    // ovruntime record kind that names it. A record mislabelled either way would otherwise be
    // undefined behaviour in these three lines, and it is the kind of mislabelling nothing else on
    // this path would notice: every column is a bare f32.
    { omni::physx::OvxAttr::kDeformableBendingStiffness, true,
      [](const PxDeformableMaterial& m)
      { return static_cast<const PxDeformableSurfaceMaterial&>(m).getBendingStiffness(); } },
    { omni::physx::OvxAttr::kDeformableThickness, true,
      [](const PxDeformableMaterial& m)
      { return static_cast<const PxDeformableSurfaceMaterial&>(m).getThickness(); } },
    { omni::physx::OvxAttr::kDeformableBendingDamping, true,
      [](const PxDeformableMaterial& m)
      { return static_cast<const PxDeformableSurfaceMaterial&>(m).getBendingDamping(); } },
};

const MaterialAttributeRow* materialRowFor(const std::string& name)
{
    for (const MaterialAttributeRow& row : kMaterialAttributes)
        if (name == row.name)
            return &row;
    return nullptr;
}

void buildDeformableMaterialGroups(ReadSession& s, IPhysicsSource& source, const std::vector<std::string>& names)
{
    const size_t firstMaterialGroup = s.groups.size();

    // Exhaustive by NAME, never by position or width: every column here is a single f32, so nothing about a
    // value identifies which attribute it is. An unhandled name warns and yields nothing rather than falling
    // through to a neighbour (REQ-READ-ATTRS-001 AC-4).
    std::vector<const MaterialAttributeRow*> wanted;
    for (const std::string& nm : names)
    {
        if (const MaterialAttributeRow* row = materialRowFor(nm))
            wanted.push_back(row);
        else
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not a deformable material attribute — skipped.", nm.c_str());
    }
    if (wanted.empty())
        return;

    // One walk for every requested attribute, not one per attribute.
    struct MaterialRow
    {
        ObjectKey key;
        const PxDeformableMaterial* material;
        bool isSurface;
    };
    std::vector<MaterialRow> materials;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        const bool recordSaysSurface = rec.mType == omni::physx::ePTDeformableSurfaceMaterial;
        if ((rec.mType != omni::physx::ePTDeformableVolumeMaterial && !recordSaysSurface) || !rec.mPtr)
            continue;
        const PxDeformableMaterial* material = static_cast<const PxDeformableMaterial*>(rec.mPtr);

        // PhysX decides which kind this is, not the record. `isSurface` is what the three
        // surface-only getters downcast on, so deriving it from the record type would make a
        // mislabelled record undefined behaviour; asking PxBase::is<>() makes it an omitted row
        // instead, which is what AC-9 already says to do with a property an object does not have.
        const bool surface = material->is<PxDeformableSurfaceMaterial>();
        if (surface != recordSaysSurface)
        {
            CARB_LOG_WARN("ovxReadAttributes: deformable material is recorded as %s but PhysX reports "
                          "%s — its surface-only properties are omitted.",
                          recordSaysSurface ? "surface" : "volume", surface ? "surface" : "volume");
        }
        materials.push_back({ rec.mKey, material, surface });
    }
    if (materials.empty())
        return;

    // One prim list for the whole read where the prim sets coincide. They do not always: a
    // surface-only attribute covers a subset, and a shared list must describe exactly the prims its
    // group carries, so the sharing is keyed on which subset an attribute needs.
    ovx_primpath_list_t allList = OVX_INVALID_PRIMPATH_LIST;
    ovx_primpath_list_t surfaceList = OVX_INVALID_PRIMPATH_LIST;

    for (const MaterialAttributeRow* row : wanted)
    {
        GroupStore g;
        std::vector<ObjectKey> keys;
        keys.reserve(materials.size());
        g.floats.reserve(materials.size());
        for (const MaterialRow& m : materials)
        {
            if (row->surfaceOnly && !m.isSurface)
                continue; // AC-9: a volume material has no such property, so it contributes no row
            keys.push_back(m.key);
            g.floats.push_back(row->read(*m.material));
        }
        if (keys.empty())
            continue;

        // Keyed on WHICH subset the attribute takes, not on how many rows it happened to produce.
        // A count is identity by proxy: it is sound only while `surface` is the one proper subset,
        // and it says nothing about that -- add a volume-only property later and two different
        // subsets of equal size would share one list describing the wrong prims, silently, because
        // a shared list must describe exactly the prims its group carries. Costs one extra interned
        // list in the all-surface scene, where the two subsets coincide.
        if (!finalizeGroup(s, source, std::move(g), row->name, keys, static_cast<int64_t>(keys.size()), 1,
                           /*canonicalHandles*/ nullptr, row->surfaceOnly ? &surfaceList : &allList))
        {
            rollbackGroups(s, firstMaterialGroup);
            s.backendFailed = true;
            return;
        }
    }
}

// ----------------------------------------------------------------------------
// Particle sets (points / velocities — an array group per particle-set prim)
// ----------------------------------------------------------------------------

// Per scene: the read's own device view over this scene's particle sets, plus what the emit needs
// about each one. Keyed by the object-lifetime epoch alone, exactly like the deformable cache -- a
// particle set being created or retired is the only thing that can invalidate the view, and nothing
// else about a set changes its rows.
struct ParticleReadCacheEntry
{
    uint64_t dbEpoch = 0;
    std::unique_ptr<omni::physx::tensors::GpuPointSetReadView> view;
    std::vector<ObjectKey> keys; // one per view row, in the view's order
    std::vector<InternalParticleSet*> sets; // ditto: the reframe matrix and the pending flags
    // What the view was BUILT from, kept so a read can tell whether it still holds. Not derivable
    // from the view: it uploads these to the device records and does not retain them. See the
    // re-validation in buildParticleGroups for why the object-lifetime epoch cannot cover this.
    std::vector<const ::physx::PxVec4*> builtPositions;
    std::vector<::physx::PxU32> builtCounts;
};
std::unordered_map<const PxScene*, ParticleReadCacheEntry> g_particleReadCache;

// Every particle set in `sc` that can actually serve, in a stable order.
//
// A set still pending its first upload IS included, with its device pointers like any other. The
// device buffer has not been written yet, so the gather fills its slice with whatever was there --
// and `patchPendingSets` below overwrites that slice with the authored host values afterwards.
//
// Dropping a pending set instead would break the contract: a pre-step read is REQUIRED to return the authored
// positions, exactly as rigid-body and deformable pre-step reads do (TestParticles asserts it by name --
// NvBugs 6481119).
//
// One walk over db.getRecords() PER SCENE rather than one per read bucketed by scene, because it runs only on
// a cache MISS and a particle set has no per-scene list to iterate the way `sc->mVolumeDeformableBodies`
// gives the deformable path one. If InternalScene ever grows a particle-set list, this should use it.
void collectParticleEntries(InternalScene* sc,
                            std::vector<omni::physx::tensors::PointSetReadEntry>& entries,
                            ParticleReadCacheEntry& ce)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    // Built from empty rather than appended to, so this function's result depends on the scene alone
    // and not on what the caller happened to leave behind -- the same contract collectDeformableEntries
    // states. Every push below adds to five parallel vectors, so one missed clear would desynchronise
    // them against each other, not merely lengthen them.
    entries.clear();
    ce.keys.clear();
    ce.sets.clear();
    ce.builtPositions.clear();
    ce.builtCounts.clear();

    // Sized off the record count on first use, not grown per push: a scene made mostly of particle
    // sets would otherwise reallocate its way up every time the epoch moves.
    bool reserved = false;
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTParticleSet || !rec.mInternalPtr)
            continue;
        InternalParticleSet* ps = reinterpret_cast<InternalParticleSet*>(rec.mInternalPtr);

        // NOT filtered on mEnabled here. It is runtime-mutable and disabling a set retires no
        // database record, so the object-lifetime epoch guarding this cache cannot see it -- a set
        // disabled after the cache was built would keep being published. The live check is at the
        // emit instead. `mNumParticles` and `mParticleBuffer` are structural for the set's lifetime
        // and are checked here, where they decide whether a row can exist at all.
        if (ps->mNumParticles == 0 || !ps->mParticleBuffer)
            continue;

        // Scene identity through the parent system, the same route the per-set CUDA context is
        // resolved by: a particle set has no scene of its own.
        PxPBDParticleSystem* pxPs = ps->mParentParticleSystem ? ps->mParentParticleSystem->mPS : nullptr;
        if (!pxPs || pxPs->getScene() != sc->getScene())
            continue;

        omni::physx::tensors::PointSetReadEntry e;
        e.positions = reinterpret_cast<const PxVec4*>(ps->mParticleBuffer->getPositionInvMasses());
        e.velocities = reinterpret_cast<const PxVec4*>(ps->mParticleBuffer->getVelocities());
        e.numPoints = ps->mNumParticles;
        if (!e.positions || !e.velocities)
            continue;

        if (!reserved)
        {
            const size_t bound = db.getRecords().size();
            entries.reserve(bound);
            ce.keys.reserve(bound);
            ce.sets.reserve(bound);
            reserved = true;
        }
        // All five pushed together, below every `continue` above: they are parallel arrays indexed
        // by view row, so a push that some rows skip would desynchronise them against each other
        // rather than merely shorten one.
        entries.push_back(e);
        ce.keys.push_back(ps->mKey);
        ce.sets.push_back(ps);
        ce.builtPositions.push_back(e.positions);
        ce.builtCounts.push_back(e.numPoints);
    }
}

} // namespace (reopened below)

namespace omni::physx::ovx
{
// The particle counterpart of instancerWriteTargets (ADR-0012). It hands back the SETS
// rather than a view, because the write's destination is each set's staging pair -- see
// ParticleWriteTargets for why that is the mechanism and not a shortcut.
//
// collectParticleEntries is called rather than reimplemented so the write inherits the read's
// admission rules whole: a set with no particles, no buffer, or belonging to another scene is
// skipped identically, and the ORDER matches, which is what lets a caller read a column and write it
// straight back.
//
// The scratch cache entry is deliberate. The enumeration fills one, and the write wants only three
// of its five parallel arrays; reusing the read's cached entry instead would mean this call could
// invalidate a read cache as a side effect.
bool particleWriteTargets(::physx::PxScene* scene, const uint32_t /*scope*/, ParticleWriteTargets& out)
{
    out.keys.clear();
    out.sets.clear();
    out.counts.clear();

    ActiveContext ctx;
    if (!getActiveContext("particleWriteTargets", ctx))
        return false;

    InternalScene* sc = nullptr;
    for (const ScenePair& pair : allPhysicsScenesWithInternal())
    {
        if (pair.first == scene)
        {
            sc = pair.second;
            break;
        }
    }
    if (!sc)
        return true; // not a live physics scene: no sets to offer, not a failure

    std::vector<omni::physx::tensors::PointSetReadEntry> entries;
    ParticleReadCacheEntry scratch;
    collectParticleEntries(sc, entries, scratch);

    out.keys = std::move(scratch.keys);
    out.sets = std::move(scratch.sets);
    out.counts.reserve(scratch.builtCounts.size());
    for (const PxU32 n : scratch.builtCounts)
        out.counts.push_back(uint32_t(n));
    return true;
}
} // namespace omni::physx::ovx

namespace
{

// A particle column that could not be gathered has to SAY so, for the reason spelled out at
// ReadSession::backendFailed: without it the read reports success and then runs out of groups.
// Suppression is per cause, because CARB_LOG_ERROR_ONCE latches at its single expansion point and
// would otherwise print only whichever cause fired first for the life of the process.
void failParticleBackend(ReadSession& session, PointSetFailure which)
{
    static bool logged[size_t(PointSetFailure::eCount)] = {};
    bool& seen = logged[size_t(which)];
    if (!seen)
    {
        seen = true;
        CARB_LOG_ERROR("ovphysx read: particle column(s) could not be gathered (%s); they are omitted.",
                       pointSetFailureText(which));
    }
    session.backendFailed = true;
}

void buildParticleGroups(ReadSession& s,
                         IPhysicsSource& source,
                         AttachedStage& as,
                         const std::vector<std::string>& names)
{
    bool wantPoints = false, wantVel = false;
    for (const std::string& nm : names)
    {
        if (nm == omni::physx::OvxAttr::kPoints || nm == omni::physx::OvxAttr::kPositions) wantPoints = true;
        else if (nm == omni::physx::OvxAttr::kVelocities) wantVel = true;
        // `positions` is an accepted legacy alias for `points` that attrNamesForType deliberately does
        // not advertise (AC-19). `points` is the one public name, so a caller who asks for everything
        // the type offers cannot get the same quantity back twice. Accepting more than is advertised
        // is safe in the way the reverse is not, and emission below echoes the caller's spelling, so
        // asking for `positions` yields a group NAMED `positions` -- never the substitution AC-4
        // forbids.
        else CARB_LOG_WARN("ovxReadAttributes: '%s' is not a particle output attribute "
                           "(only points / positions / velocities) — skipped.", nm.c_str());
    }
    if (!wantPoints && !wantVel)
        return;

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const uint64_t dbEpochNow = omni::physx::internal::recordLifetimeEpoch();
    std::vector<PxScene*> live;

    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        InternalScene* sc = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
        PxScene* scene = sc->getScene();
        if (!scene)
            continue;
        live.push_back(scene);

        // Per scene rather than once for the read: under /physics/sceneMultiGPUMode scenes are
        // handed round-robin context managers, so the buffers below are not all on one device.
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr || !ctxMgr->getCudaContext())
        {
            // Not a failure: particles are device-only by design (ADR-0008 Decision 9), so a sim
            // with no CUDA context cannot produce this column at all -- structural, not transient.
            CARB_LOG_WARN("ovxReadAttributes: particle read needs a CUDA context — none available.");
            continue;
        }
        PxCudaContext* cu = ctxMgr->getCudaContext();
        PxScopedCudaLock _lock(*ctxMgr);

        ParticleReadCacheEntry& ce = g_particleReadCache[scene];
        if (ce.view && ce.dbEpoch != dbEpochNow)
        {
            ce.view.reset();
            ce.keys.clear();
            ce.sets.clear();
            ce.builtPositions.clear();
            ce.builtCounts.clear();
        }

        // The epoch is NOT sufficient for this cache, and the difference is a dangling device
        // pointer rather than a stale filter.
        //
        // `InternalParticleSet::resize()` runs on an ordinary USD edit -- authoring `points` or
        // `velocities` on the set prim reaches it through updateParticlePositions -- and it mutates
        // both of the things a cached row is built from, in two ways that the object-lifetime epoch
        // cannot see because neither creates nor retires a record:
        //
        //   * a resize within mMaxParticles just assigns mNumParticles, so a cached `numPoints`
        //     over-reports (publishing retired particles as measured state) or under-reports
        //     (silently truncating the row);
        //   * a resize PAST mMaxParticles releases mParticleBuffer, builds a new one, and patches
        //     the database record in place (`objectRecord->mPtr = mParticleBuffer`) rather than
        //     retiring it. The record upload happens once at view construction and never again, so
        //     the gather would dereference the RELEASED buffer.
        //
        // So the cached rows are re-validated against the live set here, at O(numSets) -- the same
        // order as the mEnabled filter below, and for the same underlying reason that filter exists:
        // a runtime-mutable property the epoch is blind to. This one has to rebuild rather than
        // filter, because what changed is what the view was BUILT from.
        if (ce.view)
        {
            bool stale = ce.sets.size() != ce.builtPositions.size() || ce.sets.size() != ce.builtCounts.size();
            for (size_t i = 0; !stale && i < ce.sets.size(); ++i)
            {
                // mParticleBuffer only. Non-nullness is guaranteed at build by
                // collectParticleEntries, the same invariant the staging and emit loops rely on --
                // and this runs after the epoch check, so ps is no less resolved here than there.
                const InternalParticleSet* ps = ce.sets[i];
                if (!ps->mParticleBuffer)
                {
                    stale = true;
                    break;
                }
                // Both halves matter: the COUNT decides the row's length, and the BUFFER is what the
                // uploaded record points at.
                stale = ps->mNumParticles != ce.builtCounts[i] ||
                        reinterpret_cast<const PxVec4*>(ps->mParticleBuffer->getPositionInvMasses()) !=
                            ce.builtPositions[i];
            }
            if (stale)
            {
                ce.view.reset();
                ce.keys.clear();
                ce.sets.clear();
                ce.builtPositions.clear();
                ce.builtCounts.clear();
            }
        }
        if (!ce.view)
        {
            std::vector<omni::physx::tensors::PointSetReadEntry> entries;
            collectParticleEntries(sc, entries, ce);
            if (!entries.empty())
                ce.view.reset(new omni::physx::tensors::GpuPointSetReadView(ctxMgr, entries));
            if (!ce.view || !ce.view->isUsable())
            {
                // A scene holding no servable particle set is an ANSWER, not a failure -- and note
                // that "no servable set" includes every set still pending its first upload, which is
                // a transient state a caller should not see reported as a backend error.
                if (!entries.empty())
                    failParticleBackend(s, PointSetFailure::eViewUnbuildable);
                ce.view.reset();
                ce.keys.clear();
                ce.sets.clear();
                ce.builtPositions.clear();
                ce.builtCounts.clear();
                continue;
            }
            ce.dbEpoch = dbEpochNow;
        }

        const PxU32 numSets = ce.view->getCount();
        const PxU32 columnFloats = ce.view->columnFloats();
        if (numSets == 0 || columnFloats == 0)
            continue;

        // Every failure below records itself and falls through to the emit guard rather than taking
        // a `continue`, for the reason the deformable path does: an exit that skips the rest of the
        // scene is how a short read gets reported as a complete one.
        CUdeviceptr columnBase = 0;
        uint32_t pointsColumn = 0, velocityColumn = 0;
        int devOrd = -1;
        bool deviceOk = true;
        const uint32_t numColumns = (wantPoints ? 1u : 0u) + (wantVel ? 1u : 0u);
        const size_t totalFloats = size_t(columnFloats) * numColumns;

        // Declared out here because the pending-set patch below reframes through it too.
        std::vector<omni::physx::tensors::PointSetTransform> xf;
        if (wantPoints)
        {
            // World -> prim-local, from the RUNTIME's own copy rather than by resolving the prim's
            // transform out of USD. InternalParticleSet::mWorldToLocal is maintained by the parse
            // layer -- written at creation (UsdInterfaceParticle.cpp) and rewritten on an xform
            // change (UsdInterface.cpp) -- which is the same guarantee InternalDeformableBody's
            // mWorldToSimMesh carries, and the deformable read has always used it.
            //
            // Both are PxMat44d, so this is the element-wise narrow toPointSetTransform does, not a
            // relayout: the reframe moved off pxr's row-vector GfMatrix4f onto PhysX's column-vector
            // PxMat44T, and a GfMatrix4f wrapper here would have transposed the matrix silently.
            //
            // Resolved every read and never cached, for the reason the deformable path is not: moving the
            // prim changes this while creating and retiring nothing, so the object lifetime epoch guarding
            // the view cannot see it. One entry per SET, not per particle.
            xf.resize(numSets);
            for (PxU32 i = 0; i < numSets; ++i)
                toPointSetTransform(ce.sets[i]->mWorldToLocal, xf[i]);
            if (!uploadReframeMatrices(s, ctxMgr, cu, *ce.view, xf.data(), numSets))
            {
                failParticleBackend(s, PointSetFailure::eTransformUpload);
                deviceOk = false;
            }
        }

        if (deviceOk)
        {
            // Pool-or-allocate; acquireColumn also registers the buffer for release.
            columnBase = static_cast<CUdeviceptr>(s.acquireColumn(ctxMgr, cu, totalFloats * sizeof(float)));
            if (!columnBase)
            {
                failParticleBackend(s, PointSetFailure::eColumnAlloc);
                deviceOk = false;
            }
        }

        if (deviceOk)
        {
            // Checked, for the same reason as the deformable path: the pending pre-upload slices are
            // patched onto a zeroed allocation, so a set the patch does not reach keeps whatever the
            // zero left. If the zero never happened that is allocator contents published as points.
            if (cu->memsetD32Async(columnBase, 0u, totalFloats, 0) != 0)
            {
                failParticleBackend(s, PointSetFailure::eColumnZero);
                deviceOk = false;
            }
        }

        if (deviceOk)
        {

            uint32_t columnIdx = 0;
            pointsColumn = wantPoints ? columnIdx++ : 0u;
            velocityColumn = wantVel ? columnIdx++ : 0u;
            auto columnPtr = [&](uint32_t idx)
            { return reinterpret_cast<void*>(columnBase + size_t(idx) * columnFloats * sizeof(float)); };

            using Column = omni::physx::tensors::GpuPointSetReadView::Column;
            if (wantPoints)
                deviceOk = ce.view->getColumnOvStage(Column::ePoints, columnPtr(pointsColumn));
            if (deviceOk && wantVel)
                deviceOk = ce.view->getColumnOvStage(Column::eVelocities, columnPtr(velocityColumn));
            if (!deviceOk)
                failParticleBackend(s, PointSetFailure::eGatherLaunch);

            // A set still pending its first upload has never had its device buffer written, so the
            // gather just filled its slice with whatever was there. Its authored values live in the
            // pinned HOST buffer until the first step seeds the device one, and a pre-step read has
            // to return those -- so they are reframed here and copied over that slice.
            //
            // Reframed through pointSetTransformPoint, the same function the kernel calls, rather
            // than through GfMatrix4f::Transform: the two must not disagree about a value depending
            // on which side of the first step it was read.
            //
            // Every pending set's floats go into a SINGLE staging buffer, each copy issued from a stable
            // offset inside it, so no source is reused before its copy runs.
            struct PendingCopy
            {
                CUdeviceptr dst;
                size_t offset; // where this set's values start in the staging buffer, in floats
                size_t floats;
            };
            std::vector<PendingCopy> pendingCopies;

            // Gated on a set actually being pending, because the ceiling below is 3 floats per particle in
            // the SCENE and the steady state uses none of it. The probe is per SET, not per particle.
            const auto anyPending = [&]()
            {
                for (PxU32 i = 0; i < numSets; ++i)
                {
                    const InternalParticleSet* ps = ce.sets[i];
                    if (wantPoints && (ps->mUploadDirtyFlags & ParticleBufferFlags::ePOSITIONS) && ps->mPositions)
                        return true;
                    if (wantVel && (ps->mUploadDirtyFlags & ParticleBufferFlags::eVELOCITIES) && ps->mVelocities)
                        return true;
                }
                return false;
            };

            // One staging buffer for the whole patch, sized to the ceiling (every set pending, on every
            // requested attribute) BEFORE anything is staged, so no set's slice can be moved by a later
            // set's growth. PINNED and session-owned when it can be: the async copies then
            // read it past the read's release-time completion wait, with NO per-read streamSynchronize. If a
            // pinned buffer cannot be acquired it falls back to a pageable local vector, which a terminal
            // host block below must order before the vector dies -- correct, just blocking.
            const size_t ceilingFloats = size_t(columnFloats) * numColumns;
            float* pendingBase = nullptr;
            size_t pendingCap = 0;  // floats pendingBase can hold
            size_t pendingUsed = 0; // floats staged so far
            bool pendingPinned = false;
            std::vector<float> pendingFallback;
            if (deviceOk && ceilingFloats && anyPending())
            {
                size_t capBytes = 0;
                void* const pinned = s.acquirePinned(ctxMgr, cu, ceilingFloats * sizeof(float), capBytes);
                if (pinned)
                {
                    pendingBase = static_cast<float*>(pinned);
                    pendingCap = capBytes / sizeof(float);
                    pendingPinned = true;
                }
                else
                {
                    pendingFallback.resize(ceilingFloats);
                    pendingBase = pendingFallback.data();
                    pendingCap = ceilingFloats;
                }
            }

            auto stagePendingSets = [&](uint32_t column, bool arePoints)
            {
                const uint32_t flag = arePoints ? ParticleBufferFlags::ePOSITIONS :
                                                  ParticleBufferFlags::eVELOCITIES;
                for (PxU32 i = 0; i < numSets; ++i)
                {
                    // Never null: collectParticleEntries skips a record with no mInternalPtr before
                    // pushing, so a row exists only for a resolved set. The emit loop below checks
                    // mEnabled and mParticleBuffer, which is a different question -- those are
                    // runtime-mutable, and this one is fixed at build.
                    InternalParticleSet* ps = ce.sets[i];
                    const PxVec4* src = arePoints ? ps->mPositions : ps->mVelocities;
                    if (!(ps->mUploadDirtyFlags & flag) || !src)
                        continue;
                    const uint32_t n = ce.view->pointCountFor(i);
                    if (n == 0)
                        continue;
                    // The ceiling guarantees room; this bounds the writes regardless, so a miscount can
                    // never run past the staging buffer (and covers a failed staging allocation).
                    if (!pendingBase || pendingUsed + size_t(n) * 3 > pendingCap)
                        continue;

                    const size_t at = pendingUsed;
                    pendingUsed += size_t(n) * 3;
                    for (uint32_t v = 0; v < n; ++v)
                    {
                        const float src3[3] = { src[v].x, src[v].y, src[v].z };
                        if (arePoints)
                        {
                            omni::physx::tensors::pointSetTransformPoint(xf[i], src3, &pendingBase[at + v * 3]);
                        }
                        else
                        {
                            // Velocities are WORLD on this path too -- the device column does not
                            // reframe them, so neither does this.
                            pendingBase[at + v * 3 + 0] = src3[0];
                            pendingBase[at + v * 3 + 1] = src3[1];
                            pendingBase[at + v * 3 + 2] = src3[2];
                        }
                    }
                    const CUdeviceptr dst = columnBase +
                        (size_t(column) * columnFloats + ce.view->setOffsetFloats(i)) * sizeof(float);
                    pendingCopies.push_back({ dst, at, size_t(n) * 3 });
                }
            };

            if (deviceOk && wantPoints)
                stagePendingSets(pointsColumn, /*arePoints*/ true);
            if (deviceOk && wantVel)
                stagePendingSets(velocityColumn, /*arePoints*/ false);

            // A failed enqueue leaves that set's slice holding whatever the gather wrote, published as
            // though it were the authored pre-upload values, so the result is checked.
            bool patchOk = true;
            for (const PendingCopy& copy : pendingCopies)
            {
                if (cu->memcpyHtoDAsync(copy.dst, pendingBase + copy.offset, copy.floats * sizeof(float), 0) != 0)
                {
                    patchOk = false;
                    break;
                }
            }
            // No host block on the pinned path: the staging buffer is session-owned and
            // freed only past this read's completion event, which already orders the copies. The pageable
            // fallback still must block -- its vector dies at the end of this scope, so returning with a
            // copy in flight would let CUDA read freed memory. Synchronize even if an enqueue failed:
            // earlier copies in the batch are already reading it.
            if (!pendingPinned && !pendingCopies.empty())
            {
                if (cu->streamSynchronize(0) != 0)
                    patchOk = false;
            }
            if (!patchOk)
            {
                failParticleBackend(s, PointSetFailure::ePendingPatch);
                return;
            }

            // NO host sync on the ordinary path: the read publishes one completion event per CUDA context
            // and the consumer waits on data.cuda_sync.
            devOrd = static_cast<int>(ctxMgr->getDevice());
        }

        // ONE array group per attribute, carrying one tensor per prim -- not one group per set.
        //
        // The participating rows are resolved per READ, not taken from the cache, and that is a correctness
        // requirement: `mEnabled` flips at runtime (InternalParticleSet::setEnabled /
        // InternalPbdParticleSystem::setEnabled) and disabling a set creates and retires NO database record,
        // so `recordLifetimeEpoch` does not move and cached membership would never be rebuilt.
        //
        // What it does NOT have to guard is a dangling pointer: `setEnabled(false)` calls
        // PxPBDParticleSystem::removeParticleBuffer, which DETACHES the buffer but does not release
        // it -- the release is `SAFE_RELEASE(mParticleBuffer)` in ~InternalParticleSet, and that
        // destruction does retire a record and does move the epoch. So `PointSetReadEntry::positions`
        // stays valid across a disable; the gather may read a detached buffer's stale values for
        // that row, and this filter is what stops those values being published.
        //
        // The same shape the deformable path uses for `isSleeping()`, and for the same reason.
        std::vector<ObjectKey> keys;
        std::vector<PxU32> rows;
        keys.reserve(numSets);
        rows.reserve(numSets);
        for (PxU32 i = 0; i < numSets; ++i)
        {
            const InternalParticleSet* ps = ce.sets[i];
            // mEnabled and mParticleBuffer only -- both flip at runtime without retiring the record.
            // Non-nullness is guaranteed at build by collectParticleEntries, so checking it here
            // would imply a case that cannot happen.
            if (!ps->mEnabled || !ps->mParticleBuffer)
                continue;
            keys.push_back(ce.keys[i]);
            rows.push_back(i);
        }
        if (keys.empty())
            continue; // every set in this scene is disabled -- an answer, not a failure

        ovx_primpath_list_t sharedList = OVX_INVALID_PRIMPATH_LIST;
        auto emit = [&](const char* name, uint32_t column)
        {
            GroupStore g;
            g.isArray = true;
            g.deviceOrdinal = devOrd;
            g.ctxMgr = ctxMgr;
            g.shapes.reserve(rows.size());
            g.tensors.reserve(rows.size());
            const CUdeviceptr base = columnBase + size_t(column) * columnFloats * sizeof(float);
            for (size_t r = 0; r < rows.size(); ++r)
            {
                const PxU32 i = rows[r];
                g.shapes.push_back(static_cast<int64_t>(ce.view->pointCountFor(i)));
                DLTensor t{};
                t.data = reinterpret_cast<void*>(base + size_t(ce.view->setOffsetFloats(i)) * sizeof(float));
                t.device = DLDevice{ kDLCUDA, devOrd };
                t.ndim = 1;
                t.dtype = DLDataType{ kDLFloat, 32, 3 };
                t.shape = nullptr; // patched to &g.shapes[r] at fetch, where the address is stable
                t.strides = nullptr;
                t.byte_offset = 0;
                g.tensors.push_back(t);
            }
            if (!finalizeArrayGroup(s, source, std::move(g), name, keys, nullptr, &sharedList))
            {
                // Publishing is where the prim list is built, so a failure here drops the whole
                // column. The drain would still reach END_OF_ITERATION, handing the caller a short
                // result that reads as a complete one -- so fail the read instead of omitting it.
                CARB_LOG_ERROR("ovphysx read: could not publish the %s column", name);
                s.backendFailed = true;
            }
        };

        // Request-ordered, multiplicity-preserving, and named with the caller's own spelling -- see
        // buildDeformableGroups for the reasoning (AC-19). Echoing the spelling is what lets the
        // `positions` alias be accepted without ever being the substitution AC-4 forbids.
        //
        // AC-9: a column the gather could not produce is OMITTED, never published zero-filled.
        for (const std::string& nm : names)
        {
            if (nm == omni::physx::OvxAttr::kPoints || nm == omni::physx::OvxAttr::kPositions)
            {
                if (deviceOk)
                    emit(nm.c_str(), pointsColumn);
            }
            else if (nm == omni::physx::OvxAttr::kVelocities)
            {
                if (deviceOk)
                    emit(nm.c_str(), velocityColumn);
            }
        }
    }

    purgeDeadSceneEntries(g_particleReadCache, live);
}

// ----------------------------------------------------------------------------
// Per-type dispatch
// ----------------------------------------------------------------------------

void buildGroups(ReadSession& s,
                 IPhysicsSource& source,
                 AttachedStage& as,
                 uint32_t type,
                 uint32_t scope,
                 const std::vector<std::string>& names)
{
    using namespace omni::physx;
    switch (type)
    {
    case kOvxRigidBody:
        buildRigidBodyGroups(s, source, as, scope, names);
        break;
    case kOvxArticulationLink:
        buildArticulationLinkGroups(s, source, scope, names);
        break;
    case kOvxArticulationJoint:
        buildJointStateGroups(s, source, names);
        break;
    case kOvxArticulation:
        buildArticulationRootGroups(s, source, scope, names);
        break;
    case kOvxVehicleWheel:
        buildVehicleWheelGroups(s, source, scope, names);
        break;
    case kOvxDeformableVolume:
    case kOvxDeformableSurface:
        buildDeformableGroups(s, source, type, scope, names);
        break;
    case kOvxParticleSet:
        buildParticleGroups(s, source, as, names);
        break;
    case kOvxDeformableMaterial:
        buildDeformableMaterialGroups(s, source, names);
        break;
    case kOvxFixedTendon:
        buildTendonGroups(s, source, /*fixed=*/true, names);
        break;
    case kOvxSpatialTendon:
        buildTendonGroups(s, source, /*fixed=*/false, names);
        break;
    default:
        CARB_LOG_WARN("ovxReadAttributes: object type %u not yet implemented.", type);
        break;
    }
}

// Output attribute names available for a query type (for ovxFetchQueryResult).
std::vector<std::string> attrNamesForType(uint32_t type)
{
    using namespace omni::physx;
    switch (type)
    {
    // Derived from the tables that serve these types, so discovery cannot advertise a different set
    // from the one a read actually produces.
    case kOvxRigidBody:
    case kOvxArticulationLink:
    {
        std::vector<std::string> out;
        out.reserve(std::size(kRigidAttributes) + std::size(kArticulationLinkOnlyAttributes));
        for (const RigidAttributeRow& row : kRigidAttributes)
            out.emplace_back(row.token);
        // A link is a rigid body PLUS what only the articulation view serves, which is why the two
        // types share this branch. A plain rigid body has no inbound joint, so it must not advertise
        // a column the read would refuse to produce.
        if (type == kOvxArticulationLink)
            for (const char* token : kArticulationLinkOnlyAttributes)
                out.emplace_back(token);
        return out;
    }
    case kOvxArticulationJoint:
    {
        std::vector<std::string> out;
        out.reserve(std::size(kJointAttributes) + std::size(kJointPropertyAttributes));
        for (const JointAttributeRow& row : kJointAttributes)
            out.emplace_back(row.token);
        // BOTH joint tables, because buildJointState serves both: the state/control columns above and the
        // host-resident authored properties.
        for (const JointPropertyAttributeRow& row : kJointPropertyAttributes)
            out.emplace_back(row.token);
        return out;
    }
    case kOvxArticulation:
    {
        std::vector<std::string> out;
        out.reserve(std::size(kArticulationRootAttributes));
        for (const ArticulationRootAttributeRow& row : kArticulationRootAttributes)
            out.emplace_back(row.token);
        return out;
    }
    case kOvxVehicleWheel:
        // From the same list the read filters on, so the two cannot disagree.
        return std::vector<std::string>(std::begin(kVehicleWheelAttrs), std::end(kVehicleWheelAttrs));
    case kOvxDeformableVolume:
        // Split from the surface and particle cases rather than sharing one line: a volume body is
        // the only kind with a COLLISION mesh distinct from its simulation mesh, so it is the only
        // one that advertises collisionElementIndices. Discovery has to say so -- AC-2 is about the
        // advertised set matching the served one, and the read refuses an attribute this type has
        // no source for.
        return { OvxAttr::kPoints, OvxAttr::kVelocities, OvxAttr::kRestPoints, OvxAttr::kSimElementIndices,
                 OvxAttr::kCollisionElementIndices };
    case kOvxDeformableSurface:
        return { OvxAttr::kPoints, OvxAttr::kVelocities, OvxAttr::kRestPoints, OvxAttr::kSimElementIndices };
    case kOvxParticleSet:
        // `points` is the single advertised name for this quantity. The read also accepts `positions`
        // as a legacy alias and this list deliberately omits it, so discovery never offers one
        // quantity under two names -- see AC-19 and the prescan in buildParticleGroups.
        return { OvxAttr::kPoints, OvxAttr::kVelocities };
    case kOvxDeformableMaterial:
    {
        // Derived from the table the READ walks, not restated. A hand-written list is a second place to
        // remember: add a row and it is served but never advertised, so a caller asking what the type offers
        // never learns to request it -- the defect AC-2 exists to make impossible.
        std::vector<std::string> out;
        out.reserve(std::size(kMaterialAttributes));
        for (const MaterialAttributeRow& row : kMaterialAttributes)
            out.emplace_back(row.name);
        return out;
    }
    case kOvxFixedTendon:
    case kOvxSpatialTendon:
    {
        // Same table for both kinds, filtered the same way the read filters it, so the two cannot
        // advertise a property the other would then refuse to produce.
        const bool fixed = (type == kOvxFixedTendon);
        std::vector<std::string> out;
        out.reserve(std::size(kTendonAttributes));
        for (const TendonAttributeRow& row : kTendonAttributes)
            if (fixed || !row.fixedOnly)
                out.emplace_back(row.token);
        return out;
    }
    default:
        return {};
    }
}

// Distinct matched prims for a query type+scope (for ovxFetchQueryResult: count +
// a seed key to reach the dictionary).
std::vector<ObjectKey> collectMatchedKeys(AttachedStage& as, uint32_t type, uint32_t scope)
{
    using namespace omni::physx;
    std::vector<ObjectKey> keys;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (type == kOvxRigidBody)
    {
        std::vector<PxRigidBody*> bodies;
        std::vector<ObjectKey> standaloneKeys;
        // Once, not once per scene. The set is the union across every scene, so it never varied
        // with the loop it used to sit inside -- it was rebuilt whole for each iteration.
        const ActiveActorSet activeSet = (scope == kOvxActive) ? collectActiveActors() : ActiveActorSet{};
        for (PxScene* scene : allPhysicsScenes()) // all scenes' standalone bodies (query counts every match)
            enumerateStandaloneRigid(scope, scene, activeSet, bodies, standaloneKeys);
        for (const ObjectKey& k : standaloneKeys)
            keys.push_back(k);
        // Keys only: this is the query, and it wants nothing but which instancers matched -- no pose
        // readback and no per-instance reframe.
        std::vector<InstancerAccum> instancers;
        enumerateInstancers(as, scope, activeSet, instancers, nullptr, InstancerCollect::eKeysOnly);
        for (const InstancerAccum& a : instancers)
            keys.push_back(a.instancerKey);
    }
    else if (type == kOvxArticulationLink)
    {
        // Same predicate the read uses, and indexed rather than range-based because the active set
        // is keyed by record index.
        const ActiveActorSet activeSet = (scope == kOvxActive) ? collectActiveActors() : ActiveActorSet{};
        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        for (size_t ri = 0; ri < records.size(); ++ri)
        {
            const InternalDatabase::Record& rec = records[ri];
            if (rec.mType != ePTLink || !rec.mPtr)
                continue;
            PxArticulationLink* const link = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>();
            if (!link || !linkInScope(*link, ri, scope, activeSet))
                continue;
            keys.push_back(rec.mKey);
        }
    }
    else if (type == kOvxArticulation)
    {
        const std::vector<PxScene*> scenes = allPhysicsScenes();
        purgeDeadSceneEntries(g_articulationReadCache.byScene, scenes);
        if (!refreshArticulationStructuralCache(scenes))
            return keys;

        std::unordered_map<const PxScene*, ActiveActorSet> activeByScene;
        if (scope == kOvxActive)
            activeByScene.reserve(scenes.size());
        for (const ArticulationStructuralRef& ref : g_articulationReadCache.rootsInDatabaseOrder)
        {
            const std::unordered_map<const PxScene*, ArticulationReadCacheEntry>::const_iterator found =
                g_articulationReadCache.byScene.find(ref.scene);
            if (found == g_articulationReadCache.byScene.end() || ref.index >= found->second.rootKeys.size())
                continue;
            if (found->second.duplicateRootPointers)
            {
                CARB_LOG_WARN_ONCE(
                    "ovphysx query: duplicate ePTArticulation pointers make the "
                    "whole-articulation result ambiguous; that scene is omitted.");
                continue;
            }

            ActiveActorSet inactive;
            const ActiveActorSet* active = &inactive;
            if (scope == kOvxActive)
            {
                std::unordered_map<const PxScene*, ActiveActorSet>::iterator state = activeByScene.find(ref.scene);
                if (state == activeByScene.end())
                    state = activeByScene.emplace(ref.scene, collectActiveActorsForScene(ref.scene)).first;
                active = &state->second;
            }
            if (articulationRootInScope(found->second, ref.index, scope, *active, ref.scene))
                keys.push_back(found->second.rootKeys[ref.index]);
        }
    }
    else if (type == kOvxArticulationJoint)
    {
        const std::vector<PxScene*> scenes = allPhysicsScenes();
        purgeDeadSceneEntries(g_articulationReadCache.byScene, scenes);
        if (!refreshArticulationStructuralCache(scenes))
            return keys;
        for (const ArticulationStructuralRef& ref : g_articulationReadCache.jointsInDatabaseOrder)
        {
            const std::unordered_map<const PxScene*, ArticulationReadCacheEntry>::const_iterator found =
                g_articulationReadCache.byScene.find(ref.scene);
            if (found != g_articulationReadCache.byScene.end() && ref.index < found->second.joints.size())
                keys.push_back(found->second.joints[ref.index].key);
        }
    }
    else if (type == kOvxVehicleWheel)
    {
        // Through the read's own row rule rather than a record filter of its own: which wheels of a
        // vehicle become rows is what forEachVehicleWheelRow decides, and discovery has to count
        // exactly the rows a read would emit.
        const ActiveActorSet activeSet =
            (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};
        for (const ScenePair& sp : allPhysicsScenesWithInternal())
        {
            for (InternalVehicle* veh : sp.second->mVehicles)
            {
                PxRigidDynamic* actor = veh ? veh->getRigidDynamicActor() : nullptr;
                if (!actor || !vehicleActorInScope(*actor, scope, activeSet))
                    continue;
                forEachVehicleWheelRow(*veh, [&](uint32_t, const ObjectKey& key) { keys.push_back(key); });
            }
        }
    }
    else if (type == kOvxDeformableVolume || type == kOvxDeformableSurface)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
        {
            if (rec.mType != ePTScene || !rec.mInternalPtr)
                continue;
            InternalScene* sc = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
            if (type == kOvxDeformableVolume)
                for (InternalVolumeDeformableBody* b : sc->mVolumeDeformableBodies)
                    keys.push_back(b->mSimMeshKey);
            else
                for (InternalSurfaceDeformableBody* b : sc->mSurfaceDeformableBodies)
                    keys.push_back(b->mSimMeshKey);
        }
    }
    else if (type == kOvxParticleSet)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
            if (rec.mType == ePTParticleSet && rec.mInternalPtr)
                keys.push_back(reinterpret_cast<InternalParticleSet*>(rec.mInternalPtr)->mKey);
    }
    else if (type == kOvxDeformableMaterial)
    {
        // Both material kinds under one type, matching the read: IDeformableMaterialView is one
        // view over both, and the three surface-only properties are distinguished by which prims
        // carry a value rather than by a second object type.
        for (const InternalDatabase::Record& rec : db.getRecords())
            if ((rec.mType == ePTDeformableVolumeMaterial || rec.mType == ePTDeformableSurfaceMaterial) && rec.mPtr)
                keys.push_back(rec.mKey);
    }
    else if (type == kOvxFixedTendon || type == kOvxSpatialTendon)
    {
        // Through the read's own enumeration rather than a record filter of its own: the roots-only
        // rule and the "must be listed by its articulation" rule are what decide which records
        // become rows, and discovery has to count exactly the rows a read would emit.
        const bool fixed = (type == kOvxFixedTendon);
        for (PxScene* scene : allPhysicsScenes())
        {
            std::vector<TendonRec> tendons;
            std::vector<PxArticulationReducedCoordinate*> artis;
            enumerateTendons(scene, fixed, tendons, artis);
            for (const TendonRec& tr : tendons)
                keys.push_back(tr.key);
        }
    }
    return keys;
}

} // namespace

namespace omni::physx
{

OvxOutputQueryHandle ovxQuery(uint32_t type, uint32_t scope)
{
    ActiveContext ctx;
    if (!getActiveContext("ovxQuery", ctx))
        return 0;

    std::lock_guard<std::mutex> lock(g_mutex);
    const uint64_t h = g_nextQuery++;
    QueryState qs;
    qs.type = type;
    qs.scope = scope;
    qs.attach = ctx.stage->getAttachHandle();
    qs.dict = sourceDictionary(*ctx.source);
    g_queries[h] = std::move(qs);
    return h;
}

ovx_path_dictionary_t* ovxQueryDictionary(OvxOutputQueryHandle query)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    if (it == g_queries.end())
        return nullptr;
    // Handing this out across a reattach is the whole bug in miniature: the caller would intern and
    // resolve paths through the previous attach's dictionary.
    if (!attachStillOwns(it->second.attach, "ovxQueryDictionary"))
        return nullptr;
    return it->second.dict;
}

bool ovxFetchQueryResult(OvxOutputQueryHandle query, ovstage_query_result_t* outResult)
{
    if (!outResult)
        return false;
    *outResult = ovstage_query_result_t{};

    ActiveContext ctx;
    if (!getActiveContext("ovxFetchQueryResult", ctx))
        return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    if (it == g_queries.end())
    {
        CARB_LOG_ERROR("ovxFetchQueryResult: invalid query handle %llu.", (unsigned long long)query);
        return false;
    }
    QueryState& q = it->second;
    // Before any use of q.dict, and before enumerating: the enumeration below walks the CURRENT
    // attach, so without this a stale query answers about attach B while its tokens belong to A.
    if (!attachStillOwns(q.attach, "ovxFetchQueryResult"))
        return false;

    const std::vector<ObjectKey> matched = collectMatchedKeys(*ctx.stage, q.type, q.scope);
    outResult->total_prim_count = matched.size();

    // Intern the type's attribute names into the shared source dictionary (tokens
    // are dict-lifetime, valid until ovxReleaseQuery).
    const std::vector<std::string> names = attrNamesForType(q.type);
    q.resultAttrs.clear();
    if (!names.empty() && q.dict)
        for (const std::string& n : names)
            q.resultAttrs.push_back(internToken(q.dict, n));
    outResult->attributes = q.resultAttrs.empty() ? nullptr : q.resultAttrs.data();
    outResult->attribute_count = q.resultAttrs.size();
    return true;
}

OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query, const ovx_string_or_token_t* attrs, size_t attrCount)
{
    if (attrCount && !attrs)
    {
        CARB_LOG_ERROR("ovxReadAttributes: attrs must be non-null when attrCount (%zu) > 0.", attrCount);
        return 0;
    }
    ActiveContext ctx;
    if (!getActiveContext("ovxReadAttributes", ctx))
        return 0;

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    if (it == g_queries.end())
    {
        CARB_LOG_ERROR("ovxReadAttributes: invalid query handle %llu.", (unsigned long long)query);
        return 0;
    }
    // The token resolution below goes through q.dict, and the gather below that enumerates the
    // CURRENT attach. Both are wrong for a query minted against a previous one.
    if (!attachStillOwns(it->second.attach, "ovxReadAttributes"))
        return 0;
    const QueryState q = it->second;

    // Resolve attribute names (string-or-token → string). Discovery
    // (ovxFetchQueryResult) hands back interned tokens, so a caller may feed those
    // tokens straight back here; a token is resolved to its canonical string
    // through the query's shared source dictionary. The string form remains a
    // convenience for callers that did not go through discovery.
    std::vector<std::string> names;
    names.reserve(attrCount);
    for (size_t i = 0; i < attrCount; ++i)
    {
        if (attrs[i].token != OVX_INVALID_TOKEN)
        {
            ovx_string_t resolved{ nullptr, 0 };
            if (q.dict &&
                ovx_path_dictionary_token_to_string(q.dict, attrs[i].token, &resolved) == OVX_OK &&
                resolved.ptr && resolved.length)
            {
                names.emplace_back(resolved.ptr, resolved.length);
            }
            else
            {
                CARB_LOG_WARN("ovxReadAttributes: attribute token %llu could not be resolved against the "
                              "query dictionary — skipped.",
                              (unsigned long long)attrs[i].token);
            }
        }
        else if (attrs[i].string.ptr)
        {
            names.emplace_back(attrs[i].string.ptr, attrs[i].string.length);
        }
    }

    // Reclaim any output-column pool whose context died since the last read, before this read
    // allocates against the live ones.
    flushStaleColumnPools();

    ReadSession s;
    buildGroups(s, *ctx.source, *ctx.stage, q.type, q.scope, names);

    // Every device column was produced by stream-ordered work on its own context's null stream and
    // nothing blocked the host for it. Record one event per context covering that context's work;
    // the consumer waits on it through data.cuda_sync instead of the read waiting on their behalf
    // (ADR-0008). An event only orders work in the context that recorded it, so a read spanning
    // several devices needs one each.
    for (DeviceContextAllocs& alloc : s.deviceAllocs)
    {
        if (!alloc.ctxMgr || !alloc.ctxMgr->getCudaContext())
            continue;
        PxScopedCudaLock _lock(*alloc.ctxMgr);
        PxCudaContext* cu = alloc.ctxMgr->getCudaContext();
        CUevent evt = nullptr;
        if (cu->eventCreate(&evt, CU_EVENT_DISABLE_TIMING) == CUDA_SUCCESS && evt)
        {
            if (cu->eventRecord(evt, 0) == CUDA_SUCCESS)
            {
                alloc.event = reinterpret_cast<uintptr_t>(evt);
            }
            else
            {
                cu->eventDestroy(evt);
            }
        }
        if (!alloc.event)
        {
            // Could not signal completion asynchronously; fall back to blocking so the columns
            // are complete when handed out rather than emitting a sync token the consumer
            // cannot honour.
            //
            // The fallback's own result is the last thing standing between a consumer and an
            // incomplete tensor: with no event AND no successful drain there is nothing ordering
            // the gather before the caller reads the column, so the read has to fail rather than
            // publish groups whose completion nobody can establish.
            if (cu->streamSynchronize(0) != CUDA_SUCCESS)
            {
                CARB_LOG_ERROR("ovxReadAttributes: could not record a completion event for a device "
                               "context, and the blocking fallback failed; this context's columns are "
                               "suppressed and its buffers will be quarantined rather than handed out "
                               "or freed with their completion unknown.");
                // backendFailed alone was not enough. It surfaces only at END of iteration, so the
                // groups were still handed out first -- and with no event, data.cuda_sync stayed
                // {0, 0}, which tells the consumer the column needs no wait at all. The flag below
                // is what suppresses those groups and what stops release from freeing under them.
                alloc.completionUnknown = true;
                s.backendFailed = true;
            }
        }
    }
    s.attach = ctx.stage->getAttachHandle();
    const uint64_t h = g_nextRead++;
    g_reads[h] = std::move(s);
    return h;
}

// Whether this group was gathered in a context that could neither record a completion event nor
// drain. Host columns are never affected: they are complete before the group is built.
static bool completionUnknownFor(const ReadSession& s, const GroupStore& g)
{
    if (g.deviceOrdinal < 0 || !g.ctxMgr)
        return false;
    for (const DeviceContextAllocs& alloc : s.deviceAllocs)
        if (alloc.ctxMgr == g.ctxMgr)
            return alloc.completionUnknown;
    return false;
}

OvxReadStatus ovxFetchReadNext(OvxReadHandle read, ovstage_read_group_t* outGroup)
{
    if (!outGroup)
        return kOvxReadStatusError;
    *outGroup = ovstage_read_group_t{};

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_reads.find(read);
    if (it == g_reads.end())
    {
        CARB_LOG_ERROR("ovxFetchReadNext: invalid read handle %llu.", (unsigned long long)read);
        return kOvxReadStatusError;
    }
    ReadSession& s = it->second;
    // Groups already handed out stay valid -- their tensors are session-owned memory. What must not
    // happen is handing out MORE of them once the attach they describe is gone: prims.list refers to
    // the previous attach's dictionary, so the caller would resolve paths through freed storage.
    if (!attachStillOwns(s.attach, "ovxFetchReadNext"))
        return kOvxReadStatusError;
    if (s.cursor >= s.groups.size())
    {
        // Everything that succeeded has been handed over. Whether that was the WHOLE answer is the difference
        // between these two statuses: a backend build or gather that failed omitted its columns, and
        // end-of-iteration would tell the caller it had seen everything. Groups already drained stay valid
        // either way -- this reports that the set was short, it does not retract what was delivered.
        if (s.backendFailed)
        {
            CARB_LOG_ERROR("ovxFetchReadNext: read %llu is incomplete -- a backend read failed and "
                           "its columns were omitted (see the preceding warning for which).",
                           (unsigned long long)read);
            return kOvxReadStatusError;
        }
        return kOvxReadStatusEndOfIteration; // all groups consumed (not an error)
    }

    // Skip any column gathered in a context whose completion could not be established. Handing one
    // out is the failure this guards: without an event data.cuda_sync is {0, 0}, which tells the
    // consumer the column is already complete and needs no wait -- so it would read a buffer a
    // gather may still be filling, and see plausible values rather than an error. The read still
    // ends in kOvxReadStatusError via backendFailed; this decides that the caller never touches the
    // affected memory in the meantime.
    while (s.cursor < s.groups.size() && completionUnknownFor(s, s.groups[s.cursor]))
        ++s.cursor;
    if (s.cursor >= s.groups.size())
    {
        CARB_LOG_ERROR("ovxFetchReadNext: read %llu is incomplete -- a device context could not "
                       "confirm its gathers completed and its columns were suppressed.",
                       (unsigned long long)read);
        return kOvxReadStatusError;
    }

    GroupStore& g = s.groups[s.cursor];
    // Stable addresses inside the stored group, patched here because the vector may have grown
    // since the tensors were built.
    for (size_t ti = 0; ti < g.tensors.size(); ++ti)
        g.tensors[ti].shape = &g.shapes[ti];

    outGroup->read_group_id = (ovstage_read_group_id_t)(s.cursor + 1);
    outGroup->attribute = g.attribute;
    outGroup->ordinal = 0;
    outGroup->is_delete = false;
    outGroup->is_array = g.isArray;
    outGroup->semantic = g.semantic;
    outGroup->prims = ovstage_prim_group_t{ g.list, 0, g.primCount, nullptr };
    outGroup->data = ovstage_data_t{};
    outGroup->data.tensors = g.tensors.data();
    outGroup->data.tensor_count = static_cast<uint32_t>(g.tensors.size());
    // Device columns carry the completion event of the context they were gathered in; the consumer
    // must wait on it before reading. Event only, no stream: ovstage treats a non-zero `stream` as a
    // host-side drain of everything queued on it, which is a heavier barrier than this handoff
    // needs. Host columns are already complete and keep {0, 0}.
    if (g.deviceOrdinal >= 0 && g.ctxMgr)
    {
        for (const DeviceContextAllocs& alloc : s.deviceAllocs)
        {
            if (alloc.ctxMgr == g.ctxMgr && alloc.event)
            {
                outGroup->data.cuda_sync = ovstage_cuda_sync_t{ 0, alloc.event };
                break;
            }
        }
    }
    if (!g.indexMap.empty())
    {
        outGroup->data.count = (uint32_t)g.indexMap.size();
        outGroup->data.index_map = g.indexMap.data();
    }
    outGroup->meta = ovstage_attribute_meta_t{};

    ++s.cursor;
    return kOvxReadStatusOk;
}

// Drop this group's stage-derived handles. Split out from ovxReleaseRead because the two have
// different owners: a prim list belongs to the SOURCE's path dictionary, which the stage owner
// destroys, while the device columns belong to this session and outlive the read whenever a
// consumer is still borrowing one. Releasing the session is therefore deferred until the last
// borrow drops -- potentially after the stage is gone -- so the dictionary-dependent half has to
// be freed here instead, at a point where the caller still guarantees a live stage. Idempotent:
// the list handle is cleared so ovxReleaseRead does not destroy it a second time.
void ovxReleaseGroup(OvxReadHandle read, const ovstage_read_group_t* group)
{
    if (!group || group->prims.list == OVX_INVALID_PRIMPATH_LIST)
        return;
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_reads.find(read);
    if (it == g_reads.end())
        return;
    ReadSession& s = it->second;
    if (!s.dict)
        return;
    // Stale: the dictionary this session's lists live in died with its attach, and so did the lists.
    // Forget the claim rather than destroy through a freed dictionary -- there is nothing left to
    // free, and dereferencing s.dict here is the use-after-free this guard exists for.
    if (!attachStillOwns(s.attach, "ovxReleaseGroup"))
    {
        s.listRefs.clear();
        for (GroupStore& g : s.groups)
            g.list = OVX_INVALID_PRIMPATH_LIST;
        s.dict = nullptr;
        return;
    }
    // Groups covering one prim set share a list, so this releases THIS group's claim and destroys
    // the list only once nothing else holds it. Destroying on the first release would leave the
    // remaining groups pointing at freed memory.
    //
    // The group is identified by its id, not by matching the list value: with a shared list every group
    // matches, so a repeated release of one group would retire a DIFFERENT group's claim and bring the list
    // down while that group is still held. By id a repeated release is a no-op.
    if (group->read_group_id == 0 || static_cast<size_t>(group->read_group_id) > s.groups.size())
        return;
    GroupStore& g = s.groups[static_cast<size_t>(group->read_group_id) - 1];
    if (g.list == OVX_INVALID_PRIMPATH_LIST)
        return; // already released

    // Id resolves the slot; the list value proves the caller handed us a group FROM THIS READ. Ids
    // are per-read ordinals, so group 3 of read A and group 3 of read B are both "3" -- passing A's
    // group with B's handle would otherwise retire B's third group's claim and could destroy a list
    // B is still handing out. The public sidecar normally resolves ids inside the target read, so
    // this is defence for the internal contract rather than a reachable public bug.
    if (g.list != static_cast<ovx_primpath_list_t>(group->prims.list))
    {
        CARB_LOG_ERROR_ONCE("ovxReleaseGroup: group id %llu does not belong to this read; ignoring the "
                            "release rather than retiring another group's prim list.",
                            static_cast<unsigned long long>(group->read_group_id));
        return;
    }

    const ovx_primpath_list_t released = g.list;
    g.list = OVX_INVALID_PRIMPATH_LIST;
    const std::unordered_map<ovx_primpath_list_t, uint32_t>::iterator ref = s.listRefs.find(released);
    if (ref == s.listRefs.end())
        return; // untracked: nothing to destroy, and nothing to double-free
    if (--ref->second > 0)
        return; // another group still holds it
    s.listRefs.erase(ref);
    ovx_path_dictionary_destroy_path_list(s.dict, released);
}

void ovxReleaseRead(OvxReadHandle read)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_reads.find(read);
    if (it == g_reads.end())
        return;
    ReadSession& s = it->second;
    // Usually already empty: the owner releases each group (and, before tearing down the stage,
    // every remaining handle) while the dictionary is guaranteed live. This is the fallback for a
    // session released with the stage still up.
    // One pass over the distinct lists rather than a nested walk over groups.
    //
    // Only when the attach that owns them is still the live one. Across a detach or reattach the
    // dictionary and every list in it are already gone; walking them here would destroy through
    // freed storage. The device buffers below are NOT affected -- they belong to the CUDA context,
    // which each borrow keeps alive independently of the attach -- so they are still freed.
    if (s.dict && !attachStillOwns(s.attach, "ovxReleaseRead"))
        s.dict = nullptr;
    if (s.dict)
    {
        for (const std::pair<const ovx_primpath_list_t, uint32_t>& entry : s.listRefs)
            if (entry.first != OVX_INVALID_PRIMPATH_LIST && entry.second > 0)
                ovx_path_dictionary_destroy_path_list(s.dict, entry.first);
    }
    s.listRefs.clear();
    for (GroupStore& g : s.groups)
        g.list = OVX_INVALID_PRIMPATH_LIST;
    // Each context frees its own: a device pointer and an event are only valid in the context that
    // produced them, so the manager that allocated the buffers is the one that must free them.
    for (DeviceContextAllocs& alloc : s.deviceAllocs)
    {
        if (!alloc.ctxMgr || !alloc.ctxMgr->getCudaContext())
            continue;
        PxScopedCudaLock _lock(*alloc.ctxMgr);
        PxCudaContext* cu = alloc.ctxMgr->getCudaContext();
        // The read handed out its columns without blocking, so the gathers that fill them may
        // still be in flight. Wait for this session's own work before freeing what it writes
        // into. This is the one place the read blocks, and it is at release, not at read.
        // Starts false where completion is unknown. It used to start true unconditionally and be
        // reconsidered only inside `if (alloc.event)`, so the one case with no event AND no
        // successful drain -- the case that most needs the guard -- walked straight into the frees.
        bool drained = !alloc.completionUnknown;
        if (alloc.completionUnknown)
        {
            // One more attempt, because release happens later than the read: whatever made the
            // producer's drain fail may have cleared, and a successful drain here is the difference
            // between reclaiming the memory and leaking it for the life of the process.
            drained = cu->streamSynchronize(0) == CUDA_SUCCESS;
        }
        if (alloc.event)
        {
            CUevent evt = reinterpret_cast<CUevent>(alloc.event);
            // The wait's result decides whether the frees below are safe. Freeing after a FAILED
            // wait hands back memory a gather may still be writing into, which the allocator can
            // then reissue -- silent corruption in whatever gets it next.
            if (cu->eventSynchronize(evt) != CUDA_SUCCESS)
            {
                // One fallback: drain the whole context. Nothing finer is available once the event
                // itself is unusable.
                drained = cu->streamSynchronize(0) == CUDA_SUCCESS;
            }
            cu->eventDestroy(evt);
            alloc.event = 0;
        }

        if (!drained)
        {
            // QUARANTINE. Leaking these buffers is the lesser fault: the process has a CUDA error
            // it has not cleared, the completion of the work writing into them is unknown, and a
            // freed-but-live destination is unrecoverable where a leak is merely expensive.
            CARB_LOG_ERROR("ovphysx read release: could not confirm the device gather completed; %zu device "
                           "buffer(s) are deliberately not freed rather than freed while possibly in use.",
                           alloc.buffers.size());
            continue;
        }

        // Returned to the per-context pool for the next read to reuse, not freed.
        // This is past the completion wait above, so each buffer is idle -- the same guarantee that
        // made freeing it here safe, and it covers the pinned staging too: those are the sources of the
        // read's async HtoD copies, so they are only idle once this context's work has completed. A
        // quarantined context took the `continue` above and never reaches this, so neither a device
        // destination nor a pinned source still in flight is ever pooled.
        for (const PooledColumn& c : alloc.buffers)
            poolReleaseColumn(alloc.ctxMgr, cu, c.ptr, c.bytes);
        for (const PooledColumn& c : alloc.pinned)
            poolReleasePinned(alloc.ctxMgr, cu, c.ptr, c.bytes);
    }

    // Match every acquireReference() from deviceAllocsFor(), in a pass of its own so the balance
    // does not depend on which exit the loop above took -- a context with no CUDA context, and a
    // quarantined one whose buffers were deliberately not freed, both skip it via `continue`.
    //
    // The quarantine case releases too: the reference exists to keep the manager alive while this
    // session still owes it a free, and a session that has given up on freeing owes it nothing.
    // Pinning the manager for the process lifetime would leak it on top of the buffers.
    for (DeviceContextAllocs& alloc : s.deviceAllocs)
    {
        if (alloc.ctxMgr)
        {
            alloc.ctxMgr->release();
            alloc.ctxMgr = nullptr;
        }
    }

    g_reads.erase(it);
}


void ovxReleaseQuery(OvxOutputQueryHandle query)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    // No attach guard needed, and deliberately so: this dereferences nothing. QueryState owns only
    // its own vector and a BORROWED dict pointer, so erasing is safe whether or not the attach that
    // minted it is still live -- and it must stay safe, since dropping a stale query is exactly what
    // a caller does after the guards above reject one.
    g_queries.erase(query);
}

} // namespace omni::physx

namespace omni::physx::ovx
{

// Declared in OvxPhysicsShared.h; see there for why one counter serves both directions. Forwards to
// the tensors-layer counter that is homed next to the row cache it identifies, so the reads, the
// loose-rigid write, and the point-instancer write (which mints directly) all share ONE keyspace.
uint64_t nextRowsVersion()
{
    return tensors::nextOvStageRowsVersion();
}

// Declared in OvxPhysicsShared.h for the write session; see there for the lock-order note.
bool queryTypeScope(uint64_t query, uint32_t& outType, uint32_t& outScope)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const std::unordered_map<uint64_t, QueryState>::const_iterator it = g_queries.find(query);
    if (it == g_queries.end())
        return false;
    outType = it->second.type;
    outScope = it->second.scope;
    return true;
}

} // namespace omni::physx::ovx
