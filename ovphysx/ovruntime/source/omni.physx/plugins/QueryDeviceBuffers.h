// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-INPUT-DEVICE-001
// @covers AC-11
//
// @implements REQ-TENSOR-FK-001
// @covers AC-5
//
// @implements REQ-INPUT-DEVICE-001
// @covers AC-12
//
// @implements REQ-READ-DEVICE-001
// @covers AC-12

#pragma once

#include <common/utilities/CudaShimWrappers.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace physx
{
class PxScene;
}

namespace omni::physx::ovx
{

// The read buffer pool (ADR-0027), defined beside the read. `takeFromReadPool` hands out an idle device allocation of
// at least `bytes` from `ctxMgr`'s pool, its size in `capacityOut`, or 0 when the pool has none. `returnToReadPool`
// takes an idle allocation of `bytes` back, or frees it when the pool is full; never throws. A read's scratch goes
// back with the version of the list at its head, and a nonzero `version` asked for prefers a scratch still holding
// it, so that list need not be uploaded again; `versionOut` is the version the allocation came back with, 0 for
// none. Call both under `ctxMgr`'s lock.
CUdeviceptr takeFromReadPool(::physx::PxCudaContextManager* ctxMgr,
                             size_t bytes,
                             size_t& capacityOut,
                             uint64_t version = 0,
                             uint64_t* versionOut = nullptr);
bool returnToReadPool(::physx::PxCudaContextManager* ctxMgr,
                      CUdeviceptr ptr,
                      size_t bytes,
                      uint64_t version = 0) noexcept;

// Device buffers a query's operations reuse across calls instead of allocating at every call and freeing at its end:
// write columns, a kinematic update's index list, a read's columns and staging. The same buffer for the same use keeps
// its device pointer from step to step, which is what a captured (CUDA-graph) fill or read needs. The query holds this
// and every open write or read session shares it, because a session may outlive its query (ADR-0045). A buffer has
// exactly one owner at a time: a user takes it and gives it back once it has queued its last device use, recorded in
// the buffer's `ready` event, which the next user waits on before writing it (ADR-0052, ADR-0055). A read takes any
// buffer large enough, from the read buffer pool when its query holds none, and the buffers it drops go back to that
// pool.
struct QueryDeviceBuffers
{
    // Where a buffer is used: the scene, the attribute's table entry, and a sub-index for kinds that plan
    // several groups per scene and attribute (a tendon kind, an instancer's object key).
    struct Key
    {
        const void* scene = nullptr;
        const void* row = nullptr;
        uint64_t sub = 0;
        bool operator==(const Key& other) const
        {
            return scene == other.scene && row == other.row && sub == other.sub;
        }
    };
    // Uses with no attribute-table entry key with a null `row` and one of these in `sub`'s high bits.
    static constexpr uint64_t kDeformableColumn = 1ull << 32;
    static constexpr uint64_t kKinematicUpdateIndices = 3ull << 32;
    // A rigid-body or link read's columns and staging, one buffer per scene and body type in the low bits.
    static constexpr uint64_t kReadRigidColumns = 4ull << 32;
    // A whole-articulation read's, one buffer per scene and row set in the low bits: 0 the scene partition, 1 + c
    // inverse dynamics cohort c.
    static constexpr uint64_t kReadArticulationColumns = 5ull << 32;
    // A joint read's, a link read's incoming joint force and a tendon read's, one buffer per scene; a tendon read's
    // low bit says whether it reads fixed tendons.
    static constexpr uint64_t kReadJointColumns = 6ull << 32;
    static constexpr uint64_t kReadLinkForceColumns = 7ull << 32;
    static constexpr uint64_t kReadTendonColumns = 8ull << 32;
    // A rigid-body read's point-instancer columns, one buffer per scene.
    static constexpr uint64_t kReadInstancerColumns = 9ull << 32;
    // A deformable read's simulated columns, one buffer per scene; the low bit says whether it reads volumes. A
    // particle read's, one buffer per scene.
    static constexpr uint64_t kReadDeformableColumns = 10ull << 32;
    static constexpr uint64_t kReadParticleColumns = 11ull << 32;
    // A deformable skin-mesh read's column, one buffer per scene.
    static constexpr uint64_t kReadSkinMeshColumns = 12ull << 32;

    struct KeyHash
    {
        size_t operator()(const Key& key) const
        {
            size_t hash = std::hash<const void*>()(key.scene);
            hash = hash * 1099511628211ull ^ std::hash<const void*>()(key.row);
            return hash * 1099511628211ull ^ std::hash<uint64_t>()(key.sub);
        }
    };
    struct Buffer
    {
        CUdeviceptr data = 0;
        // Joint and tendon groups: the uploaded record list, sized by the same row count as `data`, and the
        // content version of the shared records it holds (0 = upload again). A read keeps its plan's version here,
        // and no list.
        CUdeviceptr recs = 0;
        uint64_t recsVersion = 0;
        // A stream-ordered write commit's or read's own device scratch (ADR-0052, ADR-0055) and its size.
        CUdeviceptr scratch = 0;
        size_t scratchBytes = 0;
        // The content version of the list at the head of `scratch` (0 = upload again).
        uint64_t scratchVersion = 0;
        // Recorded after the last queued device use of these buffers; 0 when there is none to wait for.
        uintptr_t ready = 0;
        // A read's events PhysX's copies join its stream through (ADR-0055); 0 for every other use.
        uintptr_t fork = 0;
        uintptr_t join = 0;
        // A read's: the size of `data`, which with `scratch` comes from and goes back to the read buffer pool. 0
        // for every other use, whose buffers are freed outright.
        size_t dataBytes = 0;
    };

    // Whether `take` would hand out a buffer for `key` on `ctxMgr`'s context spanning `bytes` and holding content
    // version `recsVersion`. Changes nothing and calls no CUDA API, so a call being captured into a CUDA graph can
    // ask before taking.
    bool holds(const Key& key, ::physx::PxCudaContextManager* ctxMgr, size_t bytes, uint64_t recsVersion)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        const std::unordered_map<Key, Parked, KeyHash>::const_iterator it = mIdle.find(key);
        return it != mIdle.end() && it->second.ctxMgr == ctxMgr && it->second.bytes == bytes &&
               it->second.buffer.recsVersion == recsVersion;
    }

    // Whether any buffer is parked under `key`. Changes nothing and calls no CUDA API, like `holds`, for a call being
    // captured that must refuse before it does anything else.
    bool parked(const Key& key)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return mIdle.find(key) != mIdle.end();
    }

    // Hands out the buffer parked under `key` when it lives on `ctxMgr`'s context and spans `bytes`, or with
    // `orLarger` at least `bytes`; anything else parked there is freed, and `freeFailed` reports whether that free
    // failed. Call under `ctxMgr`'s lock.
    bool take(const Key& key,
              ::physx::PxCudaContextManager* ctxMgr,
              size_t bytes,
              Buffer& out,
              bool& freeFailed,
              bool orLarger = false)
    {
        freeFailed = false;
        Parked parked;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            const std::unordered_map<Key, Parked, KeyHash>::iterator it = mIdle.find(key);
            if (it == mIdle.end())
                return false;
            parked = it->second;
            mIdle.erase(it);
        }
        const size_t capacity = parked.buffer.dataBytes ? parked.buffer.dataBytes : parked.bytes;
        if (parked.ctxMgr == ctxMgr && (orLarger ? capacity >= bytes : parked.bytes == bytes))
        {
            out = parked.buffer;
            ctxMgr->release(); // the taker holds its own reference
            return true;
        }
        freeFailed = !freeParked(parked, parked.ctxMgr == ctxMgr);
        return false;
    }

    // Parks `buffer` under `key`, or frees it when it has no device memory -- `take` hands out what is parked as
    // usable -- when one is already parked there, or when host memory runs out recording it. False when that free
    // failed; the buffer is then neither parked nor reusable. Never throws, since a destructor hands buffers back
    // through it. Call under `ctxMgr`'s lock, once the device no longer uses the buffer or `buffer.ready` is
    // recorded after its last queued use.
    bool give(const Key& key, ::physx::PxCudaContextManager* ctxMgr, size_t bytes, const Buffer& buffer) noexcept
    {
        Parked parked{ ctxMgr, bytes, buffer };
        ctxMgr->acquireReference();
        try
        {
            std::lock_guard<std::mutex> lock(mMutex);
            if (buffer.data && mIdle.emplace(key, parked).second)
                return true;
        }
        catch (...)
        {
        }
        return freeParked(parked, true);
    }

    // Frees every parked buffer whose scene is not in `liveScenes`. No later take can match one, so it would
    // otherwise stay parked until the query is released. False when a free failed. Call without any context
    // lock held.
    bool dropRetiredScenes(const std::vector<::physx::PxScene*>& liveScenes)
    {
        std::vector<Parked> retired;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            for (std::unordered_map<Key, Parked, KeyHash>::iterator it = mIdle.begin(); it != mIdle.end();)
            {
                if (std::find(liveScenes.begin(), liveScenes.end(), it->first.scene) == liveScenes.end())
                {
                    retired.push_back(it->second);
                    it = mIdle.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
        bool freed = true;
        for (Parked& parked : retired)
            freed = freeParked(parked, false) && freed;
        return freed;
    }

    // Frees every buffer parked under `scene` and `row` whose sub-index is not in `liveSubs`: the column of a
    // group that no longer exists, such as a removed instancer's, which no later take asks for. False when a
    // free failed. Call without any context lock held.
    bool dropAbsent(const void* scene, const void* row, const std::unordered_set<uint64_t>& liveSubs)
    {
        std::vector<Parked> absent;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            for (std::unordered_map<Key, Parked, KeyHash>::iterator it = mIdle.begin(); it != mIdle.end();)
            {
                if (it->first.scene == scene && it->first.row == row && liveSubs.count(it->first.sub) == 0)
                {
                    absent.push_back(it->second);
                    it = mIdle.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
        bool freed = true;
        for (Parked& parked : absent)
            freed = freeParked(parked, false) && freed;
        return freed;
    }

    // Frees every parked buffer. False when a free failed. The last release of the query or of a write sharing
    // this calls it, so a failure reaches that release. Call without any context lock held.
    bool drain() noexcept
    {
        // One entry at a time: a container to swap the entries into would allocate (MSVC's does when empty), and
        // each free takes its context lock, which must not be taken under mMutex.
        bool freed = true;
        for (;;)
        {
            Parked parked;
            {
                std::lock_guard<std::mutex> lock(mMutex);
                if (mIdle.empty())
                    break;
                const std::unordered_map<Key, Parked, KeyHash>::iterator it = mIdle.begin();
                parked = it->second;
                mIdle.erase(it);
            }
            freed = freeParked(parked, false) && freed;
        }
        return freed;
    }

    // Reached with buffers still parked only on a teardown path no release reports through.
    ~QueryDeviceBuffers()
    {
        if (!drain())
            CARB_LOG_ERROR("Releasing a query could not reclaim its parked CUDA buffers");
    }

private:
    struct Parked
    {
        ::physx::PxCudaContextManager* ctxMgr = nullptr; // referenced while parked
        size_t bytes = 0;
        Buffer buffer;
    };

public:
    // Frees `buffer` once its last queued device use has completed, through the optional CUDA interface, whose
    // results reach the caller; a read's column and scratch go back to `ctxMgr`'s read buffer pool instead. False
    // when the wait or a free failed; the memory is then lost rather than freed while in use or freed twice.
    // Cannot throw. Call under the buffer's context lock.
    static bool freeBuffer(const Buffer& buffer, ::physx::PxCudaContextManager* ctxMgr = nullptr) noexcept
    {
        omni::physx::IOptionalCuda* const cuda = omni::physx::cudaShimWrappers::getCudaShim();
        if (!cuda)
            return false;
        if (buffer.ready && !cuda->eventSynchronize(buffer.ready, nullptr))
            return false;
        const bool pooled = ctxMgr && buffer.dataBytes;
        const bool dataFreed = !buffer.data || (pooled ? returnToReadPool(ctxMgr, buffer.data, buffer.dataBytes) :
                                                         cuda->memFree(buffer.data, nullptr));
        const bool recsFreed = !buffer.recs || cuda->memFree(buffer.recs, nullptr);
        const bool scratchFreed =
            !buffer.scratch || (pooled ? returnToReadPool(ctxMgr, buffer.scratch, buffer.scratchBytes,
                                                          buffer.scratchVersion) :
                                         cuda->memFree(buffer.scratch, nullptr));
        bool eventsDestroyed = true;
        for (const uintptr_t event : { buffer.ready, buffer.fork, buffer.join })
            eventsDestroyed = (!event || cuda->eventDestroy(event, nullptr)) && eventsDestroyed;
        return dataFreed && recsFreed && scratchFreed && eventsDestroyed;
    }

private:
    // False when a free failed or the context is gone; see freeBuffer.
    static bool freeParked(Parked& parked, bool contextLocked) noexcept
    {
        bool freed = false;
        if (parked.ctxMgr->getCudaContext())
        {
            if (contextLocked)
            {
                freed = freeBuffer(parked.buffer, parked.ctxMgr);
            }
            else
            {
                ::physx::PxScopedCudaLock lock(*parked.ctxMgr);
                freed = freeBuffer(parked.buffer, parked.ctxMgr);
            }
        }
        parked.ctxMgr->release();
        return freed;
    }

    std::mutex mMutex;
    std::unordered_map<Key, Parked, KeyHash> mIdle;
};

} // namespace omni::physx::ovx
