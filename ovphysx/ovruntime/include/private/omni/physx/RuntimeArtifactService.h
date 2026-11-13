// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-UPDATE-001
 * @covers AC-4 AC-10
 * @implements REQ-CACHE-STORE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CACHE-ATTACH-001
 * @covers AC-1
 */
#pragma once
#include "CompiledStageStore.h"
#include <carb/Framework.h>
#include <carb/tasking/ITasking.h>
#include <memory>
#include <algorithm>
#include <mutex>
#include <vector>

namespace omni::physx::cache
{
// Tests give every case its own datastore; null restores UJITSO's default. The
// local-only rule applies to it with the configurator features given (Local
// without them).
void useCompiledStageDataStoreForTest(carb::datastore::IDataStore* dataStore);
void useCompiledStageDataStoreForTest(carb::datastore::IDataStore* dataStore, uint32_t features);
carb::datastore::IDataStore* compiledStageDataStoreForTest();

// Physics-runtime client of the datastore/blob-key infrastructure used by
// UJITSO. It neither owns a cooking service nor submits mesh-cooking requests.
// Keep service references alive through all synchronous storage callbacks.
class RuntimeArtifactService
{
public:
    ~RuntimeArtifactService()
    {
        drain();
    }
    using Mode = CompiledStageStore::Mode;
    using ArtifactKind = CompiledStageStore::ArtifactKind;
    using Bundle = CompiledStageStore::Bundle;
    using Digest = CompiledStageStore::Digest;
    // A stored native section is deserialized in place by PhysX, which trusts its
    // counts and types; the envelope checksum detects corruption but is no
    // authentication. Hub, GRPC and Nucleus stores can deliver another machine's
    // bytes, so only a purely local datastore is used. `features` are
    // carb::datastore::DataStoreConfigurator::Features.
    static bool localOnly(uint32_t features);
    // Bundles live in the process's UJITSO datastore, the one collision cooking
    // uses. Null when UJITSO is not loaded or its datastore is not purely local.
    static std::unique_ptr<RuntimeArtifactService> create(Mode mode);
    bool visit(ArtifactKind kind,
               const Digest& input,
               const Digest& compatibility,
               const CompiledStageStore::Visitor& visitor,
               std::optional<Digest>* delivered = nullptr) const
    {
        drain();
        return adapter(kind).visit(input, compatibility, visitor, delivered);
    }
    // The consumer could not use the bundle delivered for input; see
    // CompiledStageStore::remove.
    void remove(ArtifactKind kind, const Digest& input, const Digest& compatibility, const Digest& payload) const
    {
        drain();
        adapter(kind).remove(input, compatibility, payload);
    }
    bool put(ArtifactKind kind, const Bundle& bundle) const
    {
        drain();
        return adapter(kind).put(bundle);
    }
    // Envelope checksums and datastore writes need only the owned bundle bytes,
    // never SDK objects, so they may finish beside later simulation. Lookups,
    // synchronous puts and destruction join pending publications first, so a
    // caller never observes this service's writes out of order. Without a
    // tasking service the bundle is published before returning.
    bool publish(ArtifactKind kind, Bundle&& bundle) const
    {
        carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
        if (!tasking)
            return put(kind, bundle);
        // Make room first: once the task exists, recording its future must not fail,
        // or nothing would join a task that uses this service.
        std::lock_guard<std::mutex> lock(mPendingMutex);
        if (mPending.size() == mPending.capacity())
            mPending.reserve(std::max<size_t>(4, mPending.size() * 2));
        carb::tasking::Future<bool> future =
            tasking->addTask(carb::tasking::Priority::eLow, {}, [this, kind, owned = std::move(bundle)]() {
                try
                {
                    return adapter(kind).put(owned);
                }
                catch (...)
                {
                    return false;
                }
            });
        mPending.push_back(std::move(future));
        return true;
    }
    // Join every queued publication. A failed publication only loses reuse.
    void drain() const
    {
        std::vector<carb::tasking::Future<bool>> pending;
        {
            std::lock_guard<std::mutex> lock(mPendingMutex);
            pending.swap(mPending);
        }
        for (auto& future : pending)
            future.get();
    }

private:
    RuntimeArtifactService(omni::core::ObjectPtr<carb::datastore::IDataStore> data,
                           const omni::blobkey::IBlobKey& keys,
                           Mode mode)
        : mData(std::move(data)), mKeys(keys), mMode(mode)
    {
    }
    CompiledStageStore adapter(ArtifactKind kind) const
    {
        return { *mData, mKeys, mMode, 512 * 1024 * 1024, kind };
    }
    omni::core::ObjectPtr<carb::datastore::IDataStore> mData;
    const omni::blobkey::IBlobKey& mKeys;
    Mode mMode;
    mutable std::mutex mPendingMutex;
    mutable std::vector<carb::tasking::Future<bool>> mPending;
};
} // namespace omni::physx::cache
