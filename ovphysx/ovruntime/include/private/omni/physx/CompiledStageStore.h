// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-STORE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-8
 */
#pragma once
#include <omni/physics/parse/CacheArchive.h>
#include <carb/InterfaceUtils.h>
#include <carb/datastore/DataStoreUtils.inl>
#include <carb/tasking/ITasking.h>
#include <carb/thread/Futex.h>
#include <algorithm>
#include <atomic>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

namespace omni::physx::cache
{
// Local IDataStore adapter, using the same storage family as Ujitso cooking.
// The creator owns both services for this adapter's lifetime. It chooses an
// isolated cache directory; no cooking settings or remote policy are modified.
class CompiledStageStore
{
public:
    using Bundle = omni::physics::parse::cache::Bundle;
    using Bytes = omni::physics::parse::cache::Bytes;
    using Digest = omni::physics::parse::cache::Digest;
    enum class Mode
    {
        eDisabled,
        eReadOnly,
        eReadWrite
    };
    enum class ArtifactKind : uint32_t
    {
        eWholeStage = 1,
        eReferenceSubtree = 2
    };
    CompiledStageStore(carb::datastore::IDataStore& store,
                       const omni::blobkey::IBlobKey& keys,
                       Mode mode = Mode::eDisabled,
                       size_t maxBytes = 512 * 1024 * 1024,
                       ArtifactKind kind = ArtifactKind::eWholeStage)
        : mStore(store), mKeys(keys), mMode(mode), mMaxBytes(maxBytes), mKind(kind)
    {
    }

    using BundleView = omni::physics::parse::cache::BundleView;
    using Visitor = std::function<bool(const BundleView&)>;
    bool get(const Digest& input, const Digest& compatibility, Bundle& output) const
    {
        Bundle candidate;
        if (!visit(input, compatibility, [&](const BundleView& view) {
                candidate.input = view.input;
                candidate.compatibility = view.compatibility;
                candidate.runtime.assign(view.runtime.data, view.runtime.data + view.runtime.size);
                candidate.native.assign(view.native.data, view.native.data + view.native.size);
                return true;
            }))
            return false;
        output = std::move(candidate);
        return true;
    }
    // The visitor runs synchronously only after complete envelope validation.
    // Borrowed bytes and all workers consuming them must stay inside this call.
    // delivered, when given, receives the payload whose bundle reached the visitor,
    // also when the visitor throws.
    bool visit(const Digest& input,
               const Digest& compatibility,
               const Visitor& visitor,
               std::optional<Digest>* delivered = nullptr) const
    {
        if (delivered)
            delivered->reset();
        if (mMode == Mode::eDisabled)
            return false;
        // Slots fill in order, so the first empty one ends the lookup. A malformed
        // index entry, or a payload that was evicted, fails validation or was
        // rejected by a consumer, falls through to the next slot, as put skips it.
        for (uint32_t slot = 0; slot < kIndexSlots; ++slot)
        {
            Payload payload;
            bool occupied = false;
            if (!indexed(input, compatibility, slot, payload, &occupied))
            {
                if (occupied)
                    continue;
                return false;
            }
            if (rejected(payload.digest))
                continue;
            if (delivered)
                *delivered = payload.digest;
            bool valid = false;
            const Delivery delivery = visitPayload(payload, input, compatibility, &visitor, valid);
            if (delivery == Delivery::eValid)
                return valid;
            if (delivered)
                delivered->reset();
            if (delivery == Delivery::eInvalid)
                reject(payload.digest); // skip its read and checksum next time
        }
        return false;
    }
    // A consumer could not use a delivered bundle (for example, hydration rejected
    // it). Later lookups skip that payload and a later put replaces its slot. The
    // rejection is recorded in the datastore for every process, by a read-only
    // store too: it never stores bundles, but a bad one must not keep failing in
    // every later process. Only when the datastore refuses the record does this
    // process remember the rejection instead.
    void reject(const Digest& payload) const
    {
        if (mMode == Mode::eDisabled)
            return;
        omni::blobkey::KeyHash key;
        if (rejectionKey(payload, key))
        {
            std::lock_guard<WriteMutex> lock(writeMutex());
            if (write(key, Bytes{ 1 }))
                return;
        }
        ProcessRejections& rejections = processRejections();
        std::lock_guard<std::mutex> lock(rejections.mutex);
        rejections.payloads.insert(payload);
    }
    // Remove a bundle delivered for input that the consumer could not use, for
    // example because its restore failed. As reject, and the store, read-only or
    // not, also drops the index entries that name it and its bytes. IDataStore
    // cannot delete, so those shrink to one-byte tombstones where the backend
    // overwrites. Lookups skip a tombstoned entry, and the next put for input
    // reuses its slot.
    void remove(const Digest& input, const Digest& compatibility, const Digest& payload) const
    {
        reject(payload);
        if (mMode == Mode::eDisabled)
            return;
        std::lock_guard<WriteMutex> lock(writeMutex());
        for (uint32_t slot = 0; slot < kIndexSlots; ++slot)
        {
            Payload entry;
            bool occupied = false;
            if (!indexed(input, compatibility, slot, entry, &occupied))
            {
                if (occupied)
                    continue;
                return;
            }
            // Another process can reclaim this slot between the read above and the
            // shrink below; IDataStore has no compare-and-swap. The shrink then drops
            // that new entry, which only loses its reuse: lookups skip a tombstone,
            // and the next put fills the slot again.
            omni::blobkey::KeyHash indexKey;
            if (entry.digest == payload && makeIndexKey(input, compatibility, slot, indexKey))
            {
                shrink(indexKey);
                shrink(entry.key);
            }
        }
    }

    // Rejections this process remembered belong to the process's datastore.
    // Tests that replace it forget them.
    static void forgetProcessRejections()
    {
        ProcessRejections& rejections = processRejections();
        std::lock_guard<std::mutex> lock(rejections.mutex);
        rejections.payloads.clear();
    }

    bool put(const Bundle& bundle) const
    {
        using namespace omni::physics::parse::cache;
        if (mMode != Mode::eReadWrite || bundle.runtime.size() > mMaxBytes ||
            bundle.native.size() > mMaxBytes - bundle.runtime.size())
            return false;
        const Bytes payload = encodeBundle(bundle);
        if (payload.size() > mMaxBytes)
            return false;
        // Publications may run on several workers; the slot scan and its writes
        // must not interleave with another put, reject or remove in this process.
        std::lock_guard<WriteMutex> lock(writeMutex());
        // Bind the lookup to the envelope checksum encodeBundle appended;
        // decodeBundleView verifies it before exposing either section.
        Digest payloadDigest{};
        std::copy(payload.end() - payloadDigest.size(), payload.end(), payloadDigest.begin());
        // A consumer rejected these exact bytes. Lookups would skip them in any slot.
        if (rejected(payloadDigest))
            return false;
        omni::blobkey::KeyHash payloadKey;
        if (!makeKey("ovruntime.compiled-stage.payload.v2", payloadDigest.data(), payloadDigest.size(), payload.size(),
                     payloadKey))
            return false;
        Writer index;
        index.u64(payload.size());
        index.bytes(payloadDigest.data(), payloadDigest.size());
        // A slot naming a usable payload is kept, and then nothing is written. An
        // evicted, corrupt or rejected slot is reclaimed by writing its index first:
        // a reader that sees it before the payload skips it as evicted, and a backend
        // that refuses to overwrite costs no payload write. An empty slot, which
        // only follows the filled ones, gets the payload first.
        struct Reclaimable
        {
            uint32_t slot;
            // The rejected or corrupt payload a reclaimed slot replaces.
            std::optional<omni::blobkey::KeyHash> stale;
        };
        std::vector<Reclaimable> reclaimable;
        uint32_t empty = kIndexSlots;
        for (uint32_t slot = 0; slot < kIndexSlots && empty == kIndexSlots; ++slot)
        {
            Payload existing;
            bool occupied = false;
            if (!indexed(bundle.input, bundle.compatibility, slot, existing, &occupied))
            {
                if (occupied)
                    reclaimable.push_back({ slot, std::nullopt }); // a corrupt index entry
                else
                    empty = slot;
                continue;
            }
            if (existing.digest == payloadDigest)
                return write(payloadKey, payload); // restores an evicted copy
            const bool refused = rejected(existing.digest);
            bool unused = false;
            const Delivery delivery = refused ?
                                          Delivery::eInvalid :
                                          visitPayload(existing, bundle.input, bundle.compatibility, nullptr, unused);
            if (delivery == Delivery::eValid)
                return true; // another usable bundle already serves this input
            reclaimable.push_back({ slot, delivery == Delivery::eInvalid ? std::optional(existing.key) : std::nullopt });
        }
        for (const Reclaimable& entry : reclaimable)
        {
            omni::blobkey::KeyHash indexKey;
            if (makeIndexKey(bundle.input, bundle.compatibility, entry.slot, indexKey) && write(indexKey, index.data()))
            {
                const bool stored = write(payloadKey, payload);
                if (entry.stale)
                    shrink(*entry.stale);
                return stored;
            }
        }
        omni::blobkey::KeyHash indexKey;
        return empty < kIndexSlots && makeIndexKey(bundle.input, bundle.compatibility, empty, indexKey) &&
               write(payloadKey, payload) && write(indexKey, index.data());
    }

private:
    // Each eviction or rejection on a backend that cannot overwrite consumes a slot.
    static constexpr uint32_t kIndexSlots = 16;
    enum class Delivery
    {
        eMissing,
        eInvalid,
        eValid
    };
    struct Payload
    {
        uint64_t size = 0;
        Digest digest{};
        omni::blobkey::KeyHash key{};
    };
    // occupied, when given, reports whether the slot holds any entry, valid or not.
    bool indexed(
        const Digest& input, const Digest& compatibility, uint32_t slot, Payload& payload, bool* occupied = nullptr) const
    {
        using namespace omni::physics::parse::cache;
        omni::blobkey::KeyHash indexKey;
        Bytes index;
        if (!makeIndexKey(input, compatibility, slot, indexKey))
            return false;
        // An entry has a fixed size; a larger one is malformed but still occupies its slot.
        bool present = false;
        const bool found = read(indexKey, kIndexSize, index, &present);
        if (occupied)
            *occupied = present;
        if (!found || index.size() != kIndexSize)
            return false;
        Reader r(index.data(), index.size());
        payload.size = r.u64();
        return r.bytes(payload.digest.data(), payload.digest.size()) && r.done() && payload.size &&
               payload.size <= mMaxBytes &&
               makeKey("ovruntime.compiled-stage.payload.v2", payload.digest.data(), payload.digest.size(),
                       payload.size, payload.key);
    }
    bool rejectionKey(const Digest& payload, omni::blobkey::KeyHash& key) const
    {
        return makeKey("ovruntime.compiled-stage.rejected.v1", payload.data(), payload.size(), 1, key);
    }
    // Rejections in this process that the datastore refused to record.
    struct ProcessRejections
    {
        std::mutex mutex;
        std::set<Digest> payloads;
    };
    static ProcessRejections& processRejections()
    {
        static ProcessRejections rejections;
        return rejections;
    }
    // Serializes this process's puts, rejections and removals across their slot
    // scans and writes. Publications run as carb tasks: a writer waiting here yields
    // its worker to other tasks, such as other instances' steps. The lock is a word
    // that waiters sleep on through carb.tasking's futex, so it holds no object of a
    // tasking instance: on Linux ovphysx can shut Carbonite down and start it again
    // in one process, and each wait uses the tasking interface current then. Without
    // carb.tasking a waiter sleeps on the word itself. Never destroyed, since
    // carb.tasking can unload first.
    class WriteMutex
    {
    public:
        void lock()
        {
            // 0 free, 1 held, 2 held with waiters ("Futexes Are Tricky", mutex 2).
            uint32_t state = 0;
            if (mState.compare_exchange_strong(state, 1, std::memory_order_acquire))
                return;
            if (state != 2)
                state = mState.exchange(2, std::memory_order_acquire);
            while (state != 0)
            {
                if (carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>())
                    tasking->futexWait(mState, uint32_t(2));
                else
                    carb::thread::futex::wait(mState, uint32_t(2));
                state = mState.exchange(2, std::memory_order_acquire);
            }
        }
        void unlock()
        {
            if (mState.exchange(0, std::memory_order_release) != 2)
                return;
            if (carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>())
                tasking->futexWakeup(mState, 1);
            else
                carb::thread::futex::notify_one(mState);
        }

    private:
        std::atomic<uint32_t> mState{ 0 };
    };
    static WriteMutex& writeMutex()
    {
        static WriteMutex* mutex = new WriteMutex();
        return *mutex;
    }
    bool rejected(const Digest& payload) const
    {
        {
            ProcessRejections& rejections = processRejections();
            std::lock_guard<std::mutex> lock(rejections.mutex);
            if (rejections.payloads.count(payload))
                return true;
        }
        omni::blobkey::KeyHash key;
        Bytes marker;
        return rejectionKey(payload, key) && read(key, 1, marker);
    }
    // Validate and consume directly while datastore memory is borrowed. Without a
    // visitor this only validates the stored envelope.
    Delivery visitPayload(const Payload& payload,
                          const Digest& input,
                          const Digest& compatibility,
                          const Visitor* visitor,
                          bool& valid) const
    {
        using namespace omni::physics::parse::cache;
        using namespace carb::datastore;
        struct Request
        {
            size_t size, limit;
            const Digest& payload;
            const Digest& input;
            const Digest& compatibility;
            const Visitor* visitor;
            Delivery delivery = Delivery::eMissing;
            bool valid = false;
            std::exception_ptr error;
        } request{ size_t(payload.size), mMaxBytes, payload.digest, input, compatibility, visitor };
        const OperationResult status = mStore.get(
            getDefaultStorageContext("ovruntime compiled-stage"), GetOptionsBuilder().setLevel(true, false, false),
            &payload.key, 1, nullptr,
            [](void* context, const GetCallbackResult& result) {
                auto& request = *static_cast<Request*>(context);
                if (result.result != OperationResult::SUCCESS || result.keyCount != 1 || !result.index ||
                    result.index[0] != 0 || !result.sizesInBytes || !result.dataBlocks || !result.dataBlocks[0])
                    return;
                request.delivery = Delivery::eInvalid;
                const auto* bytes = result.dataBlocks[0];
                if (result.sizesInBytes[0] != request.size || request.size < 32 ||
                    !std::equal(request.payload.begin(), request.payload.end(), bytes + request.size - 32))
                    return;
                // Never unwind through the datastore plugin; rethrow after it returns.
                try
                {
                    BundleView view;
                    if (!decodeBundleView(bytes, request.size, request.input, request.compatibility, view, request.limit))
                        return;
                    request.delivery = Delivery::eValid;
                    request.valid = !request.visitor || (*request.visitor)(view);
                }
                catch (...)
                {
                    request.error = std::current_exception();
                }
            },
            &request);
        if (request.error)
            std::rethrow_exception(request.error);
        if (status != OperationResult::SUCCESS)
            return Delivery::eMissing;
        valid = request.valid;
        return request.delivery;
    }
    static constexpr size_t kIndexSize = 8 + 32;
    bool makeKey(std::string_view domain,
                 const void* data,
                 size_t size,
                 uint64_t payloadSize,
                 omni::blobkey::KeyHash& output) const
    {
        omni::physics::parse::cache::Writer w;
        w.string(domain);
        w.bytes(data, size);
        return mKeys.keyhash.create(&output, w.data().data(), w.data().size(), payloadSize) ==
               omni::blobkey::KeyHash::success;
    }
    bool makeIndexKey(const Digest& input, const Digest& compatibility, uint32_t slot, omni::blobkey::KeyHash& output) const
    {
        omni::physics::parse::cache::Writer w;
        w.u32(uint32_t(mKind));
        w.u32(slot);
        w.bytes(input.data(), input.size());
        w.bytes(compatibility.data(), compatibility.size());
        return makeKey("ovruntime.compiled-stage.index.v4", w.data().data(), w.data().size(), kIndexSize, output);
    }
    bool write(const carb::datastore::HashKey& key, const Bytes& data) const
    {
        using namespace carb::datastore;
        const uint8_t* bytes = data.data();
        const size_t size = data.size();
        const OperationResult result = mStore.setWithKey(getDefaultStorageContext("ovruntime compiled-stage"),
                                                         SetOptionsBuilder().setLevel(true, false, false), &key, 1,
                                                         &bytes, &size, nullptr, nullptr);
        if (result == OperationResult::SUCCESS)
            return true;
        // Some immutable stores report an existing key as NOT_SUPPORTED.
        // Verify its bytes before treating a duplicate builder as successful.
        Bytes existing;
        return result == OperationResult::NOT_SUPPORTED && read(key, size, existing) && existing == data;
    }
    // IDataStore cannot delete. Where the backend overwrites, an unreachable payload
    // shrinks to one byte instead of staying until eviction; otherwise it stays.
    void shrink(const carb::datastore::HashKey& key) const
    {
        using namespace carb::datastore;
        const uint8_t tombstone = 0;
        const uint8_t* bytes = &tombstone;
        const size_t size = 1;
        mStore.setWithKey(getDefaultStorageContext("ovruntime compiled-stage"),
                          SetOptionsBuilder().setLevel(true, false, false), &key, 1, &bytes, &size, nullptr, nullptr);
    }
    // present, when given, reports whether the key holds bytes of any size.
    bool read(const carb::datastore::HashKey& key, size_t limit, Bytes& output, bool* present = nullptr) const
    {
        using namespace carb::datastore;
        struct Request
        {
            size_t limit;
            Bytes bytes;
            bool found = false;
            bool present = false;
            std::exception_ptr error;
        } request{ limit, {}, false, false, {} };
        const OperationResult status = mStore.get(
            getDefaultStorageContext("ovruntime compiled-stage"), GetOptionsBuilder().setLevel(true, false, false),
            &key, 1, nullptr,
            [](void* context, const GetCallbackResult& result) {
                Request& request = *static_cast<Request*>(context);
                if (result.result != OperationResult::SUCCESS || result.keyCount != 1 || !result.index ||
                    result.index[0] != 0 || !result.sizesInBytes || !result.dataBlocks || !result.dataBlocks[0])
                    return;
                request.present = true;
                if (result.sizesInBytes[0] > request.limit)
                    return;
                // Backend memory is borrowed only until this callback returns.
                try
                {
                    request.bytes.assign(result.dataBlocks[0], result.dataBlocks[0] + result.sizesInBytes[0]);
                }
                catch (...)
                {
                    request.error = std::current_exception();
                    return;
                }
                request.found = true;
            },
            &request);
        if (request.error)
            std::rethrow_exception(request.error);
        if (present)
            *present = status == OperationResult::SUCCESS && request.present;
        if (status != OperationResult::SUCCESS || !request.found)
            return false;
        output = std::move(request.bytes);
        return true;
    }
    carb::datastore::IDataStore& mStore;
    const omni::blobkey::IBlobKey& mKeys;
    Mode mMode;
    size_t mMaxBytes;
    ArtifactKind mKind;
};
} // namespace omni::physx::cache
