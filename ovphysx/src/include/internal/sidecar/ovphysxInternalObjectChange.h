// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

struct OvphysxObjectChangeLifetime;

// Called with no lifetime mutex held. The callback may drop the last
// shared_ptr to `life`, so it must not touch `life` after invoking onQuiescent.
inline void ovphysxObjectChangeNotifyQuiescent(OvphysxObjectChangeLifetime& life);

// Shared by the public subscription and every copy of the runtime callback.
// Dispatch snapshots copy the std::function, so `copies` stays nonzero until
// that snapshot is destroyed, which is after the user callback returns.
// Unsubscribe waits until both counters are zero before freeing subscriber state.
// A close started from inside a callback sets closeWhenQuiescent instead of waiting.
struct OvphysxObjectChangeLifetime
{
    std::mutex mutex;
    std::condition_variable cv;
    int copies = 0;
    int inFlight = 0;
    bool closeWhenQuiescent = false;
    uint64_t closeSubscriptionId = 0;
    void (*onQuiescent)(uint64_t subscriptionId) = nullptr;
};

inline void ovphysxObjectChangeNotifyQuiescent(OvphysxObjectChangeLifetime& life)
{
    void (*onQuiescent)(uint64_t) = nullptr;
    uint64_t subscriptionId = 0;
    {
        std::lock_guard<std::mutex> lock(life.mutex);
        if (!life.closeWhenQuiescent || life.copies != 0 || life.inFlight != 0)
            return;
        life.closeWhenQuiescent = false;
        onQuiescent = life.onQuiescent;
        subscriptionId = life.closeSubscriptionId;
        life.onQuiescent = nullptr;
    }
    if (onQuiescent != nullptr)
        onQuiescent(subscriptionId);
}

// One retain per live std::function copy. Move transfers the retain.
struct OvphysxObjectChangeKeep
{
    std::shared_ptr<OvphysxObjectChangeLifetime> life;

    explicit OvphysxObjectChangeKeep(std::shared_ptr<OvphysxObjectChangeLifetime> lifetime)
        : life(std::move(lifetime))
    {
        retain();
    }

    OvphysxObjectChangeKeep(const OvphysxObjectChangeKeep& other)
        : life(other.life)
    {
        retain();
    }

    OvphysxObjectChangeKeep& operator=(const OvphysxObjectChangeKeep& other)
    {
        if (this != &other)
        {
            release();
            life = other.life;
            retain();
        }
        return *this;
    }

    OvphysxObjectChangeKeep(OvphysxObjectChangeKeep&& other) noexcept
        : life(std::move(other.life))
    {
    }

    OvphysxObjectChangeKeep& operator=(OvphysxObjectChangeKeep&& other) noexcept
    {
        if (this != &other)
        {
            release();
            life = std::move(other.life);
        }
        return *this;
    }

    ~OvphysxObjectChangeKeep() { release(); }

    void retain()
    {
        if (!life)
            return;
        std::lock_guard<std::mutex> lock(life->mutex);
        ++life->copies;
    }

    void release()
    {
        if (!life)
            return;
        // Local owner keeps the mutex alive until the lock is released, even if this was the last reference.
        std::shared_ptr<OvphysxObjectChangeLifetime> owner = std::move(life);
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            --owner->copies;
            owner->cv.notify_all();
        }
        ovphysxObjectChangeNotifyQuiescent(*owner);
    }
};

// Sidecar object-change subscription bridge. Translates omni::physx's
// std::function-based IPhysicsObjectChangeCallback into the C-friendly
// function-pointer signatures below so the main library can forward user
// callbacks without depending on USD/omni.physx types.

extern "C" {

// C-compatible callback signatures matching ovphysx_object_change_callbacks_t.
// Defined here (rather than including ovphysx_types.h) so the sidecar stays
// self-contained and the C ABI surface remains thin.
//
// prim_path_ptr / prim_path_len: UTF-8 prim path with
//                                prim_path_ptr[prim_path_len] == '\0'. Use len
//                                for content. Storage is owned by the sidecar
//                                and valid only for the callback duration.
// physx_type:                    omni::physx::PhysXType integer value.
// user_data:                     opaque caller pointer passed through unchanged.
typedef void (*ovphysx_internal_object_created_fn)(
    const char* prim_path_ptr, size_t prim_path_len, int physx_type, void* user_data);

typedef void (*ovphysx_internal_object_destroyed_fn)(
    const char* prim_path_ptr, size_t prim_path_len, int physx_type, void* user_data);

typedef void (*ovphysx_internal_all_objects_destroyed_fn)(void* user_data);

// Subscribe to omni::physx object change notifications.
//
// Any of the function-pointer parameters may be NULL. The sidecar skips NULL
// callbacks rather than invoking them. At least one of the three must be
// non-NULL, which the caller checks. The sidecar itself does not validate.
//
// Returns the omni::physx SubscriptionId on success, or UINT64_MAX on failure,
// matching OVPHYSX_INVALID_SUBSCRIPTION_ID in the public ABI. This is intentionally
// not the omni::physx kInvalidSubscriptionId value, which is only 40 bits and
// would otherwise be a valid IPhysx ID on the ovphysx side. Pair with
// ovphysx_internal_unsubscribe_object_changes().
//
// lifetime_keep points at a std::shared_ptr<OvphysxObjectChangeLifetime>. Each
// stored callback copies an OvphysxObjectChangeKeep so a dispatch snapshot holds
// the subscriber alive until that snapshot ends.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_subscribe_object_changes(
    uint64_t attach,
    ovphysx_internal_object_created_fn on_created,
    ovphysx_internal_object_destroyed_fn on_destroyed,
    ovphysx_internal_all_objects_destroyed_fn on_all_destroyed,
    void* user_data,
    void* lifetime_keep);

// Unsubscribe a previously-registered object-change subscription.
// Returns 0 on success, non-zero on failure (e.g. omni::physx unavailable).
OVPHYSX_INTERNAL_API int ovphysx_internal_unsubscribe_object_changes(
    uint64_t subscription_id);

// SDK-side function-pointer typedefs (mirror the above exports) for dlsym use.
typedef uint64_t (*OvphysxSidecarSubscribeObjectChangesFn)(
    uint64_t,
    ovphysx_internal_object_created_fn,
    ovphysx_internal_object_destroyed_fn,
    ovphysx_internal_all_objects_destroyed_fn,
    void* /* user_data */,
    void* /* lifetime_keep */);
typedef int      (*OvphysxSidecarUnsubscribeObjectChangesFn)(uint64_t);

} // extern "C"

// Resolved sidecar function pointers (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarSubscribeObjectChangesFn>   g_sidecarSubscribeObjectChanges;
extern std::atomic<OvphysxSidecarUnsubscribeObjectChangesFn> g_sidecarUnsubscribeObjectChanges;
