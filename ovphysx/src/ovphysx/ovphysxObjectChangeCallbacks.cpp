// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// PhysX object change notifications: forward subscribe/unsubscribe calls to
// the omni::physx IPhysx singleton via the internal sidecar.
//
// The internal sidecar links USD/omni.physx; this file does not. The sidecar
// loader resolves the subscribe/unsubscribe function pointers at instance
// creation and publishes them via g_sidecarSubscribeObjectChanges and
// g_sidecarUnsubscribeObjectChanges; this file just reads those.
// Subscriptions are process-global (single IPhysx singleton); see the
// docstring on ovphysx_subscribe_object_changes in ovphysx.h.

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalObjectChange.h"  // g_sidecarSubscribeObjectChanges / Unsubscribe

#include <carb/Framework.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

// Sidecar subscribe/unsubscribe atomics owned here next to their consumers;
// loader writes them during loadInternalSidecar() via the externs in
// ovphysxInternalObjectChange.h.
std::atomic<OvphysxSidecarSubscribeObjectChangesFn>   g_sidecarSubscribeObjectChanges{nullptr};
std::atomic<OvphysxSidecarUnsubscribeObjectChangesFn> g_sidecarUnsubscribeObjectChanges{nullptr};

namespace {

// Trampolines: take the clone-plugin C signature (char*, size_t, int) and call
// the user-facing C signature (ovphysx_string_t, ovphysx_physx_type_t).
//
// The internal sidecar holds the user_data and dispatches it back to us. We pack
// (user fn, user user_data) into a heap-allocated TrampolineState and use that
// as the clone-plugin user_data, so the static trampoline below can recover
// the original callback + user_data without TLS or globals.
struct TrampolineState
{
    ovphysx_object_created_fn        on_created;
    ovphysx_object_destroyed_fn      on_destroyed;
    ovphysx_all_objects_destroyed_fn on_all_destroyed;
    void*                            user_data;
    // Guards against concurrent double-unsubscribe: the first caller flips
    // this true under g_subMutex; competing callers see it and return
    // NOT_FOUND without re-calling the plugin's unsubscribe. Reset to false
    // by the unsubscribe path only on plugin-call failure so the caller can
    // retry; on success the entire entry is erased.
    bool                             unsubscribe_in_progress = false;
};

void created_trampoline(const char* p, size_t n, int t, void* state_void)
{
    auto* state = static_cast<TrampolineState*>(state_void);
    if (state && state->on_created)
    {
        ovphysx_string_t s = { p, n };
        state->on_created(s, static_cast<ovphysx_physx_type_t>(t), state->user_data);
    }
}

void destroyed_trampoline(const char* p, size_t n, int t, void* state_void)
{
    auto* state = static_cast<TrampolineState*>(state_void);
    if (state && state->on_destroyed)
    {
        ovphysx_string_t s = { p, n };
        state->on_destroyed(s, static_cast<ovphysx_physx_type_t>(t), state->user_data);
    }
}

void all_destroyed_trampoline(void* state_void)
{
    auto* state = static_cast<TrampolineState*>(state_void);
    if (state && state->on_all_destroyed)
        state->on_all_destroyed(state->user_data);
}

// We need to keep the TrampolineState alive for the lifetime of the
// subscription so the IPhysx callbacks can dispatch through it. Map keyed by
// our subscription ID (== underlying IPhysx subscription ID).
static std::mutex g_subMutex;
static std::unordered_map<uint64_t, std::unique_ptr<TrampolineState>> g_subs;

constexpr uint64_t kInvalidSub = OVPHYSX_INVALID_SUBSCRIPTION_ID;

} // anonymous namespace


OVPHYSX_API ovphysx_result_t ovphysx_subscribe_object_changes(
    const ovphysx_object_change_callbacks_t* callbacks,
    ovphysx_subscription_id_t* out_subscription)
{
    if (!out_subscription)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_subscription is NULL");
    *out_subscription = kInvalidSub;

    if (!callbacks)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "callbacks is NULL");

    // At least one callback must be set; an all-NULL struct would subscribe
    // to nothing and leak a subscription that can only be cleaned by unsubscribe.
    if (!callbacks->on_object_created && !callbacks->on_object_destroyed &&
        !callbacks->on_all_objects_destroyed)
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
            "at least one callback function pointer in ovphysx_object_change_callbacks_t must be non-NULL");
    }

    auto subscribe = g_sidecarSubscribeObjectChanges.load(std::memory_order_acquire);
    if (!subscribe)
    {
        return set_error(OVPHYSX_API_ERROR,
            "internal sidecar not loaded -- cannot subscribe to object changes "
            "(create an ovphysx instance first)");
    }

    auto state = std::make_unique<TrampolineState>();
    state->on_created       = callbacks->on_object_created;
    state->on_destroyed     = callbacks->on_object_destroyed;
    state->on_all_destroyed = callbacks->on_all_objects_destroyed;
    state->user_data        = callbacks->user_data;
    TrampolineState* state_ptr = state.get();

    uint64_t id = subscribe(
        callbacks->on_object_created       ? created_trampoline       : nullptr,
        callbacks->on_object_destroyed     ? destroyed_trampoline     : nullptr,
        callbacks->on_all_objects_destroyed ? all_destroyed_trampoline : nullptr,
        state_ptr);

    if (id == kInvalidSub)
    {
        return set_error(OVPHYSX_API_ERROR,
            "omni::physx interface unavailable -- ensure an ovphysx instance has been created");
    }

    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        g_subs.emplace(id, std::move(state));
    }

    *out_subscription = id;
    return success();
}


OVPHYSX_API ovphysx_result_t ovphysx_unsubscribe_object_changes(
    ovphysx_subscription_id_t subscription)
{
    if (subscription == kInvalidSub)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "subscription is OVPHYSX_INVALID_SUBSCRIPTION_ID");

    // Validate existence and claim the unsubscribe (atomic via g_subMutex) so
    // a concurrent caller racing on the same ID sees the in-progress flag and
    // returns NOT_FOUND without double-calling the plugin's unsubscribe. The
    // TrampolineState must outlive any in-flight callback, so we keep it in
    // g_subs through the plugin call and only erase on success.
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        auto it = g_subs.find(subscription);
        if (it == g_subs.end() || it->second->unsubscribe_in_progress)
            return set_error(OVPHYSX_API_NOT_FOUND, "unknown or already-unsubscribed subscription id");
        it->second->unsubscribe_in_progress = true;
    }

    // Helper to clear the in-progress flag on the error paths so the caller
    // can retry. No-op if the entry has already been erased by another path.
    auto clear_in_progress = [subscription]() {
        std::lock_guard<std::mutex> lk(g_subMutex);
        auto it = g_subs.find(subscription);
        if (it != g_subs.end())
            it->second->unsubscribe_in_progress = false;
    };

    auto unsubscribe = g_sidecarUnsubscribeObjectChanges.load(std::memory_order_acquire);
    if (!unsubscribe)
    {
        // Impossible-state error: the plugin must have been loaded for the
        // subscription to exist. Leave state in g_subs so the still-active
        // IPhysx subscription keeps a valid user_data pointer.
        clear_in_progress();
        return set_error(OVPHYSX_API_ERROR,
            "internal sidecar unavailable during unsubscribe -- subscription state retained");
    }

    int rc = unsubscribe(subscription);
    if (rc != 0)
    {
        // C-side unsubscribe failed; the IPhysx subscription may still be
        // active. Leave state in g_subs so any future callback finds valid
        // user_data. The leak is permanent for this subscription.
        clear_in_progress();
        return set_error(OVPHYSX_API_ERROR, "internal sidecar failed to unsubscribe");
    }

    // C-side subscription is gone. Safe to free the state now.
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        g_subs.erase(subscription);
    }
    return success();
}
