// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-STRING-001
 * @covers AC-4
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-1 AC-4 AC-6
 *
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-5
 *
 * @implements REQ-CAPI-OBJCHANGE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-OBJCHANGE-002
 * @covers AC-1 AC-2 AC-3 AC-4
 */


// PhysX object change notifications: forward subscribe/unsubscribe calls to
// the omni::physx IPhysx singleton via the internal sidecar, scoped to the
// ovphysx instance that registered the subscription.

#include "ovphysx/ovphysx.h"
#include "ovphysx/experimental/ovphysx.hpp"
#include "internal/Profiling.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalObjectChange.h"  // g_sidecarSubscribeObjectChanges / Unsubscribe

#include <carb/Framework.h>
#include <carb/tasking/ITasking.h>

#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

// Written by loadInternalSidecar() through the externs in
// ovphysxInternalObjectChange.h.
std::atomic<OvphysxSidecarSubscribeObjectChangesFn>   g_sidecarSubscribeObjectChanges{nullptr};
std::atomic<OvphysxSidecarUnsubscribeObjectChangesFn> g_sidecarUnsubscribeObjectChanges{nullptr};

namespace {

struct TrampolineState
{
    ovphysx_instance_handle_t        owner = OVPHYSX_INVALID_HANDLE;
    ovphysx_object_created_fn        on_created;
    ovphysx_object_destroyed_fn      on_destroyed;
    ovphysx_all_objects_destroyed_fn on_all_destroyed;
    void*                            user_data;
    bool                             unsubscribe_in_progress = false;
    std::atomic<bool>                closed{false};
    void                           (*releaseUserData)(void*) = nullptr;
    uint64_t                         runtime_subscription = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    std::shared_ptr<OvphysxObjectChangeLifetime> lifetime;
    // Lock-free so a runtime worker can filter without taking the subscription
    // mutex. True only while this subscription is bound to its instance's live
    // attachment. A peer instance's attachment does not clear it.
    std::atomic<bool>                deliver{false};
};

struct ActiveCallbackGuard;

// A Carbonite task can suspend on a task mutex and resume on another worker.
// The guard has to follow that task. Native SDK threads have no task context,
// so they keep the head in thread-local storage. One process-lifetime slot is
// allocated on first use; task storage keys are too scarce to allocate per
// callback. SDK shutdown may release the framework on Linux; this slot relies
// on the pinned tasking implementation retaining its key allocation across those
// restarts. Windows retains the framework until process exit.
thread_local ActiveCallbackGuard* t_activeCallback = nullptr;
carb::tasking::TaskStorageKey g_activeCallbackStorage = carb::tasking::kInvalidTaskStorageKey;

carb::tasking::ITasking* objectChangeTasking()
{
    carb::Framework* framework = carb::getFramework();
    if (framework == nullptr)
        return nullptr;
    return framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
}

bool inCarboniteTask(carb::tasking::ITasking* tasking)
{
    return tasking != nullptr && tasking->getTaskContext() != carb::tasking::kInvalidTaskContext;
}

carb::tasking::TaskStorageKey objectChangeCallbackStorage(carb::tasking::ITasking* tasking)
{
    static std::once_flag once;
    std::call_once(once, [tasking]() {
        g_activeCallbackStorage = tasking->allocTaskStorage(nullptr);
    });
    // Continuing without the slot would let a migrated task look idle and
    // deadlock the next unsubscribe. Same failure mode as a failed set.
    if (g_activeCallbackStorage == carb::tasking::kInvalidTaskStorageKey)
        std::terminate();
    return g_activeCallbackStorage;
}

ActiveCallbackGuard* activeCallback()
{
    carb::tasking::ITasking* tasking = objectChangeTasking();
    if (!inCarboniteTask(tasking))
        return t_activeCallback;
    return static_cast<ActiveCallbackGuard*>(
        tasking->getTaskStorage(objectChangeCallbackStorage(tasking)));
}

void setActiveCallback(ActiveCallbackGuard* guard)
{
    carb::tasking::ITasking* tasking = objectChangeTasking();
    if (!inCarboniteTask(tasking))
    {
        t_activeCallback = guard;
        return;
    }
    if (!tasking->setTaskStorage(objectChangeCallbackStorage(tasking), guard))
        std::terminate();
}

struct ActiveCallbackGuard
{
    TrampolineState* state;
    ActiveCallbackGuard* previous;
    explicit ActiveCallbackGuard(TrampolineState* s)
        : state(s)
        , previous(activeCallback())
    {
        setActiveCallback(this);
    }
    ~ActiveCallbackGuard() { setActiveCallback(previous); }
    ActiveCallbackGuard(const ActiveCallbackGuard&) = delete;
    ActiveCallbackGuard& operator=(const ActiveCallbackGuard&) = delete;
};

bool isActiveOnThisThread(const TrampolineState* state)
{
    for (const ActiveCallbackGuard* g = activeCallback(); g; g = g->previous)
    {
        if (g->state == state)
            return true;
    }
    return false;
}

struct InFlightGuard
{
    OvphysxObjectChangeLifetime& life;
    explicit InFlightGuard(OvphysxObjectChangeLifetime& lifetime)
        : life(lifetime)
    {
        std::lock_guard<std::mutex> lock(life.mutex);
        ++life.inFlight;
    }
    ~InFlightGuard()
    {
        {
            std::lock_guard<std::mutex> lock(life.mutex);
            --life.inFlight;
            life.cv.notify_all();
        }
        ovphysxObjectChangeNotifyQuiescent(life);
    }
    InFlightGuard(const InFlightGuard&) = delete;
    InFlightGuard& operator=(const InFlightGuard&) = delete;
};

void runDeferredUserDataRelease(TrampolineState* state);

// Runs after fn() has left, including when fn() throws. The std::function is
// no longer executing, so deleting its state here is not a use-after-free.
// InFlightGuard is still alive, so the trampoline is not erased first.
struct DeferredUserDataRelease
{
    TrampolineState* state;
    ~DeferredUserDataRelease() { runDeferredUserDataRelease(state); }
};

template <typename Fn>
void invokeUser(TrampolineState* state, Fn&& fn)
{
    InFlightGuard flight(*state->lifetime);
    ActiveCallbackGuard active(state);
    const DeferredUserDataRelease release{state};
    fn();
    static_cast<void>(release);
}

bool subscriptionIsLive(const TrampolineState* state)
{
    return state && state->deliver.load(std::memory_order_acquire);
}

void created_trampoline(const char* p, size_t n, int t, void* state_void)
{
    TrampolineState* state = static_cast<TrampolineState*>(state_void);
    if (!state || state->closed.load(std::memory_order_acquire) || !state->on_created || !subscriptionIsLive(state))
        return;
    const ovphysx_string_t primPath = { p, n };
    invokeUser(state, [&] { state->on_created(primPath, static_cast<ovphysx_physx_type_t>(t), state->user_data); });
}

void destroyed_trampoline(const char* p, size_t n, int t, void* state_void)
{
    TrampolineState* state = static_cast<TrampolineState*>(state_void);
    if (!state || state->closed.load(std::memory_order_acquire) || !state->on_destroyed || !subscriptionIsLive(state))
        return;
    const ovphysx_string_t primPath = { p, n };
    invokeUser(state, [&] { state->on_destroyed(primPath, static_cast<ovphysx_physx_type_t>(t), state->user_data); });
}

void all_destroyed_trampoline(void* state_void)
{
    TrampolineState* state = static_cast<TrampolineState*>(state_void);
    if (!state || state->closed.load(std::memory_order_acquire) || !state->on_all_destroyed || !subscriptionIsLive(state))
        return;
    invokeUser(state, [&] { state->on_all_destroyed(state->user_data); });
}

static std::mutex g_subMutex;
static std::unordered_map<uint64_t, std::unique_ptr<TrampolineState>> g_subs;

void eraseClosedSubscription(uint64_t subscriptionId)
{
    std::lock_guard<std::mutex> lk(g_subMutex);
    g_subs.erase(subscriptionId);
}

void runDeferredUserDataRelease(TrampolineState* state)
{
    void (*releaseUserData)(void*) = nullptr;
    void* userData = nullptr;
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        releaseUserData = state->releaseUserData;
        userData = state->user_data;
        state->releaseUserData = nullptr;
        if (releaseUserData != nullptr)
            state->user_data = nullptr;
    }
    if (releaseUserData != nullptr)
        releaseUserData(userData);
}

constexpr uint64_t kInvalidSub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
std::atomic<uint64_t> g_nextSubscription{1};

uint64_t bindSubscription(TrampolineState& state, uint64_t attach)
{
    OvphysxSidecarSubscribeObjectChangesFn subscribe =
        g_sidecarSubscribeObjectChanges.load(std::memory_order_acquire);
    if (!subscribe)
        return kInvalidSub;
    const uint64_t id = subscribe(attach,
        state.on_created ? created_trampoline : nullptr,
        state.on_destroyed ? destroyed_trampoline : nullptr,
        state.on_all_destroyed ? all_destroyed_trampoline : nullptr,
        &state,
        &state.lifetime);
    if (id != kInvalidSub)
        state.deliver.store(true, std::memory_order_release);
    return id;
}

ovphysx_result_t unsubscribeOwned(
    ovphysx_instance_handle_t handle, ovphysx_subscription_id_t subscription, bool requireOwnerMatch)
{
    if (subscription == kInvalidSub)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "subscription is OVPHYSX_INVALID_SUBSCRIPTION_ID");
    // A dispatch snapshot retains every subscriber until the running callback
    // returns. Waiting for any of them here deadlocks. Leave them registered.
    if (activeCallback() != nullptr)
    {
        return set_error(OVPHYSX_API_INVALID_STATE,
            "cannot unsubscribe an object-change subscription from inside a callback");
    }

    uint64_t runtimeSubscription = kInvalidSub;
    std::shared_ptr<OvphysxObjectChangeLifetime> lifetime;
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        auto it = g_subs.find(subscription);
        if (it == g_subs.end() || it->second->unsubscribe_in_progress)
            return set_error(OVPHYSX_API_NOT_FOUND, "unknown or already-unsubscribed subscription id");
        if (requireOwnerMatch && it->second->owner != handle)
            return set_error(OVPHYSX_API_NOT_FOUND, "subscription belongs to another instance");
        it->second->unsubscribe_in_progress = true;
        runtimeSubscription = it->second->runtime_subscription;
        lifetime = it->second->lifetime;
    }

    auto clear_in_progress = [subscription]() {
        std::lock_guard<std::mutex> lk(g_subMutex);
        auto it = g_subs.find(subscription);
        if (it != g_subs.end())
            it->second->unsubscribe_in_progress = false;
    };

    auto unsubscribe = g_sidecarUnsubscribeObjectChanges.load(std::memory_order_acquire);
    if (runtimeSubscription != kInvalidSub && !unsubscribe)
    {
        clear_in_progress();
        return set_error(OVPHYSX_API_ERROR,
            "internal sidecar unavailable during unsubscribe -- subscription state retained");
    }

    const int rc = runtimeSubscription == kInvalidSub ? 0 : unsubscribe(runtimeSubscription);
    if (rc != 0)
    {
        clear_in_progress();
        return set_error(OVPHYSX_API_ERROR, "internal sidecar failed to unsubscribe");
    }

    if (lifetime)
    {
        std::unique_lock<std::mutex> lock(lifetime->mutex);
        lifetime->cv.wait(lock, [&] { return lifetime->copies == 0 && lifetime->inFlight == 0; });
    }

    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        g_subs.erase(subscription);
    }
    return success();
}

// Returns 1 when the caller must free user_data now, 0 when this subscription's
// callback is on the stack and releaseUserData runs after that callback returns.
int closeObjectChangeFromCallbackImpl(
    ovphysx_instance_handle_t handle,
    ovphysx_subscription_id_t subscription,
    void (*releaseUserData)(void*))
{
    std::shared_ptr<OvphysxObjectChangeLifetime> life;
    uint64_t runtimeSubscription = kInvalidSub;
    bool releaseNow = true;
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        const std::unordered_map<uint64_t, std::unique_ptr<TrampolineState>>::iterator it =
            g_subs.find(subscription);
        if (it == g_subs.end())
            return 1;
        TrampolineState* state = it->second.get();
        if (state->owner != handle)
            return 1;
        const bool active = isActiveOnThisThread(state);
        if (state->closed.load(std::memory_order_acquire))
        {
            if (active && releaseUserData != nullptr && state->releaseUserData == nullptr)
                state->releaseUserData = releaseUserData;
            return active ? 0 : 1;
        }
        state->closed.store(true, std::memory_order_release);
        state->deliver.store(false, std::memory_order_release);
        state->unsubscribe_in_progress = true;
        if (active && releaseUserData != nullptr)
        {
            state->releaseUserData = releaseUserData;
            releaseNow = false;
        }
        runtimeSubscription = state->runtime_subscription;
        state->runtime_subscription = kInvalidSub;
        life = state->lifetime;
    }

    const OvphysxSidecarUnsubscribeObjectChangesFn unsubscribe =
        g_sidecarUnsubscribeObjectChanges.load(std::memory_order_acquire);
    if (runtimeSubscription != kInvalidSub && unsubscribe != nullptr)
        (void)unsubscribe(runtimeSubscription);

    if (life)
    {
        {
            std::lock_guard<std::mutex> lock(life->mutex);
            life->closeWhenQuiescent = true;
            life->closeSubscriptionId = subscription;
            life->onQuiescent = &eraseClosedSubscription;
        }
        ovphysxObjectChangeNotifyQuiescent(*life);
    }
    return releaseNow ? 1 : 0;
}

} // anonymous namespace


OVPHYSX_API ovphysx_result_t ovphysx_subscribe_object_changes(
    ovphysx_instance_handle_t handle,
    const ovphysx_object_change_callbacks_t* callbacks,
    ovphysx_subscription_id_t* out_subscription)
{
    OVPHYSX_API_PROFILE_ZONE(subscribe_object_changes);
    if (!out_subscription)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_subscription is NULL");
    *out_subscription = kInvalidSub;

    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid OVPhysX handle");

    if (!callbacks)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "callbacks is NULL");

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
    state->lifetime         = std::make_shared<OvphysxObjectChangeLifetime>();
    state->owner            = handle;
    state->on_created       = callbacks->on_object_created;
    state->on_destroyed     = callbacks->on_object_destroyed;
    state->on_all_destroyed = callbacks->on_all_objects_destroyed;
    state->user_data        = callbacks->user_data;
    if (instance->attachHandle != 0)
    {
        state->runtime_subscription = bindSubscription(*state, instance->attachHandle);
        if (state->runtime_subscription == kInvalidSub)
            return set_error(OVPHYSX_API_ERROR, "runtime rejected object-change subscription");
    }
    const uint64_t id = g_nextSubscription.fetch_add(1, std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        g_subs.emplace(id, std::move(state));
    }

    *out_subscription = id;
    return success();
}


OVPHYSX_API ovphysx_result_t ovphysx_unsubscribe_object_changes(
    ovphysx_instance_handle_t handle,
    ovphysx_subscription_id_t subscription)
{
    OVPHYSX_API_PROFILE_ZONE(unsubscribe_object_changes);
    if (!get_instance(handle))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid OVPhysX handle");
    return unsubscribeOwned(handle, subscription, /*requireOwnerMatch=*/true);
}

namespace ovphysx {
namespace detail {
OVPHYSX_API int closeObjectChangeFromCallback(
    ovphysx_instance_handle_t handle,
    ovphysx_subscription_id_t subscription,
    void (*releaseUserData)(void*))
{
    return closeObjectChangeFromCallbackImpl(handle, subscription, releaseUserData);
}
} // namespace detail
} // namespace ovphysx

// Public subscriptions belong to the ovphysx instance, including intervals with
// no attachment. Runtime subscriptions are retired by detach and renewed on attach.
bool ovphysx_object_change_attach_instance(ovphysx_instance_handle_t handle, uint64_t attach)
{
    std::lock_guard<std::mutex> lock(g_subMutex);
    for (const std::pair<const uint64_t, std::unique_ptr<TrampolineState>>& entry : g_subs)
    {
        TrampolineState& state = *entry.second;
        if (state.owner == handle && !state.closed.load(std::memory_order_acquire) &&
            state.runtime_subscription == kInvalidSub)
        {
            state.runtime_subscription = bindSubscription(state, attach);
            if (state.runtime_subscription == kInvalidSub)
                return false;
        }
    }
    return true;
}

void ovphysx_object_change_detached_instance(ovphysx_instance_handle_t handle)
{
    std::lock_guard<std::mutex> lock(g_subMutex);
    for (const std::pair<const uint64_t, std::unique_ptr<TrampolineState>>& entry : g_subs)
        if (entry.second->owner == handle)
        {
            entry.second->deliver.store(false, std::memory_order_release);
            entry.second->runtime_subscription = kInvalidSub;
        }
}

void ovphysx_object_change_cleanup_instance(ovphysx_instance_handle_t handle)
{
    std::vector<ovphysx_subscription_id_t> owned;
    {
        std::lock_guard<std::mutex> lk(g_subMutex);
        for (const std::pair<const uint64_t, std::unique_ptr<TrampolineState>>& entry : g_subs)
        {
            if (entry.second && entry.second->owner == handle)
                owned.push_back(entry.first);
        }
    }
    for (ovphysx_subscription_id_t id : owned)
    {
        const ovphysx_result_t result = unsubscribeOwned(handle, id, /*requireOwnerMatch=*/false);
        if (result.status == OVPHYSX_API_INVALID_STATE)
            (void)closeObjectChangeFromCallbackImpl(handle, id, nullptr);
    }
}
