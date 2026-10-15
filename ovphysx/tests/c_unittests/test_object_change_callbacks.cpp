// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Tests for ovphysx_subscribe_object_changes / ovphysx_unsubscribe_object_changes.
//
// Coverage:
//   - ABI / argument-validation: NULL callbacks, NULL out_subscription,
//     all-NULL callback functions, invalid subscription on unsubscribe,
//     unknown subscription returns NOT_FOUND, subscribe/unsubscribe roundtrip.
//   - Integration: the all-destroyed callback fires on ovphysx_detach_ovstage(),
//     unsubscribe stops delivery of subsequent events, and the initial stage
//     population delivers no per-object created/destroyed callbacks
//     (NVBugs 6473870).
//   - C++ RAII wrapper: destructor auto-unsubscribes, move transfers
//     ownership and leaves the source inactive.
//
// Subscriptions are per-instance. Tests use the shared CPU PhysXTestFixture.
//
// Known contract: ovphysx_clone() does not emit object_created notifications.
// Pointer caches refresh after the clone task wait, not from this subscription.
// CloneDeliversNoObjectChangeCallbacks locks that in.

/**
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-1 AC-4
 *
 * @implements REQ-CAPI-OBJCHANGE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-OBJCHANGE-002
 * @covers AC-2 AC-3 AC-4
 *
 * AC-2 (overlapping range applies only its unread suffix) needs a post-attach
 * ovstage edit, which this fixture cannot author. It is covered by the runtime
 * case named in TEST-CAPI-OVSTAGE-UPDATE-001.
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/experimental/ovphysx.hpp"
#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "test_utilities.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace test_utils;

namespace {

bool wait_task_success(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
                     uint64_t timeout_ns = 10'000'000'000ULL) {
    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_task(handle, task, timeout_ns, &wait_result);
    // The wait can return SUCCESS while individual ops failed. Those are printed
    // before the wait result is destroyed so test failures are not silent.
    bool any_op_failed = wait_result.num_errors > 0;
    if (any_op_failed && wait_result.error_tasks) {
        for (uint32_t i = 0; i < wait_result.num_errors; ++i) {
            ovphysx_string_t err = ovphysx_get_last_task_error(handle, wait_result.error_tasks[i]);
            if (err.ptr && err.length > 0) {
                std::cerr << "op error: " << std::string(err.ptr, err.length) << std::endl;
            }
        }
    }
    ovphysx_destroy_wait_task_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS && !any_op_failed;
}

bool load_usd_and_wait(ovphysx_instance_handle_t handle, const char* usd_path,
                       ovphysx_usd_handle_t& out_handle) {
    out_handle = 1;
    return attach_usd_with_ovstage(handle, usd_path);
}

bool step_and_wait(ovphysx_instance_handle_t handle, float dt) {
    ovphysx_enqueue_result_t res = ovphysx_step(handle, dt);
    return res.status == OVPHYSX_API_SUCCESS && wait_task_success(handle, res);
}

// Thread-safe recorder for callback events. Used by integration tests.
struct EventRecorder {
    std::mutex m;
    std::vector<std::pair<std::string, ovphysx_physx_type_t>> created;
    std::vector<std::pair<std::string, ovphysx_physx_type_t>> destroyed;
    std::atomic<int> allDestroyedCount{0};

    void recordCreated(ovphysx_string_t path, ovphysx_physx_type_t t) {
        std::lock_guard<std::mutex> lk(m);
        created.emplace_back(std::string(path.ptr, path.length), t);
    }

    void recordDestroyed(ovphysx_string_t path, ovphysx_physx_type_t t) {
        std::lock_guard<std::mutex> lk(m);
        destroyed.emplace_back(std::string(path.ptr, path.length), t);
    }
};

void recCreated(ovphysx_string_t path, ovphysx_physx_type_t t, void* user_data) {
    static_cast<EventRecorder*>(user_data)->recordCreated(path, t);
}
void recDestroyed(ovphysx_string_t path, ovphysx_physx_type_t t, void* user_data) {
    static_cast<EventRecorder*>(user_data)->recordDestroyed(path, t);
}
void recAllDestroyed(void* user_data) {
    static_cast<EventRecorder*>(user_data)->allDestroyedCount.fetch_add(1);
}

// RAII guard for a raw C subscription. Unsubscribes even if an ASSERT_* aborts
// the test scope. Otherwise the IPhysx subscription would outlive the stack
// EventRecorder it captures, and the fixture teardown reset() would dispatch
// the all-destroyed callback into freed memory.
struct ScopedSubscription {
    ovphysx_instance_handle_t handle = 0;
    ovphysx_subscription_id_t id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ~ScopedSubscription() {
        if (id != OVPHYSX_INVALID_SUBSCRIPTION_ID && handle != 0) {
            ovphysx_unsubscribe_object_changes(handle, id);
        }
    }
};

} // anonymous namespace


// ============================================================================
// Argument-validation tests
// ============================================================================

TEST_F(PhysXTestFixture, ObjectChangeSubscribeNullCallbacks) {
    ovphysx_subscription_id_t sub = 12345;  // sentinel that must be overwritten to INVALID
    ovphysx_result_t r = ovphysx_subscribe_object_changes(m_handle, nullptr, &sub);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);
}

TEST_F(PhysXTestFixture, ObjectChangeSubscribeNullOutId) {
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    ovphysx_result_t r = ovphysx_subscribe_object_changes(m_handle, &cb, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, ObjectChangeSubscribeAllNullCallbacks) {
    ovphysx_object_change_callbacks_t cb{};  // every fn-ptr is NULL
    ovphysx_subscription_id_t sub = 0;
    ovphysx_result_t r = ovphysx_subscribe_object_changes(m_handle, &cb, &sub);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);
}

TEST_F(PhysXTestFixture, ObjectChangeUnsubscribeInvalidIdRejected) {
    ovphysx_result_t r = ovphysx_unsubscribe_object_changes(m_handle, OVPHYSX_INVALID_SUBSCRIPTION_ID);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, ObjectChangeUnsubscribeUnknownIdReturnsNotFound) {
    // Pick a value that is unlikely to collide with a real IPhysx subscription
    // (which uses small monotonic IDs). A double-unsubscribe / typo from the
    // user should not crash and should be observable as NOT_FOUND.
    ovphysx_result_t r = ovphysx_unsubscribe_object_changes(m_handle, 0xDEADBEEFULL);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_FOUND);
}

TEST_F(PhysXTestFixture, ObjectChangeSubscribeUnsubscribeRoundtrip) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    cb.on_object_destroyed = recDestroyed;
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ovphysx_subscription_id_t sub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_result_t r = ovphysx_subscribe_object_changes(m_handle, &cb, &sub);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);

    ovphysx_result_t u = ovphysx_unsubscribe_object_changes(m_handle, sub);
    EXPECT_EQ(u.status, OVPHYSX_API_SUCCESS);

    // Second unsubscribe on the same ID must be NOT_FOUND, not crash.
    ovphysx_result_t u2 = ovphysx_unsubscribe_object_changes(m_handle, sub);
    EXPECT_EQ(u2.status, OVPHYSX_API_NOT_FOUND);
}


// ============================================================================
// Integration tests
// ============================================================================

class ObjectChangeIntegrationTest : public PhysXTestFixture {
protected:
    void SetUp() override {
        PhysXTestFixture::SetUp();
        ovphysx_usd_handle_t usd = 0;
        ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/basic_simulation.usda", usd))
            << "Failed to load scene";
        // First step attaches PhysX to the stage so that ovphysx_detach_ovstage() has
        // objects to tear down. The omni.physx "sim started" gate that
        // normally suppresses pre-simulation events is bypassed by the clone
        // plugin (ovphysxClone.cpp) setting stopCallbackWhenSimStopped=false
        // on subscription.
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

// NOTE: ovphysx_clone() does not fire object-change notifications. See
// CloneDeliversNoObjectChangeCallbacks.

// ovphysx_detach_ovstage() tears everything down in bulk. Subscribers must receive
// the all-destroyed notification rather than N per-object destructions.
TEST_F(ObjectChangeIntegrationTest, ResetTriggersAllDestroyedNotification) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));

    EXPECT_GE(rec.allDestroyedCount.load(), 1)
        << "ovphysx_detach_ovstage() should fire the all-objects-destroyed callback at least once";
    // guard destructor unsubscribes.
}

// @implements REQ-CAPI-INSTANCE-SCOPE-001
// @covers AC-4 AC-6
TEST_F(PhysXTestFixture, SubscriptionSurvivesDetachAndReattach) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;
    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);
    const ovphysx_subscription_id_t original = guard.id;
    for (int cycle = 0; cycle < 2; ++cycle) {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/basic_simulation.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
        const int previous = rec.allDestroyedCount.load();
        ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
        EXPECT_GT(rec.allDestroyedCount.load(), previous);
        EXPECT_EQ(guard.id, original);
    }
    ASSERT_EQ(ovphysx_unsubscribe_object_changes(m_handle, original).status, OVPHYSX_API_SUCCESS);
    guard.id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    const int afterUnsubscribe = rec.allDestroyedCount.load();
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/basic_simulation.usda"));
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(rec.allDestroyedCount.load(), afterUnsubscribe);
}

// After unsubscribe, no further events are delivered. ovphysx_detach_ovstage() is
// the stimulus because ovphysx_clone() does not fire object-change
// notifications, so a post-unsubscribe clone is silent regardless and would
// make this assertion vacuous.
TEST_F(ObjectChangeIntegrationTest, UnsubscribeStopsDelivery) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ovphysx_subscription_id_t sub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &sub).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_unsubscribe_object_changes(m_handle, sub).status, OVPHYSX_API_SUCCESS);

    // reset() would fire on_all_objects_destroyed if the subscription were
    // still active. After unsubscribe the callback count must stay at zero.
    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));

    EXPECT_EQ(rec.allDestroyedCount.load(), 0)
        << "No callbacks should be delivered after unsubscribe";
}

// Regression for NVBugs 6473870 / OMPE-102206 and NVBugs 6605420.
// Subscribing before the initial stage population and then attaching a fresh
// scene must not deliver any per-object created callbacks. ovphysx sets
// stopCallbackWhenSimStopped=false to receive reset/clone events while stopped,
// which must not bypass the initial-population suppression and leak one created
// event per actor/shape. Replaying the ordinal already parsed by attach must
// also be a no-op rather than delivering that initial population through
// update_from_ovstage. The caller already has that state from setup, so the
// count must be zero.
//
// Runs on the base fixture (which does not pre-attach), so the subscription is
// active before attach_ovstage(), matching the shipped C-API path.
TEST_F(PhysXTestFixture, InitialPopulationDeliversNoCreatedCallbacks) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    cb.on_object_destroyed = recDestroyed;
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    // Attach + initially populate a fresh scene while subscribed.
    ovphysx_usd_handle_t usd = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/basic_simulation.usda", usd))
        << "Failed to load scene";

    uint64_t attachOrdinal = 0;
    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        const auto attachmentIt = ovstage_test_attachments().find(m_handle);
        ASSERT_NE(attachmentIt, ovstage_test_attachments().end());
        ASSERT_FALSE(attachmentIt->second.empty());
        attachOrdinal = attachmentIt->second.back().ordinal;
    }
    ovstage_ordinal_range_t replayRange{};
    replayRange.start_ordinal = attachOrdinal;
    replayRange.end_ordinal = attachOrdinal;
    replayRange.has_start_ordinal = true;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_update_from_ovstage(m_handle, replayRange)).status, OVPHYSX_API_SUCCESS)
        << "replaying the attach ordinal should be a successful no-op";

    // The first step lets any deferred initial population run. It is still the
    // initial population and must remain silent.
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));

    size_t createdCount = 0;
    size_t destroyedCount = 0;
    {
        std::lock_guard<std::mutex> lk(rec.m);
        createdCount = rec.created.size();
        destroyedCount = rec.destroyed.size();
    }
    EXPECT_EQ(createdCount, 0u)
        << "initial attach/update population must not fire object_created callbacks";
    EXPECT_EQ(destroyedCount, 0u)
        << "initial attach/update population must not fire object_destroyed callbacks";
    // guard destructor unsubscribes.
}

struct DrainProbe {
    std::atomic<bool> entered{false};
    std::atomic<bool> callbackFinished{false};
    std::atomic<int> marker{1};
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    bool sawFinished = false;
    int markerAfter = 0;
};

void drainAllDestroyed(void* user_data) {
    DrainProbe* probe = static_cast<DrainProbe*>(user_data);
    probe->entered.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    probe->marker.store(2);
    probe->callbackFinished.store(true);
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-1
TEST_F(ObjectChangeIntegrationTest, UnsubscribeWaitsForInFlightCallback) {
    DrainProbe probe;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = drainAllDestroyed;
    cb.user_data = &probe;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);
    const ovphysx_subscription_id_t id = guard.id;

    std::thread unsubscriber([&] {
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!probe.entered.load()) {
            if (std::chrono::steady_clock::now() >= deadline)
                return;
            std::this_thread::yield();
        }
        const ovphysx_result_t result = ovphysx_unsubscribe_object_changes(m_handle, id);
        probe.status = result.status;
        probe.sawFinished = probe.callbackFinished.load();
        probe.markerAfter = probe.marker.load();
    });
    struct JoinOnExit {
        std::thread& thread;
        ~JoinOnExit()
        {
            if (thread.joinable())
                thread.join();
        }
    } join{unsubscriber};

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    guard.id = OVPHYSX_INVALID_SUBSCRIPTION_ID;

    if (unsubscriber.joinable())
        unsubscriber.join();

    EXPECT_TRUE(probe.entered.load());
    EXPECT_EQ(probe.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(probe.sawFinished);
    EXPECT_EQ(probe.markerAfter, 2);
}

struct SelfUnsubscribeProbe {
    ovphysx_instance_handle_t handle = 0;
    ovphysx_subscription_id_t id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    int calls = 0;
};

void selfUnsubscribeAllDestroyed(void* user_data) {
    SelfUnsubscribeProbe* probe = static_cast<SelfUnsubscribeProbe*>(user_data);
    ++probe->calls;
    probe->status = ovphysx_unsubscribe_object_changes(probe->handle, probe->id).status;
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-2
TEST_F(ObjectChangeIntegrationTest, UnsubscribeFromOwnCallbackReturnsInvalidState) {
    SelfUnsubscribeProbe probe;
    probe.handle = m_handle;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = selfUnsubscribeAllDestroyed;
    cb.user_data = &probe;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);
    probe.id = guard.id;

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(probe.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.calls, 1);

    ASSERT_EQ(ovphysx_unsubscribe_object_changes(m_handle, guard.id).status, OVPHYSX_API_SUCCESS);
    guard.id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    const int afterUnsubscribe = probe.calls;
    reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(probe.calls, afterUnsubscribe);
}

struct OtherUnsubscribeProbe {
    ovphysx_instance_handle_t handle = 0;
    ovphysx_subscription_id_t otherId = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    int calls = 0;
    int otherCalls = 0;
};

void unsubscribeOtherAllDestroyed(void* user_data) {
    OtherUnsubscribeProbe* probe = static_cast<OtherUnsubscribeProbe*>(user_data);
    ++probe->calls;
    probe->status = ovphysx_unsubscribe_object_changes(probe->handle, probe->otherId).status;
}

void countOtherAllDestroyed(void* user_data) {
    OtherUnsubscribeProbe* probe = static_cast<OtherUnsubscribeProbe*>(user_data);
    ++probe->otherCalls;
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-2
TEST_F(ObjectChangeIntegrationTest, UnsubscribeFromOtherCallbackReturnsInvalidState) {
    OtherUnsubscribeProbe probe;
    probe.handle = m_handle;

    ovphysx_object_change_callbacks_t otherCb{};
    otherCb.on_all_objects_destroyed = countOtherAllDestroyed;
    otherCb.user_data = &probe;
    ScopedSubscription other;
    other.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &otherCb, &other.id).status, OVPHYSX_API_SUCCESS);
    probe.otherId = other.id;

    ovphysx_object_change_callbacks_t selfCb{};
    selfCb.on_all_objects_destroyed = unsubscribeOtherAllDestroyed;
    selfCb.user_data = &probe;
    ScopedSubscription self;
    self.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &selfCb, &self.id).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(probe.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.calls, 1);
    EXPECT_EQ(probe.otherCalls, 1);

    ASSERT_EQ(ovphysx_unsubscribe_object_changes(m_handle, other.id).status, OVPHYSX_API_SUCCESS);
    other.id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    const int otherAfter = probe.otherCalls;
    reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(probe.otherCalls, otherAfter);
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-3
TEST_F(ObjectChangeIntegrationTest, CppSubscriptionDestroyedFromCallbackStopsDelivery) {
    int calls = 0;
    std::shared_ptr<int> captured = std::make_shared<int>(42);
    std::weak_ptr<int> weak = captured;
    ovphysx::ObjectChangeSubscription subscription;
    ovphysx::ObjectChangeCallbacks callbacks;
    callbacks.onAllDestroyed = [captured, &calls, &subscription]() {
        ++calls;
        subscription = ovphysx::ObjectChangeSubscription();
    };
    subscription = ovphysx::subscribeObjectChanges(m_handle, std::move(callbacks));
    callbacks = {};
    captured.reset();
    ASSERT_TRUE(subscription.isActive());
    ASSERT_FALSE(weak.expired());

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(calls, 1);
    EXPECT_FALSE(subscription.isActive());
    EXPECT_TRUE(weak.expired());

    // The first reset detaches the stage, so a second reset would be silent
    // even if this subscription were still live.
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/basic_simulation.usda"));
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(calls, 1);
}

struct TaskGuardUser {
    ovphysx_instance_handle_t handle = 0;
    ovphysx_subscription_id_t activeId = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    std::atomic<int> selfStatus{-1};
    std::atomic<int> invalidCount{0};
    std::atomic<int> absentCount{0};
};

void taskGuardReset(void* user) {
    TaskGuardUser* taskUser = static_cast<TaskGuardUser*>(user);
    EXPECT_EQ(test_utils::complete_task(taskUser->handle, ovphysx_detach_ovstage(taskUser->handle)).status,
              OVPHYSX_API_SUCCESS);
}

void taskGuardProbe(void* user) {
    TaskGuardUser* taskUser = static_cast<TaskGuardUser*>(user);
    // Unknown id: the guard runs before any runtime unsubscribe. Detach holds
    // that runtime lock across this callback, so a real id would deadlock.
    const ovphysx_result_t result =
        ovphysx_unsubscribe_object_changes(taskUser->handle, static_cast<ovphysx_subscription_id_t>(~0ULL) - 1);
    if (result.status == OVPHYSX_API_INVALID_STATE)
        taskUser->invalidCount.fetch_add(1, std::memory_order_release);
    else if (result.status == OVPHYSX_API_NOT_FOUND)
        taskUser->absentCount.fetch_add(1, std::memory_order_release);
}

void taskGuardAllDestroyed(void* user_data) {
    TaskGuardUser* taskUser = static_cast<TaskGuardUser*>(user_data);
    ovphysx_wait_on_task_guard_for_test_internal();
    const ovphysx_result_t result = ovphysx_unsubscribe_object_changes(taskUser->handle, taskUser->activeId);
    taskUser->selfStatus.store(static_cast<int>(result.status), std::memory_order_release);
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-2
TEST_F(ObjectChangeIntegrationTest, CallbackGuardFollowsCarboniteTaskNotWorker) {
    TaskGuardUser taskUser;
    taskUser.handle = m_handle;

    ovphysx_object_change_callbacks_t callbacks{};
    callbacks.on_all_objects_destroyed = taskGuardAllDestroyed;
    callbacks.user_data = &taskUser;
    ScopedSubscription active;
    active.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &callbacks, &active.id).status, OVPHYSX_API_SUCCESS);
    taskUser.activeId = active.id;

    ASSERT_TRUE(ovphysx_with_suspended_carbonite_task_for_test_internal(taskGuardReset, taskGuardProbe, &taskUser));
    EXPECT_EQ(taskUser.invalidCount.load(std::memory_order_acquire), 0);
    EXPECT_GE(taskUser.absentCount.load(std::memory_order_acquire), 8);
    EXPECT_EQ(taskUser.selfStatus.load(std::memory_order_acquire), static_cast<int>(OVPHYSX_API_INVALID_STATE));
}

// @implements REQ-CAPI-OBJCHANGE-001
// @covers AC-3
TEST_F(ObjectChangeIntegrationTest, CppSubscriptionReleasesCaptureWhenCallbackThrows) {
    std::shared_ptr<int> captured = std::make_shared<int>(1);
    const std::weak_ptr<int> weak = captured;
    int calls = 0;
    ovphysx::ObjectChangeSubscription subscription;
    ovphysx::ObjectChangeCallbacks callbacks;
    callbacks.onAllDestroyed = [&calls, &subscription, captured]() {
        ++calls;
        subscription = ovphysx::ObjectChangeSubscription();
        throw std::runtime_error("object-change callback failed");
    };
    captured.reset();
    subscription = ovphysx::subscribeObjectChanges(m_handle, std::move(callbacks));
    ASSERT_TRUE(subscription.isActive());

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(calls, 1);
    EXPECT_FALSE(subscription.isActive());
    EXPECT_TRUE(weak.expired());

    // The first reset detaches the stage, so a second reset would be silent
    // even if this subscription were still live.
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/basic_simulation.usda"));
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(calls, 1);
}

// @implements REQ-CAPI-OBJCHANGE-002
// @covers AC-2
TEST_F(ObjectChangeIntegrationTest, ResetDeliversNoPerObjectDestruction) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_destroyed = recDestroyed;
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));

    EXPECT_GE(rec.allDestroyedCount.load(), 1);
    std::lock_guard<std::mutex> lk(rec.m);
    EXPECT_EQ(rec.destroyed.size(), 0u);
}

// @implements REQ-CAPI-OBJCHANGE-002
// @covers AC-4
// Clone is rejected after the first step, so this case attaches and clones
// before any step. Initial population is also silent; the assertion is that
// the clone itself adds no callbacks on top of that.
TEST_F(PhysXTestFixture, CloneDeliversNoObjectChangeCallbacks) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    cb.on_object_destroyed = recDestroyed;
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    guard.handle = m_handle;
    ASSERT_EQ(ovphysx_subscribe_object_changes(m_handle, &cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/basic_simulation.usda"));

    const char* sourcePath = "/World/envs/env0";
    const char* targetPath = "/World/envs/envObjChangeClone";
    const ovphysx_string_t source = { sourcePath, std::strlen(sourcePath) };
    ovphysx_string_t target = { targetPath, std::strlen(targetPath) };
    ovphysx_enqueue_result_t clone_res = ovphysx_clone(m_handle, source, &target, 1, nullptr, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, clone_res));

    EXPECT_EQ(rec.allDestroyedCount.load(), 0);
    std::lock_guard<std::mutex> lk(rec.m);
    EXPECT_EQ(rec.created.size(), 0u);
    EXPECT_EQ(rec.destroyed.size(), 0u);
}


// ============================================================================
// C++ RAII wrapper
// ============================================================================

TEST_F(ObjectChangeIntegrationTest, CppSubscriptionAutoUnsubscribes) {
    EventRecorder rec;
    ovphysx_subscription_id_t leakedId = OVPHYSX_INVALID_SUBSCRIPTION_ID;

    {
        ovphysx::ObjectChangeCallbacks cbs;
        cbs.onCreated = [&rec](std::string_view path, ovphysx_physx_type_t t) {
            ovphysx_string_t s = { path.data(), path.size() };
            rec.recordCreated(s, t);
        };
        auto sub = ovphysx::subscribeObjectChanges(m_handle, std::move(cbs));
        ASSERT_TRUE(sub.isActive());
        leakedId = sub.id();
        // sub goes out of scope here and its destructor unsubscribes.
    }

    // Verify the underlying subscription is gone: a second unsubscribe must
    // return NOT_FOUND.
    ovphysx_result_t u = ovphysx_unsubscribe_object_changes(m_handle, leakedId);
    EXPECT_EQ(u.status, OVPHYSX_API_NOT_FOUND);
}

TEST_F(ObjectChangeIntegrationTest, CppSubscriptionMoveTransfersOwnership) {
    auto subA = ovphysx::subscribeObjectChanges(
        m_handle,
        ovphysx::ObjectChangeCallbacks{ /* onCreated */ [](std::string_view, ovphysx_physx_type_t){} });
    ASSERT_TRUE(subA.isActive());
    ovphysx_subscription_id_t id = subA.id();

    ovphysx::ObjectChangeSubscription subB(std::move(subA));
    EXPECT_FALSE(subA.isActive());
    EXPECT_TRUE(subB.isActive());
    EXPECT_EQ(subB.id(), id);
    // subB destructor unsubscribes.
}

TEST_F(PhysXTestFixture, CppSubscriptionReleasesStateAfterInstanceDestroy) {
    ovphysx_instance_handle_t owner = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&owner).status, OVPHYSX_API_SUCCESS);

    std::shared_ptr<int> captured = std::make_shared<int>(42);
    std::weak_ptr<int> weak = captured;
    {
        ovphysx::ObjectChangeCallbacks callbacks;
        callbacks.onCreated = [captured](std::string_view, ovphysx_physx_type_t) {};
        ovphysx::ObjectChangeSubscription subscription =
            ovphysx::subscribeObjectChanges(owner, std::move(callbacks));
        ASSERT_TRUE(subscription.isActive());
        callbacks = {};
        captured.reset();
        ASSERT_FALSE(weak.expired());

        ASSERT_EQ(ovphysx_destroy_instance(owner).status, OVPHYSX_API_SUCCESS);
        subscription.unsubscribe();
        EXPECT_TRUE(weak.expired());
    }
}
