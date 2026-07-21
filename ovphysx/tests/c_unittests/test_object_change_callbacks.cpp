// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Tests for ovphysx_subscribe_object_changes / ovphysx_unsubscribe_object_changes.
//
// Coverage:
//   - ABI / argument-validation: NULL callbacks, NULL out_subscription,
//     all-NULL callback functions, invalid subscription on unsubscribe,
//     unknown subscription returns NOT_FOUND, subscribe/unsubscribe roundtrip.
//   - Integration: all-destroyed callback fires on ovphysx_reset_stage();
//     unsubscribe stops delivery of subsequent events; the initial stage
//     population delivers no per-object created/destroyed callbacks
//     (NVBugs 6473870).
//   - C++ RAII wrapper: destructor auto-unsubscribes, move transfers
//     ownership and leaves the source inactive.
//
// Subscriptions are process-global -- callbacks fire for any attached stage
// in the process. Tests use the shared CPU PhysXTestFixture and rely on
// ovphysx_reset_stage() in fixture teardown to clean state between tests.
//
// Known gap: ovphysx_clone() does not currently emit object_created
// notifications (upstream omni.physx replicator bypasses the notification
// path). When that's wired up, add a regression test for it.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/experimental/ovphysx.hpp"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

using namespace test_utils;

namespace {

bool wait_op_success(ovphysx_handle_t handle, ovphysx_op_index_t op_index,
                     uint64_t timeout_ns = 10'000'000'000ULL) {
    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_op(handle, op_index, timeout_ns, &wait_result);
    // The wait can return SUCCESS while individual ops failed; surface those
    // before destroying the wait result so test failures aren't silent.
    bool any_op_failed = wait_result.num_errors > 0;
    if (any_op_failed && wait_result.error_op_indices) {
        for (uint32_t i = 0; i < wait_result.num_errors; ++i) {
            ovphysx_string_t err = ovphysx_get_last_op_error(wait_result.error_op_indices[i]);
            if (err.ptr && err.length > 0) {
                std::cerr << "op error: " << std::string(err.ptr, err.length) << std::endl;
            }
        }
    }
    ovphysx_destroy_wait_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS && !any_op_failed;
}

bool load_usd_and_wait(ovphysx_handle_t handle, const char* usd_path,
                       ovphysx_usd_handle_t& out_handle) {
    out_handle = 1;
    return attach_usd_with_ovstage(handle, usd_path);
}

bool step_and_wait(ovphysx_handle_t handle, float dt) {
    ovphysx_enqueue_result_t res = ovphysx_step(handle, dt);
    return res.status == OVPHYSX_API_SUCCESS && wait_op_success(handle, res.op_index);
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

// RAII guard for a raw C subscription. Ensures unsubscribe is called even if
// an ASSERT_* aborts the test scope -- otherwise the IPhysx subscription
// would outlive the stack EventRecorder it captures, and fixture teardown
// reset() would dispatch the all-destroyed callback into freed memory.
struct ScopedSubscription {
    ovphysx_subscription_id_t id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ~ScopedSubscription() {
        if (id != OVPHYSX_INVALID_SUBSCRIPTION_ID) {
            ovphysx_unsubscribe_object_changes(id);
        }
    }
};

} // anonymous namespace


// ============================================================================
// Argument-validation tests
// ============================================================================

TEST_F(PhysXTestFixture, ObjectChangeSubscribeNullCallbacks) {
    ovphysx_subscription_id_t sub = 12345;  // sentinel; must be overwritten to INVALID
    ovphysx_result_t r = ovphysx_subscribe_object_changes(nullptr, &sub);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);
}

TEST_F(PhysXTestFixture, ObjectChangeSubscribeNullOutId) {
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    ovphysx_result_t r = ovphysx_subscribe_object_changes(&cb, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, ObjectChangeSubscribeAllNullCallbacks) {
    ovphysx_object_change_callbacks_t cb{};  // every fn-ptr is NULL
    ovphysx_subscription_id_t sub = 0;
    ovphysx_result_t r = ovphysx_subscribe_object_changes(&cb, &sub);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);
}

TEST_F(PhysXTestFixture, ObjectChangeUnsubscribeInvalidIdRejected) {
    ovphysx_result_t r = ovphysx_unsubscribe_object_changes(OVPHYSX_INVALID_SUBSCRIPTION_ID);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, ObjectChangeUnsubscribeUnknownIdReturnsNotFound) {
    // Pick a value that is unlikely to collide with a real IPhysx subscription
    // (which uses small monotonic IDs). A double-unsubscribe / typo from the
    // user should not crash and should be observable as NOT_FOUND.
    ovphysx_result_t r = ovphysx_unsubscribe_object_changes(0xDEADBEEFULL);
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
    ovphysx_result_t r = ovphysx_subscribe_object_changes(&cb, &sub);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(sub, OVPHYSX_INVALID_SUBSCRIPTION_ID);

    ovphysx_result_t u = ovphysx_unsubscribe_object_changes(sub);
    EXPECT_EQ(u.status, OVPHYSX_API_SUCCESS);

    // Second unsubscribe on the same ID must be NOT_FOUND, not crash.
    ovphysx_result_t u2 = ovphysx_unsubscribe_object_changes(sub);
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
        // First step attaches PhysX to the stage so that ovphysx_reset_stage() has
        // objects to tear down. The omni.physx "sim started" gate that
        // normally suppresses pre-simulation events is bypassed by the clone
        // plugin setting stopCallbackWhenSimStopped=false on subscription;
        // see ovphysxClone.cpp.
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

// NOTE: ovphysx_clone() does NOT fire object_created notifications today.
// The clone path goes through IPhysxReplicator, which builds PhysX actors
// directly rather than re-entering PhysXUsdPhysicsInterface::createObject()
// where sendObjectCreationNotification() lives. Tracked as a known limitation
// of the omni.physx side; covered by the docstring on
// ovphysx_subscribe_object_changes (in ovphysx.h). When/if upstream wires
// notifications into the replicator path, add a regression test here.

// ovphysx_reset_stage() tears everything down in bulk. Subscribers must receive
// the all-destroyed notification rather than N per-object destructions.
TEST_F(ObjectChangeIntegrationTest, ResetTriggersAllDestroyedNotification) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    ASSERT_EQ(ovphysx_subscribe_object_changes(&cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_res = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_op_success(m_handle, reset_res.op_index));

    EXPECT_GE(rec.allDestroyedCount.load(), 1)
        << "ovphysx_reset_stage() should fire the all-objects-destroyed callback at least once";
    // guard destructor unsubscribes.
}

// After unsubscribe, no further events are delivered. Use ovphysx_reset_stage() as
// the stimulus -- ovphysx_clone() does NOT currently fire object-change
// notifications (see ovphysx_subscribe_object_changes docstring), so a
// post-unsubscribe clone is silent regardless and would make this assertion
// vacuous.
TEST_F(ObjectChangeIntegrationTest, UnsubscribeStopsDelivery) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ovphysx_subscription_id_t sub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(&cb, &sub).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_unsubscribe_object_changes(sub).status, OVPHYSX_API_SUCCESS);

    // reset() WOULD fire on_all_objects_destroyed if the subscription were
    // still active. After unsubscribe the callback count must stay at zero.
    ovphysx_enqueue_result_t reset_res = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_op_success(m_handle, reset_res.op_index));

    EXPECT_EQ(rec.allDestroyedCount.load(), 0)
        << "No callbacks should be delivered after unsubscribe";
}

// Regression for NVBugs 6473870 / OMPE-102206. Subscribing before the initial
// stage population and then attaching a fresh scene must NOT deliver any
// per-object created callbacks: ovphysx sets stopCallbackWhenSimStopped=false
// (to receive reset/clone events while stopped), which used to bypass the
// initial-population suppression and leak one created event per actor/shape.
// The caller already has that state from setup, so the count must be zero.
//
// This is the shipped C-API path QA exercised, on the base fixture (which does
// not pre-attach), so the subscription is active before attach_ovstage().
TEST_F(PhysXTestFixture, InitialPopulationDeliversNoCreatedCallbacks) {
    EventRecorder rec;
    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = recCreated;
    cb.on_object_destroyed = recDestroyed;
    cb.on_all_objects_destroyed = recAllDestroyed;
    cb.user_data = &rec;

    ScopedSubscription guard;
    ASSERT_EQ(ovphysx_subscribe_object_changes(&cb, &guard.id).status, OVPHYSX_API_SUCCESS);

    // Attach + initially populate a fresh scene while subscribed.
    ovphysx_usd_handle_t usd = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/basic_simulation.usda", usd))
        << "Failed to load scene";
    // First step lets any deferred initial population run; it is still the
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
        auto sub = ovphysx::subscribeObjectChanges(std::move(cbs));
        ASSERT_TRUE(sub.isActive());
        leakedId = sub.id();
        // sub goes out of scope here -> destructor unsubscribes
    }

    // Verify the underlying subscription is gone: a second unsubscribe must
    // return NOT_FOUND.
    ovphysx_result_t u = ovphysx_unsubscribe_object_changes(leakedId);
    EXPECT_EQ(u.status, OVPHYSX_API_NOT_FOUND);
}

TEST_F(ObjectChangeIntegrationTest, CppSubscriptionMoveTransfersOwnership) {
    auto subA = ovphysx::subscribeObjectChanges(
        ovphysx::ObjectChangeCallbacks{ /* onCreated */ [](std::string_view, ovphysx_physx_type_t){} });
    ASSERT_TRUE(subA.isActive());
    ovphysx_subscription_id_t id = subA.id();

    ovphysx::ObjectChangeSubscription subB(std::move(subA));
    EXPECT_FALSE(subA.isActive());
    EXPECT_TRUE(subB.isActive());
    EXPECT_EQ(subB.id(), id);
    // subB destructor unsubscribes.
}
