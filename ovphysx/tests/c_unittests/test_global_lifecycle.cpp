// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-3 AC-6
 *
 * @implements REQ-PYTHON-LIFECYCLE-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-ATTACH-OWNER-001
 * @covers AC-3
 */

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"
#include "ovphysx_test_utils.h"
#include "test_utilities.h"

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

namespace
{
void expectSuccess(ovphysx_result_t result)
{
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

ovphysx_result_t successfulPeerTask(ovphysx_handle_t, ovphysx_op_index_t, void*)
{
    return { OVPHYSX_API_SUCCESS };
}
}

TEST(GlobalLifecycle, SecondInitializeBeforeShutdownReturnsError)
{
    expectSuccess(ovphysx_initialize());
    EXPECT_EQ(ovphysx_initialize().status, OVPHYSX_API_ERROR);
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, ShutdownClearsInitializeState)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, ShutdownWithoutInitializeReturnsError)
{
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, ExtraShutdownAfterBalancedInitializeReturnsError)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, CreateInstanceBeforeInitializeReturnsError)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);
}

TEST(GlobalLifecycle, InitializeShutdownInitializeAgain)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, CreateInstanceAfterShutdownRequiresReinitialize)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, InitializeDoesNotCreateInstance)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, RecreateInstanceAfterLastDestroySucceeds)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t firstHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &firstHandle));
    ASSERT_NE(firstHandle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(firstHandle));

    ovphysx_handle_t secondHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &secondHandle));
    ASSERT_NE(secondHandle, OVPHYSX_INVALID_HANDLE);
    expectSuccess(ovphysx_destroy_instance(secondHandle));

    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, RepeatedDestroyReturnsErrorAndLeavesLifecycleUsable)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t firstHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &firstHandle));
    ASSERT_NE(firstHandle, OVPHYSX_INVALID_HANDLE);

    ovphysx_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &peerHandle));
    ASSERT_NE(peerHandle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(firstHandle));

    ovphysx_user_task_desc_t taskDesc{};
    taskDesc.run = successfulPeerTask;
    const ovphysx_enqueue_result_t peerTask = ovphysx_add_user_task(peerHandle, &taskDesc);
    ASSERT_EQ(peerTask.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(peerTask.op_index, 0u);

    EXPECT_EQ(ovphysx_destroy_instance(firstHandle).status, OVPHYSX_API_ERROR);

    ovphysx_op_wait_result_t waitResult{};
    expectSuccess(ovphysx_wait_op(peerHandle, peerTask.op_index, UINT64_MAX, &waitResult));
    ovphysx_destroy_wait_result(&waitResult);
    expectSuccess(ovphysx_destroy_instance(peerHandle));

    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, ShutdownOnlyClearsInitializeStateWhileInstanceIsLive)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_shutdown());
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);

    expectSuccess(ovphysx_destroy_instance(handle));
}

namespace
{
struct ShutdownDrainState
{
    std::mutex mutex;
    std::condition_variable cv;
    size_t calls = 0;
    bool entered = false;
    bool release = false;
};

void shutdownDrainCallback(
    ovphysx_log_level_t,
    ovphysx_string_t,
    ovphysx_string_t,
    double,
    void* userData)
{
    ShutdownDrainState* state = static_cast<ShutdownDrainState*>(userData);
    std::unique_lock<std::mutex> lock(state->mutex);
    ++state->calls;
    state->entered = true;
    state->cv.notify_all();
    state->cv.wait(lock, [state] { return state->release; });
}

struct ReplacementCallbackState
{
    std::mutex mutex;
    std::condition_variable cv;
    size_t calls = 0;
};

void replacementCallback(
    ovphysx_log_level_t,
    ovphysx_string_t,
    ovphysx_string_t,
    double,
    void* userData)
{
    ReplacementCallbackState* state = static_cast<ReplacementCallbackState*>(userData);
    std::lock_guard<std::mutex> lock(state->mutex);
    ++state->calls;
    state->cv.notify_all();
}

class ScopedLogCallbackDisable
{
public:
    ~ScopedLogCallbackDisable()
    {
        (void)ovphysx_set_log_callback(
            OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    }
};
}

TEST(GlobalLifecycle, ShutdownDrainsAcceptedCallback)
{
    expectSuccess(ovphysx_initialize());
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);
    expectSuccess(ovphysx_destroy_instance(handle));

    ShutdownDrainState state;
    ReplacementCallbackState replacementState;
    const ScopedLogCallbackDisable callbackGuard;
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE));
    expectSuccess(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, shutdownDrainCallback, &state));

    std::thread producer([] { ovphysx_log_emit_test_messages(); });
    bool entered = false;
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        entered = state.cv.wait_for(
            lock, std::chrono::seconds(5), [&state] { return state.entered; });
        if (!entered)
            state.release = true;
    }
    if (!entered)
    {
        state.cv.notify_all();
        producer.join();
        ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        ovphysx_shutdown();
        FAIL() << "Callback did not enter within five seconds";
        return;
    }

    std::promise<void> replacementStarted;
    std::future<void> replacementStartedFuture = replacementStarted.get_future();
    std::future<ovphysx_result_t> replacement = std::async(
        std::launch::async,
        [&replacementStarted, &replacementState]
        {
            replacementStarted.set_value();
            return ovphysx_set_log_callback(
                OVPHYSX_LOG_VERBOSE, nullptr, replacementCallback, &replacementState);
        });
    replacementStartedFuture.wait();
    EXPECT_EQ(replacement.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

    // Observe the newly published registration before starting shutdown. The
    // replacement publishes it before draining the old callback, so this is a
    // deterministic handshake that it owns the in-progress transition.
    std::future<void> replacementProbe =
        std::async(std::launch::async, [] { ovphysx_log_emit_test_messages(); });
    bool replacementObserved = false;
    {
        std::unique_lock<std::mutex> lock(replacementState.mutex);
        replacementObserved = replacementState.cv.wait_for(
            lock, std::chrono::seconds(5), [&replacementState] { return replacementState.calls != 0; });
    }
    if (!replacementObserved)
    {
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.release = true;
        }
        state.cv.notify_all();
        producer.join();
        replacementProbe.wait();
        replacement.wait();
        ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        ovphysx_shutdown();
        FAIL() << "Replacement callback was not published within five seconds";
        return;
    }
    replacementProbe.get();

    std::promise<void> shutdownStarted;
    std::future<void> shutdownStartedFuture = shutdownStarted.get_future();
    std::future<ovphysx_result_t> shutdown = std::async(
        std::launch::async,
        [&shutdownStarted]
        {
            shutdownStarted.set_value();
            return ovphysx_shutdown();
        });
    shutdownStartedFuture.wait();
    EXPECT_EQ(shutdown.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.release = true;
    }
    state.cv.notify_all();
    producer.join();
    EXPECT_EQ(replacement.get().status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(shutdown.get().status, OVPHYSX_API_SUCCESS);

    size_t callsAfterShutdown = 0;
    {
        std::lock_guard<std::mutex> lock(replacementState.mutex);
        callsAfterShutdown = replacementState.calls;
    }
    ovphysx_log_emit_test_messages();
    expectSuccess(ovphysx_flush_log(UINT64_MAX));
    {
        std::lock_guard<std::mutex> lock(replacementState.mutex);
        EXPECT_EQ(replacementState.calls, callsAfterShutdown)
            << "Successful shutdown must disable the callback before returning";
    }
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_WARNING));
}

TEST(GlobalLifecycle, LiveHandleShutdownEndsCallbackDelivery)
{
    expectSuccess(ovphysx_initialize());
    ReplacementCallbackState state;
    const ScopedLogCallbackDisable callbackGuard;
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE));
    expectSuccess(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, replacementCallback, &state));

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);
    ovphysx_log_emit_test_messages();
    expectSuccess(ovphysx_flush_log(UINT64_MAX));

    size_t callsBeforeShutdown = 0;
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        callsBeforeShutdown = state.calls;
    }
    ASSERT_NE(callsBeforeShutdown, 0u);

    expectSuccess(ovphysx_shutdown());
    // Instance destruction emits a production INFO record. Successful shutdown
    // ended application delivery before this still-owned handle was destroyed.
    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_flush_log(UINT64_MAX));
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        EXPECT_EQ(state.calls, callsBeforeShutdown)
            << "Live-handle shutdown must end application callback delivery";
    }

    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_WARNING));
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

namespace
{
struct ShutdownDestroyState
{
    std::mutex mutex;
    std::condition_variable cv;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_result_t destroyResult = { OVPHYSX_API_ERROR };
    bool entered = false;
    bool destroyRequested = false;
    bool destroyFinished = false;
};

void shutdownDestroyCallback(
    ovphysx_log_level_t,
    ovphysx_string_t,
    ovphysx_string_t,
    double,
    void* userData)
{
    ShutdownDestroyState* state = static_cast<ShutdownDestroyState*>(userData);
    {
        std::unique_lock<std::mutex> lock(state->mutex);
        if (state->entered)
            return;
        state->entered = true;
        state->cv.notify_all();
        if (!state->cv.wait_for(
                lock, std::chrono::seconds(5), [state] { return state->destroyRequested; }))
        {
            return;
        }
    }

    const ovphysx_result_t result = ovphysx_destroy_instance(state->handle);
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->destroyResult = result;
        state->destroyFinished = true;
    }
    state->cv.notify_all();
}

}

TEST(GlobalLifecycle, LiveInstanceShutdownDrainDoesNotHoldInstanceMapLock)
{
    expectSuccess(ovphysx_initialize());
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ShutdownDestroyState state;
    expectSuccess(ovphysx_create_instance(&args, &state.handle));
    ASSERT_NE(state.handle, OVPHYSX_INVALID_HANDLE);
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE));
    const ScopedLogCallbackDisable callbackGuard;
    expectSuccess(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, shutdownDestroyCallback, &state));

    std::thread producer([] { ovphysx_log_emit_test_messages(); });
    bool entered = false;
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        entered = state.cv.wait_for(
            lock, std::chrono::seconds(5), [&state] { return state.entered; });
        if (!entered)
            state.destroyRequested = true;
    }
    if (!entered)
    {
        state.cv.notify_all();
        producer.join();
        ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        ovphysx_destroy_instance(state.handle);
        ovphysx_shutdown();
        FAIL() << "Callback did not enter within five seconds";
        return;
    }

    std::promise<void> shutdownStarted;
    std::future<void> shutdownStartedFuture = shutdownStarted.get_future();
    std::future<ovphysx_result_t> shutdown = std::async(
        std::launch::async,
        [&shutdownStarted]
        {
            shutdownStarted.set_value();
            return ovphysx_shutdown();
        });
    shutdownStartedFuture.wait();
    EXPECT_EQ(shutdown.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.destroyRequested = true;
    }
    state.cv.notify_all();

    {
        std::unique_lock<std::mutex> lock(state.mutex);
        EXPECT_TRUE(state.cv.wait_for(
            lock, std::chrono::seconds(5), [&state] { return state.destroyFinished; }));
    }
    producer.join();
    ASSERT_EQ(shutdown.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_EQ(shutdown.get().status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(state.destroyResult.status, OVPHYSX_API_SUCCESS);
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_WARNING));

    // Balance the runtime reference released by the callback-side destroy.
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

// Regression (MR !8218, REQ-CAPI-ATTACH-OWNER-001 AC-3): a permanently failed
// pending op (never observed via ovphysx_wait_op(), so
// wait_for_all_pending_ops() keeps failing) must not make
// omni_sdk_physx_destroy()'s internal ovphysx_detach_ovstage() call return
// before releasing g_liveAttachOwner. Otherwise the process-wide live-attach
// latch stays attributed to the destroyed handle and every later instance's
// attach_ovstage() is rejected.
TEST(GlobalLifecycle, DestroyAfterFailedPendingOpReleasesLiveAttachOwnerAcrossInstancesAndShutdown)
{
    expectSuccess(ovphysx_initialize());

    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";

    // Instance A: attach ovstage, then leave a permanently-failed op in its
    // pending-op tracking. wait_for_all_pending_ops() never consumes a failed
    // op, it stays until observed by wait_op(), which this test never does.
    ovphysx_create_args args_a = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_a = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args_a, &handle_a));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_user_task_desc_t failing_task{};
    failing_task.run = [](ovphysx_handle_t, ovphysx_op_index_t, void*) -> ovphysx_result_t {
        return { OVPHYSX_API_ERROR };
    };
    ovphysx_enqueue_result_t failed_task = ovphysx_add_user_task(handle_a, &failing_task);
    ASSERT_EQ(failed_task.status, OVPHYSX_API_ERROR);
    ASSERT_NE(failed_task.op_index, 0u);

    // Destroy A without ever observing the failed op: destroy's own
    // wait_for_all_pending_ops() and the ovphysx_detach_ovstage() call it
    // makes both fail. Destroy must still fully release A's ownership.
    expectSuccess(ovphysx_destroy_instance(handle_a));

    // Instance B must be able to attach: the latch must not still be held by
    // the now-destroyed A.
    ovphysx_create_args args_b = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_b = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args_b, &handle_b));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_b, usd_path))
        << "B's attach must succeed once A's failed destroy released the live-attach latch";
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    test_utils::destroy_ovstage_test_attachments(handle_b);
    expectSuccess(ovphysx_destroy_instance(handle_b));

    // A full shutdown/reinitialize cycle must also stay clean: no instances
    // are alive at this point, so ovphysx_shutdown() drains the runtime, and
    // a fresh instance can attach normally afterward.
    expectSuccess(ovphysx_shutdown());
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args_c = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_c = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args_c, &handle_c));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_c, usd_path))
        << "A fresh instance after shutdown/reinitialize must be able to attach";
    EXPECT_EQ(ovphysx_step_sync(handle_c, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    test_utils::destroy_ovstage_test_attachments(handle_c);
    expectSuccess(ovphysx_destroy_instance(handle_c));

    expectSuccess(ovphysx_shutdown());
}
