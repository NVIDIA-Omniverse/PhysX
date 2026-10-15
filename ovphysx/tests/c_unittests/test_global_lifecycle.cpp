// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-3 AC-6 AC-7
 *
 * @implements REQ-PYTHON-LIFECYCLE-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-ATTACH-OWNER-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-4
 */

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx_test_utils.h"
#include "test_utilities.h"

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>

namespace
{
void expectSuccess(ovphysx_result_t result)
{
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
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
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);
}

TEST(GlobalLifecycle, InitializeShutdownInitializeAgain)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, StartupConfigUnlocksAfterRuntimeShutdown)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle));
    expectSuccess(ovphysx_destroy_instance(handle));

    EXPECT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH,
            ovphysx_config_value_string(OVPHYSX_LITERAL("before-shutdown.json"))).status,
        OVPHYSX_API_INVALID_STATE);

    expectSuccess(ovphysx_shutdown());
    expectSuccess(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH,
        ovphysx_config_value_string(OVPHYSX_LITERAL("after-shutdown.json"))));

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, CreateInstanceAfterShutdownRequiresReinitialize)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_create_instance(&handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, InitializeDoesNotCreateInstance)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, RecreateInstanceAfterLastDestroySucceeds)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t firstHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&firstHandle));
    ASSERT_NE(firstHandle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(firstHandle));

    ovphysx_instance_handle_t secondHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&secondHandle));
    ASSERT_NE(secondHandle, OVPHYSX_INVALID_HANDLE);
    expectSuccess(ovphysx_destroy_instance(secondHandle));

    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, RepeatedDestroyReturnsErrorAndLeavesLifecycleUsable)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t firstHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&firstHandle));
    ASSERT_NE(firstHandle, OVPHYSX_INVALID_HANDLE);

    ovphysx_instance_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&peerHandle));
    ASSERT_NE(peerHandle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(firstHandle));

    const ovphysx_enqueue_result_t peerTask = ovphysx_detach_ovstage(peerHandle);
    ASSERT_EQ(peerTask.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(peerTask.op_index, 0u);

    EXPECT_EQ(ovphysx_destroy_instance(firstHandle).status, OVPHYSX_API_ERROR);

    ovphysx_wait_task_result_t waitResult{};
    expectSuccess(ovphysx_wait_task(peerHandle, peerTask, UINT64_MAX, &waitResult));
    ovphysx_destroy_wait_task_result(&waitResult);
    expectSuccess(ovphysx_destroy_instance(peerHandle));

    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, ShutdownOnlyClearsInitializeStateWhileInstanceIsLive)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle));
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
    size_t warningTestCalls = 0;
};

void replacementCallback(
    ovphysx_log_level_t level,
    ovphysx_string_t message,
    ovphysx_string_t,
    double,
    void* userData)
{
    ReplacementCallbackState* state = static_cast<ReplacementCallbackState*>(userData);
    std::lock_guard<std::mutex> lock(state->mutex);
    ++state->calls;
    if (level == OVPHYSX_LOG_WARNING && message.ptr &&
        std::string(message.ptr, message.length) == "[LogTest] WARNING test message")
    {
        ++state->warningTestCalls;
    }
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

TEST(GlobalLifecycle, PreInitLoggingConfigSurvivesLifecycleCycles)
{
    ovphysx_config_value_t originalOutput{};
    ASSERT_EQ(ovphysx_get_global_config(
        OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT, &originalOutput).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(originalOutput.type, OVPHYSX_CONFIG_VALUE_BOOL);

    ReplacementCallbackState states[2];
    const ScopedLogCallbackDisable callbackGuard;
    struct Cleanup
    {
        uint32_t logLevel;
        bool defaultOutput;
        bool initialized = false;
        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;

        ~Cleanup()
        {
            if (handle != OVPHYSX_INVALID_HANDLE)
                (void)ovphysx_destroy_instance(handle);
            if (initialized)
                (void)ovphysx_shutdown();
            (void)ovphysx_set_log_level(logLevel);
            (void)ovphysx_enable_default_log_output(defaultOutput);
        }
    } cleanup{ovphysx_get_log_level(), originalOutput.value.bool_value};

    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_enable_default_log_output(false).status, OVPHYSX_API_SUCCESS);

    size_t callsAfterShutdown[2] = {};
    for (size_t epoch = 0; epoch < 2; ++epoch)
    {
        ReplacementCallbackState& state = states[epoch];
        ASSERT_EQ(ovphysx_set_log_callback(
            OVPHYSX_LOG_VERBOSE, nullptr, replacementCallback, &state).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
        cleanup.initialized = true;
        ASSERT_EQ(ovphysx_create_instance(&cleanup.handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(cleanup.handle, OVPHYSX_INVALID_HANDLE);

        EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_VERBOSE));
        ovphysx_config_value_t output{};
        ASSERT_EQ(ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT, &output).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(output.type, OVPHYSX_CONFIG_VALUE_BOOL);
        EXPECT_FALSE(output.value.bool_value);

        size_t warningCallsBeforeEmission = 0;
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            warningCallsBeforeEmission = state.warningTestCalls;
        }
        ovphysx_log_emit_test_messages();
        {
            std::unique_lock<std::mutex> lock(state.mutex);
            EXPECT_TRUE(state.cv.wait_for(lock, std::chrono::seconds(5),
                [&state, warningCallsBeforeEmission] { return state.warningTestCalls > warningCallsBeforeEmission; }));
        }
        expectSuccess(ovphysx_flush_log(UINT64_MAX));

        ASSERT_EQ(ovphysx_destroy_instance(cleanup.handle).status, OVPHYSX_API_SUCCESS);
        cleanup.handle = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
        cleanup.initialized = false;

        EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_VERBOSE));
        ASSERT_EQ(ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT, &output).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(output.type, OVPHYSX_CONFIG_VALUE_BOOL);
        EXPECT_FALSE(output.value.bool_value);
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            callsAfterShutdown[epoch] = state.calls;
        }
        ovphysx_log_emit_test_messages();
        expectSuccess(ovphysx_flush_log(UINT64_MAX));
        for (size_t retired = 0; retired <= epoch; ++retired)
        {
            std::lock_guard<std::mutex> lock(states[retired].mutex);
            EXPECT_EQ(states[retired].calls, callsAfterShutdown[retired])
                << "Shutdown must keep the retired callback disabled across later epochs";
        }
    }
}

TEST(GlobalLifecycle, ShutdownDrainsAcceptedCallback)
{
    expectSuccess(ovphysx_initialize());
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle));
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

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle));
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
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
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
    ShutdownDestroyState state;
    expectSuccess(ovphysx_create_instance(&state.handle));
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

// An uncollected failed task must not prevent instance teardown from releasing
// that instance's runtime attachment. A later instance must remain able to attach.
TEST(GlobalLifecycle, DestroyAfterFailedPendingOpReleasesLiveAttachOwnerAcrossInstancesAndShutdown)
{
    expectSuccess(ovphysx_initialize());

    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";

    // Leave A's accepted failure uncollected until instance teardown.
    ovphysx_instance_handle_t handle_a = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle_a));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_string_t target = OVPHYSX_LITERAL("/World/TaskCloneTarget");
    const ovphysx_enqueue_result_t failed_task = ovphysx_clone(
        handle_a, OVPHYSX_LITERAL("/World/TaskMissingSource"), &target, 1, nullptr, nullptr);
    ASSERT_EQ(failed_task.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(failed_task.op_index, 0u);

    // Destroy A without collecting its failure; its runtime attachment must still be released.
    expectSuccess(ovphysx_destroy_instance(handle_a));

    // Instance B must be able to attach: the latch must not still be held by
    // the now-destroyed A.
    ovphysx_instance_handle_t handle_b = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle_b));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_b, usd_path))
        << "B's attach must succeed once A's failed destroy released A's runtime attachment";
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    test_utils::destroy_ovstage_test_attachments(handle_b);
    expectSuccess(ovphysx_destroy_instance(handle_b));

    // A full shutdown/reinitialize cycle must also stay clean: no instances
    // are alive at this point, so ovphysx_shutdown() drains the runtime, and
    // a fresh instance can attach normally afterward.
    expectSuccess(ovphysx_shutdown());
    expectSuccess(ovphysx_initialize());

    ovphysx_instance_handle_t handle_c = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&handle_c));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle_c, usd_path))
        << "A fresh instance after shutdown/reinitialize must be able to attach";
    EXPECT_EQ(ovphysx_step_sync(handle_c, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    test_utils::destroy_ovstage_test_attachments(handle_c);
    expectSuccess(ovphysx_destroy_instance(handle_c));

    expectSuccess(ovphysx_shutdown());
}

// Existing diagnostic from test_log_level.cpp emits a real sidecar error.
void* unresolvedSidecarPointer();

TEST(GlobalLifecycle, SidecarLogPolicySurvivesFrameworkRestart)
{
    const uint32_t previousLevel = ovphysx_get_log_level();
    const char* diagnostic = "Internal sidecar: ovphysx_internal_get_physx_ptr could not resolve ObjectKey";
    for (int cycle = 0; cycle < 3; ++cycle)
    {
        SCOPED_TRACE(cycle);
        expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_NONE));
        expectSuccess(ovphysx_initialize());
        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
        for (const uint32_t level : {OVPHYSX_LOG_NONE, OVPHYSX_LOG_INFO, OVPHYSX_LOG_NONE})
        {
            SCOPED_TRACE(level);
            expectSuccess(ovphysx_set_log_level(level));
            expectSuccess(ovphysx_log_capture_start());
            EXPECT_EQ(unresolvedSidecarPointer(), nullptr);
            expectSuccess(ovphysx_flush_log(UINT64_MAX));
            EXPECT_EQ(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, diagnostic), level != OVPHYSX_LOG_NONE);
            ovphysx_log_capture_stop();
        }
        expectSuccess(ovphysx_destroy_instance(handle));
        expectSuccess(ovphysx_shutdown());
    }
    expectSuccess(ovphysx_set_log_level(previousLevel));
}

TEST(GlobalLifecycle, CallbackAfterLiveShutdownWaitsForNextBootstrap)
{
    expectSuccess(ovphysx_initialize());
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    expectSuccess(ovphysx_shutdown());

    ReplacementCallbackState state;
    const ScopedLogCallbackDisable callbackGuard;
    const uint32_t previousLevel = ovphysx_get_log_level();
    expectSuccess(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE));
    expectSuccess(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, replacementCallback, &state));
    ovphysx_log_emit_test_messages();
    expectSuccess(ovphysx_flush_log(UINT64_MAX));
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        EXPECT_EQ(state.calls, 0u) << "A pending callback must not register with stopped SDK logging";
    }
    expectSuccess(ovphysx_destroy_instance(handle));

    expectSuccess(ovphysx_initialize());
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();
    expectSuccess(ovphysx_flush_log(UINT64_MAX));
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        EXPECT_TRUE(state.cv.wait_for(lock, std::chrono::seconds(5),
            [&state] { return state.warningTestCalls != 0; }));
    }
    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
    expectSuccess(ovphysx_set_log_level(previousLevel));
}
