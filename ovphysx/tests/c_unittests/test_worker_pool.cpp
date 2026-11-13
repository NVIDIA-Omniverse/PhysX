// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-THREADING-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-5 AC-7 AC-11
 */

// Worker-pool sizing is decided once per process, when the first instance
// starts carb.tasking, so each case below needs a process where no earlier test
// has started the pool. Both suites are routed to their own gtest passes in
// scripts/test_cpp.cmake and excluded from the shared cpu pass.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>
#include <gtest/gtest.h>

#include "ovphysxTestHelpers.h"
#include "ovphysx_test_utils.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace
{

constexpr const char* kTaskingThreadCountPath = "/plugins/carb.tasking.plugin/threadCount";
constexpr const char* kTbbThreadCountPath = "/plugins/omni.tbb.globalcontrol/maxThreadCount";
constexpr const char* kSlashlessTaskingThreadCountPath = "plugins/carb.tasking.plugin/threadCount";
constexpr const char* kSlashlessTbbThreadCountPath = "plugins/omni.tbb.globalcontrol/maxThreadCount";

struct WorkerPoolCounts
{
    uint32_t taskingThreads{ 0 };
    int32_t taskingSetting{ 0 };
    int32_t tbbSetting{ 0 };
    uint32_t hardwareConcurrency{ 0 };
};

bool readWorkerPoolCounts(WorkerPoolCounts& counts)
{
    return ovphysx_get_worker_pool_counts_for_test_internal(
        &counts.taskingThreads, &counts.taskingSetting, &counts.tbbSetting, &counts.hardwareConcurrency);
}

ovphysx_config_entry_t carboniteIntEntry(const char* path, const char* value)
{
    return ovphysx_config_entry_carbonite(
        ovphysx_string_t{ path, std::char_traits<char>::length(path) },
        ovphysx_string_t{ value, std::char_traits<char>::length(value) });
}

void recordThread(void* user)
{
    *static_cast<std::thread::id*>(user) = std::this_thread::get_id();
}

// Counts the warnings that an instance runs synchronously on a one-thread pool.
void countSynchronousWarnings(ovphysx_log_level_t, ovphysx_string_t message, ovphysx_string_t, double, void* user)
{
    const std::string text = message.ptr ? std::string(message.ptr, message.length) : std::string();
    if (text.find("asynchronous execution disabled for this pool size (1 threads)") != std::string::npos)
        ++*static_cast<std::atomic<int>*>(user);
}

} // namespace

// AC-1: ovphysx imposes no worker count, so an unconfigured pool stays sized to
// the machine. A bounded default measured slower on GPU-heavy replicated scenes.
TEST(WorkerPoolDefaultTest, UnconfiguredPoolsKeepTheCarboniteDefault)
{
    // These passes run with OVPHYSX_TEST_LIFECYCLE_OWNS_INIT, so the case owns
    // initialization and nothing has configured the pools before it.
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    WorkerPoolCounts counts;
    ASSERT_TRUE(readWorkerPoolCounts(counts)) << "carb.tasking must be running after create_instance";

    ASSERT_GT(counts.hardwareConcurrency, 0u);
    EXPECT_EQ(counts.taskingSetting, 0)
        << "ovphysx must write no tasking worker default; zero leaves the pool sized to the machine";
    EXPECT_EQ(counts.tbbSetting, 0) << "ovphysx must write no TBB worker default";
    EXPECT_GE(counts.taskingThreads, 1u);
    EXPECT_GT(
        tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism),
        0u);

    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

// AC-2 / AC-3: create-time overrides reach both pools before they start, and
// AC-4: a later attempt to resize either one fails instead of silently no-opping.
TEST(WorkerPoolOverrideTest, ExplicitCountsStartThePoolsAndLaterChangesAreRejected)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);

    const ovphysx_config_entry_t entries[] = {
        carboniteIntEntry(kTaskingThreadCountPath, "4"),
        carboniteIntEntry(kTbbThreadCountPath, "5"),
    };

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries;
    args.config_entry_count = static_cast<uint32_t>(std::size(entries));

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance_with_args(&args, &handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    WorkerPoolCounts counts;
    ASSERT_TRUE(readWorkerPoolCounts(counts));
    EXPECT_EQ(counts.taskingThreads, 4u) << "the requested worker count must be the one the pool started with";
    EXPECT_EQ(counts.tbbSetting, 5);
    EXPECT_EQ(
        tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism),
        5u)
        << "the requested TBB limit must be active, not only stored in ISettings";

    // The pool is running now, so neither spelling at either entry point may
    // accept a new size.
    const ovphysx_config_entry_t lateTasking = carboniteIntEntry(kTaskingThreadCountPath, "3");
    const ovphysx_config_entry_t lateTbb = carboniteIntEntry(kTbbThreadCountPath, "3");
    const ovphysx_config_entry_t lateSlashlessTasking =
        carboniteIntEntry(kSlashlessTaskingThreadCountPath, "3");
    const ovphysx_config_entry_t lateSlashlessTbb =
        carboniteIntEntry(kSlashlessTbbThreadCountPath, "3");
    EXPECT_EQ(ovphysx_set_global_config_entry(lateTasking).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_set_global_config_entry(lateTbb).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_set_global_config_entry(lateSlashlessTasking).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_set_global_config_entry(lateSlashlessTbb).status, OVPHYSX_API_ERROR);

    ovphysx_create_args lateArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    lateArgs.config_entries = &lateTasking;
    lateArgs.config_entry_count = 1;
    ovphysx_instance_handle_t lateHandle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance_with_args(&lateArgs, &lateHandle).status, OVPHYSX_API_ERROR);

    lateArgs.config_entries = &lateSlashlessTasking;
    EXPECT_EQ(ovphysx_create_instance_with_args(&lateArgs, &lateHandle).status, OVPHYSX_API_ERROR);
    lateArgs.config_entries = &lateSlashlessTbb;
    EXPECT_EQ(ovphysx_create_instance_with_args(&lateArgs, &lateHandle).status, OVPHYSX_API_ERROR);

    // A rejected resize must leave the running pool untouched.
    WorkerPoolCounts afterRejection;
    ASSERT_TRUE(readWorkerPoolCounts(afterRejection));
    EXPECT_EQ(afterRejection.taskingThreads, 4u);
    EXPECT_EQ(afterRejection.tbbSetting, 5);

    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

// REQ-CAPI-CONFIG-001 AC-11: legacy entries set before the first instance are
// applied by that create, before plugins start the pool, and are not replayed
// by a later create.
TEST(WorkerPoolQueuedEntryTest, PreCreateEntriesApplyOnceBeforePluginsLoad)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    bool readback = false;
    ASSERT_EQ(ovphysx_test_get_settings_bool("/physics/suppressReadback", &readback).status, OVPHYSX_API_ERROR)
        << "Carbonite must not be running yet, or the entries below bypass the queue";

    const ovphysx_config_entry_t readbackOn = ovphysx_config_entry_carbonite(
        OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true"));
    const ovphysx_config_entry_t readbackOff = ovphysx_config_entry_carbonite(
        OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("false"));
    {
        // The queue owns its copies; the caller's strings end with this scope.
        const std::string path = kTaskingThreadCountPath;
        const std::string value = "3";
        ASSERT_EQ(ovphysx_set_global_config_entry(carboniteIntEntry(path.c_str(), value.c_str())).status,
                  OVPHYSX_API_SUCCESS);
    }
    ovphysx_config_entry_t invalidType{};
    invalidType.key_type = OVPHYSX_CONFIG_KEY_TYPE_COUNT;
    const ovphysx_config_entry_t invalidEntries[] = {
        invalidType,
        ovphysx_config_entry_bool(OVPHYSX_CONFIG_BOOL_COUNT, true),
        ovphysx_config_entry_int32(OVPHYSX_CONFIG_INT32_COUNT, 1),
        ovphysx_config_entry_float(OVPHYSX_CONFIG_FLOAT_COUNT, 1.0f),
        ovphysx_config_entry_string(OVPHYSX_CONFIG_STRING_COUNT, OVPHYSX_LITERAL("value")),
        ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/test/key\0suffix"), OVPHYSX_LITERAL("true")),
        ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/test/key"), OVPHYSX_LITERAL("true\0suffix")),
    };
    // Rejected entries must not poison creation or discard the valid queue.
    for (const ovphysx_config_entry_t& invalidEntry : invalidEntries)
        EXPECT_EQ(ovphysx_set_global_config_entry(invalidEntry).status, OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(ovphysx_set_global_config_entry(readbackOn).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_carbonite(
                  OVPHYSX_LITERAL("/physics/cudaDevice"), OVPHYSX_LITERAL("0"))).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(-1)).status,
              OVPHYSX_API_INVALID_ARGUMENT);

    bool contactProcessingDisabled = false;
    ASSERT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_disable_contact_processing(true)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING,
                                            &contactProcessingDisabled).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(contactProcessingDisabled) << "queued entries are not effective until create";

    ovphysx_instance_handle_t first = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&first).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING,
                                            &contactProcessingDisabled).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(contactProcessingDisabled);

    WorkerPoolCounts counts;
    ASSERT_TRUE(readWorkerPoolCounts(counts));
    EXPECT_EQ(counts.taskingThreads, 3u) << "the queued entry must reach carb.tasking before it starts";
    ASSERT_EQ(ovphysx_test_get_settings_bool("/physics/suppressReadback", &readback).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(readback);

    ASSERT_EQ(ovphysx_set_global_config_entry(readbackOff).status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t second = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&second).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_test_get_settings_bool("/physics/suppressReadback", &readback).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(readback) << "a later create must not replay a queued entry";

    EXPECT_EQ(ovphysx_destroy_instance(second).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(first).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

// Invalid startup values must be rejected before any process-wide setting is
// changed, so a caller can recover with a default construction.
TEST(WorkerPoolInvalidTest, RejectedCountsDoNotPoisonLaterConstruction)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);

    const ovphysx_config_entry_t invalidTasking = carboniteIntEntry(kTaskingThreadCountPath, "-1");
    ovphysx_create_args invalidArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    invalidArgs.config_entries = &invalidTasking;
    invalidArgs.config_entry_count = 1;
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance_with_args(&invalidArgs, &handle).status, OVPHYSX_API_INVALID_ARGUMENT);

    const ovphysx_config_entry_t invalidTbb = carboniteIntEntry(kTbbThreadCountPath, "0");
    invalidArgs.config_entries = &invalidTbb;
    EXPECT_EQ(ovphysx_create_instance_with_args(&invalidArgs, &handle).status, OVPHYSX_API_INVALID_ARGUMENT);

    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    WorkerPoolCounts counts;
    ASSERT_TRUE(readWorkerPoolCounts(counts));
    EXPECT_EQ(counts.taskingSetting, 0);
    EXPECT_EQ(counts.tbbSetting, 0);

    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

// A carb.tasking pool of one thread leaves no room for asynchronous execution: an
// instance created while OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION is true gets no worker
// thread, runs each call on the calling thread before it returns, and one warning names
// the pool size. The setting itself stays true.
TEST(WorkerPoolOneThreadTest, InstancesRunSynchronously)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    // On, whatever OVPHYSX_TEST_EXECUTION chose for this process.
    ASSERT_EQ(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, ovphysx_config_value_bool(true)).status,
              OVPHYSX_API_SUCCESS);
    std::atomic<int> warnings{ 0 };
    ASSERT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr, &countSynchronousWarnings, &warnings).status,
              OVPHYSX_API_SUCCESS);

    const ovphysx_config_entry_t oneThread = carboniteIntEntry(kTaskingThreadCountPath, "1");
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = &oneThread;
    args.config_entry_count = 1;
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance_with_args(&args, &handle).status, OVPHYSX_API_SUCCESS);

    WorkerPoolCounts counts;
    ASSERT_TRUE(readWorkerPoolCounts(counts));
    EXPECT_EQ(counts.taskingThreads, 1u);
    ovphysx_config_value_t asyncExecution{};
    ASSERT_EQ(ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, &asyncExecution).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(asyncExecution.value.bool_value);

    std::thread::id ranOn;
    const ovphysx_enqueue_result_t job =
        ovphysx_submit_test_callback_job_for_test_internal(handle, &recordThread, &ranOn);
    ASSERT_EQ(job.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ranOn, std::this_thread::get_id()) << "the job ran on the calling thread before the call returned";
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_PENDING;
    ASSERT_EQ(ovphysx_query_task_status(handle, job, &status).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(status, OVPHYSX_TASK_STATUS_SUCCEEDED);

    EXPECT_EQ(ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(warnings.load(), 1);
    ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);

    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}
