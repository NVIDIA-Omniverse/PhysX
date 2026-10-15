// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-THREADING-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

// Worker-pool sizing is decided once per process, when the first instance
// starts carb.tasking, so each case below needs a process where no earlier test
// has started the pool. Both suites are routed to their own gtest passes in
// scripts/test_cpp.cmake and excluded from the shared cpu pass.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>
#include <gtest/gtest.h>

#include "ovphysxTestHelpers.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cstdint>

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
