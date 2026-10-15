// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>

#include "test_utilities.h"

static inline bool ovphysxTestRequireCuda()
{
    const char* v = std::getenv("OVPHYSX_TEST_REQUIRE_CUDA");
    // Only the exact value "1" enables the requirement.
    return v && v[0] == '1' && v[1] == '\0';
}

static inline ::testing::AssertionResult waitForTaskSuccess(
    ovphysx_instance_handle_t handle,
    ovphysx_enqueue_result_t task,
    uint64_t timeout_ns = 1'000'000'000ULL)
{
    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t result = ovphysx_wait_task(handle, task, timeout_ns, &wait_result);

    std::ostringstream message;
    bool ok = (result.status == OVPHYSX_API_SUCCESS) && (wait_result.num_errors == 0);

    if (result.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        if (err.length > 0) {
            message << std::string(err.ptr, err.length);
        }
    }

    if (wait_result.num_errors > 0 && wait_result.error_tasks) {
        for (size_t i = 0; i < wait_result.num_errors; ++i) {
            if (i > 0 || !message.str().empty()) {
                message << " | ";
            }
            ovphysx_string_t op_err = ovphysx_get_last_task_error(handle, wait_result.error_tasks[i]);
            if (op_err.ptr && op_err.length > 0) {
                message << std::string(op_err.ptr, op_err.length);
            } else {
                message << "(no error message for op " << wait_result.error_tasks[i].op_index << ")";
            }
        }
    }
    if (ok) {
        ovphysx_destroy_wait_task_result(&wait_result);
        return ::testing::AssertionSuccess();
    }

    if (wait_result.pending_task.op_index != 0) {
        if (!message.str().empty()) {
            message << " | ";
        }
        message << "lowest_pending_op_index=" << wait_result.pending_task.op_index;
    }

    ovphysx_destroy_wait_task_result(&wait_result);

    return ::testing::AssertionFailure()
           << "ovphysx_wait_task(" << task.op_index << ") failed with status "
           << result.status << ": " << message.str();
}

// Process-global shared CPU instance. Carbonite and the Python interpreter
// cannot be cleanly finalized and re-initialized in the same process (DLL init
// routines fail on reload). All CPU-mode fixtures share this single instance,
// resetting simulation state between tests without destroying the instance.
inline ovphysx_instance_handle_t& sharedCpuInstance()
{
    static ovphysx_instance_handle_t h = 0;
    return h;
}

inline bool ensureSharedCpuInstance()
{
    ovphysx_instance_handle_t& h = sharedCpuInstance();
    if (h != 0) return true;

    ovphysx_result_t r = ovphysx_create_instance(&h);
    if (r.status != OVPHYSX_API_SUCCESS) { h = 0; return false; }
    return true;
}

inline void destroySharedCpuInstance()
{
    ovphysx_instance_handle_t& h = sharedCpuInstance();
    if (h != 0) {
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(h))
            << "Fixture detach failed during shared instance destruction";
        const ovphysx_result_t destroyed = ovphysx_destroy_instance(h);
        EXPECT_EQ(destroyed.status, OVPHYSX_API_SUCCESS);
        if (destroyed.status == OVPHYSX_API_SUCCESS)
            test_utils::destroy_ovstage_test_stages(h);
        h = 0;
    }
}

// Fixture base class for tests that need a PhysX CPU instance. It uses the
// process-global shared instance and resets simulation state between tests.
//
// NOTE: Tests that need GPU TensorBinding should use a GPU-specific fixture instead.
class PhysXTestFixture : public ::testing::Test {
protected:
    ovphysx_instance_handle_t m_handle = 0;

    void SetUp() override {
        ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU PhysX instance";
        m_handle = sharedCpuInstance();
    }

    void TearDown() override {
        if (m_handle != 0) {
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle))
                << "Fixture detach failed during teardown";
            m_handle = 0;
        }
    }
};
