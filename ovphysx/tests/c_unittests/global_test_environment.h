// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
    // Only treat "1" as enabled; "0" or empty means disabled.
    return v && v[0] == '1' && v[1] == '\0';
}

static inline ::testing::AssertionResult waitForOperationSuccess(
    ovphysx_handle_t handle,
    ovphysx_op_index_t op_index,
    uint64_t timeout_ns = 1'000'000'000ULL)
{
    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t result = ovphysx_wait_op(handle, op_index, timeout_ns, &wait_result);

    std::ostringstream message;
    bool ok = (result.status == OVPHYSX_API_SUCCESS) && (wait_result.num_errors == 0);

    if (result.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        if (err.length > 0) {
            message << std::string(err.ptr, err.length);
        }
    }

    if (wait_result.num_errors > 0 && wait_result.error_op_indices) {
        for (size_t i = 0; i < wait_result.num_errors; ++i) {
            if (i > 0 || !message.str().empty()) {
                message << " | ";
            }
            ovphysx_string_t op_err = ovphysx_get_last_op_error(wait_result.error_op_indices[i]);
            if (op_err.ptr && op_err.length > 0) {
                message << std::string(op_err.ptr, op_err.length);
            } else {
                message << "(no error message for op " << wait_result.error_op_indices[i] << ")";
            }
        }
    }
    if (ok) {
        ovphysx_destroy_wait_result(&wait_result);
        return ::testing::AssertionSuccess();
    }

    if (wait_result.lowest_pending_op_index != 0) {
        if (!message.str().empty()) {
            message << " | ";
        }
        message << "lowest_pending_op_index=" << wait_result.lowest_pending_op_index;
    }

    ovphysx_destroy_wait_result(&wait_result);

    return ::testing::AssertionFailure()
           << "ovphysx_wait_op(" << op_index << ") failed with status "
           << result.status << ": " << message.str();
}

// Process-global shared CPU instance. Carbonite and the Python interpreter
// cannot be cleanly finalized and re-initialized in the same process (DLL init
// routines fail on reload). All CPU-mode fixtures share this single instance,
// resetting simulation state between tests without destroying the instance.
inline ovphysx_handle_t& sharedCpuInstance()
{
    static ovphysx_handle_t h = 0;
    return h;
}

inline bool ensureSharedCpuInstance()
{
    ovphysx_handle_t& h = sharedCpuInstance();
    if (h != 0) return true;

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_result_t r = ovphysx_create_instance(&args, &h);
    if (r.status != OVPHYSX_API_SUCCESS) { h = 0; return false; }
    return true;
}

inline void destroySharedCpuInstance()
{
    ovphysx_handle_t& h = sharedCpuInstance();
    if (h != 0) {
        ovphysx_enqueue_result_t rr = ovphysx_reset_stage(h);
        if (rr.status == OVPHYSX_API_SUCCESS && rr.op_index != 0) {
            ovphysx_op_wait_result_t wait_result{};
            ovphysx_wait_op(h, rr.op_index, 10'000'000'000ULL, &wait_result);
            ovphysx_destroy_wait_result(&wait_result);
        }
        test_utils::destroy_ovstage_test_attachments(h);
        ovphysx_destroy_instance(h);
        h = 0;
    }
}

// Test fixture base class for tests that need a PhysX CPU instance.
// Uses a process-global shared instance; resets simulation state between tests.
//
// NOTE: Uses CPU device. Tests that need GPU TensorBinding should use a
// GPU-specific fixture instead.
class PhysXTestFixture : public ::testing::Test {
protected:
    ovphysx_handle_t m_handle = 0;

    void SetUp() override {
        ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU PhysX instance";
        m_handle = sharedCpuInstance();
    }

    void TearDown() override {
        if (m_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                EXPECT_TRUE(waitForOperationSuccess(m_handle, reset_result.op_index))
                    << "Fixture reset failed during teardown";
            } else {
                EXPECT_EQ(reset_result.status, OVPHYSX_API_SUCCESS)
                    << "Fixture reset enqueue failed during teardown";
            }
            test_utils::destroy_ovstage_test_attachments(m_handle);
            m_handle = 0;
        }
    }
};
