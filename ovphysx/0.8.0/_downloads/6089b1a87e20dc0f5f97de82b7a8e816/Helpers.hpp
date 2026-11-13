// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-TASK-001
// @covers AC-4 AC-5

#ifndef OVPHYSX_EXPERIMENTAL_HELPERS_HPP
#define OVPHYSX_EXPERIMENTAL_HELPERS_HPP

//
// C++17 helper utilities for the ovphysx C API.
// Provides RAII wrappers and convenience functions for multi-step operations.
//

#include <string>
#include <utility>

#include "ovphysx/ovphysx.h"

namespace ovphysx {
namespace physx {

/**
 * @brief Move-only owner of a task wait result and its return status.
 *
 * Error text remains valid until this result is destroyed, including after the
 * instance is destroyed. A moved-from result owns no storage.
 * Like the C API, waits require an empty result. To reuse a populated owner,
 * explicitly release its current output with `result = WaitResult{};` first.
 *
 * @code{.cpp}
 * #include <ovphysx/experimental/Helpers.hpp>
 * #include <iostream>
 *
 * static ovphysx_api_status_t wait_and_report_errors(
 *     ovphysx_instance_handle_t handle,
 *     ovphysx_enqueue_result_t task,
 *     ovphysx_timeout_t timeout_ns)
 * {
 *     ovphysx::physx::WaitResult result;
 *     result.waitTask(handle, task, timeout_ns);
 *     for (size_t index = 0; index < result.errorCount(); ++index) {
 *         ovphysx_enqueue_result_t failed = result.errorTaskAt(index);
 *         ovphysx_string_t error = ovphysx_get_last_task_error(handle, failed);
 *         std::cerr << "Task " << failed.op_index << " failed: "
 *                   << std::string(error.ptr, error.length) << '\n';
 *     }
 *     return result.status();
 * }
 * @endcode
 */
class WaitResult {
public:
    WaitResult() = default;

    ~WaitResult() {
        ovphysx_destroy_wait_task_result(&m_result);
    }

    WaitResult(WaitResult&& other) noexcept
        : m_result(std::exchange(other.m_result, {})),
          m_status(std::exchange(other.m_status, OVPHYSX_API_SUCCESS)) {}

    WaitResult& operator=(WaitResult&& other) noexcept {
        if (this != &other) {
            ovphysx_destroy_wait_task_result(&m_result);
            m_result = std::exchange(other.m_result, {});
            m_status = std::exchange(other.m_status, OVPHYSX_API_SUCCESS);
        }
        return *this;
    }

    WaitResult(const WaitResult&) = delete;
    WaitResult& operator=(const WaitResult&) = delete;

    ovphysx_api_status_t waitTask(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
                                 ovphysx_timeout_t timeout_ns = OVPHYSX_TIMEOUT_INFINITE) {
        m_status = ovphysx_wait_task(handle, task, timeout_ns, &m_result).status;
        return m_status;
    }

    ovphysx_api_status_t waitAll(ovphysx_instance_handle_t handle,
                                ovphysx_timeout_t timeout_ns = OVPHYSX_TIMEOUT_INFINITE) {
        m_status = ovphysx_wait_all(handle, timeout_ns, &m_result).status;
        return m_status;
    }

    const ovphysx_wait_task_result_t* get() const { return &m_result; }
    ovphysx_api_status_t status() const { return m_status; }
    bool hasErrors() const { return m_result.num_errors > 0; }
    size_t errorCount() const { return m_result.num_errors; }
    ovphysx_enqueue_result_t pendingTask() const { return m_result.pending_task; }

    ovphysx_enqueue_result_t errorTaskAt(size_t index) const {
        if (m_result.error_tasks && index < m_result.num_errors) {
            return m_result.error_tasks[index];
        }
        return {};
    }

private:
    ovphysx_wait_task_result_t m_result{};
    ovphysx_api_status_t m_status = OVPHYSX_API_SUCCESS;
};

} // namespace physx
} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_HELPERS_HPP
