// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OVPHYSX_EXPERIMENTAL_HELPERS_HPP
#define OVPHYSX_EXPERIMENTAL_HELPERS_HPP

//
// C++17 helper utilities for the ovphysx C API.
// Provides RAII wrappers and convenience functions for multi-step operations.
//

#include <string>
#include <vector>
#include <cstring>

#include "ovphysx/ovphysx.h"

namespace ovphysx {
namespace physx {

/**
 * @brief RAII wrapper for ovphysx_op_wait_result_t
 *
 * Automatically calls ovphysx_destroy_wait_result when destroyed.
 * Use get() to pass to ovphysx_wait_op.
 *
 * @code{.cpp}
 * #include <ovphysx/experimental/Helpers.hpp>
 * #include <iostream>
 *
 * static ovphysx_result_t wait_and_report_errors(
 *     ovphysx_handle_t handle,
 *     ovphysx_op_index_t op_index,
 *     ovphysx_timeout_t timeout_ns)
 * {
 *     ovphysx::physx::WaitResult wait_result;
 *     ovphysx_result_t result =
 *         ovphysx_wait_op(handle, op_index, timeout_ns, wait_result.get());
 *     for (size_t index = 0; index < wait_result.errorCount(); ++index) {
 *         ovphysx_op_index_t failed_op = wait_result.errorOpIndexAt(index);
 *         ovphysx_string_t error = ovphysx_get_last_op_error(failed_op);
 *         std::cerr << "Op " << failed_op << " failed: "
 *                   << std::string(error.ptr, error.length) << '\n';
 *     }
 *     return result;
 * }
 * @endcode
 */
class WaitResult {
public:
    WaitResult() : m_result{} {}
    
    ~WaitResult() {
        release();
    }
    
    WaitResult(WaitResult&& other) noexcept : m_result(other.m_result) {
        other.m_result = ovphysx_op_wait_result_t{};
    }
    
    WaitResult& operator=(WaitResult&& other) noexcept {
        if (this != &other) {
            release();
            m_result = other.m_result;
            other.m_result = ovphysx_op_wait_result_t{};
        }
        return *this;
    }
    
    // Non-copyable
    WaitResult(const WaitResult&) = delete;
    WaitResult& operator=(const WaitResult&) = delete;
    
    /// Get pointer to underlying result (pass to ovphysx_wait_op)
    ovphysx_op_wait_result_t* get() { return &m_result; }
    const ovphysx_op_wait_result_t* get() const { return &m_result; }
    
    /// Check if there were any errors
    bool hasErrors() const { return m_result.num_errors > 0; }
    
    /// Number of errors
    size_t errorCount() const { return m_result.num_errors; }
    
    /// Get lowest pending operation index (0 if all complete)
    ovphysx_op_index_t lowestPendingOpIndex() const { return m_result.lowest_pending_op_index; }
    
    /// Get the failed operation index at position i
    ovphysx_op_index_t errorOpIndexAt(size_t i) const {
        if (m_result.error_op_indices && i < m_result.num_errors) {
            return m_result.error_op_indices[i];
        }
        return 0;
    }

private:
    void release() {
        ovphysx_destroy_wait_result(&m_result);
        m_result = ovphysx_op_wait_result_t{};
    }
    
    ovphysx_op_wait_result_t m_result;
};

} // namespace physx
} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_HELPERS_HPP
