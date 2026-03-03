// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_EXPERIMENTAL_HELPERS_HPP
#define OVPHYSX_EXPERIMENTAL_HELPERS_HPP

//
// C++17 Helper utilities for ovphysx C API
// Provides RAII wrappers and convenience functions for complex operations
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
 * Automatically calls ovphysx_destroy_errors when destroyed.
 * Use get() to pass to ovphysx_wait_op.
 * 
 * Example:
 *   WaitResult wait(handle);
 *   ovphysx_result_t r = ovphysx_wait_op(handle, op_index, timeout, wait.get());
 *   
 *   if (wait.hasErrors()) {
 *       for (const auto& err : wait.errors()) {
 *           std::cerr << "Error: " << err << std::endl;
 *       }
 *   }
 *   // errors freed automatically when wait goes out of scope
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
    
    /// Copy all error messages to a vector of strings
    std::vector<std::string> errors() const {
        std::vector<std::string> result;
        if (m_result.errors && m_result.num_errors > 0) {
            result.reserve(m_result.num_errors);
            for (size_t i = 0; i < m_result.num_errors; ++i) {
                const ovphysx_string_t& err = m_result.errors[i].error;
                if (err.ptr && err.length > 0) {
                    result.emplace_back(err.ptr, err.length);
                } else {
                    result.emplace_back();
                }
            }
        }
        return result;
    }
    
    /// Get error at specific index
    std::string errorAt(size_t i) const {
        if (m_result.errors && i < m_result.num_errors) {
            const ovphysx_string_t& err = m_result.errors[i].error;
            if (err.ptr && err.length > 0) {
                return std::string(err.ptr, err.length);
            }
        }
        return {};
    }
    
    /// Get operation index for error at specific index
    ovphysx_op_index_t opIndexAt(size_t i) const {
        if (m_result.errors && i < m_result.num_errors) {
            return m_result.errors[i].op_index;
        }
        return 0;
    }

private:
    void release() {
        if (m_result.errors && m_result.num_errors > 0) {
            ovphysx_destroy_errors(m_result.errors, m_result.num_errors);
        }
        m_result = ovphysx_op_wait_result_t{};
    }
    
    ovphysx_op_wait_result_t m_result;
};

} // namespace physx
} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_HELPERS_HPP
