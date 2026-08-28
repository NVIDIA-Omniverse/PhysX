// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <mutex>
#include <vector>
#include <string>
#include "ovphysx/ovphysx_export.h"
#include "AsyncEventManager.h"
#include "ovphysx/ovphysx_types.h"

namespace ovphysx::async {

// Use the C types as the source of truth
using async_event_handle_t = ::async_event_handle_t;
using async_status_t = ::async_status_t;

/**
 * @brief Internal event status structure
 */
struct EventStatus {
    async_status_t status;
    std::string error_message;
    bool is_valid;
    
    EventStatus() : status(ASYNC_STATUS_PENDING), is_valid(true) {}
};

/**
 * @brief Async event manager for ovphysx operations
 *
 * This utility provides a common async event system for ovphysx's async
 * operations without exposing system-specific types.
 */
class OVPHYSX_API AsyncEventManager {
public:
    // Core event management functions
    static async_event_handle_t create_event();
    static async_status_t poll_event(async_event_handle_t event_handle);
    static void complete_event(async_event_handle_t event_handle, bool success, const char* error_msg = nullptr);
    static void cleanup_event(async_event_handle_t event_handle);
    static std::string get_event_error(async_event_handle_t event_handle);
    
    // Thread safety and debugging
    static std::mutex& get_mutex();
    static size_t get_active_event_count();
    static void cleanup_all_events();
    
    // Internal validation
    static bool is_valid_event(async_event_handle_t event_handle);
    
    static void shutdown();
};

OVPHYSX_API ovphysx_op_index_t register_operation(ovphysx_handle_t handle, async_event_handle_t event);
OVPHYSX_API async_event_handle_t get_event_for_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index);
OVPHYSX_API std::vector<ovphysx_op_index_t> get_pending_ops(ovphysx_handle_t handle, ovphysx_op_index_t target_op_index);

} // namespace ovphysx::async
