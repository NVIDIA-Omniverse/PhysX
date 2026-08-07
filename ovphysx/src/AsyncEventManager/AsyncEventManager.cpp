// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "AsyncEventManager/AsyncEventManager.hpp"

#include "internal/sdk/ovphysxSDK.hpp"

#include <algorithm>
#include <sstream>
#include <carb/logging/Log.h>

namespace ovphysx::async {

// Global async event state
struct AsyncEventState {
    async_event_handle_t next_event_handle = 1;
    std::mutex event_mutex;
    std::unordered_map<async_event_handle_t, EventStatus> event_statuses;
};

static AsyncEventState g_async_state;


async_event_handle_t AsyncEventManager::create_event() {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    
    async_event_handle_t event_handle = g_async_state.next_event_handle++;
    EventStatus status;
    g_async_state.event_statuses[event_handle] = status;
    
    return event_handle;
}

async_status_t AsyncEventManager::poll_event(async_event_handle_t event_handle) {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    
    auto it = g_async_state.event_statuses.find(event_handle);
    if (it == g_async_state.event_statuses.end()) {
        return ASYNC_STATUS_FAILED;
    }
    
    return it->second.status;
}

void AsyncEventManager::complete_event(async_event_handle_t event_handle, bool success, const char* error_msg) {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    
    auto it = g_async_state.event_statuses.find(event_handle);
    if (it == g_async_state.event_statuses.end()) {
        return;
    }
    
    EventStatus& status = it->second;
    status.status = success ? ASYNC_STATUS_COMPLETED : ASYNC_STATUS_FAILED;
    if (error_msg) {
        status.error_message = error_msg;
    }
}

void AsyncEventManager::cleanup_event(async_event_handle_t event_handle) {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    g_async_state.event_statuses.erase(event_handle);
}

std::string AsyncEventManager::get_event_error(async_event_handle_t event_handle) {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    
    auto it = g_async_state.event_statuses.find(event_handle);
    if (it == g_async_state.event_statuses.end()) {
        return "";
    }
    
    return it->second.error_message;
}

std::mutex& AsyncEventManager::get_mutex() {
    return g_async_state.event_mutex;
}

size_t AsyncEventManager::get_active_event_count() {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    return g_async_state.event_statuses.size();
}

void AsyncEventManager::cleanup_all_events() {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    g_async_state.event_statuses.clear();
}

bool AsyncEventManager::is_valid_event(async_event_handle_t event_handle) {
    std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
    auto it = g_async_state.event_statuses.find(event_handle);
    return it != g_async_state.event_statuses.end() && it->second.is_valid;
}

void AsyncEventManager::shutdown() {
    CARB_LOG_INFO("[AsyncEventManager] Shutting down...");
    
    // Detect and report leaks
    size_t active_count = 0;
    bool leaks_detected = false;
    {
        std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
        active_count = g_async_state.event_statuses.size();
        leaks_detected = (active_count > 0);
    }
    
    if (leaks_detected) {
        std::lock_guard<std::mutex> guard(g_async_state.event_mutex);
        
        std::ostringstream report;
        report << "Leak report: " << active_count << " active events, "
               << "next handle ID: " << g_async_state.next_event_handle
               << ", total created: " << (g_async_state.next_event_handle - 1);
        
        for (const auto& pair : g_async_state.event_statuses) {
            const auto& handle = pair.first;
            const auto& status = pair.second;
            report << " | Handle " << handle
                   << " status=" << static_cast<int>(status.status)
                   << " valid=" << (status.is_valid ? "true" : "false");
            if (!status.error_message.empty()) {
                report << " error=\"" << status.error_message << "\"";
            }
        }

        CARB_LOG_ERROR("[AsyncEventManager] %zu event handle(s) were not properly cleaned up! %s",
                      active_count, report.str().c_str());
    } else {
        CARB_LOG_INFO("[AsyncEventManager] No handle leaks detected - clean shutdown.");
    }
    
    cleanup_all_events();
    
    CARB_LOG_INFO("[AsyncEventManager] Shutdown complete.");
}

ovphysx_op_index_t register_operation(ovphysx_handle_t handle, async_event_handle_t event) {
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance) {
        return 0; // Invalid
    }
    
    ovphysx_op_index_t op_index = instance->next_op_index.fetch_add(1);
    
    std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
    instance->op_to_event[op_index] = event;
    
    return op_index;
}

async_event_handle_t get_event_for_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index) {
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
    auto op_it = instance->op_to_event.find(op_index);
    if (op_it != instance->op_to_event.end()) {
        return op_it->second;
    }
    return 0;
}

// Get all pending operation indices up to and including the target op_index
// If target_op_index is OVPHYSX_OP_INDEX_ALL, returns all registered operations
std::vector<ovphysx_op_index_t> get_pending_ops(ovphysx_handle_t handle, ovphysx_op_index_t target_op_index) {
    std::vector<ovphysx_op_index_t> pending_ops;
    
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance) {
        return pending_ops;
    }
    
    std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
    
    for (const auto& [op_idx, event] : instance->op_to_event) {
        if (target_op_index == OVPHYSX_OP_INDEX_ALL || op_idx <= target_op_index) {
            pending_ops.push_back(op_idx);
        }
    }
    
    // Sort to ensure we wait in order
    std::sort(pending_ops.begin(), pending_ops.end());
    
    return pending_ops;
}



} // namespace ovphysx::async
