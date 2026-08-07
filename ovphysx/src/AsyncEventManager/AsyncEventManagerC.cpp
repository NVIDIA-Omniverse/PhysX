// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "AsyncEventManager/AsyncEventManager.h"
#include "AsyncEventManager/AsyncEventManager.hpp"
#include <carb/logging/Log.h>

extern "C" {

async_event_handle_t async_create_event(void) {
    try {
        return ovphysx::async::AsyncEventManager::create_event();
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_create_event failed: %s", e.what());
        return 0;
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_create_event failed: unknown exception");
        return 0;
    }
}

async_status_t async_poll_event(async_event_handle_t event_handle) {
    try {
        return static_cast<async_status_t>(ovphysx::async::AsyncEventManager::poll_event(event_handle));
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_poll_event failed for handle %llu: %s",
                       static_cast<unsigned long long>(event_handle), e.what());
        return ASYNC_STATUS_FAILED;
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_poll_event failed for handle %llu: unknown exception",
                       static_cast<unsigned long long>(event_handle));
        return ASYNC_STATUS_FAILED;
    }
}

void async_complete_event(async_event_handle_t event_handle, bool success, const char* error_msg) {
    try {
        ovphysx::async::AsyncEventManager::complete_event(event_handle, success, error_msg);
    } catch (const std::exception& e) {
        // Expected: can fail if event_handle is invalid (already cleaned up)
        CARB_LOG_WARN("[AsyncEventManager] async_complete_event warning for handle %llu: %s (may be expected if event already cleaned up)",
                      static_cast<unsigned long long>(event_handle), e.what());
    } catch (...) {
        CARB_LOG_WARN("[AsyncEventManager] async_complete_event warning for handle %llu: unknown exception (may be expected if event already cleaned up)",
                      static_cast<unsigned long long>(event_handle));
    }
}

void async_cleanup_event(async_event_handle_t event_handle) {
    try {
        ovphysx::async::AsyncEventManager::cleanup_event(event_handle);
    } catch (const std::exception& e) {
        // Expected: can fail if event_handle is invalid (already cleaned up or never created)
        CARB_LOG_WARN("[AsyncEventManager] async_cleanup_event warning for handle %llu: %s (may be expected if event already cleaned up)",
                      static_cast<unsigned long long>(event_handle), e.what());
    } catch (...) {
        CARB_LOG_WARN("[AsyncEventManager] async_cleanup_event warning for handle %llu: unknown exception (may be expected if event already cleaned up)",
                      static_cast<unsigned long long>(event_handle));
    }
}

bool async_is_valid_event(async_event_handle_t event_handle) {
    try {
        return ovphysx::async::AsyncEventManager::is_valid_event(event_handle);
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_is_valid_event failed for handle %llu: %s",
                       static_cast<unsigned long long>(event_handle), e.what());
        return false;
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_is_valid_event failed for handle %llu: unknown exception",
                       static_cast<unsigned long long>(event_handle));
        return false;
    }
}

size_t async_get_active_event_count(void) {
    try {
        return ovphysx::async::AsyncEventManager::get_active_event_count();
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_get_active_event_count failed: %s", e.what());
        return 0;
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_get_active_event_count failed: unknown exception");
        return 0;
    }
}

void async_cleanup_all_events(void) {
    try {
        ovphysx::async::AsyncEventManager::cleanup_all_events();
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_cleanup_all_events failed: %s", e.what());
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_cleanup_all_events failed: unknown exception");
    }
}

void async_shutdown(void) {
    try {
        ovphysx::async::AsyncEventManager::shutdown();
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[AsyncEventManager] async_shutdown failed: %s", e.what());
    } catch (...) {
        CARB_LOG_ERROR("[AsyncEventManager] async_shutdown failed: unknown exception");
    }
}

} // extern "C"
