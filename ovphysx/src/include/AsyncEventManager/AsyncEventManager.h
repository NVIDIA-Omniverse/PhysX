// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ASYNC_EVENT_MANAGER_H
#define ASYNC_EVENT_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ovphysx/ovphysx_export.h"

#ifdef __cplusplus
extern "C" {
#endif

// Primary async event types - this is the source of truth for async event types
typedef uint64_t async_event_handle_t;

// Async status type - explicitly int32_t
typedef int32_t async_status_t;

// Async status values
#define ASYNC_STATUS_PENDING   ((async_status_t)0)
#define ASYNC_STATUS_COMPLETED ((async_status_t)1)
#define ASYNC_STATUS_FAILED    ((async_status_t)2)

// This header provides the C interface functions for the shared async utility

/**
 * @brief Create an async event
 * 
 * @return Event handle (0 if failed)
 */
OVPHYSX_API async_event_handle_t async_create_event(void);

/**
 * @brief Poll an async event for status
 * 
 * @param event_handle Event handle to poll
 * @return Current status of the event
 */
OVPHYSX_API async_status_t async_poll_event(async_event_handle_t event_handle);

/**
 * @brief Complete an async event
 * 
 * @param event_handle Event handle to complete
 * @param success True if successful, false if failed
 * @param error_msg Error message (can be NULL)
 */
OVPHYSX_API void async_complete_event(async_event_handle_t event_handle, bool success, const char* error_msg);

/**
 * @brief Clean up an async event
 * 
 * @param event_handle Event handle to clean up
 */
OVPHYSX_API void async_cleanup_event(async_event_handle_t event_handle);

/**
 * @brief Check if an event handle is valid
 * 
 * @param event_handle Event handle to check
 * @return True if valid, false otherwise
 */
OVPHYSX_API bool async_is_valid_event(async_event_handle_t event_handle);

/**
 * @brief Get count of active events (for debugging)
 * 
 * @return Number of active events
 */
OVPHYSX_API size_t async_get_active_event_count(void);

/**
 * @brief Clean up all events (for testing/cleanup)
 */
OVPHYSX_API void async_cleanup_all_events(void);

/**
 * @brief Shutdown with handle leak detection and reporting
 */
OVPHYSX_API void async_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // ASYNC_EVENT_MANAGER_H
