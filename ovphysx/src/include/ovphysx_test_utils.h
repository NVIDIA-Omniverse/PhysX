// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-7
 */

/**
 * @file ovphysx_test_utils.h
 * @brief Internal test utilities for ovphysx log diagnostics.
 *
 * These functions are exported from the shared library for use by the ovphysx
 * test suite. They are not part of the public API and should not be used by
 * external applications. Users who need to capture log messages should register
 * a callback via ovphysx_set_log_callback() instead.
 */

#ifndef OVPHYSX_TEST_UTILS_H
#define OVPHYSX_TEST_UTILS_H

#include "ovphysx/ovphysx.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Start capturing log messages.
     *
     * Registers an internal log listener that records all messages emitted by
     * the Carbonite logging system. Messages accumulate until
     * ovphysx_log_capture_stop() is called.
     *
     * @return ovphysx_result_t with status.
     *
     * @pre  Carbonite framework must be initialized (an instance must exist).
     * @post Log messages are being captured.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_log_capture_start(void);

    /**
     * @brief Stop capturing and discard all captured messages.
     *
     * Removes the internal log listener and frees captured message storage.
     * Safe to call even if capture was not started.
     */
    OVPHYSX_API void ovphysx_log_capture_stop(void);

    /**
     * @brief Search captured messages for a substring at a given level.
     *
     * @param level Log level to search (ovphysx_log_level_t).
     * @param substring Substring to search for (case-sensitive).
     * @return true if at least one captured message at the given level contains
     *         the substring.
     */
    OVPHYSX_API bool ovphysx_log_capture_find(uint32_t level, const char* substring);

    /**
     * @brief Count captured messages at a given level.
     *
     * @param level Log level to count (ovphysx_log_level_t).
     * @return Number of messages captured at this level.
     */
    OVPHYSX_API uint32_t ovphysx_log_capture_count(uint32_t level);

    /**
     * @brief Emit deterministic records for logging tests.
     *
     * Emits ovphysx-source records at ERROR, WARNING, INFO, and VERBOSE, plus
     * a FATAL suppression probe when the configured level is NONE. Use with
     * the capture API to verify source and severity filtering.
     */
    OVPHYSX_API void ovphysx_log_emit_test_messages(void);

    /**
     * @brief Set Carbonite's process-global logging enable for tests.
     *
     * This is test-only access to host-owned state. Tests must restore the
     * prior enabled state before returning.
     */
    OVPHYSX_API void ovphysx_log_set_global_enabled_for_test(bool enabled);

    /**
     * @brief Query Carbonite's process-global logging enable for tests.
     */
    OVPHYSX_API bool ovphysx_log_get_global_enabled_for_test(void);

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_TEST_UTILS_H */
