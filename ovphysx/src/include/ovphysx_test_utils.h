// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @file ovphysx_test_utils.h
 * @brief Internal test utilities for ovphysx log diagnostics.
 *
 * These functions are exported from the shared library for use by the ovphysx
 * test suite. They are NOT part of the public API and should not be used by
 * external applications. Users who need to capture log messages should register
 * a callback via ovphysx_register_log_callback() instead.
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
     * @brief Emit one test message at each log level.
     *
     * Emits messages with known content at ERROR, WARNING, INFO, and VERBOSE
     * levels. Use with the capture API to verify log level filtering.
     *
     * Messages contain the level name, e.g., "[LogTest] ERROR test message".
     */
    OVPHYSX_API void ovphysx_log_emit_test_messages(void);

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_TEST_UTILS_H */
