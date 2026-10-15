// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-3 AC-5 AC-6 AC-7
 */

#ifndef OVPHYSX_LOG_MANAGER_HPP
#define OVPHYSX_LOG_MANAGER_HPP

#include "ovphysx/ovphysx_types.h"

namespace ovphysx
{

/**
 * @brief Called by CarboniteLoader::initialize() after logging is available.
 *
 * Applies the stored log level to ovphysx's named Carbonite sources and registers the
 * UserCallbackLogger if an application callback was set before initialization.
 */
void onCarboniteLoggingReady();

/** Reset framework-owned registrations before releasing Carbonite. */
void onCarboniteLoggingStopped();

/** Flush Carbonite, close logger registration until bootstrap, and disable/drain the callback. */
ovphysx_result_t shutdownLogCallback();

/** Return whether the calling thread is inside the application log callback. */
bool isInLogCallback();

/** Scoped-config implementation for the process log threshold. */
ovphysx_result_t setLogLevel(uint32_t level);
uint32_t getLogLevel();

/** Scoped-config implementation for Carbonite's default console logger. */
ovphysx_result_t setDefaultLogOutput(bool enable);
bool getDefaultLogOutput();

} // namespace ovphysx

#endif // OVPHYSX_LOG_MANAGER_HPP
