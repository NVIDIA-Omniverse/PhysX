// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_LOG_MANAGER_HPP
#define OVPHYSX_LOG_MANAGER_HPP

namespace ovphysx
{

/**
 * @brief Called by CarboniteLoader::initialize() after logging is available.
 *
 * Applies the stored log level to Carbonite's setLevelThreshold() and
 * registers the UserCallbackLogger if any callbacks were registered pre-init.
 */
void onCarboniteLoggingReady();

} // namespace ovphysx

#endif // OVPHYSX_LOG_MANAGER_HPP
