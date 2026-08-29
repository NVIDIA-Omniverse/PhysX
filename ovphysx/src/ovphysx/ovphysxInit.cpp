// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file ovphysxInit.cpp
 * @brief Library initialization status tracking
 * 
 * This file tracks USD compatibility check results for the ovphysx library.
 * 
 * NOTE: This file intentionally has NO library constructor/destructor hooks
 * (no DllMain, no __attribute__((constructor/destructor))).
 * 
 * RATIONALE:
 * - Library initialization (USD version check, config loading) is deferred to 
 *   ovphysx_create() to avoid loader lock deadlocks with Carbonite framework
 * - Library cleanup is handled by Carbonite's own shutdown and C++ static destructors
 * - Adding DllMain/constructor hooks that call Carbonite during load/unload causes
 *   double-free crashes due to non-deterministic plugin shutdown order
 * 
 * If you need to add initialization/cleanup in the future, ensure it does NOT:
 * - Call any Carbonite functions (framework, plugins, interfaces)
 * - Allocate/free Carbonite-managed resources (dictionary items, settings)
 * - Trigger USD or other heavy library loads
 * - Access non-trivial global objects that might not be initialized yet
 */

#include "UsdVersionCheck/UsdVersionCheck.h"
#include <string>

namespace {

// Global state for USD compatibility check results
// These flags are set by ovphysx_create() in ovphysx.cpp after USD version checking
bool g_usd_check_performed = false;
bool g_usd_check_failed = false;
std::string g_usd_check_error;

} // anonymous namespace

// Public API to check if USD compatibility check passed
extern "C" {

/**
 * @brief Check if USD version compatibility check passed
 * 
 * This can be called by application code to verify the library
 * initialized successfully before using it.
 * 
 * @param out_error_message If not NULL, will be filled with error message if check failed
 * @return 0 if check passed, non-zero if check failed
 */
int omni_sdk_physx_check_usd_compatibility_status(const char** out_error_message) {
    // Check failure status first, even if check not marked as "performed"
    // (it may be marked for retry but still represents a failed check)
    if (g_usd_check_failed) {
        if (out_error_message) {
            *out_error_message = g_usd_check_error.c_str();
        }
        return 1;
    }
    
    if (!g_usd_check_performed) {
        if (out_error_message) {
            *out_error_message = "USD compatibility check not yet performed";
        }
        return -1;
    }
    
    if (out_error_message) {
        *out_error_message = nullptr;
    }
    return 0;
}

} // extern "C"
