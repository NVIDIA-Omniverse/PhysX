// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-2 AC-3 AC-5
 *
 * @implements REQ-CAPI-PROFILER-002
 * @covers AC-4 AC-11 AC-12
 */

#ifndef OVPHYSX_INTERNAL_CARBONITE_LOADER_STATE_HPP
#define OVPHYSX_INTERNAL_CARBONITE_LOADER_STATE_HPP

#include "ovphysx/ovphysx_types.h"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>

namespace ovphysx
{

struct CarboniteLoaderState
{
    std::mutex bootstrapMutex;
    // Base bootstrap lasts until framework release: explicit SDK shutdown on Linux,
    // process exit on Windows, where Carbonite must protect dependency DLL teardown.
    bool bootstrapDone = false;
    // A foreign host remains responsible for the framework it populated before ovphysx.
    bool ownsFramework = true;
    // Live loaders holding the framework. Only the last loader releases the profiler.
    std::atomic<int> activeLoaders{0};

    // Keep the private cache alive until framework shutdown stops the datastore.
    // Cleanup holds bootstrapMutex and clears the path only after successful removal.
    std::filesystem::path processCacheDir;

    // Last-loader shutdown takes bootstrapMutex before profilerMutex.
    std::mutex profilerMutex;
    bool profilerBackendActive = false;
    // The bound backend and destination remain fixed until the last loader shuts down.
    int32_t profilerBackendBound = OVPHYSX_PROFILER_BACKEND_NONE;
    bool profilerBindFailed = false;
    bool profilerHostOwned = false;
    bool profilerOutputPathCaptured = false;
    std::string profilerBoundOutputPath;

    // Register once per framework; reset only after releasing that framework.
    bool nvtxPluginRegistered = false;
};

} // namespace ovphysx

#endif // OVPHYSX_INTERNAL_CARBONITE_LOADER_STATE_HPP
