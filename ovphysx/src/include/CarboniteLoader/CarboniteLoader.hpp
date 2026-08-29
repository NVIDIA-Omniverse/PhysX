// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef OVPHYSX_CARBONITE_LOADER_HPP
#define OVPHYSX_CARBONITE_LOADER_HPP

#include <string>
#include <utility>
#include <vector>
#include <stdint.h>

// Forward declare to avoid forcing include on consumers
namespace omni { namespace physx { class IPhysxSimulation; } }

namespace ovphysx
{

/**
 * @brief Bootstraps Carbonite dependencies and starts the linked PhysX runtime.
 * 
 * Simplified loader for flat _install/plugins/ structure.
 * 
 * Expected layout:
 *   _install/
 *   |-- lib/libovphysx.so
 *   `-- plugins/           <- remaining Carbonite and USD dependency libraries
 *       |-- usd/           <- USD plugInfo.json registry
 *       `-- bin/deps/      <- Vulkan/CUDA libraries
 * 
 * Environment variables:
 * - OVPHYSX_DISABLE_GPU: If set, skip loading GPU plugins (CPU-only mode)
 */
class CarboniteLoader
{
public:
    CarboniteLoader();
    ~CarboniteLoader();
    
    // Non-copyable
    CarboniteLoader(const CarboniteLoader&) = delete;
    CarboniteLoader& operator=(const CarboniteLoader&) = delete;

    /**
     * Start Carbonite for ovphysx SDK mode.
     *
     * This may reuse an existing Carbonite framework created by another OV library.
     * Carbonite remains responsible for dependency-plugin registration and
     * interface-version compatibility while ovphysx prepares the SDK plugin search paths.
     *
     * This loads only the base Carbonite plugins. The USD dependencies and linked
     * PhysX runtime are prepared by the remaining loader phases.
     */
    bool initialize();

    /**
     * Make the namespaced USD runtime available to later plugin loads.
     *
     * If another OV library already loaded the same namespaced USD monolith, reuse
     * that loaded image. Otherwise, load ovphysx's packaged namespaced USD library.
     */
    bool preloadUsdLibraries();

    /**
     * Load Carbonite plugins that need USD.
     *
     * This must run after preloadUsdLibraries() so these plugins bind to the same
     * namespaced USD runtime that ovrtx or another OV library may already use.
     */
    bool loadUsdDependentPlugins();

    /**
     * Load ovphysx's bundled PhysX dependency plugins and start the static runtime.
     *
     * Carbonite handles dependency plugin loading. This returns false if the
     * static PhysX runtime cannot provide the IPhysxSimulation table ovphysx needs.
     */
    bool loadPhysxPlugins();

    /**
     * Details for the last Carbonite startup or plugin-load failure.
     *
     * ovphysx_create_instance() copies this into the public last-error string so
     * callers get the specific reason instead of a generic bootstrap failure.
     */
    const std::string& getLastError() const;

    /**
     * Configure the internal Carbonite GPU-plugin bootstrap sentinel.
     *
     * Current ovphysx callers pass -2 before initialize() to skip Carbonite GPU
     * plugins that PhysX does not use. This does not select the PhysX CUDA
     * ordinal; active_cuda_gpus is applied immediately before scene attachment.
     * Values other than -2 are currently unused by ovphysx; that branch is
     * retained to preserve existing internal CarboniteLoader behavior.
     *
     * Note: /physics/suppressReadback (DirectGPU-API mode) is NOT managed
     * here — it is opt-in by the host. Callers who want DirectGPU set the
     * Carbonite setting themselves before any ovphysx call.
     */
    static void setStartupCudaDevice(int32_t cudaDevice);

    /**
     * Shutdown (called automatically by destructor).
     */
    void shutdown();

    /**
     * Access the PhysX simulation function table from the linked runtime.
     * @return Interface pointer or nullptr if not loaded.
     */
    omni::physx::IPhysxSimulation* getPhysxSimulation() const;

private:
    // Build plugin search paths from m->pluginsDir (always) and plugins/bin/deps (if present).
    // Returns the owned strings and const char* views into them as a pair.
    // Callers must keep the first vector alive while using the second.
    std::pair<std::vector<std::string>, std::vector<const char*>> buildSearchPaths() const;

    struct Impl;
    Impl* m;
};

} // namespace ovphysx

#endif // OVPHYSX_CARBONITE_LOADER_HPP
