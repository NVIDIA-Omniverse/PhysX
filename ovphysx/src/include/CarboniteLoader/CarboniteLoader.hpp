// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
 * Loader for the flat _install/plugins/ layout.
 * 
 * Expected layout:
 *   _install/
 *   |-- lib/libovphysx.so
 *   |-- schemas/physx/     <- codeless PhysX USD schemas (data only, never loaded here)
 *   `-- plugins/           <- Carbonite and PhysX dependency libraries
 *       `-- bin/deps/      <- Vulkan/CUDA libraries
 * 
 */
class CarboniteLoader
{
public:
    CarboniteLoader();
    ~CarboniteLoader();
    
    CarboniteLoader(const CarboniteLoader&) = delete;
    CarboniteLoader& operator=(const CarboniteLoader&) = delete;

    /**
     * Start Carbonite for ovphysx SDK mode.
     *
     * This may reuse an existing Carbonite framework created by another OV library.
     * Carbonite remains responsible for dependency-plugin registration and
     * interface-version compatibility while ovphysx prepares the SDK plugin search paths.
     *
     * This loads only the base Carbonite plugins. The linked PhysX runtime is
     * prepared by loadPhysxPlugins().
     */
    bool initialize();

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
