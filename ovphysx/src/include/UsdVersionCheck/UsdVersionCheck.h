// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @file UsdVersionCheck.h
 * @brief Namespaced USD detection and compatibility checking for C++
 * 
 * ovphysx ships with OV namespaced monolithic USD. This module detects an
 * already-loaded OV namespaced USD runtime so ovphysx can validate that it is
 * compatible and share it with other OV libraries. Classic host USD is
 * intentionally ignored here; it can coexist through the normal host package
 * and plugin paths.
 */

#pragma once

#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"
#include <string>
#include <vector>
#include <utility>

namespace omni {
namespace sdk {
namespace usd_version {

/**
 * @brief Result of USD detection in process
 */
struct UsdDetectionResult {
    bool is_loaded;          ///< True if USD is detected in process
    std::string version;     ///< Version string (e.g., "23.11"), empty if not loaded
    std::string library_path; ///< Path to USD library directory, empty if unknown
};

/**
 * @brief Configuration for USD version requirements
 */
struct UsdConfig {
    bool required;                              ///< Whether USD is required
    std::string version_spec;                   ///< PEP 440 version specification
    std::string build_package;                  ///< Exact packman package id (e.g. "0.25.11.kit.2-gl.19811") ovphysx was built against, or empty if unknown. Used for drift diagnostics against a differently-built USD that happens to satisfy version_spec.
};

/**
 * @brief Configuration for a Carbonite plugin dependency
 */
struct PluginConfig {
    std::string name;      ///< Plugin name (e.g., "omni.physx.plugin")
    bool required;         ///< Whether plugin is required (fail if missing)
    std::string notes;     ///< Optional notes about the plugin
};

/**
 * @brief Configuration for library dependencies
 */
struct DependenciesConfig {
    std::vector<std::string> preload_libraries_linux;  ///< Libraries to preload on Linux with RTLD_GLOBAL
    std::vector<PluginConfig> plugins;                 ///< Carbonite plugins to load
};

/**
 * @brief Library configuration
 */
struct LibraryConfig {
    std::string name;                ///< Library name (e.g., "ovphysx")
    std::string version;             ///< Library version
    std::string description;         ///< Library description
    UsdConfig usd;                   ///< USD requirements
    DependenciesConfig dependencies; ///< Library dependencies
};

/**
 * @brief Detect if OV namespaced USD is already loaded in the current process.
 * 
 * Uses platform-specific methods to check process memory without triggering a
 * load. Classic modular USD libraries such as libusd_tf are ignored.
 * 
 * @return UsdDetectionResult with detection status and version information
 */
OVPHYSX_API UsdDetectionResult detectUsdInProcess();

/**
 * @brief Extract USD version from a namespaced monolithic USD library path.
 * 
 * Prefer the version embedded in library names such as libov_25.11usd_ms.so
 * or ov_25.11usd_ms.dll. Falls back to nearby plugInfo.json metadata when
 * available.
 * 
 * @param library_path Path to USD library directory (optional hint)
 * @return Version string in format "XX.YY" or "XX.YY.ZZ", or "unknown" if extraction fails
 */
OVPHYSX_API std::string extractUsdVersion(const std::string& library_path = "");

/**
 * @brief Check if loaded USD version satisfies requirement specification.
 * 
 * Implements PEP 440-style version comparison for compatibility checking.
 * 
 * @param loaded_version Version string of loaded USD (e.g., "23.11")
 * @param requirement_spec PEP 440 version spec (e.g., "==25.11")
 * @return True if version is compatible, False otherwise
 */
OVPHYSX_API bool checkCompatibility(const std::string& loaded_version, const std::string& requirement_spec);


/**
 * @brief Load library configuration from config.toml file.
 * 
 * **INTERNAL/TESTING USE ONLY** - Called automatically by initializeUsdVersionCheck().
 * 
 * **FATAL ERROR if called after initialization!**
 * The config defines irreversible runtime state:
 * - Preloaded libraries (dlopen RTLD_GLOBAL) cannot be unloaded
 * - Loaded Carbonite plugins cannot be swapped
 * - USD once initialized cannot be reinitialized
 * 
 * Attempting to reload config after initialization will fail with CARB_LOG_ERROR.
 * Direct calls are only safe in isolated test scenarios before initialization.
 * 
 * @param config_path Path to config.toml file
 * @param out_config Output parameter for parsed configuration
 * @return True if loading succeeded, false on error or if already initialized
 */
OVPHYSX_API bool loadConfig(const std::string& config_path, LibraryConfig& out_config);

/**
 * @brief Find the config.toml file for this library.
 * 
 * **INTERNAL USE ONLY** - This function is called automatically by initializeUsdVersionCheck().
 * Direct calls are only needed for testing.
 * 
 * Checks only for config.toml directly alongside the library binary. The build system
 * ensures the config file is copied to this location via POST_BUILD commands.
 * 
 * @param out_path Output parameter for config file path
 * @param library_root Optional library root directory (empty = auto-detect)
 * @return True if found, false if not found (errors logged via CARB_LOG_ERROR)
 */
OVPHYSX_API bool findConfigFile(std::string& out_path, const std::string& library_root = "");

/**
 * @brief Initialize the namespaced USD compatibility check.
 * 
 * This is the single entry point for USD version management. This function:
 * - Finds and loads config.toml
 * - Caches configuration once
 * - Validates any already-loaded OV namespaced USD runtime
 * 
 * **CRITICAL**: This function requires Carbonite to be initialized and MUST NOT
 * be called from DllMain or __attribute__((constructor)) as it can cause deadlocks.
 * Call this AFTER Carbonite initialization (e.g., in ovphysx_create_instance()).
 * 
 * **IDEMPOTENT**: Safe to call multiple times. Returns success immediately if already initialized.
 * 
 * @param library_root Root directory of library installation (optional, auto-detected if empty)
 * @return ovphysx_result_t with status; on failure, call ovphysx_get_last_error() to retrieve the error message
 */
OVPHYSX_API ovphysx_result_t initializeUsdVersionCheck();

/**
 * @brief Cleanup USD version checking system resources.
 * 
 * Should be called during library shutdown to release cached config.
 */
OVPHYSX_API void shutdownUsdVersionCheck();

/**
 * @brief Get preload libraries for Linux from cached configuration.
 * 
 * Returns the list of libraries that must be preloaded on Linux (from dependencies.preload_libraries_linux).
 * Requires initializeUsdVersionCheck() to have been called first.
 * 
 * @return Pointer to vector of library names, or nullptr if config not loaded
 */
OVPHYSX_API const std::vector<std::string>* getPreloadLibrariesLinux();

/**
 * @brief Get Carbonite plugins from cached configuration.
 * 
 * Returns the list of Carbonite plugins to load (from dependencies.plugins).
 * Requires initializeUsdVersionCheck() to have been called first.
 * 
 * @return Pointer to vector of plugin configs, or nullptr if config not loaded
 */
OVPHYSX_API const std::vector<PluginConfig>* getPlugins();

/**
 * @brief Format a detailed error message for OV namespaced USD incompatibility.
 * 
 * @param loaded_version Version string of incompatible USD
 * @param loaded_path Path where USD was loaded from (optional)
 * @param config Library configuration
 * @return Formatted multi-line error message
 */
OVPHYSX_API std::string formatCompatibilityError(const std::string& loaded_version,
                                    const std::string& loaded_path,
                                    const LibraryConfig& config);


/**
 * @brief Helper function to get the directory containing the current shared library.
 * 
 * This is useful for locating config.toml relative to the library itself.
 * 
 * @return Path to directory containing the calling shared library
 */
OVPHYSX_API std::string getLibraryDirectory();

/**
 * @brief Get the Carbonite plugins directory.
 *
 * Carbonite plugins can also be used in Kit-based applications.
 * 
 * Returns the absolute path to the plugins/ directory based on the library location.
 * Common Layout: _install/lib/libovphysx.so -> _install/plugins/
 * 
 * @return Absolute path to plugins directory, or empty string on error
 */
OVPHYSX_API std::string getPluginsDirectory();

/**
 * @brief Get the directory path of a loaded shared library.
 * 
 * This function can locate any library that is already loaded in the process.
 * Useful for discovering where external code (like the application) loaded USD from.
 * 
 * @param library_name Name of the loaded library
 * @return Directory path where the library is loaded from, or empty string if not found
 */
OVPHYSX_API std::string getLoadedLibraryPath(const std::string& library_name);

/**
 * @brief Extract a packman package identifier from a resolved library path.
 *
 * Packman caches packages under paths like
 * `.../packman/chk/usd.py312.manylinux_2_35_x86_64.stock.release/0.25.11.kit.2-gl.19811/lib/libusd_tf.so`.
 * This helper returns the trailing `<version>` component (e.g. `"0.25.11.kit.2-gl.19811"`)
 * so callers can compare it against a build-time-baked identifier.
 *
 * @param resolved_path Absolute, symlink-resolved path to a library file or directory.
 * @return The packman version segment if the path looks packman-shaped, otherwise empty.
 */
OVPHYSX_API std::string extractPackmanPackageId(const std::string& resolved_path);

} // namespace usd_version
} // namespace sdk
} // namespace omni
