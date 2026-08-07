// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @file UsdVersionCheck.cpp
 * @brief USD Version Detection and Compatibility Checking Implementation
 */

#include "UsdVersionCheck/UsdVersionCheck.h"
#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include "internal/sdk/NamespacedUsdLibraryUtils.hpp"
#include "internal/sdk/ovphysxSDK.hpp"

#include <fstream>
#include <sstream>
#include <regex>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <filesystem>

// Carbonite includes for config file parsing
#include <carb/ClientUtils.h>
#include <carb/dictionary/IDictionary.h>
#include <carb/dictionary/ISerializer.h>
#include <carb/dictionary/DictionaryUtils.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/extras/Path.h>
#include <carb/logging/Log.h>

namespace omni {
namespace sdk {
namespace usd_version {

namespace {

// Cached config dictionary (loaded once during library initialization)
static carb::dictionary::Item* g_cached_config = nullptr;

// Cached library configuration (loaded once during library initialization)
static std::unique_ptr<LibraryConfig> g_cached_library_config = nullptr;

std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        // Trim leading whitespace/quotes
        size_t start = token.find_first_not_of(" \t\n\r\"");
        if (start == std::string::npos) {
            // Token is all whitespace/quotes - skip it
            continue;
        }
        
        // Trim trailing whitespace/quotes
        size_t end = token.find_last_not_of(" \t\n\r\"");
        // end cannot be npos here because start was valid
        
        token = token.substr(start, end - start + 1);
        
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    
    return tokens;
}

std::string normalizeUsdVersionForCompatibility(const std::string& version, bool* out_normalized = nullptr) {
    if (out_normalized) {
        *out_normalized = false;
    }
    if (version.empty() || version == "unknown") {
        return version;
    }
    std::vector<std::string> parts = split(version, '.');
    if (parts.size() >= 2 && parts[0] == "0") {
        try {
            int major = std::stoi(parts[0]);
            int minor = std::stoi(parts[1]);
            if (major == 0 && minor >= 20) {
                std::string normalized = parts[1] + "." + (parts.size() > 2 ? parts[2] : "0");
                if (parts.size() > 3) {
                    normalized += "." + parts[3];
                }
                if (out_normalized) {
                    *out_normalized = true;
                }
                return normalized;
            }
        } catch (const std::exception&) {
            return version;
        }
    }
    return version;
}

// Helper: Parse version string into components
struct Version {
    int major = 0;
    int minor = 0;
    int patch = 0;
    
    Version() = default;
    
    explicit Version(const std::string& ver_str) {
        std::vector<std::string> parts = split(ver_str, '.');
        if (parts.size() > 0) major = std::stoi(parts[0]);
        if (parts.size() > 1) minor = std::stoi(parts[1]);
        if (parts.size() > 2) patch = std::stoi(parts[2]);
    }
    
    int compare(const Version& other) const {
        if (major != other.major) return major < other.major ? -1 : 1;
        if (minor != other.minor) return minor < other.minor ? -1 : 1;
        if (patch != other.patch) return patch < other.patch ? -1 : 1;
        return 0;
    }
    
    bool operator<(const Version& other) const { return compare(other) < 0; }
    bool operator<=(const Version& other) const { return compare(other) <= 0; }
    bool operator>(const Version& other) const { return compare(other) > 0; }
    bool operator>=(const Version& other) const { return compare(other) >= 0; }
    bool operator==(const Version& other) const { return compare(other) == 0; }
    bool operator!=(const Version& other) const { return compare(other) != 0; }
    
    std::string toString() const {
        std::stringstream ss;
        ss << major << "." << minor;
        if (patch > 0) ss << "." << patch;
        return ss.str();
    }
};

// Helper: Get directory from a full path (or return path if already a directory)
std::string getDirectoryFromPath(const std::string& path) {
    if (path.empty()) {
        return "";
    }
    size_t last_slash = path.find_last_of("/\\");
    if (last_slash == std::string::npos) {
        return path;
    }
    return path.substr(0, last_slash);
}

// Helper: Parse version from an OV namespaced USD monolith path.
std::string parseVersionFromPath(const std::string& path) {
    // Linux: libov_25.11usd_ms.so
    // Windows: ov_25.11usd_ms.dll
    std::regex pattern(R"((?:^|[/\\])(?:lib)?ov_(\d+)\.(\d+)usd_ms\.(?:so|dll)(?:\.\d+)*)",
                       std::regex::icase);
    std::smatch match;
    if (std::regex_search(path, match, pattern)) {
        return match[1].str() + "." + match[2].str();
    }
    return "";
}

} // anonymous namespace

std::string extractPackmanPackageId(const std::string& resolved_path) {
    if (resolved_path.empty()) {
        return "";
    }

    // Packman caches packages at .../packman/chk/<package-name>/<version>/<files>...
    // Walk parents looking for a cursor whose grandparent is the literal
    // `chk` directory, whose own name starts with a digit (version),
    // and return that cursor's name.
    //
    // Loop termination: std::filesystem::path("/").parent_path() returns "/"
    // on Linux (and similarly for Windows roots), so we must compare each
    // step against the previous cursor and break on the fixed point. An
    // explicit iteration cap is a second safety net in case of surprises
    // (e.g. UNC paths with odd fixpoint semantics on Windows).
    std::filesystem::path cursor = std::filesystem::path(resolved_path).lexically_normal();
    for (int steps = 0; steps < 128 && !cursor.empty() && cursor.has_parent_path(); ++steps) {
        std::filesystem::path parent = cursor.parent_path();
        if (!parent.empty() && parent.has_parent_path()) {
            std::string name = cursor.filename().string();
            std::string grandparent_name = parent.parent_path().filename().string();
            if (grandparent_name == "chk" && !name.empty() &&
                std::isdigit(static_cast<unsigned char>(name[0]))) {
                return name;
            }
        }
        if (parent == cursor) {
            break;  // reached filesystem root (`/` on Linux, drive root on Windows).
        }
        cursor = std::move(parent);
    }
    return "";
}

UsdDetectionResult detectUsdInProcess() {
    UsdDetectionResult result = {false, "", ""};

    // Namespaced ovphysx ignores classic host USD. Only an already-loaded OV
    // namespaced monolith is relevant for compatibility/reuse.
    std::string loaded_full_path = omni::sdk::internal::findLoadedNamespacedUsdLibrary();
    if (!loaded_full_path.empty()) {
        result.is_loaded = true;
        result.library_path = getDirectoryFromPath(loaded_full_path);
        result.version = extractUsdVersion(loaded_full_path);
        CARB_LOG_INFO("USD detect: OV namespaced USD hit (path: %s, version: %s)",
                      loaded_full_path.c_str(),
                      result.version.empty() ? "unknown" : result.version.c_str());
    } else {
        CARB_LOG_INFO("USD detect: no OV namespaced USD library found");
    }

    return result;
}

std::string extractUsdVersion(const std::string& library_path) {
    // Try to parse version from library path
    if (!library_path.empty()) {
        std::string version = parseVersionFromPath(library_path);
        if (!version.empty()) {
            return version;
        }
    }

    // Try to find and parse plugInfo.json
    if (!library_path.empty()) {
        std::string base_path = library_path;
        size_t last_slash = base_path.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            std::string filename = base_path.substr(last_slash + 1);
            if (filename.find(".so") != std::string::npos || filename.find(".dll") != std::string::npos) {
                base_path = base_path.substr(0, last_slash);
            }
        }
        // USD typically has plugInfo.json in various subdirectories
        std::vector<std::string> search_paths = {
            base_path + "/usd/plugInfo.json",
            base_path + "/../share/usd/plugins/plugInfo.json",
            base_path + "/../resources/plugInfo.json"
        };
        
        for (const auto& plug_path : search_paths) {
            std::ifstream file(plug_path);
            if (file.is_open()) {
                std::stringstream buffer;
                buffer << file.rdbuf();
                std::string content = buffer.str();
                
                // Look for version in JSON (very simplified parsing)
                std::regex version_regex(R"xxx("version"\s*:\s*"([^"]+)")xxx");
                std::smatch match;
                if (std::regex_search(content, match, version_regex)) {
                    return match[1].str();
                }
            }
        }
    }
    
    // Note: PXR_VERSION is a compile-time define, not queryable at runtime
    
    return "unknown";
}

bool checkCompatibility(const std::string& loaded_version, const std::string& requirement_spec) {
    if (loaded_version == "unknown") {
        CARB_LOG_WARN("[USD Compatibility] Cannot verify compatibility with unknown USD version");
        // Conservative: allow unknown versions but warn
        return true;
    }
    
    try {
        bool normalized = false;
        std::string normalized_version = normalizeUsdVersionForCompatibility(loaded_version, &normalized);
        Version loaded(normalized_version);
        
        std::vector<std::string> requirements = split(requirement_spec, ',');
        
        for (const auto& req : requirements) {
            // Parse operator and version; supports ~= for PEP 440 compatible release
            std::regex req_regex(R"((~=|>=|<=|>|<|==|!=)?(\d+(?:\.\d+)*))");
            std::smatch match;
            
            if (std::regex_search(req, match, req_regex)) {
                std::string op = match[1].str();
                if (op.empty()) op = "==";
                
                std::string req_version_str = match[2].str();
                Version req_version(req_version_str);
                
                // Handle ~= (compatible release) operator
                // PEP 440: ~=X.Y is equivalent to >=X.Y, ==X.*  (i.e., >=X.Y, <X+1.0)
                //          ~=X.Y.Z is equivalent to >=X.Y.Z, ==X.Y.* (i.e., >=X.Y.Z, <X.Y+1.0)
                if (op == "~=") {
                    // Check lower bound: loaded >= req_version
                    if (loaded < req_version) return false;
                    
                    // Check upper bound based on version precision
                    // Count dots to determine if it's X.Y or X.Y.Z format
                    size_t dot_count = std::count(req_version_str.begin(), req_version_str.end(), '.');
                    
                    if (dot_count >= 2) {
                        // ~=X.Y.Z means <X.Y+1.0 (patch-level compatible)
                        Version upper_bound = req_version;
                        upper_bound.minor++;
                        upper_bound.patch = 0;
                        if (loaded >= upper_bound) return false;
                    } else {
                        // ~=X.Y means <X+1.0 (minor-level compatible)
                        Version upper_bound = req_version;
                        upper_bound.major++;
                        upper_bound.minor = 0;
                        upper_bound.patch = 0;
                        if (loaded >= upper_bound) return false;
                    }
                    continue;
                }
                
                // Evaluate standard constraints
                if (op == ">=" && loaded < req_version) return false;
                if (op == "<=" && loaded > req_version) return false;
                if (op == ">" && loaded <= req_version) return false;
                if (op == "<" && loaded >= req_version) return false;
                if (op == "==" && loaded != req_version) return false;
                if (op == "!=" && loaded == req_version) return false;
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        CARB_LOG_ERROR("[USD Compatibility] Error checking compatibility: %s", e.what());
        // On error, be conservative and allow
        return true;
    }
}

bool loadConfig(const std::string& config_path, LibraryConfig& out_config) {
    // SAFETY CHECK: Prevent config reloading after initialization
    // Loading a new config after initialization would create inconsistent state:
    // - Preloaded libraries (dlopen RTLD_GLOBAL) cannot be unloaded
    // - Loaded plugins cannot be swapped out
    // - USD once initialized cannot be reinitialized
    // This is a programming error - fail fast.
    if (g_cached_library_config) {
        CARB_LOG_ERROR("FATAL: loadConfig() called after initialization");
        CARB_LOG_ERROR("Config defines irreversible runtime state and cannot be reloaded");
        CARB_LOG_ERROR("This is a programming error - config must only be loaded once at startup");
        return false;
    }
    
    // LOADER LOCK SAFETY: Check if Carbonite framework is available
    // This function MUST NOT be called from DllMain/__attribute__((constructor))
    // as it requires Carbonite to be initialized
    carb::Framework* framework = carb::getFramework();
    if (!framework) {
        CARB_LOG_ERROR("FATAL: Carbonite framework not available - config parsing requires Carbonite");
        CARB_LOG_ERROR("This function must be called after Carbonite initialization, not from library constructor");
        return false;
    }
    
    carb::dictionary::ISerializer* serializer = 
        framework->tryAcquireInterface<carb::dictionary::ISerializer>(
            "carb.dictionary.serializer-toml.plugin");
    
    if (!serializer) {
        CARB_LOG_ERROR("Failed to acquire TOML serializer plugin");
        return false;
    }
    
    carb::dictionary::Item* config_dict = 
        carb::dictionary::createDictionaryFromFile(serializer, config_path.c_str());
    
    if (!config_dict) {
        CARB_LOG_ERROR("Failed to parse config file: %s. Check syntax and file format.", config_path.c_str());
        return false;
    }
    
    carb::dictionary::IDictionary* dict = carb::dictionary::getCachedDictionaryInterface();
    if (!dict) {
        CARB_LOG_ERROR("FATAL: Failed to acquire dictionary interface - Carbonite framework issue");
        return false;
    }
    
    // Parse configuration structure with existence checks
    const carb::dictionary::Item* lib_name = dict->getItem(config_dict, "library/name");
    out_config.name = lib_name ? dict->get<std::string>(lib_name) : "";
    
    const carb::dictionary::Item* lib_version = dict->getItem(config_dict, "library/version");
    out_config.version = lib_version ? dict->get<std::string>(lib_version) : "";
    
    const carb::dictionary::Item* lib_desc = dict->getItem(config_dict, "library/description");
    out_config.description = lib_desc ? dict->get<std::string>(lib_desc) : "";
    
    const carb::dictionary::Item* usd_required = dict->getItem(config_dict, "usd/required");
    out_config.usd.required = usd_required ? dict->get<bool>(usd_required) : false;
    
    const carb::dictionary::Item* usd_version_spec = dict->getItem(config_dict, "usd/version_spec");
    out_config.usd.version_spec = usd_version_spec ? dict->get<std::string>(usd_version_spec) : "";

    const carb::dictionary::Item* usd_build_package = dict->getItem(config_dict, "usd/build_package");
    out_config.usd.build_package = usd_build_package ? dict->get<std::string>(usd_build_package) : "";
    
    // Parse dependencies.preload_libraries_linux array
    const carb::dictionary::Item* preload_array = dict->getItem(config_dict, "dependencies/preload_libraries_linux");
    if (preload_array) {
        size_t count = dict->getItemChildCount(preload_array);
        for (size_t i = 0; i < count; i++) {
            std::string index_key = std::to_string(i);
            const carb::dictionary::Item* lib_item = dict->getItem(preload_array, index_key);
            if (lib_item) {
                std::string preload_lib = dict->get<std::string>(lib_item);
                if (!preload_lib.empty()) {
                    out_config.dependencies.preload_libraries_linux.push_back(preload_lib);
                }
            }
        }
    }
    
    // Parse dependencies.plugins array (array of tables in TOML)
    const carb::dictionary::Item* plugins_array = dict->getItem(config_dict, "dependencies/plugins");
    if (plugins_array) {
        size_t count = dict->getItemChildCount(plugins_array);
        for (size_t i = 0; i < count; i++) {
            std::string index_key = std::to_string(i);
            const carb::dictionary::Item* plugin_item = dict->getItem(plugins_array, index_key);
            if (plugin_item) {
                PluginConfig plugin;
                
                const carb::dictionary::Item* name_item = dict->getItem(plugin_item, "name");
                plugin.name = name_item ? dict->get<std::string>(name_item) : "";
                
                const carb::dictionary::Item* required_item = dict->getItem(plugin_item, "required");
                plugin.required = required_item ? dict->get<bool>(required_item) : false;
                
                const carb::dictionary::Item* notes_item = dict->getItem(plugin_item, "notes");
                plugin.notes = notes_item ? dict->get<std::string>(notes_item) : "";
                
                if (!plugin.name.empty()) {
                    out_config.dependencies.plugins.push_back(plugin);
                }
            }
        }
    }
    
    // Cache the dictionary for later use (schema validation, etc.)
    if (g_cached_config) {
        dict->destroyItem(g_cached_config);
    }
    g_cached_config = config_dict;
    
    return true;
}

std::string formatCompatibilityError(const std::string& loaded_version,
                                    const std::string& loaded_path,
                                    const LibraryConfig& config) {
    std::stringstream ss;
    
    ss << "\n" << std::string(70, '=') << "\n";
    ss << "USD VERSION INCOMPATIBILITY DETECTED\n";
    ss << std::string(70, '=') << "\n\n";
    
    ss << "Library: " << config.name << " v" << config.version << "\n";
    ss << "  Required USD: " << config.usd.version_spec << "\n\n";
    
    ss << "Current Process:\n";
    ss << "  USD Version: " << loaded_version << " (INCOMPATIBLE)\n";
    
    if (!loaded_path.empty()) {
        ss << "  Loaded From: " << loaded_path << "\n";
    }
    
    ss << "\nThe process already loaded an OV namespaced USD runtime that does not match "
       << config.name << ". To fix this:\n";
    ss << "  - Use matching OV runtime packages so all OV libraries share the same\n";
    ss << "    namespaced USD ABI\n";
    ss << "  - Or let " << config.name << " load its packaged namespaced USD before any\n";
    ss << "    incompatible OV namespaced USD runtime is loaded\n";
    ss << "  - Classic host USD can coexist, but it must stay isolated through the\n";
    ss << "    normal host package and plugin paths\n\n";
    
    ss << "For more information, see documentation on USD version management.\n";
    ss << std::string(70, '=') << "\n";
    
    return ss.str();
}

bool findConfigFile(std::string& out_path, const std::string& library_root) {
    // LOADER LOCK SAFETY: Check if Carbonite framework is available
    // This function MUST NOT be called from DllMain/__attribute__((constructor))
    carb::Framework* framework = carb::getFramework();
    if (!framework) {
        CARB_LOG_ERROR("FATAL: Carbonite framework not available - cannot search for config file");
        CARB_LOG_ERROR("This function must be called after Carbonite initialization, not from library constructor");
        return false;
    }
    
    carb::filesystem::IFileSystem* fs = carb::getCachedInterface<carb::filesystem::IFileSystem>();
    if (!fs) {
        CARB_LOG_ERROR("Failed to acquire filesystem interface");
        return false;
    }
    
    // Config file is always alongside the library binary
    // Build system ensures this via POST_BUILD copy commands
    std::string lib_dir = library_root.empty() ? getLibraryDirectory() : library_root;
    carb::extras::Path config_path = carb::extras::Path(lib_dir) / "config.toml";
    const std::string config_path_string = config_path.getString();
#if carb_filesystem_IFileSystem >= CARB_HEXVERSION(2, 0)
    if (fs->exists(config_path_string)) {
#else
    if (fs->exists(config_path_string.c_str())) {
#endif
        out_path = config_path.getString();
        return true;
    }
    
    CARB_LOG_ERROR("Config file not found at: %s", config_path.getString().c_str());
    CARB_LOG_ERROR("Build system should copy config.toml alongside library binary");
    return false;
}

static ovphysx_result_t checkUsdCompatibility(
    const std::string& config_path,
    const std::string& library_root
) {
    // Config must be loaded by initializeUsdVersionCheck() before this is called
    if (!g_cached_library_config) {
        std::string error_msg = "FATAL: checkUsdCompatibility() called before config loaded. "
                                "This is a programming error - initializeUsdVersionCheck() must be called first";
        CARB_LOG_ERROR("%s", error_msg.c_str());
        return set_error(OVPHYSX_API_ERROR, error_msg);
    }
    
    const LibraryConfig& config = *g_cached_library_config;
    if (!config.usd.required) {
        return success();
    }
    
    auto detection = detectUsdInProcess();
    if (detection.is_loaded) {
        // USD already loaded - validate compatibility
        if (detection.version == "unknown") {
            if (!detection.library_path.empty()) {
                CARB_LOG_WARN(
                    "[USD Compatibility] OV namespaced USD is loaded but version could not be determined (path: %s). "
                    "Expected a versioned library name (e.g., libov_25.11usd_ms.so).",
                    detection.library_path.c_str());
            } else {
                CARB_LOG_WARN(
                    "[USD Compatibility] OV namespaced USD is loaded but version could not be determined. "
                    "Expected a versioned library name (e.g., libov_25.11usd_ms.so).");
            }
        }
        bool normalized = false;
        std::string normalized_version = normalizeUsdVersionForCompatibility(detection.version, &normalized);
        if (!checkCompatibility(normalized_version, config.usd.version_spec)) {
            std::string reported_version = detection.version;
            if (normalized && detection.version != "unknown") {
                reported_version += " (normalized to " + normalized_version + ")";
            }
            std::string error_msg = formatCompatibilityError(
                reported_version, detection.library_path, config
            );
            CARB_LOG_ERROR("%s", error_msg.c_str());
            return set_error(OVPHYSX_API_ERROR, error_msg);
        }

        // Packman-id drift check. version_spec passed above, but a different
        // packman build of the same major.minor (e.g. 25.11.kit.1 vs 25.11.kit.2)
        // can still be ABI-incompatible with the USD ovphysx was linked against.
        // This block surfaces that drift so callers have an actionable log line
        // instead of a silent mismatch that manifests only later as
        // "Dependency: [omni::physics::schema::IUsdPhysics v1.1] failed to be
        // resolved." during Carbonite plugin loading.
        if (!config.usd.build_package.empty()) {
            std::string loaded_pkg_id = extractPackmanPackageId(detection.library_path);
            if (loaded_pkg_id.empty() && detection.version != "unknown") {
                CARB_LOG_INFO(
                    "[%s] USD loaded from a non-packman path (%s); cannot cross-check "
                    "against build-time package %s.",
                    config.name.c_str(),
                    detection.library_path.empty() ? "<unknown path>" : detection.library_path.c_str(),
                    config.usd.build_package.c_str());
            } else if (!loaded_pkg_id.empty() && loaded_pkg_id != config.usd.build_package) {
                CARB_LOG_WARN(
                    "[%s] USD build drift: ovphysx was built against packman package '%s', "
                    "but the USD currently loaded in this process is '%s' (from %s). "
                    "The version spec (%s) is still satisfied, but these are different "
                    "builds and can be ABI-incompatible -- if downstream Carbonite plugins "
                    "later fail to resolve USD-dependent interfaces, align both libraries "
                    "on the same USD packman pin.",
                    config.name.c_str(),
                    config.usd.build_package.c_str(),
                    loaded_pkg_id.c_str(),
                    detection.library_path.empty() ? "<unknown path>" : detection.library_path.c_str(),
                    config.usd.version_spec.c_str());
            }
        }

        if (normalized && detection.version != "unknown") {
            CARB_LOG_INFO("[%s] Using USD %s (normalized to %s, compatible)",
                          config.name.c_str(), detection.version.c_str(), normalized_version.c_str());
        } else {
            CARB_LOG_INFO("[%s] Using USD %s (compatible)", config.name.c_str(), detection.version.c_str());
        }
    } else {
        CARB_LOG_INFO("[%s] No USD loaded (will rely on SDK preload)", config.name.c_str());
    }
    
    return success();
}

ovphysx_result_t initializeUsdVersionCheck() {
    // LOADER LOCK SAFETY: This function requires Carbonite to be initialized
    // and MUST NOT be called from DllMain/__attribute__((constructor)) as it:
    // - Calls carb::getFramework() and acquires interfaces (potential deadlock)
    // - Loads TOML config files using Carbonite plugins
    // - May trigger plugin loads and framework initialization
    //
    // Call this AFTER Carbonite is initialized (e.g., in ovphysx_create_instance())
    // If already initialized (idempotent), returns success immediately.
    if (g_cached_library_config) {
        return success();
    }
    
    std::string lib_root = getLibraryDirectory();
    
    std::string config_path;
    if (!findConfigFile(config_path, lib_root)) {
        std::string error_msg = "FATAL: config.toml not found. This file is required for USD version validation and library initialization. "
                                "The library cannot function correctly without it. Check installation integrity.";
        CARB_LOG_ERROR("%s", error_msg.c_str());
        return set_error(OVPHYSX_API_ERROR, error_msg);
    }
    
    LibraryConfig parsedConfig;
    if (!loadConfig(config_path, parsedConfig)) {
        std::string error_msg = "FATAL: Failed to parse config.toml. Check file syntax and format.";
        CARB_LOG_ERROR("%s", error_msg.c_str());
        return set_error(OVPHYSX_API_ERROR, error_msg);
    }
    g_cached_library_config = std::make_unique<LibraryConfig>(std::move(parsedConfig));
    
    ovphysx_result_t result = checkUsdCompatibility(config_path, lib_root);
    
    // If compatibility check failed, clear cached config to ensure subsequent calls
    // don't incorrectly report success (idempotence requirement)
    if (result.status != OVPHYSX_API_SUCCESS) {
        g_cached_library_config.reset();
    }
    
    return result;
}

void shutdownUsdVersionCheck() {
    // LOADER LOCK SAFETY: This function is safe to call from library destructor
    // even if initialization was deferred or never called.
    // It only cleans up resources if they were actually allocated.
    
    // NOTE: We intentionally do NOT call destroyItem() on g_cached_config here.
    // When called from library destructor (__attribute__((destructor))), Carbonite
    // plugins are shutting down and the dictionary plugin will clean up its own
    // items. Calling destroyItem() during shutdown causes a double-free.
    // Just clear our pointer and let Carbonite handle its own cleanup.
    g_cached_config = nullptr;
    g_cached_library_config.reset();
}

const std::vector<std::string>* getPreloadLibrariesLinux() {
    if (!g_cached_library_config) {
        CARB_LOG_ERROR("Config not loaded. Call initializeUsdVersionCheck() first.");
        return nullptr;
    }
    return &g_cached_library_config->dependencies.preload_libraries_linux;
}

const std::vector<PluginConfig>* getPlugins() {
    if (!g_cached_library_config) {
        CARB_LOG_ERROR("Config not loaded. Call initializeUsdVersionCheck() first.");
        return nullptr;
    }
    return &g_cached_library_config->dependencies.plugins;
}

std::string getLibraryDirectory() {
    return omni::sdk::usd_schema_paths::getLibraryDirectory();
}

std::string getPluginsDirectory() {
    return omni::sdk::usd_schema_paths::getPluginsDirectory();
}

std::string getLoadedLibraryPath(const std::string& library_name) {
#ifdef _WIN32
    HMODULE hModule = GetModuleHandleA(library_name.c_str());
    if (hModule != NULL) {
        char path[MAX_PATH];
        if (GetModuleFileNameA(hModule, path, MAX_PATH) > 0) {
            std::string full_path(path);
            size_t last_slash = full_path.find_last_of("\\/");
            if (last_slash != std::string::npos) {
                return full_path.substr(0, last_slash);
            }
        }
    }
    return "";
#else
    void* handle = dlopen(library_name.c_str(), RTLD_NOLOAD | RTLD_LAZY);
    if (handle != nullptr) {
        struct link_map* map = nullptr;
        std::string result;
        if (dlinfo(handle, RTLD_DI_LINKMAP, &map) == 0 && map != nullptr) {
            std::string full_path(map->l_name);
            size_t last_slash = full_path.find_last_of('/');
            if (last_slash != std::string::npos) {
                std::string dir = full_path.substr(0, last_slash);
                // Resolve any relative path components
                char resolved_path[PATH_MAX];
                if (realpath(dir.c_str(), resolved_path) != nullptr) {
                    result = std::string(resolved_path);
                } else {
                    result = dir;
                }
            }
        }
        dlclose(handle);
        return result;
    }
    return "";
#endif
}

} // namespace usd_version
} // namespace sdk
} // namespace omni
