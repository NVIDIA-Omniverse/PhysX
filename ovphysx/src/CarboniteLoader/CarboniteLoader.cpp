// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PACKAGING-USDFREE-001
 * @covers AC-6
 *
 * @implements REQ-PACKAGING-OMNICLIENT-001
 * @covers AC-2
 *
 * @implements REQ-PACKAGING-CLOSURE-001
 * @covers AC-2
 */

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "ovphysx/ovphysx_types.h"
#include "LogManager.hpp"
#include <omni/physx/PhysXRuntime.h>

#include <cstdlib>
#include <cstring>
#include <chrono>
#include <mutex>
#include <random>
#include <string>
#include <climits>
#include <filesystem>
#include <atomic>
#include <system_error>
#include <thread>
#include <unordered_set>
#include <vector>

#include "UsdSchemaPaths/UsdSchemaPaths.h"

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
    #define PATH_MAX MAX_PATH
#else
    #include <fcntl.h>
    #include <sys/stat.h>
    #include <unistd.h>
#endif

// Carbonite
#include <carb/ClientUtils.h>
#include <carb/dictionary/IDictionary.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/logging/Log.h>
#include <carb/logging/Logger.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <carb/tokens/ITokens.h>
#include <omni/core/Omni.h>
#include <omni/ext/IExt.h>
#include <omni/physics/tensors/TensorApi.h>

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSettings.h>

// Static Carbonite plugins do not self-register when their archives are linked
// into ovphysx. Each generated registerPlugin symbol has to be referenced and
// called explicitly so the linker keeps the archive object and the interfaces
// become visible to this carb::Framework instance.
extern "C" bool carb_assets_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_datasource_file_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_dictionary_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_dictionary_serializer_json_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_dictionary_serializer_toml_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_eventdispatcher_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_events_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_settings_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_tasking_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_tokens_plugin_registerPlugin(carb::Framework*);
extern "C" bool carb_variant_plugin_registerPlugin(carb::Framework*);

CARB_STATIC_BINARY_GLOBALS("omni_physx_sdk")

bool isProcessGpuDisabled();

namespace ovphysx
{

namespace
{
    static std::mutex g_bootstrapMutex;
    static bool g_bootstrapDone = false;
    // Live CarboniteLoaders holding the process-wide framework. Guards teardown of the
    // process-private cooked-collider cache (see releaseProcessCacheDirLocked).
    static std::atomic<int> g_activeLoaders{ 0 };
    // Note: /physics/suppressReadback (DirectGPU-API mode) is NOT managed here.
    // It is opt-in by the host, which sets the Carbonite setting before any
    // ovphysx call. ovphysx never writes it because DirectGPU is incompatible
    // with contact modification (surface velocity, custom contact callbacks).
    // See the create_args doc comment in ovphysx_types.h for the trade-off.
    // After-load interface probe. Each entry pairs a plugin name (used only for
    // the error message) with a check that the interface version ovphysx was
    // compiled against is acquirable.
    //
    // tryAcquireInterface<T>() matches (name, major, minor) against what the
    // loaded plugin advertises, so the probe also fails when a foreign host
    // loaded a same-named plugin at a different major or an older minor. That
    // skew would otherwise surface later as "Dependency: <iface> failed to be
    // resolved" or a null pointer dereference inside ovphysx. A plugin that
    // did not load at all fails the probe the same way.
    struct PluginProbe
    {
        const char* plugin;
        bool (*acquired)(carb::Framework*);
    };
    // tryAcquireInterface is non-owning for the singleton plugin interfaces
    // probed below, so the returned pointer needs no release and is discarded.
    // This is a presence and version check, not an acquisition.
#define OVPHYSX_PROBE(plugin_name, IFACE)                                      \
    {                                                                          \
        plugin_name,                                                           \
        [](carb::Framework* fw) -> bool {                                      \
            return fw->tryAcquireInterface<IFACE>() != nullptr;                \
        }                                                                      \
    }

    static const PluginProbe kFoundationProbes[] = {
        OVPHYSX_PROBE("carb.dictionary.plugin", carb::dictionary::IDictionary),
        OVPHYSX_PROBE("carb.settings.plugin",   carb::settings::ISettings),
        OVPHYSX_PROBE("carb.tokens.plugin",     carb::tokens::ITokens),
        OVPHYSX_PROBE("carb.tasking.plugin",    carb::tasking::ITasking),
        OVPHYSX_PROBE("carb.filesystem.plugin", carb::filesystem::IFileSystem),
    };

#undef OVPHYSX_PROBE

    // Probes each plugin's primary interface and logs the ones that did not
    // resolve. A false return is a fatal load-time error: ovphysx cannot
    // continue without these interfaces.
    static bool verifyLoadedInterfaces(carb::Framework* framework,
                                       const PluginProbe* probes,
                                       size_t count,
                                       const char* stage)
    {
        std::vector<const char*> missing;
        missing.reserve(count);
        for (size_t i = 0; i < count; ++i)
        {
            if (!probes[i].acquired(framework))
            {
                missing.push_back(probes[i].plugin);
            }
        }
        if (missing.empty())
        {
            return true;
        }

        std::string list;
        for (size_t i = 0; i < missing.size(); ++i)
        {
            if (i)
                list += ", ";
            list += missing[i];
        }
        CARB_LOG_ERROR(
            "[CarboniteLoader] %s: ovphysx could not acquire its expected "
            "interfaces from %zu plugin(s): %s. This typically means a foreign-host "
            "plugin advertises an incompatible interface version (carb's "
            "tryAcquireInterface requires same major and minor>=requested), or the "
            "plugin failed to load. Common causes: ovphysx and the host library "
            "(e.g. ovrtx) were built against different Carbonite/USD "
            "versions. Set OVPHYSX_COEXIST_REFUSE=1 to refuse coexistence at load "
            "time, or align the host's plugin set with ovphysx's expected versions.",
            stage, missing.size(), list.c_str());
        return false;
    }

    // Force the settings ovphysx needs before PhysX plugins load. These are not
    // user preferences. They define the SDK runtime shape: USD writeback off,
    // no renderer or NGX side systems.
    static void enforceRequiredSettings(carb::Framework* framework)
    {
        auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (!settings)
            return;

        struct RequiredSetting { const char* key; bool required; const char* reason; };
        const RequiredSetting requiredSettings[] = {
            {"/physics/updateToUsd",  false, "ovphysx reads state via tensor API, not USD writeback"},
            {"/ngx/enabled",          false, "ovphysx is headless -- NGX not needed"},
            {"/renderer/enabled",     false, "ovphysx is headless -- renderer not needed"},
        };
        for (const auto& s : requiredSettings)
        {
            if (settings->isAccessibleAs(carb::dictionary::ItemType::eBool, s.key))
            {
                bool current = settings->getAsBool(s.key);
                if (current != s.required)
                {
                    CARB_LOG_WARN("[CarboniteLoader] Overriding %s from %s to %s (%s)",
                                  s.key, current ? "true" : "false",
                                  s.required ? "true" : "false", s.reason);
                }
            }
            settings->setBool(s.key, s.required);
        }
    }

#ifdef _WIN32
    bool addToPath(const std::string& dir)
    {
        if (dir.empty()) return false;
        const char* currentPath = std::getenv("PATH");
        std::string newPath = dir;
        if (currentPath && currentPath[0] != '\0')
        {
            newPath += ";";
            newPath += currentPath;
        }
        return _putenv_s("PATH", newPath.c_str()) == 0;
    }
#endif
}

struct CarboniteLoader::Impl
{
    bool frameworkAcquired = false;
    std::string pluginsDir;  // Path to _install/plugins/
    std::string lastError;
    omni::physx::IPhysxSimulation* physxSim = nullptr;
};

CarboniteLoader::CarboniteLoader()
    : m(new Impl())
{
}

CarboniteLoader::~CarboniteLoader()
{
    shutdown();
    delete m;
    m = nullptr;
}

const std::string& CarboniteLoader::getLastError() const
{
    static const std::string empty;
    return m ? m->lastError : empty;
}

// First half of ovphysx startup.
// This prepares Carbonite, core settings, app paths, logging, and base plugins.
// It deliberately stops before loading the PhysX plugins so callers can apply
// user config before PhysX starts.
bool CarboniteLoader::initialize()
{
    std::lock_guard<std::mutex> guard(g_bootstrapMutex);
    if (m)
        m->lastError.clear();
    
    if (g_bootstrapDone)
    {
        // Re-apply log level and register pending callbacks (level may change between instances)
        ovphysx::onCarboniteLoggingReady();

        // Re-populate the per-instance plugins path for subsequent instances.
        m->frameworkAcquired = true;
        g_activeLoaders.fetch_add(1, std::memory_order_relaxed);
        m->pluginsDir = omni::sdk::usd_schema_paths::getPluginsDirectory();
        if (m->pluginsDir.empty())
        {
            CARB_LOG_ERROR("[CarboniteLoader] Failed to determine plugins directory (bootstrap re-init)");
            return false;
        }

        m->physxSim = omni::physx::runtime::tryGetPhysxSimulationInterface();
        if (!m->physxSim)
        {
            CARB_LOG_INFO("[CarboniteLoader] Base bootstrap already complete; PhysX runtime is not started yet");
        }
        return true;
    }
    
    // Determine plugins directory (sibling to lib/), with Windows fallbacks.
    m->pluginsDir = omni::sdk::usd_schema_paths::getPluginsDirectory();
#ifdef _WIN32
    if (m->pluginsDir.empty() || !std::filesystem::exists(m->pluginsDir))
    {
        // Fall back to the directory where ovphysx.dll is loaded from.
        std::string moduleDir = omni::sdk::usd_schema_paths::getLoadedLibraryDirectory("ovphysx.dll");
        if (!moduleDir.empty())
        {
            std::filesystem::path candidate = std::filesystem::path(moduleDir) / "plugins";
            if (std::filesystem::exists(candidate))
            {
                m->pluginsDir = candidate.string();
            }
            else
            {
                candidate = std::filesystem::path(moduleDir).parent_path() / "plugins";
                if (std::filesystem::exists(candidate))
                {
                    m->pluginsDir = candidate.string();
                }
            }
        }
    }
#endif
    if (m->pluginsDir.empty())
    {
        CARB_LOG_ERROR("[CarboniteLoader] Failed to determine plugins directory");
        return false;
    }
    CARB_LOG_INFO("[CarboniteLoader] Loading plugins from: %s", m->pluginsDir.c_str());

#ifdef _WIN32
    addToPath(m->pluginsDir);
#endif

    // carb::getFramework() returns the module-local pointer. When ovphysx is
    // loaded as a regular shared library (via Python / ctypes) rather than as a
    // Carbonite plugin, that pointer is null even if the process already has a
    // framework. acquireFrameworkAndRegisterBuiltins() finds the existing
    // process-wide framework and sets the local pointer. Whether the framework
    // existed already is tracked for diagnostics.
    carb::Framework* framework = carb::getFramework();
    bool frameworkAlreadyExisted = (framework != nullptr);
    if (!framework)
    {
        OmniCoreStartArgs coreArgs{};
        coreArgs.flags = fStartFlagDisableIStructuredLog;
        framework = carb::acquireFrameworkAndRegisterBuiltins(&coreArgs);
        if (!framework)
        {
            CARB_LOG_ERROR("[CarboniteLoader] Failed to acquire Carbonite framework");
            return false;
        }
        // ctypes/shared-library embedding case: carb::getFramework() returns null because
        // the module-local pointer was unset, but after acquire the process-wide framework
        // may already contain plugins loaded by another client.
        if (framework->getPluginCount() > 0)
            frameworkAlreadyExisted = true;
    }
    m->frameworkAcquired = true;
    g_activeLoaders.fetch_add(1, std::memory_order_relaxed);

    // Apply the global log level and register any pending user callbacks.
    ovphysx::onCarboniteLoggingReady();

    std::unordered_set<std::string> preExistingPluginNames;
    if (frameworkAlreadyExisted)
    {
        // ====================================================================
        // The Carbonite framework pre-exists with no direct PhysX runtime yet.
        // Where the pre-existing plugins live distinguishes two cases:
        //   (a) every plugin has a null libPath or lives under ovphysx's own
        //       install tree. These are static built-ins registered by
        //       acquireFrameworkAndRegisterBuiltins() or a re-entry within
        //       this process. Proceed with plugin loading.
        //   (b) some plugin libPath points outside ovphysx's install tree.
        //       Another library (ovrtx, etc.) has bootstrapped Carbonite and
        //       populated its Framework. Registering into it can produce a
        //       torn registry ("Ignoring plugin: same name already loaded" on
        //       SONAME collisions) and the silent "Dependency:
        //       [omni::physics::schema::IUsdPhysics v1.1] failed to be
        //       resolved" cascade downstream.
        //
        // The parent of pluginsDir (the _install/ root) is the ownership
        // boundary, so plugins loaded from _install/lib/ or _install/plugins/
        // both count as ovphysx's own.
        // ====================================================================
        const size_t preExistingPluginCount = framework->getPluginCount();
        if (preExistingPluginCount > 0)
        {
            std::vector<carb::PluginDesc> loaded(preExistingPluginCount);
            framework->getPlugins(loaded.data());

            std::error_code canonEc;
            std::filesystem::path ourRoot;
            if (!m->pluginsDir.empty())
            {
                ourRoot = std::filesystem::weakly_canonical(
                    std::filesystem::path(m->pluginsDir).parent_path(), canonEc);
                if (canonEc)
                {
                    ourRoot = std::filesystem::path(m->pluginsDir).parent_path();
                }
            }
            const std::string ourRootStr = ourRoot.string();

            // Surface the inputs to the foreign-plugin classification. This is
            // only diagnostic in the static runtime shape, but still helps catch
            // staging configurations where ovrtx and ovphysx share install-root
            // prefixes and isUnderOurRoot false-positives every plugin as "ours".
            CARB_LOG_INFO("[CarboniteLoader] coexist-detect: preExistingPluginCount=%zu, ourRootStr=\"%s\", m->pluginsDir=\"%s\"",
                          preExistingPluginCount, ourRootStr.c_str(), m->pluginsDir.c_str());

            auto isUnderOurRoot = [&ourRootStr](const std::string& absPath) {
                if (ourRootStr.empty()) return true;  // unknown install root, cannot classify, assume ours
                if (absPath.empty()) return false;
                if (absPath.compare(0, ourRootStr.size(), ourRootStr) != 0) return false;
                // Require a path separator (or exact match) after the prefix so
                // "/install/x" doesn't match "/install-sibling/y".
                if (absPath.size() == ourRootStr.size()) return true;
                char c = absPath[ourRootStr.size()];
                return c == '/' || c == '\\';
            };

            std::string foreignExample;
            size_t foreignCount = 0;
            size_t builtinCount = 0;
            size_t ownedCount = 0;
            for (const carb::PluginDesc& desc : loaded)
            {
                if (desc.impl.name)
                {
                    preExistingPluginNames.emplace(desc.impl.name);
                }

                if (!desc.libPath || desc.libPath[0] == '\0')
                {
                    // Static / built-in plugin with no file on disk to attribute.
                    ++builtinCount;
                    continue;
                }
                std::error_code ec;
                auto resolved = std::filesystem::weakly_canonical(
                    std::filesystem::path(desc.libPath), ec);
                const std::string absPath = ec ? std::string(desc.libPath) : resolved.string();
                const bool ours = isUnderOurRoot(absPath);
                CARB_LOG_INFO("[CarboniteLoader] coexist-detect plugin: name=\"%s\" libPath=\"%s\" classifiedAs=%s",
                              desc.impl.name ? desc.impl.name : "<unnamed>",
                              absPath.c_str(),
                              ours ? "ours" : "FOREIGN");
                if (ours)
                {
                    ++ownedCount;
                    continue;
                }
                ++foreignCount;
                if (foreignExample.empty())
                {
                    foreignExample = desc.impl.name ? desc.impl.name : "<unnamed>";
                    foreignExample += " from ";
                    foreignExample += absPath;
                }
            }
            CARB_LOG_INFO("[CarboniteLoader] coexist-detect: builtinCount=%zu ownedCount=%zu foreignCount=%zu",
                          builtinCount, ownedCount, foreignCount);

            if (foreignCount > 0)
            {
                // Coexistence is the default. The verifyLoadedInterfaces probes
                // below produce a named-cause error if any expected interface
                // fails to resolve at the version ovphysx was built against,
                // which covers both a missing plugin and one loaded at a wrong
                // version by a foreign host. OVPHYSX_COEXIST_REFUSE=1 fails
                // fast at load instead.
                const char* refuseEnv = std::getenv("OVPHYSX_COEXIST_REFUSE");
                const bool refuse = refuseEnv && refuseEnv[0] == '1';
                if (refuse)
                {
                    CARB_LOG_ERROR(
                        "[CarboniteLoader] Refusing coexistence (OVPHYSX_COEXIST_REFUSE=1): "
                        "another library has bootstrapped Carbonite with %zu plugin(s) loaded "
                        "outside ovphysx's install tree (e.g. %s). With this opt-out set, "
                        "ovphysx fails fast at load rather than attempting to share the "
                        "framework. Unset OVPHYSX_COEXIST_REFUSE (or set it to anything "
                        "other than 1) to allow coexistence; or initialize ovphysx before "
                        "the other library, or run ovphysx in a separate process.",
                        foreignCount, foreignExample.c_str());
                    return false;
                }
                CARB_LOG_INFO(
                    "[CarboniteLoader] Foreign Carbonite framework detected: %zu plugin(s) "
                    "loaded outside ovphysx's install tree (e.g. %s). Proceeding with "
                    "coexistence; the interface probes below will surface any "
                    "version skew. Set OVPHYSX_COEXIST_REFUSE=1 to opt out.",
                    foreignCount, foreignExample.c_str());

                // Another library (e.g. ovrtx) bootstrapped Carbonite before
                // ovphysx was loaded. ovphysx never releases the framework, so
                // teardown is owned by process exit.
            }
            else
            {
                CARB_LOG_INFO(
                    "[CarboniteLoader] Carbonite framework pre-exists with %zu plugin(s), "
                    "all under our install tree or static built-ins -- proceeding with "
                    "plugin loading",
                    preExistingPluginCount);
            }
        }
        else
        {
            CARB_LOG_INFO("[CarboniteLoader] Carbonite framework pre-exists but is empty -- proceeding with plugin loading");
        }
    }

    CARB_LOG_INFO("[CarboniteLoader] Plugins directory: %s", m->pluginsDir.c_str());

    // Configure plugin search paths. The main plugins directory is always
    // searched. "plugins/bin/deps" belongs to older install layouts and is
    // added only when it exists, so those layouts keep working.
    auto [searchPathStrings, searchPaths] = buildSearchPaths();
    const size_t searchPathCount = searchPaths.size();
    
    // Register the core Carbonite plugins. This has to happen before the GPU
    // and monitoring plugins load.
    // Static plugin registration is not idempotent: Carbonite requires unique
    // plugin names and fails a duplicate registration. The coexistence snapshot
    // above is consulted first because getPluginDesc() logs a warning for every
    // miss, which would report one false warning per static plugin on a clean
    // standalone startup.
    struct StaticPluginRegistration
    {
        const char* name;
        bool (*registerPlugin)(carb::Framework*);
    };
    const StaticPluginRegistration staticPlugins[] = {
        { "carb.dictionary.plugin", carb_dictionary_plugin_registerPlugin },
        { "carb.settings.plugin", carb_settings_plugin_registerPlugin },
        { "carb.tokens.plugin", carb_tokens_plugin_registerPlugin },
        { "carb.variant.plugin", carb_variant_plugin_registerPlugin },
        { "carb.eventdispatcher.plugin", carb_eventdispatcher_plugin_registerPlugin },
        { "carb.events.plugin", carb_events_plugin_registerPlugin },
        { "carb.tasking.plugin", carb_tasking_plugin_registerPlugin },
        { "carb.assets.plugin", carb_assets_plugin_registerPlugin },
        { "carb.datasource-file.plugin", carb_datasource_file_plugin_registerPlugin },
        { "carb.dictionary.serializer-toml.plugin", carb_dictionary_serializer_toml_plugin_registerPlugin },
        { "carb.dictionary.serializer-json.plugin", carb_dictionary_serializer_json_plugin_registerPlugin },
    };
    const auto hasRegisteredPlugin = [framework](const char* name) {
        const carb::PluginDesc& desc = framework->getPluginDesc(name);
        return desc.impl.name && std::strcmp(desc.impl.name, name) == 0;
    };
    for (const StaticPluginRegistration& plugin : staticPlugins)
    {
        // The process-shared framework is mutable, so snapshot hits are revalidated.
        // getPluginDesc() is silent on a hit, and a co-tenant may have unregistered the plugin.
        if (preExistingPluginNames.count(plugin.name) != 0 && hasRegisteredPlugin(plugin.name))
        {
            CARB_LOG_INFO("[CarboniteLoader] Static Carbonite plugin already registered, skipping: %s", plugin.name);
            continue;
        }

        if (!plugin.registerPlugin(framework))
        {
            if (hasRegisteredPlugin(plugin.name))
            {
                CARB_LOG_INFO("[CarboniteLoader] Static Carbonite plugin became registered, skipping: %s", plugin.name);
                continue;
            }

            CARB_LOG_ERROR("[CarboniteLoader] Failed to register static Carbonite plugin: %s", plugin.name);
            return false;
        }
    }
    if (!verifyLoadedInterfaces(framework,
                                kFoundationProbes,
                                sizeof(kFoundationProbes) / sizeof(kFoundationProbes[0]),
                                "Foundation"))
    {
        return false;
    }

    // Static Carbonite builds do not use the app-directory path for ovphysx
    // plugin discovery. Runtime paths are derived from the loaded ovphysx module.
    CARB_LOG_INFO("[CarboniteLoader] Skipping app-directory configuration; ovphysx derives runtime paths from its module");
    
    enforceRequiredSettings(framework);
    if (auto* settings = framework->tryAcquireInterface<carb::settings::ISettings>())
    {
        
        // ====================================================================
        // Apply process-wide bootstrap settings before PhysX plugins load.
        // Explicit CPU-only mode must reach the foundation before any CUDA
        // probe. Device ordinal selection is different: PhysX reads
        // /physics/cudaDevice lazily when the first GPU scene attaches, so the
        // public active_cuda_gpus path writes it immediately before attachment.
        // /physics/suppressReadback is host opt-in and never written here.
        // ====================================================================
        if (::isProcessGpuDisabled())
        {
            settings->setBool(omni::physx::kSettingForceCpuMode, true);
            CARB_LOG_INFO("[CarboniteLoader] Startup /physics/forceCpuMode=true");
        }

        // The public active_cuda_gpus path deliberately leaves this setting
        // untouched during bootstrap and applies it immediately before scene
        // attachment. Process-wide CPU-only mode is controlled separately via
        // ovphysx_set_cpu_mode().
        CARB_LOG_INFO("[CarboniteLoader] Startup: CUDA ordinal selection deferred to scene attachment");

        // Log the host-set suppressReadback value at INFO so a host that expects
        // DirectGPU but did not set it can see the misconfiguration.
        {
            const bool alreadySet = (settings->getItemType("/physics/suppressReadback") != carb::dictionary::ItemType::eCount);
            if (alreadySet)
            {
                const bool currentValue = settings->getAsBool("/physics/suppressReadback");
                CARB_LOG_INFO("[CarboniteLoader] /physics/suppressReadback=%s (host-set)", currentValue ? "true" : "false");
            }
            else
            {
                CARB_LOG_INFO("[CarboniteLoader] /physics/suppressReadback unset — DirectGPU disabled (default).");
            }
        }
    }

    // ========================================================================
    // Explicit CPU-only mode is already surfaced through /physics/forceCpuMode
    // before omni.physx.plugin starts. That lets PhysXFoundation avoid CUDA
    // probes during plugin startup, before IPhysxFoundation can be acquired.

    // ========================================================================
    // Load monitoring/system plugins before infrastructure plugins.
    // ========================================================================
    static const char* kMonitoringPlugins[] = {
        "omni.platforminfo.plugin",
    };
    {
        carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
        desc.loadedFileWildcards = kMonitoringPlugins;
        desc.loadedFileWildcardCount = sizeof(kMonitoringPlugins) / sizeof(kMonitoringPlugins[0]);
        desc.searchPaths = searchPaths.data();
        desc.searchPathCount = static_cast<uint32_t>(searchPathCount);
        framework->loadPlugins(desc);
    }
    
    // ========================================================================
    // Infrastructure plugins.
    // ========================================================================
    {
        // Load global TBB control before PhysX plugins so the process-wide
        // worker cap is established before runtime worker pools start.
        static const char* kInfrastructurePlugins[] = {
            "omni.tbb.globalcontrol.plugin",
        };

        carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
        desc.loadedFileWildcards = kInfrastructurePlugins;
        desc.loadedFileWildcardCount = sizeof(kInfrastructurePlugins) / sizeof(kInfrastructurePlugins[0]);
        desc.searchPaths = searchPaths.data();
        desc.searchPathCount = static_cast<uint32_t>(searchPathCount);
        framework->loadPlugins(desc);
        CARB_LOG_INFO("[CarboniteLoader] Infrastructure plugins loaded");
    }

    g_bootstrapDone = true;
    CARB_LOG_INFO("[CarboniteLoader] Initialization complete (base plugins)");
    return true;
}

// Build the plugin search path list for Carbonite loadPlugins().
// The string vector owns the memory. The pointer vector is only valid while the
// returned string vector stays alive in the caller.
std::pair<std::vector<std::string>, std::vector<const char*>> CarboniteLoader::buildSearchPaths() const
{
    std::vector<std::string> strs;
    strs.reserve(2);
    strs.push_back(m->pluginsDir);

    const std::string binDepsDir = m->pluginsDir + "/bin/deps";
    if (std::filesystem::exists(binDepsDir))
        strs.push_back(binDepsDir);

    std::vector<const char*> ptrs;
    ptrs.reserve(strs.size());
    for (auto& s : strs)
        ptrs.push_back(s.c_str());
    return {std::move(strs), std::move(ptrs)};
}

namespace
{

// Wire UJITSO for local, in-process cooked-collider caching with no Hub/Nucleus/GRPC. Must run
// before carb.ujitso.default loads (it builds its datastore from /UJITSO/datastore/* at plugin
// startup) and before omni.physx.cooking loads (its service reads ujitsoCollisionCooking at
// construction).
//
// The cache directory is app-provided through PhysXConfig(cooked_collider_cache_dir=...) (C:
// OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY), which lands in /UJITSO/datastore/localCachePath
// before this runs. ovphysx is a library, so it neither reads the environment for a location nor
// persists to a path of its own choosing. Some writable path is still needed, because
// carb.ujitso.default otherwise defaults the datastore to <carb app dir>/cache/DerivedDataCache,
// and carb's app dir is the resolved interpreter dir, which is the non-writable /usr/bin for a
// Linux venv. carb.datastore then logs two [Error] lines on every scene attach (NVBugs 6504275).
// The fallback is a process-private temp dir, discarded on shutdown.

// The process-private cache dir, empty when unused. The datastore and this directory are
// process-wide and outlive any single instance, so cleanup is refcounted and happens when the
// last loader shuts down, never while another instance may still be cooking against it. That
// last-loader decision (the g_activeLoaders check in CarboniteLoader::shutdown()) and the cleanup
// both run under g_bootstrapMutex, the same mutex initialize() holds across its g_activeLoaders
// increment, so a concurrent ovphysx_create_instance() waits instead of becoming the new first
// loader while the tree is still being removed. atexit is a best-effort backstop for callers
// that never shut down. Abrupt termination may leave the directory behind.
std::filesystem::path g_processCacheDir;

std::string uniqueToken()
{
    std::random_device rd;
    return std::to_string(rd()) + "-" + std::to_string(rd());
}

// Best effort with a short bounded retry. carb.ujitso.default/carb.datastore own the on-disk
// write-back of a completed cook and there is no API to wait for it: ovphysx_wait_op() and
// PhysX::wait_all() only drain the cook compute queue (the PhysxCookingComputeResult callback),
// which can fire before the bytes are persisted. A cook that completes just before process exit
// can therefore race this remove_all(). Retrying narrows that window for short write-backs but
// does not close it, and on Windows the datastore may also hold files open. `retry=false` is
// reserved for the late native atexit backstop, where retry sleeps would delay process
// termination. The 5x20ms budget is an arbitrary starting point, not a measured write-back
// duration.
//
// g_processCacheDir is deliberately not cleared on success: atexit fires after shutdown() and
// may catch files the datastore writes between the shutdown remove_all and process exit.
// remove_all on a missing path returns success, so the atexit call is always safe, and a failed
// attempt leaves the path in place for a later retry.
//
// The caller must hold g_bootstrapMutex.
void releaseProcessCacheDirLocked(bool retry)
{
    if (g_processCacheDir.empty())
        return;
    const int kMaxAttempts = retry ? 5 : 1;
    constexpr auto kRetryDelay = std::chrono::milliseconds(20);
    std::error_code ec;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        std::filesystem::remove_all(g_processCacheDir, ec);
        if (!ec)
            return;
        // A permission failure is not a transient write-back race and will not clear by
        // waiting, so stop retrying.
        if (ec == std::errc::permission_denied)
            break;
        if (attempt + 1 < kMaxAttempts)
            std::this_thread::sleep_for(kRetryDelay);
    }
    CARB_LOG_WARN("[CarboniteLoader] Failed to remove process-private cache '%s': %s; "
                  "it remains for the host's temp-directory cleanup.",
                  g_processCacheDir.string().c_str(), ec.message().c_str());
}

// atexit backstop for callers that never call CarboniteLoader::shutdown(). Python destroys its
// live instances from an earlier atexit callback and normally reaches the retrying shutdown path
// first. Non-Python callers may still have a live loader here, so this last single attempt remains
// best-effort and avoids adding retry latency during late process teardown.
void releaseProcessCacheDirAtExit()
{
    std::lock_guard<std::mutex> guard(g_bootstrapMutex);
    releaseProcessCacheDirLocked(/*retry=*/false);
}

// Create a file in `dir` to prove this process can write there, then remove it. Exclusive
// create with a process-unique name: it never follows or truncates a pre-existing entry, and
// only ever removes the entry it just created (cpp:S5443).
bool canWriteInto(const std::filesystem::path& dir)
{
    const std::filesystem::path probe = dir / (".ovphysx-probe-" + uniqueToken());
#ifdef _WIN32
    HANDLE h = ::CreateFileW(probe.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                             FILE_ATTRIBUTE_TEMPORARY | FILE_FLAG_DELETE_ON_CLOSE, nullptr);
    if (h == INVALID_HANDLE_VALUE)
        return false;
    ::CloseHandle(h); // FILE_FLAG_DELETE_ON_CLOSE removes it
    return true;
#else
    const int fd = ::open(probe.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, S_IRUSR | S_IWUSR);
    if (fd < 0)
        return false;
    ::close(fd);
    std::error_code ec;
    std::filesystem::remove(probe, ec);
    return true;
#endif
}

// Fresh, process-private cache dir under the OS temp dir. The name is unpredictable and the
// directory is never adopted from an existing entry, so another local user cannot pre-create or
// hijack it in a shared temp root.
std::filesystem::path createProcessCacheDir()
{
    std::error_code ec;
    const std::filesystem::path tempRoot = std::filesystem::temp_directory_path(ec);
    // A relative TMPDIR would put the cache under whatever cwd the process was launched from.
    if (ec || !tempRoot.is_absolute())
        return {};

#ifndef _WIN32
    // mkdtemp gives an unpredictable name and 0700 in one step, independent of the umask.
    std::string pattern = (tempRoot / "ovphysx-cache-XXXXXX").string();
    if (::mkdtemp(pattern.data()) == nullptr)
        return {};
    return std::filesystem::path(pattern);
#else
    // No mkdtemp on Windows, but the per-user temp dir is already ACL-restricted to its owner.
    // create_directory fails rather than following a pre-existing entry.
    for (unsigned attempt = 0; attempt < 64; ++attempt)
    {
        std::filesystem::path candidate = tempRoot / ("ovphysx-cache-" + uniqueToken());
        if (std::filesystem::create_directory(candidate, ec))
            return candidate;
    }
    return {};
#endif
}

void configureUjitsoLocalCache(carb::settings::ISettings* settings)
{
    if (!settings)
        return;

    // Local-only datastore: local on, Hub/Nucleus/GRPC off. setDefault lets an explicit app
    // override (via PhysXConfig.carbonite_overrides) still opt in, while flipping carb's GRPC
    // default from true to false. ovphysx is kitless and in-process.
    settings->setDefaultBool("/UJITSO/datastore/allowLocalDataStore", true);
    settings->setDefaultBool("/UJITSO/datastore/allowHubDataStore", false);
    settings->setDefaultBool("/UJITSO/datastore/allowNucleusDataStore", false);
    settings->setDefaultBool("/UJITSO/datastore/allowGRPCDataStore", false);

    // Pre-seed collision cooking on so the cooking service reads `true` at construction.
    // omni.physx.cooking loads before omni.physx seeds this default.
    settings->setDefaultBool(omni::physx::kSettingUjitsoCollisionCooking, true);

    // Nothing to cache to: the app opted out of the local datastore, or out of cooking.
    if (!settings->getAsBool("/UJITSO/datastore/allowLocalDataStore") ||
        !settings->getAsBool(omni::physx::kSettingUjitsoCollisionCooking))
        return;

    // The app configured a dir: create it and confirm it is writable. If it is not, fall through
    // to the process-private cache. Leaving the unusable path in place would make carb.datastore
    // retry it and log the very errors this avoids. Warn, never fail.
    const char* configured = settings->getStringBuffer("/UJITSO/datastore/localCachePath");
    if (configured && *configured)
    {
        // u8path: the app passes UTF-8 (Python str), which is not the Windows narrow encoding.
        const std::filesystem::path dir = std::filesystem::u8path(configured);
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (std::filesystem::is_directory(dir, ec) && canWriteInto(dir))
            return;
        CARB_LOG_WARN("[CarboniteLoader] Configured UJITSO cooked-collider cache dir '%s' is not "
                      "writable; cooking will not persist across runs.", configured);
    }

    g_processCacheDir = createProcessCacheDir();
    if (g_processCacheDir.empty())
    {
        // Nowhere writable at all. Suppress the local datastore rather than let
        // carb.ujitso.default fall back to its unwritable app-dir default.
        settings->setBool("/UJITSO/datastore/allowLocalDataStore", false);
        CARB_LOG_WARN("[CarboniteLoader] No writable cooked-collider cache directory available; "
                      "collision cooking runs uncached.");
        return;
    }

    // atexit as well as CarboniteLoader::shutdown(): shutdown is not guaranteed to run (callers
    // may leave the instance alive and let the OS reclaim at process exit).
    static std::once_flag cleanupOnce;
    std::call_once(cleanupOnce, [] { std::atexit(&releaseProcessCacheDirAtExit); });

    // u8string: Carbonite setting strings are UTF-8, while path::string() would convert through
    // the Windows narrow code page and mangle a non-ASCII temp path.
    const std::string dir = g_processCacheDir.u8string();
    settings->setString("/UJITSO/datastore/localCachePath", dir.c_str());
    CARB_LOG_INFO("[CarboniteLoader] No cooked-collider cache directory configured "
                  "(PhysXConfig.cooked_collider_cache_dir); using the process-private cache '%s'. "
                  "Cooked colliders are discarded at shutdown.", dir.c_str());
}

} // namespace

// Load the Carbonite dependencies, then start the linked PhysX runtime.
// IPhysxSimulation is provided directly by the runtime, not acquired from Carbonite.
bool CarboniteLoader::loadPhysxPlugins()
{
    if (m)
        m->lastError.clear();

    if (!m || !m->frameworkAcquired || m->pluginsDir.empty())
    {
        CARB_LOG_ERROR("[CarboniteLoader] Cannot load PhysX plugins (framework or plugins dir missing)");
        return false;
    }

    auto* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("[CarboniteLoader] Carbonite framework unavailable");
        return false;
    }

    // The linked PhysX runtime is process-wide. Repeated ovphysx instances use
    // its existing function table without repeating dependency setup.
    if (!m->physxSim)
        m->physxSim = omni::physx::runtime::tryGetPhysxSimulationInterface();
    if (m->physxSim)
    {
        CARB_LOG_INFO("[CarboniteLoader] PhysX runtime already started -- reusing");
        return true;
    }

    auto [searchPathStrings, searchPaths] = buildSearchPaths();

    // UJITSO local in-process cooked-collider cache (NVBugs 6262606). Without the UJITSO
    // plugins the cooking service finds no carb::ujitso::IRegistry and silently cooks
    // uncached, re-cooking every collider on every launch. Seed the local-only datastore
    // and collision-cooking settings, then load the UJITSO plugins before
    // omni.physx.cooking constructs its service (see configureUjitsoLocalCache).
    if (auto* ujitsoSettings = framework->tryAcquireInterface<carb::settings::ISettings>())
        configureUjitsoLocalCache(ujitsoSettings);

    static const char* kUjitsoPlugins[] = {
        "omni.blobkey.plugin",        // IBlobKey (no deps)
        "carb.datastore.plugin",      // datastore factories (needs IBlobKey)
        "carb.ujitsoagent.plugin",    // IFactory/IAgent/IService/IRegistry (carb.datasource-file is static)
        "carb.ujitso.default.plugin", // builds the default agent + datastore from /UJITSO/datastore/*
    };
    CARB_LOG_INFO("[CarboniteLoader] Loading UJITSO plugins (local in-process cooked-collider cache)");
    carb::PluginLoadingDesc ujitsoDesc = carb::PluginLoadingDesc::getDefault();
    ujitsoDesc.loadedFileWildcards = kUjitsoPlugins;
    ujitsoDesc.loadedFileWildcardCount = sizeof(kUjitsoPlugins) / sizeof(kUjitsoPlugins[0]);
    ujitsoDesc.searchPaths = searchPaths.data();
    ujitsoDesc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
    framework->loadPlugins(ujitsoDesc);

    // UJITSO is mandatory when collision cooking is enabled. If the registry
    // (carb.ujitsoagent) or the default agent/datastore (carb.ujitso.default) did not
    // load, fail fast rather than silently cook uncached. (A non-writable cache dir is
    // a separate, non-fatal WARN in configureUjitsoLocalCache.)
    {
        auto* ujitsoSettings = framework->tryAcquireInterface<carb::settings::ISettings>();
        const bool cookingEnabled =
            !ujitsoSettings || ujitsoSettings->getAsBool(omni::physx::kSettingUjitsoCollisionCooking);
        const auto pluginLoaded = [framework](const char* name) {
            const auto& d = framework->getPluginDesc(name);
            return d.impl.name && std::strcmp(d.impl.name, name) == 0;
        };
        // NB: getPluginDesc matches the registered impl name, which differs from the
        // file name for the default plugin: the file is libcarb.ujitso.default.plugin.so
        // but its PLUGIN_NAME is "carb.ujitsodefault.plugin" (no dot). The load
        // wildcards above are file-based, so they still match the .so on disk.
        const bool registryUp = pluginLoaded("carb.ujitsoagent.plugin");
        const bool defaultUp = pluginLoaded("carb.ujitsodefault.plugin");
        if (cookingEnabled && !(registryUp && defaultUp))
        {
            m->lastError = std::string("UJITSO collision cooking is enabled but its plugins failed to load (") +
                           (registryUp ? "" : "carb.ujitsoagent ") + (defaultUp ? "" : "carb.ujitso.default ") +
                           "missing) from " + m->pluginsDir +
                           ". Verify the ovphysx wheel/SDK ships carb.ujitsoagent, carb.ujitso.default, "
                           "carb.datastore and omni.blobkey, or disable UJITSO via "
                           "/physics/cooking/ujitsoCollisionCooking=false.";
            CARB_LOG_ERROR("[CarboniteLoader] %s", m->lastError.c_str());
            return false;
        }
        if (registryUp && defaultUp)
            CARB_LOG_INFO("[CarboniteLoader] UJITSO cooked-collider cache active (local, in-process)");
    }

    static const char* kRuntimeDependentPlugins[] = {
        "omni.localcache.plugin",
        "omni.kvdb.plugin",
    };

    size_t beforeCount = framework->getPluginCount();
    // OvruntimePhysX is statically linked into ovphysx, so ovphysx owns the
    // runtime startup explicitly after Carbonite dependencies are available.
    omni::physx::runtime::startup();

    CARB_LOG_INFO("[CarboniteLoader] Loading runtime-dependent Carbonite plugins (before: %zu plugins)", beforeCount);
    carb::PluginLoadingDesc dependentDesc = carb::PluginLoadingDesc::getDefault();
    dependentDesc.loadedFileWildcards = kRuntimeDependentPlugins;
    dependentDesc.loadedFileWildcardCount = sizeof(kRuntimeDependentPlugins) / sizeof(kRuntimeDependentPlugins[0]);
    dependentDesc.searchPaths = searchPaths.data();
    dependentDesc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
    framework->loadPlugins(dependentDesc);
    size_t afterCount = framework->getPluginCount();
    CARB_LOG_INFO("[CarboniteLoader] Runtime dependencies loaded (after: %zu plugins, delta: %zu)", afterCount, afterCount - beforeCount);

    if (omni::physx::runtime::tryGetTensorApiInterface())
    {
        CARB_LOG_INFO("[CarboniteLoader] tensor TensorApi available (PhysX backend folded into static PhysX runtime)");
    }
    else
    {
        CARB_LOG_WARN("[CarboniteLoader] tensor TensorApi unavailable - TensorBinding API may not work");
    }

    m->physxSim = omni::physx::runtime::tryGetPhysxSimulationInterface();
    CARB_LOG_INFO("[CarboniteLoader] IPhysxSimulation runtime available: %s", (m->physxSim ? "YES" : "NO"));
    if (!m->physxSim)
    {
        m->lastError = "PhysX runtime could not start IPhysxSimulation from " + m->pluginsDir +
                       ". Check the Carbonite log above for dependency loading or runtime-startup errors. "
                       "Verify that the ovphysx SDK or wheel installation contains matching native "
                       "libraries and plugins for this platform.";
        CARB_LOG_ERROR("================================================================================");
        CARB_LOG_ERROR("ERROR: PhysX runtime could not start IPhysxSimulation!");
        CARB_LOG_ERROR("Plugin search path: %s", m->pluginsDir.c_str());
        CARB_LOG_ERROR("Check the Carbonite log above for dependency loading or runtime-startup errors.");
        CARB_LOG_ERROR("Verify that the ovphysx SDK or wheel installation contains matching native "
                       "libraries and plugins for this platform.");
        CARB_LOG_ERROR("================================================================================");
        return false;
    }

    return true;
}

void CarboniteLoader::shutdown()
{
    if (!m || !m->frameworkAcquired)
        return;

    CARB_LOG_INFO("[CarboniteLoader] Shutdown starting (framework=%p)", static_cast<void*>(carb::getFramework()));
    m->physxSim = nullptr;
    m->frameworkAcquired = false;
    // The process-private cooked-collider cache is process-wide, so only the last loader may
    // remove it. An earlier one would pull it out from under an instance still cooking. The
    // decrement and the cleanup share g_bootstrapMutex with initialize()'s g_activeLoaders
    // increment, so a concurrent ovphysx_create_instance() cannot start reusing the cache dir
    // while it is still being removed.
    {
        std::lock_guard<std::mutex> guard(g_bootstrapMutex);
        if (g_activeLoaders.fetch_sub(1, std::memory_order_acq_rel) == 1)
            releaseProcessCacheDirLocked(/*retry=*/true);
    }
    CARB_LOG_INFO("[CarboniteLoader] Shutdown complete");
}

omni::physx::IPhysxSimulation* CarboniteLoader::getPhysxSimulation() const
{
    return m ? m->physxSim : nullptr;
}

} // namespace ovphysx
