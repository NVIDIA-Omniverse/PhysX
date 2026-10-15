// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-7
 *
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-HOST-EXIT-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PACKAGING-USDFREE-001
 * @covers AC-6
 *
 * @implements REQ-PACKAGING-OMNICLIENT-001
 * @covers AC-2
 *
 * @implements REQ-PACKAGING-CLOSURE-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-THREADING-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-PROFILER-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-9
 */

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "internal/GlobalState.hpp"
#include "ovphysx/ovphysx_types.h"
#include "LogManager.hpp"
#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include <omni/physx/PhysXRuntime.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <random>
#include <string>
#include <climits>
#include <filesystem>
#include <atomic>
#include <system_error>
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
#include <carb/profiler/Profile.h>
#include <carb/settings/ISettings.h>

#include "internal/Profiling.h"
#include <carb/tasking/ITasking.h>
#include <carb/thread/Util.h>
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

// Set by the build to whether carb.profiler-nvtx.static was found (see CMakeLists.txt).
// A carb package without it builds without the sink rather than failing to configure.
#ifndef OVPHYSX_CARB_NVTX_BACKEND
    #define OVPHYSX_CARB_NVTX_BACKEND 0
#endif

#if OVPHYSX_CARB_NVTX_BACKEND
// Carbonite's own NVTX profiler backend. Deliberately not in the list above: those
// plugins are required, this one is registered only when NVTX profiling is asked for.
extern "C" bool carb_profiler_nvtx_plugin_registerPlugin(carb::Framework*);
#endif

CARB_STATIC_BINARY_GLOBALS("omni_physx_sdk")

bool isProcessGpuDisabled();

namespace ovphysx
{

namespace
{
#if CARB_PLATFORM_WINDOWS
    void releaseProcessCacheAfterFrameworkShutdown(void*, void*);
#endif
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

    static const PluginProbe kBootstrapProbes[] = {
        OVPHYSX_PROBE("carb.dictionary.plugin", carb::dictionary::IDictionary),
        OVPHYSX_PROBE("carb.settings.plugin",   carb::settings::ISettings),
    };

    static const PluginProbe kTaskingProbes[] = {
        OVPHYSX_PROBE("carb.tasking.plugin", carb::tasking::ITasking),
    };

    static const PluginProbe kPostTaskingProbes[] = {
        OVPHYSX_PROBE("carb.tokens.plugin",     carb::tokens::ITokens),
        OVPHYSX_PROBE("carb.filesystem.plugin", carb::filesystem::IFileSystem),
    };

#undef OVPHYSX_PROBE

    static constexpr char kTaskingThreadCountPath[] = "/plugins/carb.tasking.plugin/threadCount";
    static constexpr char kTbbThreadCountPath[] = "/plugins/omni.tbb.globalcontrol/maxThreadCount";

    // omni.tbb.globalcontrol exposes a marker interface whose acquisition runs
    // plugin startup and constructs the process-wide tbb::global_control.
    struct TbbGlobalControl
    {
        CARB_PLUGIN_INTERFACE("omni::tbb::IGlobalControl", 1, 0)
    };

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
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(loaderState.bootstrapMutex);
    if (m)
        m->lastError.clear();
    
    if (loaderState.bootstrapDone)
    {
        // Re-apply log level and register pending callbacks (level may change between instances)
        ovphysx::onCarboniteLoggingReady();

        // Re-populate the per-instance plugins path for subsequent instances.
        m->frameworkAcquired = true;
        loaderState.activeLoaders.fetch_add(1, std::memory_order_relaxed);
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
    loaderState.activeLoaders.fetch_add(1, std::memory_order_relaxed);

    // The sidecar DSO survives Linux framework release. Rebind its borrowed
    // globals and source registration once per bootstrap, before policy replay.
    if (!refreshInternalSidecarRuntime())
        return false;

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
                loaderState.ownsFramework = false;
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

                // The foreign host owns this framework's teardown.
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
                                kBootstrapProbes,
                                sizeof(kBootstrapProbes) / sizeof(kBootstrapProbes[0]),
                                "Bootstrap"))
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

#if CARB_PLATFORM_WINDOWS
    // Windows retains Carbonite's ExitProcess hook: OVStage/Fabric still depend
    // on it for safe DLL teardown. A framework release hook runs after dependency
    // plugins have stopped, so cache removal cannot race datastore write-back.
    if (!framework->addReleaseHook(nullptr, &releaseProcessCacheAfterFrameworkShutdown, nullptr))
    {
        CARB_LOG_ERROR("[CarboniteLoader] Failed to register process-private cache cleanup");
        return false;
    }
#endif
    loaderState.bootstrapDone = true;
    CARB_LOG_INFO(
        "[CarboniteLoader] Initialization complete (base plugins; tasking deferred until create config is applied)");
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

std::string uniqueToken()
{
    std::random_device rd;
    return std::to_string(rd()) + "-" + std::to_string(rd());
}

// The caller holds bootstrapMutex and has stopped the datastore. Waiting for a cook alone
// is insufficient: the datastore owns background write-back and open cache files.
void releaseProcessCacheDirLocked()
{
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    if (loaderState.processCacheDir.empty())
        return;
    std::error_code ec;
    std::filesystem::remove_all(loaderState.processCacheDir, ec);
    if (!ec)
    {
        loaderState.processCacheDir.clear();
        return;
    }
    // Carbonite has already stopped; report filesystem failures through the host's stderr.
    std::fprintf(stderr, "[Warning] [CarboniteLoader] Failed to remove process-private cache '%s': %s; "
                  "it remains for the host's temp-directory cleanup.\n",
                  loaderState.processCacheDir.string().c_str(), ec.message().c_str());
}

#if CARB_PLATFORM_WINDOWS
void releaseProcessCacheAfterFrameworkShutdown(void*, void*)
{
    auto& loaderState = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(loaderState.bootstrapMutex);
    releaseProcessCacheDirLocked();
}
#endif

// Explicit shutdown runs this before host runtime finalization. The native exit registration
// also covers C hosts that destroy their instances but omit ovphysx_shutdown().
void releaseProcessCacheDirAtExit()
{
#if CARB_PLATFORM_WINDOWS
    // Windows dependency shutdown belongs to Carbonite's ExitProcess hook.
    // Releasing the framework here removes that hook and exposes OVStage/Fabric
    // static-destructor crashes. The framework release hook removes the cache.
    return;
#else
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(loaderState.bootstrapMutex);

    // Active native handles still own runtime objects. Explicit SDK shutdown leaves these
    // handles for the caller to destroy; do not invalidate them from an exit backstop.
    if (loaderState.activeLoaders.load(std::memory_order_acquire) != 0)
        return;
    // An external host may still use these plugins and their datastore.
    if (!loaderState.ownsFramework)
        return;
    if (!loaderState.bootstrapDone)
    {
        releaseProcessCacheDirLocked();
        return;
    }
    if (carb::getFramework())
    {
        omni::physx::runtime::shutdown();
        ovphysx::onCarboniteLoggingStopped();
        carb::releaseFrameworkAndShutdown();
    }
    loaderState.bootstrapDone = false;
    releaseProcessCacheDirLocked();
    loaderState.nvtxPluginRegistered = false;
#endif
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

    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    loaderState.processCacheDir = createProcessCacheDir();
    if (loaderState.processCacheDir.empty())
    {
        // Nowhere writable at all. Suppress the local datastore rather than let
        // carb.ujitso.default fall back to its unwritable app-dir default.
        settings->setBool("/UJITSO/datastore/allowLocalDataStore", false);
        CARB_LOG_WARN("[CarboniteLoader] No writable cooked-collider cache directory available; "
                      "collision cooking runs uncached.");
        return;
    }

    // u8string: Carbonite setting strings are UTF-8, while path::string() would convert through
    // the Windows narrow code page and mangle a non-ASCII temp path.
    const std::string dir = loaderState.processCacheDir.u8string();
    settings->setString("/UJITSO/datastore/localCachePath", dir.c_str());
    CARB_LOG_INFO("[CarboniteLoader] No cooked-collider cache directory configured "
                  "(PhysXConfig.cooked_collider_cache_dir); using the process-private cache '%s'. "
                  "Cooked colliders are discarded at shutdown.", dir.c_str());
}

// ---------------------------------------------------------------------------
// The profiler backend.
//
// Three emitters feed carb::profiler: the omni.physx runtime's CARB_PROFILE_* zones
// over stage load, cooking and step orchestration; the ovphysx C API entry points;
// and the PhysX SDK's non-detached zones, which the runtime bridges across. In a
// kitless process none of them reach anybody -- no backend is ever loaded, so
// g_carbProfiler stays null and every zone is a branch. Binding one backend here
// publishes all three at once.
//
// Two are selectable. NVTX feeds an Nsight Systems capture and is linked statically.
// The chrome-trace writer is carb.profiler-cpu, a shared plugin already in the
// shipped plugin set, which writes its JSON when the profiler shuts down.
//
// One group of zones never arrives here at all: the PhysX SDK's detached
// (cross-thread) zones, which IProfiler cannot express and the runtime therefore
// drops. No capture has Basic.simulate or its sibling phases, whichever backend is
// bound.
//
// Shared by live loaders: bound at most once per loader epoch, and released by
// the last loader, which also writes the trace file. The datastore and its cache
// instead remain until framework shutdown.
// ---------------------------------------------------------------------------
constexpr const char* kCarbNvtxPlugin = "carb.profiler-nvtx.plugin";
constexpr const char* kCarbTracePlugin = "carb.profiler-cpu.plugin";

// carb.profiler-cpu's own settings, read by the plugin when the profiler starts.
constexpr const char* kTraceSaveProfile = "/plugins/carb.profiler-cpu.plugin/saveProfile";
constexpr const char* kTraceCompressProfile = "/plugins/carb.profiler-cpu.plugin/compressProfile";
constexpr const char* kTraceFilePath = "/plugins/carb.profiler-cpu.plugin/filePath";

const char* profilerPluginName(int32_t backend)
{
    return backend == OVPHYSX_PROFILER_BACKEND_TRACE ? kCarbTracePlugin : kCarbNvtxPlugin;
}

// Messages emitted after profilerMutex is released. A log handler may call
// back into ovphysx_get_global_config, which takes that same mutex.
struct DeferredCarbLog
{
    enum class Level
    {
        eInfo,
        eWarn,
    };

    struct Item
    {
        Level level;
        std::string text;
    };

    std::vector<Item> items;

    void info(const std::string& text) { items.push_back({Level::eInfo, text}); }
    void warn(const std::string& text) { items.push_back({Level::eWarn, text}); }

    void emit() const
    {
        for (const Item& item : items)
        {
            if (item.level == Level::eWarn)
                CARB_LOG_WARN("%s", item.text.c_str());
            else
                CARB_LOG_INFO("%s", item.text.c_str());
        }
    }
};

std::string formatCarbLog(const char* text)
{
    return std::string(text);
}

template <typename... Args>
std::string formatCarbLog(const char* fmt, Args... args)
{
    char buffer[768];
    // The format string is a literal at every call site. It is a parameter
    // here only so the formatted text can be stored until the profiler mutex
    // is released.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
    std::snprintf(buffer, sizeof(buffer), fmt, args...);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
    return std::string(buffer);
}

// Every registered plugin that offers carb::profiler::IProfiler, by name. Logged when
// the binding did not come out as intended, since the name a static plugin registers
// under is the thing most likely to be wrong and is otherwise invisible.
void logProfilerProviders(carb::Framework* framework, DeferredCarbLog& logs)
{
    const size_t count = framework->getPluginCount();
    if (count == 0)
        return;

    std::vector<carb::PluginDesc> plugins(count);
    framework->getPlugins(plugins.data());

    const carb::InterfaceDesc wanted = carb::profiler::IProfiler::getInterfaceDesc();
    for (const carb::PluginDesc& plugin : plugins)
    {
        for (size_t i = 0; i < plugin.interfaceCount; ++i)
        {
            if (plugin.interfaces[i].name && std::strcmp(plugin.interfaces[i].name, wanted.name) == 0)
            {
                logs.warn(formatCarbLog(
                    "[CarboniteLoader]   carb::profiler::IProfiler provider: %s (provides v%u.%u, "
                    "this build requests v%u.%u)",
                    plugin.impl.name ? plugin.impl.name : "(unnamed)",
                    plugin.interfaces[i].version.major, plugin.interfaces[i].version.minor,
                    wanted.version.major, wanted.version.minor));
            }
        }
    }
}

// Makes the selected backend's plugin available for acquisition. Returns false only
// when the request cannot be honoured at all; the acquire in the caller is what proves
// the plugin actually provides IProfiler.
bool registerProfilerBackendPlugin(carb::Framework* framework,
                                   const std::vector<const char*>& searchPaths,
                                   int32_t backend,
                                   DeferredCarbLog& logs)
{
    if (backend == OVPHYSX_PROFILER_BACKEND_TRACE)
    {
        // A shared plugin rather than a static one -- the carb package builds a static
        // library for the NVTX backend only. Carbonite skips a plugin whose name is
        // already loaded, so this is safe even if a host loaded it first.
        const char* wildcards[] = { kCarbTracePlugin };
        carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
        desc.loadedFileWildcards = wildcards;
        desc.loadedFileWildcardCount = 1;
        desc.searchPaths = searchPaths.data();
        desc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
        framework->loadPlugins(desc);
        return true;
    }

#if OVPHYSX_CARB_NVTX_BACKEND
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    // At most once per framework: see nvtxPluginRegistered. A later loader epoch
    // reuses the registration; Linux owned-framework shutdown resets the flag.
    if (loaderState.nvtxPluginRegistered)
    {
        return true;
    }

    if (!carb_profiler_nvtx_plugin_registerPlugin(framework))
    {
        // Not fatal: the zones stay unpublished, which is no reason to fail instance
        // creation. A profiling opt-in that quietly does nothing would be worse, hence
        // the warning.
        logs.warn(formatCarbLog(
            "[CarboniteLoader] Could not register %s; the profile zones will not "
            "appear in an NVTX capture",
            kCarbNvtxPlugin));
        return false;
    }
    loaderState.nvtxPluginRegistered = true;
    return true;
#else
    CARB_UNUSED(framework);
    CARB_UNUSED(searchPaths);
    logs.warn(formatCarbLog(
        "[CarboniteLoader] This build has no NVTX profiler backend (carb.profiler-nvtx.static "
        "was absent at configure time); select the trace backend instead"));
    return false;
#endif
}

// Points carb.profiler-cpu at its destination. Must run before CARB_PROFILE_STARTUP():
// the plugin reads these when the profiler starts.
void configureTraceBackend(carb::settings::ISettings* settings, DeferredCarbLog& logs)
{
    settings->setBool(kTraceSaveProfile, true);

    // Plain JSON, not the plugin's gzip. The point of this backend is an artifact a
    // script can read without a decompression step; a caller who wants the space back
    // can compress the file afterwards.
    settings->setBool(kTraceCompressProfile, false);

    const char* outputPath = settings->getStringBuffer(ovphysx::profiling::kSettingProfilerOutputPath);
    if (outputPath && outputPath[0] != '\0')
    {
        settings->setString(kTraceFilePath, outputPath);
        logs.info(formatCarbLog(
            "[CarboniteLoader] Chrome trace will be written to '%s'", outputPath));
    }
    else
    {
        // Drop any destination an earlier epoch set. The plugin setting is process-global
        // and outlives a loader epoch, so leaving it would send this run's trace to the
        // previous run's file -- overwriting it, while the log below promises the
        // default.
        settings->destroyItem(kTraceFilePath);

        // The plugin's own default is a timestamped ct-profile file in the working
        // directory. Named in the log because an unconfigured run otherwise leaves the
        // caller hunting for where the trace went.
        logs.info(formatCarbLog(
            "[CarboniteLoader] Chrome trace will be written to the plugin default "
            "(ct-profile <date> <time>.json in the working directory); set "
            "PhysXConfig.profiler_output_path or OVPHYSX_PROFILER_OUTPUT to choose"));
    }
}

void enableProfilerBackendIfRequested(carb::Framework* framework, const std::vector<const char*>& searchPaths)
{
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    DeferredCarbLog logs;
    std::unique_lock<std::mutex> guard(loaderState.profilerMutex);
    // Declared after the lock so it runs first on every return: unlock, then log,
    // then refresh. A handler that reads the bound profiler must not find this
    // mutex still held, and refresh itself can log.
    struct Flush
    {
        DeferredCarbLog& logs;
        std::unique_lock<std::mutex>& guard;
        bool refresh;
        ~Flush()
        {
            if (guard.owns_lock())
                guard.unlock();
            logs.emit();
            if (refresh)
                omni::physx::runtime::refreshProfilerCallback();
        }
    } flush{logs, guard, false};

    carb::settings::ISettings* settings = framework->tryAcquireInterface<carb::settings::ISettings>();
    if (!settings)
        return;

    const int32_t selected = settings->getAsInt(ovphysx::profiling::kSettingProfilerBackend);
    if (selected == OVPHYSX_PROFILER_BACKEND_NONE)
    {
        if (loaderState.profilerBackendActive)
        {
            // Asking for none while a backend is bound is the same case as asking for a
            // different one: the bound backend keeps publishing until the last loader
            // shuts down, because releasing it here would silence an instance that is
            // still simulating.
            logs.warn(formatCarbLog(
                "[CarboniteLoader] Profiler backend '%s' is already bound process-wide; this "
                "instance asked for none and the zones keep publishing to the bound one "
                "until the last instance shuts down",
                ovphysx::profiling::backendName(
                    static_cast<ovphysx_profiler_backend_t>(loaderState.profilerBackendBound))));
        }

        // Tell the runtime either way. It caches the installed SDK callback from
        // whenever it was last resolved, and the PhysX SDK is created once per direct-runtime lifecycle:
        // returning without a refresh would leave the callback as the first instance
        // left it. Flush runs the refresh after the mutex is released.
        flush.refresh = true;
        return;
    }

    const char* pluginName = profilerPluginName(selected);
    carb::profiler::IProfiler* backend = nullptr;

    if (!loaderState.profilerBackendActive)
    {
        loaderState.profilerBindFailed = false;
        if (!registerProfilerBackendPlugin(framework, searchPaths, selected, logs))
        {
            loaderState.profilerBindFailed = true;
            return;
        }

        if (selected == OVPHYSX_PROFILER_BACKEND_TRACE)
            configureTraceBackend(settings, logs);

        // Name it as this client's IProfiler, so the unqualified acquire inside
        // registerProfilerForClient() resolves to this backend and not to whichever
        // other one Carbonite finds -- the install tree ships more than one plugin
        // providing this interface. Per-client, so a host tenant's own choice is
        // untouched.
        framework->setDefaultPlugin<carb::profiler::IProfiler>(pluginName);

        // Then acquire it once through the *non-optional* entry point, which is what
        // initializes a freshly registered plugin. registerProfilerForClient() acquires
        // optionally, and an optional acquire returns null for a plugin that has not
        // been initialized rather than initializing it -- so registering and binding
        // without this call leaves the profiler unbound and every zone dropped, with
        // no error anywhere.
        backend = framework->acquireInterface<carb::profiler::IProfiler>(pluginName);
        if (!backend)
        {
            logs.warn(formatCarbLog(
                "[CarboniteLoader] %s did not provide carb::profiler::IProfiler; the "
                "profile zones cannot reach it",
                pluginName));
            logProfilerProviders(framework, logs);
            loaderState.profilerBindFailed = true;
            return;
        }

        // Rebind rather than just register. registerProfilerForClient() is a one-shot
        // per module, and this module's profiler may already be bound: its load hook
        // binds whichever backend Carbonite loaded first. Deregistering first makes the
        // re-registration resolve through the default just set, instead of keeping a
        // binding that predates it.
        carb::profiler::deregisterProfilerForClient();
        carb::profiler::registerProfilerForClient();

        // Zone masks in the runtime are mixed: most zones pass 0, which resolves to
        // kCaptureMaskDefault (bit 1), while the PhysX SDK bridge uses 1 << 8. Capture
        // everything rather than track that set. Set before startup, not after:
        // IProfiler documents changing the mask on a started profiler as undefined.
        CARB_PROFILE_SET_CAPTURE_MASK(carb::profiler::kCaptureMaskAll);
        CARB_PROFILE_STARTUP();
        loaderState.profilerBackendActive = true;
        loaderState.profilerBackendBound = selected;
        loaderState.profilerBindFailed = false;
        loaderState.profilerHostOwned = false;
        loaderState.profilerOutputPathCaptured = false;
        loaderState.profilerBoundOutputPath.clear();
        if (selected == OVPHYSX_PROFILER_BACKEND_TRACE)
        {
            const char* configured = settings->getStringBuffer(kTraceFilePath);
            if (configured && configured[0] != '\0')
            {
                loaderState.profilerBoundOutputPath = configured;
                loaderState.profilerOutputPathCaptured = true;
            }
        }

        // The runtime installs the SDK profiler callback when it creates the PhysX
        // SDK, which happens once per direct-runtime lifecycle: an
        // instance that asks for profiling after an earlier one ran without it would
        // otherwise find no callback installed, and every SDK zone would be dropped
        // before reaching this backend. Flush runs it after the mutex is released.
        flush.refresh = true;
    }
    else
    {
        // The first selection wins for the rest of the loader epoch, because the profiler
        // is bound once and started once. Say so rather than let the caller conclude the
        // setting was ignored when a capture comes back in the other format.
        if (selected != loaderState.profilerBackendBound)
        {
            logs.warn(formatCarbLog(
                "[CarboniteLoader] Profiler backend '%s' is already bound process-wide; this "
                "instance asked for '%s' and will publish to the bound one",
                ovphysx::profiling::backendName(
                    static_cast<ovphysx_profiler_backend_t>(loaderState.profilerBackendBound)),
                ovphysx::profiling::backendName(static_cast<ovphysx_profiler_backend_t>(selected))));
        }
        else if (selected == OVPHYSX_PROFILER_BACKEND_TRACE)
        {
            // The destination is configured on the plugin once, before it starts, so a
            // later instance asking for a different file does not get one -- and would
            // otherwise go looking for a trace that was never going to be written there.
            const char* requested = settings->getStringBuffer(ovphysx::profiling::kSettingProfilerOutputPath);
            const char* configured = settings->getStringBuffer(kTraceFilePath);
            const bool configuredIsDefault = !configured || configured[0] == '\0';
            if (requested && requested[0] != '\0' && (configuredIsDefault || std::strcmp(requested, configured) != 0))
            {
                logs.warn(formatCarbLog(
                    "[CarboniteLoader] The chrome trace destination is already set to %s for this "
                    "process; this instance asked for '%s', which is ignored. The trace is written "
                    "to the bound destination when the last instance shuts down.",
                    configuredIsDefault ? "the plugin default" : configured, requested));
            }
        }
        backend = framework->tryAcquireInterface<carb::profiler::IProfiler>(
            profilerPluginName(loaderState.profilerBackendBound));
    }

    // Reported on every creation that asks for profiling, not just the one that bound
    // the backend: the binding is a process-wide fact decided once, but the instance
    // asking now is the one whose capture depends on the answer. The alternative is not
    // "nothing" -- a host that bound its own backend keeps the zones, and the only
    // symptom is a capture missing rows, with nothing to explain why.
    carb::profiler::IProfiler* active = carb::profiler::getProfiler();
    if (loaderState.profilerBackendActive)
        loaderState.profilerHostOwned = active == nullptr || active != backend;
    if (active && active == backend)
    {
        logs.info(formatCarbLog(
            "[CarboniteLoader] Profile zones are publishing to the '%s' backend",
            ovphysx::profiling::backendName(
                static_cast<ovphysx_profiler_backend_t>(loaderState.profilerBackendBound))));
    }
    else
    {
        if (!active)
            logs.warn(formatCarbLog(
                "[CarboniteLoader] No Carbonite profiler is bound for this module; the "
                "profile zones reach no consumer"));
        else
            logs.warn(formatCarbLog(
                "[CarboniteLoader] Another Carbonite profiler backend owns this process; the "
                "profile zones go there rather than to the selected backend"));

        // Which plugins can provide the interface at all. Only on the unhappy path,
        // where the alternative is guessing from an empty capture.
        logProfilerProviders(framework, logs);
    }
}

void disableProfilerBackend()
{
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(loaderState.profilerMutex);
    if (!loaderState.profilerBackendActive)
    {
        loaderState.profilerBindFailed = false;
        loaderState.profilerHostOwned = false;
        loaderState.profilerOutputPathCaptured = false;
        loaderState.profilerBoundOutputPath.clear();
        return;
    }

    // Order matters for the trace backend: CARB_PROFILE_SHUTDOWN() is what makes
    // carb.profiler-cpu write its JSON, so the file appears at this point and not
    // before. Deregistering first would drop the interface pointer that writes it.
    CARB_PROFILE_SHUTDOWN();
    carb::profiler::deregisterProfilerForClient();
    loaderState.profilerBackendActive = false;
    loaderState.profilerBackendBound = OVPHYSX_PROFILER_BACKEND_NONE;
    loaderState.profilerBindFailed = false;
    loaderState.profilerHostOwned = false;
    loaderState.profilerOutputPathCaptured = false;
    loaderState.profilerBoundOutputPath.clear();
}

} // namespace

BoundProfilerState boundProfilerState()
{
    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(loaderState.profilerMutex);
    BoundProfilerState state;
    state.active = loaderState.profilerBackendActive;
    state.bindFailed = loaderState.profilerBindFailed;
    state.hostOwned = loaderState.profilerHostOwned;
    state.backend = loaderState.profilerBackendBound;
    state.outputPathCaptured = loaderState.profilerOutputPathCaptured;
    state.outputPath = loaderState.profilerBoundOutputPath;
    return state;
}

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

    // Before the reuse path below: the backend selection is resolved per instance
    // creation, so an instance that turns profiling on in a process where an earlier
    // one left it off still gets its zones published. Search paths are needed because
    // the trace backend is a shared plugin that has to be found on disk.
    {
        auto [profilerSearchPathStrings, profilerSearchPaths] = buildSearchPaths();
        enableProfilerBackendIfRequested(framework, profilerSearchPaths);
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

    // Tasking and TBB establish their process-wide worker limits when their
    // interfaces are acquired. initialize() therefore registers but deliberately
    // does not acquire either interface, and ovphysx_create_instance() applies
    // its config entries between initialize() and this call, so an
    // application-supplied worker count is in place before either pool starts.
    //
    // ovphysx supplies no default of its own: an unconfigured pool keeps
    // Carbonite's machine-sized default. A bounded default was measured on
    // GPU-heavy replicated scenes and was slower than the machine-sized pool,
    // so the count is left to the application.
    carb::settings::ISettings* workerSettings =
        framework->tryAcquireInterface<carb::settings::ISettings>();
    if (!workerSettings)
    {
        m->lastError = "Cannot configure worker pools because Carbonite ISettings is unavailable";
        CARB_LOG_ERROR("[CarboniteLoader] %s", m->lastError.c_str());
        return false;
    }

    // An unset tasking count reads as zero and means "size to the machine".
    // TBB distinguishes an absent setting from explicit zero; zero is invalid
    // when present because tbb::global_control requires a positive limit.
    const int32_t configuredTaskingThreads = workerSettings->getAsInt(kTaskingThreadCountPath);
    const int32_t configuredTbbThreads = workerSettings->getAsInt(kTbbThreadCountPath);
    const bool hasConfiguredTbbThreads =
        workerSettings->isAccessibleAs(carb::dictionary::ItemType::eInt, kTbbThreadCountPath);
    if (configuredTaskingThreads < 0 || configuredTbbThreads < 0 ||
        (hasConfiguredTbbThreads && configuredTbbThreads == 0))
    {
        m->lastError =
            "Worker thread counts are invalid: /plugins/carb.tasking.plugin/threadCount and "
            "/plugins/omni.tbb.globalcontrol/maxThreadCount must be >= 0 and an explicit TBB limit must be > 0";
        CARB_LOG_ERROR(
            "[CarboniteLoader] %s (tasking=%d, TBB=%d)",
            m->lastError.c_str(),
            configuredTaskingThreads,
            configuredTbbThreads);
        return false;
    }

    carb::tasking::ITasking* existingTasking =
        framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
    if (existingTasking)
    {
        CARB_LOG_INFO(
            "[CarboniteLoader] Reusing pre-started carb.tasking pool with %u workers; "
            "startup setting is %d and cannot resize it",
            existingTasking->getDesc().threadCount,
            configuredTaskingThreads);
    }
    else
    {
        CARB_LOG_INFO(
            "[CarboniteLoader] Starting carb.tasking with configured threadCount=%d "
            "(0 means size to the machine; available logical CPUs=%u)",
            configuredTaskingThreads,
            carb::thread::hardware_concurrency());
    }

    if (!verifyLoadedInterfaces(
            framework, kTaskingProbes, sizeof(kTaskingProbes) / sizeof(kTaskingProbes[0]), "Tasking"))
    {
        m->lastError = "Carbonite tasking failed to start after worker configuration";
        return false;
    }

    carb::tasking::ITasking* tasking =
        framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
    CARB_LOG_INFO(
        "[CarboniteLoader] Worker pools configured: carb.tasking effective=%u, TBB configured=%d",
        tasking ? tasking->getDesc().threadCount : 0,
        configuredTbbThreads);

    if (!verifyLoadedInterfaces(
            framework,
            kPostTaskingProbes,
            sizeof(kPostTaskingProbes) / sizeof(kPostTaskingProbes[0]),
            "Post-tasking foundation"))
    {
        m->lastError = "Carbonite foundation interfaces failed after tasking startup";
        return false;
    }

    // Establish TBB's process-wide cap before the PhysX runtime or cooking
    // plugins can start TBB workers.
    static const char* const kInfrastructurePlugins[] = {
        "omni.tbb.globalcontrol.plugin",
    };
    carb::PluginLoadingDesc infrastructureDesc = carb::PluginLoadingDesc::getDefault();
    infrastructureDesc.loadedFileWildcards = kInfrastructurePlugins;
    infrastructureDesc.loadedFileWildcardCount =
        sizeof(kInfrastructurePlugins) / sizeof(kInfrastructurePlugins[0]);
    infrastructureDesc.searchPaths = searchPaths.data();
    infrastructureDesc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
    framework->loadPlugins(infrastructureDesc);
    TbbGlobalControl* tbbGlobalControl =
        framework->tryAcquireInterface<TbbGlobalControl>("omni.tbb.globalcontrol.plugin");
    if (!tbbGlobalControl)
    {
        m->lastError = "TBB global control failed to start after worker configuration";
        CARB_LOG_ERROR("[CarboniteLoader] %s", m->lastError.c_str());
        return false;
    }
    CARB_LOG_INFO(
        "[CarboniteLoader] TBB global control started with configured maxThreadCount=%d",
        configuredTbbThreads);

    // Monitoring plugins may depend on tasking, so load them only after the
    // configured tasking pool is running.
    static const char* const kMonitoringPlugins[] = {
        "omni.platforminfo.plugin",
    };
    carb::PluginLoadingDesc monitoringDesc = carb::PluginLoadingDesc::getDefault();
    monitoringDesc.loadedFileWildcards = kMonitoringPlugins;
    monitoringDesc.loadedFileWildcardCount =
        sizeof(kMonitoringPlugins) / sizeof(kMonitoringPlugins[0]);
    monitoringDesc.searchPaths = searchPaths.data();
    monitoringDesc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
    framework->loadPlugins(monitoringDesc);

    // UJITSO local in-process cooked-collider cache (NVBugs 6262606). Without the UJITSO
    // plugins the cooking service finds no carb::ujitso::IRegistry and silently cooks
    // uncached, re-cooking every collider on every launch. Seed the local-only datastore
    // and collision-cooking settings, then load the UJITSO plugins before
    // omni.physx.cooking constructs its service (see configureUjitsoLocalCache).
    if (auto* ujitsoSettings = framework->tryAcquireInterface<carb::settings::ISettings>())
        configureUjitsoLocalCache(ujitsoSettings);

    static const char* const kUjitsoPlugins[] = {
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

    static const char* const kRuntimeDependentPlugins[] = {
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

    std::atexit(&releaseProcessCacheDirAtExit);
    return true;
}

void shutdownCarbonite()
{
    // Drain before taking bootstrapMutex: callbacks can query loader state. This API
    // also updates the thread-local last error, so never call it from native atexit,
    // which runs after C++ thread-local destructors on Linux.
    ovphysx::shutdownLogCallback();
    releaseProcessCacheDirAtExit();
}

void CarboniteLoader::shutdown()
{
    if (!m || !m->frameworkAcquired)
        return;

    CarboniteLoaderState& loaderState = globalState().carboniteLoader;
    CARB_LOG_INFO("[CarboniteLoader] Shutdown starting (framework=%p)", static_cast<void*>(carb::getFramework()));
    m->physxSim = nullptr;
    m->frameworkAcquired = false;
    // The datastore outlives individual loaders. Keep its cache until SDK shutdown has
    // stopped its background writers. The last-loader transition still releases profiling.
    {
        std::lock_guard<std::mutex> guard(loaderState.bootstrapMutex);
        if (loaderState.activeLoaders.fetch_sub(1, std::memory_order_acq_rel) == 1)
        {
            // Register after any lazy plugin startup during this lifecycle, so a C host
            // that omits SDK shutdown can release Carbonite before its terminating hook.
            std::atexit(&releaseProcessCacheDirAtExit);
            // Same last-loader rule, and for the same reason: the profiler is
            // process-wide, so dropping it on an arbitrary per-instance shutdown
            // would silence the zones of an instance still running -- and, for the
            // trace backend, would write the file while work was still being recorded.
            disableProfilerBackend();
        }
    }
    CARB_LOG_INFO("[CarboniteLoader] Shutdown complete");
}

omni::physx::IPhysxSimulation* CarboniteLoader::getPhysxSimulation() const
{
    return m ? m->physxSim : nullptr;
}

} // namespace ovphysx
