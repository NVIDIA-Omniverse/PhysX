// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "cuda_shim/CudaShim.h"
#include "ovphysx/ovphysx_types.h"
#include "LogManager.hpp"
#include <omni/physx/PhysXRuntime.h>

#include <cstdlib>
#include <cstring>
#include <mutex>
#include <climits>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <unordered_set>
#include <vector>

#include "internal/sdk/LibraryPathUtils.hpp"
#include "internal/sdk/NamespacedUsdLibraryUtils.hpp"
#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include "UsdVersionCheck/UsdVersionCheck.h"

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
    #define PATH_MAX MAX_PATH
#else
    #include <dlfcn.h>
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

// Static Carbonite plugins do not self-register just because their archives are
// linked into ovphysx. Each generated registerPlugin symbol must be referenced
// and called explicitly so the linker pulls in the archive object and the
// interfaces become visible to this carb::Framework instance. This mirrors
// the static-carb Physics wiring and the ovrtx static-carb setup; keep
// the explicit list until Carbonite provides an aggregate registration helper.
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
    static bool g_usdPreloadDone = false;
    // Borrowed when reusing a co-tenant's already-loaded OmniClient, otherwise
    // loaded from ovphysx's package path. Intentionally kept resident for the
    // process lifetime. ovphysx is a shared library, not the whole program, so
    // it cannot safely pick a time to call omniClientShutdown().
    static void* g_omniClientLibraryHandle = nullptr;

    // Startup settings that must be applied before PhysX plugins load.
    // These are process-wide (Carbonite settings are global).
    // -2 = CPU mode (GPU not requested by user), -1 = auto-select GPU, >=0 = specific GPU ordinal.
    // Always written by ovphysx_create_instance() via setStartupCudaDevice() before initialize().
    //
    // Note: /physics/suppressReadback (DirectGPU-API mode) is NOT managed here.
    // It is opt-in by the host: callers who want DirectGPU set the Carbonite
    // setting before any ovphysx call. ovphysx never writes this setting
    // because DirectGPU is incompatible with contact modification (used by
    // surface velocity, custom contact callbacks). See create_args doc-comment
    // in ovphysx_types.h for the full trade-off.
    static std::atomic<int32_t> g_startupCudaDevice{-2};

    // After-load interface probe: each entry pairs a plugin name (used only
    // for the error message) with a tryAcquireInterface lambda that returns
    // true iff ovphysx-side compile-time-expected version of that plugin's
    // primary interface is acquirable.
    //
    // Why this catches more than just "did the .so load":
    //   carb's tryAcquireInterface<T>() does (name, major, minor) version
    //   matching against what the loaded plugin advertises.  If a foreign
    //   host loaded a same-named plugin first at a different major (or older
    //   minor), our probe sees null at exactly the moment we'd otherwise
    //   cascade later into "Dependency: <iface> failed to be resolved" or a
    //   null pointer dereference inside ovphysx.  Same probe also fires when
    //   the .so simply didn't load (file missing, dlopen failed) -- from
    //   ovphysx's perspective those are the same problem.
    struct PluginProbe
    {
        const char* plugin;
        bool (*acquired)(carb::Framework*);
    };
    // tryAcquireInterface is non-owning for the singleton plugin interfaces
    // probed below; the returned pointer does not need release.  We discard it
    // intentionally -- this is a presence/version check, not an acquisition.
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

    // Probe each plugin's primary interface and report any null returns.
    // Returns true if all interfaces resolved, false otherwise.  On false the
    // caller should treat this as a fatal load-time error: ovphysx cannot
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

    // Force the settings ovphysx needs before PhysX plugins load.
    // These are not user preferences; they define the SDK runtime shape:
    // USD writeback off, and no renderer/NGX side systems.
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

    // Log level is managed globally by LogManager (ovphysx_set_log_level / ovphysx_get_log_level).
    // Consolidated GPU-disable check with optional reason string.
    // GPU is disabled if any of these hold:
    //   1. OVPHYSX_DISABLE_GPU env var is set
    //   2. User requested CPU mode (cudaDevice == -2)
    //   3. No CUDA driver/device detected at runtime (via the internal CUDA shim)
    //
    // We short-circuit on (1) and (2) to avoid probing the driver when CPU is explicit.
    //
    // NOTE: Device resolution for AUTO is handled via the same runtime CUDA shim
    // used by PhysX foundation. It never links against libcuda/nvcuda and lets
    // ovphysx decide whether GPU-only plugins are safe before PhysX starts.
    // NOTE on thread-safety: std::getenv is not thread-safe with concurrent
    // setenv/putenv on some platforms. These functions are only called during
    // initialization which is serialised by g_bootstrapMutex, and env vars
    // must not be mutated by other threads after initialization begins.
    static bool isGpuDisabledByStartupRequest(const char** outReason = nullptr)
    {
        if (outReason)
            *outReason = nullptr;

        if (std::getenv("OVPHYSX_DISABLE_GPU") != nullptr)
        {
            if (outReason)
                *outReason = "OVPHYSX_DISABLE_GPU env var set";
            return true;
        }

        const int32_t cudaDevice = g_startupCudaDevice.load(std::memory_order_acquire);
        if (cudaDevice == -2)
        {
            if (outReason)
                *outReason = "CPU mode requested";
            return true;
        }

        return false;
    }

    static bool isGpuDisabled(const char** outReason = nullptr)
    {
        if (outReason)
            *outReason = "unknown";

        if (isGpuDisabledByStartupRequest(outReason))
            return true;

        if (!omni::physx::cudaShim::isCudaAvailable())
        {
            if (outReason)
                *outReason = "no CUDA driver/device available";
            return true;
        }

        // GPU available -- outReason is only meaningful when returning true (disabled).
        if (outReason)
            *outReason = nullptr;
        return false;
    }
    
    bool isTruthySetting(const char* value)
    {
        if (!value || value[0] == '\0')
            return false;
        return (strcmp(value, "1") == 0) ||
               (strcmp(value, "true") == 0) ||
               (strcmp(value, "True") == 0) ||
               (strcmp(value, "TRUE") == 0);
    }

    bool shouldSkipUsdPreload()
    {
        auto* framework = carb::getFramework();
        auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (!settings)
            return false;

        const char* key = "/ovphysx/skipUsdLibPreload";
        auto itemType = settings->getItemType(key);
        if (itemType == carb::dictionary::ItemType::eBool)
        {
            return settings->getAsBool(key);
        }
        if (itemType == carb::dictionary::ItemType::eString)
        {
            return isTruthySetting(settings->getStringBuffer(key));
        }
        return false;
    }

    bool verifyOmniClientProvider(const std::string& pluginsDir)
    {
        const std::filesystem::path versionPath = std::filesystem::path(pluginsDir) / "ovstage-omniclient.version";
        std::ifstream versionFile(versionPath);
        std::string expectedVersion;
        if (!versionFile || !std::getline(versionFile, expectedVersion) || expectedVersion.empty())
        {
            CARB_LOG_ERROR(
                "[CarboniteLoader] Missing OVStage OmniClient provenance file: %s", versionPath.string().c_str());
            return false;
        }
        if (!expectedVersion.empty() && expectedVersion.back() == '\r')
            expectedVersion.pop_back();
        if (expectedVersion.empty())
        {
            CARB_LOG_ERROR(
                "[CarboniteLoader] Empty OVStage OmniClient provenance file: %s", versionPath.string().c_str());
            return false;
        }

        using GetVersionStringFn = const char* (*)();
#ifdef _WIN32
        GetVersionStringFn getVersion = reinterpret_cast<GetVersionStringFn>(
            GetProcAddress(static_cast<HMODULE>(g_omniClientLibraryHandle), "omniClientGetVersionString"));
#else
        GetVersionStringFn getVersion =
            reinterpret_cast<GetVersionStringFn>(dlsym(g_omniClientLibraryHandle, "omniClientGetVersionString"));
#endif
        const char* actualVersion = getVersion ? getVersion() : nullptr;
        if (!actualVersion || expectedVersion != actualVersion)
        {
            CARB_LOG_ERROR(
                "[CarboniteLoader] Refusing OmniClient outside the matched OVStage runtime "
                "(expected '%s', loaded '%s')",
                expectedVersion.c_str(), actualVersion ? actualVersion : "<version unavailable>");
            return false;
        }
        return true;
    }

    // The OVStage-provided resolver does not depend on libcarb, but it still
    // links against OVStage's matched OmniClient. Standalone ovphysx does not load
    // carb.omniclient.plugin, so load the packaged OmniClient library directly
    // and pin it for the process lifetime. ovrtx can use the Carbonite
    // OmniClient plugin wrapper route; ovphysx only has the packaged
    // OmniClient C library, whose shutdown API is a whole-program operation.
    bool loadStaticLinkedOmniClient(const std::string& pluginsDir)
    {
        if (g_omniClientLibraryHandle)
            return verifyOmniClientProvider(pluginsDir);

#    ifdef _WIN32
        HMODULE existing = nullptr;
        if (GetModuleHandleExA(0, "omniclient.dll", &existing) && existing)
        {
            CARB_LOG_INFO("[CarboniteLoader] Reusing already-loaded OmniClient: omniclient.dll");
            g_omniClientLibraryHandle = existing;
            return verifyOmniClientProvider(pluginsDir);
        }

        std::filesystem::path clientPath = std::filesystem::path(pluginsDir) / "omniclient.dll";
        g_omniClientLibraryHandle = LoadLibraryW(clientPath.wstring().c_str());
        if (!g_omniClientLibraryHandle)
        {
            CARB_LOG_ERROR("[CarboniteLoader] Failed to load static-linked OmniClient: %ls",
                           clientPath.wstring().c_str());
            return false;
        }
#    else
        // A co-tenant library may already have loaded OmniClient. Reuse and
        // promote that handle instead of loading a second copy from ovphysx's
        // package path. RTLD_NOLOAD bumps the refcount; we intentionally keep
        // that reference for the process lifetime.
        g_omniClientLibraryHandle = dlopen("libomniclient.so", RTLD_NOW | RTLD_NOLOAD | RTLD_GLOBAL);
        if (g_omniClientLibraryHandle)
        {
            CARB_LOG_INFO("[CarboniteLoader] Reusing already-loaded OmniClient: libomniclient.so");
            return verifyOmniClientProvider(pluginsDir);
        }

        std::filesystem::path clientPath = std::filesystem::path(pluginsDir) / "libomniclient.so";
        g_omniClientLibraryHandle = dlopen(clientPath.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
        if (!g_omniClientLibraryHandle)
        {
            CARB_LOG_ERROR("[CarboniteLoader] Failed to load static-linked OmniClient: %s", dlerror());
            return false;
        }
#    endif

        return verifyOmniClientProvider(pluginsDir);
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

void CarboniteLoader::setStartupCudaDevice(int32_t cudaDevice)
{
    g_startupCudaDevice.store(cudaDevice, std::memory_order_release);
}

struct CarboniteLoader::Impl
{
    bool frameworkAcquired = false;
    std::string pluginsDir;  // Path to _install/plugins/
    std::string usdLibDir;   // Preferred USD library directory
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
// It deliberately stops before loading USD-linked and PhysX plugins so callers
// can preload/reuse namespaced USD first and apply user config before PhysX starts.
bool CarboniteLoader::initialize()
{
    std::lock_guard<std::mutex> guard(g_bootstrapMutex);
    if (m)
        m->lastError.clear();
    
    if (g_bootstrapDone)
    {
        // Re-apply log level and register pending callbacks (level may change between instances)
        ovphysx::onCarboniteLoggingReady();

        // Re-populate per-instance paths (pluginsDir/usdLibDir) for subsequent tests.
        m->frameworkAcquired = true;
        m->pluginsDir = omni::sdk::usd_version::getPluginsDirectory();
        if (!m->pluginsDir.empty())
        {
            m->usdLibDir = m->pluginsDir;
            try
            {
                std::filesystem::path pluginsPath(m->pluginsDir);
                std::filesystem::path libsPath = pluginsPath.parent_path().parent_path() / "ovphysx.libs";
                if (std::filesystem::exists(libsPath))
                {
                    m->usdLibDir = libsPath.string();
                }
            }
            catch (const std::exception&)
            {
                // Best-effort; fall back to plugins directory.
                m->usdLibDir = m->pluginsDir;
            }
        }
        else
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
    m->pluginsDir = omni::sdk::usd_version::getPluginsDirectory();
#ifdef _WIN32
    if (m->pluginsDir.empty() || !std::filesystem::exists(m->pluginsDir))
    {
        // Fall back to the directory where ovphysx.dll is loaded from.
        std::string moduleDir = omni::sdk::usd_version::getLoadedLibraryPath("ovphysx.dll");
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
    
    // Detect wheel-style USD lib directory (ovphysx.libs) and prefer it when present
    m->usdLibDir = m->pluginsDir;
    try
    {
        std::filesystem::path pluginsPath(m->pluginsDir);
        std::filesystem::path libsPath = pluginsPath.parent_path().parent_path() / "ovphysx.libs";
        if (std::filesystem::exists(libsPath))
        {
            m->usdLibDir = libsPath.string();
            CARB_LOG_INFO("[CarboniteLoader] Using USD libs from: %s", m->usdLibDir.c_str());
        }
    }
    catch (const std::exception& e)
    {
        CARB_LOG_WARN("[CarboniteLoader] Failed to probe ovphysx.libs: %s (falling back to plugins directory)", e.what());
    }

    // Set only the namespaced USD plugin discovery env var. Do not point a
    // host's classic USD at namespaced schema plugins.
    {
        std::string error;
        bool registered = false;
        if (!omni::sdk::usd_schema_paths::registerSchemaPathsOnce(&error, &registered))
        {
            CARB_LOG_ERROR("[CarboniteLoader] %s", error.c_str());
            return false;
        }
        if (registered)
        {
            CARB_LOG_INFO("[CarboniteLoader] Registered ovphysx USD schema paths in %s",
                          omni::sdk::usd_schema_paths::kNamespacedUsdPluginPathEnvVar);
        }
    }
    
#ifdef _WIN32
    addToPath(m->pluginsDir);
    if (m->usdLibDir != m->pluginsDir)
        addToPath(m->usdLibDir);
    std::string gpuPathDir = m->pluginsDir + "/gpu";
    if (!isGpuDisabled() && std::filesystem::exists(gpuPathDir))
        addToPath(gpuPathDir);
#endif

    // Acquire Carbonite framework.
    //
    // carb::getFramework() returns the module-local pointer.  When ovphysx is
    // loaded as a regular shared library (via Python / ctypes) rather than as
    // a Carbonite plugin, this pointer is initially null even if Kit already
    // created the framework.  acquireFrameworkAndRegisterBuiltins() will find
    // the existing process-wide framework and set our local pointer.
    //
    // We still track whether the framework existed already for diagnostics.
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

    // Set up logging -- apply the global level and register any pending user callbacks.
    ovphysx::onCarboniteLoggingReady();

    std::unordered_set<std::string> preExistingPluginNames;
    if (frameworkAlreadyExisted)
    {
        // ====================================================================
        // Carbonite framework pre-exists, with no direct PhysX runtime yet.
        // Distinguish two sub-cases by looking at *where* the pre-existing
        // plugins live:
        //   (a) all plugins have null libPath or live under ovphysx's own
        //       install tree → this is either static built-ins registered by
        //       acquireFrameworkAndRegisterBuiltins(), or a re-entry within
        //       our own process. Proceed with plugin loading.
        //   (b) any plugin libPath points outside ovphysx's install tree →
        //       another library (ovrtx, etc.) has bootstrapped Carbonite
        //       and populated its Framework. Our plugin registrations would
        //       land in a torn registry (either ovphysx's own Framework
        //       instance, or the host's with "Ignoring plugin: same name
        //       already loaded" for SONAME collisions), producing the silent
        //       "Dependency: [omni::physics::schema::IUsdPhysics v1.1]
        //       failed to be resolved" cascade downstream. Refuse fast with
        //       a diagnosable error.
        //
        // Use the parent of pluginsDir (our _install/ root) as the "ours"
        // boundary so plugins loaded from _install/lib/ or _install/plugins/
        // are both recognized as ours.
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
                if (ourRootStr.empty()) return true;  // unknown install root -- can't classify, assume ours
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
                    // Static / built-in plugin -- no file on disk to attribute.
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
                // Default: proceed with coexistence.  ABI alignment
                // (ovphysx 0.4.1 + ovrtx 0.3.0+) makes this safe for the
                // supported version pair, and the verifyLoadedInterfaces
                // probes below produce a named-cause error if any expected
                // interface fails to resolve at the version ovphysx was built
                // against (covers both "plugin missing" and "plugin loaded at
                // a wrong version" against a foreign host).
                //
                // OVPHYSX_COEXIST_REFUSE=1 restores the legacy fail-fast-at-
                // load behavior for users who would rather not attempt
                // coexistence.
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

                // Co-tenant scenario: another library (e.g. ovrtx) bootstrapped
                // Carbonite and registered plugins before we were loaded. Static
                // ovphysx no longer releases the framework, so teardown remains
                // process-exit owned.
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

    // Log key paths now that logging is available (useful for debugging loading issues)
    CARB_LOG_INFO("[CarboniteLoader] Plugins directory: %s", m->pluginsDir.c_str());
    CARB_LOG_INFO("[CarboniteLoader] USD library directory: %s", m->usdLibDir.c_str());
    const char* pxrPluginPath = std::getenv("PXR_PLUGINPATH_NAME");
    if (pxrPluginPath)
        CARB_LOG_INFO("[CarboniteLoader] PXR_PLUGINPATH_NAME: %s", pxrPluginPath);
    const char* ovPluginPath = std::getenv("OV_PXR_PLUGINPATH_2511");
    if (ovPluginPath)
        CARB_LOG_INFO("[CarboniteLoader] OV_PXR_PLUGINPATH_2511: %s", ovPluginPath);
    
    // ========================================================================
    // Configure plugin search paths.
    //
    // We always search the main plugins directory.
    //
    // Note: "plugins/bin/deps" is a defensive legacy remnant from older install
    // layouts (pre-flattening of the ovphysx SDK _install tree). Current ovphysx
    // artifacts may not ship it, but if it exists we include it so older layouts
    // continue to work.
    // ========================================================================
    auto [searchPathStrings, searchPaths] = buildSearchPaths();
    const size_t searchPathCount = searchPaths.size();
    
    // Load core Carbonite plugins (preliminary - foundational) - must absolutely
    // happen before GPU/monitoring.
    // Static plugin registration is not idempotent: Carbonite requires plugin
    // names to be unique and returns failure for duplicate registrations. Reuse
    // the coexistence snapshot above for the normal pre-existing-plugin path:
    // getPluginDesc() logs a warning for every expected miss, which made a clean
    // standalone startup report one false warning per static plugin.
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
        // The process-shared framework is mutable, so revalidate snapshot hits;
        // getPluginDesc() is silent on a hit, and a co-tenant may have unregistered it.
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

    if (!loadStaticLinkedOmniClient(m->pluginsDir))
        return false;

    // Static Carbonite builds do not use the app-directory path for ovphysx
    // plugin discovery; runtime paths are derived from the loaded ovphysx module.
    CARB_LOG_INFO("[CarboniteLoader] Skipping app-directory configuration; ovphysx derives runtime paths from its module");
    
    enforceRequiredSettings(framework);
    if (auto* settings = framework->tryAcquireInterface<carb::settings::ISettings>())
    {
        
        // ====================================================================
        // CRITICAL: Apply startup CUDA settings BEFORE PhysX plugins load.
        //
        // PhysX foundation startup consults /physics/cudaDevice to choose
        // the CUDA ordinal. Provided by ovphysx_create_instance() via
        // CarboniteLoader::setStartupCudaDevice() and applied here (after
        // carb.settings.plugin is loaded) but before the static PhysX runtime
        // is started.
        //
        // /physics/suppressReadback (DirectGPU-API mode) is NOT applied here.
        // It is opt-in by the host: callers who want DirectGPU set the
        // Carbonite setting themselves before any ovphysx call. ovphysx never
        // writes this setting because DirectGPU is incompatible with contact
        // modification (used by surface velocity, custom contact callbacks).
        //
        // /physics/cudaDevice is process-wide and PhysX reads it during plugin
        // startup. A conflicting pre-set value means the process is already in
        // a different mode, so fail instead of silently using the wrong mode.
        // ====================================================================
        if (::isProcessGpuDisabled())
        {
            settings->setBool(omni::physx::kSettingForceCpuMode, true);
            CARB_LOG_INFO("[CarboniteLoader] Startup /physics/forceCpuMode=true");
        }

        const int32_t cudaDevice = g_startupCudaDevice.load(std::memory_order_acquire);
        if (cudaDevice != -2)
        {
            const bool alreadySet = (settings->getItemType("/physics/cudaDevice") != carb::dictionary::ItemType::eCount);
            if (alreadySet)
            {
                const int32_t currentValue = settings->getAsInt("/physics/cudaDevice");
                if (currentValue != cudaDevice)
                {
                    m->lastError = "/physics/cudaDevice is already set to " + std::to_string(currentValue) +
                                   ", but ovphysx requested " + std::to_string(cudaDevice) +
                                   ". This process-global setting cannot be changed after PhysX startup begins.";
                    CARB_LOG_ERROR("[CarboniteLoader] %s", m->lastError.c_str());
                    return false;
                }
                else
                {
                    CARB_LOG_INFO("[CarboniteLoader] /physics/cudaDevice=%d (already set)", cudaDevice);
                }
            }
            else
            {
                settings->setInt("/physics/cudaDevice", cudaDevice);
                CARB_LOG_INFO("[CarboniteLoader] Startup /physics/cudaDevice=%d", cudaDevice);
            }
        }
        else
        {
            // No active_cuda_gpus set: leave /physics/cudaDevice at the PhysX default.
            // Per-scene device selection (CPU vs GPU dynamics) is owned by PhysX via
            // physxScene:enableGPUDynamics in USD. Process-wide CPU-only mode is
            // controlled separately via ovphysx_set_cpu_mode().
            CARB_LOG_INFO("[CarboniteLoader] Startup: no active_cuda_gpus — /physics/cudaDevice left at default");
        }

        // Surface the host-set suppressReadback value at INFO so misconfigurations
        // (e.g. host expecting DirectGPU but didn't actually set it) are visible.
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
    // Load monitoring/system plugins before GPU plugins.
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
    // GPU and infrastructure plugins
    //
    // NOTE: GPU availability was decided through the runtime CUDA shim above,
    // before any plugin that directly depends on the CUDA driver is loaded.
    // ========================================================================
    const char* disableReason = nullptr;
    const bool disableGpu = isGpuDisabled(&disableReason);

    {
        std::vector<const char*> gpuPlugins;
        // Static SDKs disable structured logging during Omni Core startup.
        // Loading omni.structuredlog.plugin dynamically here re-enters TOML
        // config parsing before the static framework is fully initialized.
        // omni.tbb.globalcontrol must be loaded because omni.fabric.plugin
        // declares omni::tbb::IGlobalControl as a CARB_PLUGIN_IMPL_DEPS
        // dependency; without it Fabric refuses to load and downstream stage
        // creation fails.
        gpuPlugins.push_back("omni.tbb.globalcontrol.plugin");

        if (!disableGpu)
        {
            CARB_LOG_INFO("[CarboniteLoader] GPU enabled - loading GPU plugins");
            gpuPlugins.push_back("omni.gpucompute-cuda.plugin");
        }
        else
        {
            CARB_LOG_INFO("[CarboniteLoader] GPU disabled (%s)", disableReason ? disableReason : "unknown");
        }
        
        // Recompute search paths using the finalized GPU availability decision.
        // On Windows omni.gpucompute-cuda.plugin is isolated into plugins/gpu/,
        // so this path must be present before loading the CUDA compute plugin.
        // buildSearchPaths() still adds plugins/gpu/ only when GPU is enabled,
        // so the no-GPU crash-safety isolation is preserved.
        auto [gpuSearchPathStrings, gpuSearchPaths] = buildSearchPaths();

        carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
        desc.loadedFileWildcards = gpuPlugins.data();
        desc.loadedFileWildcardCount = static_cast<uint32_t>(gpuPlugins.size());
        desc.searchPaths = gpuSearchPaths.data();
        desc.searchPathCount = static_cast<uint32_t>(gpuSearchPaths.size());
        framework->loadPlugins(desc);
        CARB_LOG_INFO("[CarboniteLoader] GPU plugins loaded");

        // Regression guard (NVBugs 6262606): when GPU is enabled the CUDA compute
        // plugin must be discoverable here. If it is not, Fabric silently falls
        // back to CPU-only (logs eRequireCuda) and GPU interop is lost. Surface
        // that failure loudly at the loader instead of leaving only the downstream
        // Fabric error. On no-GPU systems disableGpu is true so we skip the check
        // (no lookup, no FAST_FAIL plugin load on driverless Windows).
        if (!disableGpu)
        {
            const carb::PluginDesc& cudaDesc = framework->getPluginDesc("omni.gpucompute-cuda.plugin");
            if (cudaDesc.libPath == nullptr)
                CARB_LOG_WARN("[CarboniteLoader] GPU enabled but omni.gpucompute-cuda.plugin was not "
                              "loaded -- Fabric CUDA will be unavailable. Verify plugins/gpu/ exists and "
                              "is on the plugin search path.");
            else
                CARB_LOG_INFO("[CarboniteLoader] omni.gpucompute-cuda.plugin loaded: %s", cudaDesc.libPath);
        }
    }

    g_bootstrapDone = true;
    CARB_LOG_INFO("[CarboniteLoader] Initialization complete (base plugins)");
    return true;
}

// Load the non-PhysX plugins that need USD (omni.usd and related runtime pieces).
// ovphysx uses the ovstage data path; ovstage loads whatever runtime dependencies
// it needs internally for population.
bool CarboniteLoader::loadUsdDependentPlugins()
{
    if (!m || !m->frameworkAcquired || m->pluginsDir.empty())
    {
        CARB_LOG_ERROR("[CarboniteLoader] Cannot load USD-dependent plugins (framework or plugins dir missing)");
        return false;
    }

    auto* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("[CarboniteLoader] Carbonite framework unavailable");
        return false;
    }

    // A full host (Kit / Isaac Sim) that already loaded the whole USD-dependent
    // plugin stack cannot be detected by usdrt::population::IUtils presence
    // alone: a *partial* host (e.g. ovrtx) can provide IUtils for rendering
    // without providing omni.physicsschema.plugin / omni.usdphysics.plugin /
    // etc. Skipping this block on IUtils presence would silently drop
    // ovphysx's own physics schema plugin and fail PhysX runtime startup
    // with no actionable log line, so we always load below regardless.
    //
    // IPhysxSimulation is no longer a Carbonite-acquired interface, so there is
    // no full-host IPhysxSimulation acquire/version branch here. Standalone and
    // partial hosts fall through to the normal plugin-load path below.
    // IUtils presence is now informational only -- Carbonite's existing
    // "Ignoring plugin: same name already loaded" handling skips any
    // peer-owned plugins without incident.
    auto [searchPathStrings, searchPaths] = buildSearchPaths();

    const char* disableReason = nullptr;
    const bool disableGpu = isGpuDisabled(&disableReason);

    // NOTE: We keep USD loading functional on CPU-only systems by avoiding GPU foundation plugins
    // when GPU is disabled/unavailable. This prevents driverless systems from failing just by
    // calling ovphysx_create_instance() in CPU mode.
    //
    // The order here is intentional. We keep the USD core plugin first and (when GPU is enabled)
    // load GPU foundation immediately after it, before other USD/graphics plugins.
    static const char* kUsdPluginsTail[] = {
        "carb.graphics-vulkan.plugin",
        "carb.shadercompiler-slang.plugin",
        "omni.streamingstatus.plugin",
        "carb.glinterop.plugin",
        // (`omni.usd*.plugin` is loaded below and covers the USD physics/schema plugins.)
    };

    std::vector<const char*> usdPlugins;
    usdPlugins.push_back("omni.usd*.plugin");
    if (!disableGpu)
    {
        usdPlugins.push_back("omni.gpu_foundation*.plugin");
    }
    else
    {
        CARB_LOG_INFO("[CarboniteLoader] Skipping GPU foundation plugins during USD load (GPU disabled)");
    }
    usdPlugins.insert(
        usdPlugins.end(), kUsdPluginsTail, kUsdPluginsTail + (sizeof(kUsdPluginsTail) / sizeof(kUsdPluginsTail[0])));

    CARB_LOG_INFO("[CarboniteLoader] Loading USD-dependent plugins");
    carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
    desc.loadedFileWildcards = usdPlugins.data();
    desc.loadedFileWildcardCount = static_cast<uint32_t>(usdPlugins.size());
    desc.searchPaths = searchPaths.data();
    desc.searchPathCount = static_cast<uint32_t>(searchPaths.size());


    const size_t pluginCountBeforeUsdLoad = framework->getPluginCount();
    framework->loadPlugins(desc);
    const size_t pluginCountAfterUsdLoad = framework->getPluginCount();

    // Diagnostic introspection (see env-var OVPHYSX_COEXIST_DIAGNOSTICS). Answers:
    //   1. did our post-load registry gain plugins?
    //   2. is omni.physicsschema.plugin registered, and at what libPath?
    //   3. which interfaces does it advertise (and at what versions)?
    //   4. can we acquireInterface<IUsdPhysics> right now?
    {
        const char* diagEnv = std::getenv("OVPHYSX_COEXIST_DIAGNOSTICS");
        if (diagEnv && diagEnv[0] == '1')
        {
            std::fprintf(stderr,
                         "[ovphysx-diagnostics] loadUsdDependentPlugins: framework=%p "
                         "pluginCount %zu -> %zu (delta %lld)\n",
                         static_cast<void*>(framework),
                         pluginCountBeforeUsdLoad,
                         pluginCountAfterUsdLoad,
                         static_cast<long long>(pluginCountAfterUsdLoad) -
                             static_cast<long long>(pluginCountBeforeUsdLoad));

            const auto& schemaDesc = framework->getPluginDesc("omni.physicsschema.plugin");
            if (schemaDesc.libPath == nullptr)
            {
                std::fprintf(stderr,
                             "[ovphysx-diagnostics] omni.physicsschema.plugin: NOT REGISTERED in this framework\n");
            }
            else
            {
                std::fprintf(stderr,
                             "[ovphysx-diagnostics] omni.physicsschema.plugin: REGISTERED libPath=%s "
                             "interfaceCount=%zu\n",
                             schemaDesc.libPath, schemaDesc.interfaceCount);
                for (size_t i = 0; i < schemaDesc.interfaceCount; ++i)
                {
                    const auto& iface = schemaDesc.interfaces[i];
                    std::fprintf(stderr,
                                 "[ovphysx-diagnostics]   provides interface [%zu]: %s v%u.%u\n",
                                 i,
                                 iface.name ? iface.name : "<null>",
                                 iface.version.major, iface.version.minor);
                }
            }

            // Probe by-name via getPluginDesc above. We deliberately do NOT
            // tryAcquireInterface<IUsdPhysics>() here because that would
            // require pulling the ovruntime private header into ovphysx's
            // include surface. The name/version tuple above answers the same
            // schema-provider question without depending on PhysX plugin
            // interface metadata.

            // Dump the full plugin registry post-load. If omni.physicsschema
            // is "NOT REGISTERED" above but happens to be listed below under
            // a different name, we'll see it here. Also helpful to see which
            // other ovphysx plugins made it versus didn't.
            const size_t registryTotal = framework->getPluginCount();
            std::vector<carb::PluginDesc> registry(registryTotal);
            framework->getPlugins(registry.data());
            std::fprintf(stderr,
                         "[ovphysx-diagnostics] post-load full plugin registry (%zu entries):\n",
                         registryTotal);
            for (const auto& p : registry)
            {
                std::fprintf(stderr,
                             "[ovphysx-diagnostics]   %-40s  libPath=%s\n",
                             p.impl.name ? p.impl.name : "<null>",
                             p.libPath ? p.libPath : "<null>");
            }
        }
    }

    return true;
}

// Build the plugin search path list for Carbonite loadPlugins().
// The string vector owns the memory; the pointer vector is only valid while the
// returned string vector stays alive in the caller.
std::pair<std::vector<std::string>, std::vector<const char*>> CarboniteLoader::buildSearchPaths() const
{
    std::vector<std::string> strs;
    strs.reserve(3);
    strs.push_back(m->pluginsDir);

    // GPU-only plugins (omni.cubric.plugin, omni.gpucompute-cuda.plugin) are isolated
    // in plugins/gpu/. Only add this search path when GPU is enabled, preventing carb's
    // lazy plugin discovery from loading them on nogpu systems (which crashes with
    // 0xc0000409 / FAST_FAIL_INVALID_ARG when nvcuda64.dll has no device context).
    const std::string gpuPluginDir = m->pluginsDir + "/gpu";
    if (!isGpuDisabled() && std::filesystem::exists(gpuPluginDir))
        strs.push_back(gpuPluginDir);

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

// Wire UJITSO for LOCAL, in-process cooked-collider caching -- no Hub/Nucleus/GRPC.
// Must run BEFORE carb.ujitso.default loads (it builds its datastore from
// /UJITSO/datastore/* at plugin startup) and BEFORE omni.physx.cooking loads (its
// service reads ujitsoCollisionCooking at construction).
//
// The cache directory is APP-PROVIDED: the consuming app passes it via
// PhysXConfig(cooked_collider_cache_dir=...) (C: OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY),
// which is written to /UJITSO/datastore/localCachePath before this runs. ovphysx does
// not read environment variables and does not pick a location on the app's behalf: if
// the app configured no directory, cooking simply runs without cross-run persistence.
void configureUjitsoLocalCache(carb::settings::ISettings* settings)
{
    if (!settings)
        return;

    // Local-only datastore: local on; Hub/Nucleus/GRPC off. setDefault so an explicit
    // app override (via PhysXConfig.carbonite_overrides) can still opt in, but our
    // default flips carb's GRPC default (true)->false. ovphysx is kitless and in-process.
    settings->setDefaultBool("/UJITSO/datastore/allowLocalDataStore", true);
    settings->setDefaultBool("/UJITSO/datastore/allowHubDataStore", false);
    settings->setDefaultBool("/UJITSO/datastore/allowNucleusDataStore", false);
    settings->setDefaultBool("/UJITSO/datastore/allowGRPCDataStore", false);

    // Pre-seed collision cooking ON so the cooking service reads `true` at construction
    // (omni.physx.cooking loads before omni.physx seeds this default -- the seeding race).
    settings->setDefaultBool(omni::physx::kSettingUjitsoCollisionCooking, true);

    // If the app configured a cache directory, make sure it exists so the datastore can
    // persist to it (a non-writable dir degrades to memory-only). WARN, never fail.
    const char* cachePath = settings->getStringBuffer("/UJITSO/datastore/localCachePath");
    if (cachePath && *cachePath)
    {
        std::error_code ec;
        std::filesystem::create_directories(cachePath, ec);
        if (!std::filesystem::exists(cachePath))
        {
            CARB_LOG_WARN("[CarboniteLoader] Configured UJITSO cooked-collider cache dir '%s' is not "
                          "writable; cooking will not persist across runs.", cachePath);
        }
    }
    else
    {
        CARB_LOG_INFO("[CarboniteLoader] No cooked-collider cache directory configured "
                      "(PhysXConfig.cooked_collider_cache_dir); collision cooking runs without "
                      "cross-run persistence.");
    }
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

    // Cubric is GPU-only -- skip explicit loading on CPU-only machines.
    // On Linux this avoids dlopen failure (cubric has DT_NEEDED libcuda.so.1).
    // On Windows, cubric's carbOnPluginStartupEx calls CUDA which crashes with
    // FAST_FAIL_INVALID_ARG when nvcuda64.dll has no device context.
    // The DLL is also isolated to plugins/gpu/ (see install.cmake) to prevent
    // carb's lazy plugin discovery from loading it during populateFromUsd.
    const char* disableReason = nullptr;
    const bool gpuDisabled = isGpuDisabled(&disableReason);

    static const char* kDependencyPluginsGpu[] = {
        "omni.cubric.plugin",
    };

    if (gpuDisabled)
    {
        CARB_LOG_INFO("[CarboniteLoader] Skipping PhysX non-USD dependency plugins (GPU disabled, skipping cubric)");
    }
    else
    {
        CARB_LOG_INFO("[CarboniteLoader] Loading PhysX non-USD dependencies");
        carb::PluginLoadingDesc depDesc = carb::PluginLoadingDesc::getDefault();
        depDesc.loadedFileWildcards = kDependencyPluginsGpu;
        depDesc.loadedFileWildcardCount = static_cast<uint32_t>(sizeof(kDependencyPluginsGpu) / sizeof(kDependencyPluginsGpu[0]));
        depDesc.searchPaths = searchPaths.data();
        depDesc.searchPathCount = static_cast<uint32_t>(searchPaths.size());
        framework->loadPlugins(depDesc);
    }

    // ------------------------------------------------------------------
    // UJITSO local in-process cooked-collider cache (NVBugs 6262606). The kitless
    // loader historically never loaded the UJITSO plugins, so the cooking service
    // found no carb::ujitso::IRegistry and silently cooked uncached -- re-cooking
    // every collider on every IsaacLab launch. Seed the local-only datastore +
    // collision-cooking settings, then load the UJITSO plugins HERE, before
    // omni.physx.cooking constructs its service. No Hub/Nucleus/GRPC (see
    // configureUjitsoLocalCache).
    // ------------------------------------------------------------------
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

// Ensure USD symbols resolve to one namespaced USD runtime in this process.
// On Linux, an already-loaded namespaced USD monolith is promoted to global
// visibility; otherwise the monolith from the ovphysx package is loaded.
// On Windows, DLL resolution already gives process-wide reuse, so detection is
// logged and no explicit preload is needed.
bool CarboniteLoader::preloadUsdLibraries()
{
#ifdef _WIN32
    const std::string loadedNamespacedUsd = omni::sdk::internal::findLoadedNamespacedUsdLibrary();
    if (!loadedNamespacedUsd.empty())
    {
        CARB_LOG_INFO("[CarboniteLoader] Reusing already-loaded namespaced USD: %s",
                      loadedNamespacedUsd.c_str());
    }
    return true;
#else
    if (g_usdPreloadDone)
    {
        return true;
    }
    if (shouldSkipUsdPreload())
    {
        CARB_LOG_INFO("[CarboniteLoader] Skipping USD preload (setting /ovphysx/skipUsdLibPreload=true)");
        g_usdPreloadDone = true;
        return true;
    }
    if (m->pluginsDir.empty())
    {
        CARB_LOG_WARN("[CarboniteLoader] USD preload skipped: plugins directory not set");
        return false;
    }
    const std::string& usdLibDir = m->usdLibDir.empty() ? m->pluginsDir : m->usdLibDir;

    const std::string loadedNamespacedUsd = omni::sdk::internal::findLoadedNamespacedUsdLibrary();
    if (!loadedNamespacedUsd.empty())
    {
        // If another OV library already loaded the same namespaced USD runtime,
        // promote that handle to RTLD_GLOBAL visibility so ovphysx plugins and
        // the clone library bind to the existing image instead of loading a
        // second copy from this package.
        //
        // TOCTOU note: there is a narrow window between findLoadedNamespacedUsdLibrary()
        // returning a hit above and the RTLD_NOLOAD promotion below. If a peer dlclose's
        // the library in that gap, the dlopen here returns null and we fail this
        // initialization. We accept the gap: ovphysx initialization runs early in the
        // process, and OV peers that load USD do not unload it during the same call
        // chain. If this assumption ever breaks, the failure mode is a clean
        // initialization error, not a corrupt load.
        void* existing = dlopen(loadedNamespacedUsd.c_str(), RTLD_NOW | RTLD_NOLOAD | RTLD_GLOBAL);
        if (!existing)
        {
            CARB_LOG_WARN("[CarboniteLoader] Failed to promote already-loaded namespaced USD %s: %s",
                          loadedNamespacedUsd.c_str(), dlerror());
            return false;
        }

        CARB_LOG_INFO("[CarboniteLoader] Reusing already-loaded namespaced USD: %s",
                      loadedNamespacedUsd.c_str());
        g_usdPreloadDone = true;
        return true;
    }

    std::string monolithicLib;
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        for (auto& entry : fs::directory_iterator(usdLibDir, ec))
        {
            auto name = entry.path().filename().string();
            if (name.find("usd_ms.so") != std::string::npos && name.substr(0, 3) == "lib")
            {
                monolithicLib = entry.path().string();
                break;
            }
        }
    }

    if (monolithicLib.empty())
    {
        CARB_LOG_WARN("[CarboniteLoader] Namespaced USD monolith (*usd_ms.so) not found in: %s", usdLibDir.c_str());
        return false;
    }

    void* h = dlopen(monolithicLib.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (!h)
    {
        CARB_LOG_WARN("[CarboniteLoader] %s not loaded: %s", monolithicLib.c_str(), dlerror());
        return false;
    }

    CARB_LOG_INFO("[CarboniteLoader] Preloaded namespaced USD: %s", monolithicLib.c_str());
    g_usdPreloadDone = true;
    return true;
#endif
}

void CarboniteLoader::shutdown()
{
    if (!m || !m->frameworkAcquired)
        return;

    CARB_LOG_INFO("[CarboniteLoader] Shutdown starting (framework=%p)", static_cast<void*>(carb::getFramework()));
    m->physxSim = nullptr;
    m->frameworkAcquired = false;
    CARB_LOG_INFO("[CarboniteLoader] Shutdown complete");
}

omni::physx::IPhysxSimulation* CarboniteLoader::getPhysxSimulation() const
{
    return m ? m->physxSim : nullptr;
}

} // namespace ovphysx
