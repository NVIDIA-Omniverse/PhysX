// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "LogManager.hpp"
#include "Log.h"
#include "internal/GlobalState.hpp"
#include "internal/Profiling.h"
#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include <ovphysx/ovphysx.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysXRuntime.h>
#include <carb/Framework.h>
#include <carb/detail/SetRuntimeGlobals.h>
#include <carb/dictionary/IDictionary.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <exception>
#include <mutex>

// Supplied by the static browser host, registered into the existing stage framework.
void ovphysxWasmRegisterEvents();
extern "C" carb::Framework* ovphysx_wasm_borrow_framework();

namespace ovphysx
{
namespace
{
bool requireConfig(ovphysx_global_config_key_t key, bool expected, std::string& error)
{
    ovphysx_config_value_t value{};
    const ovphysx_result_t result = ovphysx_get_global_config(key, &value);
    if (result.status == OVPHYSX_API_SUCCESS && value.type == OVPHYSX_CONFIG_VALUE_BOOL &&
        value.value.bool_value == expected)
        return true;
    error = key == OVPHYSX_GLOBAL_CONFIG_CPU_ONLY ?
        "Static WASM runtime requires CPU_ONLY=true before instance creation" :
        "Static WASM runtime requires ASYNC_EXECUTION=false before instance creation";
    return false;
}
}

struct CarboniteLoader::Impl
{
    bool frameworkAcquired = false;
    std::string lastError;
    omni::physx::IPhysxSimulation* physxSim = nullptr;
};

CarboniteLoader::CarboniteLoader() : m(new Impl()) {}
CarboniteLoader::~CarboniteLoader()
{
    shutdown();
    delete m;
}

const std::string& CarboniteLoader::getLastError() const
{
    return m->lastError;
}

bool CarboniteLoader::initialize()
{
    m->lastError.clear();
    if (m->frameworkAcquired)
        return true;
    carb::Framework* framework = ovphysx_wasm_borrow_framework();
    if (!framework)
    {
        m->lastError = "Static WASM runtime requires the existing OVStage host framework";
        return false;
    }
    if (g_carbFramework && g_carbFramework != framework)
    {
        m->lastError = "WASM physics is already attached to a different host framework";
        return false;
    }
    // Each module caches the same borrowed host and allocator in its own Carbonite globals.
    g_carbFramework = framework;
    carb::detail::setRuntimeGlobalsFromFramework(framework);
    carb::settings::ISettings* settings = framework->tryAcquireExistingInterface<carb::settings::ISettings>();
    carb::tasking::ITasking* tasking = framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
    if (!settings || !tasking || !framework->tryAcquireExistingInterface<carb::dictionary::IDictionary>())
    {
        m->lastError = "Static WASM runtime requires host settings, dictionary and tasking interfaces";
        return false;
    }
    if (tasking->getDesc().threadCount != 0)
    {
        m->lastError = "Static WASM runtime currently supports a zero-worker host scheduler";
        return false;
    }
    if (!requireConfig(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, true, m->lastError) ||
        !requireConfig(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, false, m->lastError))
        return false;

    CarboniteLoaderState& state = globalState().carboniteLoader;
    {
        std::lock_guard<std::mutex> guard(state.bootstrapMutex);
        if (!state.bootstrapDone)
        {
            try
            {
                ovphysxWasmRegisterEvents();
            }
            catch (const std::exception& error)
            {
                m->lastError = error.what();
                return false;
            }
            if (!framework->tryAcquireExistingInterface<carb::events::IEvents>())
            {
                m->lastError = "Static WASM runtime requires a registered IEvents provider";
                return false;
            }
            settings->setBool(omni::physx::kSettingForceCpuMode, true);
            settings->setInt(omni::physx::kSettingNumThreads, 0);
            settings->setBool(omni::physx::kSettingUjitsoCollisionCooking, false);
            settings->setBool(omni::physx::kSettingUjitsoRemoteCacheEnabled, false);
            state.ownsFramework = false;
            state.compiledCacheUnavailable.store(true, std::memory_order_release);
            state.bootstrapDone = true;
        }
        state.activeLoaders.fetch_add(1, std::memory_order_relaxed);
        m->frameworkAcquired = true;
    }
    // Logging callbacks may reenter configuration, so invoke them without the loader lock.
    ovphysxWasmInitializeLog();
    onCarboniteLoggingReady();
    return true;
}

bool CarboniteLoader::loadPhysxPlugins()
{
    m->lastError.clear();
    if (!m->frameworkAcquired)
    {
        m->lastError = "Static WASM base bootstrap has not completed";
        return false;
    }
    if (!requireConfig(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, true, m->lastError) ||
        !requireConfig(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, false, m->lastError))
        return false;
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings = framework ?
        framework->tryAcquireExistingInterface<carb::settings::ISettings>() : nullptr;
    if (!settings)
    {
        m->lastError = "The borrowed OVStage host settings interface is unavailable";
        return false;
    }
    if (!settings->getAsBool(omni::physx::kSettingForceCpuMode))
    {
        m->lastError = "Static WASM runtime cannot disable the process CPU-only setting";
        return false;
    }
    if (settings->getAsInt(omni::physx::kSettingNumThreads) != 0 ||
        settings->getAsBool(omni::physx::kSettingUjitsoCollisionCooking) ||
        settings->getAsBool(omni::physx::kSettingUjitsoRemoteCacheEnabled))
    {
        m->lastError = "Static WASM runtime requires zero solver workers and disabled UJITSO caches";
        return false;
    }
    if (settings->getAsInt(profiling::kSettingProfilerBackend) != OVPHYSX_PROFILER_BACKEND_NONE)
    {
        m->lastError = "Native profiler plugins are unavailable in the static WASM runtime";
        return false;
    }
    try
    {
        // This is the original runtime factory and initialization sequence.
        omni::physx::runtime::startup();
        m->physxSim = omni::physx::runtime::tryGetPhysxSimulationInterface();
    }
    catch (const std::exception& error)
    {
        m->lastError = error.what();
        return false;
    }
    if (!m->physxSim)
    {
        m->lastError = "The linked PhysX runtime did not publish IPhysxSimulation";
        return false;
    }
    return true;
}

void CarboniteLoader::shutdown()
{
    if (!m->frameworkAcquired)
        return;
    CarboniteLoaderState& state = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(state.bootstrapMutex);
    m->physxSim = nullptr;
    m->frameworkAcquired = false;
    state.activeLoaders.fetch_sub(1, std::memory_order_acq_rel);
    // Public ovphysx_shutdown owns runtime draining. OVStage owns the shared host.
}

omni::physx::IPhysxSimulation* CarboniteLoader::getPhysxSimulation() const
{
    return m->physxSim;
}

BoundProfilerState boundProfilerState()
{
    return {};
}

bool usesConfiguredCookedColliderCache()
{
    return false;
}

bool compiledStageCacheUnavailable()
{
    return true;
}

void shutdownCarbonite()
{
    shutdownLogCallback();
    CarboniteLoaderState& state = globalState().carboniteLoader;
    std::lock_guard<std::mutex> guard(state.bootstrapMutex);
    if (state.activeLoaders.load(std::memory_order_acquire) == 0)
        state.bootstrapDone = false;
    // This module borrowed the host; it must not reset or release its services.
}
}
