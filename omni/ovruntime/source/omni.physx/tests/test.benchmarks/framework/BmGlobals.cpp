// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"


#include "BmGlobals.h"

#include "BmUtils.h"

#include <carb/ClientUtils.h>
#include <carb/dictionary/IDictionary.h>
#include <carb/dictionary/ISerializer.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/logging/Log.h>
#include <carb/logging/Logger.h>
#include <carb/logging/LoggingSettingsUtils.h>
#include <carb/settings/ISettings.h>
#include <carb/settings/SettingsUtils.h>
#include <carb/profiler/IProfiler.h>

#include <omni/physx/IPhysxFoundation.h>

#include <pxr/base/plug/registry.h>

#include <algorithm>
#include <filesystem>
#include <string>

// ------------------------------------------------------------------------

using namespace carb;

namespace
{
BmRegistrable** sRegistry = 0;
uint32_t sSize = 0;
uint32_t sCapacity;

// All physics plugins needed by the benchmark suite, loaded via Carbonite directly (no Kit extension manager).
// Order matters: dependencies must be loaded before dependents.
const std::vector<const char*> kPhysicsPlugins = {
    // Carbonite infrastructure plugins (found in CARB_SDK_SEARCH_PATH)
    "carb.tasking.plugin",
    // RTX plugins (found in RTX_PLUGINS_SEARCH_PATH)
    "carb.stats.plugin",
    // Ujitso dependencies (carb.datasource-file from CARB_SDK, datastore+blobkey from RTX)
    "carb.datasource-file.plugin",
    "carb.datastore.plugin",
    "omni.blobkey.plugin",
    // Ujitso plugins (found in RTX_UJITSO_AGENT_SEARCH_PATH / RTX_UJITSO_DEFAULT_SEARCH_PATH)
    "carb.ujitsoagent.plugin",
    "carb.ujitso.default.plugin",
    // TBB global control (found in RTX_USD_SEARCH_PATH, required by omni.fabric)
    "omni.tbb.globalcontrol.plugin",
    // Fabric plugin (found in RTX_FABRIC_SEARCH_PATH)
    "omni.fabric.plugin",
    // USDRT plugins (found in RTX_SCENEGRAPH_SEARCH_PATH / RTX_USDRT_SEARCH_PATH)
    // These are hybrid Carbonite+ONI plugins: omniCarbStartup registers both sides.
    "usdrt.scenegraph.plugin",
    "usdrt.hierarchy.plugin",
    "usdrt.population.plugin",
    // Cubric adapter (found in RTX_CUBRIC_SEARCH_PATH, required by omni.physx.fabric.plugin)
    "omni.cubric.plugin",
    // GPU compute CUDA backend (found in RTX_GPUCOMPUTE_SEARCH_PATH, required by fabric GPU compute)
    "omni.gpucompute-cuda.plugin",
    // Our built plugins (found in "." relative to executable)
    "omni.convexdecomposition.plugin",
    "omni.usdphysics.plugin",
    "omni.physx.gpu.plugin",
    "omni.physx.foundation.plugin",
    "omni.physx.cooking.plugin",
    "omni.physx.plugin",
    // Fabric plugin (loaded here so fabric benchmarks work)
    "omni.physx.fabric.plugin",
    // Scripting (needed for python benchmarks)
    "carb.scripting-python.plugin",
};

} // namespace

void bmRegisterBenchmark(BmRegistrable& benchmark)
{
    if (sSize == sCapacity)
    {
        BmRegistrable** oldRegistry = sRegistry;
        uint32_t oldCapacity = sCapacity;
        sCapacity = sCapacity ? sCapacity * 2 : 16;

        sRegistry = new BmRegistrable*[sCapacity];
        memcpy(sRegistry, oldRegistry, oldCapacity * sizeof(BmRegistrable*));
        if (oldRegistry)
            delete[] oldRegistry;
    }
    sRegistry[sSize++] = &benchmark;
}

// adapted from similar code in gtest
bool matchString(const char* pattern, const char* str)
{
    switch (*pattern)
    {
    case '\0':
    case ':':
        return *str == '\0';
    case '?': // any single character.
        return *str != '\0' && matchString(pattern + 1, str + 1);
    case '*': // any (possibly empty) string  of characters.
        return (*str != '\0' && matchString(pattern, str + 1)) || matchString(pattern + 1, str);
    default:
        return *pattern == *str && matchString(pattern + 1, str + 1);
    }
}


bool matchFilter(const char* filter, const char* name)
{
    bool match = false;
    bool noIncludeFilter = true;
    while (filter)
    {
        bool exclude = *filter == '-';
        filter += exclude; // skips the exclude prefix (the '-' character)

        if (matchString(filter, name))
        {
            if (exclude)
                return false;
            else
                match = true;
        }

        filter = strchr(filter, ':'); // finds the next pattern in the filter.
        filter += !!filter; // skips the pattern separater (the ':' character).

        noIncludeFilter &= exclude;
    }
    return match || noIncludeFilter;
}


struct StrLess
{
    bool operator()(const BmRegistrable* a, const BmRegistrable* b) const
    {
        return BmStricmp(a->getName(), b->getName()) < 0;
    }
};

void bmGetRegister(std::vector<BmRegistrable*>& a, const char* filterString, bool includeHidden)
{
    for (uint32_t i = 0; i < sSize; i++)
    {
        if (matchFilter(filterString, sRegistry[i]->getName()) && (!sRegistry[i]->isHidden() || includeHidden))
            a.push_back(sRegistry[i]);
    }

    std::sort(a.begin(), a.end(), StrLess());
}

BmGlobals::BmGlobals(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool profile,
    bool enableTracy, const char** kitArguments, uint32_t kitArgumentCount)
    : mSanityCheck(sanityCheck), mNumThreads(numThreads), mForceGpu(forceGpu), mProfile(profile), mCurrentTestName(""), mDataFolder(dataFolder)
{
    mThis = this;

    mApp = new carb::AppScoped();
    mApp->startupEmpty();

    // Threading limits (match Kit CI defaults, OMPE-59303)
    {
        carb::settings::ISettings* s = mApp->getFramework()->acquireInterface<carb::settings::ISettings>();
        s->setDefaultInt("/plugins/carb.tasking.plugin/threadCount", 16);
        s->setDefaultInt("/plugins/omni.tbb.globalcontrol/maxThreadCount", 16);
    }

    // Register physxSchema and physxSchemaAddition with the USD plug registry.
    // Without this, USD schema types (e.g. PhysxArticulationAPI) are unknown and
    // schema-based APIs fail silently. The path is set by CMake from usd_ext_physics.
#if defined(USD_EXT_PHYSICS_PLUGIN_PATH)
    {
        std::filesystem::path schemaPluginDir =
            std::filesystem::absolute(std::filesystem::path(USD_EXT_PHYSICS_PLUGIN_PATH));
        std::string schemaPluginDirStr = schemaPluginDir.string();
        PXR_NS::PlugRegistry& plugRegistry = PXR_NS::PlugRegistry::GetInstance();
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/PhysxSchema/resources");
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/PhysxSchemaAddition/resources");
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/OmniUsdPhysicsDeformableSchema/resources");
    }
#endif

    carb::FrameworkScoped& framework = *mApp;

    // Load all physics plugins directly via Carbonite (no Kit extension manager)
    framework.loadPlugins(kPhysicsPlugins);

    // Load Tracy profiler plugin if requested
    if (enableTracy)
    {
        CARB_LOG_INFO("Loading Tracy profiler plugin...");
        mApp->loadPlugins({ "carb.profiler-tracy.plugin" });

        carb::profiler::IProfiler* profilerTracy = framework->tryAcquireInterface<carb::profiler::IProfiler>("carb.profiler-tracy.plugin");
        if (profilerTracy)
        {
            // Set capture mask to 1 to start profiling
            profilerTracy->setCaptureMask(1);
            CARB_LOG_INFO("Tracy profiler enabled and connected.");
        }
        else
        {
            CARB_LOG_WARN("Failed to acquire Tracy profiler interface. Make sure carb.profiler-tracy.plugin is available.");
        }
    }

    omni::physx::IPhysxFoundation* foundation = framework->acquireInterface<omni::physx::IPhysxFoundation>();

    mFileSystem = framework->acquireInterface<carb::filesystem::IFileSystem>();

    mScriptingPython =
        framework->tryAcquireInterface<carb::scripting::IScripting>("carb.scripting-python.plugin");
    if (mScriptingPython)
    {
        mScriptingPython->addPluginBindingFoldersToSearchPath();
        mContextPython = mScriptingPython->createContext();
    }
    else
    {
        CARB_LOG_WARN("Python scripting plugin not available — Python benchmarks will be skipped.");
        mContextPython = nullptr;
    }
}

BmGlobals::~BmGlobals()
{
    delete mApp;
}

std::string BmGlobals::getDataFolder() const
{
    if (mDataFolder)
        return mDataFolder;
    else
    {
        // exe lives in _build/linux-x86_64/release/ (or _build/<platform>/<config>/ on Windows)
        // Test data lives in omni/ovexts/ (sibling of omni/ovruntime/)
        static const std::string g_carbBuildDirectory = "/../../../../ovexts";
        return  getFileSystem()->getAppDirectoryPath() + g_carbBuildDirectory;
    }
}


void BmGlobals::release()
{
    if (mScriptingPython)
    {
        if (mContextPython)
            mScriptingPython->destroyContext(mContextPython);
        carb::getFramework()->releaseInterface(mScriptingPython);
    }
}

void bmCreateGlobals(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool profile,
    bool enableTracy, const char** kitArguments, uint32_t kitArgumentCount)
{
    new BmGlobals(sanityCheck, dataFolder, numThreads, forceGpu, profile, enableTracy,
        kitArguments, kitArgumentCount);
}

void bmDestroyGlobals()
{
    BmGlobals::getInstance().release();

    delete &BmGlobals::getInstance();
}

BmGlobals* BmGlobals::mThis;

// ------------------------------------------------------------------------

BmResult bmGetDefaultResult(BmRecord& r)
{
    BmResult result;
    result.maxMemory = 0;
    uint64_t size = r.time.size();

    uint64_t total = 0;
    uint64_t stdDevSum = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        total += r.time[i];
        stdDevSum += r.stdDev[i];
    }

    if (size == 0) {
        result.avgTime = 0;
        result.stdDevTime = 0;
    } else {
        result.avgTime = total / size;
        result.stdDevTime = stdDevSum / size;
    }

    return result;
}
