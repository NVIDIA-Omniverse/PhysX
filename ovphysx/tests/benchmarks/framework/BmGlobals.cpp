// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Adapted from omni.physx/tests/test.benchmarks/framework/BmGlobals.cpp.
//
// The registry / glob-filter / accumulator logic is preserved verbatim so
// future updates from omni.physx merge cleanly. The BmGlobals constructor
// is rewritten because ovphysx does not bootstrap through a
// carb::AppScoped + manual plugin list — it goes through ovphysx::PhysX
// (which handles Carbonite startup, plugin loading and GPU foundation
// creation internally).

#include "BmGlobals.h"

#include "BmUtils.h"

#include <carb/ClientUtils.h>

#include <ovphysx/ovphysx_config.h>
#include <ovphysx/experimental/ovphysx.hpp>

#include <algorithm>
#include <cstring>
#include <string>

namespace
{
BmRegistrable** sRegistry = 0;
uint32_t sSize = 0;
uint32_t sCapacity = 0;
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


// ---------------------------------------------------------------------------
// BmGlobals: ovphysx-flavored. Creates a single PhysX instance (CPU or GPU
// depending on forceGpu). The unused omni.physx-specific parameters are
// accepted but ignored to keep the constructor signature interchangeable.
// ---------------------------------------------------------------------------

BmGlobals::BmGlobals(bool sanityCheck,
                     const char* dataFolder,
                     int32_t numThreads,
                     bool forceGpu,
                     bool profile,
                     bool /*enableTracy*/,
                     const char** /*kitArguments*/,
                     uint32_t /*kitArgumentCount*/)
    : mSanityCheck(sanityCheck),
      mNumThreads(numThreads),
      mForceGpu(forceGpu),
      mProfile(profile),
      mLifecycleInitialized(false),
      mCurrentTestName(""),
      mDataFolder(dataFolder),
      mPhysX(nullptr),
      mFramework(nullptr),
      mFileSystem(nullptr)
{
    mThis = this;

    ovphysx::CreateArgs args;
    // Device (CPU vs GPU) is resolved per-stage during ovstage ingest from
    // physxScene:enableGPUDynamics, not as a create-arg. The forceGpu() gate
    // on each benchmark ensures only stages matching the current pass's
    // device are loaded.

    // --threads=N on the CLI sets the Carbonite /physics/numThreads setting
    // BEFORE PhysX bootstrap, so the dispatcher comes up with N workers
    // (1 = single-threaded baseline; 0 = auto). N == -1 means "do not
    // override" — leave the default in place. The PhysicsScene's USD attrs
    // don't carry a thread-count knob; this is the only correct route.
    // The entries array must outlive PhysX::create — make it a static/stack
    // local that survives the call.
    ovphysx_config_entry_t numThreadsEntry{};
    if (numThreads >= 0)
    {
        numThreadsEntry = ovphysx_config_entry_num_threads(numThreads);
        args.setConfigEntries(&numThreadsEntry, 1);
    }

    ovphysx_result_t initResult = ovphysx_initialize();
    if (initResult.status != OVPHYSX_API_SUCCESS)
    {
        printFormatted("BmGlobals: ovphysx_initialize failed with status=%d", static_cast<int>(initResult.status));
        return;
    }
    mLifecycleInitialized = true;

    mPhysX = new ovphysx::PhysX();
    ovphysx_api_status_t st = ovphysx::PhysX::create(*mPhysX, args);
    if (st != OVPHYSX_API_SUCCESS)
    {
        printFormatted("BmGlobals: PhysX::create failed with status=%d", static_cast<int>(st));
        delete mPhysX;
        mPhysX = nullptr;
        ovphysx_shutdown();
        mLifecycleInitialized = false;
        return;
    }

    // Acquire carb framework + IFileSystem now that ovphysx::PhysX::create
    // has performed Carbonite bootstrap. BmOutput.cpp uses these to write
    // its report and baseline files.
    //
    // carb::getFramework() reads a module-local pointer. ovphysx initialized
    // its own (inside libovphysx.so) but ours -- in the benchmark binary --
    // is still null at this point. We use carb::acquireFramework() (rather
    // than acquireFrameworkAndRegisterBuiltins) so we only set the local
    // pointer without registering atexit handlers that would conflict with
    // ovphysx's own teardown on process exit.
    mFramework = carb::getFramework();
    if (!mFramework)
    {
        mFramework = carb::acquireFramework(g_carbClientName.c_str());
        if (mFramework)
        {
            g_carbFramework = mFramework;
        }
    }
    if (mFramework)
    {
        mFileSystem = mFramework->acquireInterface<carb::filesystem::IFileSystem>();
    }
}

BmGlobals::~BmGlobals()
{
    // Clear our local framework / IFileSystem references (they belong to
    // ovphysx; we just observed them) before destroying the PhysX runtime.
    mFileSystem = nullptr;
    mFramework = nullptr;
    g_carbFramework = nullptr;
    delete mPhysX;
    mPhysX = nullptr;
    if (mLifecycleInitialized)
    {
        ovphysx_shutdown();
        mLifecycleInitialized = false;
    }
}

std::string BmGlobals::getDataFolder() const
{
    if (mDataFolder)
        return mDataFolder;
    if (mFileSystem)
    {
        // Match the omni.physx fallback: relative to the executable's directory.
        // Note that the actual base path differs because ovphysx and omni.physx
        // tree layouts are different; the --data CLI flag is the preferred way
        // to point at fixtures and is what scripts/test_benchmarks_cpp.cmake uses.
        return std::string(mFileSystem->getAppDirectoryPath()) + "/../../../tests/data";
    }
    return "tests/data";
}

void BmGlobals::release()
{
    // No carb scripting context to tear down here. Kept as a no-op so the
    // teardown sequence matches the omni.physx harness.
}

void bmCreateGlobals(bool sanityCheck,
                     const char* dataFolder,
                     int32_t numThreads,
                     bool forceGpu,
                     bool profile,
                     bool enableTracy,
                     const char** kitArguments,
                     uint32_t kitArgumentCount)
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

// ---------------------------------------------------------------------------

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
