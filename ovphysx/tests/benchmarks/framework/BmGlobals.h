// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-4
 */

// Adapted from omni.physx/tests/test.benchmarks/framework/BmGlobals.h.
//
// The class shape, registration mechanism (BmRegistrable / Register<>), the
// matching-filter helpers, and the BmRecord / BmResult types are kept
// identical so updates from omni.physx are easy to merge back. The one
// deviation is BmRecord::executed, documented at its declaration below.
// Upstream has no way to tell an unexecuted record from a measured one,
// which the fail-closed contract requires. The carb accessors are also
// preserved so BmOutput.cpp stays byte-identical, but they query carb via
// the runtime initialized by ovphysx::PhysX::create() instead of the
// omni.physx AppScoped path. Python-scripting accessors return null because
// ovphysx has no python benchmark hook through this code path. The python
// suite lives in tests/python_benchmarks/ and is pytest-driven.

#ifndef BENCHMARK_GLOBALS_H
#define BENCHMARK_GLOBALS_H

#include <carb/Defines.h>
#include <carb/Framework.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/scripting/IScripting.h>

#include <ovphysx/experimental/ovphysx.hpp>

#include <string>
#include <vector>

class BmBenchmark;

void bmRegisterBenchmark(class BmRegistrable&);

class BmRegistrable
{
public:
    BmRegistrable(const char* name) : mName(name)
    {
        bmRegisterBenchmark(*this);
    }
    virtual ~BmRegistrable()
    {
    }
    virtual BmBenchmark* create() = 0;

    virtual bool isHidden() const = 0;
    const char* getName() const
    {
        return mName.c_str();
    }
    void addPostfix(const char* postfix)
    {
        mName = mName + std::string(postfix);
    }

private:
    std::string mName;
};


#if CARB_PLATFORM_WINDOWS
#    pragma warning(push)
#    pragma warning(disable : 4316) //  object allocated on the heap may not be aligned 128
#endif

template <class B, bool hide = false>
class Register : BmRegistrable
{
public:
    Register(const char* name) : BmRegistrable(name)
    {
    }
    virtual bool isHidden() const
    {
        return hide;
    }
    virtual BmBenchmark* create()
    {
        return new B;
    }
};

#if CARB_PLATFORM_WINDOWS
#    pragma warning(pop)
#endif


void bmGetRegister(std::vector<BmRegistrable*>& out, const char* filter, bool includeHidden);


class BmGlobals
{
public:
    BmGlobals(bool sanityCheck,
              const char* dataFolder,
              int32_t numThreads,
              bool forceGpu,
              bool directGpu,
              bool profile,
              bool enableTracy,
              const char** kitArguments,
              uint32_t kitArgumentCount);
    ~BmGlobals();

    static BmGlobals& getInstance()
    {
        return *mThis;
    }

    // Releases and destroys the shared instance, leaving mThis null. Idempotent, because the
    // harness terminates early on bootstrap and on timing-diagnostics failure as well as at
    // the end of a normal run, and a second call must not double-free.
    static void destroyInstance();

    bool sanityCheck() const
    {
        return mSanityCheck;
    }

    bool enableProfile() const
    {
        return mProfile;
    }

    int32_t numThreads() const
    {
        return mNumThreads;
    }

    bool forceGpu() const
    {
        return mForceGpu;
    }

    bool directGpu() const
    {
        return mDirectGpu;
    }

    carb::Framework* getFramework()
    {
        return mFramework;
    }

    // Python scripting is not wired up in ovphysx's benchmark harness. The
    // python suite lives at tests/python_benchmarks/. These accessors return
    // null so the unmodified BmOutput.cpp / Harness.cpp from omni.physx
    // still compile.
    carb::scripting::IScripting* getPythonScripting() const { return nullptr; }
    carb::scripting::Context* getPythonContext() const { return nullptr; }

    carb::filesystem::IFileSystem* getFileSystem() const
    {
        return mFileSystem;
    }

    // ovphysx-specific accessor for the single shared PhysX runtime.
    ovphysx::PhysX* getPhysX() const
    {
        return mPhysX;
    }

    std::string getDataFolder() const;

    void release();

private:
    BmGlobals& operator=(const BmGlobals&);
    static BmGlobals* mThis;
    // True only when this object acquired the module-local carb framework itself, which is
    // the only case in which ~BmGlobals may clear it.
    bool mOwnsCarbFramework = false;

    bool mSanityCheck;
    int32_t mNumThreads;
    bool mForceGpu;
    // Enables GPU tensors. Suppresses GPU readback, PhysX to USD writes,
    // and USD prim edits flowing through Fabric to PhysX.
    bool mDirectGpu;
    bool mProfile;
    bool mLifecycleInitialized;
    const char* mCurrentTestName;
    const char* mDataFolder;

    ovphysx::PhysX* mPhysX;
    carb::Framework* mFramework;
    carb::filesystem::IFileSystem* mFileSystem;
};

void bmCreateGlobals(bool sanityCheck,
                     const char* dataFolder,
                     int32_t numThreads,
                     bool forceGpu,
                     bool directGpu,
                     bool profile,
                     bool enableTracy,
                     const char** kitArguments = NULL,
                     uint32_t kitArgumentCount = 0);
void bmDestroyGlobals();


class BmRecord
{
public:
    std::vector<uint64_t> time;
    std::vector<uint64_t> stdDev; // for each step the standard deviation over all accepted runs

    // False until the harness has completed this row's measured runs. A record
    // that never ran holds no samples, and bmGetDefaultResult() reduces an
    // empty record to zero. That value is indistinguishable from a real
    // measurement in the report and, under --regenerate, becomes a zero
    // baseline entry that permanently disables comparison for that row. Only
    // an executed record may be published.
    bool executed = false;
};


struct BmResult
{
    uint64_t avgTime;
    uint64_t stdDevTime;
    size_t maxMemory;
};


BmResult bmGetDefaultResult(BmRecord& r);

#endif
