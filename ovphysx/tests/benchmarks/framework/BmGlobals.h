// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Adapted from omni.physx/tests/test.benchmarks/framework/BmGlobals.h.
//
// The class shape, registration mechanism (BmRegistrable / Register<>), the
// matching-filter helpers, and the BmRecord / BmResult types are kept
// identical so updates from omni.physx are easy to merge back. The carb
// accessors are also preserved -- BmOutput.cpp uses them and we want it
// byte-identical -- but they query carb via the runtime initialized by
// ovphysx::PhysX::create() instead of the omni.physx AppScoped path.
// Python-scripting accessors return null because ovphysx has no python
// benchmark hook through this code path (the python suite lives in
// tests/python_benchmarks/ and is pytest-driven).

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
              bool profile,
              bool enableTracy,
              const char** kitArguments,
              uint32_t kitArgumentCount);
    ~BmGlobals();

    static BmGlobals& getInstance()
    {
        return *mThis;
    }

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

    carb::Framework* getFramework()
    {
        return mFramework;
    }

    // Python scripting is not wired up in ovphysx's benchmark harness; the
    // python suite lives at tests/python_benchmarks/. These accessors return
    // null so the unmodified BmOutput.cpp / Harness.cpp from omni.physx
    // still compile.
    carb::scripting::IScripting* getPythonScripting() const { return nullptr; }
    carb::scripting::Context* getPythonContext() const { return nullptr; }

    carb::filesystem::IFileSystem* getFileSystem() const
    {
        return mFileSystem;
    }

    // ovphysx-specific accessor: the single shared PhysX runtime.
    ovphysx::PhysX* getPhysX() const
    {
        return mPhysX;
    }

    std::string getDataFolder() const;

    void release();

private:
    BmGlobals& operator=(const BmGlobals&);
    static BmGlobals* mThis;

    bool mSanityCheck;
    int32_t mNumThreads;
    bool mForceGpu;
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
};


struct BmResult
{
    uint64_t avgTime;
    uint64_t stdDevTime;
    size_t maxMemory;
};


BmResult bmGetDefaultResult(BmRecord& r);

#endif
