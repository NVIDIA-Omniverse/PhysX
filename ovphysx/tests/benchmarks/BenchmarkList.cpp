// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Same shape as omni.physx/tests/test.benchmarks/BenchmarkList.cpp: each
// benchmark .cpp exposes an init*() symbol that this file references, just
// to force the translation unit into the binary so its static registrations
// (Register<BClass> bench_BClass{...};) fire. The set of init functions
// listed here is the ovphysx-specific set of benchmarks.

#include "framework/UsdPCH.h"

#include "BenchmarkList.h"

#include "framework/BmGlobals.h"

extern void initSmoke();
extern void initUsdLoad();
extern void initStepCpu();
extern void initStepGpu();
extern void initClone();
extern void initTensorBindings();
extern void initLowLoad();
extern void initLabCartpole();
extern void initLabAnymal();


void bmInitialize(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool profile,
    bool enableTracy, const char** kitArguments, uint32_t kitArgumentCount)
{
    bmCreateGlobals(sanityCheck, dataFolder, numThreads, forceGpu, profile, enableTracy,
        kitArguments, kitArgumentCount);

    initSmoke();
    initUsdLoad();
    initStepCpu();
    initStepGpu();
    initClone();
    initTensorBindings();
    initLowLoad();
    initLabCartpole();
    initLabAnymal();
}

void bmTerminate()
{
    bmDestroyGlobals();
}
