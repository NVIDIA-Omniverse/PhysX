// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-004
 * @covers AC-1 AC-5
 */

// Same shape as omni.physx/tests/test.benchmarks/BenchmarkList.cpp. Each
// benchmark .cpp exposes an init*() symbol that this file references to force
// the translation unit into the binary so its static registrations
// (Register<BClass> bench_BClass{...};) run. The init functions listed here
// are the ovphysx-specific set of benchmarks.

#include "framework/UsdPCH.h"

#include "BenchmarkList.h"

#include "framework/BmGlobals.h"

extern void initSmoke();
extern void initUsdLoad();
extern void initInstancingAttach();
extern void initStepCpu();
extern void initStepGpu();
extern void initClone();
extern void initOutputRead();
extern void initOutputWrite();
extern void initTensorBindings();
extern void initLowLoad();
extern void initLabCartpole();
extern void initLabAnymal();
extern void initAuthoringPopulationAdd();
extern void initAuthoringTeleport();
extern void initAuthoringDrain();
extern void initAuthoringPopulationChurn();
extern void initAuthoringPopulationScale();
extern void initAuthoringRuntimeWriteSpawn();
extern void initWriteScaling();
extern void initContactReport();


void bmInitialize(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool directGpu,
    bool profile, bool enableTracy, const char** kitArguments, uint32_t kitArgumentCount)
{
    bmCreateGlobals(sanityCheck, dataFolder, numThreads, forceGpu, directGpu, profile, enableTracy,
        kitArguments, kitArgumentCount);

    initSmoke();
    initUsdLoad();
    initInstancingAttach();
    initStepCpu();
    initStepGpu();
    initClone();
    initOutputRead();
    initOutputWrite();
    initTensorBindings();
    initLowLoad();
    initLabCartpole();
    initLabAnymal();
    initAuthoringPopulationAdd();
    initAuthoringTeleport();
    initAuthoringDrain();
    initAuthoringPopulationChurn();
    initAuthoringPopulationScale();
    initAuthoringRuntimeWriteSpawn();
    initWriteScaling();
    initContactReport();
}

void bmTerminate()
{
    bmDestroyGlobals();
}
