// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"


#include "BenchmarkList.h"

#include "framework/BmGlobals.h"

// there needs to be a function of this form in every benchmark file, and that function should be called by the
// initialize routine below. It doesn't need to do anything, it just forces the compilation unit into the executable
// without having to hack any build settings

extern void initPhysicsUSDSimulate();
extern void initPhysicsAttributeChange();
extern void initPhysicsKinematicBodies();
extern void initPhysicsVehicle();
extern void initPhysicsSimulate();
extern void initPhysicsReplicator();
extern void initPhysicsCompetitive();


void bmInitialize(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool profile,
    bool enableTracy, const char** kitArguments, uint32_t kitArgumentCount)
{
    bmCreateGlobals(sanityCheck, dataFolder, numThreads, forceGpu, profile, enableTracy,
        kitArguments, kitArgumentCount);

    initPhysicsUSDSimulate();
    initPhysicsAttributeChange();
    initPhysicsKinematicBodies();
    initPhysicsVehicle();
    initPhysicsReplicator();
    initPhysicsCompetitive();
}

void bmTerminate()
{
    bmDestroyGlobals();
}
