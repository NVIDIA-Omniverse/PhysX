// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "Benchmark.h"
#include "Interaction.h"
#include "SceneQuery.h"
#include "Simulation.h"
#include "StageUpdate.h"

namespace omni
{
namespace physics
{

// Simulation structure that contains the functions
// classes for each subsystem.
struct Simulation
{
    BenchmarkFns benchmarkFns;
    InteractionFns interactionFns;
    SceneQueryFns sceneQueryFns;
    SimulationFns simulationFns;
    StageUpdateFns stageUpdateFns;
};

} // namespace physics
} // namespace omni
