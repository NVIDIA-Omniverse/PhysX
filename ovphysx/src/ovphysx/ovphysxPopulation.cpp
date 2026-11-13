// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-DESC-001
 * @covers AC-1
 */

#include "ovphysx/ovphysx_population.h"

#include "internal/Profiling.h"

#include <omni/physics/population/PhysicsPopulationDesc.h>

const ovstage_population_desc_t* ovphysx_get_physics_population_desc(void)
{
    OVPHYSX_API_PROFILE_ZONE(get_physics_population_desc);
    return &omni::physics::population::physicsPopulationDesc();
}
