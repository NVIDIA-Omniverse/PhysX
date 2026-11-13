// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-DESC-001
 * @covers AC-1
 */

/* The population header compiles on its own: it brings the OVStage description types it returns. */
#include "ovphysx/ovphysx_population.h"

_Static_assert(
    _Generic(&ovphysx_get_physics_population_desc, const ovstage_population_desc_t* (*)(void): 1, default: 0),
    "population-only include names ovphysx_get_physics_population_desc");
_Static_assert(OVSTAGE_POPULATION_DOMAIN_RENDERING == 1, "population-only include names the RENDERING domain");
