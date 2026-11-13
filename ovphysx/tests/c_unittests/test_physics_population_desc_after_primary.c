// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-DESC-001
 * @covers AC-1
 */

/* The population header composes with the primary header, which does not include it. */
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_population.h"

_Static_assert(
    _Generic(&ovphysx_get_physics_population_desc, const ovstage_population_desc_t* (*)(void): 1, default: 0),
    "primary then population include names ovphysx_get_physics_population_desc");
