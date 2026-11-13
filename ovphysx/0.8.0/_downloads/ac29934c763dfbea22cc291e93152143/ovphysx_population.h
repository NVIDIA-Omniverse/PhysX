// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-POPULATION-DESC-001
 * @covers AC-1 AC-5
 */
/** @endcond */

#ifndef OVPHYSX_POPULATION_H
#define OVPHYSX_POPULATION_H

#include "ovphysx/ovphysx_export.h"

#include <ovstage/ovstage_population_types.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @defgroup ovphysx_population Physics population description
     * What ovphysx reads from a USD stage, as the OVStage population description that populates it.
     *
     * Include this header alongside `ovphysx.h`, which does not include it.
     */

    /** @addtogroup ovphysx_population */
    /** @{ */

    /**
     * @brief The OVStage population description of everything ovphysx reads from a USD stage.
     *
     * OVStage populates physics data only through population descriptions passed to its `*_with_desc` entry
     * points (`ovstage_population_open_usd_from_file_with_desc`, `ovstage_population_open_usd_from_string_with_desc`,
     * `ovstage_population_repopulate_usd_with_desc`). This is the description ovphysx needs: physics prims with
     * their physics properties, the geometry of the shapes, character controllers, deformables and particle sets,
     * collider GeomSubsets, prims that bind a physics material, PointInstancers instancing physics, Boundable prims
     * a physics relationship targets, physics instance roots, the stage units, and the values ovphysx relies on
     * where nothing is authored: unit-scaled defaults, the resolved physics material and collection membership.
     * The physics population description reference in the ovphysx documentation lists it in full.
     *
     * Its `domains` is `OVSTAGE_POPULATION_DOMAIN_NONE`. Pass it alone to populate physics. To populate render
     * prims as well, pass it in the same array as a description whose `domains` is
     * `OVSTAGE_POPULATION_DOMAIN_RENDERING`. A description of your own for other data goes in the same array;
     * two descriptions in one call may not derive the same column.
     *
     * @code
     * ovstage_population_desc_t descs[2] = { *ovphysx_get_physics_population_desc(), { 0 } };
     * descs[1].domains = OVSTAGE_POPULATION_DOMAIN_RENDERING;
     * ovstage_population_open_usd_from_file_with_desc(stage, path, ordinal, OVSTAGE_POPULATION_DEFAULT_TIME, descs, 2);
     * @endcode
     *
     * The description is immutable static data: the same pointer on every call, valid while the ovphysx library
     * stays loaded, never NULL. The call needs no ovphysx_initialize, instance or attached stage, does not load
     * the OVStage runtime, and is safe from any thread. Its content is versioned with ovphysx, which generates it
     * from the same contract as its parser.
     *
     * @return The physics population description; never NULL.
     */
    OVPHYSX_API const ovstage_population_desc_t* ovphysx_get_physics_population_desc(void);

    /** @} */

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_POPULATION_H */
