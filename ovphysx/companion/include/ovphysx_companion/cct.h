// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-CCT-001
// @covers AC-1 AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_CCT_H
#define OVPHYSX_COMPANION_CCT_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
enum {
    OVPHYSX_COMPANION_CCT_DESCRIBE = 0,
    OVPHYSX_COMPANION_CCT_MOVE = 1,
    OVPHYSX_COMPANION_CCT_TELEPORT = 2,
    OVPHYSX_COMPANION_CCT_RESIZE = 3,
    OVPHYSX_COMPANION_CCT_REMOVE = 4
};
typedef struct ovphysx_companion_cct_command {
    uint32_t size, version, action, reserved;
    uint64_t generation;
    double position[3];
    float displacement[3];
    float height, radius, dt, min_distance;
    uint32_t filters;
} ovphysx_companion_cct_command_t;
typedef struct ovphysx_companion_cct_state {
    uint32_t size, version;
    uint64_t attachment, key, generation;
    double position[3];
    float up[3], gravity[3];
    float height, radius;
    uint32_t collisions;
} ovphysx_companion_cct_state_t;
/** CPU capsule controllers created by ovstage population/attachment. Hold the
 *  attachment's borrowed or owned companion lease while calling this table.
 *  Calls require idle owner admission and never step simulation or edit USD.
 *  All positions/distances use world stage length units; dt is seconds in (0,1].
 *  Describe with generation=0 acquires the current controller identity. All other
 *  commands require it, including Remove. Deletion/recreation invalidates identity.
 *  Move's filters: bit0 static shapes, bit1 dynamic shapes, bit2 other controllers.
 *  Scene-query flags apply; simulation collision-group policy is not implicit.
 *  Collisions: bit0 side, bit1 above, bit2 below. No Python callback runs in move.
 *  Teleport does not test collisions. Resize preserves feet without testing overlap.
 *  Remove releases only the native CCT, leaving source input untouched; recreate
 *  through explicit source repopulation or a new attachment. GPU scenes unavailable.
 *  Rejected calls preserve output; Remove success returns an empty state.
 */
typedef struct ovphysx_companion_cct_api {
    uint32_t size, version;
    uint64_t capabilities; /* bit0 CPU capsule controls */
    ovphysx_companion_status_t (*execute)(uint64_t attachment, const char* path,
        const ovphysx_companion_cct_command_t* command, uint32_t output_size,
        ovphysx_companion_cct_state_t* output);
} ovphysx_companion_cct_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_cct_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_cct_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
