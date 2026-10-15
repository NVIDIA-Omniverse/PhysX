// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-DEBUG-001
// @covers AC-1 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#ifndef OVPHYSX_COMPANION_DEBUG_H
#define OVPHYSX_COMPANION_DEBUG_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
#define OVPHYSX_COMPANION_DEBUG_LIMIT 8u
/** Parameter bit positions match the documented Python Parameter enum. */
typedef struct ovphysx_companion_debug_config {
    uint32_t size, version;
    uint64_t parameters;
    float scale;
    uint32_t enabled, culling, reserved;
    float minimum[3], maximum[3];
} ovphysx_companion_debug_config_t;
typedef struct ovphysx_companion_debug_vertex {
    float position[3];
    uint32_t color; /* packed 0xAARRGGBB */
} ovphysx_companion_debug_vertex_t;
typedef struct ovphysx_companion_debug_info {
    uint32_t size, version;
    uint64_t attachment, step;
    uint64_t points, lines, triangles; /* primitive counts, not vertex counts */
} ovphysx_companion_debug_info_t;
/** CPU copied debug geometry; capabilities bit0. No export/PVD connectivity is
 * advertised by this table. Configure takes exclusive host visualization policy;
 * settings apply on the next simulation step. Scale is multiplied by the owner's
 * viewportGizmoScale. Positions/culling bounds are world stage length units.
 * Capture requires idle completion and creates an owned bounded immutable frame.
 * Actor/shape wireframes use each scene's latest pre-step poses (the last substep
 * when substepping); contact/friction primitives describe that step's contacts.
 * info.step is the owner's native execution counter, including substeps and scenes,
 * not a host-frame number.
 * Limit (8) means max_vertices was exceeded; no truncated frame is published.
 * Frames survive owner retirement until release; no runtime lease is needed to
 * copy or release them. Hold a companion owner lease for configure/capture.
 * Copy primitive kind 0/1/2 means point/line/triangle. NULL/zero queries required
 * vertices. Insufficient capacity changes only required_vertices and returns
 * INVALID_ARGUMENT. All other failed calls preserve output. Retained copies have
 * no implied ordering or per-object identity. Host renders on its chosen thread.
 */
typedef struct ovphysx_companion_debug_api {
    uint32_t size, version;
    uint64_t capabilities, parameter_mask;
    ovphysx_companion_status_t (*configure)(uint64_t attachment, const ovphysx_companion_debug_config_t* config);
    ovphysx_companion_status_t (*capture)(uint64_t attachment, uint64_t max_vertices, uint64_t* frame);
    ovphysx_companion_status_t (*describe)(uint64_t frame, uint32_t size, ovphysx_companion_debug_info_t* info);
    ovphysx_companion_status_t (*copy)(uint64_t frame, uint32_t kind, ovphysx_companion_debug_vertex_t* data,
        uint64_t capacity_vertices, uint64_t* required_vertices);
    ovphysx_companion_status_t (*release)(uint64_t frame);
} ovphysx_companion_debug_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_debug_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_debug_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
