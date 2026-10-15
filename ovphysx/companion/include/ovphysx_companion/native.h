// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_NATIVE_H
#define OVPHYSX_COMPANION_NATIVE_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
/** Control-plane only. Module callbacks use the separate build-matched C++ SDK.
 *  v1 requires hard process CPU mode before load; other modes return UNAVAILABLE
 *  before loading code. GPU/DirectGPU are not advertised by this version.
 *  Paths must be absolute. Load retains both module code and runtime services.
 *  Close returns BUSY until all native objects and snapshots release the type;
 *  retry after deleting the objects or detaching. Never force-unload the module.
 *  Handles are process-local, never reused. Output handles change only on success.
 */
typedef struct ovphysx_companion_native_api
{
    uint32_t size;
    uint32_t version;
    ovphysx_companion_status_t (*load)(uint64_t source_instance, const char* absolute_path, uint64_t* module);
    ovphysx_companion_status_t (*close)(uint64_t module);
    /** Exact native attachment and authored joint path. Operations require idle
     *  state and reject another module's joint. Refresh consumes current sealed
     *  source parameters; update the owner's input ordinal before refreshing.
     *  action: 0 mark dirty, 1 set flags, 2 get flags, 3 refresh parameters.
     *  flags is required for set/get; bit 0 is ALWAYS_UPDATE. */
    ovphysx_companion_status_t (*joint_control)(uint64_t module, uint64_t attachment,
        const char* path, uint32_t action, uint32_t* flags);
} ovphysx_companion_native_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_native_api_t* output);

/** Companion CPU provider capabilities. Unset bits must not be inferred from
 * registration success. Articulation DOFs, GPU/DirectGPU callbacks, GPU-only
 * particle/deformable interaction, SDF fallback and mesh merging are unavailable.
 * Optional query callbacks are reported per registered module, independently.
 */
enum {
    OVPHYSX_COMPANION_NATIVE_JOINT_CPU = 1u << 0,
    OVPHYSX_COMPANION_NATIVE_GEOMETRY_CPU = 1u << 1,
    OVPHYSX_COMPANION_NATIVE_JOINT_REFRESH = 1u << 2,
    OVPHYSX_COMPANION_NATIVE_RAYCAST = 1u << 3,
    OVPHYSX_COMPANION_NATIVE_OVERLAP = 1u << 4,
    OVPHYSX_COMPANION_NATIVE_SWEEP = 1u << 5,
    OVPHYSX_COMPANION_NATIVE_DEBUG = 1u << 6
};
typedef struct ovphysx_companion_native_capabilities_api {
    uint32_t size;
    uint32_t version;
    /** A live module is required; output remains unchanged on failure.
     * Missing optional callbacks produce no hits/overlap/debug in native PhysX;
     * consumers should disable their corresponding actions using these bits. */
    ovphysx_companion_status_t (*query)(uint64_t module, uint32_t* capabilities);
} ovphysx_companion_native_capabilities_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_capabilities_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_native_capabilities_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
