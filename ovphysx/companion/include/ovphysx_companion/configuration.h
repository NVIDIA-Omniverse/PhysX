// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-CONFIG-001
// @covers AC-1 AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_CONFIGURATION_H
#define OVPHYSX_COMPANION_CONFIGURATION_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
/** Captured native request values. No dispatcher/context pointer is exposed.
 * Resource requests are not a claim that a CUDA context was successfully created.
 * flags: bit0 contact-report processing suppressed (collision response stays enabled), bit1 native USD pose output, bit2 velocity output,
 * bit3 particle output, bit4 borrow active CUDA context, bit5 synchronous CUDA,
 * bit6 PhysX dispatcher, bit7 suppress readback, bit8 local-space velocity output.
 * Solver/pipeline: -1 follows authored input, 0 PGS/CPU, 1 TGS/GPU. Device -1 is auto.
 */
typedef struct ovphysx_companion_configuration_snapshot {
    uint32_t size,version;
    uint64_t attachment,step;
    uint32_t threads,flags;
    int32_t cuda_device,multi_gpu_mode,gpu_pipeline,solver;
    float gizmo_scale;
    uint32_t reserved;
} ovphysx_companion_configuration_snapshot_t;
/** Exact idle owner; named read and contact-report policy operations.
 * Other settings are immutable through this interface. Use copied preview
 * options to construct a replacement, keeping peer configuration unchanged.
 * Failure preserves caller output. No simulation or source edit is performed.
 */
typedef struct ovphysx_companion_configuration_api {
    uint32_t size,version;
    uint64_t capabilities; /* bit0 copied config; bit1 owner-local contact policy */
    ovphysx_companion_status_t (*get_configuration)(uint64_t attachment,
        uint32_t output_size,ovphysx_companion_configuration_snapshot_t* output);
    ovphysx_companion_status_t (*set_contact_processing_disabled)(uint64_t attachment,uint32_t disabled,
        uint32_t output_size,ovphysx_companion_configuration_snapshot_t* output);
} ovphysx_companion_configuration_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_configuration_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_configuration_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
