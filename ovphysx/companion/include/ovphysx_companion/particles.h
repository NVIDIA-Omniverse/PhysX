// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-PARTICLES-001
// @covers AC-1 AC-2 AC-4
#ifndef OVPHYSX_COMPANION_PARTICLES_H
#define OVPHYSX_COMPANION_PARTICLES_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ovphysx_companion_sampling_options {
    float radius, volume_radius;
    uint32_t max_samples, attempts;
} ovphysx_companion_sampling_options_t;
/** Stopped CPU sampling, no native stage dependency. Input and output share one
 *  caller-declared Cartesian frame/unit. Arrays are packed XYZ/triangle triples.
 *  Output capacities are max_samples, weights are barycentric XYZ and triangle
 *  is -1 for interior samples. No output buffer is written on failure.
 *  Capability bits: 0 surface, 1 volume, 2 smoothing, 3 anisotropy, 4 isosurface.
 *  This version advertises only bits 0 and 1, both CPU. No deterministic seed API.
 *  Open retains existing runtime services. Close returns Busy during sampling;
 *  callers drain cancelled work before retrying. It never owns/steps an attachment.
 */
typedef struct ovphysx_companion_particles_api {
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*open)(uint64_t,uint64_t*);
    ovphysx_companion_status_t (*close)(uint64_t);
    ovphysx_companion_status_t (*sample)(uint64_t,const float*,uint32_t,const uint32_t*,uint32_t,
        const ovphysx_companion_sampling_options_t*,uint32_t,float*,int32_t*,float*,uint32_t*);
} ovphysx_companion_particles_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_particles_acquire(
    uint32_t,uint32_t,ovphysx_companion_particles_api_t*);
#ifdef __cplusplus
}
#endif
#endif
