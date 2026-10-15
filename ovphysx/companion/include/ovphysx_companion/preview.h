// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-PREVIEW-001
// @covers AC-1 AC-2 AC-4 AC-5 AC-6 AC-7
#ifndef OVPHYSX_COMPANION_PREVIEW_H
#define OVPHYSX_COMPANION_PREVIEW_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif

#define OVPHYSX_COMPANION_PREVIEW_VERSION 1u
/** Copied settings. Bits select overrides; all other values come from source
 *  attachment settings (or runtime defaults if the source instance is unattached).
 *  The core instance is only a bootstrap/settings source: it may be destroyed
 *  before this preview. The stage/dictionary/sealed ordinal must remain alive
 *  until close succeeds. No USD pointer is accepted.
 */
enum {
    OVPHYSX_COMPANION_PREVIEW_THREADS = 1u << 0,
    OVPHYSX_COMPANION_PREVIEW_DEVICE = 1u << 1,
    OVPHYSX_COMPANION_PREVIEW_PIPELINE = 1u << 2,
    OVPHYSX_COMPANION_PREVIEW_SOLVER = 1u << 3
};
typedef struct ovphysx_companion_preview_desc {
    uint32_t size;
    uint32_t version;
    uint64_t source_instance;
    const void* stage;
    void* dictionary;
    uint64_t read_ordinal;
    uint32_t overrides;
    uint32_t threads;
    int32_t cuda_device;
    int32_t gpu_pipeline; /* -1 authored, 0 CPU, 1 GPU */
    int32_t solver; /* -1 authored, 0 PGS, 1 TGS */
    uint32_t excluded_path_count;
    const char* const* excluded_paths;
} ovphysx_companion_preview_desc_t;

typedef struct ovphysx_companion_pose {
    float position[3]; /* scene world, stage distance units */
    float orientation[4]; /* xyzw */
} ovphysx_companion_pose_t;

/** All operations are synchronous admissions. simulate starts one pending
 *  interval; fetch collects only that interval. close drains only this owner.
 *  Same-owner concurrent calls and callback reentry report BUSY. Distinct
 *  owners may step independently. Failure preserves output and live handles.
 *  The caller must close every preview before core shutdown. Handles are unique
 *  in this table, not interchangeable with borrowed tools session handles.
 */
typedef struct ovphysx_companion_preview_api {
    uint32_t size;
    uint32_t version;
    ovphysx_companion_status_t (*open)(const ovphysx_companion_preview_desc_t*, uint64_t*);
    ovphysx_companion_status_t (*close)(uint64_t);
    ovphysx_companion_status_t (*identity)(uint64_t, ovphysx_companion_identity_t*);
    ovphysx_companion_status_t (*simulate)(uint64_t, float);
    ovphysx_companion_status_t (*fetch)(uint64_t);
    ovphysx_companion_status_t (*statistics)(uint64_t, const char*, uint32_t, ovphysx_companion_scene_statistics_t*);
    ovphysx_companion_status_t (*rigid_pose)(uint64_t, const char*, ovphysx_companion_pose_t*);
    ovphysx_companion_status_t (*update)(uint64_t, uint64_t, uint64_t);
} ovphysx_companion_preview_api_t;

/** Exact version/size negotiation; no runtime initialization, no output on failure. */
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_preview_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_preview_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
