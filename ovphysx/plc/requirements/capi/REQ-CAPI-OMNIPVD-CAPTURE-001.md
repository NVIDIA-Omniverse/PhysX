<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OMNIPVD-CAPTURE-001
title: Attachment Scoped OmniPVD Recording Options
status: implemented
owner: ovphysx
---

## Description

The C API selects producer-side OmniPVD objects and optional secondary data before a FILE or TCP session starts, and processes receiver selections at the owner's public-step submission safe point.

## Acceptance Criteria

- AC-1: **One start function.** `ovphysx_ovd_start_recording(handle, destination, options)` accepts an optional `ovphysx_omnipvd_recording_options_t` with `bool record_secondary_data` and length-bearing scene and object path arrays. The destination layout is unchanged. C clients must rebuild and pass `NULL` to retain unfiltered behavior; the former two-argument C signature is not binary-compatible.
- AC-2: **Validation before opening.** Missing nonempty arrays and malformed exact absolute paths return `INVALID_ARGUMENT`; missing or unsupported physics objects return `NOT_FOUND`, and neither failure opens the destination or activates sampling.
- AC-3: **Owner resolution.** Populated lists require the caller's attachment, resolve exclusively within it, and combine as a union; scenes select current and future members, object paths select every supported native instance at the exact prim, and duplicate paths are permitted. Object registry scans hold one owner execution scope and report a busy owner as `INVALID_STATE`. Concurrent calls on the same instance remain unsupported.
- AC-4: **Copied policy.** Changes to caller arrays or `record_secondary_data` after start do not change capture; owner stop permits a fresh filter on restart, while active takeover and peer stop retain existing rejection and receipt-preservation rules.
- AC-5: **Primary and secondary data.** Poses, velocities, geometry and structural identity are always recorded for selected objects. `record_secondary_data=true` enables all other supported attributes, including contacts and forces, and is the value set by `OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT`. `false` records primary data only. A null descriptor selects all data and objects; empty lists select all objects, including starts by an unattached peer.
- AC-6: **Streamed class coverage.** Object paths accept rigid statics/dynamics, whole articulations, PBD particle systems, individual particle/diffuse buffers and deformable surfaces/volumes in ordinary GPU and DirectGPU scenes. Exact prototype paths include all native instances; selecting one buffer excludes sibling payloads. Primary particle payloads contain active position/inverse-mass and velocity entries plus uint32 phases in snapshots and after phase updates. Diffuse position/lifetime and velocity arrays require secondary data. Deformable vertex positions and velocities remain recorded. Fixed-base root-joint aliases are accepted only when the registry identifies a whole articulation. Actual links, joints, constraints, shapes, materials, meshes and aggregates cannot be selected directly; their streamed data follows selected owners and native dependency rules.

- AC-7: **Discovery before recording.** `ovphysx_ovd_start_recording` with `options.wait_for_selection=true` accepts only TCP and requires the caller's live attachment. It emits an identity catalog bounded to that attachment and the optional maximum filter without motion, geometry, contacts or GPU payload readback. The owner reports an active session while waiting for selection; peer status is false, peer stop is rejected, and owner stop/detach finalizes the session. Receiver selection cannot expand the original scope. The options' `record_secondary_data` is ignored while waiting. `wait_for_selection` defaults to false; null options retain immediate unrestricted recording. Each receiver selection chooses whether secondary data is recorded.

- AC-8: **Receiver submission safe point.** A pending receiver request is processed on the owner's public `ovphysx_step` or `ovphysx_step_sync` submission thread before submitting or executing that step. Processing waits for earlier queued and running work across all instances without collecting their task outcomes; empty polls do not drain peer work. Busy runtime or companion activity postpones the request to a later public step. `ovphysx_warmup` does not poll receiver requests.

## Test References

- [TEST-CAPI-OMNIPVD-CAPTURE-001](../../tests/capi/TEST-CAPI-OMNIPVD-CAPTURE-001.md) (AC-1 through AC-8)

## Code References

- `ovphysx/include/ovphysx/ovphysx_types.h` and `ovphysx/include/ovphysx/ovphysx.h`: options and unified start export (AC-1, AC-5).
- `ovphysx/src/ovphysx/ovphysx.cpp`: validation, owner-scoped resolution, discovery and public-step receiver processing (AC-2 through AC-5, AC-7, AC-8).
- `ovphysx/ovruntime/include/omni/physx/OmniPvdCaptureResolver.h`: runtime-owned declaration used by the C bridge; runtime resolution is specified separately by REQ-OMNIPVD-CAPTURE-001 AC-5 (AC-3, AC-6).
- `ovphysx/src/include/ovphysx_ABI_check.h` and `ovphysx/src/ovphysx/ovphysx_ABI_check.cpp`: descriptor layout probes (AC-1).
- `ovphysx/tests/c_unittests/test_c_api_compatibility.c` and `ovphysx/tests/c_unittests/test_omnipvd_recording.cpp`: ABI, decoded CPU/ordinary GPU/DirectGPU sessions and receiver submission ordering (AC-1 through AC-8).
- `ovphysx/src/ovphysx/ovphysxTestHelpers.cpp`: private bridge creating a real runtime-owned D6 joint for unsupported-type rejection coverage (AC-6).

## Dependencies

- [REQ-CAPI-OMNIPVD-LATE-001](REQ-CAPI-OMNIPVD-LATE-001.md)
