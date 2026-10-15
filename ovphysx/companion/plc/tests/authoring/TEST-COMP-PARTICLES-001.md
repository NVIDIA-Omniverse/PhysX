<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PARTICLES-001
maps_to: REQ-COMP-PARTICLES-001
type: integration
---

## Coverage status

Implemented and executed for advertised CPU sampling and Python authoring. Covers [REQ-COMP-PARTICLES-001](../../requirements/authoring/REQ-COMP-PARTICLES-001.md) AC-1 through AC-4.

## Scenario

Resample an edited mesh while a previous particle/postprocess job is pending.

## Given

- A closed sample volume, deterministic seed where supported, an initial sampler and a selected postprocess mode.
- A Python result-application target and owned CPU output arrays.

## When

- Start sampling, edit the mesh and resample, apply the latest completed output, then delete/release the sampler while work remains.

## Then

- The edit causes recomputation under the new revision rather than reuse of old sampling state (REQ AC-1).
- Supported outputs have valid counts/layouts and unsupported postprocess/backend pairs return capability errors (REQ AC-2).
- Only Python changes the requested particle/surface prim and leaves selection policy to the caller (REQ AC-3).
- Old results are refused and drain leaves no callbacks or owned allocations (REQ AC-4).

## Executable coverage

- [Native integration](../../../tests_native/test_particles.py): `test_surface_volume_mapping_bounds_and_capabilities` verifies pair spacing, triangle/weight reconstruction, interior sentinels and explicit unavailable operations (AC-2); `test_sampling_invalid_geometry_and_abi_canaries` preserves all output on invalid geometry/negotiation (AC-2); `test_resample_revision_cancel_and_release_drain` exercises pending revision replacement, cancellation, old-delivery rejection and teardown join (AC-1/4); `test_sampling_during_pending_preview_preserves_simulation` verifies independent sampling during an uncollected preview interval (AC-4).
- [Customer USD](../../../tests/test_particles.py): `test_particle_apply_transform_undo_stale_and_selected_system`, `test_surface_output_foreign_revision_units_and_atomicity`, `test_particle_sampling_editor_sample` exercise selected output, transforms, deletion/revision rejection, units, undo/redo and an isolated native worker (AC-1/2/3/4).

All four native integration and three customer-USD cases pass; the native sampling
shard passes. No GPU postprocessing or particle-solver qualification is claimed.
Draining is checked with a deterministic held worker; no native allocation-counter
instrumentation or SDK cancellation preemption is assumed.
