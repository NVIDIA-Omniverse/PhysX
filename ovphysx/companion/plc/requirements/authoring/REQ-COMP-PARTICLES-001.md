<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-PARTICLES-001
title: "Particle sampling and postprocessing data services"
status: implemented
owner: ovphysx
---

## Description

Replace the legacy private particle sampler/postprocessor lifecycle with optional typed services; Python authors particle or surface output.

## Acceptance Criteria

- AC-1: **Lifecycle.** A sampler is scoped to a source identity/revision and supports create, resample and release; input edits invalidate cached sampling state.
- AC-2: **Typed output.** Sampling and supported smoothing, anisotropy and isosurface operations return owned arrays with coordinate/units/topology metadata and per-operation backend capabilities.
- AC-3: **Python authoring.** Shared Python helpers apply selected particle sets or output surfaces through explicit customer-USD edits; native particle services never select prims or mutate USD.
- AC-4: **Cleanup.** Cancellation, object deletion and owner teardown drain work and release sampler/postprocessing resources; stale results cannot be applied.

## Test References

- [TEST-COMP-PARTICLES-001](../../tests/authoring/TEST-COMP-PARTICLES-001.md)

## Code References

- [Native table](../../../include/ovphysx_companion/particles.h) and [stopped service bridge](../../../src/Cooking.cpp).
- [Sampling lifecycle and immutable records](../../../python/ovphysx_companion/particles.py).
- [Customer-USD capture/application](../../../python/ovphysx_companion/usd/particles.py).
- [Editor/worker sample](../../../samples/particle_authoring.py).

## Advertised operations

CPU Poisson surface sampling and optional interior filling return copied points,
triangle IDs and barycentrics in the input frame and stage units. Surface radius
is minimum separation; volume radius is average spacing. Volume input must be a
closed oriented two-manifold without intersecting shells. Maximum sample count
bounds output; stable random seeds/order are not advertised. Smoothing, anisotropy,
isosurface and GPU operations have separate absent capabilities and return
Unavailable. The surface USD helper accepts explicitly supplied provider data;
it does not imply that a native isosurface provider exists.

Each Sampler retains one immutable input/revision; resampling recomputes and
invalidates older deliveries. Host edits/deletion call invalidate/close. Close
cancels publication and drains every outstanding request before service release.
The service shares the existing stopped cooking lease and can outlive its
bootstrap core instance. It owns no attachment and performs no USD calls.
Python applies only the selected managed Points/Mesh output in an explicit layer,
validates the current source revision, converts world points into destination
local coordinates, and returns an undo patch. Particle system selection belongs
to the caller; the helper never creates a solver or silently saves a layer.

## Dependencies

- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)
