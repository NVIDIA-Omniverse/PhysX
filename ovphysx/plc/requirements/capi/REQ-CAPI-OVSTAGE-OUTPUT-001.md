<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-OUTPUT-001
title: Application-Owned C++ World-Transform Publication
status: implemented
owner: ovphysx
---

## Description

`ovphysx::utils::writeWorldTransformsToOvstage()` publishes fixed physics poses
to an application-owned ovstage. Each matrix composes the physics pose with the
signed world scale ovphysx stores per body and serves as `worldScale` through
the read API, so publication reads no world matrix from the stage. The
optional `OvStageOutputCache` retains reusable output buffers outside the
physics simulation. The application owns simulation, output ordinals and stage
sealing.

## Acceptance Criteria

- AC-1: **Utility surface and selection.** The C++17 utility in
  `experimental/OvStageOutput.hpp` publishes every fixed rigid-body and
  articulation-link pose group on CPU or supported CUDA devices. It skips
  point-instancer array groups and reports their count in
  `instancerGroupsSkipped`. Its public header requires neither CUDA nor PhysX
  SDK headers or a CUDA compiler in the consuming application.
- AC-2: **World matrices preserve scale.** Publication writes row-vector
  float64, 16-lane MATRIX values to `omni:fabric:worldMatrix`, combining the
  physics `position` and `orientation` with the `worldScale` read in the same
  physics read. That scale is the parser's signed decomposition of the body's
  world matrix at creation, including its reflection convention; shear is not
  preserved. Direct edits to the stage's world matrices do not change the
  published scale, nor does a runtime scale edit that physics does not apply
  (character controllers, DirectGPU rigid bodies, articulation links, kinematic
  bodies driven by time-sampled transforms): the
  utility publishes the creation scale over the matrix a hierarchy
  recomputation derived from the edit. A body with no world matrix on the
  stage is published. A character controller's matrix
  uses its prim orientation, with the capsule-axis correction applied by the
  physics read, and its scale keeps the controller prim's authored axes.
  The utility writes neither `omni:xform` nor `omni:resetXformStack` directly and does
  not update descendants.
- AC-3: **Application-owned ordering.** The utility does not step, advance a
  write floor, drain stage edits, or publish a transform journal. Its documented
  caller obligations are to supply the actual attached stage, complete the
  simulation step, seal previous output before reuse, and choose an output
  ordinal above the write floor that is never drained into physics.
- AC-4: **Optional owning cache.** Without a cache, each call allocates its
  output buffers. A cache owns reusable CPU/CUDA matrix buffers per published
  prim set and device, binds to the instance, stage pointer and attachment on
  first use, and rejects reuse with a different binding. Cached, uncached and
  refreshed calls publish the same matrices. `refresh()` discards buffers but
  retains the binding, so it does not permit reuse after detach/reattach. The
  instance and stage must outlive the cache, and callers serialize its use
  with simulation and stage edits.
- AC-5: **Native CUDA execution.** On a DirectGPU scene the pose and
  `worldScale` columns are device-resident; matrix composition reads them on
  the device without a host copy, runs in the primary context of the columns'
  DLPack device ordinal, waits for every producer's readiness, and completes
  before borrowed data or output buffers are released or reused. The utility
  does not access the simulation's CUDA manager and restores the calling
  thread's prior CUDA context on return.
- AC-6: **Results, errors and lifetimes.** `OvStageOutputResult` reports native
  status, completed matrix rows, skipped instancer groups and error text;
  `ok()` means `OVPHYSX_API_SUCCESS`, including empty output. A stage other
  than the instance's attached stage is rejected before stage access or cache
  binding. A missing or malformed pose or `worldScale` column and nonfinite
  pose or scale values fail before the first write, without silent
  replacement values. Physics read storage remains alive until its consumers
  finish.
  A later write failure reports earlier completed rows in `matricesWritten`;
  those writes remain unsealed and are not rolled back.
- AC-7: **Published poses join the hierarchy at the output ordinal.** After a
  publication that wrote at least one matrix, the utility computes the
  hierarchy at `outputOrdinal` with the CPU model
  (`OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU`), whatever the runtime
  default, before the caller seals it. The CPU model folds each written world
  matrix into that prim's local transform against its parent at that ordinal;
  the GPU models would instead rederive the world matrix from the old local
  transform and discard the output. A later edit of a parent then moves the
  published body with it, in OVStage and, once the edit is drained, in physics
  (OMPE-120176). A failed computation reports an error like a failed write;
  the written matrices stay unsealed.

## Known limitations

Only fixed rigid-body and articulation-link transforms are published. Native
point-instancer arrays, vehicle wheels and other outputs remain application
work. CUDA columns must be addressable from the primary context of their
DLPack device ordinal. Each publication that writes a matrix also runs one CPU
hierarchy computation, also when the runtime default is a GPU model.

## Test References

- [TEST-CAPI-OVSTAGE-OUTPUT-001](../../tests/capi/TEST-CAPI-OVSTAGE-OUTPUT-001.md)
  records coverage and outstanding validation.

## Code References

- ovphysx/include/ovphysx/experimental/OvStageOutput.hpp
  (`OvStageOutputCache`, `OvStageOutputResult`, `writeWorldTransformsToOvstage`;
  AC-1 through AC-7)
- ovphysx/src/ovphysx/OvStageOutput.cpp (selection, the position /
  orientation / worldScale read, cache, publication, hierarchy computation and
  ownership; AC-1 through AC-7)
- ovphysx/src/ovphysx/OvStageOutputCuda.cu (device matrix processing; AC-2, AC-5)
- ovphysx/src/ovphysx/OvStageOutputMath.h (shared CPU/CUDA matrix composition;
  AC-2, AC-6)
- ovphysx/include/ovphysx/ovphysx_attributes.h (`OVPHYSX_ATTR_WORLD_SCALE`,
  the stored scale AC-2 composes)
