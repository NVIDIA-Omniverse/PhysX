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
to an application-owned ovstage. The optional `OvStageOutputCache` retains
scale snapshots and reusable buffers outside the physics simulation. The
application owns simulation, output ordinals and stage sealing.

## Acceptance Criteria

- AC-1: **Utility surface and selection.** The C++17 utility in
  `experimental/OvStageOutput.hpp` publishes every fixed rigid-body and
  articulation-link pose group on CPU or supported CUDA devices. It skips
  point-instancer array groups and reports their count in
  `instancerGroupsSkipped`. Its public header requires neither CUDA nor PhysX
  SDK headers or a CUDA compiler in the consuming application.
- AC-2: **World matrices preserve scale.** Publication writes row-vector
  float64, 16-lane MATRIX values to `omni:fabric:worldMatrix`, combining the
  physics position and orientation with signed scale captured from a sealed
  world matrix. Scale matches `omni::physx::decomposeMatrix`, including its
  reflection convention; shear is not preserved. The utility changes neither
  `omni:xform` nor `omni:resetXformStack` and does not update descendants.
- AC-3: **Application-owned ordering.** The utility does not step, advance a
  write floor, drain stage edits, or publish a transform journal. Its documented
  caller obligations are to supply the actual attached stage, complete the
  simulation step, provide current hierarchy-computed and sealed world
  matrices, seal previous output before reuse, and choose an output ordinal
  above the write floor that is never drained into physics.
- AC-4: **Optional owning cache.** Without a cache, each call derives scale
  from current stage values. A cache owns its snapshots and buffers, binds to
  the instance, stage pointer and attachment on first use, and rejects reuse
  with a different binding. `refresh()` discards snapshots and buffers after
  authored transform or topology changes but retains the binding, so it does
  not permit reuse after detach/reattach. The instance and stage must outlive
  the cache, and callers serialize its use with simulation and stage edits.
- AC-5: **Native CUDA execution.** CUDA pose processing and matrix composition
  run in the primary context of the column's DLPack device ordinal, wait for
  producer readiness, and complete before borrowed data or output buffers are
  released or reused. The utility does not access the simulation's CUDA manager
  and restores the calling thread's prior CUDA context on return.
- AC-6: **Results, errors and lifetimes.** `OvStageOutputResult` reports native
  status, completed matrix rows, skipped instancer groups and error text;
  `ok()` means `OVPHYSX_API_SUCCESS`, including empty output. A stage other
  than the instance's attached stage is rejected before stage access or cache
  binding. Missing or malformed source matrices and invalid pose data fail
  without silent replacement values. Borrowed stage reads are released before overlapping
  writes, and physics read storage remains alive until its consumers finish.
  A later write failure reports earlier completed rows in `matricesWritten`;
  those writes remain unsealed and are not rolled back.

## Known limitations

Only fixed rigid-body and articulation-link transforms are published. Native
point-instancer arrays, vehicle wheels and other outputs remain application
work. CUDA columns must be addressable from the primary context of their
DLPack device ordinal. A later hierarchy computation can replace these direct
world matrices from authored local transforms.

## Test References

- [TEST-CAPI-OVSTAGE-OUTPUT-001](../../tests/capi/TEST-CAPI-OVSTAGE-OUTPUT-001.md)
  records coverage and outstanding validation.

## Code References

- ovphysx/include/ovphysx/experimental/OvStageOutput.hpp
  (`OvStageOutputCache`, `OvStageOutputResult`, `writeWorldTransformsToOvstage`;
  AC-1 through AC-6)
- ovphysx/src/ovphysx/OvStageOutput.cpp (selection, scale capture, cache,
  publication and ownership; AC-1 through AC-6)
- ovphysx/src/ovphysx/OvStageOutputCuda.cu (device matrix processing; AC-2, AC-5)
- ovphysx/src/ovphysx/OvStageOutputMath.h (shared CPU/CUDA matrix composition;
  AC-2, AC-6)
- ovphysx/ovruntime/source/common/source/foundation/MatrixTools.cpp
  (`omni::physx::decomposeMatrix`; canonical scale convention used by AC-2)
