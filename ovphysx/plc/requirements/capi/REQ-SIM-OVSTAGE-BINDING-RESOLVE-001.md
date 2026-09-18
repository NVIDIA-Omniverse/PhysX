<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-SIM-OVSTAGE-BINDING-RESOLVE-001
title: Resolve rigid-body binding paths in batches
status: implemented
owner: ovphysx
---

## Description

Rigid-body tensor binding creation resolves literal path candidates in a batch.
The OVStage source checks cold paths through an explicit path-list query and a
latest usd-path read. It does not scan the stage with a string membership filter
for each requested body, and it does not treat an interned path as a live prim.
No public API or persistent cross-drain cache is added.

## Acceptance Criteria

- AC-1: Batched matching preserves input pattern order, per-pattern no-match
  diagnostics, wildcard semantics, and deduplication by physics body identity,
  including articulation-root/root-link aliases.
- AC-2: Source matching retains the internal-database fallback for physics-only
  clones without an authored source prim.
- AC-3: One cold existence batch uses one explicit path-list query and one
  latest usd-path read. Returned live prim rows, not query membership count,
  establish existence. Duplicate paths and canonical aliases map back to every
  original input slot; invalid and absent paths return false.
- AC-4: Existence results are published and memoized only after a successful,
  complete read with valid prim-list indices. Operation or read failures do not
  publish partial cold results. A clean empty read establishes absence.
- AC-5: Existing existence-memo invalidation remains in force. After a drain
  invalidates the memo, deletion and recreation are reflected by the next batch.

## Test References

- TEST-SIM-OVSTAGE-BINDING-RESOLVE-001

## Code References

- ovphysx/ovruntime/source/omni.physx/plugins/tensors/base/BaseSimulationView.cpp
- ovphysx/ovruntime/source/omni.physics.ovstage/OvstageSource.cpp

## Dependencies

- Existing IPhysicsSource batch matching and physics-only clone fallback.
- OVStage path-list query ownership and latest usd-path read liveness contracts.
