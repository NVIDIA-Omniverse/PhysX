<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PARTICLE-ACTIONS-001
title: Preserve particle authoring and distinguish postprocess capabilities
status: proposed
owner: physics-ux
---

## Description

Preserve particle authoring and distinguish postprocess capabilities. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Authoring.** Retain add/remove particle system, particle set, sampling and PBD material commands, with targets, fluid/solid state, offsets and mass/density policy. Diffuse, anisotropy, smoothing and isosurface schemas remain separately inspectable/undoable when registered.
- AC-2: **Sampling.** Capture mesh/source revision and volume/surface sampling options, use SamplingService CPU jobs where supported and publish selected points through usd.particles. Disabling sampling stops future work without deleting unrelated authored data; topology/option changes invalidate previous results.
- AC-3: **Postprocess.** Each diffuse/smoothing/anisotropy/isosurface and live particle-output mode shall have a separately verified provider and test. Their current absence is an open parity gap: do not alias CPU Poisson sampling to these operations or show stale authored points as completed simulation output.

## Test References

Specification only — not implemented.

- [TEST-UX-PARTICLE-ACTIONS-001](../../tests/features/TEST-UX-PARTICLE-ACTIONS-001.md)

## Code References

- [particles](../../../../../python/ovphysx_companion/particles.py) — existing companion substrate; host integration remains proposed.
- [usd/particles](../../../../../python/ovphysx_companion/usd/particles.py) — existing companion substrate; host integration remains proposed.
- [cooking](../../../../../python/ovphysx_companion/cooking.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0023-particle-properties](../../adr/ADR-0023-particle-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
