<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PICK-002
maps_to: REQ-RT-PICK-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Held input follows one picked body and termination clears transient state. Verifies REQ-RT-PICK-001 AC-2, AC-3, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running force-grab session with known original damping and a second body behind the first; input queue and force spies.

## When

- Press, change the ray so another body is nearer, deliver several held updates including one with zero due substeps, then terminate.
- Parameterize termination as release, Pause, Stop, detach, target removal and gate closure; send another held event without a new press.

## Then

- Removal or an obsolete generation never dereferences the selected actor. (REQ-RT-PICK-001 AC-2).
- Grab stays on the originally selected body and uses only the latest target; zero-due updates apply no force, and release completes without another step. (REQ-RT-PICK-001 AC-3).
- Every termination restores saved damping, wakes a still-valid target, clears commands/feedback and releases owned temporary objects. No grab resumes without a new press. (REQ-RT-PICK-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-PICK-001](../../requirements/simulation/REQ-RT-PICK-001.md).
