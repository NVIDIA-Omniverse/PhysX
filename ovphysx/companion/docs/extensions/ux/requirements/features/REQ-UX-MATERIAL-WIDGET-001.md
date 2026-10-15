<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MATERIAL-WIDGET-001
title: Apply compliant-contact dependencies only on user edits
status: proposed
owner: physics-ux
---

## Description

Apply compliant-contact dependencies only on user edits. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Read-only display.** PhysX friction/restitution extensions and compliant controls shall coexist with shared USD material fields; opening/rebuilding or showing an inactive engine shall not mutate authored values.
- AC-2: **Zero-stiffness edit.** A deliberate compliant-stiffness transition to zero shall set damping to zero and acceleration-spring false in the same undo command; one undo shall restore all three previous values.
- AC-3: **Input validation.** Nonfinite/schema-invalid inputs shall be rejected before authoring; dependency enablement shall accurately reflect effective stiffness while preserving values until a deliberate edit.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MATERIAL-WIDGET-001](../../tests/features/TEST-UX-MATERIAL-WIDGET-001.md) — Inspect inconsistent compliant data without rewriting it.
- [TEST-UX-MATERIAL-WIDGET-002](../../tests/features/TEST-UX-MATERIAL-WIDGET-002.md) — Undo zero-stiffness dependency edits atomically.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0024](../../adr/ADR-0024-material-properties.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
