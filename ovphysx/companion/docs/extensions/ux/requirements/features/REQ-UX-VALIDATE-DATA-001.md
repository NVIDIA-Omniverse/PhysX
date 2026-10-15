<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-VALIDATE-DATA-001
title: Retain static physics asset checks and selected repairs
status: proposed
owner: physics-ux
---

## Description

Retain static physics asset checks and selected repairs. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Rule inventory.** Expose separately identified API-conflict, articulation-joint, joint-pose, mass, simulation-owner, deformable-schema and collision-approximation checks with path, severity and stable diagnostic code. Declare each check as USD-only, cooking or simulation; running USD-only checks creates no native attachment.
- AC-2: **Repairs.** Present deterministic repair proposals without editing during validation. Apply only user-selected fresh proposals to the active writable layer through EditSession, including exact undo; invalid/read-only/stale selections fail before partial changes.
- AC-3: **Coverage.** Retain missing rules as unavailable/incomplete checks rather than declaring the asset valid. The current companion joint-frame/mass/transform/cook checks are reusable primitives, not evidence that the complete legacy validator catalog has migrated.

## Test References

Specification only — not implemented.

- [TEST-UX-VALIDATE-DATA-001](../../tests/features/TEST-UX-VALIDATE-DATA-001.md)

## Code References

- [validation](../../../../../python/ovphysx_companion/validation.py) — existing companion substrate; host integration remains proposed.
- [usd/validation](../../../../../python/ovphysx_companion/usd/validation.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0027-authoring-runtime-provider](../../adr/ADR-0027-authoring-runtime-provider.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
