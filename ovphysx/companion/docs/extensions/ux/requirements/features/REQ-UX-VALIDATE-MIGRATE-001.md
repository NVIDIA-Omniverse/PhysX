<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-VALIDATE-MIGRATE-001
title: Retain explicit schema migration and compatibility repairs
status: proposed
owner: physics-ux
---

## Description

Retain explicit schema migration and compatibility repairs. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Detection.** Backward-compatibility, Newton attribute migration and mimic-joint migration rules shall report exact affected API instances/properties and distinguish unsupported conversion from no issue. Detection must not switch the selected simulation backend or rewrite the stage.
- AC-2: **Conversion.** An approved migration shall preflight target codeless schemas, single-DOF/mimic relationship constraints and any unit/value conversions. Preserve reference targets and unrelated opinions, reject ambiguous conversions without mutation and publish as one reversible customer-USD transaction.
- AC-3: **Repeatability.** Re-running a successful migration shall make no additional edits; undo restores the old schema/values and redo restores the same new opinions. Selection/stage replacement invalidates pending repair proposals.

## Test References

Specification only — not implemented.

- [TEST-UX-VALIDATE-MIGRATE-001](../../tests/features/TEST-UX-VALIDATE-MIGRATE-001.md)

## Code References

- [usd/validation](../../../../../python/ovphysx_companion/usd/validation.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [schemas](../../../../../python/ovphysx_companion/schemas.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0027-authoring-runtime-provider](../../adr/ADR-0027-authoring-runtime-provider.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
