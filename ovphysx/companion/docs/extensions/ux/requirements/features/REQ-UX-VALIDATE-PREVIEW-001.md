<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-VALIDATE-PREVIEW-001
title: Run simulation-dependent validation in an isolated companion owner
status: proposed
owner: physics-ux
---

## Description

Run simulation-dependent validation in an isolated companion owner. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Admission.** Joint-state/settling and other simulation-dependent checks shall require explicit user action, declared capabilities and a copied/snapshot source. Create ValidationPreview owner B without force-loading, stepping or acquiring the playback facade A.
- AC-2: **Results.** Checks shall distinguish physical violations from Busy, unsupported backend, timeout and execution failure. Use completed state, declared tolerances and bounded steps; never treat unavailable validation as a clean asset.
- AC-3: **Retirement.** Cancel/stage replacement/close shall drain B and suppress stale publication. Results and selected repairs reach customer USD only through the initiating EditSession; teardown must leave A tasks, stage and configuration intact.

## Test References

Specification only — not implemented.

- [TEST-UX-VALIDATE-PREVIEW-001](../../tests/features/TEST-UX-VALIDATE-PREVIEW-001.md)

## Code References

- [validation](../../../../../python/ovphysx_companion/validation.py) — existing companion substrate; host integration remains proposed.
- [preview](../../../../../python/ovphysx_companion/preview.py) — existing companion substrate; host integration remains proposed.
- [usd/validation](../../../../../python/ovphysx_companion/usd/validation.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0027-authoring-runtime-provider](../../adr/ADR-0027-authoring-runtime-provider.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
