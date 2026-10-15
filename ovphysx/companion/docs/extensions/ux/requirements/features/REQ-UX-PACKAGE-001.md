<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PACKAGE-001
title: Keep extension bundles and fixture dependencies explicit
status: proposed
owner: physics-ux
---

## Description

Keep extension bundles and fixture dependencies explicit. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Dependencies.** Bundles shall declare core, optional companion, codeless schemas and Kit host dependencies independently; register resources before the customer USD schema registry is initialized. Startup shall negotiate companion compatibility without loading another USD or SDK image.
- AC-2: **Examples and fixtures.** Classify demos, benchmarks, internal scenes, mini/visual/vehicle tests and separate-backend examples with explicit required capabilities and assets. Test-only APIs stay in test extensions and cannot become product dependencies merely to make a demo run.
- AC-3: **Reload.** Enable/disable/reload shall leave one provider registration and no stale graph/menu/widget/update subscriptions. Missing optional packages produce a focused error before partial startup; shutdown preserves other providers and owned authored USD data.

## Test References

Specification only — not implemented.

- [TEST-UX-PACKAGE-001](../../tests/features/TEST-UX-PACKAGE-001.md)

## Code References

- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0025-demo-browser](../../adr/ADR-0025-demo-browser.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
