<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-COMMANDS-001
title: Preserve registered command behavior through companion transactions
status: proposed
owner: physics-ux
---

## Description

Preserve registered command behavior through companion transactions. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Command identity.** The host shall catalogue every registered legacy command, including UI, vehicle and CCT command modules, with its ID, arguments/defaults, selection/edit-target behavior and result. Retained aliases shall delegate to reviewed Python helpers; renamed or unsupported commands shall return an explicit migration error before editing.
- AC-2: **Transactions.** Apply/unapply typed and codeless APIs, component add/remove, multiple-apply instances, attributes, relationships, metadata, materials, pairs and joints shall preserve authored absence, list operations and unrelated opinions under grouped undo/redo. Multi-selection preflight shall prevent partial mutation on invalid/read-only targets.
- AC-3: **Lifetime.** Async cook/create commands shall capture source revision, suppress stale/cancelled publication and return one host undo patch on success. Reload shall register each alias once and release only its own commands; native callbacks shall never call Kit commands.

## Test References

Specification only — not implemented.

- [TEST-UX-COMMANDS-001](../../tests/features/TEST-UX-COMMANDS-001.md)

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0001-physics-menu](../../adr/ADR-0001-physics-menu.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
