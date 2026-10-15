<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PROPERTY-GENERIC-001
title: Preserve generic physics property models and schema contributions
status: proposed
owner: physics-ux
---

## Description

Preserve generic physics property models and schema contributions. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Schema models.** The property registry shall retain rigid body, collision/mesh approximation, mass/material, joint/drive/limit, articulation/root and simulation-owner fields in addition to specialized widgets. Display inherited/default/authored and mixed-selection values without authoring on rebuild.
- AC-2: **Applicability.** Respect API conflicts, multiple-apply instance names, relationship target constraints, units, read-only layers and inactive providers. Add/remove/reset and custom-property builders shall edit only the selected schema/instance through an undoable EditSession transaction.
- AC-3: **Registration.** Stable parent-schema/widget ordering, context menus, schema aliases and external builders shall register/unregister once. Provider/selection replacement shall release listeners/models without removing other providers' contributions.

## Test References

Specification only — not implemented.

- [TEST-UX-PROPERTY-GENERIC-001](../../tests/features/TEST-UX-PROPERTY-GENERIC-001.md)

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0020-scene-properties](../../adr/ADR-0020-scene-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
