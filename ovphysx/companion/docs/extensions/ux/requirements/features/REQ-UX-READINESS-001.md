<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-READINESS-001
title: Preserve capability checks and safe physics variant selection
status: proposed
owner: physics-ux
---

## Description

Preserve capability checks and safe physics variant selection. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Capability model.** Readiness shall compare composed stage schemas/features against declared engine and companion operation capabilities, distinguishing unsupported schema, absent service and unqualified backend. Opening the capability UI must not initialize any candidate engine.
- AC-2: **Variant selection.** Propose compatible variant-set choices with reasons and preserve authored selection until the user explicitly applies a valid choice. Apply through customer USD with undo, invalidate source/preview results and never silently change runtime owner or enable legacy extensions.
- AC-3: **Refresh.** Stage open/close, schema/variant edits and provider registration shall refresh the current result once, discard old-stage work and unregister notice/rule callbacks on shutdown; unknown capabilities cannot be treated as supported.

## Test References

Specification only — not implemented.

- [TEST-UX-READINESS-001](../../tests/features/TEST-UX-READINESS-001.md)

## Code References

- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.
- [schemas](../../../../../python/ovphysx_companion/schemas.py) — existing companion substrate; host integration remains proposed.
- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0027-authoring-runtime-provider](../../adr/ADR-0027-authoring-runtime-provider.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
