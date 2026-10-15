<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-METRICS-001
title: Preserve physics unit conversion and notice ownership
status: proposed
owner: physics-ux
---

## Description

Preserve physics unit conversion and notice ownership. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Dimensions.** Metrics rules shall use distance/mass exponents including schema-dependent fields, convert source to destination stage units and preserve dimensionless values. Missing conditional rules report an explicit unresolved attribute rather than guessing a scale.
- AC-2: **Atomic conversion.** Conversion shall run on customer USD through a bounded transaction, temporarily suppressing owned expensive overlay refresh and restoring its prior state on success/failure. Authored conversions still emit one logical input invalidation; they are not simulation output to suppress indefinitely.
- AC-3: **Lifecycle.** Nested/concurrent stage conversions keep independent stage identities and restoration state. Unregister removes all custom rules/callbacks; an exception or stage close cannot leave joint overlays or notices globally disabled.

## Test References

Specification only — not implemented.

- [TEST-UX-METRICS-001](../../tests/features/TEST-UX-METRICS-001.md)

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [exchange](../../../../../python/ovphysx_companion/exchange.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0026-authoring-stage-lifecycle](../../adr/ADR-0026-authoring-stage-lifecycle.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
