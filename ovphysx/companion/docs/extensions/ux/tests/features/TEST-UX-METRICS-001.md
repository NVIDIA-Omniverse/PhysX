<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-METRICS-001
maps_to: REQ-UX-METRICS-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve physics unit conversion and notice ownership, including failure, source replacement and exact undo/teardown.

## Given

Two stages with metersPerUnit 1 and 0.01 and mass-unit ratio 1000; attributes with distance-only, mass-only, mixed exponents and dimensionless values, plus a schema-conditional unknown field. Start with different joint-overlay states.

## When

Convert each stage using the registered exponent rules, nest a second conversion, inject an exception after preflight and close one stage during conversion.

## Then

- Each known value equals v times lengthRatio**distanceExponent times massRatio**massExponent; dimensionless values are identical. The unknown conditional field is unresolved and never guessed. (REQ AC-1).
- The committed conversion is a single undoable transaction with one logical input invalidation. Exceptions do not leave partial edits and restore the prior overlay state. (REQ AC-2).
- Each stage restores its own prior state independent of nesting. After shutdown no conversion callbacks remain and ordinary authored notices still reach the input coordinator. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-METRICS-001](../../requirements/features/REQ-UX-METRICS-001.md).
