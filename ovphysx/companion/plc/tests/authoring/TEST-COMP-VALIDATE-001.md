<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-VALIDATE-001
maps_to: REQ-COMP-VALIDATE-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-VALIDATE-001; executable evidence and qualification limits follow.

## Scenario

Validate an asset with schema, convex-cooking and joint-state problems while another scene is playing.

## Given

- An asset containing one fixture-defined defect in each validation class.
- A live main simulation, a companion-native validation attachment on a distinct ovstage, and a saved authored-layer snapshot; a capability-absent worker/unavailable variant.

## When

- Run each validator, inspect diagnostics, then apply a selected repair and attempt a stale repair after an edit.

## Then

- Pure checks have zero runtime/attach calls and each result declares its validation class (REQ AC-1).
- All three defects have stable object-specific codes and no check edits the input (REQ AC-2).
- Native validation steps only its selected attachment and leaves playback pending work unchanged; unavailable capability chooses its explicit fallback or reports unavailable (REQ AC-3).
- Only the chosen fresh repair applies, undo restores the original opinions and the stale repair makes no edit (REQ AC-4).

## Executable coverage

[test_validation.py](../../../tests/test_validation.py):

Pure checks do not initialize PhysX; diagnostics, selected repair, undo/redo and stale rejection pass. tests_native/test_validation.py exercises malformed collision cooking and dedicated-preview motion validation while playback keeps an uncollected receipt.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

## Review regression

`test_sampled_repair_targets_captured_time_and_preserves_other_samples` captures
a non-unit joint frame at time 15, repairs and revalidates at 15, verifies the
default and neighboring samples are unchanged, then undoes the exact edit.
