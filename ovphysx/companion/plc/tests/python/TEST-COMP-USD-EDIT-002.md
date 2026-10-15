<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-USD-EDIT-002
maps_to: REQ-COMP-USD-EDIT-001
type: integration
---

## Scenario

Publish a multi-mesh USD batch and exercise validation, write failures, undo conflicts and host scheduling.

This automated scenario covers the Python subset only; it does not claim the
full native/GPU/Kit obligation. See the [requirement scope](../../requirements/python/REQ-COMP-USD-EDIT-001.md).

## Given

- A stage with two polygon meshes, explicit root/session/local output layers, a host notice guard and fault-injectable USD Set calls.

## When

- Apply valid frames, inject failures on the second mesh, reject invalid input, undo/redo, try unrelated layer changes and invoke from another thread.

## Then

- Default and sampled output land in the supplied layer, respecting stage-to-layer time conversion and restoring the previous edit target (REQ AC-1).
- Invalid arrays, duplicate destinations, stale sources, foreign layers and nonwritable layers fail before edits (REQ AC-2).
- A failed Set rolls back the whole batch; undo refuses unrelated changes and a failed host guard returns recoverable original layer content (REQ AC-3).
- Cross-thread and reentrant calls fail; the headless host guard suppresses input feedback and commands can call patch undo/redo (REQ AC-4).

## Executable coverage

[tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py):

- `test_batch_output_preserves_rest_topology_and_explicit_time`
- `test_stage_time_converts_to_local_layer_time`
- `test_output_preflight_is_whole_batch`
- `test_failed_application_rolls_back_and_restores_edit_target`
- `test_false_usd_set_rolls_back`
- `test_thread_layer_and_undo_conflict_checks`
- `test_guard_failure_returns_recoverable_layer`
- `test_output_notice_guard_and_reentrant_edits`

Run the separate customer-USD invocation in the [sample instructions](../../../samples/README.md).
