<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEFORM-WRITE-002
maps_to: REQ-COMP-DEFORM-WRITE-001
type: integration
---

## Scenario

Publish ready destination-space CPU frames to preview/bake layers with stale-data and host-notice checks.

This automated scenario covers the Python subset only; it does not claim the
full native/GPU/Kit obligation. See the [requirement scope](../../requirements/python/REQ-COMP-DEFORM-WRITE-001.md).

## Given

- Static polygon destinations, known mapped CPU arrays, explicit preview/bake layers and a headless solver-input notice observer.

## When

- Write complete batches, undo/redo preview, bake two frames, reject invalid arrays and topology changes, and run the extension sample.

## Then

- The destination identity/count/topology is checked and unsupported frame/unit declarations fail before mutation (REQ AC-1).
- Only explicit ready CPU arrays are accepted, with one vectorized USD array publication per mesh/property; implicit device conversion is never attempted (REQ AC-2).
- Output carries one step/time per batch and writes only points/optional velocities at explicit stage time, preserving rest/topology opinions (REQ AC-3).
- Stale results fail before edits and the supplied headless host guard prevents output notices from becoming input updates (REQ AC-4).

## Executable coverage

[tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py):

- `test_batch_output_preserves_rest_topology_and_explicit_time`
- `test_output_preflight_is_whole_batch`
- `test_output_rejects_changed_topology_and_instances`
- `test_stage_time_converts_to_local_layer_time`
- `test_output_notice_guard_and_reentrant_edits`
- `test_extension_samples_run`
- `test_extension_sample_cli`

Run the separate customer-USD invocation in the [sample instructions](../../../samples/README.md).

Additional executed coverage: [test_deformable_authoring.py](../../../tests/test_deformable_authoring.py) checks TetMesh publication, native embedding and frame conversion; [test_output.py](../../../tests_native/test_output.py) checks all ragged prims, owned copies and explicit transfer consent using a controlled read transport. Actual GPU extraction remains unverified.
