<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-OVSTAGE-001
maps_to: REQ-COMP-OVSTAGE-001
type: integration
---

## Coverage status

Implemented and executed with separate native and customer-USD processes. Covers [REQ-COMP-OVSTAGE-001](../../requirements/exchange/REQ-COMP-OVSTAGE-001.md) AC-1 through AC-4.

## Scenario

Run a bidirectional host loop with an authored input edit and a simulation output frame.

## Given

- An attached ovstage, a host input-update counter, a USD mirror notice adapter and a separate authoring snapshot.
- Separate input/output ordinals and one declared writer per destination.

## When

- Apply one authored edit, step/read, explicitly publish ovstage and USD output, then process host notices and start a snapshot computation.

## Then

- The authored edit produces exactly one explicit input update and the stage remains alive through detach (REQ AC-1).
- Output changes appear only after explicit publication, and the output ordinal is never submitted as input (REQ AC-2).
- The snapshot computation leaves the live attachment unchanged (REQ AC-3).
- Output-generated USD notices cause zero new solver input updates (REQ AC-4).

## Executable coverage

`test_population_nested_transforms_preserved_by_live_and_preview` in
[preview integration](../../../tests_native/test_preview.py) verifies composed
parent/body world matrices before attachment and body poses in both playback and
independent preview (AC-1).
Population uses ALL domains with OVStage 0.3; physics snapshot filtering is unchanged.

[Native integration](../../../tests_native/test_exchange.py), all five cases pass:

- `test_core_authored_input_explicit_output_and_snapshot`: one authored update, immutable copied output, unchanged ovstage world matrix before publication, skipped output mass sentinel, later authored input and snapshot mass computation preserving live owner/step (AC-1/2/3).
- `test_preview_flow_owner_election_sealing_and_retirement`: independent preview, exact-stage validation, one writer, required sealing/consumption, thread affinity and owner retirement (AC-1/2).
- `test_exchange_preflight_and_failed_publication_latch`: preflight leaves cursor unchanged; failed publication latches rather than risking replay (AC-2).
- `test_explicit_flow_sample`: runs the stageupdate sample (AC-1/2/3).
- `test_ragged_output_explicit_lanes_and_cpu_copy`: independent array rows and explicit vector lanes, no implicit aliasing or input updates (AC-2/3).

[Customer-USD integration](../../../tests/test_exchange.py):
`test_independent_usd_output_notice_suppression_and_failure` passes, including
normal authored notices, output/undo/redo suppression and injected rollback,
without importing ovstage/native USD (AC-3/4).

Full ovphysx `validate_all.cmake` passed all six groups in 474.13 seconds after
this change, including the native and customer-USD companion test directories.


Round-two output tests reject physics mass, points, transform and drive inputs
before publication. Native exchange publishes `companion:output:mass=100`, then changes
density on that same body's input prim and verifies effective mass remains 7.

Round-three output tests also reject custom-physics inputs (`companion:stiffness`,
`companion:radius`) and the bare `companion:output` prefix as output names, and
`test_custom_physics_inputs_cannot_declare_output_namespace` rejects an
ExternalSchema parameter in `companion:output:` (AC-2).
[Native plugins](../../../tests_native/test_plugins.py)
`test_flow_output_never_becomes_custom_joint_input` publishes
`companion:output:stiffness=250` for the spring sample on the core playback
owner at output ordinal 2, consumes an unrelated body-density edit at ordinal 3,
then refreshes the joint and verifies it still reads the authored stiffness 125
(AC-2).
The `companion_bad_OUTPUT_PARAM` fixture declares a native plugin parameter in
`companion:output:`; `test_incompatible_plugin_never_starts[OUTPUT_PARAM]`
verifies registration fails with INVALID_ARGUMENT before the plugin starts (AC-2).
