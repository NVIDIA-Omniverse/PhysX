<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEFORM-WRITE-001
maps_to: REQ-COMP-DEFORM-WRITE-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-DEFORM-WRITE-001; executable evidence and qualification limits follow.

## Scenario

Preview and bake two deformable frames with a visual mesh denser than the simulation mesh.

## Given

- Known simulation motion, a verified embedding and visual-frame transform, and saved rest/topology opinions.
- A host notice counter, preview layer and bake layer; CPU and supported GPU output variants.

## When

- Publish one preview frame and two time-sampled bake frames, check mapped visual positions, then try a frame after topology changed.

## Then

- Each destination receives the expected mapped/frame-converted positions, not a same-count assumption (REQ AC-1).
- GPU reads wait/transfer correctly, USD writes are batched and an ovstage-only run avoids forced host transfer (REQ AC-2).
- Preview/bake land at the chosen layer/time and saved rest/bind/topology opinions remain unchanged (REQ AC-3).
- The stale frame is rejected and writeback-generated notices cause no solver input update (REQ AC-4).

## Executable coverage

[test_deformable_authoring.py](../../../tests/test_deformable_authoring.py):

Native mapping, transformed visual output, triangle normal-derivative velocities and TetMesh publication are exercised. tests/test_usd_helpers.py covers preview/bake time mapping, atomic batches, unchanged rest/topology and stale/notice rejection; test_exchange.py verifies no input feedback. tests_native/test_output.py verifies ragged readiness/explicit device transfer with a device transport fixture, not a real GPU simulation.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

Completed output capture uses the core `read_sync` convenience: the internal query,
session and all group fetches complete before copied body/deformable data is exposed.
