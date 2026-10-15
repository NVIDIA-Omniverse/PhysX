<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEBUG-001
maps_to: REQ-COMP-DEBUG-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-DEBUG-001; executable evidence and qualification limits follow.

## Scenario

Compare a retained live debug frame with a stopped collision overlay and capability-gated exports.

## Given

- A collider scene, known debug draw settings and a retained frame lease.
- A stopped authoring snapshot, statistics version request and one unavailable export feature.

## When

- Read a debug frame/counters, advance the simulation, read the retained frame and produce the stopped overlay; request unavailable export.

## Then

- Retained geometry remains coherent for its original step with documented colors/frame/culling (REQ AC-1).
- Counters have documented scope/units and no mutable internal pointer escapes (REQ AC-2).
- The stopped overlay is generated without attach and represents the cooked collider (REQ AC-3).
- The absent export reports unsupported without changing recording state (REQ AC-4).

## Executable coverage

[test_debug.py](../../../tests_native/test_debug.py):

Completed copied frames, owner identity, canaries/capacity, pending admission and retirement pass. Stopped cooked overlays verify transformed geometry, normals and per-shape color without attach. RepX/legacy-PVD requests fail through separate availability discovery. Statistics are exercised by test_bridge.py.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
