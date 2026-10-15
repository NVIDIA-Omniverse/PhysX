<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVD-NAMES-001
maps_to: REQ-CAPI-OVD-NAMES-001
type: integration
---

## Scenario

Canonical and compatibility consumers preserve the recording contract.

## Given

- Current public headers/bindings and a matching native library.
- Retired startup-config probes and canonical/forwarding recording callers.

## When

- Run `tests/python_tests/cpu_tests/test_public_ovd_api.py` through the CPU pytest runner and run the native OVD recording tests.
- Compile/link canonical and old recording calls and exercise FILE/TCP lifecycle tests.
- On a capable runtime with an attached scene, run one recording session per
  export spelling: start and stop through that spelling, and query the active
  and stopped states through both spellings. Run the existing restart case.
- Start FILE recording before any simulation step, execute exactly one step,
  stop, and parse the finalized command stream for its frame boundaries.

## Then

- The canonical handle preserves representation and callback types (AC-1).
- Native names link; old and canonical exports return equivalent invalid-handle results and observe the same successful capable-runtime lifecycle; Python method names remain available (AC-2).
- Configuration rejects all retired OVD startup fields, including the old capability field; the legacy OVD header/module are absent. Raw/typed retired keys return `INVALID_ARGUMENT` with migration guidance, without reusing their slots or renumbering unrelated settings (AC-3).
- Capable instances start, stop, and restart; incapable instances reject start. The recording started before any step completes the first simulation step's pre/post frames 1 and 2 and opens frame 3 for the next step (start-frame IDs `[1, 2, 3]`, stop-frame IDs `[1, 2]`), proving first-frame capture without a warmup step (AC-4).
