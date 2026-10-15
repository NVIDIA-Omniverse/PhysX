<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-INPUT-001
title: Consume sealed controls without feeding output back
status: proposed
owner: physics-runtime
---

## Description

Consume sealed controls without feeding output back. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Control provenance.** After initial attach consumption, the adapter shall drain only new sealed application-control ordinals through ovphysx_update_from_ovstage before the affected substep. Output ordinals shall never be included, including inside a batched inclusive range.
- AC-2: **Range semantics.** Input selection shall respect inclusive ranges and endpoint-only semantics when start is omitted, never applying already consumed controls twice; unsealed input shall not reach simulation.
- AC-3: **Conversions.** Supported control values shall reach the intended object with their API-defined units/frames; authored angular velocity in degrees/second shall become runtime radians/second without converting unrelated data or kinematic surface semantics into dynamic body velocity.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-INPUT-001](../../tests/simulation/TEST-RT-INPUT-001.md) — Interleaved control and output ordinals never feed physics back into itself.
- [TEST-RT-INPUT-002](../../tests/simulation/TEST-RT-INPUT-002.md) — Control velocities retain intended identity and unit conversions.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_ovstage_drain.py](../../../../../../tests/python_tests/test_ovstage_drain.py) — test_drain_rigid_velocity and test_drain_rigid_velocity_mixed_kinematic (current API reference).
- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).

## Dependencies

- [ADR-0005](../../adr/ADR-0005-simulation-update-and-substepping.md).
- [ADR-0010](../../adr/ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
