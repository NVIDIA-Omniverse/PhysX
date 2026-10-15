<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-STEP-001
title: Advance one scene with fixed substeps and bounded catch-up
status: proposed
owner: physics-runtime
---

## Description

Advance one scene with fixed substeps and bounded catch-up. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Single rate.** Assuming exactly one physics scene, the adapter shall resolve its effective rate through Umbrella using authored PhysX, authored Newton fallback, then recognized-scene 60 Hz default; invalid/zero rate shall fail Play. Rate and minimum-frame-rate changes take effect only in a new play epoch.
- AC-2: **Accumulator.** For finite positive elapsed dt in Running with enableUpdate=true, use h=1/f, available=r+dt and n=floor(available/h). With minimum frame rate m>0, cap=max(1,floor(f/m)), n=min(n,cap), and rNext=min(available-n*h,cap*h); otherwise rNext=available-n*h. Execute exactly n intervals h and retain rNext.
- AC-3: **Nonadvancing updates.** Stopped, Paused, disabled, zero-elapsed and invalid-time updates shall perform no solver step or output. Negative/nonfinite elapsed or nonfinite currentTime shall not modify the remainder. Disabled updates shall not service editor input/cooking/queries or accumulate catch-up debt; cancellation may still clean up an existing interaction.
- AC-4: **Substep order.** Each due substep shall consume its eligible controls, deliver pre-step callbacks once, apply ray forces for h, step once, wait, and publish completed output. Backend post-step callbacks shall not be duplicated and shall not substitute for the presentation fence.
- AC-5: **Clock and sampling.** The completed solver clock/count shall advance only for successful h intervals; dropped wall time shall not advance it. Host samples shall be ordered at currentTime-(n-1-i)*h for i=0..n-1. Seek/rewind shall require Stop and a new epoch.
- AC-6: **Remainder lifetime.** Pause/resume shall preserve remainder; Stop and a fresh Play shall clear it. m=0 shall not acquire an undocumented work cap, and finite large elapsed values shall not overflow integer step-count calculations.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-STEP-001](../../tests/simulation/TEST-RT-STEP-001.md) — Resolve and freeze the single-scene timestep.
- [TEST-RT-STEP-002](../../tests/simulation/TEST-RT-STEP-002.md) — Fractional time and capped backlog reproduce the legacy formula.
- [TEST-RT-STEP-003](../../tests/simulation/TEST-RT-STEP-003.md) — Invalid or non-simulating updates do not maintain an editor backend.
- [TEST-RT-STEP-004](../../tests/simulation/TEST-RT-STEP-004.md) — Each substep has one callback/force/step sequence and a distinct sample time.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [test_step_boundary_conditions.py](../../../../../../tests/python_tests/cpu_tests/test_step_boundary_conditions.py) — NaN dt rejection and synchronous/asynchronous step boundary cases (current API reference).

## Dependencies

- [ADR-0005](../../adr/ADR-0005-simulation-update-and-substepping.md).
- [REQ-RT-DRIVER-001](REQ-RT-DRIVER-001.md).
