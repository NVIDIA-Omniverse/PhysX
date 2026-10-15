<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-FORCE-001
title: Reproduce legacy mouse-force calculations per substep
status: proposed
owner: physics-runtime
---

## Description

Reproduce legacy mouse-force calculations per substep. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Push impulse.** For normalized ray u, acceleration A, mass m and substep h, push shall apply J=u*A*m*h at the hit point using impulse semantics, including its moment about COM.
- AC-2: **Grab impulse.** With k=50*mousePickingForce, retained distance d/local point, world picked point p and COM c, compute target=origin+u*d, vPoint=v+omega cross (p-c), action=target-p-0.02*vPoint and J=k*m*action*h applied at p. Delegate the retained command and native per-substep computation to companion force picking; allow a nonnegative coefficient, including zero and live per-grab changes.
- AC-3: **Damping and gravity.** Grab shall preserve original angular damping and establish max(original,0.025*initial k) once on press; later coefficient changes affect impulses only; when gravity is enabled and nonzero, add COM velocity change -g*max(0,|g|-0.1*k*d)/|g|*h, otherwise none. Termination shall restore the original damping.
- AC-4: **Application safety.** Apply once per due substep between completed steps, never once per render frame or with h applied twice. The adapter shall update the managed companion grab, without borrowing SDK pointers, computing the grab force in Python or starting a separate simulation loop. The native service shall preserve other force producers and prevent retention of the interaction acceleration.
- AC-5: **Joint grab.** When available, joint grab shall use its own temporary kinematic target/free-axis D6 with tau=0.16, D=2*m/tau, K=D*D/(4*m), linear cap=factor*m*3000 and angular cap=factor*|inertia|*3.14; factor is 0.3 with additional constraints, else 1. Set both target and kinematic pose and clean up owned resources.
- AC-6: **Deformable grab.** When available, deformable interaction shall use barycentric hit/attachment/filter semantics and a target radius of 10 stage length units, with owned attachment/filter cleanup; it shall not treat a rigid ray hit as equivalent.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-FORCE-001](../../tests/simulation/TEST-RT-FORCE-001.md) — Push applies a point impulse with the substep duration.
- [TEST-RT-FORCE-002](../../tests/simulation/TEST-RT-FORCE-002.md) — Grab includes point velocity, temporary damping and gravity assistance.
- [TEST-RT-FORCE-003](../../tests/simulation/TEST-RT-FORCE-003.md) — Interaction impulses do not persist or corrupt other force producers.
- [TEST-RT-FORCE-004](../../tests/simulation/TEST-RT-FORCE-004.md) — Joint grab uses the legacy drive constants and owns its temporary objects.
- [TEST-RT-FORCE-005](../../tests/simulation/TEST-RT-FORCE-005.md) — Deformable grab preserves barycentric attachment behavior.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

## Dependencies

- [ADR-0009](../../adr/ADR-0009-raycast-and-mouse-forces.md).
- [REQ-RT-PICK-001](REQ-RT-PICK-001.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
