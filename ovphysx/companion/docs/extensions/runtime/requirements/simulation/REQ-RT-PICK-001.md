<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-PICK-001
title: Resolve and retain ray interaction within the play session
status: proposed
owner: physics-runtime
---

## Description

Resolve and retain ray interaction within the play session. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Ray query.** Only Running accepts a new press. Validate finite origin/nonzero finite direction, normalize direction, raycast with legacy maximum distance 1000000 stage units, and select the closest eligible hit with collision/visibility filtering; non-dynamic, kinematic or simulation-disabled targets receive no impulse.
- AC-2: **Identity.** Resolve opaque hit identities with the public ovphysx path facility, copy borrowed hit/path data, preserve point-instancer identity and bind the selected object to the current simulation generation; stale or removed targets shall cancel interaction.
- AC-3: **Command lifetime.** Press selects mode/settings and captures local point/distance; held grab input updates the target on the same body, while push may repick. Retain only the latest repeating command; apply forces only on due substeps and process release without needing another step.
- AC-4: **Termination.** Release, pause, Stop, detach, target removal and gate closure shall cancel pending interaction, restore temporary damping, release adapter-owned targets/constraints, wake a valid target and remove feedback; a subsequent grab requires a new press.
- AC-5: **Mode availability.** Before accepting a drag, the runtime shall expose whether the complete chosen mode works on that object/execution mode. Incomplete DirectGPU force parity or unavailable joint/deformable operations shall report unavailable without loading a stopped stage or silently substituting another mode.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-PICK-001](../../tests/simulation/TEST-RT-PICK-001.md) — Ray selection resolves eligible objects and copies transient identity.
- [TEST-RT-PICK-002](../../tests/simulation/TEST-RT-PICK-002.md) — Held input follows one picked body and termination clears transient state.
- [TEST-RT-PICK-003](../../tests/simulation/TEST-RT-PICK-003.md) — Unavailable interaction modes fail explicitly without fallback.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

## Dependencies

- [ADR-0009](../../adr/ADR-0009-raycast-and-mouse-forces.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
- [REQ-RT-DETACH-001](REQ-RT-DETACH-001.md).
