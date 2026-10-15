<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-DRIVER-001
title: One serialized simulation owner behind the generic stage hooks
status: proposed
owner: physics-runtime
---

## Description

One serialized simulation owner behind the generic stage hooks. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Sole driver.** The runtime provider shall receive stage/timeline work only through omni.physics.stageupdate and Umbrella StageUpdateFns; enabling it shall not create an additional PhysX/Fabric stage-update node or background step loop.
- AC-2: **Supported surface.** The provider shall register attach, start/resume, enabled update, pause, Stop/onReset, detach and ray input, and leave forceLoadPhysicsFromUSD, resetSimulation and releasePhysicsObjects unregistered; unsupported requests shall not create, step or tear down simulation.
- AC-3: **Serialization.** All play-session creation, stepping, interaction, output and teardown shall execute serially; reentrant lifecycle requests shall wait for a safe completed-substep boundary and never access a destroyed generation.
- AC-4: **Gating.** Presenter/timeline gating shall prevent simulation production through the generic driver; opening that gate shall not replay missed elapsed intervals or bypass the explicit Play transition.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-DRIVER-001](../../tests/simulation/TEST-RT-DRIVER-001.md) — Generic hooks are the only producer.
- [TEST-RT-DRIVER-002](../../tests/simulation/TEST-RT-DRIVER-002.md) — Reentrant pause and presenter gating cannot create a second producer.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-ovphysx-stageupdate-lifecycle.md).
