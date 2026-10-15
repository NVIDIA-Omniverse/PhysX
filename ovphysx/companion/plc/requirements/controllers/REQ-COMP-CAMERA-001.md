<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-CAMERA-001
title: "Camera behavior as a state consumer"
status: implemented
owner: ovphysx
---

## Description

Follow and drone camera behavior consumes supported pose/velocity reads; camera USD edits and input policy stay in Python or the host.

## Acceptance Criteria

- AC-1: **Read contract.** Camera controllers consume identified completed body pose/velocity data with explicit units/frame conversion and do not acquire raw actor pointers for ordinary follow behavior.
- AC-2: **Authoring.** Camera pose output uses the caller-selected destination and edit target through Python or host USD code; native simulation remains USD-free.
- AC-3: **Restoration.** Disable, target deletion and stage close stop updates and restore captured camera opinions when the selected mode promises restoration.
- AC-4: **Missing state.** Unavailable targets or stale frames produce a documented hold/reset/disable outcome and never follow another object because its key was reused.

## Implementation scope

`capture_body_output` reads and copies completed core body columns with explicit
attachment/path/source revision, world frame and length units. Follow behavior
uses body-local meter offsets and world velocity look-ahead; drone behavior consumes
host-provided local velocity and yaw/pitch input. Repeat/older steps hold the prior
pose; missing/replaced targets disable following. Customer-USD output converts to
the selected camera parent's frame and stage units, writes only the selected layer,
and restores it on close, target loss or edit-session close. Restoration requires
exclusive ownership of that preview layer and rejects conflicting edits. It never
overwrites an unrelated writer. The native-read/customer-USD sample uses an explicit
worker. CPU reads are exercised; device copies require explicit opt-in.

## Test References

- [TEST-COMP-CAMERA-001](../../tests/controllers/TEST-COMP-CAMERA-001.md) - executable native read, consumer and customer-USD restoration coverage.

## Code References

- [Copied body reads](../../../python/ovphysx_companion/output.py)
- [Follow and drone consumers](../../../python/ovphysx_companion/camera.py)
- [Customer-USD output](../../../python/ovphysx_companion/usd/camera.py)
- [Session cleanup callbacks](../../../python/ovphysx_companion/usd/session.py)
- [Extension flow sample](../../../samples/camera_follow.py)

Existing core read substrate:

- [ovphysx/python/ovphysx/api.py](../../../../python/ovphysx/api.py)
- [ovphysx/python/ovphysx/utils/simulation.py](../../../../python/ovphysx/utils/simulation.py)

## Dependencies

- [REQ-COMP-QUERY-001](../live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
