<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-UPDATE-001
title: Monotonic OVStage Update API
status: implemented
owner: ovphysx
---

## Description

`ovphysx_update_from_ovstage()` shall expose monotonic consumption of
caller-owned OVStage ordinals. The ordinal parsed by attachment and every
successfully drained ordinal are consumed once; repeating them does not repeat
physics changes or user callbacks.

## Acceptance Criteria

- AC-1: A valid update range containing only consumed ordinals returns
  `OVPHYSX_API_SUCCESS` as a no-op and emits no object-change callbacks.
- AC-2: A valid range overlapping consumed and unread ordinals applies only its
  unread suffix. Post-attach population remains observable, and replaying the
  successfully consumed ordinal does not deliver its callbacks again.

## Test References

- TEST-CAPI-OVSTAGE-UPDATE-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_update_from_ovstage` contract)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_update_from_ovstage` forwarding)
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp (`InitialPopulationDeliversNoCreatedCallbacks`)

## Dependencies

- REQ-SIM-OVSTAGE-UPDATE-001
