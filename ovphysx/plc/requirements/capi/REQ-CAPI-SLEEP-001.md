<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SLEEP-001
title: Public Sleeping-State Read Attribute
status: implemented
owner: ovphysx
---

## Description

Expose current sleeping state through the existing query/read surface, separately
from the write-only `wake` command. This enables consumers to observe sleep state
without inferring it from velocity or query membership.

## Acceptance Criteria

- AC-1: **Public vocabulary.** `OVPHYSX_ATTR_IS_SLEEPING` names `isSleeping`.
  `ovphysx_writability` classifies it READ_ONLY for rigid bodies and articulation
  links. Existing `wake` classification and write restrictions remain unchanged.
- AC-2: **Frontend transport.** The C read surface and Python `PhysX.read` /
  `read_tokens` expose the host uint8 scalar through existing group identity and
  ownership rules. 1 means asleep, 0 awake; an articulation link reports its
  articulation's state. On scenes with sleeping disabled, including DirectGPU,
  emitted values are 0; normal readiness and disabled-row omissions still apply.
  Retained Python arrays preserve their read snapshot across later state changes.

## Test References

- [TEST-CAPI-SLEEP-001](../../tests/capi/TEST-CAPI-SLEEP-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_attributes.h (`OVPHYSX_ATTR_IS_SLEEPING`)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`kWritability`, session forwarding)
- ovphysx/python/ovphysx/api.py (`PhysX.read`, `PhysX.read_tokens`)
- ovphysx/docs/read_write/readable.md (sleep-state meaning and supported modes)

## Dependencies

- REQ-PYTHON-READ-001 (existing Python array ownership)
