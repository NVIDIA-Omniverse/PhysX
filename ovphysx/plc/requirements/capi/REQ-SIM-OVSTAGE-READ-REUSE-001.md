<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-SIM-OVSTAGE-READ-REUSE-001
title: Reuse mapped OVStage values during physics updates
status: implemented
owner: ovphysx
---

## Description

The OVStage change feed reuses fixed-size CPU values already returned by a read,
including groups with a data index map. Mapping the logical prim order onto the
transported tensor rows must not require a new query and read for each body.
Scalar source getters also reuse mapped and masked bucket rows within the
existing bucket lifetime. No public API or cross-update value cache is added.

## Acceptance Criteria

- AC-1: Supported fixed-width mapped CPU groups are gathered in logical key
  order before bulk dispatch. Prim-list mapping and data-row mapping are
  independent; repeated data rows and tensor byte offsets remain correct.
- AC-2: Scalar getters served by a mapped bucket return the selected row without
  a live attribute read. Both implicit and explicit compact strides are accepted.
- AC-3: A scalar bucket's presence mask uses 64-bit words. Present rows are
  decoded; missing rows return an authoritative absence without a live read.
  Replacing or clearing the bucket ends the previous values' lifetime.
- AC-4: Unsupported or invalid layouts retain the existing live-read fallback.
  Every mapped row is validated before extending a coalesced column. Array,
  device, and transform lifetime contracts remain unchanged; masked feed batches
  retain per-group dispatch so missing-value callbacks are preserved.
- AC-5: Reordered and subset velocity edits across successive sealed drains
  update the corresponding physics objects while preserving untouched objects.

- AC-6: Ragged CPU array groups accept explicit compact strides as well as
  implicit compact layout; malformed and non-compact layouts still fall back.
  Empty nonnegative shapes require no element access.

## Test References

- TEST-SIM-OVSTAGE-READ-REUSE-001

## Code References

- ovphysx/ovruntime/source/omni.physics.ovstage/OvstageChangeFeed.cpp
- ovphysx/ovruntime/source/omni.physics.ovstage/OvstageSource.cpp
- ovphysx/ovruntime/source/omni.physics.ovstage/ReadGroupUtils.h

## Dependencies

- OVStage fixed-size read-group shape, index-map, mask, and ownership contracts.
