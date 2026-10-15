<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEFORM-WRITEBACK-001
title: Publish completed deformable results through customer USD
status: proposed
owner: physics-ux
---

## Description

Publish completed deformable results through customer USD. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Readback.** The adapter shall request completed simulation position/velocity data from the exact owner/step and explicitly synchronize/copy device data to owned CPU buffers before Python access. It shall retain simulation/collision/visual domains, topology and mapping identity; unavailable GPU extraction is a parity gap.
- AC-2: **Mapping.** Map points/velocities through the matching triangle/tet embedding with declared local/world frame and length/time units; do not conflate element index domains or reuse mapping after topology changes. Reject mismatched owner, step or source before writing.
- AC-3: **Destination.** Use usd.writeback with customer-installed USD, an explicit writable preview/bake layer and numeric time. One writer owns that destination, output notices do not feed authored input, stale results author nothing and undo/cancel restores only owned output opinions.

## Test References

Specification only — not implemented.

- [TEST-UX-DEFORM-WRITEBACK-001](../../tests/features/TEST-UX-DEFORM-WRITEBACK-001.md)

## Code References

- [copied output](../../../../../python/ovphysx_companion/output.py) — existing companion substrate; host integration remains proposed.
- [mesh](../../../../../python/ovphysx_companion/mesh.py) — existing companion substrate; host integration remains proposed.
- [usd/writeback](../../../../../python/ovphysx_companion/usd/writeback.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0021-deformable-properties](../../adr/ADR-0021-deformable-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
