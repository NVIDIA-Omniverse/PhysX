<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-ATTACH-USD-001
maps_to: REQ-COMP-ATTACH-USD-001
type: integration
---

## Coverage status

Implemented and executed. Covers AC-1 through AC-4.

## Executable coverage

Executed in [test_attachment_elements.py](../../../tests/test_attachment_elements.py):
`test_native_element_and_filter_proposals_author_idempotently_and_reject_stale`,
`test_transform_attachment_affine_anchors_managed_removal_and_stale`, and
`test_attachment_proposal_extension_sample`, alongside six existing element,
filter, topology and owned-removal cases (nine total, all passed). The sample
runs native computation in an isolated worker and authors customer USD in the
editor. Repeated commands recapture the current revision; undo invalidates old
proposals. Typed endpoints are checked before edits and unrelated content stays.

## Scenario

Create, refresh and remove generated attachments next to a user-authored attachment.

## Given

- A customer USD stage with two deformable/rigid endpoints, a writable layer and an unrelated hand-authored attachment.
- A completed native result and a second result saved before an endpoint topology edit.

## When

- Extract inputs, apply the first result twice, refresh/remove generated children, then try applying the stale result.

## Then

- Native input contains arrays/records and no pxr objects or encoded pointers (REQ AC-1).
- Authored prims contain the expected endpoint relationships and numerical mapping/filter values in the chosen layer (REQ AC-2).
- The second application creates no duplicates and cleanup preserves the user-authored attachment (REQ AC-3).
- Stale application makes no edit and valid application supports rollback/host undo (REQ AC-4).
