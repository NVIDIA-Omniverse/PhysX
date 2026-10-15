<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-ATTACH-USD-002
maps_to: REQ-COMP-ATTACH-USD-001
type: integration
---

## Scenario

Author a vertex-to-vertex surface attachment from captured endpoints and computed-pair fixture data.

This automated scenario covers the Python subset only; it does not claim the
full native/GPU/Kit obligation. See the [requirement scope](../../requirements/python/REQ-COMP-ATTACH-USD-001.md).

## Given

- Registered codeless schemas, two surface simulation polygon meshes and a selected local layer.

## When

- Capture endpoints, apply vertex pairs, undo/redo, add unrelated children/properties, repeat the update and try stale/invalid/unowned destinations.

## Then

- Extraction returns points, topology, source revision, transform and length units without USD objects in the record (REQ AC-1).
- The typed attachment prim contains the expected endpoint relationships and index arrays (REQ AC-2).
- A repeated update is idempotent and preserves unrelated children/properties; unowned destinations are rejected (REQ AC-3).
- Stale/deleted/recreated endpoints and invalid indices fail without partial edits; the patch supports undo/redo (REQ AC-4).

## Executable coverage

[tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py):

- `test_capture_contains_only_source_data`
- `test_attachment_apply_update_and_undo_preserve_unowned_content`
- `test_attachment_preflight_and_ownership`
- `test_stale_results_fail_without_edits`
- `test_extension_samples_run`
- `test_extension_sample_cli`

Run the separate customer-USD invocation in the [sample instructions](../../../samples/README.md).

- [Element and TetMesh cases](../../../tests/test_attachment_elements.py) additionally cover typed triangle/tetrahedron coordinates, collision groups, source invalidation and managed disabling with user children preserved.
