<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-COMMANDS-001
maps_to: REQ-UX-COMMANDS-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve registered command behavior through companion transactions, including failure, source replacement and exact undo/teardown.

## Given

A layered stage with an un-authored default, an explicit default, two multiple-apply tendon instances, a relationship list-op, two editable selected prims and one read-only prim; the command catalog provides each constructor signature.

## When

Invoke every retained catalog command by its registered alias and default/explicit arguments; undo/redo each successful invocation. Include a multi-selection operation containing the read-only prim and a collider cook held at a completion barrier; replace the stage before releasing it. Reload the command extension twice.

## Then

- Registered aliases and argument defaults match the catalog or an explicit approved migration entry; result prims and selected edit target match the command-specific fixture. (REQ AC-1).
- Layer snapshots after undo match the baseline exactly, including absent values, list-ops and the untouched tendon instance. The invalid batch has no edits and no undo entry. (REQ AC-2).
- The delayed old-stage cook produces no prim or undo entry. Successful cooking adds exactly one undo item; reload leaves one registration per owned alias and native callbacks issue zero Kit calls. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-COMMANDS-001](../../requirements/features/REQ-UX-COMMANDS-001.md).
