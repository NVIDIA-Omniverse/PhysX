<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0006 - Host UX over optional runtime capabilities
status: accepted
date: 2026-09-30
last_updated: 2026-10-01
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Legacy UX extensions mix presentation, commands, solver queries, controller input and internal test surfaces. Migrating by copying interface names would expose unrelated responsibilities.

## Decision

1. Keep widgets, selection, graph registration, input devices, timeline and command/undo in the host adapter.
2. Reuse core query/read/write/debug/recording operations where semantics match; add narrow optional events, inspection, interaction and controller services where needed.
3. Default mouse grab to managed force picking. Companion native code retains the picked local point and initial ray distance and runs the legacy mass-scaled impulse, point-velocity damping and gravity assistance once per actual scene substep. Allow zero and live changes to the per-grab force coefficient; establish angular damping once on press and restore it on cancellation. D6 joint drag remains an explicitly selected alternative; preserve the original C interaction ABI with a separate versioned force-picking table.
4. Keep vehicle/CCT behavior native where necessary; cameras ordinarily consume supported state. Gate each backend/mode explicitly.
5. Classify validation as USD-only, cooking-based or simulation-dependent; return diagnostics and let Python apply repairs.
6. Maintain interface/consumer coverage with an explicit non-export category for unit-test hooks, empty identities and unused interfaces. Preserve helper compatibility deliberately rather than inferring parity from matching names.

The host selects and filters hits, supplies pointer targets, and closes grabs on release, Pause/Stop, focus loss or input disable. It never implements the force law in Python or creates a second simulation loop. Force grabs use the same exact attachment/instance lifetime as joint drags.

**Out of scope**

- A Kit dependency in companion native or shared Python services.
- Automatic replication/clone or Fabric/ovstage parity without semantic validation.

## Alternatives considered

- Publish every legacy IPhysx table: exports tests, widgets and missing providers as if they were supported services.
- Move all controllers to body writes: cannot reproduce native CCT or specialized vehicle behavior.
- Treat a surviving header as capability evidence: does not establish a working implementation or backend.

## Consequences

- **Positive:** The host retains its UX model while headless clients can consume the useful physics services.
- **Costs:** Implement adapters, capability diagnostics, thread/phase-aware event delivery and per-consumer migration fixtures.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

## References

- [REQ-COMP-QUERY-001](../requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-EVENTS-001](../requirements/live/REQ-COMP-EVENTS-001.md)
- [REQ-COMP-DEBUG-001](../requirements/live/REQ-COMP-DEBUG-001.md)
- [REQ-COMP-INTERACTION-001](../requirements/live/REQ-COMP-INTERACTION-001.md)
- [REQ-COMP-VEHICLE-001](../requirements/controllers/REQ-COMP-VEHICLE-001.md)
- [REQ-COMP-CCT-001](../requirements/controllers/REQ-COMP-CCT-001.md)
- [REQ-COMP-CAMERA-001](../requirements/controllers/REQ-COMP-CAMERA-001.md)
- [REQ-COMP-VALIDATE-001](../requirements/authoring/REQ-COMP-VALIDATE-001.md)
- [REQ-COMP-HOST-001](../requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-MIGRATION-001](../requirements/host/REQ-COMP-MIGRATION-001.md)

## Controller and query boundary clarification (2026-10-01)

Companion CCT controls and scene queries are deliberate optional attachment services for legacy workflows, not temporary promises to move them into generic core writes or a separate library. Reuse core operations where their semantics match; future providers can replace these adapters without adding them to the main ovphysx surface.
