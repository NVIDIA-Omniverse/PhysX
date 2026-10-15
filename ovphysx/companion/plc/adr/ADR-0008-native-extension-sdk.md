<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0008 - Native extension SDK with explicit ABI and lifetime
status: accepted
date: 2026-09-30
last_updated: 2026-10-01
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Custom joint and geometry callbacks already avoid USD types, but native SDK ABI, external parameter ingestion and object/module lifetime need a supported contract before plugins can rely on them.

## Decision

1. Expose the existing runtime custom joint and geometry interfaces directly through an optional build-matched C++ SDK surface. Preserve their callback structures and PhysX types; keep them outside ordinary core and companion headers.
2. Validate descriptor size/version, SDK build and compiler/runtime compatibility before callbacks. Resolve PhysX utilities through the existing owner or an approved utility library.
3. Return Busy from unregister while objects, result leases or callbacks remain; retain plugin code, registration and geometry type storage until release/drain completes.
4. Use real external schemas and immutable typed parameters or versioned blobs. Register schema resources separately in customer USD and the population context.
5. Keep solver/contact callbacks compiled native. Require live flag-update regression coverage for joints and exactly-once geometry destruction, including construction rollback.
6. Advertise CPU/GPU/DirectGPU and articulation limitations explicitly; generic registration does not promise specialized mesh-merge parity.

**Out of scope**

- Cross-version compatibility for arbitrary PhysX C++ binaries.
- Automatic articulation DOF/tensor integration for custom constraints.

## Alternatives considered

- Require every native caller to adopt another callback model: duplicates the existing runtime contract and obstructs migration of native extensions.
- Run solver/contact callbacks in Python: introduces interpreter/GIL behavior into hot native execution.
- Unregister immediately and trust callers: frees state while live shapes or solver callbacks can reference it.

## Consequences

- **Positive:** External native plugins gain a bounded contract over existing machinery without native USD coupling.
- **Costs:** Fix callback lifecycle, validate actual external schema population, publish samples and exercise failure/unload paths.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

### Direct native interface amendment (2026-10-01)

The project owner requires direct access to the existing custom joint and geometry
interfaces. A companion lease retains the already loaded runtime and creates no
scene or attachment. Registrations belong to that runtime; object callbacks and
live joint controls name exact attachments. Native clients own callback state and
code, retire registrations through the existing safe unregister operations, and
then release the runtime lease. The compiled-module loader is an optional
convenience; its callback adapter is not required for direct C++ use.

The ordinary core and companion headers compile without PhysX or Carbonite SDK
headers. Only explicitly included native interface/access headers require the
matching PhysX headers. Customer USD schema authoring remains in optional Python;
solver/contact callbacks remain compiled C++.

## References

- [REQ-COMP-NATIVE-ABI-001](../requirements/native/REQ-COMP-NATIVE-ABI-001.md)
- [REQ-COMP-NATIVE-SCHEMA-001](../requirements/native/REQ-COMP-NATIVE-SCHEMA-001.md)
- [REQ-COMP-JOINT-001](../requirements/native/REQ-COMP-JOINT-001.md)
- [REQ-COMP-GEOMETRY-001](../requirements/native/REQ-COMP-GEOMETRY-001.md)
