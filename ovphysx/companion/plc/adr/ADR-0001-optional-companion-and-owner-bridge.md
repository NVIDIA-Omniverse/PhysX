<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0001 - Optional companion with one runtime owner
status: accepted
date: 2026-09-30
last_updated: 2026-10-05
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

UX authoring, live inspection and native extensions need capabilities beyond the core simulation API. The runtime is linked into the core owner; loading another copy would duplicate state and invalidate cross-library ownership assumptions.

## Decision

1. Keep companion facades, bindings and utilities in a sibling component beside ovruntime. Reusable native computation remains in ovruntime; ovruntime has no upward dependency on companion headers or Python.
2. Export a dedicated size/versioned companion acquisition entry point from the existing core owner. Keep companion tables separate from the primary simulation header.
3. Bind live tools to an explicit instance and attachment generation. The companion cannot attach or step independently of that owner; snapshot services have a separate explicit lifetime.
4. Advertise services and backend support individually. Optional installation and capability absence do not prevent core simulation.

**Out of scope**

- Independent multi-world simulation and a new shared runtime distribution.
- Publishing the entire internal IPhysx interface set.

## Alternatives considered

- Put editor operations in the core API: expands the simulation contract with UI-specific lifecycles.
- Link a second static runtime into a companion library: duplicates registries, solver ownership and native object identity.
- Split the runtime into a new shared owner first: adds packaging work that is unnecessary for the initial companion.

## Consequences

- **Positive:** Core users keep a small dependency surface, while companion clients reach the same live world.
- **Costs:** Implement owner-scoped acquisition, capability descriptors, drain ordering and a package compatibility check.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

### Multiple-attachment amendment (2026-09-30)

One runtime owner means one loaded runtime/SDK service host, not one live scene.
Borrowed tools sessions still cannot attach or step independently of their
selected owner. The explicit preview service in
[Companion ADR-0007](ADR-0007-preview-ownership.md) may own a separate
native attachment through that same bridge. Its process lifetime lease and
attachment-scoped dispatch are required before advertising this capability.
This refines Decision 3 and removes independent native attachments from the
out-of-scope reading; changing the public facade's single-attach policy remains
separate work. Shared SDK settings/recording are coordinated process operations.

### Companion ownership amendment (2026-10-01)

The SDK lifecycle owns one `CompanionRuntime` coordinator. Its members own
operation/transition admission, native-borrow reentry tracking, service leases
and the typed companion handle registries, including registry locks and handle
sequences. Guards receive that coordinator explicitly. Companion synchronization
must not live in namespace globals, static guard members or thread-local globals.
Per-preview pending simulation state stays on the preview; an exclusive runtime
transition inspects the retained previews instead of maintaining a second counter.
Independent attachments can hold operation admission concurrently. A transition
rejects active operations, and same-thread internal destroy/detach nesting remains
valid. Explicit service close remains the runtime/code retirement boundary.

### Public facade amendment (2026-10-05)

Public core instances now own independent attachments under
[REQ-CAPI-ATTACH-OWNER-001](../../../plc/requirements/capi/REQ-CAPI-ATTACH-OWNER-001.md).
This supersedes the earlier facade restriction; it does not change the single
loaded runtime host or the companion's borrowed-tools and dedicated-preview
ownership contracts. Distinct public core peers may coexist with native preview.

## References

- [REQ-COMP-BOUNDARY-001](../requirements/architecture/REQ-COMP-BOUNDARY-001.md)
- [REQ-COMP-API-001](../requirements/architecture/REQ-COMP-API-001.md)
- [REQ-COMP-OWNER-001](../requirements/architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-CONFIG-001](../requirements/architecture/REQ-COMP-CONFIG-001.md)
