<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0004 - Customer USD authoring belongs in optional Python utilities
status: accepted
date: 2026-09-30
last_updated: 2026-10-02
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Applications need USD attachment/deformable authoring and writeback while keeping C++ free of USD. Customers already own the USD installation and stage used by their tools.

## Decision

1. Use a base companion Python package for data services and an optional usd subpackage for extraction, authoring, repair and output application.
2. Lazily import customer-installed pxr; ship no alternate USD runtime or compiled USD bindings. Pass scalars, strings, records and buffers across native bindings, never USD pointers or opaque stage IDs for dereferencing.
3. Require an explicit stage, edit target and time policy. Validate first and apply reversible batches; the host provides undo, persistence, approved thread and preview policy.
4. Reuse or delegate existing Python authoring helpers. The same implementation serves standalone callers and Kit.
5. Keep OVD decoding native only when it produces USD-independent data; construct the USD representation in Python.
6. An explicit native worker resolves ovphysx, ovstage and installed companion
   from its chosen interpreter. Do not prepend host site-packages or inherit
   host PYTHONPATH. Source-checkout development may bootstrap only the companion
   python directory, preserving the worker's own native dependencies.

**Out of scope**

- Selecting or installing the customer USD build automatically.
- Shipping the test-only native USD parsing backend.

## Alternatives considered

- Restore the native USD authoring backend: violates the requested native boundary and creates USD ABI coupling.
- Implement separate Kit and standalone authoring code: duplicates schemas, repairs and writeback behavior.
- Bundle a companion USD installation: conflicts with customer ownership and may load incompatible runtimes.

## Consequences

- **Positive:** USD workflows remain available to Python without expanding the native dependency closure.
- **Costs:** Test supported customer USD/schema combinations, missing dependencies, edit rollback and helper compatibility.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

## References

- [REQ-COMP-PYTHON-001](../requirements/python/REQ-COMP-PYTHON-001.md)
- [REQ-COMP-USD-EDIT-001](../requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-OVD-001](../requirements/diagnostics/REQ-COMP-OVD-001.md)
