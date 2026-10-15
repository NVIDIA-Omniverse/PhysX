<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-IDENTITY-001
maps_to: REQ-COMP-IDENTITY-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-IDENTITY-001; executable evidence and qualification limits follow.

## Scenario

Compute for an instanced object, then change topology and reuse the same numeric token in another source.

## Given

- Two independent source dictionaries with identical numeric tokens naming different paths.
- Two instances of one mesh and a saved result for the first instance at revision R.

## When

- Resolve identities against each dictionary; delete/recreate one instance and edit topology before applying the saved result.

## Then

- Resolution follows the provided dictionary and never assumes numeric token equivalence (REQ AC-1).
- The two instances remain distinct, and recreation invalidates the old generation (REQ AC-2).
- Applying the revision-R result after topology changes returns stale before any output is authored (REQ AC-3).
- Missing and foreign objects are reported explicitly without resolving against the other source (REQ AC-4).

## Executable coverage

[test_inspection.py](../../../tests_native/test_inspection.py):

Native inspection/query/interaction/plugin tests exercise exact owner and instance references, reverse paths, foreign/stale rejection, authored clones and object deletion/recreation. tests/test_usd_helpers.py and test_attachment_elements.py reject stale source/topology before publication. USD instance authoring remains explicitly unsupported.

The authored-custom-clone case in `tests_native/test_plugins.py` clones at ordinal 2,
customizes copied parameters at ordinal 3, then seals and attaches at ordinal 3.
Its original mass, stiffness, and independent-lifetime assertions remain unchanged.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
