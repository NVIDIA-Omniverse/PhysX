<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-ATTRIBUTES-001
title: Native-Free Attribute Discovery
status: implemented
owner: ovphysx
---

## Description

`ovphysx.attributes` exposes immutable static runtime I/O metadata for tools and users before a simulation is created.

## Acceptance Criteria

- AC-1: **Import.** Importing and querying the catalog loads no PhysX, Warp, OpenUSD, ovstage, NumPy or ctypes dependency.
- AC-2: **Discovery.** `get_attribute` and `list_attributes` select target/name/representation uses and filter directional support; target names, plain integers and `SimObjectType` are accepted; other enums are rejected to avoid legacy `ObjectType` collisions. Unknown targets and invalid representations raise `ValueError` and unsupported names raise `KeyError`.
- AC-3: **Metadata.** Descriptors expose target-specific type, units, frame, independent read/write request names, layouts, conditions, value constraints, applicability, effects, directional aliases and population mappings, preserving angular and instancer conventions. Unit metadata exposes explicit distance/mass/sec/deg/rad expressions, conditional variants and ordered component unit IDs, without population scale exponents. Directional unit overrides describe asymmetric packed encodings; unsupported directions expose no layout.
- AC-4: **Immutability.** Returned descriptors are hashable and their nested metadata cannot be mutated; stable names and shared registries are read-only.

## Test References

- [TEST-PYTHON-ATTRIBUTES-001](../../tests/python/TEST-PYTHON-ATTRIBUTES-001.md)

## Code References

- `ovphysx/python/ovphysx/attributes.py`
- `ovphysx/python/ovphysx/_attribute_data.py`
- `ovphysx/tests/python_tests/test_attribute_metadata.py`

## Dependencies

- [REQ-CAPI-ATTRIBUTE-NAMES-001](../capi/REQ-CAPI-ATTRIBUTE-NAMES-001.md) (public attribute vocabulary)
- [REQ-PYTHON-READ-001](REQ-PYTHON-READ-001.md) (query target and read-group conventions)

