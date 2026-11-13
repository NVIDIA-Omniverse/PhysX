<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-ATTRIBUTES-001
maps_to: REQ-PYTHON-ATTRIBUTES-001
type: unit
---

## Scenario

Discover metadata without a native SDK and reject mutation.

## Given

- `tests/python_tests/test_attribute_metadata.py` and the source package or installed wheel.

## When

- Run `AttributeMetadataTest`, including its fresh interpreter with native imports explicitly blocked.

## Then

- The catalog imports and resolves angular velocity without native imports (REQ AC-1).
- Target lookup, directional filters and errors have documented behavior; invalid representations consistently raise ValueError (REQ AC-2).
- Rigid angular velocity uses radians/second, joint coordinates distinguish degrees, instancer positions use their own frame and singular request name, particle aliases are read-only, and scope layouts differ by direction, and vehicle ranges/applicability match the contract (REQ AC-3).
- Units expose readable expressions and immutable conditional/component metadata without exponent maps; spatial tendons use distance-based units and inverse-dynamics outputs distinguish matrix and effort units (REQ AC-3, AC-4).
- Equal descriptors work as dictionary keys; mutation of the dataclass, nested layout, value constraints or unit registry raises an exception (REQ AC-4).
- `SimObjectType` and plain integer targets resolve correctly; legacy `ObjectType` and unrelated integer/string enums raise `ValueError`. Culling-box write units separate the flag from bounds; unsupported directions omit layouts (REQ AC-2, AC-3).
