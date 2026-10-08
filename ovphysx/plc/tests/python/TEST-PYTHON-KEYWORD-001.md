<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-KEYWORD-001
maps_to: REQ-PYTHON-KEYWORD-001
type: unit
---

## Scenario

The selected public Python call signatures and their shipped type stubs are
inspected without creating a native `PhysX` instance.

## Given

- The installed runtime Python API and its `api.pyi` type stub.
- The expected positional and keyword-only parameter names for each selected
  entry point.

## When

- Runtime callables are inspected with `inspect.signature`.
- The type stub is parsed with Python's `ast` module.

## Then

- Every selected runtime signature has exactly the expected positional and
  keyword-only parameter split (REQ AC-1).
- Every matching type-stub signature has the same split (REQ AC-2).
