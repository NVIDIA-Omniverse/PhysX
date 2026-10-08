<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PATTERN-001
title: Path Pattern Component Length Limit
status: implemented
owner: ovphysx
---

## Description

Every ovphysx entry point that takes a physics-object path pattern or an
explicit path list (tensor bindings, SDF views, contact bindings) is matched
one component at a time (the text between `/` separators, a parenthesized
group counting as one component), and the runtime compiles each component
to a regular expression whose stack cost grows with the component length. No
legitimate prim name or alternation list approaches
`omni::physics::tensors::kMaxPathPatternComponentLength` (4096 characters), so
ovphysx shall treat a longer component as invalid input and reject it at the
public boundary. The caller gets an argument error naming the limit instead of
a crash or a silently empty result; the runtime keeps the same bound as a
backstop for input that reaches it another way.

## Acceptance Criteria

- AC-1: `ovphysx_create_tensor_binding` returns `OVPHYSX_API_INVALID_ARGUMENT`,
  with an error message naming the limit, when `pattern` or any `prim_paths`
  entry contains a component longer than `kMaxPathPatternComponentLength`. No
  binding is created.
- AC-2: `ovphysx_create_sdf_view` (`pattern`) and
  `ovphysx_create_contact_binding` (`sensor_patterns` and `filter_patterns`)
  apply the same rejection.
- AC-3: A component of exactly `kMaxPathPatternComponentLength` characters is
  accepted and matched normally. Components are delimited by `/` outside
  balanced parentheses, the same rule as the runtime matcher's group-aware
  tokenizer, so a parenthesized group that spans a `/` is measured whole. The
  check is never looser than the runtime; it is stricter only for an unclosed
  `(` followed by a trailing `/`, which the runtime trims before tokenizing.
- AC-4: An oversized component that still reaches the runtime matcher never
  matches and logs an error instead of crashing (owned by ovruntime
  [REQ-PUBLICAPI-001](../../../ovruntime/plc/requirements/api/REQ-PUBLICAPI-001.md) AC-49).

## Test References

- TEST-CAPI-PATTERN-001

## Code References

- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp (`hasOversizedPathComponent`,
  `oversizedPathComponentMessage`)
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (AC-1, AC-3)
- ovphysx/src/ovphysx/ovphysxSdfView.cpp (AC-2)
- ovphysx/src/ovphysx/ovphysxContactBinding.cpp (AC-2)
- ovphysx/include/ovphysx/ovphysx.h (pattern-matching documentation block)
- ovphysx/python/ovphysx/api.py (`create_tensor_binding`, `create_sdf_view` and
  `create_contact_binding` docstrings)
- ovphysx/ovruntime/include/omni/physics/tensors/ISimulationView.h
  (`kMaxPathPatternComponentLength`, the shared bound)
- ovphysx/tests/python_tests/cpu_tests/test_pattern_component_length.py

## Dependencies

- [REQ-PUBLICAPI-001](../../../ovruntime/plc/requirements/api/REQ-PUBLICAPI-001.md)
  AC-49 - the ovruntime matcher's backstop for the same bound
