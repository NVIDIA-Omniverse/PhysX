<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-LEGACY-001
title: Explicit Legacy C API Header
status: implemented
owner: ovphysx
---

## Description

The primary C header ships the supported instance, session, stepping, and
interop surface. Transitional families (tensor bindings and binding-coupled
control, scene queries, contact bindings, SDF views, and
`ovphysx_get_object_type`) are declared in a dedicated public header,
`ovphysx/ovphysx_legacy.h`. Legacy-only types are defined once in
`ovphysx_legacy_types.h`. `ovphysx.h` includes the legacy header so existing
single-include consumers keep compiling. Moving a declaration does not remove
the exported implementation.

## Acceptance Criteria

- AC-1: The SDK install tree packages `include/ovphysx/ovphysx_legacy.h` and
  `include/ovphysx/ovphysx_legacy_types.h`. The wheel does not ship an
  SDK-style include tree; those headers appear in the wheel only as
  rendered-documentation downloads, per REQ-PACKAGING-DOCS-001 AC-3. A
  translation unit that includes only `ovphysx_legacy.h` can name every
  moved family listed in the Description.
- AC-2: Each public type has one definition. Combined includes of `ovphysx.h`
  and `ovphysx_legacy.h` in either order do not produce duplicate typedefs or
  include cycles. `ovphysx_types.h` does not include `ovphysx_legacy_types.h`.
- AC-3: A translation unit that includes only `ovphysx.h` can still name a
  moved symbol, because `ovphysx.h` includes `ovphysx_legacy.h`.
- AC-4: The four include orders compile as C and as C++: `ovphysx.h` only,
  `ovphysx_legacy.h` only, legacy then primary, primary then legacy.
- AC-5: Relocating a declaration does not remove its exported symbol. A
  program linked against the same `libovphysx` can still call the moved
  functions.
- AC-6: `ovphysx_warmup` remains declared directly by `ovphysx.h`; it is not
  part of the deprecated legacy surface.
- AC-7: A C++ translation unit that includes only
  `ovphysx/experimental/ovphysx.hpp` and does not call a deprecated API
  compiles with deprecated-declaration diagnostics as errors. Internal
  forwarding from a deprecated C++ wrapper to a deprecated C function does
  not leak that diagnostic at include time. A call to the deprecated C++
  wrapper still diagnoses.

## Test References

- TEST-CAPI-LEGACY-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (umbrella include of `ovphysx_legacy.h`)
- ovphysx/include/ovphysx/ovphysx_legacy.h
- ovphysx/include/ovphysx/ovphysx_legacy_types.h
- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/tests/c_unittests/test_legacy_header_includes.cpp
- ovphysx/tests/c_unittests/test_legacy_header_include_*.cpp
- ovphysx/tests/c_unittests/test_legacy_header_include_*.c (Linux C11 `c_api_compatibility_test`)
- ovphysx/include/ovphysx/experimental/ovphysx.hpp
- ovphysx/include/ovphysx/ovphysx_export.h
- ovphysx/tests/c_unittests/test_public_header_no_deprecated.cpp
- ovphysx/tests/c_unittests/CMakeLists.txt

## Dependencies

- [ADR-0037](../../../ovruntime/plc/adr/ADR-0037-explicit-legacy-api-boundary.md)
