<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-LEGACY-001
maps_to: REQ-CAPI-LEGACY-001
type: unit
---

## Scenario

The packaged legacy header compiles standalone and in either include order
with the primary header, and a primary-only translation unit can still name
a moved symbol.

## Given

- `ovphysx/tests/c_unittests/test_legacy_header_includes.cpp` plus the four
  compile-only C++ translation units `test_legacy_header_include_*.cpp`,
  compiled into `c_unittests`.
- The four matching C translation units `test_legacy_header_include_*.c`,
  compiled into the Linux C11 `c_api_compatibility_test` target (same include
  orders; skipped on Windows with the rest of that target).
- Public headers under `ovphysx/include/ovphysx/`.
- `ovphysx/tests/c_unittests/test_public_header_no_deprecated.cpp`,
  compiled as the `public_header_no_deprecated_test` OBJECT target with
  deprecated-declaration diagnostics as errors (does not use
  `ovphysx_apply_compiler_defaults`).

## When

- Four compile-time include orders are instantiated as separate C and C++
  translation units: `ovphysx.h` only, `ovphysx_legacy.h` only, legacy then
  primary, primary then legacy.
- Each fragment names at least one symbol from a moved family (tensor
  binding, scene query, contact binding, SDF view, or `ovphysx_get_object_type`).
- A runtime gtest case takes the address of `ovphysx_create_tensor_binding`
  after including only `ovphysx.h` (the umbrella path).

## Then

- The C++ test binary links (REQ AC-4, AC-5). The Linux C11 target compiles
  the four C include-order translation units (REQ AC-4).
- The address of `ovphysx_create_tensor_binding` is non-null (REQ AC-3, AC-5).
- `ovphysx_legacy.h` does not include `ovphysx.h` (REQ AC-2), checked by
  scanning the header text.
- `ovphysx_types.h` does not include `ovphysx_legacy_types.h` (REQ AC-2).
- `ovphysx.h` contains `#include "ovphysx/ovphysx_legacy.h"` (REQ AC-3).
- `ovphysx_warmup` is declared in `ovphysx.h`, not `ovphysx_legacy.h`
  (REQ AC-6).
- The experimental C++ header include-only translation unit compiles under
  deprecated-declaration errors (REQ AC-7).
