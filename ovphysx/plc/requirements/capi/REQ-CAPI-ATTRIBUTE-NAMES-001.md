<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ATTRIBUTE-NAMES-001
title: Portable Attribute Names and Dictionary Tokens
status: implemented
owner: ovphysx
---

## Description

Public attributes have generated string-backed names. Explicit interning resolves a portable string into the caller-owned dictionary.

## Acceptance Criteria

- AC-1: **Portable names.** Existing `OVPHYSX_ATTR_*` spellings and debug name exports remain compatible; every public attribute has an `OVPHYSX_NAME_*` object containing its canonical string and an invalid token. The one exception is a name removed with its attribute: `OVPHYSX_ATTR_JACOBIAN_SHAPE` and `OVPHYSX_NAME_JACOBIAN_SHAPE` are gone, since `jacobian` carries its own dimensions, and a read naming `jacobianShape` reports it as an unsupported attribute.
- AC-2: **Dictionary ownership.** `ovphysx_intern_attribute_name` invokes the supplied dictionary on every call and never caches numeric tokens across calls or owners.
- AC-3: **Errors.** The helper clears output on failure and forwards backend status and error ownership. Invalid arguments return an empty-error failure. The public helper and generated name declarations compile as C11 and C++ using installed SDK headers without source-tree includes.

## Test References

- [TEST-CAPI-ATTRIBUTE-NAMES-001](../../tests/capi/TEST-CAPI-ATTRIBUTE-NAMES-001.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx_attributes.h`
- `ovphysx/include/ovphysx/generated/AttributeNames.inc`
- `ovphysx/src/ovphysx/generated/AttributeNames.inc`
- `ovphysx/src/ovphysx/ovphysxAttributeNames.cpp`
- `ovphysx/include/ovphysx/ovphysx_attribute_tokens.h`
- `ovphysx/tests/c_unittests/test_attribute_names.cpp`
- `ovphysx/tests/c_unittests/test_public_header_no_deprecated.c`
- `ovphysx/tests/c_unittests/test_public_header_no_deprecated.cpp`
- `ovphysx/tests/installed_headers/headers.c`
- `ovphysx/tests/installed_headers/headers.cpp`
- `ovphysx/scripts/test_cpp_samples.cmake` (installed-header compilation)

## Dependencies

- [REQ-CAPI-STRING-001](REQ-CAPI-STRING-001.md) (length-prefixed input strings)

