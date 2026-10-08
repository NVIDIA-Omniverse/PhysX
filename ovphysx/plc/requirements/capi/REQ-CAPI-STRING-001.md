<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-STRING-001
title: Asymmetric String Termination Contract
status: implemented
owner: ovphysx
---

## Description

ovphysx accepts length-prefixed input substring views without requiring a
trailing NUL. Every string successfully produced or delivered by ovphysx,
including populated output arrays and callback parameters, has a non-NULL
pointer and `ptr[length] == '\0'`.

## Acceptance Criteria

- AC-1: Input `ovphysx_string_t` values remain length-based and need not be
  NUL-terminated.
- AC-2: Error-query strings, including empty results, have non-NULL pointers and
  a NUL at `ptr[length]`.
- AC-3: Successful caller-buffer string getters and populated ovphysx-owned
  name/path arrays expose a NUL at `ptr[length]` on every produced string.
  On `OVPHYSX_API_BUFFER_TOO_SMALL`, config string getters preserve `length` as
  the caller's capacity, NUL-terminate any non-empty buffer, and report the
  required capacity separately.
- AC-4: String callback parameters, including log and object-change paths, expose
  a NUL at `ptr[length]` for their callback lifetime.

## Test References

- TEST-CAPI-STRING-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/src/ovphysx/LogManager.cpp
- ovphysx/src/ovphysx/ovphysxObjectChangeCallbacks.cpp
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp
- ovphysx/src/ovphysx/ovphysxContactBinding.cpp
- ovphysx/src/include/internal/sidecar/ovphysxInternalObjectChange.h
- ovphysx/src/ovphysxInternal/ovphysxInternalObjectChange.cpp
- ovphysx/tests/c_unittests/test_user_task.cpp
- ovphysx/tests/c_unittests/test_global_settings.cpp
- ovphysx/tests/c_unittests/test_articulation_metadata.cpp
- ovphysx/tests/c_unittests/test_contact_binding.cpp
- ovphysx/tests/c_unittests/test_log_level.cpp

## Dependencies

- None
