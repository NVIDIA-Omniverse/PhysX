<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OMNIPVD-001
title: OmniPVD Startup Transport Configuration
status: implemented
owner: ovphysx
---

## Description

The typed C configuration selects the OmniPVD destination when an instance is
created while preserving existing FILE recording behavior.

## Acceptance Criteria

- AC-1: Transport, TCP address, TCP port, and millisecond timeout keys and named builders are appended without changing existing config-key values.
- AC-2: Invalid destinations return `OVPHYSX_API_INVALID_ARGUMENT`; startup-only keys return an error instead of mutating a live runtime.
- AC-3: Omitting transport selects FILE and preserves existing file capture behavior.

## Test References

- TEST-CAPI-OMNIPVD-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx_config.h
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_global_settings.cpp
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp
- ovphysx/tests/python_tests/test_types_sync.py

## Dependencies

- REQ-OMNIPVD-TRANSPORT-001
