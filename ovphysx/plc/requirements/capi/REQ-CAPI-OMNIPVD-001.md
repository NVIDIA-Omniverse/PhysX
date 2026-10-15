<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OMNIPVD-001
title: Retired OmniPVD Startup Configuration
status: implemented
owner: ovphysx
---

## Description

Standalone ovphysx rejects retired OVD startup configuration. Recording uses the
global capability and an explicit FILE or TCP destination after attachment.

## Acceptance Criteria

- AC-1: Public config enums and builders expose no OVD startup keys or legacy capability helper. The legacy OVD config header is removed. Retired bool slots 3/4, int32 slots 2/3, and string slots 0/2/3 remain reserved; unrelated values and COUNT sentinels do not change.
- AC-2: Retired typed numeric keys and raw Carbonite OVD startup keys, with or without a leading slash, return `OVPHYSX_API_INVALID_ARGUMENT` at create and post-create configuration, with a diagnostic directing callers to `OVD_RECORDING_CAPABLE` and explicit recording start. Legacy typed getters reject retired slots with the same result and migration guidance.
- AC-3: Rejection does not change configured capability or start output. Plain creation and attachment remain dormant; unrelated legacy configuration keeps its existing behavior.

## Test References

- TEST-CAPI-OMNIPVD-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx_legacy_types.h
- ovphysx/include/ovphysx/ovphysx_legacy.h
- ovphysx/include/ovphysx/ovphysx_legacy_config.h
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_global_settings.cpp
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp
- ovphysx/tests/python_tests/test_types_sync.py

## Dependencies

- REQ-CAPI-OMNIPVD-LATE-001
