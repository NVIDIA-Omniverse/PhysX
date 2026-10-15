<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVD-NAMES-001
title: Public instance and OVD naming migration
status: implemented
owner: ovphysx
---

## Description

Explicit instance and recording names preserve their ABI and lifecycle while
standalone startup configuration is removed in favor of explicit recording.

## Acceptance Criteria

- AC-1: `ovphysx_instance_handle_t` remains a uint64 handle with zero invalid; public declarations, callbacks, native wrappers, and bindings use it consistently.
- AC-2: Canonical `ovphysx_ovd_*_recording` symbols preserve arguments/results and lifecycle; old symbols remain real forwarding exports in 0.7. Python method names remain unchanged.
- AC-3: Only the scoped global key exposes recording capability. Old startup controls, the legacy OVD header/module, and `PhysXConfig.omnipvd_recording_capable` are removed. Retired raw/typed keys return `INVALID_ARGUMENT` with migration guidance; their numeric slots stay reserved and unrelated values do not change.
- AC-4: Capability remains process-global, creation-time, and false by default; explicit recording starts after attach and can capture the first simulation frame without a preliminary step.

## Test References

- [TEST-CAPI-OVD-NAMES-001](../../tests/capi/TEST-CAPI-OVD-NAMES-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/ovphysx_config.h
- ovphysx/include/ovphysx/experimental/ovphysx.hpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/python/ovphysx/config.py
- ovphysx/python/ovphysx/types.py
- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/api.py
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp
- ovphysx/tests/python_tests/cpu_tests/test_public_ovd_api.py

## Dependencies

- [REQ-CAPI-OMNIPVD-LATE-001](REQ-CAPI-OMNIPVD-LATE-001.md)
