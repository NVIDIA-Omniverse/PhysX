<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-LATE-001
title: Python Sequential Late OmniPVD Recording
status: implemented
owner: ovphysx
---

## Description

Python mirrors the OvPhysX explicit sequential recording API with a typed exact
destination. The `PhysX` object that starts recording owns that session; peer
objects neither report nor stop the owner's session.

## Acceptance Criteria

- AC-1: `OmniPvdDestination.file` and `.tcp` construct only canonical FILE and TCP destination tuples.
- AC-2: Module-level `set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)` selects the C BOOL global key (`0x0000000c`) before plain `PhysX()` and defaults false. Wrong types and cross-scope keys are rejected without changing the value. The scoped setter rejects writes after first successful create until successful native shutdown with no live instances, and configured values survive shutdown. The last Python lifecycle owner performs that shutdown when destroyed. `PhysXConfig` has no recording-capability or startup-output fields.
- AC-3: On Windows x86_64 and Linux x86_64/aarch64, a default `PhysX()` rejects start with `INVALID_STATE`, directing callers to set `OVD_RECORDING_CAPABLE` before the first instance, and an opted-in `PhysX` supports sequential FILE/TCP methods.
- AC-4: `PhysX.start_recording`, `stop_recording`, and `is_recording` map directly to the C API.
- AC-5: A failed destination open may be retried; an active session cannot be replaced and reports active until stop.
- AC-6: After stop, another session may start and each completed FILE recording is non-empty at its exact path.
- AC-7: In a fresh subprocess, creation, attach, and reattach remain dormant. Explicit start before the first simulation step captures that step. Only the starting `PhysX` reports active and can stop its session; after detach and dormant reattach, a new explicit session may have a different owner.
- AC-8: The synchronous Python methods inherit the OvPhysX same-thread and caller-serialization contract; concurrent recording/lifecycle calls across shared-runtime objects have no guarantee.

## Test References

- TEST-PYTHON-OMNIPVD-LATE-001 (AC-1 through AC-8)

## Code References

- ovphysx/python/ovphysx/config.py
- ovphysx/python/ovphysx/__init__.py
- ovphysx/python/ovphysx/__init__.pyi
- ovphysx/python/ovphysx/api.py
- ovphysx/python/ovphysx/api.pyi
- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/types.py
- ovphysx/docs/developer_guide.md
- ovphysx/tests/python_tests/test_omnipvd_recording.py
- ovphysx/tests/python_tests/lifecycle_tests/test_config_global.py
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py

## Dependencies

- REQ-CAPI-OMNIPVD-LATE-001
