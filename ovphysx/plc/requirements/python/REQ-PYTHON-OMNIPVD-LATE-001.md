<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-LATE-001
title: Python Sequential Late OmniPVD Recording
status: implemented
owner: ovphysx
---

## Description

Python mirrors the OvPhysX sequential late recording API with a typed exact destination. The `PhysX` object created
with startup output owns that startup session; peer objects neither report nor stop the owner's session.

## Acceptance Criteria

- AC-1: `OmniPvdDestination.file` and `.tcp` construct only canonical FILE and TCP destination tuples.
- AC-2: `PhysXConfig.omnipvd_recording_capable` and `ConfigBool.OMNIPVD_RECORDING_CAPABLE` map to the C creation setting and default to unset/false.
- AC-3: On Windows x86_64 and Linux x86_64/aarch64, a default `PhysX()` rejects late start with `INVALID_STATE` and an opted-in `PhysX` supports the existing sequential FILE/TCP methods.
- AC-4: `PhysX.start_recording`, `stop_recording`, and `is_recording` map directly to the C API.
- AC-5: A failed destination open may be retried; an active session cannot be replaced and reports active until stop.
- AC-6: After stop, another session may start and each completed FILE recording is non-empty at its exact path.
- AC-7: In a cold-start subprocess, startup ownership is reserved for the creating `PhysX`, becomes active when a stage is attached, remains invisible to a peer that cannot stop it, and lets the owner stop and restart to a late FILE destination. Reattaching with startup output configured starts a new startup session owned by the reattaching `PhysX`; after it stops that session, another late FILE destination may start.
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

## Dependencies

- REQ-CAPI-OMNIPVD-LATE-001
