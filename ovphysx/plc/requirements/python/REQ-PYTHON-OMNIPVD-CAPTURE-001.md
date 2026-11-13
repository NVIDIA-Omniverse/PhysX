<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-CAPTURE-001
title: Python OmniPVD Filter Descriptor
status: implemented
owner: ovphysx
---

## Description

Python exposes the C recording filter through a pure descriptor and optional arguments for recording and TCP discovery.

## Acceptance Criteria

- AC-1: **Pure immutable descriptor.** `OmniPvdFilter` imports without loading native bindings; the frozen descriptor copies scene/object path sequences to tuples and rejects invalid sequence types and non-exact absolute paths. Its `record_secondary_data: bool = True` option accepts only `bool` values, rejecting integers and other non-boolean values. Primary data remains recorded for either value; `True` also enables secondary data.
- AC-2: **Synchronous forwarding.** `PhysX.start_recording(destination, filter=...)` retains every UTF-8 selector buffer through the C call and forwards the `scenes` and `objects` union and `record_secondary_data` boolean for FILE and TCP; `filter=None` passes null options through the same start export for unrestricted recording.
- AC-3: **Native lifecycle and ABI.** Native validation errors remain visible in Python, invalid paths create no output, stop permits a fresh selective or unrestricted session, and ctypes size, alignment and field offsets match the C options. Import requires the options layout export and rejects an incompatible layout before calling the changed start signature. Particle-system and deformable selections accept real payload recording in both ordinary GPU and DirectGPU scenes.
- AC-4: **TCP discovery forwarding.** `PhysX.start_recording(destination, *, filter=None, wait_for_selection=True)` rejects FILE destinations before calling C and forwards TCP destinations and optional filters to `ovphysx_ovd_start_recording` with `wait_for_selection=true`, retaining all UTF-8 buffers through the call. A missing filter uses default options with empty selectors and `wait_for_selection=true`; native status and error text remain visible. The boolean keyword defaults to false and rejects non-boolean values. The binding verifies the wait field offset as well as the options size before native calls, including libraries whose old options fit the same size through tail padding. Discovery requires this instance's live attachment; missing filters or empty lists select all supported objects in that attachment, while scene/object lists bound the maximum discovery and future recording scope that receiver selection cannot widen. The native waiting session publishes a catalog before detailed recording; `is_recording()` includes this owned session and `stop_recording()` stops it.

## Test References

- [TEST-PYTHON-OMNIPVD-CAPTURE-001](../../tests/python/TEST-PYTHON-OMNIPVD-CAPTURE-001.md) (AC-1 through AC-4)

## Code References

- `ovphysx/python/ovphysx/config.py`, `ovphysx/python/ovphysx/__init__.py` and `ovphysx/python/ovphysx/__init__.pyi`: public descriptor (AC-1).
- `ovphysx/python/ovphysx/api.py`, `ovphysx/python/ovphysx/api.pyi` and `ovphysx/python/ovphysx/_bindings.py`: optional arguments, discovery entry point and ctypes forwarding (AC-2 through AC-4).
- `ovphysx/tests/python_tests/test_omnipvd_capture.py`, `ovphysx/tests/python_tests/test_omnipvd_recording.py` and `ovphysx/tests/python_tests/test_abi_checks.py`: pure configuration, retained buffers, discovery forwarding, native restart and ABI coverage (AC-1 through AC-4).

## Dependencies

- [REQ-CAPI-OMNIPVD-CAPTURE-001](../capi/REQ-CAPI-OMNIPVD-CAPTURE-001.md)
