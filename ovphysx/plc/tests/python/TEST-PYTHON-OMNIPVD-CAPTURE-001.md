<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-CAPTURE-001
maps_to: REQ-PYTHON-OMNIPVD-CAPTURE-001
type: integration
---

## Scenario

Python configuration and synchronous marshaling preserve one native recording filter across validation, recording, TCP discovery and restart.

## Given

- An import of the source package before a native build for pure configuration tests.
- A fake synchronous C start that forces garbage collection while inspecting each borrowed selector.
- A capable native runtime with `simple_physics_scene.usda` for the subprocess recording case and C ABI probes for layout checks.

## When

- Construct a default filter and filters with `record_secondary_data=True` and `False`; reject non-boolean values, including integers `0` and `1`; mutate source lists; supply malformed path sequences.
- Call selective FILE and TCP start with both path lists, duplicates and each `record_secondary_data` boolean value.
- Call TCP discovery with no filter and with both secondary-data values, non-ASCII selectors and duplicates; force garbage collection inside the fake C call. Attempt FILE discovery, invalid Python descriptors and native `INVALID_STATE`/`NOT_IMPLEMENTED` responses. Inspect the discovery binding's argument and result types.
- Attempt a missing object path, then record selected primary data, stop and restart unrestricted output.
- Compare options size, alignment and all field offsets against the native probes; simulate a library missing the recording-options export at bootstrap.
- Record particle-system, deformable surface and deformable volume actors in fresh CUDA subprocesses with ordinary readback and DirectGPU enabled.

## Then

- Pure imports expose `OmniPvdFilter`, `record_secondary_data` defaults to `True` and rejects non-boolean values, copied tuples survive caller mutation, and invalid shapes raise the documented Python errors (REQ AC-1).
- Garbage collection cannot invalidate any selector buffer, values match the original union and `record_secondary_data`, and a null filter uses the same export with null options (REQ AC-2).
- The missing object path reports `NOT_FOUND` without output, valid CPU and GPU sessions finalize nonempty files in both GPU modes and the ABI layouts agree. An older library missing the options export fails bootstrap with a version diagnostic before any start call (REQ AC-3).

Automated cases: `ovphysx/tests/python_tests/test_omnipvd_capture.py`, the filter buffer and selective restart cases in `ovphysx/tests/python_tests/test_omnipvd_recording.py`, and the filter layout case in `ovphysx/tests/python_tests/test_abi_checks.py`.

Bootstrap mismatch coverage: `test_missing_native_symbol_reports_versions` in `ovphysx/tests/python_tests/test_bindings_internal.py`.
- Discovery uses the same C start entry point with `wait_for_selection=true`; missing filters use empty default selectors and supplied UTF-8 destination/selector values survive garbage collection. FILE and malformed descriptors fail before a C call, and native errors preserve the start_recording operation name, status and detail (REQ AC-4). The discovery cases of `test_tcp_streams_to_ready_listener` run a real session: `is_recording()` stays true across a step, and `stop_recording()` ends it with a clean EOF. The C API tests cover the catalog.

The `test_start_recording_wait_for_selection_*` cases exercise catalog-first marshaling and validation through the unified start.

`test_recording_options_reject_old_layout_with_same_size` rejects a pre-discovery native layout even when the wait flag fits its former tail padding (REQ AC-4).
