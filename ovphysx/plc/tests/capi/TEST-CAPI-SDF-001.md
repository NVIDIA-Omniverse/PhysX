<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SDF-001
maps_to: REQ-CAPI-SDF-001
type: integration
---

## Scenario

`ovphysx_evaluate_sdf_shapes` and `PhysX.evaluate_sdf_shapes` evaluate `tests/data/sdf_cube.usda`'s cube (a static
SDF collider of half-extent 0.5 at the origin) as the SDF view's value test did, and refuse malformed calls,
unowned queries and CPU instances.

## Given

- C: the shared CPU instance, and a DirectGPU instance (suppressReadback) for the GPU suite; the cube's query
  opened with the pattern `/World/Cube`.
- Python: the DirectGPU session instance (`physx_sdk`) and the CPU one, with `sdf_cube.usda` loaded.

## When

- C, compiled as C11: `ovphysx_geometry_query.h` included on its own, and after `ovphysx.h`.
- C, without a stage: evaluations with a null tensor, null data, a points last dimension of 4, an out last
  dimension of 3, float64, a host tensor, disagreeing N, disagreeing Q, Q of 0, different devices, N of
  65536 and N x Q of 2^31; then a well-formed call on an unowned query.
- C, CPU instance: a well-formed evaluation of the cube's query; then a query opened before `detach_ovstage` and
  evaluated after the cube is attached again.
- C and Python, DirectGPU, no warmup call first: (0.45, 0, 0) and (0.55, 0, 0) evaluated; then N of 2.
- C, DirectGPU: points copied on a new non-blocking stream that records an event, evaluated with that stream and
  event as the producer handoff (the handoff's API shape).
- Python, DirectGPU, warmed up first: new points copied from pinned memory on a non-blocking stream held for
  0.5 s by a host function queued first, with an event recorded after the copy; with the stream and the event
  checked still pending, the evaluation is called with the stream alone, the event alone, and both.
- Python, DirectGPU: two points then five through fresh calls; the whole set against the pattern; a pattern
  matching nothing; N of 65536; an evaluation after `detach_ovstage` and a reload.
- Python, CPU: host arrays; a non-C-contiguous (strided) array; an empty pattern, a pattern with an embedded NUL,
  an oversized pattern component.
- C and Python, without an instance: the writability of `position` on SDF_SHAPE.

## Then

- Both translation units compile, each naming `ovphysx_evaluate_sdf_shapes` with its signature (and the header
  alone naming `OVPHYSX_QUERY_TARGET_SDF_SHAPE`) (AC-1).
- Every malformed C call returns `OVPHYSX_API_INVALID_ARGUMENT` and the unowned query `OVPHYSX_API_NOT_FOUND`,
  with `op_index` 0 (AC-1).
- On the CPU instance the cube is one row and the task fails naming the DirectGPU requirement (AC-1, AC-2); the
  query kept across `detach_ovstage` fails its task naming it as no longer attached, and still releases (AC-2).
  Neither refused call logs an auto-warmup, so neither steps the simulation, while the first accepted DirectGPU
  evaluation logs exactly one (AC-2).
- The inside point reads a negative distance and the outside one a positive distance, each with gradient x
  above 0.5; N of 2 fails the task naming one SDF shape (AC-2, AC-4).
- The handed-off points evaluate as written: in C the swapped points read swapped signs; in Python the
  evaluation reads the late producer's points, whose signs are the reverse of those already on the device
  (AC-1, AC-2, AC-4).
- The five-point call agrees with the two-point one on their shared points and reads +x, +y and -z gradients
  inside; the whole set equals the pattern; a pattern matching nothing says no SDF shape matched; N of 65536 raises
  `RuntimeError` naming the 65535 limit; the reloaded cube evaluates as before (AC-4).
- Host arrays raise `RuntimeError` naming the CUDA device, the strided array and the empty pattern `ValueError`,
  and the NUL and oversized patterns `RuntimeError` (AC-4).
- SDF_SHAPE has no writable attribute: `position` classifies as UNCLASSIFIED in both languages (AC-1).

## Test locations

- ovphysx/tests/c_unittests/test_sdf_shapes.cpp (`SdfShapesTest.*`, `SdfShapesGpuTest.*`)
- ovphysx/tests/c_unittests/test_geometry_query_header_include_only.c and
  test_geometry_query_after_primary.c (the `c_api_compatibility_test` target)
- ovphysx/tests/python_tests/test_sdf_shapes_gpu.py
- ovphysx/tests/python_tests/cpu_tests/test_sdf_shapes_cpu.py
