<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-READ-001
title: Warp Output-Read Frontend
status: implemented
owner: ovphysx
---

## Description

The Python `PhysX.read()` and `read_tokens()` APIs expose physics output through
one directly usable array type on CPU and CUDA: `warp.array`. Non-empty arrays
alias the native read session's snapshot storage and retain that session until
the last Warp or downstream framework view releases it. The native C exchange
descriptor remains DLTensor-based and is not part of the Python result type.

CUDA result construction orders the native producer completion event onto the
current Warp stream for the column's device before returning. This is an
asynchronous stream wait, not a host synchronization. The caller establishes a
Warp stream dependency before using the result on another stream.

## Acceptance Criteria

- AC-1: Every returned tensor is a `warp.array` on the native CPU or CUDA
  device. `index_map` and `prim_index_map`, when present, are CPU
  `warp.array(dtype=wp.uint32)` values whatever the read's device. NumPy is not an
  output-read result type, and the public ovphysx `ManagedDLTensor` wrapper is
  removed rather than kept as an alternative one. Whatever DLPack typing remains
  in Python belongs to the compatibility APIs and is transitional.

- AC-2: DLPack scalar code and bits map to the matching Warp scalar dtype, and a
  `dtype.lanes` above 1 becomes a trailing shape dimension while a single-lane
  column gains none. Unsupported dtype or device fails explicitly. Native output
  columns remain dense with `byte_offset = 0` as required by REQ-READ-DEVICE-001
  AC-5.

- AC-3: A non-empty array aliases the native pointer without a conversion copy
  and owns one read-session lease. Closing the `ReadResult` releases its group
  metadata and owner reference, but arrays, Warp slices, and downstream DLPack
  consumers remain valid until their last reference is dropped. An empty tensor
  column is a valid Warp-owned empty array that retains no native lease; an empty
  index map is `None`.

- AC-4: Before returning a CUDA group's arrays, Python enqueues that group's
  non-zero producer event onto the current Warp stream under the matching device
  context, once per group. A failed wait raises before any unordered result is
  returned.

- AC-5: `ReadResult.close()` and context-manager exit both release group/path
  metadata, drop the result reference, and drain immediately due native cleanup
  on the owning thread. Finalization on another thread performs bookkeeping
  only; native read teardown remains deferred to an owning-thread SDK call.

- AC-6: The wheel declares one bounded, tested Warp feature line as a normal
  dependency, and runtime types, public type stubs, docs, samples, and tests all
  describe the same Warp return contract. Warp's transitive NumPy dependency is
  not presented as a direct NumPy output-read contract.

## Test References

- TEST-PYTHON-READ-001

## Code References

- ovphysx/python/ovphysx/api.py (`ReadGroup`, `ReadResult`, `PhysX._iterate_read_groups`, `PhysX._dltensor_to_warp_array`)
- ovphysx/python/ovphysx/api.pyi
- ovphysx/python/pyproject.toml
- ovphysx/tests/python_tests/test_output_read.py
- ovphysx/tests/python_tests/test_output_read_device.py
- ovphysx/tests/python_tests/test_output_read_device_stream.py
- ovphysx/tests/python_tests/cpu_tests/test_output_read_cpu.py
- ovphysx/tests/python_tests/test_output_read_lifetime.py
- ovphysx/tests/python_tests/test_dlpack_metadata.py
- ovphysx/tests/python_tests/test_type_stubs.py

## Dependencies

- [ADR-0023](../../../ovruntime/plc/adr/ADR-0023-warp-python-output-read-frontend.md)
- [REQ-CAPI-CUDA-002](../capi/REQ-CAPI-CUDA-002.md)
- [REQ-READ-CORE-001](../../../ovruntime/plc/requirements/output/REQ-READ-CORE-001.md)
- [REQ-READ-DEVICE-001](../../../ovruntime/plc/requirements/output/REQ-READ-DEVICE-001.md)
