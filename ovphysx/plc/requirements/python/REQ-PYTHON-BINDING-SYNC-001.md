<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-BINDING-SYNC-001
title: CUDA Buffer Readiness at Synchronous Tensor Calls
status: implemented
owner: ovphysx
---

## Description

The Python adapter must finish producer work before passing a CUDA tensor to a
synchronous native tensor operation, including when physics uses a separate CUDA
context. Warp arrays use their current stream; other CUDA producers use the
`__dlpack_device__` and `__dlpack__(stream=...)` handoff. Raw `DLTensor`
descriptors carry no producer stream, so callers must make their storage ready.

## Acceptance Criteria

- AC-1: **Producer readiness.** `TensorBinding.read` waits for pending writes to
  its destination through the producer's stream handoff before native physics
  writes there.
- AC-2: **Input readiness.** `TensorBinding.write` waits for pending writes to the
  source and any CUDA indices or mask through their producer handoffs before
  consuming them.
- AC-3: **Descriptor reuse.** Reusing the same tensor through a cached binding
  descriptor provides the same producer ordering as its first use.

## Test References

- [TEST-PYTHON-BINDING-SYNC-001](../../tests/python/TEST-PYTHON-BINDING-SYNC-001.md)

## Code References

- ovphysx/python/ovphysx/_dlpack_utils.py (`synchronize_tensor_producer`, `acquire_dltensor`)
- ovphysx/python/ovphysx/api.py (`TensorBinding.read`, `TensorBinding.write`)
- ovphysx/tests/python_tests/test_tensor_warp_smoke_gpu.py

## Dependencies

None.
