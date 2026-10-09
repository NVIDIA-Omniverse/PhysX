# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CPU-mode pre-step session write (REQ-CAPI-WRITE-001 AC-5a).

On CPU a write before the first step commits and is applied. DirectGPU is the
mode that refuses; that case lives in tests/python_tests/test_write_api.py
(the GPU suite's physx_sdk fixture enables suppressReadback).
"""

# @implements REQ-CAPI-WRITE-001
# @covers AC-5a
# @maps_to TEST-CAPI-WRITE-001

import numpy as np
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _to_host(t) -> np.ndarray:
    return t if isinstance(t, np.ndarray) else t.numpy()


def _fill(t, block) -> None:
    if isinstance(t, np.ndarray):
        t.reshape(np.asarray(block).shape)[:] = block
        return
    t.assign(np.ascontiguousarray(block).reshape(t.shape))


def _read_positions(physx):
    with physx.read(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result.groups, "CPU can read authored state before the first step"
        return np.concatenate([_to_host(g.tensors[0]).reshape(g.prim_count, -1) for g in result.groups])


def test_prestep_write_applies_on_cpu(physx_sdk):
    """CPU: commit before the first step succeeds and the values read back (AC-5a)."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
        assert w.groups, "expected at least one writable group"
        float_off = 0
        chunks = []
        for g in w.groups:
            n = g.prim_count
            block = np.arange(n * 3, dtype=np.float32).reshape(n, 3) + 100.0 + float_off
            _fill(g.tensors[0], block)
            w.commit(g)
            chunks.append(block)
            float_off += n * 3
    target = np.concatenate(chunks)
    after = _read_positions(physx_sdk)
    np.testing.assert_allclose(after, target, rtol=0, atol=1e-3)
