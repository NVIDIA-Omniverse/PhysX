# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import ctypes
import subprocess
import sys
import textwrap

import pytest
from ovphysx._dlpack_utils import _validate_c_contiguous_layout
from ovphysx.dlpack import DLTensor


def test_validate_c_contiguous_layout_rejects_invalid_metadata():
    """Test metadata validation bounds reads to supported tensor ranks."""
    shape = (ctypes.c_int64 * 8)(*([1] * 8))

    for rank in (-1, 0, 9):
        dl_tensor = DLTensor()
        dl_tensor.ndim = rank
        dl_tensor.shape = shape
        with pytest.raises(ValueError, match="Tensor rank must be between 1 and 8"):
            _validate_c_contiguous_layout(dl_tensor)

    dl_tensor = DLTensor()
    dl_tensor.ndim = 1
    with pytest.raises(ValueError, match="Tensor shape must not be null"):
        _validate_c_contiguous_layout(dl_tensor)


def test_validate_c_contiguous_layout_accepts_supported_metadata():
    """Test maximum-rank and implicit-stride metadata remain supported."""
    shape = (ctypes.c_int64 * 8)(*([1] * 8))
    strides = (ctypes.c_int64 * 8)(*([1] * 8))

    dl_tensor = DLTensor()
    dl_tensor.ndim = 8
    dl_tensor.shape = shape
    dl_tensor.strides = strides
    _validate_c_contiguous_layout(dl_tensor)

    dl_tensor.strides = None
    _validate_c_contiguous_layout(dl_tensor)

    empty_shape = (ctypes.c_int64 * 1)(0)
    empty_strides = (ctypes.c_int64 * 1)(123)
    dl_tensor = DLTensor()
    dl_tensor.ndim = 1
    dl_tensor.shape = empty_shape
    dl_tensor.strides = empty_strides
    _validate_c_contiguous_layout(dl_tensor)


def test_acquire_dltensor_rejects_unbounded_rank_without_crashing():
    """Test malformed producer metadata is rejected without crashing Python."""
    script = textwrap.dedent("""
        import ctypes

        from ovphysx._dlpack_utils import acquire_dltensor
        from ovphysx.dlpack import DLManagedTensor, PyCapsule_Destructor, PyCapsule_New


        managed = DLManagedTensor()
        managed.dl_tensor.ndim = 1 << 28
        wild_pointer = ctypes.cast(ctypes.c_void_p(1), ctypes.POINTER(ctypes.c_int64))
        managed.dl_tensor.shape = wild_pointer
        managed.dl_tensor.strides = wild_pointer
        capsule = PyCapsule_New(
            ctypes.addressof(managed), b"dltensor", PyCapsule_Destructor()
        )


        class Producer:
            def __dlpack__(self):
                return capsule


        try:
            acquire_dltensor(Producer())
        except ValueError as exc:
            if "Tensor rank must be between 1 and 8" not in str(exc):
                raise
        else:
            raise AssertionError("Malformed DLPack metadata was accepted")
        """)

    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"DLPack validation child exited with {result.returncode}\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
