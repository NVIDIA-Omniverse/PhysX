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


def _make_dltensor(buf, n, code, bits):
    """Build a 1-D DLTensor over `buf` with `n` elements of the given dtype code/bits."""
    shape = (ctypes.c_int64 * 1)(n)
    t = DLTensor()
    t.data = ctypes.cast(buf, ctypes.c_void_p) if buf is not None else None
    t.ndim = 1
    t.shape = shape
    t.byte_offset = 0
    t.dtype.code = code
    t.dtype.bits = bits
    t.dtype.lanes = 1
    return t, shape  # keep `shape` (and the caller's buf) alive for the call


def test_dltensor_bool_column_decodes_as_numpy_bool():
    """A non-empty kDLBool (code=6, bits=8) column decodes to numpy bool (issue #8, !7861/!7882 review).

    setBool writes {kDLBool, 8, 1}; the read-column decoder must map it AND honor the
    numpy dtype (bool), not return uint8 for the non-empty path.
    """
    import numpy as np
    from ovphysx.api import PhysX

    buf = (ctypes.c_uint8 * 3)(1, 0, 1)
    t, _shape = _make_dltensor(buf, 3, 6, 8)

    arr = PhysX._dltensor_to_numpy(t, np)
    assert arr.shape == (3,)
    assert arr.dtype == np.bool_
    assert arr.tolist() == [True, False, True]


def test_dltensor_bool_empty_and_nonempty_dtype_parity():
    """Empty and non-empty kDLBool columns return the same numpy dtype (bool)."""
    import numpy as np
    from ovphysx.api import PhysX

    empty_t, _es = _make_dltensor(None, 0, 6, 8)
    empty = PhysX._dltensor_to_numpy(empty_t, np)
    assert empty.shape == (0,)
    assert empty.dtype == np.bool_

    buf = (ctypes.c_uint8 * 2)(1, 0)
    nonempty_t, _ns = _make_dltensor(buf, 2, 6, 8)
    nonempty = PhysX._dltensor_to_numpy(nonempty_t, np)
    assert nonempty.dtype == empty.dtype == np.bool_


def test_dltensor_int64_column_dtype_parity():
    """A non-bool column keeps its natural dtype (int64) through the astype copy."""
    import numpy as np
    from ovphysx.api import PhysX

    buf = (ctypes.c_int64 * 2)(-3, 7)
    t, _shape = _make_dltensor(buf, 2, 0, 64)  # kDLInt, 64

    arr = PhysX._dltensor_to_numpy(t, np)
    assert arr.dtype == np.int64
    assert arr.tolist() == [-3, 7]


def test_dltensor_unsupported_dtype_raises_typeerror():
    """A genuinely unmapped dtype (float16) raises instead of silently decoding as float32."""
    import numpy as np
    from ovphysx.api import PhysX

    buf = (ctypes.c_uint8 * 2)(0, 0)
    t, _shape = _make_dltensor(buf, 1, 2, 16)  # kDLFloat, 16 (float16) - not supported

    with pytest.raises(TypeError):
        PhysX._dltensor_to_numpy(t, np)
