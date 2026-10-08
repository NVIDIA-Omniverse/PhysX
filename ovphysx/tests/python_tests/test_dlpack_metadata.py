# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-2 AC-3
# @maps_to TEST-PYTHON-READ-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-INPUT-DEVICE-001
# @covers AC-4
# @maps_to TEST-INPUT-DEVICE-001

import ctypes
import subprocess
import sys
import textwrap

import numpy as np
import pytest
from ovphysx._dlpack_utils import _validate_c_contiguous_layout
from ovphysx.dlpack import DLDeviceType, DLTensor


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
        from ovphysx.dlpack import DLManagedTensor

        PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
        PyCapsule_New = ctypes.pythonapi.PyCapsule_New
        PyCapsule_New.argtypes = [ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor]
        PyCapsule_New.restype = ctypes.py_object

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


def _make_dltensor(
    buf,
    shape_values,
    code,
    bits,
    *,
    lanes=1,
    device_type=DLDeviceType.kDLCPU,
    device_id=0,
):
    """Build a DLTensor and retain its ctypes metadata for the converter call."""
    shape = (ctypes.c_int64 * len(shape_values))(*shape_values)
    t = DLTensor()
    t.data = ctypes.cast(buf, ctypes.c_void_p) if buf is not None else None
    t.device.device_type = device_type
    t.device.device_id = device_id
    t.ndim = len(shape_values)
    t.shape = shape
    t.strides = None
    t.byte_offset = 0
    t.dtype.code = code
    t.dtype.bits = bits
    t.dtype.lanes = lanes
    t._test_keepalive = (buf, shape)
    return t


class _BorrowHolder:
    def __init__(self):
        self.retains = 0
        self.releases = 0

    def retain(self):
        self.retains += 1

    def release_borrow(self, *_ignored):
        self.releases += 1


def _convert(t, holder=None):
    import warp as wp
    from ovphysx.api import PhysX

    return object.__new__(PhysX)._dltensor_to_warp_array(t, wp, holder or _BorrowHolder())


def _convert_write(t):
    import warp as wp
    from ovphysx.api import PhysX

    return object.__new__(PhysX)._dltensor_to_warp_array(t, wp, None)


def test_dltensor_bool_column_wraps_as_warp_bool():
    """A non-empty kDLBool column is a zero-copy Warp bool array."""
    import gc

    import warp as wp

    buf = (ctypes.c_uint8 * 3)(1, 0, 1)
    holder = _BorrowHolder()
    arr = _convert(_make_dltensor(buf, [3], 6, 8), holder)

    assert isinstance(arr, wp.array)
    assert arr.shape == (3,)
    assert arr.dtype == wp.bool
    assert arr.ptr == ctypes.addressof(buf)
    assert arr.numpy().tolist() == [True, False, True]
    assert holder.retains == 1 and holder.releases == 0

    del arr
    gc.collect()
    assert holder.releases == 1


def test_dltensor_bool_empty_and_nonempty_dtype_parity():
    """Empty and non-empty kDLBool columns have the same Warp dtype."""
    import warp as wp

    empty_holder = _BorrowHolder()
    empty = _convert(_make_dltensor(None, [0], 6, 8), empty_holder)
    assert empty.shape == (0,)
    assert empty.dtype == wp.bool
    assert empty_holder.retains == 0, "an empty Warp-owned array must not retain the read session"

    buf = (ctypes.c_uint8 * 2)(1, 0)
    nonempty = _convert(_make_dltensor(buf, [2], 6, 8))
    assert nonempty.dtype == empty.dtype == wp.bool


@pytest.mark.parametrize(
    ("code", "bits", "ctype", "warp_name"),
    [
        (2, 32, ctypes.c_float, "float32"),
        (0, 64, ctypes.c_int64, "int64"),
        (6, 8, ctypes.c_uint8, "bool"),
    ],
)
def test_supported_dltensor_dtypes_map_to_warp(code, bits, ctype, warp_name):
    """A float, an int, and the non-trivial kDLBool mapping produce the Warp scalar."""
    import warp as wp

    buf = (ctype * 1)(1)
    arr = _convert(_make_dltensor(buf, [1], code, bits))

    assert arr.dtype == getattr(wp, warp_name)
    expected = [True] if warp_name == "bool" else [1]
    assert arr.numpy().reshape(-1).tolist() == expected


def test_dltensor_lanes_become_trailing_warp_dimension():
    """Native dtype lanes become a trailing scalar dimension."""
    import warp as wp

    backing = (ctypes.c_float * 6)(*range(6))
    t = _make_dltensor(backing, [2], 2, 32, lanes=3)
    arr = _convert(t)

    assert isinstance(arr, wp.array)
    assert arr.ptr == ctypes.addressof(backing)
    assert arr.shape == (2, 3)
    assert arr.numpy().tolist() == [[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]]


def test_dltensor_byte_offset_shifts_the_aliased_base_address():
    """byte_offset is part of the address, not the shape.

    Dropping it aliases the column from the wrong first element and reads past the end
    of the allocation, with no error anywhere.
    """
    import warp as wp

    backing = (ctypes.c_float * 6)(*range(6))
    t = _make_dltensor(backing, [3], 2, 32)
    t.byte_offset = 3 * ctypes.sizeof(ctypes.c_float)
    arr = _convert(t)

    assert isinstance(arr, wp.array)
    assert arr.ptr == ctypes.addressof(backing) + t.byte_offset
    assert arr.numpy().tolist() == [3.0, 4.0, 5.0]


def test_write_dltensor_nonempty_cpu_array_aliases_byte_offset_without_a_lease():
    """A non-empty write array aliases mapped CPU storage and does not own its lifetime."""
    import warp as wp

    backing = (ctypes.c_float * 6)(*range(6))
    t = _make_dltensor(backing, [3], 2, 32)
    t.byte_offset = 3 * ctypes.sizeof(ctypes.c_float)
    arr = _convert_write(t)

    assert isinstance(arr, wp.array)
    assert arr.device.is_cpu
    assert arr.ptr == ctypes.addressof(backing) + t.byte_offset
    assert arr.deleter is None
    arr.assign(np.array([10.0, 11.0, 12.0], dtype=np.float32))
    assert list(backing)[3:] == [10.0, 11.0, 12.0]


def test_write_dltensor_empty_cpu_array_is_warp_owned():
    """A zero-element write tensor needs no external pointer or session lease."""
    import warp as wp

    arr = _convert_write(_make_dltensor(None, [0], 2, 32))

    assert isinstance(arr, wp.array)
    assert arr.device.is_cpu
    assert arr.shape == (0,)
    assert arr.dtype == wp.float32
    assert arr.ptr is None
    assert arr.deleter is not None


def test_dltensor_unsupported_dtype_raises_typeerror():
    """A genuinely unmapped dtype (float16) raises instead of silently decoding as float32."""
    buf = (ctypes.c_uint8 * 2)(0, 0)
    t = _make_dltensor(buf, [1], 2, 16)  # kDLFloat 16 (float16) is not supported

    with pytest.raises(TypeError):
        _convert(t)


def test_dltensor_unsupported_device_is_rejected():
    buf = (ctypes.c_float * 1)(1.0)
    t = _make_dltensor(buf, [1], 2, 32, device_type=DLDeviceType.kDLCUDAHost)
    with pytest.raises(TypeError, match="unsupported DLPack device"):
        _convert(t)
