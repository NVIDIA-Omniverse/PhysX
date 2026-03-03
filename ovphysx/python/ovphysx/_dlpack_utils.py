# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
DLPack utility functions for tensor interoperability.

Internal module - not part of the public API.
"""

import ctypes
from ctypes import POINTER, c_int64, c_void_p

from .dlpack import (
    DLDataType,
    DLDataTypeCode,
    DLDevice,
    DLDeviceType,
    DLManagedTensor,
    DLTensor,
    PyCapsule_GetPointer,
    PyCapsule_IsValid,
)


def _validate_c_contiguous_layout(dl_tensor: DLTensor) -> None:
    """Validate that DLTensor strides describe C-contiguous layout."""
    if not dl_tensor.strides:
        return

    expected_stride = 1
    for dim_idx in range(dl_tensor.ndim - 1, -1, -1):
        dim = dl_tensor.shape[dim_idx]

        # For dimensions of size 0 or 1, stride is irrelevant.
        if dim <= 1:
            continue

        if dl_tensor.strides[dim_idx] != expected_stride:
            raise ValueError(
                "Tensor must be C-contiguous (row-major). "
                "Call .contiguous() on a PyTorch tensor before passing it."
            )

        expected_stride *= dim


def acquire_dltensor(obj) -> tuple[DLTensor, object | None]:
    """Extract DLTensor and a keepalive reference (if needed).

    For objects implementing __dlpack__(), the returned DLTensor points into a
    DLManagedTensor owned by a Python capsule. We must keep the capsule alive
    until the native call completes.

    Args:
        obj: DLTensor, object with __dlpack__(), or numpy-like array

    Returns:
        Tuple of (DLTensor, keepalive_object). The keepalive_object must be
        kept alive for the duration of any C calls using the DLTensor.
    """
    if isinstance(obj, DLTensor):
        return obj, None

    # __dlpack__ protocol (NumPy >= 1.22, PyTorch, etc.)
    if hasattr(obj, "__dlpack__"):
        capsule = obj.__dlpack__()
        if PyCapsule_IsValid(capsule, b"dltensor") != 1:
            raise TypeError("__dlpack__() did not return a valid 'dltensor' capsule")
        managed_ptr = PyCapsule_GetPointer(capsule, b"dltensor")
        if not managed_ptr:
            raise TypeError("Failed to extract DLManagedTensor from capsule")
        managed = ctypes.cast(managed_ptr, POINTER(DLManagedTensor)).contents
        _validate_c_contiguous_layout(managed.dl_tensor)
        return managed.dl_tensor, capsule

    # NumPy-like
    if hasattr(obj, "__array_interface__"):
        import numpy as np

        arr = np.asarray(obj)
        return numpy_to_dltensor(arr), None

    raise TypeError(
        f"Object of type {type(obj).__name__} is not DLPack-compatible. "
        "Pass a NumPy array, PyTorch tensor, or object with __dlpack__ method."
    )


def numpy_to_dltensor(arr) -> DLTensor:
    """Convert NumPy array to DLTensor.

    The returned DLTensor has a _keepalive attribute that holds references to
    the underlying data buffers. The caller must keep the DLTensor alive for
    the duration of any C calls using it.

    Args:
        arr: NumPy array (must be C-contiguous)

    Returns:
        DLTensor with _keepalive attribute containing references that must
        stay alive during C calls.
    """
    import numpy as np

    if not arr.flags["C_CONTIGUOUS"]:
        raise ValueError("Array must be C-contiguous")

    dl_tensor = DLTensor()
    dl_tensor.data = arr.ctypes.data_as(c_void_p)
    dl_tensor.ndim = arr.ndim

    # Set up shape - must be kept alive for duration of C call
    shape_array = (c_int64 * arr.ndim)(*arr.shape)
    dl_tensor.shape = ctypes.cast(shape_array, POINTER(c_int64))

    # Strides (in bytes -> elements) - must be kept alive for duration of C call
    strides_array = (c_int64 * arr.ndim)(*[s // arr.itemsize for s in arr.strides])
    dl_tensor.strides = ctypes.cast(strides_array, POINTER(c_int64))

    # Device
    dl_tensor.device = DLDevice()
    dl_tensor.device.device_type = DLDeviceType.kDLCPU
    dl_tensor.device.device_id = 0

    # Data type
    dl_tensor.dtype = DLDataType()
    if arr.dtype == np.float32:
        dl_tensor.dtype.code = DLDataTypeCode.kDLFloat
        dl_tensor.dtype.bits = 32
    elif arr.dtype == np.int32:
        dl_tensor.dtype.code = DLDataTypeCode.kDLInt
        dl_tensor.dtype.bits = 32
    elif arr.dtype == np.float64:
        dl_tensor.dtype.code = DLDataTypeCode.kDLFloat
        dl_tensor.dtype.bits = 64
    elif arr.dtype == np.bool_:
        dl_tensor.dtype.code = DLDataTypeCode.kDLBool
        dl_tensor.dtype.bits = 8
    elif arr.dtype == np.uint8:
        dl_tensor.dtype.code = DLDataTypeCode.kDLUInt
        dl_tensor.dtype.bits = 8
    else:
        raise ValueError(f"Unsupported dtype: {arr.dtype}")
    dl_tensor.dtype.lanes = 1

    dl_tensor.byte_offset = 0

    # Store references on the tensor to prevent GC during C call
    dl_tensor._keepalive = (arr, shape_array, strides_array)

    return dl_tensor
