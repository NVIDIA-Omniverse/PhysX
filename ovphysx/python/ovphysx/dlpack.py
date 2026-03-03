# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""DLPack tensor structures for zero-copy data interchange.

This module provides ctypes wrappers for the vendored DLPack C header, enabling
efficient data sharing between the C library and Python without copying.

NOTE: This file is NOT copied from another repo. It is a hand-written Python/ctypes
mirror of the C structs defined in ovphysx/dlpack/dlpack.h (which itself is the
upstream header from https://github.com/dmlc/dlpack). When the vendored C header
is updated, this file must be updated to match.
"""

import ctypes
from typing import Any, Callable, Optional

__all__ = [
    "DLDeviceType",
    "DLDataTypeCode",
    "DLDevice",
    "DLDataType",
    "DLTensor",
    "DLManagedTensor",
    "ManagedDLTensor",
    "DLPACK_VERSION",
]

# DLPack version 1.3
DLPACK_MAJOR_VERSION = 1
DLPACK_MINOR_VERSION = 3
# Legacy compat
DLPACK_VERSION = (DLPACK_MAJOR_VERSION << 8) | DLPACK_MINOR_VERSION

_c_str_dltensor = b"dltensor"
_c_str_used_dltensor = b"used_dltensor"


class DLDeviceType(ctypes.c_int):
    """The enum that encodes the type of the device where
    DLTensor memory is allocated.
    """

    kDLCPU = 1
    kDLCUDA = 2
    kDLCUDAHost = 3
    kDLOpenCL = 4
    kDLVulkan = 7
    kDLMetal = 8
    kDLVPI = 9
    kDLROCM = 10
    kDLROCMHost = 11
    kDLExtDev = 12
    kDLCUDAManaged = 13
    kDLOneAPI = 14
    kDLWebGPU = 15
    kDLHexagon = 16
    kDLMAIA = 17
    kDLTrn = 18

    def __str__(self):
        return {
            self.kDLCPU: "CPU",
            self.kDLCUDA: "CUDA",
            self.kDLCUDAHost: "CUDAHost",
            self.kDLOpenCL: "OpenCL",
            self.kDLVulkan: "Vulkan",
            self.kDLMetal: "Metal",
            self.kDLVPI: "VPI",
            self.kDLROCM: "ROCM",
            self.kDLROCMHost: "ROCMHost",
            self.kDLExtDev: "ExtDev",
            self.kDLCUDAManaged: "CUDAManaged",
            self.kDLOneAPI: "OneAPI",
            self.kDLWebGPU: "WebGPU",
            self.kDLHexagon: "Hexagon",
            self.kDLMAIA: "MAIA",
            self.kDLTrn: "Trainium",
        }.get(self.value, f"Device{self.value}")


class DLDataTypeCode(ctypes.c_uint8):
    """An integer that encodes the category of DLTensor elements' data type."""

    kDLInt = 0
    kDLUInt = 1
    kDLFloat = 2
    kDLOpaqueHandle = 3
    kDLBfloat = 4
    kDLComplex = 5
    kDLBool = 6
    # FP8 types (DLPack 1.x)
    kDLFloat8_e3m4 = 7
    kDLFloat8_e4m3 = 8
    kDLFloat8_e4m3b11fnuz = 9
    kDLFloat8_e4m3fn = 10
    kDLFloat8_e4m3fnuz = 11
    kDLFloat8_e5m2 = 12
    kDLFloat8_e5m2fnuz = 13
    kDLFloat8_e8m0fnu = 14
    # FP6 types
    kDLFloat6_e2m3fn = 15
    kDLFloat6_e3m2fn = 16
    # FP4 types
    kDLFloat4_e2m1fn = 17

    def __str__(self):
        return {
            self.kDLInt: "int",
            self.kDLUInt: "uint",
            self.kDLFloat: "float",
            self.kDLOpaqueHandle: "void_p",
            self.kDLBfloat: "bfloat",
            self.kDLComplex: "complex",
            self.kDLBool: "bool",
        }.get(self.value, f"type{self.value}")


class DLDevice(ctypes.Structure):
    """Represents the device where DLTensor memory is allocated."""

    _fields_ = [
        ("device_type", DLDeviceType),
        ("device_id", ctypes.c_int32),
    ]

    def __str__(self) -> str:
        if self.device_id != 0:
            return f"{self.device_type}:{self.device_id}"
        return str(self.device_type)


class DLDataType(ctypes.Structure):
    """Descriptor of data type for elements of DLTensor."""

    _fields_ = [
        ("code", DLDataTypeCode),
        ("bits", ctypes.c_uint8),
        ("lanes", ctypes.c_uint16),
    ]

    TYPE_MAP = {
        "int8": (DLDataTypeCode.kDLInt, 8, 1),
        "int16": (DLDataTypeCode.kDLInt, 16, 1),
        "int32": (DLDataTypeCode.kDLInt, 32, 1),
        "int64": (DLDataTypeCode.kDLInt, 64, 1),
        "uint8": (DLDataTypeCode.kDLUInt, 8, 1),
        "uint16": (DLDataTypeCode.kDLUInt, 16, 1),
        "uint32": (DLDataTypeCode.kDLUInt, 32, 1),
        "uint64": (DLDataTypeCode.kDLUInt, 64, 1),
        "float16": (DLDataTypeCode.kDLFloat, 16, 1),
        "float32": (DLDataTypeCode.kDLFloat, 32, 1),
        "float64": (DLDataTypeCode.kDLFloat, 64, 1),
        "bfloat16": (DLDataTypeCode.kDLBfloat, 16, 1),
        # Multi-lane types (for images)
        "uint8x4": (DLDataTypeCode.kDLUInt, 8, 4),
        "float32x4": (DLDataTypeCode.kDLFloat, 32, 4),
    }

    def __str__(self) -> str:
        # Try reverse lookup in TYPE_MAP
        for name, (code_val, bits, lanes) in self.TYPE_MAP.items():
            if self.code == code_val and self.bits == bits and self.lanes == lanes:
                return name
        # Fallback
        if self.lanes > 1:
            return f"{self.code}{self.bits}x{self.lanes}"
        return f"{self.code}{self.bits}"


class DLTensor(ctypes.Structure):
    """Plain C Tensor object, does not manage memory."""

    _fields_ = [
        ("data", ctypes.c_void_p),
        ("device", DLDevice),
        ("ndim", ctypes.c_int32),
        ("dtype", DLDataType),
        ("shape", ctypes.POINTER(ctypes.c_int64)),
        ("strides", ctypes.POINTER(ctypes.c_int64)),
        ("byte_offset", ctypes.c_uint64),
    ]


class DLManagedTensor(ctypes.Structure):
    """C structure for managed DLPack tensor."""

    _fields_ = [
        ("dl_tensor", DLTensor),
        ("manager_ctx", ctypes.c_void_p),
        ("deleter", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
    ]


# Python C API bindings for capsule protocol
PyMem_RawMalloc = ctypes.pythonapi.PyMem_RawMalloc
PyMem_RawMalloc.argtypes = [ctypes.c_size_t]
PyMem_RawMalloc.restype = ctypes.c_void_p

PyMem_RawFree = ctypes.pythonapi.PyMem_RawFree
PyMem_RawFree.argtypes = [ctypes.c_void_p]
PyMem_RawFree.restype = None

Py_IncRef = ctypes.pythonapi.Py_IncRef
Py_IncRef.argtypes = [ctypes.py_object]
Py_IncRef.restype = None

Py_DecRef = ctypes.pythonapi.Py_DecRef
Py_DecRef.argtypes = [ctypes.py_object]
Py_DecRef.restype = None

PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.c_void_p)

PyCapsule_New = ctypes.pythonapi.PyCapsule_New
PyCapsule_New.argtypes = [ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor]
PyCapsule_New.restype = ctypes.py_object

PyCapsule_IsValid = ctypes.pythonapi.PyCapsule_IsValid
PyCapsule_IsValid.argtypes = [ctypes.py_object, ctypes.c_char_p]
PyCapsule_IsValid.restype = ctypes.c_int

PyCapsule_GetPointer = ctypes.pythonapi.PyCapsule_GetPointer
PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
PyCapsule_GetPointer.restype = ctypes.c_void_p


def _to_dlpack_capsule(dl_tensor: DLTensor, manager_ctx: Any, deleter_callback: Optional[Callable]) -> Any:
    """Create DLPack capsule from DLTensor (CPU-only, copies shape metadata).

    Per DLPack spec: Capsule named "dltensor", consumer (NumPy) renames to
    "used_dltensor" after extraction. Capsule destructor checks name and calls
    deleter only if unconsumed (prevents double-free).
    """

    # Handle multi-lane types (e.g. RGBA with lanes=4) by expanding to extra dimension
    actual_ndim = dl_tensor.ndim + (1 if dl_tensor.dtype.lanes > 1 else 0)

    # Allocate DLManagedTensor + shape array in one block
    managed_size = ctypes.sizeof(DLManagedTensor)
    shape_size = actual_ndim * ctypes.sizeof(ctypes.c_int64)
    total_size = managed_size + shape_size

    mem_ptr = PyMem_RawMalloc(total_size)
    if not mem_ptr:
        raise MemoryError("Failed to allocate DLManagedTensor")

    managed_tensor = DLManagedTensor.from_address(mem_ptr)

    # Copy DLTensor fields (shallow copy - pointers reference C memory)
    managed_tensor.dl_tensor.data = dl_tensor.data
    managed_tensor.dl_tensor.device = dl_tensor.device
    managed_tensor.dl_tensor.ndim = actual_ndim
    managed_tensor.dl_tensor.byte_offset = dl_tensor.byte_offset

    # Copy dtype, adjusting lanes if expanded
    managed_tensor.dl_tensor.dtype.code = dl_tensor.dtype.code
    managed_tensor.dl_tensor.dtype.bits = dl_tensor.dtype.bits
    managed_tensor.dl_tensor.dtype.lanes = 1 if dl_tensor.dtype.lanes > 1 else dl_tensor.dtype.lanes

    # Copy shape array for safety
    shape_ptr = ctypes.cast(mem_ptr + managed_size, ctypes.POINTER(ctypes.c_int64))
    for i in range(dl_tensor.ndim):
        shape_ptr[i] = dl_tensor.shape[i]

    # Add lanes as extra dimension if multi-lane
    if dl_tensor.dtype.lanes > 1:
        shape_ptr[dl_tensor.ndim] = dl_tensor.dtype.lanes

    managed_tensor.dl_tensor.shape = shape_ptr

    # CPU tensors are typically contiguous
    managed_tensor.dl_tensor.strides = None

    # Keep Python context alive
    managed_tensor.manager_ctx = id(manager_ctx)
    Py_IncRef(manager_ctx)

    # C deleter callback (keep reference to prevent GC)
    @ctypes.CFUNCTYPE(None, ctypes.c_void_p)
    def c_deleter(managed_ptr):
        mt = DLManagedTensor.from_address(managed_ptr)
        ctx = ctypes.cast(mt.manager_ctx, ctypes.py_object).value

        if deleter_callback is not None:
            deleter_callback(ctx)

        Py_DecRef(ctx)
        PyMem_RawFree(managed_ptr)

    managed_tensor.deleter = c_deleter

    # Capsule destructor (keep reference to prevent GC)
    @PyCapsule_Destructor
    def capsule_destructor(capsule_ptr):
        capsule = ctypes.cast(capsule_ptr, ctypes.py_object)
        if PyCapsule_IsValid(capsule, _c_str_dltensor):
            managed_ptr = PyCapsule_GetPointer(capsule, _c_str_dltensor)
            mt = DLManagedTensor.from_address(managed_ptr)
            if mt.deleter:
                mt.deleter(managed_ptr)

    capsule = PyCapsule_New(mem_ptr, _c_str_dltensor, capsule_destructor)

    # Keep callback references alive by attaching to manager_ctx
    if not hasattr(manager_ctx, "_dlpack_callbacks"):
        manager_ctx._dlpack_callbacks = []
    manager_ctx._dlpack_callbacks.append((c_deleter, capsule_destructor))

    return capsule


class ManagedDLTensor:
    """Managed DLPack tensor wrapper (CPU-only)."""

    def __init__(self, dl_tensor: DLTensor, manager_ctx: Any, deleter_callback: Optional[Callable] = None):
        self._dl_tensor = dl_tensor
        self._manager_ctx = manager_ctx
        self._deleter_callback = deleter_callback
        self._cleanup_done = False

    @property
    def shape(self) -> tuple[int, ...]:
        """Shape as Python tuple."""
        return tuple(self._dl_tensor.shape[i] for i in range(self._dl_tensor.ndim))

    @property
    def ndim(self) -> int:
        """Number of dimensions."""
        return self._dl_tensor.ndim

    @property
    def dtype(self):
        """Data type descriptor."""
        return self._dl_tensor.dtype

    @property
    def data(self) -> int:
        """Data pointer address."""
        return self._dl_tensor.data

    @property
    def device(self):
        """Device info."""
        return self._dl_tensor.device

    @property
    def raw_dltensor(self) -> DLTensor:
        """Access underlying DLTensor (advanced use)."""
        return self._dl_tensor

    def to_bytes(self) -> bytes:
        """Get pixel data as bytes (creates copy)."""
        size = self._calculate_byte_size()
        buffer = (ctypes.c_uint8 * size).from_address(self.data)
        return bytes(buffer)

    def _calculate_byte_size(self) -> int:
        """Calculate total buffer size in bytes."""
        total_elements = 1
        for dim in self.shape:
            total_elements *= dim
        bytes_per_element = (self.dtype.bits // 8) * self.dtype.lanes
        return total_elements * bytes_per_element

    def __dlpack_device__(self) -> tuple[int, int]:
        """Return (device_type, device_id) tuple."""
        return (self._dl_tensor.device.device_type.value, self._dl_tensor.device.device_id)

    def __dlpack__(self, stream=None) -> Any:
        """Create DLPack capsule for NumPy interop."""
        return _to_dlpack_capsule(self._dl_tensor, self._manager_ctx, self._deleter_callback)

    def __del__(self):
        """Call cleanup callback on destruction."""
        if not self._cleanup_done and self._deleter_callback is not None:
            try:
                self._deleter_callback(self._manager_ctx)
                self._cleanup_done = True
            except Exception:
                pass

    def __repr__(self) -> str:
        return f"ManagedDLTensor(shape={self.shape}, dtype={self.dtype}, device={self.device})"
