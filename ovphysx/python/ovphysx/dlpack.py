# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""DLPack tensor structures for zero-copy data interchange.

This module provides ctypes wrappers for the vendored DLPack C header, enabling
efficient data sharing between the C library and Python without copying.

NOTE: This file is NOT copied from another repo. It is a hand-written Python/ctypes
mirror of the C structs defined in ovphysx/dlpack/dlpack.h (which itself is the
upstream header from https://github.com/dmlc/dlpack). When the vendored C header
is updated, this file must be updated to match.
"""

import ctypes
from numbers import Integral

__all__ = [
    "DLDeviceType",
    "DLDataTypeCode",
    "DLDevice",
    "DLDataType",
    "DLTensor",
    "DLManagedTensor",
    "DLPACK_VERSION",
]

# DLPack version 1.3
DLPACK_MAJOR_VERSION = 1
DLPACK_MINOR_VERSION = 3
# Legacy compat
DLPACK_VERSION = (DLPACK_MAJOR_VERSION << 8) | DLPACK_MINOR_VERSION


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

    def __int__(self):
        return self.value

    def __eq__(self, other):
        other_value = getattr(other, "value", other)
        if not isinstance(other_value, Integral):
            return NotImplemented
        return self.value == int(other_value)

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return NotImplemented
        return not result

    def __hash__(self):
        return hash(self.value)

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
        # Reverse lookup in TYPE_MAP.
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



# Python C API bindings for reading DLPack capsules (see _dlpack_utils).
PyCapsule_IsValid = ctypes.pythonapi.PyCapsule_IsValid
PyCapsule_IsValid.argtypes = [ctypes.py_object, ctypes.c_char_p]
PyCapsule_IsValid.restype = ctypes.c_int

PyCapsule_GetPointer = ctypes.pythonapi.PyCapsule_GetPointer
PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
PyCapsule_GetPointer.restype = ctypes.c_void_p
