# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import ctypes
from typing import Any

__all__: list[str]

DLPACK_VERSION: int

class DLDeviceType(ctypes.c_int):
    kDLCPU: int
    kDLCUDA: int
    kDLCUDAHost: int
    kDLOpenCL: int
    kDLVulkan: int
    kDLMetal: int
    kDLVPI: int
    kDLROCM: int
    kDLROCMHost: int
    kDLExtDev: int
    kDLCUDAManaged: int
    kDLOneAPI: int
    kDLWebGPU: int
    kDLHexagon: int
    kDLMAIA: int
    kDLTrn: int

class DLDataTypeCode(ctypes.c_uint8):
    kDLInt: int
    kDLUInt: int
    kDLFloat: int
    kDLOpaqueHandle: int
    kDLBfloat: int
    kDLComplex: int
    kDLBool: int
    def __int__(self) -> int: ...

class DLDevice(ctypes.Structure):
    device_type: DLDeviceType
    device_id: int

class DLDataType(ctypes.Structure):
    code: DLDataTypeCode
    bits: int
    lanes: int
    TYPE_MAP: dict[str, tuple[DLDataTypeCode, int, int]]

class DLTensor(ctypes.Structure):
    data: int
    device: DLDevice
    ndim: int
    dtype: DLDataType
    shape: Any
    strides: Any
    byte_offset: int

class DLManagedTensor(ctypes.Structure):
    dl_tensor: DLTensor
    manager_ctx: int
    deleter: Any
