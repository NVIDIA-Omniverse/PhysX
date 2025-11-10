# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Warp frontend for data-oriented interface"""

import ctypes
import warp as wp

import omni.physics.tensors
from math import prod

from .frontend_base import *

class FrontendWarp(FrontendBase):

    DTYPE_TO_WARP = dict(
        [
            (omni.physics.tensors.float32, wp.float32),
            (omni.physics.tensors.float64, wp.float64),
            (omni.physics.tensors.int8, wp.int8),
            (omni.physics.tensors.int16, wp.int16),
            (omni.physics.tensors.int32, wp.int32),
            (omni.physics.tensors.int64, wp.int64),
            (omni.physics.tensors.uint8, wp.uint8),
            (omni.physics.tensors.uint16, wp.uint16),
            (omni.physics.tensors.uint32, wp.uint32),
            (omni.physics.tensors.uint64, wp.uint64),
        ]
    )

    DTYPE_FROM_WARP = dict(
        [
            (wp.float32, omni.physics.tensors.float32),
            (wp.float64, omni.physics.tensors.float64),
            (wp.int8, omni.physics.tensors.int8),
            (wp.int16, omni.physics.tensors.int16),
            (wp.int32, omni.physics.tensors.int32),
            (wp.int64, omni.physics.tensors.int64),
            (wp.uint8, omni.physics.tensors.uint8),
            (wp.uint16, omni.physics.tensors.uint16),
            (wp.uint32, omni.physics.tensors.uint32),
            (wp.uint64, omni.physics.tensors.uint64),

            (wp.vec2, omni.physics.tensors.float32),
            (wp.vec3, omni.physics.tensors.float32),
            (wp.vec4, omni.physics.tensors.float32),

            # TODO: handle more warp types...
        ]
    )

    def __init__(self, device_ordinal=-1):
        super().__init__()
        self.device_ordinal = device_ordinal
        self.device = self._parse_device(device_ordinal)
    
    def _parse_device(self, device_ordinal):
        if device_ordinal == -1:
            return "cpu"
        else:
            return f"cuda:{device_ordinal}"

    def _device_from_warp(self, wp_device):
        if wp_device.is_cpu:
            return -1
        elif wp_device.is_cuda:
            return wp_device.ordinal
        else:
            raise Exception("Unrecognized Warp array device:", wp_device)

    def create_tensor(self, shape, dtype, device=None):
        wp_dtype = FrontendWarp.DTYPE_TO_WARP[dtype]
        if device is None:
            tensor = wp.zeros(shape, dtype=wp_dtype, device=self.device)
            device_ordinal = self.device_ordinal
        else:
            api_device = self._parse_device(device)
            tensor = wp.zeros(shape, dtype=wp_dtype, device=api_device)
            device_ordinal = device
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = dtype
        desc.shape = shape
        desc.data_address = 0 if tensor.ptr is None and prod(tuple(tensor.shape))==0 else tensor.ptr
        desc.device = device_ordinal
        return tensor, desc

    def get_tensor_desc(self, tensor):
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = FrontendWarp.DTYPE_FROM_WARP[tensor.dtype]
        type_length = wp.types.type_length(tensor.dtype)
        if type_length > 1:
            desc.shape = tensor.shape + (type_length,)
        else:
            desc.shape = tensor.shape
        desc.data_address = 0 if tensor.ptr is None and prod(tuple(tensor.shape))==0 else tensor.ptr
        desc.device = self._device_from_warp(tensor.device)
        return desc

    def as_contiguous_float32(self, tensor):
        scalar_type = wp.types.type_scalar_type(tensor.dtype)
        if wp.types.type_ctype(tensor.dtype) == ctypes.c_float or wp.types.type_ctype(scalar_type) == ctypes.c_float:
            return tensor
        else:
            raise TypeError("A float32 tensor is required")

    def as_contiguous_uint32(self, tensor):
        ctype = wp.types.type_ctype(tensor.dtype)
        if ctype == ctypes.c_uint32 or ctype == ctypes.c_int32:
            return tensor
        else:
            raise TypeError("An int32 or uint32 tensor is required")

    def as_contiguous_uint8(self, tensor):
        ctype = wp.types.type_ctype(tensor.dtype)
        if ctype == ctypes.c_uint8 or ctype == ctypes.c_int8:
            return tensor
        else:
            raise TypeError("An int8 or uint8 tensor is required")
