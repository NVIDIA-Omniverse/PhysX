# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Numpy frontend for data-oriented interface"""

import numpy as np

import omni.physics.tensors

from .frontend_base import *


class FrontendNumpy(FrontendBase):

    DTYPE_TO_NUMPY = dict(
        [
            (omni.physics.tensors.float32, np.float32),
            (omni.physics.tensors.float64, np.float64),
            (omni.physics.tensors.int8, np.int8),
            (omni.physics.tensors.int16, np.int16),
            (omni.physics.tensors.int32, np.int32),
            (omni.physics.tensors.int64, np.int64),
            (omni.physics.tensors.uint8, np.uint8),
            (omni.physics.tensors.uint16, np.uint16),
            (omni.physics.tensors.uint32, np.uint32),
            (omni.physics.tensors.uint64, np.uint64),
        ]
    )

    DTYPE_FROM_NUMPY = dict(
        [
            (np.float32, omni.physics.tensors.float32),
            (np.float64, omni.physics.tensors.float64),
            (np.int8, omni.physics.tensors.int8),
            (np.int16, omni.physics.tensors.int16),
            (np.int32, omni.physics.tensors.int32),
            (np.int64, omni.physics.tensors.int64),
            (np.uint8, omni.physics.tensors.uint8),
            (np.uint16, omni.physics.tensors.uint16),
            (np.uint32, omni.physics.tensors.uint32),
            (np.uint64, omni.physics.tensors.uint64),
        ]
    )

    def __init__(self):
        super().__init__()
        self.device = "cpu"

    def create_tensor(self, shape, dtype, device=None):
        np_dtype = FrontendNumpy.DTYPE_TO_NUMPY[dtype]
        tensor = np.zeros(shape, dtype=np_dtype)
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = dtype
        desc.shape = shape
        # desc.data_address = tensor.__array_interface__["data"][0]
        desc.data_address = tensor.ctypes.data
        desc.device = -1
        return tensor, desc

    def get_tensor_desc(self, tensor):
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = FrontendNumpy.DTYPE_FROM_NUMPY[tensor.dtype.type]
        desc.shape = tensor.shape
        # desc.data_address = tensor.__array_interface__["data"][0]
        desc.data_address = tensor.ctypes.data
        desc.device = -1
        return desc

    def as_contiguous_float32(self, tensor):
        return np.ascontiguousarray(tensor, dtype=np.float32)

    def as_contiguous_uint32(self, tensor):
        return np.ascontiguousarray(tensor, dtype=np.uint32)

    def as_contiguous_uint8(self, tensor):
        return np.ascontiguousarray(tensor, dtype=np.uint8)
