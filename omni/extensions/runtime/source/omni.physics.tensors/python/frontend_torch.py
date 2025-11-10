# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""PyTorch frontend for data-oriented interface"""

import torch

import omni.physics.tensors

from .frontend_base import *


class FrontendTorch(FrontendBase):

    DTYPE_TO_TORCH = dict(
        [
            (omni.physics.tensors.float32, torch.float32),
            (omni.physics.tensors.float64, torch.float64),
            (omni.physics.tensors.int8, torch.int8),
            (omni.physics.tensors.int16, torch.int16),
            (omni.physics.tensors.int32, torch.int32),
            (omni.physics.tensors.int64, torch.int64),
            (omni.physics.tensors.uint8, torch.uint8),
            (omni.physics.tensors.uint16, torch.int16),  # torch does not support uint16
            (omni.physics.tensors.uint32, torch.int32),  # torch does not support uint32
            (omni.physics.tensors.uint64, torch.int64),  # torch does not support uint64
        ]
    )

    DTYPE_FROM_TORCH = dict(
        [
            (torch.float32, omni.physics.tensors.float32),
            (torch.float64, omni.physics.tensors.float64),
            (torch.int8, omni.physics.tensors.int8),
            (torch.int16, omni.physics.tensors.int16),
            (torch.int32, omni.physics.tensors.int32),
            (torch.int64, omni.physics.tensors.int64),
            (torch.uint8, omni.physics.tensors.uint8),
            # torch does not support uint16, uint32, or uint64
        ]
    )

    def __init__(self, device_ordinal=-1):
        super().__init__()
        self.device_ordinal = device_ordinal
        self.device = self._parse_device(device_ordinal)

    def _parse_device(self, device_ordinal):
        if device_ordinal == -1:
            return torch.device("cpu")
        elif device_ordinal >= 0:
            return torch.device("cuda", device_ordinal)

    def _device_from_frontend(self, torch_device):
        if torch_device.type == "cpu":
            return -1
        elif torch_device.type == "cuda":
            return torch_device.index
        else:
            raise Exception("Unsupported Torch tensor device:", torch_device)

    def create_tensor(self, shape, dtype, device=None):
        torch_dtype = FrontendTorch.DTYPE_TO_TORCH[dtype]
        if device is None:
            tensor = torch.zeros(shape, dtype=torch_dtype, device=self.device)
            device_ordinal = self.device_ordinal
        else:
            api_device = self._parse_device(device)
            tensor = torch.zeros(shape, dtype=torch_dtype, device=api_device)
            device_ordinal = device
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = dtype
        desc.shape = shape
        desc.data_address = tensor.data_ptr()
        desc.device = device_ordinal
        return tensor, desc

    def get_tensor_desc(self, tensor):
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = FrontendTorch.DTYPE_FROM_TORCH[tensor.dtype]
        desc.shape = tuple(tensor.shape)
        desc.data_address = tensor.data_ptr()
        desc.device = self._device_from_frontend(tensor.device)
        return desc

    def as_contiguous_float32(self, tensor):
        return tensor.to(torch.float32).contiguous()

    def as_contiguous_uint32(self, tensor):
        # note: torch does not support uint32
        return tensor.to(torch.int32).contiguous()

    def as_contiguous_uint8(self, tensor):
        return tensor.to(torch.uint8).contiguous()
