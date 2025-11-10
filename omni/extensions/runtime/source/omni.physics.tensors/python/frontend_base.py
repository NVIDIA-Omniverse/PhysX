# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from abc import ABC, abstractmethod


class FrontendBase(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def create_tensor(self, shape, dtype, device=None):
        """Creates tensor"""

    @abstractmethod
    def get_tensor_desc(self, tensor):
        """Returns tensor descriptor"""

    @abstractmethod
    def as_contiguous_float32(self, tensor):
        """Returns a contiguous tensor with float32 dtype"""

    @abstractmethod
    def as_contiguous_uint32(self, tensor):
        """Returns a contiguous tensor with uint32 dtype"""
