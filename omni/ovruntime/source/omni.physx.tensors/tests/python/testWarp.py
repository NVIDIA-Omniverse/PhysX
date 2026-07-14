# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Warp utility tests.

import unittest

import numpy as np
import warp as wp

import _tensors_setup  # noqa: F401

import warp_utils as wp_utils


_HAS_GPU = wp.is_cuda_available()


class PhysxTensorsWarpTests(unittest.TestCase):

    def _test_warp_device(self, wp_device):
        self.assertTrue(wp.is_device_available(wp_device), f"Warp device {wp_device} is not available")
        with wp.ScopedDevice(wp_device):
            n = 10
            a = wp_utils.arange(n, device=wp_device)
            result = a.numpy().squeeze()
            expected = np.arange(n)
            self.assertTrue(np.array_equal(result, expected), "Warp arange() failed")

    def test_warp_cpu(self):
        self._test_warp_device("cpu")

    def test_warp_gpu(self):
        self._test_warp_device("cuda:0")
