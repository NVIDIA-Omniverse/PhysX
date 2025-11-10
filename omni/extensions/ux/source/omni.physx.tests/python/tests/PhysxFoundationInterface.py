# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.test import AsyncTestCase
from omni.physxfoundation import get_physx_foundation_interface


class PhysxFoundationInterfaceTest(AsyncTestCase):
    async def test_cuda_device_check(self):
        iface = get_physx_foundation_interface()
        print(iface)
        self.assertTrue(iface.cuda_device_check())
        iface = None
