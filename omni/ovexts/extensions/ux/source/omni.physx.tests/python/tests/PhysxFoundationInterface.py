# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from omni.kit.test import AsyncTestCase
from omni.physxfoundation import get_physx_foundation_interface


class PhysxFoundationInterfaceTest(AsyncTestCase):
    async def test_cuda_device_check(self):
        iface = get_physx_foundation_interface()
        print(iface)
        self.assertTrue(iface.cuda_device_check())
        iface = None
