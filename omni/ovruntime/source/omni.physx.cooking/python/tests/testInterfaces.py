# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import omni.kit.test


class TestPhysxCookingInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_get_physx_cooking_service_private_interface(self):
        from omni.physxcooking import get_physx_cooking_service_private_interface
        iface = get_physx_cooking_service_private_interface()
        self.assertIsNotNone(iface)
