# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test


class TestPhysxFabricInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_get_physx_fabric_interface(self):
        from omni.physxfabric import get_physx_fabric_interface
        iface = get_physx_fabric_interface()
        self.assertIsNotNone(iface)
