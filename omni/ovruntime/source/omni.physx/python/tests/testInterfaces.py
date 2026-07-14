# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test


class TestPhysxInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_get_physx_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_simulation_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_simulation_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_cooking_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_cooking_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_scene_query_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_scene_query_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_visualization_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_visualization_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_stage_update_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_stage_update_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_statistics_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_statistics_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_replicator_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_replicator_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_benchmarks_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_benchmarks_interface()
        self.assertIsNotNone(iface)
