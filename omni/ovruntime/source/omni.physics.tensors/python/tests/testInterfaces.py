# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test


class TestPhysicsTensorsInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_acquire_tensor_api(self):
        from omni.physics.tensors import acquire_tensor_api
        from omni.physics.tensors import SimulationView

        iface = acquire_tensor_api()
        self.assertIsNotNone(iface)
