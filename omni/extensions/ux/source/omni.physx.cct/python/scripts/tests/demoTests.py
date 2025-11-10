# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.physxdemos as demo


class PhysXCctDemoTests(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_cct_arena(self):
        async def do_steps(scene_class):
            print(scene_class)
            print(scene_class.__module__)
            await self.wait(60)
            await self.step(20)

        await demo.test(
            "omni.physxcct.scripts.scenes.CharacterControllerDemo",
            after_stage_async_fn=do_steps,
            params={
                "omni.physxcct.scripts.scenes.CharacterControllerDemo":
                [
                    {"first_person": False},
                    {"first_person": True}
                ]
            }
        )

    async def test_cct_bridges(self):
        async def do_steps(scene_class):
            print(scene_class)
            print(scene_class.__module__)
            await self.wait(60)
            await self.step(20)

        await demo.test("omni.physxcct.scripts.scenes.BridgesDemo", after_stage_async_fn=do_steps)
