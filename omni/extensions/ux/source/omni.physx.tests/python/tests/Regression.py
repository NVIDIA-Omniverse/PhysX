# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from pxr import Gf, UsdPhysics, PhysxSchema
import carb
import omni.usd
from omni.physx.bindings import _physx as pb
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.kit.commands import execute
import omni.kit.undo
from omni.kit.undo import history


# Regression tests for Jira issues.
class PhysicsRegressionTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Kit
    async def test_regression_OM18152(self):  # Make update velocities to USD set by default
        usd_context = omni.usd.get_context()
        await usd_context.new_stage_async()
        updToUSD = carb.settings.get_settings().get_as_bool(pb.SETTING_UPDATE_VELOCITIES_TO_USD)
        self.assertTrue(updToUSD)

    async def test_regression_OM40411(self):  # Apply presets should not overflow history
        num = 10

        async def process(fn, expected):
            stage = await self.new_stage()
            boxes = [physicsUtils.add_box(stage, "/box") for i in range(0, num)]
            history.clear_history()
            fn(boxes)
            history_len = len(history.get_history())
            print(history_len)
            self.assertTrue(history_len == expected)
            omni.kit.undo.undo()

        def rb_preset(_):
            execute("SetRigidBody", path="/World", approximationShape="convexHull", kinematic=False)

        await process(rb_preset, 1)

        def sc_preset(_):
            execute("SetStaticCollider", path="/World", approximationShape="none")

        await process(sc_preset, 1)

        # all below should at some point conflate, 1 group = 1 history item, that however needs an omni.kit.undo rewrite

        def rb_add_component(boxes):
            omni.kit.undo.begin_group()
            for prim in boxes:
                execute("AddPhysicsComponent", usd_prim=prim, component="PhysicsRigidBodyAPI")
            omni.kit.undo.end_group()

        await process(rb_add_component, num + 1)

        def rb_remove_component(boxes):
            omni.kit.undo.begin_group()
            for prim in boxes:
                execute("RemovePhysicsComponent", usd_prim=prim, component="PhysicsRigidBodyAPI")
            omni.kit.undo.end_group()

        await process(rb_remove_component, num + 1)

        def sc_ad_component(boxes):
            omni.kit.undo.begin_group()
            for prim in boxes:
                execute("AddPhysicsComponent", usd_prim=prim, component="PhysicsCollisionAPI")
            omni.kit.undo.end_group()

        await process(sc_ad_component, num + 1)

        def sc_remove_component(boxes):
            omni.kit.undo.begin_group()
            for prim in boxes:
                execute("RemovePhysicsComponent", usd_prim=prim, component="PhysicsCollisionAPI")
            omni.kit.undo.end_group()

        await process(sc_remove_component, 2 * num + 1)
