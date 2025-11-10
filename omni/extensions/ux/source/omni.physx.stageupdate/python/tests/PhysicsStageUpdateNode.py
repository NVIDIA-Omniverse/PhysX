# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.utils as physxUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from omni.physxtests import utils
from omni.physxstageupdate import get_physx_stage_update_node_interface
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils
import unittest
import carb
import omni.timeline


class PhysicsStageUpdateNodeStage(PhysicsKitStageAsyncTestCase):
    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()

    async def wait_for_stage(self):
        from omni.physx.bindings._physx import SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP
        for _ in range(carb.settings.acquire_settings_interface().get(SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP)):
            await omni.kit.app.get_app().next_update_async()

    def create_simple_scene(self, stage):
        # create rigid body - rootLink
        physicsUtils.add_rigid_box(stage, "/cube", Gf.Vec3f(0.1), Gf.Vec3f(0.0), Gf.Quatf(1.0))

    def create_base_scene(self, stage):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        sphereActorPath = "/sphereActor"

        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 4.0)
        orientation = Gf.Quatf(1.0)

        self.spherePrim = physicsUtils.add_rigid_sphere(stage, sphereActorPath, radius, position, orientation)

    async def test_stage_update_attached_node(self):
        stage = await self.new_stage()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        self.assertTrue(get_physx_stage_update_node_interface().is_node_attached())

        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)

        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        del stage
        await self.wait_for_stage()

    async def test_stage_update_detached_node(self):
        stage = await self.new_stage()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        get_physx_stage_update_node_interface().detach_node()

        self.assertTrue(not get_physx_stage_update_node_interface().is_node_attached())

        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        get_physx_stage_update_node_interface().attach_node()

        del stage
        await self.wait_for_stage()

    async def test_stage_update_reatached_node(self):
        stage = await self.new_stage()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        get_physx_stage_update_node_interface().detach_node()

        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        await omni.kit.app.get_app().next_update_async()

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        get_physx_stage_update_node_interface().attach_node()

        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)

        omni.timeline.get_timeline_interface().stop()

        del stage
        await self.wait_for_stage()
