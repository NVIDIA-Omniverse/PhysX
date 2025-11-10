# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pathlib import Path
from omni.physx.scripts import physicsUtils
from omni.physx.scripts import utils as baseUtils
from omni.physxtests import utils
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import carb.input
import carb.tokens
from omni.physx import get_physx_visualization_interface
from omni.physxui import get_physxui_interface
from omni.kit.viewport.utility import frame_viewport_selection
import omni.kit.app


class PhysxVisualTest(TestCase):
    category = TestCategory.Core

    async def test_physics_visual_simulation_inverse(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))
        utils.physics_scene_setup(stage)

        await self.step(100)
        await self.do_visual_test(inverse=True)

    async def test_physics_visual_simulation_precise(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))
        utils.physics_scene_setup(stage)

        await self.step(20, precise=True)
        await self.do_visual_test()
        await self.new_stage()

    async def test_physics_visual_simulation_buggy(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        default_prim_path = str(stage.GetDefaultPrim().GetPath())
        # add base scene
		# build relative data path
        filePath = Path(__file__).parent.parent.parent.joinpath("data/BuggyVisualTest.usd")

        abspath = carb.tokens.get_tokens_interface().resolve(str(filePath))
        #CarRoot corresponds to top level node in the test file
        stage.DefinePrim(default_prim_path + "/Buggy").GetReferences().AddReference(abspath, "/CarRoot")

        utils.physics_scene_setup(stage, Gf.Vec3f(0.0, -98.0, 0.0))

        frame_viewport_selection()

        await self.step(20, precise=True)
        await self.do_visual_test(1e-3)
        await self.new_stage()

    async def test_physics_visual_simulation_stable_state(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        up_axis = UsdGeom.GetStageUpAxis(stage)
        up_offset = baseUtils.getAxisAlignedVector(up_axis, 25)
        physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(10.0), up_offset)
        physicsUtils.add_ground_plane(stage, "/groundPlane", up_axis, 50, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        utils.physics_scene_setup(stage)

        await self.step(50)
        await self.do_visual_test()
        await self.new_stage()

    async def test_physics_visual_debug_draw_articulation(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)

        # create articulation link
        size = Gf.Vec3f(25.0)
        position = Gf.Vec3f(0.0)
        cube0 = physicsUtils.add_rigid_cube(stage, "/cubeActor0", size, position)
        position = Gf.Vec3f(0.0, 100.0, 0.0)
        cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

        fixedBaseJoint = UsdPhysics.FixedJoint.Define(self._stage, "/rootJoint")
        val1 = [cube0.GetPrim().GetPath()]
        fixedBaseJoint.CreateBody1Rel().SetTargets(val1)
        UsdPhysics.ArticulationRootAPI.Apply(fixedBaseJoint.GetPrim())

        revoluteJoint = UsdPhysics.RevoluteJoint.Define(self._stage, "/revoluteJoint")
        val1 = [cube0.GetPrim().GetPath()]
        val2 = [cube1.GetPrim().GetPath()]
        revoluteJoint.CreateBody0Rel().SetTargets(val1)
        revoluteJoint.CreateBody1Rel().SetTargets(val2)
        revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 2.0, 0.0))
        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -2.0, 0.0))
        await self.step(1)

        get_physxui_interface().enable_debug_visualization(True)
        get_physx_visualization_interface().enable_visualization(True)
        get_physx_visualization_interface().set_visualization_parameter("CollisionShapes", True)

        await self.step(50)

        await self.do_visual_test()        
        get_physx_visualization_interface().set_visualization_parameter("CollisionShapes", False)
        get_physx_visualization_interface().enable_visualization(False)
        get_physxui_interface().enable_debug_visualization(False)
        await self.new_stage()
