# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
import omni.kit.viewport.utility as viewport_utils
import omni.kit.ui_test as ui_test
import omni.ui as ui
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import carb.tokens
import omni.kit.app
from omni.physx.bindings._physx import (
    SETTING_DISPLAY_JOINTS,
)


class PhysxVisualJointsTest(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._viewport_settings[SETTING_DISPLAY_JOINTS] = (True, False)
        self._viewport_settings["/persistent/app/viewport/gizmo/constantScale"] = (30, 30)
        # Default to move mode to further continuously test that our transform gizmo is appearing properly.
        self._viewport_settings["/app/window/hideUi"] = (True, False)
        self._viewport_settings["/app/transform/operation"] = ("move", "move")

    def create_cubes(self, stage):
        size = Gf.Vec3f(25.0)
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        cube0 = physicsUtils.add_collider_cube(stage, "/cubeActor0", size, position)
        position = Gf.Vec3f(0.0, 100.0, 0.0)
        cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

        self.body0_cube = cube0.GetPrim()
        self.body1_cube = cube1.GetPrim()

    def create_xforms(self, stage):
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        xform0 = physicsUtils.add_xform(stage, "/xformActor0", position)
        position = Gf.Vec3f(0.0, 100.0, 0.0)
        xform1 = physicsUtils.add_rigid_xform(stage, "/xformActor1", position)

        self.static_xform = xform0.GetPrim()
        self.rb_xform = xform1.GetPrim()

    def attach_to_cubes(self):
        self.joint.GetBody0Rel().SetTargets([self.body0_cube.GetPath()])
        self.joint.GetBody1Rel().SetTargets([self.body1_cube.GetPath()])

    def attach_to_xforms(self):
        self.joint.GetBody0Rel().SetTargets([self.static_xform.GetPath()])
        self.joint.GetBody1Rel().SetTargets([self.rb_xform.GetPath()])

    def detach(self):
        self.joint.GetBody0Rel().SetTargets([])
        self.joint.GetBody1Rel().SetTargets([])

    def create_joint(self, stage, joint_type):

        path = "/World/"+joint_type+"Joint"

        if joint_type == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(self._stage, path)
            joint.CreateLowerLimitAttr().Set(-45)
            joint.CreateUpperLimitAttr().Set(90)
            joint.CreateAxisAttr().Set(UsdGeom.Tokens.x)
        elif joint_type == "prismatic":
            joint = UsdPhysics.PrismaticJoint.Define(self._stage, path)
            joint.CreateLowerLimitAttr().Set(-50)
            joint.CreateUpperLimitAttr().Set(25)
            joint.CreateAxisAttr().Set(UsdGeom.Tokens.y)
        elif joint_type == "fixed":
            joint = UsdPhysics.FixedJoint.Define(self._stage, path)
        elif joint_type == "d6":
            joint = UsdPhysics.Joint.Define(self._stage, path)
            limitAPI = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.transX)
            limitAPI.CreateLowAttr(-20.0)
            limitAPI.CreateHighAttr(20.0)
            limitAPI = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.transZ)
            limitAPI.CreateLowAttr(-20.0)
            limitAPI.CreateHighAttr(40.0)
            limitAPI = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotX)
            limitAPI.CreateLowAttr(-45.0)
            limitAPI.CreateHighAttr(90.0)
        elif joint_type == "distance":
            joint = UsdPhysics.DistanceJoint.Define(self._stage, path)
            joint.CreateMinDistanceAttr().Set(0)
            joint.CreateMaxDistanceAttr().Set(50)
        elif joint_type == "spherical":
            joint = UsdPhysics.SphericalJoint.Define(self._stage, path)
            joint.CreateConeAngle0LimitAttr().Set(30)
            joint.CreateConeAngle1LimitAttr().Set(60)
            joint.CreateAxisAttr().Set(UsdGeom.Tokens.y)
        elif joint_type == "gear":
            joint = PhysxSchema.PhysxPhysicsGearJoint.Define(self._stage, path)
        elif joint_type == "rack":
            joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(self._stage, path)

        self.joint = joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -4.0, 0.0))
        self.attach_to_cubes()

    def setup_joint_scenario(self, stage, joint_type):
        self.create_cubes(stage)
        self.create_joint(stage, joint_type)
        camera_state = ViewportCameraState()
        camera_state.set_position_world(Gf.Vec3d(350.0, 0.0, -75.0), True)
        camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, -75.0), True)

    async def setup_viewport_window(self):
        window = ui.Workspace.get_window("Viewport")
        window.padding_x = 0
        window.padding_y = 0
        window.position_x = 0
        window.position_y = 0
        window.noTabBar = True
        window.flags = (
                        ui.WINDOW_FLAGS_NO_TITLE_BAR |
                        ui.WINDOW_FLAGS_NO_CLOSE |
                        ui.WINDOW_FLAGS_NO_COLLAPSE |
                        ui.WINDOW_FLAGS_NO_MOVE |
                        ui.WINDOW_FLAGS_NO_RESIZE |
                        ui.WINDOW_FLAGS_NO_SCROLLBAR
                        )
        window.auto_resize = False
        window.width = self._viewport_data[0]
        window.height = self._viewport_data[1]
        await self.wait(1)
        viewport_api = viewport_utils.get_viewport_from_window_name("Viewport")
        viewport_api.resolution = (window.width, window.height)
        viewport_api.resolution_scale = 1
        await viewport_utils.next_viewport_frame_async(viewport_api)
        await self.wait(10)

    async def test_physics_visual_joint_selection(self):
        stage = await self.new_stage()

        res = True
        for joint_type in ["revolute", "prismatic", "fixed", "d6", "distance", "spherical", "gear", "rack"]:
            stage = await self.new_stage()
            await self.setup_viewport_test()

            utils.physics_scene_setup(stage)
            self.setup_joint_scenario(stage, joint_type)

            await self.wait(1)

            usdContext = omni.usd.get_context()
            usdContext.get_selection().set_selected_prim_paths([str(self.joint.GetPrim().GetPath())], True)

            await self.wait(20)

            await self.setup_viewport_window()
            res &= await self.do_visual_test(img_suffix=f"_{joint_type}_cube", skip_assert=True, threshold=0.0025, use_renderer_capture=True)

            await self.wait(1)

            self.create_xforms(stage)

            self.attach_to_xforms()

            await self.setup_viewport_test()

            await self.wait(20)

            await self.setup_viewport_window()
            res &= await self.do_visual_test(img_suffix=f"_{joint_type}_xform", skip_assert=True, threshold=0.0035, use_renderer_capture=True)

            await self.setup_viewport_test()

            await self.wait(1)

            self.detach()

            await self.wait(20)

            await self.setup_viewport_window()
            res &= await self.do_visual_test(img_suffix=f"_{joint_type}_detach", skip_assert=True, threshold=0.0025, use_renderer_capture=True)


        self.assertTrue(res)
        await self.new_stage()


    async def test_physics_visual_joint_billboard(self):
        res = True

        for joint_type in ["revolute", "prismatic", "fixed", "d6", "distance", "spherical", "gear", "rack"]:
            stage = await self.new_stage()
            await self.setup_viewport_test()

            utils.physics_scene_setup(stage)
            self.setup_joint_scenario(stage, joint_type)

            await self.setup_viewport_window()
            res &= await self.do_visual_test(img_suffix=f"_{joint_type}", skip_assert=True, threshold=0.0025, use_renderer_capture=True)

        self.assertTrue(res)
        await self.new_stage()

    async def test_physics_visual_joint_billboard_visibility(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)

        self.setup_joint_scenario(stage, "revolute")

        await omni.kit.app.get_app().next_update_async()

        usdContext = omni.usd.get_context()
        usdContext.get_selection().set_selected_prim_paths([], True)

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        imageable = UsdGeom.Imageable(self.joint)
        imageable.MakeInvisible()

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        await self.setup_viewport_window()
        await self.do_visual_test(threshold=0.0025, use_renderer_capture=True)
        await self.new_stage()

    async def test_physics_visual_joint_billboard_parent_visibility(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)

        self.setup_joint_scenario(stage, "revolute")

        await omni.kit.app.get_app().next_update_async()

        usdContext = omni.usd.get_context()
        usdContext.get_selection().set_selected_prim_paths([], True)

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        imageable = UsdGeom.Imageable(self.body0_cube)
        imageable.MakeInvisible()

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        await self.setup_viewport_window()
        await self.do_visual_test(threshold=0.0025, use_renderer_capture=True)
        await self.new_stage()

    async def test_physics_visual_joint_align_workflow(self):
        stage = await self.new_stage()
        self.create_cubes(stage)

        camera_state = ViewportCameraState()
        camera_state.set_position_world(Gf.Vec3d(350.0, 0.0, -75.0), True)
        camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, -75.0), True)
        await self.wait(10)

        async def setup_it(body_id):
            await self.wait(10)
            await self.setup_viewport_test()
            await self.wait(10)

        def break_it():
            self.joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            self.joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        async def fix_it(identifier, text):
            await self.wait(2)
            window = ui.Workspace.get_window("Property")
            window.focus()
            await self.wait(2)

            pos0_widget = ui_test.find_all(f"Property//Frame/**/physprop_physics:{identifier}")[0]
            await pos0_widget.right_click()
            await ui_test.select_context_menu(f"Align {text}", offset=ui_test.Vec2(50, 5))
            omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
            omni.usd.get_context().get_selection().set_selected_prim_paths([str(self.joint.GetPrim().GetPath())], True)            
            await self.wait(10)

        async def test_it(suffix, joint_type):
            await self.setup_viewport_window()
            return await self.do_visual_test(
                img_name=f"joint_align_workflow_{joint_type}",
                img_suffix=f"_{suffix}",
                threshold=0.0025,
                skip_assert=True,
                use_renderer_capture=True
            )

        pos_fix_list = [
            ("localPos0", "Position to Body 0", 1),
            ("localPos1", "Position to Body 1", 0),
        ]

        tm_fix_list = [
            ("localPos0", "Transform to Body 0", 1),
            ("localRot0", "Transform to Body 0", 1),
            ("localPos1", "Transform to Body 1", 0),
            ("localRot1", "Transform to Body 1", 0),
        ]

        result = True

        for joint_type, fix_list in [
                ("revolute", pos_fix_list),
                ("spherical", pos_fix_list),
                ("fixed", tm_fix_list),
                ("prismatic", tm_fix_list),
        ]:
            self.create_joint(stage, joint_type)
            omni.usd.get_context().get_selection().set_selected_prim_paths([str(self.joint.GetPrim().GetPath())], True)

            for identifier, text, select_body_id in fix_list:
                break_it()
                await setup_it(0)
                result &= await test_it(f"broken_{identifier}", joint_type)

                await fix_it(identifier, text)
                await setup_it(select_body_id)
                result &= await test_it(f"fixed_{identifier}", joint_type)

            stage.RemovePrim(self.joint.GetPrim().GetPrimPath())

        await self.new_stage()
        self.assertTrue(result)
