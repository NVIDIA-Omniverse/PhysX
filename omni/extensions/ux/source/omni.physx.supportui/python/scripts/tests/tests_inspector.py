# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.usd
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema
from omni.physxtests import utils
import omni.ui as ui
import omni.kit.ui_test as ui_test
from omni.physxtestsvisual.utils import TestCase
from omni.kit.viewport.utility import frame_viewport_selection, get_active_viewport
from ..utils import ui_wait
from omni.physxtests.utils.physicsBase import TestCategory
import omni.kit.window.property as kit_window_property
import unittest
from omni.physx.scripts.assets_paths import AssetFolders

DOCKED_TEST_WIDTH = 790
DOCKED_TEST_HEIGHT = 600


class PhysicsInspectorTests(TestCase):
    category = TestCategory.Core

    def _get_asset_path(self, asset: str):
        return self.asset_paths[asset]

    async def setUp(self):
        await super().setUp()
        self.asset_paths = {
            "franka": utils.get_tests_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Franka/franka_alt_fingers.usd"),
        }
        self._stage = await utils.new_stage_setup(False)
        self._settings = carb.settings.get_settings()
        self._prev_inspector_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED)

    async def tearDown(self):
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, self._prev_inspector_enabled)
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
        self._stage = None
        await utils.new_stage_setup()

    async def test_inspector_basic(self):
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, False)
        self._create_articulation("/World", self._stage)
        scene: UsdPhysics.Scene = UsdPhysics.Scene.Define(self._stage, "/physics_scene")
        scene.CreateGravityMagnitudeAttr().Set(0.0)
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, True)
        await ui_wait(10)
        inspector_window = await self._select_and_focus_inspector_window("/World", 1)
        slider_path = "//Frame/VStack[0]/ZStack[0]/VStack[1]/ScrollingFrame[0]/ZStack[0]/TreeView[0]/ZStack[1]/FloatSlider[0]"
        slider = ui_test.find(f"{inspector_window.title}{slider_path}")
        await ui_test.emulate_mouse_move_and_click(slider.position + ui_test.Vec2(2, 10))

        joint_prim: Usd.Prim = self._stage.GetPrimAtPath("/World/articulation/revoluteJoint")
        joint_state: PhysxSchema.JointStateAPI = PhysxSchema.JointStateAPI.Get(joint_prim, "angular")
        joint_value = joint_state.GetPositionAttr().Get()
        self.assertAlmostEqual(joint_value, -90.0, delta=0.01)
        self._viewport_focus_selection()
        await ui_wait(20)
        await self.setup_viewport_test(1000, 800)
        await ui_wait(20)
        all_tests_passed = True
        all_tests_passed = all_tests_passed and await self.do_visual_test(
            img_name="",
            img_suffix="test_inspector_basic",
            use_distant_light=True,
            skip_assert=True,
            threshold=0.0025
        )

        # Commit inspector changes
        button_path = "//Frame/VStack[0]/HStack[0]/Button[4]"
        commit_button = ui_test.find(f"{inspector_window.title}{button_path}")
        await commit_button.click()
        await ui_wait(10)

        # Enter regular simulation mode
        await self.step(num_steps=1)

        # Move the joint somewhere using the inspector
        await ui_test.emulate_mouse_move_and_click(slider.position + ui_test.Vec2(20, 10))
        await self.step(num_steps=30)
        all_tests_passed = all_tests_passed and await self.do_visual_test(
            img_name="",
            img_suffix="test_inspector_basic_simulation",
            use_distant_light=True,
            skip_assert=True,
            threshold=0.0025
        )

        # Exit simulation mode
        await self.step(num_steps = 0, stop_timeline_after=True)

        # Now we should be back to the same state as just before entering simulation
        all_tests_passed = all_tests_passed and await self.do_visual_test(
            img_name="",
            img_suffix="test_inspector_basic_simulation_after",
            use_distant_light=True,
            skip_assert=True,
            threshold=0.0025
        )

        self.assertTrue(all_tests_passed)
        await self.new_stage()

    def _viewport_focus_selection(self):
        active_viewport = get_active_viewport()
        if active_viewport:
            frame_viewport_selection(active_viewport)

    async def _select_and_focus_inspector_window(self, path, index) -> ui.Window:
        # select prim and focus property window so the widgets are built
        omni.usd.get_context().get_selection().set_selected_prim_paths([path], False)
        await ui_wait(10)
        window = ui.Workspace.get_window(f"Physics Inspector: ###PhysicsInspector{index}")
        button_path = "//Frame/VStack[0]/HStack[0]/Button[2]"
        selection_button = ui_test.find(f"{window.title}{button_path}")
        await selection_button.click()
        await ui_wait(10)
        return window

    def _create_articulation(self, defaultPrimPath: str, stage: Usd.Stage):
        articulationPath = defaultPrimPath + "/articulation"
        UsdGeom.Xform.Define(stage, articulationPath)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))

        # box0 static
        boxActorPath = articulationPath + "/box0"

        position = Gf.Vec3f(0.0, 0.0, 1000.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(1.0, 0.0, 0.0)
        size = 100.0
        scale = Gf.Vec3f(0.1, 1.0, 0.1)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        # Box1
        boxActorPath = articulationPath + "/box1"

        size = 100.0
        position = Gf.Vec3f(0.0, 120.0, 1000.0)
        color = Gf.Vec3f(0.0, 0.0, 1.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        scale = Gf.Vec3f(0.1, 1.0, 0.1)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

        # fixed root joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, articulationPath + "/rootJoint")
        fixedJoint.CreateBody1Rel().SetTargets([Sdf.Path(articulationPath + "/box0")])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # revolute joint
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, articulationPath + "/revoluteJoint")

        revoluteJoint.CreateAxisAttr("X")
        revoluteJoint.CreateLowerLimitAttr(-90.0)
        revoluteJoint.CreateUpperLimitAttr(90)

        revoluteJoint.CreateBody0Rel().SetTargets([articulationPath + "/box0"])
        revoluteJoint.CreateBody1Rel().SetTargets([articulationPath + "/box1"])

        revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        jointStateAPI = PhysxSchema.JointStateAPI.Apply(revoluteJoint.GetPrim(), "angular")
        jointStateAPI.CreatePositionAttr().Set(45.0)
        jointStateAPI.CreateVelocityAttr().Set(0.0)

    async def test_inspector_window_multiple_articulations(self):
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, False)
        root_prim = Sdf.Path("/World")
        franka_path = root_prim.AppendPath("franka")
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        franka_prim: Usd.Prim = self._stage.DefinePrim(franka_path)
        franka_prim.GetReferences().AddReference(self._get_asset_path("franka"))
        joint4_path = franka_prim.GetPath().AppendPath("panda_link3/panda_joint4")
        joint4_prim: UsdPhysics.Joint = UsdPhysics.Joint.Get(self._stage, joint4_path)
        joint4_prim.CreateExcludeFromArticulationAttr().Set(True)
        scene: UsdPhysics.Scene = UsdPhysics.Scene.Define(self._stage, "/World/physics_scene")
        scene.CreateGravityMagnitudeAttr().Set(0.0)
        await utils.wait_for_stage_loading_status(omni.usd.get_context(), 10)
        await ui_wait(20)
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, True)
        await ui_wait(10)
        inspector_window: ui.Window = await self._select_and_focus_inspector_window("/World/franka", 1)
        joint1_slider_path = "//Frame/VStack[0]/ZStack[0]/VStack[1]/ScrollingFrame[0]/ZStack[0]/TreeView[0]/VStack[1]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/TreeView[0]/ZStack[1]/FloatSlider[0]"
        # joint4_slider_path = "//Frame/VStack[0]/ZStack[0]/VStack[1]/ScrollingFrame[0]/ZStack[0]/TreeView[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/TreeView[0]/ZStack[1]/FloatSlider[0]"
        slider = ui_test.find(f"{inspector_window.title}{joint1_slider_path}")
        # this articulation is not stable, so we get spurious values for the changed values.
        # For some reason also omni.ui updates the window graphics only after a resize / scrollbar event,
        # that's why we setup the docked test in advance
        await self.setup_docked_test(inspector_window.title, "Physics Debug", DOCKED_TEST_WIDTH, DOCKED_TEST_HEIGHT)
        await self.wait(3)
        await ui_test.emulate_mouse_move_and_click(slider.position + ui_test.Vec2(2, 10))
        await ui_wait(500)  # Wait for inspector simulation to stop
        await self._test_physics_inspector_window(img_suffix="_multiple_articulations",
                                                  inspector_window=inspector_window)

    async def _test_physics_inspector_window(self, img_suffix, inspector_window: ui.Window, window_restore="Physics Debug"):
        window_name = inspector_window.title
        # resize window so that all components can fit without scrolling to get proper computed_height
        # (some invisible widgets are not counted towards the height)
        await self.setup_docked_test(window_name, window_restore, DOCKED_TEST_WIDTH, DOCKED_TEST_HEIGHT)
        await self.wait(3)
        # get target height and scroll offset and do a quickresize
        target_height = inspector_window.frame.computed_height
        target_scroll = inspector_window.frame.screen_position_y

        self.force_resize_window(window_name, DOCKED_TEST_WIDTH, target_height)
        await self.wait(3)

        # and scroll and waaait for it to stabilize
        kit_window_property.get_window()._window_frame.vertical_scrollbar_policy = ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF
        kit_window_property.get_window()._window_frame.scroll_y = target_scroll
        await self.wait(20)

        # resize to correct width, this will refresh the scrollbar being off even with no-window
        self.force_resize_window(window_name, 800, target_height)
        await self.wait(20)

        return await self.do_visual_test(img_suffix=img_suffix)
