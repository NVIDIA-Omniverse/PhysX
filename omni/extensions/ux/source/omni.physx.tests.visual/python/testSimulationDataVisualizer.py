# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.usd
from omni.physx.scripts import physicsUtils
from omni.physxui.scripts.simulationDataVisualizer import SimulationDataVisualizerWindowManager, SimulationDataVisualizerWindow, SETTINGS_UI_WINDOW_OPACITY
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physxtestsvisual.utils import TestCase
from pxr import Gf, UsdPhysics
import omni.physx.bindings._physx as physx_bindings
from omni.physx import get_physx_interface
import carb.settings
import omni.ui as ui
import omni.kit.ui_test as ui_test
from carb.input import MouseEventType

class PhysxSimulationDataVisualizerTest(TestCase):
    category = TestCategory.Core

    SETTING_USE_FABRIC_SCENE_DELEGATE = "/app/useFabricSceneDelegate"

    async def setUp(self):
        await super().setUp()
        settings = carb.settings.get_settings()
        self._initial_use_fabric_scene_delegate = settings.get_as_bool(__class__.SETTING_USE_FABRIC_SCENE_DELEGATE)
        # Make sure that fabric scene delegate is set to enabled for USDRT tests to work.
        settings.set(__class__.SETTING_USE_FABRIC_SCENE_DELEGATE, True)

    async def tearDown(self):
        carb.settings.get_settings().set(__class__.SETTING_USE_FABRIC_SCENE_DELEGATE, self._initial_use_fabric_scene_delegate)
        await super().tearDown()

    async def _test_simulation_data_visualizer(self, test_img_suffix, simulation_steps = 180, width = SimulationDataVisualizerWindow.WINDOW_WIDTH_MIN, height = 1050, use_usdrt = False):
        settings = carb.settings.get_settings()

        initial_setting = settings.get_as_bool(physx_bindings.SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER)
        settings.set_bool(physx_bindings.SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER, False)

        initial_opacity = settings.get_as_float(SETTINGS_UI_WINDOW_OPACITY)
        settings.set(SETTINGS_UI_WINDOW_OPACITY, 1.0)

        initial_sections_expanded = SimulationDataVisualizerWindow.SharedStates.sections_expanded.copy()
        SimulationDataVisualizerWindow.SharedStates.sections_expanded = {"Local Offset", 
                                                                         "World Pose", 
                                                                         "Linear Velocity", 
                                                                         "Angular Velocity", 
                                                                         "Mass/inertia"}
        await self.wait(10)

        # USD sync settings
        initial_update_to_usd = settings.get_as_bool(physx_bindings.SETTING_UPDATE_TO_USD)
        initial_update_velocities_to_usd = settings.get_as_bool(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)

        settings.set(physx_bindings.SETTING_UPDATE_TO_USD, True)
        settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, True)

        SETTING_FABRIC_ENABLED = "physics/fabricEnabled"
        SETTING_FABRIC_UPDATE_TRANSFORMATIONS = "physics/fabricUpdateTransformations"
        SETTING_FABRIC_UPDATE_VELOCITIES = "physics/fabricUpdateVelocities"

        initial_fabric_enabled = settings.get_as_bool(SETTING_FABRIC_ENABLED)
        initial_fabric_update_transformations = settings.get_as_bool(SETTING_FABRIC_UPDATE_TRANSFORMATIONS)
        initial_fabric_update_velocities = settings.get_as_bool(SETTING_FABRIC_UPDATE_VELOCITIES)
        
        settings.set(SETTING_FABRIC_ENABLED, use_usdrt)
        settings.set(SETTING_FABRIC_UPDATE_TRANSFORMATIONS, use_usdrt)
        settings.set(SETTING_FABRIC_UPDATE_VELOCITIES, use_usdrt)

        simulation_data_visualizer_manager = SimulationDataVisualizerWindowManager()

        await self.wait(10)
        window = simulation_data_visualizer_manager._windows[-1]

        if window is None or not window.visible or not isinstance(window, SimulationDataVisualizerWindow):
            self.fail("Could not retrieve Simulation Data Visualizer window.")

        # Setup the window early to prevent mouse tooltip appearing.
        await self._setup_base_settings()
        await self._setup_docked(window, None, width=width, height=height, no_mouse=False)
        await self.wait(10)
        await ui_test.input.emulate_mouse(MouseEventType.MOVE, ui_test.Vec2(0, 0))
        await self.wait(10)

        if simulation_steps > 0:
            await self.step(simulation_steps, precise=True, stop_timeline_after=True)
            await self.wait(1)

        result = await self.do_visual_test(img_name=("test_simulation_data_visualizer_" + test_img_suffix), skip_assert=True)

        await self.wait(10)

        simulation_data_visualizer_manager.destroy()

        physx_iface = get_physx_interface()
        physx_iface.reset_simulation()

        settings.set_bool(physx_bindings.SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER, initial_setting)
        settings.set(SETTINGS_UI_WINDOW_OPACITY, initial_opacity)

        settings.set(physx_bindings.SETTING_UPDATE_TO_USD, initial_update_to_usd)
        settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, initial_update_velocities_to_usd)

        settings.set(SETTING_FABRIC_ENABLED, initial_fabric_enabled)
        settings.set(SETTING_FABRIC_UPDATE_TRANSFORMATIONS, initial_fabric_update_transformations)
        settings.set(SETTING_FABRIC_UPDATE_VELOCITIES, initial_fabric_update_velocities)

        SimulationDataVisualizerWindow.SharedStates.sections_expanded = initial_sections_expanded

        await self.wait(1)
        return result

    async def test_simulation_data_visualizer_rigid_body(self):
        for i in range(2):
            stage = await self.new_stage()
            prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0), Gf.Vec3f(0.0, 1000.0, 0.0), ang_velocity=Gf.Vec3f(360.0, 180.0, 90.0))
            
            omni.usd.get_context().get_selection().set_selected_prim_paths([prim.GetPath().pathString], False)

            result = await self._test_simulation_data_visualizer("rigid_body" if i == 0 else "rigid_body_usdrt", use_usdrt=(i == 1))

            self.assertTrue(result)

        await self.new_stage()

    async def test_simulation_data_visualizer_revolute_joint(self):
        size = Gf.Vec3f(25.0)
 
        for i in range(2):
            stage = await self.new_stage()

            position = Gf.Vec3f(0.0)
            cube0 = physicsUtils.add_cube(stage, "/cubeActor0", size, position)
            UsdPhysics.CollisionAPI.Apply(cube0)

            position = Gf.Vec3f(0.0, 0.0, 100.0)
            cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

            revoluteJoint = UsdPhysics.RevoluteJoint.Define(self._stage, "/revoluteJoint")
            val1 = [cube0.GetPrim().GetPath()]
            val2 = [cube1.GetPrim().GetPath()]
            revoluteJoint.CreateBody0Rel().SetTargets(val1)
            revoluteJoint.CreateBody1Rel().SetTargets(val2)
            revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 2.0))
            revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -2.0))
            revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            revoluteJoint.CreateAxisAttr("X")
            revoluteJoint.CreateLowerLimitAttr(-float('inf'))
            revoluteJoint.CreateUpperLimitAttr(float('inf'))

            omni.usd.get_context().get_selection().set_selected_prim_paths([revoluteJoint.GetPath().pathString], False)

            result = await self._test_simulation_data_visualizer("revolute_joint" if i == 0 else "revolute_joint_usdrt", use_usdrt=(i == 1))

            self.assertTrue(result)

        await self.new_stage()

    async def test_simulation_data_visualizer_prismatic_joint(self):
        size = Gf.Vec3f(25.0)

        for i in range(2):
            stage = await self.new_stage()

            position = Gf.Vec3f(0.0)
            cube0 = physicsUtils.add_cube(stage, "/cubeActor0", size, position)
            UsdPhysics.CollisionAPI.Apply(cube0)

            position = Gf.Vec3f(0.0, 0.0, 100.0)
            cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

            prismaticJoint = UsdPhysics.PrismaticJoint.Define(self._stage, "/prismaticJoint")
            val1 = [cube0.GetPrim().GetPath()]
            val2 = [cube1.GetPrim().GetPath()]
            prismaticJoint.CreateBody0Rel().SetTargets(val1)
            prismaticJoint.CreateBody1Rel().SetTargets(val2)
            prismaticJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 2.0))
            prismaticJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -2.0))
            prismaticJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            prismaticJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            prismaticJoint.CreateAxisAttr("Y")
            prismaticJoint.CreateLowerLimitAttr(-float('inf'))
            prismaticJoint.CreateUpperLimitAttr(float('inf'))

            omni.usd.get_context().get_selection().set_selected_prim_paths([prismaticJoint.GetPath().pathString], False)

            result = await self._test_simulation_data_visualizer("prismatic_joint" if i == 0 else "prismatic_joint_usdrt", use_usdrt=(i == 1))

            self.assertTrue(result)

        await self.new_stage()

    async def test_simulation_data_visualizer_distance_joint(self):
        size = Gf.Vec3f(25.0)

        for i in range(2):
            stage = await self.new_stage()

            position = Gf.Vec3f(0.0)
            cube0 = physicsUtils.add_cube(stage, "/cubeActor0", size, position)
            UsdPhysics.CollisionAPI.Apply(cube0)

            position = Gf.Vec3f(0.0, 0.0, 100.0)
            cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

            distanceJoint = UsdPhysics.DistanceJoint.Define(self._stage, "/distanceJoint")
            val1 = [cube0.GetPrim().GetPath()]
            val2 = [cube1.GetPrim().GetPath()]
            distanceJoint.CreateBody0Rel().SetTargets(val1)
            distanceJoint.CreateBody1Rel().SetTargets(val2)
            distanceJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 2.0))
            distanceJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -2.0))
            distanceJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            distanceJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            distanceJoint.CreateMinDistanceAttr(1.0)
            distanceJoint.CreateMaxDistanceAttr(50.0)

            omni.usd.get_context().get_selection().set_selected_prim_paths([distanceJoint.GetPath().pathString], False)

            result = await self._test_simulation_data_visualizer("distance_joint" if i == 0 else "distance_joint_usdrt", use_usdrt=(i == 1))

            self.assertTrue(result)

        await self.new_stage()

    async def test_simulation_data_visualizer_spherical_joint(self):
        size = Gf.Vec3f(25.0)

        for i in range(2):
            stage = await self.new_stage()

            position = Gf.Vec3f(0.0)
            cube0 = physicsUtils.add_cube(stage, "/cubeActor0", size, position)
            UsdPhysics.CollisionAPI.Apply(cube0)

            position = Gf.Vec3f(0.0, 0.0, 100.0)
            cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

            sphericalJoint = UsdPhysics.SphericalJoint.Define(self._stage, "/sphericalJoint")
            val1 = [cube0.GetPrim().GetPath()]
            val2 = [cube1.GetPrim().GetPath()]
            sphericalJoint.CreateBody0Rel().SetTargets(val1)
            sphericalJoint.CreateBody1Rel().SetTargets(val2)
            sphericalJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 2.0))
            sphericalJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -2.0))
            sphericalJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            sphericalJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            sphericalJoint.CreateAxisAttr("X")
            sphericalJoint.CreateConeAngle0LimitAttr(90.0)
            sphericalJoint.CreateConeAngle1LimitAttr(90.0)

            omni.usd.get_context().get_selection().set_selected_prim_paths([sphericalJoint.GetPath().pathString], False)

            result = await self._test_simulation_data_visualizer("spherical_joint" if i == 0 else "spherical_joint_usdrt", use_usdrt=(i == 1))

            self.assertTrue(result)

        await self.new_stage()
