# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import carb
import omni.usd
from omni.physx.scripts import physicsUtils
from omni.physics.ui.scripts.simulationDataVisualizer import SimulationDataVisualizerWindowManager, SimulationDataVisualizerWindow, SETTINGS_UI_WINDOW_OPACITY, SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physxtestsvisual.utils import TestCase
from pxr import Gf, UsdPhysics
import omni.physx.bindings._physx as physx_bindings
from omni.physx import get_physx_interface
from omni.physics.core import get_physics_stage_update_interface, get_physics_interaction_interface
import carb.settings
import omni.ui as ui
import omni.kit.ui_test as ui_test
from carb.input import MouseEventType

class PhysxSimulationDataVisualizerTest(TestCase):
    category = TestCategory.Core

    async def _test_simulation_data_visualizer(self, test_img_suffix, simulation_steps = 180, width = SimulationDataVisualizerWindow.WINDOW_WIDTH_MIN, height = 1050):
        settings = carb.settings.get_settings()

        initial_opacity = settings.get_as_float(SETTINGS_UI_WINDOW_OPACITY)
        settings.set(SETTINGS_UI_WINDOW_OPACITY, 1.0)

        initial_setting = settings.get_as_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED)
        settings.set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, True)

        initial_plots_enabled = SimulationDataVisualizerWindow.SharedStates.plots_enabled.copy()

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

        physics_stage_update_interface = get_physics_stage_update_interface()
        physics_stage_update_interface.force_load_physics_from_usd()

        prim_path = omni.usd.get_context().get_selection().get_selected_prim_paths()[0]
        debug_data = get_physics_interaction_interface().get_prim_debug_data(prim_path)

        for key in debug_data.keys():
            SimulationDataVisualizerWindow.SharedStates.plots_enabled.add(key)

        await self.wait(10)


        window = ui.Workspace.get_window(SimulationDataVisualizerWindow.WINDOW_TITLE)
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

        result = await self.do_visual_test(img_name=("test_simulation_data_visualizer_" + test_img_suffix), skip_assert=True, use_renderer_capture=True)

        await self.wait(10)

        physx_iface = get_physx_interface()
        physx_iface.reset_simulation()

        settings.set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, initial_setting)
        settings.set(SETTINGS_UI_WINDOW_OPACITY, initial_opacity)

        settings.set(physx_bindings.SETTING_UPDATE_TO_USD, initial_update_to_usd)
        settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, initial_update_velocities_to_usd)

        settings.set(SETTING_FABRIC_ENABLED, initial_fabric_enabled)
        settings.set(SETTING_FABRIC_UPDATE_TRANSFORMATIONS, initial_fabric_update_transformations)
        settings.set(SETTING_FABRIC_UPDATE_VELOCITIES, initial_fabric_update_velocities)

        SimulationDataVisualizerWindow.SharedStates.plots_enabled = initial_plots_enabled

        await self.wait(1)
        return result

    async def test_simulation_data_visualizer_rigid_body(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0), Gf.Vec3f(0.0, 1000.0, 0.0), ang_velocity=Gf.Vec3f(360.0, 180.0, 90.0))
        
        omni.usd.get_context().get_selection().set_selected_prim_paths([prim.GetPath().pathString], False)

        result = await self._test_simulation_data_visualizer("rigid_body")

        self.assertTrue(result)

        await self.new_stage()

    async def test_simulation_data_visualizer_revolute_joint(self):
        size = Gf.Vec3f(25.0)
 
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

        result = await self._test_simulation_data_visualizer("revolute_joint")

        await self.new_stage()

        self.assertTrue(result)

    async def test_simulation_data_visualizer_prismatic_joint(self):
        size = Gf.Vec3f(25.0)

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

        result = await self._test_simulation_data_visualizer("prismatic_joint")

        await self.new_stage()

        self.assertTrue(result)

    async def test_simulation_data_visualizer_distance_joint(self):
        size = Gf.Vec3f(25.0)

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

        result = await self._test_simulation_data_visualizer("distance_joint")

        await self.new_stage()

        self.assertTrue(result)

    async def test_simulation_data_visualizer_spherical_joint(self):
        size = Gf.Vec3f(25.0)

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

        result = await self._test_simulation_data_visualizer("spherical_joint")

        await self.new_stage()

        self.assertTrue(result)
