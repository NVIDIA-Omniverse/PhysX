# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Ported from omni.physx.tests PhysicsScene.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from physicsBase import (
    PhysicsMemoryStageBaseTestCase,
    TestCategory,
    add_rigid_box,
    check_stats,
    ExpectMessage,
)
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils, UsdLux, UsdShade
import _physx
import carb.settings


class PhysicsSceneTestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def setUp(self):
        super().setUp()

    def _setup_multi_scene(self, stage, remove_bodies=False):
        # Create PhysX scene
        self._physx_scene = UsdPhysics.Scene.Define(stage, "/World/scene0")
        PhysxSchema.PhysxSceneAPI.Apply(self._physx_scene.GetPrim())

        self._physx_cube_prim = add_rigid_box(stage, "/PhysxCube", size=Gf.Vec3f(50.0))

        # Create other scene
        self._other_scene = UsdPhysics.Scene.Define(stage, "/World/scene1")
        self._other_cube_prim = add_rigid_box(stage, "/OtherCube", size=Gf.Vec3f(50.0))

        if remove_bodies:
            self._physx_cube_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            self._other_cube_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    def test_invalid_scene_desc(self):
        stage = self.new_stage()

        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        _physx.acquire_physx_simulation_interface().detach_stage()

        physics_scene = UsdPhysics.Scene.Define(stage, "/World/scene")
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        # 0.0 is not valid we should fall back to some small value
        physx_scene_api.CreateBounceThresholdAttr().Set(0.0)

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

    def test_scene_default_parameters(self):
        stage = self.new_stage()
        UsdPhysics.Scene.Define(stage, "/World/scene")
        scene_prim = UsdPhysics.Scene(stage.GetPrimAtPath("/World/scene"))
        gravity = scene_prim.GetGravityDirectionAttr().Get()
        gravity_magnitude = scene_prim.GetGravityMagnitudeAttr().Get()
        self.assertTrue(Gf.IsClose(Gf.Vec3f(0.0), gravity, 0.01))
        self.assertLess(gravity_magnitude, -1e10)

    def test_multi_sim_rigid_body_explicit_ownership(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage)

        # PhysX scene
        rbo_api = UsdPhysics.RigidBodyAPI(self._physx_cube_prim)
        rbo_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene, apply API to as fake simulator ID
        self._other_scene.GetPrim().ApplyAPI(UsdLux.ShapingAPI)
        rbo_api = UsdPhysics.RigidBodyAPI(self._other_cube_prim)
        rbo_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numStaticRigids": 0})

    def test_multi_sim_rigid_body_default_simulator(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage)

        settings = carb.settings.acquire_settings_interface()
        default_sim = settings.get(_physx.SETTING_DEFAULT_SIMULATOR)

        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "PhysX")
        # PhysX scene
        rbo_api = UsdPhysics.RigidBodyAPI(self._physx_cube_prim)
        rbo_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene
        rbo_api = UsdPhysics.RigidBodyAPI(self._other_cube_prim)
        rbo_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0})

        # default sim is not physx
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "None")
        _physx.acquire_physx_simulation_interface().detach_stage()
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numStaticRigids": 0})
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, default_sim)

    def test_multi_sim_static_body_explicit_ownership(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage, True)

        # PhysX scene
        col_api = UsdPhysics.CollisionAPI(self._physx_cube_prim)
        col_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene, apply API to as fake simulator ID
        self._other_scene.GetPrim().ApplyAPI(UsdLux.ShapingAPI)
        col_api = UsdPhysics.CollisionAPI(self._other_cube_prim)
        col_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 0, "numStaticRigids": 1})

    def test_multi_sim_static_body_default_simulator(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage, True)

        settings = carb.settings.acquire_settings_interface()
        default_sim = settings.get(_physx.SETTING_DEFAULT_SIMULATOR)

        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "PhysX")
        # PhysX scene
        col_api = UsdPhysics.CollisionAPI(self._physx_cube_prim)
        col_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene
        col_api = UsdPhysics.CollisionAPI(self._other_cube_prim)
        col_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 0, "numStaticRigids": 2})

        # default sim is not physx
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "None")
        _physx.acquire_physx_simulation_interface().detach_stage()
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 0, "numStaticRigids": 1})
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, default_sim)

    def test_multi_sim_joints_default_simulator(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage)

        settings = carb.settings.acquire_settings_interface()
        default_sim = settings.get(_physx.SETTING_DEFAULT_SIMULATOR)

        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "PhysX")
        # PhysX scene
        body_api = UsdPhysics.RigidBodyAPI(self._physx_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, "/World/fixedJoint0")
        fixed_joint.CreateBody1Rel().SetTargets([self._physx_cube_prim.GetPrimPath()])

        # Other scene
        body_api = UsdPhysics.RigidBodyAPI(self._other_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, "/World/fixedJoint1")
        fixed_joint.CreateBody1Rel().SetTargets([self._other_cube_prim.GetPrimPath()])

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        self.step()
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 2})

        # default sim is not physx
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "None")
        _physx.acquire_physx_simulation_interface().detach_stage()
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        self.step()
        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numConstraints": 1})
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, default_sim)

    def test_multi_sim_joints_different_owners(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage)

        body_api = UsdPhysics.RigidBodyAPI(self._physx_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene
        body_api = UsdPhysics.RigidBodyAPI(self._other_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, "/World/fixedJoint0")
        fixed_joint.CreateBody0Rel().SetTargets([self._physx_cube_prim.GetPrimPath()])
        fixed_joint.CreateBody1Rel().SetTargets([self._other_cube_prim.GetPrimPath()])

        message = "Cannot create joint between bodies that belong to different owners. Joint: /World/fixedJoint0"
        with ExpectMessage(self, message):
            _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        self.step()
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})

    def test_multi_sim_articulation_different_owners(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self._setup_multi_scene(stage)

        body_api = UsdPhysics.RigidBodyAPI(self._physx_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._physx_scene.GetPrim().GetPrimPath()])

        # Other scene
        body_api = UsdPhysics.RigidBodyAPI(self._other_cube_prim)
        body_api.GetSimulationOwnerRel().SetTargets([self._other_scene.GetPrim().GetPrimPath()])

        revolute_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint0")
        revolute_joint.CreateBody0Rel().SetTargets([self._physx_cube_prim.GetPrimPath()])
        revolute_joint.CreateBody1Rel().SetTargets([self._other_cube_prim.GetPrimPath()])

        UsdPhysics.ArticulationRootAPI.Apply(self._physx_cube_prim)

        message0 = "Articulation contains bodies with different simulation owners. Articulation: /PhysxCube"
        message1 = "Cannot create joint between bodies that belong to different owners. Joint: /World/revoluteJoint0"
        with ExpectMessage(self, [message0, message1]):
            _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        self.step()
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0, "numArticulations": 0})

    def test_multi_sim_articulation_default_simulation(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        settings = carb.settings.acquire_settings_interface()
        default_sim = settings.get(_physx.SETTING_DEFAULT_SIMULATOR)

        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "PhysX")

        UsdPhysics.Scene.Define(stage, "/World/scene0")

        physx_cube_prim = add_rigid_box(stage, "/PhysxCube", size=Gf.Vec3f(50.0))
        other_cube_prim = add_rigid_box(stage, "/OtherCube", size=Gf.Vec3f(50.0))

        revolute_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint0")
        revolute_joint.CreateBody0Rel().SetTargets([physx_cube_prim.GetPrimPath()])
        revolute_joint.CreateBody1Rel().SetTargets([other_cube_prim.GetPrimPath()])

        UsdPhysics.ArticulationRootAPI.Apply(physx_cube_prim)

        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        self.step()
        check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0, "numArticulations": 1})

        # default sim is not physx
        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, "None")
        _physx.acquire_physx_simulation_interface().detach_stage()
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        self.step()
        check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0, "numConstraints": 0, "numArticulations": 0})

        settings.set_string(_physx.SETTING_DEFAULT_SIMULATOR, default_sim)

    def test_physics_default_scene(self):
        stage = self.new_stage()

        settings = carb.settings.acquire_settings_interface()
        default_update_to_usd = settings.get(_physx.SETTING_UPDATE_TO_USD)

        settings.set_bool(_physx.SETTING_UPDATE_TO_USD, False)

        xform = UsdGeom.Cube.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        for _ in range(10):
            self.step()

        transform = _physx.acquire_physx_interface().get_rigidbody_transformation("/xform")
        position = transform["position"]

        self.assertTrue(position[1] < 0.0)

        settings.set_bool(_physx.SETTING_UPDATE_TO_USD, default_update_to_usd)

    def test_physics_scene_default_material_delete(self):
        stage = self.new_stage()

        material_path = "/World/defaultMaterial"
        UsdShade.Material.Define(stage, material_path)
        UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))

        physics_scene = UsdPhysics.Scene.Define(stage, "/World/scene")

        # Add default material via MaterialBindingAPI
        binding_api = UsdShade.MaterialBindingAPI.Apply(physics_scene.GetPrim())
        material_prim = UsdShade.Material(stage.GetPrimAtPath(material_path))
        binding_api.Bind(material_prim, UsdShade.Tokens.weakerThanDescendants, "physics")

        self.step()

        # Remove prims
        stage.RemovePrim(material_path)
        stage.RemovePrim("/World/scene")

        self.step()

        # Add again physics scene
        physics_scene = UsdPhysics.Scene.Define(stage, "/World/scene")

        self.step()


if __name__ == "__main__":
    unittest.main()
