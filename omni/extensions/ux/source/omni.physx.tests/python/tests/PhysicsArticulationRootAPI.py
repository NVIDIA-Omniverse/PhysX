# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.utils as physxUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils, PhysicsSchemaTools
import unittest
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase
from omni.physx import get_physx_simulation_interface


class PhysicsArticulationRootAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.articulation_stage_and_scene_setup()

        # create xform and 3 boxes
        self._boxes_parent_xform = UsdGeom.Xform.Define(self._stage, "/World/articulation")

        box_actor0_path = "/articulation/boxActor0"
        box_actor1_path = "/articulation/boxActor1"
        box_actor2_path = "/articulation/boxActor2"

        size = Gf.Vec3f(100.0)
        position = Gf.Vec3f(0.0, 0.0, 200.0)
        self._box0 = physicsUtils.add_box(self._stage, box_actor0_path, size, position)

        position = Gf.Vec3f(0.0, 0.0, 400.0)
        self._box1_rigid = physicsUtils.add_rigid_box(self._stage, box_actor1_path, size, position)

        position = Gf.Vec3f(0.0, 0.0, 600.0)
        self._box2_rigid = physicsUtils.add_rigid_box(self._stage, box_actor2_path, size, position)

    async def test_articulation_joint_excluded_error_closed_circuit(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(self._box0.GetPrim())

        joint0 = physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        joint1 = physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box2_rigid.GetPrim())

        message = f"RigidBody ({self._box0.GetPrimPath()}) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints:\n{self._box0.GetPrimPath()}\n{joint0.GetPrimPath()}\n{joint1.GetPrimPath()}\n from articulation, the joint will be now excluded from the articulation."
        with utils.ExpectMessage(self, message):
            self.step()

    @unittest.skip("OM-58555")
    async def test_articulation_change_listener_enable_disable(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._boxes_parent_xform.GetPrim())
        physxUtils.createJoint(self._stage, "Fixed", None, self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        physx_articulation_api.CreateArticulationEnabledAttr().Set(True)
        self.step()
        # expect articulation in the simulation
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 1, "numConstraints": 0})

        physx_articulation_api.CreateArticulationEnabledAttr().Set(False)
        self.step()
        # articulation should not be in the simulation
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 0, "numConstraints": 2})

        physx_articulation_api.CreateArticulationEnabledAttr().Set(True)
        self.step()
        # expect articulation in the simulation
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 1, "numConstraints": 0})

    async def test_articulation_parse_not_created_when_disabled(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._boxes_parent_xform.GetPrim())
        physxUtils.createJoint(self._stage, "Fixed", None, self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        physx_articulation_api.CreateArticulationEnabledAttr().Set(False)
        self.step()
        # articulation should not be in the simulation
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 0, "numConstraints": 2})

    async def test_articulation_parse_created_when_enabled(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._boxes_parent_xform.GetPrim())
        physxUtils.createJoint(self._stage, "Fixed", None, self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        physx_articulation_api.CreateArticulationEnabledAttr().Set(True)
        self.step()
        # expect articulation in the simulation
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 1, "numConstraints": 0})

    async def test_articulation_created_when_applied_to_xform(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(self._box0.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        self.step()
        self._check_physx_object_counts({"numBoxShapes": 3, "numDynamicRigids": 2, "numStaticRigids": 1, "numArticulations": 1, "numConstraints": 0})
        self.step()

    async def test_articulation_not_created_when_applied_to_static_body(self):
        UsdPhysics.CollisionAPI.Apply(self._box0.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(self._box0.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        self.step()
        self._check_physx_object_counts({"numBoxShapes": 3, "numDynamicRigids": 2, "numStaticRigids": 1, "numArticulations": 0, "numConstraints": 2})
        self.step()

    async def test_articulation_not_created_when_applied_to_disabled_rigidbody(self):
        UsdPhysics.CollisionAPI.Apply(self._box0.GetPrim())
        box0_rigidbody = UsdPhysics.RigidBodyAPI.Apply(self._box0.GetPrim())
        box0_rigidbody.GetRigidBodyEnabledAttr().Set(False)
        UsdPhysics.ArticulationRootAPI.Apply(self._box0.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        self.step()
        self._check_physx_object_counts({"numBoxShapes": 3, "numDynamicRigids": 2, "numStaticRigids": 1, "numArticulations": 0, "numConstraints": 2})
        self.step()
        
    async def test_articulation_parse_created_then_deleted(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._boxes_parent_xform.GetPrim())
        physxUtils.createJoint(self._stage, "Fixed", None, self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        get_physx_simulation_interface().flush_changes()

        self.step()
        
        self._stage.RemovePrim(self._boxes_parent_xform.GetPrim().GetPrimPath())
        
        self.step()
        
        # expect no articulation in the simulation
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numArticulations": 0, "numConstraints": 0})
        
    async def test_articulation_parse_kinematics_error(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())        
        fixed_joint = physxUtils.createJoint(self._stage, "Fixed", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())
        rbo_api = UsdPhysics.RigidBodyAPI.Apply(self._box0.GetPrim())
        UsdPhysics.CollisionAPI.Apply(self._box0.GetPrim())
        rbo_api.GetKinematicEnabledAttr().Set(True)
        
        fixed_joint_path = fixed_joint.GetPrimPath()
        message = (f"Articulations with kinematic bodies are not supported, please exclude joint ({fixed_joint_path}) from articulation.")
        with utils.ExpectMessage(self, message):
            self.step()

        # expect no articulation in the simulation
        self._check_physx_object_counts({"numBoxShapes": 3, "numDynamicRigids": 2, "numKinematicBodies": 1, "numArticulations": 0, "numConstraints": 2})

    async def _run_articulation_teleport_test(self):
        self._setup_fixed_joint()

        base_link_trans = self._get_link_translation_op(self._base_link).Get()
        dynamic_link_trans = self._get_link_translation_op(self._dynamic_link).Get()
        delta = dynamic_link_trans - base_link_trans

        self.assertFloatIterableAlmostEqual(Gf.Vec3f(0.0, 0.0, 0.0), base_link_trans, delta=0.001)

        new_pos = Gf.Vec3f(1.8, 2.7, 1.3)
        self._get_link_translation_op(self._base_link).Set(new_pos)
        self.step()

        base_link_trans = self._get_link_translation_op(self._base_link).Get()
        self.assertFloatIterableAlmostEqual(base_link_trans, new_pos, delta=0.001)

        dynamic_link_new_delta = self._get_link_translation_op(self._dynamic_link).Get() - base_link_trans
        self.assertFloatIterableAlmostEqual(dynamic_link_new_delta, delta, delta=0.001)

    async def test_floating_root_teleport(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=False)
        await self._run_articulation_teleport_test()

    async def test_fixed_root_teleport(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=True)
        await self._run_articulation_teleport_test()

    async def test_fixed_articulation_delete(self):            
        self._setup_two_link_articulation(make_fixed_base=False)
        
        self.step()
        
        self._stage.RemovePrim(self._base_link.GetPrim().GetPrimPath())
        
        self.step()

    async def test_floating_root_linear_velocity(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=False)
        self._setup_fixed_joint()
        self._body_api = UsdPhysics.RigidBodyAPI(self._base_link.GetPrim())

        dt = 1.0 / 60.0
        self.step(num_steps=1, dt=dt)

        base_link_trans = self._get_link_translation_op(self._base_link).Get()
        self.assertFloatIterableAlmostEqual(Gf.Vec3f(0.0, 0.0, 0.0), base_link_trans, delta=0.001)

        vel = Gf.Vec3f(1.0, 2.0, 3.0)
        self._body_api.GetVelocityAttr().Set(vel)
        nb_steps = 10
        self.step(num_steps=nb_steps, dt=dt)

        base_link_trans = self._get_link_translation_op(self._base_link).Get()
        self.assertFloatIterableAlmostEqual(base_link_trans, vel * dt * nb_steps, delta=0.001)

    async def test_floating_root_angular_velocity(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=False)
        self._setup_fixed_joint()
        self._body_api = UsdPhysics.RigidBodyAPI(self._base_link.GetPrim())

        dt = 1.0 / 60.0
        self.step(num_steps=1, dt=dt)
        self.assertQuaternionAlmostEqual(Gf.Quatf.GetIdentity(), self._get_rotation_quaternion(self._dynamic_link), delta_deg=0.1)

        ang_vel = 2.0
        vel = Gf.Vec3f(0.0, ang_vel, 0.0)
        self._body_api.GetAngularVelocityAttr().Set(vel)
        nb_steps = 10
        self.step(num_steps=nb_steps, dt=dt)
        angle = self._get_rotation_angle(self._dynamic_link)
        self.assertAlmostEqual(angle, ang_vel * dt * nb_steps, delta=0.01)

    async def test_articulation_root_delete(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=False)
        self._setup_revolute_joint()

        self.step()        
        self._check_physx_object_counts({"numBoxShapes": 4, "numDynamicRigids": 4, "numArticulations": 1, "numConstraints": 0})        
        
        self._stage.RemovePrim(self._base_link.GetPrim().GetPrimPath())
        self._stage.RemovePrim(self._dynamic_link.GetPrim().GetPrimPath())
        
        self.step()
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numArticulations": 0, "numConstraints": 0})        

    async def test_articulation_sleeping(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=True)
        self._setup_revolute_joint()
        
        stage_id = UsdUtils.StageCache.Get().GetId(self._stage).ToLongInt()

        self.step()        
        self._check_physx_object_counts({"numBoxShapes": 4, "numDynamicRigids": 4, "numArticulations": 1, "numConstraints": 0})        

        art_encoded = PhysicsSchemaTools.sdfPathToInt(self._stage.GetDefaultPrim().GetPrimPath())
                
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(not is_sleeping)

        get_physx_simulation_interface().put_to_sleep(stage_id, art_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(is_sleeping)
        
        get_physx_simulation_interface().wake_up(stage_id, art_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(not is_sleeping)
        
    async def test_articulation_link_sleeping(self):
        self._disable_gravity()
        self._setup_two_link_articulation(make_fixed_base=False)
        self._setup_revolute_joint()
        
        stage_id = UsdUtils.StageCache.Get().GetId(self._stage).ToLongInt()

        self.step()        
        self._check_physx_object_counts({"numBoxShapes": 4, "numDynamicRigids": 4, "numArticulations": 1, "numConstraints": 0})        

        art_encoded = PhysicsSchemaTools.sdfPathToInt(self._dynamic_link_path)
                
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(not is_sleeping)

        get_physx_simulation_interface().put_to_sleep(stage_id, art_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(is_sleeping)
        
        get_physx_simulation_interface().wake_up(stage_id, art_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, art_encoded)
        self.assertTrue(not is_sleeping)        
        
    async def test_articulation_collision_resync(self):
        self._disable_gravity()
        self._link_length = 0.05 * self._length_scale
        self._link_width = self._link_length
        size = Gf.Vec3f(self._link_width, self._link_length, self._link_width)

        self._stage.RemovePrim(self._boxes_parent_xform.GetPrim().GetPrimPath())

        # root link
        root_link_prim = UsdGeom.Xform.Define(self._stage, "/rootLink")        
        UsdPhysics.RigidBodyAPI.Apply(root_link_prim.GetPrim())
        box_collider0 = UsdGeom.Cube.Define(self._stage, "/rootLink/Cube")
        UsdPhysics.CollisionAPI.Apply(box_collider0.GetPrim())

        # floating base apply to base
        UsdPhysics.ArticulationRootAPI.Apply(root_link_prim.GetPrim())

        # dynamic link
        dynamic_link_prim = UsdGeom.Xform.Define(self._stage, "/dynamicLink")        
        UsdPhysics.RigidBodyAPI.Apply(dynamic_link_prim.GetPrim())
        box_collider1 = UsdGeom.Cube.Define(self._stage, "/dynamicLink/Cube")
        UsdPhysics.CollisionAPI.Apply(box_collider1.GetPrim())
        
        # Joint between
        revolute_joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/revoluteJoint")
        revolute_joint.CreateBody0Rel().SetTargets([root_link_prim.GetPrim().GetPrimPath()])
        revolute_joint.CreateBody1Rel().SetTargets([dynamic_link_prim.GetPrim().GetPrimPath()])
        
        self.step()
        
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0, "numArticulations": 1, "numConstraints": 0})
        
        # resync change the collider types
        box_collider0.GetPrim().SetTypeName("Sphere")
        box_collider1.GetPrim().SetTypeName("Sphere")

        self.step()
        
        self._check_physx_object_counts({"numBoxShapes": 0, "numSphereShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0, "numArticulations": 1, "numConstraints": 0})

    async def test_articulation_collision_delete(self):
        self._disable_gravity()
        self._link_length = 0.05 * self._length_scale
        self._link_width = self._link_length
        size = Gf.Vec3f(self._link_width, self._link_length, self._link_width)

        self._stage.RemovePrim(self._boxes_parent_xform.GetPrim().GetPrimPath())

        # root link
        root_link_prim = UsdGeom.Xform.Define(self._stage, "/rootLink")        
        UsdPhysics.RigidBodyAPI.Apply(root_link_prim.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(root_link_prim.GetPrim())
        mass_api.GetMassAttr().Set(1.0)
        box_collider0 = UsdGeom.Cube.Define(self._stage, "/rootLink/Cube")
        UsdPhysics.CollisionAPI.Apply(box_collider0.GetPrim())

        # floating base apply to base
        UsdPhysics.ArticulationRootAPI.Apply(root_link_prim.GetPrim())

        # dynamic link
        dynamic_link_prim = UsdGeom.Xform.Define(self._stage, "/dynamicLink")        
        UsdPhysics.RigidBodyAPI.Apply(dynamic_link_prim.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(dynamic_link_prim.GetPrim())
        mass_api.GetMassAttr().Set(1.0)
        box_collider1 = UsdGeom.Cube.Define(self._stage, "/dynamicLink/Cube")
        UsdPhysics.CollisionAPI.Apply(box_collider1.GetPrim())
        
        # Joint between
        revolute_joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/revoluteJoint")
        revolute_joint.CreateBody0Rel().SetTargets([root_link_prim.GetPrim().GetPrimPath()])
        revolute_joint.CreateBody1Rel().SetTargets([dynamic_link_prim.GetPrim().GetPrimPath()])
        
        self.step()
        
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0, "numArticulations": 1, "numConstraints": 0})
        
        # resync change the collider types
        self._stage.RemovePrim(box_collider0.GetPrim().GetPrimPath())
        self._stage.RemovePrim(box_collider1.GetPrim().GetPrimPath())

        self.step()
        
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 2, "numStaticRigids": 0, "numArticulations": 1, "numConstraints": 0})

    #
    # test misconfiguration of having rigidBodyEnabled set to false on an articulation link.
    # Should not crash and send error message.
    #
    async def test_articulation_rigid_body_enabled_false(self):
        await self._setup_stage_and_bodies()

        rigidBodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
        path = self._dynamic_box.GetPath()

        UsdPhysics.ArticulationRootAPI.Apply(self._dynamic_box)

        self.step()

        message = (f"Setting rigidBodyEnabled to false is not supported if the rigid body is part of an articulation. Body: {path}")

        with utils.ExpectMessage(self, message):
            rigidBodyAPI.GetRigidBodyEnabledAttr().Set(False)

        self.step()

        # setting the value back to a legal state should not trigger an error message
        rigidBodyAPI.GetRigidBodyEnabledAttr().Set(True)

        self.step()

    async def test_articulation_max_velocity_angular(self):
        self._disable_gravity()
        await self._revolute_test_setup("X", "vel", setupAsArticulation=True, targetValue = 360.0 * 100.0)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(self._joint.GetPrim())

        target_velocities = (0.0, 90.0, 360.0, 0.0)
        for target_velocity in target_velocities:
            physxJointAPI.GetMaxJointVelocityAttr().Set(target_velocity)
            self.step(10)
            current_velocity = rigidBodyAPI.GetAngularVelocityAttr().Get()
            self.assertAlmostEqual(current_velocity[0], target_velocity, delta=0.01)

    async def test_articulation_max_velocity_linear(self):
        self._disable_gravity()
        await self._prismatic_test_setup("X", "vel", setupAsArticulation=True, targetValue = 1000000.0)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(self._joint.GetPrim())

        target_velocities = (0.0, 100.0, 10000.0, 0.0)
        for target_velocity in target_velocities:
            physxJointAPI.GetMaxJointVelocityAttr().Set(target_velocity)
            self.step(10)
            current_velocity = rigidBodyAPI.GetVelocityAttr().Get()
            self.assertAlmostEqual(current_velocity[0], target_velocity, delta=1.0)

