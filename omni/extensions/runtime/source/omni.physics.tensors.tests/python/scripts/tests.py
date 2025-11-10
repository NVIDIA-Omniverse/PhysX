# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import os
import sys, traceback
from typing import List

import carb
import carb.settings
import omni.kit.app
import omni.kit.commands
import omni.usd

import unittest

import numpy as np
import warp as wp
from  . import warp_utils as wp_utils

from pxr import Usd, Gf, Sdf, UsdGeom, UsdUtils, UsdLux, UsdShade
from pxr import UsdPhysics, PhysxSchema

import omni.physx.bindings._physx as physx_bindings

from omni.physxtests.utils.physicsBase import (
    PhysicsBaseAsyncTestCase,
    #PhysicsMemoryStageBaseAsyncTestCase,
    #PhysicsKitStageAsyncTestCase,
)

from .scenario import *

# run tests in Kit's USD context or stage in memory
_ATTACH_TO_USD_CONTEXT = True

# run one simulation step before using the tensor API (currently required for GPU pipeline)
_WARM_START = True

# python frontend for tensor API
_FRONTEND = "warp"

# whether to keep tests alive longer for visual inspection
_KEEPALIVE = True

np.set_printoptions(suppress=True)


class GridTestBase(GridScenarioBase):
    def __init__(self, test_case, grid_params: GridParams, sim_params: SimParams, device_params: DeviceParams):
        super().__init__(grid_params, sim_params, device_params)
        self.test_case = test_case
        self.minsteps = 0
        self.maxsteps = 300

    def check_simulation_view(self, sim_view, expected_device):
        self.test_case.assertIsInstance(sim_view, omni.physics.tensors.impl.api.SimulationView)
        self.test_case.assertEqual(sim_view.device, expected_device)

    def check_articulation_view(self, arti_view, expected_count, expected_max_links, expected_max_dofs, require_homogeneous=False):
        self.test_case.assertIsInstance(arti_view, omni.physics.tensors.impl.api.ArticulationView)
        self.test_case.assertEqual(arti_view.count, expected_count)
        self.test_case.assertEqual(arti_view.max_links, expected_max_links)
        self.test_case.assertEqual(arti_view.max_dofs, expected_max_dofs)
        if require_homogeneous:
            self.test_case.assertTrue(arti_view.is_homogeneous)

    def check_rigid_body_view(self, rb_view, expected_count):
        self.test_case.assertIsInstance(rb_view, omni.physics.tensors.impl.api.RigidBodyView)
        self.test_case.assertEqual(rb_view.count, expected_count)

    def check_rigid_contact_view(self, rc_view, expected_sensor_count, expected_filter_count):
        self.test_case.assertIsInstance(rc_view, omni.physics.tensors.impl.api.RigidContactView)
        self.test_case.assertEqual(rc_view.sensor_count, expected_sensor_count)
        self.test_case.assertEqual(rc_view.filter_count, expected_filter_count)

    def check_particle_system_view(self, ps_view, expected_count):
        self.test_case.assertIsInstance(ps_view, omni.physics.tensors.impl.api.ParticleSystemView)
        self.test_case.assertEqual(ps_view.count, expected_count)

    def check_particle_cloth_view(self, pc_view, expected_count):
        self.test_case.assertIsInstance(pc_view, omni.physics.tensors.impl.api.ParticleClothView)
        self.test_case.assertEqual(pc_view.count, expected_count)

    def check_soft_body_view(self, sb_view, expected_count):
        self.test_case.assertIsInstance(sb_view, omni.physics.tensors.impl.api.SoftBodyView)
        self.test_case.assertEqual(sb_view.count, expected_count)

    def check_soft_body_material_view(self, sm_view, expected_count):
        self.test_case.assertIsInstance(sm_view, omni.physics.tensors.impl.api.SoftBodyMaterialView)
        self.test_case.assertEqual(sm_view.count, expected_count)

    def check_deformable_body_view(self, db_view, expected_count):
        self.test_case.assertIsInstance(db_view, omni.physics.tensors.impl.api.DeformableBodyView)
        self.test_case.assertEqual(db_view.count, expected_count)

    def check_deformable_material_view(self, dm_view, expected_count):
        self.test_case.assertIsInstance(dm_view, omni.physics.tensors.impl.api.DeformableMaterialView)
        self.test_case.assertEqual(dm_view.count, expected_count)

    def check_particle_material_view(self, pm_view, expected_count):
        self.test_case.assertIsInstance(pm_view, omni.physics.tensors.impl.api.ParticleMaterialView)
        self.test_case.assertEqual(pm_view.count, expected_count)

    def check_sdf_shape_view(self, sdf_view, expected_count):
        self.test_case.assertIsInstance(sdf_view, omni.physics.tensors.impl.api.SdfShapeView)
        self.test_case.assertEqual(sdf_view.count, expected_count)

    def to_warp(self, numpy_arr, type=wp.float32):
        return wp.from_numpy(numpy_arr, dtype=type, device=self.sim.device)

    def start(self, frontend="warp", stage_id=-1):
        with wp.ScopedDevice(self.wp_device):
            # create simulation view
            self.sim = omni.physics.tensors.create_simulation_view(frontend, stage_id)
            self.check_simulation_view(self.sim, self.wp_device)

            # call implementation
            self.on_start(self.sim)


    def physics_step(self, stepno, dt):
        with wp.ScopedDevice(self.wp_device):
            # call implementation
            self.on_physics_step(self.sim, stepno, dt)

    # override stopping conditions
    def should_quit(self, stepno):
        # make sure test completed within maxsteps
        if self.maxsteps is not None:
            self.test_case.assertLess(stepno, self.maxsteps, "Test failed to finish within maxsteps")

        return super().should_quit(stepno)

    @abstractmethod
    def on_start(self, sim):
        """Called when simulation starts"""

    @abstractmethod
    def on_physics_step(self, sim, stepno, dt):
        """Called after every physics simulation step"""


class TestSimulationViewGravity(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 10.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        sim.set_gravity(carb.Float3(0, 0, 1))
        gravity = sim.get_gravity()

        self.test_case.assertEqual(gravity.x, 0)
        self.test_case.assertEqual(gravity.y, 0)
        self.test_case.assertEqual(gravity.z, 1)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(cartpoles, self.num_envs, 3, 2, True)

        mt = cartpoles.shared_metatype

        self.test_case.assertIsNotNone(mt)
        self.test_case.assertIn("rail", mt.link_indices)
        self.test_case.assertIn("cart", mt.link_indices)
        self.test_case.assertIn("pole", mt.link_indices)
        self.test_case.assertEqual(mt.link_indices["rail"], 0)
        self.test_case.assertEqual(mt.link_indices["cart"], 1)
        self.test_case.assertEqual(mt.link_indices["pole"], 2)

        self.test_case.assertIn("cartJoint", mt.dof_indices)
        self.test_case.assertIn("poleJoint", mt.dof_indices)
        self.test_case.assertEqual(mt.dof_indices["cartJoint"], 0)
        self.test_case.assertEqual(mt.dof_indices["poleJoint"], 1)

        self.test_case.assertEqual(len(cartpoles.prim_paths), self.num_envs)
        self.test_case.assertTrue(f"/envs/env{i}/cartpole" in cartpoles.prim_paths for i in list(range(self.num_envs)))
        self.test_case.assertTrue(f"/envs/env{i}/cartpole/{joint}" in cartpoles.dof_paths for i in list(range(self.num_envs)) for joint in ["rootJoint", "cartJoint", "poleJoint"])
        self.test_case.assertTrue(f"/envs/env{i}/cartpole/{link}" in cartpoles.link_paths for i in list(range(self.num_envs)) for link in ["rail", "cart", "pole"])

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass


class TestHumanoidView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)

        mt = humanoids.shared_metatype
        self.test_case.assertIsNotNone(mt)

        """
        print(mt.link_count, "links:")
        for name in mt.link_names:
            print(" ", name)
        print(mt.joint_count, "joints:")
        for name in mt.joint_names:
            print(" ", name)
        print(mt.dof_count, "dofs:")
        for name in mt.dof_names:
            print(" ", name)
        """

        expected_link_names = [
            "torso",
            "head",
            "lower_waist",
            "right_upper_arm",
            "left_upper_arm",
            "pelvis",
            "right_lower_arm",
            "left_lower_arm",
            "right_thigh",
            "left_thigh",
            "right_hand",
            "left_hand",
            "right_shin",
            "left_shin",
            "right_foot",
            "left_foot",
        ]
        expected_parent_names = [
            "",
            "torso",
            "torso",
            "torso",
            "torso",
            "lower_waist",
            "right_upper_arm",
            "left_upper_arm",
            "pelvis",
            "pelvis",
            "right_lower_arm",
            "left_lower_arm",
            "right_thigh",
            "left_thigh",
            "right_shin",
            "left_shin",
        ]

        expected_dof_names = [
            "abdomen_z",
            "abdomen_y",
            "right_shoulder1",
            "right_shoulder2",
            "left_shoulder1",
            "left_shoulder2",
            "abdomen_x",
            "right_elbow",
            "left_elbow",
            "right_hip_x",
            "right_hip_y",
            "right_hip_z",
            "left_hip_x",
            "left_hip_y",
            "left_hip_z",
            "right_knee",
            "left_knee",
            "right_ankle_y",
            "right_ankle_x",
            "left_ankle_y",
            "left_ankle_x",
        ]

        expected_parent_indices = {}
        for (child, parent) in zip(expected_link_names, expected_parent_names):
            if parent != "": # note that the root of floating articulation does not have a parent
                parent_index = mt.link_indices[parent]
                expected_parent_indices[child] = parent_index

        """
        print(mt.link_names)
        print(expected_link_names)
        print(mt.dof_names)
        print(expected_dof_names)
        print(mt.link_parents)
        print(expected_parent_names)
        print(mt.link_parent_indices)
        print(expected_parent_indices)
        """
        
        # check the parent links
        self.test_case.assertSequenceEqual(mt.link_parents, expected_parent_names)
        self.test_case.assertSequenceEqual(mt.link_parent_indices, expected_parent_indices)

        # check ordered link and DOF names
        self.test_case.assertSequenceEqual(mt.link_names, expected_link_names)
        self.test_case.assertSequenceEqual(mt.dof_names, expected_dof_names)

        # check DOF types
        dof_types = humanoids.get_dof_types()
        dof_types_np = dof_types.numpy().reshape(humanoids.count, humanoids.max_dofs)[0]
        expected_dof_types = np.repeat(np.uint8(omni.physics.tensors.DofType.Rotation), humanoids.max_dofs)
        self.test_case.assertTrue(np.array_equal(dof_types_np, expected_dof_types))
        #print(dof_types_np)
        #print(expected_dof_types)

        # check DOF motions
        dof_motions = humanoids.get_dof_motions()
        dof_motions_np = dof_motions.numpy().reshape(humanoids.count, humanoids.max_dofs)[0]
        expected_dof_motions = np.repeat(np.uint8(omni.physics.tensors.DofMotion.Limited), humanoids.max_dofs)
        self.test_case.assertTrue(np.array_equal(dof_motions_np, expected_dof_motions))
        #print(dof_motions_np)
        #print(expected_dof_motions)

        # check DOF limits
        dof_limits = humanoids.get_dof_limits()
        dof_limits_np = dof_limits.numpy().reshape(humanoids.count, humanoids.max_dofs, 2)[0]
        #print(dof_limits_np)

        # limits from original mjcf
        expected_dof_limits = {
            "abdomen_z": (-45, 45),
            "abdomen_y": (-75, 30),
            "abdomen_x": (-35, 35),
            "right_shoulder1": (-90, 70),
            "right_shoulder2": (-90, 70),
            "right_elbow": (-90, 50),
            "left_shoulder1": (-90, 70),
            "left_shoulder2": (-90, 70),
            "left_elbow": (-90, 50),
            "right_hip_x": (-45, 15),
            "right_hip_y": (-120, 45),
            "right_hip_z": (-60, 35),
            "right_knee": (-160, 2),
            "right_ankle_y": (-50, 50),
            "right_ankle_x": (-50, 50),
            "left_hip_x": (-45, 15),
            "left_hip_y": (-120, 45),
            "left_hip_z": (-60, 35),
            "left_knee": (-160, 2),
            "left_ankle_y": (-50, 50),
            "left_ankle_x": (-50, 50),
        }

        for dof_name, expected_limits in expected_dof_limits.items():
            self.test_case.assertIn(dof_name, mt.dof_indices)
            dof_idx = mt.dof_indices[dof_name]
            lower = np.degrees(dof_limits_np[dof_idx, 0])
            upper = np.degrees(dof_limits_np[dof_idx, 1])
            expected_lower = np.float32(expected_limits[0])
            expected_upper = np.float32(expected_limits[1])
            # print("%s (%f, %f) -> (%f, %f)" % (dof_name, lower, upper, expected_lower, expected_upper))
            self.test_case.assertAlmostEqual(lower, expected_lower, places=4)
            self.test_case.assertAlmostEqual(upper, expected_upper, places=4)

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass

class TestArticulationSpecialCases(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs= 16
        grid_params = GridParams(self.num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")
        transform = Transform((0.0, 0.0, 0.0))

        # pendulum_path = Sdf.Path(actor_path)
        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p)
        xform_prim = xform.GetPrim()

        root_link_path = actor_path.AppendChild("RootLink")
        fixed_joint_link_path = root_link_path.AppendChild("FixedJointLink")
 
        # make it an articulation
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(xform_prim)
        articulation_api.CreateSleepThresholdAttr(0.0)
        articulation_api.CreateEnabledSelfCollisionsAttr().Set(False)

        # add fixed root link:
        link_position = Gf.Vec3f(0,0,1.0)
        physicsUtils.add_rigid_sphere(self.stage, root_link_path, 0.1, link_position)
        joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path)
        joint.CreateBody1Rel().SetTargets([root_link_path])

    def on_start(self, sim):
        self.articulations = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(self.articulations, self.num_envs, 1, 0, True)

    def on_physics_step(self, sim, stepno, dt):
        articulations =  self.articulations
        articulations.get_dof_stiffnesses()
        stiffness = np.ones((articulations.count, articulations.max_dofs)) * 100
        all_indices = wp_utils.arange(articulations.count)
        wp_stiffness = wp.from_numpy(stiffness, dtype=wp.float32, device="cpu")
        articulations.set_dof_stiffnesses(wp_stiffness, all_indices)
        self.test_case.assertTrue(np.allclose(articulations.get_dof_stiffnesses().numpy(), stiffness))
        if stepno == 1:
            self.finish()

class TestArticulationViewDuplicateNames(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs= 16
        grid_params = GridParams(self.num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        def add_rigid_child_link(stage, parent_path, child_name):
            child_path = parent_path.AppendChild(child_name)
            child_xform = UsdGeom.Xform.Define(stage, child_path)

            xformable = UsdGeom.Xformable(child_xform.GetPrim())
            xformable.SetResetXformStack(True)

            UsdPhysics.RigidBodyAPI.Apply(child_xform.GetPrim())
            mass_api = UsdPhysics.MassAPI.Apply(child_xform.GetPrim())
            mass_api.CreateMassAttr(1.0)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.1, 0.1, 0.1))

            return child_path

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")
        transform = Transform((0.0, 0.0, 0.0))

        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p)
        xform_prim = xform.GetPrim()

        root_link_path = actor_path.AppendChild("RootLink")
        fixed_joint_link_path = root_link_path.AppendChild("FixedJointLink")

        # make it an articulation
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(xform_prim)
        articulation_api.CreateSleepThresholdAttr(0.0)
        articulation_api.CreateEnabledSelfCollisionsAttr().Set(False)

        # add fixed root link:
        link_position = Gf.Vec3f(0,0,1.0)
        physicsUtils.add_rigid_sphere(self.stage, root_link_path, 0.1, link_position)
        root_joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path)
        root_joint.CreateBody1Rel().SetTargets([root_link_path])

        # add first child link
        child1_link_path = add_rigid_child_link(self.stage, root_link_path, "ChildLink1")
        child1_joint_link_path = root_link_path.AppendChild("ChildJoint1")
        child1_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child1_joint_link_path)
        child1_joint.CreateBody0Rel().SetTargets([root_link_path])
        child1_joint.CreateBody1Rel().SetTargets([child1_link_path])
        child1_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        child1_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        child1_joint.CreateLocalPos1Attr().Set(link_position)
        child1_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        child1_joint.CreateAxisAttr("X")

        # add a child to first child link
        child1_tip_link_path = add_rigid_child_link(self.stage, child1_link_path, "TipLink")
        child1_tip_joint_link_path = child1_link_path.AppendChild("TipJoint")
        child1_tip_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child1_tip_joint_link_path)
        child1_tip_joint.CreateBody0Rel().SetTargets([child1_link_path])
        child1_tip_joint.CreateBody1Rel().SetTargets([child1_tip_link_path])
        child1_tip_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        child1_tip_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        child1_tip_joint.CreateLocalPos1Attr().Set(link_position)
        child1_tip_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        child1_tip_joint.CreateAxisAttr("X")

        # add second child link
        child2_link_path = add_rigid_child_link(self.stage, root_link_path, "ChildLink2")
        child2_joint_link_path = root_link_path.AppendChild("ChildJoint2")
        child2_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child2_joint_link_path)
        child2_joint.CreateBody0Rel().SetTargets([root_link_path])
        child2_joint.CreateBody1Rel().SetTargets([child2_link_path])
        child2_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        child2_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        child2_joint.CreateLocalPos1Attr().Set(link_position)
        child2_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        child2_joint.CreateAxisAttr("X")

        # add a child to second child link
        child2_tip_link_path = add_rigid_child_link(self.stage, child2_link_path, "TipLink")
        child2_tip_joint_link_path = child2_link_path.AppendChild("TipJoint")
        child2_tip_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child2_tip_joint_link_path)
        child2_tip_joint.CreateBody0Rel().SetTargets([child2_link_path])
        child2_tip_joint.CreateBody1Rel().SetTargets([child2_tip_link_path])
        child2_tip_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        child2_tip_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        child2_tip_joint.CreateLocalPos1Attr().Set(link_position)
        child2_tip_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        child2_tip_joint.CreateAxisAttr("X")

    def on_start(self, sim):
        self.simple_articulation = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(self.simple_articulation, self.num_envs, 5, 4, True)

        art_metatype = self.simple_articulation.get_metatype(0)
        # Check there is no duplicate names
        expected_joint_names = [
            "ChildJoint1",
            "ChildJoint2",
            "TipJoint",
            "TipJoint_0",
        ]
        expected_link_names = [
            "RootLink",
            "ChildLink1",
            "ChildLink2",
            "TipLink",
            "TipLink_0",
        ]
        expected_dof_names = expected_joint_names
        self.test_case.assertTrue((expected_joint_names == art_metatype.joint_names), "all joint names are unique")
        self.test_case.assertTrue((expected_link_names == art_metatype.link_names), "all link names are unique")
        self.test_case.assertTrue((expected_dof_names == art_metatype.dof_names), "all dof names are unique")

        # Check indices are correct
        expected_joint_indices = [0, 1, 2, 3]
        expected_link_indices = [0, 1, 2, 3, 4]
        expected_dof_indices = [0, 1, 2, 3]
        for i in range(art_metatype.joint_count):
            joint_name = expected_joint_names[i]
            joint_index = art_metatype.joint_indices[joint_name]
            self.test_case.assertTrue((expected_joint_indices[i] == joint_index), "joint index is correct")
        for i in range(art_metatype.dof_count):
            dof_name = expected_dof_names[i]
            dof_index = art_metatype.dof_indices[dof_name]
            self.test_case.assertTrue((expected_dof_indices[i] == dof_index), "dof index is correct")
        for i in range(art_metatype.link_count):
            link_name = expected_link_names[i]
            link_index = art_metatype.link_indices[link_name]
            self.test_case.assertTrue((expected_link_indices[i] == link_index), "link index is correct")

        # Check parent links
        expected_parent_link_names = [
            "RootLink",
            "RootLink",
            "ChildLink1",
            "ChildLink2",
        ]
        expected_parent_link_indices = [0, 0, 1, 2]
        for i in range(art_metatype.link_count - 1):
            parent_link_name = art_metatype.link_parents[i+1]
            child_link_name = art_metatype.link_names[i+1]
            parent_link_index = art_metatype.link_parent_indices[child_link_name]
            self.test_case.assertTrue((expected_parent_link_names[i] == parent_link_name), "link parent name is correct")
            self.test_case.assertTrue((expected_parent_link_indices[i] == parent_link_index), "link parent index is correct")

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass

class TestArticulationDofProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")

        # joint limits
        limits = np.zeros((humanoids.count, humanoids.max_dofs, 2))
        limits[:, :, 0] = -0.1
        limits[:, :, 1] = 0.1
        all_indices = wp_utils.arange(humanoids.count)
        wp_limits = wp.from_numpy(limits, dtype=wp.float32, device="cpu")
        humanoids.set_dof_limits(wp_limits, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_limits().numpy(), limits))

        # joint limits indexed
        indexed_indices = [0]
        limits[indexed_indices, :, 0] = -0.2
        limits[indexed_indices, :, 1] = 2.0
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_limits = wp.from_numpy(limits, dtype=wp.float32, device="cpu")
        humanoids.set_dof_limits(wp_limits, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_limits().numpy(), limits))

        # joint stiffness
        stiffness = np.ones((humanoids.count, humanoids.max_dofs)) * 100
        all_indices = wp_utils.arange(humanoids.count)
        wp_stiffness = wp.from_numpy(stiffness, dtype=wp.float32, device="cpu")
        humanoids.set_dof_stiffnesses(wp_stiffness, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_stiffnesses().numpy(), stiffness))

        # joint stiffness indexed
        indexed_indices = [0]
        stiffness[indexed_indices, :] = 50
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_stiffness = wp.from_numpy(stiffness, dtype=wp.float32, device="cpu")
        humanoids.set_dof_stiffnesses(wp_stiffness, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_stiffnesses().numpy(), stiffness))

        # joint damping
        damping = np.ones((humanoids.count, humanoids.max_dofs)) * 100
        all_indices = wp_utils.arange(humanoids.count)
        wp_damping = wp.from_numpy(damping, dtype=wp.float32, device="cpu")
        humanoids.set_dof_dampings(wp_damping, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_dampings().numpy(), damping))

        # joint damping indexed
        indexed_indices = [0]
        damping[indexed_indices, :] = 50
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_damping = wp.from_numpy(damping, dtype=wp.float32, device="cpu")
        humanoids.set_dof_dampings(wp_damping, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_dampings().numpy(), damping))

        # joint max_force
        max_force = np.ones((humanoids.count, humanoids.max_dofs)) * 1000
        all_indices = wp_utils.arange(humanoids.count)
        wp_mf = wp.from_numpy(max_force, dtype=wp.float32, device="cpu")
        humanoids.set_dof_max_forces(wp_mf, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_max_forces().numpy(), max_force))

        # joint max_force indexed
        indexed_indices = [0]
        max_force[indexed_indices, :] = 2000
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_mf = wp.from_numpy(max_force, dtype=wp.float32, device="cpu")
        humanoids.set_dof_max_forces(wp_mf, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_max_forces().numpy(), max_force))

        # joint drive_model_properties
        drive_model_properties = np.zeros((humanoids.count, humanoids.max_dofs, 3))
        drive_model_properties[:, :, 0] = 0.1
        drive_model_properties[:, :, 1] = 0.2
        drive_model_properties[:, :, 2] = 0.3
        all_indices = wp_utils.arange(humanoids.count)
        wp_mf = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
        humanoids.set_dof_drive_model_properties(wp_mf, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_drive_model_properties().numpy(), drive_model_properties))

        # joint drive_model_properties indexed
        indexed_indices = [0]
        drive_model_properties[indexed_indices, :, 0] = 0.5
        drive_model_properties[indexed_indices, :, 1] = 0.6
        drive_model_properties[indexed_indices, :, 2] = 0.7
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_mf = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
        humanoids.set_dof_drive_model_properties(wp_mf, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_drive_model_properties().numpy(), drive_model_properties))

        # DEPRECATED
        # joint friction_coefficients
        friction_coefficients = np.ones((humanoids.count, humanoids.max_dofs))
        all_indices = wp_utils.arange(humanoids.count)
        wp_fc = wp.from_numpy(friction_coefficients, dtype=wp.float32, device="cpu")
        humanoids.set_dof_friction_coefficients(wp_fc, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_friction_coefficients().numpy(), friction_coefficients))

        # DEPRECATED
        # joint friction_coefficients indexed
        indexed_indices = [0]
        friction_coefficients[indexed_indices, :] = 0.5
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_fc = wp.from_numpy(friction_coefficients, dtype=wp.float32, device="cpu")
        humanoids.set_dof_friction_coefficients(wp_fc, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_friction_coefficients().numpy(), friction_coefficients))

         # joint friction_properties
        friction_properties = np.zeros((humanoids.count, humanoids.max_dofs, 3))
        friction_properties[:, :, 0] = 0.8
        friction_properties[:, :, 1] = 0.7
        friction_properties[:, :, 2] = 0.6
        all_indices = wp_utils.arange(humanoids.count)
        wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
        humanoids.set_dof_friction_properties(wp_fc, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_friction_properties().numpy(), friction_properties))

        # joint friction_properties indexed
        indexed_indices = [0]
        dof_indices = [5, 10]
        friction_properties[indexed_indices, dof_indices, 0] = 0.5
        friction_properties[indexed_indices, dof_indices, 1] = 0.4
        friction_properties[indexed_indices, dof_indices, 2] = 0.3
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
        humanoids.set_dof_friction_properties(wp_fc, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_friction_properties().numpy(), friction_properties))

        # joint max_velocities
        max_velocities = 1000 * np.arange(0, humanoids.count * humanoids.max_dofs).reshape(humanoids.count, humanoids.max_dofs)
        all_indices = wp_utils.arange(humanoids.count)
        wp_mv = wp.from_numpy(max_velocities, dtype=wp.float32, device="cpu")
        humanoids.set_dof_max_velocities(wp_mv, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_max_velocities().numpy(), max_velocities))

        # joint max_velocities indexed
        indexed_indices = [0]
        max_velocities[indexed_indices, :] = 2000
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_mv = wp.from_numpy(max_velocities, dtype=wp.float32, device="cpu")
        humanoids.set_dof_max_velocities(wp_mv, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_max_velocities().numpy(), max_velocities))

        # joint armature
        armature = np.ones((humanoids.count, humanoids.max_dofs))
        all_indices = wp_utils.arange(humanoids.count)
        wp_armature = wp.from_numpy(armature, dtype=wp.float32, device="cpu")
        humanoids.set_dof_armatures(wp_armature, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_armatures().numpy(), armature))

        # joint armature indexed
        indexed_indices = [0]
        armature[indexed_indices, :] = 0
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_armature = wp.from_numpy(armature, dtype=wp.float32, device="cpu")
        humanoids.set_dof_armatures(wp_armature, indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_dof_armatures().numpy(), armature))

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass


class TestArticulationJointBodyOrder(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole1")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        actor_path = self.env_template_path.AppendChild("cartpole2")
        transform = Transform((0.0, 0.0, 3.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        self.atol=1e-05
        
        for i in range(self.num_envs):
            cart_joint = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/cartJoint" % i)
            root_path = cart_joint.CreateBody0Rel().GetTargets()[0]
            child_path = cart_joint.CreateBody1Rel().GetTargets()[0]
            cart_joint.CreateBody0Rel().SetTargets([child_path])
            cart_joint.CreateBody1Rel().SetTargets([root_path])

            pole_joint = UsdPhysics.PrismaticJoint.Get(self.stage, "/envs/env%d/cartpole2/poleJoint" % i)
            root_path = pole_joint.CreateBody0Rel().GetTargets()[0]
            child_path = pole_joint.CreateBody1Rel().GetTargets()[0]
            pole_joint.CreateBody0Rel().SetTargets([child_path])
            pole_joint.CreateBody1Rel().SetTargets([root_path])

            local_pos_0 = pole_joint.GetLocalPos0Attr().Get()
            local_pos_1 = pole_joint.GetLocalPos1Attr().Get()
            pole_joint.CreateLocalPos1Attr().Set(local_pos_0)
            pole_joint.CreateLocalPos0Attr().Set(local_pos_1)

            local_rot_0 = pole_joint.GetLocalRot0Attr().Get()
            local_rot_1 = pole_joint.GetLocalRot1Attr().Get()
            pole_joint.CreateLocalRot1Attr().Set(local_rot_0)
            pole_joint.CreateLocalRot0Attr().Set(local_rot_1)

        self.apply_joint_state_api()
        self.apply_drive_api()

    def on_start(self, sim):
        self.cartpoles_1 = sim.create_articulation_view("/envs/*/cartpole1")
        self.cartpoles_2 = sim.create_articulation_view("/envs/*/cartpole2")
        self.cartpoles_1_indices = wp_utils.arange(self.cartpoles_1.count, device=sim.device)
        self.cartpoles_2_indices = wp_utils.arange(self.cartpoles_2.count, device=sim.device)
        self.check_articulation_view(self.cartpoles_1, self.num_envs, 3, 2, True)
        self.check_articulation_view(self.cartpoles_2, self.num_envs, 3, 2, True)

    def apply_joint_state_api(self):
        for i in range(self.num_envs):
            for env in range(1,3):
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(1.0)
                jointStateAPI.CreateVelocityAttr().Set(2.0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(30.0)
                jointStateAPI.CreateVelocityAttr().Set(40.0)

    def apply_drive_api(self):
        for i in range(self.num_envs):
            for env in range(1,3):
                driveAPI = UsdPhysics.DriveAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                driveAPI.CreateTargetPositionAttr(1.0)
                driveAPI.CreateTargetVelocityAttr(2.0)
                driveAPI.CreateStiffnessAttr(800.0)
                driveAPI.CreateDampingAttr(50.0)

                driveAPI = UsdPhysics.DriveAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                driveAPI.CreateTargetPositionAttr(90.)
                driveAPI.CreateTargetVelocityAttr(10.0)
                driveAPI.CreateStiffnessAttr(800.0)
                driveAPI.CreateDampingAttr(50.0)


class TestArticulationCentroidalMomentumAndMass(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("Humanoid")
        transform = Transform((1.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((-1.0, 0.0, 0.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.humanoids = sim.create_articulation_view("/envs/*/Humanoid/torso")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno==1:
            vr_ants = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
            vr_humanoids = self.humanoids.get_root_velocities().numpy().reshape((self.humanoids.count, 6)).copy()
            vr_ants[:,3]+=0.1
            vr_humanoids[:,3]+=0.5
            self.ants.set_root_velocities(self.to_warp(vr_ants), self.all_indices)
            self.humanoids.set_root_velocities(self.to_warp(vr_humanoids), self.all_indices)
            com_ants = self.ants.get_articulation_mass_center(True).numpy().reshape((self.ants.count, 3)).copy()
            com_humanoids = self.humanoids.get_articulation_mass_center(True).numpy().reshape((self.humanoids.count, 3)).copy()
            self.test_case.assertTrue(np.allclose(com_ants[:,0:1], com_ants[:,0:1]*0, rtol=1e-1, atol=1e-1), "symmetrical articulation")
            self.test_case.assertTrue(np.allclose(com_humanoids[:,0:1], com_humanoids[:,0:1]*0, rtol=1e-1, atol=1e-1), "symmetrical articulation")

        if stepno==2:
            v_humanoids = self.humanoids.get_dof_velocities().numpy().reshape((self.humanoids.count, self.humanoids.max_dofs)).copy()
            v_ants = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            v_humanoids[:,:]+=0.5
            v_ants[:,:]+=0.1
            self.humanoids.set_dof_velocities(self.to_warp(v_humanoids), self.all_indices)
            self.ants.set_dof_velocities(self.to_warp(v_ants), self.all_indices)

        if stepno==3:
            v_humanoids = self.humanoids.get_dof_velocities().numpy().reshape((self.humanoids.count, self.humanoids.max_dofs)).copy()
            v_ants = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            cmm_humanoids = self.humanoids.get_articulation_centroidal_momentum().numpy().reshape((self.humanoids.count, 6, self.humanoids.max_dofs + 7)).copy()
            cmm_ants = self.ants.get_articulation_centroidal_momentum().numpy().reshape((self.ants.count, 6, self.ants.max_dofs + 7)).copy()
            expected_cmm_humanoids = np.tile(cmm_humanoids[0], (self.humanoids.count, 1)).reshape((self.humanoids.count, 6, self.humanoids.max_dofs + 7))
            expected_cmm_ants = np.tile(cmm_ants[0], (self.ants.count, 1)).reshape((self.ants.count, 6, self.ants.max_dofs + 7))
            self.test_case.assertTrue(np.allclose(expected_cmm_humanoids, cmm_humanoids, rtol=1e-03, atol=1e-3), "all envs have similar humanoids centroidal mass")
            self.test_case.assertTrue(np.allclose(expected_cmm_ants, cmm_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants centroidal momentum")

            self.finish()


class TestHeterogeneousSceneArticulations(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "cabinet.usda")
        actor_path = self.env_template_path.AppendChild("cabinet")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "franka.usda")
        actor_path = self.env_template_path.AppendChild("franka")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)


    def on_start(self, sim):
        self.cabinet = sim.create_articulation_view("/envs/*/cabinet")
        self.franka = sim.create_articulation_view("/envs/*/franka")
        self.all_indices = wp_utils.arange(self.cabinet.count, device=sim.device)


    def on_physics_step(self, sim, stepno, dt):
        if stepno==1:
            p_cabinet = self.cabinet.get_link_transforms().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 7)).copy()
            p_franka = self.franka.get_link_transforms().numpy().reshape((self.franka.count, self.franka.max_links, 7)).copy()

            self.rp_cabinet = self.cabinet.get_root_transforms().numpy().reshape((self.cabinet.count, 7)).copy()
            self.rp_franka = self.franka.get_root_transforms().numpy().reshape((self.franka.count, 7)).copy()
            self.rp_cabinet[:,2]-=0.6
            self.rp_franka[:,2]-=0.2
            self.cabinet.set_root_transforms(self.to_warp(self.rp_cabinet), self.all_indices)
            self.franka.set_root_transforms(self.to_warp(self.rp_franka), self.all_indices)

        if stepno==2:
            root_rel_cabinet = np.zeros((self.cabinet.count, self.cabinet.max_links, 7))
            root_rel_franka = np.zeros((self.cabinet.count, self.franka.max_links, 7))

            for i in range(self.cabinet.max_links):
                root_rel_cabinet[:,i,:]=self.rp_cabinet
            for i in range(self.franka.max_links):
                root_rel_franka[:,i,:]=self.rp_franka

            p_cabinet = self.cabinet.get_link_transforms().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 7)).copy() - root_rel_cabinet
            p_franka = self.franka.get_link_transforms().numpy().reshape((self.franka.count, self.franka.max_links, 7)).copy() - root_rel_franka
            expected_p_cabinets = np.tile(p_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_links, 7))
            expected_p_frankas = np.tile(p_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_links, 7))
            self.test_case.assertTrue(np.allclose(expected_p_cabinets, p_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet link transforms")
            self.test_case.assertTrue(np.allclose(expected_p_frankas, p_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka link transforms")

        if stepno==3:
            p_cabinet = self.cabinet.get_dof_positions().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            p_franka = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            p_cabinet[:,:]+=0.3
            p_franka[:,:]+=0.5
            self.cabinet.set_dof_positions(self.to_warp(p_cabinet), self.all_indices)
            self.franka.set_dof_positions(self.to_warp(p_franka), self.all_indices)

        if stepno==4:
            p_cabinet = self.cabinet.get_dof_positions().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            p_franka = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_p_cabinets = np.tile(p_cabinet[0], (self.cabinet.count, 1))
            expected_p_frankas = np.tile(p_franka[0], (self.franka.count, 1))
            self.test_case.assertTrue(np.allclose(expected_p_cabinets, p_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet dof positions")
            self.test_case.assertTrue(np.allclose(expected_p_frankas, p_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka dof positions")

        if stepno==5:
            v_cabinet = self.cabinet.get_link_velocities().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 6)).copy()
            v_franka = self.franka.get_link_velocities().numpy().reshape((self.franka.count, self.franka.max_links, 6)).copy()

            vr_cabinet = self.cabinet.get_root_velocities().numpy().reshape((self.cabinet.count, 6)).copy()
            vr_franka = self.franka.get_root_velocities().numpy().reshape((self.franka.count, 6)).copy()

            vr_cabinet[:,:]+=0.3
            vr_franka[:,:]+=0.5

            self.cabinet.set_root_velocities(self.to_warp(vr_cabinet), self.all_indices)
            self.franka.set_root_velocities(self.to_warp(vr_franka), self.all_indices)

        if stepno==6:
            v_cabinet = self.cabinet.get_link_velocities().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 6)).copy()
            v_franka = self.franka.get_link_velocities().numpy().reshape((self.franka.count, self.franka.max_links, 6)).copy()
            expected_v_cabinets = np.tile(v_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_links, 6))
            expected_v_frankas = np.tile(v_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_links, 6))
            self.test_case.assertTrue(np.allclose(expected_v_cabinets, v_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet link velocities")
            self.test_case.assertTrue(np.allclose(expected_v_frankas, v_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka link velocities")

        if stepno==7:
            j_cabinet = self.cabinet.get_jacobians().numpy().reshape((self.cabinet.count, (self.cabinet.max_dofs) * 6 * (self.cabinet.max_links - 1))).copy()
            j_franka = self.franka.get_jacobians().numpy().reshape((self.franka.count, (self.franka.max_dofs) * 6 * (self.franka.max_links - 1))).copy()
            expected_j_cabinets = np.tile(j_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, (self.cabinet.max_dofs) * 6 * (self.cabinet.max_links - 1)))
            expected_j_frankas = np.tile(j_franka[0], (self.franka.count, 1)).reshape((self.franka.count, (self.franka.max_dofs) * 6 * (self.franka.max_links - 1)))
            self.test_case.assertTrue(np.allclose(expected_j_cabinets, j_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet Jacobian")
            self.test_case.assertTrue(np.allclose(expected_j_frankas, j_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka Jacobian")

        if stepno==8:
            cc_cabinet = self.cabinet.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            cc_franka = self.franka.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_cc_cabinet = np.tile(cc_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_dofs))
            expected_cc_frankas = np.tile(cc_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_cc_cabinet, cc_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet Coriolis and Centrifugal compensation forces")
            self.test_case.assertTrue(np.allclose(expected_cc_frankas, cc_franka, rtol=4e-03, atol=1e-3), "all envs have similar franka Coriolis and Centrifugal compensation forces")

        if stepno==9:
            g_cabinet = self.cabinet.get_gravity_compensation_forces().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            g_franka = self.franka.get_gravity_compensation_forces().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_g_cabinet = np.tile(g_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_dofs))
            expected_g_frankas = np.tile(g_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_g_cabinet, g_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet gravity compensation forces")
            self.test_case.assertTrue(np.allclose(expected_g_frankas, g_franka, rtol=4e-03, atol=1e-3), "all envs have similar franka gravity compensation forces")

        if stepno==10:
            mm_cabinet = self.cabinet.get_generalized_mass_matrices().numpy().reshape((self.cabinet.count, (self.cabinet.max_dofs) * (self.cabinet.max_dofs))).copy()
            mm_franka = self.franka.get_generalized_mass_matrices().numpy().reshape((self.franka.count, self.franka.max_dofs * self.franka.max_dofs)).copy()
            expected_mm_cabinet = np.tile(mm_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, (self.cabinet.max_dofs) * (self.cabinet.max_dofs)))
            expected_mm_frankas = np.tile(mm_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs * self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_mm_cabinet, mm_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet mass matrices")
            self.test_case.assertTrue(np.allclose(expected_mm_frankas, mm_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka mass matrices")

        if stepno==11:
            self.finish()


class TestHeterogeneousBaseArticulations(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 5.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, -5.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno==1:
            p_cartpoles = self.cartpoles.get_link_transforms().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 7)).copy()
            p_ants = self.ants.get_link_transforms().numpy().reshape((self.ants.count, self.ants.max_links, 7)).copy()

            self.rp_cartpoles = self.cartpoles.get_root_transforms().numpy().reshape((self.cartpoles.count, 7)).copy()
            self.rp_ants = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7)).copy()
            self.rp_cartpoles[:,2] -= 0.2
            self.rp_ants[:,2] -= 0.2
            self.cartpoles.set_root_transforms(self.to_warp(self.rp_cartpoles), self.all_indices)
            self.ants.set_root_transforms(self.to_warp(self.rp_ants), self.all_indices)

        if stepno==2:
            root_rel_cartpoles = np.zeros((self.cartpoles.count, self.cartpoles.max_links, 7))
            root_rel_ants = np.zeros((self.ants.count, self.ants.max_links, 7))

            for i in range(self.cartpoles.max_links):
                root_rel_cartpoles[:,i,:] = self.rp_cartpoles
            for i in range(self.ants.max_links):
                root_rel_ants[:,i,:] = self.rp_ants

            p_cartpoles = self.cartpoles.get_link_transforms().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 7)).copy() - root_rel_cartpoles
            p_ants = self.ants.get_link_transforms().numpy().reshape((self.ants.count, self.ants.max_links, 7)).copy() - root_rel_ants
            expected_p_cartpoless = np.tile(p_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_links, 7))
            expected_p_ants = np.tile(p_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_links, 7))
            self.test_case.assertTrue(np.allclose(expected_p_cartpoless, p_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles link transforms")
            self.test_case.assertTrue(np.allclose(expected_p_ants, p_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants link transforms")

        if stepno==3:
            p_cartpoles = self.cartpoles.get_dof_positions().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            p_ants = self.ants.get_dof_positions().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            p_cartpoles[:,:] += 0.5
            p_ants[:,:] += 0.1
            self.cartpoles.set_dof_positions(self.to_warp(p_cartpoles), self.all_indices)
            self.ants.set_dof_positions(self.to_warp(p_ants), self.all_indices)

        if stepno==4:
            p_cartpoles = self.cartpoles.get_dof_positions().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            p_ants = self.ants.get_dof_positions().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            expected_p_cartpoless = np.tile(p_cartpoles[0], (self.cartpoles.count, 1))
            expected_p_ants = np.tile(p_ants[0], (self.ants.count, 1))
            self.test_case.assertTrue(np.allclose(expected_p_cartpoless, p_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles dof positions")
            self.test_case.assertTrue(np.allclose(expected_p_ants, p_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants dof positions")

        if stepno==5:
            vr_cartpoles = self.cartpoles.get_root_velocities().numpy().reshape((self.cartpoles.count, 6)).copy()
            vr_ants = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
            vr_cartpoles[:,:] += 0.7
            vr_ants[:,:] += 0.9
            self.cartpoles.set_root_velocities(self.to_warp(vr_cartpoles), self.all_indices)
            self.ants.set_root_velocities(self.to_warp(vr_ants), self.all_indices)

        if stepno==6:
            v_cartpoles = self.cartpoles.get_link_velocities().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 6)).copy()
            v_ants = self.ants.get_link_velocities().numpy().reshape((self.ants.count, self.ants.max_links, 6)).copy()
            expected_v_cartpoless = np.tile(v_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_links, 6))
            expected_v_ants = np.tile(v_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_links, 6))
            self.test_case.assertTrue(np.allclose(expected_v_cartpoless, v_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles link velocities")
            self.test_case.assertTrue(np.allclose(expected_v_ants, v_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants link velocities")

        if stepno==7:
            j_cartpoles = self.cartpoles.get_jacobians().numpy().reshape((self.cartpoles.count, (self.cartpoles.max_dofs) * 6 * (self.cartpoles.max_links - 1))).copy()
            j_ants = self.ants.get_jacobians().numpy().reshape((self.ants.count, (self.ants.max_dofs + 6) * 6 * self.ants.max_links)).copy()
            expected_j_cartpoless = np.tile(j_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, (self.cartpoles.max_dofs) * 6 * (self.cartpoles.max_links - 1)))
            expected_j_ants = np.tile(j_ants[0], (self.ants.count, 1)).reshape((self.ants.count, (self.ants.max_dofs + 6) * 6 * self.ants.max_links))
            self.test_case.assertTrue(np.allclose(expected_j_cartpoless, j_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles Jacobian")
            self.test_case.assertTrue(np.allclose(expected_j_ants, j_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants Jacobian")

        if stepno==8:
            cc_cartpoles = self.cartpoles.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            cc_ants = self.ants.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.ants.count, self.ants.max_dofs + 6)).copy()
            expected_cc_cartpoless = np.tile(cc_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs))
            expected_cc_ants = np.tile(cc_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_dofs + 6))
            self.test_case.assertTrue(np.allclose(expected_cc_cartpoless, cc_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles Coriolis and Centrifugal compensation forces")
            self.test_case.assertTrue(np.allclose(expected_cc_ants, cc_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants Coriolis and Centrifugal compensation forces")

        if stepno==9:
            g_cartpoles = self.cartpoles.get_gravity_compensation_forces().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            g_ants = self.ants.get_gravity_compensation_forces().numpy().reshape((self.ants.count, self.ants.max_dofs + 6)).copy()
            expected_g_cartpoless = np.tile(g_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs))
            expected_g_ants = np.tile(g_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_dofs + 6))
            self.test_case.assertTrue(np.allclose(expected_g_cartpoless, g_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles gravity compensation forces")
            self.test_case.assertTrue(np.allclose(expected_g_ants, g_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants gravity compensation forces")

        if stepno==10:
            mm_cartpoles = self.cartpoles.get_generalized_mass_matrices().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs * self.cartpoles.max_dofs)).copy()
            mm_ants = self.ants.get_generalized_mass_matrices().numpy().reshape((self.ants.count, (self.ants.max_dofs + 6) * (self.ants.max_dofs + 6))).copy()
            expected_mm_cartpoless = np.tile(mm_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs * self.cartpoles.max_dofs))
            expected_mm_ants = np.tile(mm_ants[0], (self.ants.count, 1)).reshape((self.ants.count, (self.ants.max_dofs + 6) * (self.ants.max_dofs + 6)))
            self.test_case.assertTrue(np.allclose(expected_mm_cartpoless, mm_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles mass matrices")
            self.test_case.assertTrue(np.allclose(expected_mm_ants, mm_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants mass matrices")

        if stepno==11:
            self.finish()

class TestArticulationJointBodyOrderLimits(TestArticulationJointBodyOrder):
    def __init__(self, test_case, device_params):
            super().__init__(test_case, device_params)
            for i in range(self.num_envs):
                pole_joint_1 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole1/poleJoint" % i)
                pole_joint_1.CreateLowerLimitAttr().Set(-2)
                pole_joint_1.CreateUpperLimitAttr().Set(3)
                pole_joint_2 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/poleJoint" % i)
                pole_joint_2.CreateLowerLimitAttr().Set(-2)
                pole_joint_2.CreateUpperLimitAttr().Set(3)

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2

        if stepno==5:
            dof_limits_1 = self.cartpoles_1.get_dof_limits().numpy().reshape((self.cartpoles_1.count, num_dof, 2)).copy()
            dof_limits_2 = self.cartpoles_2.get_dof_limits().numpy().reshape((self.cartpoles_2.count, num_dof, 2)).copy()
            # print(dof_limits_1, dof_limits_2)
            self.test_case.assertTrue(np.allclose(dof_limits_1, dof_limits_2, rtol=1e-03, atol=self.atol), "similar dof limits")

            submitted_dof_limits_1 = dof_limits_1  + 0.1
            submitted_dof_limits_2 = dof_limits_2  + 0.1

            self.cartpoles_1.set_dof_limits(wp.from_numpy(submitted_dof_limits_1, dtype=wp.float32, device= "cpu"), wp_utils.arange(self.cartpoles_1.count, device="cpu"))
            self.cartpoles_1.set_dof_limits(wp.from_numpy(submitted_dof_limits_2, dtype=wp.float32, device= "cpu"), wp_utils.arange(self.cartpoles_2.count, device="cpu"))

            new_dof_limits_1 = self.cartpoles_1.get_dof_limits().numpy().reshape((self.cartpoles_1.count, num_dof, 2)).copy()
            new_dof_limits_2 = self.cartpoles_2.get_dof_limits().numpy().reshape((self.cartpoles_2.count, num_dof, 2)).copy()
            # print(new_dof_limits_1, new_dof_limits_2)
            self.test_case.assertTrue(np.allclose(new_dof_limits_1, new_dof_limits_1, rtol=1e-03, atol=self.atol), "similar dof limits")  
            self.finish()

class TestArticulationJointBodyOrderPosition(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==5:
            dof_positions_1 = self.cartpoles_1.get_dof_positions().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_positions_2 = self.cartpoles_2.get_dof_positions().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_positions_1, dof_positions_2, rtol=1e-03, atol=self.atol), "similar dof positions sign")

            submitted_dof_positions_1 = dof_positions_1  + 1
            submitted_dof_positions_2 = dof_positions_2  + 1

            self.cartpoles_1.set_dof_positions(self.to_warp(submitted_dof_positions_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_positions(self.to_warp(submitted_dof_positions_2), self.cartpoles_2_indices)

            new_dof_positions_1 = self.cartpoles_1.get_dof_positions().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_positions_2 = self.cartpoles_2.get_dof_positions().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(new_dof_positions_1, new_dof_positions_2)
            self.test_case.assertTrue(np.allclose(new_dof_positions_1, new_dof_positions_1, rtol=1e-03, atol=self.atol), "similar dof positions sign")  
            self.finish()


class TestArticulationJointBodyOrderVelocity(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==5:
            dof_velocities_1 = self.cartpoles_1.get_dof_velocities().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_velocities_2 = self.cartpoles_2.get_dof_velocities().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_velocities_1, dof_velocities_2, rtol=1e-03, atol=self.atol), "similar dof velocities sign")

            submitted_dof_velocities_1 = dof_velocities_1  + 1
            submitted_dof_velocities_2 = dof_velocities_2  + 1

            self.cartpoles_1.set_dof_velocities(self.to_warp(submitted_dof_velocities_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_velocities(self.to_warp(submitted_dof_velocities_2), self.cartpoles_2_indices)

            new_dof_velocities_1 = self.cartpoles_1.get_dof_velocities().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_velocities_2 = self.cartpoles_2.get_dof_velocities().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(new_dof_velocities_1, new_dof_velocities_2)
            self.test_case.assertTrue(np.allclose(new_dof_velocities_1, new_dof_velocities_1, rtol=1e-03, atol=self.atol), "similar dof velocities sign")   
            self.finish()

class TestArticulationJointBodyOrderPositionTarget(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==5:
            dof_positions_1 = self.cartpoles_1.get_dof_position_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_positions_2 = self.cartpoles_2.get_dof_position_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(dof_positions_1, dof_positions_2)
            self.test_case.assertTrue(np.allclose(dof_positions_1, dof_positions_2, rtol=1e-03, atol=self.atol), "similar dof position targets sign")

            submitted_dof_positions_1 = dof_positions_1  + 1
            submitted_dof_positions_2 = dof_positions_2  + 1

            self.cartpoles_1.set_dof_position_targets(self.to_warp(submitted_dof_positions_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_position_targets(self.to_warp(submitted_dof_positions_2), self.cartpoles_2_indices)

            new_dof_positions_1 = self.cartpoles_1.get_dof_position_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_positions_2 = self.cartpoles_2.get_dof_position_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(new_dof_positions_1, new_dof_positions_2)
            self.test_case.assertTrue(np.allclose(new_dof_positions_1, new_dof_positions_1, rtol=1e-03, atol=self.atol), "similar dof position targets sign")  
            self.finish()


class TestArticulationJointBodyOrderVelocityTarget(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==5:
            dof_velocity_targets_1 = self.cartpoles_1.get_dof_velocity_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_velocity_targets_2 = self.cartpoles_2.get_dof_velocity_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_velocity_targets_1, dof_velocity_targets_2, rtol=1e-03, atol=self.atol), "similar dof velocity targets sign")

            submitted_dof_velocity_targets_1 = dof_velocity_targets_1  + 1
            submitted_dof_velocity_targets_2 = dof_velocity_targets_2  + 1

            self.cartpoles_1.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets_2), self.cartpoles_2_indices)

            new_dof_velocity_targets_1 = self.cartpoles_1.get_dof_velocity_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_velocity_targets_2 = self.cartpoles_2.get_dof_velocity_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(new_dof_velocity_targets_1, new_dof_velocity_targets_2)
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets_1, new_dof_velocity_targets_1, rtol=1e-03, atol=self.atol), "similar dof velocity targets sign") 
            self.finish()


class TestArticulationJointBodyOrderDofForce(TestArticulationJointBodyOrder):
    def __init__(self,test_case, device_params):
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            for env in [1,2]:
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==4:
            dof_actuation_forces_1 = self.cartpoles_1.get_dof_actuation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_actuation_forces_2 = self.cartpoles_2.get_dof_actuation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_actuation_forces_1, dof_actuation_forces_2, rtol=1e-03, atol=self.atol), "similar dof actuation forces")

            submitted_dof_actuation_forces_1 = dof_actuation_forces_1  + 1
            submitted_dof_actuation_forces_2 = dof_actuation_forces_2  + 1

            self.cartpoles_1.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces_2), self.cartpoles_2_indices)

            new_dof_actuation_forces_1 = self.cartpoles_1.get_dof_actuation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_actuation_forces_2 = self.cartpoles_2.get_dof_actuation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(new_dof_actuation_forces_1, new_dof_actuation_forces_2)
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces_1, new_dof_actuation_forces_1, rtol=1e-03, atol=self.atol), "similar dof actuation forces")

            # using deprecated API
            dof_gravity_forces_1 = self.cartpoles_1.get_generalized_gravity_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_gravity_forces_2 = self.cartpoles_2.get_generalized_gravity_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(dof_gravity_forces_1, dof_gravity_forces_2)
            self.test_case.assertTrue(np.allclose(dof_gravity_forces_1, dof_gravity_forces_2, rtol=1e-03, atol=self.atol), "similar gravity compensation forces")

            # using new API
            dof_gravity_forces_3 = self.cartpoles_1.get_gravity_compensation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_gravity_forces_4 = self.cartpoles_2.get_gravity_compensation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(dof_gravity_forces_1, dof_gravity_forces_2)
            self.test_case.assertTrue(np.allclose(dof_gravity_forces_3, dof_gravity_forces_4, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")

            # using deprecated API
            dof_coriolis_and_centrifugal_forces_1 = self.cartpoles_1.get_coriolis_and_centrifugal_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_coriolis_and_centrifugal_forces_2 = self.cartpoles_2.get_coriolis_and_centrifugal_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(dof_coriolis_and_centrifugal_forces_1, dof_coriolis_and_centrifugal_forces_2)
            self.test_case.assertTrue(np.allclose(dof_coriolis_and_centrifugal_forces_1, dof_coriolis_and_centrifugal_forces_2, rtol=1e-03, atol=self.atol), "similar coriolis and centrifugal forces")

            # using new API
            dof_coriolis_and_centrifugal_forces_3 = self.cartpoles_1.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_coriolis_and_centrifugal_forces_4 = self.cartpoles_2.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(dof_coriolis_and_centrifugal_forces_1, dof_coriolis_and_centrifugal_forces_2)
            self.test_case.assertTrue(np.allclose(dof_coriolis_and_centrifugal_forces_3, dof_coriolis_and_centrifugal_forces_4, rtol=1e-03, atol=self.atol), "similar coriolis and centrifugal forces")

            # comparing old and new API
            self.test_case.assertTrue(np.allclose(dof_gravity_forces_1, dof_gravity_forces_3, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")
            self.test_case.assertTrue(np.allclose(dof_gravity_forces_2, dof_gravity_forces_4, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")
            self.test_case.assertTrue(np.allclose(dof_coriolis_and_centrifugal_forces_1, dof_coriolis_and_centrifugal_forces_3, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")
            self.test_case.assertTrue(np.allclose(dof_coriolis_and_centrifugal_forces_2, dof_coriolis_and_centrifugal_forces_4, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")

            self.finish()


class TestArticulationJointBodyOrderLinkForce(TestArticulationJointBodyOrder):
    def __init__(self,test_case, device_params):
        super().__init__(test_case, device_params)
        target_p_pole = 45
        target_p_cart = 1
        for i in range(self.num_envs):
            for env in [1,2]:
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                # negating the target signs to avoid the mirroring artifacts 
                target = target_p_cart if env==1 else -target_p_cart
                driveAPI = UsdPhysics.DriveAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                driveAPI.CreateTargetPositionAttr(target)
                driveAPI.CreateTargetVelocityAttr(0.0)
                target = target_p_pole if env==1 else -target_p_pole
                driveAPI = UsdPhysics.DriveAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                driveAPI.CreateTargetPositionAttr(target)
                driveAPI.CreateTargetVelocityAttr(0.0)

    def transform_to_physx_child(self, view, joint, link_forces, i, j):
        body_0_path = joint.GetBody0Rel().GetTargets()[0]
        body_1_path = joint.GetBody1Rel().GetTargets()[0]
        body_0_name = self.stage.GetPrimAtPath(body_0_path).GetName()
        body_1_name = self.stage.GetPrimAtPath(body_1_path).GetName()
        body_0_tensor_idx = view.get_metatype(i).link_indices[body_0_name]
        body_1_tensor_idx = view.get_metatype(i).link_indices[body_1_name]
        body_0_xform = view.get_link_transforms().numpy()[i,body_0_tensor_idx,:]
        body_1_xform = view.get_link_transforms().numpy()[i,body_1_tensor_idx,:]
        body_0_link_p = Gf.Vec3f(*body_0_xform[0:3].tolist())
        body_0_link_q = Gf.Quatf(body_0_xform[-1].item(), *body_0_xform[3:6].tolist())
        body_1_link_p = Gf.Vec3f(*body_1_xform[0:3].tolist())
        body_1_link_q = Gf.Quatf(body_1_xform[-1].item(), *body_1_xform[3:6].tolist())
        body_0_scale = self.stage.GetPrimAtPath(body_0_path).GetAttribute('xformOp:scale').Get()
        body_1_scale = self.stage.GetPrimAtPath(body_1_path).GetAttribute('xformOp:scale').Get()
        body_0_p = joint.GetLocalPos0Attr().Get()
        body_0_q = joint.GetLocalRot0Attr().Get()
        body_1_p = joint.GetLocalPos1Attr().Get()
        body_1_q = joint.GetLocalRot1Attr().Get()
        body_1_xform_p = body_1_link_p + Gf.Rotation(body_1_link_q).TransformDir(Gf.CompMult(body_1_scale, body_1_p)) 
        body_0_xform_p = body_0_link_p + Gf.Rotation(body_0_link_q).TransformDir(Gf.CompMult(body_0_scale, body_0_p))
        d_global =  body_0_xform_p - body_1_xform_p
        F = Gf.Vec3f(*(link_forces[i, j ,0:3].tolist()))
        T = Gf.Vec3f(*(link_forces[i, j ,3:6].tolist()))
        d = Gf.Rotation(body_1_link_q *  body_1_q).GetInverse().TransformDir(d_global) #distance in body_1 link joint frame
        T-= Gf.Cross(d, F)
        # print("body_1_name: ", body_1_name, body_1_link_p, body_1_link_q , body_1_p, body_1_q, body_1_xform_p )
        # print("body_0_name: ", body_0_name, body_0_link_p, body_0_link_q, body_0_p, body_0_q, body_0_xform_p)
        J = Gf.Rotation(body_0_link_q * body_0_q).GetInverse() * Gf.Rotation(body_1_link_q * body_1_q)
        link_forces[i, j ,0:3] = J.TransformDir(F)
        link_forces[i, j ,3:6] = J.TransformDir(T)

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=2
        if stepno==5:
            projected_forces_1 = self.cartpoles_1.get_dof_projected_joint_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            projected_forces_2 = self.cartpoles_2.get_dof_projected_joint_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            # print(projected_forces_1,"\n", projected_forces_2)
            self.test_case.assertTrue(np.allclose(projected_forces_1, -projected_forces_2, rtol=1e-03, atol=self.atol), "similar projected dof forces")

            # joint child local forces
            link_forces_1 = self.cartpoles_1.get_link_incoming_joint_force().numpy().reshape((self.cartpoles_1.count, 3, 6)).copy()
            link_forces_2 = self.cartpoles_2.get_link_incoming_joint_force().numpy().reshape((self.cartpoles_2.count, 3, 6)).copy()
            # print(link_forces_1,"\n", link_forces_2)
        
            for i in range(self.num_envs):
                cart_joint_2 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/cartJoint" % i)
                self.transform_to_physx_child(self.cartpoles_2, cart_joint_2, link_forces_2, i, 1)

                pole_joint_2 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/poleJoint" % i)
                self.transform_to_physx_child(self.cartpoles_2, pole_joint_2, link_forces_2, i, 2)

            # print(link_forces_1,"\n", link_forces_2)
            self.test_case.assertTrue(np.allclose(link_forces_1, -link_forces_2, rtol=0.001, atol=0.001*np.linalg.norm(link_forces_1)), "similar link forces in global frame")
            self.finish()

class TestArticulationGetSet(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs = 16
        grid_params = GridParams(self.num_envs, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.atol=1e-05

        for i in range(self.num_envs):
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env%d/ant/torso" % i))
            articulation_api.CreateEnabledSelfCollisionsAttr().Set(False)

    def on_start(self, sim):
        # self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.ants = sim.create_articulation_view(["/envs/env[0-5]/ant/torso", "/envs/env[6-9]/ant/torso", "/envs/env1[0-5]/ant/torso"])
        self.ants_subset = sim.create_articulation_view("/envs/env[0-5]/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.check_articulation_view(self.ants_subset, min(self.num_envs, 6), 9, 8, True)
        
class TestArticulationKinematicUpdate(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof)).copy()
        # TODO: check whether the link transforms stay the same after the first step with the condition that all the forces and velocities are zero
        # self.ants.set_disable_gravities(wp.from_numpy(np.ones((self.num_envs, self.ants.max_links)), dtype=wp.uint8, device="cpu"), 
        #                                 wp_utils.arange(self.ants.count, device="cpu"))
        # self.ants.set_dof_velocities(self.to_warp(np.zeros((self.ants.count, num_dof))), self.all_indices)
        # self.ants.set_root_velocities(self.to_warp(np.zeros((self.ants.count, 6))), self.all_indices)
        # tests for get after set
        if stepno==1:
            self.current_xforms = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7)).copy()
            submitted_dof_positions = dof_positions + 1.0
            self.ants.set_dof_positions(self.to_warp(submitted_dof_positions), self.all_indices)
            # NOTE: links kinematics are already updated after using set_dof_positions for CPU but for GPU they will be updated after the following
            sim.update_articulations_kinematic()
            self.new_xforms = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7)).copy()
            self.test_case.assertFalse(np.allclose(self.new_xforms, self.current_xforms, rtol=1e-03, atol=1e-03), "current transforms should be different from the one before setting the joint position")

            # reset the dof positions to their original values
            self.ants.set_dof_positions(self.to_warp(dof_positions), self.all_indices)
            sim.update_articulations_kinematic()
            self.new_xforms = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7)).copy()
            self.test_case.assertTrue(np.allclose(self.new_xforms, self.current_xforms, rtol=1e-03, atol=1e-03), "updated transforms should be similar to the original transforms")
            self.finish()

class TestArticulationGetSetRootTransforms(TestArticulationGetSet):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim
        roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7)).copy()
        roots_subset = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7)).copy()
        delta =np.array([0,0,1.0,0,0,0,0])

        # # tests for get after set
        if stepno==1:
            submitted_roots = roots  + delta
            self.ants.set_root_transforms(self.to_warp(submitted_roots), self.all_indices)
            # new roots should be available to read
            new_roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7))
            # print(new_roots[:,2], submitted_roots[:,2], roots[:,2])
            self.test_case.assertTrue(np.allclose(new_roots[:,2], roots[:,2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[:,2], submitted_roots[:,2], rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            submitted_roots = roots  + delta
            subset_indices = self.to_warp(self.all_indices.numpy()[0:5], wp.uint32) 
            self.ants.set_root_transforms(self.to_warp(submitted_roots), subset_indices)
            # new roots should be available to read
            new_roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7))
            # print(new_roots[0:5,2], submitted_roots[0:5,2], roots[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_roots[0:5,2], roots[0:5,2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[0:5,2], submitted_roots[0:5,2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[5:,:], roots[5:,:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[5:,2], submitted_roots[5:,2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_roots = roots_subset  + delta
            self.ants_subset.set_root_transforms(self.to_warp(submitted_roots), subset_indices)
            # new roots velocities should be available to read
            new_roots = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7))
            # print(new_roots[0:5,2], submitted_roots[0:5,2], roots[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_roots, roots_subset + delta, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots, submitted_roots, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetRootVelocities(TestArticulationGetSet):
    
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
        roots_subset_vels = self.ants_subset.get_root_velocities().numpy().reshape((self.ants_subset.count, 6)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_root_vels = root_vels  + 0.5
            self.ants.set_root_velocities(self.to_warp(submitted_root_vels), self.all_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6))
            # print(new_root_vels, "\n", submitted_root_vels, "\n", root_vels)
            self.test_case.assertTrue(np.allclose(new_root_vels, root_vels + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels, submitted_root_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:5], wp.uint32) 
            submitted_root_vels = root_vels  + 1
            self.ants.set_root_velocities(self.to_warp(submitted_root_vels), subset_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6))
            # print(new_root_vels[0:5], "\n", submitted_root_vels[0:5], "\n", root_vels[0:5])
            self.test_case.assertTrue(np.allclose(new_root_vels[0:5], root_vels[0:5] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[5:], root_vels[5:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[0:5], submitted_root_vels[0:5], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[5:], submitted_root_vels[5:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_root_vels = roots_subset_vels  + 1
            self.ants_subset.set_root_velocities(self.to_warp(submitted_root_vels), subset_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants_subset.get_root_velocities().numpy().reshape((self.ants_subset.count, 6))
            ind=subset_indices.numpy()
            # print(new_root_vels[ind], "\n", submitted_root_vels[ind], "\n", roots_subset_vels[ind])
            self.test_case.assertTrue(np.allclose(new_root_vels, roots_subset_vels + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels, submitted_root_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetDofPositions(TestArticulationGetSet):
    
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_positions = self.ants_subset.get_dof_positions().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_dof_positions = dof_positions  + 0.5
            self.ants.set_dof_positions(self.to_warp(submitted_dof_positions), self.all_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_positions[:,0], submitted_dof_positions[:,0], dof_positions[:,0])
            self.test_case.assertTrue(np.allclose(new_dof_positions, dof_positions + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions, submitted_dof_positions, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32) 
            submitted_dof_positions = dof_positions  + 1
            self.ants.set_dof_positions(self.to_warp(submitted_dof_positions), subset_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_positions[:,0], submitted_dof_positions[:,0], dof_positions[:,0])
            self.test_case.assertTrue(np.allclose(new_dof_positions[0:2], dof_positions[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[2:], dof_positions[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[0:2], submitted_dof_positions[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[2:], submitted_dof_positions[2:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_positions = dofs_subset_positions  + 1
            self.ants_subset.set_dof_positions(self.to_warp(submitted_dof_positions), subset_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants_subset.get_dof_positions().numpy().reshape((self.ants_subset.count, num_dof))
            # print(new_dof_positions[:,0], submitted_dof_positions[:,0], dofs_subset_positions[:,0])
            self.test_case.assertTrue(np.allclose(new_dof_positions, dofs_subset_positions + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions, submitted_dof_positions, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetDofPositionTarget(TestArticulationGetSet):
    
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_position_targets = self.ants_subset.get_dof_position_targets().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_dof_position_targets = dof_position_targets  + 0.5
            self.ants.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), self.all_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_position_targets[0], submitted_dof_position_targets[0], dof_position_targets[0])
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, dof_position_targets + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, submitted_dof_position_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32) 
            submitted_dof_position_targets = dof_position_targets  + 1
            self.ants.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), subset_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[0:2], dof_position_targets[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[2:], dof_position_targets[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[0:2], submitted_dof_position_targets[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[2:], submitted_dof_position_targets[2:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_position_targets = dofs_subset_position_targets  + 1
            self.ants_subset.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), subset_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants_subset.get_dof_position_targets().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, dofs_subset_position_targets + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, submitted_dof_position_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetDofVelocities(TestArticulationGetSet):
    
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_vels = self.ants_subset.get_dof_velocities().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_dof_vels = dof_vels  + 0.5
            self.ants.set_dof_velocities(self.to_warp(submitted_dof_vels), self.all_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_vels[0], submitted_dof_vels[0], dof_vels[0])
            self.test_case.assertTrue(np.allclose(new_dof_vels, dof_vels + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels, submitted_dof_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32) 
            submitted_dof_vels = dof_vels  + 1
            self.ants.set_dof_velocities(self.to_warp(submitted_dof_vels), subset_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_vels[0:2], dof_vels[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[2:], dof_vels[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[0:2], submitted_dof_vels[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[2:], submitted_dof_vels[2:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_vels = dofs_subset_vels  + 1
            self.ants_subset.set_dof_velocities(self.to_warp(submitted_dof_vels), subset_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants_subset.get_dof_velocities().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_vels, dofs_subset_vels + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels, submitted_dof_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetDofVelocityTarget(TestArticulationGetSet):
    
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_velocity_targets = self.ants_subset.get_dof_velocity_targets().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_dof_velocity_targets = dof_velocity_targets  + 0.5
            self.ants.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), self.all_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_velocity_targets[0], submitted_dof_velocity_targets[0], dof_velocity_targets[0])
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, dof_velocity_targets + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, submitted_dof_velocity_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32) 
            submitted_dof_velocity_targets = dof_velocity_targets  + 1
            self.ants.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), subset_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[0:2], dof_velocity_targets[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[2:], dof_velocity_targets[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[0:2], submitted_dof_velocity_targets[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[2:], submitted_dof_velocity_targets[2:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_velocity_targets = dofs_subset_velocity_targets  + 1
            self.ants_subset.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), subset_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants_subset.get_dof_velocity_targets().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, dofs_subset_velocity_targets + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, submitted_dof_velocity_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationGetSetDofActuationForces(TestArticulationGetSet):
    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim 
        num_dof=self.ants.max_dofs
        dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_actuation_forces = self.ants_subset.get_dof_actuation_forces().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_dof_actuation_forces = dof_actuation_forces  + 0.5
            self.ants.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), self.all_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof))
            # print(new_dof_actuation_forces[0], submitted_dof_actuation_forces[0], dof_actuation_forces[0])
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, dof_actuation_forces + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, submitted_dof_actuation_forces, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32) 
            submitted_dof_actuation_forces = dof_actuation_forces  + 1
            self.ants.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), subset_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[0:2], dof_actuation_forces[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[2:], dof_actuation_forces[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[0:2], submitted_dof_actuation_forces[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[2:], submitted_dof_actuation_forces[2:]-1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_actuation_forces = dofs_subset_actuation_forces  + 1
            self.ants_subset.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), subset_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants_subset.get_dof_actuation_forces().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, dofs_subset_actuation_forces + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, submitted_dof_actuation_forces, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()


class TestArticulationGetSetAppliedForces(TestArticulationGetSet):
    def __init__(self, test_case, device_params, is_global=False):
        super().__init__(test_case, device_params)
        self.is_global=is_global

    def on_start(self, sim):  
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.ants_subset = sim.create_articulation_view("/envs/env[0-5]/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.check_articulation_view(self.ants_subset, 6, 9, 8, True)      
        force_offset = 1
        transforms= self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))
        positions=transforms[:,:,0:3]
        rotations=transforms[:,:,3:7]
        self.indices = wp_utils.arange(self.ants.count, device=sim.device)
        gForce=wp.vec3(0.0, 0.0, 10.0)
        if self.is_global:
            positions[:,:,0:3]+=np.array([0,0,force_offset])
            # forces = wp_utils.fill_vec3(self.ants.count, value=gForce, device=sim.device)
            self.forces_wp=wp_utils.fill_vec3(self.ants.count* self.ants.max_links, value=gForce, device=sim.device)
            self.force_subset_wp=wp_utils.fill_vec3(6*self.ants.max_links, value=gForce, device=sim.device)
        else:
            positions[:,:,0:3]=np.array([0,0,force_offset])
            # Trasform the gForce to local space
            from pxr import Gf
            forces = np.zeros((self.num_envs,self.ants.max_links,3))
            for e in range(self.num_envs):
                for b in range(self.ants.max_links):
                    rot=rotations[e,b,:].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    forces[e, b, :]=lForce
            forces_wp = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
            force_subset_wp = wp.from_numpy(forces[0:6].flatten(), dtype=wp.float32, device=sim.device)
            self.forces_wp=forces_wp
            self.force_subset_wp=force_subset_wp

        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.positions_subset_wp = wp.from_numpy(positions[0:6].flatten(), dtype=wp.float32, device=sim.device)


    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim
        # # tests for get after set for all bodies
        if stepno==1:
            self.init_height= self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[:,:,2].copy()
            self.ants.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, self.indices, self.is_global)
        if stepno==10:
            height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[:,:,2].copy()
            self.test_case.assertTrue((height > self.init_height).all(), "larger heights")

        # tests for setting a subsets of envs (last 6 envs)
        if stepno==20:
            self.init_height= self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[-6:,:,2].copy()
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 *self.ants.max_links:], wp.uint32) 
            self.ants.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, subset_indices, self.is_global)
        if stepno==30:
            height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[-6:,:,2].copy()
            self.test_case.assertTrue((height >= self.init_height-0.0001).all(), "larger heights")

        # # tests for setting via a different view and subset (first 6 envs)
        if stepno==40:
            self.init_height= self.ants_subset.get_link_transforms().numpy().reshape((6, self.ants_subset.max_links, 7))[:,:,2].copy()
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            self.ants_subset.apply_forces_and_torques_at_position(self.force_subset_wp, None, self.positions_subset_wp, subset_indices, self.is_global)
        if stepno==50:
            height = self.ants_subset.get_link_transforms().numpy().reshape((6, self.ants_subset.max_links, 7))[:,:,2].copy()
            self.test_case.assertTrue((height >= self.init_height).all(), "larger heights")
            self.finish()


class TestArticulationGetSetLinkGravity(TestArticulationGetSet):
    def on_physics_step(self, sim, stepno, dt):
        physics_scene = UsdPhysics.Scene.Define(self.stage, self.scene_path)
        physics_scene.CreateGravityMagnitudeAttr().Set(10)
        self.all_indices = wp_utils.arange(self.ants.count, device="cpu")
        self.sim=sim 
        link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links)).copy()
        # # tests for get after set
        if stepno==1:
            submitted_link_gravities = np.ones((self.ants.count, self.ants.max_links), dtype=np.uint8)
            self.ants.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), self.all_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links))
            # print(new_link_gravities[0], submitted_link_gravities[0], link_gravities[0])
            self.test_case.assertTrue((new_link_gravities==link_gravities + 1).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities==submitted_link_gravities).all(), "get returns the current set values")

        if stepno==2:
            subset_indices = wp.from_numpy(self.all_indices.numpy()[0:2], wp.uint32, device="cpu")
            submitted_link_gravities = np.zeros((self.ants.count, self.ants.max_links), dtype=np.uint8)
            self.ants.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), subset_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links))
            self.test_case.assertTrue((new_link_gravities[0:2]==link_gravities[0:2] - 1).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[2:]==link_gravities[2:]).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[0:2]==submitted_link_gravities[0:2]).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[2:]==submitted_link_gravities[2:] + 1).all(), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.ants_subset.count)
            submitted_link_gravities = np.ones((self.ants_subset.count, self.ants_subset.max_links), dtype=np.uint8)
            # disable only a subset of link gravities
            submitted_link_gravities[:,5:] = 0
            self.ants_subset.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), subset_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants_subset.get_disable_gravities().numpy().reshape((self.ants_subset.count, self.ants_subset.max_links))
            self.test_case.assertTrue((new_link_gravities==submitted_link_gravities).all(), "get returns the current set values")

        if stepno==20:
            roots_heights = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7))[:,2]
            enabled_gravity_root_heights = roots_heights[:5]
            disabled_gravity_root_heights = roots_heights[5:]
            self.test_case.assertTrue((np.mean(enabled_gravity_root_heights) < np.mean(disabled_gravity_root_heights)).all(), "lower heights for envs where gravity is active")
            self.finish()

class TestRigidBodiesGetSet(GridTestBase):
    def __init__(self, test_case, device_params, is_global=False):
        # set up stage
        self.num_envs=16
        grid_params = GridParams(self.num_envs, 4.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 20
        self.is_global=is_global
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.body_per_env=0
        # set a known mass
        mass = 0.5
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.body_per_env+=3

        transform = Transform((-0.5, 0.0, 0.0))
        ball = self.create_rigid_ball(actor_path.AppendChild("right_ball"), transform, 0.1)
        mass_api = UsdPhysics.MassAPI(ball)
        mass_api.GetMassAttr().Set(mass)
        self.body_per_env+=1

        transform = Transform((0.5, 0.0, 0.0))
        ball = self.create_rigid_ball(actor_path.AppendChild("left_ball"), transform, 0.1)
        mass_api = UsdPhysics.MassAPI(ball)
        mass_api.GetMassAttr().Set(mass)
        self.body_per_env+=1
        self.atol=1e-05

class TestRigidBodiesGetSetAppliedForces(TestRigidBodiesGetSet):
    def on_start(self, sim):

        self.rb_view = sim.create_rigid_body_view("/envs/*/ant/right_ball|left_ball|right_back_leg|right_back_foot|torso")
        self.rb_view_subset = sim.create_rigid_body_view("/envs/env[0-5]/ant/right_ball|left_ball|right_back_leg|right_back_foot|torso")
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.check_rigid_body_view(self.rb_view_subset, 6 * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=sim.device)
        
        force_offset = 1
        transforms= self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))
        positions=transforms[:,:,0:3]
        rotations=transforms[:,:,3:7]
        self.indices = wp_utils.arange(self.rb_view.count, device=sim.device)
        gForce=wp.vec3(0.0, 0.0, 150.0)
        if self.is_global:
            positions[:,:,0:3]+=np.array([0,0,force_offset])
            forces = wp_utils.fill_vec3(self.rb_view.count, value=gForce, device=sim.device)
        else:
            positions[:,:,0:3]=np.array([0,0,force_offset])
            # Trasform the gForce to local space
            from pxr import Gf
            forces = np.zeros((self.num_envs,self.body_per_env,3))
            for e in range(self.num_envs):
                for b in range(self.body_per_env):
                    rot=rotations[e,b,:].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    forces[e, b, :]=lForce
    
        forces_wp = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
        force_subset_wp = wp.from_numpy(forces[0:6].flatten(), dtype=wp.float32, device=sim.device)
        self.forces_wp=forces_wp
        self.force_subset_wp=force_subset_wp
        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.positions_subset_wp = wp.from_numpy(positions[0:6].flatten(), dtype=wp.float32, device=sim.device)


    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim
        # # tests for get after set for all bodies
        if stepno==1:
            self.init_height= self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[:,:,2].copy()
            self.rb_view.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, self.indices, self.is_global)
        if stepno==10:
            height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[:,:,2].copy()
            self.test_case.assertTrue((height > self.init_height).all(), "larger heights")

        # tests for setting a subsets of envs (last 6 envs)
        if stepno==60:
            self.init_height= self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[-6:,:,2].copy()
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 *self.body_per_env:], wp.uint32) 
            self.rb_view.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, subset_indices, self.is_global)
        if stepno==70:
            height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[-6:,:,2].copy()
            self.test_case.assertTrue((height >= self.init_height-0.0001).all(), "larger heights")

        # # tests for setting via a different view and subset (first 6 envs)
        if stepno==100:
            self.init_height= self.rb_view_subset.get_transforms().numpy().reshape((6, self.body_per_env, 7))[:,:,2].copy()
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)
            self.rb_view_subset.apply_forces_and_torques_at_position(self.force_subset_wp, None, self.positions_subset_wp, subset_indices, self.is_global)
        if stepno==110:
            height = self.rb_view_subset.get_transforms().numpy().reshape((6, self.body_per_env, 7))[:,:,2].copy()
            self.test_case.assertTrue((height >= self.init_height).all(), "larger heights")
            self.finish()

class TestRigidBodiesGetSetTransforms(TestRigidBodiesGetSet):
    def on_start(self, sim):
        self.body_per_env = 3
        self.rb_view = sim.create_rigid_body_view("/envs/*/ant/right_ball|left_ball|torso")
        self.rb_view_subset = sim.create_rigid_body_view("/envs/env[0-5]/ant/right_ball|left_ball|torso")
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.check_rigid_body_view(self.rb_view_subset, 6 * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim
        transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7)).copy()
        transforms_subset = self.rb_view_subset.get_transforms().numpy().reshape((self.rb_view_subset.count, 7)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_transforms = transforms  + np.array([0,0,1.0,0,0,0,0])
            self.rb_view.set_transforms(self.to_warp(submitted_transforms), self.all_indices)
            # new transforms should be available to read
            new_transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7))
            # print(new_transforms[:,2], submitted_transforms[:,2], transforms[:,2])
            self.test_case.assertTrue(np.allclose(new_transforms[:,2], transforms[:,2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:,2], submitted_transforms[:,2], rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            submitted_transforms = transforms  + np.array([0,0,1.0,0,0,0,0])
            subset_indices = self.to_warp(self.all_indices.numpy()[-6*self.body_per_env:], wp.uint32) 
            self.rb_view.set_transforms(self.to_warp(submitted_transforms), subset_indices)
            # new transforms should be available to read
            new_transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7))
            # print(new_transforms[0:5,2], submitted_transforms[0:5,2], transforms[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_transforms[-6*self.body_per_env:,2], transforms[-6*self.body_per_env:,2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[-6*self.body_per_env:,2], submitted_transforms[-6*self.body_per_env:,2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:(self.num_envs-6)*self.body_per_env,:], transforms[:(self.num_envs-6)*self.body_per_env,:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:(self.num_envs-6)*self.body_per_env,2], submitted_transforms[:(self.num_envs-6)*self.body_per_env,2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)
            delta =np.array([0,0,1.0,0,0,0,0])
            submitted_transforms = transforms_subset  + delta
            self.rb_view_subset.set_transforms(self.to_warp(submitted_transforms), subset_indices)
            # new transforms velocities should be available to read
            new_transforms = self.rb_view_subset.get_transforms().numpy().reshape((self.rb_view_subset.count, 7))
            # print(new_transforms[0:5,2], submitted_transforms[0:5,2], transforms[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_transforms, transforms_subset + delta, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms, submitted_transforms, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestRigidBodiesGetSetVelocities(TestRigidBodiesGetSet):
    def on_start(self, sim):
        self.body_per_env = 3
        self.rb_view = sim.create_rigid_body_view("/envs/*/ant/right_ball|left_ball|torso")
        self.rb_view_subset = sim.create_rigid_body_view("/envs/env[0-5]/ant/right_ball|left_ball|torso")
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.check_rigid_body_view(self.rb_view_subset, 6 * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        self.sim=sim
        velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6)).copy()
        velocities_subset = self.rb_view_subset.get_velocities().numpy().reshape((self.rb_view_subset.count, 6)).copy()

        # # tests for get after set
        if stepno==1:
            submitted_velocities = velocities  + np.array([0,0,1,0,0,0])
            self.rb_view.set_velocities(self.to_warp(submitted_velocities), self.all_indices)
            # new velocities should be available to read
            new_velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6))
            # print(new_velocities[:,2], submitted_velocities[:,2], velocities[:,2])
            self.test_case.assertTrue(np.allclose(new_velocities[:,2], velocities[:,2] + 1, rtol=1e-02, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:,2], submitted_velocities[:,2], rtol=1e-02, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno==2:
            submitted_velocities = velocities  + np.array([0,0,1,0,0,0])
            subset_indices = self.to_warp(self.all_indices.numpy()[-6*self.body_per_env:], wp.uint32) 
            self.rb_view.set_velocities(self.to_warp(submitted_velocities), subset_indices)
            # new velocities should be available to read
            new_velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6))
            # print(new_velocities[0:5,2], submitted_velocities[0:5,2], velocities[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_velocities[-6*self.body_per_env:,2], velocities[-6*self.body_per_env:,2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[-6*self.body_per_env:,2], submitted_velocities[-6*self.body_per_env:,2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:(self.num_envs-6)*self.body_per_env,:], velocities[:(self.num_envs-6)*self.body_per_env,:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:(self.num_envs-6)*self.body_per_env,2], submitted_velocities[:(self.num_envs-6)*self.body_per_env,2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno==3:
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)
            submitted_velocities = velocities_subset  + 1
            self.rb_view_subset.set_velocities(self.to_warp(submitted_velocities), subset_indices)
            # new velocities velocities should be available to read
            new_velocities = self.rb_view_subset.get_velocities().numpy().reshape((self.rb_view_subset.count, 6))
            # print(new_velocities[0:5,2], submitted_velocities[0:5,2], velocities[0:5,2] )
            self.test_case.assertTrue(np.allclose(new_velocities, velocities_subset + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities, submitted_velocities, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno==5:   
            self.finish()

class TestArticulationRoots(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, self.num_envs, 9, 8, True)

        self.ants = ants
        self.all_indices = wp_utils.arange(ants.count, device=sim.device)

        # call concrete subclass implementation
        self.on_start_impl(sim)

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationRootTransforms(TestArticulationRoots):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        rt = self.ants.get_root_transforms()
        lt = self.ants.get_link_transforms()
        rt_np = rt.numpy().reshape((self.ants.count, 7))
        lt_np = lt.numpy().reshape((self.ants.count, self.ants.max_links, 7))
        #print(rt_np)
        #print(lt_np[..., :3])
        rt_z_step = np.linspace(0.0, 1.0, self.ants.count, dtype=np.float32)
        lt_z_step = np.repeat(rt_z_step, self.ants.max_links).reshape((self.ants.count, self.ants.max_links))
        #print(rt_z_step)
        #print(lt_z_step)
        rt_np[..., 2] += rt_z_step
        lt_np[..., 2] += lt_z_step

        rt = wp.from_numpy(rt_np, dtype=wp.float32, device=sim.device)
        self.ants.set_root_transforms(rt, self.all_indices)

        self.expected_root_transforms = rt_np
        self.expected_link_transforms = lt_np
                        
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            rt = self.ants.get_root_transforms()
            lt = self.ants.get_link_transforms()
            rt_np = rt.numpy().reshape((self.ants.count, 7))
            lt_np = lt.numpy().reshape((self.ants.count, self.ants.max_links, 7))
            # print(rt_np[...,2])
            # print(self.expected_root_transforms[...,2])
            # print(lt_np[...,2])
            # print(self.expected_link_transforms[...,2])
            self.test_case.assertTrue(np.allclose(rt_np, self.expected_root_transforms, rtol=1e-03, atol=1e-04), "expected root transforms")
            self.test_case.assertTrue(np.allclose(lt_np, self.expected_link_transforms, rtol=1e-03, atol=1e-04), "expected link transforms")
            self.finish()


class TestArticulationRootVelocities(TestArticulationRoots):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        # desired root velocities
        root_vels_np = np.zeros((self.ants.count, 6))
        root_linear_z = np.linspace(0.1, 1.0, self.ants.count, dtype=np.float32)
        root_vels_np[..., 2] = root_linear_z
        #print(root_linear_z)

        # expected link velocities
        link_vels_np = np.zeros((self.ants.count, self.ants.max_links, 6))
        link_linear_z = np.repeat(root_linear_z, self.ants.max_links).reshape((self.ants.count, self.ants.max_links))
        link_vels_np[..., 2] = link_linear_z
        #print(link_linear_z)

        root_vels = wp.from_numpy(root_vels_np, dtype=wp.float32, device=sim.device)
        self.ants.set_root_velocities(root_vels, self.all_indices)

        self.expected_root_vels = root_vels_np
        self.expected_link_vels = link_vels_np

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 10:
            root_vels = self.ants.get_root_velocities()
            link_vels = self.ants.get_link_velocities()
            root_vels_np = root_vels.numpy().reshape((self.ants.count, 6))
            link_vels_np = link_vels.numpy().reshape((self.ants.count, self.ants.max_links, 6))
            #print(root_vels_np)
            #print(link_vels_np)
            #print(self.expected_root_velocities)
            self.test_case.assertTrue(np.allclose(root_vels_np, self.expected_root_vels, rtol=1e-03, atol=1e-03), "expected root velocities")
            self.test_case.assertTrue(np.allclose(link_vels_np, self.expected_link_vels, rtol=1e-03, atol=1e-03), "expected link velocities")
            self.finish()


class TestLinearDofs(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(42)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartRailNoPole.usda")
        actor_path = self.env_template_path.AppendChild("railcart")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        railcarts = sim.create_articulation_view("/envs/*/railcart")
        self.check_articulation_view(railcarts, self.num_envs, 2, 1, True)

        self.railcarts = railcarts
        self.all_indices = wp_utils.arange(railcarts.count, device=sim.device)

        # call concrete subclass implementation
        self.on_start_impl(sim)

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestLinearDofPositions(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        dof_pos = wp_utils.linspace(self.railcarts.count, -2.5, 2.5, include_end=True, device=sim.device)
        #print(dof_pos.numpy().squeeze())
        self.railcarts.set_dof_positions(dof_pos, self.all_indices)
        self.desired_positions = dof_pos

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            dof_pos = self.railcarts.get_dof_positions()
            p = dof_pos.numpy().squeeze()
            d = self.desired_positions.numpy().squeeze()
            #print(np.dstack((d, p)))
            self.test_case.assertTrue(np.allclose(p, d, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestLinearDofVelocities(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        self.vmin = -2.0
        self.vmax = 2.0
        dof_vel = wp_utils.linspace(self.railcarts.count, self.vmin, self.vmax, include_end=True, device=sim.device)
        #print(dof_pos.numpy().squeeze())
        self.railcarts.set_dof_velocities(dof_vel, self.all_indices)
        self.desired_velocities = dof_vel.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        teststep = 10
        if stepno == teststep:
            dof_pos = self.railcarts.get_dof_positions()
            dof_vel = self.railcarts.get_dof_velocities()
            p = dof_pos.numpy().squeeze()
            v = dof_vel.numpy().squeeze()
            # test velocities
            self.test_case.assertTrue(abs(v[0] - self.vmin) < 0.01, "expected min vel")
            self.test_case.assertTrue(abs(v[-1] - self.vmax) < 0.01, "expected max vel")
            self.test_case.assertTrue(np.allclose(v, self.desired_velocities), "expected velocities")
            # test positions
            dt = 1.0 / self.sim_params.time_steps_per_second
            expected_pos = teststep * dt * self.desired_velocities
            #print(np.dstack((p, expected_pos)))
            self.test_case.assertTrue(np.allclose(p, expected_pos, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestLinearDofForces(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        # set initial positions
        self.pmin = -2.5
        self.pmax = 2.5
        dof_pos = wp_utils.linspace(self.railcarts.count, self.pmin, self.pmax, include_end=True, device=sim.device)
        self.railcarts.set_dof_positions(dof_pos, self.all_indices)
        # allocate force buffer
        self.forces = wp.zeros((self.num_envs, self.railcarts.max_dofs), dtype=float, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        dof_pos = self.railcarts.get_dof_positions()
        dof_vel = self.railcarts.get_dof_velocities()

        if stepno == 1:
            # check if the initial DOF positions are in the right range
            p = dof_pos.numpy()
            #print(stepno, p)
            self.test_case.assertTrue(abs(p[0, 0] - self.pmin) < 0.01, "expected min pos")
            self.test_case.assertTrue(abs(p[-1, 0] - self.pmax) < 0.01, "expected max pos")
        if stepno >= 100:
            # check if the poles are upright and stable
            p = dof_pos.numpy()
            v = dof_pos.numpy()
            #print(stepno, p)
            self.test_case.assertTrue((np.abs(p) < 0.01).all(), "expected positions")
            self.test_case.assertTrue((np.abs(v) < 0.01).all(), "expected velocities")
            self.finish()

        # compute and apply forces
        stiffness = 1000.0
        damping = 120.0
        wp_utils.compute_dof_forces(dof_pos, dof_vel, self.forces, stiffness, damping, device=sim.device)
        self.railcarts.set_dof_actuation_forces(self.forces, self.all_indices)

        if stepno == 99: 
            applied_efforts = self.railcarts.get_dof_actuation_forces()
            # print(np.dstack((applied_efforts.numpy(), self.forces.numpy())))
            self.test_case.assertTrue(np.allclose(applied_efforts.numpy().flatten(), self.forces.numpy(), rtol=1e-02, atol=1e-02), "expected actuation forces") 

class TestLinearDofPositionTargets(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

        # configure drives
        for i in range(self.num_envs):
            stiffness = 2000.0
            damping = 250.0
            max_force = 4000.0
            joint_prim = self.stage.GetPrimAtPath("/envs/env%d/railcart/cartJoint" % i)
            set_drive(joint_prim, "linear", "position", 0.0, stiffness, damping, max_force)

    def on_start_impl(self, sim):
        pmin = -2.5
        pmax = 2.5
        targets = wp_utils.linspace(self.railcarts.count, pmin, pmax, include_end=True, device=sim.device)
        self.railcarts.set_dof_position_targets(targets, self.all_indices)
        self.targets = targets.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 60:
            dof_pos = self.railcarts.get_dof_positions()
            p = dof_pos.numpy().squeeze()
            #print(np.dstack((p, self.targets)))
            self.test_case.assertTrue(np.allclose(p, self.targets, rtol=1e-02, atol=1e-02), "expected positions")
            self.finish()


class TestLinearDofVelocityTargets(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

        # configure drives
        for i in range(self.num_envs):
            stiffness = 0.0
            damping = 500.0
            max_force = 4000.0
            joint_prim = self.stage.GetPrimAtPath("/envs/env%d/railcart/cartJoint" % i)
            set_drive(joint_prim, "linear", "velocity", 0.0, stiffness, damping, max_force)

    def on_start_impl(self, sim):
        vmin = -2.0
        vmax = 2.0
        targets = wp_utils.linspace(self.railcarts.count, vmin, vmax, include_end=True, device=sim.device)
        self.railcarts.set_dof_velocity_targets(targets, self.all_indices)
        self.targets = targets.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            dof_vel = self.railcarts.get_dof_velocities()
            v = dof_vel.numpy().squeeze()
            #print(np.dstack((v, self.targets)))
            self.test_case.assertTrue(np.allclose(v, self.targets, rtol=1e-02, atol=1e-02), "expected velocities")
            self.finish()


class TestAngularDofs(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2.1
        grid_params.col_spacing = 2.1
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPoleNoRail.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 1.1))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(cartpoles, self.num_envs, 2, 1, True)

        self.cartpoles = cartpoles
        self.all_indices = wp_utils.arange(cartpoles.count, device=sim.device)

        # call concrete subclass implementation
        self.on_start_impl(sim)

        
class TestAngularDofPositions(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        pmin = -0.5 * math.pi
        pmax = 0.5 * math.pi
        dof_pos = wp_utils.linspace(self.cartpoles.count, pmin, pmax, include_end=True, device=sim.device)
        self.cartpoles.set_dof_positions(dof_pos, self.all_indices)
        self.desired_positions = dof_pos

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            dof_pos = self.cartpoles.get_dof_positions()
            p = dof_pos.numpy().squeeze()
            d = self.desired_positions.numpy().squeeze()
            #print(np.dstack((d, p)))
            self.test_case.assertTrue(np.allclose(p, d, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestAngularDofVelocities(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        self.vmin = -math.pi
        self.vmax = math.pi
        dof_vel = wp_utils.linspace(self.cartpoles.count, self.vmin, self.vmax, include_end=True, device=sim.device)
        #print(dof_pos.numpy().squeeze())
        self.cartpoles.set_dof_velocities(dof_vel, self.all_indices)
        self.desired_velocities = dof_vel.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        teststep = 10
        if stepno == teststep:
            dof_pos = self.cartpoles.get_dof_positions()
            dof_vel = self.cartpoles.get_dof_velocities()
            p = dof_pos.numpy().squeeze()
            v = dof_vel.numpy().squeeze()
            # test velocities
            self.test_case.assertTrue(abs(v[0] - self.vmin) < 0.01, "expected min vel")
            self.test_case.assertTrue(abs(v[-1] - self.vmax) < 0.01, "expected max vel")
            self.test_case.assertTrue(np.allclose(v, self.desired_velocities), "expected velocities")
            # test positions
            dt = 1.0 / self.sim_params.time_steps_per_second
            expected_pos = teststep * dt * self.desired_velocities
            #print(np.dstack((p, expected_pos)))
            self.test_case.assertTrue(np.allclose(p, expected_pos, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestAngularDofForces(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        # set initial positions
        self.pmin = -0.95 * math.pi
        self.pmax = 0.95 * math.pi
        dof_pos = wp_utils.linspace(self.cartpoles.count, self.pmin, self.pmax, include_end=True, device=sim.device)
        #print(dof_pos.numpy().squeeze())
        self.cartpoles.set_dof_positions(dof_pos, self.all_indices)

        # allocate force buffer
        self.forces = wp.zeros((self.num_envs, self.cartpoles.max_dofs), dtype=float, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        dof_pos = self.cartpoles.get_dof_positions()
        dof_vel = self.cartpoles.get_dof_velocities()

        if stepno == 1:
            # check if the initial DOF positions are in the right range
            p = dof_pos.numpy()
            #print(stepno, p)
            self.test_case.assertTrue(abs(p[0] - self.pmin) < 0.01, "expected min pos")
            self.test_case.assertTrue(abs(p[-1] - self.pmax) < 0.01, "expected max pos")
        if stepno >= 100:
            # check if the poles are upright and stable
            p = dof_pos.numpy()
            v = dof_pos.numpy()
            #print(stepno, p)
            self.test_case.assertTrue((np.abs(p) < 0.01).all(), "expected positions")
            self.test_case.assertTrue((np.abs(v) < 0.01).all(), "expected velocities")
            applied_efforts = self.cartpoles.get_dof_actuation_forces()
            self.test_case.assertTrue(np.allclose(applied_efforts.numpy(), self.forces.numpy(), rtol=1e-03, atol=1e-03), "expected actuation forces")
            self.finish()

        # compute and apply forces
        stiffness = 20.0
        damping = 4.0
        wp_utils.compute_dof_forces(dof_pos, dof_vel, self.forces, stiffness, damping, device=sim.device)
        self.cartpoles.set_dof_actuation_forces(self.forces, self.all_indices)


class TestAngularDofPositionTargets(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

        # configure drives
        for i in range(self.num_envs):
            stiffness = 1000.0
            damping = 80.0
            max_force = 1000.0
            joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/poleJoint" % i)
            set_drive(joint_prim, "angular", "position", 0.0, stiffness, damping, max_force)

    def on_start_impl(self, sim):
        pmin = -0.5 * math.pi
        pmax = 0.5 * math.pi
        targets = wp_utils.linspace(self.cartpoles.count, pmin, pmax, include_end=True, device=sim.device)
        self.cartpoles.set_dof_position_targets(targets, self.all_indices)
        self.targets = targets.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 60:
            dof_pos = self.cartpoles.get_dof_positions()
            p = dof_pos.numpy().squeeze()
            #print(np.dstack((p, self.targets)))
            self.test_case.assertTrue(np.allclose(p, self.targets, rtol=1e-02, atol=1e-02), "expected positions")
            self.finish()


class TestAngularDofVelocityTargets(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

        # configure drives
        for i in range(self.num_envs):
            stiffness = 0.0
            damping = 500.0
            max_force = 1000.0
            joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/poleJoint" % i)
            set_drive(joint_prim, "angular", "velocity", 0.0, stiffness, damping, max_force)

    def on_start_impl(self, sim):
        vmin = -math.pi
        vmax = math.pi
        targets = wp_utils.linspace(self.cartpoles.count, vmin, vmax, include_end=True, device=sim.device)
        self.cartpoles.set_dof_velocity_targets(targets, self.all_indices)
        self.targets = targets.numpy().squeeze()

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            dof_vel = self.cartpoles.get_dof_velocities()
            v = dof_vel.numpy().squeeze()
            #print(np.dstack((v, self.targets)))
            self.test_case.assertTrue(np.allclose(v, self.targets, rtol=1e-02, atol=1e-02), "expected velocities")
            self.finish()

class TestJacobians(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, self.num_envs, 9, 8, True)

        self.ants = ants
        self.all_indices = wp_utils.arange(ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            jacobian_shape = self.ants.jacobian_shape
            jacobians = self.ants.get_jacobians()
            self.test_case.assertTrue(jacobian_shape == (6 + (self.ants.max_links - 1) * 6, 6 + self.ants.max_dofs))
            self.test_case.assertTrue(jacobians.shape == (self.ants.count, self.ants.max_links, jacobian_shape[0] // self.ants.max_links, jacobian_shape[1]))
            self.finish()

class TestMassMatrices(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, self.num_envs, 9, 8, True)

        self.ants = ants
        self.all_indices = wp_utils.arange(ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            # using depreacted API
            mm_shape = self.ants.mass_matrix_shape
            mass_matrices = self.ants.get_mass_matrices()
            self.test_case.assertTrue(mm_shape == (self.ants.max_dofs, self.ants.max_dofs))
            self.test_case.assertTrue(mass_matrices.shape == (self.ants.count, mm_shape[0], mm_shape[1]))

            # using new API
            mm_shape = self.ants.generalized_mass_matrix_shape
            mass_matrices = self.ants.get_generalized_mass_matrices()
            self.test_case.assertTrue(mm_shape == (self.ants.max_dofs + 6, self.ants.max_dofs + 6))
            self.test_case.assertTrue(mass_matrices.shape == (self.ants.count, self.ants.max_dofs + 6, self.ants.max_dofs + 6))

            self.finish()

class TestCoriolisCentrifugal(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, self.num_envs, 9, 8, True)

        self.ants = ants
        self.all_indices = wp_utils.arange(ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        force_amt = 1000.0
        rc_indices = wp_utils.arange(self.ants.count, device=sim.device)
        rc_forces = wp_utils.fill_float32(self.ants.count*self.ants.max_dofs, force_amt, device=sim.device)
        self.ants.set_dof_actuation_forces(rc_forces, rc_indices)

        if stepno == 20:
            # using deprecated API
            cori_centri_forces = self.ants.get_coriolis_and_centrifugal_forces()
            self.test_case.assertTrue(cori_centri_forces.shape == (self.ants.count, self.ants.max_dofs))

            # using new API
            cori_centri_forces = self.ants.get_coriolis_and_centrifugal_compensation_forces()
            self.test_case.assertTrue(cori_centri_forces.shape == (self.ants.count, self.ants.max_dofs + 6))

            applied_efforts = self.ants.get_dof_actuation_forces()
            # print(np.dstack((applied_efforts.numpy(), rc_forces.numpy())))
            self.test_case.assertTrue(np.allclose(applied_efforts.numpy().flatten(), rc_forces.numpy(), rtol=1e-03, atol=1e-03), "expected actuation forces")
            self.finish()

class TestGravityCompensation(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, self.num_envs, 9, 8, True)

        self.ants = ants
        self.all_indices = wp_utils.arange(ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        force_amt = 1000.0
        rc_indices = wp_utils.arange(self.ants.count, device=sim.device)
        rc_forces = wp_utils.fill_float32(self.ants.count*self.ants.max_dofs, force_amt, device=sim.device)
        self.ants.set_dof_actuation_forces(rc_forces, rc_indices)
        if stepno == 20:
            gravity_compensation = self.ants.get_gravity_compensation_forces()
            self.test_case.assertTrue(gravity_compensation.shape == (self.ants.count, self.ants.max_dofs + 6))
            applied_efforts = self.ants.get_dof_actuation_forces()
            # print(np.dstack((applied_efforts.numpy(), rc_forces.numpy())))
            self.test_case.assertTrue(np.allclose(applied_efforts.numpy().flatten(), rc_forces.numpy(), rtol=1e-03, atol=1e-03), "expected actuation forces")
            self.finish()

class TestLinkAccelerations(GridTestBase):
    def __init__(self, test_case, device_params, force_on_fixed_link):
        self.gravityMag = 10.0
        self.num_envs= 16
        grid_params = GridParams(self.num_envs)
        # grid_params.num_rows = 6
        # grid_params.row_spacing = 3
        # grid_params.col_spacing = 3
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = self.gravityMag
        sim_params.add_default_ground = False
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        physicsUtils.add_quad_plane(self.stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(-1), Gf.Vec3f(0.5))
        self.set_camera_properties(Gf.Vec3f(0, -15, 5))
        # add pendulum
        pendulum_path = self.env_template_path.AppendChild("pendulum")
        transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.0
        self.link_half_length = 0.5

        self.fixed_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0.1, 5), 25).GetQuat())
        self.link_paths = self.create_pendulum_articulation(pendulum_path,
                                                            transform,
                                                            link_half_length=self.link_half_length,
                                                            link_mass=self.link_mass,
                                                            add_fixed_joint_link=False,
                                                            revolute_joint_axis="Y",
                                                            revolute_joint_frame_quat= Gf.Quatf(1.0),
                                                            fixed_joint_frame_quat=self.fixed_joint_frame_quat)
    
    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 2, 1, True)
        self.vmin = -math.pi
        self.vmax = math.pi
        dof_vel = wp_utils.linspace(self.pendulums.count, self.vmin, self.vmax, include_end=True, device=sim.device)
        # dof_vel = wp_utils.fill_float32(self.pendulums.count, 2.5,  device=sim.device)
        dof_pos = wp_utils.fill_float32(self.pendulums.count, 0.0, device=sim.device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.pendulums.set_dof_velocities(dof_vel, self.all_indices)
        self.pendulums.set_dof_positions(dof_pos, self.all_indices)
        self.pre_dof_vel = self.pendulums.get_dof_velocities().numpy() 

    def on_physics_step(self, sim, stepno, dt):
        if stepno > 1:
            dofVel = self.pendulums.get_dof_velocities().numpy().reshape((self.pendulums.count, ))
            dofPos = self.pendulums.get_dof_positions().numpy().reshape((self.pendulums.count, )).copy()
            numerical_dofAcc = (self.pendulums.get_dof_velocities().numpy() - self.pre_dof_vel) / dt
            Io = 1.0 / 3.0 * self.link_mass * self.link_half_length * self.link_half_length * 4
            gravity_torque = self.link_mass * self.gravityMag * self.link_half_length * np.cos(dofPos)
            dofAcc = gravity_torque / Io 
            self.test_case.assertTrue(np.allclose(numerical_dofAcc.flatten(), dofAcc.flatten(), atol= 0.01 * np.abs(dofAcc)), "expected numerical dof acceleration")
            acc = self.pendulums.get_link_accelerations().numpy().reshape((self.num_envs, 2, 6))

            self.reference_acc = np.zeros((self.pendulums.count, 2, 6))
            self.reference_acc[:, 1, 0] = -self.link_half_length * dofAcc * np.sin(dofPos) - self.link_half_length * dofVel * dofVel * np.cos(dofPos) 
            self.reference_acc[:, 1, 2] = -self.link_half_length * dofAcc * np.cos(dofPos) + self.link_half_length * dofVel * dofVel * np.sin(dofPos)
            self.reference_acc[:, 1, 4] = dofAcc

            # print("accelerations:\n",np.dstack((acc[1, 1, :], self.reference_acc[1, 1, :])))
            self.test_case.assertTrue(np.allclose(acc[:, :, :3], self.reference_acc[:, :, :3], atol = 0.01 * np.linalg.norm(acc[:, :, :3])), "expected link linear acceleration")
            self.test_case.assertTrue(np.allclose(acc[:, :, 3:], self.reference_acc[:, :, 3:], atol = 0.01 * np.linalg.norm(acc[:, :, 3:])), "expected link angular acceleration")

        if stepno == 50:
            self.finish()

        self.pre_dof_vel = np.copy(self.pendulums.get_dof_velocities().numpy())

# base class for configuring the rotational part of the articulation
class TestJointForceDofProjection(GridTestBase):
    def __init__(self, test_case, device_params, free_rotation_axis, dof_torque_multiplier, free_torsional_axis=False, free_all_axes=False):
        self.gravityMag = 10.0
        self.num_envs= 16
        self.free_rotation_axis = free_rotation_axis
        # to test with a spherical joint with 2 DOF
        self.free_torsional_axis = free_torsional_axis
        # to simulate a spherical joint instead of revolute joint
        self.free_all_axes = free_all_axes 
        self.dof_torque_multiplier = dof_torque_multiplier
        grid_params = GridParams(self.num_envs)
        grid_params.env_spacing = 3.0
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = self.gravityMag
        sim_params.add_default_ground = False
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        physicsUtils.add_quad_plane(self.stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(-1), Gf.Vec3f(0.5))
        # self.physx_scene.CreateSolverTypeAttr("PGS")
        self.rel_tol=0.04
        self.set_camera_properties(Gf.Vec3f(0, -5, 1))
        # add pendulum
        self.pendulum_path = self.env_template_path.AppendChild("pendulum")
        self.transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.0
        self.link_half_length = 0.5
        self.link_height = 0.05
        self.rel_tol=0.15
        self.D6_joint_frame_quat = Gf.Quatf(1.0)  # rotation from world frame to joint frame
        self.usd_d6_free_rot_dofs = {'rotX': False, 'rotY': False , 'rotZ': False}
        self.usd_d6_free_rot_dofs[self.free_rotation_axis] = True

        # if free_torsional_axis then rotation around the link axis is also allowed
        # if free_all_axes then rotation around all the axes are allowed
        # rotate the joint frames so the rotation axis is aligned with world Y axis
        if self.free_rotation_axis == "rotX":
            # rotate local frame 90 around z axis so the the rotation x axis is aligned with the  world Y axis  
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
            if self.free_torsional_axis:
                # usd_d6 y axis is aligned with the link axis
                self.usd_d6_free_rot_dofs['rotY'] = True
            if self.free_all_axes:
                # free the other axis as well
                self.usd_d6_free_rot_dofs['rotZ'] = True

        elif self.free_rotation_axis == "rotZ":
            # rotate local -90 around x axis so the the rotation z axis is aligned with the world Y axis  
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())
            if self.free_torsional_axis:
                # usd_d6 x axis is aligned with the link axis
                self.usd_d6_free_rot_dofs['rotX'] = True
            if self.free_all_axes:
                # free the other axis as well
                self.usd_d6_free_rot_dofs['rotY'] = True
        else:
            if self.free_torsional_axis:
                # usd_d6 x axis is aligned with the link axis
                self.usd_d6_free_rot_dofs['rotX'] = True
            if self.free_all_axes:
                # free the other axis as well
                self.usd_d6_free_rot_dofs['rotZ'] = True
    
        if self.free_torsional_axis:
            self.num_arti_dofs=2
        else:
            self.num_arti_dofs=1
        
        if self.free_all_axes:
                self.num_arti_dofs=3

        self.materialPath = "/material"
        UsdShade.Material.Define(self.stage, self.materialPath)
        material = UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self.materialPath))
        material.CreateDynamicFrictionAttr(0.0)
        material.CreateStaticFrictionAttr(0.0)
        material.CreateRestitutionAttr(0.0)

        self.configure_test()

    def set_up_actuated_dof_force(self, num_dofs, motor_torque):
        forces = np.zeros((self.pendulums.count, num_dofs))
        if not self.free_torsional_axis and not self.free_all_axes:
            # single dof case
            forces[:,0] = motor_torque
        else:
            # multiple dof cases
            if self.free_rotation_axis == "rotX": 
                # y axis is aligned with the link axis (torsion axis)
                # the actuated dof is the first element regardless of if there are 2 or 3 dofs in the joint 
                forces[:,0] = motor_torque
            elif self.free_rotation_axis == "rotY":
                # x axis is aligned with the link axis (torsion axis)
                # if there are 2 dofs in the joint, x, y are the free axes, and the second dof is actuated (y)
                # if there are 3 dofs in the joint, x, y, z are the free axes, and the second dof is still the actuated one (y)
                forces[:,1] = motor_torque
            elif self.free_rotation_axis == "rotZ":
                # x axis is aligned with the link axis (torsion axis)
                if not self.free_all_axes: 
                    # if there are only 2 dofs in the joint, x and z are the free axes so the second element is the the actuated one (z)
                    forces[:,1] = motor_torque
                else: 
                    # if there are 3 dofs in the joint, x, y and z are the free axes so the third element is the the actuated one (z)
                    forces[:,2] = motor_torque
        return forces

    def fix_collider_and_link_properties(self, ball, pendulum, pendulum_link):
        # TODO: need to remove this once the issue with the TGS solver is resolved
        if self.physx_scene.GetSolverTypeAttr().Get() == "TGS":
            rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(ball)
            rb_api.CreateSolverVelocityIterationCountAttr(0)
            rb_api.CreateSolverPositionIterationCountAttr(8)
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(pendulum)
            articulation_api.CreateSolverVelocityIterationCountAttr(0)
            articulation_api.CreateSolverPositionIterationCountAttr(8)
        
        UsdPhysics.RigidBodyAPI.Apply(ball).CreateKinematicEnabledAttr(True)
        
        physicsUtils.add_physics_material_to_prim(self.stage, ball, self.materialPath)
        physicsUtils.add_physics_material_to_prim(self.stage, pendulum_link, self.materialPath)

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        dof_pos = wp_utils.fill_float32(self.pendulums.count* self.pendulums.max_dofs, 0.0, device=sim.device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.pendulums.set_dof_positions(dof_pos, self.all_indices)
        forces = self.set_up_actuated_dof_force(self.pendulums.max_dofs, self.motor_torque)
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.pendulums.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)
        self.on_start_impl()

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            joint_forces = self.pendulums.get_link_incoming_joint_force().numpy()
            # print("joint forces=\n", joint_forces[0])
            # print("reference forces=\n", self.reference_forces)
            self.test_case.assertTrue((np.abs(joint_forces - self.reference_forces_rep) <= self.rel_tol * self.reference_forces_mag).all(), "correct joint force")

            dof_forces=self.pendulums.get_dof_projected_joint_forces()
            # print("dof_forces=\n", dof_forces.numpy().flatten())
            # print("applied_dof_forces=\n", self.applied_dof_forces)
            self.test_case.assertTrue(np.allclose(dof_forces.numpy().flatten(), self.applied_dof_forces.numpy().flatten(), atol=0.1), "motion projected forces similar to actuation forces")

            self.test_case.assertTrue((np.abs(joint_forces[:,-1,:] - self.reference_forces_rep[:,-1,:]) <= self.rel_tol * self.reference_forces_mag[:,-1,:]).all(), 
                                      "TF sensor forces are similar to constraint forces")    
            self.finish()

# configures the articulation with a single link
class TestJointForceDofProjectionSingleLink(TestJointForceDofProjection):
    def configure_test(self):
        self.link_paths = self.create_custom_pendulum_articulation(self.pendulum_path,
                                                                    self.transform,
                                                                    link_half_length=self.link_half_length,
                                                                    link_height = self.link_height,
                                                                    link_mass=self.link_mass,
                                                                    add_fixed_joint_link = False,
                                                                    joint_type= "D6",
                                                                    rotational_joint_frame_quat= self.D6_joint_frame_quat,
                                                                    usd_d6_free_rot_dof = self.usd_d6_free_rot_dofs,
                                                                    fixed_joint_frame_quat=None)

        actor_path = self.env_template_path.AppendChild("ball")
        ball_size = 0.1
        distance  = self.link_half_length * 2
        transform = Transform((distance , 0.0, -ball_size - self.link_height))
        ball = self.create_rigid_ball(actor_path, transform, ball_size)
        self.fix_collider_and_link_properties(ball, self.stage.GetPrimAtPath(self.pendulum_path) ,self.stage.GetPrimAtPath(self.link_paths["child"]))

        mxg = self.link_mass * self.gravityMag
        # clock-wise is positive
        self.motor_torque = self.dof_torque_multiplier * self.link_mass * self.gravityMag * self.link_half_length
        if self.motor_torque > - 1 * self.link_mass * self.gravityMag * self.link_half_length :
            contact_force = mxg / 2.0 + self.motor_torque/ (2 * self.link_half_length)
        else:
            contact_force = 0

        # T = - 1 x m x g x L will balance the gravity weight and make the contact force go to zero
        # T = 0 results in the ball contact and D6 joint supporting half of the weight each due to symmetry
        # T = + 1 x m x g x L  leads to contact supporting all the weight 
        # print("link weights = %f, ball contact force = %f, motor torque = %f"%(mxg, contact_force, self.motor_torque))

        # compute expeced spatial joint forces in world frame:
        D6_joint_force_W = Gf.Vec3f(0)
        D6_joint_torque_W = Gf.Vec3f(0)

        # forces in childs attachment 
        D6_joint_force_W[2] =  mxg - contact_force
        D6_joint_torque_W[1] = self.motor_torque

        #link rotations
        child_rotation = self.stage.GetPrimAtPath(self.link_paths["child"]).GetAttribute('xformOp:orient').Get()
        # transform into joint frames:
        D6_joint_force_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_force_W)
        D6_joint_torque_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_torque_W)

        self.reference_forces = np.zeros((2, 6), dtype=np.float32)
        self.reference_forces[1, 0:3] = D6_joint_force_J
        self.reference_forces[1, 3:6] = D6_joint_torque_J
        self.reference_forces_rep = np.broadcast_to(self.reference_forces, (self.num_envs, 2, 6))
        self.reference_forces_mag =  np.zeros((self.num_envs, 2, 6))
        for i in range(6):
            self.reference_forces_mag[:,:,i] = np.linalg.norm(self.reference_forces_rep, axis=2)

    def on_start_impl(self):
        self.check_articulation_view(self.pendulums, self.num_envs, 2, self.num_arti_dofs, True)

# configures the articulation with two links and models a force-torque sensor
class TestJointForceDofProjectionTwoLinks(TestJointForceDofProjection):
    def configure_test(self):
        # force-torque sensor is modeled as a fixed joint
        FT_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90).GetQuat())
        self.link_paths = self.create_custom_pendulum_articulation(self.pendulum_path,
                                                                    self.transform,
                                                                    link_half_length=self.link_half_length,
                                                                    link_height = self.link_height,
                                                                    link_mass=self.link_mass,
                                                                    add_fixed_joint_link = True,
                                                                    joint_type= "D6",
                                                                    rotational_joint_frame_quat= self.D6_joint_frame_quat,
                                                                    usd_d6_free_rot_dof = self.usd_d6_free_rot_dofs,
                                                                    fixed_joint_frame_quat=FT_frame_quat)

        actor_path = self.env_template_path.AppendChild("ball")
        ball_size = 0.1
        distance  = self.link_half_length * 4 
        transform = Transform((distance , 0.0, -ball_size - self.link_height))
        ball = self.create_rigid_ball(actor_path, transform, ball_size)
        
        self.fix_collider_and_link_properties(ball, self.stage.GetPrimAtPath(self.pendulum_path),self.stage.GetPrimAtPath(self.link_paths["fixed"]))

        mxg = self.link_mass * self.gravityMag
        # clock-wise is positive
        self.motor_torque = self.dof_torque_multiplier * self.link_mass * self.gravityMag * self.link_half_length
        if self.motor_torque > - 4 * self.link_mass * self.gravityMag * self.link_half_length :
            contact_force = mxg + self.motor_torque/ (4 * self.link_half_length)
        else:
            contact_force = 0

        # T = - 4 x m x g x L will balance the gravity weight and make the contact force go to zero
        # T = 0 results in the ball contact and D6 joint supporting half of the weight each due to symmetry
        # T = + 4 x m x g x L  leads to contact supporting all the weight 
        # print("link weights = %f, ball contact force = %f, motor torque = %f"%(2 * mxg, contact_force, self.motor_torque))

        # compute expeced spatial joint forces in world frame:
        D6_joint_force_W = Gf.Vec3f(0)
        D6_joint_torque_W = Gf.Vec3f(0)
        FT_sensor_force_W = Gf.Vec3f(0)
        FT_sensor_torque_W = Gf.Vec3f(0)

        # forces in childs attachment 
        D6_joint_force_W[2] =  2 * mxg - contact_force
        D6_joint_torque_W[1] = self.motor_torque
        FT_sensor_force_W[2] =  mxg - contact_force 
        FT_sensor_torque_W[1] = -mxg * self.link_half_length + 2 * contact_force * self.link_half_length

        #link rotations
        child_rotation = self.stage.GetPrimAtPath(self.link_paths["child"]).GetAttribute('xformOp:orient').Get()
        fixed_rotation = self.stage.GetPrimAtPath(self.link_paths["fixed"]).GetAttribute('xformOp:orient').Get()
        # transform into joint frames:
        D6_joint_force_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_force_W)
        D6_joint_torque_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_torque_W)
        FT_sensor_force_J = Gf.Rotation((FT_frame_quat * fixed_rotation).GetInverse()).TransformDir(FT_sensor_force_W)
        FT_sensor_torque_J = Gf.Rotation((FT_frame_quat * fixed_rotation).GetInverse()).TransformDir(FT_sensor_torque_W)

        self.reference_forces = np.zeros((3, 6), dtype=np.float32)
        self.reference_forces[1, 0:3] = D6_joint_force_J
        self.reference_forces[1, 3:6] = D6_joint_torque_J
        self.reference_forces[2, 0:3] = FT_sensor_force_J
        self.reference_forces[2, 3:6] = FT_sensor_torque_J
        self.reference_forces_rep = np.broadcast_to(self.reference_forces, (self.num_envs, 3, 6))
        self.reference_forces_mag =  np.zeros((self.num_envs, 3, 6))
        for i in range(6):
            self.reference_forces_mag[:,:,i] = np.linalg.norm(self.reference_forces_rep, axis=2)

    def on_start_impl(self):
        self.check_articulation_view(self.pendulums, self.num_envs, 3, self.num_arti_dofs, True)


class TestForceTorqueSensor(GridTestBase):
    def __init__(self, test_case, device_params, rotation_axis):
        self.gravityMag = 10.0
        self.num_envs= 16
        self.rotation_axis = rotation_axis
        grid_params = GridParams(self.num_envs)
        grid_params.env_spacing = 3.0
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = self.gravityMag
        sim_params.add_default_ground = False
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        physicsUtils.add_quad_plane(self.stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(-1), Gf.Vec3f(0.5))
        # self.physx_scene.CreateSolverTypeAttr("PGS")
        self.set_camera_properties(Gf.Vec3f(0, -15, 5))
        # add pendulum
        pendulum_path = self.env_template_path.AppendChild("pendulum")
        transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.0
        self.link_half_length = 0.5

        # force-torque sensor is modeled as a fixed joint
        self.FT_frame_quat = Gf.Quatf(1.0) #Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90).GetQuat())
        self.revolute_joint_frame_quat = Gf.Quatf(1.0)
        if self.rotation_axis == "X":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
        if self.rotation_axis == "Z":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())

        self.link_paths = self.create_pendulum_articulation(pendulum_path,
                                                            transform,
                                                            link_half_length=self.link_half_length,
                                                            link_mass=self.link_mass,
                                                            add_fixed_joint_link = True,
                                                            revolute_joint_axis= self.rotation_axis,
                                                            revolute_joint_frame_quat= self.revolute_joint_frame_quat,
                                                            fixed_joint_frame_quat=self.FT_frame_quat)

        mxg = self.link_mass * self.gravityMag
        self.motor_torque = -4 * self.link_mass * self.gravityMag * self.link_half_length  

        # compute expeced spatial joint forces in world frame:
        revolute_joint_force_W = Gf.Vec3f(0)
        revolute_joint_torque_W = Gf.Vec3f(0)
        FT_sensor_force_W = Gf.Vec3f(0)
        FT_sensor_torque_W = Gf.Vec3f(0)
        # forces in childs attachment 
        revolute_joint_force_W[2] =  mxg - self.motor_torque / (4 * self.link_half_length)
        revolute_joint_torque_W[1] = self.motor_torque
        FT_sensor_force_W[2] =  - self.motor_torque / (4 * self.link_half_length)
        FT_sensor_torque_W[1] = + mxg * self.link_half_length + self.motor_torque / 2.0

        child_rotation = self.stage.GetPrimAtPath(self.link_paths["child"]).GetAttribute('xformOp:orient').Get()
        fixed_rotation = self.stage.GetPrimAtPath(self.link_paths["fixed"]).GetAttribute('xformOp:orient').Get()
        # transform into joint frames:
        revolute_joint_force_J = Gf.Rotation((self.revolute_joint_frame_quat).GetInverse()).TransformDir(revolute_joint_force_W)
        revolute_joint_torque_J = Gf.Rotation((self.revolute_joint_frame_quat).GetInverse()).TransformDir(revolute_joint_torque_W)
        FT_sensor_force_J = Gf.Rotation((self.FT_frame_quat).GetInverse()).TransformDir(FT_sensor_force_W)
        FT_sensor_torque_J = Gf.Rotation((self.FT_frame_quat).GetInverse()).TransformDir(FT_sensor_torque_W)

        self.reference_forces = np.zeros((3, 6), dtype=np.float32)
        self.reference_forces[1, 0:3] = revolute_joint_force_J
        self.reference_forces[1, 3:6] = revolute_joint_torque_J
        self.reference_forces[2, 0:3] = FT_sensor_force_J
        self.reference_forces[2, 3:6] = FT_sensor_torque_J
        self.reference_forces_rep = np.broadcast_to(self.reference_forces, (self.num_envs, 3, 6))
        self.reference_forces_mag =  np.zeros((self.num_envs, 3, 6))
        for i in range(6):
            self.reference_forces_mag[:,:,i] = np.linalg.norm(self.reference_forces_rep, axis=2)

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)
        dof_pos = wp_utils.fill_float32(self.pendulums.count, 0.0, device=sim.device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.pendulums.set_dof_positions(dof_pos, self.all_indices)
        forces = np.ones((self.pendulums.count,self.pendulums.max_dofs)) * self.motor_torque
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.pendulums.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            joint_forces = self.pendulums.get_link_incoming_joint_force().numpy()
            # print("joint forces=\n", joint_forces[0])
            # print("reference forces=\n", self.reference_forces)
            self.test_case.assertTrue((np.abs(joint_forces - self.reference_forces_rep) <= 0.04 * self.reference_forces_mag).all(), "correct joint force")
            
            dof_forces=self.pendulums.get_dof_projected_joint_forces().numpy()
            self.test_case.assertTrue(np.allclose(dof_forces, self.applied_dof_forces.numpy(), rtol=0.04), "motion projected forces similar to actuation forces")
            self.test_case.assertTrue((np.abs(joint_forces[:,-1,:] - self.reference_forces_rep[:,-1,:]) <= 0.04 * self.reference_forces_mag[:,-1,:]).all(), 
                                      "TF sensor forces are similar to constraint forces")    
            self.finish()

class TestJointActuationForces(GridTestBase):
    np.set_printoptions(precision=5)
    def __init__(self, test_case, device_params):
        self.num_envs = 21
        grid_params = GridParams(self.num_envs)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(self.cartpoles, self.num_envs, 3, 2, True)
        dof_forces=self.cartpoles.get_dof_projected_joint_forces().numpy()
        self.test_case.assertTrue((np.abs(dof_forces) < 1e-4).all(), "zero dof forces")

        self.all_indices = wp_utils.arange(self.cartpoles.count, device=sim.device)
        # forces = np.random.rand( self.cartpoles.count,self.cartpoles.max_dofs)
        forces = np.ones((self.cartpoles.count,self.cartpoles.max_dofs)) * 10
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.cartpoles.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            dof_forces=self.cartpoles.get_dof_projected_joint_forces().numpy()
            # print(dof_forces)
            # print(self.applied_dof_forces.numpy())

            # print(np.dstack((dof_forces, self.applied_dof_forces.numpy())))
            self.test_case.assertTrue(np.allclose(dof_forces, self.applied_dof_forces.numpy(),  rtol=0.04), "motion projected forces similar to actuation forces")

            joint_forces = self.cartpoles.get_link_incoming_joint_force().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 6))
            actuation_force = dof_forces.reshape((self.cartpoles.count, self.cartpoles.max_dofs))

            # print("\n=========================\n", self.cartpoles.max_links, self.cartpoles.max_dofs)
            # print("joint_forces= \n", joint_forces[0])
            # print("constraint_forces= \n",constraint_forces[0])
            # print("dof_forces= \n", actuation_force[0])
            # print("active force= \n",active_force[0] )
        if stepno == 20:
            self.finish()

class TestArticulationDofDriveType(GridTestBase):
    np.set_printoptions(precision=5)
    def __init__(self, test_case, device_params):
        self.num_envs = 16
        grid_params = GridParams(self.num_envs)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.expected_drive_types = np.zeros((self.num_envs, 2)) 
        for i in range(self.num_envs):
            stiffness = 2000.0
            damping = 250.0
            max_force = 4000.0
            # i%0 -> no drive [0, 0]
            # i%1 -> cart drive (force) [1, 0]
            # i%2 -> pole drive (acceleration) [0, 2]
            # i%3 -> cart drive, pole drive  (force, acceleration) [1, 2]
            if i%4 == 1 or i%4 == 3:
                joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/cartJoint" % i)
                set_drive(joint_prim, "linear", "position", 0.0, stiffness, damping, max_force)
                angularDriveAPI = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
                angularDriveAPI.CreateTypeAttr("force")
                self.expected_drive_types[i, 0] = 1
            if i%4 == 2 or i%4 == 3:
                joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/poleJoint" % i)
                set_drive(joint_prim, "angular", "position", 0.0, stiffness, damping, max_force)
                angularDriveAPI = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                angularDriveAPI.CreateTypeAttr("acceleration")
                self.expected_drive_types[i, 1] = 2

    def on_start(self, sim):
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(self.cartpoles, self.num_envs, 3, 2, True)
        drive_types=self.cartpoles.get_drive_types().numpy()
        self.test_case.assertTrue(np.allclose(drive_types, self.expected_drive_types), "expected drive types")

    def on_physics_step(self, sim, stepno, dt):
        self.finish()

class TestLinkIncomingJointForce(GridTestBase):

    def __init__(self, test_case, device_params, force_on_fixed_link, revolute_joint_axis):
        self.gravityMag = 10.0
        grid_params = GridParams(42)
        grid_params.num_rows = 6
        grid_params.row_spacing = 3
        grid_params.col_spacing = 3
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, -1.0, 0.0)
        sim_params.gravity_mag = self.gravityMag
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # add pendulum
        pendulum_path = self.env_template_path.AppendChild("pendulum")
        transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.5
        self.link_half_length = 0.45

        self.revolute_joint_axis = revolute_joint_axis
        self.revolute_joint_frame_quat = Gf.Quatf(1.0)  # rotation from world frame to joint frame
        if self.revolute_joint_axis == "X":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90).GetQuat())
        if self.revolute_joint_axis == "Y":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat())

        self.fixed_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0.1, 5), 25).GetQuat())
        self.link_paths = self.create_pendulum_articulation(pendulum_path,
                                                                 transform,
                                                                 link_half_length=self.link_half_length,
                                                                 link_mass=self.link_mass,
                                                                 add_fixed_joint_link=True,
                                                                 revolute_joint_axis=self.revolute_joint_axis,
                                                                 revolute_joint_frame_quat=self.revolute_joint_frame_quat,
                                                                 fixed_joint_frame_quat=self.fixed_joint_frame_quat)

        self.force_on_fixed_link = force_on_fixed_link

        mxg = self.link_mass * self.gravityMag
        # compute world-frame force to hold link in place:
        self.child_force = Gf.Vec3f(0)
        self.fixed_force = Gf.Vec3f(0)
        force_z = 5
        if force_on_fixed_link:
            self.fixed_force[1] = 4.0 / 3.0 * mxg
            self.fixed_force[2] = force_z
        else:
            self.child_force[1] = 4.0 * mxg
            self.child_force[2] = force_z

        # AD this works but we need to figure out ways of doing this the right way.
        physicsUtils.add_force_torque(self.stage, self.link_paths["child"], force=self.child_force, isWorldSpace=True, mode="force")
        physicsUtils.add_force_torque(self.stage, self.link_paths["fixed"], force=self.fixed_force, isWorldSpace=True, mode="force")

        # compute expected spatial joint forces in world frame:
        revolute_joint_force_W = Gf.Vec3f(0)
        revolute_joint_torque_W = Gf.Vec3f(0)
        fixed_joint_force_W = Gf.Vec3f(0)
        fixed_joint_torque_W = Gf.Vec3f(0)
        if self.force_on_fixed_link:
            revolute_joint_force_W[1] = 2.0 / 3.0 * mxg
            revolute_joint_force_W[2] = -force_z
            revolute_joint_torque_W[1] = 3 * force_z * self.link_half_length
            fixed_joint_force_W[1] = -1.0 / 3.0 * mxg
            fixed_joint_force_W[2] = -force_z
            fixed_joint_torque_W[2] = -1.0 / 3.0 * mxg * self.link_half_length
            fixed_joint_torque_W[1] = force_z * self.link_half_length
        else:
            revolute_joint_force_W[1] = -2.0 * mxg
            revolute_joint_force_W[2] = -force_z
            revolute_joint_torque_W[1] = 1 * force_z * self.link_half_length
            fixed_joint_force_W[1] = mxg
            fixed_joint_torque_W[2] = mxg * self.link_half_length

        # transform into joint frames:
        fixed_joint_force_J = Gf.Rotation(self.fixed_joint_frame_quat.GetInverse()).TransformDir(fixed_joint_force_W)
        fixed_joint_torque_J = Gf.Rotation(self.fixed_joint_frame_quat.GetInverse()).TransformDir(fixed_joint_torque_W)
        revolute_joint_force_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(revolute_joint_force_W)
        revolute_joint_torque_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(revolute_joint_torque_W)

        self.reference_forces = np.zeros((3, 6), dtype=np.float32)
        self.reference_forces[1, 0:3] = revolute_joint_force_J
        self.reference_forces[1, 3:6] = revolute_joint_torque_J
        self.reference_forces[2, 0:3] = fixed_joint_force_J
        self.reference_forces[2, 3:6] = fixed_joint_torque_J

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)

        self.childLinks = sim.create_rigid_body_view("/envs/*/pendulum/ChildLink")
        self.fixedJointLinks = sim.create_rigid_body_view("/envs/*/pendulum/FixedJointLink")

        # only use this if we use gpu pipeline, otherwise forces will be added on top of the ForceAPI forces
        if self.device_params.use_gpu_pipeline:
            indices = wp_utils.arange(self.pendulums.count, device=sim.device)

            if self.force_on_fixed_link:
                forces = wp_utils.fill_vec3(self.pendulums.count, value=self.fixed_force, device=sim.device)
                self.fixedJointLinks.apply_forces_and_torques_at_position(forces, None, None, indices, True)
            else:
                forces = wp_utils.fill_vec3(self.pendulums.count, value=self.child_force, device=sim.device)
                self.childLinks.apply_forces_and_torques_at_position(forces, None, None, indices, True)

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            
            incoming_forces = self.pendulums.get_link_incoming_joint_force()
            forces = incoming_forces.numpy()
            
            reference_forces_rep = np.broadcast_to(self.reference_forces, (self.pendulums.count, 3, 6))
            self.test_case.assertTrue(np.allclose(forces, reference_forces_rep, 4.0e-2, 4.0e-2))
            
        self.finish()

class TestJointPerformanceEnvelope(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "franka.usda")
        actor_path = self.env_template_path.AppendChild("franka")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.physx_scene.CreateSolverTypeAttr("PGS")

    def on_start(self, sim):
        self.franka = sim.create_articulation_view("/envs/*/franka")
        self.all_indices = wp_utils.arange(self.franka.count, device=sim.device)
        self.cpu_all_indices = wp_utils.arange(self.franka.count, device="cpu")
        self.check_articulation_view(self.franka, self.num_envs, 11, 9, True)

        # setting drive with performance envelope
        drive_model_properties = np.zeros((self.franka.count, self.franka.max_dofs, 3))
        max_force = np.zeros((self.franka.count, self.franka.max_dofs))
        damping = np.zeros((self.franka.count, self.franka.max_dofs))
        max_force[:, 0] = 1.0
        max_force[::2, 0] = 1.0e-5
        damping[:, 0] = 1.0e9
        drive_model_properties[:, 0, 0] = 2.0 # speed effort gradient
        drive_model_properties[:, 0, 1] = 1.0e9 # max actuator velocity
        drive_model_properties[:, 0, 2] = 2.0 # velocity dependent resistance
        wp_mf = wp.from_numpy(max_force, dtype=wp.float32, device="cpu")
        wp_d = wp.from_numpy(damping, dtype=wp.float32, device="cpu")
        wp_dm = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
        self.franka.set_dof_max_forces(wp_mf, self.cpu_all_indices)
        self.franka.set_dof_dampings(wp_d, self.cpu_all_indices)
        self.franka.set_dof_drive_model_properties(wp_dm, self.cpu_all_indices)

    def on_physics_step(self, sim, stepno, dt):
        # setting drive position target
        if stepno == 1:
            v_target_frankas = self.franka.get_dof_velocity_targets().numpy().reshape((self.franka.count, self.franka.max_dofs))
            v_target_frankas[:, 0] = 0.5
            self.franka.set_dof_velocity_targets(self.to_warp(v_target_frankas), self.all_indices)

        # check that the pendulum moves slower when the maximum force is lower
        if stepno == 2:
            v_frankas = self.franka.get_dof_velocities().numpy().reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(v_frankas[::2, :], v_frankas[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(v_frankas[1::2, :], v_frankas[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((v_frankas[::2, 0] < v_frankas[1, 0]).all())
            # reset franka position and velocity
            p_frankas = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs))
            p_frankas[:, :] = 0.0
            v_frankas[:, :] = 0.0
            self.franka.set_dof_positions(self.to_warp(p_frankas), self.all_indices)
            self.franka.set_dof_velocities(self.to_warp(v_frankas), self.all_indices)

        # change speed effort gradient
        if stepno == 3:
            max_force = self.franka.get_dof_max_forces().numpy()
            drive_model_properties = self.franka.get_dof_drive_model_properties().numpy()
            max_force[:, 0] = 1.0
            drive_model_properties[::2, 0, 0] = 1.0e15 # speed effort gradient
            wp_mf = wp.from_numpy(max_force, dtype=wp.float32, device="cpu")
            wp_dm = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
            self.franka.set_dof_max_forces(wp_mf, self.cpu_all_indices)
            self.franka.set_dof_drive_model_properties(wp_dm, self.cpu_all_indices)
        # check that the franka moves slower for higher speed effort gradient
        if stepno == 4:
            v_frankas = self.franka.get_dof_velocities().numpy().reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(v_frankas[::2, :], v_frankas[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(v_frankas[1::2, :], v_frankas[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((v_frankas[::2, 0] < v_frankas[1, 0]).all())
            # reset franka position and velocity
            p_frankas = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs))
            p_frankas[:, :] = 0.0
            v_frankas[:, :] = 0.0
            self.franka.set_dof_positions(self.to_warp(p_frankas), self.all_indices)
            self.franka.set_dof_velocities(self.to_warp(v_frankas), self.all_indices)

        # change max actuator velocity
        if stepno == 5:
            drive_model_properties = self.franka.get_dof_drive_model_properties().numpy()
            drive_model_properties[:, 0, 0] = 2.0 # speed effort gradient
            drive_model_properties[::2, 0, 1] = 1.0 # max actuator velocity
            wp_dm = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
            self.franka.set_dof_drive_model_properties(wp_dm, self.cpu_all_indices)

        # check that the franka moves slower for lower max actuator velocity
        if stepno == 6:
            v_frankas = self.franka.get_dof_velocities().numpy().reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(v_frankas[::2, :], v_frankas[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(v_frankas[1::2, :], v_frankas[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((v_frankas[::2, 0] < v_frankas[1, 0]).all())
            # reset franka position and velocity
            p_frankas = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs))
            p_frankas[:, :] = 0.0
            v_frankas[:, :] = 0.0
            self.franka.set_dof_positions(self.to_warp(p_frankas), self.all_indices)
            self.franka.set_dof_velocities(self.to_warp(v_frankas), self.all_indices)

        # change velocity dependent resistance
        if stepno == 7:
            drive_model_properties = self.franka.get_dof_drive_model_properties().numpy()
            drive_model_properties[:, 0, 1] = 1.0e9 # max actuator velocity
            drive_model_properties[::2, 0, 2] = 1.0e3 # velocity dependent resistance
            wp_dm = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
            self.franka.set_dof_drive_model_properties(wp_dm, self.cpu_all_indices)

        # check that the franka moves slower for lower max actuator velocity
        if stepno == 8:
            v_frankas = self.franka.get_dof_velocities().numpy().reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(v_frankas[::2, :], v_frankas[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(v_frankas[1::2, :], v_frankas[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((v_frankas[::2, 0] < v_frankas[1, 0]).all())
            # reset franka position and velocity
            p_frankas = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs))
            p_frankas[:, :] = 0.0
            v_frankas[:, :] = 0.0
            self.franka.set_dof_positions(self.to_warp(p_frankas), self.all_indices)
            self.franka.set_dof_velocities(self.to_warp(v_frankas), self.all_indices)

        if stepno == 9:
            self.finish()

class TestJointFriction(GridTestBase):

    def __init__(self, test_case, device_params):
        self.gravityMag = 10.0
        grid_params = GridParams(42)
        grid_params.num_rows = 6
        grid_params.row_spacing = 3
        grid_params.col_spacing = 3
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, -1.0, 0.0)
        sim_params.gravity_mag = self.gravityMag
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # add pendulum
        pendulum_path = self.env_template_path.AppendChild("pendulum")
        transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.5
        self.link_half_length = 0.45

        self.revolute_joint_frame_quat = Gf.Quatf(1.0)  # rotation from world frame to joint frame
        self.fixed_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0.1, 5), 25).GetQuat())
        self.link_paths = self.create_pendulum_articulation(pendulum_path,
                                                                 transform,
                                                                 link_half_length=self.link_half_length,
                                                                 link_mass=self.link_mass,
                                                                 add_fixed_joint_link=True,
                                                                 revolute_joint_axis="Z",
                                                                 revolute_joint_frame_quat=self.revolute_joint_frame_quat,
                                                                 fixed_joint_frame_quat=self.fixed_joint_frame_quat)

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.cpu_all_indices = wp_utils.arange(self.pendulums.count, device="cpu")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)

        # setting joint friction
        friction_properties = np.zeros((self.pendulums.count, self.pendulums.max_dofs, 3))
        friction_properties[:, :, 0] = 100.0
        friction_properties[:, :, 1] = 0.7
        friction_properties[:, :, 2] = 0.6
        wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
        self.pendulums.set_dof_friction_properties(wp_fc, self.cpu_all_indices)

    def on_physics_step(self, sim, stepno, dt):
        # setting pendulum pose
        if stepno == 1:
            p_pendulums = self.pendulums.get_dof_positions().numpy().reshape((self.pendulums.count, self.pendulums.max_dofs))
            p_pendulums[:, :] = 0.5
            self.pendulums.set_dof_positions(self.to_warp(p_pendulums), self.all_indices)

        # check that the pendulum does not move for high static friction effort
        if stepno == 2:
            self.test_case.assertTrue(np.allclose(self.pendulums.get_dof_positions().numpy().reshape((self.pendulums.count, self.pendulums.max_dofs)), 0.5, 1.0e-4, 1.0e-4))

        # change friction properties and test dynamic friction effort
        if stepno == 3:
            friction_properties = self.pendulums.get_dof_friction_properties().numpy()
            friction_properties[:, :, 0] = 3.0
            friction_properties[:, :, 1] = 0.1
            friction_properties[::2, :, 1] = 3.0
            wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
            self.pendulums.set_dof_friction_properties(wp_fc, self.cpu_all_indices)

        # check that the pendulum moves slower for higher dynamic friction effort
        if stepno == 5:
            p_pendulums = self.pendulums.get_dof_positions().numpy().reshape((self.pendulums.count, self.pendulums.max_dofs))
            self.test_case.assertTrue(np.allclose(p_pendulums[::2, :], p_pendulums[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(p_pendulums[1::2, :], p_pendulums[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((p_pendulums[::2, :] > p_pendulums[1, :]).all())
            # reset pendulum position
            p_pendulums[:, :] = 0.5
            self.pendulums.set_dof_positions(self.to_warp(p_pendulums), self.all_indices)

        # change friction properties and test viscous friction coefficient
        if stepno == 6:
            friction_properties = self.pendulums.get_dof_friction_properties().numpy()
            friction_properties[:, :, 0] = 0.5
            friction_properties[:, :, 1] = 0.5
            friction_properties[:, :, 2] = 50.0
            friction_properties[::2, :, 2] = 0.5
            wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
            self.pendulums.set_dof_friction_properties(wp_fc, self.cpu_all_indices)

        # check that the pendulum moves slower for higher viscous friction coefficient
        if stepno == 8:
            p_pendulums = self.pendulums.get_dof_positions().numpy().reshape((self.pendulums.count, self.pendulums.max_dofs))
            self.test_case.assertTrue(np.allclose(p_pendulums[::2, :], p_pendulums[0, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue(np.allclose(p_pendulums[1::2, :], p_pendulums[1, :], 1.0e-4, 1.0e-4))
            self.test_case.assertTrue((p_pendulums[::2, :] < p_pendulums[1, :]).all())

        if stepno == 8:
            self.finish()


# Regression test for incorrect inertia set for rigid body and articulation views
# In the setup, the principal axes are set to effective 90 deg around +z axis of body frame
# which effectively swizzles the diagonal inertia elements
# With the previously missing principal axis transform (setCmassLocalPose) set in the inertia setters,
# this will lead to a get/set/get cycle changing inertia
class TestReproInertiaSetGetConsistency(GridTestBase):
    def __init__(self, test_case, device_params, as_articulation=False):
        # set up stage
        grid_params = GridParams(3, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.as_articulation = as_articulation

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.0)))
        ball_prim = self.stage.GetPrimAtPath(actor_path)
        ball_mass_api = UsdPhysics.MassAPI(ball_prim)
        ball_mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(0.5, 0.5, 0.5, 0.5))
        ball_mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.1, 0.2, 0.3))
        if self.as_articulation:
            UsdPhysics.ArticulationRootAPI.Apply(ball_prim)

    def on_start(self, sim):
        if self.as_articulation:
            balls = sim.create_articulation_view("/envs/*/ball")
            self.check_articulation_view(balls, self.num_envs, 1, 0, True)
        else:
            balls = sim.create_rigid_body_view("/envs/*/ball")
            self.check_rigid_body_view(balls, self.num_envs)
        
        all_indices = wp_utils.arange(balls.count)

        # inertias should be invariant under get -> set -> get
        initial_inertias = balls.get_inertias()
        initial_inertias_np = initial_inertias.numpy().copy()
        balls.set_inertias(initial_inertias, all_indices)
        new_inertias = balls.get_inertias()

        self.test_case.assertTrue(np.allclose(initial_inertias_np, new_inertias.numpy()))  
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestArticulationBodyProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count)

        # masses
        masses = np.ones((humanoids.count, humanoids.max_links)) * 100.0
        wp_masses = wp.from_numpy(masses, dtype=wp.float32, device="cpu")
        humanoids.set_masses(wp_masses, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_masses().numpy(), masses))

        # inv masses
        self.test_case.assertTrue(humanoids.get_inv_masses().numpy().shape == (self.num_envs, humanoids.max_links))

        # COMs
        com = humanoids.get_coms().numpy().reshape(self.num_envs, humanoids.max_links, 7)
        com[:, :, 0] += 0.1
        wp_coms = wp.from_numpy(com, dtype=wp.float32, device="cpu")
        humanoids.set_coms(wp_coms, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_coms().numpy(), com))

        # inertias
        inertias = humanoids.get_inertias().numpy().reshape(self.num_envs, humanoids.max_links, 9)
        inertias[:, :, [0, 4, 8]] += 0.1
        wp_inertias = wp.from_numpy(inertias, dtype=wp.float32, device="cpu")
        humanoids.set_inertias(wp_inertias, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_inertias().numpy(), inertias))

        # inv inertias
        self.test_case.assertTrue(humanoids.get_inv_inertias().numpy().shape == (self.num_envs, humanoids.max_links, 9))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestArticulationShapeProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count)

        # materials
        material_properties = np.zeros((humanoids.count, humanoids.max_shapes, 3))
        material_properties[:, :, 0] = 0.8
        material_properties[:, :, 1] = 0.7
        material_properties[:, :, 2] = 0.6
        wp_properties = wp.from_numpy(material_properties, dtype=wp.float32, device="cpu")
        humanoids.set_material_properties(wp_properties, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_material_properties().numpy(), material_properties))

        # contact offsets
        contact_offsets = np.ones((humanoids.count, humanoids.max_shapes)) * 0.1
        wp_contact_offsets = wp.from_numpy(contact_offsets, dtype=wp.float32, device="cpu")
        humanoids.set_contact_offsets(wp_contact_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_contact_offsets().numpy(), contact_offsets))

        # rest offsets
        rest_offsets = np.ones((humanoids.count, humanoids.max_shapes)) * 0.05
        wp_rest_offsets = wp.from_numpy(rest_offsets, dtype=wp.float32, device="cpu")
        humanoids.set_rest_offsets(wp_rest_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_rest_offsets().numpy(), rest_offsets))

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass

class TestArticulationFixedTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "ShadowHand.usda")
        actor_path = self.env_template_path.AppendChild("shadow_hand")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        hands = sim.create_articulation_view("/envs/*/shadow_hand")

        self.check_articulation_view(hands, self.num_envs, 26, 24, True)
        all_indices = wp_utils.arange(hands.count, device=sim.device)

        self.test_case.assertEqual(hands.max_fixed_tendons, 4)

        num_tendons = hands.count * hands.max_fixed_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        limits_np = np.arange(0, num_tendons * 2).reshape(hands.count, hands.max_fixed_tendons, 2)
        limits_np[:, :, 0] = -2.0 * limits_np[:, :, 0]
        limits_np[:, :, 1] = 2.0 * limits_np[:, :, 1]
        limits = wp.from_numpy(limits_np, dtype=wp.float32, device=sim.device)
        rest_lengths_np = 0.5 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        rest_lengths = wp.from_numpy(rest_lengths_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        hands.set_fixed_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, limits=limits, rest_lengths=rest_lengths, offsets=offsets, indices=all_indices)

        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limits().numpy(), limits.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_rest_lengths().numpy(), rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_offsets().numpy(), offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestArticulationHeterogeneousFixedTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "FixedTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("FixedTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "ShadowHand.usda")
        actor_path = self.env_template_path.AppendChild("shadow_hand")
        transform = Transform((0.0, 0.0, -1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        fixed_tendon_test = sim.create_articulation_view("/envs/*/FixedTendonTest")
        hands = sim.create_articulation_view("/envs/*/shadow_hand")

        self.check_articulation_view(fixed_tendon_test, self.num_envs, 5, 4, True)
        self.check_articulation_view(hands, self.num_envs, 26, 24, True)
        all_indices = wp_utils.arange(hands.count, device=sim.device)
        hands_indices_np = np.array([0, 4, 5])
        hands_indices = wp.from_numpy(hands_indices_np, dtype=wp.int32, device=sim.device)

        self.test_case.assertEqual(fixed_tendon_test.max_fixed_tendons, 1)
        self.test_case.assertEqual(hands.max_fixed_tendons, 4)

        num_tendons = fixed_tendon_test.count * fixed_tendon_test.max_fixed_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        limits_np = np.arange(0, num_tendons * 2).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons, 2)
        limits_np[:, :, 0] = -2.0 * limits_np[:, :, 0]
        limits_np[:, :, 1] = 2.0 * limits_np[:, :, 1]
        limits = wp.from_numpy(limits_np, dtype=wp.float32, device=sim.device)
        rest_lengths_np = 0.5 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        rest_lengths = wp.from_numpy(rest_lengths_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        hands_stiffnesses_np = hands.get_fixed_tendon_stiffnesses().numpy()
        hands_stiffnesses_np[hands_indices_np, :] = 10.0
        hands_stiffnesses = wp.from_numpy(hands_stiffnesses_np, dtype=wp.float32, device=sim.device)
        hands_dampings_np = hands.get_fixed_tendon_dampings().numpy()
        hands_dampings_np[hands_indices_np, :] = 50.0
        hands_dampings = wp.from_numpy(hands_dampings_np, dtype=wp.float32, device=sim.device)
        hands_limit_stiffnesses_np = hands.get_fixed_tendon_limit_stiffnesses().numpy()
        hands_limit_stiffnesses_np[hands_indices_np, :] = 100.0
        hands_limit_stiffnesses = wp.from_numpy(hands_limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        hands_limits_np = hands.get_fixed_tendon_limits().numpy().reshape(hands.count, hands.max_fixed_tendons, 2)
        hands_limits_np[hands_indices_np, :, 0] = -2.0
        hands_limits_np[hands_indices_np, :, 1] = 2.0
        hands_limits = wp.from_numpy(hands_limits_np, dtype=wp.float32, device=sim.device)
        hands_rest_lengths_np = hands.get_fixed_tendon_rest_lengths().numpy()
        hands_rest_lengths_np[hands_indices_np, :] = 0.5
        hands_rest_lengths = wp.from_numpy(hands_rest_lengths_np, dtype=wp.float32, device=sim.device)
        hands_offsets_np = hands.get_fixed_tendon_offsets().numpy()
        hands_offsets_np[hands_indices_np, :] = 0.1
        hands_offsets = wp.from_numpy(hands_offsets_np, dtype=wp.float32, device=sim.device)

        fixed_tendon_test.set_fixed_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, limits=limits, rest_lengths=rest_lengths, offsets=offsets, indices=all_indices)
        hands.set_fixed_tendon_properties(stiffnesses=hands_stiffnesses, dampings=hands_dampings, limit_stiffnesses=hands_limit_stiffnesses, limits=hands_limits, rest_lengths=hands_rest_lengths, offsets=hands_offsets, indices=hands_indices)

        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_limits().numpy(), limits.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_rest_lengths().numpy(), rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_offsets().numpy(), offsets.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_stiffnesses().numpy(), hands_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_dampings().numpy(), hands_dampings.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limit_stiffnesses().numpy(), hands_limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limits().numpy(), hands_limits.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_rest_lengths().numpy(), hands_rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_offsets().numpy(), hands_offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestArticulationSpatialTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "SpatialTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("SpatialTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        spatial_tendon_test = sim.create_articulation_view("/envs/*/SpatialTendonTest")

        self.check_articulation_view(spatial_tendon_test, self.num_envs, 5, 4, True)
        all_indices = wp_utils.arange(spatial_tendon_test.count, device=sim.device)

        self.test_case.assertEqual(spatial_tendon_test.max_spatial_tendons, 1)

        num_tendons = spatial_tendon_test.count * spatial_tendon_test.max_spatial_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)
        spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, offsets=offsets, indices=all_indices)

        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_offsets().numpy(), offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestArticulationHeterogeneousSpatialTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "SpatialTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("SpatialTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "MultipleSpatialTendonsTest.usda")
        actor_path = self.env_template_path.AppendChild("MultipleSpatialTendonsTest")
        transform = Transform((0.0, 0.0, -1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)


    def on_start(self, sim):
        spatial_tendon_test = sim.create_articulation_view("/envs/*/SpatialTendonTest")
        multiple_spatial_tendon_test = sim.create_articulation_view("/envs/*/MultipleSpatialTendonsTest")

        self.check_articulation_view(spatial_tendon_test, self.num_envs, 5, 4, True)
        self.check_articulation_view(multiple_spatial_tendon_test, self.num_envs, 9, 8, True)
        all_indices = wp_utils.arange(spatial_tendon_test.count, device=sim.device)
        multiple_indices_np = np.array([0, 4, 5])
        multiple_indices = wp.from_numpy(multiple_indices_np, dtype=wp.int32, device=sim.device)

        self.test_case.assertEqual(spatial_tendon_test.max_spatial_tendons, 1)
        self.test_case.assertEqual(multiple_spatial_tendon_test.max_spatial_tendons, 2)

        num_tendons = spatial_tendon_test.count * spatial_tendon_test.max_spatial_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        multiple_stiffnesses_np = multiple_spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy()
        multiple_stiffnesses_np[multiple_indices_np, :] = 10.0
        multiple_stiffnesses = wp.from_numpy(multiple_stiffnesses_np, dtype=wp.float32, device=sim.device)
        multiple_dampings_np = multiple_spatial_tendon_test.get_spatial_tendon_dampings().numpy()
        multiple_dampings_np[multiple_indices_np, :] = 100.0
        multiple_dampings = wp.from_numpy(multiple_dampings_np, dtype=wp.float32, device=sim.device)
        multiple_limit_stiffnesses_np = multiple_spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy()
        multiple_limit_stiffnesses_np[multiple_indices_np, :] = 50.0
        multiple_limit_stiffnesses = wp.from_numpy(multiple_limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        multiple_offsets_np = multiple_spatial_tendon_test.get_spatial_tendon_offsets().numpy()
        multiple_offsets_np[multiple_indices_np, :] = 0.1
        multiple_offsets = wp.from_numpy(multiple_offsets_np, dtype=wp.float32, device=sim.device)

        spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, offsets=offsets, indices=all_indices)
        multiple_spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=multiple_stiffnesses, dampings=multiple_dampings, limit_stiffnesses=multiple_limit_stiffnesses, offsets=multiple_offsets, indices=multiple_indices)

        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_offsets().numpy(), offsets.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), multiple_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_dampings().numpy(), multiple_dampings.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), multiple_limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_offsets().numpy(), multiple_offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestRigidBodyView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)

        self.test_case.assertEqual(len(balls.prim_paths), self.num_envs)
        self.test_case.assertTrue(f"/envs/env{i}/ball" in balls.prim_paths for i in list(range(self.num_envs)))

        self.finish()

    def on_physics_step(self, stepno, dt):
        pass

class TestRigidBodyProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = wp_utils.arange(balls.count)

        # masses
        masses = np.ones((balls.count, 1)) * 100.0
        wp_masses = wp.from_numpy(masses, dtype=wp.float32, device="cpu")
        balls.set_masses(wp_masses, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_masses().numpy(), masses))

        # inv masses
        self.test_case.assertTrue(balls.get_inv_masses().numpy().shape[0] == self.num_envs)

        # COMs
        com = balls.get_coms().numpy().reshape(self.num_envs, 7)
        com[:, 0] += 0.1
        wp_coms = wp.from_numpy(com, dtype=wp.float32, device="cpu")
        balls.set_coms(wp_coms, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_coms().numpy(), com))

        # inertias
        inertias = balls.get_inertias().numpy().reshape(self.num_envs, 9)
        inertias[:, [0, 4, 8]] += 0.1
        wp_inertias = wp.from_numpy(inertias, dtype=wp.float32, device="cpu")
        balls.set_inertias(wp_inertias, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_inertias().numpy(), inertias))

        # inv inertias
        self.test_case.assertTrue(balls.get_inv_inertias().numpy().shape == (self.num_envs, 9))

        # disable gravity
        gravities = balls.get_disable_gravities().numpy()
        gravities[0:8] = 0
        gravities[8:16] = 1
        wp_gravities = wp.from_numpy(gravities, dtype=wp.uint8, device="cpu")
        balls.set_disable_gravities(wp_gravities, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_disable_gravities().numpy(), wp_gravities.numpy()))

        # disable simulation
        simulations = balls.get_disable_gravities().numpy()
        simulations[0:8] = 1
        simulations[8:16] = 0
        wp_simulations = wp.from_numpy(simulations, dtype=wp.uint8, device="cpu")
        balls.set_disable_simulations(wp_simulations, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_disable_simulations().numpy(), wp_simulations.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestObjectType(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)
        
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, -2.0, 2.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        asset_path = os.path.join(get_asset_root(), "franka.usda")
        actor_path = self.env_template_path.AppendChild("franka")
        transform = Transform((0.0, 2.0, 2.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        actor_path = self.env_template_path.AppendChild("customBall")
        transform = Transform((0.0, 2.0, -2.0))
        self.create_multi_shape_rigid_body(actor_path, transform, 0.15)

    def on_start(self, sim):
        object_type_rigid = sim.get_object_type("/envs/env0/ball")
        self.test_case.assertEqual(object_type_rigid, omni.physics.tensors.ObjectType.RigidBody, "Object type of ball should be RigidBody")

        object_type_rigid = sim.get_object_type("/envs/env0/customBall")
        self.test_case.assertEqual(object_type_rigid, omni.physics.tensors.ObjectType.RigidBody, "Object type of customBall should be RigidBody")

        # ArticulationRootAPI is applied to torso not the top level ant xform, so no articulation will be recognized here. The articulation view should include torso prim 
        object_type_articulation = sim.get_object_type("/envs/env0/ant")
        self.test_case.assertEqual(object_type_articulation, omni.physics.tensors.ObjectType.Invalid, "Object type of ant should be Invalid")

        # note that ArticulationRootLink implies Articulation for the torso prim as well
        object_type_articulation_root = sim.get_object_type("/envs/env0/ant/torso")
        self.test_case.assertEqual(object_type_articulation_root, omni.physics.tensors.ObjectType.ArticulationRootLink, "Object type of ant's torso should be ArticulationRoot")

        object_type_articulation_link = sim.get_object_type("/envs/env0/ant/right_back_leg")
        self.test_case.assertEqual(object_type_articulation_link, omni.physics.tensors.ObjectType.ArticulationLink, "Object type of ant's right_back_leg should be ArticulationLink")

        object_type_articulation_joint = sim.get_object_type("/envs/env0/ant/joints/front_left_foot")
        self.test_case.assertEqual(object_type_articulation_joint, omni.physics.tensors.ObjectType.ArticulationJoint, "Object type of ant's front_left_foot joint should be ArticulationJoint")

        # ArticulationRootAPI is applied to franka, so it should be recognized as an articulation
        object_type_articulation = sim.get_object_type("/envs/env0/franka")
        self.test_case.assertEqual(object_type_articulation, omni.physics.tensors.ObjectType.Articulation, "Object type of franka should be Articulation")

        object_type_root_link = sim.get_object_type("/envs/env0/franka/panda_link0")
        self.test_case.assertEqual(object_type_root_link, omni.physics.tensors.ObjectType.ArticulationRootLink, "Object type of panda's panda_link0 should be ArticulationRootLink")

        object_type_link = sim.get_object_type("/envs/env0/franka/panda_link1")
        self.test_case.assertEqual(object_type_link, omni.physics.tensors.ObjectType.ArticulationLink, "Object type of panda's panda_link1 should be ArticulationLink")

        object_type_joint = sim.get_object_type("/envs/env0/franka/panda_link0/panda_joint1")
        self.test_case.assertEqual(object_type_joint, omni.physics.tensors.ObjectType.ArticulationJoint, "Object type of panda's panda_joint1 should be ArticulationJoint")

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestRigidBodyShapeProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_multi_shape_rigid_body(actor_path, transform, 0.15)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = wp_utils.arange(balls.count)

        # materials
        material_properties = np.zeros((balls.count, balls.max_shapes, 3))
        material_properties[:, :, 0] = 0.8
        material_properties[:, :, 1] = 0.7
        material_properties[:, :, 2] = 0.6
        wp_properties = wp.from_numpy(material_properties, dtype=wp.float32, device="cpu")
        balls.set_material_properties(wp_properties, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_material_properties().numpy(), material_properties))

        # contact offsets
        contact_offsets = np.random.rand(balls.count, balls.max_shapes)
        wp_contact_offsets = wp.from_numpy(contact_offsets, dtype=wp.float32, device="cpu")
        balls.set_contact_offsets(wp_contact_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_contact_offsets().numpy(), contact_offsets))

        # rest offsets, should be less than contact_offset
        rest_offsets = contact_offsets / 2
        wp_rest_offsets = wp.from_numpy(rest_offsets, dtype=wp.float32, device="cpu")
        balls.set_rest_offsets(wp_rest_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(balls.get_rest_offsets().numpy(), rest_offsets))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

class TestRigidBodyStates(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)

        self.balls = balls
        self.all_indices = wp_utils.arange(balls.count, device=sim.device)

        # call concrete subclass implementation
        self.on_start_impl(sim)

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyTransforms(TestRigidBodyStates):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        transforms = self.balls.get_transforms()
        transforms_np = transforms.numpy().reshape((self.balls.count, 7))
        #print(transforms_np)
        z_step = np.linspace(0.0, 1.0, self.balls.count, dtype=np.float32)
        #print(z_step)
        transforms_np[..., 2] += z_step

        transforms = wp.from_numpy(transforms_np, dtype=wp.float32, device=sim.device)
        self.balls.set_transforms(transforms, self.all_indices)

        self.expected_transforms = transforms_np
            
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            transforms = self.balls.get_transforms()
            transforms_np = transforms.numpy().reshape((self.balls.count, 7))
            #print(transforms_np)
            #print(self.expected_transforms)
            self.test_case.assertTrue(np.allclose(transforms_np, self.expected_transforms, rtol=1e-03, atol=1e-04), "expected transforms")
            self.finish()


class TestRigidBodyVelocities(TestRigidBodyStates):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        #print("===== SETTING =====")
        # desired velocities
        vels_np = np.zeros((self.balls.count, 6))
        linear_z = np.linspace(0.1, 1.0, self.balls.count, dtype=np.float32)
        vels_np[..., 2] = linear_z
        #print(linear_z)
        #print(vels_np)

        vels = wp.from_numpy(vels_np, dtype=wp.float32, device=sim.device)
        self.balls.set_velocities(vels, self.all_indices)

        self.expected_vels = vels_np
            
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 10:
            #print("===== GETTING =====")
            vels = self.balls.get_velocities()
            vels_np = vels.numpy().reshape((self.balls.count, 6))
            #print("EXPECTED:")
            #print(self.expected_vels)
            #print("ACTUAL:")
            #print(vels_np)
            self.test_case.assertTrue(np.allclose(vels_np, self.expected_vels, rtol=1e-03, atol=1e-03), "expected velocities")
            self.finish()


class TestRigidBodyAccelerations(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(4, 1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 1.0)
        sim_params.gravity_mag = -10
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

        if _KEEPALIVE:
            self.minsteps = 60

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)

        self.balls = balls
        self.all_indices = wp_utils.arange(balls.count, device=sim.device)
        #print("===== SETTING =====")
        # desired velocities
        vels_np = np.zeros((self.balls.count, 6))
        linear_z = np.linspace(0.0, 1.0, self.balls.count, dtype=np.float32)
        vels_np[..., 2] = linear_z
        #print(linear_z)
        #print(vels_np)

        vels = wp.from_numpy(vels_np, dtype=wp.float32, device=sim.device)
        self.balls.set_velocities(vels, self.all_indices)
            
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 2:
            self.old_vels = self.balls.get_velocities().numpy().reshape((self.balls.count, 6)).copy()
        if stepno == 3:
            self.new_vels = self.balls.get_velocities().numpy().reshape((self.balls.count, 6)).copy()
            acc = self.balls.get_accelerations().numpy().reshape((self.balls.count, 6)).copy()
            acc_finite_difference = (self.new_vels - self.old_vels)/dt
            self.test_case.assertTrue(np.allclose(acc, acc_finite_difference, rtol=1e-03, atol=1e-03), "expected acceleration")
            analytical_acc = np.tile(np.array([0,0,-10,0,0,0]), (self.balls.count, 1))
            self.test_case.assertTrue(np.allclose(acc, analytical_acc, rtol=1e-03, atol=1e-03), "expected acceleration")
            self.finish()

class TestRigidBodyForce(GridTestBase):
    def __init__(self, test_case, device_params, is_global=True):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        self.is_global = is_global
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        from pxr import Gf
        q = Gf.Quatf(Gf.Rotation(Gf.Vec3d([0, 1, 0]), 90).GetQuat())
        self.transform = Transform ((0.0, 0.0, 0.5), q ) 
        self.create_rigid_ball(actor_path, self.transform, 0.15)
        
    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)
        gForce=wp.vec3(0.0, 0.0, 4000.0)

        if self.is_global:
            forces = wp_utils.fill_vec3(balls.count, value=gForce, device=sim.device)
        else:
            # Trasform the gForce to local space
            q=self.transform.q.GetInverse()
            lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
            forces = wp_utils.fill_vec3(balls.count, value=lForce, device=sim.device)
            
        indices = wp_utils.arange(balls.count, device=sim.device)
        balls.apply_forces_and_torques_at_position(forces, None, None, indices, self.is_global)

        self.balls = balls

    def on_physics_step(self, sim, stepno, dt):
        transforms = self.balls.get_transforms()
        z = transforms.numpy().reshape(self.balls.count, 7)[:, 2]
        #print(z)
        if stepno == 1:
            # check if balls above initial position
            self.test_case.assertTrue((z > 0.5).all(), "launch positions")
        elif stepno == 27:
            # check if balls near max height
            self.test_case.assertTrue((z > 1.5).all(), "peak positions")
        elif stepno >= 60:
            # check if balls back down
            self.test_case.assertTrue((z < 0.2).all(), "end positions")
            self.finish()

class TestRigidBodyForceAtPos(GridTestBase):
    def __init__(self, test_case, device_params, is_global=True):
        # set up stage
        self.num_envs=16
        grid_params = GridParams(self.num_envs, 4.0)
        sim_params = SimParams()
        self.is_global=is_global
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.body_per_env=0

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, -10.0, 2.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.body_per_env+=9

        # a different set of ants to test articulation only APIs
        actor_path_2 = self.env_template_path.AppendChild("ant_2")
        transform = Transform((0.0, 10.0, 2.0))
        self.create_actor_from_asset(actor_path_2, transform, asset_path)

        transform = Transform((-0.5, 0.0, 0.0))
        self.create_rigid_ball(actor_path.AppendChild("right_ball"), transform, 0.1)
        self.body_per_env+=1

        transform = Transform((0.5, 0.0, 0.0))
        self.create_rigid_ball(actor_path.AppendChild("left_ball"), transform, 0.1)
        self.body_per_env+=1

    def on_start(self, sim):
        force_offset = 1
        rb_view = sim.create_rigid_body_view(["/envs/*/ant/*_ball",
                                            "/envs/*/ant/*_foot",
                                            "/envs/*/ant/*_leg",
                                            "/envs/*/ant/torso"])
        arti_view = sim.create_articulation_view("/envs/*/ant_2/torso")
        self.rb_view_root_indices =[] # map articulation roots from the rb view to indices used later
        for index, path in enumerate(rb_view.prim_paths):
            if path[-5:] == "torso":
                self.rb_view_root_indices.append(index)

        self.check_rigid_body_view(rb_view, self.num_envs * self.body_per_env)
        self.check_articulation_view(arti_view, self.num_envs, arti_view.max_links, arti_view.max_dofs, True)
        transforms= rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))
        positions=transforms[:,:,0:3]
        rotations=transforms[:,:,3:7]

        transforms= arti_view.get_link_transforms().numpy().reshape((self.num_envs, arti_view.max_links, 7))
        arti_positions=transforms[:,:,0:3]
        arti_rotations=transforms[:,:,3:7]
        self.indices = wp_utils.arange(rb_view.count, device=sim.device)
        self.arti_indices = wp_utils.arange(arti_view.count, device=sim.device)
        gForce=wp.vec3(0.0, 0.0, 100.0)
        if self.is_global:
            positions[:,:,0:3]+=np.array([0,0,force_offset])
            arti_positions[:,:,0:3]+=np.array([0,0,force_offset])
            forces = wp_utils.fill_vec3(rb_view.count, value=gForce, device=sim.device)
            arti_forces = wp_utils.fill_vec3(arti_view.count * arti_view.max_links, value=gForce, device=sim.device)
        else:
            positions[:,:,0:3]=np.array([0,0,force_offset])
            arti_positions[:,:,0:3]=np.array([0,0,force_offset])
            # Trasform the gForce to local space
            from pxr import Gf
            forces = np.zeros((self.num_envs,self.body_per_env,3))
            arti_forces = np.zeros((self.num_envs, arti_view.max_links, 3))
            for e in range(self.num_envs):
                for b in range(self.body_per_env):
                    rot=rotations[e,b,:].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    forces[e, b, :]=lForce
                for b in range(arti_view.max_links):
                    rot=arti_rotations[e,b,:].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    arti_forces[e, b, :]=lForce

            forces = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
            arti_forces =  wp.from_numpy(arti_forces.flatten(), dtype=wp.float32, device=sim.device)

        self.forces=forces
        self.arti_forces=arti_forces
        self.positions = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.arti_positions = wp.from_numpy(arti_positions.flatten(), dtype=wp.float32, device=sim.device)
        self.rb_view=rb_view
        self.arti_view=arti_view

        # # test coms
        # coms = np.zeros((rb_view.count,7))
        # coms[:,-1]=1
        # self.coms = wp.from_numpy(coms, dtype=wp.float32, device="cpu")
        # self.cpuIndices= wp_utils.arange(rb_view.count, device="cpu")
        # self.rb_view.set_coms(self.coms, self.cpuIndices) # this should trigger another mass update

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1 :
            self.rb_view.apply_forces_and_torques_at_position(self.forces, None, self.positions, self.indices, self.is_global)
            self.arti_view.apply_forces_and_torques_at_position(self.arti_forces, None, self.arti_positions, self.arti_indices, self.is_global)
        if stepno==10:
            arti_transforms = self.arti_view.get_link_transforms().numpy().reshape((self.num_envs, self.arti_view.max_links, 7))[:,0]
            transforms = self.rb_view.get_transforms().numpy().reshape((self.num_envs * self.body_per_env, 7))[self.rb_view_root_indices]
            self.test_case.assertTrue(np.allclose(arti_transforms[:,2], transforms[:,2], rtol=1e-03, atol=1e-2), "Similar root height regardless of the type of view to use to apply forces")
            self.test_case.assertTrue(np.allclose(arti_transforms[:,3:], transforms[:,3:], rtol=1e-03, atol=1e-2), "Similar root orientation regardless of the type of view to use to apply forces")
            self.finish()


class TestSdfShapeView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(4, 3.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("shape")
        transform = Transform((0.0, 0.0, 1.0))
        self.length = 0.5
        self.num_points = 100
        object = self.create_sdf_object(actor_path, transform, self.length)
        self.sdF_api_margin = PhysxSchema.PhysxSDFMeshCollisionAPI(object).GetSdfMarginAttr().Get()

    def on_start(self, sim):
        sdfs_view = sim.create_sdf_shape_view("/envs/*/shape", self.num_points * 2)
        self.check_sdf_shape_view(sdfs_view, self.num_envs)
        self.sdfs_view = sdfs_view

    def on_physics_step(self, sim, stepno, dt):
        points = np.zeros((self.num_envs, 2 * self.num_points, 3))
        points[:, :self.num_points, 0] = self.length - self.sdF_api_margin/2
        points[:, self.num_points:, 0] = self.length + self.sdF_api_margin/2
        num_points_row = int(np.sqrt(self.num_points))
        delta = self.length / num_points_row
        for i in range(num_points_row):
            for j in range(num_points_row):
                # start from some small distance away to make sure points don't fall on the surface
                points[:, i * num_points_row + j, 1] = -self.length + 2.0 * i * delta - delta /10
                points[:, i * num_points_row + j, 2] = -self.length + 2.0 * j * delta - delta /10
                points[:, self.num_points + i * num_points_row + j, 1] = -self.length  + 2.0 * i * delta - delta /10
                points[:, self.num_points + i * num_points_row + j, 2] = -self.length  + 2.0 * j * delta - delta /10
        points_wp = wp.from_numpy(points.flatten(), dtype=wp.float32, device=sim.device)
        sdfs = self.sdfs_view.get_sdf_and_gradients(points_wp)
        sdfs_np = sdfs.numpy().reshape(self.sdfs_view.count, 2 * self.num_points, 4)
        d = np.abs(points) - self.length
        expected = np.linalg.norm(np.maximum(d, 0.0), axis=2) + np.minimum(np.max(d, axis=2), 0.0)
        # print("sdf= \n", np.dstack((sdfs_np[0, :, -1] , expected[0, :])))
        self.test_case.assertTrue(
            np.allclose(sdfs_np[:, :, -1], expected, rtol=0.1, atol=0.1), "expected sdf values"
        )
        # inside grad
        g1 = np.zeros((self.sdfs_view.count, 2 * self.num_points, 3))
        # outside grad
        g2 = np.zeros((self.sdfs_view.count, 2 * self.num_points, 3))
        distance =  points - self.length
        for i in range(self.sdfs_view.count):
            for j in range(self.num_points  * 2):
                is_inside = np.max(d[i, j]) < 0
                if is_inside:
                    c = np.argmax(d[i, j])
                    # inside sdf gradient direction is from the sample point to surface 
                    g1[i, j, c] = 1 
                else:
                    # outside sdf gradient direction is from the surface to the sample point 
                    grad = np.sign(distance[i, j]) * np.maximum(d[i, j], 0.0) / np.linalg.norm(np.maximum(d[i, j], 0.0))
                    g2[i, j, :] = grad

        exptected_gradient = g1 + g2

        self.test_case.assertTrue(
            np.allclose(sdfs_np[:, :, :-1], exptected_gradient, atol=0.1), "expected sdf gradient values"
        )
        self.finish()

class TestRigidContacts(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(32, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("box")
        transform = Transform((0.0, 0.0, 0.5))
        box = self.create_rigid_box(actor_path, transform, Gf.Vec3f(0.3, 0.3, 0.3))

        # set a known mass
        box_mass = 1.0
        mass_api = UsdPhysics.MassAPI(box)
        mass_api.GetMassAttr().Set(box_mass)

        # !!! disable sleeping, because sleeping bodies don't get contact reports
        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(box)
        physx_rb.CreateSleepThresholdAttr().Set(0)

        # add contact sensor
        contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(box)
        contact_sensor.CreateThresholdAttr().Set(0)

        self.box_mass = box_mass

    def on_start(self, sim):
        contacts = sim.create_rigid_contact_view(["/envs/env[0-9]/box","/envs/env[1-3][0-9]/box" ])
        self.check_rigid_contact_view(contacts, self.num_envs, 0)
        self.contacts = contacts

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 60:
            net_forces = self.contacts.get_net_contact_forces(dt)
            net_forces_np = net_forces.numpy().reshape(self.contacts.sensor_count, 3)
            expected = np.array([[0.0, 0.0, 9.81]] * self.contacts.sensor_count, dtype=np.float32)
            #print(net_forces_np)
            #print(expected)
            self.test_case.assertTrue(np.allclose(net_forces_np, expected, rtol=1e-03, atol=1e-2), "expected net contact forces")
            self.finish()


class TestRigidContactMatrix(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs= 32
        grid_params = GridParams(self.num_envs, 1.0)
        self.g = 10.0
        sim_params = SimParams()
        sim_params.gravity_mag = self.g
        self.device_params= device_params
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        ball_path = self.env_template_path.AppendChild("ball")
        box_path = self.env_template_path.AppendChild("box")
        ball_transform = Transform((0.0, 0.0, 1.0))
        box_transform = Transform((0.0, 0.0, 0.4))
        ball = self.create_rigid_ball(ball_path, ball_transform, 0.25)
        box = self.create_rigid_box(box_path, box_transform, Gf.Vec3f(0.3, 0.3, 0.3))

        # set known masses
        ball_mass = 1.0
        box_mass = 1.0
        ball_mass_api = UsdPhysics.MassAPI(ball)
        ball_mass_api.GetMassAttr().Set(ball_mass)
        box_mass_api = UsdPhysics.MassAPI(box)
        box_mass_api.GetMassAttr().Set(box_mass)

        # !!! disable sleeping, because sleeping bodies don't get contact reports
        ball_physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(ball)
        ball_physx_rb.CreateSleepThresholdAttr().Set(0)
        box_physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(box)
        box_physx_rb.CreateSleepThresholdAttr().Set(0)

        # add contact sensors
        ball_contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(ball)
        ball_contact_sensor.CreateThresholdAttr().Set(0)
        box_contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(box)
        box_contact_sensor.CreateThresholdAttr().Set(0)

        self.ball_mass = ball_mass
        self.box_mass = box_mass

        # ensure the surface material paramter for the friction test
        self.materialPath = "/material"
        UsdShade.Material.Define(self.stage, self.materialPath)
        material = UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self.materialPath))
        material.CreateDynamicFrictionAttr(0.5)
        material.CreateStaticFrictionAttr(0.5)
        material.CreateRestitutionAttr(0.0)
        physicsUtils.add_physics_material_to_prim(self.stage, box, self.materialPath)
        physicsUtils.add_physics_material_to_prim(self.stage, ball, self.materialPath)
        physicsUtils.add_physics_material_to_prim(self.stage, self.stage.GetPrimAtPath("/groundPlane"), self.materialPath)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)
        boxes = sim.create_rigid_body_view("/envs/*/box")
        self.check_rigid_body_view(balls, self.num_envs)
        self.box_indices = wp_utils.arange(boxes.count, device=sim.device)
        self.boxes = boxes

        # select all the prims but via a list of expression 
        sensor_pattern = [
            "/envs/env[0-9]/ball", # ball0 to ball9
            "/envs/env[1-3][0-9]/ball" # ball10-ball32 (self.num_envs)
            ]
        # when the sensor_pattern is a list then for each element of the list a separate filter list should be provided as follows
        # this ensures that the pattern matching can be done correctly between the filter prims and the corresponding sensor prim
        ball_filter_patterns = [
            ["/groundPlane", "/envs/env[0-9]/box"], # filter patterns for sensor_pattern[0]
            ["/groundPlane", "/envs/env[1-3][0-9]/box"], # filter patterns for sensor_pattern[1]
        ]
        ball_contacts = sim.create_rigid_contact_view(sensor_pattern, filter_patterns=ball_filter_patterns, max_contact_data_count=self.num_envs * 3)
        self.check_rigid_contact_view(ball_contacts, self.num_envs, len(ball_filter_patterns[-1]))
        expected_sensor_paths = [ f"/envs/env{i}/ball" for i in range(self.num_envs)]
        expected_filter_paths = [["/groundPlane", f"/envs/env{i}/box"] for i in range(self.num_envs)]
        self.test_case.assertTrue(ball_contacts.sensor_paths == expected_sensor_paths)
        self.test_case.assertTrue(ball_contacts.filter_paths == expected_filter_paths)

        box_filter_patterns = [
            "/groundPlane",
            "/envs/*/ball",
        ]
        box_contacts = sim.create_rigid_contact_view("/envs/*/box", filter_patterns=box_filter_patterns, max_contact_data_count=self.num_envs * 6)
        self.check_rigid_contact_view(box_contacts, self.num_envs, len(box_filter_patterns))
        expected_sensor_paths = [ f"/envs/env{i}/box" for i in range(self.num_envs)]
        expected_filter_paths = [["/groundPlane", f"/envs/env{i}/ball"] for i in range(self.num_envs)]
        self.test_case.assertTrue(box_contacts.sensor_paths == expected_sensor_paths)
        self.test_case.assertTrue(box_contacts.filter_paths == expected_filter_paths)

        # keep the ball hovering for the first little while
        ball_indices = wp_utils.arange(balls.count, device=sim.device)
        vels = wp.zeros(balls.count * 6, dtype=wp.float32, device=sim.device)
        balls.set_velocities(vels, ball_indices)
        hover_force = self.ball_mass * self.g
        ball_forces = wp_utils.fill_vec3(balls.count, value=wp.vec3(0.0, 0.0, hover_force), device=sim.device)
        balls.apply_forces(ball_forces, ball_indices)

        self.balls = balls
        self.ball_forces = ball_forces
        self.ball_indices = ball_indices
        self.ball_contacts = ball_contacts
        self.box_contacts = box_contacts

    def on_physics_step(self, sim, stepno, dt):

        if stepno < 60:
            # keep the ball hovering
            self.balls.apply_forces(self.ball_forces, self.ball_indices)

        if stepno == 60:
            ball_force_matrix = self.ball_contacts.get_contact_force_matrix(dt)
            ball_force_matrix_np = ball_force_matrix.numpy().reshape(self.ball_contacts.sensor_count, self.ball_contacts.filter_count, 3)
            ball_net_forces = self.ball_contacts.get_net_contact_forces(dt)
            ball_net_forces_np = ball_net_forces.numpy().reshape(self.ball_contacts.sensor_count, 3)
            # print("Ball filter 0: ", ball_force_matrix_np[0, 0])
            # print("Ball filter 1: ", ball_force_matrix_np[0, 1])
            # print("Ball net force:", ball_net_forces_np)

            box_force_matrix = self.box_contacts.get_contact_force_matrix(dt)
            box_force_matrix_np = box_force_matrix.numpy().reshape(self.box_contacts.sensor_count, self.box_contacts.filter_count, 3)
            box_net_forces = self.box_contacts.get_net_contact_forces(dt)
            box_net_forces_np = box_net_forces.numpy().reshape(self.box_contacts.sensor_count, 3)
            # print("Box filter 0: ", box_force_matrix_np[0, 0])
            # print("Box filter 1: ", box_force_matrix_np[0, 1])
            # print("Box net force:", box_net_forces_np)

            ball_net_forces_expected = np.array([[0.0, 0.0, 0.0]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_net_forces_np, ball_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_net_forces_expected = np.array([[0.0, 0.0, self.g]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_net_forces_np, box_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            ball_force_matrix_expected = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_force_matrix_np, ball_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_force_matrix_expected = np.array([[[0.0, 0.0, self.g], [0.0, 0.0, 0.0]]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_force_matrix_np, box_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_forces = wp_utils.fill_vec3(self.boxes.count, value=wp.vec3(self.g, 0.0, 0.0), device=sim.device)
            self.boxes.apply_forces(box_forces, self.box_indices)

        elif stepno == 120:
            ball_force_matrix = self.ball_contacts.get_contact_force_matrix(dt)
            ball_force_matrix_np = ball_force_matrix.numpy().reshape(self.ball_contacts.sensor_count, self.ball_contacts.filter_count, 3)
            ball_net_forces = self.ball_contacts.get_net_contact_forces(dt)
            ball_net_forces_np = ball_net_forces.numpy().reshape(self.ball_contacts.sensor_count, 3)
            # print("Ball filter 0: ", ball_force_matrix_np[0, 0])
            # print("Ball filter 1: ", ball_force_matrix_np[0, 1])
            # print("Ball net force:", ball_net_forces_np)

            box_force_matrix = self.box_contacts.get_contact_force_matrix(dt)
            box_force_matrix_np = box_force_matrix.numpy().reshape(self.box_contacts.sensor_count, self.box_contacts.filter_count, 3)
            box_net_forces = self.box_contacts.get_net_contact_forces(dt)
            box_net_forces_np = box_net_forces.numpy().reshape(self.box_contacts.sensor_count, 3)
            # print("Box filter 0: ", box_force_matrix_np[0, 0])
            # print("Box filter 1: ", box_force_matrix_np[0, 1])
            # print("Box net force:", box_net_forces_np)

            ball_net_forces_expected = np.array([[0.0, 0.0, self.g]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_net_forces_np, ball_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_net_forces_expected = np.array([[0.0, 0.0, self.g]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_net_forces_np, box_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            ball_force_matrix_expected = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, self.g]]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_force_matrix_np, ball_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_force_matrix_expected = np.array([[[0.0, 0.0, 2*self.g], [0.0, 0.0, -self.g]]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_force_matrix_np, box_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            # Getting individual contact forces 
            box_forces, box_points, box_normals, box_distances, box_counts, box_start_indices = self.box_contacts.get_contact_data(dt)
            ball_forces, ball_points, ball_normals, ball_distances, ball_counts, ball_start_indices = self.ball_contacts.get_contact_data(dt)
            box_force_aggregate = np.zeros(( self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            for i in range(box_counts.shape[0]):
                for j in range(box_counts.shape[1]):
                    start_idx=box_start_indices.numpy()[i,j]
                    count = box_counts.numpy()[i,j]
                    if count > 0:
                        forces = box_forces.numpy()[start_idx:start_idx+count] * box_normals.numpy()[start_idx:start_idx+count]
                        box_force_aggregate[i,j] = np.sum(forces, axis= 0)
            
            # print("box_force \n", box_forces)
            # print("box_counts \n", box_counts)
            # print("box_start_indices \n", box_start_indices)
            # print("box_force_aggregate \n", box_force_aggregate)
            self.test_case.assertTrue(np.allclose(box_force_matrix.numpy(), box_force_aggregate, rtol=1e-03, atol=1e-2), "aggregate of the individual contact forces equals the contact matrix value")

            ball_force_aggregate = np.zeros(( self.ball_contacts.sensor_count, self.ball_contacts.filter_count, 3))
            for i in range(ball_counts.shape[0]):
                for j in range(ball_counts.shape[1]):
                    start_idx=ball_start_indices.numpy()[i,j]
                    count = ball_counts.numpy()[i,j]
                    if count > 0:
                        forces = ball_forces.numpy()[start_idx:start_idx+count] * ball_normals.numpy()[start_idx:start_idx+count]
                        ball_force_aggregate[i,j] = np.sum(forces, axis= 0)
            
            # print("ball_force \n", ball_force)
            # print("ball_counts \n", ball_counts)
            # print("ball_start_indices \n", ball_start_indices)
            # print("ball_force_aggregate \n", ball_force_aggregate)
            self.test_case.assertTrue(np.allclose(ball_force_matrix.numpy(), ball_force_aggregate, rtol=1e-03, atol=1e-2), "aggregate of the individual contact forces equals the contact matrix value")
            # apply a tangential force to test the friction forces
            # only gpu pipeline is supported for now
            if not self.device_params.use_gpu_pipeline:
                self.finish()
            balls_forces = wp_utils.fill_vec3(self.balls.count, value=wp.vec3(self.g, 0.0, 0.0), device=sim.device)
            ball_indices = wp_utils.arange(self.balls.count, device=sim.device)
            self.boxes.apply_forces(balls_forces, ball_indices)

        elif stepno == 121:
            box_frictions, box_points, box_counts, box_start_indices = self.box_contacts.get_friction_data(dt)
            # sum accros all the points
            box_friction_aggregate = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            box_friction_patch_point_average = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            # print("box_counts data\n",  box_counts)
            # print("box_start_indices data\n",  box_start_indices)
            # print("box_frictions data\n",  box_frictions)
            # print("box_points data\n",  box_points)
            # print("anchor_counts: \n", anchor_counts)

            for i in range(box_counts.shape[0]):
                for j in range(box_counts.shape[1]):
                    start_idx=box_start_indices.numpy()[i,j]
                    count = box_counts.numpy()[i,j]
                    if count > 0:
                        # the following is a num_contact x 3 matrix
                        friction = box_frictions.numpy()[start_idx: start_idx + count]
                        # print(box_frictions.shape, friction.shape)
                        # the following is a 1 x 3 matrix
                        box_friction_aggregate[i,j,:] = np.sum(friction, axis= 0)
                        # the following is a num_contact x 3 matrix
                        anchor_points = box_points.numpy()[start_idx: start_idx + count]
                        # sum of the anchor points forces num_contact x 3 matrix
                        patch_force= np.sum(np.sqrt(friction*friction), axis=0)
                        # weighted point w.r.t. to the magnitude of the anchor force comparing to the overall force across anchor points, shape num_contact x 3 matrix
                        weighted_point = np.sum(anchor_points*np.abs(friction), axis=0)/patch_force                
                        box_friction_patch_point_average[i,j] =weighted_point

            # print("box_friction_aggregate ", box_friction_aggregate)
            net_from_matrix = np.sum(box_friction_aggregate, axis= (1))
            # print("box_friction_aggregate\n", box_friction_aggregate)
            # print("net_from_matrix\n", net_from_matrix)
            # sum across different anchor points
            box_force_matrix_expected = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            # the only friction force is between the box and ground ft <= mu * fn = 0.5 * (2 * m * g) = 10 -> no slip
            box_force_matrix_expected[:,0,0]=-self.g
            self.test_case.assertTrue(np.allclose(box_friction_aggregate, box_force_matrix_expected, rtol=1e-02, atol=5e-1), "expected net contact forces")

            box_fricton_point_expected = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            box_fricton_point_expected[:,0,:]=self.boxes.get_transforms().numpy()[:,0:3] - np.array([0,0, 0.15])
            box_fricton_point_expected[:,1,:]=self.boxes.get_transforms().numpy()[:,0:3] + np.array([0,0, 0.15])
            self.test_case.assertTrue(np.allclose(box_friction_patch_point_average, box_fricton_point_expected, rtol=1e-02, atol=1e-1), "expected net contact forces")

            self.finish()


class TestRigidContactPerfTest(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs= 1
        self.num_filters_per_env = 4096
        self.num_sensors = 16
        grid_params = GridParams(self.num_envs, 100.0)
        self.g = 10.0
        sim_params = SimParams()
        sim_params.gravity_mag = self.g
        self.device_params= device_params
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.physx_scene.GetGpuMaxRigidContactCountAttr().Set(1000000)
        ball_mass = 1.0
        box_mass = 1.0

        grid_num = np.floor(np.sqrt(self.num_sensors))
        # set up env template
        for i in range(self.num_sensors):
            box_path = self.env_template_path.AppendChild(f"box_{i}")
            box_transform = Transform((i%grid_num - grid_num/2, i//grid_num-grid_num/2, 0.4))
            box = self.create_rigid_box(box_path, box_transform, Gf.Vec3f(0.3, 0.3, 0.3))
            # set known masses
            box_mass_api = UsdPhysics.MassAPI(box)
            box_mass_api.GetMassAttr().Set(box_mass)
            # !!! disable sleeping, because sleeping bodies don't get contact reports
            box_physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(box)
            box_physx_rb.CreateSleepThresholdAttr().Set(0)
            # add contact sensors
            box_contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(box)
            box_contact_sensor.CreateThresholdAttr().Set(0)

        # ensure the surface material paramter for the friction test
        grid_num = np.floor(np.sqrt(self.num_filters_per_env))
        for i in range(self.num_filters_per_env):
            ball_path = self.env_template_path.AppendChild(f"ball_{i}")
            ball_transform = Transform((i%grid_num - grid_num/2, i//grid_num-grid_num/2, 1.0))
            ball = self.create_rigid_ball(ball_path, ball_transform, 0.2)
            ball_mass_api = UsdPhysics.MassAPI(ball)
            ball_mass_api.GetMassAttr().Set(ball_mass)
            ball_physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(ball)
            ball_physx_rb.CreateSleepThresholdAttr().Set(0)

            ball_contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(ball)
            ball_contact_sensor.CreateThresholdAttr().Set(0)

        self.ball_mass = ball_mass
        self.box_mass = box_mass


    def on_start(self, sim):
        import time
        t0 = time.time()
        box_contacts = sim.create_rigid_contact_view(
                                                    [f"/envs/env0/box_{j}" for j in range(self.num_sensors)], 
                                                    filter_patterns= [[f"/envs/env0/ball_{j}" for j in range(self.num_filters_per_env)]] * self.num_sensors, 
                                                    max_contact_data_count=self.num_envs * self.num_filters_per_env)
        t1 = time.time()
        print("create_rigid_contact_view : time taken", t1-t0)
        self.check_rigid_contact_view(box_contacts, self.num_sensors, self.num_filters_per_env)
        expected_sensor_paths = [f"/envs/env0/box_{j}" for j in range(self.num_sensors)]
        expected_filter_paths = [[f"/envs/env0/ball_{j}" for j in range(self.num_filters_per_env)]] * self.num_sensors
        self.test_case.assertTrue(box_contacts.sensor_paths == expected_sensor_paths)
        self.test_case.assertTrue(box_contacts.filter_paths == expected_filter_paths)
        self.test_case.assertTrue(t1-t0 < 1.0, "create_rigid_contact_view should be fast when input lists are actual prim paths rather than patterns")

    def on_physics_step(self, sim, stepno, dt):
        self.finish()

class TestArticulationContacts(GridTestBase): 
    def prepare_contacts(self, prim_at_path, apply_rigid_body_api=True):
        if apply_rigid_body_api:
            rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim_at_path)
            rb_api.CreateSleepThresholdAttr().Set(0)

        # prepare contact sensors
        cr_api = PhysxSchema.PhysxContactReportAPI.Apply(prim_at_path)
        cr_api.CreateThresholdAttr().Set(0)

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(4, 4.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)
        envs_prim=self.stage.GetPrimAtPath(self.env_template_path)

        actor_path = self.env_template_path.AppendChild("box")
        transform = Transform((0.0, 0.0, 0.5))
        box_size=0.3
        box = self.create_rigid_box(actor_path, transform, Gf.Vec3f(box_size,box_size,box_size))

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        ant_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, .0, 2.0))
        self.create_actor_from_asset(ant_path, transform, asset_path)
        ant_prim=self.stage.GetPrimAtPath(ant_path)

        # set a known mass
        box_mass = 1.0
        mass_api = UsdPhysics.MassAPI(box)
        mass_api.GetMassAttr().Set(box_mass)
        self.prepare_contacts(box)

        for prim in self.stage.Traverse():
            self.prepare_contacts(prim, False)

    def on_start(self, sim):
        self.cubes = sim.create_rigid_body_view("/envs/*/box")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")

        self.cubes_contact = sim.create_rigid_contact_view("/envs/*/box")
        self.check_rigid_contact_view(self.cubes_contact, self.num_envs, 0)

        self.cube_contact_filtered = sim.create_rigid_contact_view("/envs/*/box", filter_patterns= [ "/groundPlane", "/envs/*/ant/torso"])
        self.check_rigid_contact_view(self.cube_contact_filtered, self.num_envs, 2)

        self.ant_contact = sim.create_rigid_contact_view("/envs/*/ant/*")
        self.check_rigid_contact_view(self.ant_contact, self.num_envs * 9, 0)

        self.ant_contact_filtered = sim.create_rigid_contact_view("/envs/*/ant/torso", filter_patterns= [ "/groundPlane", "/envs/*/box"])
        self.check_rigid_contact_view(self.ant_contact_filtered, self.num_envs, 2)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 100:
            net_forces = self.cubes_contact.get_net_contact_forces(dt).numpy().reshape(self.cubes_contact.sensor_count, 3)
            expected = np.array([[0.0, 0.0, 9.81]] * self.cubes_contact.sensor_count, dtype=np.float32)

            ### match the sensor names from rigid contact view with the articulation
            ant_net_forces = self.ant_contact.get_net_contact_forces(dt).numpy().reshape(self.num_envs, self.ants.max_links, 3)
            ant_net_forces =ant_net_forces.reshape(self.ant_contact.sensor_count, 3)
            sensor_names= np.array(self.ant_contact.sensor_names)
            for key, value in self.ants.shared_metatype.link_indices.items():
                sensor_indices=np.where(sensor_names==key)
                # print(key, "\n", ant_net_forces[sensor_indices])

            box_contact_matrix=self.cube_contact_filtered.get_contact_force_matrix(dt).numpy().reshape(self.num_envs, 2, 3)
            torso_contact_matrix=self.ant_contact_filtered.get_contact_force_matrix(dt).numpy().reshape(self.num_envs, 2, 3)

            self.test_case.assertTrue(np.allclose(torso_contact_matrix[:,1,:], -box_contact_matrix[:,1,:], rtol=1e-02, atol=1e-2), "expected net contact forces")
            self.test_case.assertTrue(np.allclose(net_forces, expected, rtol=1e-02, atol=1e-2), "expected net contact forces")

            self.finish()

class TestParticleClothPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        cloth_path = self.env_template_path.AppendChild("cloth")
        self.initial_positions = self.create_particle_cloth(cloth_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        cloths = sim.create_particle_cloth_view("/envs/*/cloth")
        self.check_particle_cloth_view(cloths, self.num_envs)
        self.cloths = cloths

    def on_physics_step(self, sim, stepno, dt):
        positions = self.cloths.get_positions()
        # reshape because warp just has flat arrays.
        z = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:, :, 2].flatten()

        if stepno == 10:
            # reset first and last one
            pos_numpy = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)
            pos_numpy[0, :, 2] = 0.5
            pos_numpy[-1, :, 2] = 0.5

            positions = wp.from_numpy(pos_numpy, dtype=wp.float32, device=sim.device)
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_positions(positions, indices)
        elif stepno > 1 and stepno < 10:
            self.test_case.assertTrue((z <= self.z).all(), "falling")
        elif stepno == 11:
            z1 = z[:self.cloths.max_particles_per_cloth]
            z1old = self.z[:self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((z1 >= z1old).all(), "position reset")
        
            zmid = z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            zmidold = self.z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((zmid <= zmidold).all(), "falling")

            zend = z[-self.cloths.max_particles_per_cloth:]
            zendold = self.z[-self.cloths.max_particles_per_cloth:]
            self.test_case.assertTrue((zend >= zendold).all(), "position reset")

        self.z = z

        if stepno >= 20:
            self.finish()


class TestParticleClothVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        cloth_path = self.env_template_path.AppendChild("cloth")
        self.initial_positions = self.create_particle_cloth(cloth_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        cloths = sim.create_particle_cloth_view("/envs/*/cloth")
        self.check_particle_cloth_view(cloths, self.num_envs)
        self.cloths = cloths

    def on_physics_step(self, sim, stepno, dt):
        velocities = self.cloths.get_velocities()

        # reshape because warp just has flat arrays.
        z = velocities.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:, :, 2].flatten()

        if stepno == 10:
            # add a positive velocity for the first and last one
            pos_numpy = velocities.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)
            pos_numpy[0, :, 2] = 2.0
            pos_numpy[-1, :, 2] = 2.0        
            
            velocities = wp.from_numpy(pos_numpy, dtype=wp.float32, device=sim.device)
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_velocities(velocities, indices)
        elif stepno > 1 and stepno < 10:
            self.test_case.assertTrue((z <= self.z).all(), "falling") # falling down, negative velocity!
        elif stepno == 11:
            z1 = z[:self.cloths.max_particles_per_cloth]
            z1old = self.z[:self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((z1 >= z1old).all(), "velocity reset") # positive velovity
            self.test_case.assertTrue((z1 > 0.0).all(), "positive velocity")

            zmid = z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            zmidold = self.z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((zmid <= zmidold).all(), "falling")

            zend = z[-self.cloths.max_particles_per_cloth:]
            zendold = self.z[-self.cloths.max_particles_per_cloth:]
            self.test_case.assertTrue((zend >= zendold).all(), "velocity reset")
            self.test_case.assertTrue((z1 > 0.0).all(), "positive velocity")

        self.z = z

        if stepno >= 20:
            self.finish()

class TestParticleClothMasses(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        cloth_path = self.env_template_path.AppendChild("cloth")
        self.initial_positions = self.create_particle_cloth(cloth_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        cloths = sim.create_particle_cloth_view("/envs/*/cloth")
        self.check_particle_cloth_view(cloths, self.num_envs)
        self.cloths = cloths

    def on_physics_step(self, sim, stepno, dt):
        positions = self.cloths.get_positions()

        # reshape because warp just has flat arrays.
        z = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:, :, 2].flatten()

        if stepno == 10:
            # set invMass to 0 for first and last one
            mass_numpy = np.zeros((self.cloths.count, self.cloths.max_particles_per_cloth))
            masses = wp.from_numpy(mass_numpy, dtype=wp.float32, device=sim.device)
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_masses(masses, indices)
        elif stepno > 1 and stepno < 10:
            self.test_case.assertTrue((z <= self.z).all(), "falling") # falling down!
        elif stepno == 11:
            z1 = z[:self.cloths.max_particles_per_cloth]
            z1old = self.z[:self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((z1 == z1old).all(), "mass set to 0 - no movement") # fixed position because of mass 0.

            zmid = z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            zmidold = self.z[self.cloths.max_particles_per_cloth:-self.cloths.max_particles_per_cloth]
            self.test_case.assertTrue((zmid <= zmidold).all(), "falling")

            zend = z[-self.cloths.max_particles_per_cloth:]
            zendold = self.z[-self.cloths.max_particles_per_cloth:]
            self.test_case.assertTrue((zend == zendold).all(), "mass set to 0 - no movement")

            # set mass to something positive again
            mass_numpy = np.full((self.cloths.count, self.cloths.max_particles_per_cloth), 5.0, dtype=float)
            masses = wp.from_numpy(mass_numpy, dtype=wp.float32, device=sim.device)
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_masses(masses, indices)

        self.z = z

        if stepno >= 20:
            self.finish()

class SpringEnum:
    DAMPING = 0
    STIFFNESS = 1

class TestParticleClothSprings(GridTestBase):

    def __init__(self, test_case, device_params, spring_attr):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.spring_attr = spring_attr

        # set up env template
        cloth_path = self.env_template_path.AppendChild("cloth")
        # setup cloths with low spring stiffness
        self.initial_positions = self.create_particle_cloth(cloth_path=cloth_path, stiffness=10.0, damping=0.8, dimX=1, dimY=1, rotate=True)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        cloths = sim.create_particle_cloth_view("/envs/*/cloth")
        self.check_particle_cloth_view(cloths, self.num_envs)
        self.cloths = cloths

    def on_physics_step(self, sim, stepno, dt):
        positions = self.cloths.get_positions()

        # reshape because warp just has flat arrays.
        z = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:, 2:, 2].flatten()

        z1s = z[:2]
        z2s = z[2:4]
        z14s = z[28:30]
        z15s = z[30:]

        if stepno == 1:
            # set mass to 0 for first 2 vertices of each cloth
            masses = self.cloths.get_masses()
            mass = masses.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 1)
            mass[:,:2,:] = 0.0

            masses_new = wp.from_numpy(mass, dtype=wp.float32, device=sim.device)
            indices_numpy = np.arange(0, self.cloths.count)
            indices_new = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_masses(masses_new, indices_new)

        elif stepno == 10:
            if self.spring_attr == SpringEnum.DAMPING:
                # set damping to 0.02 for first and last one
                old_val = self.cloths.get_spring_damping()
                new_val = 0.02
            elif self.spring_attr == SpringEnum.STIFFNESS:
                old_val = self.cloths.get_spring_stiffness()
                new_val = 1

            old_val_numpy = old_val.numpy().reshape(self.cloths.count, self.cloths.max_springs_per_cloth, 1)
            old_val_numpy[0, :, :] = new_val
            old_val_numpy[self.cloths.count - 1, :, :] = new_val
            new_data = wp.from_numpy(old_val_numpy, dtype=wp.float32, device=sim.device)
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            if self.spring_attr == SpringEnum.DAMPING:
                self.cloths.set_spring_damping(new_data, indices)
            elif self.spring_attr == SpringEnum.STIFFNESS:
                self.cloths.set_spring_stiffness(new_data, indices)
        elif stepno < 10:
            # before we change anything, positions should be almost equal for all
            self.test_case.assertTrue((np.abs(z1s - z2s) < 1e-4).all(), "all cloths behaving the same")
            self.test_case.assertTrue((np.abs(z14s - z15s) < 1e-4).all(), "all cloths behaving the same")
        elif stepno > 10:
            self.test_case.assertFalse((np.abs(z1s - z2s) < 1e-4).all(), "cloth behaving differently")
            self.test_case.assertFalse((np.abs(z14s - z15s) < 1e-4).all(), "cloth behaving differently")           

        if stepno >= 20:
            self.finish()    


class OffsetEnum:
    FLUIDRESTOFFSET = 0
    SOLIDRESTOFFSET = 1
    PARTICLECONTACTOFFSET = 2


class TestParticleSystemOffsets(GridTestBase):
    # there is no real good way to test the actual effect of changing these offsets
    # so this only tests if we are getting the same thing out of physx as we put in
    # the SDK tests should cover the effects

    def __init__(self, test_case, device_params, offsetType):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.offsetType = offsetType

        # set up env template
        particle_path = self.env_template_path.AppendChild("cloth")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_systems = sim.create_particle_system_view("/envs/*/particleSystem")
        self.check_particle_system_view(particle_systems, self.num_envs)
        self.particle_systems = particle_systems

    def on_physics_step(self, sim, stepno, dt):
        if self.offsetType == OffsetEnum.SOLIDRESTOFFSET:
            offsets = self.particle_systems.get_solid_rest_offsets()
        elif self.offsetType == OffsetEnum.FLUIDRESTOFFSET:
            offsets = self.particle_systems.get_fluid_rest_offsets()
        elif self.offsetType == OffsetEnum.PARTICLECONTACTOFFSET:
            offsets = self.particle_systems.get_particle_contact_offsets()
        else:
            self.test_case.assertFalse(True)

        off_np = offsets.numpy()

        if stepno == 5:
            if self.offsetType != OffsetEnum.PARTICLECONTACTOFFSET:
                off_np[0] = 0.08
                off_np[1] = 0.02
                off_np[-1] = 0.04
            else:
                off_np[0] = 0.15
                off_np[1] = 0.10
                off_np[-1] = 0.25

            indices_np = np.array([0, 1, self.particle_systems.count - 1])

            off_wp = wp.from_numpy(off_np, dtype=wp.float32, device="cpu")
            indices_wp = wp.from_numpy(indices_np, dtype=wp.uint32, device="cpu")
            if self.offsetType == OffsetEnum.SOLIDRESTOFFSET:
                self.particle_systems.set_solid_rest_offsets(off_wp, indices_wp)
            elif self.offsetType == OffsetEnum.FLUIDRESTOFFSET:
                self.particle_systems.set_fluid_rest_offsets(off_wp, indices_wp)
            elif self.offsetType == OffsetEnum.PARTICLECONTACTOFFSET:
                self.particle_systems.set_particle_contact_offsets(off_wp, indices_wp)
            else:
                self.test_case.assertFalse(True)

        elif stepno > 1:
            self.test_case.assertTrue((off_np == self.off_np).all())

        self.off_np = off_np

        if stepno > 10:
            self.finish()


class TestParticleSystemWind(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # AD TODO: setup an actual dynamics test to check if this has any effect.

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_systems = sim.create_particle_system_view("/envs/*/particleSystem")
        self.check_particle_system_view(particle_systems, self.num_envs)
        self.particle_systems = particle_systems

    def on_physics_step(self, sim, stepno, dt):
        wind = self.particle_systems.get_wind()
        wind_np = wind.numpy().reshape(self.particle_systems.count, 3)

        if stepno == 5:
            
            wind_np[0, :] = [1.0, 1.0, 1.0]
            wind_np[1, :] = [2.0, 2.0, 2.0]
            wind_np[-1, :] = [0.25, 3.5, 2.0]

            indices_np = np.array([0, 1, self.particle_systems.count - 1])

            wind_wp = wp.from_numpy(wind_np, dtype=wp.float32, device="cpu")
            indices_wp = wp.from_numpy(indices_np, dtype=wp.uint32, device="cpu")
            self.particle_systems.set_wind(wind_wp, indices_wp)

        elif stepno > 1:
            self.test_case.assertTrue((wind_np == self.wind_np).all())

        self.wind_np = wind_np

        if stepno > 10:
            self.finish()


class TestParticleMaterialFriction(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_materials = sim.create_particle_material_view("/envs/*/particleMaterial")
        particle_cloths = sim.create_particle_cloth_view("/envs/*/particles")
        self.check_particle_material_view(particle_materials, self.num_envs)
        self.check_particle_cloth_view(particle_cloths, self.num_envs)
        self.particle_materials = particle_materials
        self.cloths = particle_cloths

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            # set velocity to 10 and z position to low such that the cloths start touching the plane quickly.
            velocities = self.cloths.get_velocities()
            velocities_np = velocities.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)
            velocities_np[:, :, 1] = 10.0

            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)
            positions_np[:, :, 2] = 0.02

            velocities = wp.from_numpy(velocities_np, dtype=wp.float32, device=sim.device)
            positions = wp.from_numpy(positions_np, dtype=wp.float32, device=sim.device)
            indices_numpy = np.arange(0, self.cloths.count)
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device=sim.device)
            self.cloths.set_velocities(velocities, indices)
            self.cloths.set_positions(positions, indices)

            # set friction to very low for first and last cloth
            friction = self.particle_materials.get_friction()
            friction_np = friction.numpy()
            friction_np[0] = 0.002
            friction_np[-1] = 0.002

            friction = wp.from_numpy(friction_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.particle_materials.set_friction(friction, indices)

        elif stepno == 10:
            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)
            self.test_case.assertTrue((positions_np[0,:,1] > positions_np[1,:,1]).all(), "cloth with less friction sliding further.")
            self.test_case.assertTrue((positions_np[15,:,1] > positions_np[14,:,1]).all(), "cloth with less friction sliding further.")

            self.finish()


class TestParticleMaterialDamping(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_materials = sim.create_particle_material_view("/envs/*/particleMaterial")
        particle_cloths = sim.create_particle_cloth_view("/envs/*/particles")
        self.check_particle_material_view(particle_materials, self.num_envs)
        self.check_particle_cloth_view(particle_cloths, self.num_envs)
        self.particle_materials = particle_materials
        self.cloths = particle_cloths

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            # set damping to high for first and last cloth
            damping = self.particle_materials.get_damping()
            damping_np = damping.numpy()
            damping_np[0] = 10.0
            damping_np[-1] = 10.0

            damping = wp.from_numpy(damping_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.particle_materials.set_damping(damping, indices)

        elif stepno == 10:
            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:,:,2]

            self.test_case.assertTrue((positions_np[0,:] > positions_np[1,:]).all(), "cloth with more damping falling slower.")
            self.test_case.assertTrue((positions_np[15,:] > positions_np[14,:]).all(), "cloth with more damping falling slower.")

            self.finish()


class TestParticleMaterialGravityScale(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_materials = sim.create_particle_material_view("/envs/*/particleMaterial")
        particle_cloths = sim.create_particle_cloth_view("/envs/*/particles")
        self.check_particle_material_view(particle_materials, self.num_envs)
        self.check_particle_cloth_view(particle_cloths, self.num_envs)
        self.particle_materials = particle_materials
        self.cloths = particle_cloths

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            # set gravity scale to low for first and last cloth
            gravity_scale = self.particle_materials.get_gravity_scale()
            gravity_scale_np = gravity_scale.numpy()
            gravity_scale_np[0] = 0.1
            gravity_scale_np[-1] = 0.1

            gravity_scale = wp.from_numpy(gravity_scale_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.cloths.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.particle_materials.set_gravity_scale(gravity_scale, indices)

        elif stepno == 10:
            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:,:,2]

            self.test_case.assertTrue((positions_np[0,:] > positions_np[1,:]).all(), "cloth with smaller gravity scale falling slower.")
            self.test_case.assertTrue((positions_np[15,:] > positions_np[14,:]).all(), "cloth with smaller gravity scale falling slower.")

            self.finish()


class TestParticleMaterialLift(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_materials = sim.create_particle_material_view("/envs/*/particleMaterial")
        particle_cloths = sim.create_particle_cloth_view("/envs/*/particles")
        particle_systems = sim.create_particle_system_view("/envs/*/particleSystem")
        self.check_particle_material_view(particle_materials, self.num_envs)
        self.check_particle_cloth_view(particle_cloths, self.num_envs)
        self.check_particle_system_view(particle_systems, self.num_envs)
        self.particle_materials = particle_materials
        self.cloths = particle_cloths
        self.particle_systems = particle_systems

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            # set lift to high first and last cloth
            lift = self.particle_materials.get_lift()
            lift_np = lift.numpy()
            lift_np[0] = 100.0
            lift_np[1:-1] = 0.0
            lift_np[-1] = 100.0

            # set some wind to get a more pronounced effect of lift
            wind = self.particle_systems.get_wind()
            wind_np = wind.numpy().reshape(self.cloths.count, 3)
            wind_np[:, 2] = 50.0
            wind = wp.from_numpy(wind_np, dtype=wp.float32, device="cpu")

            lift = wp.from_numpy(lift_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.arange(0, self.cloths.count)
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.particle_materials.set_lift(lift, indices)
            self.particle_systems.set_wind(wind, indices)

        elif stepno == 20:
            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:,:,2]

            self.test_case.assertTrue((positions_np[0,:] > positions_np[1,:]).all(), "cloth with more lift falling slower.")
            self.test_case.assertTrue((positions_np[15,:] > positions_np[14,:]).all(), "cloth with more lift falling slower.")

            self.finish()


class TestParticleMaterialDrag(GridTestBase):

    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        particle_path = self.env_template_path.AppendChild("particles")
        self.initial_positions = self.create_particle_cloth(particle_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        particle_materials = sim.create_particle_material_view("/envs/*/particleMaterial")
        particle_cloths = sim.create_particle_cloth_view("/envs/*/particles")
        particle_systems = sim.create_particle_system_view("/envs/*/particleSystem")
        self.check_particle_material_view(particle_materials, self.num_envs)
        self.check_particle_cloth_view(particle_cloths, self.num_envs)
        self.check_particle_system_view(particle_systems, self.num_envs)
        self.particle_materials = particle_materials
        self.cloths = particle_cloths
        self.particle_systems = particle_systems

    def on_physics_step(self, sim, stepno, dt):

        if stepno == 1:
            # set drag to high first and last cloth
            drag = self.particle_materials.get_drag()
            drag_np = drag.numpy()
            drag_np[0] = 100.0
            drag_np[1:-1] = 0.0
            drag_np[-1] = 100.0

            # set some wind to get a more pronounced effect of lift
            wind = self.particle_systems.get_wind()
            wind_np = wind.numpy().reshape(self.cloths.count, 3)
            wind_np[:, 1] = 20.0
            wind = wp.from_numpy(wind_np, dtype=wp.float32, device="cpu")

            drag = wp.from_numpy(drag_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.arange(0, self.cloths.count)
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.particle_materials.set_drag(drag, indices)
            self.particle_systems.set_wind(wind, indices)

        elif stepno == 20:
            positions = self.cloths.get_positions()
            positions_np = positions.numpy().reshape(self.cloths.count, self.cloths.max_particles_per_cloth, 3)[:,:,2]

            self.test_case.assertTrue((positions_np[0,:] > positions_np[1,:]).all(), "cloth with more drag falling slower.")
            self.test_case.assertTrue((positions_np[15,:] > positions_np[14,:]).all(), "cloth with more drag falling slower.")

            self.finish()

class TestUsdPrimDeletion(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 4.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 1
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.1, 0.1, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.2)
        self.actor_path= actor_path

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)


    def on_start(self, sim):
        rigid_body_view = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(rigid_body_view, self.num_envs)
        self.rigid_body_view = rigid_body_view

        articulation_view = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(articulation_view, self.num_envs, 9, 8, True)
        self.articulation_view = articulation_view

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            self.test_case.assertTrue(sim.is_valid, "simulation view is valid.")
        if stepno == 2:
            prim = self.stage.GetPrimAtPath("/envs/env2/ball")
            prim.SetActive(False)
            self.test_case.assertTrue(not sim.is_valid, "simulation view should have been invalidated.")
        elif stepno == 10:
            self.finish()



class TestSimViewInvalidate(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 4.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 1
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.1, 0.1, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.2)
        self.actor_path= actor_path

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        rigid_body_view = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(rigid_body_view, self.num_envs)
        self.rigid_body_view = rigid_body_view

        articulation_view = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(articulation_view, self.num_envs, 9, 8, True)
        self.articulation_view = articulation_view

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            self.test_case.assertTrue(sim.is_valid, "simulation view is valid.")
        if stepno == 2:
            sim.invalidate()
            self.test_case.assertTrue(not sim.is_valid, "simulation view was invalidated.")
            # no warning should be thrown anymore 
            self.stage.RemovePrim("/envs/env2/ball")
        if stepno == 3:
            self.finish()


class TestSoftBodyMultipleView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        # print("create /envs/env[5-8]/softBody")
        self.soft_bodies_1 = sim.create_soft_body_view("/envs/env[5-8]/softBody")
        self.check_soft_body_view(self.soft_bodies_1, 4)
        
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            # print("create /envs/*/softBody")
            self.soft_bodies_2 = sim.create_soft_body_view("/envs/*/softBody")
            self.check_soft_body_view(self.soft_bodies_2, self.num_envs)
            # create and destroy a view
            # print("create /envs/env[3-7]/softBody")
            soft_bodies_3 = sim.create_soft_body_view("/envs/env[3-7]/softBody")
            self.check_soft_body_view(soft_bodies_3, 5)

        if stepno == 50:
            # create and destroy a view
            # print("create /envs/env[3-7]/softBody")
            self.soft_bodies_3 = sim.create_soft_body_view("/envs/env[3-7]/softBody")
            self.check_soft_body_view(self.soft_bodies_3, 5)
            pos_1 = self.soft_bodies_1.get_sim_nodal_positions().numpy().reshape(self.soft_bodies_1.count, self.soft_bodies_1.max_sim_vertices_per_body, 3)
            pos_2 = self.soft_bodies_2.get_sim_nodal_positions().numpy().reshape(self.soft_bodies_2.count, self.soft_bodies_2.max_sim_vertices_per_body, 3)
            pos_3 = self.soft_bodies_3.get_sim_nodal_positions().numpy().reshape(self.soft_bodies_3.count, self.soft_bodies_3.max_sim_vertices_per_body, 3)
            self.test_case.assertTrue(np.allclose(pos_2[3:8], pos_3, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.test_case.assertTrue(np.allclose(pos_2[5:9], pos_1, rtol=1e-01, atol=1e-02), "similar values regardless of the view")

            self.finish()

class TestSoftBodyElementIndices(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)
        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)
        self.num_env=4
        self.num_ele=2
        self.deformable_body = PhysxSchema.PhysxDeformableBodyAPI(self.stage.GetPrimAtPath(deformable_path))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.sim_element_indices=self.soft_bodies.get_sim_element_indices().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 4)
        # print("sim_element_indices:\n", self.sim_element_indices[0:self.num_env,0:self.num_ele])
        self.element_indices=self.soft_bodies.get_element_indices().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 4)
        # print("element_indices:\n", self.element_indices[0:self.num_env,0:self.num_ele])
    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            sim_element_indices=self.soft_bodies.get_sim_element_indices().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 4)
            element_indices=self.soft_bodies.get_element_indices().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 4)
            self.test_case.assertTrue((sim_element_indices == self.sim_element_indices).all(), "simulation mesh indices don't cahnge with time")
            self.test_case.assertTrue((element_indices == self.element_indices).all(), "collision mesh indices don't cahnge with time")
            self.finish()

class TestSoftBodyElementRestValues(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(11, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)
        self.num_env=4
        self.num_dofs=1

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            rest_pose=self.soft_bodies.get_element_rest_poses().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 9)
            # print("rest_pose:\n", rest_pose[0:self.num_env,0:self.num_dofs])
            
            rest_pose=self.soft_bodies.get_sim_element_rest_poses().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 9)
            # print("sim_rest_pose:\n", rest_pose[0:self.num_env,0:self.num_dofs] )

            def_grad=self.soft_bodies.get_element_deformation_gradients().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 9)
            # print("deformation_grad:\n", def_grad[0:self.num_env,0:self.num_dofs])
            expected_def_grad = np.zeros((self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 9))
            expected_def_grad[:,:,[0,4,8]] = 1
            self.test_case.assertTrue(np.allclose(def_grad, expected_def_grad, rtol=1e-01, atol=1e-02), "identity deformation gradient")

            def_grad=self.soft_bodies.get_sim_element_deformation_gradients().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 9)
            # print("sim_deformation_grad:\n", def_grad[0:self.num_env,0:self.num_dofs])
            expected_def_grad = np.zeros((self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 9))
            expected_def_grad[:,:,[0,4,8]] = 1
            self.test_case.assertTrue(np.allclose(def_grad, expected_def_grad, rtol=1e-01, atol=1e-02), "identity deformation gradient for simulation mesh")

            rotations=self.soft_bodies.get_element_rotations().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 4) 
            expected_rotations = np.zeros((self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 4))
            # print("rotations:\n", rotations[0:self.num_env,0:self.num_dofs])
            expected_rotations[:,:,[3]] = 1
            self.test_case.assertTrue(np.allclose(rotations, expected_rotations, rtol=1e-01, atol=1e-02), "identity quaternions")

            rotations=self.soft_bodies.get_sim_element_rotations().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 4) 
            expected_rotations = np.zeros((self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 4))
            # print("rotations:\n", rotations[0:self.num_env,0:self.num_dofs])
            expected_rotations[:,:,[3]] = 1
            self.test_case.assertTrue(np.allclose(rotations, expected_rotations, rtol=1e-01, atol=1e-02), "identity quaternions for simulation mesh")

            stresses=self.soft_bodies.get_element_stresses().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 9)
            # print("stresses:\n", stresses[0:self.num_env,0:self.num_dofs])
            self.test_case.assertTrue(np.allclose(stresses, stresses * 0 , atol=10), "almost zero stresses")

            stresses=self.soft_bodies.get_sim_element_stresses().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_elements_per_body, 9)
            # print("stresses:\n", stresses[0:self.num_env,0:self.num_dofs])
            self.test_case.assertTrue(np.allclose(stresses, stresses * 0 , atol=10), "almost zero stresses for simulation mesh")

            self.finish()

class TestSoftBodySimPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)
        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path, resolution=3)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.wp_all_indices = wp_utils.arange(self.soft_bodies.count, device=sim.device)
        # not used just to make see everything works though
        positions = self.soft_bodies.get_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_vertices_per_body, 3)
        
    def on_physics_step(self, sim, stepno, dt):
        delta= 2.0
        middle_env=self.soft_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno ==100:
            self.positions = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            set_vals=self.positions + np.array([0,0,delta])
            wp_pos = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.soft_bodies.set_sim_nodal_positions(wp_pos, self.wp_all_indices)
            self.soft_bodies.set_sim_nodal_positions(wp_pos, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 101:
            self.set_positions  = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
        if stepno == 150:
            # self.test_case.assertTrue(np.allclose(self.positions +  np.array([0,0,delta]), self.set_positions, rtol=1e-01, atol=1e-02), "set simulation positins works correctly.")
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1]+  np.array([0,0,delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=1e-01), "set_sim_nodal_positions works correctly.")            
            self.finish()

class TestSoftBodySimVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.wp_all_indices = wp_utils.arange(self.soft_bodies.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        middle_env=self.soft_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno == 150:
            self.settled_velocities = self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            set_vals=self.settled_velocities +   np.array([0,0,10.0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.soft_bodies.set_sim_nodal_velocities(wp_vel, self.wp_all_indices )
            self.soft_bodies.set_sim_nodal_velocities(wp_vel, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 151:
            self.velocities = self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
        if stepno==200:
            self.test_case.assertTrue((self.velocities[:middle_env-1,:,-1] > self.settled_velocities[:middle_env-1,:,-1] ).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:,:,-1], self.settled_velocities[middle_env+1:,:,-1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()

class TestSoftBodyStagingBuffers(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)
        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.wp_all_indices = wp_utils.arange(self.soft_bodies.count, device=sim.device)
        middle_env=self.soft_bodies.count//2
        first_half= np.arange(0, middle_env)
        self.wp_half_indices = wp.from_numpy(first_half, dtype=wp.uint32)
    def on_physics_step(self, sim, stepno, dt):
        delta= 2.0
        if stepno ==100:
            self.positions = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            self.settled_velocities = self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            set_pos=self.positions + np.array([0,0,delta])
            set_vals=self.settled_velocities +   np.array([0,0,10.0])
            wp_pos = wp.from_numpy(set_pos, dtype=wp.float32)
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.soft_bodies.set_sim_nodal_positions(wp_pos, self.wp_all_indices)
            # self.soft_bodies.set_sim_nodal_velocities(wp_vel, self.wp_all_indices)
            self.soft_bodies.set_sim_nodal_velocities(wp_vel, self.wp_half_indices)
            self.soft_bodies.set_sim_nodal_positions(wp_pos, self.wp_half_indices)
        if stepno == 101:
            self.set_positions  = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            self.velocities = self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
        if stepno == 199:
            middle_env = self.soft_bodies.count//2
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1]+  np.array([0,0,delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=2e-01), "set simulation positins works correctly.")            
            self.test_case.assertTrue((self.velocities[:middle_env-1,:,-1] > self.settled_velocities[:middle_env-1,:,-1] ).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:,:,-1], self.settled_velocities[middle_env+1:,:,-1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()


class TestSoftBodyKinematicTargets(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(4, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path, resolution=5)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        self.materials = sim.create_soft_body_material_view("/envs/*/deformableBodyMaterial")
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.init_positions = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
        self.wp_all_indices = wp_utils.arange(self.soft_bodies.count, device=sim.device)
        self.top_surface_indices = self.init_positions[:,:,2]>1.2
        self.kinematic_targets = np.zeros((self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 4))
        self.kinematic_targets[:,:,0:3] = self.init_positions
        self.kinematic_targets[:,:,-1] = 1 - self.top_surface_indices
        kinematic_targets=wp.from_numpy(self.kinematic_targets, dtype=wp.float32)
        self.soft_bodies.set_sim_kinematic_targets(kinematic_targets, self.wp_all_indices)

    def on_physics_step(self, sim, stepno, dt):
        Amp = 0.1
        height = Amp * np.sin(2 * np.pi * stepno/200)
        self.current_targets = self.soft_bodies.get_sim_kinematic_targets().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 4)
        self.kinematic_targets = self.current_targets + np.array([0,0,height,0])
        wp_kinematic_targets = wp.from_numpy(self.kinematic_targets , dtype=wp.float32)
        self.soft_bodies.set_sim_kinematic_targets(wp_kinematic_targets, self.wp_all_indices)
        if stepno >= 200:
            kt = self.soft_bodies.get_sim_kinematic_targets().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 4)
            self.test_case.assertTrue((self.current_targets == kt).all(), "same set and get values.")
            positions = self.soft_bodies.get_sim_nodal_positions().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            combined_z = np.where(self.top_surface_indices, kt[...,2], positions[...,2])
            combined = positions
            combined[...,2]=combined_z            
            self.test_case.assertTrue(np.allclose(combined, positions, rtol=1e-03, atol=1e-04), "kinematic nodes match the current nodes.")
            self.finish()

class TestSoftBodyMaterialElasticity(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        soft_body_materials = sim.create_soft_body_material_view("/envs/*/deformableBodyMaterial")

        self.check_soft_body_material_view(soft_body_materials, self.num_envs)
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.materials = soft_body_materials

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            E = self.materials.get_youngs_modulus()
            E_np = E.numpy().flatten()
            E_default= E_np[0]
            E_np[0] = 50 * E_default
            E_np[-1] = 0.1 * E_default
            E = wp.from_numpy(E_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.materials.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            set_E = self.materials.set_youngs_modulus(E, indices)
            expected_E = np.ones(self.soft_bodies.count) * E_default
            expected_E[0] =  E_np[0] 
            expected_E[-1] =  E_np[-1]
            self.test_case.assertTrue(np.allclose(expected_E, E_np, rtol=1e-3), "similar get and set values")

        elif stepno == 40:                  
            deformation=self.soft_bodies.get_element_deformation_gradients().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_elements_per_body, 9)
            mean_body_deformation_mag=np.mean(np.mean(deformation*deformation, axis=2), axis=1)
            self.test_case.assertTrue((mean_body_deformation_mag[0] < mean_body_deformation_mag[1:-1]).all(), "smaller deformations for stiffer material.")
            self.test_case.assertTrue((mean_body_deformation_mag[-1] > mean_body_deformation_mag[1:-1]).all(), "larger deformations for softer material.")
            self.finish()


class TestSoftBodyMaterialDamping(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path, resolution = 4)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        soft_body_materials = sim.create_soft_body_material_view("/envs/*/deformableBodyMaterial")

        self.check_soft_body_material_view(soft_body_materials, self.num_envs)
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.materials = soft_body_materials

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            d = self.materials.get_damping()
            d_np = d.numpy()
            d_np[-1] = 1
            d = wp.from_numpy(d_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.materials.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.materials.set_damping(d, indices)

        elif stepno == 40:                  
            vel=self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            mean_body_magnitude_mag=np.mean(np.mean(np.abs(vel), axis=2), axis=1)
            self.test_case.assertTrue((mean_body_magnitude_mag[-1] < mean_body_magnitude_mag[:-1]).all(), "smaller overall velocity for more damped material.")
            self.finish()


class TestSoftBodyMaterialDynamicFriction(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("softBody")
        self.create_soft_body(deformable_path, resolution = 4)

        # ensure the surface material paramter for the friction test
        self.ground_material_path = "/groundMaterial"
        UsdShade.Material.Define(self.stage, self.ground_material_path)
        ground_material = UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self.ground_material_path))
        physicsUtils.add_physics_material_to_prim(self.stage, self.stage.GetPrimAtPath("/groundPlane"), self.ground_material_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        soft_bodies = sim.create_soft_body_view("/envs/*/softBody")
        soft_body_materials = sim.create_soft_body_material_view("/envs/*/deformableBodyMaterial")

        self.check_soft_body_material_view(soft_body_materials, self.num_envs)
        self.check_soft_body_view(soft_bodies, self.num_envs)
        self.soft_bodies = soft_bodies
        self.materials = soft_body_materials

        # enforce 'max' friction combine mode to apply the soft body friction coefficient between the soft body and rigid body.
        ground_material_prim = self.stage.GetPrimAtPath(self.ground_material_path)
        physxMaterialAPI = PhysxSchema.PhysxMaterialAPI.Apply(ground_material_prim)
        physxMaterialAPI.CreateFrictionCombineModeAttr().Set("max")

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            d = self.materials.get_dynamic_friction()
            d_np = d.numpy()
            d_np[-1] = 1
            d = wp.from_numpy(d_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.materials.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.materials.set_dynamic_friction(d, indices)
        elif stepno == 100:
            vel=self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            set_vals=vel + np.array([0,10,0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            wp_all_indices = wp_utils.arange(self.soft_bodies.count, device=sim.device)
            self.soft_bodies.set_sim_nodal_velocities(wp_vel, wp_all_indices)
        elif stepno == 105:
            vel=self.soft_bodies.get_sim_nodal_velocities().numpy().reshape(self.soft_bodies.count, self.soft_bodies.max_sim_vertices_per_body, 3)
            mean_body_magnitude_mag=np.mean(np.mean(np.abs(vel), axis=1), axis=1)
            self.test_case.assertTrue((mean_body_magnitude_mag[-1] < mean_body_magnitude_mag[:-1]).all(), "smaller overall velocity for larger frictions.")
            self.finish()


class TestVolumeDeformableBodyMultipleView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        # print("create /envs/env[5-8]/deformableBody")
        self.deformable_bodies_1 = sim.create_volume_deformable_body_view("/envs/env[5-8]/deformableBody")
        self.check_deformable_body_view(self.deformable_bodies_1, 4)
        
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            # print("create /envs/*/deformableBody")
            self.deformable_bodies_2 = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_2, self.num_envs)
            # create and destroy a view
            # print("create /envs/env[3-7]/deformableBody")
            deformable_bodies_3 = sim.create_volume_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(deformable_bodies_3, 5)

        if stepno == 50:
            # create and destroy a view
            # print("create /envs/env[3-7]/deformableBody")
            self.deformable_bodies_3 = sim.create_volume_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_3, 5)
            pos_1 = self.deformable_bodies_1.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_1.count, self.deformable_bodies_1.max_simulation_nodes_per_body, 3)
            pos_2 = self.deformable_bodies_2.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_2.count, self.deformable_bodies_2.max_simulation_nodes_per_body, 3)
            pos_3 = self.deformable_bodies_3.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_3.count, self.deformable_bodies_3.max_simulation_nodes_per_body, 3)
            self.test_case.assertTrue(np.allclose(pos_2[3:8], pos_3, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.test_case.assertTrue(np.allclose(pos_2[5:9], pos_1, rtol=1e-01, atol=1e-02), "similar values regardless of the view")

            self.finish()

class TestSurfaceDeformableBodyMultipleView(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        # print("create /envs/env[5-8]/deformableBody")
        self.deformable_bodies_1 = sim.create_surface_deformable_body_view("/envs/env[5-8]/deformableBody")
        self.check_deformable_body_view(self.deformable_bodies_1, 4)
        
    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            # print("create /envs/*/deformableBody")
            self.deformable_bodies_2 = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_2, self.num_envs)
            # create and destroy a view
            # print("create /envs/env[3-7]/deformableBody")
            deformable_bodies_3 = sim.create_surface_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(deformable_bodies_3, 5)

        if stepno == 50:
            # create and destroy a view
            # print("create /envs/env[3-7]/deformableBody")
            self.deformable_bodies_3 = sim.create_surface_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_3, 5)
            pos_1 = self.deformable_bodies_1.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_1.count, self.deformable_bodies_1.max_simulation_nodes_per_body, 3)
            pos_2 = self.deformable_bodies_2.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_2.count, self.deformable_bodies_2.max_simulation_nodes_per_body, 3)
            pos_3 = self.deformable_bodies_3.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_3.count, self.deformable_bodies_3.max_simulation_nodes_per_body, 3)
            self.test_case.assertTrue(np.allclose(pos_2[3:8], pos_3, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.test_case.assertTrue(np.allclose(pos_2[5:9], pos_1, rtol=1e-01, atol=1e-02), "similar values regardless of the view")

            self.finish()            

class TestVolumeDeformableBodyElementIndices(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        self.num_env=4
        self.num_ele=2

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(self.volume_deformable_bodies, self.num_envs)

        self.simulation_element_indices=self.volume_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_elements_per_body, 4)
        self.collision_element_indices=self.volume_deformable_bodies.get_collision_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_elements_per_body, 4)

        self.sim_mesh_paths = self.volume_deformable_bodies.simulation_mesh_prim_paths
        self.coll_mesh_paths = self.volume_deformable_bodies.collision_mesh_prim_paths

        self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.sim_mesh_paths, self.simulation_element_indices))
        self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.coll_mesh_paths, self.collision_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            simulation_element_indices=self.volume_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_elements_per_body, 4)
            collision_element_indices=self.volume_deformable_bodies.get_collision_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_elements_per_body, 4)
            self.test_case.assertTrue((simulation_element_indices == self.simulation_element_indices).all(), "simulation mesh indices don't change with time")
            self.test_case.assertTrue((collision_element_indices == self.collision_element_indices).all(), "collision mesh indices don't change with time")
            self.finish()

class TestSurfaceDeformableBodyElementIndices(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        self.num_env=4
        self.num_ele=2

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(self.surface_deformable_bodies, self.num_envs)
        self.simulation_element_indices=self.surface_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_elements_per_body, 3)
        self.sim_mesh_paths = self.surface_deformable_bodies.simulation_mesh_prim_paths

        self.test_case.assertTrue(self.check_tri_element_indices(self.stage, self.sim_mesh_paths, self.simulation_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            simulation_element_indices=self.surface_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_elements_per_body, 3)
            self.test_case.assertTrue((simulation_element_indices == self.simulation_element_indices).all(), "simulation mesh indices don't change with time")
            self.finish()

class TestDeformableBodyRest(GridTestBase):
    def __init__(self, test_case, device_params, test_surface):
        self.test_surface = test_surface
        self.num_nodes_per_element = 3 if test_surface else 4

        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        if self.test_surface:
            self.create_surface_deformable_body(deformable_path)
        else:
            self.create_volume_deformable_body(deformable_path)
        
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        self.num_env=4
        self.num_ele=2

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        if self.test_surface:
            self.bodies_view = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        else:
            self.bodies_view = sim.create_volume_deformable_body_view("/envs/*/deformableBody")

        self.check_deformable_body_view(self.bodies_view, self.num_envs)

        self.rest_element_indices=self.bodies_view.get_rest_element_indices().numpy().reshape(self.bodies_view.count, self.bodies_view.max_simulation_elements_per_body, self.num_nodes_per_element)
        self.rest_nodal_positions=self.bodies_view.get_rest_nodal_positions().numpy().reshape(self.bodies_view.count, self.bodies_view.max_rest_nodes_per_body, 3)
        self.sim_mesh_paths = self.bodies_view.simulation_mesh_prim_paths

        # this assumes identical topology for sim mesh and rest shape
        if self.test_surface:
            self.test_case.assertTrue(self.check_tri_element_indices(self.stage, self.sim_mesh_paths, self.rest_element_indices))
        else:
            self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.sim_mesh_paths, self.rest_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            rest_element_indices=self.bodies_view.get_rest_element_indices().numpy().reshape(self.bodies_view.count, self.bodies_view.max_simulation_elements_per_body, self.num_nodes_per_element)
            rest_nodal_positions=self.bodies_view.get_rest_nodal_positions().numpy().reshape(self.bodies_view.count, self.bodies_view.max_rest_nodes_per_body, 3)
            self.test_case.assertTrue((rest_element_indices == self.rest_element_indices).all(), "rest shape indices don't change with time")
            self.test_case.assertTrue((rest_nodal_positions == self.rest_nodal_positions).all(), "rest shape positions don't change with time")
            self.finish()
            rest_element_indices=self.bodies_view.get_rest_element_indices().numpy().reshape(self.bodies_view.count, self.bodies_view.max_simulation_elements_per_body, self.num_nodes_per_element)
            rest_nodal_positions=self.bodies_view.get_rest_nodal_positions().numpy().reshape(self.bodies_view.count, self.bodies_view.max_rest_nodes_per_body, 3)
            self.test_case.assertTrue((rest_element_indices == self.rest_element_indices).all(), "rest shape indices don't change with time")
            self.test_case.assertTrue((rest_nodal_positions == self.rest_nodal_positions).all(), "rest shape positions don't change with time")
            self.finish()


class TestVolumeDeformableBodySimulationPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        # not used just to make see everything works though
        positions = self.volume_deformable_bodies.get_collision_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_nodes_per_body, 3)
        
    def on_physics_step(self, sim, stepno, dt):
        delta= 2.0
        middle_env=self.volume_deformable_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno ==100:
            self.positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals=self.positions + np.array([0,0,delta])
            wp_pos = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, self.wp_all_indices)
            self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 101:
            self.set_positions  = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 150:
            # self.test_case.assertTrue(np.allclose(self.positions +  np.array([0,0,delta]), self.set_positions, rtol=1e-01, atol=1e-02), "set simulation positins works correctly.")
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1]+  np.array([0,0,delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=1e-01), "set_simulation_nodal_positions works correctly.")            
            self.finish()

class TestSurfaceDeformableBodySimulationPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.surface_deformable_bodies = surface_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.surface_deformable_bodies.count, device=sim.device)
        # not used just to make see everything works though
        positions = self.surface_deformable_bodies.get_collision_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_collision_nodes_per_body, 3)
        
    def on_physics_step(self, sim, stepno, dt):
        delta= 2.0
        middle_env=self.surface_deformable_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno ==100:
            self.positions = self.surface_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals=self.positions + np.array([0,0,delta])
            wp_pos = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, self.wp_all_indices)
            self.surface_deformable_bodies.set_simulation_nodal_positions(wp_pos, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 101:
            self.set_positions  = self.surface_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 150:
            # self.test_case.assertTrue(np.allclose(self.positions +  np.array([0,0,delta]), self.set_positions, rtol=1e-01, atol=1e-02), "set simulation positins works correctly.")
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1]+  np.array([0,0,delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=1e-01), "set_simulation_nodal_positions works correctly.")            
            self.finish()

class TestVolumeDeformableBodySimulationVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        middle_env=self.volume_deformable_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno == 150:
            self.settled_velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals=self.settled_velocities +   np.array([0,0,10.0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.volume_deformable_bodies.set_sim_nodal_velocities(wp_vel, self.wp_all_indices )
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 151:
            self.velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno==200:
            self.test_case.assertTrue((self.velocities[:middle_env-1,:,-1] > self.settled_velocities[:middle_env-1,:,-1] ).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:,:,-1], self.settled_velocities[middle_env+1:,:,-1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()

class TestSurfaceDeformableBodySimulationVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.surface_deformable_bodies = surface_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.surface_deformable_bodies.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        middle_env=self.surface_deformable_bodies.count//2
        first_half= np.arange(0, middle_env)
        if stepno == 150:
            self.settled_velocities = self.surface_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals=self.settled_velocities +   np.array([0,0,10.0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.surface_deformable_bodies.set_sim_nodal_velocities(wp_vel, self.wp_all_indices )
            self.surface_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 151:
            self.velocities = self.surface_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno==200:
            self.test_case.assertTrue((self.velocities[:middle_env-1,:,-1] > self.settled_velocities[:middle_env-1,:,-1] ).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:,:,-1], self.settled_velocities[middle_env+1:,:,-1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()

class TestVolumeDeformableBodySimulationKinematicTargets(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(4, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        #self.materials = sim.create_deformable_material_view("/envs/*/deformableMaterial")
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.init_positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        self.top_surface_indices = self.init_positions[:,:,2]>1.2
        self.kinematic_targets = np.zeros((self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4))
        self.kinematic_targets[:,:,0:3] = self.init_positions
        self.kinematic_targets[:,:,-1] = 1 - self.top_surface_indices
        kinematic_targets=wp.from_numpy(self.kinematic_targets, dtype=wp.float32)
        self.volume_deformable_bodies.set_simulation_nodal_kinematic_targets(kinematic_targets, self.wp_all_indices)

    def on_physics_step(self, sim, stepno, dt):
        Amp = 0.1
        height = Amp * np.sin(2 * np.pi * stepno/200)
        self.current_targets = self.volume_deformable_bodies.get_simulation_nodal_kinematic_targets().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4)
        self.kinematic_targets = self.current_targets + np.array([0,0,height,0])
        wp_kinematic_targets = wp.from_numpy(self.kinematic_targets , dtype=wp.float32)
        self.volume_deformable_bodies.set_simulation_nodal_kinematic_targets(wp_kinematic_targets, self.wp_all_indices)
        if stepno >= 200:
            kt = self.volume_deformable_bodies.get_simulation_nodal_kinematic_targets().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4)
            self.test_case.assertTrue((self.current_targets == kt).all(), "same set and get values.")
            positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            combined_z = np.where(self.top_surface_indices, kt[...,2], positions[...,2])
            combined = positions
            combined[...,2]=combined_z            
            self.test_case.assertTrue(np.allclose(combined, positions, rtol=1e-03, atol=1e-04), "kinematic nodes match the current nodes.")
            self.finish()

class TestVolumeDeformableBodyStagingBuffers(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        middle_env=self.volume_deformable_bodies.count//2
        first_half= np.arange(0, middle_env)
        self.wp_half_indices = wp.from_numpy(first_half, dtype=wp.uint32)

    def on_physics_step(self, sim, stepno, dt):
        delta= 2.0
        if stepno ==100:
            self.positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            self.settled_velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_pos=self.positions + np.array([0,0,delta])
            set_vals=self.settled_velocities +   np.array([0,0,10.0])
            wp_pos = wp.from_numpy(set_pos, dtype=wp.float32)
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            # self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, self.wp_all_indices)
            # self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, self.wp_all_indices)
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, self.wp_half_indices)
            self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, self.wp_half_indices)
        if stepno == 101:
            self.set_positions  = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            self.velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 199:
            middle_env = self.volume_deformable_bodies.count//2
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1]+  np.array([0,0,delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=2e-01), "set simulation positins works correctly.")            
            self.test_case.assertTrue((self.velocities[:middle_env-1,:,-1] > self.settled_velocities[:middle_env-1,:,-1] ).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:,:,-1], self.settled_velocities[middle_env+1:,:,-1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()


class TestDeformableMaterialYoungsModulus(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(8, 3.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        volume_deformable_path = self.env_template_path.AppendChild("volumeDeformableBody")
        self.create_volume_deformable_body(volume_deformable_path,
            translate=Gf.Vec3d(0.0, -0.7, 0.0)
        )

        surface_deformable_path = self.env_template_path.AppendChild("surfaceDeformableBody")
        self.create_surface_deformable_body(surface_deformable_path,
            resolution=20,
            translate=Gf.Vec3d(0.0, 0.7, 2.0),
            rotate=Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -90.0),
            attach=True
        )

        # add ground plane
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/volumeDeformableBody")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/surfaceDeformableBody")
        volume_deformable_materials = sim.create_deformable_material_view("/envs/*/volumeDeformableMaterial")
        surface_deformable_materials = sim.create_deformable_material_view("/envs/*/surfaceDeformableMaterial")

        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.check_deformable_material_view(volume_deformable_materials, self.num_envs)
        self.check_deformable_material_view(surface_deformable_materials, self.num_envs)
    
        self.volume_deformable_bodies = volume_deformable_bodies
        self.surface_deformable_bodies = surface_deformable_bodies
        self.volume_materials = volume_deformable_materials
        self.surface_materials = surface_deformable_materials

    def setup_elasticity(self, material_view, scale):
        E = material_view.get_youngs_modulus()
        E_np = E.numpy().flatten()
        E_default = E_np[0]*scale
        E_np[0] = 10 * E_default
        E_np[1:-1] = 0.2 * E_default
        E_np[-1] = 0.1 * E_default
        E = wp.from_numpy(E_np, dtype=wp.float32, device="cpu")
        v_indices_np = np.arange(material_view.count)
        v_indices = wp.from_numpy(v_indices_np, dtype=wp.uint32, device="cpu")
        material_view.set_youngs_modulus(E, v_indices)
        E_check = material_view.get_youngs_modulus()
        E_check_np = E_check.numpy().flatten()
        self.test_case.assertTrue(np.allclose(E_check_np, E_np, rtol=1e-7), "similar get and set values")

    @staticmethod
    def get_sim_position_ranges(deformable_body_view):
        positions = deformable_body_view.get_simulation_nodal_positions().numpy().reshape(
            deformable_body_view.count, 
            deformable_body_view.max_simulation_nodes_per_body, 
            3
        )
        return [Gf.Range3d(Gf.Vec3d(*pts.min(0).tolist()), Gf.Vec3d(*pts.max(0).tolist())) for pts in positions]

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            self.setup_elasticity(self.volume_materials, 1.0)
            self.setup_elasticity(self.surface_materials, 0.05)

        elif stepno == 25:
            volume_ranges = self.get_sim_position_ranges(self.volume_deformable_bodies)
            surface_ranges = self.get_sim_position_ranges(self.surface_deformable_bodies)

            # volumes compress more with lower youngs modulus
            self.test_case.assertTrue(volume_ranges[0].GetSize()[2] > volume_ranges[1].GetSize()[2])
            self.test_case.assertTrue(volume_ranges[1].GetSize()[2] > volume_ranges[-1].GetSize()[2])
            # surface stretch more with lower youngs modulus
            self.test_case.assertTrue(surface_ranges[0].GetSize()[2] < surface_ranges[1].GetSize()[2])
            self.test_case.assertTrue(surface_ranges[1].GetSize()[2] < surface_ranges[-1].GetSize()[2])

            self.finish()

class TestDeformableMaterialDynamicFriction(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground=False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set active stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        # set up env template
        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)
        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # ensure the surface material paramter for the friction test
        self.ground_material_path = "/groundMaterial"
        UsdShade.Material.Define(self.stage, self.ground_material_path)
        ground_material = UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self.ground_material_path))
        physicsUtils.add_physics_material_to_prim(self.stage, self.stage.GetPrimAtPath("/groundPlane"), self.ground_material_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        volume_deformable_materials = sim.create_deformable_material_view("/envs/*/volumeDeformableMaterial")

        self.check_deformable_material_view(volume_deformable_materials, self.num_envs)
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.volume_materials = volume_deformable_materials

        # enforce 'max' friction combine mode to apply the soft body friction coefficient between the soft body and rigid body.
        ground_material_prim = self.stage.GetPrimAtPath(self.ground_material_path)
        physxMaterialAPI = PhysxSchema.PhysxMaterialAPI.Apply(ground_material_prim)
        physxMaterialAPI.CreateFrictionCombineModeAttr().Set("max")

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            d = self.volume_materials.get_dynamic_friction()
            d_np = d.numpy()
            d_np[-1] = 1
            d = wp.from_numpy(d_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.volume_materials.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.volume_materials.set_dynamic_friction(d, indices)

        elif stepno == 100:
            vel=self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals=vel + np.array([0,10,0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp_all_indices)
        elif stepno == 105:
            vel=self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            mean_body_magnitude_mag=np.mean(np.mean(np.abs(vel), axis=1), axis=1)
            self.test_case.assertTrue((mean_body_magnitude_mag[-1] < mean_body_magnitude_mag[:-1]).all(), "smaller overall velocity for larger frictions.")
            self.finish()


class PhysicsTensorsTests(PhysicsBaseAsyncTestCase):

    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()

    @classmethod
    def setUpClass(cls):

        # Note: need to catch exceptions, otherwise we just see "ERROR" as output and Kit may hang
        try:
            cls.push_extensions()
        except:
            traceback.print_exc()

    @classmethod
    def tearDownClass(cls):
        # Note: need to catch exceptions, otherwise we just see "ERROR" as output and Kit may hang
        try:
            cls.pop_extensions()
        except:
            traceback.print_exc()

    @classmethod
    def push_extensions(self):
        #print("ENABLING EXTENSIONS")
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        self.saved_ext_enabled_tensor_api = ext_mgr.is_extension_enabled("omni.physx.tensors")
        if not self.saved_ext_enabled_tensor_api:
            ext_mgr.set_extension_enabled_immediate("omni.physx.tensors", True)
        self.saved_ext_enabled_fabric = ext_mgr.is_extension_enabled("omni.physx.fabric")
        if not self.saved_ext_enabled_fabric:
            ext_mgr.set_extension_enabled_immediate("omni.physx.fabric", True)
        self.saved_ext_enabled_warp = ext_mgr.is_extension_enabled("omni.warp")
        if not self.saved_ext_enabled_warp:
            ext_mgr.set_extension_enabled_immediate("omni.warp", True)

        # HACK: prevent using cached kernels, which might fail due to a bug if the kernel module previously failed to compile
        self.saved_wp_config_cache_kernels = wp.config.cache_kernels
        wp.config.cache_kernels = False


    @classmethod
    def pop_extensions(self):
        # HACK: restore kernel caching
        if hasattr(self, "saved_wp_config_cache_kernels"):
            wp.config.cache_kernels = self.saved_wp_config_cache_kernels

        #print("RESTORING EXTENSIONS")
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        if hasattr(self, "saved_ext_enabled_tensor_api") and not self.saved_ext_enabled_tensor_api:
            ext_mgr.set_extension_enabled_immediate("omni.physx.tensors", False)
        if hasattr(self, "saved_ext_enabled_fabric") and not self.saved_ext_enabled_fabric:
            ext_mgr.set_extension_enabled_immediate("omni.physx.fabric", False)
        if hasattr(self, "saved_ext_enabled_warp") and not self.saved_ext_enabled_warp:
            ext_mgr.set_extension_enabled_immediate("omni.warp", False)


    async def run_test(self, scenario: GridTestBase, attach_to_usd_context=_ATTACH_TO_USD_CONTEXT, save_file_name=None):
        stage = scenario.stage

        # !!!
        #save_file_name = "teststage.usda"

        # save stage if a filename was given
        if save_file_name is not None:
            with open(save_file_name, "w") as text_file:
                text_file.write(stage.GetRootLayer().ExportToString())

        # run in context or in memory
        if attach_to_usd_context:
            sync_params = SyncParams(sync_usd=True, sync_fabric=True, transforms_only=False)
            runner = RunnerInContextAsync(scenario, _FRONTEND, sync_params)

            #await runner.run(_WARM_START)

            exc_info = None
            try:
                #print("+++++++++++++++++++++++++++++++ starting")
                await runner.start(_WARM_START)
                #print("+++++++++++++++++++++++++++++++ simulating")
                await runner.simulate()
            except Exception as e:
                # save the exception info
                #traceback.print_exc()
                exc_info = sys.exc_info()
            finally:
                # make sure stop() is called to cleanup
                #print("+++++++++++++++++++++++++++++++ stopping")
                try:
                    await runner.stop()
                except:
                    # ignore exceptions in stop() if an earlier exception has occured
                    if exc_info is None:
                        exc_info = sys.exc_info()

            # re-raise the exception with original stack trace
            if exc_info is not None:
                raise exc_info[1].with_traceback(exc_info[2])

        else:
            sync_params = SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False)
            runner = RunnerInMemory(scenario, _FRONTEND, sync_params)

            #runner.run(_WARM_START)

            exc_info = None
            try:
                runner.start(_WARM_START)
                runner.simulate()
            except Exception as e:
                # save the exception info
                #traceback.print_exc()
                exc_info = sys.exc_info()
            finally:
                # make sure stop() is called to cleanup
                try:
                    runner.stop()
                except:
                    # ignore exceptions in stop() if an earlier exception has occured
                    if exc_info is None:
                        exc_info = sys.exc_info()

            # re-raise the exception with original stack trace
            if exc_info is not None:
                raise exc_info[1].with_traceback(exc_info[2])


    async def test_data_paths(self):
        asset_root = get_asset_root()
        self.assertTrue(os.path.isdir(asset_root))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "Ant.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartPole.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartPoleNoRail.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartRailNoPole.usda")))


    async def _test_warp_device(self, wp_device):
        self.assertTrue(wp.is_device_available(wp_device), f"Warp device {wp_device} is not available")
        with wp.ScopedDevice(wp_device):
            n = 10
            a = wp_utils.arange(n, device=wp_device)
            result = a.numpy().squeeze()
            expected = np.arange(n)
            #print("Result:", result)
            #print("Expected:", expected)
            self.assertTrue(np.array_equal(result, expected), "Warp arange() failed")

    async def test_warp_cpu(self):
        await self._test_warp_device("cpu")

    async def test_warp_gpu(self):
        await self._test_warp_device("cuda:0")


    async def test_simulation_view_gravity_cpu(self):
        await self.run_test(TestSimulationViewGravity(self, DeviceParams(False, False)))

    async def test_simulation_view_gravity_gpu(self):
        await self.run_test(TestSimulationViewGravity(self, DeviceParams(True, True)))

    async def test_articulation_view_cpu(self):
        await self.run_test(TestArticulationView(self, DeviceParams(False, False)))

    async def test_articulation_view_gpu(self):
        await self.run_test(TestArticulationView(self, DeviceParams(True, True)))


    async def test_humanoid_view_cpu(self):
        await self.run_test(TestHumanoidView(self, DeviceParams(False, False)))

    async def test_humanoid_view_gpu(self):
        await self.run_test(TestHumanoidView(self, DeviceParams(True, True)))


    async def test_articulation_dof_properties_cpu(self):
        await self.run_test(TestArticulationDofProperties(self, DeviceParams(False, False)))

    async def test_articulation_dof_properties_gpu(self):
        await self.run_test(TestArticulationDofProperties(self, DeviceParams(True, True)))


    async def test_articulation_special_cases_cpu(self):
        await self.run_test(TestArticulationSpecialCases(self, DeviceParams(False, False)))
    async def test_articulation_special_cases_gpu(self):
        await self.run_test(TestArticulationSpecialCases(self, DeviceParams(True, True)))


    async def test_articulation_view_duplicate_name_cpu(self):
        await self.run_test(TestArticulationViewDuplicateNames(self, DeviceParams(False, False)))


    async def test_articulation_joint_body_order_dof_limits_cc(self):
        await self.run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_limits_gc(self):
        await self.run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_limits_gg(self):
        await self.run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_dof_positions_cc(self):
        await self.run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_positions_gc(self):
        await self.run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_positions_gg(self):
        await self.run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_dof_velocities_cc(self):
        await self.run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_velocities_gc(self):
        await self.run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_velocities_gg(self):
        await self.run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_dof_position_targets_cc(self):
        await self.run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_position_targets_gc(self):
        await self.run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_position_targets_gg(self):
        await self.run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_dof_velocity_targets_cc(self):
        await self.run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_velocity_targets_gc(self):
        await self.run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_velocity_targets_gg(self):
        await self.run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_dof_forces_cc(self):
        await self.run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_dof_forces_gc(self):
        await self.run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_dof_forces_gg(self):
        await self.run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(True, True)))

    async def test_articulation_joint_body_order_link_forces_cc(self):
        await self.run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(False, False)))
    async def test_articulation_joint_body_order_link_forces_gc(self):
        await self.run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(True, False)))
    async def test_articulation_joint_body_order_link_forces_gg(self):
        await self.run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(True, True)))

    async def test_articulation_get_set_root_transforms_cc(self):
        await self.run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(False, False)))
    async def test_articulation_get_set_root_transforms_gc(self):
        await self.run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(True, False)))
    async def test_articulation_get_set_root_transforms_gg(self):
        await self.run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(True, True)))

    async def test_articulation_get_set_root_velocities_cc(self):
        await self.run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(False, False)))
    async def test_articulation_get_set_root_velocities_gc(self):
        await self.run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(True, False)))
    async def test_articulation_get_set_root_velocities_gg(self):
        await self.run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(True, True)))

    async def test_articulation_kinematic_update_cc(self):
        await self.run_test(TestArticulationKinematicUpdate(self, DeviceParams(False, False)))
    async def test_articulation_kinematic_update_gc(self):
        await self.run_test(TestArticulationKinematicUpdate(self, DeviceParams(True, False)))
    async def test_articulation_kinematic_update_gg(self):
        await self.run_test(TestArticulationKinematicUpdate(self, DeviceParams(True, True)))

    async def test_articulation_get_set_dof_positions_cc(self):
        await self.run_test(TestArticulationGetSetDofPositions(self, DeviceParams(False, False)))
    async def test_articulation_get_set_dof_positions_gc(self):
        await self.run_test(TestArticulationGetSetDofPositions(self, DeviceParams(True, False)))
    async def test_articulation_get_set_dof_positions_gg(self):
        await self.run_test(TestArticulationGetSetDofPositions(self, DeviceParams(True, True)))

    async def test_articulation_get_set_dof_position_target_cc(self):
        await self.run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(False, False)))
    async def test_articulation_get_set_dof_position_target_gc(self):
        await self.run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(True, False)))
    async def test_articulation_get_set_dof_position_target_gg(self):
        await self.run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(True, True)))

    async def test_articulation_get_set_dof_velocities_cc(self):
        await self.run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(False, False)))
    async def test_articulation_get_set_dof_velocities_gc(self):
        await self.run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(True, False)))
    async def test_articulation_get_set_dof_velocities_gg(self):
        await self.run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(True, True)))

    async def test_articulation_get_set_dof_velocity_targets_cc(self):
        await self.run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(False, False)))
    async def test_articulation_get_set_dof_velocity_targets_gc(self):
        await self.run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(True, False)))
    async def test_articulation_get_set_dof_velocity_targets_gg(self):
        await self.run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(True, True)))

    async def test_articulation_get_set_dof_actuation_forces_cc(self):
        await self.run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(False, False)))
    async def test_articulation_get_set_dof_actuation_forces_gc(self):
        await self.run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(True, False)))
    async def test_articulation_get_set_dof_actuation_forces_gg(self):
        await self.run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(True, True)))

    async def test_articulation_dof_drive_type_cc(self):
        await self.run_test(TestArticulationDofDriveType(self, DeviceParams(False, False)))
    async def test_articulation_dof_drive_type_gc(self):
        await self.run_test(TestArticulationDofDriveType(self, DeviceParams(True, False)))
    async def test_articulation_dof_drive_type_gg(self):
        await self.run_test(TestArticulationDofDriveType(self, DeviceParams(True, True)))

    async def test_articulation_get_set_applied_forces_global_cc(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(False, False), is_global=True))
    async def test_articulation_get_set_applied_forces_global_gc(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, False), is_global=True))
    async def test_articulation_get_set_applied_forces_global_gg(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, True), is_global=True))

    async def test_articulation_get_set_applied_forces_local_cc(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(False, False), is_global=False))
    async def test_articulation_get_set_applied_forces_local_gc(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, False), is_global=False))
    async def test_articulation_get_set_applied_forces_local_gg(self):
        await self.run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, True), is_global=False))

    async def test_articulation_get_set_link_gravities_cc(self):
        await self.run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(False, False)))
    async def test_articulation_get_set_link_gravities_gc(self):
        await self.run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(True, False)))
    async def test_articulation_get_set_link_gravities_gg(self):
        await self.run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(True, True)))

    async def test_rigid_body_get_set_applied_forces_cc(self):
        await self.run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(False, False)))
    async def test_rigid_body_get_set_applied_forces_gc(self):
        await self.run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(True, False)))
    async def test_rigid_body_get_set_applied_forces_gg(self):
        await self.run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(True, True)))

    async def test_rigid_body_get_set_transforms_cc(self):
        await self.run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(False, False)))
    async def test_rigid_body_get_set_transforms_gc(self):
        await self.run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(True, False)))
    async def test_rigid_body_get_set_transforms_gg(self):
        await self.run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(True, True)))

    async def test_rigid_body_get_set_velocities_cc(self):
        await self.run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(False, False)))
    async def test_rigid_body_get_set_velocities_gc(self):
        await self.run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(True, False)))
    async def test_rigid_body_get_set_velocities_gg(self):
        await self.run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(True, True)))

    async def test_articulation_root_transforms_cc(self):
        await self.run_test(TestArticulationRootTransforms(self, DeviceParams(False, False)))

    async def test_articulation_root_transforms_gc(self):
        await self.run_test(TestArticulationRootTransforms(self, DeviceParams(True, False)))

    async def test_articulation_root_transforms_gg(self):
        await self.run_test(TestArticulationRootTransforms(self, DeviceParams(True, True)))


    async def test_articulation_root_velocities_cc(self):
        await self.run_test(TestArticulationRootVelocities(self, DeviceParams(False, False)))

    async def test_articulation_root_velocities_gc(self):
        await self.run_test(TestArticulationRootVelocities(self, DeviceParams(True, False)))

    async def test_articulation_root_velocities_gg(self):
        await self.run_test(TestArticulationRootVelocities(self, DeviceParams(True, True)))


    async def test_linear_dof_positions_cc(self):
        await self.run_test(TestLinearDofPositions(self, DeviceParams(False, False)))

    async def test_linear_dof_positions_gc(self):
        await self.run_test(TestLinearDofPositions(self, DeviceParams(True, False)))

    async def test_linear_dof_positions_gg(self):
        await self.run_test(TestLinearDofPositions(self, DeviceParams(True, True)))


    async def test_linear_dof_velocities_cc(self):
        await self.run_test(TestLinearDofVelocities(self, DeviceParams(False, False)))

    async def test_linear_dof_velocities_gc(self):
        await self.run_test(TestLinearDofVelocities(self, DeviceParams(True, False)))

    async def test_linear_dof_velocities_gg(self):
        await self.run_test(TestLinearDofVelocities(self, DeviceParams(True, True)))


    async def test_linear_dof_forces_cc(self):
        await self.run_test(TestLinearDofForces(self, DeviceParams(False, False)))

    async def test_linear_dof_forces_gc(self):
        await self.run_test(TestLinearDofForces(self, DeviceParams(True, False)))

    async def test_linear_dof_forces_gg(self):
        await self.run_test(TestLinearDofForces(self, DeviceParams(True, True)))


    async def test_linear_dof_position_targets_cc(self):
        await self.run_test(TestLinearDofPositionTargets(self, DeviceParams(False, False)))

    async def test_linear_dof_position_targets_gc(self):
        await self.run_test(TestLinearDofPositionTargets(self, DeviceParams(True, False)))

    async def test_linear_dof_position_targets_gg(self):
        await self.run_test(TestLinearDofPositionTargets(self, DeviceParams(True, True)))


    async def test_linear_dof_velocity_targets_cc(self):
        await self.run_test(TestLinearDofVelocityTargets(self, DeviceParams(False, False)))

    async def test_linear_dof_velocity_targets_gc(self):
        await self.run_test(TestLinearDofVelocityTargets(self, DeviceParams(True, False)))

    async def test_linear_dof_velocity_targets_gg(self):
        await self.run_test(TestLinearDofVelocityTargets(self, DeviceParams(True, True)))


    async def test_angular_dof_positions_cc(self):
        await self.run_test(TestAngularDofPositions(self, DeviceParams(False, False)))

    async def test_angular_dof_positions_gc(self):
        await self.run_test(TestAngularDofPositions(self, DeviceParams(True, False)))

    async def test_angular_dof_positions_gg(self):
        await self.run_test(TestAngularDofPositions(self, DeviceParams(True, True)))


    async def test_angular_dof_velocities_cc(self):
        await self.run_test(TestAngularDofVelocities(self, DeviceParams(False, False)))

    async def test_angular_dof_velocities_gc(self):
        await self.run_test(TestAngularDofVelocities(self, DeviceParams(True, False)))

    async def test_angular_dof_velocities_gg(self):
        await self.run_test(TestAngularDofVelocities(self, DeviceParams(True, True)))


    async def test_angular_dof_forces_cc(self):
        await self.run_test(TestAngularDofForces(self, DeviceParams(False, False)))

    async def test_angular_dof_forces_gc(self):
        await self.run_test(TestAngularDofForces(self, DeviceParams(True, False)))

    async def test_angular_dof_forces_gg(self):
        await self.run_test(TestAngularDofForces(self, DeviceParams(True, True)))


    async def test_angular_dof_position_targets_cc(self):
        await self.run_test(TestAngularDofPositionTargets(self, DeviceParams(False, False)))

    async def test_angular_dof_position_targets_gc(self):
        await self.run_test(TestAngularDofPositionTargets(self, DeviceParams(True, False)))

    async def test_angular_dof_position_targets_gg(self):
        await self.run_test(TestAngularDofPositionTargets(self, DeviceParams(True, True)))


    async def test_angular_dof_velocity_targets_cc(self):
        await self.run_test(TestAngularDofVelocityTargets(self, DeviceParams(False, False)))

    async def test_angular_dof_velocity_targets_gc(self):
        await self.run_test(TestAngularDofVelocityTargets(self, DeviceParams(True, False)))

    async def test_angular_dof_velocity_targets_gg(self):
        await self.run_test(TestAngularDofVelocityTargets(self, DeviceParams(True, True)))


    async def test_heterogeneous_scene_articulation_cc(self):
        await self.run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(False, False)))

    async def test_heterogeneous_scene_articulation_gc(self):
        await self.run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(True, False)))

    async def test_heterogeneous_scene_articulation_gg(self):
        await self.run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(True, True)))

    async def test_heterogeneous_base_articulation_cc(self):
        await self.run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(False, False)))

    async def test_heterogeneous_base_articulation_gc(self):
        await self.run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(True, False)))

    async def test_heterogeneous_base_articulation_gg(self):
        await self.run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(True, True)))

    async def test_articulation_centroidal_momentum_cc(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(False, False)))

    async def test_articulation_centroidal_momentum_gc(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, False)))

    async def test_articulation_centroidal_momentum_gg(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, True)))

    async def test_articulation_centroidal_momentum_cc(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(False, False)))

    async def test_articulation_centroidal_momentum_gc(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, False)))

    async def test_articulation_centroidal_momentum_gg(self):
        await self.run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, True)))


    async def test_jacobians_cc(self):
        await self.run_test(TestJacobians(self, DeviceParams(False, False)))

    async def test_jacobians_gc(self):
        await self.run_test(TestJacobians(self, DeviceParams(True, False)))

    async def test_jacobians_gg(self):
        await self.run_test(TestJacobians(self, DeviceParams(True, True)))


    async def test_mass_matrices_cc(self):
        await self.run_test(TestMassMatrices(self, DeviceParams(False, False)))

    async def test_mass_matrices_gc(self):
        await self.run_test(TestMassMatrices(self, DeviceParams(True, False)))

    async def test_mass_matrices_gg(self):
        await self.run_test(TestMassMatrices(self, DeviceParams(True, True)))


    async def test_coriolis_centrifugal_cc(self):
        await self.run_test(TestCoriolisCentrifugal(self, DeviceParams(False, False)))

    async def test_coriolis_centrifugal_gc(self):
        await self.run_test(TestCoriolisCentrifugal(self, DeviceParams(True, False)))

    async def test_coriolis_centrifugal_gg(self):
        await self.run_test(TestCoriolisCentrifugal(self, DeviceParams(True, True)))


    async def test_gravity_compensation_cc(self):
        await self.run_test(TestGravityCompensation(self, DeviceParams(False, False)))

    async def test_gravity_compensation_gc(self):
        await self.run_test(TestGravityCompensation(self, DeviceParams(True, False)))

    async def test_gravity_compensation_gg(self):
        await self.run_test(TestGravityCompensation(self, DeviceParams(True, True)))


    async def test_link_accelerations_cc(self):
        await self.run_test(TestLinkAccelerations(self, DeviceParams(False, False), True))
    
    async def test_link_accelerations_gc(self):
        await self.run_test(TestLinkAccelerations(self, DeviceParams(True, False), True))

    async def test_link_accelerations_gg(self):
        await self.run_test(TestLinkAccelerations(self, DeviceParams(True, True), True))

    async def test_link_incoming_joint_force_force_on_fixed_link_x_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "X"))

    async def test_link_incoming_joint_force_force_on_fixed_link_x_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "X"))

    async def test_link_incoming_joint_force_force_on_fixed_link_x_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "X"))

    async def test_link_incoming_joint_force_force_on_fixed_link_y_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "Y"))

    async def test_link_incoming_joint_force_force_on_fixed_link_y_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "Y"))

    async def test_link_incoming_joint_force_force_on_fixed_link_y_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "Y"))

    async def test_link_incoming_joint_force_force_on_fixed_link_z_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "Z"))

    async def test_link_incoming_joint_force_force_on_fixed_link_z_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "Z"))

    async def test_link_incoming_joint_force_force_on_fixed_link_z_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "Z"))


    async def test_link_incoming_joint_force_force_on_child_link_x_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "X"))

    async def test_link_incoming_joint_force_force_on_child_link_x_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "X"))

    async def test_link_incoming_joint_force_force_on_child_link_x_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "X"))

    async def test_link_incoming_joint_force_force_on_child_link_y_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "Y"))

    async def test_link_incoming_joint_force_force_on_child_link_y_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "Y"))

    async def test_link_incoming_joint_force_force_on_child_link_y_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "Y"))

    async def test_link_incoming_joint_force_force_on_child_link_z_cc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "Z"))

    async def test_link_incoming_joint_force_force_on_child_link_z_gc(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "Z"))

    async def test_link_incoming_joint_force_force_on_child_link_z_gg(self):
        await self.run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "Z"))


    async def test_joint_friction_cc(self):
        await self.run_test(TestJointFriction(self, DeviceParams(False, False)))

    async def test_joint_friction_gc(self):
        await self.run_test(TestJointFriction(self, DeviceParams(True, False)))

    async def test_joint_friction_gg(self):
        await self.run_test(TestJointFriction(self, DeviceParams(True, True)))


    async def test_joint_performance_envelope_cc(self):
        await self.run_test(TestJointPerformanceEnvelope(self, DeviceParams(False, False)))

    async def test_joint_performance_envelope_gc(self):
        await self.run_test(TestJointPerformanceEnvelope(self, DeviceParams(True, False)))

    async def test_joint_performance_envelope_gg(self):
        await self.run_test(TestJointPerformanceEnvelope(self, DeviceParams(True, True)))


    # no contact force cases for D6 joint
    async def test_force_projection_single_link_x_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", -1))

    async def test_force_projection_single_link_x_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", -1))

    async def test_force_projection_single_link_x_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", -1))

    async def test_force_projection_single_link_y_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", -1))

    async def test_force_projection_single_link_y_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", -1))

    async def test_force_projection_single_link_y_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", -1))

    async def test_force_projection_single_link_z_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", -1))

    async def test_force_projection_single_link_z_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", -1))

    async def test_force_projection_single_link_z_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", -1))


    # zero dof torque cases for D6 joint
    async def test_force_projection_single_link_x_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", 0))

    async def test_force_projection_single_link_x_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", 0))

    async def test_force_projection_single_link_x_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", 0))

    async def test_force_projection_single_link_y_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", 0))

    async def test_force_projection_single_link_y_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", 0))

    async def test_force_projection_single_link_y_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", 0))

    async def test_force_projection_single_link_z_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", 0))

    async def test_force_projection_single_link_z_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", 0))

    async def test_force_projection_single_link_z_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", 0))

    # positive contact force and dof force cases for D6 joint
    async def test_force_projection_single_link_x_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1))

    async def test_force_projection_single_link_x_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1))

    async def test_force_projection_single_link_x_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1))

    async def test_force_projection_single_link_y_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1))

    async def test_force_projection_single_link_y_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1))

    async def test_force_projection_single_link_y_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1))

    async def test_force_projection_single_link_z_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1))

    async def test_force_projection_single_link_z_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1))

    async def test_force_projection_single_link_z_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1))


    # positive contact force and dof force cases for D6 joint with 2 rotational DOFs
    async def test_force_projection_single_link_x_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1, True))

    async def test_force_projection_single_link_x_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1, True))

    async def test_force_projection_single_link_x_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1, True))

    async def test_force_projection_single_link_y_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1, True))

    async def test_force_projection_single_link_y_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1, True))

    async def test_force_projection_single_link_y_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1, True))

    async def test_force_projection_single_link_z_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1, True))

    async def test_force_projection_single_link_z_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1, True))

    async def test_force_projection_single_link_z_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1, True))


    # positive contact force and dof force cases for D6 joint with 3 rotational DOFs
    async def test_force_projection_single_link_x_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1, True, True))

    async def test_force_projection_single_link_x_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1, True, True))

    async def test_force_projection_single_link_x_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1, True, True))

    async def test_force_projection_single_link_y_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1, True, True))

    async def test_force_projection_single_link_y_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1, True, True))

    async def test_force_projection_single_link_y_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1, True, True))

    async def test_force_projection_single_link_z_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1, True, True))

    async def test_force_projection_single_link_z_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1, True, True))

    async def test_force_projection_single_link_z_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1, True, True))


    # no contact force cases for D6 joint
    async def test_force_projection_double_link_x_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", -4))

    async def test_force_projection_double_link_x_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", -4))

    async def test_force_projection_double_link_x_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", -4))

    async def test_force_projection_double_link_y_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", -4))

    async def test_force_projection_double_link_y_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", -4))

    async def test_force_projection_double_link_y_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", -4))

    async def test_force_projection_double_link_z_zero_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", -4))

    async def test_force_projection_double_link_z_zero_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", -4))

    async def test_force_projection_double_link_z_zero_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", -4))


    # zero dof torque cases for D6 joint
    async def test_force_projection_double_link_x_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", 0))

    async def test_force_projection_double_link_x_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", 0))

    async def test_force_projection_double_link_x_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", 0))

    async def test_force_projection_double_link_y_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", 0))

    async def test_force_projection_double_link_y_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", 0))

    async def test_force_projection_double_link_y_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", 0))

    async def test_force_projection_double_link_z_zero_dof_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", 0))

    async def test_force_projection_double_link_z_zero_dof_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", 0))

    async def test_force_projection_double_link_z_zero_dof_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", 0))

    # positive contact force and dof force cases for D6 joint
    async def test_force_projection_double_link_x_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4))

    async def test_force_projection_double_link_x_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4))

    async def test_force_projection_double_link_x_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4))

    async def test_force_projection_double_link_y_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4))

    async def test_force_projection_double_link_y_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4))

    async def test_force_projection_double_link_y_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4))

    async def test_force_projection_double_link_z_dof_contact_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4))

    async def test_force_projection_double_link_z_dof_contact_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4))

    async def test_force_projection_double_link_z_dof_contact_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4))


    # positive contact force and dof force cases for D6 joint with 2 rotational DOFs
    async def test_force_projection_double_link_x_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4, True))

    async def test_force_projection_double_link_x_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4, True))

    async def test_force_projection_double_link_x_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4, True))

    async def test_force_projection_double_link_y_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4, True))

    async def test_force_projection_double_link_y_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4, True))

    async def test_force_projection_double_link_y_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4, True))

    async def test_force_projection_double_link_z_dof_contact_2dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4, True))

    async def test_force_projection_double_link_z_dof_contact_2dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4, True))

    async def test_force_projection_double_link_z_dof_contact_2dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4, True))


    # positive contact force and dof force cases for D6 joint with 3 rotational DOFs
    async def test_force_projection_double_link_x_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4, True, True))

    async def test_force_projection_double_link_x_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4, True, True))

    async def test_force_projection_double_link_x_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4, True, True))

    async def test_force_projection_double_link_y_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4, True, True))

    async def test_force_projection_double_link_y_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4, True, True))

    async def test_force_projection_double_link_y_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4, True, True))

    async def test_force_projection_double_link_z_dof_contact_3dofs_cc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4, True, True))

    async def test_force_projection_double_link_z_dof_contact_3dofs_gc(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4, True, True))

    async def test_force_projection_double_link_z_dof_contact_3dofs_gg(self):
        await self.run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4, True, True))

    # revolute joint 
    async def test_force_torque_sensor_x_cc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "X"))

    async def test_force_torque_sensor_x_gc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "X"))

    async def test_force_torque_sensor_x_gg(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "X"))

    async def test_force_torque_sensor_y_cc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "Y"))

    async def test_force_torque_sensor_y_gc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "Y"))

    async def test_force_torque_sensor_y_gg(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "Y"))

    async def test_force_torque_sensor_z_cc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "Z"))

    async def test_force_torque_sensor_z_gc(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "Z"))

    async def test_force_torque_sensor_z_gg(self):
        await self.run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "Z"))

    # dof forces for the cartpole articulation
    async def test_joint_actuation_force_cc(self):
        await self.run_test(TestJointActuationForces(self, DeviceParams(False, False)))

    async def test_joint_actuation_force_gc(self):
        await self.run_test(TestJointActuationForces(self, DeviceParams(True, False)))

    async def test_joint_actuation_force_gg(self):
        await self.run_test(TestJointActuationForces(self, DeviceParams(True, True)))

    # no need for GPU, this is a getter/setter test that goes through PhysX CPU API
    async def test_inertia_set_get_consistency_rigid_body_cpu(self):
        await self.run_test(TestReproInertiaSetGetConsistency(self, DeviceParams(False, False), as_articulation=False))

    # no need for GPU, this is a getter/setter test that goes through PhysX CPU API
    async def test_inertia_set_get_consistency_articulation_cpu(self):
        await self.run_test(TestReproInertiaSetGetConsistency(self, DeviceParams(False, False), as_articulation=True))

    async def test_articulation_body_properties_cpu(self):
        await self.run_test(TestArticulationBodyProperties(self, DeviceParams(False, False)))

    async def test_articulation_body_properties_gpu(self):
        await self.run_test(TestArticulationBodyProperties(self, DeviceParams(True, True)))


    async def test_articulation_shape_properties_cpu(self):
        await self.run_test(TestArticulationShapeProperties(self, DeviceParams(False, False)))

    async def test_articulation_shape_properties_gpu(self):
        await self.run_test(TestArticulationShapeProperties(self, DeviceParams(True, True)))


    async def test_articulation_fixed_tendon_properties_cc(self):
        await self.run_test(TestArticulationFixedTendonProperties(self, DeviceParams(False, False)))

    async def test_articulation_fixed_tendon_properties_gc(self):
        await self.run_test(TestArticulationFixedTendonProperties(self, DeviceParams(True, False)))

    async def test_articulation_fixed_tendon_properties_gg(self):
        await self.run_test(TestArticulationFixedTendonProperties(self, DeviceParams(True, True)))

    async def test_articulation_heterogeneous_fixed_tendon_properties_cc(self):
        await self.run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(False, False)))

    async def test_articulation_heterogeneous_fixed_tendon_properties_gc(self):
        await self.run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(True, False)))

    async def test_articulation_heterogeneous_fixed_tendon_properties_gg(self):
        await self.run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(True, True)))

    async def test_articulation_spatial_tendon_properties_cc(self):
        await self.run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(False, False)))

    async def test_articulation_spatial_tendon_properties_gc(self):
        await self.run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(True, False)))

    async def test_articulation_spatial_tendon_properties_gg(self):
        await self.run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(True, True)))

    async def test_articulation_heterogeneous_spatial_tendon_properties_cc(self):
        await self.run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(False, False)))

    async def test_articulation_heterogeneous_spatial_tendon_properties_gc(self):
        await self.run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(True, False)))

    async def test_articulation_heterogeneous_spatial_tendon_properties_gg(self):
        await self.run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(True, True)))


    async def test_rigid_body_view_cpu(self):
        await self.run_test(TestRigidBodyView(self, DeviceParams(False, False)))

    async def test_rigid_body_view_gpu(self):
        await self.run_test(TestRigidBodyView(self, DeviceParams(True, True)))


    async def test_rigid_body_properties_cpu(self):
        await self.run_test(TestRigidBodyProperties(self, DeviceParams(False, False)))

    async def test_rigid_body_properties_gpu(self):
        await self.run_test(TestRigidBodyProperties(self, DeviceParams(True, True)))

    async def test_object_type_cpu(self):
        await self.run_test(TestObjectType(self, DeviceParams(False, False)))
    
    async def test_object_type_gpu(self):
        await self.run_test(TestObjectType(self, DeviceParams(True, True)))

    async def test_rigid_body_shape_properties_cpu(self):
        await self.run_test(TestRigidBodyShapeProperties(self, DeviceParams(False, False)))

    async def test_rigid_body_shape_properties_gpu(self):
        await self.run_test(TestRigidBodyShapeProperties(self, DeviceParams(True, True)))


    async def test_rigid_body_transforms_cc(self):
        await self.run_test(TestRigidBodyTransforms(self, DeviceParams(False, False)))

    async def test_rigid_body_transforms_gc(self):
        await self.run_test(TestRigidBodyTransforms(self, DeviceParams(True, False)))

    async def test_rigid_body_transforms_gg(self):
        await self.run_test(TestRigidBodyTransforms(self, DeviceParams(True, True)))


    async def test_rigid_body_velocities_cc(self):
        await self.run_test(TestRigidBodyVelocities(self, DeviceParams(False, False)))

    async def test_rigid_body_velocities_gc(self):
        await self.run_test(TestRigidBodyVelocities(self, DeviceParams(True, False)))

    async def test_rigid_body_velocities_gg(self):
        await self.run_test(TestRigidBodyVelocities(self, DeviceParams(True, True)))


    async def test_rigid_body_accelerations_cc(self):
        await self.run_test(TestRigidBodyAccelerations(self, DeviceParams(False, False)))

    async def test_rigid_body_accelerations_gc(self):
        await self.run_test(TestRigidBodyAccelerations(self, DeviceParams(True, False)))

    async def test_rigid_body_accelerations_gg(self):
        await self.run_test(TestRigidBodyAccelerations(self, DeviceParams(True, True)))


    async def test_rigid_body_global_force_cc(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(False, False)), True)

    async def test_rigid_body_global_force_gc(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(True, False)), True)

    async def test_rigid_body_global_force_gg(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(True, True)), True)

    async def test_rigid_body_local_force_cc(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(False, False), False))

    async def test_rigid_body_local_force_gc(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(True, False), False))

    async def test_rigid_body_local_force_gg(self):
        await self.run_test(TestRigidBodyForce(self, DeviceParams(True, True)), False)

    async def test_rigid_body_global_force_at_pos_cc(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(False, False), True))

    async def test_rigid_body_global_force_at_pos_gc(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, False), True))

    async def test_rigid_body_global_force_at_pos_gg(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, True), True))

    async def test_rigid_body_local_force_at_pos_cc(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(False, False), False))

    async def test_rigid_body_local_force_at_pos_gc(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, False), False))

    async def test_rigid_body_local_force_at_pos_gg(self):
        await self.run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, True), False))

    async def test_rigid_contacts_cc(self):
        await self.run_test(TestRigidContacts(self, DeviceParams(False, False)))

    async def test_rigid_contacts_gc(self):
        await self.run_test(TestRigidContacts(self, DeviceParams(True, False)))

    async def test_rigid_contacts_gg(self):
        await self.run_test(TestRigidContacts(self, DeviceParams(True, True)))

    async def test_sdf_shapes_gg(self):
        await self.run_test(TestSdfShapeView(self, DeviceParams(True, True)))

    async def test_articulation_contacts_cc(self):
        await self.run_test(TestArticulationContacts(self, DeviceParams(False, False)))

    async def test_articulation_contacts_gc(self):
        await self.run_test(TestArticulationContacts(self, DeviceParams(True, False)))

    async def test_articulation_contacts_gg(self):
        await self.run_test(TestArticulationContacts(self, DeviceParams(True, True)))

    async def test_rigid_contact_matrix_cc(self):
        await self.run_test(TestRigidContactMatrix(self, DeviceParams(False, False)))

    async def test_rigid_contact_matrix_gc(self):
        await self.run_test(TestRigidContactMatrix(self, DeviceParams(True, False)))

    async def test_rigid_contact_matrix_gg(self):
        await self.run_test(TestRigidContactMatrix(self, DeviceParams(True, True)))
    async def test_particle_cloth_positions_gg(self):
        await self.run_test(TestParticleClothPositions(self, DeviceParams(True, True)))

    async def test_rigid_contact_perf_cc(self):
        await self.run_test(TestRigidContactPerfTest(self, DeviceParams(False, False)))

    async def test_rigid_contact_perf_gg(self):
        await self.run_test(TestRigidContactPerfTest(self, DeviceParams(True, False)))

    async def test_rigid_contact_perf_gg(self):
        await self.run_test(TestRigidContactPerfTest(self, DeviceParams(True, True)))


    async def test_particle_cloth_velocities_gg(self):
        await self.run_test(TestParticleClothVelocities(self, DeviceParams(True, True)))

    async def test_particle_cloth_masses_gg(self):
        await self.run_test(TestParticleClothMasses(self, DeviceParams(True, True)))

    async def test_particle_cloth_spring_damping_gg(self):
        await self.run_test(TestParticleClothSprings(self, DeviceParams(True, True), SpringEnum.DAMPING))

    async def test_particle_cloth_spring_stiffness_gg(self):
        await self.run_test(TestParticleClothSprings(self, DeviceParams(True, True), SpringEnum.STIFFNESS))


    async def test_particle_system_solid_rest_offset_gc(self):
        await self.run_test(TestParticleSystemOffsets(self, DeviceParams(True, True), OffsetEnum.SOLIDRESTOFFSET))

    async def test_particle_system_fluid_rest_offset_gc(self):
        await self.run_test(TestParticleSystemOffsets(self, DeviceParams(True, True), OffsetEnum.SOLIDRESTOFFSET))

    async def test_particle_system_particle_contact_offset_gc(self):
        await self.run_test(TestParticleSystemOffsets(self, DeviceParams(True, True), OffsetEnum.PARTICLECONTACTOFFSET))

    async def test_particle_system_wind_gc(self):
        await self.run_test(TestParticleSystemWind(self, DeviceParams(True, True)))


    async def test_particle_material_friction_gc(self):
        await self.run_test(TestParticleMaterialFriction(self, DeviceParams(True, True)))

    async def test_particle_material_damping_gc(self):
        await self.run_test(TestParticleMaterialDamping(self, DeviceParams(True, True)))

    async def test_particle_material_gravity_scale_gc(self):
        await self.run_test(TestParticleMaterialGravityScale(self, DeviceParams(True, True)))

    async def test_particle_material_lift_gc(self):
        await self.run_test(TestParticleMaterialLift(self, DeviceParams(True, True)))

    async def test_particle_material_drag_gc(self):
        await self.run_test(TestParticleMaterialDrag(self, DeviceParams(True, True)))

    async def test_soft_body_multiple_views_gg(self):
        await self.run_test(TestSoftBodyMultipleView(self, DeviceParams(True, True)))

    async def test_soft_body_element_indices_gg(self):
        await self.run_test(TestSoftBodyElementIndices(self, DeviceParams(True, True)))

    async def test_soft_body_sim_positions_gg(self):
        await self.run_test(TestSoftBodySimPositions(self, DeviceParams(True, True)))

    @unittest.skip("OMPE-24852")
    async def test_soft_body_sim_velocities_gg(self):
        await self.run_test(TestSoftBodySimVelocities(self, DeviceParams(True, True)))

    async def test_soft_body_sim_kinematic_targets_gg(self):
        await self.run_test(TestSoftBodyKinematicTargets(self, DeviceParams(True, True)))

    @unittest.skip("OM-91738")
    async def test_soft_body_sim_staging_buffers_gg(self):
        await self.run_test(TestSoftBodyStagingBuffers(self, DeviceParams(True, True)))

    async def test_soft_body_material_elasticity_cc(self):
        await self.run_test(TestSoftBodyMaterialElasticity(self, DeviceParams(True, True)))

    async def test_soft_body_material_damping_cc(self):
        await self.run_test(TestSoftBodyMaterialDamping(self, DeviceParams(True, True)))

    async def test_soft_body_material_dynamic_friction_cc(self):
        await self.run_test(TestSoftBodyMaterialDynamicFriction(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_multiple_views_gg(self):
        await self.run_test(TestVolumeDeformableBodyMultipleView(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_element_indices_gg(self):
        await self.run_test(TestVolumeDeformableBodyElementIndices(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_rest_gg(self):
        await self.run_test(TestDeformableBodyRest(self, DeviceParams(True, True), test_surface=False))

    async def test_volume_deformable_body_simulation_positions_gg(self):
        await self.run_test(TestVolumeDeformableBodySimulationPositions(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_simulation_velocities_gg(self):
        await self.run_test(TestVolumeDeformableBodySimulationVelocities(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_simulation_kinematic_targets_gg(self):
        await self.run_test(TestVolumeDeformableBodySimulationKinematicTargets(self, DeviceParams(True, True)))

    async def test_volume_deformable_body_staging_buffers_gg(self):
        await self.run_test(TestVolumeDeformableBodyStagingBuffers(self, DeviceParams(True, True)))

    async def test_surface_deformable_body_multiple_views_gg(self):
        await self.run_test(TestSurfaceDeformableBodyMultipleView(self, DeviceParams(True, True)))

    async def test_surface_deformable_body_element_indices_gg(self):
        await self.run_test(TestSurfaceDeformableBodyElementIndices(self, DeviceParams(True, True)))

    async def test_surface_deformable_body_rest_gg(self):
        await self.run_test(TestDeformableBodyRest(self, DeviceParams(True, True), test_surface=True))

    async def test_surface_deformable_body_simulation_positions_gg(self):
        await self.run_test(TestSurfaceDeformableBodySimulationPositions(self, DeviceParams(True, True)))

    async def test_surface_deformable_body_simulation_velocities_gg(self):
        await self.run_test(TestSurfaceDeformableBodySimulationVelocities(self, DeviceParams(True, True)))

    async def test_deformable_material_youngs_modulus_cc(self):
        await self.run_test(TestDeformableMaterialYoungsModulus(self, DeviceParams(True, True)))

    async def test_deformable_material_dynamic_friction_cc(self):
        await self.run_test(TestDeformableMaterialDynamicFriction(self, DeviceParams(True, True)))
 
    async def test_prim_deletion_cc(self):
        await self.run_test(TestUsdPrimDeletion(self, DeviceParams(False, False)))
 
    async def test_sim_view_invalidate_cc(self):
        await self.run_test(TestSimViewInvalidate(self, DeviceParams(False, False)))
    async def test_sim_view_invalidate_gc(self):
        await self.run_test(TestSimViewInvalidate(self, DeviceParams(True, False)))
    async def test_sim_view_invalidate_gg(self):
        await self.run_test(TestSimViewInvalidate(self, DeviceParams(True, True)))
