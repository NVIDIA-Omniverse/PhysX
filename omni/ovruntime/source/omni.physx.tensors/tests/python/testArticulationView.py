# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys
import unittest

import numpy as np
import warp as wp

import _tensors_setup  # noqa: F401

import omni.physics.tensors
import warp_utils as wp_utils

from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)

_WARM_START = True
_FRONTEND = "warp"


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

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

    def on_physics_step(self, sim, stepno, dt):
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
            if parent != "":  # note that the root of floating articulation does not have a parent
                parent_index = mt.link_indices[parent]
                expected_parent_indices[child] = parent_index

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

        # check DOF motions
        dof_motions = humanoids.get_dof_motions()
        dof_motions_np = dof_motions.numpy().reshape(humanoids.count, humanoids.max_dofs)[0]
        expected_dof_motions = np.repeat(np.uint8(omni.physics.tensors.DofMotion.Limited), humanoids.max_dofs)
        self.test_case.assertTrue(np.array_equal(dof_motions_np, expected_dof_motions))

        # check DOF limits
        dof_limits = humanoids.get_dof_limits()
        dof_limits_np = dof_limits.numpy().reshape(humanoids.count, humanoids.max_dofs, 2)[0]

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
            self.test_case.assertAlmostEqual(lower, expected_lower, places=4)
            self.test_case.assertAlmostEqual(upper, expected_upper, places=4)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationSpecialCases(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs = 16
        grid_params = GridParams(self.num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

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
        link_position = Gf.Vec3f(0, 0, 1.0)
        physicsUtils.add_rigid_sphere(self.stage, root_link_path, 0.1, link_position)
        joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path)
        joint.CreateBody1Rel().SetTargets([root_link_path])

    def on_start(self, sim):
        self.articulations = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(self.articulations, self.num_envs, 1, 0, True)

    def on_physics_step(self, sim, stepno, dt):
        articulations = self.articulations
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
        self.num_envs = 16
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
        link_position = Gf.Vec3f(0, 0, 1.0)
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
            parent_link_name = art_metatype.link_parents[i + 1]
            child_link_name = art_metatype.link_names[i + 1]
            parent_link_index = art_metatype.link_parent_indices[child_link_name]
            self.test_case.assertTrue((expected_parent_link_names[i] == parent_link_name), "link parent name is correct")
            self.test_case.assertTrue((expected_parent_link_indices[i] == parent_link_index), "link parent index is correct")

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
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

        from omni.physx.bindings._physx import (
            PERF_ENV_API
        )
        lower_waist_joint = self.stage.GetPrimAtPath(actor_path.AppendPath("joints/lower_waist"))
        if lower_waist_joint:
            lower_waist_joint.ApplyAPI(PERF_ENV_API, UsdPhysics.Tokens.rotX)

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
        # Only DOF 0 (lower_waist rotX) has the API, so only it will have values
        result = humanoids.get_dof_drive_model_properties().numpy()
        self.test_case.assertTrue(np.allclose(result[:, 0, :], drive_model_properties[:, 0, :]))
        self.test_case.assertFalse(np.any(np.isclose(result[:, 1:, :], drive_model_properties[:, 1:, :])))

        # joint drive_model_properties indexed
        indexed_indices = [0]
        drive_model_properties[indexed_indices, :, 0] = 0.5
        drive_model_properties[indexed_indices, :, 1] = 0.6
        drive_model_properties[indexed_indices, :, 2] = 0.7
        indices = wp.from_numpy(indexed_indices, dtype=wp.int32, device="cpu")
        wp_mf = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
        humanoids.set_dof_drive_model_properties(wp_mf, indices)
        result = humanoids.get_dof_drive_model_properties().numpy()
        self.test_case.assertTrue(np.allclose(result[indexed_indices, 0, :], drive_model_properties[indexed_indices, 0, :]))
        self.test_case.assertFalse(np.any(np.isclose(result[indexed_indices, 1:, :], drive_model_properties[indexed_indices, 1:, :])))

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

    def on_physics_step(self, sim, stepno, dt):
        pass


# ---------------------------------------------------------------------------
# Import physicsUtils (made available by _physics_setup.py on sys.path)
# ---------------------------------------------------------------------------
import physicsUtils  # noqa: E402


# ---------------------------------------------------------------------------
# Test case class
# ---------------------------------------------------------------------------

class PhysxTensorsArticulationViewTests(unittest.TestCase):

    def _run_test(self, scenario):
        sync_params = SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False)
        runner = RunnerInMemory(scenario, _FRONTEND, sync_params)
        exc_info = None
        try:
            runner.start(_WARM_START)
            runner.simulate()
        except Exception:
            exc_info = sys.exc_info()
        finally:
            try:
                runner.stop()
            except Exception:
                if exc_info is None:
                    exc_info = sys.exc_info()
        if exc_info is not None:
            raise exc_info[1].with_traceback(exc_info[2])

    def test_articulation_view_cpu(self):
        self._run_test(TestArticulationView(self, DeviceParams(False, False)))

    def test_articulation_view_gpu(self):
        self._run_test(TestArticulationView(self, DeviceParams(True, True)))

    def test_humanoid_view_cpu(self):
        self._run_test(TestHumanoidView(self, DeviceParams(False, False)))

    def test_humanoid_view_gpu(self):
        self._run_test(TestHumanoidView(self, DeviceParams(True, True)))

    def test_articulation_dof_properties_cpu(self):
        self._run_test(TestArticulationDofProperties(self, DeviceParams(False, False)))

    def test_articulation_dof_properties_gpu(self):
        self._run_test(TestArticulationDofProperties(self, DeviceParams(True, True)))

    def test_articulation_special_cases_cpu(self):
        self._run_test(TestArticulationSpecialCases(self, DeviceParams(False, False)))

    def test_articulation_special_cases_gpu(self):
        self._run_test(TestArticulationSpecialCases(self, DeviceParams(True, True)))

    def test_articulation_view_duplicate_name_cpu(self):
        self._run_test(TestArticulationViewDuplicateNames(self, DeviceParams(False, False)))
