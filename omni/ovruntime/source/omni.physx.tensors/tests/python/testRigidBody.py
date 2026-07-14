# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys
import unittest
import math

import numpy as np
import warp as wp

import _tensors_setup  # noqa: F401

import omni.physics.tensors
import warp_utils as wp_utils

from pxr import Gf, Sdf, UsdPhysics, PhysxSchema, UsdGeom

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)

_WARM_START = True
_FRONTEND = "warp"
_KEEPALIVE = False


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

class TestRigidBodyView(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)

        self.test_case.assertEqual(len(balls.prim_paths), self.num_envs)
        self.test_case.assertTrue(all(f"/envs/env{i}/ball" in balls.prim_paths for i in range(self.num_envs)))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyEnableDisablePhysics(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(8, 2.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.num_balls = 4
        for i in range(self.num_balls):
            actor_path = self.env_template_path.AppendChild(f"ball_{i}")
            transform = Transform((0.0, 0.0, 1.0 + i * 0.5))
            ball = self.create_rigid_ball(actor_path, transform, 0.1)

            mass_api = UsdPhysics.MassAPI(ball)
            mass_api.GetMassAttr().Set(1.0)

    def on_start(self, sim):
        self.balls = sim.create_rigid_body_view("/envs/*/ball_*")
        self.check_rigid_body_view(self.balls, self.num_envs * self.num_balls)

        self.all_indices = wp_utils.arange(self.balls.count, device=sim.device)
        self.subset_inds_disabled_np = np.arange(0, self.balls.count, 2, dtype=np.int32)
        self.subset_inds_enabled_np = np.arange(1, self.balls.count, 2, dtype=np.int32)
        self.subset_indices_disabled_wp = wp.from_numpy(self.subset_inds_disabled_np, dtype=wp.int32, device=sim.device)
        self.subset_indices_enabled_wp = wp.from_numpy(self.subset_inds_enabled_np, dtype=wp.int32, device=sim.device)

        self.wp_disable_flags = wp.from_numpy(np.ones(self.balls.count, dtype=np.uint8), dtype=wp.uint8, device="cpu")
        self.wp_enable_flags = wp.from_numpy(np.zeros(self.balls.count, dtype=np.uint8), dtype=wp.uint8, device="cpu")

        self.balls.set_disable_simulations(self.wp_disable_flags, self.subset_indices_disabled_wp)

        current_disable_flags = self.balls.get_disable_simulations().numpy().flatten()
        self.test_case.assertTrue(np.all(current_disable_flags[self.subset_inds_disabled_np] == True))
        self.test_case.assertTrue(np.all(current_disable_flags[self.subset_inds_enabled_np] == False))

        self.dt = 1.0 / self.sim_params.time_steps_per_second

    def on_physics_step(self, sim, stepno, dt):
        current_velocities = self.balls.get_velocities().numpy().reshape(self.balls.count, 6)
        current_velocities_z = current_velocities[:, 2]

        if stepno == 1:
            expected_velocity_z = -2.0 * self.dt * self.sim_params.gravity_mag
            self.test_case.assertTrue(np.allclose(current_velocities_z[self.subset_inds_disabled_np], 0.0),
                                      f"Disabled objects should not fall when physics is disabled (step {stepno})")
            self.test_case.assertTrue(np.allclose(current_velocities_z[self.subset_inds_enabled_np], expected_velocity_z),
                                      f"Enabled objects should fall when physics is enabled (step {stepno})")

            # swap enabled / disabled
            self.balls.set_disable_simulations(self.wp_disable_flags, self.subset_indices_enabled_wp)
            self.balls.set_disable_simulations(self.wp_enable_flags, self.subset_indices_disabled_wp)

            current_disable_flags = self.balls.get_disable_simulations().numpy().flatten()
            self.test_case.assertTrue(np.all(current_disable_flags[self.subset_inds_enabled_np] == True))
            self.test_case.assertTrue(np.all(current_disable_flags[self.subset_inds_disabled_np] == False))

            # Wake up now-enabled rigid bodies to ensure they respond to physics immediately
            self.balls.wake_up(self.all_indices)

        elif stepno == 2:
            expected_velocity_z = -self.dt * self.sim_params.gravity_mag
            self.test_case.assertTrue(np.allclose(current_velocities_z[self.subset_inds_enabled_np], 0.0),
                                      f"Disabled objects should not fall when physics is disabled (step {stepno})")
            self.test_case.assertTrue(np.allclose(current_velocities_z[self.subset_inds_disabled_np], expected_velocity_z),
                                      f"Enabled objects should fall when physics is enabled (step {stepno})")
            self.finish()


class TestRigidBodyProperties(GridTestBase):
    def __init__(self, test_case, device_params):
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
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

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

        # ArticulationRootAPI is applied to torso not the top level ant xform, so no articulation will be recognized here.
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


class TestRigidBodyCompliantContact(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_multi_shape_rigid_body(actor_path, transform, 0.5)

    def on_start(self, sim):
        self.balls = sim.create_rigid_body_view("/envs/*/ball")
        self.all_indices = wp_utils.arange(self.balls.count)
        self.stiffness = 100000
        self.damping = 1000
        # materials
        self.compliant_material_properties = np.zeros((self.balls.count, self.balls.max_shapes, 4))
        self.compliant_material_properties[:, :, 0] = 0.8
        self.compliant_material_properties[:, :, 1] = 0.7
        self.compliant_material_properties[:, :, 2] = self.stiffness
        self.compliant_material_properties[:, :, 3] = self.damping
        self.compliant_material_combine_modes = np.zeros((self.balls.count, self.balls.max_shapes, 3))
        self.compliant_material_combine_modes[:, :, 0] = 1
        self.compliant_material_combine_modes[:, :, 1] = 2
        self.compliant_material_combine_modes[:, :, 2] = 3
        self.wp_properties = wp.from_numpy(self.compliant_material_properties, dtype=wp.float32, device="cpu")
        self.wp_modes = wp.from_numpy(self.compliant_material_combine_modes, dtype=wp.uint8, device="cpu")
        self.balls.set_compliant_material_properties(self.wp_properties, self.wp_modes, self.all_indices)
        prop, mode = self.balls.get_compliant_material_properties()
        self.test_case.assertTrue(np.allclose(prop.numpy(), self.compliant_material_properties[:, :, 2:]))
        self.test_case.assertTrue(np.allclose(mode.numpy(), self.compliant_material_combine_modes[:, :, 1:]))

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 100:
            transforms = self.balls.get_transforms().numpy().reshape(self.balls.count, 7)
            # penetration dx = mg/(4*k) based on static equilibrium and 4 contact points, 0.25 is rest position for rigid/perfect contact
            expected_z = 0.25 - self.balls.get_masses().numpy().reshape(self.balls.count, ) * 10 / (4 * self.stiffness)
            self.test_case.assertTrue(np.allclose(transforms[:, 2], expected_z, rtol=1e-03, atol=1e-02), "expected z")

            # try a different stiffness
            self.stiffness = self.stiffness * 10
            self.compliant_material_properties[:, :, 2] = self.stiffness
            self.wp_properties = wp.from_numpy(self.compliant_material_properties, dtype=wp.float32, device="cpu")
            self.balls.set_compliant_material_properties(self.wp_properties, self.wp_modes, self.all_indices)
        if stepno == 299:
            transforms = self.balls.get_transforms().numpy().reshape(self.balls.count, 7)
            # penetration dx = mg/(4*k) based on static equilibrium and 4 contact points, 0.25 is rest position for rigid/perfect contact
            expected_z = 0.25 - self.balls.get_masses().numpy().reshape(self.balls.count, ) * 10 / (4 * self.stiffness)
            self.test_case.assertTrue(np.allclose(transforms[:, 2], expected_z, rtol=1e-03, atol=1e-02), "expected z")
            self.finish()


class TestArticulationCompliantContact(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count)

    def on_physics_step(self, sim, stepno, dt):
        # After settling phase
        if stepno == 50:
            self.stiffness = 100000
            self.damping = 1000
            # materials
            self.compliant_material_properties = np.zeros((self.ants.count, self.ants.max_shapes, 4))
            self.compliant_material_properties[:, :, 0] = 0.8
            self.compliant_material_properties[:, :, 1] = 0.7
            self.compliant_material_properties[:, :, 2] = self.stiffness
            self.compliant_material_properties[:, :, 3] = self.damping
            self.compliant_material_combine_modes = np.zeros((self.ants.count, self.ants.max_shapes, 3))
            self.compliant_material_combine_modes[:, :, 0] = 1
            self.compliant_material_combine_modes[:, :, 1] = 2
            self.compliant_material_combine_modes[:, :, 2] = 3
            self.wp_properties = wp.from_numpy(self.compliant_material_properties, dtype=wp.float32, device="cpu")
            self.wp_modes = wp.from_numpy(self.compliant_material_combine_modes, dtype=wp.uint8, device="cpu")
            self.ants.set_compliant_material_properties(self.wp_properties, self.wp_modes, self.all_indices)
            prop, mode = self.ants.get_compliant_material_properties()
            self.test_case.assertTrue(np.allclose(prop.numpy(), self.compliant_material_properties[:, :, 2:]))
            self.test_case.assertTrue(np.allclose(mode.numpy(), self.compliant_material_combine_modes[:, :, 1:]))

            self.rigid_transforms = self.ants.get_root_transforms().numpy().reshape(self.ants.count, 7)

        if stepno == 100:
            transforms = self.ants.get_root_transforms().numpy().reshape(self.ants.count, 7)
            # penetration dx = mg/(4*k) based on static equilibrium and 4 contact points, 0.25 is rest position for rigid/perfect contact
            expected_z = self.rigid_transforms[:, 2] - np.sum(self.ants.get_masses().numpy(), axis=1).reshape(self.ants.count, ) * 10 / (4 * self.stiffness)
            self.test_case.assertTrue(np.allclose(transforms[:, 2], expected_z, rtol=1e-03, atol=1e-02), "expected z")

            # try a different stiffness
            self.stiffness = self.stiffness * 10
            self.compliant_material_properties[:, :, 2] = self.stiffness
            self.wp_properties = wp.from_numpy(self.compliant_material_properties, dtype=wp.float32, device="cpu")
            self.ants.set_compliant_material_properties(self.wp_properties, self.wp_modes, self.all_indices)
        if stepno == 200:
            transforms = self.ants.get_root_transforms().numpy().reshape(self.ants.count, 7)
            # penetration dx = mg/(4*k) based on static equilibrium and 4 contact points, 0.25 is rest position for rigid/perfect contact
            expected_z = self.rigid_transforms[:, 2] - np.sum(self.ants.get_masses().numpy(), axis=1).reshape(self.ants.count, ) * 10 / (4 * self.stiffness)
            self.test_case.assertTrue(np.allclose(transforms[:, 2], expected_z, rtol=1e-03, atol=1e-02), "expected z")
            self.finish()


class TestRigidBodyStates(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

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

        self.on_start_impl(sim)

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyTransforms(TestRigidBodyStates):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        transforms = self.balls.get_transforms()
        transforms_np = transforms.numpy().reshape((self.balls.count, 7))
        z_step = np.linspace(0.0, 1.0, self.balls.count, dtype=np.float32)
        transforms_np[..., 2] += z_step

        transforms = wp.from_numpy(transforms_np, dtype=wp.float32, device=sim.device)
        self.balls.set_transforms(transforms, self.all_indices)

        self.expected_transforms = transforms_np

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            transforms = self.balls.get_transforms()
            transforms_np = transforms.numpy().reshape((self.balls.count, 7))
            self.test_case.assertTrue(np.allclose(transforms_np, self.expected_transforms, rtol=1e-03, atol=1e-04), "expected transforms")
            self.finish()


class TestRigidBodyVelocities(TestRigidBodyStates):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        vels_np = np.zeros((self.balls.count, 6))
        linear_z = np.linspace(0.1, 1.0, self.balls.count, dtype=np.float32)
        vels_np[..., 2] = linear_z

        vels = wp.from_numpy(vels_np, dtype=wp.float32, device=sim.device)
        self.balls.set_velocities(vels, self.all_indices)

        self.expected_vels = vels_np

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 10:
            vels = self.balls.get_velocities()
            vels_np = vels.numpy().reshape((self.balls.count, 6))
            self.test_case.assertTrue(np.allclose(vels_np, self.expected_vels, rtol=1e-03, atol=1e-03), "expected velocities")
            self.finish()


class TestRigidBodyAccelerations(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(4, 1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 1.0)
        sim_params.gravity_mag = -10
        super().__init__(test_case, grid_params, sim_params, device_params)

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
        vels_np = np.zeros((self.balls.count, 6))
        linear_z = np.linspace(0.0, 1.0, self.balls.count, dtype=np.float32)
        vels_np[..., 2] = linear_z

        vels = wp.from_numpy(vels_np, dtype=wp.float32, device=sim.device)
        self.balls.set_velocities(vels, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 2:
            self.old_vels = self.balls.get_velocities().numpy().reshape((self.balls.count, 6)).copy()
        if stepno == 3:
            self.new_vels = self.balls.get_velocities().numpy().reshape((self.balls.count, 6)).copy()
            acc = self.balls.get_accelerations().numpy().reshape((self.balls.count, 6)).copy()
            acc_finite_difference = (self.new_vels - self.old_vels) / dt
            self.test_case.assertTrue(np.allclose(acc, acc_finite_difference, rtol=1e-03, atol=1e-03), "expected acceleration")
            analytical_acc = np.tile(np.array([0, 0, -10, 0, 0, 0]), (self.balls.count, 1))
            self.test_case.assertTrue(np.allclose(acc, analytical_acc, rtol=1e-03, atol=1e-03), "expected acceleration")
            self.finish()


class TestRigidBodyForce(GridTestBase):
    def __init__(self, test_case, device_params, is_global=True):
        grid_params = GridParams(16, 1.0)
        sim_params = SimParams()
        self.is_global = is_global
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        q = Gf.Quatf(Gf.Rotation(Gf.Vec3d([0, 1, 0]), 90).GetQuat())
        self.transform = Transform((0.0, 0.0, 0.5), q)
        self.create_rigid_ball(actor_path, self.transform, 0.15)

    def on_start(self, sim):
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)
        gForce = wp.vec3(0.0, 0.0, 4000.0)

        if self.is_global:
            forces = wp_utils.fill_vec3(balls.count, value=gForce, device=sim.device)
        else:
            # Transform the gForce to local space
            q = self.transform.q.GetInverse()
            lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
            forces = wp_utils.fill_vec3(balls.count, value=lForce, device=sim.device)

        indices = wp_utils.arange(balls.count, device=sim.device)
        balls.apply_forces_and_torques_at_position(forces, None, None, indices, self.is_global)

        self.balls = balls

    def on_physics_step(self, sim, stepno, dt):
        transforms = self.balls.get_transforms()
        z = transforms.numpy().reshape(self.balls.count, 7)[:, 2]
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
        self.num_envs = 16
        grid_params = GridParams(self.num_envs, 4.0)
        sim_params = SimParams()
        self.is_global = is_global
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.body_per_env = 0

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, -10.0, 2.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.body_per_env += 9

        # a different set of ants to test articulation only APIs
        actor_path_2 = self.env_template_path.AppendChild("ant_2")
        transform = Transform((0.0, 10.0, 2.0))
        self.create_actor_from_asset(actor_path_2, transform, asset_path)

        transform = Transform((-0.5, 0.0, 0.0))
        self.create_rigid_ball(actor_path.AppendChild("right_ball"), transform, 0.1)
        self.body_per_env += 1

        transform = Transform((0.5, 0.0, 0.0))
        self.create_rigid_ball(actor_path.AppendChild("left_ball"), transform, 0.1)
        self.body_per_env += 1

    def on_start(self, sim):
        force_offset = 1
        rb_view = sim.create_rigid_body_view(["/envs/*/ant/*_ball",
                                              "/envs/*/ant/*_foot",
                                              "/envs/*/ant/*_leg",
                                              "/envs/*/ant/torso"])
        arti_view = sim.create_articulation_view("/envs/*/ant_2/torso")
        self.rb_view_root_indices = []  # map articulation roots from the rb view to indices used later
        for index, path in enumerate(rb_view.prim_paths):
            if path[-5:] == "torso":
                self.rb_view_root_indices.append(index)

        self.check_rigid_body_view(rb_view, self.num_envs * self.body_per_env)
        self.check_articulation_view(arti_view, self.num_envs, arti_view.max_links, arti_view.max_dofs, True)
        transforms = rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))
        positions = transforms[:, :, 0:3]
        rotations = transforms[:, :, 3:7]

        transforms = arti_view.get_link_transforms().numpy().reshape((self.num_envs, arti_view.max_links, 7))
        arti_positions = transforms[:, :, 0:3]
        arti_rotations = transforms[:, :, 3:7]
        self.indices = wp_utils.arange(rb_view.count, device=sim.device)
        self.arti_indices = wp_utils.arange(arti_view.count, device=sim.device)
        gForce = wp.vec3(0.0, 0.0, 100.0)
        if self.is_global:
            positions[:, :, 0:3] += np.array([0, 0, force_offset])
            arti_positions[:, :, 0:3] += np.array([0, 0, force_offset])
            forces = wp_utils.fill_vec3(rb_view.count, value=gForce, device=sim.device)
            arti_forces = wp_utils.fill_vec3(arti_view.count * arti_view.max_links, value=gForce, device=sim.device)
        else:
            positions[:, :, 0:3] = np.array([0, 0, force_offset])
            arti_positions[:, :, 0:3] = np.array([0, 0, force_offset])
            # Transform the gForce to local space
            forces = np.zeros((self.num_envs, self.body_per_env, 3))
            arti_forces = np.zeros((self.num_envs, arti_view.max_links, 3))
            for e in range(self.num_envs):
                for b in range(self.body_per_env):
                    rot = rotations[e, b, :].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    forces[e, b, :] = lForce
                for b in range(arti_view.max_links):
                    rot = arti_rotations[e, b, :].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    arti_forces[e, b, :] = lForce

            forces = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
            arti_forces = wp.from_numpy(arti_forces.flatten(), dtype=wp.float32, device=sim.device)

        self.forces = forces
        self.arti_forces = arti_forces
        self.positions = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.arti_positions = wp.from_numpy(arti_positions.flatten(), dtype=wp.float32, device=sim.device)
        self.rb_view = rb_view
        self.arti_view = arti_view

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            self.rb_view.apply_forces_and_torques_at_position(self.forces, None, self.positions, self.indices, self.is_global)
            self.arti_view.apply_forces_and_torques_at_position(self.arti_forces, None, self.arti_positions, self.arti_indices, self.is_global)
        if stepno == 10:
            arti_transforms = self.arti_view.get_link_transforms().numpy().reshape((self.num_envs, self.arti_view.max_links, 7))[:, 0]
            transforms = self.rb_view.get_transforms().numpy().reshape((self.num_envs * self.body_per_env, 7))[self.rb_view_root_indices]
            self.test_case.assertTrue(np.allclose(arti_transforms[:, 2], transforms[:, 2], rtol=1e-03, atol=1e-2), "Similar root height regardless of the type of view to use to apply forces")
            self.test_case.assertTrue(np.allclose(arti_transforms[:, 3:], transforms[:, 3:], rtol=1e-03, atol=1e-2), "Similar root orientation regardless of the type of view to use to apply forces")
            self.finish()


class TestRigidBodiesGetSet(GridTestBase):
    def __init__(self, test_case, device_params, is_global=False):
        self.num_envs = 16
        grid_params = GridParams(self.num_envs, 4.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 20
        self.is_global = is_global
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.body_per_env = 0
        # set a known mass
        mass = 0.5
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        self.body_per_env += 3

        transform = Transform((-0.5, 0.0, 0.0))
        ball = self.create_rigid_ball(actor_path.AppendChild("right_ball"), transform, 0.1)
        mass_api = UsdPhysics.MassAPI(ball)
        mass_api.GetMassAttr().Set(mass)
        self.body_per_env += 1

        transform = Transform((0.5, 0.0, 0.0))
        ball = self.create_rigid_ball(actor_path.AppendChild("left_ball"), transform, 0.1)
        mass_api = UsdPhysics.MassAPI(ball)
        mass_api.GetMassAttr().Set(mass)
        self.body_per_env += 1
        self.atol = 1e-05


class TestRigidBodiesGetSetAppliedForces(TestRigidBodiesGetSet):
    def compute_forces(self, sim):
        force_offset = 1
        transforms = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))
        positions = transforms[:, :, 0:3]
        rotations = transforms[:, :, 3:7]
        self.indices = wp_utils.arange(self.rb_view.count, device=sim.device)
        gForce = wp.vec3(0.0, 0.0, 150.0)
        if self.is_global:
            positions[:, :, 0:3] += np.array([0, 0, force_offset])
            forces = wp_utils.fill_vec3(self.rb_view.count, value=gForce, device=sim.device)
        else:
            positions[:, :, 0:3] = np.array([0, 0, force_offset])
            # Transform the gForce to local space
            forces = np.zeros((self.num_envs, self.body_per_env, 3))
            for e in range(self.num_envs):
                for b in range(self.body_per_env):
                    rot = rotations[e, b, :].tolist()
                    q = Gf.Quatf(rot[3], rot[0], rot[1], rot[2])
                    lForce = q.GetConjugate().Transform(Gf.Vec3f(*gForce))
                    forces[e, b, :] = lForce

        forces_wp = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
        force_subset_wp = wp.from_numpy(forces[0:6].flatten(), dtype=wp.float32, device=sim.device)

        self.forces_wp = forces_wp
        self.force_subset_wp = force_subset_wp

        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.positions_subset_wp = wp.from_numpy(positions[0:6].flatten(), dtype=wp.float32, device=sim.device)

    def on_start(self, sim):
        self.rb_view = sim.create_rigid_body_view("/envs/*/ant/right_ball|left_ball|right_back_leg|right_back_foot|torso")
        self.rb_view_subset = sim.create_rigid_body_view("/envs/env[0-5]/ant/right_ball|left_ball|right_back_leg|right_back_foot|torso")
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.check_rigid_body_view(self.rb_view_subset, 6 * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=sim.device)

        self.indices = wp_utils.arange(self.rb_view.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        # tests for get after set for all bodies
        if stepno == 1:
            self.init_height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[:, :, 2].copy()

            self.compute_forces(sim)
            self.rb_view.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, self.indices, self.is_global)
        if stepno == 10:
            height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[:, :, 2].copy()
            self.test_case.assertTrue((height > self.init_height).all(), "larger heights")

        # tests for setting a subsets of envs (last 6 envs)
        if stepno == 60:
            self.init_height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[-6:, :, 2].copy()
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 * self.body_per_env:], wp.uint32)

            self.compute_forces(sim)
            self.rb_view.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, subset_indices, self.is_global)
        if stepno == 70:
            height = self.rb_view.get_transforms().numpy().reshape((self.num_envs, self.body_per_env, 7))[-6:, :, 2].copy()
            self.test_case.assertTrue((height >= self.init_height - 0.0001).all(), "larger heights")

        # tests for setting via a different view and subset (first 6 envs)
        if stepno == 100:
            self.init_height = self.rb_view_subset.get_transforms().numpy().reshape((6, self.body_per_env, 7))[:, :, 2].copy()
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)

            self.compute_forces(sim)
            self.rb_view_subset.apply_forces_and_torques_at_position(self.force_subset_wp, None, self.positions_subset_wp, subset_indices, self.is_global)
        if stepno == 110:
            height = self.rb_view_subset.get_transforms().numpy().reshape((6, self.body_per_env, 7))[:, :, 2].copy()
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
        self.sim = sim
        transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7)).copy()
        transforms_subset = self.rb_view_subset.get_transforms().numpy().reshape((self.rb_view_subset.count, 7)).copy()

        # tests for get after set
        if stepno == 1:
            submitted_transforms = transforms + np.array([0, 0, 1.0, 0, 0, 0, 0])
            self.rb_view.set_transforms(self.to_warp(submitted_transforms), self.all_indices)
            # new transforms should be available to read
            new_transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7))
            self.test_case.assertTrue(np.allclose(new_transforms[:, 2], transforms[:, 2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:, 2], submitted_transforms[:, 2], rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            submitted_transforms = transforms + np.array([0, 0, 1.0, 0, 0, 0, 0])
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 * self.body_per_env:], wp.uint32)
            self.rb_view.set_transforms(self.to_warp(submitted_transforms), subset_indices)
            # new transforms should be available to read
            new_transforms = self.rb_view.get_transforms().numpy().reshape((self.rb_view.count, 7))
            self.test_case.assertTrue(np.allclose(new_transforms[-6 * self.body_per_env:, 2], transforms[-6 * self.body_per_env:, 2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[-6 * self.body_per_env:, 2], submitted_transforms[-6 * self.body_per_env:, 2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:(self.num_envs - 6) * self.body_per_env, :], transforms[:(self.num_envs - 6) * self.body_per_env, :], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms[:(self.num_envs - 6) * self.body_per_env, 2], submitted_transforms[:(self.num_envs - 6) * self.body_per_env, 2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)
            delta = np.array([0, 0, 1.0, 0, 0, 0, 0])
            submitted_transforms = transforms_subset + delta
            self.rb_view_subset.set_transforms(self.to_warp(submitted_transforms), subset_indices)
            # new transforms should be available to read
            new_transforms = self.rb_view_subset.get_transforms().numpy().reshape((self.rb_view_subset.count, 7))
            self.test_case.assertTrue(np.allclose(new_transforms, transforms_subset + delta, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_transforms, submitted_transforms, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
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
        self.sim = sim
        velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6)).copy()
        velocities_subset = self.rb_view_subset.get_velocities().numpy().reshape((self.rb_view_subset.count, 6)).copy()

        # tests for get after set
        if stepno == 1:
            submitted_velocities = velocities + np.array([0, 0, 1, 0, 0, 0])
            self.rb_view.set_velocities(self.to_warp(submitted_velocities), self.all_indices)
            # new velocities should be available to read
            new_velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6))
            self.test_case.assertTrue(np.allclose(new_velocities[:, 2], velocities[:, 2] + 1, rtol=1e-02, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:, 2], submitted_velocities[:, 2], rtol=1e-02, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            submitted_velocities = velocities + np.array([0, 0, 1, 0, 0, 0])
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 * self.body_per_env:], wp.uint32)
            self.rb_view.set_velocities(self.to_warp(submitted_velocities), subset_indices)
            # new velocities should be available to read
            new_velocities = self.rb_view.get_velocities().numpy().reshape((self.rb_view.count, 6))
            self.test_case.assertTrue(np.allclose(new_velocities[-6 * self.body_per_env:, 2], velocities[-6 * self.body_per_env:, 2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[-6 * self.body_per_env:, 2], submitted_velocities[-6 * self.body_per_env:, 2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:(self.num_envs - 6) * self.body_per_env, :], velocities[:(self.num_envs - 6) * self.body_per_env, :], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities[:(self.num_envs - 6) * self.body_per_env, 2], submitted_velocities[:(self.num_envs - 6) * self.body_per_env, 2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.rb_view_subset.count, device=sim.device)
            submitted_velocities = velocities_subset + 1
            self.rb_view_subset.set_velocities(self.to_warp(submitted_velocities), subset_indices)
            # new velocities should be available to read
            new_velocities = self.rb_view_subset.get_velocities().numpy().reshape((self.rb_view_subset.count, 6))
            self.test_case.assertTrue(np.allclose(new_velocities, velocities_subset + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_velocities, submitted_velocities, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestReproInertiaSetGetConsistency(GridTestBase):
    def __init__(self, test_case, device_params, as_articulation=False):
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


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsRigidBodyTests(unittest.TestCase):

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

    # -- Basic view creation --

    def test_rigid_body_view_cpu(self):
        self._run_test(TestRigidBodyView(self, DeviceParams(False, False)))

    def test_rigid_body_view_gpu(self):
        self._run_test(TestRigidBodyView(self, DeviceParams(True, True)))

    # -- Transforms, velocities, accelerations --

    def test_rigid_body_transforms_cc(self):
        self._run_test(TestRigidBodyTransforms(self, DeviceParams(False, False)))

    def test_rigid_body_transforms_gc(self):
        self._run_test(TestRigidBodyTransforms(self, DeviceParams(True, False)))

    def test_rigid_body_transforms_gg(self):
        self._run_test(TestRigidBodyTransforms(self, DeviceParams(True, True)))

    def test_rigid_body_velocities_cc(self):
        self._run_test(TestRigidBodyVelocities(self, DeviceParams(False, False)))

    def test_rigid_body_velocities_gc(self):
        self._run_test(TestRigidBodyVelocities(self, DeviceParams(True, False)))

    def test_rigid_body_velocities_gg(self):
        self._run_test(TestRigidBodyVelocities(self, DeviceParams(True, True)))

    def test_rigid_body_accelerations_cc(self):
        self._run_test(TestRigidBodyAccelerations(self, DeviceParams(False, False)))

    def test_rigid_body_accelerations_gc(self):
        self._run_test(TestRigidBodyAccelerations(self, DeviceParams(True, False)))

    def test_rigid_body_accelerations_gg(self):
        self._run_test(TestRigidBodyAccelerations(self, DeviceParams(True, True)))

    # -- Forces --

    def test_rigid_body_global_force_cc(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(False, False), is_global=True))

    def test_rigid_body_global_force_gc(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(True, False), is_global=True))

    def test_rigid_body_global_force_gg(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(True, True), is_global=True))

    def test_rigid_body_local_force_cc(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(False, False), is_global=False))

    def test_rigid_body_local_force_gc(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(True, False), is_global=False))

    def test_rigid_body_local_force_gg(self):
        self._run_test(TestRigidBodyForce(self, DeviceParams(True, True), is_global=False))

    def test_rigid_body_global_force_at_pos_cc(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(False, False), is_global=True))

    def test_rigid_body_global_force_at_pos_gc(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, False), is_global=True))

    def test_rigid_body_global_force_at_pos_gg(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, True), is_global=True))

    def test_rigid_body_local_force_at_pos_cc(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(False, False), is_global=False))

    def test_rigid_body_local_force_at_pos_gc(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, False), is_global=False))

    def test_rigid_body_local_force_at_pos_gg(self):
        self._run_test(TestRigidBodyForceAtPos(self, DeviceParams(True, True), is_global=False))

    # -- Get/Set applied forces, transforms, velocities --

    def test_rigid_body_get_set_applied_forces_cc(self):
        self._run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(False, False)))

    def test_rigid_body_get_set_applied_forces_gc(self):
        self._run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(True, False)))

    def test_rigid_body_get_set_applied_forces_gg(self):
        self._run_test(TestRigidBodiesGetSetAppliedForces(self, DeviceParams(True, True)))

    def test_rigid_body_get_set_transforms_cc(self):
        self._run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(False, False)))

    def test_rigid_body_get_set_transforms_gc(self):
        self._run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(True, False)))

    def test_rigid_body_get_set_transforms_gg(self):
        self._run_test(TestRigidBodiesGetSetTransforms(self, DeviceParams(True, True)))

    def test_rigid_body_get_set_velocities_cc(self):
        self._run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(False, False)))

    def test_rigid_body_get_set_velocities_gc(self):
        self._run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(True, False)))

    def test_rigid_body_get_set_velocities_gg(self):
        self._run_test(TestRigidBodiesGetSetVelocities(self, DeviceParams(True, True)))

    # -- Properties --

    def test_rigid_body_properties_cpu(self):
        self._run_test(TestRigidBodyProperties(self, DeviceParams(False, False)))

    def test_rigid_body_properties_gpu(self):
        self._run_test(TestRigidBodyProperties(self, DeviceParams(True, True)))

    def test_object_type_cpu(self):
        self._run_test(TestObjectType(self, DeviceParams(False, False)))

    def test_object_type_gpu(self):
        self._run_test(TestObjectType(self, DeviceParams(True, True)))

    def test_rigid_body_shape_properties_cpu(self):
        self._run_test(TestRigidBodyShapeProperties(self, DeviceParams(False, False)))

    def test_rigid_body_shape_properties_gpu(self):
        self._run_test(TestRigidBodyShapeProperties(self, DeviceParams(True, True)))

    # -- Compliant contact --

    def test_rigid_body_compliant_material_cpu(self):
        self._run_test(TestRigidBodyCompliantContact(self, DeviceParams(False, False)))

    def test_rigid_body_compliant_material_gpu(self):
        self._run_test(TestRigidBodyCompliantContact(self, DeviceParams(True, True)))

    def test_articulation_compliant_material_properties_cpu(self):
        self._run_test(TestArticulationCompliantContact(self, DeviceParams(False, False)))

    def test_articulation_compliant_material_properties_gpu(self):
        self._run_test(TestArticulationCompliantContact(self, DeviceParams(True, True)))

    # -- Enable/disable physics --

    def test_rigid_body_enable_disable_physics_cc(self):
        self._run_test(TestRigidBodyEnableDisablePhysics(self, DeviceParams(False, False)))

    # -- Inertia consistency --

    def test_inertia_set_get_consistency_rigid_body_cpu(self):
        self._run_test(TestReproInertiaSetGetConsistency(self, DeviceParams(False, False), as_articulation=False))

    def test_inertia_set_get_consistency_articulation_cpu(self):
        self._run_test(TestReproInertiaSetGetConsistency(self, DeviceParams(False, False), as_articulation=True))
