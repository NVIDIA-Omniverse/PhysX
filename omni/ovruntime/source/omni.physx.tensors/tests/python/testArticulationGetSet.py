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

from pxr import Gf, UsdPhysics, PhysxSchema

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
    set_drive,
)

_WARM_START = True
_FRONTEND = "warp"


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

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
        self.atol = 1e-05

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
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof)).copy()
        # tests for get after set
        if stepno == 1:
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
        self.sim = sim
        roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7)).copy()
        roots_subset = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7)).copy()
        delta = np.array([0, 0, 1.0, 0, 0, 0, 0])

        # # tests for get after set
        if stepno == 1:
            submitted_roots = roots + delta
            self.ants.set_root_transforms(self.to_warp(submitted_roots), self.all_indices)
            # new roots should be available to read
            new_roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7))
            self.test_case.assertTrue(np.allclose(new_roots[:, 2], roots[:, 2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[:, 2], submitted_roots[:, 2], rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            submitted_roots = roots + delta
            subset_indices = self.to_warp(self.all_indices.numpy()[0:5], wp.uint32)
            self.ants.set_root_transforms(self.to_warp(submitted_roots), subset_indices)
            # new roots should be available to read
            new_roots = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7))
            self.test_case.assertTrue(np.allclose(new_roots[0:5, 2], roots[0:5, 2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[0:5, 2], submitted_roots[0:5, 2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[5:, :], roots[5:, :], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots[5:, 2], submitted_roots[5:, 2] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_roots = roots_subset + delta
            self.ants_subset.set_root_transforms(self.to_warp(submitted_roots), subset_indices)
            # new roots velocities should be available to read
            new_roots = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7))
            self.test_case.assertTrue(np.allclose(new_roots, roots_subset + delta, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_roots, submitted_roots, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetRootVelocities(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
        roots_subset_vels = self.ants_subset.get_root_velocities().numpy().reshape((self.ants_subset.count, 6)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_root_vels = root_vels + 0.5
            self.ants.set_root_velocities(self.to_warp(submitted_root_vels), self.all_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6))
            self.test_case.assertTrue(np.allclose(new_root_vels, root_vels + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels, submitted_root_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:5], wp.uint32)
            submitted_root_vels = root_vels + 1
            self.ants.set_root_velocities(self.to_warp(submitted_root_vels), subset_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6))
            self.test_case.assertTrue(np.allclose(new_root_vels[0:5], root_vels[0:5] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[5:], root_vels[5:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[0:5], submitted_root_vels[0:5], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels[5:], submitted_root_vels[5:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_root_vels = roots_subset_vels + 1
            self.ants_subset.set_root_velocities(self.to_warp(submitted_root_vels), subset_indices)
            # new roots velocities should be available to read
            new_root_vels = self.ants_subset.get_root_velocities().numpy().reshape((self.ants_subset.count, 6))
            ind = subset_indices.numpy()
            self.test_case.assertTrue(np.allclose(new_root_vels, roots_subset_vels + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_root_vels, submitted_root_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetDofPositions(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_positions = self.ants_subset.get_dof_positions().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_dof_positions = dof_positions + 0.5
            self.ants.set_dof_positions(self.to_warp(submitted_dof_positions), self.all_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_positions, dof_positions + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions, submitted_dof_positions, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32)
            submitted_dof_positions = dof_positions + 1
            self.ants.set_dof_positions(self.to_warp(submitted_dof_positions), subset_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants.get_dof_positions().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_positions[0:2], dof_positions[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[2:], dof_positions[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[0:2], submitted_dof_positions[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions[2:], submitted_dof_positions[2:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_positions = dofs_subset_positions + 1
            self.ants_subset.set_dof_positions(self.to_warp(submitted_dof_positions), subset_indices)
            # new dofs positions should be available to read
            new_dof_positions = self.ants_subset.get_dof_positions().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_positions, dofs_subset_positions + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_positions, submitted_dof_positions, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetDofPositionTarget(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_position_targets = self.ants_subset.get_dof_position_targets().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_dof_position_targets = dof_position_targets + 0.5
            self.ants.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), self.all_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, dof_position_targets + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, submitted_dof_position_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32)
            submitted_dof_position_targets = dof_position_targets + 1
            self.ants.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), subset_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants.get_dof_position_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[0:2], dof_position_targets[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[2:], dof_position_targets[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[0:2], submitted_dof_position_targets[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets[2:], submitted_dof_position_targets[2:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_position_targets = dofs_subset_position_targets + 1
            self.ants_subset.set_dof_position_targets(self.to_warp(submitted_dof_position_targets), subset_indices)
            # new dofs position_targets should be available to read
            new_dof_position_targets = self.ants_subset.get_dof_position_targets().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, dofs_subset_position_targets + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_position_targets, submitted_dof_position_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetDofVelocities(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_vels = self.ants_subset.get_dof_velocities().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_dof_vels = dof_vels + 0.5
            self.ants.set_dof_velocities(self.to_warp(submitted_dof_vels), self.all_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_vels, dof_vels + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels, submitted_dof_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32)
            submitted_dof_vels = dof_vels + 1
            self.ants.set_dof_velocities(self.to_warp(submitted_dof_vels), subset_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_vels[0:2], dof_vels[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[2:], dof_vels[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[0:2], submitted_dof_vels[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels[2:], submitted_dof_vels[2:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_vels = dofs_subset_vels + 1
            self.ants_subset.set_dof_velocities(self.to_warp(submitted_dof_vels), subset_indices)
            # new dofs velocities should be available to read
            new_dof_vels = self.ants_subset.get_dof_velocities().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_vels, dofs_subset_vels + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_vels, submitted_dof_vels, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetDofVelocityTarget(TestArticulationGetSet):

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_velocity_targets = self.ants_subset.get_dof_velocity_targets().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_dof_velocity_targets = dof_velocity_targets + 0.5
            self.ants.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), self.all_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, dof_velocity_targets + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, submitted_dof_velocity_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32)
            submitted_dof_velocity_targets = dof_velocity_targets + 1
            self.ants.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), subset_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants.get_dof_velocity_targets().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[0:2], dof_velocity_targets[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[2:], dof_velocity_targets[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[0:2], submitted_dof_velocity_targets[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets[2:], submitted_dof_velocity_targets[2:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_velocity_targets = dofs_subset_velocity_targets + 1
            self.ants_subset.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets), subset_indices)
            # new dofs velocity_targets should be available to read
            new_dof_velocity_targets = self.ants_subset.get_dof_velocity_targets().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, dofs_subset_velocity_targets + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets, submitted_dof_velocity_targets, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetDofActuationForces(TestArticulationGetSet):
    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = self.ants.max_dofs
        dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof)).copy()
        dofs_subset_actuation_forces = self.ants_subset.get_dof_actuation_forces().numpy().reshape((self.ants_subset.count, num_dof)).copy()

        # # tests for get after set
        if stepno == 1:
            submitted_dof_actuation_forces = dof_actuation_forces + 0.5
            self.ants.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), self.all_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, dof_actuation_forces + 0.5, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, submitted_dof_actuation_forces, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting a subsets of envs
        if stepno == 2:
            subset_indices = self.to_warp(self.all_indices.numpy()[0:2], wp.uint32)
            submitted_dof_actuation_forces = dof_actuation_forces + 1
            self.ants.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), subset_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants.get_dof_actuation_forces().numpy().reshape((self.ants.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[0:2], dof_actuation_forces[0:2] + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[2:], dof_actuation_forces[2:], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[0:2], submitted_dof_actuation_forces[0:2], rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces[2:], submitted_dof_actuation_forces[2:] - 1, rtol=1e-03, atol=self.atol), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            submitted_dof_actuation_forces = dofs_subset_actuation_forces + 1
            self.ants_subset.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces), subset_indices)
            # new dofs forces should be available to read
            new_dof_actuation_forces = self.ants_subset.get_dof_actuation_forces().numpy().reshape((self.ants_subset.count, num_dof))
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, dofs_subset_actuation_forces + 1, rtol=1e-03, atol=self.atol), "get returns the current set values")
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces, submitted_dof_actuation_forces, rtol=1e-03, atol=self.atol), "get returns the current set values")

        if stepno == 5:
            self.finish()


class TestArticulationGetSetAppliedForces(TestArticulationGetSet):
    def __init__(self, test_case, device_params, global_frame=False):
        super().__init__(test_case, device_params)
        self.is_global = global_frame

    def on_start(self, sim):
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.ants_subset = sim.create_articulation_view("/envs/env[0-5]/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.check_articulation_view(self.ants_subset, 6, 9, 8, True)
        force_offset = 1
        transforms = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))
        positions = transforms[:, :, 0:3]
        rotations = transforms[:, :, 3:7]
        self.indices = wp_utils.arange(self.ants.count, device=sim.device)
        gForce = wp.vec3(0.0, 0.0, 10.0)
        if self.is_global:
            positions[:, :, 0:3] += np.array([0, 0, force_offset])
            self.forces_wp = wp_utils.fill_vec3(self.ants.count * self.ants.max_links, value=gForce, device=sim.device)
            self.force_subset_wp = wp_utils.fill_vec3(6 * self.ants.max_links, value=gForce, device=sim.device)
        else:
            positions[:, :, 0:3] = np.array([0, 0, force_offset])
            # Transform the gForce to local space
            forces = np.zeros((self.num_envs, self.ants.max_links, 3))
            for e in range(self.num_envs):
                for b in range(self.ants.max_links):
                    rot = rotations[e, b, :].tolist()
                    q = Gf.Quatf(*np.array(rot))
                    lForce = (q * Gf.Quatf(0, Gf.Vec3f(*gForce)) * q.GetInverse()).GetImaginary()
                    forces[e, b, :] = lForce
            forces_wp = wp.from_numpy(forces.flatten(), dtype=wp.float32, device=sim.device)
            force_subset_wp = wp.from_numpy(forces[0:6].flatten(), dtype=wp.float32, device=sim.device)
            self.forces_wp = forces_wp
            self.force_subset_wp = force_subset_wp

        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=sim.device)
        self.positions_subset_wp = wp.from_numpy(positions[0:6].flatten(), dtype=wp.float32, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        # # tests for get after set for all bodies
        if stepno == 1:
            self.init_height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[:, :, 2].copy()
            self.ants.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, self.indices, self.is_global)
        if stepno == 10:
            height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[:, :, 2].copy()
            self.test_case.assertTrue((height > self.init_height).all(), "larger heights")

        # tests for setting a subsets of envs (last 6 envs)
        if stepno == 20:
            self.init_height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[-6:, :, 2].copy()
            subset_indices = self.to_warp(self.all_indices.numpy()[-6 * self.ants.max_links:], wp.uint32)
            self.ants.apply_forces_and_torques_at_position(self.forces_wp, None, self.positions_wp, subset_indices, self.is_global)
        if stepno == 30:
            height = self.ants.get_link_transforms().numpy().reshape((self.num_envs, self.ants.max_links, 7))[-6:, :, 2].copy()
            self.test_case.assertTrue((height >= self.init_height - 0.0001).all(), "larger heights")

        # # tests for setting via a different view and subset (first 6 envs)
        if stepno == 40:
            self.init_height = self.ants_subset.get_link_transforms().numpy().reshape((6, self.ants_subset.max_links, 7))[:, :, 2].copy()
            subset_indices = wp_utils.arange(self.ants_subset.count, device=sim.device)
            self.ants_subset.apply_forces_and_torques_at_position(self.force_subset_wp, None, self.positions_subset_wp, subset_indices, self.is_global)
        if stepno == 50:
            height = self.ants_subset.get_link_transforms().numpy().reshape((6, self.ants_subset.max_links, 7))[:, :, 2].copy()
            self.test_case.assertTrue((height >= self.init_height).all(), "larger heights")
            self.finish()


class TestArticulationGetSetLinkGravity(TestArticulationGetSet):
    def on_physics_step(self, sim, stepno, dt):
        physics_scene = UsdPhysics.Scene.Define(self.stage, self.scene_path)
        physics_scene.CreateGravityMagnitudeAttr().Set(10)
        self.all_indices = wp_utils.arange(self.ants.count, device="cpu")
        self.sim = sim
        link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links)).copy()
        # # tests for get after set
        if stepno == 1:
            submitted_link_gravities = np.ones((self.ants.count, self.ants.max_links), dtype=np.uint8)
            self.ants.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), self.all_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links))
            self.test_case.assertTrue((new_link_gravities == link_gravities + 1).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities == submitted_link_gravities).all(), "get returns the current set values")

        if stepno == 2:
            subset_indices = wp.from_numpy(self.all_indices.numpy()[0:2], wp.uint32, device="cpu")
            submitted_link_gravities = np.zeros((self.ants.count, self.ants.max_links), dtype=np.uint8)
            self.ants.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), subset_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants.get_disable_gravities().numpy().reshape((self.ants.count, self.ants.max_links))
            self.test_case.assertTrue((new_link_gravities[0:2] == link_gravities[0:2] - 1).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[2:] == link_gravities[2:]).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[0:2] == submitted_link_gravities[0:2]).all(), "get returns the current set values")
            self.test_case.assertTrue((new_link_gravities[2:] == submitted_link_gravities[2:] + 1).all(), "get returns the current set values")

        # tests for setting via a different view and subset
        if stepno == 3:
            subset_indices = wp_utils.arange(self.ants_subset.count)
            submitted_link_gravities = np.ones((self.ants_subset.count, self.ants_subset.max_links), dtype=np.uint8)
            # disable only a subset of link gravities
            submitted_link_gravities[:, 5:] = 0
            self.ants_subset.set_disable_gravities(wp.from_numpy(submitted_link_gravities, dtype=wp.uint8, device="cpu"), subset_indices)
            # new link gravities should be available to read
            new_link_gravities = self.ants_subset.get_disable_gravities().numpy().reshape((self.ants_subset.count, self.ants_subset.max_links))
            self.test_case.assertTrue((new_link_gravities == submitted_link_gravities).all(), "get returns the current set values")

        if stepno == 20:
            roots_heights = self.ants_subset.get_root_transforms().numpy().reshape((self.ants_subset.count, 7))[:, 2]
            enabled_gravity_root_heights = roots_heights[:5]
            disabled_gravity_root_heights = roots_heights[5:]
            self.test_case.assertTrue((np.mean(enabled_gravity_root_heights) < np.mean(disabled_gravity_root_heights)).all(), "lower heights for envs where gravity is active")
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
            if i % 4 == 1 or i % 4 == 3:
                joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/cartJoint" % i)
                set_drive(joint_prim, "linear", "position", 0.0, stiffness, damping, max_force)
                angularDriveAPI = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
                angularDriveAPI.CreateTypeAttr("force")
                self.expected_drive_types[i, 0] = 1
            if i % 4 == 2 or i % 4 == 3:
                joint_prim = self.stage.GetPrimAtPath("/envs/env%d/cartpole/poleJoint" % i)
                set_drive(joint_prim, "angular", "position", 0.0, stiffness, damping, max_force)
                angularDriveAPI = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                angularDriveAPI.CreateTypeAttr("acceleration")
                self.expected_drive_types[i, 1] = 2

    def on_start(self, sim):
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(self.cartpoles, self.num_envs, 3, 2, True)
        drive_types = self.cartpoles.get_drive_types().numpy()
        self.test_case.assertTrue(np.allclose(drive_types, self.expected_drive_types), "expected drive types")

    def on_physics_step(self, sim, stepno, dt):
        self.finish()


# ---------------------------------------------------------------------------
# Test case class
# ---------------------------------------------------------------------------

class PhysxTensorsArticulationGetSetTests(unittest.TestCase):

    def _run_test(self, scenario):
        sync_params = SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False)
        runner = RunnerInMemory(scenario, "warp", sync_params)
        exc_info = None
        try:
            runner.start(True)
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

    def test_articulation_kinematic_update_cc(self):
        self._run_test(TestArticulationKinematicUpdate(self, DeviceParams(False, False)))

    def test_articulation_kinematic_update_gc(self):
        self._run_test(TestArticulationKinematicUpdate(self, DeviceParams(True, False)))

    def test_articulation_kinematic_update_gg(self):
        self._run_test(TestArticulationKinematicUpdate(self, DeviceParams(True, True)))

    def test_articulation_get_set_root_transforms_cc(self):
        self._run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(False, False)))

    def test_articulation_get_set_root_transforms_gc(self):
        self._run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(True, False)))

    def test_articulation_get_set_root_transforms_gg(self):
        self._run_test(TestArticulationGetSetRootTransforms(self, DeviceParams(True, True)))

    def test_articulation_get_set_root_velocities_cc(self):
        self._run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(False, False)))

    def test_articulation_get_set_root_velocities_gc(self):
        self._run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(True, False)))

    def test_articulation_get_set_root_velocities_gg(self):
        self._run_test(TestArticulationGetSetRootVelocities(self, DeviceParams(True, True)))

    def test_articulation_get_set_dof_positions_cc(self):
        self._run_test(TestArticulationGetSetDofPositions(self, DeviceParams(False, False)))

    def test_articulation_get_set_dof_positions_gc(self):
        self._run_test(TestArticulationGetSetDofPositions(self, DeviceParams(True, False)))

    def test_articulation_get_set_dof_positions_gg(self):
        self._run_test(TestArticulationGetSetDofPositions(self, DeviceParams(True, True)))

    def test_articulation_get_set_dof_position_target_cc(self):
        self._run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(False, False)))

    def test_articulation_get_set_dof_position_target_gc(self):
        self._run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(True, False)))

    def test_articulation_get_set_dof_position_target_gg(self):
        self._run_test(TestArticulationGetSetDofPositionTarget(self, DeviceParams(True, True)))

    def test_articulation_get_set_dof_velocities_cc(self):
        self._run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(False, False)))

    def test_articulation_get_set_dof_velocities_gc(self):
        self._run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(True, False)))

    def test_articulation_get_set_dof_velocities_gg(self):
        self._run_test(TestArticulationGetSetDofVelocities(self, DeviceParams(True, True)))

    def test_articulation_get_set_dof_velocity_targets_cc(self):
        self._run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(False, False)))

    def test_articulation_get_set_dof_velocity_targets_gc(self):
        self._run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(True, False)))

    def test_articulation_get_set_dof_velocity_targets_gg(self):
        self._run_test(TestArticulationGetSetDofVelocityTarget(self, DeviceParams(True, True)))

    def test_articulation_get_set_dof_actuation_forces_cc(self):
        self._run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(False, False)))

    def test_articulation_get_set_dof_actuation_forces_gc(self):
        self._run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(True, False)))

    def test_articulation_get_set_dof_actuation_forces_gg(self):
        self._run_test(TestArticulationGetSetDofActuationForces(self, DeviceParams(True, True)))

    def test_articulation_get_set_applied_forces_global_cc(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(False, False), global_frame=True))

    def test_articulation_get_set_applied_forces_global_gc(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, False), global_frame=True))

    def test_articulation_get_set_applied_forces_global_gg(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, True), global_frame=True))

    def test_articulation_get_set_applied_forces_local_cc(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(False, False), global_frame=False))

    def test_articulation_get_set_applied_forces_local_gc(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, False), global_frame=False))

    def test_articulation_get_set_applied_forces_local_gg(self):
        self._run_test(TestArticulationGetSetAppliedForces(self, DeviceParams(True, True), global_frame=False))

    def test_articulation_get_set_link_gravities_cc(self):
        self._run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(False, False)))

    def test_articulation_get_set_link_gravities_gc(self):
        self._run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(True, False)))

    def test_articulation_get_set_link_gravities_gg(self):
        self._run_test(TestArticulationGetSetLinkGravity(self, DeviceParams(True, True)))

    def test_articulation_dof_drive_type_cc(self):
        self._run_test(TestArticulationDofDriveType(self, DeviceParams(False, False)))

    def test_articulation_dof_drive_type_gc(self):
        self._run_test(TestArticulationDofDriveType(self, DeviceParams(True, False)))

    def test_articulation_dof_drive_type_gg(self):
        self._run_test(TestArticulationDofDriveType(self, DeviceParams(True, True)))
