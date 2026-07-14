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

from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
    set_drive,
)

_WARM_START = True
_FRONTEND = "warp"
_KEEPALIVE = False


# ---------------------------------------------------------------------------
# Scenario classes — Linear DOFs (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

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
        self.railcarts.set_dof_positions(dof_pos, self.all_indices)
        self.desired_positions = dof_pos

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            dof_pos = self.railcarts.get_dof_positions()
            p = dof_pos.numpy().squeeze()
            d = self.desired_positions.numpy().squeeze()
            self.test_case.assertTrue(np.allclose(p, d, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestLinearDofVelocities(TestLinearDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        self.vmin = -2.0
        self.vmax = 2.0
        dof_vel = wp_utils.linspace(self.railcarts.count, self.vmin, self.vmax, include_end=True, device=sim.device)
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
            self.test_case.assertTrue(abs(p[0, 0] - self.pmin) < 0.01, "expected min pos")
            self.test_case.assertTrue(abs(p[-1, 0] - self.pmax) < 0.01, "expected max pos")
        if stepno >= 100:
            # check if the poles are upright and stable
            p = dof_pos.numpy()
            v = dof_pos.numpy()
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
            self.test_case.assertTrue(np.allclose(v, self.targets, rtol=1e-02, atol=1e-02), "expected velocities")
            self.finish()


# ---------------------------------------------------------------------------
# Scenario classes — Angular DOFs (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

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
            self.test_case.assertTrue(np.allclose(p, d, rtol=1e-03, atol=1e-04), "expected positions")
            self.finish()


class TestAngularDofVelocities(TestAngularDofs):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)

    def on_start_impl(self, sim):
        self.vmin = -math.pi
        self.vmax = math.pi
        dof_vel = wp_utils.linspace(self.cartpoles.count, self.vmin, self.vmax, include_end=True, device=sim.device)
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
        self.cartpoles.set_dof_positions(dof_pos, self.all_indices)

        # allocate force buffer
        self.forces = wp.zeros((self.num_envs, self.cartpoles.max_dofs), dtype=float, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        dof_pos = self.cartpoles.get_dof_positions()
        dof_vel = self.cartpoles.get_dof_velocities()

        if stepno == 1:
            # check if the initial DOF positions are in the right range
            p = dof_pos.numpy()
            self.test_case.assertTrue(abs(p[0] - self.pmin) < 0.01, "expected min pos")
            self.test_case.assertTrue(abs(p[-1] - self.pmax) < 0.01, "expected max pos")
        if stepno >= 100:
            # check if the poles are upright and stable
            p = dof_pos.numpy()
            v = dof_pos.numpy()
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
            self.test_case.assertTrue(np.allclose(v, self.targets, rtol=1e-02, atol=1e-02), "expected velocities")
            self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsLinearAngularDofTests(unittest.TestCase):

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

    # Linear DOF tests

    def test_linear_dof_positions_cc(self):
        self._run_test(TestLinearDofPositions(self, DeviceParams(False, False)))

    def test_linear_dof_positions_gc(self):
        self._run_test(TestLinearDofPositions(self, DeviceParams(True, False)))

    def test_linear_dof_positions_gg(self):
        self._run_test(TestLinearDofPositions(self, DeviceParams(True, True)))

    def test_linear_dof_velocities_cc(self):
        self._run_test(TestLinearDofVelocities(self, DeviceParams(False, False)))

    def test_linear_dof_velocities_gc(self):
        self._run_test(TestLinearDofVelocities(self, DeviceParams(True, False)))

    def test_linear_dof_velocities_gg(self):
        self._run_test(TestLinearDofVelocities(self, DeviceParams(True, True)))

    def test_linear_dof_forces_cc(self):
        self._run_test(TestLinearDofForces(self, DeviceParams(False, False)))

    def test_linear_dof_forces_gc(self):
        self._run_test(TestLinearDofForces(self, DeviceParams(True, False)))

    def test_linear_dof_forces_gg(self):
        self._run_test(TestLinearDofForces(self, DeviceParams(True, True)))

    def test_linear_dof_position_targets_cc(self):
        self._run_test(TestLinearDofPositionTargets(self, DeviceParams(False, False)))

    def test_linear_dof_position_targets_gc(self):
        self._run_test(TestLinearDofPositionTargets(self, DeviceParams(True, False)))

    def test_linear_dof_position_targets_gg(self):
        self._run_test(TestLinearDofPositionTargets(self, DeviceParams(True, True)))

    def test_linear_dof_velocity_targets_cc(self):
        self._run_test(TestLinearDofVelocityTargets(self, DeviceParams(False, False)))

    def test_linear_dof_velocity_targets_gc(self):
        self._run_test(TestLinearDofVelocityTargets(self, DeviceParams(True, False)))

    def test_linear_dof_velocity_targets_gg(self):
        self._run_test(TestLinearDofVelocityTargets(self, DeviceParams(True, True)))

    # Angular DOF tests

    def test_angular_dof_positions_cc(self):
        self._run_test(TestAngularDofPositions(self, DeviceParams(False, False)))

    def test_angular_dof_positions_gc(self):
        self._run_test(TestAngularDofPositions(self, DeviceParams(True, False)))

    def test_angular_dof_positions_gg(self):
        self._run_test(TestAngularDofPositions(self, DeviceParams(True, True)))

    def test_angular_dof_velocities_cc(self):
        self._run_test(TestAngularDofVelocities(self, DeviceParams(False, False)))

    def test_angular_dof_velocities_gc(self):
        self._run_test(TestAngularDofVelocities(self, DeviceParams(True, False)))

    def test_angular_dof_velocities_gg(self):
        self._run_test(TestAngularDofVelocities(self, DeviceParams(True, True)))

    def test_angular_dof_forces_cc(self):
        self._run_test(TestAngularDofForces(self, DeviceParams(False, False)))

    def test_angular_dof_forces_gc(self):
        self._run_test(TestAngularDofForces(self, DeviceParams(True, False)))

    def test_angular_dof_forces_gg(self):
        self._run_test(TestAngularDofForces(self, DeviceParams(True, True)))

    def test_angular_dof_position_targets_cc(self):
        self._run_test(TestAngularDofPositionTargets(self, DeviceParams(False, False)))

    def test_angular_dof_position_targets_gc(self):
        self._run_test(TestAngularDofPositionTargets(self, DeviceParams(True, False)))

    def test_angular_dof_position_targets_gg(self):
        self._run_test(TestAngularDofPositionTargets(self, DeviceParams(True, True)))

    def test_angular_dof_velocity_targets_cc(self):
        self._run_test(TestAngularDofVelocityTargets(self, DeviceParams(False, False)))

    def test_angular_dof_velocity_targets_gc(self):
        self._run_test(TestAngularDofVelocityTargets(self, DeviceParams(True, False)))

    def test_angular_dof_velocity_targets_gg(self):
        self._run_test(TestAngularDofVelocityTargets(self, DeviceParams(True, True)))
