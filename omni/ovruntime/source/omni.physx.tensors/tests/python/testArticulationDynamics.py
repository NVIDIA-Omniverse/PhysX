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
from omni.physx.scripts import physicsUtils

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
        rt_z_step = np.linspace(0.0, 1.0, self.ants.count, dtype=np.float32)
        lt_z_step = np.repeat(rt_z_step, self.ants.max_links).reshape((self.ants.count, self.ants.max_links))
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

        # expected link velocities
        link_vels_np = np.zeros((self.ants.count, self.ants.max_links, 6))
        link_linear_z = np.repeat(root_linear_z, self.ants.max_links).reshape((self.ants.count, self.ants.max_links))
        link_vels_np[..., 2] = link_linear_z

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
            self.test_case.assertTrue(np.allclose(root_vels_np, self.expected_root_vels, rtol=1e-03, atol=1e-03), "expected root velocities")
            self.test_case.assertTrue(np.allclose(link_vels_np, self.expected_link_vels, rtol=1e-03, atol=1e-03), "expected link velocities")
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
        rc_forces = wp_utils.fill_float32(self.ants.count * self.ants.max_dofs, force_amt, device=sim.device)
        self.ants.set_dof_actuation_forces(rc_forces, rc_indices)

        if stepno == 20:
            cori_centri_forces = self.ants.get_coriolis_and_centrifugal_compensation_forces()
            self.test_case.assertTrue(cori_centri_forces.shape == (self.ants.count, self.ants.max_dofs + 6))

            applied_efforts = self.ants.get_dof_actuation_forces()
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
        rc_forces = wp_utils.fill_float32(self.ants.count * self.ants.max_dofs, force_amt, device=sim.device)
        self.ants.set_dof_actuation_forces(rc_forces, rc_indices)
        if stepno == 20:
            gravity_compensation = self.ants.get_gravity_compensation_forces()
            self.test_case.assertTrue(gravity_compensation.shape == (self.ants.count, self.ants.max_dofs + 6))
            applied_efforts = self.ants.get_dof_actuation_forces()
            self.test_case.assertTrue(np.allclose(applied_efforts.numpy().flatten(), rc_forces.numpy(), rtol=1e-03, atol=1e-03), "expected actuation forces")
            self.finish()


class TestLinkAccelerations(GridTestBase):
    def __init__(self, test_case, device_params):
        self.gravityMag = 10.0
        self.num_envs = 16
        grid_params = GridParams(self.num_envs)
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
                                                            revolute_joint_frame_quat=Gf.Quatf(1.0),
                                                            fixed_joint_frame_quat=self.fixed_joint_frame_quat)

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 2, 1, True)
        self.vmin = -math.pi
        self.vmax = math.pi
        dof_vel = wp_utils.linspace(self.pendulums.count, self.vmin, self.vmax, include_end=True, device=sim.device)
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
            self.test_case.assertTrue(np.allclose(numerical_dofAcc.flatten(), dofAcc.flatten(), atol=0.01 * np.abs(dofAcc)), "expected numerical dof acceleration")
            acc = self.pendulums.get_link_accelerations().numpy().reshape((self.num_envs, 2, 6))

            self.reference_acc = np.zeros((self.pendulums.count, 2, 6))
            self.reference_acc[:, 1, 0] = -self.link_half_length * dofAcc * np.sin(dofPos) - self.link_half_length * dofVel * dofVel * np.cos(dofPos)
            self.reference_acc[:, 1, 2] = -self.link_half_length * dofAcc * np.cos(dofPos) + self.link_half_length * dofVel * dofVel * np.sin(dofPos)
            self.reference_acc[:, 1, 4] = dofAcc

            self.test_case.assertTrue(np.allclose(acc[:, :, :3], self.reference_acc[:, :, :3], atol=0.01 * np.linalg.norm(acc[:, :, :3])), "expected link linear acceleration")
            self.test_case.assertTrue(np.allclose(acc[:, :, 3:], self.reference_acc[:, :, 3:], atol=0.01 * np.linalg.norm(acc[:, :, 3:])), "expected link angular acceleration")

        if stepno == 50:
            self.finish()

        self.pre_dof_vel = np.copy(self.pendulums.get_dof_velocities().numpy())


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


# ---------------------------------------------------------------------------
# Test case class
# ---------------------------------------------------------------------------

class PhysxTensorsArticulationDynamicsTests(unittest.TestCase):

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

    def test_articulation_root_transforms_cc(self):
        self._run_test(TestArticulationRootTransforms(self, DeviceParams(False, False)))

    def test_articulation_root_transforms_gc(self):
        self._run_test(TestArticulationRootTransforms(self, DeviceParams(True, False)))

    def test_articulation_root_transforms_gg(self):
        self._run_test(TestArticulationRootTransforms(self, DeviceParams(True, True)))

    def test_articulation_root_velocities_cc(self):
        self._run_test(TestArticulationRootVelocities(self, DeviceParams(False, False)))

    def test_articulation_root_velocities_gc(self):
        self._run_test(TestArticulationRootVelocities(self, DeviceParams(True, False)))

    def test_articulation_root_velocities_gg(self):
        self._run_test(TestArticulationRootVelocities(self, DeviceParams(True, True)))

    def test_jacobians_cc(self):
        self._run_test(TestJacobians(self, DeviceParams(False, False)))

    def test_jacobians_gc(self):
        self._run_test(TestJacobians(self, DeviceParams(True, False)))

    def test_jacobians_gg(self):
        self._run_test(TestJacobians(self, DeviceParams(True, True)))

    def test_mass_matrices_cc(self):
        self._run_test(TestMassMatrices(self, DeviceParams(False, False)))

    def test_mass_matrices_gc(self):
        self._run_test(TestMassMatrices(self, DeviceParams(True, False)))

    def test_mass_matrices_gg(self):
        self._run_test(TestMassMatrices(self, DeviceParams(True, True)))

    def test_coriolis_centrifugal_cc(self):
        self._run_test(TestCoriolisCentrifugal(self, DeviceParams(False, False)))

    def test_coriolis_centrifugal_gc(self):
        self._run_test(TestCoriolisCentrifugal(self, DeviceParams(True, False)))

    def test_coriolis_centrifugal_gg(self):
        self._run_test(TestCoriolisCentrifugal(self, DeviceParams(True, True)))

    def test_gravity_compensation_cc(self):
        self._run_test(TestGravityCompensation(self, DeviceParams(False, False)))

    def test_gravity_compensation_gc(self):
        self._run_test(TestGravityCompensation(self, DeviceParams(True, False)))

    def test_gravity_compensation_gg(self):
        self._run_test(TestGravityCompensation(self, DeviceParams(True, True)))

    def test_link_accelerations_cc(self):
        self._run_test(TestLinkAccelerations(self, DeviceParams(False, False)))

    def test_link_accelerations_gc(self):
        self._run_test(TestLinkAccelerations(self, DeviceParams(True, False)))

    def test_link_accelerations_gg(self):
        self._run_test(TestLinkAccelerations(self, DeviceParams(True, True)))

    def test_joint_friction_cc(self):
        self._run_test(TestJointFriction(self, DeviceParams(False, False)))

    def test_joint_friction_gc(self):
        self._run_test(TestJointFriction(self, DeviceParams(True, False)))

    def test_joint_friction_gg(self):
        self._run_test(TestJointFriction(self, DeviceParams(True, True)))
