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

from omni.physx.scripts import physicsUtils

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)

_WARM_START = True
_FRONTEND = "warp"


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

class TestForceTorqueSensor(GridTestBase):
    def __init__(self, test_case, device_params, rotation_axis):
        self.gravityMag = 10.0
        self.num_envs = 16
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
        self.FT_frame_quat = Gf.Quatf(1.0)
        self.revolute_joint_frame_quat = Gf.Quatf(1.0)
        if self.rotation_axis == "X":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
        if self.rotation_axis == "Z":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())

        self.link_paths = self.create_pendulum_articulation(pendulum_path,
                                                            transform,
                                                            link_half_length=self.link_half_length,
                                                            link_mass=self.link_mass,
                                                            add_fixed_joint_link=True,
                                                            revolute_joint_axis=self.rotation_axis,
                                                            revolute_joint_frame_quat=self.revolute_joint_frame_quat,
                                                            fixed_joint_frame_quat=self.FT_frame_quat)

        mxg = self.link_mass * self.gravityMag
        self.motor_torque = -4 * self.link_mass * self.gravityMag * self.link_half_length

        # compute expected spatial joint forces in world frame:
        revolute_joint_force_W = Gf.Vec3f(0)
        revolute_joint_torque_W = Gf.Vec3f(0)
        FT_sensor_force_W = Gf.Vec3f(0)
        FT_sensor_torque_W = Gf.Vec3f(0)
        # forces in childs attachment
        revolute_joint_force_W[2] = mxg - self.motor_torque / (4 * self.link_half_length)
        revolute_joint_torque_W[1] = self.motor_torque
        FT_sensor_force_W[2] = -self.motor_torque / (4 * self.link_half_length)
        FT_sensor_torque_W[1] = +mxg * self.link_half_length + self.motor_torque / 2.0

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
        self.reference_forces_mag = np.zeros((self.num_envs, 3, 6))
        for i in range(6):
            self.reference_forces_mag[:, :, i] = np.linalg.norm(self.reference_forces_rep, axis=2)

    def on_start(self, sim):
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)
        dof_pos = wp_utils.fill_float32(self.pendulums.count, 0.0, device=sim.device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.pendulums.set_dof_positions(dof_pos, self.all_indices)
        forces = np.ones((self.pendulums.count, self.pendulums.max_dofs)) * self.motor_torque
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.pendulums.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            joint_forces = self.pendulums.get_link_incoming_joint_force().numpy()
            self.test_case.assertTrue((np.abs(joint_forces - self.reference_forces_rep) <= 0.04 * self.reference_forces_mag).all(), "correct joint force")

            dof_forces = self.pendulums.get_dof_projected_joint_forces().numpy()
            self.test_case.assertTrue(np.allclose(dof_forces, self.applied_dof_forces.numpy(), rtol=0.04), "motion projected forces similar to actuation forces")
            self.test_case.assertTrue((np.abs(joint_forces[:, -1, :] - self.reference_forces_rep[:, -1, :]) <= 0.04 * self.reference_forces_mag[:, -1, :]).all(),
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
        dof_forces = self.cartpoles.get_dof_projected_joint_forces().numpy()
        self.test_case.assertTrue((np.abs(dof_forces) < 1e-4).all(), "zero dof forces")

        self.all_indices = wp_utils.arange(self.cartpoles.count, device=sim.device)
        forces = np.ones((self.cartpoles.count, self.cartpoles.max_dofs)) * 10
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.cartpoles.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            dof_forces = self.cartpoles.get_dof_projected_joint_forces().numpy()
            self.test_case.assertTrue(np.allclose(dof_forces, self.applied_dof_forces.numpy(), rtol=0.04), "motion projected forces similar to actuation forces")

            joint_forces = self.cartpoles.get_link_incoming_joint_force().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 6))
            actuation_force = dof_forces.reshape((self.cartpoles.count, self.cartpoles.max_dofs))

        if stepno == 20:
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
        drive_model_properties[:, 0, 0] = 2.0  # speed effort gradient
        drive_model_properties[:, 0, 1] = 1.0e9  # max actuator velocity
        drive_model_properties[:, 0, 2] = 2.0  # velocity dependent resistance
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
            drive_model_properties[::2, 0, 0] = 1.0e15  # speed effort gradient
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
            drive_model_properties[:, 0, 0] = 2.0  # speed effort gradient
            drive_model_properties[::2, 0, 1] = 1.0  # max actuator velocity
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
            drive_model_properties[:, 0, 1] = 1.0e9  # max actuator velocity
            drive_model_properties[::2, 0, 2] = 1.0e3  # velocity dependent resistance
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


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsForceSensorTests(unittest.TestCase):

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

    def test_force_torque_sensor_x_cc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "X"))

    def test_force_torque_sensor_x_gc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "X"))

    def test_force_torque_sensor_x_gg(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "X"))

    def test_force_torque_sensor_y_cc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "Y"))

    def test_force_torque_sensor_y_gc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "Y"))

    def test_force_torque_sensor_y_gg(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "Y"))

    def test_force_torque_sensor_z_cc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(False, False), "Z"))

    def test_force_torque_sensor_z_gc(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, False), "Z"))

    def test_force_torque_sensor_z_gg(self):
        self._run_test(TestForceTorqueSensor(self, DeviceParams(True, True), "Z"))

    def test_joint_actuation_force_cc(self):
        self._run_test(TestJointActuationForces(self, DeviceParams(False, False)))

    def test_joint_actuation_force_gc(self):
        self._run_test(TestJointActuationForces(self, DeviceParams(True, False)))

    def test_joint_actuation_force_gg(self):
        self._run_test(TestJointActuationForces(self, DeviceParams(True, True)))

    def test_link_incoming_joint_force_force_on_fixed_link_x_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "X"))

    def test_link_incoming_joint_force_force_on_fixed_link_x_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "X"))

    def test_link_incoming_joint_force_force_on_fixed_link_x_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "X"))

    def test_link_incoming_joint_force_force_on_fixed_link_y_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "Y"))

    def test_link_incoming_joint_force_force_on_fixed_link_y_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "Y"))

    def test_link_incoming_joint_force_force_on_fixed_link_y_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "Y"))

    def test_link_incoming_joint_force_force_on_fixed_link_z_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), True, "Z"))

    def test_link_incoming_joint_force_force_on_fixed_link_z_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), True, "Z"))

    def test_link_incoming_joint_force_force_on_fixed_link_z_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), True, "Z"))

    def test_link_incoming_joint_force_force_on_child_link_x_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "X"))

    def test_link_incoming_joint_force_force_on_child_link_x_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "X"))

    def test_link_incoming_joint_force_force_on_child_link_x_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "X"))

    def test_link_incoming_joint_force_force_on_child_link_y_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "Y"))

    def test_link_incoming_joint_force_force_on_child_link_y_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "Y"))

    def test_link_incoming_joint_force_force_on_child_link_y_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "Y"))

    def test_link_incoming_joint_force_force_on_child_link_z_cc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(False, False), False, "Z"))

    def test_link_incoming_joint_force_force_on_child_link_z_gc(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, False), False, "Z"))

    def test_link_incoming_joint_force_force_on_child_link_z_gg(self):
        self._run_test(TestLinkIncomingJointForce(self, DeviceParams(True, True), False, "Z"))

    def test_joint_performance_envelope_cc(self):
        self._run_test(TestJointPerformanceEnvelope(self, DeviceParams(False, False)))

    def test_joint_performance_envelope_gc(self):
        self._run_test(TestJointPerformanceEnvelope(self, DeviceParams(True, False)))

    def test_joint_performance_envelope_gg(self):
        self._run_test(TestJointPerformanceEnvelope(self, DeviceParams(True, True)))
