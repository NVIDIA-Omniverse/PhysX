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

from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom, UsdShade

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)
from omni.physx.scripts import physicsUtils

_WARM_START = True
_FRONTEND = "warp"


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

class TestJointForceDofProjection(GridTestBase):
    def __init__(self, test_case, device_params, free_rotation_axis, dof_torque_multiplier, free_torsional_axis=False, free_all_axes=False):
        self.gravityMag = 10.0
        self.num_envs = 16
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
        self.rel_tol = 0.04
        self.set_camera_properties(Gf.Vec3f(0, -5, 1))
        # add pendulum
        self.pendulum_path = self.env_template_path.AppendChild("pendulum")
        self.transform = Transform((0.0, 0.0, 0.0))
        self.link_mass = 1.0
        self.link_half_length = 0.5
        self.link_height = 0.05
        self.rel_tol = 0.15
        self.D6_joint_frame_quat = Gf.Quatf(1.0)  # rotation from world frame to joint frame
        self.usd_d6_free_rot_dofs = {'rotX': False, 'rotY': False, 'rotZ': False}
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
            self.num_arti_dofs = 2
        else:
            self.num_arti_dofs = 1

        if self.free_all_axes:
            self.num_arti_dofs = 3

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
            forces[:, 0] = motor_torque
        else:
            # multiple dof cases
            if self.free_rotation_axis == "rotX":
                # y axis is aligned with the link axis (torsion axis)
                # the actuated dof is the first element regardless of if there are 2 or 3 dofs in the joint
                forces[:, 0] = motor_torque
            elif self.free_rotation_axis == "rotY":
                # x axis is aligned with the link axis (torsion axis)
                # if there are 2 dofs in the joint, x, y are the free axes, and the second dof is actuated (y)
                # if there are 3 dofs in the joint, x, y, z are the free axes, and the second dof is still the actuated one (y)
                forces[:, 1] = motor_torque
            elif self.free_rotation_axis == "rotZ":
                # x axis is aligned with the link axis (torsion axis)
                if not self.free_all_axes:
                    # if there are only 2 dofs in the joint, x and z are the free axes so the second element is the the actuated one (z)
                    forces[:, 1] = motor_torque
                else:
                    # if there are 3 dofs in the joint, x, y and z are the free axes so the third element is the the actuated one (z)
                    forces[:, 2] = motor_torque
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
        dof_pos = wp_utils.fill_float32(self.pendulums.count * self.pendulums.max_dofs, 0.0, device=sim.device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=sim.device)
        self.pendulums.set_dof_positions(dof_pos, self.all_indices)
        forces = self.set_up_actuated_dof_force(self.pendulums.max_dofs, self.motor_torque)
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=sim.device)
        self.pendulums.set_dof_actuation_forces(self.applied_dof_forces, self.all_indices)
        self.on_start_impl()

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 1:
            joint_forces = self.pendulums.get_link_incoming_joint_force().numpy()
            self.test_case.assertTrue((np.abs(joint_forces - self.reference_forces_rep) <= self.rel_tol * self.reference_forces_mag).all(), "correct joint force")

            dof_forces = self.pendulums.get_dof_projected_joint_forces()
            self.test_case.assertTrue(np.allclose(dof_forces.numpy().flatten(), self.applied_dof_forces.numpy().flatten(), atol=0.1), "motion projected forces similar to actuation forces")

            self.test_case.assertTrue((np.abs(joint_forces[:, -1, :] - self.reference_forces_rep[:, -1, :]) <= self.rel_tol * self.reference_forces_mag[:, -1, :]).all(),
                                      "TF sensor forces are similar to constraint forces")
            self.finish()


# configures the articulation with a single link
class TestJointForceDofProjectionSingleLink(TestJointForceDofProjection):
    def configure_test(self):
        self.link_paths = self.create_custom_pendulum_articulation(self.pendulum_path,
                                                                    self.transform,
                                                                    link_half_length=self.link_half_length,
                                                                    link_height=self.link_height,
                                                                    link_mass=self.link_mass,
                                                                    add_fixed_joint_link=False,
                                                                    joint_type="D6",
                                                                    rotational_joint_frame_quat=self.D6_joint_frame_quat,
                                                                    usd_d6_free_rot_dof=self.usd_d6_free_rot_dofs,
                                                                    fixed_joint_frame_quat=None)

        actor_path = self.env_template_path.AppendChild("ball")
        ball_size = 0.1
        distance = self.link_half_length * 2
        transform = Transform((distance, 0.0, -ball_size - self.link_height))
        ball = self.create_rigid_ball(actor_path, transform, ball_size)
        self.fix_collider_and_link_properties(ball, self.stage.GetPrimAtPath(self.pendulum_path), self.stage.GetPrimAtPath(self.link_paths["child"]))

        mxg = self.link_mass * self.gravityMag
        # clock-wise is positive
        self.motor_torque = self.dof_torque_multiplier * self.link_mass * self.gravityMag * self.link_half_length
        if self.motor_torque > -1 * self.link_mass * self.gravityMag * self.link_half_length:
            contact_force = mxg / 2.0 + self.motor_torque / (2 * self.link_half_length)
        else:
            contact_force = 0

        # T = - 1 x m x g x L will balance the gravity weight and make the contact force go to zero
        # T = 0 results in the ball contact and D6 joint supporting half of the weight each due to symmetry
        # T = + 1 x m x g x L  leads to contact supporting all the weight

        # compute expected spatial joint forces in world frame:
        D6_joint_force_W = Gf.Vec3f(0)
        D6_joint_torque_W = Gf.Vec3f(0)

        # forces in childs attachment
        D6_joint_force_W[2] = mxg - contact_force
        D6_joint_torque_W[1] = self.motor_torque

        # link rotations
        child_rotation = self.stage.GetPrimAtPath(self.link_paths["child"]).GetAttribute('xformOp:orient').Get()
        # transform into joint frames:
        D6_joint_force_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_force_W)
        D6_joint_torque_J = Gf.Rotation((self.D6_joint_frame_quat * child_rotation).GetInverse()).TransformDir(D6_joint_torque_W)

        self.reference_forces = np.zeros((2, 6), dtype=np.float32)
        self.reference_forces[1, 0:3] = D6_joint_force_J
        self.reference_forces[1, 3:6] = D6_joint_torque_J
        self.reference_forces_rep = np.broadcast_to(self.reference_forces, (self.num_envs, 2, 6))
        self.reference_forces_mag = np.zeros((self.num_envs, 2, 6))
        for i in range(6):
            self.reference_forces_mag[:, :, i] = np.linalg.norm(self.reference_forces_rep, axis=2)

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
                                                                    link_height=self.link_height,
                                                                    link_mass=self.link_mass,
                                                                    add_fixed_joint_link=True,
                                                                    joint_type="D6",
                                                                    rotational_joint_frame_quat=self.D6_joint_frame_quat,
                                                                    usd_d6_free_rot_dof=self.usd_d6_free_rot_dofs,
                                                                    fixed_joint_frame_quat=FT_frame_quat)

        actor_path = self.env_template_path.AppendChild("ball")
        ball_size = 0.1
        distance = self.link_half_length * 4
        transform = Transform((distance, 0.0, -ball_size - self.link_height))
        ball = self.create_rigid_ball(actor_path, transform, ball_size)

        self.fix_collider_and_link_properties(ball, self.stage.GetPrimAtPath(self.pendulum_path), self.stage.GetPrimAtPath(self.link_paths["fixed"]))

        mxg = self.link_mass * self.gravityMag
        # clock-wise is positive
        self.motor_torque = self.dof_torque_multiplier * self.link_mass * self.gravityMag * self.link_half_length
        if self.motor_torque > -4 * self.link_mass * self.gravityMag * self.link_half_length:
            contact_force = mxg + self.motor_torque / (4 * self.link_half_length)
        else:
            contact_force = 0

        # T = - 4 x m x g x L will balance the gravity weight and make the contact force go to zero
        # T = 0 results in the ball contact and D6 joint supporting half of the weight each due to symmetry
        # T = + 4 x m x g x L  leads to contact supporting all the weight

        # compute expected spatial joint forces in world frame:
        D6_joint_force_W = Gf.Vec3f(0)
        D6_joint_torque_W = Gf.Vec3f(0)
        FT_sensor_force_W = Gf.Vec3f(0)
        FT_sensor_torque_W = Gf.Vec3f(0)

        # forces in childs attachment
        D6_joint_force_W[2] = 2 * mxg - contact_force
        D6_joint_torque_W[1] = self.motor_torque
        FT_sensor_force_W[2] = mxg - contact_force
        FT_sensor_torque_W[1] = -mxg * self.link_half_length + 2 * contact_force * self.link_half_length

        # link rotations
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
        self.reference_forces_mag = np.zeros((self.num_envs, 3, 6))
        for i in range(6):
            self.reference_forces_mag[:, :, i] = np.linalg.norm(self.reference_forces_rep, axis=2)

    def on_start_impl(self):
        self.check_articulation_view(self.pendulums, self.num_envs, 3, self.num_arti_dofs, True)


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsForceProjectionTests(unittest.TestCase):

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

    # --- Single link: no contact force (dof_torque_multiplier = -1) ---

    def test_force_projection_single_link_x_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", -1))

    def test_force_projection_single_link_x_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", -1))

    def test_force_projection_single_link_x_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", -1))

    def test_force_projection_single_link_y_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", -1))

    def test_force_projection_single_link_y_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", -1))

    def test_force_projection_single_link_y_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", -1))

    def test_force_projection_single_link_z_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", -1))

    def test_force_projection_single_link_z_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", -1))

    def test_force_projection_single_link_z_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", -1))

    # --- Single link: zero dof torque (dof_torque_multiplier = 0) ---

    def test_force_projection_single_link_x_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", 0))

    def test_force_projection_single_link_x_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", 0))

    def test_force_projection_single_link_x_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", 0))

    def test_force_projection_single_link_y_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", 0))

    def test_force_projection_single_link_y_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", 0))

    def test_force_projection_single_link_y_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", 0))

    def test_force_projection_single_link_z_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", 0))

    def test_force_projection_single_link_z_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", 0))

    def test_force_projection_single_link_z_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", 0))

    # --- Single link: positive contact force and dof force (dof_torque_multiplier = +1) ---

    def test_force_projection_single_link_x_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1))

    def test_force_projection_single_link_x_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1))

    def test_force_projection_single_link_x_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1))

    def test_force_projection_single_link_y_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1))

    def test_force_projection_single_link_y_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1))

    def test_force_projection_single_link_y_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1))

    def test_force_projection_single_link_z_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1))

    def test_force_projection_single_link_z_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1))

    def test_force_projection_single_link_z_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1))

    # --- Single link: positive contact + dof force with 2 rotational DOFs ---

    def test_force_projection_single_link_x_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1, True))

    def test_force_projection_single_link_x_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1, True))

    def test_force_projection_single_link_x_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1, True))

    def test_force_projection_single_link_y_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1, True))

    def test_force_projection_single_link_y_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1, True))

    def test_force_projection_single_link_y_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1, True))

    def test_force_projection_single_link_z_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1, True))

    def test_force_projection_single_link_z_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1, True))

    def test_force_projection_single_link_z_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1, True))

    # --- Single link: positive contact + dof force with 3 rotational DOFs ---

    def test_force_projection_single_link_x_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotX", +1, True, True))

    def test_force_projection_single_link_x_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotX", +1, True, True))

    def test_force_projection_single_link_x_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotX", +1, True, True))

    def test_force_projection_single_link_y_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotY", +1, True, True))

    def test_force_projection_single_link_y_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotY", +1, True, True))

    def test_force_projection_single_link_y_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotY", +1, True, True))

    def test_force_projection_single_link_z_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(False, False), "rotZ", +1, True, True))

    def test_force_projection_single_link_z_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, False), "rotZ", +1, True, True))

    def test_force_projection_single_link_z_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionSingleLink(self, DeviceParams(True, True), "rotZ", +1, True, True))

    # --- Double link: no contact force (dof_torque_multiplier = -4) ---

    def test_force_projection_double_link_x_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", -4))

    def test_force_projection_double_link_x_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", -4))

    def test_force_projection_double_link_x_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", -4))

    def test_force_projection_double_link_y_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", -4))

    def test_force_projection_double_link_y_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", -4))

    def test_force_projection_double_link_y_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", -4))

    def test_force_projection_double_link_z_zero_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", -4))

    def test_force_projection_double_link_z_zero_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", -4))

    def test_force_projection_double_link_z_zero_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", -4))

    # --- Double link: zero dof torque (dof_torque_multiplier = 0) ---

    def test_force_projection_double_link_x_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", 0))

    def test_force_projection_double_link_x_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", 0))

    def test_force_projection_double_link_x_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", 0))

    def test_force_projection_double_link_y_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", 0))

    def test_force_projection_double_link_y_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", 0))

    def test_force_projection_double_link_y_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", 0))

    def test_force_projection_double_link_z_zero_dof_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", 0))

    def test_force_projection_double_link_z_zero_dof_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", 0))

    def test_force_projection_double_link_z_zero_dof_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", 0))

    # --- Double link: positive contact force and dof force (dof_torque_multiplier = +4) ---

    def test_force_projection_double_link_x_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4))

    def test_force_projection_double_link_x_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4))

    def test_force_projection_double_link_x_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4))

    def test_force_projection_double_link_y_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4))

    def test_force_projection_double_link_y_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4))

    def test_force_projection_double_link_y_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4))

    def test_force_projection_double_link_z_dof_contact_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4))

    def test_force_projection_double_link_z_dof_contact_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4))

    def test_force_projection_double_link_z_dof_contact_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4))

    # --- Double link: positive contact + dof force with 2 rotational DOFs ---

    def test_force_projection_double_link_x_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4, True))

    def test_force_projection_double_link_x_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4, True))

    def test_force_projection_double_link_x_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4, True))

    def test_force_projection_double_link_y_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4, True))

    def test_force_projection_double_link_y_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4, True))

    def test_force_projection_double_link_y_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4, True))

    def test_force_projection_double_link_z_dof_contact_2dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4, True))

    def test_force_projection_double_link_z_dof_contact_2dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4, True))

    def test_force_projection_double_link_z_dof_contact_2dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4, True))

    # --- Double link: positive contact + dof force with 3 rotational DOFs ---

    def test_force_projection_double_link_x_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotX", +4, True, True))

    def test_force_projection_double_link_x_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotX", +4, True, True))

    def test_force_projection_double_link_x_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotX", +4, True, True))

    def test_force_projection_double_link_y_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotY", +4, True, True))

    def test_force_projection_double_link_y_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotY", +4, True, True))

    def test_force_projection_double_link_y_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotY", +4, True, True))

    def test_force_projection_double_link_z_dof_contact_3dofs_cc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(False, False), "rotZ", +4, True, True))

    def test_force_projection_double_link_z_dof_contact_3dofs_gc(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, False), "rotZ", +4, True, True))

    def test_force_projection_double_link_z_dof_contact_3dofs_gg(self):
        self._run_test(TestJointForceDofProjectionTwoLinks(self, DeviceParams(True, True), "rotZ", +4, True, True))
