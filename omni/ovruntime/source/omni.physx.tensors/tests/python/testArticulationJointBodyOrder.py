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

from pxr import Gf, UsdPhysics, PhysxSchema

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)

_WARM_START = True
_FRONTEND = "warp"


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

        self.atol = 1e-05

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
            for env in range(1, 3):
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(1.0)
                jointStateAPI.CreateVelocityAttr().Set(2.0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(30.0)
                jointStateAPI.CreateVelocityAttr().Set(40.0)

    def apply_drive_api(self):
        for i in range(self.num_envs):
            for env in range(1, 3):
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
        self.sim = sim
        num_dof = 2

        if stepno == 5:
            dof_limits_1 = self.cartpoles_1.get_dof_limits().numpy().reshape((self.cartpoles_1.count, num_dof, 2)).copy()
            dof_limits_2 = self.cartpoles_2.get_dof_limits().numpy().reshape((self.cartpoles_2.count, num_dof, 2)).copy()
            self.test_case.assertTrue(np.allclose(dof_limits_1, dof_limits_2, rtol=1e-03, atol=self.atol), "similar dof limits")

            submitted_dof_limits_1 = dof_limits_1 + 0.1
            submitted_dof_limits_2 = dof_limits_2 + 0.1

            self.cartpoles_1.set_dof_limits(wp.from_numpy(submitted_dof_limits_1, dtype=wp.float32, device="cpu"), wp_utils.arange(self.cartpoles_1.count, device="cpu"))
            self.cartpoles_1.set_dof_limits(wp.from_numpy(submitted_dof_limits_2, dtype=wp.float32, device="cpu"), wp_utils.arange(self.cartpoles_2.count, device="cpu"))

            new_dof_limits_1 = self.cartpoles_1.get_dof_limits().numpy().reshape((self.cartpoles_1.count, num_dof, 2)).copy()
            new_dof_limits_2 = self.cartpoles_2.get_dof_limits().numpy().reshape((self.cartpoles_2.count, num_dof, 2)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_limits_1, new_dof_limits_1, rtol=1e-03, atol=self.atol), "similar dof limits")
            self.finish()


class TestArticulationJointBodyOrderPosition(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 5:
            dof_positions_1 = self.cartpoles_1.get_dof_positions().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_positions_2 = self.cartpoles_2.get_dof_positions().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_positions_1, dof_positions_2, rtol=1e-03, atol=self.atol), "similar dof positions sign")

            submitted_dof_positions_1 = dof_positions_1 + 1
            submitted_dof_positions_2 = dof_positions_2 + 1

            self.cartpoles_1.set_dof_positions(self.to_warp(submitted_dof_positions_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_positions(self.to_warp(submitted_dof_positions_2), self.cartpoles_2_indices)

            new_dof_positions_1 = self.cartpoles_1.get_dof_positions().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_positions_2 = self.cartpoles_2.get_dof_positions().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_positions_1, new_dof_positions_1, rtol=1e-03, atol=self.atol), "similar dof positions sign")
            self.finish()


class TestArticulationJointBodyOrderVelocity(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 5:
            dof_velocities_1 = self.cartpoles_1.get_dof_velocities().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_velocities_2 = self.cartpoles_2.get_dof_velocities().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_velocities_1, dof_velocities_2, rtol=1e-03, atol=self.atol), "similar dof velocities sign")

            submitted_dof_velocities_1 = dof_velocities_1 + 1
            submitted_dof_velocities_2 = dof_velocities_2 + 1

            self.cartpoles_1.set_dof_velocities(self.to_warp(submitted_dof_velocities_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_velocities(self.to_warp(submitted_dof_velocities_2), self.cartpoles_2_indices)

            new_dof_velocities_1 = self.cartpoles_1.get_dof_velocities().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_velocities_2 = self.cartpoles_2.get_dof_velocities().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_velocities_1, new_dof_velocities_1, rtol=1e-03, atol=self.atol), "similar dof velocities sign")
            self.finish()


class TestArticulationJointBodyOrderPositionTarget(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 5:
            dof_positions_1 = self.cartpoles_1.get_dof_position_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_positions_2 = self.cartpoles_2.get_dof_position_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_positions_1, dof_positions_2, rtol=1e-03, atol=self.atol), "similar dof position targets sign")

            submitted_dof_positions_1 = dof_positions_1 + 1
            submitted_dof_positions_2 = dof_positions_2 + 1

            self.cartpoles_1.set_dof_position_targets(self.to_warp(submitted_dof_positions_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_position_targets(self.to_warp(submitted_dof_positions_2), self.cartpoles_2_indices)

            new_dof_positions_1 = self.cartpoles_1.get_dof_position_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_positions_2 = self.cartpoles_2.get_dof_position_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_positions_1, new_dof_positions_1, rtol=1e-03, atol=self.atol), "similar dof position targets sign")
            self.finish()


class TestArticulationJointBodyOrderVelocityTarget(TestArticulationJointBodyOrder):
    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 5:
            dof_velocity_targets_1 = self.cartpoles_1.get_dof_velocity_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_velocity_targets_2 = self.cartpoles_2.get_dof_velocity_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_velocity_targets_1, dof_velocity_targets_2, rtol=1e-03, atol=self.atol), "similar dof velocity targets sign")

            submitted_dof_velocity_targets_1 = dof_velocity_targets_1 + 1
            submitted_dof_velocity_targets_2 = dof_velocity_targets_2 + 1

            self.cartpoles_1.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_velocity_targets(self.to_warp(submitted_dof_velocity_targets_2), self.cartpoles_2_indices)

            new_dof_velocity_targets_1 = self.cartpoles_1.get_dof_velocity_targets().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_velocity_targets_2 = self.cartpoles_2.get_dof_velocity_targets().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_velocity_targets_1, new_dof_velocity_targets_1, rtol=1e-03, atol=self.atol), "similar dof velocity targets sign")
            self.finish()


class TestArticulationJointBodyOrderDofForce(TestArticulationJointBodyOrder):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            for env in [1, 2]:
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 4:
            dof_actuation_forces_1 = self.cartpoles_1.get_dof_actuation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_actuation_forces_2 = self.cartpoles_2.get_dof_actuation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_actuation_forces_1, dof_actuation_forces_2, rtol=1e-03, atol=self.atol), "similar dof actuation forces")

            submitted_dof_actuation_forces_1 = dof_actuation_forces_1 + 1
            submitted_dof_actuation_forces_2 = dof_actuation_forces_2 + 1

            self.cartpoles_1.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_dof_actuation_forces(self.to_warp(submitted_dof_actuation_forces_2), self.cartpoles_2_indices)

            new_dof_actuation_forces_1 = self.cartpoles_1.get_dof_actuation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            new_dof_actuation_forces_2 = self.cartpoles_2.get_dof_actuation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(new_dof_actuation_forces_1, new_dof_actuation_forces_1, rtol=1e-03, atol=self.atol), "similar dof actuation forces")

            dof_gravity_forces_3 = self.cartpoles_1.get_gravity_compensation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_gravity_forces_4 = self.cartpoles_2.get_gravity_compensation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_gravity_forces_3, dof_gravity_forces_4, rtol=1e-03, atol=self.atol), "similar generalized gravity forces")

            dof_coriolis_and_centrifugal_forces_3 = self.cartpoles_1.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            dof_coriolis_and_centrifugal_forces_4 = self.cartpoles_2.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(dof_coriolis_and_centrifugal_forces_3, dof_coriolis_and_centrifugal_forces_4, rtol=1e-03, atol=self.atol), "similar coriolis and centrifugal forces")

            self.finish()


class TestArticulationJointBodyOrderLinkForce(TestArticulationJointBodyOrder):
    def __init__(self, test_case, device_params):
        super().__init__(test_case, device_params)
        target_p_pole = 45
        target_p_cart = 1
        for i in range(self.num_envs):
            for env in [1, 2]:
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"), "angular")
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)
                # negating the target signs to avoid the mirroring artifacts
                target = target_p_cart if env == 1 else -target_p_cart
                driveAPI = UsdPhysics.DriveAPI.Apply(self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"), "linear")
                driveAPI.CreateTargetPositionAttr(target)
                driveAPI.CreateTargetVelocityAttr(0.0)
                target = target_p_pole if env == 1 else -target_p_pole
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
        body_0_xform = view.get_link_transforms().numpy()[i, body_0_tensor_idx, :]
        body_1_xform = view.get_link_transforms().numpy()[i, body_1_tensor_idx, :]
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
        body_1_xform_p = body_1_link_p + Gf.Vec3f(Gf.Rotation(body_1_link_q).TransformDir(Gf.CompMult(body_1_scale, body_1_p)))
        body_0_xform_p = body_0_link_p + Gf.Vec3f(Gf.Rotation(body_0_link_q).TransformDir(Gf.CompMult(body_0_scale, body_0_p)))
        d_global = body_0_xform_p - body_1_xform_p
        F = Gf.Vec3f(*(link_forces[i, j, 0:3].tolist()))
        T = Gf.Vec3f(*(link_forces[i, j, 3:6].tolist()))
        d = Gf.Vec3f(Gf.Rotation(body_1_link_q * body_1_q).GetInverse().TransformDir(d_global))  # distance in body_1 link joint frame
        T -= Gf.Cross(d, F)
        J = Gf.Rotation(body_0_link_q * body_0_q).GetInverse() * Gf.Rotation(body_1_link_q * body_1_q)
        link_forces[i, j, 0:3] = J.TransformDir(F)
        link_forces[i, j, 3:6] = J.TransformDir(T)

    def on_physics_step(self, sim, stepno, dt):
        self.sim = sim
        num_dof = 2
        if stepno == 5:
            projected_forces_1 = self.cartpoles_1.get_dof_projected_joint_forces().numpy().reshape((self.cartpoles_1.count, num_dof)).copy()
            projected_forces_2 = self.cartpoles_2.get_dof_projected_joint_forces().numpy().reshape((self.cartpoles_2.count, num_dof)).copy()
            self.test_case.assertTrue(np.allclose(projected_forces_1, -projected_forces_2, rtol=1e-03, atol=self.atol), "similar projected dof forces")

            # joint child local forces
            link_forces_1 = self.cartpoles_1.get_link_incoming_joint_force().numpy().reshape((self.cartpoles_1.count, 3, 6)).copy()
            link_forces_2 = self.cartpoles_2.get_link_incoming_joint_force().numpy().reshape((self.cartpoles_2.count, 3, 6)).copy()

            for i in range(self.num_envs):
                cart_joint_2 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/cartJoint" % i)
                self.transform_to_physx_child(self.cartpoles_2, cart_joint_2, link_forces_2, i, 1)

                pole_joint_2 = UsdPhysics.RevoluteJoint.Get(self.stage, "/envs/env%d/cartpole2/poleJoint" % i)
                self.transform_to_physx_child(self.cartpoles_2, pole_joint_2, link_forces_2, i, 2)

            self.test_case.assertTrue(np.allclose(link_forces_1, -link_forces_2, rtol=0.001, atol=0.001 * np.linalg.norm(link_forces_1)), "similar link forces in global frame")
            self.finish()


class PhysxTensorsArticulationJointBodyOrderTests(unittest.TestCase):

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

    def test_articulation_joint_body_order_dof_limits_cc(self):
        self._run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_limits_gc(self):
        self._run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_limits_gg(self):
        self._run_test(TestArticulationJointBodyOrderLimits(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_dof_positions_cc(self):
        self._run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_positions_gc(self):
        self._run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_positions_gg(self):
        self._run_test(TestArticulationJointBodyOrderPosition(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_dof_velocities_cc(self):
        self._run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_velocities_gc(self):
        self._run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_velocities_gg(self):
        self._run_test(TestArticulationJointBodyOrderVelocity(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_dof_position_targets_cc(self):
        self._run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_position_targets_gc(self):
        self._run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_position_targets_gg(self):
        self._run_test(TestArticulationJointBodyOrderPositionTarget(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_dof_velocity_targets_cc(self):
        self._run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_velocity_targets_gc(self):
        self._run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_velocity_targets_gg(self):
        self._run_test(TestArticulationJointBodyOrderVelocityTarget(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_dof_forces_cc(self):
        self._run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_dof_forces_gc(self):
        self._run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_dof_forces_gg(self):
        self._run_test(TestArticulationJointBodyOrderDofForce(self, DeviceParams(True, True)))

    def test_articulation_joint_body_order_link_forces_cc(self):
        self._run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(False, False)))

    def test_articulation_joint_body_order_link_forces_gc(self):
        self._run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(True, False)))

    def test_articulation_joint_body_order_link_forces_gg(self):
        self._run_test(TestArticulationJointBodyOrderLinkForce(self, DeviceParams(True, True)))
