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


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

class TestArtJointFreeMotionToLimitMotion(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16, 5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")

        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        xform_prim = xform.GetPrim()
        root_link_path = actor_path.AppendChild("RootLink")
        rigidBody0Path = root_link_path.AppendChild("rigidBody0")

        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(xform_prim)

        # Create rigid body 0
        rigidBodyXform = UsdGeom.Xform.Define(self.stage, rigidBody0Path)
        rigidBodyPrim = rigidBodyXform.GetPrim()
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(rigidBodyPrim)
        rigidBodyAPI.CreateRigidBodyEnabledAttr(True)
        massAPI = UsdPhysics.MassAPI.Apply(rigidBodyPrim)
        massAPI.CreateMassAttr(1.0)
        massAPI.CreateDiagonalInertiaAttr(Gf.Vec3f(1.0, 1.0, 1.0))

        # Create rigid body 1
        rigidBody1Path = root_link_path.AppendChild("rigidBody1")
        rigidBodyXform = UsdGeom.Xform.Define(self.stage, rigidBody1Path)
        rigidBodyPrim = rigidBodyXform.GetPrim()
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(rigidBodyPrim)
        rigidBodyAPI.CreateRigidBodyEnabledAttr(True)
        massAPI = UsdPhysics.MassAPI.Apply(rigidBodyPrim)
        massAPI.CreateMassAttr(1.0)
        massAPI.CreateDiagonalInertiaAttr(Gf.Vec3f(1.0, 1.0, 1.0))

        # Create a D6 joint between the two prims
        d6JointPath = root_link_path.AppendChild("d6Joint")
        d6Joint = UsdPhysics.Joint.Define(self.stage, d6JointPath)
        d6Joint.CreateBody0Rel().SetTargets([rigidBody0Path])
        d6Joint.CreateBody1Rel().SetTargets([rigidBody1Path])
        d6JointPrim = d6Joint.GetPrim()

        # Create a fixed joint between the root link and the world.
        fixedJointPath = root_link_path.AppendChild("FixedJoint")
        fixedJoint = UsdPhysics.FixedJoint.Define(self.stage, fixedJointPath)
        fixedJoint.CreateBody0Rel().AddTarget(rigidBody0Path)

        # Create a spherical joint by locking the translational axes
        lockedAxes = [UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY, UsdPhysics.Tokens.transZ]
        for i in range(3):
            limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, lockedAxes[i])
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

        self.d6JointPrim = d6JointPrim

    def on_start(self, sim):
        self.artivView = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(self.artivView, self.num_envs, 2, 3, True)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            limitAPI = UsdPhysics.LimitAPI.Apply(self.d6JointPrim, UsdPhysics.Tokens.rotX)
            limits = np.zeros((self.num_envs, 3, 2))
            limits[:, :, 0] = -0.1
            limits[:, :, 1] = 0.1
            all_indices = wp_utils.arange(self.num_envs)
            wp_limits = wp.from_numpy(limits, dtype=wp.float32, device="cpu")

            # Attempt to change state of rotX to limited.
            # In Kit, this triggers a warning message:
            #   "setDofLimits - Cannot update articulation joint limits because
            #    the joint was not initially configured with limits."
            # In standalone mode we just verify the call doesn't crash.
            self.artivView.set_dof_limits(wp_limits, all_indices)

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
        self.actor_path = actor_path

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
        self.actor_path = actor_path

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


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsMiscTests(unittest.TestCase):

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

    def test_prim_deletion_cc(self):
        self._run_test(TestUsdPrimDeletion(self, DeviceParams(False, False)))

    def test_sim_view_invalidate_cc(self):
        self._run_test(TestSimViewInvalidate(self, DeviceParams(False, False)))

    def test_sim_view_invalidate_gc(self):
        self._run_test(TestSimViewInvalidate(self, DeviceParams(True, False)))

    def test_sim_view_invalidate_gg(self):
        self._run_test(TestSimViewInvalidate(self, DeviceParams(True, True)))

    def test_articulation_joint_free_motion_to_limited_motion(self):
        self._run_test(TestArtJointFreeMotionToLimitMotion(self, DeviceParams(False, False)))
