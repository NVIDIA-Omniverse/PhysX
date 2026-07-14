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

import physicsUtils

_WARM_START = True
_FRONTEND = "warp"
_KEEPALIVE = False


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

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
        contacts = sim.create_rigid_contact_view(["/envs/env[0-9]/box", "/envs/env[1-3][0-9]/box"])
        self.check_rigid_contact_view(contacts, self.num_envs, 0)
        self.contacts = contacts

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 60:
            net_forces = self.contacts.get_net_contact_forces(dt)
            net_forces_np = net_forces.numpy().reshape(self.contacts.sensor_count, 3)
            expected = np.array([[0.0, 0.0, 9.81]] * self.contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(net_forces_np, expected, rtol=1e-03, atol=1e-2), "expected net contact forces")
            self.finish()


class TestRigidContactMatrix(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        self.num_envs = 32
        grid_params = GridParams(self.num_envs, 1.0)
        self.g = 10.0
        sim_params = SimParams()
        sim_params.gravity_mag = self.g
        self.device_params = device_params
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

        # ensure the surface material parameter for the friction test
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
            "/envs/env[0-9]/ball",       # ball0 to ball9
            "/envs/env[1-3][0-9]/ball"   # ball10-ball32 (self.num_envs)
        ]
        # when the sensor_pattern is a list then for each element of the list a separate filter list should be provided
        ball_filter_patterns = [
            ["/groundPlane", "/envs/env[0-9]/box"],
            ["/groundPlane", "/envs/env[1-3][0-9]/box"],
        ]
        ball_contacts = sim.create_rigid_contact_view(sensor_pattern, filter_patterns=ball_filter_patterns, max_contact_data_count=self.num_envs * 3)
        self.check_rigid_contact_view(ball_contacts, self.num_envs, len(ball_filter_patterns[-1]))
        expected_sensor_paths = [f"/envs/env{i}/ball" for i in range(self.num_envs)]
        expected_filter_paths = [["/groundPlane", f"/envs/env{i}/box"] for i in range(self.num_envs)]
        self.test_case.assertTrue(ball_contacts.sensor_paths == expected_sensor_paths)
        self.test_case.assertTrue(ball_contacts.filter_paths == expected_filter_paths)

        box_filter_patterns = [
            "/groundPlane",
            "/envs/*/ball",
        ]
        box_contacts = sim.create_rigid_contact_view("/envs/*/box", filter_patterns=box_filter_patterns, max_contact_data_count=self.num_envs * 6)
        self.check_rigid_contact_view(box_contacts, self.num_envs, len(box_filter_patterns))
        expected_sensor_paths = [f"/envs/env{i}/box" for i in range(self.num_envs)]
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

            box_force_matrix = self.box_contacts.get_contact_force_matrix(dt)
            box_force_matrix_np = box_force_matrix.numpy().reshape(self.box_contacts.sensor_count, self.box_contacts.filter_count, 3)
            box_net_forces = self.box_contacts.get_net_contact_forces(dt)
            box_net_forces_np = box_net_forces.numpy().reshape(self.box_contacts.sensor_count, 3)

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

            box_force_matrix = self.box_contacts.get_contact_force_matrix(dt)
            box_force_matrix_np = box_force_matrix.numpy().reshape(self.box_contacts.sensor_count, self.box_contacts.filter_count, 3)
            box_net_forces = self.box_contacts.get_net_contact_forces(dt)
            box_net_forces_np = box_net_forces.numpy().reshape(self.box_contacts.sensor_count, 3)

            ball_net_forces_expected = np.array([[0.0, 0.0, self.g]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_net_forces_np, ball_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_net_forces_expected = np.array([[0.0, 0.0, self.g]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_net_forces_np, box_net_forces_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            ball_force_matrix_expected = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, self.g]]] * self.ball_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(ball_force_matrix_np, ball_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            box_force_matrix_expected = np.array([[[0.0, 0.0, 2 * self.g], [0.0, 0.0, -self.g]]] * self.box_contacts.sensor_count, dtype=np.float32)
            self.test_case.assertTrue(np.allclose(box_force_matrix_np, box_force_matrix_expected, rtol=1e-03, atol=1e-2), "expected net contact forces")

            # Getting individual contact forces
            box_forces, box_points, box_normals, box_distances, box_counts, box_start_indices = self.box_contacts.get_contact_data(dt)
            ball_forces, ball_points, ball_normals, ball_distances, ball_counts, ball_start_indices = self.ball_contacts.get_contact_data(dt)
            box_force_aggregate = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            for i in range(box_counts.shape[0]):
                for j in range(box_counts.shape[1]):
                    start_idx = box_start_indices.numpy()[i, j]
                    count = box_counts.numpy()[i, j]
                    if count > 0:
                        forces = box_forces.numpy()[start_idx:start_idx + count] * box_normals.numpy()[start_idx:start_idx + count]
                        box_force_aggregate[i, j] = np.sum(forces, axis=0)

            self.test_case.assertTrue(np.allclose(box_force_matrix.numpy(), box_force_aggregate, rtol=1e-03, atol=1e-2), "aggregate of the individual contact forces equals the contact matrix value")

            ball_force_aggregate = np.zeros((self.ball_contacts.sensor_count, self.ball_contacts.filter_count, 3))
            for i in range(ball_counts.shape[0]):
                for j in range(ball_counts.shape[1]):
                    start_idx = ball_start_indices.numpy()[i, j]
                    count = ball_counts.numpy()[i, j]
                    if count > 0:
                        forces = ball_forces.numpy()[start_idx:start_idx + count] * ball_normals.numpy()[start_idx:start_idx + count]
                        ball_force_aggregate[i, j] = np.sum(forces, axis=0)

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
            # sum across all the points
            box_friction_aggregate = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            box_friction_patch_point_average = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))

            for i in range(box_counts.shape[0]):
                for j in range(box_counts.shape[1]):
                    start_idx = box_start_indices.numpy()[i, j]
                    count = box_counts.numpy()[i, j]
                    if count > 0:
                        friction = box_frictions.numpy()[start_idx: start_idx + count]
                        box_friction_aggregate[i, j, :] = np.sum(friction, axis=0)
                        anchor_points = box_points.numpy()[start_idx: start_idx + count]
                        patch_force = np.sum(np.sqrt(friction * friction), axis=0)
                        weighted_point = np.sum(anchor_points * np.abs(friction), axis=0) / patch_force
                        box_friction_patch_point_average[i, j] = weighted_point

            net_from_matrix = np.sum(box_friction_aggregate, axis=(1))
            box_force_matrix_expected = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            # the only friction force is between the box and ground ft <= mu * fn = 0.5 * (2 * m * g) = 10 -> no slip
            box_force_matrix_expected[:, 0, 0] = -self.g
            self.test_case.assertTrue(np.allclose(box_friction_aggregate, box_force_matrix_expected, rtol=1e-02, atol=5e-1), "expected net contact forces")

            box_fricton_point_expected = np.zeros((self.box_contacts.sensor_count, self.box_contacts.filter_count, 3))
            box_fricton_point_expected[:, 0, :] = self.boxes.get_transforms().numpy()[:, 0:3] - np.array([0, 0, 0.15])
            box_fricton_point_expected[:, 1, :] = self.boxes.get_transforms().numpy()[:, 0:3] + np.array([0, 0, 0.15])
            self.test_case.assertTrue(np.allclose(box_friction_patch_point_average, box_fricton_point_expected, rtol=1e-02, atol=1e-1), "expected net contact forces")

            self.finish()


class TestRawContactData(GridTestBase):
    """Test raw contact data API that returns all contacts without filter patterns.

    Creates stacked boxes (top_box on bottom_box) to verify:
    - Contact counts are correct
    - Other body paths are correctly reported
    - Forces are reasonable
    """
    def __init__(self, test_case, device_params):
        # set up stage with stacked boxes
        self.num_envs = 4
        grid_params = GridParams(self.num_envs, 2.0)
        self.g = 10.0
        sim_params = SimParams()
        sim_params.gravity_mag = self.g
        self.device_params = device_params
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.box_mass = 1.0
        self.box_size = 0.3

        # Create bottom box - sits on ground
        bottom_box_path = self.env_template_path.AppendChild("bottom_box")
        bottom_transform = Transform((0.0, 0.0, self.box_size / 2.0))
        bottom_box = self.create_rigid_box(bottom_box_path, bottom_transform, Gf.Vec3f(self.box_size))
        mass_api = UsdPhysics.MassAPI(bottom_box)
        mass_api.GetMassAttr().Set(self.box_mass)
        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(bottom_box)
        physx_rb.CreateSleepThresholdAttr().Set(0)
        contact_api = PhysxSchema.PhysxContactReportAPI.Apply(bottom_box)
        contact_api.CreateThresholdAttr().Set(0)

        # Create top box - stacked on bottom box
        top_box_path = self.env_template_path.AppendChild("top_box")
        top_transform = Transform((0.0, 0.0, self.box_size * 1.5))
        top_box = self.create_rigid_box(top_box_path, top_transform, Gf.Vec3f(self.box_size))
        mass_api = UsdPhysics.MassAPI(top_box)
        mass_api.GetMassAttr().Set(self.box_mass)
        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(top_box)
        physx_rb.CreateSleepThresholdAttr().Set(0)
        contact_api = PhysxSchema.PhysxContactReportAPI.Apply(top_box)
        contact_api.CreateThresholdAttr().Set(0)

        # Create ground plane using physicsUtils
        physicsUtils.add_quad_plane(self.stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(-1), Gf.Vec3f(0.5))

    def on_start(self, sim):
        # Create contact views WITHOUT filter patterns - will use raw contact data
        self.top_box_contacts = sim.create_rigid_contact_view(
            "/envs/*/top_box",
            max_contact_data_count=self.num_envs * 10
        )
        self.check_rigid_contact_view(self.top_box_contacts, self.num_envs, 0)
        self.test_case.assertEqual(self.top_box_contacts.filter_count, 0, "No filters should be specified for top boxes")

        self.bottom_box_contacts = sim.create_rigid_contact_view(
            "/envs/*/bottom_box",
            max_contact_data_count=self.num_envs * 10
        )
        self.check_rigid_contact_view(self.bottom_box_contacts, self.num_envs, 0)
        self.test_case.assertEqual(self.bottom_box_contacts.filter_count, 0, "No filters should be specified for bottom boxes")

    def on_physics_step(self, sim, stepno, dt):
        if stepno < 60:
            return

        if stepno == 60:
            # Get raw contact data for top boxes
            (forces, points, normals, separations, counts, start_indices,
             other_actor_ids) = self.top_box_contacts.get_raw_contact_data(dt)
            counts_np = counts.numpy().flatten()
            start_indices_np = start_indices.numpy().flatten()
            forces_np = forces.numpy().flatten()
            normals_np = normals.numpy().reshape(-1, 3)
            other_actor_ids_np = other_actor_ids.numpy().flatten()

            def get_sensor_slice(start_indices, counts, sensor_idx):
                start = int(start_indices[sensor_idx])
                count = int(counts[sensor_idx])
                return start, count

            def compute_net_force(forces, normals, start, count):
                """Compute net contact force vector: sum(force * normal)."""
                if count == 0:
                    return np.zeros(3)
                slc = slice(start, start + count)
                force_vectors = forces[slc, np.newaxis] * normals[slc]
                return np.sum(force_vectors, axis=0)

            # Verify all top box sensors have contacts
            self.test_case.assertTrue(np.all(counts_np > 0),
                f"All top box sensors should have contacts. Counts: {counts_np}")

            # Verify other actor paths using batch get_other_actor_paths_from_ids
            start, count = get_sensor_slice(start_indices_np, counts_np, 0)
            if count > 0:
                self.test_case.assertNotEqual(other_actor_ids_np[start], 0,
                    "Should return non-zero actor IDs")
                ids_cpu = wp.array(other_actor_ids_np[start:start + count], dtype=wp.uint64, device="cpu")
                paths = self.top_box_contacts.get_other_actor_paths_from_ids(ids_cpu)
                has_bottom_box = any("bottom_box" in p for p in paths if p)
                self.test_case.assertTrue(has_bottom_box,
                    f"Top box should contact bottom_box. Paths: {paths}")

            # Get raw contact data for bottom boxes
            (forces2, points2, normals2, separations2, counts2, start_indices2,
             other_actor_ids2) = self.bottom_box_contacts.get_raw_contact_data(dt)

            counts2_np = counts2.numpy().flatten()
            start_indices2_np = start_indices2.numpy().flatten()
            forces2_np = forces2.numpy().flatten()
            normals2_np = normals2.numpy().reshape(-1, 3)
            other_actor_ids2_np = other_actor_ids2.numpy().flatten()

            # Verify all bottom box sensors have contacts
            self.test_case.assertTrue(np.all(counts2_np > 0),
                f"All bottom box sensors should have contacts. Counts: {counts2_np}")

            # Verify other actor paths using batch get_other_actor_paths_from_ids
            start2, count2 = get_sensor_slice(start_indices2_np, counts2_np, 0)
            if count2 > 0:
                self.test_case.assertNotEqual(other_actor_ids2_np[start2], 0,
                    "Should return non-zero actor IDs")
                ids2_cpu = wp.array(other_actor_ids2_np[start2:start2 + count2], dtype=wp.uint64, device="cpu")
                paths2 = self.bottom_box_contacts.get_other_actor_paths_from_ids(ids2_cpu)
                has_ground = any("groundPlane" in p or "Ground" in p for p in paths2 if p)
                has_top_box = any("top_box" in p for p in paths2 if p)
                self.test_case.assertTrue(has_ground or has_top_box,
                    f"Bottom box should contact ground and/or top_box. Paths: {paths2}")

            # Verify contact forces using Newton's second law
            expected_weight = self.box_mass * self.g

            # Top box net force
            start, count = get_sensor_slice(start_indices_np, counts_np, 0)
            if count > 0:
                net_force = compute_net_force(forces_np, normals_np, start, count)
                self.test_case.assertAlmostEqual(net_force[2], expected_weight, delta=expected_weight * 0.5,
                    msg=f"Top box net Z force should be ~{expected_weight}, got {net_force[2]}")

            # Bottom box net force
            start2, count2 = get_sensor_slice(start_indices2_np, counts2_np, 0)
            if count2 > 0:
                net_force2 = compute_net_force(forces2_np, normals2_np, start2, count2)
                self.test_case.assertAlmostEqual(net_force2[2], expected_weight, delta=expected_weight * 0.5,
                    msg=f"Bottom box net Z force should be ~{expected_weight}, got {net_force2[2]}")

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
        envs_prim = self.stage.GetPrimAtPath(self.env_template_path)

        actor_path = self.env_template_path.AppendChild("box")
        transform = Transform((0.0, 0.0, 0.5))
        box_size = 0.3
        box = self.create_rigid_box(actor_path, transform, Gf.Vec3f(box_size, box_size, box_size))

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        ant_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, .0, 2.0))
        self.create_actor_from_asset(ant_path, transform, asset_path)
        ant_prim = self.stage.GetPrimAtPath(ant_path)

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

        self.cube_contact_filtered = sim.create_rigid_contact_view("/envs/*/box", filter_patterns=["/groundPlane", "/envs/*/ant/torso"])
        self.check_rigid_contact_view(self.cube_contact_filtered, self.num_envs, 2)

        self.ant_contact = sim.create_rigid_contact_view("/envs/*/ant/*")
        self.check_rigid_contact_view(self.ant_contact, self.num_envs * 9, 0)

        self.ant_contact_filtered = sim.create_rigid_contact_view("/envs/*/ant/torso", filter_patterns=["/groundPlane", "/envs/*/box"])
        self.check_rigid_contact_view(self.ant_contact_filtered, self.num_envs, 2)

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 100:
            net_forces = self.cubes_contact.get_net_contact_forces(dt).numpy().reshape(self.cubes_contact.sensor_count, 3)
            expected = np.array([[0.0, 0.0, 9.81]] * self.cubes_contact.sensor_count, dtype=np.float32)

            # match the sensor names from rigid contact view with the articulation
            ant_net_forces = self.ant_contact.get_net_contact_forces(dt).numpy().reshape(self.num_envs, self.ants.max_links, 3)
            ant_net_forces = ant_net_forces.reshape(self.ant_contact.sensor_count, 3)
            sensor_names = np.array(self.ant_contact.sensor_names)
            for key, value in self.ants.shared_metatype.link_indices.items():
                sensor_indices = np.where(sensor_names == key)

            box_contact_matrix = self.cube_contact_filtered.get_contact_force_matrix(dt).numpy().reshape(self.num_envs, 2, 3)
            torso_contact_matrix = self.ant_contact_filtered.get_contact_force_matrix(dt).numpy().reshape(self.num_envs, 2, 3)

            self.test_case.assertTrue(np.allclose(torso_contact_matrix[:, 1, :], -box_contact_matrix[:, 1, :], rtol=1e-02, atol=1e-2), "expected net contact forces")
            self.test_case.assertTrue(np.allclose(net_forces, expected, rtol=1e-02, atol=1e-2), "expected net contact forces")

            self.finish()


class TestRigidContactPerfTest(GridTestBase):
    def __init__(self, test_case, device_params):
        self.num_envs = 1
        self.num_filters_per_env = 4096
        self.num_sensors = 16
        grid_params = GridParams(self.num_envs, 100.0)
        self.g = 10.0
        sim_params = SimParams()
        sim_params.gravity_mag = self.g
        self.device_params = device_params
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.physx_scene.GetGpuMaxRigidContactCountAttr().Set(1000000)
        ball_mass = 1.0
        box_mass = 1.0

        grid_num = np.floor(np.sqrt(self.num_sensors))
        for i in range(self.num_sensors):
            box_path = self.env_template_path.AppendChild(f"box_{i}")
            box_transform = Transform((i % grid_num - grid_num / 2, i // grid_num - grid_num / 2, 0.4))
            box = self.create_rigid_box(box_path, box_transform, Gf.Vec3f(0.3, 0.3, 0.3))
            box_mass_api = UsdPhysics.MassAPI(box)
            box_mass_api.GetMassAttr().Set(box_mass)
            box_physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(box)
            box_physx_rb.CreateSleepThresholdAttr().Set(0)
            box_contact_sensor = PhysxSchema.PhysxContactReportAPI.Apply(box)
            box_contact_sensor.CreateThresholdAttr().Set(0)

        grid_num = np.floor(np.sqrt(self.num_filters_per_env))
        for i in range(self.num_filters_per_env):
            ball_path = self.env_template_path.AppendChild(f"ball_{i}")
            ball_transform = Transform((i % grid_num - grid_num / 2, i // grid_num - grid_num / 2, 1.0))
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
            filter_patterns=[[f"/envs/env0/ball_{j}" for j in range(self.num_filters_per_env)]] * self.num_sensors,
            max_contact_data_count=self.num_envs * self.num_filters_per_env)
        t1 = time.time()
        print("create_rigid_contact_view : time taken", t1 - t0)
        self.check_rigid_contact_view(box_contacts, self.num_sensors, self.num_filters_per_env)
        expected_sensor_paths = [f"/envs/env0/box_{j}" for j in range(self.num_sensors)]
        expected_filter_paths = [[f"/envs/env0/ball_{j}" for j in range(self.num_filters_per_env)]] * self.num_sensors
        self.test_case.assertTrue(box_contacts.sensor_paths == expected_sensor_paths)
        self.test_case.assertTrue(box_contacts.filter_paths == expected_filter_paths)
        self.test_case.assertTrue(t1 - t0 < 1.0, "create_rigid_contact_view should be fast when input lists are actual prim paths rather than patterns")

    def on_physics_step(self, sim, stepno, dt):
        self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsContactsTests(unittest.TestCase):

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

    def test_rigid_contacts_cc(self):
        self._run_test(TestRigidContacts(self, DeviceParams(False, False)))

    def test_rigid_contacts_gc(self):
        self._run_test(TestRigidContacts(self, DeviceParams(True, False)))

    def test_rigid_contacts_gg(self):
        self._run_test(TestRigidContacts(self, DeviceParams(True, True)))

    def test_rigid_contact_matrix_cc(self):
        self._run_test(TestRigidContactMatrix(self, DeviceParams(False, False)))

    def test_rigid_contact_matrix_gc(self):
        self._run_test(TestRigidContactMatrix(self, DeviceParams(True, False)))

    def test_rigid_contact_matrix_gg(self):
        self._run_test(TestRigidContactMatrix(self, DeviceParams(True, True)))

    def test_raw_contact_data_cc(self):
        self._run_test(TestRawContactData(self, DeviceParams(False, False)))

    def test_raw_contact_data_gc(self):
        self._run_test(TestRawContactData(self, DeviceParams(True, False)))

    def test_raw_contact_data_gg(self):
        self._run_test(TestRawContactData(self, DeviceParams(True, True)))

    def test_articulation_contacts_cc(self):
        self._run_test(TestArticulationContacts(self, DeviceParams(False, False)))

    def test_articulation_contacts_gc(self):
        self._run_test(TestArticulationContacts(self, DeviceParams(True, False)))

    def test_articulation_contacts_gg(self):
        self._run_test(TestArticulationContacts(self, DeviceParams(True, True)))

    def test_rigid_contact_perf_cc(self):
        self._run_test(TestRigidContactPerfTest(self, DeviceParams(False, False)))

    def test_rigid_contact_perf_gg(self):
        self._run_test(TestRigidContactPerfTest(self, DeviceParams(True, True)))
