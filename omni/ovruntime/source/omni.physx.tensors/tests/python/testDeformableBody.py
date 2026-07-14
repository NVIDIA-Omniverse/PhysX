# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Deformable body tests (multiple views, element indices, rest shapes,
# simulation positions/velocities, kinematic targets, staging buffers).
# NOTE: These tests require GPU dynamics (DeviceParams(True, True)).

import os
import sys
import unittest

import numpy as np
import warp as wp

import _tensors_setup  # noqa: F401

import omni.physics.tensors
import warp_utils as wp_utils

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, UsdUtils, PhysxSchema
from omni.physx.scripts import physicsUtils
from omni.physx import get_physx_simulation_interface

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)

_WARM_START = True
_FRONTEND = "warp"

_HAS_GPU = wp.is_cuda_available()


# ---------------------------------------------------------------------------
# Scenario classes (adapted from omni.physics.tensors.tests)
# ---------------------------------------------------------------------------

class TestVolumeDeformableBodyMultipleView(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.deformable_bodies_1 = sim.create_volume_deformable_body_view("/envs/env[5-8]/deformableBody")
        self.check_deformable_body_view(self.deformable_bodies_1, 4)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            self.deformable_bodies_2 = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody",
                                                                               "/envs/env10/deformableBody",
                                                                               "/envs/env[1-2][1-5]/deformableBody"])
            self.check_deformable_body_view(self.deformable_bodies_2, self.num_envs)
            deformable_bodies_3 = sim.create_volume_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(deformable_bodies_3, 5)
        if stepno == 50:
            self.deformable_bodies_3 = sim.create_volume_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_3, 5)
            pos_1 = self.deformable_bodies_1.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_1.count, self.deformable_bodies_1.max_simulation_nodes_per_body, 3)
            pos_2 = self.deformable_bodies_2.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_2.count, self.deformable_bodies_2.max_simulation_nodes_per_body, 3)
            pos_3 = self.deformable_bodies_3.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_3.count, self.deformable_bodies_3.max_simulation_nodes_per_body, 3)
            self.test_case.assertTrue(np.allclose(pos_2[3:8], pos_3, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.test_case.assertTrue(np.allclose(pos_2[5:9], pos_1, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.finish()


class TestSurfaceDeformableBodyMultipleView(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.deformable_bodies_1 = sim.create_surface_deformable_body_view("/envs/env[5-8]/deformableBody")
        self.check_deformable_body_view(self.deformable_bodies_1, 4)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 20:
            # Patterns env[0-9] (10 envs) and env[5-9] (5 envs) overlap on
            # env5..env9, so the view is the *union* — 10 unique bodies.
            # processSurfaceDeformableBodyEntries dedups across patterns
            # by PhysX pointer, so overlap cannot push the same body twice.
            self.deformable_bodies_2 = sim.create_surface_deformable_body_view(["/envs/env[0-9]/deformableBody",
                                                                                "/envs/env[5-9]/deformableBody"])
            self.check_deformable_body_view(self.deformable_bodies_2, 10)
            deformable_bodies_3 = sim.create_surface_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(deformable_bodies_3, 5)
        if stepno == 50:
            self.deformable_bodies_3 = sim.create_surface_deformable_body_view("/envs/env[3-7]/deformableBody")
            self.check_deformable_body_view(self.deformable_bodies_3, 5)
            pos_1 = self.deformable_bodies_1.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_1.count, self.deformable_bodies_1.max_simulation_nodes_per_body, 3)
            pos_2 = self.deformable_bodies_2.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_2.count, self.deformable_bodies_2.max_simulation_nodes_per_body, 3)
            pos_3 = self.deformable_bodies_3.get_simulation_nodal_positions().numpy().reshape(self.deformable_bodies_3.count, self.deformable_bodies_3.max_simulation_nodes_per_body, 3)
            # First-occurrence order is preserved by the cross-pattern
            # dedup, so pos_2[0..9] still corresponds to env0..env9 and
            # the slice comparisons below continue to hold.
            self.test_case.assertTrue(np.allclose(pos_2[3:8], pos_3, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.test_case.assertTrue(np.allclose(pos_2[5:9], pos_1, rtol=1e-01, atol=1e-02), "similar values regardless of the view")
            self.finish()


class TestVolumeDeformableBodyElementIndices(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.volume_deformable_bodies = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody",
                                                                               "/envs/env10/deformableBody",
                                                                               "/envs/env[1-2][1-5]/deformableBody"])
        self.check_deformable_body_view(self.volume_deformable_bodies, self.num_envs)
        self.simulation_element_indices = self.volume_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_elements_per_body, 4)
        self.collision_element_indices = self.volume_deformable_bodies.get_collision_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_elements_per_body, 4)
        self.sim_mesh_paths = self.volume_deformable_bodies.simulation_mesh_prim_paths
        self.coll_mesh_paths = self.volume_deformable_bodies.collision_mesh_prim_paths
        self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.sim_mesh_paths, self.simulation_element_indices))
        self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.coll_mesh_paths, self.collision_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            simulation_element_indices = self.volume_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_elements_per_body, 4)
            collision_element_indices = self.volume_deformable_bodies.get_collision_element_indices().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_elements_per_body, 4)
            self.test_case.assertTrue((simulation_element_indices == self.simulation_element_indices).all(), "simulation mesh indices don't change with time")
            self.test_case.assertTrue((collision_element_indices == self.collision_element_indices).all(), "collision mesh indices don't change with time")
            self.finish()


class TestSurfaceDeformableBodyElementIndices(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        self.surface_deformable_bodies = sim.create_surface_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        self.check_deformable_body_view(self.surface_deformable_bodies, self.num_envs)
        self.simulation_element_indices = self.surface_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_elements_per_body, 3)
        self.sim_mesh_paths = self.surface_deformable_bodies.simulation_mesh_prim_paths
        self.test_case.assertTrue(self.check_tri_element_indices(self.stage, self.sim_mesh_paths, self.simulation_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            simulation_element_indices = self.surface_deformable_bodies.get_simulation_element_indices().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_elements_per_body, 3)
            self.test_case.assertTrue((simulation_element_indices == self.simulation_element_indices).all(), "simulation mesh indices don't change with time")
            self.finish()


class TestDeformableBodyRest(GridTestBase):
    def __init__(self, test_case, device_params, test_surface):
        self.test_surface = test_surface
        self.num_nodes_per_element = 3 if test_surface else 4
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        if self.test_surface:
            self.create_surface_deformable_body(deformable_path)
        else:
            self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        if self.test_surface:
            self.bodies_view = sim.create_surface_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        else:
            self.bodies_view = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        self.check_deformable_body_view(self.bodies_view, self.num_envs)
        self.rest_element_indices = self.bodies_view.get_rest_element_indices().numpy().reshape(self.bodies_view.count, self.bodies_view.max_simulation_elements_per_body, self.num_nodes_per_element)
        self.rest_nodal_positions = self.bodies_view.get_rest_nodal_positions().numpy().reshape(self.bodies_view.count, self.bodies_view.max_rest_nodes_per_body, 3)
        self.sim_mesh_paths = self.bodies_view.simulation_mesh_prim_paths
        if self.test_surface:
            self.test_case.assertTrue(self.check_tri_element_indices(self.stage, self.sim_mesh_paths, self.rest_element_indices))
        else:
            self.test_case.assertTrue(self.check_tet_element_indices(self.stage, self.sim_mesh_paths, self.rest_element_indices))

    def on_physics_step(self, sim, stepno, dt):
        if stepno >= 2:
            rest_element_indices = self.bodies_view.get_rest_element_indices().numpy().reshape(self.bodies_view.count, self.bodies_view.max_simulation_elements_per_body, self.num_nodes_per_element)
            rest_nodal_positions = self.bodies_view.get_rest_nodal_positions().numpy().reshape(self.bodies_view.count, self.bodies_view.max_rest_nodes_per_body, 3)
            self.test_case.assertTrue((rest_element_indices == self.rest_element_indices).all(), "rest shape indices don't change with time")
            self.test_case.assertTrue((rest_nodal_positions == self.rest_nodal_positions).all(), "rest shape positions don't change with time")
            self.finish()


class TestVolumeDeformableBodySimulationPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        # not used, just to verify collision nodal positions work
        positions = self.volume_deformable_bodies.get_collision_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_collision_nodes_per_body, 3)

    def on_physics_step(self, sim, stepno, dt):
        delta = 2.0
        middle_env = self.volume_deformable_bodies.count // 2
        first_half = np.arange(0, middle_env)
        if stepno == 100:
            self.positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals = self.positions + np.array([0, 0, delta])
            wp_pos = wp.from_numpy(set_vals, dtype=wp.float32)
            self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 101:
            self.set_positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 150:
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1] + np.array([0, 0, delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=1e-01), "set_simulation_nodal_positions works correctly.")
            self.finish()


class TestSurfaceDeformableBodySimulationPositions(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.surface_deformable_bodies = surface_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.surface_deformable_bodies.count, device=sim.device)
        # not used, just to verify collision nodal positions work
        positions = self.surface_deformable_bodies.get_collision_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_collision_nodes_per_body, 3)

    def on_physics_step(self, sim, stepno, dt):
        delta = 2.0
        middle_env = self.surface_deformable_bodies.count // 2
        first_half = np.arange(0, middle_env)
        if stepno == 100:
            self.positions = self.surface_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals = self.positions + np.array([0, 0, delta])
            wp_pos = wp.from_numpy(set_vals, dtype=wp.float32)
            self.surface_deformable_bodies.set_simulation_nodal_positions(wp_pos, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 101:
            self.set_positions = self.surface_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 150:
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1] + np.array([0, 0, delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=1e-01), "set_simulation_nodal_positions works correctly.")
            self.finish()


class TestVolumeDeformableBodySimulationVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        middle_env = self.volume_deformable_bodies.count // 2
        first_half = np.arange(0, middle_env)
        if stepno == 150:
            self.settled_velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals = self.settled_velocities + np.array([0, 0, 10.0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 151:
            self.velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 200:
            self.test_case.assertTrue((self.velocities[:middle_env-1, :, -1] > self.settled_velocities[:middle_env-1, :, -1]).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:, :, -1], self.settled_velocities[middle_env+1:, :, -1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()


class TestSurfaceDeformableBodySimulationVelocities(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_surface_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.surface_deformable_bodies = surface_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.surface_deformable_bodies.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        middle_env = self.surface_deformable_bodies.count // 2
        first_half = np.arange(0, middle_env)
        if stepno == 150:
            self.settled_velocities = self.surface_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals = self.settled_velocities + np.array([0, 0, 10.0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            self.surface_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp.from_numpy(first_half, dtype=wp.uint32))
        if stepno == 151:
            self.velocities = self.surface_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.surface_deformable_bodies.count, self.surface_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 200:
            self.test_case.assertTrue((self.velocities[:middle_env-1, :, -1] > self.settled_velocities[:middle_env-1, :, -1]).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:, :, -1], self.settled_velocities[middle_env+1:, :, -1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()


class TestVolumeDeformableBodySimulationKinematicTargets(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(10, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody"])
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.init_positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        self.top_surface_indices = self.init_positions[:, :, 2] > 1.2
        self.kinematic_targets = np.zeros((self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4))
        self.kinematic_targets[:, :, 0:3] = self.init_positions
        self.kinematic_targets[:, :, -1] = 1 - self.top_surface_indices
        kinematic_targets = wp.from_numpy(self.kinematic_targets, dtype=wp.float32)
        self.volume_deformable_bodies.set_simulation_nodal_kinematic_targets(kinematic_targets, self.wp_all_indices)

    def on_physics_step(self, sim, stepno, dt):
        Amp = 0.1
        height = Amp * np.sin(2 * np.pi * stepno / 200)
        self.current_targets = self.volume_deformable_bodies.get_simulation_nodal_kinematic_targets().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4)
        self.kinematic_targets = self.current_targets + np.array([0, 0, height, 0])
        wp_kinematic_targets = wp.from_numpy(self.kinematic_targets, dtype=wp.float32)
        self.volume_deformable_bodies.set_simulation_nodal_kinematic_targets(wp_kinematic_targets, self.wp_all_indices)
        if stepno >= 200:
            kt = self.volume_deformable_bodies.get_simulation_nodal_kinematic_targets().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 4)
            self.test_case.assertTrue((self.current_targets == kt).all(), "same set and get values.")
            positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            combined_z = np.where(self.top_surface_indices, kt[..., 2], positions[..., 2])
            combined = positions
            combined[..., 2] = combined_z
            self.test_case.assertTrue(np.allclose(combined, positions, rtol=1e-03, atol=1e-04), "kinematic nodes match the current nodes.")
            self.finish()


class TestVolumeDeformableBodyStagingBuffers(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(15, 2.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        deformable_path = self.env_template_path.AppendChild("deformableBody")
        self.create_volume_deformable_body(deformable_path)

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/deformableBody")
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
        middle_env = self.volume_deformable_bodies.count // 2
        first_half = np.arange(0, middle_env)
        self.wp_half_indices = wp.from_numpy(first_half, dtype=wp.uint32)

    def on_physics_step(self, sim, stepno, dt):
        delta = 2.0
        if stepno == 100:
            self.positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            self.settled_velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_pos = self.positions + np.array([0, 0, delta])
            set_vals = self.settled_velocities + np.array([0, 0, 10.0])
            wp_pos = wp.from_numpy(set_pos, dtype=wp.float32)
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, self.wp_half_indices)
            self.volume_deformable_bodies.set_simulation_nodal_positions(wp_pos, self.wp_half_indices)
        if stepno == 101:
            self.set_positions = self.volume_deformable_bodies.get_simulation_nodal_positions().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            self.velocities = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
        if stepno == 199:
            middle_env = self.volume_deformable_bodies.count // 2
            self.test_case.assertTrue(np.allclose(self.positions[:middle_env-1] + np.array([0, 0, delta]), self.set_positions[:middle_env-1], rtol=1e-01, atol=2e-01), "set simulation positions works correctly.")
            self.test_case.assertTrue((self.velocities[:middle_env-1, :, -1] > self.settled_velocities[:middle_env-1, :, -1]).all(), "larger overall velocities")
            self.test_case.assertTrue(np.allclose(self.velocities[middle_env+1:, :, -1], self.settled_velocities[middle_env+1:, :, -1], rtol=1e-01, atol=5e-01), "set simulation velocities works correctly.")
            self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsDeformableBodyTests(unittest.TestCase):
    """Deformable body tests — require GPU dynamics."""

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

    def test_volume_deformable_body_multiple_views_gg(self):
        self._run_test(TestVolumeDeformableBodyMultipleView(self, DeviceParams(True, True)))

    def test_volume_deformable_body_element_indices_gg(self):
        self._run_test(TestVolumeDeformableBodyElementIndices(self, DeviceParams(True, True)))

    def test_volume_deformable_body_rest_gg(self):
        self._run_test(TestDeformableBodyRest(self, DeviceParams(True, True), test_surface=False))

    def test_volume_deformable_body_simulation_positions_gg(self):
        self._run_test(TestVolumeDeformableBodySimulationPositions(self, DeviceParams(True, True)))

    def test_volume_deformable_body_simulation_velocities_gg(self):
        self._run_test(TestVolumeDeformableBodySimulationVelocities(self, DeviceParams(True, True)))

    def test_volume_deformable_body_simulation_kinematic_targets_gg(self):
        self._run_test(TestVolumeDeformableBodySimulationKinematicTargets(self, DeviceParams(True, True)))

    def test_volume_deformable_body_staging_buffers_gg(self):
        self._run_test(TestVolumeDeformableBodyStagingBuffers(self, DeviceParams(True, True)))

    def test_surface_deformable_body_multiple_views_gg(self):
        self._run_test(TestSurfaceDeformableBodyMultipleView(self, DeviceParams(True, True)))

    def test_surface_deformable_body_element_indices_gg(self):
        self._run_test(TestSurfaceDeformableBodyElementIndices(self, DeviceParams(True, True)))

    def test_surface_deformable_body_rest_gg(self):
        self._run_test(TestDeformableBodyRest(self, DeviceParams(True, True), test_surface=True))

    def test_surface_deformable_body_simulation_positions_gg(self):
        self._run_test(TestSurfaceDeformableBodySimulationPositions(self, DeviceParams(True, True)))

    def test_surface_deformable_body_simulation_velocities_gg(self):
        self._run_test(TestSurfaceDeformableBodySimulationVelocities(self, DeviceParams(True, True)))
