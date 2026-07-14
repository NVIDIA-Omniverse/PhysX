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

class TestArticulationCentroidalMomentumAndMass(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("Humanoid")
        transform = Transform((1.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((-1.0, 0.0, 0.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.humanoids = sim.create_articulation_view("/envs/*/Humanoid/torso")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            vr_ants = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
            vr_humanoids = self.humanoids.get_root_velocities().numpy().reshape((self.humanoids.count, 6)).copy()
            vr_ants[:, 3] += 0.1
            vr_humanoids[:, 3] += 0.5
            self.ants.set_root_velocities(self.to_warp(vr_ants), self.all_indices)
            self.humanoids.set_root_velocities(self.to_warp(vr_humanoids), self.all_indices)
            com_ants = self.ants.get_articulation_mass_center(True).numpy().reshape((self.ants.count, 3)).copy()
            com_humanoids = self.humanoids.get_articulation_mass_center(True).numpy().reshape((self.humanoids.count, 3)).copy()
            self.test_case.assertTrue(np.allclose(com_ants[:, 0:1], com_ants[:, 0:1] * 0, rtol=1e-1, atol=1e-1), "symmetrical articulation")
            self.test_case.assertTrue(np.allclose(com_humanoids[:, 0:1], com_humanoids[:, 0:1] * 0, rtol=1e-1, atol=1e-1), "symmetrical articulation")

        if stepno == 2:
            v_humanoids = self.humanoids.get_dof_velocities().numpy().reshape((self.humanoids.count, self.humanoids.max_dofs)).copy()
            v_ants = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            v_humanoids[:, :] += 0.5
            v_ants[:, :] += 0.1
            self.humanoids.set_dof_velocities(self.to_warp(v_humanoids), self.all_indices)
            self.ants.set_dof_velocities(self.to_warp(v_ants), self.all_indices)

        if stepno == 3:
            v_humanoids = self.humanoids.get_dof_velocities().numpy().reshape((self.humanoids.count, self.humanoids.max_dofs)).copy()
            v_ants = self.ants.get_dof_velocities().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            cmm_humanoids = self.humanoids.get_articulation_centroidal_momentum().numpy().reshape((self.humanoids.count, 6, self.humanoids.max_dofs + 7)).copy()
            cmm_ants = self.ants.get_articulation_centroidal_momentum().numpy().reshape((self.ants.count, 6, self.ants.max_dofs + 7)).copy()
            expected_cmm_humanoids = np.tile(cmm_humanoids[0], (self.humanoids.count, 1)).reshape((self.humanoids.count, 6, self.humanoids.max_dofs + 7))
            expected_cmm_ants = np.tile(cmm_ants[0], (self.ants.count, 1)).reshape((self.ants.count, 6, self.ants.max_dofs + 7))
            self.test_case.assertTrue(np.allclose(expected_cmm_humanoids, cmm_humanoids, rtol=1e-03, atol=1e-3), "all envs have similar humanoids centroidal mass")
            self.test_case.assertTrue(np.allclose(expected_cmm_ants, cmm_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants centroidal momentum")

            self.finish()


class TestHeterogeneousSceneArticulations(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "cabinet.usda")
        actor_path = self.env_template_path.AppendChild("cabinet")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "franka.usda")
        actor_path = self.env_template_path.AppendChild("franka")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.cabinet = sim.create_articulation_view("/envs/*/cabinet")
        self.franka = sim.create_articulation_view("/envs/*/franka")
        self.all_indices = wp_utils.arange(self.cabinet.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            p_cabinet = self.cabinet.get_link_transforms().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 7)).copy()
            p_franka = self.franka.get_link_transforms().numpy().reshape((self.franka.count, self.franka.max_links, 7)).copy()

            self.rp_cabinet = self.cabinet.get_root_transforms().numpy().reshape((self.cabinet.count, 7)).copy()
            self.rp_franka = self.franka.get_root_transforms().numpy().reshape((self.franka.count, 7)).copy()
            self.rp_cabinet[:, 2] -= 0.6
            self.rp_franka[:, 2] -= 0.2
            self.cabinet.set_root_transforms(self.to_warp(self.rp_cabinet), self.all_indices)
            self.franka.set_root_transforms(self.to_warp(self.rp_franka), self.all_indices)

        if stepno == 2:
            root_rel_cabinet = np.zeros((self.cabinet.count, self.cabinet.max_links, 7))
            root_rel_franka = np.zeros((self.cabinet.count, self.franka.max_links, 7))

            for i in range(self.cabinet.max_links):
                root_rel_cabinet[:, i, :] = self.rp_cabinet
            for i in range(self.franka.max_links):
                root_rel_franka[:, i, :] = self.rp_franka

            p_cabinet = self.cabinet.get_link_transforms().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 7)).copy() - root_rel_cabinet
            p_franka = self.franka.get_link_transforms().numpy().reshape((self.franka.count, self.franka.max_links, 7)).copy() - root_rel_franka
            expected_p_cabinets = np.tile(p_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_links, 7))
            expected_p_frankas = np.tile(p_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_links, 7))
            self.test_case.assertTrue(np.allclose(expected_p_cabinets, p_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet link transforms")
            self.test_case.assertTrue(np.allclose(expected_p_frankas, p_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka link transforms")

        if stepno == 3:
            p_cabinet = self.cabinet.get_dof_positions().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            p_franka = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            p_cabinet[:, :] += 0.3
            p_franka[:, :] += 0.5
            self.cabinet.set_dof_positions(self.to_warp(p_cabinet), self.all_indices)
            self.franka.set_dof_positions(self.to_warp(p_franka), self.all_indices)

        if stepno == 4:
            p_cabinet = self.cabinet.get_dof_positions().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            p_franka = self.franka.get_dof_positions().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_p_cabinets = np.tile(p_cabinet[0], (self.cabinet.count, 1))
            expected_p_frankas = np.tile(p_franka[0], (self.franka.count, 1))
            self.test_case.assertTrue(np.allclose(expected_p_cabinets, p_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet dof positions")
            self.test_case.assertTrue(np.allclose(expected_p_frankas, p_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka dof positions")

        if stepno == 5:
            v_cabinet = self.cabinet.get_link_velocities().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 6)).copy()
            v_franka = self.franka.get_link_velocities().numpy().reshape((self.franka.count, self.franka.max_links, 6)).copy()

            vr_cabinet = self.cabinet.get_root_velocities().numpy().reshape((self.cabinet.count, 6)).copy()
            vr_franka = self.franka.get_root_velocities().numpy().reshape((self.franka.count, 6)).copy()

            vr_cabinet[:, :] += 0.3
            vr_franka[:, :] += 0.5

            self.cabinet.set_root_velocities(self.to_warp(vr_cabinet), self.all_indices)
            self.franka.set_root_velocities(self.to_warp(vr_franka), self.all_indices)

        if stepno == 6:
            v_cabinet = self.cabinet.get_link_velocities().numpy().reshape((self.cabinet.count, self.cabinet.max_links, 6)).copy()
            v_franka = self.franka.get_link_velocities().numpy().reshape((self.franka.count, self.franka.max_links, 6)).copy()
            expected_v_cabinets = np.tile(v_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_links, 6))
            expected_v_frankas = np.tile(v_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_links, 6))
            self.test_case.assertTrue(np.allclose(expected_v_cabinets, v_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet link velocities")
            self.test_case.assertTrue(np.allclose(expected_v_frankas, v_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka link velocities")

        if stepno == 7:
            j_cabinet = self.cabinet.get_jacobians().numpy().reshape((self.cabinet.count, (self.cabinet.max_dofs) * 6 * (self.cabinet.max_links - 1))).copy()
            j_franka = self.franka.get_jacobians().numpy().reshape((self.franka.count, (self.franka.max_dofs) * 6 * (self.franka.max_links - 1))).copy()
            expected_j_cabinets = np.tile(j_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, (self.cabinet.max_dofs) * 6 * (self.cabinet.max_links - 1)))
            expected_j_frankas = np.tile(j_franka[0], (self.franka.count, 1)).reshape((self.franka.count, (self.franka.max_dofs) * 6 * (self.franka.max_links - 1)))
            self.test_case.assertTrue(np.allclose(expected_j_cabinets, j_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet Jacobian")
            self.test_case.assertTrue(np.allclose(expected_j_frankas, j_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka Jacobian")

        if stepno == 8:
            cc_cabinet = self.cabinet.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            cc_franka = self.franka.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_cc_cabinet = np.tile(cc_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_dofs))
            expected_cc_frankas = np.tile(cc_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_cc_cabinet, cc_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet Coriolis and Centrifugal compensation forces")
            self.test_case.assertTrue(np.allclose(expected_cc_frankas, cc_franka, rtol=4e-03, atol=1e-3), "all envs have similar franka Coriolis and Centrifugal compensation forces")

        if stepno == 9:
            g_cabinet = self.cabinet.get_gravity_compensation_forces().numpy().reshape((self.cabinet.count, self.cabinet.max_dofs)).copy()
            g_franka = self.franka.get_gravity_compensation_forces().numpy().reshape((self.franka.count, self.franka.max_dofs)).copy()
            expected_g_cabinet = np.tile(g_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, self.cabinet.max_dofs))
            expected_g_frankas = np.tile(g_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_g_cabinet, g_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet gravity compensation forces")
            self.test_case.assertTrue(np.allclose(expected_g_frankas, g_franka, rtol=4e-03, atol=1e-3), "all envs have similar franka gravity compensation forces")

        if stepno == 10:
            mm_cabinet = self.cabinet.get_generalized_mass_matrices().numpy().reshape((self.cabinet.count, (self.cabinet.max_dofs) * (self.cabinet.max_dofs))).copy()
            mm_franka = self.franka.get_generalized_mass_matrices().numpy().reshape((self.franka.count, self.franka.max_dofs * self.franka.max_dofs)).copy()
            expected_mm_cabinet = np.tile(mm_cabinet[0], (self.cabinet.count, 1)).reshape((self.cabinet.count, (self.cabinet.max_dofs) * (self.cabinet.max_dofs)))
            expected_mm_frankas = np.tile(mm_franka[0], (self.franka.count, 1)).reshape((self.franka.count, self.franka.max_dofs * self.franka.max_dofs))
            self.test_case.assertTrue(np.allclose(expected_mm_cabinet, mm_cabinet, rtol=1e-03, atol=1e-3), "all envs have similar cabinet mass matrices")
            self.test_case.assertTrue(np.allclose(expected_mm_frankas, mm_franka, rtol=1e-03, atol=1e-3), "all envs have similar franka mass matrices")

        if stepno == 11:
            self.finish()


class TestHeterogeneousBaseArticulations(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 5.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, -5.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=sim.device)

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            p_cartpoles = self.cartpoles.get_link_transforms().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 7)).copy()
            p_ants = self.ants.get_link_transforms().numpy().reshape((self.ants.count, self.ants.max_links, 7)).copy()

            self.rp_cartpoles = self.cartpoles.get_root_transforms().numpy().reshape((self.cartpoles.count, 7)).copy()
            self.rp_ants = self.ants.get_root_transforms().numpy().reshape((self.ants.count, 7)).copy()
            self.rp_cartpoles[:, 2] -= 0.2
            self.rp_ants[:, 2] -= 0.2
            self.cartpoles.set_root_transforms(self.to_warp(self.rp_cartpoles), self.all_indices)
            self.ants.set_root_transforms(self.to_warp(self.rp_ants), self.all_indices)

        if stepno == 2:
            root_rel_cartpoles = np.zeros((self.cartpoles.count, self.cartpoles.max_links, 7))
            root_rel_ants = np.zeros((self.ants.count, self.ants.max_links, 7))

            for i in range(self.cartpoles.max_links):
                root_rel_cartpoles[:, i, :] = self.rp_cartpoles
            for i in range(self.ants.max_links):
                root_rel_ants[:, i, :] = self.rp_ants

            p_cartpoles = self.cartpoles.get_link_transforms().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 7)).copy() - root_rel_cartpoles
            p_ants = self.ants.get_link_transforms().numpy().reshape((self.ants.count, self.ants.max_links, 7)).copy() - root_rel_ants
            expected_p_cartpoless = np.tile(p_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_links, 7))
            expected_p_ants = np.tile(p_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_links, 7))
            self.test_case.assertTrue(np.allclose(expected_p_cartpoless, p_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles link transforms")
            self.test_case.assertTrue(np.allclose(expected_p_ants, p_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants link transforms")

        if stepno == 3:
            p_cartpoles = self.cartpoles.get_dof_positions().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            p_ants = self.ants.get_dof_positions().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            p_cartpoles[:, :] += 0.5
            p_ants[:, :] += 0.1
            self.cartpoles.set_dof_positions(self.to_warp(p_cartpoles), self.all_indices)
            self.ants.set_dof_positions(self.to_warp(p_ants), self.all_indices)

        if stepno == 4:
            p_cartpoles = self.cartpoles.get_dof_positions().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            p_ants = self.ants.get_dof_positions().numpy().reshape((self.ants.count, self.ants.max_dofs)).copy()
            expected_p_cartpoless = np.tile(p_cartpoles[0], (self.cartpoles.count, 1))
            expected_p_ants = np.tile(p_ants[0], (self.ants.count, 1))
            self.test_case.assertTrue(np.allclose(expected_p_cartpoless, p_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles dof positions")
            self.test_case.assertTrue(np.allclose(expected_p_ants, p_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants dof positions")

        if stepno == 5:
            vr_cartpoles = self.cartpoles.get_root_velocities().numpy().reshape((self.cartpoles.count, 6)).copy()
            vr_ants = self.ants.get_root_velocities().numpy().reshape((self.ants.count, 6)).copy()
            vr_cartpoles[:, :] += 0.7
            vr_ants[:, :] += 0.9
            self.cartpoles.set_root_velocities(self.to_warp(vr_cartpoles), self.all_indices)
            self.ants.set_root_velocities(self.to_warp(vr_ants), self.all_indices)

        if stepno == 6:
            v_cartpoles = self.cartpoles.get_link_velocities().numpy().reshape((self.cartpoles.count, self.cartpoles.max_links, 6)).copy()
            v_ants = self.ants.get_link_velocities().numpy().reshape((self.ants.count, self.ants.max_links, 6)).copy()
            expected_v_cartpoless = np.tile(v_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_links, 6))
            expected_v_ants = np.tile(v_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_links, 6))
            self.test_case.assertTrue(np.allclose(expected_v_cartpoless, v_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles link velocities")
            self.test_case.assertTrue(np.allclose(expected_v_ants, v_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants link velocities")

        if stepno == 7:
            j_cartpoles = self.cartpoles.get_jacobians().numpy().reshape((self.cartpoles.count, (self.cartpoles.max_dofs) * 6 * (self.cartpoles.max_links - 1))).copy()
            j_ants = self.ants.get_jacobians().numpy().reshape((self.ants.count, (self.ants.max_dofs + 6) * 6 * self.ants.max_links)).copy()
            expected_j_cartpoless = np.tile(j_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, (self.cartpoles.max_dofs) * 6 * (self.cartpoles.max_links - 1)))
            expected_j_ants = np.tile(j_ants[0], (self.ants.count, 1)).reshape((self.ants.count, (self.ants.max_dofs + 6) * 6 * self.ants.max_links))
            self.test_case.assertTrue(np.allclose(expected_j_cartpoless, j_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles Jacobian")
            self.test_case.assertTrue(np.allclose(expected_j_ants, j_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants Jacobian")

        if stepno == 8:
            cc_cartpoles = self.cartpoles.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            cc_ants = self.ants.get_coriolis_and_centrifugal_compensation_forces().numpy().reshape((self.ants.count, self.ants.max_dofs + 6)).copy()
            expected_cc_cartpoless = np.tile(cc_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs))
            expected_cc_ants = np.tile(cc_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_dofs + 6))
            self.test_case.assertTrue(np.allclose(expected_cc_cartpoless, cc_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles Coriolis and Centrifugal compensation forces")
            self.test_case.assertTrue(np.allclose(expected_cc_ants, cc_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants Coriolis and Centrifugal compensation forces")

        if stepno == 9:
            g_cartpoles = self.cartpoles.get_gravity_compensation_forces().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs)).copy()
            g_ants = self.ants.get_gravity_compensation_forces().numpy().reshape((self.ants.count, self.ants.max_dofs + 6)).copy()
            expected_g_cartpoless = np.tile(g_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs))
            expected_g_ants = np.tile(g_ants[0], (self.ants.count, 1)).reshape((self.ants.count, self.ants.max_dofs + 6))
            self.test_case.assertTrue(np.allclose(expected_g_cartpoless, g_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles gravity compensation forces")
            self.test_case.assertTrue(np.allclose(expected_g_ants, g_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants gravity compensation forces")

        if stepno == 10:
            mm_cartpoles = self.cartpoles.get_generalized_mass_matrices().numpy().reshape((self.cartpoles.count, self.cartpoles.max_dofs * self.cartpoles.max_dofs)).copy()
            mm_ants = self.ants.get_generalized_mass_matrices().numpy().reshape((self.ants.count, (self.ants.max_dofs + 6) * (self.ants.max_dofs + 6))).copy()
            expected_mm_cartpoless = np.tile(mm_cartpoles[0], (self.cartpoles.count, 1)).reshape((self.cartpoles.count, self.cartpoles.max_dofs * self.cartpoles.max_dofs))
            expected_mm_ants = np.tile(mm_ants[0], (self.ants.count, 1)).reshape((self.ants.count, (self.ants.max_dofs + 6) * (self.ants.max_dofs + 6)))
            self.test_case.assertTrue(np.allclose(expected_mm_cartpoless, mm_cartpoles, rtol=1e-03, atol=1e-3), "all envs have similar cartpoles mass matrices")
            self.test_case.assertTrue(np.allclose(expected_mm_ants, mm_ants, rtol=1e-03, atol=1e-3), "all envs have similar ants mass matrices")

        if stepno == 11:
            self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsHeterogeneousTests(unittest.TestCase):

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

    def test_heterogeneous_scene_articulation_cc(self):
        self._run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(False, False)))

    def test_heterogeneous_scene_articulation_gc(self):
        self._run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(True, False)))

    def test_heterogeneous_scene_articulation_gg(self):
        self._run_test(TestHeterogeneousSceneArticulations(self, DeviceParams(True, True)))

    def test_heterogeneous_base_articulation_cc(self):
        self._run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(False, False)))

    def test_heterogeneous_base_articulation_gc(self):
        self._run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(True, False)))

    def test_heterogeneous_base_articulation_gg(self):
        self._run_test(TestHeterogeneousBaseArticulations(self, DeviceParams(True, True)))

    def test_articulation_centroidal_momentum_cc(self):
        self._run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(False, False)))

    def test_articulation_centroidal_momentum_gc(self):
        self._run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, False)))

    def test_articulation_centroidal_momentum_gg(self):
        self._run_test(TestArticulationCentroidalMomentumAndMass(self, DeviceParams(True, True)))
