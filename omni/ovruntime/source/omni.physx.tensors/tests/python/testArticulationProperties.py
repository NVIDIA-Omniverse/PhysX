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

class TestArticulationBodyProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count)

        # masses
        masses = np.ones((humanoids.count, humanoids.max_links)) * 100.0
        wp_masses = wp.from_numpy(masses, dtype=wp.float32, device="cpu")
        humanoids.set_masses(wp_masses, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_masses().numpy(), masses))

        # inv masses
        self.test_case.assertTrue(humanoids.get_inv_masses().numpy().shape == (self.num_envs, humanoids.max_links))

        # COMs
        com = humanoids.get_coms().numpy().reshape(self.num_envs, humanoids.max_links, 7)
        com[:, :, 0] += 0.1
        wp_coms = wp.from_numpy(com, dtype=wp.float32, device="cpu")
        humanoids.set_coms(wp_coms, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_coms().numpy(), com))

        # inertias
        inertias = humanoids.get_inertias().numpy().reshape(self.num_envs, humanoids.max_links, 9)
        inertias[:, :, [0, 4, 8]] += 0.1
        wp_inertias = wp.from_numpy(inertias, dtype=wp.float32, device="cpu")
        humanoids.set_inertias(wp_inertias, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_inertias().numpy(), inertias))

        # inv inertias
        self.test_case.assertTrue(humanoids.get_inv_inertias().numpy().shape == (self.num_envs, humanoids.max_links, 9))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationShapeProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count)

        # materials
        material_properties = np.zeros((humanoids.count, humanoids.max_shapes, 3))
        material_properties[:, :, 0] = 0.8
        material_properties[:, :, 1] = 0.7
        material_properties[:, :, 2] = 0.6
        wp_properties = wp.from_numpy(material_properties, dtype=wp.float32, device="cpu")
        humanoids.set_material_properties(wp_properties, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_material_properties().numpy(), material_properties))

        # contact offsets
        contact_offsets = np.ones((humanoids.count, humanoids.max_shapes)) * 0.1
        wp_contact_offsets = wp.from_numpy(contact_offsets, dtype=wp.float32, device="cpu")
        humanoids.set_contact_offsets(wp_contact_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_contact_offsets().numpy(), contact_offsets))

        # rest offsets
        rest_offsets = np.ones((humanoids.count, humanoids.max_shapes)) * 0.05
        wp_rest_offsets = wp.from_numpy(rest_offsets, dtype=wp.float32, device="cpu")
        humanoids.set_rest_offsets(wp_rest_offsets, all_indices)
        self.test_case.assertTrue(np.allclose(humanoids.get_rest_offsets().numpy(), rest_offsets))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationFixedTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "ShadowHand.usda")
        actor_path = self.env_template_path.AppendChild("shadow_hand")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        hands = sim.create_articulation_view("/envs/*/shadow_hand")

        self.check_articulation_view(hands, self.num_envs, 26, 24, True)
        all_indices = wp_utils.arange(hands.count, device=sim.device)

        self.test_case.assertEqual(hands.max_fixed_tendons, 4)

        num_tendons = hands.count * hands.max_fixed_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        limits_np = np.arange(0, num_tendons * 2).reshape(hands.count, hands.max_fixed_tendons, 2)
        limits_np[:, :, 0] = -2.0 * limits_np[:, :, 0]
        limits_np[:, :, 1] = 2.0 * limits_np[:, :, 1]
        limits = wp.from_numpy(limits_np, dtype=wp.float32, device=sim.device)
        rest_lengths_np = 0.5 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        rest_lengths = wp.from_numpy(rest_lengths_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(hands.count, hands.max_fixed_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        hands.set_fixed_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, limits=limits, rest_lengths=rest_lengths, offsets=offsets, indices=all_indices)

        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limits().numpy(), limits.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_rest_lengths().numpy(), rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_offsets().numpy(), offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationHeterogeneousFixedTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "FixedTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("FixedTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "ShadowHand.usda")
        actor_path = self.env_template_path.AppendChild("shadow_hand")
        transform = Transform((0.0, 0.0, -1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        fixed_tendon_test = sim.create_articulation_view("/envs/*/FixedTendonTest")
        hands = sim.create_articulation_view("/envs/*/shadow_hand")

        self.check_articulation_view(fixed_tendon_test, self.num_envs, 5, 4, True)
        self.check_articulation_view(hands, self.num_envs, 26, 24, True)
        all_indices = wp_utils.arange(hands.count, device=sim.device)
        hands_indices_np = np.array([0, 4, 5])
        hands_indices = wp.from_numpy(hands_indices_np, dtype=wp.int32, device=sim.device)

        self.test_case.assertEqual(fixed_tendon_test.max_fixed_tendons, 1)
        self.test_case.assertEqual(hands.max_fixed_tendons, 4)

        num_tendons = fixed_tendon_test.count * fixed_tendon_test.max_fixed_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        limits_np = np.arange(0, num_tendons * 2).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons, 2)
        limits_np[:, :, 0] = -2.0 * limits_np[:, :, 0]
        limits_np[:, :, 1] = 2.0 * limits_np[:, :, 1]
        limits = wp.from_numpy(limits_np, dtype=wp.float32, device=sim.device)
        rest_lengths_np = 0.5 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        rest_lengths = wp.from_numpy(rest_lengths_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(fixed_tendon_test.count, fixed_tendon_test.max_fixed_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        hands_stiffnesses_np = hands.get_fixed_tendon_stiffnesses().numpy()
        hands_stiffnesses_np[hands_indices_np, :] = 10.0
        hands_stiffnesses = wp.from_numpy(hands_stiffnesses_np, dtype=wp.float32, device=sim.device)
        hands_dampings_np = hands.get_fixed_tendon_dampings().numpy()
        hands_dampings_np[hands_indices_np, :] = 50.0
        hands_dampings = wp.from_numpy(hands_dampings_np, dtype=wp.float32, device=sim.device)
        hands_limit_stiffnesses_np = hands.get_fixed_tendon_limit_stiffnesses().numpy()
        hands_limit_stiffnesses_np[hands_indices_np, :] = 100.0
        hands_limit_stiffnesses = wp.from_numpy(hands_limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        hands_limits_np = hands.get_fixed_tendon_limits().numpy().reshape(hands.count, hands.max_fixed_tendons, 2)
        hands_limits_np[hands_indices_np, :, 0] = -2.0
        hands_limits_np[hands_indices_np, :, 1] = 2.0
        hands_limits = wp.from_numpy(hands_limits_np, dtype=wp.float32, device=sim.device)
        hands_rest_lengths_np = hands.get_fixed_tendon_rest_lengths().numpy()
        hands_rest_lengths_np[hands_indices_np, :] = 0.5
        hands_rest_lengths = wp.from_numpy(hands_rest_lengths_np, dtype=wp.float32, device=sim.device)
        hands_offsets_np = hands.get_fixed_tendon_offsets().numpy()
        hands_offsets_np[hands_indices_np, :] = 0.1
        hands_offsets = wp.from_numpy(hands_offsets_np, dtype=wp.float32, device=sim.device)

        fixed_tendon_test.set_fixed_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, limits=limits, rest_lengths=rest_lengths, offsets=offsets, indices=all_indices)
        hands.set_fixed_tendon_properties(stiffnesses=hands_stiffnesses, dampings=hands_dampings, limit_stiffnesses=hands_limit_stiffnesses, limits=hands_limits, rest_lengths=hands_rest_lengths, offsets=hands_offsets, indices=hands_indices)

        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_limits().numpy(), limits.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_rest_lengths().numpy(), rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(fixed_tendon_test.get_fixed_tendon_offsets().numpy(), offsets.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_stiffnesses().numpy(), hands_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_dampings().numpy(), hands_dampings.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limit_stiffnesses().numpy(), hands_limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_limits().numpy(), hands_limits.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_rest_lengths().numpy(), hands_rest_lengths.numpy()))
        self.test_case.assertTrue(np.allclose(hands.get_fixed_tendon_offsets().numpy(), hands_offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationSpatialTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "SpatialTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("SpatialTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        spatial_tendon_test = sim.create_articulation_view("/envs/*/SpatialTendonTest")

        self.check_articulation_view(spatial_tendon_test, self.num_envs, 5, 4, True)
        all_indices = wp_utils.arange(spatial_tendon_test.count, device=sim.device)

        self.test_case.assertEqual(spatial_tendon_test.max_spatial_tendons, 1)

        num_tendons = spatial_tendon_test.count * spatial_tendon_test.max_spatial_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)
        spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, offsets=offsets, indices=all_indices)

        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_offsets().numpy(), offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationHeterogeneousSpatialTendonProperties(GridTestBase):
    def __init__(self, test_case, device_params):
        # set up stage
        grid_params = GridParams(16, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "SpatialTendonTest.usda")
        actor_path = self.env_template_path.AppendChild("SpatialTendonTest")
        transform = Transform((0.0, 0.0, 1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)
        # set up env template
        asset_path = os.path.join(get_asset_root(), "MultipleSpatialTendonsTest.usda")
        actor_path = self.env_template_path.AppendChild("MultipleSpatialTendonsTest")
        transform = Transform((0.0, 0.0, -1.5))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        spatial_tendon_test = sim.create_articulation_view("/envs/*/SpatialTendonTest")
        multiple_spatial_tendon_test = sim.create_articulation_view("/envs/*/MultipleSpatialTendonsTest")

        self.check_articulation_view(spatial_tendon_test, self.num_envs, 5, 4, True)
        self.check_articulation_view(multiple_spatial_tendon_test, self.num_envs, 9, 8, True)
        all_indices = wp_utils.arange(spatial_tendon_test.count, device=sim.device)
        multiple_indices_np = np.array([0, 4, 5])
        multiple_indices = wp.from_numpy(multiple_indices_np, dtype=wp.int32, device=sim.device)

        self.test_case.assertEqual(spatial_tendon_test.max_spatial_tendons, 1)
        self.test_case.assertEqual(multiple_spatial_tendon_test.max_spatial_tendons, 2)

        num_tendons = spatial_tendon_test.count * spatial_tendon_test.max_spatial_tendons
        stiffnesses_np = 10 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        stiffnesses = wp.from_numpy(stiffnesses_np, dtype=wp.float32, device=sim.device)
        dampings_np = 100 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        dampings = wp.from_numpy(dampings_np, dtype=wp.float32, device=sim.device)
        limit_stiffnesses_np = 50 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        limit_stiffnesses = wp.from_numpy(limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        offsets_np = 0.1 * np.arange(0, num_tendons).reshape(spatial_tendon_test.count, spatial_tendon_test.max_spatial_tendons)
        offsets = wp.from_numpy(offsets_np, dtype=wp.float32, device=sim.device)

        multiple_stiffnesses_np = multiple_spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy()
        multiple_stiffnesses_np[multiple_indices_np, :] = 10.0
        multiple_stiffnesses = wp.from_numpy(multiple_stiffnesses_np, dtype=wp.float32, device=sim.device)
        multiple_dampings_np = multiple_spatial_tendon_test.get_spatial_tendon_dampings().numpy()
        multiple_dampings_np[multiple_indices_np, :] = 100.0
        multiple_dampings = wp.from_numpy(multiple_dampings_np, dtype=wp.float32, device=sim.device)
        multiple_limit_stiffnesses_np = multiple_spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy()
        multiple_limit_stiffnesses_np[multiple_indices_np, :] = 50.0
        multiple_limit_stiffnesses = wp.from_numpy(multiple_limit_stiffnesses_np, dtype=wp.float32, device=sim.device)
        multiple_offsets_np = multiple_spatial_tendon_test.get_spatial_tendon_offsets().numpy()
        multiple_offsets_np[multiple_indices_np, :] = 0.1
        multiple_offsets = wp.from_numpy(multiple_offsets_np, dtype=wp.float32, device=sim.device)

        spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=stiffnesses, dampings=dampings, limit_stiffnesses=limit_stiffnesses, offsets=offsets, indices=all_indices)
        multiple_spatial_tendon_test.set_spatial_tendon_properties(stiffnesses=multiple_stiffnesses, dampings=multiple_dampings, limit_stiffnesses=multiple_limit_stiffnesses, offsets=multiple_offsets, indices=multiple_indices)

        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_dampings().numpy(), dampings.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(spatial_tendon_test.get_spatial_tendon_offsets().numpy(), offsets.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_stiffnesses().numpy(), multiple_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_dampings().numpy(), multiple_dampings.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_limit_stiffnesses().numpy(), multiple_limit_stiffnesses.numpy()))
        self.test_case.assertTrue(np.allclose(multiple_spatial_tendon_test.get_spatial_tendon_offsets().numpy(), multiple_offsets.numpy()))

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsArticulationPropertyTests(unittest.TestCase):

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

    def test_articulation_body_properties_cpu(self):
        self._run_test(TestArticulationBodyProperties(self, DeviceParams(False, False)))

    def test_articulation_body_properties_gpu(self):
        self._run_test(TestArticulationBodyProperties(self, DeviceParams(True, True)))

    def test_articulation_shape_properties_cpu(self):
        self._run_test(TestArticulationShapeProperties(self, DeviceParams(False, False)))

    def test_articulation_shape_properties_gpu(self):
        self._run_test(TestArticulationShapeProperties(self, DeviceParams(True, True)))

    def test_articulation_fixed_tendon_properties_cc(self):
        self._run_test(TestArticulationFixedTendonProperties(self, DeviceParams(False, False)))

    def test_articulation_fixed_tendon_properties_gc(self):
        self._run_test(TestArticulationFixedTendonProperties(self, DeviceParams(True, False)))

    def test_articulation_fixed_tendon_properties_gg(self):
        self._run_test(TestArticulationFixedTendonProperties(self, DeviceParams(True, True)))

    def test_articulation_heterogeneous_fixed_tendon_properties_cc(self):
        self._run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(False, False)))

    def test_articulation_heterogeneous_fixed_tendon_properties_gc(self):
        self._run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(True, False)))

    def test_articulation_heterogeneous_fixed_tendon_properties_gg(self):
        self._run_test(TestArticulationHeterogeneousFixedTendonProperties(self, DeviceParams(True, True)))

    def test_articulation_spatial_tendon_properties_cc(self):
        self._run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(False, False)))

    def test_articulation_spatial_tendon_properties_gc(self):
        self._run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(True, False)))

    def test_articulation_spatial_tendon_properties_gg(self):
        self._run_test(TestArticulationSpatialTendonProperties(self, DeviceParams(True, True)))

    def test_articulation_heterogeneous_spatial_tendon_properties_cc(self):
        self._run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(False, False)))

    def test_articulation_heterogeneous_spatial_tendon_properties_gc(self):
        self._run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(True, False)))

    def test_articulation_heterogeneous_spatial_tendon_properties_gg(self):
        self._run_test(TestArticulationHeterogeneousSpatialTendonProperties(self, DeviceParams(True, True)))
