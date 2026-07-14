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

from scenario import (
    GridTestBase, GridScenarioBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory, get_asset_root,
)


# run one simulation step before using the tensor API (currently required for GPU pipeline)
_WARM_START = True

# python frontend for tensor API
_FRONTEND = "warp"

# whether to keep tests alive longer for visual inspection
_KEEPALIVE = False


class TestSimulationViewGravity(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 10.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim):
        sim.set_gravity([0, 0, 1])
        gravity = sim.get_gravity()

        self.test_case.assertEqual(gravity[0], 0)
        self.test_case.assertEqual(gravity[1], 0)
        self.test_case.assertEqual(gravity[2], 1)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class PhysxTensorsSimulationViewTests(unittest.TestCase):

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

    def test_simulation_view_gravity_cpu(self):
        self._run_test(TestSimulationViewGravity(self, DeviceParams(False, False)))

    def test_simulation_view_gravity_gpu(self):
        self._run_test(TestSimulationViewGravity(self, DeviceParams(True, True)))

    def test_data_paths(self):
        asset_root = get_asset_root()
        self.assertTrue(os.path.isdir(asset_root))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "Ant.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartPole.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartPoleNoRail.usda")))
        self.assertTrue(os.path.isfile(os.path.join(asset_root, "CartRailNoPole.usda")))
