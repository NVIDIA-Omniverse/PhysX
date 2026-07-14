# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# SDF shape tests (SDF values and gradients).
# NOTE: These tests require GPU dynamics (DeviceParams(True, True)).

import os
import sys
import unittest

import numpy as np
import warp as wp

import _tensors_setup  # noqa: F401

import omni.physics.tensors
import warp_utils as wp_utils

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
from omni.physx.scripts import physicsUtils

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

class TestSdfShapeView(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(4, 3.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("shape")
        transform = Transform((0.0, 0.0, 1.0))
        self.length = 0.5
        self.num_points = 100
        object = self.create_sdf_object(actor_path, transform, self.length)
        self.sdF_api_margin = PhysxSchema.PhysxSDFMeshCollisionAPI(object).GetSdfMarginAttr().Get()

    def on_start(self, sim):
        sdfs_view = sim.create_sdf_shape_view("/envs/*/shape", self.num_points * 2)
        self.check_sdf_shape_view(sdfs_view, self.num_envs)
        self.sdfs_view = sdfs_view

    def on_physics_step(self, sim, stepno, dt):
        points = np.zeros((self.num_envs, 2 * self.num_points, 3))
        points[:, :self.num_points, 0] = self.length - self.sdF_api_margin / 2
        points[:, self.num_points:, 0] = self.length + self.sdF_api_margin / 2
        num_points_row = int(np.sqrt(self.num_points))
        delta = self.length / num_points_row
        for i in range(num_points_row):
            for j in range(num_points_row):
                # start from some small distance away to make sure points don't fall on the surface
                points[:, i * num_points_row + j, 1] = -self.length + 2.0 * i * delta - delta / 10
                points[:, i * num_points_row + j, 2] = -self.length + 2.0 * j * delta - delta / 10
                points[:, self.num_points + i * num_points_row + j, 1] = -self.length + 2.0 * i * delta - delta / 10
                points[:, self.num_points + i * num_points_row + j, 2] = -self.length + 2.0 * j * delta - delta / 10
        points_wp = wp.from_numpy(points.flatten(), dtype=wp.float32, device=sim.device)
        sdfs = self.sdfs_view.get_sdf_and_gradients(points_wp)
        sdfs_np = sdfs.numpy().reshape(self.sdfs_view.count, 2 * self.num_points, 4)
        d = np.abs(points) - self.length
        expected = np.linalg.norm(np.maximum(d, 0.0), axis=2) + np.minimum(np.max(d, axis=2), 0.0)
        self.test_case.assertTrue(
            np.allclose(sdfs_np[:, :, -1], expected, rtol=0.1, atol=0.1), "expected sdf values"
        )
        # inside grad
        g1 = np.zeros((self.sdfs_view.count, 2 * self.num_points, 3))
        # outside grad
        g2 = np.zeros((self.sdfs_view.count, 2 * self.num_points, 3))
        distance = points - self.length
        for i in range(self.sdfs_view.count):
            for j in range(self.num_points * 2):
                is_inside = np.max(d[i, j]) < 0
                if is_inside:
                    c = np.argmax(d[i, j])
                    # inside sdf gradient direction is from the sample point to surface
                    g1[i, j, c] = 1
                else:
                    # outside sdf gradient direction is from the surface to the sample point
                    grad = np.sign(distance[i, j]) * np.maximum(d[i, j], 0.0) / np.linalg.norm(np.maximum(d[i, j], 0.0))
                    g2[i, j, :] = grad

        expected_gradient = g1 + g2

        self.test_case.assertTrue(
            np.allclose(sdfs_np[:, :, :-1], expected_gradient, atol=0.1), "expected sdf gradient values"
        )
        self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsSdfShapeTests(unittest.TestCase):
    """SDF shape tests — require GPU dynamics."""

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

    def test_sdf_shapes_gg(self):
        self._run_test(TestSdfShapeView(self, DeviceParams(True, True)))
