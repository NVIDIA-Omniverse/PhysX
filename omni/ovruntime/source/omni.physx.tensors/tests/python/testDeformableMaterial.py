# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Deformable material tests (youngs modulus, dynamic friction).
# NOTE: These tests require GPU dynamics (DeviceParams(True, True)) even though the
# source Kit tests labeled them with _cc suffix.  They will be skipped if no GPU is
# available.

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

class TestDeformableMaterialYoungsModulus(GridTestBase):
    def __init__(self, test_case, device_params):
        grid_params = GridParams(8, 3.0)
        sim_params = SimParams()
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Attach stage early — cooking requires an attached physics stage.
        # Detach afterward so RunnerInMemory can re-attach with GPU settings.
        self.stage_id = UsdUtils.StageCache.Get().Insert(self.stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(self.stage_id)

        volume_deformable_path = self.env_template_path.AppendChild("volumeDeformableBody")
        self.create_volume_deformable_body(volume_deformable_path,
            translate=Gf.Vec3d(0.0, -0.7, 0.0)
        )

        surface_deformable_path = self.env_template_path.AppendChild("surfaceDeformableBody")
        self.create_surface_deformable_body(surface_deformable_path,
            resolution=20,
            translate=Gf.Vec3d(0.0, 0.7, 2.0),
            rotate=Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -90.0),
            attach=True
        )

        get_physx_simulation_interface().detach_stage()

        physicsUtils.add_ground_plane(self.stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view("/envs/*/volumeDeformableBody")
        surface_deformable_bodies = sim.create_surface_deformable_body_view("/envs/*/surfaceDeformableBody")
        volume_deformable_materials = sim.create_deformable_material_view("/envs/*/volumeDeformableMaterial")
        surface_deformable_materials = sim.create_deformable_material_view("/envs/*/surfaceDeformableMaterial")

        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.check_deformable_body_view(surface_deformable_bodies, self.num_envs)
        self.check_deformable_material_view(volume_deformable_materials, self.num_envs)
        self.check_deformable_material_view(surface_deformable_materials, self.num_envs)

        self.volume_deformable_bodies = volume_deformable_bodies
        self.surface_deformable_bodies = surface_deformable_bodies
        self.volume_materials = volume_deformable_materials
        self.surface_materials = surface_deformable_materials

    def setup_elasticity(self, material_view, scale):
        E = material_view.get_youngs_modulus()
        E_np = E.numpy().flatten()
        E_default = E_np[0] * scale
        E_np[0] = 10 * E_default
        E_np[1:-1] = 0.2 * E_default
        E_np[-1] = 0.1 * E_default
        E = wp.from_numpy(E_np, dtype=wp.float32, device="cpu")
        v_indices_np = np.arange(material_view.count)
        v_indices = wp.from_numpy(v_indices_np, dtype=wp.uint32, device="cpu")
        material_view.set_youngs_modulus(E, v_indices)
        E_check = material_view.get_youngs_modulus()
        E_check_np = E_check.numpy().flatten()
        self.test_case.assertTrue(np.allclose(E_check_np, E_np, rtol=1e-7), "similar get and set values")

    @staticmethod
    def get_sim_position_ranges(deformable_body_view):
        positions = deformable_body_view.get_simulation_nodal_positions().numpy().reshape(
            deformable_body_view.count,
            deformable_body_view.max_simulation_nodes_per_body,
            3
        )
        return [Gf.Range3d(Gf.Vec3d(*pts.min(0).tolist()), Gf.Vec3d(*pts.max(0).tolist())) for pts in positions]

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            self.setup_elasticity(self.volume_materials, 1.0)
            self.setup_elasticity(self.surface_materials, 0.05)

        elif stepno == 25:
            volume_ranges = self.get_sim_position_ranges(self.volume_deformable_bodies)
            surface_ranges = self.get_sim_position_ranges(self.surface_deformable_bodies)

            # volumes compress more with lower youngs modulus
            self.test_case.assertTrue(volume_ranges[0].GetSize()[2] > volume_ranges[1].GetSize()[2])
            self.test_case.assertTrue(volume_ranges[1].GetSize()[2] > volume_ranges[-1].GetSize()[2])
            # surfaces stretch more with lower youngs modulus
            self.test_case.assertTrue(surface_ranges[0].GetSize()[2] < surface_ranges[1].GetSize()[2])
            self.test_case.assertTrue(surface_ranges[1].GetSize()[2] < surface_ranges[-1].GetSize()[2])

            self.finish()


class TestDeformableMaterialDynamicFriction(GridTestBase):
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

        # ensure the surface material parameter for the friction test
        self.ground_material_path = "/groundMaterial"
        UsdShade.Material.Define(self.stage, self.ground_material_path)
        ground_material = UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self.ground_material_path))
        physicsUtils.add_physics_material_to_prim(self.stage, self.stage.GetPrimAtPath("/groundPlane"), self.ground_material_path)

    def on_start(self, sim):
        sim.set_subspace_roots("/envs/*")
        volume_deformable_bodies = sim.create_volume_deformable_body_view(["/envs/env[0-9]/deformableBody", "/envs/env10/deformableBody", "/envs/env[1-2][1-5]/deformableBody"])
        volume_deformable_materials = sim.create_deformable_material_view(["/envs/env[0-9]/volumeDeformableMaterial", "/envs/env10/volumeDeformableMaterial", "/envs/env[1-2][1-5]/volumeDeformableMaterial"])

        self.check_deformable_material_view(volume_deformable_materials, self.num_envs)
        self.check_deformable_body_view(volume_deformable_bodies, self.num_envs)
        self.volume_deformable_bodies = volume_deformable_bodies
        self.volume_materials = volume_deformable_materials

        # enforce 'max' friction combine mode
        ground_material_prim = self.stage.GetPrimAtPath(self.ground_material_path)
        physxMaterialAPI = PhysxSchema.PhysxMaterialAPI.Apply(ground_material_prim)
        physxMaterialAPI.CreateFrictionCombineModeAttr().Set("max")

    def on_physics_step(self, sim, stepno, dt):
        if stepno == 1:
            d = self.volume_materials.get_dynamic_friction()
            d_np = d.numpy()
            d_np[-1] = 1
            d = wp.from_numpy(d_np, dtype=wp.float32, device="cpu")
            indices_numpy = np.array([0, self.volume_materials.count - 1])
            indices = wp.from_numpy(indices_numpy, dtype=wp.uint32, device="cpu")
            self.volume_materials.set_dynamic_friction(d, indices)

        elif stepno == 100:
            vel = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            set_vals = vel + np.array([0, 10, 0])
            wp_vel = wp.from_numpy(set_vals, dtype=wp.float32)
            wp_all_indices = wp_utils.arange(self.volume_deformable_bodies.count, device=sim.device)
            self.volume_deformable_bodies.set_simulation_nodal_velocities(wp_vel, wp_all_indices)
        elif stepno == 105:
            vel = self.volume_deformable_bodies.get_simulation_nodal_velocities().numpy().reshape(self.volume_deformable_bodies.count, self.volume_deformable_bodies.max_simulation_nodes_per_body, 3)
            mean_body_magnitude_mag = np.mean(np.mean(np.abs(vel), axis=1), axis=1)
            self.test_case.assertTrue((mean_body_magnitude_mag[-1] < mean_body_magnitude_mag[:-1]).all(), "smaller overall velocity for larger frictions.")
            self.finish()


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class PhysxTensorsDeformableMaterialTests(unittest.TestCase):
    """Deformable material tests — require GPU dynamics."""

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

    # NOTE: These use DeviceParams(True, True) — the _cc suffix in the source was a misnomer.
    def test_deformable_material_youngs_modulus_cc(self):
        self._run_test(TestDeformableMaterialYoungsModulus(self, DeviceParams(True, True)))

    def test_deformable_material_dynamic_friction_cc(self):
        self._run_test(TestDeformableMaterialDynamicFriction(self, DeviceParams(True, True)))
