# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdUtils
import _physx


class TestPhysicsSimulation(unittest.TestCase):
    """Verify physics simulation runs and produces correct results."""

    def _create_physics_stage(self):
        """Create a USD stage with a physics scene and return (stage, stage_id)."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        cache_id = UsdUtils.StageCache.Get().Insert(stage)
        return stage, cache_id.ToLongInt()

    def _add_rigid_cube(self, stage, path, position, size=1.0):
        """Add a rigid body cube at the given position."""
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(size)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    def _add_ground(self, stage, path="/Ground"):
        """Add a static ground plane collider."""
        ground = UsdGeom.Cube.Define(stage, path)
        ground.CreateSizeAttr(100.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -50.0))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        return ground

    def test_acquire_simulation_interface(self):
        """Acquire IPhysxSimulation and verify it is not None."""
        sim = _physx.acquire_physx_simulation_interface()
        self.assertIsNotNone(sim)

    def test_attach_and_detach_stage(self):
        """Attach a stage to the simulation and detach it."""
        stage, stage_id = self._create_physics_stage()
        sim = _physx.acquire_physx_simulation_interface()

        result = sim.attach_stage(stage_id)
        self.assertTrue(result)

        attached_id = sim.get_attached_stage()
        self.assertEqual(attached_id, stage_id)

        sim.detach_stage()

    def test_rigid_body_falls_under_gravity(self):
        """A rigid body cube should fall under gravity."""
        stage, stage_id = self._create_physics_stage()
        self._add_ground(stage)
        cube = self._add_rigid_cube(stage, "/Cube", position=(0.0, 0.0, 10.0))

        sim = _physx.acquire_physx_simulation_interface()
        sim.attach_stage(stage_id)

        initial_pos = cube.GetPrim().GetAttribute("xformOp:translate").Get()

        # Simulate 1 second (60 steps at 1/60s)
        dt = 1.0 / 60.0
        for i in range(60):
            sim.simulate(dt, i * dt)
            sim.fetch_results()

        final_pos = cube.GetPrim().GetAttribute("xformOp:translate").Get()

        # Cube should have fallen significantly (gravity = 9.81 m/s^2)
        # After 1s of free fall: z = z0 - 0.5*g*t^2 = 10 - 4.905 ≈ 5.1
        self.assertLess(final_pos[2], initial_pos[2] - 3.0)
        # X and Y should remain near zero
        self.assertAlmostEqual(final_pos[0], 0.0, places=2)
        self.assertAlmostEqual(final_pos[1], 0.0, places=2)

        sim.detach_stage()

    def test_multiple_simulation_steps(self):
        """Position should decrease monotonically each step during free fall."""
        stage, stage_id = self._create_physics_stage()
        cube = self._add_rigid_cube(stage, "/Cube", position=(0.0, 0.0, 20.0))

        sim = _physx.acquire_physx_simulation_interface()
        sim.attach_stage(stage_id)

        dt = 1.0 / 60.0
        prev_z = 20.0
        for i in range(30):
            sim.simulate(dt, i * dt)
            sim.fetch_results()
            pos = cube.GetPrim().GetAttribute("xformOp:translate").Get()
            self.assertLess(pos[2], prev_z, f"Step {i}: z should decrease monotonically")
            prev_z = pos[2]

        sim.detach_stage()

    def test_static_collider_does_not_move(self):
        """A static collider (no RigidBodyAPI) should not move during simulation."""
        stage, stage_id = self._create_physics_stage()
        ground = self._add_ground(stage)

        sim = _physx.acquire_physx_simulation_interface()
        sim.attach_stage(stage_id)

        initial_pos = ground.GetPrim().GetAttribute("xformOp:translate").Get()

        dt = 1.0 / 60.0
        for i in range(30):
            sim.simulate(dt, i * dt)
            sim.fetch_results()

        final_pos = ground.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertEqual(initial_pos, final_pos)

        sim.detach_stage()

    # ========== PhysxSchema tests ==========

    def test_physx_scene_api(self):
        """Apply PhysxSceneAPI and configure scene-level PhysX properties."""
        stage, stage_id = self._create_physics_stage()
        scene_prim = stage.GetPrimAtPath("/PhysicsScene")

        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        self.assertTrue(scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI))

        # Configure solver settings
        physx_scene.CreateEnableCCDAttr(True)
        physx_scene.CreateTimeStepsPerSecondAttr(120)

        self.assertTrue(physx_scene.GetEnableCCDAttr().Get())
        self.assertEqual(physx_scene.GetTimeStepsPerSecondAttr().Get(), 120)

    def test_physx_rigid_body_api(self):
        """Apply PhysxRigidBodyAPI and set PhysX-specific rigid body properties."""
        stage, _ = self._create_physics_stage()
        cube = self._add_rigid_cube(stage, "/Cube", position=(0.0, 0.0, 5.0))
        prim = cube.GetPrim()

        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI))

        # Enable CCD on the rigid body
        physx_rb.CreateEnableCCDAttr(True)
        self.assertTrue(physx_rb.GetEnableCCDAttr().Get())

        # Set solver position iterations
        physx_rb.CreateSolverPositionIterationCountAttr(16)
        self.assertEqual(physx_rb.GetSolverPositionIterationCountAttr().Get(), 16)

        physx_rb.CreateSolverVelocityIterationCountAttr(4)
        self.assertEqual(physx_rb.GetSolverVelocityIterationCountAttr().Get(), 4)

    def test_physx_collision_api(self):
        """Apply PhysxCollisionAPI and set collision properties."""
        stage, _ = self._create_physics_stage()
        cube = self._add_rigid_cube(stage, "/Cube", position=(0.0, 0.0, 5.0))
        prim = cube.GetPrim()

        physx_col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxCollisionAPI))

        # Set contact and rest offset
        physx_col.CreateContactOffsetAttr(0.02)
        physx_col.CreateRestOffsetAttr(0.01)

        self.assertAlmostEqual(physx_col.GetContactOffsetAttr().Get(), 0.02, places=4)
        self.assertAlmostEqual(physx_col.GetRestOffsetAttr().Get(), 0.01, places=4)

    def test_physx_schema_simulation_with_ccd(self):
        """Simulate with PhysxSceneAPI CCD enabled and verify rigid body falls."""
        stage, stage_id = self._create_physics_stage()
        scene_prim = stage.GetPrimAtPath("/PhysicsScene")
        self._add_ground(stage)

        # Apply PhysxSceneAPI with CCD
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene.CreateEnableCCDAttr(True)

        # Create cube with PhysxRigidBodyAPI
        cube = self._add_rigid_cube(stage, "/Cube", position=(0.0, 0.0, 10.0))
        PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim())

        sim = _physx.acquire_physx_simulation_interface()
        sim.attach_stage(stage_id)

        dt = 1.0 / 60.0
        for i in range(60):
            sim.simulate(dt, i * dt)
            sim.fetch_results()

        final_pos = cube.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertLess(final_pos[2], 7.0, "Cube should have fallen with CCD enabled")

        sim.detach_stage()

    def test_physx_material_api(self):
        """Apply PhysxMaterialAPI and configure material properties."""
        stage, _ = self._create_physics_stage()

        # Create a physics material
        material = UsdPhysics.MaterialAPI.Apply(
            stage.DefinePrim("/PhysicsMaterial", "Material")
        )
        material.CreateStaticFrictionAttr(0.5)
        material.CreateDynamicFrictionAttr(0.3)
        material.CreateRestitutionAttr(0.6)

        # Apply PhysxMaterialAPI for extended properties
        prim = stage.GetPrimAtPath("/PhysicsMaterial")
        physx_mat = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxMaterialAPI))

        physx_mat.CreateFrictionCombineModeAttr("average")
        self.assertEqual(physx_mat.GetFrictionCombineModeAttr().Get(), "average")

        physx_mat.CreateRestitutionCombineModeAttr("max")
        self.assertEqual(physx_mat.GetRestitutionCombineModeAttr().Get(), "max")


if __name__ == "__main__":
    unittest.main()
