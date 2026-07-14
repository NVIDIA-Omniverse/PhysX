# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

import unittest

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf


class TestUsdStage(unittest.TestCase):
    """Basic USD stage creation and manipulation tests."""

    def test_create_in_memory_stage(self):
        """Create an in-memory USD stage and verify it is valid."""
        stage = Usd.Stage.CreateInMemory()
        self.assertIsNotNone(stage)
        self.assertTrue(stage.GetRootLayer().anonymous)

    def test_define_xform_prim(self):
        """Create a stage with an Xform prim and verify its path."""
        stage = Usd.Stage.CreateInMemory()
        xform = UsdGeom.Xform.Define(stage, "/World")
        self.assertTrue(xform.GetPrim().IsValid())
        self.assertEqual(xform.GetPath(), Sdf.Path("/World"))

    def test_add_physics_scene(self):
        """Create a stage with a PhysicsScene and verify gravity."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        self.assertTrue(scene.GetPrim().IsValid())

        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        direction = scene.GetGravityDirectionAttr().Get()
        magnitude = scene.GetGravityMagnitudeAttr().Get()
        self.assertEqual(direction, Gf.Vec3f(0.0, 0.0, -1.0))
        self.assertAlmostEqual(magnitude, 9.81, places=2)

    def test_add_rigid_body(self):
        """Create a rigid body cube and verify physics API is applied."""
        stage = Usd.Stage.CreateInMemory()
        cube = UsdGeom.Cube.Define(stage, "/World/Cube")
        self.assertTrue(cube.GetPrim().IsValid())

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        self.assertTrue(cube.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertIsNotNone(rigid_body)

    def test_add_collision(self):
        """Create a collision shape on a cube."""
        stage = Usd.Stage.CreateInMemory()
        cube = UsdGeom.Cube.Define(stage, "/World/Cube")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        self.assertTrue(cube.GetPrim().HasAPI(UsdPhysics.CollisionAPI))


if __name__ == "__main__":
    unittest.main()
