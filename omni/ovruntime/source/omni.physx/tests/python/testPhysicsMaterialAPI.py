# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsMaterialAPI.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory, check_stats, ExpectMessage
import physicsUtils
from pxr import Sdf, Usd, Gf, UsdGeom, UsdPhysics, UsdUtils, UsdShade, PhysxSchema


class PhysicsMaterialAPITestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def setup_scene(self, compound):
        stage = self.new_stage()
        self.stage = stage

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))

        # material
        materialPath = "/physicsMaterial"
        self.material_path = materialPath
        matShade = UsdShade.Material.Define(stage, materialPath)
        UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())

        mesh_path = "/World/mesh"
        self.mesh_path = mesh_path

        if compound:
            concaveGeom = physicsUtils.create_mesh_concave(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        else:
            concaveGeom = physicsUtils.create_mesh_cube(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexHull")

        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(mesh_path)), Sdf.Path(materialPath))

    def test_material_remove_first_compound_shape(self):
        self.setup_scene(True)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 4, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()

        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    def test_material_remove_second_compound_shape(self):
        self.setup_scene(True)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 4, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    def test_material_remove_first_shape(self):
        self.setup_scene(False)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()

        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    def test_material_remove_second_shape(self):
        self.setup_scene(False)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()
        check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    def test_material_incorrect_friction(self):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))

        materialPath = "/physicsMaterial"
        self.material_path = materialPath
        matShade = UsdShade.Material.Define(stage, materialPath)
        material_api = UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())

        material_api.CreateStaticFrictionAttr().Set(-2.0)

        message = f"PhysX error: createMaterial: staticFriction must be >= 0."
        with ExpectMessage(self, message, partial_string_match=True):
            self.step()


if __name__ == "__main__":
    unittest.main()
