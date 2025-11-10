# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdPhysics, UsdGeom, PhysxSchema
from omni.physx.scripts import physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.physx.bindings._physx as pxb
import omni.usd


class BindingImplTestAsync(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_hasconflictingapis_RigidBodyAPI(self):
        stage = await self.new_stage()

        # rigid body without reset
        path = omni.usd.get_stage_next_free_path(stage, "/xform", True)
        xform = UsdGeom.Xform.Define(stage, path)
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        path = omni.usd.get_stage_next_free_path(stage, "/xform/cube", True)
        cube = physicsUtils.add_cube(stage, path)

        # expect conflict from parent
        self.assertTrue(pxb.hasconflictingapis_RigidBodyAPI(cube.GetPrim(), True))

        # add reset, expect no conflicts
        UsdGeom.Xformable(cube.GetPrim()).SetResetXformStack(True)
        self.assertFalse(pxb.hasconflictingapis_RigidBodyAPI(cube.GetPrim(), True))

        # add descendant with rb, expect conflicts
        path = omni.usd.get_stage_next_free_path(stage, "/xform/cube/xform", True)
        xform = UsdGeom.Xform.Define(stage, path)
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        self.assertTrue(pxb.hasconflictingapis_RigidBodyAPI(cube.GetPrim(), True))

    async def test_physics_descendantHasAPI_regression(self):
        stage = await self.new_stage()

        path = omni.usd.get_stage_next_free_path(stage, "/cube", True)
        cube = physicsUtils.add_cube(stage, path)
        path = omni.usd.get_stage_next_free_path(stage, "/cube/xform", True)
        xform = UsdGeom.Xform.Define(stage, path)

        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        self.assertTrue(pxb.descendantHasAPI(UsdPhysics.CollisionAPI, cube.GetPrim()))
