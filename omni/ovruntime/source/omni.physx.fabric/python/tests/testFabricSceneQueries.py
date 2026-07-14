# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, UsdPhysics, PhysicsSchemaTools

class TestFabricSceneQueries(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_overlap_mesh(self):
        stage = await self.new_stage()

        physicsUtils.add_ground_plane(stage, "/groundPlane", "Y", 750.0, Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.5))

        size = Gf.Vec3f(100.0)
        boxPrim0 = physicsUtils.add_rigid_box(stage, "/boxActor0", size, Gf.Vec3f(0.0, 200.0, 0.0))
        boxPrim1 = physicsUtils.add_rigid_box(stage, "/boxActor1", size, Gf.Vec3f(0.0, 50.0, 0.0))
       
        # Add collision filtering to allow them to overlap.
        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(boxPrim0)
        rel = filteringPairsAPI.CreateFilteredPairsRel()
        rel.AddTarget(boxPrim1.GetPath())

        await self.step(60)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(boxPrim0.GetPath())

        overlaps = []
        def report_all_overlaps(overlap_info):
            overlaps.append(overlap_info.rigid_body)
            return True

        get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], report_all_overlaps)

        if str(boxPrim0.GetPath()) in overlaps and str(boxPrim1.GetPath()) in overlaps:
            print(f"Expected overlaps found.")
        else:
            print(f"Expected overlaps missing. Found: {overlaps}")
            self.assertTrue(False)
