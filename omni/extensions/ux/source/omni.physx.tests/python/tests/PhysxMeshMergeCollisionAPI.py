# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import unittest
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physx import get_physx_simulation_interface
from pxr import Gf, UsdPhysics, UsdGeom, Sdf, PhysxSchema


class PhysxMeshMergeCollisionAPITest(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def setup_stage(self, stage, scenario="hierarchy", kinematic=True):

        self.xform0 = UsdGeom.Xform.Define(stage, "/World/Xform0")
        mesh_merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(self.xform0.GetPrim())
        mesh_merge_api.GetCollisionMeshesCollectionAPI().GetIncludesRel().AddTarget(self.xform0.GetPrim().GetPrimPath())
        UsdPhysics.CollisionAPI.Apply(self.xform0.GetPrim())        
        if kinematic:
            rboAPI = UsdPhysics.RigidBodyAPI.Apply(self.xform0.GetPrim())
            rboAPI.GetKinematicEnabledAttr().Set(True)            

        if scenario == "hierarchy":
            meshCube0 = physicsUtils.create_mesh_cube(stage, "/World/Xform0/meshCube0", 20.0)        
            UsdGeom.Xformable(meshCube0).AddTranslateOp().Set(Gf.Vec3f(-20.0, 0.0, 0.0))
        
            meshCube1 = physicsUtils.create_mesh_cube(stage, "/World/Xform0/meshCube1", 20.0)        
            UsdGeom.Xformable(meshCube1).AddTranslateOp().Set(Gf.Vec3f(20.0, 0.0, 0.0))
        elif scenario == "collection":
            UsdGeom.Xform.Define(stage, "/World/Xform1")
            meshCube0 = physicsUtils.create_mesh_cube(stage, "/World/Xform1/meshCube0", 20.0)        
            UsdGeom.Xformable(meshCube0).AddTranslateOp().Set(Gf.Vec3f(-20.0, 0.0, 0.0))
        
            meshCube1 = physicsUtils.create_mesh_cube(stage, "/World/Xform1/meshCube1", 20.0)        
            UsdGeom.Xformable(meshCube1).AddTranslateOp().Set(Gf.Vec3f(20.0, 0.0, 0.0))

            mesh_merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI(self.xform0)            
            collection_api = mesh_merge_api.GetCollisionMeshesCollectionAPI()
            collection_api.GetIncludesRel().AddTarget("/World/Xform1")
            collection_api.GetIncludesRel().AddTarget(self.xform0.GetPrim().GetPrimPath())


    async def mesh_merge_collision(self, dynamic, scenario):
        stage = await self.new_stage()

        await self.setup_stage(stage, scenario, dynamic)

        cube0 = physicsUtils.add_rigid_box(stage, "/World/cube0", 10, position=Gf.Vec3f(-20.0, 40.0, 0.0))
        cube1 = physicsUtils.add_rigid_box(stage, "/World/cube1", 10, position=Gf.Vec3f(20.0, 40.0, 0.0))
        
        self.step()
        if dynamic:
            utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0, "numTriMeshShapes": 1, "numKinematicBodies": 1})
        else:
            utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 1, "numTriMeshShapes": 1, "numKinematicBodies": 0})

        self.step(num_steps=30)

        position0 = cube0.GetAttribute("xformOp:translate").Get()
        position1 = cube1.GetAttribute("xformOp:translate").Get()

        self.assertTrue(position0[1] > 0.0)
        self.assertTrue(position1[1] > 0.0)

    async def test_mesh_merge_hierarchy_collision_dynamic(self):
        await self.mesh_merge_collision(True, "hierarchy")

    async def test_mesh_merge_hierarchy_collision_static(self):
        await self.mesh_merge_collision(False, "hierarchy")

    async def test_mesh_merge_collection_collision_dynamic(self):
        await self.mesh_merge_collision(True, "collection")

    async def test_mesh_merge_collection_collision_static(self):
        await self.mesh_merge_collision(False, "collection")

    async def mesh_merge_collision_exclude(self, dynamic, scenario):
        stage = await self.new_stage()

        await self.setup_stage(stage, scenario, dynamic)

        cube0 = physicsUtils.add_rigid_box(stage, "/World/cube0", 10, position=Gf.Vec3f(-20.0, 40.0, 0.0))
        cube1 = physicsUtils.add_rigid_box(stage, "/World/cube1", 10, position=Gf.Vec3f(20.0, 40.0, 0.0))

        mesh_merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI(self.xform0)        
        collection_api = mesh_merge_api.GetCollisionMeshesCollectionAPI()
        collection_api.GetIncludesRel().AddTarget(self.xform0.GetPrim().GetPrimPath())
        if scenario == "hierarchy":
            collection_api.GetExcludesRel().AddTarget("/World/Xform0/meshCube0")
        elif scenario == "collection":
            collection_api.GetExcludesRel().AddTarget("/World/Xform1/meshCube0")
        
        self.step()
        if dynamic:
            utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0, "numTriMeshShapes": 1, "numKinematicBodies": 1})
        else:
            utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 1, "numTriMeshShapes": 1, "numKinematicBodies": 0})

        self.step(num_steps=30)

        position0 = cube0.GetAttribute("xformOp:translate").Get()
        position1 = cube1.GetAttribute("xformOp:translate").Get()

        self.assertTrue(position0[1] < 0.0)
        self.assertTrue(position1[1] > 0.0)

    async def test_mesh_merge_hierarchy_collision_dynamic_exclude(self):
        await self.mesh_merge_collision_exclude(True, "hierarchy")

    async def test_mesh_merge_hierarchy_collision_static_exclude(self):
        await self.mesh_merge_collision_exclude(False, "hierarchy")

    async def test_mesh_merge_collection_collision_dynamic_exclude(self):
        await self.mesh_merge_collision_exclude(True, "collection")

    async def test_mesh_merge_collection_collision_static_exclude(self):
        await self.mesh_merge_collision_exclude(False, "collection")        
