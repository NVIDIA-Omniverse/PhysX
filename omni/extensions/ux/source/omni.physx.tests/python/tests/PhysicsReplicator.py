# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.utils as physxUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physx import get_physx_replicator_interface, get_physx_simulation_interface, get_physx_interface
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils
import unittest
import carb
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase


class PhysicsReplicatorTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()

    def create_link_hierarchy(self, envPath, index, stage):
        # create rigid body - rootLink
        physicsUtils.add_rigid_box(stage, envPath + "/rootLink", Gf.Vec3f(0.1), Gf.Vec3f(float(index)), Gf.Quatf(1.0))

        rootLinkPath = Sdf.Path(envPath + "/rootLink")
        rootLinkPrim = stage.GetPrimAtPath(rootLinkPath)

        fixedJointPath = Sdf.Path(envPath + "/baseFixedJoint")
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, fixedJointPath)
        fixedJoint.GetBody1Rel().AddTarget(rootLinkPath)

        # create rigid body - dynamicLink
        physicsUtils.add_rigid_box(stage, envPath + "/dynamicLink", Gf.Vec3f(0.1), Gf.Vec3f(float(index)), Gf.Quatf(1.0))

        dynamicLinkPath = Sdf.Path(envPath + "/dynamicLink")
        dynamicLinkPrim = stage.GetPrimAtPath(dynamicLinkPath)

        prismaticJointPath = Sdf.Path(envPath + "/prismaticJoint")
        prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, prismaticJointPath)
        prismaticJoint.GetBody0Rel().AddTarget(rootLinkPath)
        prismaticJoint.GetBody1Rel().AddTarget(dynamicLinkPath)
        prismaticJoint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
        prismaticJoint.GetLowerLimitAttr().Set(-90.0)
        prismaticJoint.GetUpperLimitAttr().Set(90.0)
        prismaticJoint.CreateCollisionEnabledAttr().Set(False)

        artRootPrim = stage.GetPrimAtPath(Sdf.Path(envPath))
        UsdPhysics.ArticulationRootAPI.Apply(artRootPrim)

    async def test_articulation_replication(self):
        # setup basic stage
        stage = await self.new_stage()
        
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        UsdGeom.Scope.Define(stage, "/World/envs")

        for i in range(5):
            self.create_link_hierarchy("/World/envs/env" + str(i), i, stage)
                
        def replicationAttachFn(stageId):
            exclude_paths = []
            exclude_paths.append("/World/envs")
            return exclude_paths

        def replicationAttachEndFn(stageId):
            return

        def hierarchyRenameFn(replicatePath, index):
            stringPath = "/World/envs/env" + str(index + 1);
            return stringPath            

        get_physx_simulation_interface().detach_stage()

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn);

        get_physx_simulation_interface().attach_stage(stageId)
        
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numArticulations": 0, "numConstraints": 0})
        
        get_physx_replicator_interface().replicate(stageId, "/World/envs/env0", 4)
        
        self._check_physx_object_counts({"numBoxShapes": 10, "numDynamicRigids": 10, "numArticulations": 5, "numConstraints": 0})
        
    async def test_articulation_replication_reattach(self):
        # setup basic stage
        stage = await self.new_stage()
        
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        UsdGeom.Scope.Define(stage, "/World/envs")

        self.replicator_end = False
        self.replicator_reattach_end = False

        for i in range(5):
            self.create_link_hierarchy("/World/envs/env" + str(i), i, stage)
                
        def replicationAttachFn(stageId):
            exclude_paths = []
            exclude_paths.append("/World/envs")
            return exclude_paths

        def replicationAttachEndFn(stageId):
            self.replicator_end = True
            return

        def replicationReAttachEndFn(stageId):
            self.replicator_reattach_end = True
            return

        def hierarchyRenameFn(replicatePath, index):
            stringPath = "/World/envs/env" + str(index + 1);
            return stringPath            

        def hierarchyRenameFn(replicatePath, index):
            stringPath = "/World/envs/env" + str(index + 1);
            return stringPath            

        get_physx_simulation_interface().detach_stage()

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn);

        get_physx_simulation_interface().attach_stage(stageId)
        
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numArticulations": 0, "numConstraints": 0})
        
        get_physx_replicator_interface().replicate(stageId, "/World/envs/env0", 4)
        
        self._check_physx_object_counts({"numBoxShapes": 10, "numDynamicRigids": 10, "numArticulations": 5, "numConstraints": 0})
        
        self.assertTrue(self.replicator_end == True)
        self.assertTrue(self.replicator_reattach_end == False)
        self.replicator_end = False
        self.replicator_reattach_end = False

        get_physx_simulation_interface().detach_stage()

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationReAttachEndFn, hierarchyRenameFn);

        get_physx_simulation_interface().attach_stage(stageId)
        
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numArticulations": 0, "numConstraints": 0})
        
        get_physx_replicator_interface().replicate(stageId, "/World/envs/env0", 4)
        
        self._check_physx_object_counts({"numBoxShapes": 10, "numDynamicRigids": 10, "numArticulations": 5, "numConstraints": 0})
            
        self.assertTrue(self.replicator_end == False)
        self.assertTrue(self.replicator_reattach_end == True)
            
    async def test_articulation_replication_attach_end(self):
        # setup basic stage
        stage = await self.new_stage()
        
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        UsdGeom.Scope.Define(stage, "/World/envs")

        for i in range(5):
            self.create_link_hierarchy("/World/envs/env" + str(i), i, stage)
                
        def replicationAttachFn(stageId):
            exclude_paths = []
            exclude_paths.append("/World/envs")
            return exclude_paths

        def replicationAttachEndFn(stageId):
            get_physx_replicator_interface().replicate(stageId, "/World/envs/env0", 4)


        def hierarchyRenameFn(replicatePath, index):
            stringPath = "/World/envs/env" + str(index + 1);
            return stringPath            

        get_physx_simulation_interface().detach_stage()

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn);

        get_physx_simulation_interface().attach_stage(stageId)
                
        self._check_physx_object_counts({"numBoxShapes": 10, "numDynamicRigids": 10, "numArticulations": 5, "numConstraints": 0})
        
    async def test_articulation_replication_attach_unregister(self):
        # setup basic stage
        stage = await self.new_stage()
        
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        UsdGeom.Scope.Define(stage, "/World/envs")

        for i in range(5):
            self.create_link_hierarchy("/World/envs/env" + str(i), i, stage)
                
        def replicationAttachFn(stageId):
            exclude_paths = []
            exclude_paths.append("/World/envs")
            return exclude_paths

        def replicationAttachEndFn(stageId):
            get_physx_replicator_interface().replicate(stageId, "/World/envs/env0", 4)


        def hierarchyRenameFn(replicatePath, index):
            stringPath = "/World/envs/env" + str(index + 1);
            return stringPath            

        get_physx_simulation_interface().detach_stage()

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn);

        get_physx_simulation_interface().attach_stage(stageId)
                
        self._check_physx_object_counts({"numBoxShapes": 10, "numDynamicRigids": 10, "numArticulations": 5, "numConstraints": 0})                    
        
        get_physx_replicator_interface().unregister_replicator(stageId)
        
        get_physx_interface().release_physics_objects()            
        
        get_physx_interface().force_load_physics_from_usd()
