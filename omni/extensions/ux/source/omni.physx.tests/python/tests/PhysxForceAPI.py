# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx
import omni.physx.scripts.utils as physicsBaseUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physx import get_physx_simulation_interface
from pxr import Usd, Gf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema, Vt, UsdUtils, PhysicsSchemaTools
import math
import carb
import asyncio


class PhysxForceAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def test_physx_force_world_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(True)

        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)
        
    async def test_physx_force_world_com_utils_helper(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        physicsUtils.add_force_torque(stage, cube.GetPrimPath(), force=Gf.Vec3f(0.0, 1000.0, 0.0), isWorldSpace=True)
        
        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)
        
    async def test_physx_force_world_com_direct_api(self):
        stage = await self.new_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(10.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        rbo_encoded = PhysicsSchemaTools.sdfPathToInt(cube.GetPrimPath())
        
        currentPos = physicsUtils.get_translation(cube)

        self.step()
        self.assertTrue(abs(currentPos[0]) < toleranceEpsilon)        
        
        get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, Gf.Vec3f(100000.0, 0.0, 0.0), Gf.Vec3f(0.0), 
            "Force")

        for _ in range(5):
            self.step()
                
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[0] > toleranceEpsilon)
        
    async def test_physx_force_local_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        hRt2 = math.sqrt(2.0) / 2.0
        orient = Gf.Quatf(hRt2, 0.0, 0.0, -hRt2)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size, orientation=orient)
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[0]) < toleranceEpsilon)
        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)
        self.assertTrue(abs(currentPos[2]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[0] > toleranceEpsilon)
        
    async def test_physx_force_local_com_child_prim(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        hRt2 = math.sqrt(2.0) / 2.0
        orient = Gf.Quatf(hRt2, 0.0, 0.0, -hRt2)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size, orientation=orient)
        force_path = cube.GetPrimPath().AppendChild("force")
        xform = UsdGeom.Xform.Define(stage, force_path)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))        
        xformable.AddRotateZOp().Set(180.0)

        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[0]) < toleranceEpsilon)
        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)
        self.assertTrue(abs(currentPos[2]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[0] < toleranceEpsilon)        
        
    async def test_physx_force_enabled_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 10000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(True)
        physx_force_api.CreateForceEnabledAttr().Set(False)

        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] < toleranceEpsilon)        
        
        physx_force_api.CreateForceEnabledAttr().Set(True)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)

    async def test_physx_torque_world_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateTorqueAttr().Set(Gf.Vec3f(0.0, 2.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(True)

        for _ in range(5):
            self.step()
        
        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        angular_vel = rigid_body_api.GetAngularVelocityAttr().Get()
        self.assertTrue(angular_vel[1] > toleranceEpsilon)
        
    async def test_physx_torque_world_com_direct_api(self):
        stage = await self.new_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(10.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        rbo_encoded = PhysicsSchemaTools.sdfPathToInt(cube.GetPrimPath())
        
        self.step()
        
        get_physx_simulation_interface().apply_torque(stage_id, rbo_encoded, Gf.Vec3f(0.0, 200.0, 0.0))

        for _ in range(5):
            self.step()
        
        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        angular_vel = rigid_body_api.GetAngularVelocityAttr().Get()
        self.assertTrue(angular_vel[1] > toleranceEpsilon)        
        
    async def test_physx_torque_local_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)
        hRt2 = math.sqrt(2.0) / 2.0
        orient = Gf.Quatf(hRt2, 0.0, 0.0, -hRt2)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size, orientation=orient)
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateTorqueAttr().Set(Gf.Vec3f(0.0, 2.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        for _ in range(5):
            self.step()
        
        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        angular_vel = rigid_body_api.GetAngularVelocityAttr().Get()
        self.assertTrue(angular_vel[0] > toleranceEpsilon)
        
    async def test_physx_force_world(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        force_path = cube.GetPrimPath().AppendChild("force")
        xform = UsdGeom.Xform.Define(stage, force_path)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.0))        
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
            
        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        angular_vel = rigid_body_api.GetAngularVelocityAttr().Get()
        self.assertTrue(angular_vel[0] < toleranceEpsilon)
        self.assertTrue(angular_vel[1] < toleranceEpsilon)
        self.assertTrue(angular_vel[2] < toleranceEpsilon)
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)
        self.assertTrue(currentPos[2] < toleranceEpsilon)            
        
        xform.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, -10.0, -1.0))
        
        for _ in range(5):
            self.step()
            
        currentPos = physicsUtils.get_translation(cube)        
        self.assertTrue(currentPos[2] > toleranceEpsilon)            
        
        angular_vel = rigid_body_api.GetAngularVelocityAttr().Get()
        self.assertTrue(angular_vel[0] > toleranceEpsilon)
        self.assertTrue(angular_vel[1] < toleranceEpsilon)
        self.assertTrue(angular_vel[2] < toleranceEpsilon)

    async def test_physx_force_add_remove_com(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)

        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()            
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] < toleranceEpsilon)        
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(cube)
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 5000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(True)

        for _ in range(5):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)
        
        cube.RemoveAPI(PhysxSchema.PhysxForceAPI)
        
        for _ in range(50):
            self.step()
        
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] < toleranceEpsilon)
        
    async def test_physx_force_static_body(self):
        stage = await self.new_stage()
        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        force_path = cube.GetPrimPath().AppendChild("force")
        xform = UsdGeom.Xform.Define(stage, force_path)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.0))        
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(False)
        
        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
                  
        currentPos = physicsUtils.get_translation(cube)          
        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon) 

    async def test_physx_force_kinematic_body(self):
        stage = await self.new_stage()
        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        force_path = cube.GetPrimPath().AppendChild("force")
        xform = UsdGeom.Xform.Define(stage, force_path)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.0))        
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)
        rigid_body_api.CreateKinematicEnabledAttr(True)
        
        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
                  
        currentPos = physicsUtils.get_translation(cube)          
        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)
        
    async def test_physx_force_dynamic_body_enabled_disabled(self):
        stage = await self.new_stage()
        utils.physics_scene_setup(stage)

        toleranceEpsilon = 0.01
        
        # box        
        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        force_path = cube.GetPrimPath().AppendChild("force")
        xform = UsdGeom.Xform.Define(stage, force_path)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.0))        
        
        physx_force_api = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())
        physx_force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 1000.0, 0.0))
        physx_force_api.CreateWorldFrameEnabledAttr().Set(False)

        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, cube.GetPrimPath())
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)
        
        currentPos = physicsUtils.get_translation(cube)

        self.assertTrue(abs(currentPos[1]) < toleranceEpsilon)

        for _ in range(5):
            self.step()
                            
        currentPos = physicsUtils.get_translation(cube)
        self.assertTrue(currentPos[1] > toleranceEpsilon)    
        
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(False)    
        
        for _ in range(5):
            self.step()
                            
        currentPos1 = physicsUtils.get_translation(cube)
        self.assertTrue(Gf.IsClose(currentPos, currentPos1, toleranceEpsilon))
        
