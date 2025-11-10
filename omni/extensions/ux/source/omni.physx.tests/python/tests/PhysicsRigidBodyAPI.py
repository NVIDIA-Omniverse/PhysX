# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx
import omni.physx.scripts.utils as physicsBaseUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physxtests import utils
from pxr import Sdf, Usd, Gf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema, Vt, PhysicsSchemaTools
import math
import carb
import asyncio
import os
from omni.physxcommands import SetRigidBodyCommand, RemoveRigidBodyCommand
from omni.physxtests.utils import rigidbody
from omni.physx.bindings._physx import ErrorEvent
import unittest
import omni.timeline
import numpy as np
from omni.physx.bindings._physx import SETTING_MIN_FRAME_RATE


class PhysicsRigidBodyAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_winding_order(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        density = 0.001
        size = Gf.Vec3f(100.0)
        position = Gf.Vec3f(0.0, 0.0, 200.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        for _ in range(100):
            self.step()

        prim = stage.GetPrimAtPath(defaultPrimPath + boxActorPath)
        pos = prim.GetAttribute("xformOp:translate").Get()
        
        print(pos)
        self.assertTrue(math.fabs(pos[2] - 50.0) < 10.0)

        get_physx_simulation_interface().detach_stage()

        geomPrim = UsdGeom.Mesh.Get(stage, defaultPrimPath + "/groundPlane")
        geomPrim.GetOrientationAttr().Set(UsdGeom.Tokens.leftHanded)

        prim.GetAttribute("xformOp:translate").Set(position)
        
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)
        
        for _ in range(100):
            self.step()

        prim = stage.GetPrimAtPath(defaultPrimPath + boxActorPath)
        pos = prim.GetAttribute("xformOp:translate").Get()
        
        print(pos)
        self.assertTrue(pos[2] < 0.0)
            
    # OM-28374
    async def test_physics_rigid_body_instancable(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        sphere = UsdGeom.Sphere.Define(stage, "/xform/sphere")
        UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())

        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddInternalReference("/xform")
        xform.GetPrim().SetInstanceable(True)

        # error would be send out to output about writing to instanceable prim
        self.step()
        
        
    # OM_25452
    async def test_physics_xform_setup(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        tranform = Gf.Transform(Gf.Vec3d(1.0), Gf.Rotation(Gf.Quatd(1.0)), Gf.Vec3d(5.0), Gf.Vec3d(0.0), Gf.Rotation(Gf.Quatd(1.0)))
        matrix = tranform.GetMatrix()

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTransformOp().Set(matrix)

        self.step()
        
        scale = xform.GetPrim().GetAttribute("xformOp:scale").Get()
                        
        epsilon = 0.001
        self.assertTrue(abs(scale[0] - 5.0) < epsilon)
        self.assertTrue(abs(scale[1] - 5.0) < epsilon)
        self.assertTrue(abs(scale[2] - 5.0) < epsilon)

    async def test_physics_xform_double_precision_setup(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        rbo_api = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())
        rbo_api.GetAngularVelocityAttr().Set(Gf.Vec3f(100.0, 0.0, 0.0))

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0))
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))
        xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0))

        scale = xform.GetPrim().GetAttribute("xformOp:scale").Get()
        position = xform.GetPrim().GetAttribute("xformOp:translate").Get()
        orient = xform.GetPrim().GetAttribute("xformOp:orient").Get()

        epsilon = 0.001
        self.assertTrue(Gf.IsClose(scale, Gf.Vec3d(1.0), epsilon))
        self.assertTrue(Gf.IsClose(position, Gf.Vec3d(0.0), epsilon))
        self.assertTrue(Gf.IsClose(orient.GetImaginary(), Gf.Vec3d(0.0), epsilon))
        self.assertTrue(Gf.IsClose(orient.GetReal(), 1.0, epsilon))

        for _ in range(5):
            self.step()
        
        scale = xform.GetPrim().GetAttribute("xformOp:scale").Get()
        position = xform.GetPrim().GetAttribute("xformOp:translate").Get()
        orient = xform.GetPrim().GetAttribute("xformOp:orient").Get()

        print(position)
        print(orient)

        self.assertTrue(Gf.IsClose(scale, Gf.Vec3d(1.0), epsilon))
        self.assertTrue(position[1] < 0.0)
        self.assertTrue(not Gf.IsClose(orient.GetReal(), 1.0, epsilon))
        self.assertTrue(not Gf.IsClose(orient.GetImaginary(), Gf.Vec3d(0.0), epsilon))

    async def test_physics_metrics_assembler_xformop_setup(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/cube")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp()
        xformable.AddRotateXYZOp()
        xformable.AddScaleOp()
        xformable.AddRotateXOp(opSuffix="unitsResolve").Set(90.0)
        xformable.AddScaleOp(opSuffix="unitsResolve").Set(Gf.Vec3d(1.0))

        self.step()
        
        xform_ops = xformable.GetXformOpOrderAttr().Get()
        self.assertTrue(len(xform_ops) == 5)
        
    async def test_physics_kinematic_setup(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        position = Gf.Vec3d(0.0, 0.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 500.0, 0.0)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=50, value=positionEnd)

        self.step()

        xformMatrix = xformable.GetLocalTransformation(Usd.TimeCode(50))

        translate = xformMatrix.ExtractTranslation()
        
        epsilon = 0.001
        self.assertTrue(Gf.IsClose(translate, positionEnd, epsilon))
      
    async def test_physics_kinematic_disabled_setup(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        position = Gf.Vec3d(0.0, 10.0, 0.0)        
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(position)                

        self.step()
                
        epsilon = 0.001
        xformMatrix = xformable.GetLocalTransformation()
        translate = xformMatrix.ExtractTranslation()
        
        xformOpStack = xformable.GetOrderedXformOps()
        self.assertTrue(len(xformOpStack) == 1)                
        self.assertTrue(Gf.IsClose(translate, position, epsilon))        
        
        physicsAPI.CreateKinematicEnabledAttr().Set(False)
        
        for _ in range(5):
            self.step()
            
        xformOpStack = xformable.GetOrderedXformOps()
        self.assertTrue(len(xformOpStack) == 3)

        xformMatrix = xformable.GetLocalTransformation()
        translate = xformMatrix.ExtractTranslation()
        self.assertTrue(position[1] > translate[1])
        
        get_physx_interface().reset_simulation()
                
        xformMatrix = xformable.GetLocalTransformation()
        translate = xformMatrix.ExtractTranslation()
        self.assertTrue(Gf.IsClose(translate, position, epsilon))
          
    async def test_physics_kinematic_fallback(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        
        cube = UsdGeom.Cube.Define(stage, "/xform/cube")
        
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(False)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        position = Gf.Vec3d(0.0, 0.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 500.0, 0.0)    
        xformable = UsdGeom.Xformable(xform.GetPrim())    
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=50, value=positionEnd)
        
        self.step()
        utils.check_stats(self, { "numBoxShapes": 1, "numKinematicBodies": 1, "numDynamicRigids": 0 })
        
    async def test_physics_kinematic_apply_force(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        position = Gf.Vec3d(0.0, 0.0, 0.0)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(position)

        self.step()

        xformMatrix = xformable.GetLocalTransformation()
        translate = xformMatrix.ExtractTranslation()
        
        epsilon = 0.001
        print(translate)
        self.assertTrue(Gf.IsClose(translate, position, epsilon))

        force = carb.Float3(1000.0, 1000.0, 1000.0)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()        
        rbo_encoded = PhysicsSchemaTools.sdfPathToInt("/xform")
        get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, force, position, "Impulse")

        self.step()
        self.assertTrue(Gf.IsClose(translate, position, epsilon))

    async def test_physics_delete_collision_group(self):
        #
        # Ensure deleting a collision group works while the simulation is running.
        # The actors must be moved apart to reset the filtering. Once filtering
        # is reset automatically, this will not be necessary.
        #
        stage = await self.new_stage()
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Add the mandatory scene before the simulation starts.
        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # Start the simulation
        self.step()

        # Create the rigid bodies
        upAxis = UsdGeom.GetStageUpAxis(stage)
        scaleFactor = omni.physx.scripts.utils.getUnitScaleFactor(stage)
        position = Gf.Vec3f(0.0)
        color = Gf.Vec3f(0.5)
        size = 25.0 * scaleFactor

        physicsUtils.add_rigid_box(stage, "/Cube1", size=Gf.Vec3f(100.0), position=Gf.Vec3f(30, 50, 0))
        physicsUtils.add_rigid_box(stage, "/Cube2", size=Gf.Vec3f(100.0), position=Gf.Vec3f(-30, 50, 0))
        physicsUtils.add_ground_plane(stage, "/GroundPlane", upAxis, size, position, color)

        cube1Path = defaultPrimPath + "/Cube1"
        cube2Path = defaultPrimPath + "/Cube2"
        cube1Prim = stage.GetPrimAtPath(cube1Path)

        endPos = cube1Prim.GetAttribute("xformOp:translate").Get()

        # Create the collision groups to prevent collisions.
        collisionGroup1 = UsdPhysics.CollisionGroup.Define(stage, "/CollisionGroup1")
        collisionGroup1Rel = collisionGroup1.CreateFilteredGroupsRel()
        collisionGroup1Rel.AddTarget("/CollisionGroup2")
        physicsUtils.add_collision_to_collision_group(stage, cube1Path, "/CollisionGroup1")

        collisionGroup2 = UsdPhysics.CollisionGroup.Define(stage, "/CollisionGroup2")
        physicsUtils.add_collision_to_collision_group(stage, cube2Path, "/CollisionGroup2")

        # Simulate to ensure the two blocks do not collide with each other.
        self.step()

        endPos = cube1Prim.GetAttribute("xformOp:translate").Get()
        # print(endPos)

        errTol = 0.1
        self.assertAlmostEqual(endPos[0], 30, delta=errTol)

        # Now remove the collision group that filters out the collisions and ensure the cubes push each other apart.
        message = "Physics USD: Removing a CollisionGroup while the simulation is running results in undefined behavior!"
        with omni.physxtests.utils.ExpectMessage(self, message):
            stage.RemovePrim("/CollisionGroup1")

        # This step can be removed once resetFilters is called when collision groups are removed. For now, 
        # move the two cubes apart, simulate and then reset the positions in order to get the collision filters to reset.
        cube1Prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(30, 200, 0))
        self.step()
        cube1Prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(30, 50, 0))

        # Now simualte again to ensure the two block DO collide with each other.
        for i in range(10):
            self.step()

        endPos = cube1Prim.GetAttribute("xformOp:translate").Get()
        # print(endPos)
        self.assertGreater(endPos[0], 49.0)

        await self.new_stage()

    async def test_physics_ccd(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        physicsScene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(physicsScene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)

        boxActorPath = "/boxActor0"

        density = 0.001
        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 4000.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        spherePrim = physicsUtils.add_rigid_sphere(stage, boxActorPath, radius, position, orientation, color, density, linVelocity, angularVelocity)
        physxRbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(spherePrim.GetPrim())
        physxRbAPI.CreateEnableCCDAttr().Set(True)

        physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        
        for _ in range(500):
            self.step()

        prim = stage.GetPrimAtPath(defaultPrimPath + boxActorPath)
        pos = prim.GetAttribute("xformOp:translate").Get()

        self.assertTrue(math.fabs(pos[2] - 1.0) < 1.0)


    async def test_physics_rigidbody_static_trimesh(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        physicsScene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(physicsScene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)

        # setup a rigid body with xform body and a one collision under
        ridigBody = physicsUtils.add_rigid_xform(stage, "/rigidBody")
        geomPlanePath = ridigBody.GetPrimPath().AppendChild("geom")
        physicsUtils.create_mesh_square_axis(stage, geomPlanePath, "Y", 100.0)
        planePrim = stage.GetPrimAtPath(geomPlanePath)
        UsdPhysics.CollisionAPI.Apply(planePrim)
        physicsMeshAPI = UsdPhysics.MeshCollisionAPI.Apply(planePrim)
        physicsMeshAPI.CreateApproximationAttr("none")

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, ridigBody.GetPath())
        physicsBodyAPI.CreateRigidBodyEnabledAttr(False)
        
        self.step()
        utils.check_stats(self, {"numTriMeshShapes": 1, "numStaticRigids": 1})

    async def test_physics_transform_update_float_or_double(self):
        #
        # writing back transforms to USD should work for float or double precision. This used to
        # fail for double precision where nothing got written back.
        #
        for j in range(2):
            stage = await self.new_stage()

            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
            UsdGeom.SetStageMetersPerUnit(stage, 1)

            scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1.0, 0))
            scene.CreateGravityMagnitudeAttr(10.0)

            boxActorPath = "/boxActor"
            size = 0.1
            mass = 1.0
            position = Gf.Vec3f(0.0)
            orientation = Gf.Quatf(1.0)

            cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
            cubePrim = stage.GetPrimAtPath(boxActorPath)
            cubeGeom.CreateSizeAttr(size)

            if j == 0:
                cubeGeom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(position)
                cubeGeom.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(orientation)
            else:
                cubeGeom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                    Gf.Vec3d(position[0], position[1], position[2])
                )
                cubeGeom.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(orientation))

            UsdPhysics.CollisionAPI.Apply(cubePrim)
            physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)                        
            massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
            massAPI.CreateMassAttr(mass)

            for i in range(10):
                self.step()

            posY = cubePrim.GetAttribute("xformOp:translate").Get()[1]
            self.assertTrue(posY < -0.1)


    async def test_rigid_body_api_update(self):
        stage = await self.new_stage()
        utils.physics_scene_setup(stage)

        # box
        cubePrim = physicsUtils.add_cube(stage, "/boxActor", 10.0, Gf.Vec3f(0.0))
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateRigidBodyEnabledAttr(True)

        # expect one body and one shape
        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numStaticRigids": 0 })

        physicsAPI.GetRigidBodyEnabledAttr().Set(False)

        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 0, "numStaticRigids": 1 })

        physicsAPI.GetRigidBodyEnabledAttr().Set(True)

        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numStaticRigids": 0 })


    async def test_object_changed_callbacks(self):
        stage = await self.new_stage()
        utils.physics_scene_setup(stage)

        cubePrim = physicsUtils.add_cube(stage, "/boxActor", 10.0, Gf.Vec3f(0.0))
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateRigidBodyEnabledAttr(True)

        self._object_creation_path = None
        self._object_destruction_path = None
        self._all_objects_destruction = False

        def report_object_creation(path, object_id, physx_type):
            self._object_creation_path = str(PhysicsSchemaTools.intToSdfPath(path))

        def report_object_destruction(path, object_id, physx_type):
            self._object_destruction_path = str(PhysicsSchemaTools.intToSdfPath(path))

        def report_all_objects_destruction():
            self._all_objects_destruction = True

        subId = get_physx_interface().subscribe_object_changed_notifications(object_creation_fn=report_object_creation, 
                                                                            object_destruction_fn=report_object_destruction, 
                                                                            all_objects_destruction_fn=report_all_objects_destruction, 
                                                                            stop_callback_when_sim_stopped = False)

        self.step()
        self.assertEqual(self._object_creation_path, "/boxActor")

        physicsAPI.GetRigidBodyEnabledAttr().Set(False)

        self.step()
        self.assertEqual(self._object_destruction_path, "/boxActor")

        self.detach_stage()
        self.assertEqual(self._all_objects_destruction, True)

        get_physx_interface().unsubscribe_object_change_notifications(subId)


    async def test_physx_rigid_body_api_update(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        # box
        cubePrim = physicsUtils.add_cube(stage, "/boxActor", 10.0)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateRigidBodyEnabledAttr(True)
        physicsAPI.CreateVelocityAttr(Gf.Vec3f(0.0, -0.1, 0.0))

        # expect one body and one shape
        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        self.step()

        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(cubePrim)
        physxAPI.CreateDisableGravityAttr(True)
        physicsAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0))

        currentPos = physicsUtils.get_translation(cubePrim)

        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        self.step()

        newPos = physicsUtils.get_translation(cubePrim)

        toleranceEpsilon = 0.01

        self.assertTrue(abs(currentPos[1] - newPos[1]) < toleranceEpsilon)

    async def test_physics_prim_update_map_invalid(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        UsdGeom.Xform.Define(stage, "/xform")

        self.step()
        
        UsdGeom.Xform.Define(stage, "/xform/xform")

        stage.RemovePrim("/xform")

        self.step()

    # test for OM-38709
    async def test_physics_rigid_body_session_layer(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
                
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            xform = UsdGeom.Xform.Define(stage, "/xform")
            UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        self.step()

    async def test_physics_rigidbody_trimesh_holes(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0, 0.0, 500.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, defaultPrimPath + "/sphere", radius, position)
        
        # setup trimesh
        size = 100.0
        cube_mesh = physicsUtils.create_mesh_cube(stage, defaultPrimPath + "/cube", size)
        UsdPhysics.CollisionAPI.Apply(cube_mesh.GetPrim())
        holes = [ 2, 4 ]
        cube_mesh.GetHoleIndicesAttr().Set(holes)
        
        for _ in range(100):
            self.step()
                    
        utils.check_stats(self, { "numTriMeshShapes": 1, "numStaticRigids": 1, "numSphereShapes": 1, "numDynamicRigids": 1})
            
        current_pos = physicsUtils.get_translation(sphere_prim)
        
        self.assertTrue(current_pos[2] < 0.0)
                
    async def test_physics_xformop_reset(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        tranform = Gf.Transform(Gf.Vec3d(1.0), Gf.Rotation(Gf.Quatd(1.0)), Gf.Vec3d(5.0), Gf.Vec3d(0.0), Gf.Rotation(Gf.Quatd(1.0)))
        matrix = tranform.GetMatrix()

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTransformOp().Set(matrix)
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')

        self.step()

        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
                
        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')
        
        new_matrix = xformable.GetLocalTransformation()
        
        self.assertTrue(Gf.IsClose(matrix, new_matrix, 1e-7))
        
    async def test_physics_reset_layer_clear(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())
        
        tranform = Gf.Transform(Gf.Vec3d(1.0), Gf.Rotation(Gf.Quatd(1.0)), Gf.Vec3d(5.0), Gf.Vec3d(0.0), Gf.Rotation(Gf.Quatd(1.0)))
        matrix = tranform.GetMatrix()

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformOp = xformable.AddTransformOp()
        xformOp.Set(matrix)
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')
        
        session_layer = stage.GetSessionLayer()
        self.assertTrue(session_layer.GetAttributeAtPath(xformOp.GetAttr().GetPath()) == None)
        
        stage.SetEditTarget(stage.GetSessionLayer())

        self.step()
        
        trAttr = xform.GetPrim().GetAttribute('xformOp:translate')
        scAttr = xform.GetPrim().GetAttribute('xformOp:scale')
        orAttr = xform.GetPrim().GetAttribute('xformOp:orient')
        velAttr = xform.GetPrim().GetAttribute(UsdPhysics.Tokens.physicsVelocity)
        angVelAttr = xform.GetPrim().GetAttribute(UsdPhysics.Tokens.physicsAngularVelocity)
        
        test_attr = [ trAttr, scAttr, orAttr, velAttr, angVelAttr ]
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(session_layer.GetAttributeAtPath(attr.GetPath()) != None)

        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
                
        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')
        
        self.assertTrue(session_layer.GetAttributeAtPath(xformOp.GetAttr().GetPath()) == None)
        
        for attr in test_attr:
            self.assertTrue(session_layer.GetAttributeAtPath(attr.GetPath()) == None)
                
        new_matrix = xformable.GetLocalTransformation()
        
        self.assertTrue(Gf.IsClose(matrix, new_matrix, 1e-7))

    async def test_physics_xformop_reset_rigid_body_error(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform0 = UsdGeom.Xform.Define(stage, "/xform0")
        UsdPhysics.RigidBodyAPI.Apply(xform0.GetPrim())
        
        xform1 = UsdGeom.Xform.Define(stage, "/xform0/xform1")
        UsdPhysics.RigidBodyAPI.Apply(xform1.GetPrim())
        
        # Now remove the collision group that filters out the collisions and ensure the cubes push each other apart.
        message = "Rigid Body of (/xform0/xform1) missing xformstack reset when child of another enabled rigid body " \
            "(/xform0) in hierarchy. Simulation of multiple RigidBodyAPI's in a hierarchy will cause unpredicted " \
            "results. Please fix the hierarchy or use XformStack reset."

        with utils.ExpectMessage(self, message):
            self.step()

        self._check_physx_object_counts({"numDynamicRigids": 1})

    async def test_physics_xformop_reset_rigid_body_nested(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform0 = UsdGeom.Xform.Define(stage, "/xform0")
        UsdPhysics.RigidBodyAPI.Apply(xform0.GetPrim())
        
        xform1 = UsdGeom.Xform.Define(stage, "/xform0/xform1")
        UsdPhysics.RigidBodyAPI.Apply(xform1.GetPrim())
        xform1.SetResetXformStack(True)
        
        self.step()

        self._check_physx_object_counts({"numDynamicRigids": 2})

    async def test_physics_xformop_reset_rigid_body_hierarchy_nested(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform0 = UsdGeom.Xform.Define(stage, "/xform0")
        UsdPhysics.RigidBodyAPI.Apply(xform0.GetPrim())
        
        xform1 = UsdGeom.Xform.Define(stage, "/xform0/xform1")
        xform1.SetResetXformStack(True)

        xform2 = UsdGeom.Xform.Define(stage, "/xform1/xform2")
        UsdPhysics.RigidBodyAPI.Apply(xform2.GetPrim())

        self.step()

        self._check_physx_object_counts({"numDynamicRigids": 2})

    async def test_physics_xformop_reset_suffix(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        position0 = Gf.Vec3d(0.0, -200.0, 100.0)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(position0)

        position1 = Gf.Vec3d(10.0, 0.0, 0.0)
        xformable.AddTranslateOp(opSuffix='offset').Set(position1)
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 2)        

        self.step()

        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
                
        self.assertTrue(len(xform_op_order) == 2)        
        self.assertEqual(xform_op_order[0], 'xformOp:translate')
        self.assertEqual(xform_op_order[1], 'xformOp:translate:offset')
        
        new_matrix = xformable.GetLocalTransformation()
        
        self.assertTrue(Gf.IsClose(new_matrix.ExtractTranslation(), position0 + position1, 1e-7))
        
    async def test_physics_xformop_reset_inverse(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Cube.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        position0 = Gf.Vec3d(0.0, 10.0, 0.0)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(position0)

        xformable.AddTranslateOp(isInverseOp=True)
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 2)     
        
        self.step()

        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
                
        self.assertTrue(len(xform_op_order) == 2)        
        self.assertEqual(xform_op_order[0], 'xformOp:translate')        
        
        new_matrix = xformable.GetLocalTransformation()
        
        self.assertTrue(Gf.IsClose(new_matrix.ExtractTranslation(), Gf.Vec3d(0.0), 1e-7))
        
    async def test_physics_xformop_reset_reference(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        authoring_layer = stage.GetEditTarget().GetLayer()
        
        data_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        data_folder = data_folder.replace("\\", "/") + "/"
                
        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddReference(data_folder + "sphere_ref.usda")
        
        sphere_prim = stage.GetPrimAtPath("/xformInst/Sphere")        
        sphere = UsdGeom.Xformable(sphere_prim)
        
        self.assertTrue(sphere_prim)
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()
        trAttr = sphere_prim.GetAttribute('xformOp:translate')
        scAttr = sphere_prim.GetAttribute('xformOp:scale')
        orAttr = sphere_prim.GetAttribute('xformOp:orient')
        velAttr = sphere_prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity)
        
        test_attr = [ trAttr, scAttr, orAttr, velAttr ]
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(authoring_layer.GetAttributeAtPath(attr.GetPath()) == None)

        self.assertTrue(len(xform_op_order) == 3)     
        
        for _ in range(5):
            self.step()

        test_attr = [ trAttr, velAttr ]
        
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(authoring_layer.GetAttributeAtPath(attr.GetPath()) != None)

        new_matrix = sphere.GetLocalTransformation()        
        self.assertTrue(not Gf.IsClose(new_matrix.ExtractTranslation(), Gf.Vec3d(0.0), 1e-7))
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()
                
        self.assertTrue(len(xform_op_order) == 3)                
        
        new_matrix = sphere.GetLocalTransformation()        
        self.assertTrue(Gf.IsClose(new_matrix.ExtractTranslation(), Gf.Vec3d(0.0), 1e-7))
        
        test_attr = [ trAttr, scAttr, orAttr, velAttr ]        
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(authoring_layer.GetAttributeAtPath(attr.GetPath()) == None)
        
    async def test_physics_xformop_reset_reference_opwritten(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        authoring_layer = stage.GetEditTarget().GetLayer()
        
        data_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        data_folder = data_folder.replace("\\", "/") + "/"
                
        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddReference(data_folder + "sphere_ref.usda")
        
        sphere_prim = stage.GetPrimAtPath("/xformInst/Sphere2")        
        sphere = UsdGeom.Xformable(sphere_prim)
        
        self.assertTrue(sphere_prim)
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()
        trAttr = sphere_prim.GetAttribute('xformOp:translate')
        orAttr = sphere_prim.GetAttribute('xformOp:orient')        
        
        test_attr = [ trAttr, orAttr ]
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(authoring_layer.GetAttributeAtPath(attr.GetPath()) == None)
        
        self.assertTrue(authoring_layer.GetAttributeAtPath(sphere.GetXformOpOrderAttr().GetPath()) == None)
        self.assertTrue(len(xform_op_order) == 2)     
            
        trAttr.Set(Gf.Vec3d(0,2,0))        
        self.assertTrue(authoring_layer.GetAttributeAtPath(trAttr.GetPath()) != None)
        
        for _ in range(5):
            self.step()
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()                
        self.assertTrue(len(xform_op_order) == 2)                
        
        new_matrix = sphere.GetLocalTransformation()        
        self.assertTrue(Gf.IsClose(new_matrix.ExtractTranslation(), Gf.Vec3d(0, 2, 0), 1e-7))
        
        self.assertTrue(authoring_layer.GetAttributeAtPath(trAttr.GetPath()) != None)
        self.assertTrue(authoring_layer.GetAttributeAtPath(orAttr.GetPath()) == None)
        self.assertTrue(authoring_layer.GetAttributeAtPath(sphere.GetXformOpOrderAttr().GetPath()) == None)

    async def test_physics_xformop_reset_reference_over(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        authoring_layer = stage.GetEditTarget().GetLayer()
        
        data_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        data_folder = data_folder.replace("\\", "/") + "/"
                
        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddReference(data_folder + "sphere_ref.usda")
        
        sphere_prim = stage.GetPrimAtPath("/xformInst/Sphere")        
        sphere = UsdGeom.Xformable(sphere_prim)
        
        self.assertTrue(sphere_prim)
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()
        trAttr = sphere_prim.GetAttribute('xformOp:translate')
        orAttr = sphere_prim.GetAttribute('xformOp:orient')        
        scaleAttr = sphere_prim.GetAttribute('xformOp:scale')   
        
        test_attr = [ trAttr, orAttr, scaleAttr ]
        for attr in test_attr:
            self.assertTrue(attr)
            self.assertTrue(authoring_layer.GetAttributeAtPath(attr.GetPath()) == None)
                
        self.assertTrue(authoring_layer.GetAttributeAtPath(sphere.GetXformOpOrderAttr().GetPath()) == None)
        self.assertTrue(len(xform_op_order) == 3)     
            
        trAttr.Set(Gf.Vec3d(0,2,0))        
        self.assertTrue(authoring_layer.GetAttributeAtPath(trAttr.GetPath()) != None)
        
        for _ in range(5):
            self.step()
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()        
        self.assertTrue(len(xform_op_order) == 3)        
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = sphere.GetXformOpOrderAttr().Get()                
        self.assertTrue(len(xform_op_order) == 3)                
        
        new_matrix = sphere.GetLocalTransformation()        
        self.assertTrue(Gf.IsClose(new_matrix.ExtractTranslation(), Gf.Vec3d(0, 2, 0), 1e-7))
        
        self.assertTrue(authoring_layer.GetAttributeAtPath(trAttr.GetPath()) != None)
        self.assertTrue(authoring_layer.GetAttributeAtPath(orAttr.GetPath()) == None)
        self.assertTrue(authoring_layer.GetAttributeAtPath(scaleAttr.GetPath()) == None)
        self.assertTrue(authoring_layer.GetAttributeAtPath(sphere.GetXformOpOrderAttr().GetPath()) == None)

    async def test_physics_xformop_reset_reference_rotate_xyz(self):
        stage = await self.new_stage()

        UsdGeom.Xform.Define(stage, "/World")

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        authoring_layer = stage.GetEditTarget().GetLayer()
        
        ref_stage = Usd.Stage.CreateInMemory()
        cache = UsdUtils.StageCache.Get()
        cache.Insert(ref_stage)

        cube = UsdGeom.Cube.Define(ref_stage, "/World/cube")
        cube_prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)
        cube.AddTranslateOp().Set(value=Gf.Vec3f(1.0))
        cube.AddRotateXYZOp().Set(value=Gf.Vec3f(1.0))
        cube.AddScaleOp().Set(value=Gf.Vec3f(1.0))
        
        world_prim = stage.GetPrimAtPath("/World")
        world_prim.GetReferences().AddReference(ref_stage.GetRootLayer().identifier, "/World")

        cube_prim = stage.GetPrimAtPath("/World/cube")
        cube = UsdGeom.Cube(cube_prim)
        self.assertTrue(cube_prim)
        
        xform_op_order = cube.GetXformOpOrderAttr().Get()        
        trAttr = cube_prim.GetAttribute('xformOp:translate')
        rotAttr = cube_prim.GetAttribute('xformOp:rotateXYZ')    
        scaleAttr = cube_prim.GetAttribute('xformOp:scale')   
        
        test_attr = [trAttr, rotAttr, scaleAttr]
        for attr in test_attr:
            self.assertTrue(attr)
            
        self.assertTrue(len(xform_op_order) == 3)  

        self.assertTrue(xform_op_order[0] == 'xformOp:translate')
        self.assertTrue(xform_op_order[1] == 'xformOp:rotateXYZ')
        self.assertTrue(xform_op_order[2] == 'xformOp:scale')
            
        for _ in range(5):
            self.step()
        
        xform_op_order = cube.GetXformOpOrderAttr().Get()        
        self.assertTrue(len(xform_op_order) == 3)   

        self.assertTrue(xform_op_order[0] == 'xformOp:translate')
        self.assertTrue(xform_op_order[1] == 'xformOp:orient')
        self.assertTrue(xform_op_order[2] == 'xformOp:scale')

        trAttr = cube_prim.GetAttribute('xformOp:translate')
        orAttr = cube_prim.GetAttribute('xformOp:orient')        
        scaleAttr = cube_prim.GetAttribute('xformOp:scale')   
        
        test_attr = [trAttr, orAttr, scaleAttr]
        for attr in test_attr:
            self.assertTrue(attr)

        get_physx_interface().reset_simulation()
        
        xform_op_order = cube.GetXformOpOrderAttr().Get()                
        self.assertTrue(len(xform_op_order) == 3)                
                
        self.assertTrue(xform_op_order[0] == 'xformOp:translate')
        self.assertTrue(xform_op_order[1] == 'xformOp:rotateXYZ')
        self.assertTrue(xform_op_order[2] == 'xformOp:scale')        

    async def _run_rigidbody_reset_transform_test(self, deleteAttributeDuringSim=False):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = UsdGeom.Xform.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        massApi = UsdPhysics.MassAPI.Apply(xform.GetPrim())  # avoid mass warning
        massApi.CreateMassAttr().Set(1.0)

        # setup a transform:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        xform.SetResetXformStack(True)
        xform.AddTranslateOp().Set(init_translation)
        xform.AddRotateXOp().Set(init_rotx)

        # step and check that it moved:
        self.step(5)
        pos = xform.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], init_translation[1])

        sanitized_xform_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(sanitized_xform_ops), 3)
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        for op, opName in zip(sanitized_xform_ops, opNames):
            self.assertEqual(op.GetName(), opName)
        self.assertTrue(xform.GetResetXformStack())

        if deleteAttributeDuringSim:
            xform.GetPrim().RemoveProperty('xformOp:rotateX')

        # reset and check that original ops order is restored and ops attributes as well
        get_physx_interface().reset_simulation()

        reset_xform_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(reset_xform_ops), 2)
        self.assertTrue(xform.GetResetXformStack())

        self.assertEqual(reset_xform_ops[0].GetOpType(), UsdGeom.XformOp.TypeTranslate)
        self.assertEqual(reset_xform_ops[0].Get(), init_translation)

        self.assertEqual(reset_xform_ops[1].GetOpType(), UsdGeom.XformOp.TypeRotateX)
        self.assertEqual(reset_xform_ops[1].Get(), init_rotx)

        # check that orphaned standard ops are removed:
        self.assertFalse(xform.GetPrim().GetAttribute("xformOp:scale"))
        self.assertFalse(xform.GetPrim().GetAttribute("xformOp:orient"))

    async def test_physics_rigidbody_reset_transform_test(self):
        await self._run_rigidbody_reset_transform_test()
        await self._run_rigidbody_reset_transform_test(True)

    async def _run_nondynamic_reset_and_sanitize_transform_test(self, kinematic=False):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xformable = None
        if kinematic:
            xform = UsdGeom.Xform.Define(stage, "/xform")
            xformable = UsdGeom.Xformable(xform.GetPrim())
            assert xformable
            rbAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
            rbAPI.CreateKinematicEnabledAttr().Set(True)
            massApi = UsdPhysics.MassAPI.Apply(xform.GetPrim())  # avoid mass warning
            massApi.CreateMassAttr().Set(1.0)
        else:
            xform = UsdGeom.Cube.Define(stage, "/xform")
            xform.CreateSizeAttr().Set(100.0)
            xformable = UsdGeom.Xformable(xform.GetPrim())
            assert xformable
            UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        # setup nonstandard transforms:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        xformable.AddTranslateOp().Set(init_translation)
        xformable.AddRotateXOp().Set(init_rotx)

        # step and check that it moved:
        self.step(1)

        sim_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(sim_ops), 2)
        opNames = ["xformOp:translate", "xformOp:rotateX"]
        for op, opName in zip(sim_ops, opNames):
            self.assertEqual(op.GetName(), opName)

        # remove an xform ops to verify that the ops are not restored
        xformable.CreateXformOpOrderAttr().Set(["xformOp:translate"])
        xformable.GetPrim().RemoveProperty('xformOp:rotateX')

        # set a new translation to check after reset:
        new_pos = Gf.Vec3f(1.9, 20.0, 5.5)
        post_sim_ops = xform.GetOrderedXformOps()
        post_sim_ops[0].Set(new_pos)

        # step again
        self.step(1)

        # reset and check that original ops were not restored
        get_physx_interface().reset_simulation()

        reset_xform_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(reset_xform_ops), 1)
        self.assertEqual(reset_xform_ops[0].GetOpType(), UsdGeom.XformOp.TypeTranslate)
        self.assertEqual(reset_xform_ops[0].Get(), new_pos)

    async def test_physics_static_collider_reset_sanitize_transform_test(self):
        await self._run_nondynamic_reset_and_sanitize_transform_test()

    async def test_physics_kinematic_rigidbody_reset_sanitize_transform_test(self):
        await self._run_nondynamic_reset_and_sanitize_transform_test(kinematic=True)

    async def test_physics_xformop_reset_structural_change(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = UsdGeom.Xform.Define(stage, "/xform")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        massApi = UsdPhysics.MassAPI.Apply(xform.GetPrim())  # avoid mass warning
        massApi.CreateMassAttr().Set(1.0)

        tranform = Gf.Transform(Gf.Vec3d(1.0), Gf.Rotation(Gf.Quatd(1.0)), Gf.Vec3d(5.0), Gf.Vec3d(0.0), Gf.Rotation(Gf.Quatd(1.0)))
        matrix = tranform.GetMatrix()

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTransformOp().Set(matrix)

        xform_op_order = xformable.GetXformOpOrderAttr().Get()

        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')

        self.step()

        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 3)        
        
        xform.GetPrim().GetAttribute("xformOp:scale").Set(Gf.Vec3f(2.0))
        
        self.step()
                
        get_physx_interface().reset_simulation()
        
        xform_op_order = xformable.GetXformOpOrderAttr().Get()
        
        self.assertTrue(len(xform_op_order) == 1)        
        self.assertEqual(xform_op_order[0], 'xformOp:transform')
        
        new_matrix = xformable.GetLocalTransformation()
        
        self.assertTrue(Gf.IsClose(matrix, new_matrix, 1e-7))
        
    async def test_physics_per_prim_local_space_velocities(self):
        # Setup scene with velocities in global space. Cube goes up as the velocity points up.
        stage = await self.new_stage()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0, 100.0, 0.0))
        physicsBaseUtils.set_local_space_velocities(cubePrim, False)

        utils.physics_scene_setup(stage)

        self.step()
        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] > 0.0)

        # Setup same scene but with velocities in local space. Cube falls down as the
        # velocity is in local space and the cube is rotated.
        stage = await self.new_stage()
        
        rotation = Gf.Quatf(0.0,1.0,0.0,0.0)
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0, 100.0, 0.0))
        physicsBaseUtils.set_local_space_velocities(cubePrim, True)
        
        utils.physics_scene_setup(stage)

        self.step()
        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)
        
    async def test_physics_per_prim_local_space_angular_velocities(self):

        # initial rotation of test cubes.
        rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 90.0)
        rotation_q = Gf.Quatf(rotation.GetQuat())

        # Setup scene with velocities in global space. Cube rotates around global Y.
        stage = await self.new_stage()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation_q)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 90.0, 0.0))
        physicsBaseUtils.set_local_space_velocities(cubePrim, False)

        utils.physics_scene_setup(stage)

        self.step()

        xform_rotation_q = cubePrim.GetAttribute("xformOp:orient").Get()
        xform_rotation_m = Gf.Matrix3f(xform_rotation_q)
        # Test if we've primarily been rotated around the global Y. We compare against X to compensate for float imprecision.
        self.assertTrue(abs(xform_rotation_m[1][2]) > abs(xform_rotation_m[0][2]))

        # Setup scene with velocities in local space. Cube rotates around local Y, which will be global X.
        stage = await self.new_stage()
        
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation_q)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 90.0, 0.0))
        physicsBaseUtils.set_local_space_velocities(cubePrim, True)        
        utils.physics_scene_setup(stage)

        self.step()
        
        xform_rotation_q = cubePrim.GetAttribute("xformOp:orient").Get()
        xform_rotation_m = Gf.Matrix3f(xform_rotation_q)
        # Test if we've primarily been rotated around the global X. We compare against Y to compensate for float imprecision.
        self.assertTrue(abs(xform_rotation_m[0][2]) > abs(xform_rotation_m[1][2]))

    async def test_physics_static_body_hierarchy(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        UsdGeom.Sphere.Define(stage, "/xform/sphere0")
        UsdGeom.Sphere.Define(stage, "/xform/sphere1")
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        self.step()
        
        utils.check_stats(self, { "numSphereShapes": 2, "numStaticRigids": 2 })
        
    async def test_physics_scene_update_disabled(self):
        stage = await self.new_stage()
        
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physx_scene_api.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Disabled)

        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, "/sphere", radius, position)

        self.step(5)
        self.assertTrue(physicsUtils.get_translation(sphere_prim)[1] == 0.0)
        
    async def test_physics_scene_simulate_iphysx(self):
        stage = await self.new_stage()
        
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physx_scene_api.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Disabled)

        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, "/sphere", radius, position)        
        
        for _ in range(2):
            get_physx_interface().update_simulation_scene(PhysicsSchemaTools.sdfPathToInt("/physicsScene"), 0.02, 0.0)
            get_physx_interface().update_transformations_scene(PhysicsSchemaTools.sdfPathToInt("/physicsScene"), True, True)
        
        self.assertTrue(physicsUtils.get_translation(sphere_prim)[1] < 0.0)
        
    async def test_physics_scene_simulate_iphysxsimulation(self):
        stage = await self.new_stage()
        
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physx_scene_api.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Disabled)

        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, "/sphere", radius, position)        
        
        for _ in range(2):
            get_physx_simulation_interface().simulate_scene(PhysicsSchemaTools.sdfPathToInt("/physicsScene"), 0.02, 0.0)
            get_physx_simulation_interface().fetch_results_scene(PhysicsSchemaTools.sdfPathToInt("/physicsScene"))
        
        self.assertTrue(physicsUtils.get_translation(sphere_prim)[1] < 0.0)
        
    async def test_physics_multiple_scene_simulation_owner_change(self):
        stage = await self.new_stage()
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # create multiple scenes        
        scene0 = UsdPhysics.Scene.Define(stage, "/physicsScene0")
        # create scene1 with gravity upwards
        scene1 = UsdPhysics.Scene.Define(stage, "/physicsScene1")
        scene1.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 1.0, 0.0))
        scene2 = UsdPhysics.Scene.Define(stage, "/physicsScene2")

        cube_prim = physicsUtils.add_rigid_box(stage, "/Cube", size=Gf.Vec3f(50.0))
        rbo_api = UsdPhysics.RigidBodyAPI(cube_prim)
        rbo_api.GetSimulationOwnerRel().SetTargets([scene0.GetPrim().GetPrimPath(), scene1.GetPrim().GetPrimPath(), scene2.GetPrim().GetPrimPath()])

        # Simulate a bit and let the cube fall by default gravity of scene0
        for _ in range(5):
            self.step()

        new_pos = cube_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(new_pos[1], -0.1)
        
        # change the sim owner rel to have the ownership owned by scene1
        rbo_api.GetSimulationOwnerRel().SetTargets([scene1.GetPrim().GetPrimPath(), scene2.GetPrim().GetPrimPath(), scene0.GetPrim().GetPrimPath()])
        rbo_api.GetVelocityAttr().Set(Gf.Vec3f(0.0))

        # Now simualte again and check if the body started to move up
        for _ in range(10):
            self.step()

        new_pos = cube_prim.GetAttribute("xformOp:translate").Get()
        self.assertGreater(new_pos[1], 0.1)

    async def test_physx_rigid_body_sleeping(self):
        stage = await self.new_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        
        utils.physics_scene_setup(stage)

        # box
        cubePrim = physicsUtils.add_cube(stage, "/boxActor", 10.0)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(cubePrim)
        physxAPI.CreateDisableGravityAttr(True)
        
        rbo_encoded = PhysicsSchemaTools.sdfPathToInt(cubePrim.GetPrimPath())

        # expect one body and one shape
        self.step()
        
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, rbo_encoded)
        self.assertTrue(not is_sleeping)

        for _ in range(50):
            self.step()

        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, rbo_encoded)
        self.assertTrue(is_sleeping)
        
        get_physx_simulation_interface().wake_up(stage_id, rbo_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, rbo_encoded)
        self.assertTrue(not is_sleeping)
        
        get_physx_simulation_interface().put_to_sleep(stage_id, rbo_encoded)
        self.step()
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, rbo_encoded)
        self.assertTrue(is_sleeping)

    async def test_physics_scene_gravity_magnitude(self):
        stage = await self.new_stage()
        
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        gravitymagnitudeattr = scene.CreateGravityMagnitudeAttr()
        gravitymagnitudeattr.Set(0.0)
        gravitymagnitudeattr.Set(time=0, value=0)
        gravitymagnitudeattr.Set(time=19, value=0)
        gravitymagnitudeattr.Set(time=20, value=981)
        gravitymagnitudeattr.Set(time=40, value=981)
                
        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, "/sphere", radius, position)        
        physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(sphere_prim.GetPrim())
        physx_rb_api.CreateSleepThresholdAttr().Set(0.0)

        for i in range(0, 40):
            get_physx_simulation_interface().simulate(1/60.0, i * 1/60)
            get_physx_simulation_interface().fetch_results()        
        
        epsilon = 1e-3
        self.assertTrue(abs(physicsUtils.get_translation(sphere_prim)[1]) < epsilon)
        
        for i in range(0, 40):
            get_physx_simulation_interface().simulate(1/60.0, 40 * 1/60 + i * 1/60)
            get_physx_simulation_interface().fetch_results()
        
        self.assertTrue(physicsUtils.get_translation(sphere_prim)[1] < -epsilon)
        
    async def test_physx_rigid_body_solve_contact(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 10.0)
        spherePrim = physicsUtils.add_rigid_sphere(stage, boxActorPath, radius, position)
        physxRbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(spherePrim.GetPrim())
        physxRbAPI.CreateSolveContactAttr().Set(False)

        physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        
        for _ in range(50):
            self.step()

        prim = stage.GetPrimAtPath(defaultPrimPath + boxActorPath)
        pos = prim.GetAttribute("xformOp:translate").Get()

        self.assertTrue(pos[2] < 0.0)
        
    async def test_physx_rigid_body_solve_contact_change(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 10.0)
        spherePrim = physicsUtils.add_rigid_sphere(stage, boxActorPath, radius, position)
        physxRbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(spherePrim.GetPrim())
        physxRbAPI.CreateSolveContactAttr().Set(True)

        physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        
        for _ in range(50):
            self.step()

        prim = spherePrim.GetPrim()
        pos = prim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] > 0.0)
        
        physxRbAPI.CreateSolveContactAttr().Set(False)
        
        for _ in range(20):
            self.step()
        
        pos = prim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 0.0)

    async def test_physx_sphere_points_rigid_body(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        particlePointsStr = defaultPrimPath + boxActorPath
        particlePointsPath = Sdf.Path(particlePointsStr)
        particles = UsdGeom.Points.Define(stage, particlePointsPath)

        radius = 20
        worldPos = Gf.Vec3f(0.0, 10.0, 0.0)
        positions_list = []
        widths_list = []
        lower = Gf.Vec3f(0.0, 0.0, 0) + worldPos
        particleSpacing = 2.0 * radius * 0.6
        x = lower[0]
        y = lower[1]
        z = lower[2]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    positions_list.append(Gf.Vec3f(x, y, z+50))
                    widths_list.append(2 * radius * 0.5)
                    z = z + particleSpacing
                z = lower[2]
                y = y + particleSpacing
            y = lower[1]
            x = x + particleSpacing

        positions = Vt.Vec3fArray(positions_list)
        widths = Vt.FloatArray(widths_list)

        particles.GetPointsAttr().Set(positions)
        particles.GetWidthsAttr().Set(widths)

        prim = stage.GetPrimAtPath(particlePointsPath)

        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)

        physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        #stage.Export("SpherePoints.usda")
            
        for _ in range(50):
            self.step()

        prim = stage.GetPrimAtPath(defaultPrimPath + boxActorPath)
        pos = prim.GetAttribute("xformOp:translate").Get()

        #print(f'ZPOS={pos[2]}')
        self.assertTrue(pos[2] > -40.001 and pos[2] < -39.999 )        


class PhysicsRigidBodyAPITestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_local_space_velocities(self):
        OUTPUT_VELOCITIES_LOCAL_SPACE = omni.physx.bindings._physx.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE

        settingsInterface = carb.settings.get_settings()
        outputVelocitiesLocalSpace = carb.settings.get_settings().get_as_bool(OUTPUT_VELOCITIES_LOCAL_SPACE)

        # Setup scene with velocities in global space. Cube goes up as the velocity points up.
        stage = await self.new_stage()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0, 100.0, 0.0))

        utils.physics_scene_setup(stage)
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, False)

        await self.step()
        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] > 0.0)

        # Setup same scene but with velocities in local space. Cube falls down as the
        # velocity is in local space and the cube is rotated.
        stage = await self.new_stage()
        
        rotation = Gf.Quatf(0.0,1.0,0.0,0.0)
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0, 100.0, 0.0))

        utils.physics_scene_setup(stage)
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, True)

        await self.step()
        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)

        # Restore OUTPUT_VELOCITIES_LOCAL_SPACE user setting
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, outputVelocitiesLocalSpace)

    async def test_physics_local_space_angular_velocities(self):
        OUTPUT_VELOCITIES_LOCAL_SPACE = omni.physx.bindings._physx.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE

        settingsInterface = carb.settings.get_settings()
        outputVelocitiesLocalSpace = carb.settings.get_settings().get_as_bool(OUTPUT_VELOCITIES_LOCAL_SPACE)

        # initial rotation of test cubes.
        rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 90.0)
        rotation_q = Gf.Quatf(rotation.GetQuat())

        # Setup scene with velocities in global space. Cube rotates around global Y.
        stage = await self.new_stage()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation_q)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 90.0, 0.0))

        utils.physics_scene_setup(stage)
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, False)

        await self.step()
        xform_rotation_q = cubePrim.GetAttribute("xformOp:orient").Get()
        xform_rotation_m = Gf.Matrix3f(xform_rotation_q)
        # Test if we've primarily been rotated around the global Y. We compare against X to compensate for float imprecision.
        self.assertTrue(abs(xform_rotation_m[1][2]) > abs(xform_rotation_m[0][2]))

        # Setup scene with velocities in local space. Cube rotates around local Y, which will be global X.
        stage = await self.new_stage()
        
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, orientation=rotation_q)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 90.0, 0.0))

        utils.physics_scene_setup(stage)
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, True)

        await self.step()
        xform_rotation_q = cubePrim.GetAttribute("xformOp:orient").Get()
        xform_rotation_m = Gf.Matrix3f(xform_rotation_q)
        # Test if we've primarily been rotated around the global X. We compare against Y to compensate for float imprecision.
        self.assertTrue(abs(xform_rotation_m[0][2]) > abs(xform_rotation_m[1][2]))
        
        # Restore OUTPUT_VELOCITIES_LOCAL_SPACE user setting
        settingsInterface.set_bool(OUTPUT_VELOCITIES_LOCAL_SPACE, outputVelocitiesLocalSpace)

    async def test_physics_prevent_kinematic_with_time_samples_switch_to_dynamic_body(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = UsdGeom.Xform.Define(stage, "/xform")

        cube = UsdGeom.Cube.Define(stage, "/xform/cube")

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        position = Gf.Vec3d(0.0, 0.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 500.0, 0.0)    
        xformable = UsdGeom.Xformable(xform.GetPrim())
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=50, value=positionEnd)

        utils.execute_and_check(self, "ToolbarPlayButtonClicked")
        await self.wait(1)
        physicsAPI.CreateKinematicEnabledAttr().Set(False)

        await self.wait(5)
        utils.execute_and_check(self, "ToolbarStopButtonClicked")
        self._check_physx_object_counts({"numBoxShapes": 1, "numKinematicBodies": 1, "numDynamicRigids": 0})

    async def test_physics_kinematic_switch_to_dynamic_body(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = UsdGeom.Xform.Define(stage, "/xform")

        cube = UsdGeom.Cube.Define(stage, "/xform/cube")

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        utils.execute_and_check(self, "ToolbarPlayButtonClicked")
        await self.wait(1)
        physicsAPI.CreateKinematicEnabledAttr().Set(False)

        await self.wait(5)
        utils.execute_and_check(self, "ToolbarStopButtonClicked")
        self._check_physx_object_counts({"numBoxShapes": 1, "numKinematicBodies": 0, "numDynamicRigids": 1})


class PhysicsRigidBodyAPITestAsyncRB(rigidbody.AsyncTestCase):
    category = TestCategory.Kit

    async def test_physics_commands_SetRigidBody_base(self):
        await self.base_apply_command_test(
            lambda primPath: SetRigidBodyCommand.execute(primPath, "", False),
            {"numBoxShapes": 1, "numDynamicRigids": 1},
            {"numSphereShapes": 1, "numDynamicRigids": 2},
            {"numCapsuleShapes": 1, "numDynamicRigids": 3},
            lambda primPath: RemoveRigidBodyCommand.execute(primPath),
            {"numBoxShapes": 0, "numSphereShapes": 0, "numCapsuleShapes": 0, "numDynamicRigids": 0}
        )

    async def test_physics_commands_SetRigidBody_undoredo(self):
        await self.base_undoredo_command_test(
            lambda primPath: SetRigidBodyCommand.execute(primPath, "", False)
        )

    async def test_physics_userpath_set_rigid_body(self):
        for approx in rigidbody.approximations:
            print(approx)
            await self.base_basic_userpath_command_test(
                lambda primPath: SetRigidBodyCommand.execute(primPath, approx, False),
                {"numBoxShapes": 1, "numDynamicRigids": 1}
            )

    async def test_physics_userpath_set_rigid_body_mesh(self):  # to cover meshes
        self.fail_on_log_error = True

        stage = await utils.new_stage_setup()
        utils.execute_and_check(self, "AddPhysicsScene", stage=stage, path="/physicsScene")

        prim_path = "/mesh"
        cubeMesh = UsdGeom.Mesh.Define(stage, prim_path)

        faceVertexCounts = [4, 4, 4, 4, 4, 4]
        faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]
        points = [
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
        ]

        cubeMesh.CreateFaceVertexCountsAttr(faceVertexCounts)
        cubeMesh.CreateFaceVertexIndicesAttr(faceVertexIndices)
        cubeMesh.CreatePointsAttr(points)

        SetRigidBodyCommand.execute(prim_path, "convexHull", False)
        await utils.play_and_step_and_pause(self, 5)
        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

    async def test_physics_commands_SetRigidBody_gravity(self):
        stage = await self.new_stage()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        utils.physics_scene_setup(stage)

        await self.step(5)
        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)

    async def test_physics_rigidbody_transform_sanitation(self):

        def isXformOp(attribName):
            return attribName.startswith("xformOp:")

        def isConformantXformOp(xformName):
            return (xformName == "xformOp:orient") or (xformName == "xformOp:scale") or (xformName == "xformOp:translate")

        stage = await self.new_stage()
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))

        # Add a non conformant XformOp to the cubePrim
        cubeGeom = UsdGeom.Cube.Get(stage, cubePrim.GetPath())
        cubeGeom.AddRotateXOp()
        # Check if any non-conformant xformOp is an attribute of the cube
        attributes = cubePrim.GetAttributes()
        nbrNonConformant = 0
        for a in attributes:
            attribName = a.GetName()
            if (isXformOp(attribName)):
                if (not isConformantXformOp(attribName)):
                    nbrNonConformant += 1
        # Run the Sim which also runs the xform sanitation
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)
        utils.physics_scene_setup(stage)
        await self.step()

        # Check that any non conformant xformOp was removed
        attributes = cubePrim.GetAttributes()
        nbrNonConformantAfter = 0
        for a in attributes:
            attribName = a.GetName()
            if (isXformOp(attribName)):
                if (not isConformantXformOp(attribName)):
                    nbrNonConformantAfter += 1
        self.assertTrue(nbrNonConformantAfter == 0)

    async def _run_physics_rigidbody_transform_sanitation_scale_attr(self, precision="double"):
        stage = await self.new_stage()

        cubeShape = UsdGeom.Cube.Define(stage, "/cubeActor")
        SetRigidBodyCommand.execute(cubeShape.GetPath(), "", False)
        initial_scale = Gf.Vec3f(0.1, 0.2, 0.3)
        if precision == "double":
            cubeShape.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(initial_scale)
        elif precision == "half":
            cubeShape.AddScaleOp(UsdGeom.XformOp.PrecisionHalf).Set(initial_scale)
        elif precision == "float":
            cubeShape.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(initial_scale)

        # remove scale from xform op, so we have an unscaled cube
        cubeShape.GetXformOpOrderAttr().Set([])

        await self.step()

        # Check that the sanitized xform ops stack has an original-precision scale that is close to unit scale
        stack = cubeShape.GetOrderedXformOps()
        self.assertEqual(len(stack), 3)
        # scale will be on top of stack:
        scale = stack[2].Get()
        for v in scale:
            self.assertAlmostEqual(v, 1.0, places=4)

        # step and stop and check restore to 0 went a-ok:
        await self.step(num_steps=1, stop_timeline_after=True)

        # since the xform ops storage purges everything not in the sim-start stack, the scale attribute will be gone:
        stack = cubeShape.GetOrderedXformOps()
        self.assertEqual(len(stack), 0)
        self.assertFalse(cubeShape.GetPrim().GetAttribute("xformOp:scale"))

    async def test_physics_rigidbody_transform_sanitation_scale_attribute(self):
        await self._run_physics_rigidbody_transform_sanitation_scale_attr(precision="double")
        await self._run_physics_rigidbody_transform_sanitation_scale_attr(precision="float")
        await self._run_physics_rigidbody_transform_sanitation_scale_attr(precision="half")

    async def test_physics_rigidbody_add_remove_collision(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        # setup a rigid body with xform body and a one collision under
        physicsUtils.add_rigid_xform(stage, "/rigidBody")
        cubePrim = physicsUtils.add_rigid_cube(stage, "/rigidBody/cube", density=0)

        # expect one body and one shape
        await self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})

        # expect one body and two shape
        spherePrim = physicsUtils.add_sphere(stage, "/rigidBody/sphere")
        UsdPhysics.CollisionAPI.Apply(spherePrim)

        await self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numSphereShapes": 1, "numDynamicRigids": 1})

        # expect one body and one shape
        stage.RemovePrim(cubePrim.GetPath())
        await self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numSphereShapes": 1, "numDynamicRigids": 1})

    def _on_error_event(self, event):
        if event.type == int(ErrorEvent.USD_LOAD_ERROR):
            self._errorMsg = event.payload['errorString']

    async def test_physics_rigidbody_trimesh_fallback(self):
        stage = await self.new_stage()
        stepper = utils.PhysicsStepper()

        events = get_physx_interface().get_error_event_stream()
        error_event_sub = events.create_subscription_to_pop(self._on_error_event)

        utils.physics_scene_setup(stage)

        # setup a rigid body with xform body and a one collision under
        ridigBody = physicsUtils.add_rigid_xform(stage, "/rigidBody")
        geomPlanePath = ridigBody.GetPrimPath().AppendChild("geom")
        physicsUtils.create_mesh_square_axis(stage, geomPlanePath, "Y", 100.0)
        planePrim = stage.GetPrimAtPath(geomPlanePath)
        UsdPhysics.CollisionAPI.Apply(planePrim)
        physicsMeshAPI = UsdPhysics.MeshCollisionAPI.Apply(planePrim)
        physicsMeshAPI.CreateApproximationAttr("sdf")

        # expect one body and one shape, triangle mesh should fall back to convex mesh
        await self.step()
        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

        # setup a kinematic rigid body with xform body and a one collision under
        self._errorMsg = None
        bodyPrim = physicsUtils.add_rigid_xform(stage, "/rigidBody1")
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, bodyPrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)
        geomPlanePath = "/rigidBody1/geom"
        physicsUtils.create_mesh_square_axis(stage, geomPlanePath, "Y", 100.0)
        planePrim = stage.GetPrimAtPath(geomPlanePath)
        UsdPhysics.CollisionAPI.Apply(planePrim)        
        physicsMeshAPI = UsdPhysics.MeshCollisionAPI.Apply(planePrim)
        physicsMeshAPI.CreateApproximationAttr("sdf")

        # expect one body and one shape
        await self.step()
        utils.check_stats(self, {"numTriMeshShapes": 1, "numKinematicBodies": 1})

        self.assertTrue(self._errorMsg is None)

        error_event_sub = None

    async def test_physics_rigidbody_none_fallback(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        # setup a rigid body with xform body and a one collision under
        ridigBody = physicsUtils.add_rigid_xform(stage, "/rigidBody")
        geomPlanePath = ridigBody.GetPrimPath().AppendChild("geom")
        physicsUtils.create_mesh_square_axis(stage, geomPlanePath, "Y", 100.0)
        planePrim = stage.GetPrimAtPath(geomPlanePath)                
        UsdPhysics.CollisionAPI.Apply(planePrim)
        
        # expect one body and one shape, triangle mesh should fall back to convex mesh
        await self.step()
        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

    async def test_physics_pointinstancer_update_velocities(self):
        UPDATE_VELOCITIES_TO_USD = omni.physx.bindings._physx.SETTING_UPDATE_VELOCITIES_TO_USD

        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        geomPointInstancerPath = "/pointinstancer"

        # Box instanced
        cubePrim = physicsUtils.add_cube(stage, geomPointInstancerPath + "/boxActor", 25.0)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.MassAPI.Apply(cubePrim)

        # indices
        meshIndices = [0, 0]
        positions = [Gf.Vec3f(-25.0, 0.0, 50.0), Gf.Vec3f(25.0, 0.0, 50.0)]
        orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(1.0, 0.0, 0.0, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        angularVelocities = [Gf.Vec3f(0.0, 10.0, 0.0), Gf.Vec3f(0.0, 10.0, 0.0)]

        # Create point instancer
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        shapeList = UsdGeom.PointInstancer.Define(stage, Sdf.Path(defaultPrimPath + geomPointInstancerPath))
        meshList = shapeList.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(cubePrim.GetPath())

        shapeList.GetProtoIndicesAttr().Set(meshIndices)
        shapeList.GetPositionsAttr().Set(positions)
        shapeList.GetOrientationsAttr().Set(orientations)
        shapeList.GetVelocitiesAttr().Set(linearVelocities)
        shapeList.GetAngularVelocitiesAttr().Set(angularVelocities)

        settingsInterface = carb.settings.get_settings()
        settingsInterface.set_bool(UPDATE_VELOCITIES_TO_USD, False)

        await self.step()

        newLinearVelocities = shapeList.GetVelocitiesAttr().Get()
        newAngularVelocities = shapeList.GetAngularVelocitiesAttr().Get()

        self.assertTrue(linearVelocities[0] == newLinearVelocities[0])
        self.assertTrue(linearVelocities[1] == newLinearVelocities[1])
        self.assertTrue(angularVelocities[0] == newAngularVelocities[0])
        self.assertTrue(angularVelocities[1] == newAngularVelocities[1])

        settingsInterface.set_bool(UPDATE_VELOCITIES_TO_USD, True)
        await self.step()

        newLinearVelocities = shapeList.GetVelocitiesAttr().Get()
        newAngularVelocities = shapeList.GetAngularVelocitiesAttr().Get()

        self.assertFalse(linearVelocities[0] == newLinearVelocities[0])
        self.assertFalse(linearVelocities[1] == newLinearVelocities[1])
        self.assertFalse(angularVelocities[0] == newAngularVelocities[0])
        self.assertFalse(angularVelocities[1] == newAngularVelocities[1])

    async def test_rigid_body_simulation_steps(self):
        stage = await self.new_stage()
        stage.SetTimeCodesPerSecond(60)

        # force editor and physics to have the same rate (should be 60)
        physics_rate = 60
        orig_limit = carb.settings.get_settings().get("/app/runLoops/main/rateLimitFrequency")
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(physics_rate))
        carb.settings.get_settings().set_int(SETTING_MIN_FRAME_RATE, int(physics_rate))
        dt = 1.0 / physics_rate

        # add scene
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        # Add a cube
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
        cubeGeom.CreateSizeAttr(100)
        cubePrim = stage.GetPrimAtPath(cubePath)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

        # test acceleration, velocity, position
        omni.timeline.get_timeline_interface().play()
        # warm up simulation
        await omni.kit.app.get_app().next_update_async()
        # get initial position
        p_0 = np.array(omni.usd.get_world_transform_matrix(cubePrim).ExtractTranslation())
        v_0 = np.array(rigidBodyAPI.GetVelocityAttr().Get())
        # simulate for one second
        time_elapsed = 0
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
            time_elapsed += dt
        p_1 = np.array(omni.usd.get_world_transform_matrix(cubePrim).ExtractTranslation())
        v_1 = np.array(rigidBodyAPI.GetVelocityAttr().Get())

        # check that acceleration matches gravity
        a = (v_1 - v_0) / time_elapsed
        # acceleration is correct
        # [   0.         0.      -980.99945]
        self.assertAlmostEqual(a[2], -981.0, 0)

        # check that analytical position matches expected
        p_expected = p_0 + v_0 * time_elapsed + 0.5 * a * time_elapsed ** 2

        omni.timeline.get_timeline_interface().stop()

        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(orig_limit))

    async def test_physics_rigidbody_remove_hierarchy(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        # setup a rigid body with xform body and a one collision under
        xformPrim = physicsUtils.add_xform(stage, "/xform")

        physicsUtils.add_rigid_xform(stage, "/xform/rigidBody")
        cubePrim = physicsUtils.add_rigid_cube(stage, "/xform/rigidBody/cube", density=0)

        # expect one body and one shape
        await self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})

        # expect no body nor shape
        stage.RemovePrim(xformPrim.GetPath())
        await self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0})

    async def test_physics_kinematic_rigidbody_transform_preserve(self):
        stage = await self.new_stage()

        utils.physics_scene_setup(stage)

        xform = UsdGeom.Cube.Define(stage, "/xform")

        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform.GetPrim())

        rigidBodyAPI.GetKinematicEnabledAttr().Set(True)

        position = Gf.Vec3f(0.0, -200.0, 100.0)
        positionEnd = Gf.Vec3f(0.0, 300.0, 100.0)

        translateOp = xform.AddTranslateOp() 
        translateOp.Set(position)
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=50, value=positionEnd)
        translateOp.Set(time=100, value=position)
        xform.AddScaleOp()

        ops = xform.GetOrderedXformOps()

        self.assertTrue(len(ops) == 2)

        # check if we dont overwrite it by one transform op or three ops by setup
        await self.step()

        ops = xform.GetOrderedXformOps()
        self.assertTrue(len(ops) == 2)

    async def test_physics_rigid_body_instancable_phantom_collision(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # step do initial parsing
        await self.step()

        xform = UsdGeom.Xform.Define(stage, "/xform")
        sphere = UsdGeom.Sphere.Define(stage, "/xform/sphere")
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddInternalReference("/xform")
        xform.GetPrim().SetInstanceable(True)

        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        await self.step()

        utils.check_stats(self, {"numStaticRigids": 1})
        utils.check_stats(self, {"numDynamicRigids": 1})        
