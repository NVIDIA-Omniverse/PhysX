# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class MultiShapeBodyOnPlaneDemo(demo.Base):
    title = "Multishape body on a table"
    category = demo.Categories.BASICS
    short_description = "Basic demo showing a simple compound rigid body falling on a table"
    description = "Basic demo showing a simple compound rigid body falling on a table. Press play (space) to run the simulation."
    
    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage)

        rigidCompoundPath = defaultPrimPath + "/compoundRigid"
        rigidCompoundPos = Gf.Vec3f(0.0, 0.0, 300.0)

        size = 25.0

        # Top level actor, contains rigid body and its shapes
        rigidXform = UsdGeom.Xform.Define(stage, rigidCompoundPath)
        rigidPrim = stage.GetPrimAtPath(rigidCompoundPath)

        # Rigid body transform
        rigidXform.AddTranslateOp().Set(rigidCompoundPos)
        rigidXform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(rigidPrim)
        UsdPhysics.MassAPI.Apply(rigidPrim)

        # Collision shapes
        collisionShapePath0 = rigidCompoundPath + "/physicsBoxShape0"

        shapePos = Gf.Vec3f(0.0, 0.0, -25.0)
        shapeQuat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        shapeColor = demo.get_primary_color(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath0)
        cubePrim = stage.GetPrimAtPath(collisionShapePath0)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(shapePos)
        cubeGeom.AddOrientOp().Set(shapeQuat)
        cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        collisionShapePath1 = rigidCompoundPath + "/physicsBoxShape1"

        shapePos = Gf.Vec3f(0.0, 50.0, -25.0)
        shapeQuat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        shapeColor = demo.get_primary_color(1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath1)
        cubePrim = stage.GetPrimAtPath(collisionShapePath1)
        cubeGeom.CreateSizeAttr(size / 2.0)
        half_extent = half_extent / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(shapePos)
        cubeGeom.AddOrientOp().Set(shapeQuat)
        cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        room.set_physics_material(rigidCompoundPath, room._plasticMaterialPath, True)
