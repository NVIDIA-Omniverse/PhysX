# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class CenterOfMassDemo(demo.Base):
    title = "Center of mass"
    category = demo.Categories.RIGID_BODIES
    short_description = "Basic demo showing usage of center of mass on a rigid body"
    description = "Basic demo showing usage of center of mass on a rigid body. The magenta snowman's center of mass is close to the ground position which changes its behavior."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage)

        # Snowmans
        for i in range(2):
            currentOffset = Gf.Vec3f(0.0, i * 200.0 - 100.0, room._floorOffset + 40.0)
            position = Gf.Vec3f(0.0, 0.0, 0.0) + currentOffset
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            linVelocity = Gf.Vec3f(0.0)
            angularVelocity = Gf.Vec3f(0.0)
            color = demo.get_primary_color(0)
            if i == 0:
                color = demo.get_primary_color(1)

            snowmanActorPath = defaultPrimPath + "/snowman" + str(i)
            snowmanXform = UsdGeom.Xform.Define(stage, snowmanActorPath)
            snowmanXform.AddTranslateOp().Set(position)
            snowmanXform.AddOrientOp().Set(orientation)
            snowmanXform.AddScaleOp().Set(Gf.Vec3f(1.0))
            snowmanPrim = stage.GetPrimAtPath(snowmanActorPath)
            physicsAPI = UsdPhysics.RigidBodyAPI.Apply(snowmanPrim)
            physicsAPI.CreateVelocityAttr().Set(linVelocity)
            physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
            massAPI = UsdPhysics.MassAPI.Apply(snowmanPrim)
            if i == 0:
                massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 10.0) + currentOffset)

            # lower sphere
            lowerRadius = 50.0
            sphereActorPath0 = snowmanActorPath + "/lowerSphere"
            position = Gf.Vec3f(0.0, 0.0, lowerRadius) + currentOffset
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath0)
            spherePrim = stage.GetPrimAtPath(sphereActorPath0)
            sphereGeom.CreateRadiusAttr(lowerRadius)
            sphereGeom.CreateExtentAttr([(-lowerRadius, -lowerRadius, -lowerRadius), (lowerRadius, lowerRadius, lowerRadius)])
            sphereGeom.AddTranslateOp().Set(position)
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)

            # middle sphere
            sphereActorPath1 = snowmanActorPath + "/middleSphere"
            middleRadius = 30.0
            position = Gf.Vec3f(0.0, 0.0, 2.0 * lowerRadius + middleRadius) + currentOffset

            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath1)
            spherePrim = stage.GetPrimAtPath(sphereActorPath1)
            sphereGeom.CreateRadiusAttr(middleRadius)
            sphereGeom.CreateExtentAttr([(-middleRadius, -middleRadius, -middleRadius), (middleRadius, middleRadius, middleRadius)])
            sphereGeom.AddTranslateOp().Set(position)
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)

            # left arm sphere
            sphereActorPath1 = snowmanActorPath + "/leftArmSphere"
            armRadius = 10.0
            position = Gf.Vec3f(middleRadius + armRadius, 0.0, 2.0 * lowerRadius + middleRadius) + currentOffset

            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath1)
            spherePrim = stage.GetPrimAtPath(sphereActorPath1)
            sphereGeom.CreateRadiusAttr(armRadius)
            sphereGeom.CreateExtentAttr([(-armRadius, -armRadius, -armRadius), (armRadius, armRadius, armRadius)])
            sphereGeom.AddTranslateOp().Set(position)
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)

            # right arm sphere
            sphereActorPath1 = snowmanActorPath + "/rightArmSphere"
            position = Gf.Vec3f(-middleRadius - armRadius, 0.0, 2.0 * lowerRadius + middleRadius) + currentOffset

            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath1)
            spherePrim = stage.GetPrimAtPath(sphereActorPath1)
            sphereGeom.CreateRadiusAttr(armRadius)
            sphereGeom.CreateExtentAttr([(-armRadius, -armRadius, -armRadius), (armRadius, armRadius, armRadius)])
            sphereGeom.AddTranslateOp().Set(position)
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)

            # upper sphere
            sphereActorPath2 = snowmanActorPath + "/upperSphere"
            upperRadius = 20.0
            position = Gf.Vec3f(0.0, 0.0, 2.0 * lowerRadius + 2.0 * middleRadius + upperRadius) + currentOffset

            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath2)
            spherePrim = stage.GetPrimAtPath(sphereActorPath2)
            sphereGeom.CreateRadiusAttr(upperRadius)
            sphereGeom.CreateExtentAttr([(-upperRadius, -upperRadius, -upperRadius), (upperRadius, upperRadius, upperRadius)])
            sphereGeom.AddTranslateOp().Set(position)
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)

            room.set_physics_material(snowmanActorPath, room._metalMaterialPath, True)
