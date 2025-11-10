# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class PrismaticJointDemo(demo.Base):
    title = "Prismatic joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic prismatic joint usage"
    description = "Demo showing basic prismatic joint usage."
    tags = ["Joints"]

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)


        # box0 static
        boxActorPath = defaultPrimPath + "/box0"

        position = Gf.Vec3f(0.0, 0.0, 1000.0)
        orientation = Gf.Quatf(1.0)
        color = demo.get_static_color()
        size = 100.0
        scale = Gf.Vec3f(0.1, 1.0, 0.1)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        # Box1
        boxActorPath = defaultPrimPath + "/box1"

        size = 100.0
        position = Gf.Vec3f(0.0, 120.0, 1000.0)
        color = demo.get_primary_color()
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        scale = Gf.Vec3f(0.1, 1.0, 0.1)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

        # prismatic joint
        prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, defaultPrimPath + "/prismaticJoint")

        prismaticJoint.CreateAxisAttr("Y")
        prismaticJoint.CreateLowerLimitAttr(-50)
        prismaticJoint.CreateUpperLimitAttr(50)

        prismaticJoint.CreateBody0Rel().SetTargets([defaultPrimPath + "/box0"])
        prismaticJoint.CreateBody1Rel().SetTargets([defaultPrimPath + "/box1"])

        prismaticJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        prismaticJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        prismaticJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        prismaticJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # add angular drive
        linearDriveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(defaultPrimPath + "/prismaticJoint"), "linear")
        linearDriveAPI.CreateTypeAttr("force")
        linearDriveAPI.CreateTargetPositionAttr(40.0)
        linearDriveAPI.CreateDampingAttr(0.0)
        linearDriveAPI.CreateStiffnessAttr(100.0)
        
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0, floorOffset = 870.0)
