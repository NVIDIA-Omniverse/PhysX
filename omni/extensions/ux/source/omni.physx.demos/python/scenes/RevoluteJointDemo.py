# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class RevoluteJointDemo(demo.Base):
    title = "Revolute joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic revolute joint usage"
    description = "Demo showing basic revolute joint usage."
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

        # revolute joint
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, defaultPrimPath + "/revoluteJoint")

        revoluteJoint.CreateAxisAttr("X")
        revoluteJoint.CreateLowerLimitAttr(-90.0)
        revoluteJoint.CreateUpperLimitAttr(90)

        revoluteJoint.CreateBody0Rel().SetTargets([defaultPrimPath + "/box0"])
        revoluteJoint.CreateBody1Rel().SetTargets([defaultPrimPath + "/box1"])

        revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # add angular drive
        angularDriveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(defaultPrimPath + "/revoluteJoint"), "angular")
        angularDriveAPI.CreateTypeAttr("force")
        angularDriveAPI.CreateTargetVelocityAttr(20.0)
        angularDriveAPI.CreateDampingAttr(1e10)
        angularDriveAPI.CreateStiffnessAttr(0.0)
        
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0, floorOffset = 870.0)
