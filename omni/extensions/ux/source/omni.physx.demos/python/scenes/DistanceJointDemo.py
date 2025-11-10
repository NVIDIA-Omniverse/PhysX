# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class DistanceJointDemo(demo.Base):
    title = "Distance joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic distance joint usage"
    description = "Demo showing basic distance joint usage."
    tags = ["Joints"]

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0, floorOffset = 820.0)

        # box0 static
        boxActorPath = defaultPrimPath + "/box0"
        boxActor0JointFramePath = defaultPrimPath + "/box0/jointFrame"

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

        jointFrame = UsdGeom.Xform.Define(stage, boxActor0JointFramePath)
        position = Gf.Vec3f(0.0, 50.0, 0.0)
        jointFrame.AddTranslateOp().Set(position)
        jointFrame.AddOrientOp().Set(orientation)

        # Box1
        boxActorPath = defaultPrimPath + "/box1"
        boxActor1JointFramePath = defaultPrimPath + "/box1/jointFrame"

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

        jointFrame = UsdGeom.Xform.Define(stage, boxActor1JointFramePath)
        position = Gf.Vec3f(0.0, -50.0, 0.0)
        jointFrame.AddTranslateOp().Set(position)
        jointFrame.AddOrientOp().Set(orientation)


        # distance joint
        distanceJoint = UsdPhysics.DistanceJoint.Define(stage, defaultPrimPath + "/distanceJoint")
        
        distanceJoint.CreateMinDistanceAttr(10)
        distanceJoint.CreateMaxDistanceAttr(50)

        distanceJoint.CreateBody0Rel().SetTargets([boxActor0JointFramePath])
        distanceJoint.CreateBody1Rel().SetTargets([boxActor1JointFramePath])

        distanceJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        distanceJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        distanceJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        distanceJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
