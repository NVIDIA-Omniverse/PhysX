# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class D6JointDemo(demo.Base):
    title = "D6 joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic D6 joint usage"
    description = (
        "Demo showing basic D6 joint usage. Configure locked axes using the checkboxes (checked = locked) "
        " and reloading the scene. Or remove/add locking limits in the property window."
    )
    tags = ["Joints"]

    params = {
        "Trans_X": demo.CheckboxParam(True),
        "Trans_Y": demo.CheckboxParam(True),
        "Trans_Z": demo.CheckboxParam(True),
        "Rot_X": demo.CheckboxParam(True),
        "Rot_Y": demo.CheckboxParam(False),
        "Rot_Z": demo.CheckboxParam(True),
    }

    def create(self, stage, Trans_X, Trans_Y, Trans_Z, Rot_X, Rot_Y, Rot_Z):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0, floorOffset = 870.0)

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

        # D6 fixed joint
        d6Joint = UsdPhysics.Joint.Define(stage, defaultPrimPath + "/d6Joint")

        d6Joint.CreateBody0Rel().SetTargets([defaultPrimPath + "/box0"])
        d6Joint.CreateBody1Rel().SetTargets([defaultPrimPath + "/box1"])

        worldOrigin = Gf.Vec3f(room._originWorld)

        d6Joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        d6Joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        d6Joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        d6Joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # lock all DOF (lock - low is greater than high)
        d6Prim = d6Joint.GetPrim()
        if Trans_X:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transX)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Trans_Y:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transY)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Trans_Z:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transZ)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Rot_X:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotX)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Rot_Y:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotY)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Rot_Z:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotZ)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
