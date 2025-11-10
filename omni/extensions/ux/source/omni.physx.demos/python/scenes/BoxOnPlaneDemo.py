# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics
import omni.physxdemos as demo

class BoxOnPlaneDemo(demo.Base):
    title = "Box on a table"
    category = demo.Categories.BASICS
    short_description = "Basic demo showing a box rigid body falling on a table"
    description = "Basic demo showing a box rigid body falling on a table. Press play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage)

        # Box
        boxActorPath = defaultPrimPath + "/boxActor"

        size = 25.0
        position = Gf.Vec3f(0.0, 0.0, 220.0)
        orientation = Gf.Quatf(1.0)
        linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
        angularVelocity = Gf.Vec3f(270.0, 0.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
        cubeGeom.CreateDisplayColorAttr().Set([demo.get_primary_color()])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

        room.set_physics_material(boxActorPath, room._plasticMaterialPath, True)
