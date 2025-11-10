# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import Sdf, UsdLux, UsdGeom, Gf, UsdPhysics
import omni.physxdemos as demo
import omni.timeline

class ConveyorBeltDemo(demo.Base):
    title = "Conveyor Belt"
    category = demo.Categories.CONTACTS
    short_description = "Demo showing conveyor belt behavior (linear / angular velocity)"
    description = "Demo showing conveyor belt behavior, a kinematic body with animated linear velocity attribute will convert the velocity into surface velocity. The ground plane is also a kinematic rigid body with angular velocity. Press play (space) to run the simulation."

    def create(self, stage):
        size = 25.0
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, zoom = 0.5, hasTable = False, floorOffset = -size)

        # Kinematic Circular Conveyor
        cylinderActorPath = defaultPrimPath + "/kinematicCircularConveyor"
        cylinderGeom = UsdGeom.Cylinder.Define(stage, cylinderActorPath)
        radius = size * 8.0
        height = size * 0.5
        cylinderGeom.CreateHeightAttr(height)
        cylinderGeom.CreateRadiusAttr(radius)
        cylinderGeom.GetAxisAttr().Set("Z")
        cylinderGeom.CreateExtentAttr().Set([(-radius, -radius, -height * 0.5), (radius, radius, height * 0.5)])
        cylinderPrim = stage.GetPrimAtPath(cylinderActorPath)
        cylinderGeom.CreateDisplayColorAttr().Set([demo.get_primary_color(0.5)])

        UsdPhysics.CollisionAPI.Apply(cylinderPrim)
        angularConveyor = UsdPhysics.RigidBodyAPI.Apply(cylinderPrim)
        angularConveyor.CreateKinematicEnabledAttr().Set(True)
        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 0.0, size * 0.25)
        angularConveyor.GetAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

        # Kinematic Linear Conveyor
        boxActorPath = defaultPrimPath + "/kinematicActor"
        position = Gf.Vec3f(0, 0, size)
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        half_extent = 0.5
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(0.5, 10.0, 0.5) * size) 
        cubeGeom.CreateDisplayColorAttr().Set([demo.get_primary_color(1.0)])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        linarConveyor = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        linarConveyor.CreateKinematicEnabledAttr().Set(True)
        velocityAttribute = linarConveyor.GetVelocityAttr()
        targetSurfaceVelocity = Gf.Vec3f(0.0, size * 3.0, 0.0)
        velocityAttribute.Set(time=0, value=targetSurfaceVelocity)
        velocityAttribute.Set(time=30, value=targetSurfaceVelocity)
        velocityAttribute.Set(time=31, value=-targetSurfaceVelocity)
        velocityAttribute.Set(time=60, value=-targetSurfaceVelocity)
        omni.timeline.get_timeline_interface().set_end_time(200/24)

        # dynamic actor
        boxActorPath = defaultPrimPath + "/boxActor"
        position = Gf.Vec3f(0.0, 0.0, size * 2.0)
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        half_extent = 0.5
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(size * 0.5)) 
        cubeGeom.CreateDisplayColorAttr().Set([demo.get_primary_color()])
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
