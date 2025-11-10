# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo

def setupGeom(geom, color, position, orientation):
    geom.AddTranslateOp().Set(position)
    geom.AddOrientOp().Set(orientation)
    geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    geom.CreateDisplayColorAttr().Set([color])

def setupRigidBody(prim, linVelocity, angularVelocity):
    UsdPhysics.CollisionAPI.Apply(prim)
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
    physicsAPI.CreateVelocityAttr().Set(linVelocity)
    physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
    UsdPhysics.MassAPI.Apply(prim)

class VariousShapesOnTrimeshDemo(demo.Base):
    title = "Various shapes on a table"
    category = demo.Categories.BASICS
    short_description = "Basic demo showing all supported basic shape types of a rigid body"
    description = "Basic demo showing all supported basic shape types of a rigid body falling on a table. Press play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."
    
    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        idtOrientation = Gf.Quatf(1.0)

        shapeScale = 0.12
        heightOffset = 180.0

        # Box
        x = -700*shapeScale
        boxActorPath = defaultPrimPath + "/boxActor"
        size = 100.0*shapeScale
        position = Gf.Vec3f(x, 0.0, heightOffset)
        color = demo.get_primary_color(0.0)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(180.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        setupGeom(cubeGeom, color, position, idtOrientation)

        setupRigidBody(cubePrim, linVelocity, angularVelocity)

        # Sphere
        x += 250.0*shapeScale
        sphereActorPath = defaultPrimPath + "/sphereActor"
        radius = 50.0*shapeScale
        position = Gf.Vec3f(x, 0.0, heightOffset)
        color = demo.get_primary_color(0.2)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(2.0)

        sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath)
        spherePrim = stage.GetPrimAtPath(sphereActorPath)
        sphereGeom.CreateRadiusAttr(radius)
        sphereGeom.CreateExtentAttr([(-radius, -radius, -radius), (radius, radius, radius)])
        setupGeom(sphereGeom, color, position, idtOrientation)

        setupRigidBody(spherePrim, linVelocity, angularVelocity)

        # Capsule
        x += 250.0*shapeScale
        capsuleActorPath = defaultPrimPath + "/capsuleActor"
        radius = 50.0*shapeScale
        height = 100.0*shapeScale
        position = Gf.Vec3f(x, 0.0, heightOffset)
        color = demo.get_primary_color(0.4)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(2.0)

        capsuleGeom = UsdGeom.Capsule.Define(stage, capsuleActorPath)
        capsulePrim = stage.GetPrimAtPath(capsuleActorPath)
        capsuleGeom.CreateHeightAttr(height)
        capsuleGeom.CreateRadiusAttr(radius)
        capsuleGeom.CreateExtentAttr([(-radius, -radius, -(height/2 + radius)), (radius, radius, height/2 + radius)])
        capsuleGeom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        setupGeom(capsuleGeom, color, position, idtOrientation)

        setupRigidBody(capsulePrim, linVelocity, angularVelocity)

        # Cylinder
        x += 250.0*shapeScale
        cylinderActorPath = defaultPrimPath + "/cylinderActor"
        radius = 50.0*shapeScale
        height = 100.0*shapeScale
        position = Gf.Vec3f(x, 0.0, heightOffset)
        hRt2 = math.sqrt(2.0) / 2.0
        orientation = Gf.Quatf(hRt2, 0.0, 0.0, -hRt2)
        color = demo.get_primary_color(0.6)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(2.0)

        cylinderGeom = UsdGeom.Cylinder.Define(stage, cylinderActorPath)
        cylinderPrim = stage.GetPrimAtPath(cylinderActorPath)
        cylinderGeom.CreateHeightAttr(height)
        cylinderGeom.CreateRadiusAttr(radius)
        cylinderGeom.CreateExtentAttr([(-radius, -radius, -height/2), (radius, radius, height/2)])
        cylinderGeom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        setupGeom(cylinderGeom, color, position, orientation)

        setupRigidBody(cylinderPrim, linVelocity, angularVelocity)

        # Cone
        x += 250.0*shapeScale
        coneActorPath = defaultPrimPath + "/coneActor"
        radius = 50.0*shapeScale
        height = 100.0*shapeScale
        position = Gf.Vec3f(x, 0.0, heightOffset)
        color = demo.get_primary_color(0.8)

        coneGeom = UsdGeom.Cone.Define(stage, coneActorPath)
        conePrim = stage.GetPrimAtPath(coneActorPath)
        coneGeom.CreateHeightAttr(height)
        coneGeom.CreateRadiusAttr(radius)
        coneGeom.CreateExtentAttr([(-radius, -radius, -height/2), (radius, radius, height/2)])
        coneGeom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        setupGeom(coneGeom, color, position, idtOrientation)

        setupRigidBody(conePrim, linVelocity, angularVelocity)

        # Convex
        x += 250.0*shapeScale
        convexActorPath = defaultPrimPath + "/convexActor"

        position = Gf.Vec3f(x, 0.0, heightOffset)
        color = demo.get_primary_color(1.0)

        convexGeom = UsdGeom.Mesh.Define(stage, convexActorPath)
        convexPrim = stage.GetPrimAtPath(convexActorPath)

        faceVertexCounts = [4, 4, 4, 4, 4, 4]
        faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]
        convexSize = 50*shapeScale
        convexSize2 = 25*shapeScale
        points = [
            Gf.Vec3f(-convexSize, convexSize, -convexSize),   Gf.Vec3f(convexSize, convexSize, -convexSize),   Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2, convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
            Gf.Vec3f(-convexSize, convexSize, -convexSize),   Gf.Vec3f(convexSize, convexSize, -convexSize),   Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2, convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
            Gf.Vec3f(-convexSize, convexSize, -convexSize),   Gf.Vec3f(convexSize, convexSize, -convexSize),   Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2, convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
        ]

        convexGeom.CreateFaceVertexCountsAttr(faceVertexCounts)
        convexGeom.CreateFaceVertexIndicesAttr(faceVertexIndices)
        convexGeom.CreatePointsAttr(points)

        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(convexPrim)
        meshCollisionAPI.CreateApproximationAttr("convexHull")

        setupGeom(convexGeom, color, position, idtOrientation)

        setupRigidBody(convexPrim, linVelocity, angularVelocity)

        room = demo.get_demo_room(self, stage, zoom = 0.5)

        room.set_physics_material(boxActorPath, room._plasticMaterialPath, True)
        room.set_physics_material(sphereActorPath, room._plasticMaterialPath, True)
        room.set_physics_material(capsuleActorPath, room._plasticMaterialPath, True)
        room.set_physics_material(cylinderActorPath, room._plasticMaterialPath, True)
        room.set_physics_material(coneActorPath, room._plasticMaterialPath, True)
        room.set_physics_material(convexActorPath, room._plasticMaterialPath, True)
