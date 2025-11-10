# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, Sdf, PhysxSchema
import omni.physxdemos as demo


class AnalyticConeDemo(demo.Base):
    title = "Analytic Cone"
    category = demo.Categories.CONTACTS
    short_description = "Demo showing the difference between an analytic cone shape and its convex mesh approximation"
    description = "Demo showing the difference between a mesh cone (blue), which uses a ConvexMesh collider, and a shape cone (magenta), which uses an analytic cone collision geometry. You can toggle Approximate Cones With Convex Meshes setting (off by default) in the scene Physics Settings to use analitic cone shapes or their approximations.\n\nPress play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        room = demo.get_demo_room(self, stage, zoom = 0.25)

        # Cone 0 (ConvexMesh)
        cone0ActorPath = defaultPrimPath + "/cone0Actor"

        scaleFactor = 0.125

        radius = 50.0 * scaleFactor
        height = 100.0 * scaleFactor
        position = Gf.Vec3f(250.0, -250.0, 100.0) * scaleFactor
        orientation = Gf.Quatf(0.0, 0.38268, 0.0, 0.92388)
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 1000.0)

        cone0Geom = physicsUtils.create_mesh_cone(stage, cone0ActorPath, height, radius)
        cone0Prim = stage.GetPrimAtPath(cone0ActorPath)
        cone0Geom.AddTranslateOp().Set(position)
        cone0Geom.AddOrientOp().Set(orientation)
        cone0Geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        cone0Geom.CreateDisplayColorAttr().Set([demo.get_primary_color(0)])

        UsdPhysics.CollisionAPI.Apply(cone0Prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cone0Prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cone0Prim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cone0Prim)

        # Cone 1 (CustomGeometry)
        cone1ActorPath = defaultPrimPath + "/cone1Actor"

        radius = 50.0 * scaleFactor
        height = 100.0 * scaleFactor
        position = Gf.Vec3f(-250.0, 250.0, 100.0) * scaleFactor
        orientation = Gf.Quatf(0.0, 0.38268, 0.0, 0.92388)
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 1000.0)

        cone1Geom = UsdGeom.Cone.Define(stage, cone1ActorPath)
        cone1Prim = stage.GetPrimAtPath(cone1ActorPath)
        cone1Geom.CreateHeightAttr(height)
        cone1Geom.CreateRadiusAttr(radius)
        cone1Geom.CreateExtentAttr([(-radius, -radius, -height/2), (radius, radius, height/2)])
        cone1Geom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        cone1Geom.AddTranslateOp().Set(position)
        cone1Geom.AddOrientOp().Set(orientation)
        cone1Geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        cone1Geom.CreateDisplayColorAttr().Set([demo.get_primary_color(1)])

        UsdPhysics.CollisionAPI.Apply(cone1Prim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cone1Prim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cone1Prim)
