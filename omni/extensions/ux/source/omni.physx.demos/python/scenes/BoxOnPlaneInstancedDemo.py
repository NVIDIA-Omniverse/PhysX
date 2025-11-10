# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class BoxOnPlaneInstancedDemo(demo.Base):
    title = "Instanced box on a table"
    category = demo.Categories.RIGID_BODIES
    short_description = "Basic demo showing two rigid bodies defined through a point instancer falling on a table"
    description = "Basic demo showing two rigid bodies defined through a point instancer falling on a table. Press play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage)

        geomPointInstancerPath = defaultPrimPath + "/pointinstancer"

        # Box instanced
        boxActorPath = geomPointInstancerPath + "/boxActor"
        size = 25.0

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
        cubeGeom.CreateDisplayColorAttr().Set([demo.get_primary_color(0)])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)        
        UsdPhysics.MassAPI.Apply(cubePrim)

        room.set_physics_material(boxActorPath, room._plasticMaterialPath, True)

        # indices
        meshIndices = [0, 0]
        positions = [Gf.Vec3f(-50.0, 0.0, 300.0), Gf.Vec3f(50.0, 0.0, 300.0)]
        orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(0.8660254, 0.0, 0.5, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        angularVelocities = [Gf.Vec3f(0.0, 10.0, 0.0), Gf.Vec3f(0.0)]

        # Create point instancer
        shapeList = UsdGeom.PointInstancer.Define(stage, Sdf.Path(geomPointInstancerPath))
        meshList = shapeList.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(Sdf.Path(boxActorPath))

        shapeList.GetProtoIndicesAttr().Set(meshIndices)
        shapeList.GetPositionsAttr().Set(positions)
        shapeList.GetOrientationsAttr().Set(orientations)
        shapeList.GetVelocitiesAttr().Set(linearVelocities)
        shapeList.GetAngularVelocitiesAttr().Set(angularVelocities)
