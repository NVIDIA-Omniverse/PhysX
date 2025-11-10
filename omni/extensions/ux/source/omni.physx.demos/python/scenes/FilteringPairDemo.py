# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class FilteringPairDemo(demo.Base):
    title = "Fitlering pair"
    category = demo.Categories.SIMULATION_PART
    short_description = "Demo showing collision pair filtering"
    description = "Demo showing collision pair filtering, the two boxes should not collide with each other. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        
        room = demo.get_demo_room(self, stage)

        # Box0
        boxActorPath = defaultPrimPath + "/boxActor0"

        density = 0.001
        size = Gf.Vec3f(25.0)
        position = Gf.Vec3f(0.0, 0.0, 300.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = demo.get_primary_color(0.0)
        linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
        angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

        add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        # Box1
        boxActorPath = defaultPrimPath + "/boxActor1"
        position = Gf.Vec3f(0.0, 0.0, 150.0)
        color = demo.get_primary_color(1.0)
        add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        # filter collisions between Box0, Box1
        boxPrim = stage.GetPrimAtPath(Sdf.Path(defaultPrimPath + "/boxActor0"))
        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(boxPrim)
        rel = filteringPairsAPI.CreateFilteredPairsRel()
        rel.AddTarget(Sdf.Path(defaultPrimPath + "/boxActor1"))
