# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class MaterialDensityDemo(demo.Base):
    title = "Material density"
    category = demo.Categories.MATERIALS
    short_description = "Demo showing different material density behavior"
    description = "Demo showing different material density behavior, the magenta box has a much higher density and therefore moves the blue box much farther. Note that gravity is not applied on this scene. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, gravityMod = 0.0)
        room = demo.get_demo_room(self, stage, hasTable = False, zoom = 0.5)

        # Box materials
        path = defaultPrimPath + "/material"
        UsdShade.Material.Define(stage, path)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
        material.CreateStaticFrictionAttr().Set(0)
        material.CreateDynamicFrictionAttr().Set(0)
        material.CreateRestitutionAttr().Set(0)
        material.CreateDensityAttr().Set(0.001)

        path = defaultPrimPath + "/highdensitymaterial"
        UsdShade.Material.Define(stage, path)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
        material.CreateStaticFrictionAttr().Set(0)
        material.CreateDynamicFrictionAttr().Set(0)
        material.CreateRestitutionAttr().Set(0)
        material.CreateDensityAttr().Set(0.01)

        # Boxes

        size = Gf.Vec3f(25.0)
        orientation = Gf.Quatf(1.0)
        v0 = 50

        cube_prim = add_box(stage, defaultPrimPath + "/box0", size, Gf.Vec3f(0.0, 0.0, 50.0), orientation, demo.get_primary_color(0.0))
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        rigid_body_api.CreateVelocityAttr().Set(Gf.Vec3f(v0, 0, 0))
        UsdPhysics.CollisionAPI.Apply(cube_prim)

        cube_prim = add_box(stage, defaultPrimPath + "/box1", size, Gf.Vec3f(250.0, 0.0, 50.0), orientation, demo.get_primary_color(0.0))
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)

        cube_prim = add_box(stage, defaultPrimPath + "/box2", size, Gf.Vec3f(0.0, 75.0, 50.0), orientation, demo.get_primary_color(1.0))
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        rigid_body_api.CreateVelocityAttr().Set(Gf.Vec3f(v0, 0, 0))
        UsdPhysics.CollisionAPI.Apply(cube_prim)

        cube_prim = add_box(stage, defaultPrimPath + "/box3", size, Gf.Vec3f(250.0, 75.0, 50.0), orientation, demo.get_primary_color(0.0))
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)

        # Add materials
        collisionBoxPath = defaultPrimPath + "/box0"
        add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(collisionBoxPath)), Sdf.Path(defaultPrimPath + "/material"))

        collisionBoxPath = defaultPrimPath + "/box1"
        add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(collisionBoxPath)), Sdf.Path(defaultPrimPath + "/material"))

        collisionBoxPath = defaultPrimPath + "/box2"
        add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(collisionBoxPath)), Sdf.Path(defaultPrimPath + "/highdensitymaterial"))

        collisionBoxPath = defaultPrimPath + "/box3"
        add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(collisionBoxPath)), Sdf.Path(defaultPrimPath + "/material"))
