# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class MaterialRestitutionDemo(demo.Base):
    title = "Material restitution"
    category = demo.Categories.MATERIALS
    short_description = "Demo showing different material restitution behavior."
    description = "Demo showing different material restitution behavior, each box has a different physics material assigned with different restitution values. Press play (space) to run the simulation."

    params = {        
        "default_restitution": demo.FloatParam(0.5, 0.0, 1.0, 0.05),
    }

    def create(self, stage, default_restitution):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, zoom = 0.5)

        spheresCount = 5

        # Default Material
        path = defaultPrimPath + "/defaultMaterial"
        UsdShade.Material.Define(stage, path)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(default_restitution)

        # Add material        
        add_physics_material_to_prim(stage, scene.GetPrim(), Sdf.Path(path))

        for i in range(spheresCount):
            path = defaultPrimPath + "/material" + str(i)

            mu = 0.2 + i * 0.2

            UsdShade.Material.Define(stage, path)
            material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
            material.CreateStaticFrictionAttr().Set(0.5)
            material.CreateDynamicFrictionAttr().Set(0.5)
            material.CreateRestitutionAttr().Set(mu)
            material.CreateDensityAttr().Set(0.001)

        # Spheres
        for i in range(spheresCount):
            spherePath = defaultPrimPath + "/sphere" + str(i)

            radius = 15.0
            position = Gf.Vec3f(i * 40.0 - 80.0, 0.0, 250.0)
            orientation = Gf.Quatf(1.0)
            color = demo.get_primary_color(i/(spheresCount - 1.0))
            density = 0.001

            spherePrim = add_rigid_sphere(stage, spherePath, radius, position, orientation, color, density)

            # Add material
            materialPath = defaultPrimPath + "/material" + str(i)            
            add_physics_material_to_prim(stage, spherePrim, Sdf.Path(materialPath))
