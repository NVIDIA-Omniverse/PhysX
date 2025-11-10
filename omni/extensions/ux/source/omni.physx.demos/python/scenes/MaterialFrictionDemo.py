# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class MaterialFrictionDemo(demo.Base):
    title = "Material friction"
    category = demo.Categories.MATERIALS
    short_description = "Demo showing different material friction behavior"
    description = "Demo showing different material friction behavior, each box has a different physics material assigned with different friction values. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        room = demo.get_demo_room(self, stage, zoom = 0.3, staticDynamicRestitution = Gf.Vec3f(0.0, 0.0, 1.0))

        boxCount = 3

        # Box materials
        for i in range(boxCount):
            path = defaultPrimPath + "/material" + str(i)

            mu = 0.1 * (i + 1)

            UsdShade.Material.Define(stage, path)
            material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
            material.CreateStaticFrictionAttr().Set(mu)
            material.CreateDynamicFrictionAttr().Set(mu)
            material.CreateRestitutionAttr().Set(0.0)
            material.CreateDensityAttr().Set(0.001)

        # Initial velocity
        v0 = 125.0

        # Boxes

        for i in range(boxCount):
            boxPath = defaultPrimPath + "/box" + str(i)

            size = Gf.Vec3f(12.5)
            position = Gf.Vec3f(-60.0, i * 31.25 - 25.0, 12.5)
            orientation = Gf.Quatf(1.0)
            color = demo.get_primary_color(i/(boxCount - 1.0))
            density = 0.001
            linvel = Gf.Vec3f(v0, 0.0, 0.0)

            add_rigid_box(stage, boxPath, size, position, orientation, color, density, linvel, Gf.Vec3f(0.0))

            # Add material
            materialPath = defaultPrimPath + "/material" + str(i)                       
            add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(boxPath)), Sdf.Path(materialPath))
