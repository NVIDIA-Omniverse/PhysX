# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils, utils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, UsdShade, PhysxSchema
import omni.physxdemos as demo
from carb.settings import get_settings
from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS


class ContactSlopCoefficientDemo(demo.Base):
    title = "Contact Slop Coefficient"
    category = demo.Categories.CONTACTS
    short_description = "Demo illustrating the Contact Slop Coefficient"
    description = "The PhysX rigid-body Contact Slop Coefficient attribute may help improve the rolling behavior of approximate-collision-shape wheels."

    # Use convex hull approximation.
    kit_settings = {
        SETTING_COLLISION_APPROXIMATE_CYLINDERS : True
    }

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)

        # Use convex hull approximation.
        get_settings().set_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS, True)

        # Cylinders
        slopCylinder = defaultPrimPath.AppendChild("slopWheel")
        regularWheel = defaultPrimPath.AppendChild("regularWheel")
        paths = [slopCylinder, regularWheel]

        radius = 40.0
        height = 160.0
        angularVelocity = Gf.Vec3f(0, 1500, 0)
        # somewhat ground speed match:
        xVel = angularVelocity[1] * radius * 3.1416 / 180.0
        linearVelocity = Gf.Vec3f(xVel, 0, 0)

        colors = [demo.get_primary_color(0.0), demo.get_primary_color(1.0)]
        xPos = -250.0
        zPos = 42.0
        positions = [Gf.Vec3f(xPos, 160, zPos), Gf.Vec3f(xPos, -160, zPos)]

        # material:
        materialPath = defaultPrimPath.AppendChild("physicsMaterial")
        utils.addRigidBodyMaterial(stage, materialPath, density=None, staticFriction=0.2, dynamicFriction=0.2, restitution=0.0)

        for i in range(len(paths)):
            path = paths[i]
            geom = UsdGeom.Cylinder.Define(stage, path)
            prim = stage.GetPrimAtPath(path)
            geom.CreateRadiusAttr().Set(radius)
            geom.CreateHeightAttr().Set(height)
            geom.CreateAxisAttr().Set("Y")
            geom.CreateExtentAttr().Set([(-radius, -height * 0.5, -radius), (radius, height * 0.5, radius)])
            geom.AddTranslateOp().Set(positions[i])
            geom.AddOrientOp().Set(Gf.Quatf(1.0))
            geom.AddScaleOp().Set(Gf.Vec3f(1.0))
            geom.CreateDisplayColorAttr().Set([colors[i]])

            UsdPhysics.CollisionAPI.Apply(prim)
            rbAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
            rbAPI.CreateAngularVelocityAttr().Set(angularVelocity)
            rbAPI.CreateVelocityAttr().Set(linearVelocity)
            physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physxRigidBodyAPI.CreateAngularDampingAttr(0.0)
            if path.name == "slopWheel":
                physxRigidBodyAPI.CreateContactSlopCoefficientAttr().Set(5.0)

            physicsUtils.add_physics_material_to_prim(stage, prim, materialPath)

        room = demo.get_demo_room(self, stage, camPitch = 0.47, camYaw = 0.625, zoom = 0.75, onlyUsePlane = True)
