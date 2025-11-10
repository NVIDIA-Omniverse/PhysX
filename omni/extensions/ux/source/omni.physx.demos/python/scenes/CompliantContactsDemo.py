# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, UsdShade, PhysxSchema
import omni.physxdemos as demo


class CompliantContactsDemo(demo.Base):
    title = "Compliant Contacts"
    category = demo.Categories.CONTACTS
    short_description = "Demo showing how different values of stiffness and clamping for compliant contacts affects contact events"
    description = "Compliant contacts are enabled via material properties compliantStiffness and compliantDamping. The various stiffness values can be observed on the different rigid spheres."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, zoom = 0.7, hasTable = False)

        gravity = 981.0

        # Sphere
        sphereActorPath = defaultPrimPath + "/sphereActor"
        materialPath = defaultPrimPath + "/compliantMaterial"

        numSpheres = 5
        mass = 1.0  # kg
        deflection = 5.0  # cm
        stiffness = mass * gravity / deflection
        stiffnessDelta = stiffness
        damping = 0.1 * stiffness
        dampingDelta = 0.1 * damping
        radius = 25.0
        orientation = Gf.Quatf(1.0)
        positionDelta = Gf.Vec3f(-80.0, 0.0, 0.0)
        x0 = -positionDelta[0] * (numSpheres - 1) / 2.0
        position = Gf.Vec3f(x0, 0.0, 200.0)

        for i in range(numSpheres):
            path = sphereActorPath + f"_{i}"
            sphereGeom = UsdGeom.Sphere.Define(stage, path)
            spherePrim = stage.GetPrimAtPath(path)
            sphereGeom.CreateRadiusAttr(radius)
            sphereGeom.CreateExtentAttr([(-radius, -radius, -radius), (radius, radius, radius)])
            sphereGeom.AddTranslateOp().Set(position)
            position += positionDelta
            sphereGeom.AddOrientOp().Set(orientation)
            sphereGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            color = demo.get_primary_color(i/(numSpheres - 1.0))
            sphereGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(spherePrim)
            UsdPhysics.RigidBodyAPI.Apply(spherePrim)
            massAPI = UsdPhysics.MassAPI.Apply(spherePrim)
            massAPI.CreateMassAttr().Set(mass)

            matPath = materialPath + f"_{i}"
            UsdShade.Material.Define(stage, matPath)
            UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(matPath))
            material = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(matPath))
            material.CreateCompliantContactStiffnessAttr().Set(stiffness + i * stiffnessDelta)
            material.CreateCompliantContactDampingAttr().Set(damping + i * dampingDelta)

            physicsUtils.add_physics_material_to_prim(stage, spherePrim, matPath)
