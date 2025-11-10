# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class CollisionGroupsDemo(demo.Base):
    title = "Collision groups"
    category = demo.Categories.SIMULATION_PART
    short_description = "Demo showing collision groups filtering"
    description = "Demo showing collision groups filtering, the stacks (boxes/spheres/capsules/cones) should not collide with the other stacks. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        room = demo.get_demo_room(self, stage)

        # add collision groups
        conesGroup = defaultPrimPath + "/collisionGroupCones"
        cubesGroup = defaultPrimPath + "/collisionGroupBoxes"
        capsulesGroup = defaultPrimPath + "/collisionGroupCapsules"
        spheresGroup = defaultPrimPath + "/collisionGroupSpheres"
        collisionGroupCones = UsdPhysics.CollisionGroup.Define(stage, conesGroup)
        collisionGroupBoxes = UsdPhysics.CollisionGroup.Define(stage, cubesGroup)
        collisionGroupCapsules = UsdPhysics.CollisionGroup.Define(stage, capsulesGroup)
        collisionGroupSpheres = UsdPhysics.CollisionGroup.Define(stage, spheresGroup)

        # setup groups so that they dont collide with each other
        filteredRel = collisionGroupBoxes.CreateFilteredGroupsRel()
        filteredRel.AddTarget(conesGroup)
        filteredRel.AddTarget(capsulesGroup)
        filteredRel.AddTarget(spheresGroup)

        filteredRel = collisionGroupSpheres.CreateFilteredGroupsRel()
        filteredRel.AddTarget(conesGroup)
        filteredRel.AddTarget(capsulesGroup)
        filteredRel.AddTarget(cubesGroup)

        filteredRel = collisionGroupCones.CreateFilteredGroupsRel()
        filteredRel.AddTarget(cubesGroup)
        filteredRel.AddTarget(capsulesGroup)
        filteredRel.AddTarget(spheresGroup)

        filteredRel = collisionGroupCapsules.CreateFilteredGroupsRel()
        filteredRel.AddTarget(conesGroup)
        filteredRel.AddTarget(cubesGroup)
        filteredRel.AddTarget(spheresGroup)

        numActors = 20

        # boxes/groupBoxes
        boxActorsPath = Sdf.Path(defaultPrimPath + "/boxActors")
        UsdGeom.Scope.Define(stage, boxActorsPath)

        scale = 0.5

        size = 50.0 * scale
        density = 0.001
        color = demo.get_primary_color(0.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        for i in range(numActors):
            position = Gf.Vec3f(50.0, 50.0, 500.0 + i * 100.0) * scale
            cubePath = boxActorsPath.AppendChild("boxActor" + str(i))

            add_rigid_box(
                stage,
                cubePath,
                Gf.Vec3f(size, size, size),
                position,
                orientation,
                color,
                density,
                linVelocity,
                angularVelocity,
            )
            collisionAPI = UsdPhysics.CollisionAPI.Get(stage, cubePath)

        collectionAPI = Usd.CollectionAPI.Apply(collisionGroupBoxes.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(boxActorsPath)

        # spheres/groupSpheres
        sphereActorsPath = Sdf.Path(defaultPrimPath + "/sphereActors")
        UsdGeom.Scope.Define(stage, sphereActorsPath)

        radius = 25.0 * scale
        color = demo.get_primary_color(0.33)

        for i in range(numActors):
            position = Gf.Vec3f(-50.0, -50.0, 500.0 + i * 100.0) * scale
            spherePath = sphereActorsPath.AppendChild("sphereActor" + str(i))

            add_rigid_sphere(
                stage, spherePath, radius, position, orientation, color, density, linVelocity, angularVelocity
            )

            collisionAPI = UsdPhysics.CollisionAPI.Get(stage, spherePath)

        collectionAPI = Usd.CollectionAPI.Apply(collisionGroupSpheres.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(sphereActorsPath)

        # cones/groupCones
        coneActorsPath = Sdf.Path(defaultPrimPath + "/coneActors")
        UsdGeom.Scope.Define(stage, coneActorsPath)

        radius = 25.0 * scale
        height = 50.0 * scale
        color = demo.get_primary_color(0.66)

        for i in range(numActors):
            position = Gf.Vec3f(-50.0, 50.0, 500.0 + i * 100.0) * scale
            conePath = coneActorsPath.AppendChild("coneActor" + str(i))

            add_rigid_cone(
                stage,
                conePath,
                radius,
                height,
                "Z",
                position,
                orientation,
                color,
                density,
                linVelocity,
                angularVelocity,
            )

            collisionAPI = UsdPhysics.CollisionAPI.Get(stage, conePath)

        collectionAPI = Usd.CollectionAPI.Apply(collisionGroupCones.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(coneActorsPath)

        # capsules/groupCapsules
        capsuleActorsPath = Sdf.Path(defaultPrimPath + "/capsuleActors")
        UsdGeom.Scope.Define(stage, capsuleActorsPath)

        radius = 25.0 * scale
        height = 50.0 * scale
        color = demo.get_primary_color(1.0)

        for i in range(numActors):
            position = Gf.Vec3f(50.0, -50.0, 500.0 + i * 100.0) * scale
            capsulePath = capsuleActorsPath.AppendChild("capsuleActor" + str(i))

            add_rigid_capsule(
                stage,
                capsulePath,
                radius,
                height,
                "Z",
                position,
                orientation,
                color,
                density,
                linVelocity,
                angularVelocity,
            )

            collisionAPI = UsdPhysics.CollisionAPI.Get(stage, capsulePath)

        collectionAPI = Usd.CollectionAPI.Apply(collisionGroupCapsules.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(capsuleActorsPath)
