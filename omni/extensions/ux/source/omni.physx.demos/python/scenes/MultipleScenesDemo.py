# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysicsSchemaTools
import omni.physxdemos as demo


class MultipleScenesDemo(demo.Base):
    title = "Multiple Scenes"
    category = demo.Categories.SIMULATION_PART
    short_description = "Demo showing multiple scenes usage"
    description = "Demo showing multiple scenes, the stacks (boxes/spheres/capsules/cones) should not collide with the other stacks, since they belong each to a different scene. Each scene has a different gravity. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, baseScene = demo.setup_physics_scene(self, stage)

        gravityValues = [981.0, 681.0, 31.0, 81.0]
        scenePaths = []
        self.scenePathsInt = []
        for i in range(len(gravityValues)):
            scenePath = defaultPrimPath + "/physicsScene" + str(i)
            scenePaths.append(scenePath)
            self.scenePathsInt.append(PhysicsSchemaTools.sdfPathToInt(scenePath))
            scene = UsdPhysics.Scene.Define(stage, scenePath)
            scene.CreateGravityMagnitudeAttr().Set(gravityValues[i])

        room = demo.get_demo_room(self, stage, zoom = 1.2)

        numActors = 20
        heightOffset = 500.0

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
            position = Gf.Vec3f(50.0, 50.0, heightOffset + i * 100.0) * scale
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
            rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, cubePath)
            target = scenePaths[0]
            collisionAPI.GetSimulationOwnerRel().AddTarget(target)
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(target)

        # spheres/groupSpheres
        sphereActorsPath = Sdf.Path(defaultPrimPath + "/sphereActors")
        UsdGeom.Scope.Define(stage, sphereActorsPath)

        radius = 25.0 * scale
        color = demo.get_primary_color(0.33)

        for i in range(numActors):
            position = Gf.Vec3f(-50.0, -50.0, heightOffset + i * 100.0) * scale
            spherePath = sphereActorsPath.AppendChild("sphereActor" + str(i))

            add_rigid_sphere(
                stage, spherePath, radius, position, orientation, color, density, linVelocity, angularVelocity
            )

            collisionAPI = UsdPhysics.CollisionAPI.Get(stage, spherePath)
            rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, spherePath)
            target = scenePaths[1]
            collisionAPI.GetSimulationOwnerRel().AddTarget(target)
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(target)

        # cones/groupCones
        coneActorsPath = Sdf.Path(defaultPrimPath + "/coneActors")
        UsdGeom.Scope.Define(stage, coneActorsPath)

        radius = 25.0 * scale
        height = 50.0 * scale
        color = demo.get_primary_color(0.66)

        for i in range(numActors):
            position = Gf.Vec3f(-50.0, 50.0, heightOffset + i * 100.0) * scale
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
            rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, conePath)
            target = scenePaths[2]
            collisionAPI.GetSimulationOwnerRel().AddTarget(target)
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(target)            

        # capsules/groupCapsules
        capsuleActorsPath = Sdf.Path(defaultPrimPath + "/capsuleActors")
        UsdGeom.Scope.Define(stage, capsuleActorsPath)

        radius = 25.0 * scale
        height = 50.0 * scale
        color = demo.get_primary_color(1.0)

        for i in range(numActors):
            position = Gf.Vec3f(50.0, -50.0, heightOffset + i * 100.0) * scale
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
            rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, capsulePath)
            target = scenePaths[3]
            collisionAPI.GetSimulationOwnerRel().AddTarget(target)
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(target)

        # sphere
        position = Gf.Vec3f(0.0, 0.0, 50.0) * scale
        spherePath = defaultPrimPath + "/sphereKinematicActor"
        color = demo.get_static_color()
        add_rigid_sphere(
            stage, spherePath, radius, position, orientation, color, density, linVelocity, angularVelocity
        )
        UsdPhysics.CollisionAPI.Get(stage, spherePath)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, spherePath)
        rigidBodyAPI.CreateKinematicEnabledAttr().Set(True)
        
        # ensure that each scene interacts with the common geometry (sphere defined above and all room geometry) 
        for target in scenePaths:
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(target)
            for colliderAPI in room._colliderAPIs:
                colliderAPI.GetSimulationOwnerRel().AddTarget(target)
