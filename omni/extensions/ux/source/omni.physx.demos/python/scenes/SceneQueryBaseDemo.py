# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import random
from omni.physx.scripts.physicsUtils import add_rigid_box, add_ground_plane, add_rigid_sphere, add_rigid_capsule, add_rigid_cylinder
from pxr import Gf, PhysxSchema
import omni.physxdemos as demo


def create_pyramid(stage, pyramidSize, y_position, defaultPrimPath):
    box_size = 50.0
    size = Gf.Vec3f(box_size)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    color = demo.get_primary_color()
    linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    density = 0.001
    offset = 1

    for i in range(pyramidSize):
        for j in range(pyramidSize - i):
            # Box
            boxActorPath = defaultPrimPath + "/boxActor" + str(i) + str(j)
            position = Gf.Vec3f((-box_size * pyramidSize / 2) + i * box_size / 2 + j * (box_size + offset), y_position, (box_size + offset) * (i + 0.5))
            add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)


def createSceneQueryBase(stage, defaultPrimPath, sceneType="ThreeBoxes", numStacks=3, stackHight=15):

    if sceneType == "ThreeBoxes":

        # Box0
        boxActorPath = defaultPrimPath + "/boxActor0"

        size = Gf.Vec3f(50.0)
        position = Gf.Vec3f(-200.0, 0.0, 0.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = demo.get_primary_color()
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        density = 0.001

        add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        # lock pos change
        usdPrim = stage.GetPrimAtPath(boxActorPath)
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(usdPrim)
        physxAPI.CreateLockedPosAxisAttr(7)

        # Box1
        boxActorPath = defaultPrimPath + "/boxActor1"
        position = Gf.Vec3f(0.0, 0.0, 50.0)

        add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        # lock pos change
        usdPrim = stage.GetPrimAtPath(boxActorPath)
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(usdPrim)
        physxAPI.CreateLockedPosAxisAttr(7)

        # Box2
        boxActorPath = defaultPrimPath + "/boxActor2"
        position = Gf.Vec3f(200.0, 0.0, 0.0)

        add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

        # lock pos change
        usdPrim = stage.GetPrimAtPath(boxActorPath)
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(usdPrim)
        physxAPI.CreateLockedPosAxisAttr(7)
    elif sceneType == "BoxPyramid":

        # Plane
        add_ground_plane(stage, defaultPrimPath + "/groundPlane", "Z", 1500.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        if stackHight < 5:
            stackHight = 5

        start_index = numStacks / 2
        for i in range(numStacks):
            create_pyramid(stage, stackHight, 300 * (i - start_index), defaultPrimPath)


def createSQTestScene(stage, defaultPrimPath, killRadius, spread, heightScale):

    radius = 0.5
    size = Gf.Vec3f(radius * 2.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    color = demo.get_primary_color()
    linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    density = 0.0

    nb_x = 32
    nb_y = 32
    halfScale = spread * 0.5
    offset_x = -(nb_x - 1) * halfScale
    offset_y = -(nb_y - 1) * halfScale
    count = 0
    random.seed(42)
    for y in range(0, nb_y):
        for x in range(0, nb_x):
            actorPath = defaultPrimPath + "/sqActors/actor" + str(count)
            count = count + 1

            rnd_x = random.uniform(-1.0, 1.0)
            rnd_y = random.uniform(-1.0, 1.0)
            rnd_z = random.uniform(-1.0, 1.0)
            rnd_w = random.uniform(-1.0, 1.0)
            orientation = Gf.Quatf(rnd_x, rnd_y, rnd_z, rnd_w)
            orientation.Normalize()

            rnd_x = random.uniform(-halfScale * 0.5, halfScale * 0.5)
            rnd_y = random.uniform(-halfScale * 0.5, halfScale * 0.5)
            rnd_z = random.uniform(0.0, heightScale)
            position = Gf.Vec3f(offset_x + rnd_x + x * spread, offset_y + rnd_y + y * spread, rnd_z)

            distToOrigin = math.sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2])
            if distToOrigin > killRadius:
                randomType = random.uniform(0.0, 1.0)

                if randomType > 0.75:
                    add_rigid_box(stage, actorPath, size, position, orientation, color, density, linVelocity, angularVelocity)
                else:
                    if randomType > 0.50:
                        add_rigid_sphere(stage, actorPath, radius, position, orientation, color, density, linVelocity, angularVelocity)
                    else:
                        if randomType > 0.25:
                            add_rigid_capsule(stage, actorPath, radius, radius * 2.0, "Z", position, orientation, color, density, linVelocity, angularVelocity)
                        else:
                            add_rigid_cylinder(stage, actorPath, radius, radius * 2.0, "Z", position, orientation, color, density, linVelocity, angularVelocity)
