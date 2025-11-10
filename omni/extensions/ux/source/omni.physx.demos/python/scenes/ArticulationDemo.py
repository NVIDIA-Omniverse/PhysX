# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class ArticulationDemo(demo.Base):
    title = "Articulation"
    category = demo.Categories.ARTICULATIONS
    short_description = "Articulation demo showing articulated chain of rigid bodies"
    description = "Articulation demo showing articulated chain of rigid bodies. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, metersPerUnit = 1.0)

        room = demo.get_demo_room(self, stage, hasTable = False, camElevation = 130.0, floorOffset = 2200.0, overrideSphericalLightIntensity = 2.0e5)

        scale = 0.125
        radius = 1.0 * scale
        halfHeight = 1.0 * scale
        nbBoxes = 15

        initPos = Gf.Vec3f(-2.5, 0.0, 24.0)
        pos = Gf.Vec3f(0.0, 0.0, 0.0)

        articulationPath = defaultPrimPath + "/articulation"
        UsdGeom.Xform.Define(stage, articulationPath)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))

        parentName = articulationPath

        # Create rope
        for i in range(nbBoxes):
            # link
            linkName = defaultPrimPath + "/articulation/articulationLink" + str(i)

            if i != 0:
                aticulatedJointName = defaultPrimPath + "/articulation/articulatedRevoluteJoint" + str(i)

                component = UsdPhysics.RevoluteJoint.Define(stage, aticulatedJointName)
                val0 = [Sdf.Path(parentName)]
                val1 = [Sdf.Path(linkName)]

                if parentName != "":
                    component.CreateBody0Rel().SetTargets(val0)
                component.CreateLocalPos0Attr().Set(Gf.Vec3f(radius + halfHeight, 0.0, 0.0))
                component.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

                component.CreateBody1Rel().SetTargets(val1)
                component.CreateLocalPos1Attr().Set(Gf.Vec3f(-(radius + halfHeight), 0.0, 0.0))
                component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

                component.CreateAxisAttr("Y")
                component.CreateLowerLimitAttr(float(-180 / 32.0))
                component.CreateUpperLimitAttr(float(180 / 32.0))
            else:
                aticulatedJointName = defaultPrimPath + "/articulation/rootJoint"
                component = UsdPhysics.FixedJoint.Define(stage, aticulatedJointName)

                val1 = [Sdf.Path(linkName)]
                component.CreateLocalPos0Attr().Set(initPos)
                component.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

                component.CreateBody1Rel().SetTargets(val1)
                component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
                
            capsuleGeom = UsdGeom.Capsule.Define(stage, linkName)
            capsulePrim = stage.GetPrimAtPath(linkName)
            capsuleGeom.CreateHeightAttr(halfHeight)
            capsuleGeom.CreateRadiusAttr(radius)
            capsuleGeom.CreateAxisAttr("X")
            capsuleGeom.AddTranslateOp().Set(initPos + pos)
            capsuleGeom.AddOrientOp().Set(Gf.Quatf(1.0))
            capsuleGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
            capsuleGeom.CreateDisplayColorAttr().Set([demo.get_primary_color()])

            UsdPhysics.CollisionAPI.Apply(capsulePrim)
            UsdPhysics.RigidBodyAPI.Apply(capsulePrim)            

            parentName = linkName
            pos[0] += (radius + halfHeight) * 2.0

        # create colliding box
        boxActorPath = defaultPrimPath + "/boxActor"

        size = 2.0
        position = Gf.Vec3f(Gf.Vec3f(0.0, 0.0, 24.0) + Gf.Vec3f(20.0*scale, 0.0, -5.0*scale))
        orientation = Gf.Quatf(1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0*scale, 2.0*scale, 0.1))
        cubeGeom.CreateDisplayColorAttr().Set([demo.get_static_color()])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
