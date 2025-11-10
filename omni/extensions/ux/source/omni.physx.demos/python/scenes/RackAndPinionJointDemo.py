# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import math


class RackAndPinionJointDemo(demo.Base):
    title = "Rack & pinion joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic Rack & pinion joint usage"
    description = "Demo showing basic Rack & pinion joint usage."

    def create_gear(self, stage, nbShapes, gearPos, scale, gearIndex):
        gearPath = self._defaultPrimPath + "/gear" + str(gearIndex)

        rigidXform = UsdGeom.Xform.Define(stage, gearPath)
        rigidPrim = stage.GetPrimAtPath(gearPath)

        rigidXform.AddTranslateOp().Set(gearPos)
        rigidXform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(rigidPrim)
        UsdPhysics.MassAPI.Apply(rigidPrim)

        shapePos = Gf.Vec3f(0.0, 0.0, 0.0)
        #shapeQuat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        shapeColor = demo.get_primary_color()

        for i in range(nbShapes):
            collisionShapePath = gearPath + "/physicsBoxShape" + str(i)

            coeff = i/nbShapes
            angle = math.pi*0.5*coeff
            w = math.cos((math.pi / 2.0 - angle) / 2.0)
            y = -math.sin((math.pi / 2.0 - angle) / 2.0)
            shapeQuat = Gf.Quatf(w, Gf.Vec3f(0.0, y, 0.0))

            cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath)
            cubePrim = stage.GetPrimAtPath(collisionShapePath)
            cubeGeom.CreateSizeAttr(1.0)
            cubeGeom.AddTranslateOp().Set(shapePos)
            cubeGeom.AddOrientOp().Set(shapeQuat)
            cubeGeom.AddScaleOp().Set(scale)
            cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

            UsdPhysics.CollisionAPI.Apply(cubePrim)

        # revolute joint
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, self._defaultPrimPath + "/revoluteJoint"+str(gearIndex))

        revoluteJoint.CreateAxisAttr("Y")

        revoluteJoint.CreateBody1Rel().SetTargets([gearPath])

        worldOrigin = Gf.Vec3f(self._room._originWorld)

        revoluteJoint.CreateLocalPos0Attr().Set(worldOrigin + gearPos)        
        revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    def create_rack(self, stage, nbShapes, rackPos, scale, rackIndex, rackLength):
        rackPath = self._defaultPrimPath + "/rack" + str(rackIndex)

        rigidXform = UsdGeom.Xform.Define(stage, rackPath)
        rigidPrim = stage.GetPrimAtPath(rackPath)

        rigidXform.AddTranslateOp().Set(rackPos)
        rigidXform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        UsdPhysics.RigidBodyAPI.Apply(rigidPrim)

        angle = math.pi * 0.25
        w = math.cos((math.pi / 2.0 - angle) / 2.0)
        y = -math.sin((math.pi / 2.0 - angle) / 2.0)
        shapeQuat = Gf.Quatf(w, Gf.Vec3f(0.0, y, 0.0))
        shapeColor = demo.get_primary_color()

        for i in range(nbShapes):
            collisionShapePath = rackPath + "/physicsBoxShape" + str(i)

            coeff = i / nbShapes

            shapePos = Gf.Vec3f(coeff * rackLength - rackLength * 0.5, 0.0, 0.0)

            cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath)
            cubePrim = stage.GetPrimAtPath(collisionShapePath)
            cubeGeom.CreateSizeAttr(1.0)
            cubeGeom.AddTranslateOp().Set(shapePos)
            cubeGeom.AddOrientOp().Set(shapeQuat)
            cubeGeom.AddScaleOp().Set(scale)
            cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

            UsdPhysics.CollisionAPI.Apply(cubePrim)

        # prismatic joint
        prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, self._defaultPrimPath + "/prismaticJoint" + str(rackIndex))

        prismaticJoint.CreateAxisAttr("X")

        prismaticJoint.CreateBody1Rel().SetTargets([rackPath])

        worldOrigin = Gf.Vec3f(self._room._originWorld)

        prismaticJoint.CreateLocalPos0Attr().Set(worldOrigin + rackPos)
        prismaticJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        prismaticJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        prismaticJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    def create(self, stage):
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        self._room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0)

        gearDepth = 10.0

        gear0Pos = Gf.Vec3f(0.0, 0.0, 100.0)
        radius0 = 80.0
        scale = Gf.Vec3f(radius0, gearDepth * 2.0, radius0)
        nbShapes0 = int(radius0 / 10.0)
        self.create_gear(stage, nbShapes0, gear0Pos, scale, 0)

        gear1Pos = Gf.Vec3f(0.0, 0.0, 174.0)
        radius1 = 30.0
        rackLength = 310.0
        scale = Gf.Vec3f(radius1, gearDepth, radius1)
        nbShapes1 = int(rackLength / 10.0)
        self.create_rack(stage, nbShapes1, gear1Pos, scale, 0, rackLength)

        # rack joint
        rackJoint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, self._defaultPrimPath + "/rackJoint")

        rackJoint.CreateBody0Rel().SetTargets([self._defaultPrimPath + "/gear0"])
        rackJoint.CreateBody1Rel().SetTargets([self._defaultPrimPath + "/rack0"])

        ratio = (360.0 * nbShapes1) / (rackLength * 2.0 * nbShapes0 * 2.0)
        rackJoint.CreateRatioAttr(ratio)
        rackJoint.CreateHingeRel().SetTargets([self._defaultPrimPath + "/revoluteJoint0"])
        rackJoint.CreatePrismaticRel().SetTargets([self._defaultPrimPath + "/prismaticJoint0"])

        # add angular drive
        linearDriveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(self._defaultPrimPath + "/prismaticJoint0"), "linear")
        linearDriveAPI.CreateTypeAttr("force")
        linearDriveAPI.CreateTargetPositionAttr(60.0)
        linearDriveAPI.CreateDampingAttr(0.0)
        linearDriveAPI.CreateStiffnessAttr(100.0)
