# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import math


class GearJointDemo(demo.Base):
    title = "Gear joint"
    category = demo.Categories.JOINTS
    short_description = "Demo showing basic gear joint usage"
    description = "Demo showing basic gear joint usage."
    tags = ["Joints"]

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

    def create(self, stage):
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        self._room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 60.0)

        gearDepth = 10.0

        gear0Pos = Gf.Vec3f(0.0, 0.0, 100.0)
        radius0 = 80.0
        scale = Gf.Vec3f(radius0, gearDepth*2.0, radius0)
        nbShapes = int(radius0/10.0)
        self.create_gear(stage, nbShapes, gear0Pos, scale, 0)

        gear1Pos = Gf.Vec3f(74.0, 0.0, 100.0)
        radius1 = 30.0
        scale = Gf.Vec3f(radius1, gearDepth, radius1)
        nbShapes = int(radius1/10.0)
        self.create_gear(stage, nbShapes, gear1Pos, scale, 1)

        # add angular drive
        angularDriveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(self._defaultPrimPath + "/revoluteJoint0"), "angular")
        angularDriveAPI.CreateTypeAttr("force")        
        angularDriveAPI.CreateTargetVelocityAttr(45.0)
        angularDriveAPI.CreateTargetPositionAttr(0.0)
        angularDriveAPI.CreateDampingAttr(1e10)
        angularDriveAPI.CreateStiffnessAttr(0.0)

        # gear joint
        gearJoint = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, self._defaultPrimPath + "/gearJoint")

        gearJoint.CreateBody0Rel().SetTargets([self._defaultPrimPath + "/gear0"])
        gearJoint.CreateBody1Rel().SetTargets([self._defaultPrimPath + "/gear1"])


        gearJoint.CreateGearRatioAttr(radius0/radius1)
        gearJoint.CreateHinge0Rel().SetTargets([self._defaultPrimPath + "/revoluteJoint0"])
        gearJoint.CreateHinge1Rel().SetTargets([self._defaultPrimPath + "/revoluteJoint1"])
