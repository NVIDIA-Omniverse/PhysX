# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo


class JointStateReadDemo(demo.Base):
    title = "Joint State"
    category = demo.Categories.ARTICULATIONS
    short_description = "Demo showing basic revolute/prismatic/D6 joint state API usage"
    description = "Demo showing basic revolute/prismatic/D6 joint state API usage."
    params = {
        "D6Joint": demo.CheckboxParam(False),
        "Prismatic": demo.CheckboxParam(False),
    }

    def create(self, stage, D6Joint, Prismatic):
        if D6Joint:
            self.create_d6joint(stage,True,True,True,False,True,True)
        else:
            self.create_revolute_prismatic(stage, Prismatic)
    
    def get_default_room(self, stage):
        return demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False, camElevation = 130.0, floorOffset = 870.0)

    def create_revolute_prismatic(self, stage, Prismatic):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = self.get_default_room(stage)


        articulationPath = defaultPrimPath + "/articulation"
        UsdGeom.Xform.Define(stage, articulationPath)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))


        # box0 static
        boxActorPath = articulationPath + "/box0"

        position = Gf.Vec3f(0.0, 0.0, 1000.0)
        orientation = Gf.Quatf(1.0)
        color = demo.get_static_color()
        size = 100.0
        scale = Gf.Vec3f(0.1, 1.0, 0.1)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)            

        # Box1
        boxActorPath = articulationPath + "/box1"

        size = 100.0
        position = Gf.Vec3f(0.0, 120.0, 1000.0)
        color = demo.get_primary_color()
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        scale = Gf.Vec3f(0.1, 1.0, 0.1)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

        # fixed root joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, articulationPath + "/rootJoint")
        fixedJoint.CreateBody1Rel().SetTargets( [Sdf.Path(articulationPath + "/box0")])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))


        # revolute joint
        if not Prismatic:
            revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, articulationPath + "/revoluteJoint")

            revoluteJoint.CreateAxisAttr("X")
            revoluteJoint.CreateLowerLimitAttr(-90.0)
            revoluteJoint.CreateUpperLimitAttr(90)

            revoluteJoint.CreateBody0Rel().SetTargets([articulationPath + "/box0"])
            revoluteJoint.CreateBody1Rel().SetTargets([articulationPath + "/box1"])

            revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
            revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

            revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
            revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

            jointStateAPI = PhysxSchema.JointStateAPI.Apply(revoluteJoint.GetPrim(), "angular")
            jointStateAPI.CreatePositionAttr().Set(45.0)
            jointStateAPI.CreateVelocityAttr().Set(0.0)
        else:
            prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, articulationPath + "/prismaticJoint")

            prismaticJoint.CreateAxisAttr("Y")
            prismaticJoint.CreateLowerLimitAttr(-50)
            prismaticJoint.CreateUpperLimitAttr(50)

            prismaticJoint.CreateBody0Rel().SetTargets([articulationPath + "/box0"])
            prismaticJoint.CreateBody1Rel().SetTargets([articulationPath + "/box1"])

            prismaticJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
            prismaticJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

            prismaticJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
            prismaticJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

            jointStateAPI = PhysxSchema.JointStateAPI.Apply(prismaticJoint.GetPrim(), "linear")
            jointStateAPI.CreatePositionAttr().Set(45.0)
            jointStateAPI.CreateVelocityAttr().Set(0.0)
    
    def create_d6joint(self, stage, Trans_X, Trans_Y, Trans_Z, Rot_X, Rot_Y, Rot_Z):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = self.get_default_room(stage)

        articulationPath = defaultPrimPath + "/articulation"
        UsdGeom.Xform.Define(stage, articulationPath)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))

        # box0 static
        boxActorPath = articulationPath + "/box0"

        position = Gf.Vec3f(0.0, 0.0, 1000.0)
        orientation = Gf.Quatf(1.0)
        color = demo.get_static_color()
        size = 100.0
        scale = Gf.Vec3f(0.1, 1.0, 0.1)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)    

        # Box1
        boxActorPath = articulationPath + "/box1"

        size = 100.0
        position = Gf.Vec3f(0.0, 120.0, 1000.0)
        color = demo.get_primary_color()
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        scale = Gf.Vec3f(0.1, 1.0, 0.1)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)


        # fixed root joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, articulationPath + "/rootJoint")
        fixedJoint.CreateBody1Rel().SetTargets( [Sdf.Path(articulationPath + "/box0")])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        
        # D6 fixed joint
        d6Joint = UsdPhysics.Joint.Define(stage, articulationPath + "/d6Joint")

        d6Joint.CreateBody0Rel().SetTargets([articulationPath + "/box0"])
        d6Joint.CreateBody1Rel().SetTargets([articulationPath + "/box1"])

        d6Joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        d6Joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        d6Joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        d6Joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # lock all DOF (lock - low is greater than high)
        d6Prim = d6Joint.GetPrim()
        if Trans_X:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transX)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Trans_Y:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transY)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Trans_Z:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.transZ)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        if Rot_X:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotX)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        else:
            jointStateAPI = PhysxSchema.JointStateAPI.Apply(d6Prim, UsdPhysics.Tokens.rotX)
        if Rot_Y:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotY)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        else:
            jointStateAPI = PhysxSchema.JointStateAPI.Apply(d6Prim, UsdPhysics.Tokens.rotY)
        if Rot_Z:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, UsdPhysics.Tokens.rotZ)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
        else:
            jointStateAPI = PhysxSchema.JointStateAPI.Apply(d6Prim, UsdPhysics.Tokens.rotZ)
        
        jointStateAPI.CreatePositionAttr().Set(45.0)
        jointStateAPI.CreateVelocityAttr().Set(0.0)
