# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import Sdf, UsdLux, UsdGeom, Gf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.timeline

class SurfaceVelocityDemo(demo.Base):
    title = "Surface Velocity"
    category = demo.Categories.CONTACTS
    short_description = "Demo showing surface velocity feature"
    description = "Demo showing surface velocity feature, surface velocity API enables contact modification and injects the desired linear/angular surface velocity during the contact modification callback. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, zoom = 0.5, hasTable = False, onlyUsePlane = True)

        omni.timeline.get_timeline_interface().set_end_time(1300/60)

        # Kinematic Circular Conveyor
        cylinderActorPath = defaultPrimPath + "/kinematicCircularConveyor"
        cylinderGeom = UsdGeom.Cylinder.Define(stage, cylinderActorPath)
        radius = 150.0
        height = 5.0
        cylinderGeom.CreateHeightAttr(height)
        cylinderGeom.CreateRadiusAttr(radius)
        cylinderGeom.GetAxisAttr().Set("Z")
        cylinderGeom.CreateExtentAttr().Set([(-radius, -radius, -height * 0.5), (radius, radius, height * 0.5)])
        cylinderPrim = stage.GetPrimAtPath(cylinderActorPath)
        cylinderGeom.CreateDisplayColorAttr().Set([demo.get_primary_color(0.5)])

        UsdPhysics.CollisionAPI.Apply(cylinderPrim)
        angularConveyor = UsdPhysics.RigidBodyAPI.Apply(cylinderPrim)
        angularConveyor.CreateKinematicEnabledAttr().Set(True)

        # Apply SurfaceVelocityAPI
        surfaceVelocityAPI = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(cylinderPrim)
        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 0.0, 5.0)
        surfaceVelocityAPI.GetSurfaceAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

        # Create "tank"
        tankActorPath = defaultPrimPath + "/tank"
        tankXform = UsdGeom.Xform.Define(stage, tankActorPath)

        # wheels
        numWheels = 5
        wheelRadius = 20.0
        wheelHeight = 8
        wheelOffset = 5        
        for j in range(2):
            axisPath = tankActorPath + "/axis" + str(j)
            axisXform = UsdGeom.Xform.Define(stage, axisPath)            
            if j == 0:
                axisXform.AddTranslateOp().Set(Gf.Vec3f(0.0, -50.0, 20.0))
            else:
                axisXform.AddTranslateOp().Set(Gf.Vec3f(0.0, 50.0, 20.0))
            UsdPhysics.RigidBodyAPI.Apply(axisXform.GetPrim())
            physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(axisXform.GetPrim())
            physxRigidBodyAPI.CreateSleepThresholdAttr().Set(0)
            axisSurfaceVelocityAPI = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(axisXform.GetPrim())            
            for i in range(numWheels):            
                wheelPath = axisPath + "/wheel" + str(i)
                wheelGeom = UsdGeom.Cylinder.Define(stage, wheelPath)
                wheelGeom.CreateHeightAttr(wheelHeight)
                wheelGeom.CreateRadiusAttr(wheelRadius)
                wheelGeom.GetAxisAttr().Set("Y")
                wheelGeom.CreateExtentAttr().Set([(-wheelRadius, -wheelHeight * 0.5, -wheelRadius), (wheelRadius, wheelHeight * 0.5, wheelRadius)])
                wheelPos = (i - int(numWheels / 2)) * (wheelRadius * 2.0 + wheelOffset)
                wheelGeom.AddTranslateOp().Set(Gf.Vec3f(wheelPos, 0.0, 0.0))
                UsdPhysics.CollisionAPI.Apply(wheelGeom.GetPrim())

            # time sampled values for the velocity
            velocityAttribute = axisSurfaceVelocityAPI.GetSurfaceVelocityAttr()            
            velocityAttribute.Set(time=0, value=Gf.Vec3f(0.0))
            velocityAttribute.Set(time=200, value=Gf.Vec3f(0.0))
            velocityAttribute.Set(time=300, value=Gf.Vec3f(50.0, 0.0, 0.0))
            velocityAttribute.Set(time=600, value=Gf.Vec3f(50.0, 0.0, 0.0)) 
            if j == 0:           
                velocityAttribute.Set(time=650, value=Gf.Vec3f(-50.0, 0.0, 0.0))            
                velocityAttribute.Set(time=800, value=Gf.Vec3f(-50.0, 0.0, 0.0))            
            else:
                velocityAttribute.Set(time=650, value=Gf.Vec3f(50.0, 0.0, 0.0))            
                velocityAttribute.Set(time=800, value=Gf.Vec3f(50.0, 0.0, 0.0))            
            velocityAttribute.Set(time=850, value=Gf.Vec3f(50.0, 0.0, 0.0))            
            velocityAttribute.Set(time=1150, value=Gf.Vec3f(50.0, 0.0, 0.0))            
            velocityAttribute.Set(time=1250, value=Gf.Vec3f(0.0))            
        
        # chassis
        chassisPath = tankActorPath + "/chassis"
        chassisGeom = UsdGeom.Cube.Define(stage, chassisPath)
        chassisGeom.CreateSizeAttr(100)
        chassisGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 35.0))
        chassisGeom.AddScaleOp().Set(Gf.Vec3f(2.0, 0.8, 0.2))
        UsdPhysics.RigidBodyAPI.Apply(chassisGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(chassisGeom.GetPrim())
        physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(chassisGeom.GetPrim())
        physxRigidBodyAPI.CreateSleepThresholdAttr().Set(0)

        xfCache = UsdGeom.XformCache()

        # Joints to connect the parts
        for j in range(2):
            jointPath = tankActorPath + "/axis" + str(j) + "/prismaticJoint"
            prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, jointPath)

            prismaticJoint.CreateAxisAttr("Z")
            prismaticJoint.CreateLowerLimitAttr(-5)
            prismaticJoint.CreateUpperLimitAttr(5)

            prismaticJoint.CreateBody0Rel().SetTargets([chassisPath])
            prismaticJoint.CreateBody1Rel().SetTargets([tankActorPath + "/axis" + str(j)])

            fromPose = xfCache.GetLocalToWorldTransform(chassisGeom.GetPrim())
            axisPrim = stage.GetPrimAtPath(tankActorPath + "/axis" + str(j))
            toPose = xfCache.GetLocalToWorldTransform(axisPrim)
            relPose = toPose * fromPose.GetInverse()
            relPose = relPose.RemoveScaleShear()
            pos0 = Gf.Vec3f(relPose.ExtractTranslation())
            rot0 = Gf.Quatf(relPose.ExtractRotationQuat())

            prismaticJoint.CreateLocalPos0Attr().Set(pos0)
            prismaticJoint.CreateLocalRot0Attr().Set(rot0)

            prismaticJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            prismaticJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

            # add linear drive
            linearDriveAPI = UsdPhysics.DriveAPI.Apply(prismaticJoint.GetPrim(), "linear")
            linearDriveAPI.CreateTypeAttr("force")
            linearDriveAPI.CreateTargetPositionAttr(0.0)
            linearDriveAPI.CreateDampingAttr(0.0)
            linearDriveAPI.CreateStiffnessAttr(100.0)
