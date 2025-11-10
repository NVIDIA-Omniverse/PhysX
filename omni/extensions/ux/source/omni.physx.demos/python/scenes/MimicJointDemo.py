# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
import math
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo


def _create_body(
    stage: Usd.Stage,
    path: str,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
    mass: float = 1.0,
    ) -> Usd.Prim:

    bodyXform = UsdGeom.Xform.Define(stage, path)
    bodyXform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(position)
    bodyXform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(orientation)

    bodyPrim = bodyXform.GetPrim()

    bodyAPI = UsdPhysics.RigidBodyAPI.Apply(bodyPrim)

    massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
    massAPI.GetMassAttr().Set(mass)
    massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(mass))

    return bodyPrim


class MimicJointDemo(demo.Base):
    title = "Mimic Joint"
    category = demo.Categories.ARTICULATIONS
    short_description = "Mimic joints on a rigid body articulation tree"
    description = ("A set of mimic joints connecting the positions/angles of joints of a rigid body articulation "
        "tree using different gearing and offset values. Press play (space) to run the simulation.")

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, metersPerUnit = 1.0, upAxis = UsdGeom.Tokens.y)

        cameraPath = defaultPrimPath + "/Camera"
        camera = UsdGeom.Camera.Define(stage, cameraPath)
        camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.2, 2))
        camera.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        self.demo_camera = cameraPath

        light = UsdLux.DistantLight.Define(stage, defaultPrimPath + "/Light")
        light.CreateIntensityAttr(3000)
        light.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(315.0, 0.0, 0.0))

        pinionRadius = 0.05
        linkMass = 1.0

        rootLinkPath = defaultPrimPath + "/RootLink"
        rootLinkPrim = _create_body(stage, rootLinkPath, Gf.Vec3f(0.0), mass = linkMass)

        UsdPhysics.ArticulationRootAPI.Apply(rootLinkPrim)

        fixedJoint = UsdPhysics.FixedJoint.Define(stage, rootLinkPath + "/InboundJoint")
        fixedJoint.GetBody1Rel().AddTarget(rootLinkPath)

        link0Path = defaultPrimPath + "/Link0"
        link0AngleOffset = 45.0
        link0Pos = Gf.Vec3f(0.0, pinionRadius, 0.0)
        offsetHalfRadians = 0.5 * (link0AngleOffset * math.pi) / 180.0
        link0Orient = Gf.Quatf(math.cos(offsetHalfRadians), 0.0, 0.0, math.sin(offsetHalfRadians))
        link0Prim = _create_body(stage, link0Path, link0Pos, link0Orient, mass = linkMass)

        link0BoxHalfExtent = pinionRadius / math.sqrt(2.0)
        link0Box = UsdGeom.Cube.Define(stage, link0Path + "/Geom")
        link0Box.CreateSizeAttr(2.0 * link0BoxHalfExtent)
        link0Box.CreateDisplayColorAttr([demo.get_primary_color()])

        joint0Path = link0Path + "/InboundJoint"
        joint0 = UsdPhysics.RevoluteJoint.Define(stage, joint0Path)
        joint0.GetAxisAttr().Set(UsdGeom.Tokens.z)
        joint0.GetLowerLimitAttr().Set(-180)
        joint0.GetUpperLimitAttr().Set(180)
        joint0.GetBody0Rel().AddTarget(rootLinkPath)
        joint0.GetBody1Rel().AddTarget(link0Path)
        joint0.GetLocalPos0Attr().Set(link0Pos)

        driveTarget = -10.0
        joint0DriveAPI = UsdPhysics.DriveAPI.Apply(joint0.GetPrim(), UsdPhysics.Tokens.angular)
        joint0DriveAPI.GetTargetVelocityAttr().Set(driveTarget)
        joint0DriveAPI.GetDampingAttr().Set(1.0e4)
        joint0DriveAPI.GetStiffnessAttr().Set(0.0)

        join0State = PhysxSchema.JointStateAPI.Apply(joint0.GetPrim(), UsdPhysics.Tokens.angular)
        join0State.GetPositionAttr().Set(link0AngleOffset)

        link1BoxHalfExtent = Gf.Vec3f(0.2, 0.5 * pinionRadius, link0BoxHalfExtent)

        link1Path = defaultPrimPath + "/Link1"
        link1Pos = Gf.Vec3f(0.0, link0Pos[1] + pinionRadius + link1BoxHalfExtent[1], 0.0)
        link1Prim = _create_body(stage, link1Path, link1Pos, mass = linkMass)

        link1Box = UsdGeom.Cube.Define(stage, link1Path + "/Geom")
        link1Box.CreateSizeAttr(1.0)
        link1Box.AddScaleOp().Set(2.0 * link1BoxHalfExtent)
        link1Box.CreateDisplayColorAttr([demo.get_primary_color()])

        joint1Path = link1Path + "/InboundJoint"
        joint1 = UsdPhysics.PrismaticJoint.Define(stage, joint1Path)
        joint1.GetAxisAttr().Set(UsdGeom.Tokens.x)
        joint1.GetLowerLimitAttr().Set(-1.0e10)
        joint1.GetUpperLimitAttr().Set(1.0e10)
        joint1.GetBody0Rel().AddTarget(rootLinkPath)
        joint1.GetBody1Rel().AddTarget(link1Path)
        joint1.GetLocalPos0Attr().Set(link1Pos)

        # mimic joint that should transfer the angular movement of joint0 to linear movement
        # on joint1 (like in a rack and pinion setup). Note that neither the instance name nor
        # the referenceJointAxis attribute matter in this sample as all the used joint types
        # have a single degree of freedom only.

        mimic0Gearing = (pinionRadius * math.pi) / 180.0

        # the initial angle of joint0 should be ignored => transform the initial angle to the
        # corresponding distance for joint1 and use it as offset
        mimic0Offset = -((link0AngleOffset * math.pi / 180.0) * pinionRadius)

        mimic0API = PhysxSchema.PhysxMimicJointAPI.Apply(joint1.GetPrim(), UsdPhysics.Tokens.rotX)
        mimic0API.GetReferenceJointRel().AddTarget(joint0Path)
        mimic0API.GetGearingAttr().Set(mimic0Gearing)
        mimic0API.GetOffsetAttr().Set(mimic0Offset)

        link2CapsuleHalfHeight = link0BoxHalfExtent
        link2CapsuleRadius = 0.5 * link0BoxHalfExtent
        link2CapsuleOffset = link2CapsuleHalfHeight + link2CapsuleRadius

        link2Path = defaultPrimPath + "/Link2"
        link2Pos = Gf.Vec3f(0.0, link1Pos[1] + link1BoxHalfExtent[1] + link2CapsuleOffset, 0.0)
        link2Prim = _create_body(stage, link2Path, link2Pos, mass = linkMass)

        link2Capsule = UsdGeom.Capsule.Define(stage, link2Path + "/Geom")
        link2Capsule.CreateHeightAttr(2.0 * link2CapsuleHalfHeight)
        link2Capsule.CreateRadiusAttr(link2CapsuleRadius)
        link2Capsule.CreateAxisAttr(UsdGeom.Tokens.y)
        link2Capsule.CreateDisplayColorAttr([demo.get_primary_color()])

        joint2Path = link2Path + "/InboundJoint"
        joint2 = UsdPhysics.RevoluteJoint.Define(stage, joint2Path)
        joint2.GetAxisAttr().Set(UsdGeom.Tokens.z)
        joint2.GetLowerLimitAttr().Set(-1.0e10)
        joint2.GetUpperLimitAttr().Set(1.0e10)
        joint2.GetBody0Rel().AddTarget(link1Path)
        joint2.GetBody1Rel().AddTarget(link2Path)
        joint2.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, link1BoxHalfExtent[1], 0.0))
        joint2.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, -link2CapsuleOffset, 0.0))

        # mimic joint that should transfer the linear movement of joint1 to angular movement
        # on joint2 (like in a rack and pinion setup)

        gearRadius = 5.0 * link2CapsuleOffset
        mimic1Gearing = (180.0 / math.pi) / gearRadius
        mimic1Offset = 0.0

        mimic1API = PhysxSchema.PhysxMimicJointAPI.Apply(joint2.GetPrim(), UsdPhysics.Tokens.rotZ)
        mimic1API.GetReferenceJointRel().AddTarget(joint1Path)
        mimic1API.GetGearingAttr().Set(mimic1Gearing)
        mimic1API.GetOffsetAttr().Set(mimic1Offset)

        link3CapsuleHalfHeight = link0BoxHalfExtent
        link3CapsuleRadius = 0.5 * link0BoxHalfExtent
        link3CapsuleOffset = link3CapsuleHalfHeight + link3CapsuleRadius

        link3Path = defaultPrimPath + "/Link3"
        link3Pos = Gf.Vec3f(0.0, link2Pos[1] + link2CapsuleOffset + link3CapsuleRadius, 0.0)
        link3Prim = _create_body(stage, link3Path, link3Pos, mass = linkMass)

        link3Capsule = UsdGeom.Capsule.Define(stage, link3Path + "/Geom")
        link3Capsule.CreateHeightAttr(2.0 * link3CapsuleHalfHeight)
        link3Capsule.CreateRadiusAttr(link3CapsuleRadius)
        link3Capsule.CreateAxisAttr(UsdGeom.Tokens.x)
        link3Capsule.CreateDisplayColorAttr([demo.get_primary_color()])

        joint3Path = link3Path + "/InboundJoint"
        joint3 = UsdPhysics.RevoluteJoint.Define(stage, joint3Path)
        joint3.GetAxisAttr().Set(UsdGeom.Tokens.z)
        joint3.GetLowerLimitAttr().Set(-1.0e10)
        joint3.GetUpperLimitAttr().Set(1.0e10)
        joint3.GetBody0Rel().AddTarget(link2Path)
        joint3.GetBody1Rel().AddTarget(link3Path)
        joint3.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, link2CapsuleOffset, 0.0))
        joint3.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, -link3CapsuleRadius, 0.0))

        # mimic joint that should transfer the angular movement of joint2 to angular movement
        # on joint3 using a 1:1 ratio.

        mimic2Gearing = 1.0
        mimic2Offset = 0.0

        mimic2API = PhysxSchema.PhysxMimicJointAPI.Apply(joint3.GetPrim(), UsdPhysics.Tokens.rotZ)
        mimic2API.GetReferenceJointRel().AddTarget(joint2Path)
        mimic2API.GetGearingAttr().Set(mimic2Gearing)
        mimic2API.GetOffsetAttr().Set(mimic2Offset)
