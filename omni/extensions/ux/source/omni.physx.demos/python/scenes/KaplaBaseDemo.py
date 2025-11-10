# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import set_physics_scene_asyncsimrender
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo


class KaplaDemo:
    def __init__(self):
        self.mMeshIndices = []
        self.mPositions = []
        self.mOrientations = []
        self.mLinearVelocities = []
        self.mAngularVelocities = []
        self.mBrickPrototypeIndex = 0
        self.mCylinderIndex = 0
        self.mBrickIndex = 0
        self.mWireJointIndex = 0

    def setContactOffset(self, stage, path):
        prim = stage.GetPrimAtPath(path)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physxCollisionAPI.CreateContactOffsetAttr(0.02)
        physxCollisionAPI.CreateRestOffsetAttr(0.0)

    def create_brick(self, stage, pos, rot, dims, density):
        self.mMeshIndices.append(self.mBrickPrototypeIndex)
        self.mPositions.append(pos)
        self.mOrientations.append(Gf.Quath(rot))
        self.mLinearVelocities.append(Gf.Vec3f(0.0))
        self.mAngularVelocities.append(Gf.Vec3f(0.0))

    def create_cylinder(self, stage, pos, rot, radius, height, density):
        path = self.defaultPrimPath + "/kapla/cylinders/cylinder" + str(self.mCylinderIndex)
        self.mCylinderIndex = self.mCylinderIndex + 1

        color = demo.get_primary_color(1.0)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)
        add_rigid_cylinder(stage, path, radius, height, "X", pos, rot, color, 1.0, linVelocity, angularVelocity)
        self.setContactOffset(stage, path)
        return path

    def create_brick_non_instanced(self, stage, pos, rot, dims, density):
        path = self.defaultPrimPath + "/kapla/bricks/brick" + str(self.mCylinderIndex)
        self.mCylinderIndex = self.mCylinderIndex + 1

        color = demo.get_primary_color()

        linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
        angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

        add_rigid_box(stage, path, dims, pos, rot, color, 1.0, linVelocity, angularVelocity)
        self.setContactOffset(stage, path)
        return path

    def create_cylindrical_tower(self, stage, nbRadialPoints, maxRadius, minRadius, height, dims, centerPos, inDensity):
        startHeight = dims[2]
        density = inDensity

        for i in range(height):
            radius = minRadius + (maxRadius - minRadius) * (1.0 - float(i) / float(height))
            for a in range(nbRadialPoints):
                angle = 6.28 * float(a) / float(nbRadialPoints)
                innerPos = Gf.Vec3f(math.cos(angle) * radius, math.sin(angle) * radius, dims[2] + startHeight)
                w = math.cos((3.14 / 2.0 - angle) / 2.0)
                z = -math.sin((3.14 / 2.0 - angle) / 2.0)
                rot = Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z))
                self.create_brick(stage, innerPos + centerPos, rot, dims, density)

            innerCircumference = (radius - (dims[1] - dims[0])) * 3.1415928 * 2.0
            midCircumference = (radius) * 3.1415928 * 2.0
            outerCircumference = (radius + (dims[1] - dims[0])) * 3.1415928 * 2.0

            nbInnerSlabs = int(innerCircumference / (dims[1] * 2.0))
            nbMidSlabs = int(midCircumference / (dims[1] * 2.0))
            nbOuterSlabs = int(outerCircumference / (dims[1] * 2.0))

            for a in range(nbInnerSlabs):
                angle = 6.28 * float(a) / float(nbInnerSlabs)
                innerPos = Gf.Vec3f(
                    math.cos(angle) * (radius - (dims[1] - dims[0])),
                    math.sin(angle) * (radius - (dims[1] - dims[0])),
                    3.0 * dims[2] + startHeight,
                )
                w = math.cos(-angle / 2.0)
                z = -math.sin(-angle / 2.0)
                rot = Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z))
                self.create_brick(stage, innerPos + centerPos, rot, dims, density)

            for a in range(nbMidSlabs):
                angle = 6.28 * float(a) / float(nbMidSlabs)
                innerPos = Gf.Vec3f(math.cos(angle) * (radius), math.sin(angle) * (radius), 3.0 * dims[2] + startHeight)
                w = math.cos(-angle / 2.0)
                z = -math.sin(-angle / 2.0)
                rot = Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z))
                self.create_brick(stage, innerPos + centerPos, rot, dims, density)

            for a in range(nbOuterSlabs):
                angle = 6.28 * float(a) / float(nbOuterSlabs)
                outerPos = Gf.Vec3f(
                    math.cos(angle) * (radius + (dims[1] - dims[0])),
                    math.sin(angle) * (radius + (dims[1] - dims[0])),
                    3.0 * dims[2] + startHeight,
                )
                w = math.cos(-angle / 2.0)
                z = -math.sin(-angle / 2.0)
                rot = Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z))
                self.create_brick(stage, outerPos + centerPos, rot, dims, density)

            startHeight = startHeight + 4.0 * dims[2]
            density = density * 0.975

        # Now place the lid on the structure...

        midCircumference = (minRadius - dims[1]) * 3.1415928 * 2.0
        nbMidSlabs = int(midCircumference / (dims[2] * 2.0))
        wbase = math.cos(3.14 / 2.0 / 2.0)
        ybase = -math.sin(3.14 / 2.0 / 2.0)
        baseRotation = Gf.Quatf(wbase, Gf.Vec3f(0.0, ybase, 0.0))

        for a in range(nbMidSlabs):
            angle = 6.28 * float(a) / float(nbMidSlabs)
            innerPos = Gf.Vec3f(math.cos(angle) * minRadius, math.sin(angle) * minRadius, dims[0] + startHeight)
            w = math.cos((3.14 / 2.0 - angle) / 2.0)
            z = -math.sin((3.14 / 2.0 - angle) / 2.0)
            rot = Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z))
            self.create_brick(stage, innerPos + centerPos, rot * baseRotation, dims, density)

    def create_twist_tower(self, stage, height, nbBlocksPerLayer, dims, startPos, inDensity):

        rotation = Gf.Quatf.GetIdentity()
        rotationDelta = Gf.Quatf(math.cos(3.14 / 20.0), Gf.Vec3f(0.0, 0.0, -math.sin(3.14 / 20.0)))
        density = inDensity

        startHeight = dims[2]

        for i in range(height):
            startZ = startHeight + i * dims[2] * 2 + dims[2]
            rotMatrix = Gf.Matrix3f(rotation)
            xVec = rotMatrix.GetRow(0)
            for a in range(nbBlocksPerLayer):
                pos = xVec * dims[0] * 2 * (float(a) - float(nbBlocksPerLayer) / 2.0) + Gf.Vec3f(0.0, 0.0, startZ)
                self.create_brick(stage, pos + startPos, rotation, dims, density)

            rotation = rotationDelta * rotation
            density *= 0.95

    def create_rectangular_tower(self, stage, nbX, nbY, height, dims, centerPos, density):
        startHeight = dims[2]

        nbXSupports = 1 + 2 * nbX
        xSpacing = dims[1]
        nbYSlabs = int((2.0 * dims[1] * nbY) / (2.0 * dims[2]))
        rotation = Gf.Quatf(math.cos(3.14 / 4.0), Gf.Vec3f(0, 0, -math.sin(3.14 / 4.0))) * Gf.Quatf(
            math.cos(3.14 / 4.0), Gf.Vec3f(0, -math.sin(3.14 / 4.0), 0)
        )
        idRotation = Gf.Quatf.GetIdentity()

        for i in range(height):
            # First, lets lay down the supports...

            rowColExtents = Gf.Vec3f((nbX) * dims[1], (nbY) * dims[1], 0.0)

            # Back row
            for a in range(nbXSupports):
                pos = Gf.Vec3f(a * xSpacing - rowColExtents[0], -rowColExtents[1], startHeight + dims[2])
                pos2 = Gf.Vec3f(a * xSpacing - rowColExtents[0], rowColExtents[1], startHeight + dims[2])
                self.create_brick(stage, pos + centerPos, idRotation, dims, density)
                self.create_brick(stage, pos2 + centerPos, idRotation, dims, density)

            for a in range(nbX):
                pos = Gf.Vec3f(
                    dims[1] + a * dims[1] * 2 - rowColExtents[0],
                    -rowColExtents[1] - dims[2],
                    startHeight + 2 * dims[2] + dims[0],
                )
                pos2 = Gf.Vec3f(
                    dims[1] + a * dims[1] * 2 - rowColExtents[0],
                    -rowColExtents[1] + dims[2],
                    startHeight + 2 * dims[2] + dims[0],
                )
                self.create_brick(stage, pos + centerPos, rotation, dims, density)
                self.create_brick(stage, pos2 + centerPos, rotation, dims, density)

                pos3 = Gf.Vec3f(
                    dims[1] + a * dims[1] * 2 - rowColExtents[0],
                    rowColExtents[1] - dims[2],
                    startHeight + 2 * dims[2] + dims[0],
                )
                pos4 = Gf.Vec3f(
                    dims[1] + a * dims[1] * 2 - rowColExtents[0],
                    rowColExtents[1] + dims[2],
                    startHeight + 2 * dims[2] + dims[0],
                )
                self.create_brick(stage, pos3 + centerPos, rotation, dims, density)
                self.create_brick(stage, pos4 + centerPos, rotation, dims, density)

            # Sides...
            for a in range(1, nbY):
                for b in range(3):
                    pos = Gf.Vec3f(
                        b * xSpacing - rowColExtents[0], -rowColExtents[1] + a * dims[1] * 2, startHeight + dims[2]
                    )
                    pos2 = Gf.Vec3f(
                        rowColExtents[0] - b * xSpacing, -rowColExtents[1] + a * dims[1] * 2, startHeight + dims[2]
                    )
                    self.create_brick(stage, pos + centerPos, idRotation, dims, density)
                    self.create_brick(stage, pos2 + centerPos, idRotation, dims, density)

            for a in range(1, nbYSlabs - 1):
                pos = Gf.Vec3f(
                    dims[1] - rowColExtents[0],
                    -rowColExtents[1] + dims[2] + 2 * dims[2] * a,
                    startHeight + 2 * dims[2] + dims[0],
                )
                self.create_brick(stage, pos + centerPos, rotation, dims, density)

            for a in range(1, nbYSlabs - 1):
                pos = Gf.Vec3f(
                    rowColExtents[0] - dims[1],
                    -rowColExtents[1] + dims[2] + 2 * dims[2] * a,
                    startHeight + 2 * dims[2] + dims[0],
                )
                self.create_brick(stage, pos + centerPos, rotation, dims, density)

            startHeight = startHeight + 2 * (dims[2] + dims[0])
            density *= 0.975

    def create_geometric_tower(self, stage, height, nbFacets, dims, startPos, startDensity):
        # Create a tower of geometric shaps
        angle = 3.14159 / 2.0
        zRot = Gf.Quatf(math.cos(angle / 2.0), Gf.Vec3f(0, 0, -math.sin(angle / 2.0)))

        rotation = [Gf.Quatf.GetIdentity(), Gf.Quatf(math.cos(3.1415 / 8.0), Gf.Vec3f(0, 0, -math.sin(3.1415 / 8.0)))]

        offset = dims[1]
        density = startDensity

        for i in range(height):
            zOffset = Gf.Vec3f(0, 0, dims[2] + i * dims[2] * 2.0)
            rot = rotation[i & 1]
            rotMatrix = Gf.Matrix3f(rot)

            basis0 = rotMatrix.GetRow(0)
            basis1 = rotMatrix.GetRow(1)

            pos0 = -basis0 * offset + basis1 * dims[0]
            pos1 = basis0 * offset - basis1 * dims[0]
            pos2 = basis0 * dims[0] + basis1 * offset
            pos3 = -basis0 * dims[0] - basis1 * offset

            self.create_brick(stage, pos0 + startPos + zOffset, rot, dims, density)
            self.create_brick(stage, pos1 + startPos + zOffset, rot, dims, density)
            self.create_brick(stage, pos2 + startPos + zOffset, zRot * rot, dims, density)
            self.create_brick(stage, pos3 + startPos + zOffset, zRot * rot, dims, density)

            density = density * 0.98

    def joint_transform(self, bodyPos, bodyRot, p):
        trRotInv = Gf.Rotation(bodyRot).GetInverse()
        return trRotInv.TransformDir(p - bodyPos)

    def create_wire_joint(self, stage, actor0, actor0Pos, actor0Rot, actor1, actor1Pos, actor1Rot, anchorPos):
        # Joint
        wireJointName = self.defaultPrimPath + "/kapla/wireJoints/wireJoint" + str(self.mWireJointIndex)
        self.mWireJointIndex = self.mWireJointIndex + 1

        # D6 is not yet supported, we create fixed instead
        # omni::physx::D6JointTwistDrive twistDrive = { 0.0f, 0.15f, FLT_MAX, 0u };
        # omni::physx::D6JointSwingDrive swingDrive = { 0.0f, 0.15f, FLT_MAX, 0u };
        # omni::physx::D6JointTwistMotion twistMotion = { 2u,   0.0f, 0.0f,          0.0f,
        #                                                    0.0f, 0.0f, (float)M_PI_2, -(float)M_PI_2 };
        # omni::physx::D6JointSwing1Motion swing1Motion = { 2u,   0.0f, 0.0f,          0.0f,
        #                                                    0.0f, 0.0f, (float)M_PI_2, -(float)M_PI_2 };
        # omni::physx::D6JointSwing2Motion swing2Motion = { 2u,   0.0f, 0.0f,          0.0f,
        #                                                    0.0f, 0.0f, (float)M_PI_2, -(float)M_PI_2 };

        # addJointD6(stage, wireJointName, actor0, actor1, joint_transform(actor0Pos, actor0Rot, anchorPos),
        #             actor0Rot.GetInverse(), joint_transform(actor1Pos, actor1Rot, anchorPos), actor1Rot.GetInverse(),
        #             0u, // constraint flag
        #             &twistMotion, &swing1Motion, &swing2Motion, &twistDrive, &swingDrive,
        #             nullptr, // drive targets
        #             3000.f, // break force
        #             2000.0f // break torque
        # );
        jointTrans0 = self.joint_transform(actor0Pos, actor0Rot, anchorPos)
        jointTrans1 = self.joint_transform(actor1Pos, actor1Rot, anchorPos)

        add_joint_fixed(
            stage,
            wireJointName,
            actor0,
            actor1,
            jointTrans0,
            actor0Rot.GetInverse(),
            jointTrans1,
            actor1Rot.GetInverse(),
            1e5,
            1e5,
        )

    def create_flag_joint(self, stage, actor0, actor0Pos, actor0Rot, actor1, actor1Pos, actor1Rot, anchorPos, extents):
        # fixed Joint
        flagJointName = self.defaultPrimPath + "/kapla/flagJoints/flagJoint" + str(self.mWireJointIndex)
        self.mWireJointIndex = self.mWireJointIndex + 1

        jointTrans0 = self.joint_transform(actor0Pos, actor0Rot, anchorPos)
        jointTrans1 = self.joint_transform(actor1Pos, actor1Rot, anchorPos)

        # box is scaled, we need to unapply scale
        jointTrans1[0] = jointTrans1[0] / extents[0]
        jointTrans1[1] = jointTrans1[1] / extents[1]
        jointTrans1[2] = jointTrans1[2] / extents[2]

        add_joint_fixed(
            stage,
            flagJointName,
            actor0,
            actor1,
            jointTrans0,
            actor0Rot.GetInverse(),
            jointTrans1,
            actor1Rot.GetInverse(),
            1e10,
            1e10,
        )

    def create_communication_wire(
        self,
        stage,
        startPos,
        endPos,
        connectRadius,
        connectHeight,
        density,
        offset,
        startBody,
        startBodyPos,
        startBodyRot,
        endBody,
        endBodyPos,
        endBodyRot,
        rot,
    ):
        layDir = Gf.Vec3f(endPos - startPos).GetNormalized()
        distance = (startPos - endPos).GetLength() - 2.0 * offset
        nbToConnect = int((distance) / connectHeight)
        gap = distance / nbToConnect
        pos = startPos + layDir * (offset + connectHeight / 2.0)
        rootActor = startBody
        rootActorPos = startBodyPos
        rootActorRot = startBodyRot
        anchorPos = startPos + layDir * offset

        wireDynName = []
        wireDynPos = []
        wireDynRot = []

        for a in range(nbToConnect):
            dyn = self.create_cylinder(stage, pos, rot, connectRadius, connectHeight / 2.0, density)

            # A.B. same as above, might need a bit tweaking
            # finishBody(physics, dyn, density*0.5f, 10.f);
            # rdapi->setLinearDamping(dyn, 0.1f);
            # rdapi->setAngularDamping(dyn, 0.15f);
            # rdapi->setStabilizationThreshold(dyn, 0.f);

            wireDynName.append(dyn)
            wireDynPos.append(pos)
            wireDynRot.append(rot)

            self.create_wire_joint(stage, rootActor, rootActorPos, rootActorRot, dyn, pos, rot, anchorPos)

            rootActorPos = pos
            rootActorRot = rot
            pos = pos + layDir * gap
            anchorPos = anchorPos + layDir * gap
            rootActor = dyn

        self.create_wire_joint(stage, rootActor, rootActorPos, rootActorRot, endBody, endBodyPos, endBodyRot, anchorPos)

        # Now add some bunting to the connections...
        extents = Gf.Vec3f(connectRadius * 0.5, connectHeight * 0.5, connectHeight * 0.5)
        boxRot = Gf.Quatf(math.cos(3.1415 / 4.0), Gf.Vec3f(0.0, 0.0, -math.sin(3.1415 / 4.0)))

        for a in range(2, nbToConnect, 3):
            # Create a little box to attach
            wireActorName = wireDynName[a]
            wireActorPos = wireDynPos[a]
            wireActorRot = wireDynRot[a]

            transformPos = wireActorPos + Gf.Vec3f(0.0, 0.0, -(connectRadius + 0.5 * connectHeight))
            transformRot = boxRot * wireActorRot

            dyn = self.create_brick_non_instanced(stage, transformPos, transformRot, extents, density)

            anchorPos = transformPos + Gf.Vec3f(0.0, 0.0, 0.5 * connectHeight)

            self.create_flag_joint(
                stage, wireActorName, wireActorPos, wireActorRot, dyn, transformPos, transformRot, anchorPos, extents
            )

    def create_kapla_arena(self, stage, demoInstance):

        # scale factor
        scaleFactor = 0.04

        self.defaultPrimPath, scene = demo.setup_physics_scene(demoInstance, stage)
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        set_physics_scene_asyncsimrender(scene.GetPrim())

        rootXform = UsdGeom.Xform.Define(stage, self.defaultPrimPath + "/kapla")
        rootXform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -0.1))
        rootXform.AddOrientOp().Set(Gf.Quatf(1.0))
        rootXform.AddScaleOp().Set(Gf.Vec3f(1.0))

        # custom GPU buffers
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(self.defaultPrimPath + "/physicsScene"))
        physxSceneAPI.CreateGpuTempBufferCapacityAttr(16 * 1024 * 1024 * 2)        
        physxSceneAPI.CreateGpuHeapCapacityAttr(64 * 1024 * 1024 * 2)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr(256 * 1024 * 2)
        physxSceneAPI.CreateSolverTypeAttr("TGS")

        # sizes
        extents = Gf.Vec3f(1.0, 12.5, 3.125) * scaleFactor

        # towers
        nbOuterRadialLayouts = 384
        outerRadius = 1000.0 * scaleFactor
        height = 5
        centerPos = Gf.Vec3f(0.0, 0.0, 0.0)
        self.create_cylindrical_tower(
            stage, nbOuterRadialLayouts, outerRadius, outerRadius, height, extents, centerPos, 1.0
        )
        self.create_cylindrical_tower(
            stage,
            nbOuterRadialLayouts,
            outerRadius - 25.0 * scaleFactor,
            outerRadius - 25.0 * scaleFactor,
            height,
            extents,
            centerPos,
            1.0,
        )

        # Create point instancer
        geomPointInstancerPath = self.defaultPrimPath + "/kapla/pointInstancer"
        shapeList = UsdGeom.PointInstancer.Define(stage, Sdf.Path(geomPointInstancerPath))
        meshList = shapeList.GetPrototypesRel()

        # Box instanced
        boxActorPath = geomPointInstancerPath + "/brick"
        color = demo.get_primary_color()

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        cubeGeom.AddScaleOp().Set(extents * 2.0)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDensityAttr(0.001)
        self.setContactOffset(stage, boxActorPath)

        # add mesh reference to point instancer
        meshList.AddTarget(Sdf.Path(boxActorPath))

        shapeList.GetProtoIndicesAttr().Set(self.mMeshIndices)
        shapeList.GetPositionsAttr().Set(self.mPositions)
        shapeList.GetOrientationsAttr().Set(self.mOrientations)
        shapeList.GetVelocitiesAttr().Set(self.mLinearVelocities)
        shapeList.GetAngularVelocitiesAttr().Set(self.mAngularVelocities)

    def create_kapla_tower(self, stage, demoInstance):

        # scale factor
        scaleFactor = 0.05

        self.defaultPrimPath, scene = demo.setup_physics_scene(demoInstance, stage)

        set_physics_scene_asyncsimrender(scene.GetPrim())

        rootXform = UsdGeom.Xform.Define(stage, self.defaultPrimPath + "/kapla")
        rootXform.AddTranslateOp().Set(Gf.Vec3f(0.0, 25.0, 0.0))
        rootXform.AddOrientOp().Set(Gf.Quatf(1.0))
        rootXform.AddScaleOp().Set(Gf.Vec3f(1.0))

        # custom GPU buffers
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(self.defaultPrimPath + "/physicsScene"))                
        physxSceneAPI.CreateGpuTempBufferCapacityAttr(16 * 1024 * 1024 * 2)        
        physxSceneAPI.CreateGpuHeapCapacityAttr(64 * 1024 * 1024 * 2)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr(256 * 1024 * 2)

        # sizes
        extents = Gf.Vec3f(2, 26.25, 6.25) * scaleFactor

        # create scene
        if 1:
            nbInnerInnerRadialLayouts = 40
            nbInnerRadialLayouts = 48
            nbMidRadialLayouts = 72
            nbOuterRadialLayouts = 96
            nbOuterOuterRadialLayouts = 128

            innerInnerRadius = 62.5 * scaleFactor
            innerRadius = 112.5 * scaleFactor
            midRadius = 162.5 * scaleFactor
            outerRadius = 212.5 * scaleFactor
            outerOuterRadius = 262.5 * scaleFactor

            if 1:
                centerPos = Gf.Vec3f(0.0, -550.0, 0.0) * scaleFactor
                self.create_cylindrical_tower(
                    stage, nbInnerInnerRadialLayouts, innerInnerRadius, innerInnerRadius, 22, extents, centerPos, 1
                )
                self.create_cylindrical_tower(
                    stage, nbInnerRadialLayouts, innerRadius, innerRadius, 15, extents, centerPos, 1
                )
                self.create_cylindrical_tower(
                    stage, nbMidRadialLayouts, midRadius, midRadius, 11, extents, centerPos, 1
                )
                self.create_cylindrical_tower(
                    stage, nbOuterRadialLayouts, outerRadius, outerRadius, 8, extents, centerPos, 1
                )
                self.create_cylindrical_tower(
                    stage, nbOuterOuterRadialLayouts, outerOuterRadius, outerOuterRadius, 6, extents, centerPos, 1
                )

            if 1:
                self.create_twist_tower(stage, 30, 6, extents, Gf.Vec3f(325, -700, 0) * scaleFactor, 1)
                self.create_twist_tower(stage, 30, 6, extents, Gf.Vec3f(325, -575, 0) * scaleFactor, 1)
                self.create_twist_tower(stage, 30, 6, extents, Gf.Vec3f(325, -450, 0) * scaleFactor, 1)
                self.create_twist_tower(stage, 30, 6, extents, Gf.Vec3f(325, -325, 0) * scaleFactor, 1)

                self.create_rectangular_tower(stage, 27, 19, 6, extents, Gf.Vec3f(0.0, -500.0, 0.0) * scaleFactor, 1)

            if 1:
                capRadius = math.sqrt(extents[1] * extents[1] + extents[1] * extents[1])
                startHeight = extents[2]
                density = 1.0

                # A.B. in original KaplaTower the inertias were modified, we probably need to store those modified
                # intertias
                cylinderRot = Gf.Quatf(math.cos(3.1415 / 4.0), Gf.Vec3f(0.0, -math.sin(3.1415 / 4.0), 0.0))
                cylinder0Pos = Gf.Vec3f(
                    -500.0 * scaleFactor, -887.5 * scaleFactor, (startHeight + extents[2] + 15.0 * 2.0 * extents[2])
                )
                cylinder0 = self.create_cylinder(stage, cylinder0Pos, cylinderRot, capRadius, extents[2], 4.0)

                cylinder1Pos = Gf.Vec3f(
                    500.0 * scaleFactor, -887.5 * scaleFactor, (startHeight + extents[2] + 15.0 * 2.0 * extents[2])
                )
                cylinder1 = self.create_cylinder(stage, cylinder1Pos, cylinderRot, capRadius, extents[2], 4.0)

                cylinder2Pos = Gf.Vec3f(
                    -500.0 * scaleFactor, -237.5 * scaleFactor, (startHeight + extents[2] + 15.0 * 2.0 * extents[2])
                )
                cylinder2 = self.create_cylinder(stage, cylinder2Pos, cylinderRot, capRadius, extents[2], 4.0)

                cylinder3Pos = Gf.Vec3f(
                    500.0 * scaleFactor, -237.5 * scaleFactor, (startHeight + extents[2] + 15.0 * 2.0 * extents[2])
                )
                cylinder3 = self.create_cylinder(stage, cylinder3Pos, cylinderRot, capRadius, extents[2], 4.0)

                self.create_geometric_tower(
                    stage, 15, 4, extents, Gf.Vec3f(-500.0 * scaleFactor, -887.5 * scaleFactor, startHeight), 1.0
                )
                self.create_geometric_tower(
                    stage, 15, 4, extents, Gf.Vec3f(500.0 * scaleFactor, -887.5 * scaleFactor, startHeight), 1.0
                )

                self.create_geometric_tower(
                    stage,
                    40,
                    4,
                    extents,
                    Gf.Vec3f(-500.0 * scaleFactor, -887.5 * scaleFactor, (startHeight + 16 * 2 * extents[2])),
                    0.74,
                )
                self.create_geometric_tower(
                    stage,
                    40,
                    4,
                    extents,
                    Gf.Vec3f(500.0 * scaleFactor, -887.5 * scaleFactor, (startHeight + 16 * 2 * extents[2])),
                    0.74,
                )

                self.create_geometric_tower(
                    stage, 15, 4, extents, Gf.Vec3f(-500.0 * scaleFactor, -237.5 * scaleFactor, startHeight), 1.0
                )
                self.create_geometric_tower(
                    stage, 15, 4, extents, Gf.Vec3f(500.0 * scaleFactor, -237.5 * scaleFactor, startHeight), 1.0
                )

                self.create_geometric_tower(
                    stage,
                    40,
                    4,
                    extents,
                    Gf.Vec3f(-500.0 * scaleFactor, -237.5 * scaleFactor, (startHeight + 16 * 2 * extents[2])),
                    0.74,
                )
                self.create_geometric_tower(
                    stage,
                    40,
                    4,
                    extents,
                    Gf.Vec3f(500.0 * scaleFactor, -237.5 * scaleFactor, (startHeight + 16 * 2 * extents[2])),
                    0.74,
                )

                if 1:
                    connectHeight = 0.75 * 25.0 * scaleFactor
                    self.create_communication_wire(
                        stage,
                        Gf.Vec3f(
                            -500.0 * scaleFactor,
                            -887.0 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        Gf.Vec3f(
                            500.0 * scaleFactor,
                            -887.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        0.15 * 25.0 * scaleFactor,
                        connectHeight,
                        0.05,
                        capRadius,
                        cylinder0,
                        cylinder0Pos,
                        cylinderRot,
                        cylinder1,
                        cylinder1Pos,
                        cylinderRot,
                        Gf.Quatf.GetIdentity(),
                    )

                    self.create_communication_wire(
                        stage,
                        Gf.Vec3f(
                            -500.0 * scaleFactor,
                            -237.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        Gf.Vec3f(
                            500.0 * scaleFactor,
                            -237.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        0.15 * 25.0 * scaleFactor,
                        connectHeight,
                        0.05,
                        capRadius,
                        cylinder2,
                        cylinder2Pos,
                        cylinderRot,
                        cylinder3,
                        cylinder3Pos,
                        cylinderRot,
                        Gf.Quatf.GetIdentity(),
                    )

                    self.create_communication_wire(
                        stage,
                        Gf.Vec3f(
                            -500.0 * scaleFactor,
                            -887.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        Gf.Vec3f(
                            -500.0 * scaleFactor,
                            -237.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        0.15 * 25.0 * scaleFactor,
                        connectHeight,
                        0.05,
                        capRadius,
                        cylinder0,
                        cylinder0Pos,
                        cylinderRot,
                        cylinder2,
                        cylinder2Pos,
                        cylinderRot,
                        Gf.Quatf(math.cos(3.14159 / 4.0), Gf.Vec3f(0, 0, -math.sin(3.14159 / 4.0))),
                    )

                    self.create_communication_wire(
                        stage,
                        Gf.Vec3f(
                            500.0 * scaleFactor,
                            -887.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        Gf.Vec3f(
                            500.0 * scaleFactor,
                            -237.5 * scaleFactor,
                            (startHeight + extents[2] / 2.0 + 15.0 * 2.0 * extents[2]),
                        ),
                        0.15 * 25.0 * scaleFactor,
                        connectHeight,
                        0.05,
                        capRadius,
                        cylinder1,
                        cylinder1Pos,
                        cylinderRot,
                        cylinder3,
                        cylinder3Pos,
                        cylinderRot,
                        Gf.Quatf(math.cos(3.14159 / 4.0), Gf.Vec3f(0, 0, -math.sin(3.14159 / 4.0))),
                    )

        # Create point instancer
        geomPointInstancerPath = self.defaultPrimPath + "/kapla/pointInstancer"
        shapeList = UsdGeom.PointInstancer.Define(stage, Sdf.Path(geomPointInstancerPath))
        meshList = shapeList.GetPrototypesRel()

        # Box instanced
        boxActorPath = geomPointInstancerPath + "/brick"
        color = demo.get_primary_color()

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        cubeGeom.AddScaleOp().Set(extents * 2.0)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDensityAttr(0.001)
        self.setContactOffset(stage, boxActorPath)

        # add mesh reference to point instancer
        meshList.AddTarget(Sdf.Path(boxActorPath))

        shapeList.GetProtoIndicesAttr().Set(self.mMeshIndices)
        shapeList.GetPositionsAttr().Set(self.mPositions)
        shapeList.GetOrientationsAttr().Set(self.mOrientations)
        shapeList.GetVelocitiesAttr().Set(self.mLinearVelocities)
        shapeList.GetAngularVelocitiesAttr().Set(self.mAngularVelocities)
