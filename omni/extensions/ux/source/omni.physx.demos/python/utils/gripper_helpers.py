# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni

from pxr import Sdf, UsdPhysics, Gf, UsdGeom, PhysxSchema
from omni.physx.scripts import utils, physicsUtils

import omni.physxdemos as demo

class GripperDemoBase(demo.AsyncDemoBase):
    """
    Boilerplate class to make a demo with a simplified gripper.
    """

    category = demo.Categories.NONE

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    demo_camera = Sdf.Path("/World/Camera")

    def __init__(self, enable_fabric=False):
        super().__init__(enable_tensor_api=True, enable_fabric=enable_fabric)

        # camera
        self._camera_location = Gf.Vec3f(2.0, 0.3, 0.0)
        self._camera_orientation = Gf.Quatf(0.7071068, 0.0, 0.7071068, 0.0)

        # some global sim options:
        self._pos_iterations = 4
        self._vel_iterations = 1
        self._gravity_magnitude = 9.807


    def create(self, stage):
        # setup sim
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, upAxis = UsdGeom.Tokens.y)
        self._setup_simulation(stage, defaultPrimPath, scene)
        self._setup_materials(stage)

        # surrounding environment
        room = demo.get_demo_room(self, stage, zoom = 0.05, floorOffset = 0.0)

        # setup camera, this should be called after the room setup to not overwrite the camera config
        self._setup_camera(stage)

        # Gripper
        articulationPath = defaultPrimPath + "/gripper"
        articulationXform = UsdGeom.Xform.Define(stage, articulationPath)
        articulationPrim = articulationXform.GetPrim()

        # root
        rootColor = demo.get_primary_color()
        rootPath = defaultPrimPath + "/gripper/root"
        rootGeom = UsdGeom.Cube.Define(stage, rootPath)
        rootPrim = stage.GetPrimAtPath(rootPath)
        rootPosition = Gf.Vec3f(0.2, 0.5, 0.0)
        rootOrientation = Gf.Quatf(-0.7071068, 0.0, 0.0, 0.7071068)
        rootGeom.AddTranslateOp().Set(rootPosition)
        rootGeom.AddOrientOp().Set(rootOrientation)
        rootGeom.CreateSizeAttr(1.0)
        rootGeom.AddScaleOp().Set(Gf.Vec3f(0.15, 0.06, 0.15))
        rootGeom.CreateDisplayColorAttr().Set([rootColor])
        UsdPhysics.CollisionAPI.Apply(rootPrim)
        UsdPhysics.RigidBodyAPI.Apply(rootPrim)
        UsdPhysics.ArticulationRootAPI.Apply(rootGeom.GetPrim())

        # finger 1
        finger1Color = demo.get_primary_color()
        finger1Path = defaultPrimPath + "/gripper/finger1"
        finger1JointPath = defaultPrimPath + "/gripper/finger1Joint"
        finger1Position = Gf.Vec3f(0.2, 0.375, 0.075)
        finger1Rotation = Gf.Quatf(-0.7071068, 0.0, 0.0, 0.7071068)
        finger1Joint = UsdPhysics.RevoluteJoint.Define(stage, finger1JointPath)
        finger1Joint.GetBody0Rel().AddTarget(rootPath)
        finger1Joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.33333, 0.0, 0.5))
        finger1Joint.GetLocalRot0Attr().Set(Gf.Quatf(1.0))
        finger1Joint.GetBody1Rel().AddTarget(finger1Path)
        finger1Joint.GetLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0.0, 0.0))
        finger1Joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0))
        finger1Joint.CreateAxisAttr("Y")
        finger1Joint.GetLowerLimitAttr().Set(-90)
        finger1Joint.GetUpperLimitAttr().Set(90)
        finger1Geom = UsdGeom.Cube.Define(stage, finger1Path)
        finger1Geom.AddTranslateOp().Set(finger1Position)
        finger1Geom.AddOrientOp().Set(finger1Rotation)
        finger1Prim = stage.GetPrimAtPath(finger1Path)
        finger1Geom.CreateSizeAttr(1.0)
        finger1Geom.AddScaleOp().Set(Gf.Vec3f(0.15, 0.03, 0.02))
        finger1Geom.CreateDisplayColorAttr().Set([finger1Color])
        UsdPhysics.CollisionAPI.Apply(finger1Prim)
        UsdPhysics.RigidBodyAPI.Apply(finger1Prim)

        # fingertip 1
        fingertip1Color = demo.get_primary_color()
        fingertip1Path = defaultPrimPath + "/gripper/fingertip1"
        fingertip1JointPath = defaultPrimPath + "/gripper/fingertip1Joint"
        fingertip1Position = Gf.Vec3f(0.2, 0.2305, 0.0469)
        fingertip1Rotation = Gf.Quatf(-0.6940864, -0.1350707, -0.1350707, 0.6940864)
        fingertip1JointPath = UsdPhysics.FixedJoint.Define(stage, fingertip1JointPath)
        fingertip1JointPath.GetBody0Rel().AddTarget(finger1Path)
        fingertip1JointPath.GetLocalPos0Attr().Set(Gf.Vec3f(0.5, 0.0, 0.0))
        fingertip1JointPath.GetLocalRot0Attr().Set(Gf.Quatf(1.0))
        fingertip1JointPath.GetBody1Rel().AddTarget(fingertip1Path)
        fingertip1JointPath.GetLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0.0, 0.0))
        fingertip1JointPath.GetLocalRot1Attr().Set(Gf.Quatf(-0.9815864, 0.0, 0.1910188, 0.0))
        fingertip1Geom = UsdGeom.Cube.Define(stage, fingertip1Path)
        fingertip1Geom.AddTranslateOp().Set(fingertip1Position)
        fingertip1Geom.AddOrientOp().Set(fingertip1Rotation)
        fingertip1Prim = stage.GetPrimAtPath(fingertip1Path)
        fingertip1Geom.CreateSizeAttr(1.0)
        fingertip1Geom.AddScaleOp().Set(Gf.Vec3f(0.15, 0.03, 0.02))
        fingertip1Geom.CreateDisplayColorAttr().Set([fingertip1Color])
        UsdPhysics.CollisionAPI.Apply(fingertip1Prim)
        UsdPhysics.RigidBodyAPI.Apply(fingertip1Prim)

        # finger 2
        finger2Color = demo.get_primary_color()
        finger2Path = defaultPrimPath + "/gripper/finger2"
        finger2JointPath = defaultPrimPath + "/gripper/finger2Joint"
        finger2Position = Gf.Vec3f(0.2, 0.375, -0.075)
        finger2Rotation = Gf.Quatf(-0.7071068, 0.0, 0.0, 0.7071068)
        finger2Joint = UsdPhysics.RevoluteJoint.Define(stage, finger2JointPath)
        finger2Joint.GetBody0Rel().AddTarget(rootPath)
        finger2Joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.33333, 0.0, -0.5))
        finger2Joint.GetLocalRot0Attr().Set(Gf.Quatf(1.0))
        finger2Joint.GetBody1Rel().AddTarget(finger2Path)
        finger2Joint.GetLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0.0, 0.0))
        finger2Joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0))
        finger2Joint.CreateAxisAttr("Y")
        finger2Joint.GetLowerLimitAttr().Set(-90)
        finger2Joint.GetUpperLimitAttr().Set(90)
        finger2Geom = UsdGeom.Cube.Define(stage, finger2Path)
        finger2Geom.AddTranslateOp().Set(finger2Position)
        finger2Geom.AddOrientOp().Set(finger2Rotation)
        finger2Prim = stage.GetPrimAtPath(finger2Path)
        finger2Geom.CreateSizeAttr(1.0)
        finger2Geom.AddScaleOp().Set(Gf.Vec3f(0.15, 0.03, 0.02))
        finger2Geom.CreateDisplayColorAttr().Set([finger2Color])
        UsdPhysics.CollisionAPI.Apply(finger2Prim)
        UsdPhysics.RigidBodyAPI.Apply(finger2Prim)

        # fingertip 2
        fingertip2Color = demo.get_primary_color()
        fingertip2Path = defaultPrimPath + "/gripper/fingertip2"
        fingertip2JointPath = defaultPrimPath + "/gripper/fingertip2Joint"
        fingertip2Position = Gf.Vec3f(0.2, 0.2305, -0.0469)
        fingertip2Rotation = Gf.Quatf(0.6940864, -0.1350707, -0.1350707, -0.6940864)
        fingertip2JointPath = UsdPhysics.FixedJoint.Define(stage, fingertip2JointPath)
        fingertip2JointPath.GetBody0Rel().AddTarget(finger2Path)
        fingertip2JointPath.GetLocalPos0Attr().Set(Gf.Vec3f(0.5, 0.0, 0.0))
        fingertip2JointPath.GetLocalRot0Attr().Set(Gf.Quatf(1.0))
        fingertip2JointPath.GetBody1Rel().AddTarget(fingertip2Path)
        fingertip2JointPath.GetLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0.0, 0.0))
        fingertip2JointPath.GetLocalRot1Attr().Set(Gf.Quatf(-0.9815864, 0.0, -0.1910188, 0.0))
        fingertip2Geom = UsdGeom.Cube.Define(stage, fingertip2Path)
        fingertip2Geom.AddTranslateOp().Set(fingertip2Position)
        fingertip2Geom.AddOrientOp().Set(fingertip2Rotation)
        fingertip2Prim = stage.GetPrimAtPath(fingertip2Path)
        fingertip2Geom.CreateSizeAttr(1.0)
        fingertip2Geom.AddScaleOp().Set(Gf.Vec3f(0.15, 0.03, 0.02))
        fingertip2Geom.CreateDisplayColorAttr().Set([fingertip2Color])
        UsdPhysics.CollisionAPI.Apply(fingertip2Prim)
        UsdPhysics.RigidBodyAPI.Apply(fingertip2Prim)

        # mimic joint between the two fingers
        mimicAPI = PhysxSchema.PhysxMimicJointAPI.Apply(finger1Joint.GetPrim(), UsdPhysics.Tokens.rotY)
        mimicAPI.GetReferenceJointRel().AddTarget(finger2JointPath)
        mimicAPI.GetGearingAttr().Set(1.0)
        mimicAPI.GetOffsetAttr().Set(0.0)

        # set iterations:
        physxArticulationAPI = PhysxSchema.PhysxArticulationAPI.Apply(articulationPrim)
        physxArticulationAPI.GetSolverPositionIterationCountAttr().Set(self._pos_iterations)
        physxArticulationAPI.GetSolverVelocityIterationCountAttr().Set(self._vel_iterations)

    def _setup_camera(self, stage):
        self._cam = UsdGeom.Camera.Define(stage, self.demo_camera)
        cameraPrim = stage.GetPrimAtPath(self.demo_camera)
        physicsUtils.setup_transform_as_scale_orient_translate(self._cam)
        physicsUtils.set_or_add_translate_op(self._cam, translate=self._camera_location)
        physicsUtils.set_or_add_orient_op(self._cam, orient=self._camera_orientation)
        self._cam.CreateClippingRangeAttr(Gf.Vec2f(0.001, 100))

    def _setup_simulation(self, stage, defaultPrimPath, scene):
        scenePrim = stage.GetPrimAtPath(defaultPrimPath + "/physicsScene")
        timeStepsPerSecond = 60.0
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)
        physxSceneAPI.CreateSolverTypeAttr().Set("TGS")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(self._gravity_magnitude)

    def _setup_materials(self, stage):
        physicsMaterialScopePath = stage.GetDefaultPrim().GetPath().AppendChild("PhysicsMaterials")
        UsdGeom.Scope.Define(stage, physicsMaterialScopePath)
        self._boxPhysicsMaterialPath = physicsMaterialScopePath.AppendChild("BoxMaterial")
        self._gripperPhysicsMaterialPath = physicsMaterialScopePath.AppendChild("GripperMaterial")
        utils.addRigidBodyMaterial( stage, self._boxPhysicsMaterialPath, staticFriction=1.5, dynamicFriction=1.5, restitution=0.0)
        utils.addRigidBodyMaterial(stage, self._gripperPhysicsMaterialPath, staticFriction=1.5, dynamicFriction=1.5, restitution=0.0)

    def on_shutdown(self):
        super().on_shutdown()
