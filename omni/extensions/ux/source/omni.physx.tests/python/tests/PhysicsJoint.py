# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.physx.scripts.utils as physxUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, PhysicsKitStageAsyncTestCase, TestCategory
from pxr import Usd, Gf, UsdGeom, Sdf, UsdPhysics, UsdUtils, PhysxSchema, PhysicsSchemaTools
import omni.physx.bindings._physx as pb
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase
import omni.physxui.scripts.utils as physxUiUtils
from omni.physx import get_physx_simulation_interface, get_physx_interface
from omni.physx.bindings._physx import SimulationEvent, SETTING_DISPLAY_JOINTS, SETTING_LOG_ROBOTICS, SETTING_ENABLE_EXTENDED_JOINT_ANGLES
import omni.usd
import omni.kit.test
import carb.settings
import math
from omni.physxtests import utils
from omni.physx.scripts import physicsUtils
from omni.physxui import get_physxui_interface
from omni.kit.commands import execute
import unittest

metersPerUnit = 0.01
kilogramsPerUnit = 1.0
lengthScale = 1.0 / metersPerUnit
massScale = 1.0 / kilogramsPerUnit


class PhysicsJointTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.articulation_stage_and_scene_setup()

        # create xform and 3 boxes
        self._boxes_parent_xform = UsdGeom.Xform.Define(self._stage, "/World/articulation")

        box_actor0_path = "/articulation/boxActor0"
        box_actor1_path = "/articulation/boxActor1"
        box_actor2_path = "/articulation/boxActor2"

        size = Gf.Vec3f(100.0)
        position = Gf.Vec3f(0.0, 0.0, 200.0)
        self._box0 = physicsUtils.add_box(self._stage, box_actor0_path, size, position)

        position = Gf.Vec3f(0.0, 0.0, 400.0)
        self._box1_rigid = physicsUtils.add_rigid_box(self._stage, box_actor1_path, size, position)

        position = Gf.Vec3f(0.0, 0.0, 600.0)
        self._box2_rigid = physicsUtils.add_rigid_box(self._stage, box_actor2_path, size, position)

    async def test_articulation_joint_excluded_error_closed_circuit(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._boxes_parent_xform.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(self._box0.GetPrim())

        joint0 = physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        joint1 = physxUtils.createJoint(self._stage, "Revolute", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())
        physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box2_rigid.GetPrim())

        message = f"RigidBody ({self._box0.GetPrimPath()}) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints:\n{self._box0.GetPrimPath()}\n{joint0.GetPrimPath()}\n{joint1.GetPrimPath()}\n from articulation, the joint will be now excluded from the articulation."
        with utils.ExpectMessage(self, message):
            self.step()

    async def test_joint_disjointed_body_error_position_change(self):
        joint_types = {"Fixed": True, "Revolute": True}

        for joint_type, error_expected in joint_types.items():
            with self.subTest(joint_type):
                jointPrim = physxUtils.createJoint(self._stage, joint_type, self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())
                joint = UsdPhysics.Joint(jointPrim)
                message = f"PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: {jointPrim.GetPrimPath()}"

                joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -100.0))
                with utils.ExpectMessage(self, message, expected_result=error_expected):
                    self.step(reset_simulation_after=True)

    async def test_joint_disjointed_body_error_rotation_change(self):
        joint_types = {"Fixed": True, "Revolute": False}

        for joint_type, error_expected in joint_types.items():
            with self.subTest(joint_type):
                jointPrim = physxUtils.createJoint(self._stage, joint_type, self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())
                joint = UsdPhysics.Joint(jointPrim)
                message = f"PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: {jointPrim.GetPrimPath()}"

                joint.GetLocalRot0Attr().Set(Gf.Quatf(0.0, 0.707, 0.0, 0.707))
                with utils.ExpectMessage(self, message, expected_result=error_expected):
                    self.step(reset_simulation_after=True)

    async def test_articulation_distance_joint_error(self):
        UsdPhysics.ArticulationRootAPI.Apply(self._box1_rigid.GetPrim())
        jointPrim = physxUtils.createJoint(self._stage, "Distance", self._box1_rigid.GetPrim(), self._box2_rigid.GetPrim())

        self.detach_stage()
        message = f"Articulation cannot contain a distance joint! (Please exclude it from articulation) ({jointPrim.GetPath().pathString})"
        with utils.ExpectMessage(self, message):
            self.attach_stage()

    async def test_articulation_robotics_log_settings_joint_message(self):
        settings = carb.settings.get_settings()
        UsdPhysics.ArticulationRootAPI.Apply(self._box1_rigid.GetPrim())
        jointPrim = physxUtils.createJoint(self._stage, "Prismatic", self._box2_rigid.GetPrim(), self._box1_rigid.GetPrim())
        
        message = f"Physics USD: Joint {jointPrim.GetPath().pathString} body rel does not follow articulation hierarchy; consider swapping body0/body1 rels to match."
        for log_expected in (True, False):
            self.detach_stage()
            with self.subTest(log_expected):
                settings.set(pb.SETTING_LOG_ROBOTICS, log_expected)

                with utils.ExpectMessage(self, message, expected_result=log_expected):
                    self.attach_stage()

    async def test_joint_missing_body_prim_error(self):
        UsdPhysics.RigidBodyAPI.Apply(self._box0.GetPrim())

        joint0 = physxUtils.createJoint(self._stage, "Revolute", self._box0.GetPrim(), self._box1_rigid.GetPrim())
        rev_joint = UsdPhysics.RevoluteJoint(joint0)
        rev_joint.GetBody1Rel().ClearTargets(True)
        error_prim = "/World/errorPrim"
        rev_joint.GetBody1Rel().AddTarget(error_prim)

        message = f"Joint ({rev_joint.GetPrim().GetPrimPath()}) body relationship {error_prim} points to a non existent prim, joint will not be created."
        with utils.ExpectMessage(self, message):
            self.step()

    async def test_physics_joint_collision_enabled(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # setup static actor0
        boxActor0Path = "/boxActor0"
        size = Gf.Vec3f(100.0)
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        boxPrim = physicsUtils.add_box(stage, boxActor0Path, size, position)
        UsdPhysics.CollisionAPI.Apply(boxPrim)

        # setup dynamic actor1
        boxActor1Path = "/boxActor1"
        position = Gf.Vec3f(0.0, 0.0, 300.0)
        boxPrim = physicsUtils.add_rigid_box(stage, boxActor1Path, size, position)

        # setup prismatic joint
        prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, "/prismaticJoint")
        prismaticJoint.CreateAxisAttr("Z")
        prismaticJoint.CreateBody0Rel().SetTargets([boxActor0Path])
        prismaticJoint.CreateBody1Rel().SetTargets([boxActor1Path])

        for _ in range(100):
            self.step()

        prim = stage.GetPrimAtPath(boxActor1Path)
        pos = prim.GetAttribute("xformOp:translate").Get()

        # print(pos)
        self.assertTrue(pos[2] < 0.0)

        get_physx_simulation_interface().detach_stage()

        prim.GetAttribute("xformOp:translate").Set(position)
        prim.GetAttribute("physics:velocity").Set(Gf.Vec3f(0.0))
        prismaticJoint.GetCollisionEnabledAttr().Set(True)

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)

        for _ in range(100):
            self.step()

        pos = prim.GetAttribute("xformOp:translate").Get()

        # print(pos)
        self.assertTrue(pos[2] > 90.0)

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            self._jointPath = PhysicsSchemaTools.decodeSdfPath(
                event.payload["jointPath"][0], event.payload["jointPath"][1]
            )

    async def test_physics_joint_break(self):
        stage = await self.new_stage()

        events = get_physx_interface().get_simulation_event_stream_v2()
        self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # Box0 static
        size = Gf.Vec3f(50.0)
        position = Gf.Vec3f(200.0, 0.0, 300.0)
        color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)

        box0Prim = physicsUtils.add_rigid_box(stage, "/boxActor0", size, position, color=color)
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, box0Prim.GetPrimPath())
        physicsBodyAPI.GetRigidBodyEnabledAttr().Set(False)

        # Box1 dynamic
        position = Gf.Vec3f(0.0, 0.0, 300.0)
        color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)

        self._box1Prim = physicsUtils.add_rigid_box(stage, "/boxActor1", size, position, color=color)

        # fixed joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, defaultPrimPath + "/fixedJoint")

        fixedJoint.CreateBody0Rel().SetTargets([box0Prim.GetPrimPath()])
        fixedJoint.CreateBody1Rel().SetTargets([self._box1Prim.GetPrimPath()])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(-2.0, 0.0, 0.0))
        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(2.0, 0.0, 0.0))

        fixedJoint.CreateBreakForceAttr().Set(1000.0 * massScale * lengthScale * lengthScale)

        self._jointPath = None

        for _ in range(5):
            self.step()

        self.assertTrue(fixedJoint.GetPath() == self._jointPath)

        self._simulation_event_sub = None

    async def test_physics_joint_body_resync(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # Box
        xform = UsdGeom.Xform.Define(stage, defaultPrimPath + "/xform")
        cube = UsdGeom.Cube.Define(stage, defaultPrimPath + "/xform/cube")
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # fixed joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, defaultPrimPath + "/fixedJoint")
        fixedJoint.CreateBody1Rel().SetTargets([cube.GetPrim().GetPrimPath()])

        for _ in range(5):
            self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        pos = xform.GetPrim().GetAttribute("xformOp:translate").Get()

        # print(pos)
        toleranceEpsilon = 0.01
        self.assertTrue(abs(pos[2]) < toleranceEpsilon)

        # cause a resync
        cube.AddTranslateOp().Set(Gf.Vec3f(10.0))
        for _ in range(5):
            self.step()

        pos = xform.GetPrim().GetAttribute("xformOp:translate").Get()

        # print(pos)
        self.assertTrue(abs(pos[2]) < toleranceEpsilon)

    async def test_physics_joint_delete_body_resync(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # Box0        
        cube0 = UsdGeom.Cube.Define(stage, defaultPrimPath + "/xform/cube0")
        UsdPhysics.RigidBodyAPI.Apply(cube0.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube0.GetPrim())

        # Box1        
        cube1 = UsdGeom.Cube.Define(stage, defaultPrimPath + "/xform/cube1")
        UsdPhysics.RigidBodyAPI.Apply(cube1.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube1.GetPrim())

        # fixed joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, defaultPrimPath + "/fixedJoint")
        fixedJoint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        fixedJoint.CreateBody1Rel().SetTargets([cube1.GetPrim().GetPrimPath()])

        for _ in range(2):
            self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 1})        

        # delete body1 and the joint
        stage.RemovePrim(cube1.GetPrim().GetPrimPath())
        stage.RemovePrim(fixedJoint.GetPrim().GetPrimPath())        

        for _ in range(2):
            self.step()
        
        # cause a resync - add the cube1 back
        cube1 = UsdGeom.Cube.Define(stage, defaultPrimPath + "/xform/cube1")
        UsdPhysics.RigidBodyAPI.Apply(cube1.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube1.GetPrim())

        for _ in range(2):
            self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})        

    async def test_physics_rack_pinion_joint_delete(self):
        stage = await self.new_stage()
        
        cube0 = UsdGeom.Cube.Define(stage, "/World/cube0")
        UsdPhysics.RigidBodyAPI.Apply(cube0.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube0.GetPrim())

        cube1 = UsdGeom.Cube.Define(stage, "/World/cube1")
        UsdPhysics.RigidBodyAPI.Apply(cube1.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube1.GetPrim())
        
        revolute_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint")
        revolute_joint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        prismatic_joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/prismaticJoint")
        prismatic_joint.CreateBody0Rel().SetTargets([cube1.GetPrim().GetPrimPath()])
                
        rack_joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, '/World/rackAndPinionJoint')
                
        rack_joint.CreateHingeRel().SetTargets([revolute_joint.GetPrim().GetPrimPath()])
        rack_joint.CreatePrismaticRel().SetTargets([prismatic_joint.GetPrim().GetPrimPath()])
        rack_joint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        rack_joint.CreateBody1Rel().SetTargets([cube1.GetPrim().GetPrimPath()])

        self.step()
        
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 3})
        
        stage.RemovePrim(revolute_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 1})

        stage.RemovePrim(prismatic_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})

        stage.RemovePrim(rack_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})
        
    async def test_physics_rack_pinion_body_delete(self):
        stage = await self.new_stage()
        
        cube0 = UsdGeom.Cube.Define(stage, "/World/cube0")
        UsdPhysics.RigidBodyAPI.Apply(cube0.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube0.GetPrim())

        cube1 = UsdGeom.Cube.Define(stage, "/World/cube1")
        UsdPhysics.RigidBodyAPI.Apply(cube1.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube1.GetPrim())
        
        revolute_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint")
        revolute_joint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        prismatic_joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/prismaticJoint")
        prismatic_joint.CreateBody0Rel().SetTargets([cube1.GetPrim().GetPrimPath()])
                
        rack_joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, '/World/rackAndPinionJoint')
                
        rack_joint.CreateHingeRel().SetTargets([revolute_joint.GetPrim().GetPrimPath()])
        rack_joint.CreatePrismaticRel().SetTargets([prismatic_joint.GetPrim().GetPrimPath()])
        rack_joint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        rack_joint.CreateBody1Rel().SetTargets([cube1.GetPrim().GetPrimPath()])

        self.step()
        
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 3})
        
        stage.RemovePrim(cube0.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1, "numConstraints": 1})

        stage.RemovePrim(cube1.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0, "numConstraints": 0})

        stage.RemovePrim(revolute_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0, "numConstraints": 0})

        stage.RemovePrim(prismatic_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0, "numConstraints": 0})

        stage.RemovePrim(rack_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 0, "numDynamicRigids": 0, "numConstraints": 0})
        
    async def test_physics_gear_joint_delete(self):
        stage = await self.new_stage()
        
        cube0 = UsdGeom.Cube.Define(stage, "/World/cube0")
        UsdPhysics.RigidBodyAPI.Apply(cube0.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube0.GetPrim())

        cube1 = UsdGeom.Cube.Define(stage, "/World/cube1")
        UsdPhysics.RigidBodyAPI.Apply(cube1.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube1.GetPrim())
        
        revolute_joint0 = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint0")
        revolute_joint0.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        revolute_joint1 = UsdPhysics.RevoluteJoint.Define(stage, "/World/revoluteJoint1")
        revolute_joint1.CreateBody0Rel().SetTargets([cube1.GetPrim().GetPrimPath()])
                
        gear_joint = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, '/World/gearJoint')
                
        gear_joint.CreateHinge0Rel().SetTargets([revolute_joint0.GetPrim().GetPrimPath()])
        gear_joint.CreateHinge1Rel().SetTargets([revolute_joint1.GetPrim().GetPrimPath()])
        gear_joint.CreateBody0Rel().SetTargets([cube0.GetPrim().GetPrimPath()])
        gear_joint.CreateBody1Rel().SetTargets([cube1.GetPrim().GetPrimPath()])

        self.step()
        
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 3})
        
        stage.RemovePrim(revolute_joint0.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 1})

        stage.RemovePrim(revolute_joint1.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})

        stage.RemovePrim(gear_joint.GetPrim().GetPrimPath())
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 0})
        
        
    async def _run_physics_prismatic_joint_frame_test(
        self, driveType, flipBodyRel, flipFrames, expectedDir, useArticulation=False
    ):
        axes = "XYZ"
        for i in range(3):
            ax = axes[i]
            await self._prismatic_test_setup(
                ax,
                driveType=driveType,
                flipBodyRel=flipBodyRel,
                flipFrames=flipFrames,
                setupAsArticulation=useArticulation,
            )

            self.step(1)

            # check that the box moved in the right direction:
            trans = (
                UsdGeom.Xformable(self._dynamic_box)
                .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                .ExtractTranslation()
            )
            self.assertGreater(expectedDir * trans[i], 0.0)

    async def test_physics_joint_prismatic_frames_pos_drive(self):
        driveType = "pos"
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0
        )
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=True, flipFrames=False, expectedDir=-1.0
        )
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=True, expectedDir=-1.0
        )

    async def test_physics_joint_prismatic_frames_vel_drive(self):
        driveType = "vel"
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0
        )

    async def test_physics_joint_prismatic_frames_pos_drive_articulation(self):
        driveType = "pos"
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0, useArticulation=True
        )
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=True, flipFrames=False, expectedDir=-1.0, useArticulation=True
        )
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=True, expectedDir=-1.0, useArticulation=True
        )

    async def test_physics_joint_prismatic_frames_vel_drive_articulation(self):
        driveType = "vel"
        await self._run_physics_prismatic_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0, useArticulation=True
        )

    async def _run_physics_revolute_joint_frame_test(
        self, driveType, flipBodyRel, flipFrames, expectedDir, useArticulation=False
    ):
        axes = "XYZ"
        checkDirs = {
            "X": (Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 0, 1)),
            "Y": (Gf.Vec3f(0, 0, 1), Gf.Vec3f(1, 0, 0)),
            "Z": (Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)),
        }
        for i in range(3):
            ax = axes[i]
            await self._revolute_test_setup(
                ax,
                driveType=driveType,
                flipBodyRel=flipBodyRel,
                flipFrames=flipFrames,
                setupAsArticulation=useArticulation,
            )

            self.step(1)

            # check that the box moved in the right direction:
            rot = (
                UsdGeom.Xformable(self._dynamic_box)
                .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                .ExtractRotation()
            )
            rotatedCheckVec = rot.TransformDir(checkDirs[ax][0])
            dotp = Gf.Dot(rotatedCheckVec, checkDirs[ax][1])
            self.assertGreater(expectedDir * dotp, 0.0)

    async def test_physics_joint_revolute_frames_pos_drive(self):
        driveType = "pos"
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0
        )
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=True, flipFrames=False, expectedDir=-1.0
        )
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=True, expectedDir=-1.0
        )

    async def test_physics_joint_revolute_frames_vel_drive(self):
        driveType = "vel"
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0
        )

    async def test_physics_joint_revolute_frames_pos_drive_articulation(self):
        driveType = "pos"
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0, useArticulation=True
        )
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=True, flipFrames=False, expectedDir=-1.0, useArticulation=True
        )
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=True, expectedDir=-1.0, useArticulation=True
        )

    async def test_physics_joint_revolute_frames_vel_drive_articulation(self):
        driveType = "vel"
        await self._run_physics_revolute_joint_frame_test(
            driveType=driveType, flipBodyRel=False, flipFrames=False, expectedDir=1.0, useArticulation=True
        )

    def _assertAlmostEqualVector3(self, vectorA, vectorB, tol=1.0e-5):
        for a, b in zip(vectorA, vectorB):
            self.assertAlmostEqual(a, b, delta=tol)

    async def test_physics_joint_revolute_frame_changes(self):
        axes = "XYZ"
        sincos45 = math.sqrt(2) / 2.0
        testCases = {
            "X": {"frameRot": Gf.Quatf(sincos45, 0, sincos45, 0), "checkAx": Gf.Vec3f(0, 0, -1)},
            "Y": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, 0, 1)},
            "Z": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, -1, 0)},
        }
        for i in range(3):
            ax = axes[i]
            await self._revolute_test_setup(ax, driveType="vel")
            self._driveAPI.CreateTargetVelocityAttr(5.0)
            self.step(1, dt=0.001)
            # rotate frame and check angular velocity:
            self._joint.CreateLocalRot0Attr().Set(testCases[ax]["frameRot"])

            self.step(5)
            bodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
            self.assertTrue(bodyAPI)
            angVel = bodyAPI.GetAngularVelocityAttr().Get()
            angVel.Normalize()
            self._assertAlmostEqualVector3(testCases[ax]["checkAx"], angVel)

    async def test_physics_joint_prismatic_frame_changes(self):
        axes = "XYZ"
        sincos45 = math.sqrt(2) / 2.0
        testCases = {
            "X": {"frameRot": Gf.Quatf(sincos45, 0, sincos45, 0), "checkAx": Gf.Vec3f(0, 0, -1)},
            "Y": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, 0, 1)},
            "Z": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, -1, 0)},
        }
        for i in range(3):
            ax = axes[i]
            await self._prismatic_test_setup(ax, driveType="vel")
            self._driveAPI.CreateTargetVelocityAttr(1.0)
            self.step(1, dt=0.001)
            # rotate frame and check angular velocity:
            self._joint.CreateLocalRot0Attr().Set(testCases[ax]["frameRot"])

            self.step(5)
            bodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
            self.assertTrue(bodyAPI)
            linVel = bodyAPI.GetVelocityAttr().Get()
            linVel.Normalize()
            self._assertAlmostEqualVector3(testCases[ax]["checkAx"], linVel)

    @unittest.skip("OM-42711")
    async def test_physics_joint_revolute_frame_changes_articulation(self):
        axes = "XYZ"
        sincos45 = math.sqrt(2) / 2.0
        testCases = {
            "X": {"frameRot": Gf.Quatf(sincos45, 0, sincos45, 0), "checkAx": Gf.Vec3f(0, 0, -1)},
            "Y": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, 0, 1)},
            "Z": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, -1, 0)},
        }
        for swapBodyRels in [True, False]:
            for i in range(3):
                ax = axes[i]
                await self._revolute_test_setup(
                    ax, driveType="vel", setupAsArticulation=True, flipBodyRel=swapBodyRels
                )
                self._driveAPI.CreateTargetVelocityAttr(5.0)
                self.step(1, dt=0.001)
                # rotate frame and check angular velocity:
                if swapBodyRels:
                    self._joint.CreateLocalRot1Attr().Set(testCases[ax]["frameRot"])
                else:
                    self._joint.CreateLocalRot0Attr().Set(testCases[ax]["frameRot"])

                self.step(5)
                bodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
                self.assertTrue(bodyAPI)
                angVel = bodyAPI.GetAngularVelocityAttr().Get()
                angVel.Normalize()
                self._assertAlmostEqualVector3(testCases[ax]["checkAx"], angVel)

    @unittest.skip("OM-42711")
    async def test_physics_joint_prismatic_frame_changes_articulation(self):
        axes = "XYZ"
        sincos45 = math.sqrt(2) / 2.0
        testCases = {
            "X": {"frameRot": Gf.Quatf(sincos45, 0, sincos45, 0), "checkAx": Gf.Vec3f(0, 0, -1)},
            "Y": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, 0, 1)},
            "Z": {"frameRot": Gf.Quatf(sincos45, sincos45, 0, 0), "checkAx": Gf.Vec3f(0, -1, 0)},
        }
        for swapBodyRels in [True, False]:
            for i in range(3):
                ax = axes[i]
                await self._prismatic_test_setup(
                    ax, driveType="vel", setupAsArticulation=True, flipBodyRel=swapBodyRels
                )
                self._driveAPI.CreateTargetVelocityAttr(5.0)
                self.step(1, dt=0.001)
                # rotate frame and check angular velocity:
                if swapBodyRels:
                    self._joint.CreateLocalRot1Attr().Set(testCases[ax]["frameRot"])
                else:
                    self._joint.CreateLocalRot0Attr().Set(testCases[ax]["frameRot"])

                self.step(5)
                bodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
                self.assertTrue(bodyAPI)
                linVel = bodyAPI.GetVelocityAttr().Get()
                linVel.Normalize()
                self._assertAlmostEqualVector3(testCases[ax]["checkAx"], linVel)

    def _get_link_translate_orient(self):
        translate = self._dynamic_box.GetPrim().GetAttribute("xformOp:translate").Get()
        orient = self._dynamic_box.GetPrim().GetAttribute("xformOp:orient").Get()
        return translate, orient

    def _run_prismatic_limit_test(
        self,
        lowerLimit: float,
        expectedTranslationLower: Gf.Vec3f,
        upperLimit: float,
        expectedTranslationUpper: Gf.Vec3f,
        driveType: str = "pos",
        delta: float = 0.05,  # delta to add/subtract from limits for drive target to be exceeding limit
        steps: int = 50,  # number of steps to simulate to settle into limit
        targetVel=1.0
    ):
        self._joint.CreateLowerLimitAttr(lowerLimit)
        self._joint.CreateUpperLimitAttr(upperLimit)

        targetPos = lowerLimit - delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        self.step(steps)
        translate, _ = self._get_link_translate_orient()
        self._assertAlmostEqualVector3(expectedTranslationLower, translate, tol=1e-4)

        targetPos = upperLimit + delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        self.step(steps)
        translate, _ = self._get_link_translate_orient()
        self._assertAlmostEqualVector3(expectedTranslationUpper, translate, tol=1e-4)

    async def _run_physics_joint_prismatic_limits_test(
        self, driveType="pos", flipFrames=False, flipBodyRel=False, useArticulation=False
    ):
        axes = "XYZ"
        lowerLimit = -0.05
        upperLimit = 0.1
        for i in range(3):
            ax = axes[i]
            await self._prismatic_test_setup(
                ax, driveType=driveType, flipBodyRel=flipBodyRel, flipFrames=flipFrames, setupAsArticulation=useArticulation
            )
            # use high gains and some damping:
            if driveType == "pos":
                self._driveAPI.CreateStiffnessAttr().Set(10.0)
                self._driveAPI.CreateDampingAttr().Set(1.0)
            elif driveType == "vel":
                self._driveAPI.CreateDampingAttr().Set(10.0)
            lowerExpected = Gf.Vec3f(0.0)
            upperExpected = Gf.Vec3f(0.0)
            lowerExpected[i] = lowerLimit
            upperExpected[i] = upperLimit

            # the direction flips if one but not both bodies or frames are
            if flipBodyRel ^ flipFrames:
                lowerExpected = -lowerExpected
                upperExpected = -upperExpected

            self._run_prismatic_limit_test(
                lowerLimit=lowerLimit,
                expectedTranslationLower=lowerExpected,
                upperLimit=upperLimit,
                expectedTranslationUpper=upperExpected,
                driveType=driveType
            )

            # test change listeners by adjusting limits and retesting
            self._run_prismatic_limit_test(
                lowerLimit=lowerLimit * 1.2,
                expectedTranslationLower=lowerExpected * 1.2,
                upperLimit=upperLimit * 0.9,
                expectedTranslationUpper=upperExpected * 0.9,
                driveType=driveType
            )

    async def test_physics_joint_prismatic_limits_pos_drive(self):
        await self._run_physics_joint_prismatic_limits_test()
        await self._run_physics_joint_prismatic_limits_test(flipBodyRel=True)
        await self._run_physics_joint_prismatic_limits_test(flipBodyRel=True, flipFrames=True)

    async def test_physics_joint_prismatic_limits_articulation_pos_drive(self):
        await self._run_physics_joint_prismatic_limits_test(useArticulation=True)
        await self._run_physics_joint_prismatic_limits_test(flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_prismatic_limits_test(flipBodyRel=True, flipFrames=True, useArticulation=True)

    async def test_physics_joint_prismatic_limits_vel_drive(self):
        await self._run_physics_joint_prismatic_limits_test(driveType="vel")

    async def test_physics_joint_prismatic_limits_articulation_vel_drive(self):
        await self._run_physics_joint_prismatic_limits_test(driveType="vel", useArticulation=True)

    async def test_physics_joint_prismatic_limits_articulation_pos_state(self):
        await self._run_physics_joint_prismatic_limits_test(driveType="jointStatePos", useArticulation=True)
        await self._run_physics_joint_prismatic_limits_test(driveType="jointStatePos", flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_prismatic_limits_test(driveType="jointStatePos", flipBodyRel=True, flipFrames=True, useArticulation=True)

    async def test_physics_joint_prismatic_limits_articulation_vel_state(self):
        await self._run_physics_joint_prismatic_limits_test(driveType="jointStateVel", useArticulation=True)

    # TOL IS IN DEGREES because Gf.Rotation.GetAngle returns angle in deg
    def _assertQuatsClose(self, quatA: Gf.Quatf, quatB: Gf.Quatf, tol=0.2):
        deltaRot = quatA * quatB.GetConjugate()
        angle = Gf.Rotation(Gf.Quatd(deltaRot)).GetAngle()
        self.assertLess(abs(angle), tol)

    def _run_revolute_limit_test(
        self,
        lowerLimit: float,
        expectedOrientLower: Gf.Quatf,
        upperLimit: float,
        expectedOrientUpper: Gf.Quatf,
        driveType: str = "pos",
        delta: float = 5,  # delta to add/subtract from limits for drive target to be exceeding limit
        steps: int = 50,  # number of steps to simulate to settle into limit
        saveDebugUSDA: bool = False,
        targetVel=90
    ):
        self._joint.CreateLowerLimitAttr(lowerLimit)
        self._joint.CreateUpperLimitAttr(upperLimit)

        targetPos = lowerLimit - delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        if saveDebugUSDA:
            self._stage.Export("RevLimTestLower.usda")
        self.step(steps)
        _, orient = self._get_link_translate_orient()
        self._assertQuatsClose(expectedOrientLower, orient)

        targetPos = upperLimit + delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        if saveDebugUSDA:
            self._stage.Export("RevLimTestUpper.usda")
        self.step(steps)
        _, orient = self._get_link_translate_orient()
        self._assertQuatsClose(expectedOrientUpper, orient)

    def _set_drive_target_pos_or_vel(self, targetPos, targetVel, driveType):
        if driveType == "pos":
            self._driveAPI.CreateTargetPositionAttr(targetPos)
        elif driveType == "vel":
            self._driveAPI.CreateTargetVelocityAttr(math.copysign(targetVel, targetPos))
        elif driveType == "jointStatePos":
            self._stateAPI.CreatePositionAttr(targetPos)
        elif driveType == "jointStateVel":
            self._stateAPI.CreateVelocityAttr(math.copysign(targetVel, targetPos))

    @staticmethod
    def _get_axis_rot_quat(ax: int, angleDeg: float):
        angleRad = math.radians(angleDeg)
        cosAngleDiv2 = math.cos(angleRad * 0.5)
        sinAngleDiv2 = math.sin(angleRad * 0.5)
        imVec = Gf.Vec3f(0.0)
        imVec[ax] = sinAngleDiv2
        return Gf.Quatf(cosAngleDiv2, imVec)

    async def _run_physics_joint_revolute_limits_test(self, driveType="pos", flipFrames=False, flipBodyRel=False, useArticulation=False):
        axes = "XYZ"
        lowerLimitAngle = -5
        upperLimitAngle = 10
        for i in range(3):
            ax = axes[i]
            await self._revolute_test_setup(
                ax, driveType=driveType, flipBodyRel=flipBodyRel, flipFrames=flipFrames, setupAsArticulation=useArticulation
            )

            lowerExpectedAngle = lowerLimitAngle
            upperExpectedAngle = upperLimitAngle

            # the direction flips if one but not both bodies or frames are
            if flipBodyRel ^ flipFrames:
                lowerExpectedAngle = -lowerExpectedAngle
                upperExpectedAngle = -upperExpectedAngle

            self._run_revolute_limit_test(
                lowerLimit=lowerLimitAngle,
                expectedOrientLower=self._get_axis_rot_quat(i, lowerExpectedAngle),
                upperLimit=upperLimitAngle,
                expectedOrientUpper=self._get_axis_rot_quat(i, upperExpectedAngle),
                driveType=driveType,
            )

            # test change listeners by adjusting limits and retesting
            factor = 0.6
            self._run_revolute_limit_test(
                lowerLimit=lowerLimitAngle * factor,
                expectedOrientLower=self._get_axis_rot_quat(i, lowerExpectedAngle * factor),
                upperLimit=upperLimitAngle * factor,
                expectedOrientUpper=self._get_axis_rot_quat(i, upperExpectedAngle * factor),
                driveType=driveType,
            )

    async def test_physics_joint_revolute_limits_pos_drive(self):
        await self._run_physics_joint_revolute_limits_test()
        await self._run_physics_joint_revolute_limits_test(flipBodyRel=True)
        await self._run_physics_joint_revolute_limits_test(flipBodyRel=True, flipFrames=True)

    async def test_physics_joint_revolute_limits_articulation_pos_drive(self):
        await self._run_physics_joint_revolute_limits_test(useArticulation=True)
        await self._run_physics_joint_revolute_limits_test(flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_revolute_limits_test(flipBodyRel=True, flipFrames=True, useArticulation=True)

    async def test_physics_joint_revolute_limits_vel_drive(self):
        await self._run_physics_joint_revolute_limits_test(driveType="vel")

    async def test_physics_joint_revolute_limits_articulation_vel_drive(self):
        await self._run_physics_joint_revolute_limits_test(driveType="vel", useArticulation=True)

    async def test_physics_joint_revolute_limits_articulation_pos_state(self):
        await self._run_physics_joint_revolute_limits_test(driveType="jointStatePos", useArticulation=True)
        await self._run_physics_joint_revolute_limits_test(driveType="jointStatePos", flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_revolute_limits_test(driveType="jointStatePos", flipBodyRel=True, flipFrames=True, useArticulation=True)

    async def test_physics_joint_revolute_limits_articulation_vel_state(self):
        await self._run_physics_joint_revolute_limits_test(driveType="jointStateVel", useArticulation=True)

    def _run_d6_revolute_limit_test(
        self,
        lowerLimit: float,
        expectedOrientLower: Gf.Quatf,
        upperLimit: float,
        expectedOrientUpper: Gf.Quatf,
        driveType="pos",
        delta: float = 5,  # delta to add/subtract from limits for drive target to be exceeding limit
        steps: int = 50,  # number of steps to simulate to settle into limit
        saveDebugUSDA: bool = False,
        targetVel=90
    ):
        self._d6_limitAPI.CreateLowAttr(lowerLimit)
        self._d6_limitAPI.CreateHighAttr(upperLimit)

        targetPos = lowerLimit - delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        if saveDebugUSDA:
            self._stage.Export("D6RevLimTestLower.usda")
        self.step(steps)
        _, orient = self._get_link_translate_orient()
        self._assertQuatsClose(expectedOrientLower, orient)

        targetPos = upperLimit + delta
        self._set_drive_target_pos_or_vel(targetPos, targetVel, driveType)
        if saveDebugUSDA:
            self._stage.Export("D6RevLimTestUpper.usda")
        self.step(steps)
        _, orient = self._get_link_translate_orient()
        self._assertQuatsClose(expectedOrientUpper, orient)

    async def _run_physics_joint_d6_revolute_limits_test(
        self, driveType="pos", flipFrames=False, flipBodyRel=False, useArticulation=False
    ):
        axes = "XYZ"
        lowerLimitAngle = -5
        upperLimitAngle = 10
        for i in range(3):
            ax = axes[i]
            await self._d6_revolute_test_setup(
                ax, driveType=driveType, flipBodyRel=flipBodyRel, flipFrames=flipFrames, setupAsArticulation=useArticulation
            )

            lowerExpectedAngle = lowerLimitAngle
            upperExpectedAngle = upperLimitAngle

            # the direction flips if one but not both bodies or frames are flipped
            if flipBodyRel ^ flipFrames:
                lowerExpectedAngle = -lowerExpectedAngle
                upperExpectedAngle = -upperExpectedAngle

            self._run_d6_revolute_limit_test(
                lowerLimit=lowerLimitAngle,
                expectedOrientLower=self._get_axis_rot_quat(i, lowerExpectedAngle),
                upperLimit=upperLimitAngle,
                expectedOrientUpper=self._get_axis_rot_quat(i, upperExpectedAngle),
                driveType=driveType
            )

            # test change listeners by adjusting limits and retesting
            factor = 0.6
            self._run_d6_revolute_limit_test(
                lowerLimit=lowerLimitAngle * factor,
                expectedOrientLower=self._get_axis_rot_quat(i, lowerExpectedAngle * factor),
                upperLimit=upperLimitAngle * factor,
                expectedOrientUpper=self._get_axis_rot_quat(i, upperExpectedAngle * factor),
                driveType=driveType
            )

    async def test_physics_joint_d6_revolute_limits_pos_drive(self):
        await self._run_physics_joint_d6_revolute_limits_test()
        await self._run_physics_joint_d6_revolute_limits_test(flipBodyRel=True)
        await self._run_physics_joint_d6_revolute_limits_test(flipBodyRel=True, flipFrames=True)

    async def test_physics_joint_d6_revolute_limits_vel_drive(self):
        await self._run_physics_joint_d6_revolute_limits_test(driveType="vel")

    async def test_physics_joint_d6_revolute_limits_articulation_pos_drive(self):
        await self._run_physics_joint_d6_revolute_limits_test(useArticulation=True)
        await self._run_physics_joint_d6_revolute_limits_test(flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_d6_revolute_limits_test(flipBodyRel=True, flipFrames=True, useArticulation=True)
    
    async def test_physics_joint_d6_revolute_limits_articulation_vel_drive(self):
        await self._run_physics_joint_d6_revolute_limits_test(driveType="vel", useArticulation=True)

    async def test_physics_joint_d6_revolute_limits_articulation_pos_state(self):
        await self._run_physics_joint_d6_revolute_limits_test(driveType="jointStatePos", useArticulation=True)
        await self._run_physics_joint_d6_revolute_limits_test(driveType="jointStatePos", flipBodyRel=True, useArticulation=True)
        await self._run_physics_joint_d6_revolute_limits_test(driveType="jointStatePos", flipBodyRel=True, flipFrames=True, useArticulation=True)
    
    async def test_physics_joint_d6_revolute_limits_articulation_vel_state(self):
        await self._run_physics_joint_d6_revolute_limits_test(driveType="jointStateVel", useArticulation=True)
            
    def _run_spherical_limit_test(
        self,
        axisIndex,
        coneAngle0Limit,
        coneAngle1Limit,
        flippedTestOrder=False,
        factor=1.0,
        delta: float = 5,  # delta to add/subtract from limits for drive target to be exceeding limit
        steps: int = 40,  # number of steps to simulate to settle into limit
        saveDebugUSDA: bool = False,
    ):
        # setup limits:
        coneAngle0Limit *= factor
        coneAngle1Limit *= factor
        self._joint.CreateConeAngle0LimitAttr().Set(coneAngle0Limit)
        self._joint.CreateConeAngle1LimitAttr().Set(coneAngle1Limit)

        coneAngles = (coneAngle0Limit, coneAngle1Limit)
        for i in range(2):
            coneAngle = coneAngles[i]
            if flippedTestOrder:
                coneAngle = -coneAngle
            expectedOrientLower = self._get_axis_rot_quat((axisIndex + 1 + i) % 3, -coneAngle)
            expectedOrientUpper = self._get_axis_rot_quat((axisIndex + 1 + i) % 3, coneAngle)

            targetPos = -coneAngles[i] - delta
            self._driveAPIs[i].CreateTargetPositionAttr(targetPos)
            self._driveAPIs[(i + 1) % 2].CreateTargetPositionAttr(0.0)
            if saveDebugUSDA:
                self._stage.Export("SphericalLimTestLower.usda")
            self.step(steps)
            _, orient = self._get_link_translate_orient()
            self._assertQuatsClose(expectedOrientLower, orient)

            targetPos = coneAngles[i] + delta
            self._driveAPIs[i].CreateTargetPositionAttr(targetPos)
            self._driveAPIs[(i + 1) % 2].CreateTargetPositionAttr(0.0)
            if saveDebugUSDA:
                self._stage.Export("D6RevLimTestUpper.usda")
            self.step(steps)
            _, orient = self._get_link_translate_orient()
            self._assertQuatsClose(expectedOrientUpper, orient)

    async def _run_physics_joint_spherical_limits_test(
        self, flipBodyRel=False, useArticulation=False
    ):
        axes = "XYZ"
        coneAngle0Limit = 3
        coneAngle1Limit = 6
        for i in range(3):
            ax = axes[i]
            await self._setup_stage_and_bodies(setupAsArticulation=useArticulation)
            self._setup_joint(jointType="spherical", flipBodyRel=flipBodyRel, axis=ax)

            # integration does not yet support drives on spherical, so setup extra external drive
            driveJoint = UsdPhysics.Joint.Define(self._stage, "/World/DriveJoint")
            driveJoint.CreateExcludeFromArticulationAttr().Set(True)

            if flipBodyRel:
                if useArticulation:
                    driveJoint.CreateBody1Rel().SetTargets([self._dummy_root_link.GetPath()])
                driveJoint.CreateBody0Rel().SetTargets([self._dynamic_box.GetPath()])
            else:
                if useArticulation:
                    driveJoint.CreateBody0Rel().SetTargets([self._dummy_root_link.GetPath()])
                driveJoint.CreateBody1Rel().SetTargets([self._dynamic_box.GetPath()])

            limits = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]
            freeAxis0 = "rot" + axes[(i + 1) % 3]
            freeAxis1 = "rot" + axes[(i + 2) % 3]
            limits.remove(freeAxis0)
            limits.remove(freeAxis1)
            for limit in limits:
                limitAPI = UsdPhysics.LimitAPI.Apply(driveJoint.GetPrim(), limit)
                limitAPI.CreateLowAttr(1.0)
                limitAPI.CreateHighAttr(-1.0)
            self._driveAPIs = [UsdPhysics.DriveAPI.Apply(driveJoint.GetPrim(), freeAxis0),
                               UsdPhysics.DriveAPI.Apply(driveJoint.GetPrim(), freeAxis1)]
            for api in self._driveAPIs:
                api.CreateStiffnessAttr(1.0)
                api.CreateDampingAttr(0.1)

            self._run_spherical_limit_test(
                i, coneAngle0Limit, coneAngle1Limit, flippedTestOrder=flipBodyRel
            )

            # test change listeners by adjusting limits and retesting
            self._run_spherical_limit_test(
                i, coneAngle0Limit, coneAngle1Limit, factor=0.6, flippedTestOrder=flipBodyRel
            )

    async def test_physics_joint_spherical_limits(self):
        await self._run_physics_joint_spherical_limits_test()
        await self._run_physics_joint_spherical_limits_test(flipBodyRel=True)

    async def test_physics_joint_spherical_limits_articulation(self):
        await self._run_physics_joint_spherical_limits_test(useArticulation=True)
        await self._run_physics_joint_spherical_limits_test(flipBodyRel=True, useArticulation=True)

    async def test_physics_joint_state_api_disabled(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # box0 static
        box0ActorPath = defaultPrimPath + "/box0"
        physicsUtils.add_box(stage, box0ActorPath, Gf.Vec3f(10, 100.0, 10.0), Gf.Vec3f(0, 0, 0))

        # box1
        box1ActorPath = defaultPrimPath + "/box1"
        physicsUtils.add_rigid_box(stage, box1ActorPath, Gf.Vec3f(10, 100.0, 10.0), Gf.Vec3f(0, 0, 1000.0))

        # revolute joint
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, defaultPrimPath + "/revoluteJoint")

        revoluteJoint.CreateBody0Rel().SetTargets([box0ActorPath])
        revoluteJoint.CreateBody1Rel().SetTargets([box1ActorPath])

        # Apply Joint State API
        jointStateAPI = PhysxSchema.JointStateAPI.Apply(revoluteJoint.GetPrim(), "angular")
        jointStateAPI.CreatePositionAttr().Set(45.0)
        jointStateAPI.CreateVelocityAttr().Set(0.0)
        settings = carb.settings.get_settings()

        message = "Physics USD: JointStateAPI applied to Joint " + revoluteJoint.GetPath().pathString + " it's not supported, and it will be ignored."

        settings.set(SETTING_LOG_ROBOTICS, True)
        with utils.ExpectMessage(self, message):
            self.step(reset_simulation_after=True)
        settings.set(SETTING_LOG_ROBOTICS, False)

    async def test_physics_joint_instancer(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)
        rootPrim = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(rootPrim.GetPrim())

        defaultPrimPath = stage.GetDefaultPrim().GetPath()

        # Physics scene
        UsdPhysics.Scene.Define(stage, str(defaultPrimPath) + "/physicsScene")

        # configure ropes:
        self._linkHalfLength = 3
        self._linkRadius = 0.5 * self._linkHalfLength
        self._ropeLength = 300
        self._ropeSpacing = 15.0
        self._coneAngleLimit = 110
        self._rope_damping = 10.0
        self._rope_stiffness = 1.0

        # configure collider capsule:
        self._capsuleY = 50.0
        self._capsuleHeight = 400.0
        self._capsuleRadius = 20.0
        self._capsuleRestOffset = -2.0

        def _createCapsule(path: Sdf.Path):
            capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
            capsuleGeom.CreateHeightAttr(self._linkHalfLength)
            capsuleGeom.CreateRadiusAttr(self._linkRadius)
            capsuleGeom.CreateAxisAttr("X")

            UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
            massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
            massAPI.CreateDensityAttr().Set(0.00005)

        def _createJoint(jointPath):        
            joint = UsdPhysics.Joint.Define(self._stage, jointPath)

            # locked DOF (lock - low is greater than high)
            d6Prim = joint.GetPrim()
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transX")
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transY")
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transZ")
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "rotX")
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

            # Moving DOF:
            dofs = ["rotY", "rotZ"]
            for d in dofs:
                limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
                limitAPI.CreateLowAttr(-self._coneAngleLimit)
                limitAPI.CreateHighAttr(self._coneAngleLimit)

                # joint drives for rope dynamics:
                driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
                driveAPI.CreateTypeAttr("force")
                driveAPI.CreateDampingAttr(self._rope_damping)
                driveAPI.CreateStiffnessAttr(self._rope_stiffness)

        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5
        zStart = 0

        scopePath = defaultPrimPath.AppendChild(f"Rope")
        UsdGeom.Scope.Define(self._stage, scopePath)
        
        # capsule instancer
        instancerPath = scopePath.AppendChild("rigidBodyInstancer")
        rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)
        
        capsulePath = instancerPath.AppendChild("capsule")
        _createCapsule(capsulePath)
        
        meshIndices = []
        positions = []
        orientations = []
        
        z = zStart
        y = self._capsuleY + self._capsuleRadius + self._linkRadius * 1.4            
        for linkInd in range(numLinks):
            meshIndices.append(0)
            x = xStart + linkInd * linkLength
            positions.append(Gf.Vec3f(x, y, z))
            orientations.append(Gf.Quath(1.0))

        meshList = rboInstancer.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(capsulePath)

        rboInstancer.GetProtoIndicesAttr().Set(meshIndices)
        rboInstancer.GetPositionsAttr().Set(positions)
        rboInstancer.GetOrientationsAttr().Set(orientations)
        
        # joint instancer
        jointInstancerPath = scopePath.AppendChild("jointInstancer")
        jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
        
        jointPath = jointInstancerPath.AppendChild("joint")
        _createJoint(jointPath)
        
        meshIndices = []
        body0s = []
        body0indices = []
        localPos0 = []
        localRot0 = []
        body1s = []
        body1indices = []
        localPos1 = []
        localRot1 = []      
        body0s.append(instancerPath)
        body1s.append(instancerPath)

        jointX = self._linkHalfLength - 0.5 * self._linkRadius
        for linkInd in range(numLinks - 1):
            meshIndices.append(0)
            
            body0indices.append(linkInd)
            body1indices.append(linkInd + 1)
                        
            localPos0.append(Gf.Vec3f(jointX, 0, 0)) 
            localPos1.append(Gf.Vec3f(-jointX, 0, 0)) 
            localRot0.append(Gf.Quath(1.0))
            localRot1.append(Gf.Quath(1.0))

        meshList = jointInstancer.GetPhysicsPrototypesRel()
        meshList.AddTarget(jointPath)

        jointInstancer.GetPhysicsProtoIndicesAttr().Set(meshIndices)

        jointInstancer.GetPhysicsBody0sRel().SetTargets(body0s)
        jointInstancer.GetPhysicsBody0IndicesAttr().Set(body0indices)
        jointInstancer.GetPhysicsLocalPos0sAttr().Set(localPos0)
        jointInstancer.GetPhysicsLocalRot0sAttr().Set(localRot0)

        jointInstancer.GetPhysicsBody1sRel().SetTargets(body1s)
        jointInstancer.GetPhysicsBody1IndicesAttr().Set(body1indices)
        jointInstancer.GetPhysicsLocalPos1sAttr().Set(localPos1)
        jointInstancer.GetPhysicsLocalRot1sAttr().Set(localRot1)

        for _ in range(5):
            self.step()
        utils.check_stats(self, {"numCapsuleShapes": 66, "numDynamicRigids": 66, "numConstraints": 65})

    async def _test_physics_joint_extended_angles(self, enable: bool):
        stage = await self.new_stage()

        self.settings = carb.settings.get_settings()
        enableExtendedJointAngles = self.settings.get_as_bool(SETTING_ENABLE_EXTENDED_JOINT_ANGLES)

        self.settings.set_bool(SETTING_ENABLE_EXTENDED_JOINT_ANGLES, enable)

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        scene.CreateGravityMagnitudeAttr(0.0)

        size = Gf.Vec3f(0.01, 0.01, 0.05)

        body0Path = defaultPrimPath + "/body0"
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        body0Prim = physicsUtils.add_rigid_box(stage, body0Path, size, position)

        body1Path = defaultPrimPath + "/body1"
        position = Gf.Vec3f(0.0, 0.0, 0.1)
        body1Prim = physicsUtils.add_rigid_box(stage, body1Path, size, position)

        timeStep = 1.0 / 60.0
        maxAnglePerStep = 10.0
        maxAngVel = maxAnglePerStep / timeStep

        physxBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(body0Prim)
        physxBodyAPI.CreateMaxAngularVelocityAttr(maxAngVel)
        physxBodyAPI.CreateAngularDampingAttr(0.0)

        physxBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(body1Prim)
        physxBodyAPI.CreateMaxAngularVelocityAttr(maxAngVel)
        physxBodyAPI.CreateAngularDampingAttr(0.0)

        lowerLimit = -2.0
        upperLimit = 2.0

        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, "/revoluteJoint")
        revoluteJoint.CreateAxisAttr(UsdGeom.Tokens.x)
        revoluteJoint.CreateBody1Rel().AddTarget(body0Path)
        revoluteJoint.CreateLowerLimitAttr(lowerLimit)
        revoluteJoint.CreateUpperLimitAttr(upperLimit)

        d6Joint = UsdPhysics.Joint.Define(stage, "/d6Joint")
        d6Joint.CreateBody1Rel().AddTarget(body1Path)
        d6JointPrim = d6Joint.GetPrim()
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transX)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transY)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transZ)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotX)
        limitAPI.CreateLowAttr(lowerLimit)
        limitAPI.CreateHighAttr(upperLimit)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotY)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotZ)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        revoluteJoint.CreateAxisAttr(UsdGeom.Tokens.x)

        jointLocalRot = self._get_axis_rot_quat(0, 359.0)
        # without extended angles, this will get wrapped to -1 degrees and thus the limits are not
        # breached. With extended limits, a large correction should take place since the angle
        # is way beyond the limit.

        revoluteJoint.CreateLocalRot1Attr(jointLocalRot)
        d6Joint.CreateLocalRot1Attr(jointLocalRot)

        self.step(1, timeStep)

        orient0 = body0Prim.GetAttribute("xformOp:orient").Get()
        orient1 = body1Prim.GetAttribute("xformOp:orient").Get()

        angle0 = 2.0 * math.acos(orient0.GetNormalized().GetReal()) * 180.0 / math.pi
        angle1 = 2.0 * math.acos(orient1.GetNormalized().GetReal()) * 180.0 / math.pi

        if (enable):
            self.assertGreater(angle0, 0.9 * maxAnglePerStep)
            self.assertGreater(angle1, 0.9 * maxAnglePerStep)
        else:
            self.assertAlmostEqual(angle0, 0.0, delta=0.001)
            self.assertAlmostEqual(angle1, 0.0, delta=0.001)

        self.settings.set_bool(SETTING_ENABLE_EXTENDED_JOINT_ANGLES, enableExtendedJointAngles)

    #
    # Test that with extended angles enabled, joint angles can be in range (-360, 360)
    #
    async def test_physics_joint_extended_angles_enabled(self):
        await self._test_physics_joint_extended_angles(True)

    #
    # Test that with extended angles disabled, joint angles will be mapped to (-180, 180)
    #
    async def test_physics_joint_extended_angles_disabled(self):
        await self._test_physics_joint_extended_angles(False)

    #
    # Test that it is possible to use different drive parameters for rotY (swing1) and rotZ (swing2)
    #
    async def test_physics_joint_d6_asymmetric_swing_drive(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        scene.CreateGravityMagnitudeAttr(0.0)

        objectMass = 0.5
        objectBoxHalfExtent = Gf.Vec3f(0.1)
        objectInertia = Gf.Vec3f(objectMass * (1.0 / 6.0) * (4.0 * objectBoxHalfExtent[0] * objectBoxHalfExtent[0]))

        angAcc = 1.0
        torque = angAcc * objectInertia[0]

        swing1ExpectedAngleInDegrees = 5.0
        swing1Stiffness = torque / swing1ExpectedAngleInDegrees

        dampingRatio = 1.0  # critical damping
        swing1Damping = dampingRatio * 2.0 * math.sqrt((swing1Stiffness * 180.0 / math.pi) * objectInertia[0])
        swing1Damping = swing1Damping * math.pi / 180.0

        swing2Stiffness = swing1Stiffness * 0.5
        swing2ExpectedAngleInDegrees = swing1ExpectedAngleInDegrees * 2.0  # with half the stiffness, double the angle is expected

        swing2Damping = dampingRatio * 2.0 * math.sqrt((swing2Stiffness * 180.0 / math.pi) * objectInertia[0])
        swing2Damping = swing2Damping * math.pi / 180.0

        bodyPath = defaultPrimPath + "/body"
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        bodyPrim = physicsUtils.add_rigid_box(stage, bodyPath, 2.0 * objectBoxHalfExtent, position)

        timeStep = 1.0 / 60.0

        physxBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(bodyPrim)
        physxBodyAPI.CreateAngularDampingAttr(0.0)

        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateMassAttr(objectMass)
        massAPI.CreateDiagonalInertiaAttr(objectInertia)

        d6Joint = UsdPhysics.Joint.Define(stage, "/d6Joint")
        d6Joint.CreateBody1Rel().AddTarget(bodyPath)
        d6JointPrim = d6Joint.GetPrim()

        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transX)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transY)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.transZ)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        #---
        limitAPI = UsdPhysics.LimitAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotX)
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)

        driveAPI = UsdPhysics.DriveAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotY)
        driveAPI.CreateStiffnessAttr(swing1Stiffness)
        driveAPI.CreateDampingAttr(swing1Damping)
        #---
        driveAPI = UsdPhysics.DriveAPI.Apply(d6JointPrim, UsdPhysics.Tokens.rotZ)
        driveAPI.CreateStiffnessAttr(swing2Stiffness)
        driveAPI.CreateDampingAttr(swing2Damping)

        forceAPI = PhysxSchema.PhysxForceAPI.Apply(bodyPrim)
        forceAPI.CreateTorqueAttr(Gf.Vec3f(0.0, torque, torque))
        forceAPI.CreateWorldFrameEnabledAttr(True)
        forceAPI.CreateModeAttr(PhysxSchema.Tokens.force)

        # run enough steps to reach a stable rest state
        self.step(200, timeStep)

        orient = bodyPrim.GetAttribute("xformOp:orient").Get()

        rot = Gf.Rotation(orient)

        xAxis = Gf.Vec3f(1.0, 0.0, 0.0)

        xRot = rot.TransformDir(xAxis)

        yRotAngle = 180.0 * math.atan2(-xRot[2], xRot[0]) / math.pi
        zRotAngle = 180.0 * math.atan2(xRot[1], xRot[0]) / math.pi

        errTolerance = 0.03  # 3%

        # even though applying the same torque on both axes, the different drive stiffness values for swing1 and swing2
        # should result in different rest angles along the corresponding axes
        self.assertAlmostEqual(yRotAngle, swing1ExpectedAngleInDegrees, delta=swing1ExpectedAngleInDegrees * errTolerance)
        self.assertAlmostEqual(zRotAngle, swing2ExpectedAngleInDegrees, delta=swing2ExpectedAngleInDegrees * errTolerance)


class PhysicsJointCoreTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def setUp(self):
        await self.new_stage(def_up_and_mpu=False)

        self.settings = carb.settings.get_settings()
        self.joint_helpers_setting = self.settings.get_as_bool(SETTING_DISPLAY_JOINTS)
        self.settings.set_bool(SETTING_DISPLAY_JOINTS, True)

        self.usd_context = omni.usd.get_context()
        self.message_base = "PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: "

    async def tearDown(self):
        self.settings.set_bool(SETTING_DISPLAY_JOINTS, self.joint_helpers_setting)

    def setUpBaseBoxes(self, sz0=None, pos0=None, orient0=None, sz1=None, pos1=None, orient1=None):
        boxActor0Path = "/boxActor0"
        boxActor1Path = "/boxActor1"

        size = Gf.Vec3f(0.1, 100, 10) if sz0 is None else sz0
        position = Gf.Vec3f(100.0, 0.0, 0.0) if pos0 is None else pos0
        orient = Gf.Quatf(1.0) if orient0 is None else orient0
        self.box0Rigid = physicsUtils.add_rigid_box(self._stage, boxActor0Path, size, position, orient)

        size = Gf.Vec3f(10, 10, 0.1) if sz1 is None else sz1
        position = Gf.Vec3f(-100.0, 10.0, 1.0)if pos1 is None else pos1
        orient = Gf.Quatf(1.0) if orient1 is None else orient1
        self.box1Rigid = physicsUtils.add_rigid_box(self._stage, boxActor1Path, size, position, orient)

    async def test_physics_joint_err_billboard_selection_change(self):
        self.setUpBaseBoxes()
        jointPrim = physxUtils.createJoint(self._stage, "Revolute", self.box0Rigid.GetPrim(), self.box1Rigid.GetPrim())
        jointPath = jointPrim.GetPrimPath()
        revoluteJoint = UsdPhysics.RevoluteJoint(jointPrim)

        self.usd_context.get_selection().set_selected_prim_paths([str(jointPath)], False)
        await self.wait(10)
        self.usd_context.get_selection().set_selected_prim_paths([], False)
        await self.wait(10)

        message = f"{self.message_base}{jointPath}"

        # no error - error billboard hidden
        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)

        # pos change -> error billboard appears
        tmp = revoluteJoint.GetLocalPos1Attr().Get()
        revoluteJoint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 1.0, 0.0))
        with utils.ExpectMessage(self, message):
            await self.step(stop_timeline_after=True)
        await self.wait(10)
        revoluteJoint.GetLocalPos1Attr().Set(tmp)

        # no error - error billboard hidden
        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)
        await self.wait(10)

        # clear stage to check for leaks
        await self.new_stage(def_up_and_mpu=False)

    async def test_physics_joint_align_util(self):
        def do_step(num_steps=1, dt=1.0 / 60.0):
            get_physx_simulation_interface().simulate(dt, num_steps * dt)

        self.setUpBaseBoxes()
        cache = UsdGeom.XformCache()

        gbody0 = self.box0Rigid.GetPrim()
        gbody1 = self.box1Rigid.GetPrim()

        async def do_test(name, body0, body1, body0change, body0base, testPos, testRot):
            print(f"{name} Joint Align Test: body0 {body0}, body1 {body1}, body0 change {body0change}, align to body0 {body0base}, testPos {testPos}, testRot {testRot}")
            prim = physxUtils.createJoint(self._stage, name, body0, body1)
            path = prim.GetPrimPath()
            joint = UsdPhysics.Joint(prim)
            message = f"{self.message_base}{path}"

            if testPos:
                joint_get_local_pos = joint.GetLocalPos0Attr if body0change else joint.GetLocalPos1Attr
                joint_get_local_pos().Set(Gf.Vec3f(-10.0, 10.0, 20.0))

            if testRot:
                joint_get_local_rot = joint.GetLocalRot0Attr if body0change else joint.GetLocalRot1Attr
                joint_get_local_rot().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d([1, 1, 1]), 45).GetQuat()))

            async def check_and_fix(fixPos, fixRot):
                with utils.ExpectMessage(self, message):
                    do_step()

                physxUiUtils.set_opposite_body_transform(self._stage, cache, prim, body0base, fixPos, fixRot)
                with utils.ExpectMessage(self, message, False):
                    do_step()

            await check_and_fix(testPos, testRot)
            self._stage.RemovePrim(path)

        setups = [
            ("Revolute", [(True, False)]),  # test pos fix
            ("Spherical", [(True, False)]),  # test pos fix
            ("Fixed", [(True, False), (False, True), (True, True)]),  # test pos and rot fix
            ("Prismatic", [(True, False), (False, True), (True, True)]),  # test pos and rot fix
        ]

        for setup in setups:
            for bodies in [(gbody0, gbody1), (gbody0, None)]:
                for body0change in [True, False]:
                    for body0base in [True, False]:
                        for fix in setup[1]:
                            await do_test(
                                setup[0],
                                bodies[0], bodies[1],
                                body0change, body0base,
                                fix[0], fix[1]
                            )

    async def test_physics_joint_align_util_regression(self):
        self.setUpBaseBoxes(
            sz0=Gf.Vec3f(1, 1, 1),
            pos0=Gf.Vec3f(5.872132778167725, 3.3920040130615234, 30.0),
            orient0=Gf.Quatf(0.97181, 0, 0, -0.25377),
            sz1=Gf.Vec3f(1, 1, 1),
            pos1=Gf.Vec3f(6.372028, 3.075788, 30),
            orient1=Gf.Quatf(0.94522, 0, 0, -0.35644)
        )

        body0 = self.box0Rigid.GetPrim()
        body1 = self.box1Rigid.GetPrim()

        # revolute joint, should error
        jointPrim = physxUtils.createJoint(self._stage, "Revolute", body0, body1)
        jointPath = jointPrim.GetPrimPath()
        revoluteJoint = UsdPhysics.RevoluteJoint(jointPrim)
        message = f"{self.message_base}{jointPath}"

        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)

        revoluteJoint.GetLocalPos0Attr().Set(Gf.Vec3f(0.296192, 0, 0))
        revoluteJoint.GetLocalPos1Attr().Set(Gf.Vec3f(-0.296192, 0, 0))

        revoluteJoint.GetLocalRot0Attr().Set(Gf.Quatf(0.603792, -0.368016, -0.603792, -0.368016))
        revoluteJoint.GetLocalRot1Attr().Set(Gf.Quatf(0.635829, -0.30939, -0.635829, -0.30939))

        with utils.ExpectMessage(self, message):
            await self.step(stop_timeline_after=True)

        # run fix and test
        cache = UsdGeom.XformCache()
        physxUiUtils.set_opposite_body_transform(self._stage, cache, jointPrim, body0base=True, fixpos=True, fixrot=False)
        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)

        self._stage.RemovePrim(jointPath)

        # fixed joint, should error
        jointPrim = physxUtils.createJoint(self._stage, "Fixed", body0, body1)
        jointPath = jointPrim.GetPrimPath()
        fixedJoint = UsdPhysics.FixedJoint(jointPrim)
        message = f"{self.message_base}{jointPath}"

        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)

        fixedJoint.GetLocalPos0Attr().Set(Gf.Vec3f(0.296192, 0, 0))
        fixedJoint.GetLocalPos1Attr().Set(Gf.Vec3f(-0.296192, 0, 0))

        fixedJoint.GetLocalRot0Attr().Set(Gf.Quatf(0.603792, -0.368016, -0.603792, -0.368016))
        fixedJoint.GetLocalRot1Attr().Set(Gf.Quatf(0.635829, -0.30939, -0.635829, -0.30939))

        with utils.ExpectMessage(self, message):
            await self.step(stop_timeline_after=True)

        # run fix and test
        cache = UsdGeom.XformCache()
        physxUiUtils.set_opposite_body_transform(self._stage, cache, jointPrim, body0base=True, fixpos=True, fixrot=True)
        with utils.ExpectMessage(self, message, False):
            await self.step(stop_timeline_after=True)

        self._stage.RemovePrim(jointPath)


joint_types = [("Fixed", UsdPhysics.FixedJoint),
               ("Revolute", UsdPhysics.RevoluteJoint),
               ("Prismatic", UsdPhysics.PrismaticJoint),
               ("Spherical", UsdPhysics.SphericalJoint),
               ("Distance", UsdPhysics.DistanceJoint),
               ("default", UsdPhysics.Joint)]


class PhysicsJointKitTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Kit

    async def test_physics_joint_error_msg(self):
        self._stage = await utils.new_stage_setup()
        physx = get_physx_interface()
        stepper = utils.PhysicsStepper()

        physicsUtils.add_rigid_box(
            self._stage,
            '/box0',
            Gf.Vec3f(100),
            Gf.Vec3f(0),
            Gf.Quatf(1.0),
            Gf.Vec3f(0),
            1000.0,
            Gf.Vec3f(0),
            Gf.Vec3f(0)
        )
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, '/World/box0')
        physicsBodyAPI.CreateRigidBodyEnabledAttr(False)

        physicsUtils.add_rigid_box(
            self._stage,
            '/box1',
            Gf.Vec3f(100),
            Gf.Vec3f(0),
            Gf.Quatf(1.0),
            Gf.Vec3f(0),
            1000.0,
            Gf.Vec3f(0),
            Gf.Vec3f(0)
        )
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, '/World/box1')
        physicsBodyAPI.CreateRigidBodyEnabledAttr(False)

        UsdPhysics.Scene.Define(self._stage, '/physicsScene')

        physicsJointPath = '/physicsJoint'
        physicsJoint = UsdPhysics.FixedJoint.Define(self._stage, physicsJointPath)

        # check error report for no body set on a joint
        message = 'PhysicsUSD: CreateJoint - no bodies defined at body0 and body1, joint prim: ' + physicsJointPath
        with utils.ExpectMessage(self, message):
            self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        # check error report for one static body set on a joint
        physx.release_physics_objects()
        body0Rel = physicsJoint.GetBody0Rel()
        body0Rel.AddTarget('/World/box0')
        message = 'PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: ' + physicsJointPath
        with utils.ExpectMessage(self, message):
            self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        # check error report for one static body set on a joint
        physx.release_physics_objects()
        usdPrim = self._stage.GetPrimAtPath(physicsJointPath)
        usdPrim.RemoveProperty('body0')
        body1Rel = physicsJoint.GetBody1Rel()
        body1Rel.AddTarget('/World/box1')
        message = 'PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: ' + physicsJointPath
        with utils.ExpectMessage(self, message):
            self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        # check error report for both static body set on a joint
        physx.release_physics_objects()
        body0Rel = physicsJoint.GetBody0Rel()
        body0Rel.AddTarget('/World/box0')
        message = 'PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: ' + physicsJointPath
        with utils.ExpectMessage(self, message):
            self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        # check no error report for static body and set on a joint that is not enabled
        physx.release_physics_objects()
        physicsJoint.CreateJointEnabledAttr().Set(False)
        self.assertTrue(await stepper.play_and_step_and_stop(1, 5))


        physicsJoint.CreateJointEnabledAttr().Set(True)
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, '/World/box0')
        physicsBodyAPI.CreateRigidBodyEnabledAttr(True)
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, '/World/box1')
        physicsBodyAPI.CreateRigidBodyEnabledAttr(True)

        # check error report for joint attaching to the same body
        physx.release_physics_objects()
        body0Rel = physicsJoint.GetBody0Rel()
        body0Rel.SetTargets(['/World/box1'])
        body1Rel = physicsJoint.GetBody1Rel()
        body1Rel.SetTargets(['/World/box1'])
        message = 'PhysicsUSD: CreateJoint - you cannot create a joint between a body and itself (both joint bodies must be unique) for joint prim: ' + physicsJointPath
        with utils.ExpectMessage(self, message):
            self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

    async def userpath_base_setup(self):
        self.fail_on_log_error = True

        stage = await utils.new_stage_setup()

        path = "/physicsScene"
        execute("AddPhysicsScene", stage=stage, path=path)

        prim_path = "/cube"
        utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=prim_path)

        utils.execute_and_check(self, "SetRigidBody", path=prim_path,
                                approximationShape="convexHull", kinematic=False)

        return stage

    async def test_physics_userpath_add_joint_to_world(self):
        for joint_type, api in joint_types:
            print(f"type: {joint_type}")
            stage = await self.userpath_base_setup()

            ret = utils.execute_and_check(self, "CreateJoints", stage=stage, joint_type=joint_type,
                                          paths=["/cube"], join_to_parent=False)

            await utils.play_and_step_and_stop(self, 1)

            self.assertTrue(len(ret) == 1)
            self.assertTrue(api.Get(stage, ret[0].GetPath()))

    async def test_physics_userpath_add_joint_to_parent(self):
        for joint_type, api in joint_types:
            print(f"type: {joint_type}")
            stage = await self.userpath_base_setup()

            prim_path = "/cube/cube_child"
            utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=prim_path)

            xformable = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
            xformable.SetResetXformStack(True)

            utils.execute_and_check(self, "SetRigidBody", path=prim_path,
                                    approximationShape="convexHull", kinematic=False)

            ret = utils.execute_and_check(self, "CreateJoints", stage=stage, joint_type=joint_type,
                                          paths=[prim_path], join_to_parent=True)

            await utils.play_and_step_and_stop(self, 1)

            self.assertTrue(len(ret) == 1)
            self.assertTrue(api.Get(stage, ret[0].GetPath()))

    async def test_physics_userpath_add_joint_between_selected(self):
        for joint_type, api in joint_types:
            print(f"type: {joint_type}")
            stage = await self.userpath_base_setup()

            prim_path = "/cube2"
            utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=prim_path)

            utils.execute_and_check(self, "SetRigidBody", path=prim_path,
                                    approximationShape="convexHull", kinematic=False)

            from_prim = stage.GetPrimAtPath("/cube")
            to_prim = stage.GetPrimAtPath("/cube2")

            ret = utils.execute_and_check(self, "CreateJoint", stage=stage, joint_type=joint_type,
                                          from_prim=from_prim, to_prim=to_prim)

            await utils.play_and_step_and_stop(self, 1)

            self.assertTrue(api.Get(stage, ret.GetPath()))

    async def test_physics_userpath_add_joint_between_scaled_selected(self):
        stage = await self.userpath_base_setup()

        prim_path = "/cube2"
        utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=prim_path)

        utils.execute_and_check(self, "SetRigidBody", path=prim_path,
                                approximationShape="convexHull", kinematic=False)

        from_prim = stage.GetPrimAtPath("/cube")
        to_prim = stage.GetPrimAtPath("/cube2")
        to_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0.0, 0.0, 90.0))
        to_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(1.0, 0.3, 1.0))

        ret = utils.execute_and_check(self, "CreateJoint", stage=stage, joint_type="prismatic",
                                        from_prim=from_prim, to_prim=to_prim)
        
        joint = UsdPhysics.PrismaticJoint(ret)
        rotation0 = joint.GetLocalRot0Attr().Get()        
        expected_rotation = Gf.Quatf(math.sqrt(2.0)/2, 0.0, 0.0, math.sqrt(2.0)/2)
        self.assertTrue(Gf.IsClose(expected_rotation.GetImaginary(), rotation0.GetImaginary(), 0.01))
        self.assertTrue(abs(expected_rotation.GetReal() - rotation0.GetReal()) < 0.01)
            
    # test for OM-26495
    async def test_physics_joint_visibility(self):
        self._stage = await self.new_stage()

        physicsJointPath = '/physicsJoint'        
        physicsJoint = UsdPhysics.FixedJoint.Define(self._stage, physicsJointPath)

        await omni.kit.app.get_app().next_update_async()

        imagable = UsdGeom.Imageable(physicsJoint.GetPrim())

        imagable.MakeInvisible()

        await omni.kit.app.get_app().next_update_async()

        imagable.MakeVisible()

        await omni.kit.app.get_app().next_update_async()

    # OM-35610
    async def test_physics_joint_instancable(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        xform = UsdGeom.Xform.Define(stage, "/xform")
        sphere = UsdGeom.Sphere.Define(stage, "/xform/sphere")
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

        xform = UsdGeom.Xform.Define(stage, "/xformInst")
        xform.GetPrim().GetReferences().AddInternalReference("/xform")
        xform.GetPrim().SetInstanceable(True)
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        ret = utils.execute_and_check(self, "CreateJoints", stage=stage, joint_type="Fixed",
                                        paths=["/xformInst"], join_to_parent=False)

        await utils.play_and_step_and_stop(self, 1)

        self.assertTrue(len(ret) == 1)        

    # test for OM-38815
    async def test_physics_joint_body_change(self):
        self._stage = await self.new_stage()

        isregistry = carb.settings.get_settings()
        joint_helpers = isregistry.get_as_bool(SETTING_DISPLAY_JOINTS)
        isregistry.set_bool(SETTING_DISPLAY_JOINTS, True)

        physicsJointPath = '/physicsJoint'        
        physicsJoint = UsdPhysics.RevoluteJoint.Define(self._stage, physicsJointPath)

        cube0 = physicsUtils.add_rigid_box(self._stage, "/Cube0", size=Gf.Vec3f(1.0), position=Gf.Vec3f(0, 0, 0))
        cube1 = physicsUtils.add_rigid_box(self._stage, "/Cube1", size=Gf.Vec3f(1.0), position=Gf.Vec3f(0, 100.0, 0))

        body0Rel = physicsJoint.GetBody0Rel()
        body0Rel.AddTarget(cube0.GetPrimPath())

        localPos0 = physicsJoint.GetLocalPos0Attr().Get()
        localPos1 = physicsJoint.GetLocalPos1Attr().Get()
        
        await omni.kit.app.get_app().next_update_async()

        epsilon = 0.001
        self.assertTrue(Gf.IsClose(localPos0, Gf.Vec3f(0.0), epsilon))
        self.assertTrue(Gf.IsClose(localPos1, Gf.Vec3f(0.0), epsilon))

        body1Rel = physicsJoint.GetBody1Rel()
        body1Rel.AddTarget(cube1.GetPrimPath())
        
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        localPos0 = physicsJoint.GetLocalPos0Attr().Get()
        localPos1 = physicsJoint.GetLocalPos1Attr().Get()

        self.assertTrue(Gf.IsClose(localPos0, Gf.Vec3f(0.0), epsilon))
        self.assertTrue(Gf.IsClose(localPos1, Gf.Vec3f(0.0, -100.0, 0.0), epsilon))

        await omni.kit.app.get_app().next_update_async()

        isregistry.set_bool(SETTING_DISPLAY_JOINTS, joint_helpers)

    # test for OM-38815 - removed target
    async def test_physics_joint_body_change_none(self):
        stage = await self.new_stage()

        physicsJointPath = '/physicsJoint'        
        physicsJoint = UsdPhysics.RevoluteJoint.Define(stage, physicsJointPath)

        cube0 = physicsUtils.add_rigid_box(stage, "/Cube0", size=Gf.Vec3f(1.0), position=Gf.Vec3f(0, -100, 0))
        cube1 = physicsUtils.add_rigid_box(stage, "/Cube1", size=Gf.Vec3f(1.0), position=Gf.Vec3f(0, 100.0, 0))

        body0Rel = physicsJoint.GetBody0Rel()
        body0Rel.AddTarget(cube0.GetPrimPath())

        body1Rel = physicsJoint.GetBody1Rel()
        body1Rel.AddTarget(cube1.GetPrimPath())

        physicsJoint.GetLocalPos0Attr().Set(Gf.Vec3f(0, 100, 0))
        physicsJoint.GetLocalPos1Attr().Set(Gf.Vec3f(0, -100, 0))

        localPos0 = physicsJoint.GetLocalPos0Attr().Get()
        localPos1 = physicsJoint.GetLocalPos1Attr().Get()

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        
        epsilon = 0.001
        self.assertTrue(Gf.IsClose(localPos0, Gf.Vec3f(0, 100, 0), epsilon))
        self.assertTrue(Gf.IsClose(localPos1, Gf.Vec3f(0, -100, 0), epsilon))
        
        body1Rel.ClearTargets(True)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
        
        localPos0 = physicsJoint.GetLocalPos0Attr().Get()
        localPos1 = physicsJoint.GetLocalPos1Attr().Get()

        self.assertTrue(Gf.IsClose(localPos0, Gf.Vec3f(0.0, 100.0, 0.0), epsilon))
        self.assertTrue(Gf.IsClose(localPos1, Gf.Vec3f(0.0, 0.0, 0.0), epsilon))

        await omni.kit.app.get_app().next_update_async()
        
    async def test_physics_prismatic_joint_parent_scale(self):
        stage = await self.new_stage()
        # We need to enable Joint Visualization because that's what implements the limit scaling from parent scale feature
        backup_joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)
        
        epsilon = 1e-5
        
        xformable = UsdGeom.Xform.Define(stage, "/World/xform")
        
        scale = Gf.Vec3f(1.0)        
        xformable.AddScaleOp().Set(scale)

        physics_joint_path = '/World/xform/physicsJoint'        
        physics_joint = UsdPhysics.PrismaticJoint.Define(stage, physics_joint_path)
        
        limit = 50.0

        physics_joint.GetLowerLimitAttr().Set(-limit)
        physics_joint.GetUpperLimitAttr().Set(limit)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
        
        lower_limit = physics_joint.GetLowerLimitAttr().Get()
        upper_limit = physics_joint.GetUpperLimitAttr().Get()

        self.assertTrue(abs(lower_limit + limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)
        
        scale = Gf.Vec3f(2.0)        
        xformable.GetPrim().GetAttribute("xformOp:scale").Set(scale)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
            
        lower_limit = physics_joint.GetLowerLimitAttr().Get()
        upper_limit = physics_joint.GetUpperLimitAttr().Get()

        self.assertTrue(abs(lower_limit + 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, backup_joint_vis)

    async def test_physics_distance_joint_parent_scale(self):
        stage = await self.new_stage()
        # We need to enable Joint Visualization because that's what implements the limit scaling from parent scale feature
        backup_joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)
        
        epsilon = 1e-5
        
        xformable = UsdGeom.Xform.Define(stage, "/World/xform")
        
        scale = Gf.Vec3f(1.0)        
        xformable.AddScaleOp().Set(scale)

        physics_joint_path = '/World/xform/physicsJoint'        
        physics_joint = UsdPhysics.DistanceJoint.Define(stage, physics_joint_path)
        
        limit = 50.0

        physics_joint.GetMinDistanceAttr().Set(limit)
        physics_joint.GetMaxDistanceAttr().Set(limit)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
        
        lower_limit = physics_joint.GetMinDistanceAttr().Get()
        upper_limit = physics_joint.GetMaxDistanceAttr().Get()

        self.assertTrue(abs(lower_limit - limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)
        
        scale = Gf.Vec3f(2.0)        
        xformable.GetPrim().GetAttribute("xformOp:scale").Set(scale)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
            
        lower_limit = physics_joint.GetMinDistanceAttr().Get()
        upper_limit = physics_joint.GetMaxDistanceAttr().Get()
        
        print(lower_limit)
        print(upper_limit)

        self.assertTrue(abs(lower_limit - 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, backup_joint_vis)
        
    async def test_physics_rack_pinion_joint_parent_scale(self):
        stage = await self.new_stage()
        # We need to enable Joint Visualization because that's what implements the limit scaling from parent scale feature
        backup_joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)
        
        epsilon = 1e-5
        
        xformable = UsdGeom.Xform.Define(stage, "/World/xform")
        
        scale = Gf.Vec3f(1.0)        
        xformable.AddScaleOp().Set(scale)

        physics_joint_path = '/World/xform/physicsJoint'        
        physics_joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, physics_joint_path)
        
        limit = 50.0

        physics_joint.GetRatioAttr().Set(limit)        

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
        
        ratio = physics_joint.GetRatioAttr().Get()        

        self.assertTrue(abs(ratio - limit) < epsilon)        
        
        scale = Gf.Vec3f(2.0)        
        xformable.GetPrim().GetAttribute("xformOp:scale").Set(scale)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
            
        ratio = physics_joint.GetRatioAttr().Get()        

        self.assertTrue(abs(ratio - limit / 2.0) < epsilon)        
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, backup_joint_vis)

    async def test_physics_d6_joint_parent_scale(self):
        stage = await self.new_stage()
        # We need to enable Joint Visualization because that's what implements the limit scaling from parent scale feature
        backup_joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)
        
        epsilon = 1e-5
        
        xformable = UsdGeom.Xform.Define(stage, "/World/xform")
        
        scale = Gf.Vec3f(1.0)        
        xformable.AddScaleOp().Set(scale)

        physics_joint_path = '/World/xform/physicsJoint'        
        physics_joint = UsdPhysics.Joint.Define(stage, physics_joint_path)
        
        limit = 50.0
        
        x_limit_api = UsdPhysics.LimitAPI.Apply(physics_joint.GetPrim(), UsdPhysics.Tokens.transX)
        y_limit_api = UsdPhysics.LimitAPI.Apply(physics_joint.GetPrim(), UsdPhysics.Tokens.transY)
        z_limit_api = UsdPhysics.LimitAPI.Apply(physics_joint.GetPrim(), UsdPhysics.Tokens.transZ)
        distance_limit_api = UsdPhysics.LimitAPI.Apply(physics_joint.GetPrim(), UsdPhysics.Tokens.distance)

        x_limit_api.GetLowAttr().Set(-limit)
        x_limit_api.GetHighAttr().Set(limit)

        y_limit_api.GetLowAttr().Set(-limit)
        y_limit_api.GetHighAttr().Set(limit)

        z_limit_api.GetLowAttr().Set(-limit)
        z_limit_api.GetHighAttr().Set(limit)

        distance_limit_api.GetLowAttr().Set(limit)
        distance_limit_api.GetHighAttr().Set(limit)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
        
        lower_limit = x_limit_api.GetLowAttr().Get()
        upper_limit = x_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)

        lower_limit = y_limit_api.GetLowAttr().Get()
        upper_limit = y_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)

        lower_limit = z_limit_api.GetLowAttr().Get()
        upper_limit = z_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)
        
        lower_limit = distance_limit_api.GetLowAttr().Get()
        upper_limit = distance_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit - limit) < epsilon)
        self.assertTrue(abs(upper_limit - limit) < epsilon)
        
        scale = Gf.Vec3f(2.0)        
        xformable.GetPrim().GetAttribute("xformOp:scale").Set(scale)

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
            
        lower_limit = x_limit_api.GetLowAttr().Get()
        upper_limit = x_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)

        lower_limit = y_limit_api.GetLowAttr().Get()
        upper_limit = y_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)

        lower_limit = z_limit_api.GetLowAttr().Get()
        upper_limit = z_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit + 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)
        
        lower_limit = distance_limit_api.GetLowAttr().Get()
        upper_limit = distance_limit_api.GetHighAttr().Get()

        self.assertTrue(abs(lower_limit - 2.0 * limit) < epsilon)
        self.assertTrue(abs(upper_limit - 2.0 * limit) < epsilon)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, backup_joint_vis)

    async def test_physics_joint_update_localpos01_after_body01(self): # OM-96674
        stage = await self.new_stage()
        body0 = UsdGeom.Xform.Define(stage, "/World/Body0")
        body0.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
        body1 = UsdGeom.Xform.Define(stage, "/World/Body1")
        body1.AddTranslateOp().Set(Gf.Vec3d(4, 5, 6))
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint")
        joint.GetBody0Rel().AddTarget("/World/Body0")
        joint.GetBody1Rel().AddTarget("/World/Body1")
        joint.GetLocalPos0Attr().Set(Gf.Vec3d(0, 0, 0))
        joint.GetLocalPos1Attr().Set(Gf.Vec3d(0, 0, 0))
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            get_physxui_interface().update()
                        
        localPos0 = joint.GetLocalPos0Attr().Get()
        localPos1 = joint.GetLocalPos1Attr().Get()
        
        epsilon = 0.001

        self.assertTrue(Gf.IsClose(localPos0, Gf.Vec3f(0.0), epsilon))
        self.assertTrue(Gf.IsClose(localPos1, Gf.Vec3f(0.0), epsilon))
