# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, Tf, UsdPhysics, PhysicsSchemaTools
import omni.physxdemos as demo
from omni.physx.scripts.physicsUtils import *
from omni.physx import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent


class JointBreakDemo(demo.Base):
    title = "Joint Break"
    category = demo.Categories.JOINTS
    short_description = "Demo showing joint break event report"
    description = "Demo showing joint break event report."

    def create(self, stage):
        # setup physics simulation event stream
        events = get_physx_interface().get_simulation_event_stream_v2()
        self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)
        self._stage = stage

        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        self._room = demo.get_demo_room(self, stage, hasTable = False, floorOffset = 200.0)
        
        # Box0 static
        size = Gf.Vec3f(50.0)
        position = Gf.Vec3f(200.0, 0.0, 300.0)
        color = demo.get_static_color()

        box0Prim = add_rigid_box(stage, defaultPrimPath + "/boxActor0", size, position, color=color)
        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(stage, box0Prim.GetPrimPath())
        physicsBodyAPI.GetRigidBodyEnabledAttr().Set(False)

        # Box1 dynamic        
        position = Gf.Vec3f(0.0, 0.0, 300.0)
        color = demo.get_static_color()

        self._box1Prim = add_rigid_box(stage, defaultPrimPath + "/boxActor1", size, position, color=color)

        # Box2 dynamic        
        position = Gf.Vec3f(0.0, 0.0, 500.0)
        color = demo.get_primary_color()

        add_rigid_box(stage, defaultPrimPath + "/boxActor2", size, position, color=color)

        # fixed joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, defaultPrimPath + "/fixedJoint")

        fixedJoint.CreateBody0Rel().SetTargets([box0Prim.GetPrimPath()])
        fixedJoint.CreateBody1Rel().SetTargets([self._box1Prim.GetPrimPath()])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(-2.0, 0.0, 0.0))
        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(2.0, 0.0, 0.0))

        fixedJoint.CreateBreakForceAttr().Set(1000000000.0)

    def on_shutdown(self):
        self._simulation_event_sub = None

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            jointPath = PhysicsSchemaTools.decodeSdfPath(event.payload['jointPath'][0], event.payload['jointPath'][1])
            print("Joint break event, joint " + str(jointPath) + " did break.")
            usdGeom = UsdGeom.Mesh(self._box1Prim)            
            breakColor = Vt.Vec3fArray([demo.get_primary_color()])
            usdGeom.GetDisplayColorAttr().Set(breakColor)
        if event.type == int(SimulationEvent.STOPPED):
            usdGeom = UsdGeom.Mesh(self._box1Prim)            
            origColor = Vt.Vec3fArray([demo.get_static_color()])
            usdGeom.GetDisplayColorAttr().Set(origColor)
