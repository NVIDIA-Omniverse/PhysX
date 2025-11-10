# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
import omni.physx
import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.physx.scripts.physicsUtils as physicsUtils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdShade, UsdPhysics, UsdUtils, PhysxSchema, PhysicsSchemaTools
import unittest

class OmniGraphPhysxTriggersTest(ogts.OmniGraphTestCase):

    async def setUp(self):
        await super().setUp()
        self._stage = omni.usd.get_context().get_stage()

    async def tearDown(self):
        await super().tearDown()

    def setup_stage(self):
        stage = self._stage
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")        
        self._boxActorPath0 = "/boxActor0"
        size = Gf.Vec3f(1.0)
        position = Gf.Vec3f(200.0)
        orientation = Gf.Quatf(1.0)
        self._boxActor0 = physicsUtils.add_rigid_box(stage, self._boxActorPath0, size, position, orientation)

        self._boxActorPath1 = "/boxActor1"
        size = Gf.Vec3f(1.0)
        position = Gf.Vec3f(-200.0)
        orientation = Gf.Quatf(1.0)
        self._rigidBox1 = physicsUtils.add_rigid_box(stage, self._boxActorPath1, size, position, orientation)

        self._triggerBoxPath = "/boxTrigger"
        size = Gf.Vec3f(100.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        boxPrim = physicsUtils.add_box(stage, self._triggerBoxPath, size, position, orientation)
        UsdPhysics.CollisionAPI.Apply(boxPrim)
        PhysxSchema.PhysxTriggerAPI.Apply(boxPrim)
        self._triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(boxPrim)

    async def _timeline_play_steps(self, num_frames):

        omni.timeline.get_timeline_interface().play()
        for _ in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

    def _assert_collider_outputs(self, ogController, test_node, expect_trigger, expect_other):
        outputs_leave_triggerCollider = og.Controller(ogController.attribute("outputs:triggerCollider", test_node)).get()
        self.assertEqual(outputs_leave_triggerCollider, expect_trigger)
        outputs_leave_otherCollider = og.Controller(ogController.attribute("outputs:otherCollider", test_node)).get()
        self.assertEqual(outputs_leave_otherCollider, expect_other)
       
    async def _test_parametric(self, triggerBoxPath, boxActorPath0, listenToPaths, listenToAllTriggers):
                
        og.Controller.edit({
                "evaluator_name": "execution",
                "graph_path": "/TestGraph",
            })

        # Construct the Graph to listen to all trigger events (by not specifying any input)
        ogController = og.Controller()
        keys = ogController.Keys
        (graph, test_nodes, _, _) = ogController.edit("/TestGraph",
            {
                keys.CREATE_NODES: [
                    ("on_collider_enter_trigger", "omni.physx.graph.OnTriggerCollider"),
                    ("on_collider_leave_trigger", "omni.physx.graph.OnTriggerCollider"),
                    ("write_prim_attribute", "omni.graph.nodes.WritePrimAttribute"),
                ],
                # Connect to a random node with no side effects just to get the type of events we need
                keys.CONNECT: [
                    ("on_collider_enter_trigger.outputs:enterExecOut", "write_prim_attribute.inputs:execIn"),
                    ("on_collider_leave_trigger.outputs:leaveExecOut", "write_prim_attribute.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("on_collider_enter_trigger.inputs:triggersPaths", listenToPaths),
                    ("on_collider_leave_trigger.inputs:triggersPaths", listenToPaths),
                    ("on_collider_enter_trigger.inputs:listenToAllTriggers", listenToAllTriggers),
                    ("on_collider_leave_trigger.inputs:listenToAllTriggers", listenToAllTriggers),
                ]
            }
        )

        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)
        # The cube starts outside of the volume of the trigger, so it will not cause any trigger notification

        # Check Enter to be empty
        self._assert_collider_outputs(ogController, test_nodes[0], "", "")

        # Check Leave to be empty
        self._assert_collider_outputs(ogController, test_nodes[1], "", "")

        # Manually move the cube inside the trigger volume
        self._boxActor0.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))

        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)

        # Check Enter event now to be received
        self._assert_collider_outputs(ogController, test_nodes[0], triggerBoxPath, boxActorPath0)
        # Check Leave event NOT to be received
        self._assert_collider_outputs(ogController, test_nodes[1], "", "")

        # Manually move the cube outside of the trigger volume
        self._boxActor0.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)

        # Check Leave event to be generated
        self._assert_collider_outputs(ogController, test_nodes[1], triggerBoxPath, boxActorPath0)

    async def test_trigger_enter_leave_nodes_none(self):
        # Creates a scene with a trigger box volume and two rigid boxes that are outside of such volume 
        # Do not listen to any trigger path, so that no notifications will be reported
        self.setup_stage()
        await self._test_parametric("", "", [], listenToAllTriggers=False)

    async def test_trigger_enter_leave_nodes_catch_all(self):
        # Creates a scene with a trigger box volume and two rigid boxes that are outside of such volume 
        # Do not listen to any trigger path, but set the listenToAllTriggers option == True, to get notified
        self.setup_stage()
        await self._test_parametric(self._triggerBoxPath, self._boxActorPath0, [], listenToAllTriggers=True)

    async def test_trigger_enter_leave_nodes_single(self):
        # Creates a scene with a trigger box volume and two rigid boxes that are outside of such volume 
        # Listen to the specific trigger path that will emit events
        self.setup_stage()
        await self._test_parametric(self._triggerBoxPath, self._boxActorPath0, [self._triggerBoxPath], listenToAllTriggers=False)
