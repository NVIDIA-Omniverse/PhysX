# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxcommands import AddGroundPlaneCommand
import carb
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory, PhysicsKitStageAsyncTestCase
from omni.physx import get_physx_interface, get_physx_scene_query_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import TriggerEventData, TriggerEventType
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdShade, UsdPhysics, UsdUtils, PhysxSchema, PhysicsSchemaTools
import math

class PhysxTriggerStateAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def setup_stage(self, stage, scene="Simple"):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")
        
        if scene == "Simple":
            self._boxActorPath = "/boxActor0"
            size = Gf.Vec3f(1.0)
            position = Gf.Vec3f(200.0)
            orientation = Gf.Quatf(1.0)
            self._rigidBox = physicsUtils.add_rigid_box(stage, self._boxActorPath, size, position, orientation)
            
        if scene == "ConvexDecomposition" or scene == "ConvexDecompositionAll":
            self._boxActorPath = "/boxActor0"
            mesh_path = "/boxActor0"
            concaveGeom = physicsUtils.create_mesh_concave(stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(200.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
            self._rigidBox = concaveGeom.GetPrim()

        if scene == "ConvexDecompositionAll":
            self._triggerBoxPath = "/boxActor1"
            mesh_path = "/boxActor1"
            concaveGeom = physicsUtils.create_mesh_concave(stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0))
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
            PhysxSchema.PhysxTriggerAPI.Apply(concaveGeom.GetPrim())
            self._triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(concaveGeom.GetPrim())
        else:
            self._triggerBoxPath = "/boxActor1"
            size = Gf.Vec3f(100.0)
            position = Gf.Vec3f(0.0)
            orientation = Gf.Quatf(1.0)
            boxPrim = physicsUtils.add_box(stage, self._triggerBoxPath, size, position, orientation)
            UsdPhysics.CollisionAPI.Apply(boxPrim)
            PhysxSchema.PhysxTriggerAPI.Apply(boxPrim)
            self._triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(boxPrim)


    async def test_physics_trigger_enter_leave(self):
        stage = await self.new_stage()
        self.setup_stage(stage)
        
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 1)
        self.assertTrue(triggerColliders[0] == Sdf.Path(self._boxActorPath))
        
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)        
        
    async def test_physics_trigger_collision_filter(self):
        stage = await self.new_stage()
        self.setup_stage(stage)
        
        # filter the two colliders
        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(self._rigidBox)
        rel = filteringPairsAPI.CreateFilteredPairsRel()
        rel.AddTarget(Sdf.Path(self._triggerBoxPath))
                
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)
        
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)        
        
    async def test_physics_trigger_collision_groups(self):
        stage = await self.new_stage()
        self.setup_stage(stage)
        
        # create collision group, filter with self and add both shapes to it
        collisionGroup = UsdPhysics.CollisionGroup.Define(stage, "/collisionGroup")
        filteredRel = collisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(Sdf.Path("/collisionGroup"))

        collectionAPI = Usd.CollectionAPI.Apply(collisionGroup.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(Sdf.Path(self._boxActorPath))
        collectionAPI.CreateIncludesRel().AddTarget(Sdf.Path(self._triggerBoxPath))
                        
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)
        
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)  
        
    async def test_physics_trigger_convex_decomposition_enter_leave(self):
        stage = await self.new_stage()
        self.setup_stage(stage, "ConvexDecomposition")
        
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 1)
        self.assertTrue(triggerColliders[0] == Sdf.Path(self._boxActorPath))
        
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)   
        
    async def test_physics_trigger_convex_decomposition_both_enter_leave(self):
        stage = await self.new_stage()
        self.setup_stage(stage, "ConvexDecompositionAll")
        
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 1)
        self.assertTrue(triggerColliders[0] == Sdf.Path(self._boxActorPath))
        
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(200.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)
        
    async def test_physics_trigger_api_clear(self):
        stage = await self.new_stage()
        self.setup_stage(stage)
        
        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()        
        self.assertTrue(len(triggerColliders) == 0)

        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        
        self.step()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 1)
        self.assertTrue(triggerColliders[0] == Sdf.Path(self._boxActorPath))

        get_physx_interface().release_physics_objects()

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertTrue(len(triggerColliders) == 0)

    async def test_delete_trigger_with_box_in_trigger_during_runtime(self):
        stage = await self.new_stage()

        trigger = physicsUtils.add_box(stage, "/trigger", Gf.Vec3f(100.0))
        UsdPhysics.CollisionAPI.Apply(trigger)
        PhysxSchema.PhysxTriggerAPI.Apply(trigger)
        self._triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger)

        physicsUtils.add_rigid_box(stage, "/boxActor0")

        self.step(10)

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertEqual(len(triggerColliders), 1)

        stage.RemovePrim("/trigger")
        self.assertFalse(trigger.IsValid())
        
        self.step(10)

    async def test_delete_box_in_trigger_during_runtime(self):
        stage = await self.new_stage()

        trigger = physicsUtils.add_box(stage, "/trigger", Gf.Vec3f(100.0))
        UsdPhysics.CollisionAPI.Apply(trigger)
        PhysxSchema.PhysxTriggerAPI.Apply(trigger)
        self._triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger)

        box = physicsUtils.add_rigid_box(stage, "/boxActor0")

        self.step(10)

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertEqual(len(triggerColliders), 1)

        stage.RemovePrim("/boxActor0")
        self.assertFalse(box.IsValid())

        self.step(10)

        triggerColliders = self._triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        self.assertEqual(len(triggerColliders), 0)

    async def test_physics_trigger_enter_leave_script(self):
        # Creates a scene with a trigger, listens to specific path and stage and check if the enter/leave events actually happen
        stage = await self.new_stage()
        self.setup_stage(stage)

        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
        prim_id = PhysicsSchemaTools.sdfPathToInt(self._triggerBoxPath)
        simulateI = get_physx_simulation_interface()
        self.got_called_enter = False
        self.got_called_exit = False 

        def report_trigger(trigger_data : TriggerEventData):
            self.assertEqual(trigger_data.stage_id, self.stage_id)
            self.assertEqual(trigger_data.subscription_id, self._subid)
            trigger_collider_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.trigger_collider_prim_id))
            self.assertEqual(trigger_collider_prim_path, self._triggerBoxPath)
            other_collider_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.other_collider_prim_id))
            self.assertEqual(other_collider_prim_path, self._boxActorPath)
            trigger_body_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.trigger_body_prim_id))
            self.assertEqual(trigger_body_prim_path, self._triggerBoxPath)
            other_body_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.other_body_prim_id))
            self.assertEqual(other_body_prim_path, self._boxActorPath)
            if trigger_data.event_type == TriggerEventType.TRIGGER_ON_ENTER:
                self.got_called_enter = True
                self.assertEqual(self.got_called_exit, False) # check for exit not to be already called
            if trigger_data.event_type == TriggerEventType.TRIGGER_ON_LEAVE:
                self.got_called_exit = True
                self.assertEqual(self.got_called_enter, True) # check for enter to be set before exit
            
        self._subid = simulateI.subscribe_physics_trigger_report_events(trigger_report_fn = report_trigger, stage_id = self.stage_id, prim_id = prim_id)
        self.step()
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        self.step()
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        self.step()
        simulateI.unsubscribe_physics_trigger_report_events(self._subid)
        self.assertTrue(self.got_called_enter)
        self.assertTrue(self.got_called_exit)

    async def test_physics_trigger_enter_leave_script_all(self):
        # Same as previous test but without specifiying a path / stage in subscribe_physics_trigger_report_events
        # that signals the trigger system to report all triggers no matter what
        stage = await self.new_stage()
        self.setup_stage(stage)

        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
        prim_id = PhysicsSchemaTools.sdfPathToInt(self._triggerBoxPath)
        simulateI = get_physx_simulation_interface()
        self.got_called_enter = False
        self.got_called_exit = False 

        def report_trigger(trigger_data : TriggerEventData):
            self.assertEqual(trigger_data.stage_id, self.stage_id)
            self.assertEqual(trigger_data.subscription_id, self._subid)
            trigger_collider_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.trigger_collider_prim_id))
            self.assertEqual(trigger_collider_prim_path, self._triggerBoxPath)
            other_collider_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.other_collider_prim_id))
            self.assertEqual(other_collider_prim_path, self._boxActorPath)
            trigger_body_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.trigger_body_prim_id))
            self.assertEqual(trigger_body_prim_path, self._triggerBoxPath)
            other_body_prim_path = str(PhysicsSchemaTools.intToSdfPath(trigger_data.other_body_prim_id))
            self.assertEqual(other_body_prim_path, self._boxActorPath)            
            if trigger_data.event_type == TriggerEventType.TRIGGER_ON_ENTER:
                self.got_called_enter = True
                self.assertEqual(self.got_called_exit, False) # check for exit not to be already called
            if trigger_data.event_type == TriggerEventType.TRIGGER_ON_LEAVE:
                self.got_called_exit = True
                self.assertEqual(self.got_called_enter, True) # check for enter to be set before exit
            
        self._subid = simulateI.subscribe_physics_trigger_report_events(trigger_report_fn = report_trigger)
        self.step()
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0))
        self.step()
        self._rigidBox.GetAttribute("xformOp:translate").Set(Gf.Vec3f(300.0))
        self.step()
        simulateI.unsubscribe_physics_trigger_report_events(self._subid)
        self.assertTrue(self.got_called_enter)
        self.assertTrue(self.got_called_exit)
