# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import os
import omni.ui as ui
import carb.settings
from pxr import Usd, UsdUtils, UsdGeom, UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools
from omni.physxui.scripts.physxDebugView import remove_schema
import omni.kit.ui_test as ui_test
from omni.physxtests import utils
from omni.physx import get_physx_simulation_interface, get_physx_visualization_interface


# Tests for divergent core tests using referenced USD file
class DebugViewTests(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    async def tearDown(self):
        pass

    async def new_stage(self):
        # Workaround for Kit event loop issues
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.stage_templates.new_stage_async()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        return stage
    
    def setup_stage(self, stage, scenario = "simple cube"):
        self._reference_stage = None
        if scenario == "simple cube":
            cube_prim = UsdGeom.Cube.Define(stage, "/World/cube").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(cube_prim)
            UsdPhysics.CollisionAPI.Apply(cube_prim)
            PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
        elif scenario == "reference cube" or scenario == "reference multiple cube":
            ref_stage = Usd.Stage.CreateInMemory()
            cache = UsdUtils.StageCache.Get()
            cache.Insert(ref_stage)

            cube_prim = UsdGeom.Cube.Define(ref_stage, "/World/cube").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(cube_prim)
            UsdPhysics.CollisionAPI.Apply(cube_prim)
            PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
            if scenario == "reference multiple cube":
                PhysxSchema.PhysxCookedDataAPI.Apply(cube_prim, "convexHull")
            
            world_prim = stage.GetPrimAtPath("/World")
            world_prim.GetReferences().AddReference(ref_stage.GetRootLayer().identifier, "/World")
            
            cube_prim = stage.GetPrimAtPath("/World/cube")
            
            self._reference_stage = ref_stage
            
        return cube_prim
    
    def clear_stage(self):
        if self._reference_stage:
            cache = UsdUtils.StageCache.Get()
            cache.Erase(self._reference_stage)
            self._reference_stage = None

    async def test_remove_schema_simple_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()
        
        self.assertTrue(apis == expected_apis)

        omni.kit.commands.execute("ClearPhysicsComponentsCommand", stage=stage, prim_paths=["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        self.assertTrue(len(apis) == 0)

        await self.new_stage()
        
    async def test_remove_schema_reference_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage, scenario = "reference cube")
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()
        
        self.assertTrue(apis == expected_apis)

        omni.kit.commands.execute("ClearPhysicsComponentsCommand", stage=stage, prim_paths=["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        print(apis)
        self.assertTrue(len(apis) == 0)

        await self.new_stage()
        self.clear_stage()
        
    async def test_remove_schema_multiple_applied_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        PhysxSchema.PhysxCookedDataAPI.Apply(cube_prim, "convexHull")
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI', 'PhysxCookedDataAPI:convexHull']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()
        
        self.assertTrue(apis == expected_apis)

        omni.kit.commands.execute("ClearPhysicsComponentsCommand", stage=stage, prim_paths=["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        self.assertTrue(len(apis) == 0)

        await self.new_stage()
        
    async def test_remove_physx_schema_simple_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)        
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()        
        self.assertTrue(apis == expected_apis)

        remove_schema(True, False, stage, ["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        
        self.assertTrue(len(apis) == 2)

        await self.new_stage()
        
    async def test_remove_physx_schema_reference_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage, scenario = "reference cube")      
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()        
        self.assertTrue(apis == expected_apis)

        remove_schema(True, False, stage, ["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        
        self.assertTrue(len(apis) == 2)

        await self.new_stage()
        self.clear_stage()
        
    async def test_remove_physx_schema_reference_multiple_direct_call(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage, scenario = "reference multiple cube")      
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI', 'PhysxCookedDataAPI:convexHull']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()        
        self.assertTrue(apis == expected_apis)

        remove_schema(True, False, stage, ["/World/cube"])
        
        apis = cube_prim.GetAppliedSchemas()
        
        self.assertTrue(len(apis) == 2)

        await self.new_stage()
        self.clear_stage()
        
    async def test_remove_schema_simple_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()
        
        self.assertTrue(apis == expected_apis)

        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        omni.usd.get_context().get_selection().set_selected_prim_paths(["/World/cube"], True)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Physics schema removal'")
        target_scroll = cf.widget.screen_position_y

        scroll_frame.widget.scroll_y = target_scroll
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        remove_button = cf.find("**/Button[*].text=='Remove PhysicsSchema and PhysxSchema on selection'")
        await remove_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        apis = cube_prim.GetAppliedSchemas()
        self.assertTrue(len(apis) == 0)

        await self.new_stage()
        
    async def test_remove_physx_schema_simple_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)        
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        expected_apis = ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysxRigidBodyAPI']
        expected_apis.sort()
        
        apis = cube_prim.GetAppliedSchemas()
        apis.sort()        
        self.assertTrue(apis == expected_apis)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        omni.usd.get_context().get_selection().set_selected_prim_paths(["/World/cube"], True)

        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Physics schema removal'")
        target_scroll = cf.widget.screen_position_y
        
        scroll_frame.widget.scroll_y = target_scroll

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        remove_button = cf.find("**/Button[*].text=='Remove PhysxSchema on selection'")
        await remove_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        apis = cube_prim.GetAppliedSchemas()
        
        self.assertTrue(len(apis) == 2)

        await self.new_stage()
        
    # tests play, pause, play, stop
    async def test_play_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        epsilon = 0.02
        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(init_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))
        
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Control'")

        # run
        run_button = cf.find("**/Button[*].text=='Run'")
        await run_button.click()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(omni.timeline.get_timeline_interface().is_playing())
        
        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(current_pose.ExtractTranslation()[1] < -epsilon)
        
        # pause
        await run_button.click()
        self.assertTrue(not omni.timeline.get_timeline_interface().is_playing())

        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(current_pose.ExtractTranslation(), init_pose.ExtractTranslation(), epsilon))

        # run again
        await run_button.click()        

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(omni.timeline.get_timeline_interface().is_playing())
        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(current_pose.ExtractTranslation()[1] < init_pose.ExtractTranslation()[1])

        stop_button = cf.find("**/Button[*].text=='Stop'")
        await stop_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        self.assertTrue(omni.timeline.get_timeline_interface().is_stopped())

        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(current_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))

        await self.new_stage()
        
    # tests step, play, stop
    async def test_step_play_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        epsilon = 0.02
        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(init_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))
        
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Control'")

        # Step
        step_button = cf.find("**/Button[*].text=='Step'")        

        for _ in range(10):
            await step_button.click()
            await omni.kit.app.get_app().next_update_async()            
        
        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(current_pose.ExtractTranslation()[1] < -epsilon)
        
        # Play
        run_button = cf.find("**/Button[*].text=='Run'")
        await run_button.click()
        self.assertTrue(omni.timeline.get_timeline_interface().is_playing())

        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(current_pose.ExtractTranslation()[1] < init_pose.ExtractTranslation()[1])

        stop_button = cf.find("**/Button[*].text=='Stop'")
        await stop_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        self.assertTrue(omni.timeline.get_timeline_interface().is_stopped())

        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(current_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))

        await self.new_stage()
        
    # tests step, stop
    async def test_step_stop_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        epsilon = 0.02
        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(init_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))
        
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Control'")

        # Step
        step_button = cf.find("**/Button[*].text=='Step'")        

        for _ in range(10):
            await step_button.click()
            await omni.kit.app.get_app().next_update_async()            
        
        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(current_pose.ExtractTranslation()[1] < -epsilon)
        
        stop_button = cf.find("**/Button[*].text=='Stop'")
        await stop_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        self.assertTrue(omni.timeline.get_timeline_interface().is_stopped())

        current_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(current_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))

        await self.new_stage()
        
    # Force load
    async def test_force_load_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
                
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        epsilon = 0.02
        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(init_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))
        
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Overrides'")
            
        utils.check_stats(self, { "numBoxShapes": 0,  "numDynamicRigids": 0 })

        # Force load
        force_load_button = cf.find("**/Button[*].text=='Force Physics USD Load'")
        await force_load_button.click()

        utils.check_stats(self, { "numBoxShapes": 1,  "numDynamicRigids": 1 })
        
        # Release
        release_button = cf.find("**/Button[*].text=='Release Physics Objects'")
        await release_button.click()
        
        utils.check_stats(self, { "numBoxShapes": 0,  "numDynamicRigids": 0 })

        await self.new_stage()
        
    # Detach/attach to stage update
    async def test_detach_attach_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        epsilon = 0.02
        init_pose = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(init_pose.ExtractTranslation(), Gf.Vec3d(0), epsilon))
        
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Overrides'")
                    
        utils.check_stats(self, { "numBoxShapes": 0,  "numDynamicRigids": 0 })

        # Detach
        detach_button = cf.find("**/Button[*].text=='Detach OmniPhysX StageUpdateNode'")
        await detach_button.click()

        # play -- no physics
        omni.timeline.get_timeline_interface().play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        utils.check_stats(self, { "numBoxShapes": 0,  "numDynamicRigids": 0 })
        
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        # attach back
        await detach_button.click()        
        omni.timeline.get_timeline_interface().play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        utils.check_stats(self, { "numBoxShapes": 1,  "numDynamicRigids": 1 })

        omni.timeline.get_timeline_interface().stop()
        
        await self.new_stage()
        
    # Disable global sleeping
    async def test_disable_sleeping_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        physx_schema = PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
        physx_schema.GetDisableGravityAttr().Set(True)

        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Overrides'")
                    
        # play check sleeping
        omni.timeline.get_timeline_interface().play()
        for _ in range(50):
            await omni.kit.app.get_app().next_update_async()
            
        cube_encoded = PhysicsSchemaTools.sdfPathToInt(cube_prim.GetPrimPath())
                
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, cube_encoded)
        self.assertTrue(is_sleeping)
        
        omni.timeline.get_timeline_interface().stop()

        # disable sleeping play again
        checkboxLabel = cf.find("**/Label[0].text=='Disable Sleeping'")
        await ui_test.emulate_mouse_move_and_click(checkboxLabel.position + ui_test.Vec2(200, 5))
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        omni.timeline.get_timeline_interface().play()
        for _ in range(50):
            await omni.kit.app.get_app().next_update_async()
            
        cube_encoded = PhysicsSchemaTools.sdfPathToInt(cube_prim.GetPrimPath())
                
        is_sleeping = get_physx_simulation_interface().is_sleeping(stage_id, cube_encoded)
        self.assertTrue(not is_sleeping)
        
        omni.timeline.get_timeline_interface().stop()
        
        # set it back
        await ui_test.emulate_mouse_move_and_click(checkboxLabel.position + ui_test.Vec2(200, 5))
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        await self.new_stage()
        
    # PhysX Debug vis test
    async def test_physx_debug_vis_debug_window(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        
        orig_setting = carb.settings.get_settings().get_as_float("/persistent/app/viewport/gizmo/scale")
        carb.settings.get_settings().set_float("/persistent/app/viewport/gizmo/scale", 10.0)
        
        cube_prim = self.setup_stage(stage)
        xformable = UsdGeom.Xformable(cube_prim)
        physx_schema = PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
        physx_schema.GetDisableGravityAttr().Set(True)

        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Simulation Debug Visualization'")
        # target_scroll = cf.widget.screen_position_y
        # A.B. this is not really working had to hard code here the 350
        scroll_frame.widget.scroll_y = 350
       
        # play 
        omni.timeline.get_timeline_interface().play()
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        # no debug vis
        self.assertTrue(get_physx_visualization_interface().get_nb_lines() == 0)

        enabledboxLabel = cf.find("**/Label[0].text=='Enabled'")
        await ui_test.emulate_mouse_move_and_click(enabledboxLabel.position + ui_test.Vec2(200, 5))
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
            
        self.assertTrue(get_physx_visualization_interface().get_nb_lines() == 0)

        collisionboxLabel = cf.find("**/Label[0].text=='Collision Shapes'")
        await ui_test.emulate_mouse_move_and_click(collisionboxLabel.position + ui_test.Vec2(-20, 5))

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        self.assertTrue(get_physx_visualization_interface().get_nb_lines() == 12)
        
        omni.timeline.get_timeline_interface().stop()
        
        # set it back
        await ui_test.emulate_mouse_move_and_click(enabledboxLabel.position + ui_test.Vec2(200, 5))
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        carb.settings.get_settings().set_float("/persistent/app/viewport/gizmo/scale", orig_setting)
            
        await self.new_stage()
        
    # Mass information direct
    async def test_physx_debug_mass_information_direct(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
                
        cube_prim = self.setup_stage(stage)

        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Mass Information'")
       
        omni.usd.get_context().get_selection().set_selected_prim_paths(["/World/cube"], True)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        mass_info_button = cf.find("**/Button[*].text=='Get mass information for selected prims and its sub-hierarchies'")
        await mass_info_button.click()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            
        await self.new_stage()
        
    # Mass information query
    async def test_physx_debug_mass_information_query(self):
        await self.new_stage()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
                
        cube_prim = self.setup_stage(stage)

        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                
        scroll_frame = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]")        
        scroll_frame.widget.scroll_y = 0

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        cf = ui_test.find("Physics Debug//Frame/**/CollapsableFrame[*].title=='Mass Information'")
       
        omni.usd.get_context().get_selection().set_selected_prim_paths(["/World/cube"], True)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
                                
        mass_info_button = cf.find("**/Button[*].text=='Query rigid body'")
        await mass_info_button.click()
            
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()
            
        await self.new_stage()
