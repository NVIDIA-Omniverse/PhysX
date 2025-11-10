# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.ui_test as ui_test
import omni.ui as ui
import omni.usd
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physxtestsvisual.utils import TestCase
from omni.physx.scripts import physicsUtils
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade


class PhysxDebugTest(TestCase):
    category = TestCategory.Core

    async def _select_and_focus_debug_window(self):
        window = ui.Workspace.get_window("Physics Debug")
        window.focus()
        await self.wait(20)

    async def test_physics_ui_debug_step(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))

        await self._select_and_focus_debug_window()

        position = prim.GetAttribute("xformOp:translate").Get()        
        self.assertTrue(Gf.IsClose(position[1], 0.0, 1e-3))
        
        velocity = prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity).Get()
        self.assertTrue(Gf.IsClose(velocity, Gf.Vec3f(0.0), 1e-3))

        # find main Physics frame
        # Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[2]
        step_button = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[2]")
        # stop_button = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[1]")
        # run_button = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[0]")
        
        for _ in range(10):
            await step_button.click()
            
        position = prim.GetAttribute("xformOp:translate").Get()        
        self.assertTrue(position[1] < -10.0)

        velocity = prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity).Get()
        self.assertTrue(velocity[1] < -100)


    async def test_physics_ui_debug_play_stop(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))

        await self._select_and_focus_debug_window()

        position = prim.GetAttribute("xformOp:translate").Get()        
        self.assertTrue(Gf.IsClose(position[1], 0.0, 1e-3))
        
        velocity = prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity).Get()
        self.assertTrue(Gf.IsClose(velocity, Gf.Vec3f(0.0), 1e-3))

        # find main Physics frame
        # Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[2]
        stop_button = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[1]")
        run_button = ui_test.find("Physics Debug//Frame/VStack[0]/ScrollingFrame[0]/VStack[0]/CollapsableFrame[0]/Frame[0]/ZStack[0]/VStack[0]/Frame[0]/HStack[0]/HStack[0]/Button[0]")
                
        await run_button.click()
        
        await self.wait(30)
            
        position = prim.GetAttribute("xformOp:translate").Get()        
        self.assertTrue(position[1] < -10.0)

        velocity = prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity).Get()
        self.assertTrue(velocity[1] < -100)
        
        await stop_button.click()
        
        position = prim.GetAttribute("xformOp:translate").Get()        
        self.assertTrue(Gf.IsClose(position[1], 0.0, 1e-3))
        
        velocity = prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity).Get()
        self.assertTrue(Gf.IsClose(velocity, Gf.Vec3f(0.0), 1e-3))
        
