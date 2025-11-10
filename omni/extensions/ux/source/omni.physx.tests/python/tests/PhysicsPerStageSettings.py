# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import carb.settings
import os
import omni.usd
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import tempfile
import omni.physx.bindings._physx as pbd
import omni.physxui
import omni.ui
import omni.kit.ui_test as ui_test


class PhysicsPerStageSettings(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_demos_save_settings(self):
        settings = carb.settings.get_settings()
        await self.new_stage()

        stagePickingForce = 123456

        # test change using ui and save
        winname = omni.physxui.scripts.settings.PhysicsSettings.get_window_name()
        widget = ui_test.find(f"{winname}//Frame/**/*.identifier=='{pbd.SETTING_MOUSE_PICKING_FORCE}'")
        widget.window.visible = True
        widget.window.position_x = widget.window.position_y = 0
        widget.window.width = widget.window.height = 800
        await widget.bring_to_front()
        await widget.input(str(stagePickingForce))
        tempfile_path = tempfile.gettempdir() + "/test_physics_demos_save_settings.usda"
        omni.usd.get_context().save_as_stage(tempfile_path)

        # reset
        settings.set(pbd.SETTING_MOUSE_PICKING_FORCE, settings.get(pbd.SETTING_MOUSE_PICKING_FORCE_DEFAULT))
        pickingForce = settings.get(pbd.SETTING_MOUSE_PICKING_FORCE)
        self.assertTrue(pickingForce != stagePickingForce)

        # test load
        await omni.usd.get_context().open_stage_async(tempfile_path)
        pickingForce = settings.get(pbd.SETTING_MOUSE_PICKING_FORCE)
        self.assertTrue(pickingForce == stagePickingForce)

    async def test_physics_demos_load_settings(self):
        settings = carb.settings.get_settings()
        stagePickingForce = 123456
        await self.new_stage()
        settings.set(pbd.SETTING_MOUSE_PICKING_FORCE, stagePickingForce)

        # test reset
        await self.new_stage()
        defaultPickingForce = settings.get(pbd.SETTING_MOUSE_PICKING_FORCE_DEFAULT)
        pickingForce = settings.get(pbd.SETTING_MOUSE_PICKING_FORCE)
        self.assertTrue(pickingForce != stagePickingForce)
        self.assertTrue(defaultPickingForce == pickingForce)

        # test load
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data")))
        await omni.usd.get_context().open_stage_async(schema_folder + "/SettingsTest.usda")

        pickingForce = settings.get(pbd.SETTING_MOUSE_PICKING_FORCE)
        self.assertTrue(pickingForce == stagePickingForce)
