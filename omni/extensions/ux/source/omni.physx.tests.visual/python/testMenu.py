# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
import omni.kit.ui_test as ui_test
import omni.ui as ui

CREATE_MENU_CAPTIONS = {"Physics Scene", "Ground Plane", "Physics Material", "Collision Group", "Joint", "Particle System"}


class PhysxCreateMenuTest(TestCase):
    category = TestCategory.Kit

    async def test_physics_main_menu_create(self):
        await self.new_stage()

        # Click Create in main menu
        create_menu = ui_test.get_menubar().find_menu("Create")
        self.assertTrue(create_menu is not None)
        await create_menu.click()

        # Find and click Physics submenu
        physics_menu = create_menu.find_menu("Physics")
        self.assertTrue(physics_menu is not None)
        await physics_menu.click()

        # Verify menu items exist
        menu_items_found = set()

        menu_items = physics_menu.find_all("**/")
        for ref in menu_items:
            if isinstance(ref.widget, ui.Menu) or isinstance(ref.widget, ui.MenuItem):
                if ref.widget.text in CREATE_MENU_CAPTIONS:
                    menu_items_found.add(ref.widget.text)

        self.assertEqual(len(menu_items_found), len(CREATE_MENU_CAPTIONS))

    async def _test_context_menu(self, window_name):
        # Find viewport and right click to open context menu
        window = ui_test.find(window_name)
        self.assertTrue(window is not None)
        safe_target = window.position + ui_test.Vec2(window.size.x / 2, window.size.y / 2)
        await window.focus()
        await self.wait(10)
        await window.right_click(pos=safe_target)

        # Get context menu
        menu_dict = await ui_test.get_context_menu()

        create_dict = menu_dict.get("Create")
        self.assertTrue(create_dict is not None)

        physics_dict = create_dict.get("Physics")
        self.assertTrue(physics_dict is not None)

        menu_items_found = set()

        # go through root items and items with submenus
        for k, v in physics_dict.items():
            if k == "_":
                for caption in v:
                    if caption in CREATE_MENU_CAPTIONS:
                        menu_items_found.add(caption)
            elif k in CREATE_MENU_CAPTIONS:
                menu_items_found.add(k)

        self.assertEqual(len(menu_items_found), len(CREATE_MENU_CAPTIONS))

    async def test_physics_viewport_create_context_menu(self):
        await self.new_stage()
        await self._test_context_menu("Viewport")

    async def test_physics_stage_create_context_menu(self):
        await self.new_stage()
        await self._test_context_menu("Stage")
