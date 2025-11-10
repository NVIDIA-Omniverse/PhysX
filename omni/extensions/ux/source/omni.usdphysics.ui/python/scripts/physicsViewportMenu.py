# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import omni.ui as ui
import omni.usdphysicsui.bindings._usdphysicsUI as usdPhysicsUIBinding
from omni.kit.viewport.menubar.core.model.category_model import CategoryCustomItem
from omni.kit.viewport.menubar.core import ViewportMenuDelegate
from .physicsViewportMenuHelper import PhysicsViewportMenuHelper

class PhysicsViewportMenu:

    def register_with_viewport(self):
        self._helper = PhysicsViewportMenuHelper()
        self._display_instance = omni.kit.viewport.menubar.display.get_instance()
        self._physics_menu = CategoryCustomItem("Physics", self._build_menu)
        self._display_instance.register_custom_category_item("Show By Type", self._physics_menu)

    def _build_menu(self):
        # Main menu construction code
        with ui.Menu("Physics", delegate=ViewportMenuDelegate()):
            jointsItem = self._helper.create_menu_simple_bool_item("Joints", usdPhysicsUIBinding.SETTING_DISPLAY_JOINTS)
            # first time, by passing None as subscription, we just set the setting changes notification as active
            self._helper.safe_subscribe_to_setting_change(None, [jointsItem], usdPhysicsUIBinding.SETTING_DISPLAY_JOINTS)

    def unregister_from_viewport(self):
        if self._display_instance is not None:
            self._display_instance.deregister_custom_category_item("Show By Type", self._physics_menu)
            self._physics_menu = None
            self._helper = None
