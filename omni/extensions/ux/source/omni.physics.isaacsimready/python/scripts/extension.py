# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ext

from .capability_manager import CapabilityManager
from .variant_switcher import VariantSwitcher
from .usd_physics_schemas import get_usd_physics_schema_names
from .capability_checker_ui import is_ui_available, SETTING_CAPABILITY_CHECK_WINDOW_ENABLED
from .settings import register_preferences, unregister_preferences

_capability_manager_instance = None
_variant_switcher_instance = None


def get_capability_manager():
    """Get the global CapabilityManager instance."""
    return _capability_manager_instance


def get_variant_switcher():
    """Get the global VariantSwitcher instance."""
    return _variant_switcher_instance


def get_variant_manager():
    """Get the global VariantManager instance (accessed through VariantSwitcher)."""
    if _variant_switcher_instance:
        return _variant_switcher_instance.variant_manager
    return None


def _build_capability_report_menu_item(settings, helper):
    """
    Build the Schema Capability Report menu item.
    This function is called by PhysicsViewportMenuHelper when building the viewport menu.
    """
    import omni.ui as ui
    from omni.kit.viewport.menubar.core import SelectableMenuItem

    def on_value_changed(model, menu_item):
        show = model.get_value_as_bool()
        settings.set(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED, show)

    # Create the menu item
    menu_item = SelectableMenuItem(
        "Schema Capability Report",
        ui.SimpleBoolModel(settings.get_as_bool(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED))
    )
    menu_item.model.add_value_changed_fn(lambda model: on_value_changed(model, menu_item))

    # Subscribe to setting changes to keep menu in sync
    helper.safe_subscribe_to_setting_change(None, [menu_item], SETTING_CAPABILITY_CHECK_WINDOW_ENABLED)


class PhysicsIsaacSimReadyExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._capability_manager = None
        self._variant_switcher = None
        self._menu_builder_registered = False
        self._preferences_registered = False

    def on_startup(self, _ext_id):
        global _capability_manager_instance, _variant_switcher_instance
        
        # Initialize CapabilityManager
        self._capability_manager = CapabilityManager()
        self._capability_manager.startup()
        _capability_manager_instance = self._capability_manager

        # Register all USD physics schema names
        prim_types, api_schemas = get_usd_physics_schema_names()
        self._capability_manager.register_schema_type_names(prim_types)
        self._capability_manager.register_api_schema_names(api_schemas)
    
        # Initialize VariantSwitcher (which also creates and manages VariantManager)
        self._variant_switcher = VariantSwitcher()
        self._variant_switcher.startup()
        _variant_switcher_instance = self._variant_switcher

        # Register simulation variant with the VariantManager
        self._variant_switcher.variant_manager.register_simulation_variant("Physics")

        # Register viewport menu builder
        if is_ui_available():
            self._register_viewport_menu_builder()

        # Register preferences page
        self._preferences_page = register_preferences()        

    def _register_viewport_menu_builder(self):
        """Register the menu builder with PhysicsViewportMenuHelper."""
        try:
            from omni.usdphysicsui import PhysicsViewportMenuHelper
            PhysicsViewportMenuHelper.register_additional_menu_builder(_build_capability_report_menu_item)
            self._menu_builder_registered = True
        except ImportError:
            pass

    def _unregister_viewport_menu_builder(self):
        """Unregister the menu builder."""
        if self._menu_builder_registered:
            try:
                from omni.usdphysicsui import PhysicsViewportMenuHelper
                PhysicsViewportMenuHelper.unregister_additional_menu_builder(_build_capability_report_menu_item)
            except ImportError:
                pass
            self._menu_builder_registered = False

    def on_shutdown(self):
        global _capability_manager_instance, _variant_switcher_instance
        
        # Unregister preferences page
        if self._preferences_page is not None:
            unregister_preferences(self._preferences_page)
            self._preferences_page = None
        
        self._unregister_viewport_menu_builder()
        if self._variant_switcher:
            self._variant_switcher.shutdown()
            self._variant_switcher = None
        _variant_switcher_instance = None
        if self._capability_manager:
            self._capability_manager.shutdown()
            self._capability_manager = None
        _capability_manager_instance = None
