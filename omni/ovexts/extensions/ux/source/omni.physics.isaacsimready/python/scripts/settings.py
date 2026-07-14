# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import carb.settings

# Optional dependency - omni.ui may not be available in headless mode
try:
    from omni import ui
    _HAS_UI = True
except ImportError:
    ui = None
    _HAS_UI = False

# Setting paths - these match the settings defined in extension.toml
SETTING_CAPABILITY_CHECK_ENABLED = "/exts/omni.physics.isaacsimready/capability_check_enabled"
SETTING_CAPABILITY_CHECK_WINDOW_ENABLED = "/exts/omni.physics.isaacsimready/capability_check_window_enabled"
SETTING_VARIANT_MANAGER_ENABLED = "/exts/omni.physics.isaacsimready/variant_manager_enabled"


def is_preferences_available():
    """Check if the preferences window extension and UI are available."""
    if not _HAS_UI:
        return False
    try:
        import omni.kit.window.preferences
        return True
    except ImportError:
        return False


class IsaacSimReadyPreferences:
    """
    Preferences page for Physics IsaacSim Ready extension.
    Registers extension settings in the Edit > Preferences window.
    """

    def __init__(self):
        self._name = "Physics IsaacSim Ready"
        self._widgets = []
        self._line_height = 23

    @property
    def name(self):
        return self._name

    def on_shutdown(self):
        """Cleanup when preferences page is unregistered."""
        self._widgets = []

    def _build_section(self, name, build_func):
        """Build a collapsable section with the given name and content builder."""
        with ui.CollapsableFrame(name, height=0):
            with ui.HStack():
                ui.Spacer(width=20)
                with ui.VStack():
                    build_func()

    def _add_setting_checkbox(self, name, setting_path, tooltip=""):
        """Add a checkbox setting with label."""
        from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget

        with ui.HStack(height=self._line_height):
            widget, model = create_setting_widget(setting_path, SettingType.BOOL)
            self._widgets.append(widget)
            ui.Label(name, width=ui.Fraction(100), tooltip=tooltip)

    def _build_capability_check_settings(self):
        """Build the capability check settings UI."""
        self._add_setting_checkbox(
            "Capability Check Enabled",
            SETTING_CAPABILITY_CHECK_ENABLED,
            tooltip="Enable/disable the schema capability checking feature.\n"
                    "When enabled, the system tracks which physics schemas are used in the scene."
        )
        self._add_setting_checkbox(
            "Show Capability Report Window",
            SETTING_CAPABILITY_CHECK_WINDOW_ENABLED,
            tooltip="Show/hide the Schema Capability Report window.\n"
                    "The window displays which physics schemas are used in the current scene."
        )

    def _build_variant_manager_settings(self):
        """Build the variant manager settings UI."""
        self._add_setting_checkbox(
            "Variant Manager Enabled",
            SETTING_VARIANT_MANAGER_ENABLED,
            tooltip="Enable/disable the variant manager feature.\n"
                    "When enabled, the system manages physics-related variant sets for IsaacSim ready assets."
        )

    def _build_reset_button(self):
        """Build a reset button to restore default settings."""
        def reset_to_defaults():
            settings = carb.settings.get_settings()
            # Reset to defaults defined in extension.toml
            settings.set(SETTING_CAPABILITY_CHECK_ENABLED, True)
            settings.set(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED, False)
            settings.set(SETTING_VARIANT_MANAGER_ENABLED, True)

        ui.Button(
            "Reset to Defaults",
            height=30,
            width=200,
            clicked_fn=reset_to_defaults,
            tooltip="Reset all IsaacSim Ready settings to their default values."
        )

    def build(self):
        """Build the preferences page UI."""
        with ui.VStack(height=0):
            self._build_section("Capability Checking", self._build_capability_check_settings)
            ui.Spacer(height=5)
            self._build_section("Variant Manager", self._build_variant_manager_settings)
            ui.Spacer(height=10)
            with ui.HStack():
                ui.Spacer(width=20)
                self._build_reset_button()


def register_preferences():
    """
    Register the IsaacSim Ready preferences page.
    
    Returns:
        The registered preferences page instance, or None if registration failed.
    """

    if not is_preferences_available():
        return None

    try:
        from omni.kit.window.preferences.scripts.preferences_window import PreferenceBuilder, register_page

        # Create a PreferenceBuilder-compatible class dynamically
        class IsaacSimReadyPreferenceBuilder(PreferenceBuilder):
            def __init__(self):
                super().__init__("Physics IsaacSim Ready")
                self._prefs = IsaacSimReadyPreferences()

            def on_shutdown(self):
                if self._prefs:
                    self._prefs.on_shutdown()
                    self._prefs = None

            def build(self):
                self._prefs.build()

        preferences_page = IsaacSimReadyPreferenceBuilder()
        register_page(preferences_page)
        return preferences_page
    except ImportError:
        return None


def unregister_preferences(preferences_page):
    """Unregister the IsaacSim Ready preferences page."""
    try:
        from omni.kit.window.preferences.scripts.preferences_window import unregister_page
        preferences_page.on_shutdown()
        unregister_page(preferences_page)
    except ImportError:
        pass
