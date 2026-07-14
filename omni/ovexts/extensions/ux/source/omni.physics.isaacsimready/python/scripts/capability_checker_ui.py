# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import List, Tuple, Optional

import carb

# Setting path for enabling/disabling capability checking
SETTING_CAPABILITY_CHECK_ENABLED = "/exts/omni.physics.isaacsimready/capability_check_enabled"
# Setting path for enabling/disabling capability window display
SETTING_CAPABILITY_CHECK_WINDOW_ENABLED = "/exts/omni.physics.isaacsimready/capability_check_window_enabled"

# Try to import omni.ui - it may not be available in all environments
try:
    import omni.ui as ui
    _HAS_OMNI_UI = True
except ImportError:
    _HAS_OMNI_UI = False


def is_ui_available() -> bool:
    """Check if omni.ui is available for use."""
    return _HAS_OMNI_UI


class CapabilityItem(ui.AbstractItem if _HAS_OMNI_UI else object):
    """Single item representing a schema capability check result."""

    def __init__(self, schema_name: str, is_supported: bool, schema_type: str = ""):
        if _HAS_OMNI_UI:
            super().__init__()
            self.schema_name_model = ui.SimpleStringModel(schema_name)
            self.supported_model = ui.SimpleStringModel("Yes" if is_supported else "No")
            self.type_model = ui.SimpleStringModel(schema_type)
        self.schema_name = schema_name
        self.is_supported = is_supported
        self.schema_type = schema_type


class CapabilityTableModel(ui.AbstractItemModel if _HAS_OMNI_UI else object):
    """
    Model for the capability results table.
    Displays schema names and their support status.
    """

    def __init__(self):
        if _HAS_OMNI_UI:
            super().__init__()
        self._children: List[CapabilityItem] = []

    def clear(self):
        """Clear all items from the model."""
        self._children.clear()
        if _HAS_OMNI_UI:
            self._item_changed(None)

    def set_results(self, results: List[Tuple[str, bool, str]]):
        """
        Set the capability check results.

        Args:
            results: List of tuples (schema_name, is_supported, schema_type)
        """
        self._children = [CapabilityItem(name, supported, stype) for name, supported, stype in results]
        if _HAS_OMNI_UI:
            self._item_changed(None)

    def get_item_children(self, item):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        return self._children

    def get_item_value_model_count(self, item):
        """The number of columns: Schema Name, Type, Supported"""
        return 3

    def get_item_value_model(self, item, column_id):
        """Return the value model for the given column."""
        if item is None:
            return None
        if column_id == 0:
            return item.schema_name_model
        elif column_id == 1:
            return item.type_model
        elif column_id == 2:
            return item.supported_model
        return None


class CapabilityTableDelegate(ui.AbstractItemDelegate if _HAS_OMNI_UI else object):
    """
    Delegate for rendering capability table items with visual indicators.
    """

    def __init__(self):
        if _HAS_OMNI_UI:
            super().__init__()

    def build_branch(self, model, item, column_id, level, expanded):
        """No branch needed for flat list."""
        pass

    def build_widget(self, model, item, column_id, level, expanded):
        """Create a widget per column per item."""
        if not _HAS_OMNI_UI:
            return

        value_model = model.get_item_value_model(item, column_id)
        if value_model is None:
            return

        text = value_model.as_string

        # For the "Supported" column, add color coding
        if column_id == 2:
            if text == "Yes":
                ui.Label(text, style={"color": 0xFF00FF00})  # Green
            else:
                ui.Label(text, style={"color": 0xFF0000FF})  # Red (ABGR format)
        else:
            ui.Label(text)

    def build_header(self, column_id):
        """Build the header for each column."""
        if not _HAS_OMNI_UI:
            return

        headers = ["Schema Name", "Type", "Supported"]
        if column_id < len(headers):
            ui.Label(headers[column_id], style={"font_size": 14})


class CapabilityCheckerWindow:
    """
    Window that displays capability check results in a table format.
    Supports displaying results from multiple active simulations.
    """

    WINDOW_NAME = "Schema Capability Report"

    def __init__(self):
        self._window: Optional[ui.Window] = None
        self._models: List[CapabilityTableModel] = []
        self._delegate: Optional[CapabilityTableDelegate] = None
        self._enabled_checkbox_model: Optional[ui.SimpleBoolModel] = None
        self._settings = None

    def _on_enabled_changed(self, model):
        """Called when the enabled checkbox is toggled."""
        if self._settings is not None:
            self._settings.set(SETTING_CAPABILITY_CHECK_ENABLED, model.as_bool)

    def _get_status_info(self, results: Optional[List[Tuple[str, bool, str]]], capability_check_available: bool) -> Tuple[str, int]:
        """
        Calculate status text and color for a set of results.
        
        Args:
            results: List of (schema_name, is_supported, schema_type) tuples, or None if no physics schemas found.
            capability_check_available: Whether the capability check was successful.
        
        Returns:
            Tuple of (status_text, status_color)
        """
        # No physics schemas found on the stage
        if results is None:
            return "No physics", 0xFFAAAAAA  # Gray (ABGR format)

        total = len(results)
        supported = sum(1 for _, is_supported, _ in results if is_supported)
        unsupported = total - supported

        if not capability_check_available:
            return "Capability check not available", 0xFF00A5FF  # Orange (ABGR format)
        elif unsupported == 0:
            return f"All {total} schemas supported", 0xFF00FF00  # Green
        else:
            return f"{unsupported} of {total} schemas not supported", 0xFF0000FF  # Red

    def show(self, simulation_results: List[Tuple[str, Optional[List[Tuple[str, bool, str]]], bool]]):
        """
        Show the capability checker window with results for multiple simulations.

        Args:
            simulation_results: List of tuples (simulation_name, results, capability_check_available)
                where results is a list of (schema_name, is_supported, schema_type) tuples,
                or None if no physics schemas were found on the stage.
        """
        if not _HAS_OMNI_UI:
            return

        # Create delegate (shared across all tables)
        self._delegate = CapabilityTableDelegate()
        self._models = []

        # Get settings and current enabled state
        self._settings = carb.settings.get_settings()
        is_enabled = self._settings.get(SETTING_CAPABILITY_CHECK_ENABLED)
        self._enabled_checkbox_model = ui.SimpleBoolModel(is_enabled if is_enabled is not None else True)
        self._enabled_checkbox_model.add_value_changed_fn(self._on_enabled_changed)

        # Calculate window height based on number of simulations
        base_height = 150  # Space for settings and close button
        per_simulation_height = 80  # Height per simulation section (collapsed)
        window_height = min(600, base_height + len(simulation_results) * per_simulation_height)

        # Create window
        self._window = ui.Window(
            self.WINDOW_NAME,
            width=550,
            height=window_height,
            visible=True
        )

        with self._window.frame:
            with ui.VStack(spacing=5):
                # Create a section for each simulation
                for simulation_name, results, capability_check_available in simulation_results:
                    status_text, status_color = self._get_status_info(results, capability_check_available)

                    # Collapsible frame for each simulation
                    with ui.CollapsableFrame(f"Simulation: {simulation_name}", collapsed=False):
                        with ui.VStack(spacing=3):
                            # Status row
                            with ui.HStack(height=20):
                                ui.Label("Status:", width=60)
                                ui.Label(status_text, style={"color": status_color, "font_size": 14})

                            # Only show schema details if there are results (not None/no physics)
                            if results is not None and len(results) > 0:
                                # Create model for this simulation
                                model = CapabilityTableModel()
                                model.set_results(results)
                                self._models.append(model)

                                # Collapsible schema details (collapsed by default)
                                with ui.CollapsableFrame("Schema Details", collapsed=True, height=ui.Fraction(1)):
                                    with ui.ScrollingFrame(
                                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON
                                    ):
                                        ui.TreeView(
                                            model,
                                            delegate=self._delegate,
                                            root_visible=False,
                                            header_visible=True,
                                            columns_resizable=True,
                                            column_widths=[ui.Fraction(2), ui.Fraction(1), ui.Fraction(1)],
                                            style={
                                                "TreeView": {"background_selected_color": 0x66FFFFFF},
                                                "TreeView.Item": {"margin": 4},
                                            }
                                        )

                ui.Spacer(height=5)

                # Settings section
                with ui.HStack(height=25):
                    ui.CheckBox(model=self._enabled_checkbox_model, width=20)
                    ui.Label("Enable capability checking", style={"font_size": 14})

                ui.Spacer(height=5)

    def hide(self):
        """Hide the window."""
        if self._window:
            self._window.visible = False

    def clear(self):
        """Clear the results from all tables."""
        for model in self._models:
            model.clear()

    def destroy(self):
        """Destroy the window and clean up resources."""
        if self._window:
            self._window.destroy()
            self._window = None
        self._models = []
        self._delegate = None
        self._enabled_checkbox_model = None
        self._settings = None


# Global window instance for singleton pattern
_capability_window: Optional[CapabilityCheckerWindow] = None


def is_window_enabled() -> bool:
    """Check if the capability window display is enabled."""
    settings = carb.settings.get_settings()
    enabled = settings.get(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED)
    return enabled if enabled is not None else True


def set_window_enabled(enabled: bool):
    """Set the capability window display enabled state."""
    settings = carb.settings.get_settings()
    settings.set(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED, enabled)


def toggle_window_enabled():
    """Toggle the capability window display enabled state."""
    set_window_enabled(not is_window_enabled())


def show_capability_results(simulation_results: List[Tuple[str, Optional[List[Tuple[str, bool, str]]], bool]]):
    """
    Show the capability check results in a UI window for multiple simulations.

    Args:
        simulation_results: List of tuples (simulation_name, results, capability_check_available)
            where results is a list of (schema_name, is_supported, schema_type) tuples,
            or None if no physics schemas were found on the stage.
    """
    if not _HAS_OMNI_UI:
        return

    # Check if window display is enabled
    if not is_window_enabled():
        return

    global _capability_window
    if _capability_window is None:
        _capability_window = CapabilityCheckerWindow()

    _capability_window.show(simulation_results)


def hide_capability_results():
    """Hide the capability results window."""
    global _capability_window
    if _capability_window:
        _capability_window.hide()


def clear_capability_results():
    """Clear the results from the capability results table."""
    global _capability_window
    if _capability_window:
        _capability_window.clear()


def destroy_capability_window():
    """Destroy the capability results window and clean up resources."""
    global _capability_window
    if _capability_window:
        _capability_window.destroy()
        _capability_window = None
