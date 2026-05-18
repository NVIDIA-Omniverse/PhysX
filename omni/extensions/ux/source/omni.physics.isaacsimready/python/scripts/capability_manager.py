# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import List, Set, Tuple

import carb
import omni.kit.app
import omni.usd
import usdrt
from carb.eventdispatcher import get_eventdispatcher
from omni.physics.core import SimulationId, SimulationRegistryEventType, get_physics_interface
from pxr import Plug, Tf, Usd, UsdUtils

from .capability_checker import (
    _get_plugin_schema_types,
    check_simulation_capabilities,
    get_active_simulations,
)
from .capability_checker_ui import (
    is_ui_available,
    show_capability_results,
    clear_capability_results,
    destroy_capability_window,
    SETTING_CAPABILITY_CHECK_WINDOW_ENABLED,
)


SETTING_CAPABILITY_CHECK_ENABLED = "/exts/omni.physics.isaacsimready/capability_check_enabled"


class CapabilityManager:
    """
    Manager class that receives notifications about stage opened and closed events.

    Maintains sets of registered schema type names and API schema names that are used
    to filter schemas found on opened stages for capability checking.
    """

    def __init__(self):
        self._stage_event_subs = None
        self._usd_context = None
        self._schema_type_names: Set[str] = set()
        self._api_schema_names: Set[str] = set()
        self._objects_changed_listener = None
        self._simulation_registry_sub = None
        self._update_sub = None
        self._settings = None
        self._settings_sub = None
        self._window_settings_sub = None

    def startup(self):
        """Initialize the manager and subscribe to stage events."""
        self._usd_context = omni.usd.get_context()
        self._settings = carb.settings.get_settings()
        
        # Subscribe to setting changes
        self._settings_sub = self._settings.subscribe_to_node_change_events(
            SETTING_CAPABILITY_CHECK_ENABLED,
            self._on_setting_changed
        )
        
        # Subscribe to window enabled setting changes
        self._window_settings_sub = self._settings.subscribe_to_node_change_events(
            SETTING_CAPABILITY_CHECK_WINDOW_ENABLED,
            self._on_setting_changed
        )
        
        # Enable capability checking if both settings are enabled
        if self._are_both_settings_enabled():
            self._enable_capability_checking()

    def _are_both_settings_enabled(self) -> bool:
        """Check if both capability_check_enabled and capability_check_window_enabled are true."""
        check_enabled = self._settings.get(SETTING_CAPABILITY_CHECK_ENABLED)
        window_enabled = self._settings.get(SETTING_CAPABILITY_CHECK_WINDOW_ENABLED)
        return bool(check_enabled) and bool(window_enabled)

    def shutdown(self):
        """Clean up subscriptions."""
        self._disable_capability_checking()
        if self._settings_sub is not None and self._settings is not None:
            self._settings.unsubscribe_to_change_events(self._settings_sub)
            self._settings_sub = None
        if self._window_settings_sub is not None and self._settings is not None:
            self._settings.unsubscribe_to_change_events(self._window_settings_sub)
            self._window_settings_sub = None
        self._settings = None
        self._usd_context = None
        self._schema_type_names.clear()
        self._api_schema_names.clear()
        destroy_capability_window()

    def _on_setting_changed(self, item, event_type):
        """Called when either capability_check_enabled or capability_check_window_enabled changes."""
        if self._are_both_settings_enabled():
            self._enable_capability_checking()
        else:
            self._disable_capability_checking()

    def _enable_capability_checking(self):
        """Enable capability checking by subscribing to stage and USD notice events."""
        if self._stage_event_subs is not None:
            return  # Already enabled
        
        self._stage_event_subs = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physics.isaacsimready:CapabilityManager",
                event_name=self._usd_context.stage_event_name(event_type),
                on_event=handler
            )
            for event_type, handler in (
                (omni.usd.StageEventType.OPENED, self._on_stage_opened),
                (omni.usd.StageEventType.CLOSED, self._on_stage_closed),
            )
        ]
        # Register for USD ObjectsChanged notices to detect resync changes
        self._objects_changed_listener = Tf.Notice.RegisterGlobally(
            Usd.Notice.ObjectsChanged,
            self._on_objects_changed
        )
        
        # Subscribe to simulation registry events to detect when simulators are added or activated
        physics = get_physics_interface()
        if physics:
            self._simulation_registry_sub = physics.subscribe_simulation_registry_events(
                self._on_simulation_registry_event
            )
        
        # Run capability check if a stage is already open
        if self._usd_context.get_stage() is not None:
            self._on_stage_opened(None)

    def _disable_capability_checking(self):
        """Disable capability checking by unsubscribing from stage and USD notice events."""
        # Revoke update subscription
        if self._update_sub is not None:
            self._update_sub = None

        # Unsubscribe from simulation registry events
        if self._simulation_registry_sub is not None:
            self._simulation_registry_sub = None

        # Revoke USD objects changed listener
        if self._objects_changed_listener:
            self._objects_changed_listener.Revoke()
            self._objects_changed_listener = None

        # Revoke stage event subscriptions
        if self._stage_event_subs is not None:
            self._stage_event_subs = None

        clear_capability_results()

    def register_plugin_schema(self, plugin_name: str):
        """
        Register all schema types and API schemas from a USD plugin.

        Enumerates all schema type names and API schema names declared by the plugin
        and adds them to the tracked sets.

        Args:
            plugin_name: The name of the USD plugin to register (e.g., "physxSchema", "usdPhysics").
        """
        plug_registry = Plug.Registry()
        if not plug_registry.GetPluginWithName(plugin_name):
            carb.log_error(f"Plugin '{plugin_name}' not found in USD plug registry.")
            return

        # Enumerate schema types and API names from the plugin
        prim_types, api_schemas = _get_plugin_schema_types([plugin_name])
        self._schema_type_names.update(prim_types)
        self._api_schema_names.update(api_schemas)

    def unregister_plugin_schema(self, plugin_name: str):
        """
        Unregister all schema types and API schemas from a USD plugin.

        Removes all schema type names and API schema names declared by the plugin
        from the tracked sets.

        Args:
            plugin_name: The name of the USD plugin to unregister.
        """
        plug_registry = Plug.Registry()
        if not plug_registry.GetPluginWithName(plugin_name):
            return

        # Enumerate schema types and API names from the plugin and remove them
        prim_types, api_schemas = _get_plugin_schema_types([plugin_name])
        self._schema_type_names.difference_update(prim_types)
        self._api_schema_names.difference_update(api_schemas)

    def register_schema_type_names(self, schema_type_names: Set[str]):
        """
        Register schema type names for capability tracking.

        Args:
            schema_type_names: The schema type names to register (e.g., {"PhysicsScene", "PhysicsJoint"}).
        """
        self._schema_type_names.update(schema_type_names)

    def unregister_schema_type_name(self, schema_type_name: str):
        """
        Unregister an individual schema type name from capability tracking.

        Args:
            schema_type_name: The schema type name to unregister.
        """
        self._schema_type_names.discard(schema_type_name)

    def register_api_schema_names(self, api_schema_names: Set[str]):
        """
        Register API schema names for capability tracking.

        Args:
            api_schema_names: The API schema names to register (e.g., {"PhysicsRigidBodyAPI", "PhysicsCollisionAPI"}).
        """
        self._api_schema_names.update(api_schema_names)

    def unregister_api_schema_name(self, api_schema_name: str):
        """
        Unregister an individual API schema name from capability tracking.

        Args:
            api_schema_name: The API schema name to unregister.
        """
        self._api_schema_names.discard(api_schema_name)

    def get_registered_schema_type_names(self) -> Set[str]:
        """
        Get a copy of the registered schema type names.

        Returns:
            A set of registered schema type names.
        """
        return self._schema_type_names.copy()

    def get_registered_api_schema_names(self) -> Set[str]:
        """
        Get a copy of the registered API schema names.

        Returns:
            A set of registered API schema names.
        """
        return self._api_schema_names.copy()

    def _gather_stage_schemas(self, stage) -> Tuple[Set[str], Set[str]]:
        """
        Query the stage using USDRT to gather unique prim type names and applied API schema names.

        Uses USDRT's GetPrimsWithTypeName and GetPrimsWithAppliedAPIName for efficient queries
        instead of traversing all prims. Only includes schemas that are in the registered sets.

        Args:
            stage: The USD stage to query.

        Returns:
            A tuple of (prim_types, applied_apis) where each is a set of unique schema names
            that match the registered schemas.
        """
        prim_types: Set[str] = set()
        applied_apis: Set[str] = set()

        if stage is None:
            return prim_types, applied_apis

        # Get the USDRT stage from the USD stage        
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        rt_stage = usdrt.Usd.Stage.Attach(stage_id)

        # Query for each registered schema type name
        for type_name in self._schema_type_names:
            prims = rt_stage.GetPrimsWithTypeName(type_name)
            if len(prims) > 0:
                prim_types.add(type_name)

        # Query for each registered API schema name
        for api_name in self._api_schema_names:
            prims = rt_stage.GetPrimsWithAppliedAPIName(api_name)
            if len(prims) > 0:
                applied_apis.add(api_name)

        return prim_types, applied_apis

    def check_stage_capabilities(self, stage) -> List[Tuple[str, List[Tuple[str, bool, str]], bool]]:
        """
        Run a capability check against a given stage.

        Gathers schemas from the stage that match registered schema types and API schemas,
        then checks each active simulation for capability support.

        Args:
            stage: The USD stage to check capabilities for.

        Returns:
            A list of tuples, one per active simulation:
            (simulation_name, results, capability_check_available)
            where results is a list of (schema_name, is_supported, schema_type) tuples,
            or None if no physics schemas were found on the stage.
            Returns an empty list if no registered schemas or no active simulations.
        """
        if not self._schema_type_names and not self._api_schema_names:
            return []

        if stage is None:
            return []

        # Gather schemas from the stage filtered by registered schema names
        prim_types, applied_apis = self._gather_stage_schemas(stage)

        # Get all active simulations
        active_simulations = get_active_simulations()
        if not active_simulations:
            carb.log_warn("No active simulation found for capability check.")
            return []

        prim_type_list = sorted(list(prim_types))
        api_list = sorted(list(applied_apis))

        # If no schemas found on the stage, return "No physics" for all simulations
        if not prim_type_list and not api_list:
            # Return None for results to indicate no physics schemas found
            return [(sim_name or "Unknown", None, True) for _, sim_name in active_simulations]

        # Collect results for each simulation
        # List of tuples: (simulation_name, results, capability_check_available)
        all_simulation_results: List[Tuple[str, List[Tuple[str, bool, str]], bool]] = []

        for simulation_id, simulation_name in active_simulations:
            # Collect results for UI display: (schema_name, is_supported, schema_type)
            sim_results: List[Tuple[str, bool, str]] = []
            capability_check_available = True

            # Check capabilities for prim types
            if prim_type_list:
                success, capabilities = check_simulation_capabilities(simulation_id, prim_type_list)
                if success:
                    for schema_name, is_capable in zip(prim_type_list, capabilities):
                        sim_results.append((schema_name, is_capable, "Prim Type"))
                        if not is_capable:
                            carb.log_warn(f"Prim type '{schema_name}' is not supported by simulation '{simulation_name}'.")
                else:
                    capability_check_available = False

            # Check capabilities for applied APIs
            if api_list and capability_check_available:
                success, capabilities = check_simulation_capabilities(simulation_id, api_list)
                if success:
                    for schema_name, is_capable in zip(api_list, capabilities):
                        sim_results.append((schema_name, is_capable, "API Schema"))
                        if not is_capable:
                            carb.log_warn(f"API schema '{schema_name}' is not supported by simulation '{simulation_name}'.")
                else:
                    capability_check_available = False

            # Clear results if capability check is not available
            if not capability_check_available:
                sim_results.clear()

            all_simulation_results.append((simulation_name or "Unknown", sim_results, capability_check_available))

        return all_simulation_results

    @carb.profiler.profile
    def _on_stage_opened(self, event):
        """Called when a stage is opened."""
        # Clear any previous results before running a new check
        clear_capability_results()

        stage = self._usd_context.get_stage()
        all_simulation_results = self.check_stage_capabilities(stage)

        # Display results in UI if available
        if is_ui_available() and all_simulation_results:
            show_capability_results(all_simulation_results)

    def _on_stage_closed(self, event):
        """Called when a stage is closed."""
        clear_capability_results()

    def _on_objects_changed(self, notice, sender):
        """Called when objects in the stage change via TfNotice."""
        # Check if this is for our stage
        stage = self._usd_context.get_stage() if self._usd_context else None
        if stage is None or sender != stage:
            return

        # Check if this is a resync change
        if len(notice.GetResyncedPaths()) > 0:
            self._schedule_resync_check()

    def _on_simulation_registry_event(self, event_type: SimulationRegistryEventType, id: SimulationId, name: str):
        """Called when a simulation is registered, unregistered, activated, or deactivated."""
        # Mark validation dirty when a simulator is added or activated
        if event_type in (SimulationRegistryEventType.SIMULATION_REGISTERED, SimulationRegistryEventType.SIMULATION_ACTIVATED, SimulationRegistryEventType.SIMULATION_UNREGISTERED, SimulationRegistryEventType.SIMULATION_DEACTIVATED):
            self._schedule_resync_check()

    def _schedule_resync_check(self):
        """Register for the next kit update to perform resync check."""
        if self._update_sub is not None:
            return  # Already scheduled

        self._update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_update,
            observer_name="omni.physics.isaacsimready:ResyncCheck"
        )

    @carb.profiler.profile
    def _on_update(self, event):
        """Called on kit update after a resync change was detected."""
        # Revoke subscription immediately
        if self._update_sub is not None:
            self._update_sub = None

        # Run the same check as in _on_stage_opened
        self._on_stage_opened(None)
