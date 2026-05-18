# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import Dict, List, Optional, Tuple

import carb
import omni.kit.app
import omni.usd
import omni.physics.core
from carb.eventdispatcher import get_eventdispatcher
from omni.physics.core import SimulationId, SimulationRegistryEventType, get_physics_interface
from pxr import Tf, Usd

# Optional dependency - notification manager may not be available
try:
    import omni.kit.notification_manager as nm
    _HAS_NOTIFICATION_MANAGER = True
except ImportError:
    nm = None
    _HAS_NOTIFICATION_MANAGER = False

# Optional dependency - property physics may not be available
try:
    import omni.kit.property.physics as property_physics
    _HAS_PROPERTY_PHYSICS = True
except ImportError:
    property_physics = None
    _HAS_PROPERTY_PHYSICS = False

from .variant_manager import VariantManager


SETTING_VARIANT_MANAGER_ENABLED = "/exts/omni.physics.isaacsimready/variant_manager_enabled"


class VariantSwitcher:
    """
    Manager class that maintains a mapping between simulator names and variant names.

    This class allows registering simulator-to-variant mappings, which can be used
    to switch between different simulation variants based on the active simulator.
    
    Also owns and manages a VariantManager instance for tracking prims with variant sets.
    Handles all stage event subscriptions and delegates functionality to VariantManager.
    """

    def __init__(self):
        # Dictionary mapping simulator name -> variant name
        self._simulator_variant_mapping: Dict[str, str] = {}
        
        # VariantManager instance for tracking prims with variant sets
        self._variant_manager: VariantManager = None
        
        # Stage event subscriptions
        self._stage_event_subs = None
        self._usd_context = None
        self._objects_changed_listener = None
        self._update_sub = None
        self._settings = None
        self._settings_sub = None
        
        # Simulation registry subscription
        self._simulation_registry_sub = None

    def register_simulator_variant(self, simulator_name: str, variant_name: str):
        """
        Register a mapping between a simulator name and a variant name.

        Args:
            simulator_name: The name of the simulator (e.g., "PhysX", "MuJoCo", "Newton").
            variant_name: The name of the variant to associate with this simulator.
        """
        self._simulator_variant_mapping[simulator_name] = variant_name

    def unregister_simulator_variant(self, simulator_name: str):
        """
        Remove the mapping for a simulator.

        Args:
            simulator_name: The name of the simulator to unregister.
        """
        self._simulator_variant_mapping.pop(simulator_name, None)

    def get_variant_for_simulator(self, simulator_name: str) -> Optional[str]:
        """
        Get the variant name associated with a simulator.

        Args:
            simulator_name: The name of the simulator to query.

        Returns:
            The variant name associated with the simulator, or None if not registered.
        """
        return self._simulator_variant_mapping.get(simulator_name)

    def get_all_mappings(self) -> Dict[str, str]:
        """
        Get a copy of all simulator-to-variant mappings.

        Returns:
            A dictionary mapping simulator names to variant names.
        """
        return self._simulator_variant_mapping.copy()

    def clear_mappings(self):
        """
        Clear all simulator-to-variant mappings.
        """
        self._simulator_variant_mapping.clear()

    def get_active_simulation(self) -> Optional[Tuple[omni.physics.core.SimulationId, str]]:
        """
        Get the currently active simulation.

        Returns:
            A tuple of (simulation_id, simulation_name) for the first active simulation,
            or None if no active simulation is found.
        """
        physics_interface = omni.physics.core.get_physics_interface()

        simulation_ids = physics_interface.get_simulation_ids()
        for sim_id in simulation_ids:
            if sim_id == omni.physics.core.k_invalid_simulation_id:
                continue
            if physics_interface.is_simulation_active(sim_id):
                sim_name = physics_interface.get_simulation_name(sim_id)
                return (sim_id, sim_name or "")

        return None

    def switch_variants_for_simulation(self, stage, simulation_name: str, variant_set_name: str = "Physics") -> Tuple[int, List]:
        """
        Switch the variant set on all tracked prims to the variant corresponding to the given simulation.

        Args:
            stage: The USD stage containing the prims.
            simulation_name: The name of the active simulation (e.g., "PhysX", "Newton").
            variant_set_name: The name of the variant set to switch (default: "Physics").

        Returns:
            A tuple of (number of prims switched, list of switched prim paths).
        """
        if stage is None or not self._variant_manager:
            return 0, []

        # Get the variant name for this simulation
        variant_name = self.get_variant_for_simulator(simulation_name)
        if variant_name is None:
            carb.log_info(f"VariantSwitcher: No variant mapping found for simulation '{simulation_name}'")
            return 0, []

        # Get all prims that have the specified variant set
        prim_paths = self._variant_manager.get_prims_with_variant(variant_set_name)
        if not prim_paths:
            return 0, []

        switched_count = 0
        switched_prim_paths = []
        for prim_path in prim_paths:
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                continue

            variant_sets = prim.GetVariantSets()
            if not variant_sets.HasVariantSet(variant_set_name):
                continue

            variant_set = variant_sets.GetVariantSet(variant_set_name)

            # Check if the variant exists in this variant set
            available_variants = variant_set.GetVariantNames()
            if variant_name not in available_variants:
                carb.log_warn(
                    f"VariantSwitcher: Variant '{variant_name}' not found in variant set '{variant_set_name}' "
                    f"on prim '{prim_path}'. Available variants: {available_variants}"
                )
                continue

            # Switch to the variant
            current_variant = variant_set.GetVariantSelection()
            if current_variant != variant_name:
                variant_set.SetVariantSelection(variant_name)
                switched_count += 1
                switched_prim_paths.append(str(prim_path))
                carb.log_info(
                    f"VariantSwitcher: Switched '{prim_path}' variant set '{variant_set_name}' "
                    f"from '{current_variant}' to '{variant_name}'"
                )

        if switched_count > 0:
            carb.log_info(
                f"VariantSwitcher: Switched {switched_count} prim(s) to variant '{variant_name}' "
                f"for simulation '{simulation_name}'"
            )

        return switched_count, switched_prim_paths

    def switch_variants_for_active_simulation(self, stage) -> int:
        """
        Switch variants based on the currently active simulation.

        This method checks what simulation is currently active and switches the variants
        on all tracked prims with registered variant sets to the corresponding variant.
        The variant set names are read from the VariantManager's registered simulation variants.

        Args:
            stage: The USD stage containing the prims.

        Returns:
            The total number of prims whose variants were switched.
        """
        active_sim = self.get_active_simulation()
        if active_sim is None:
            carb.log_info("VariantSwitcher: No active simulation found, skipping variant switch")
            return 0

        sim_id, sim_name = active_sim
        carb.log_info(f"VariantSwitcher: Active simulation detected: '{sim_name}' (ID: {sim_id})")

        if not self._variant_manager:
            return 0

        # Get all registered variant set names from VariantManager
        registered_variant_names = self._variant_manager.get_registered_variant_names()
        if not registered_variant_names:
            carb.log_info("VariantSwitcher: No variant set names registered, skipping variant switch")
            return 0

        total_switched = 0
        all_switched_paths = []
        for variant_set_name in registered_variant_names:
            count, switched_paths = self.switch_variants_for_simulation(stage, sim_name, variant_set_name)
            total_switched += count
            all_switched_paths.extend(switched_paths)

        # Post notification if any variants were actually switched
        if total_switched > 0 and _HAS_NOTIFICATION_MANAGER:
            variant_name = self.get_variant_for_simulator(sim_name)
            # Create a message with prim names (limit to first 3 for brevity)
            prim_list = ", ".join(all_switched_paths[:3])
            if len(all_switched_paths) > 3:
                prim_list += f" and {len(all_switched_paths) - 3} more"
            nm.post_notification(
                f"Variant switched for '{sim_name}' to '{variant_name}' on: {prim_list}",
                duration=5
            )

        return total_switched

    @property
    def variant_manager(self) -> VariantManager:
        """
        Get the VariantManager instance.

        Returns:
            The VariantManager instance owned by this switcher.
        """
        return self._variant_manager

    def startup(self):
        """Initialize the switcher and subscribe to stage events. Called when the extension starts."""
        self._variant_manager = VariantManager()
        self._variant_manager.startup()
        
        self._usd_context = omni.usd.get_context()
        self._settings = carb.settings.get_settings()
        
        # Subscribe to setting changes
        self._settings_sub = self._settings.subscribe_to_node_change_events(
            SETTING_VARIANT_MANAGER_ENABLED,
            self._on_setting_changed
        )
        
        # Enable variant tracking if setting is enabled
        if self._settings.get(SETTING_VARIANT_MANAGER_ENABLED):
            self._enable_variant_tracking()

    def shutdown(self):
        """Clean up the switcher. Called when the extension shuts down."""
        self._disable_variant_tracking()
        
        if self._settings_sub is not None and self._settings is not None:
            self._settings.unsubscribe_to_change_events(self._settings_sub)
            self._settings_sub = None
        self._settings = None
        self._usd_context = None
        
        self._simulator_variant_mapping.clear()
        if self._variant_manager:
            self._variant_manager.shutdown()
            self._variant_manager = None

    def _enable_variant_tracking(self):
        """Enable variant tracking by subscribing to stage events."""
        if self._stage_event_subs is not None:
            return  # Already enabled
        
        self._stage_event_subs = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physics.isaacsimready:VariantSwitcher",
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
        
        # Subscribe to simulation registry events to detect when simulators are activated
        physics = get_physics_interface()
        if physics:
            self._simulation_registry_sub = physics.subscribe_simulation_registry_events(
                self._on_simulation_registry_event
            )
        
        # Enable tracking in VariantManager
        if self._variant_manager:
            self._variant_manager.enable_variant_tracking()
        
        # Run variant scan if a stage is already open
        if self._usd_context.get_stage() is not None:
            self._on_stage_opened(None)

    def _disable_variant_tracking(self):
        """Disable variant tracking by unsubscribing from events."""
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

        # Disable tracking in VariantManager
        if self._variant_manager:
            self._variant_manager.disable_variant_tracking()

    def _on_setting_changed(self, item, event_type):
        """Called when the variant_manager_enabled setting changes."""
        if self._settings.get(SETTING_VARIANT_MANAGER_ENABLED):
            self._enable_variant_tracking()
        else:
            self._disable_variant_tracking()

    @carb.profiler.profile
    def _on_stage_opened(self, event):
        """Called when a stage is opened."""
        if self._variant_manager and self._usd_context:
            stage = self._usd_context.get_stage()
            self._variant_manager.on_stage_opened(stage)
            
            # After scanning the stage, switch variants based on the active simulation
            self.switch_variants_for_active_simulation(stage)

    def _on_stage_closed(self, event):
        """Called when a stage is closed."""
        if self._variant_manager:
            self._variant_manager.on_stage_closed()

    def _on_simulation_registry_event(self, event_type: SimulationRegistryEventType, sim_id: SimulationId, name: str):
        """Called when a simulation is registered, unregistered, activated, or deactivated."""
        # When a simulation is activated, switch variants to match
        if event_type == SimulationRegistryEventType.SIMULATION_ACTIVATED:
            carb.log_info(f"VariantSwitcher: Simulation '{name}' (ID: {sim_id}) activated")

            if _HAS_PROPERTY_PHYSICS:
                if property_physics.is_parent_schema_group_registered(name):
                    property_physics.activate_parent_schema_group(name)
                elif property_physics.is_parent_schema_registered(name):
                    property_physics.activate_parent_schema(name)

            if self._usd_context:
                stage = self._usd_context.get_stage()
                if stage is not None:
                    # Switch variants for all registered variant set names
                    self._switch_variants_for_simulation_name(stage, name)
        elif event_type == SimulationRegistryEventType.SIMULATION_DEACTIVATED:
            carb.log_info(f"VariantSwitcher: Simulation '{name}' (ID: {sim_id}) deactivated")
                
            if _HAS_PROPERTY_PHYSICS:
                if property_physics.is_parent_schema_group_registered(name):
                    property_physics.deactivate_parent_schema_group(name)
                elif property_physics.is_parent_schema_registered(name):
                    property_physics.deactivate_parent_schema(name)
        elif event_type == SimulationRegistryEventType.SIMULATION_UNREGISTERED:
            carb.log_info(f"VariantSwitcher: Simulation '{name}' (ID: {sim_id}) unregistered")
            if _HAS_PROPERTY_PHYSICS:
                if property_physics.is_parent_schema_group_registered(name):
                    property_physics.deactivate_parent_schema_group(name)
                elif property_physics.is_parent_schema_registered(name):
                    property_physics.deactivate_parent_schema(name)

    def _switch_variants_for_simulation_name(self, stage, simulation_name: str) -> int:
        """
        Switch variants based on a specific simulation name.

        Args:
            stage: The USD stage containing the prims.
            simulation_name: The name of the simulation to switch variants for.

        Returns:
            The total number of prims whose variants were switched.
        """
        if not self._variant_manager:
            return 0

        # Get all registered variant set names from VariantManager
        registered_variant_names = self._variant_manager.get_registered_variant_names()
        if not registered_variant_names:
            return 0

        total_switched = 0
        all_switched_paths = []
        for variant_set_name in registered_variant_names:
            count, switched_paths = self.switch_variants_for_simulation(stage, simulation_name, variant_set_name)
            total_switched += count
            all_switched_paths.extend(switched_paths)

        # Post notification if any variants were actually switched
        if total_switched > 0 and _HAS_NOTIFICATION_MANAGER:
            variant_name = self.get_variant_for_simulator(simulation_name)
            # Create a message with prim names (limit to first 3 for brevity)
            prim_list = ", ".join(all_switched_paths[:3])
            if len(all_switched_paths) > 3:
                prim_list += f" and {len(all_switched_paths) - 3} more"
            nm.post_notification(
                f"Variant switched for '{simulation_name}' to '{variant_name}' on: {prim_list}",
                duration=5
            )

        return total_switched

    def _switch_variants_for_paths(self, stage, resynced_paths: List) -> int:
        """
        Switch variants only for prims at or under the specified paths.

        Args:
            stage: The USD stage containing the prims.
            resynced_paths: List of Sdf.Path objects that were resynced.

        Returns:
            The total number of prims whose variants were switched.
        """
        if stage is None or not resynced_paths or not self._variant_manager:
            return 0

        # Get the active simulation
        active_sim = self.get_active_simulation()
        if active_sim is None:
            carb.log_info("VariantSwitcher: No active simulation found, skipping variant switch")
            return 0

        sim_id, sim_name = active_sim
        carb.log_info(f"VariantSwitcher: Active simulation detected: '{sim_name}' (ID: {sim_id})")

        # Get the variant name for this simulation
        variant_name = self.get_variant_for_simulator(sim_name)
        if variant_name is None:
            carb.log_info(f"VariantSwitcher: No variant mapping found for simulation '{sim_name}'")
            return 0

        # Get all registered variant set names from VariantManager
        registered_variant_names = self._variant_manager.get_registered_variant_names()
        if not registered_variant_names:
            return 0

        total_switched = 0
        all_switched_paths = []
        for variant_set_name in registered_variant_names:
            # Get all prims that have the specified variant set
            prim_paths = self._variant_manager.get_prims_with_variant(variant_set_name)
            if not prim_paths:
                continue

            # Filter to only include prims at or under the resynced paths
            filtered_prim_paths = []
            for prim_path in prim_paths:
                for resynced_path in resynced_paths:
                    if prim_path == resynced_path or prim_path.HasPrefix(resynced_path):
                        filtered_prim_paths.append(prim_path)
                        break

            if not filtered_prim_paths:
                continue

            # Switch variants for the filtered prims
            for prim_path in filtered_prim_paths:
                prim = stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    continue

                variant_sets = prim.GetVariantSets()
                if not variant_sets.HasVariantSet(variant_set_name):
                    continue

                variant_set = variant_sets.GetVariantSet(variant_set_name)

                # Check if the variant exists in this variant set
                available_variants = variant_set.GetVariantNames()
                if variant_name not in available_variants:
                    carb.log_warn(
                        f"VariantSwitcher: Variant '{variant_name}' not found in variant set '{variant_set_name}' "
                        f"on prim '{prim_path}'. Available variants: {available_variants}"
                    )
                    continue

                # Switch to the variant
                current_variant = variant_set.GetVariantSelection()
                if current_variant != variant_name:
                    variant_set.SetVariantSelection(variant_name)
                    total_switched += 1
                    all_switched_paths.append(str(prim_path))
                    carb.log_info(
                        f"VariantSwitcher: Switched '{prim_path}' variant set '{variant_set_name}' "
                        f"from '{current_variant}' to '{variant_name}'"
                    )

        if total_switched > 0:
            carb.log_info(
                f"VariantSwitcher: Switched {total_switched} prim(s) to variant '{variant_name}' "
                f"for simulation '{sim_name}' (resynced paths only)"
            )
            if _HAS_NOTIFICATION_MANAGER:
                # Create a message with prim names (limit to first 3 for brevity)
                prim_list = ", ".join(all_switched_paths[:3])
                if len(all_switched_paths) > 3:
                    prim_list += f" and {len(all_switched_paths) - 3} more"
                nm.post_notification(
                    f"Variant switched for '{sim_name}' to '{variant_name}' on: {prim_list}",
                    duration=5
                )

        return total_switched

    def _on_objects_changed(self, notice, sender):
        """Called when objects in the stage change via TfNotice."""
        # Check if this is for our stage
        stage = self._usd_context.get_stage() if self._usd_context else None
        if stage is None or sender != stage:
            return
        
        # Early exit if no variant manager or no variant names are registered
        if not self._variant_manager or not self._variant_manager.get_registered_variant_names():
            return

        # Collect resynced paths for processing on next update
        resynced_paths = notice.GetResyncedPaths()
        if len(resynced_paths) > 0:
            self._variant_manager.add_pending_resync_paths(list(resynced_paths))
            self._schedule_resync_check()

    def _schedule_resync_check(self):
        """Register for the next kit update to perform resync check."""
        if self._update_sub is not None:
            return  # Already scheduled

        self._update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_update,
            observer_name="omni.physics.isaacsimready:VariantResyncCheck"
        )

    @carb.profiler.profile
    def _on_update(self, event):
        """Called on kit update after a resync change was detected."""
        # Revoke subscription immediately
        if self._update_sub is not None:
            self._update_sub = None

        # Process pending resynced paths via VariantManager
        if self._variant_manager and self._usd_context:
            stage = self._usd_context.get_stage()

            filtered_paths = []
            # Filter out paths that are already internally stored in VariantManager
            if self._variant_manager.has_pending_resync_paths():
                # Collect all internally tracked paths
                all_tracked_paths = set()
                for variant_name in self._variant_manager.get_registered_variant_names():
                    all_tracked_paths.update(self._variant_manager.get_prims_with_variant(variant_name))

                # Get pending paths and filter them
                pending_paths = list(self._variant_manager._pending_resync_paths)
                filtered_paths = [path for path in pending_paths if path not in all_tracked_paths]

                # Replace pending paths with filtered ones
                self._variant_manager._pending_resync_paths.clear()
                self._variant_manager.add_pending_resync_paths(filtered_paths)

            self._variant_manager.process_resynced_paths(stage)

            # After processing resynced paths, switch variants only for the filtered paths and their children
            if filtered_paths:
                self._switch_variants_for_paths(stage, filtered_paths)
            else:
                # If no filtered paths, fall back to switching all variants
                self.switch_variants_for_active_simulation(stage)
