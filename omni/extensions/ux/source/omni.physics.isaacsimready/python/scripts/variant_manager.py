# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import Dict, List, Set

import carb
from pxr import Sdf, Usd


class VariantManager:
    """
    Manager class that tracks prims with specific variant sets on opened stages.

    Maintains a set of registered variant set names to watch for, and a dictionary
    mapping those names to the prim paths that have corresponding variant sets.
    
    When a stage is opened, the manager traverses all prims and records which ones
    have variant sets matching the registered names.
    
    Note: Stage event subscriptions are managed by VariantSwitcher, which calls
    the public handler methods on this class.
    """

    def __init__(self):
        # Set of variant set names to watch for
        self._registered_variant_names: Set[str] = set()
        
        # Dictionary mapping variant set name -> set of prim paths that have that variant set
        self._variant_prim_paths: Dict[str, Set[Sdf.Path]] = {}
        
        # Pending resynced paths to process on next update
        self._pending_resync_paths: Set[Sdf.Path] = set()
        
        # Set of variant names to scan for (filter list)
        # If empty, all registered variant names will be scanned
        self._scan_variant_names: Set[str] = set()
        
        # Flag to track if variant tracking is enabled
        self._tracking_enabled: bool = False

    def startup(self):
        """Initialize the manager."""
        pass

    def shutdown(self):
        """Clean up and clear data."""
        self._tracking_enabled = False
        self._pending_resync_paths.clear()
        self._registered_variant_names.clear()
        self._variant_prim_paths.clear()
        self._scan_variant_names.clear()

    def enable_variant_tracking(self):
        """Enable variant tracking."""
        self._tracking_enabled = True

    def disable_variant_tracking(self):
        """Disable variant tracking."""
        self._tracking_enabled = False
        self._pending_resync_paths.clear()
        # Clear prim paths but keep registered variant names
        for variant_name in self._variant_prim_paths:
            self._variant_prim_paths[variant_name] = set()

    def is_tracking_enabled(self) -> bool:
        """Check if variant tracking is currently enabled."""
        return self._tracking_enabled

    def register_simulation_variant(self, variant_name: str, stage=None):
        """
        Register a variant set name to track.

        When a stage is opened or modified, prims with this variant set name
        will be recorded in the internal dictionary.

        Args:
            variant_name: The name of the variant set to track (e.g., "SimulationType", "PhysicsVariant").
            stage: Optional USD stage to scan immediately if tracking is enabled.
        """
        if variant_name in self._registered_variant_names:
            return
        
        self._registered_variant_names.add(variant_name)
        self._variant_prim_paths[variant_name] = set()
        
        # If tracking is enabled and a stage is provided, scan it for this new variant name
        if self._tracking_enabled and stage is not None:
            self._scan_stage_for_variant(stage, variant_name)

    def unregister_simulation_variant(self, variant_name: str):
        """
        Unregister a variant set name from tracking.

        Removes the variant name and all associated prim paths from tracking.

        Args:
            variant_name: The name of the variant set to stop tracking.
        """
        self._registered_variant_names.discard(variant_name)
        self._variant_prim_paths.pop(variant_name, None)

    def register_variants_to_scan(self, variant_names: List[str]):
        """
        Register a list of variant set names to scan for.

        When scanning the stage, only variant sets in this list (that are also registered)
        will be tracked. If this list is empty, all registered variant names will be scanned.

        Args:
            variant_names: List of variant set names to add to the scan filter.
        """
        self._scan_variant_names.update(variant_names)

    def unregister_variants_to_scan(self, variant_names: List[str]):
        """
        Remove variant set names from the scan filter list.

        Args:
            variant_names: List of variant set names to remove from the scan filter.
        """
        for name in variant_names:
            self._scan_variant_names.discard(name)

    def clear_variants_to_scan(self):
        """
        Clear all variant names from the scan filter list.

        After clearing, all registered variant names will be scanned.
        """
        self._scan_variant_names.clear()

    def get_variants_to_scan(self) -> Set[str]:
        """
        Get a copy of the variant names in the scan filter list.

        Returns:
            A set of variant set names that will be scanned.
            If empty, all registered variant names will be scanned.
        """
        return self._scan_variant_names.copy()

    def get_registered_variant_names(self) -> Set[str]:
        """
        Get a copy of the registered variant set names.

        Returns:
            A set of registered variant set names being tracked.
        """
        return self._registered_variant_names.copy()

    def get_prims_with_variant(self, variant_name: str) -> Set[Sdf.Path]:
        """
        Get prim paths that have the specified variant set.

        Args:
            variant_name: The variant set name to query.

        Returns:
            A set of Sdf.Path objects that have the specified variant set,
            or an empty set if the variant name is not registered or no prims have it.
        """
        return self._variant_prim_paths.get(variant_name, set()).copy()

    def get_all_variant_prims(self) -> Dict[str, Set[Sdf.Path]]:
        """
        Get a copy of the entire variant-to-prims mapping.

        Returns:
            A dictionary mapping variant set names to sets of Sdf.Path objects.
        """
        return {name: paths.copy() for name, paths in self._variant_prim_paths.items()}

    def has_pending_resync_paths(self) -> bool:
        """Check if there are pending resync paths to process."""
        return len(self._pending_resync_paths) > 0

    def add_pending_resync_paths(self, paths: List[Sdf.Path]):
        """
        Add paths to the pending resync set.

        Args:
            paths: List of Sdf.Path objects to add for processing.
        """
        for path in paths:
            self._pending_resync_paths.add(path)

    @carb.profiler.profile
    def on_stage_opened(self, stage):
        """
        Called when a stage is opened. Scans the stage for registered variant sets.

        Args:
            stage: The USD stage that was opened.
        """
        self._scan_stage_for_variants(stage)
        
        # Log summary for debugging
        for variant_name, prim_paths in self._variant_prim_paths.items():
            if prim_paths:
                carb.log_info(f"VariantManager: Found {len(prim_paths)} prims with variant set '{variant_name}'")

    def on_stage_closed(self):
        """Called when a stage is closed. Clears prim paths but keeps registered variant names."""
        for variant_name in self._variant_prim_paths:
            self._variant_prim_paths[variant_name] = set()

    def process_resynced_paths(self, stage):
        """
        Process pending resynced paths incrementally instead of full stage rescan.

        Args:
            stage: The USD stage to scan.
        """
        if not self._pending_resync_paths:
            return
        
        if stage is None:
            self._pending_resync_paths.clear()
            return
        
        # Determine which variant names to process
        if self._scan_variant_names:
            names_to_process = self._scan_variant_names & self._registered_variant_names
        else:
            names_to_process = self._registered_variant_names
        
        if not names_to_process:
            self._pending_resync_paths.clear()
            return
        
        # Take ownership of pending paths and clear for next batch
        paths_to_process = self._pending_resync_paths
        self._pending_resync_paths = set()
        
        # For each resynced path:
        # 1. Remove any tracked prim paths that are under this path (they may have been deleted/changed)
        # 2. Re-scan the subtree under this path for variant sets
        for resynced_path in paths_to_process:
            # Remove tracked paths that are at or under the resynced path
            for variant_name in names_to_process:
                paths_to_remove = set()
                for tracked_path in self._variant_prim_paths.get(variant_name, set()):
                    if tracked_path == resynced_path or tracked_path.HasPrefix(resynced_path):
                        paths_to_remove.add(tracked_path)
                self._variant_prim_paths[variant_name] -= paths_to_remove
            
            # Re-scan the subtree at this path
            prim = stage.GetPrimAtPath(resynced_path)
            if prim.IsValid():
                self._scan_prim_subtree(prim)

    def _scan_stage_for_variants(self, stage, variant_names_filter: List[str] = None):
        """
        Traverse the stage and find all prims with registered variant sets.

        Args:
            stage: The USD stage to scan.
            variant_names_filter: Optional list of variant set names to scan for.
                If provided, only variant sets matching these names will be tracked.
                If None, uses the stored scan filter list (_scan_variant_names).
                If the stored scan list is also empty, all registered variant names will be scanned.
        """
        # Determine which variant names to scan for
        if variant_names_filter:
            # Only scan for specified names that are also registered
            names_to_scan = set(variant_names_filter) & self._registered_variant_names
        elif self._scan_variant_names:
            # Use the stored scan filter list, intersected with registered names
            names_to_scan = self._scan_variant_names & self._registered_variant_names
        else:
            # Scan for all registered variant names
            names_to_scan = self._registered_variant_names
        
        # Clear existing prim paths for the variant names we're scanning
        for variant_name in names_to_scan:
            self._variant_prim_paths[variant_name] = set()
        
        if stage is None:
            return
        
        # Early exit if no variant names to scan
        if not names_to_scan:
            return
        
        # Traverse all prims and check for variant sets
        for prim in stage.Traverse():
            if prim.HasVariantSets():
                variant_sets = prim.GetVariantSets()
                variant_set_names = variant_sets.GetNames()
                
                for variant_name in variant_set_names:
                    if variant_name in names_to_scan:
                        self._variant_prim_paths[variant_name].add(prim.GetPath())

    def _scan_stage_for_variant(self, stage, variant_name: str):
        """
        Scan the stage for a specific variant set name.

        Args:
            stage: The USD stage to scan.
            variant_name: The specific variant set name to look for.
        """
        if stage is None or variant_name not in self._registered_variant_names:
            return
        
        self._variant_prim_paths[variant_name] = set()
        
        for prim in stage.Traverse():
            if prim.HasVariantSets():
                variant_sets = prim.GetVariantSets()
                if variant_sets.HasVariantSet(variant_name):
                    self._variant_prim_paths[variant_name].add(prim.GetPath())

    def _scan_prim_subtree(self, root_prim):
        """
        Scan a prim and its descendants for registered variant sets.

        Args:
            root_prim: The root prim to start scanning from.
        """
        # Determine which variant names to scan for
        if self._scan_variant_names:
            names_to_scan = self._scan_variant_names & self._registered_variant_names
        else:
            names_to_scan = self._registered_variant_names
        
        if not names_to_scan:
            return
        
        # Use Usd.PrimRange for efficient subtree traversal
        for prim in Usd.PrimRange(root_prim):
            if prim.HasVariantSets():
                variant_sets = prim.GetVariantSets()
                variant_set_names = variant_sets.GetNames()
                
                for variant_name in variant_set_names:
                    if variant_name in names_to_scan:
                        self._variant_prim_paths[variant_name].add(prim.GetPath())
