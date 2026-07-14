# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
import omni.usd
import omni.physics.core


@dataclass
class SchemaFilter:
    """
    Filter for gathering schemas from a stage.
    
    Contains the resolved prim types and API schemas from the provided schema names.
    """
    prim_type_names: Set[str] = field(default_factory=set)
    api_schema_names: Set[str] = field(default_factory=set)
    prim_type_tf_types: Set = field(default_factory=set)
    api_schema_tf_types: Set = field(default_factory=set)

    @property
    def is_empty(self) -> bool:
        """Returns True if no schemas are in the filter."""
        return not self.prim_type_names and not self.api_schema_names


@dataclass
class CapabilityCheckResult:
    """Result of a capability check for a single schema."""
    schema_name: str
    is_capable: bool


@dataclass
class StageCapabilityReport:
    """Complete capability report for a stage."""
    simulation_id: omni.physics.core.SimulationId
    simulation_name: str
    prim_types: Set[str]
    applied_apis: Set[str]
    prim_type_capabilities: List[CapabilityCheckResult]
    api_capabilities: List[CapabilityCheckResult]
    success: bool
    error_message: Optional[str] = None

    @property
    def all_schemas_supported(self) -> bool:
        """Returns True if all schemas on the stage are supported by the simulator."""
        return all(cap.is_capable for cap in self.prim_type_capabilities + self.api_capabilities)

    @property
    def unsupported_schemas(self) -> List[str]:
        """Returns a list of schemas that are not supported by the simulator."""
        unsupported = []
        for cap in self.prim_type_capabilities + self.api_capabilities:
            if not cap.is_capable:
                unsupported.append(cap.schema_name)
        return unsupported


def _get_plugin_schema_types(plugin_names: List[str]) -> Tuple[Set[str], Set[str]]:
    """
    Get the prim type names and API schema names that belong to the specified plugins.
    
    Args:
        plugin_names: List of USD plugin registry plugin names to filter by.
        
    Returns:
        A tuple of (prim_type_names, api_schema_names) that belong to the specified plugins.
    """
    from pxr import Plug, Usd, Tf
    
    plugin_prim_types: Set[str] = set()
    plugin_api_schemas: Set[str] = set()
    
    plug_registry = Plug.Registry()
    schema_registry = Usd.SchemaRegistry()
    
    for plugin_name in plugin_names:
        plugin = plug_registry.GetPluginWithName(plugin_name)
        if plugin is None:
            continue
            
        # Get all types declared by this plugin
        declared_types = plug_registry.GetAllDerivedTypes(Tf.Type.FindByName("UsdSchemaBase"))
        
        all_types = Tf.Type.Find(Usd.SchemaBase).GetAllDerivedTypes()
        for tf_type in all_types:            
            if plugin.DeclaresType(tf_type):                
                # Get schema kind to determine if it's a prim type or API schema
                schema_kind = schema_registry.GetSchemaKind(tf_type)
                
                # Extract the schema name (remove namespace prefix like "UsdGeom" -> "Mesh")
                # The schema registry uses the full C++ type name, we need the USD schema name
                schema_type_name = schema_registry.GetSchemaTypeName(tf_type)
                if not schema_type_name:
                    continue
                
                if schema_kind == Usd.SchemaKind.AbstractTyped or schema_kind == Usd.SchemaKind.ConcreteTyped:
                    # This is a typed schema (prim type)
                    plugin_prim_types.add(schema_type_name)
                elif schema_kind in (Usd.SchemaKind.SingleApplyAPI, Usd.SchemaKind.MultipleApplyAPI):
                    # This is an API schema
                    plugin_api_schemas.add(schema_type_name)
    
    return plugin_prim_types, plugin_api_schemas


def gather_stage_schemas(
    stage,
    plugin_names: Optional[List[str]] = None
) -> Tuple[Set[str], Set[str]]:
    """
    Traverse all prims on the stage and gather unique prim type names and applied API schema names.
    
    Args:
        stage: The USD stage to traverse.
        plugin_names: Optional list of USD plugin registry plugin names. If provided, only
                      schemas that belong to these plugins will be included in the results.
        
    Returns:
        A tuple of (prim_types, applied_apis) where each is a set of unique schema names.
    """
    prim_types: Set[str] = set()
    applied_apis: Set[str] = set()

    if stage is None:
        return prim_types, applied_apis

    # Get the set of valid schema names if filtering by plugin
    filter_prim_types: Optional[Set[str]] = None
    filter_api_schemas: Optional[Set[str]] = None
    if plugin_names:
        filter_prim_types, filter_api_schemas = _get_plugin_schema_types(plugin_names)

    for prim in stage.Traverse():
        if not prim:
            continue

        # Get the prim type name
        type_name = prim.GetTypeName()
        if type_name:
            # Apply filter if plugin_names was specified
            if filter_prim_types is None or type_name in filter_prim_types:
                prim_types.add(type_name)

        # Get applied API schemas
        applied_schemas = prim.GetAppliedSchemas()
        for schema in applied_schemas:
            # Applied schemas may include instance names for multiple-apply APIs (e.g., "PhysicsLimitAPI:rotX")
            # Extract the base schema name by splitting on ":"
            schema_str = str(schema)
            base_schema = schema_str.split(":")[0]
            
            # Apply filter if plugin_names was specified
            if filter_api_schemas is None or base_schema in filter_api_schemas:
                applied_apis.add(base_schema)

    return prim_types, applied_apis


def get_active_simulations() -> List[Tuple[omni.physics.core.SimulationId, str]]:
    """
    Find all currently active simulations.
    
    Returns:
        A list of tuples (simulation_id, simulation_name) for all active simulations.
        Returns an empty list if no active simulations are found.
    """
    physics_interface = omni.physics.core.get_physics_interface()
    active_simulations = []

    simulation_ids = physics_interface.get_simulation_ids()
    for sim_id in simulation_ids:
        if sim_id == omni.physics.core.k_invalid_simulation_id:
            continue
        if physics_interface.is_simulation_active(sim_id):
            sim_name = physics_interface.get_simulation_name(sim_id)
            active_simulations.append((sim_id, sim_name or ""))

    return active_simulations


def check_simulation_capabilities(
    simulation_id: omni.physics.core.SimulationId,
    schema_names: List[str]
) -> Tuple[bool, List[bool]]:
    """
    Check if the simulation can handle the given schema types/APIs.
    
    Args:
        simulation_id: The simulation ID to check capabilities for.
        schema_names: List of schema names (prim types or API schema names).
        
    Returns:
        A tuple of (success, capabilities) where success indicates if the check was performed
        successfully, and capabilities is a list of booleans for each schema.
    """
    if not schema_names:
        return True, []

    physics_simulation = omni.physics.core.get_physics_simulation_interface()

    if physics_simulation is None or not callable(getattr(physics_simulation, 'is_capable_of_simulating', None)):
        return False, [False] * len(schema_names)

    return physics_simulation.is_capable_of_simulating(simulation_id, schema_names)


def check_stage_capabilities(stage=None) -> List[StageCapabilityReport]:
    """
    Check if the currently active simulators can handle all prim types and APIs on the stage.
    
    Args:
        stage: Optional USD stage to check. If None, uses the current stage from omni.usd context.
        
    Returns:
        A list of StageCapabilityReport containing the results of the capability check for each
        active simulation. Returns a list with a single error report if no stage or simulations found.
    """
    # Get the stage if not provided
    if stage is None:
        usd_context = omni.usd.get_context()
        if usd_context is None:
            return [StageCapabilityReport(
                simulation_id=omni.physics.core.k_invalid_simulation_id,
                simulation_name="",
                prim_types=set(),
                applied_apis=set(),
                prim_type_capabilities=[],
                api_capabilities=[],
                success=False,
                error_message="No USD context available"
            )]
        stage = usd_context.get_stage()

    if stage is None:
        return [StageCapabilityReport(
            simulation_id=omni.physics.core.k_invalid_simulation_id,
            simulation_name="",
            prim_types=set(),
            applied_apis=set(),
            prim_type_capabilities=[],
            api_capabilities=[],
            success=False,
            error_message="No stage is currently open"
        )]

    # Find all active simulations
    active_simulations = get_active_simulations()
    if not active_simulations:
        return [StageCapabilityReport(
            simulation_id=omni.physics.core.k_invalid_simulation_id,
            simulation_name="",
            prim_types=set(),
            applied_apis=set(),
            prim_type_capabilities=[],
            api_capabilities=[],
            success=False,
            error_message="No active simulation found"
        )]

    # Gather all schemas from the stage (only once, shared across all simulations)
    prim_types, applied_apis = gather_stage_schemas(stage)
    prim_type_list = sorted(list(prim_types))
    api_list = sorted(list(applied_apis))

    reports = []
    for simulation_id, simulation_name in active_simulations:
        # Check capabilities for prim types
        prim_type_success, prim_type_caps = check_simulation_capabilities(simulation_id, prim_type_list)

        prim_type_capabilities = []
        if prim_type_success and prim_type_caps:
            for schema_name, is_capable in zip(prim_type_list, prim_type_caps):
                prim_type_capabilities.append(CapabilityCheckResult(schema_name=schema_name, is_capable=is_capable))

        # Check capabilities for applied APIs
        api_success, api_caps = check_simulation_capabilities(simulation_id, api_list)

        api_capabilities = []
        if api_success and api_caps:
            for schema_name, is_capable in zip(api_list, api_caps):
                api_capabilities.append(CapabilityCheckResult(schema_name=schema_name, is_capable=is_capable))

        overall_success = prim_type_success and api_success
        error_message = None
        if not overall_success:
            error_message = "Simulation was unable to check capabilities"

        reports.append(StageCapabilityReport(
            simulation_id=simulation_id,
            simulation_name=simulation_name,
            prim_types=prim_types,
            applied_apis=applied_apis,
            prim_type_capabilities=prim_type_capabilities,
            api_capabilities=api_capabilities,
            success=overall_success,
            error_message=error_message
        ))

    return reports


def print_capability_report(reports: List[StageCapabilityReport]) -> None:
    """
    Print formatted capability reports to the console.
    
    Args:
        reports: The list of StageCapabilityReport to print.
    """
    print("=" * 60)
    print("Stage Capability Report")
    print("=" * 60)

    if not reports:
        print("No reports available.")
        print("=" * 60)
        return

    for i, report in enumerate(reports):
        if i > 0:
            print("-" * 60)

        if not report.success:
            print(f"Error: {report.error_message}")
            continue

        print(f"Simulation: {report.simulation_name} (ID: {report.simulation_id})")
        print()

        print(f"Prim Types Found ({len(report.prim_types)}):")
        for cap in report.prim_type_capabilities:
            status = "✓" if cap.is_capable else "✗"
            print(f"  [{status}] {cap.schema_name}")
        print()

        print(f"Applied APIs Found ({len(report.applied_apis)}):")
        for cap in report.api_capabilities:
            status = "✓" if cap.is_capable else "✗"
            print(f"  [{status}] {cap.schema_name}")
        print()

        if report.all_schemas_supported:
            print("Result: All schemas are supported by the active simulator.")
        else:
            unsupported = report.unsupported_schemas
            print(f"Result: {len(unsupported)} schema(s) not supported:")
            for schema in unsupported:
                print(f"  - {schema}")

    print("=" * 60)