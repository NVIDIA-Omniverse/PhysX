# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import Set, Tuple


def _get_physx_schema_names(pluginName: str) -> Tuple[Set[str], Set[str]]:
    """
    Get all schema type names from the specified plugin.
    
    Returns:
        A tuple of (prim_type_names, api_schema_names) where:
            - prim_type_names: Set of typed schema names (e.g., "PhysicsScene", "PhysicsJoint")
            - api_schema_names: Set of API schema names (e.g., "PhysicsRigidBodyAPI", "PhysicsCollisionAPI")
    """
    from pxr import Plug, Usd, Tf
    
    prim_type_names: Set[str] = set()
    api_schema_names: Set[str] = set()
    
    plug_registry = Plug.Registry()
    schema_registry = Usd.SchemaRegistry()
    
    plugin = plug_registry.GetPluginWithName(pluginName)
    if plugin is None:
        return prim_type_names, api_schema_names
    
    # Get all types derived from UsdSchemaBase
    all_types = Tf.Type.Find(Usd.SchemaBase).GetAllDerivedTypes()
    
    for tf_type in all_types:
        if not plugin.DeclaresType(tf_type):
            continue
            
        # Get the PhysX schema name for this type
        schema_type_name = schema_registry.GetSchemaTypeName(tf_type)
        if not schema_type_name:
            continue
        
        # Get schema kind to determine if it's a prim type or API schema
        schema_kind = schema_registry.GetSchemaKind(tf_type)
        
        if schema_kind in (Usd.SchemaKind.AbstractTyped, Usd.SchemaKind.ConcreteTyped):
            # This is a typed schema (prim type)
            prim_type_names.add(schema_type_name)
        elif schema_kind in (Usd.SchemaKind.SingleApplyAPI, Usd.SchemaKind.MultipleApplyAPI):
            # This is an API schema
            api_schema_names.add(schema_type_name)
    
    return prim_type_names, api_schema_names


def get_physx_schema_names() -> Tuple[Set[str], Set[str]]:
    """
    Get all PhysX schema type names.
    
    Returns:
        A set of PhysX schema type names.
    """    
    
    prim_type_names: Set[str] = set()
    api_schema_names: Set[str] = set()
    
    prim_types, api_schemas = _get_physx_schema_names("physxSchema")
    prim_type_names.update(prim_types)
    api_schema_names.update(api_schemas)

    prim_types, api_schemas = _get_physx_schema_names("physxSchemaAddition")
    prim_type_names.update(prim_types)
    api_schema_names.update(api_schemas)

    # List of schemas that are not capabilities
    not_capabilities = [
        "PhysxLimitAPI", "PhysxSceneAPI", "PhysxArticulationAPI",
        "PhysxArticulationRootAPI", "PhysxRigidBodyAPI", "PhysxCollisionAPI",
        "PhysxMaterialAPI", "PhysxJointAPI", "PhysxConvexHullCollisionAPI",
        "PhysxConvexDecompositionCollisionAPI", "PhysxTriangleMeshSimplificationCollisionAPI",
        "PhysxTriangleMeshCollisionAPI", "PhysxSDFMeshCollisionAPI", "PhysxSphereFillCollisionAPI",
        "PhysxPhysicsDistanceJointAPI", "PhysxCookedDataAPI"
    ]
    
    for schema_name in not_capabilities:
        prim_type_names.discard(schema_name)
        api_schema_names.discard(schema_name)

    return prim_type_names, api_schema_names
