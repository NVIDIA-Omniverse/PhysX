# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = []

import omni.ext

from .physx_schemas import get_physx_schema_names
from .newton_schemas import get_newton_schema_names, NewtonPhysXUiDefinitions
from .schemas import physxschema, physxschemaaddition, usdphysicsdeformableschema
from omni.physics.isaacsimready import get_capability_manager, get_variant_switcher
import omni.kit.property.physics as property

_PHYSX_COLLISION_ADVANCED_PROPS = (
    # PhysxCollisionAPI
    "physxCollision:contactOffset",
    "physxCollision:restOffset",
    "physxCollision:torsionalPatchRadius",
    "physxCollision:minTorsionalPatchRadius",
    # PhysxConvexHullCollisionAPI
    "physxConvexHullCollision:minThickness",
    # PhysxConvexDecompositionCollisionAPI
    "physxConvexDecompositionCollision:minThickness",
    "physxConvexDecompositionCollision:voxelResolution",
    "physxConvexDecompositionCollision:errorPercentage",
    "physxConvexDecompositionCollision:shrinkWrap",
    # PhysxSphereFillCollisionAPI
    "physxSphereFillCollision:voxelResolution",
    "physxSphereFillCollision:seedCount",
    "physxSphereFillCollision:fillMode",
    # PhysxTriangleMeshSimplificationCollisionAPI
    "physxTriangleMeshSimplificationCollision:weldTolerance",
    # PhysxTriangleMeshCollisionAPI
    "physxTriangleMeshCollision:weldTolerance",
    # PhysxSDFMeshCollisionAPI
    "physxSDFMeshCollision:sdfSubgridResolution",
    "physxSDFMeshCollision:sdfBitsPerSubgridPixel",
    "physxSDFMeshCollision:sdfNarrowBandThickness",
    "physxSDFMeshCollision:sdfMargin",
    "physxSDFMeshCollision:sdfEnableRemeshing",
    "physxSDFMeshCollision:sdfTriangleCountReductionFactor",
)


class PhysicsPhysxUIExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

    def on_startup(self, _ext_id):
        capability_manager = get_capability_manager()

        # Register all PhysX schema names
        prim_types, api_schemas = get_physx_schema_names()
        capability_manager.register_schema_type_names(prim_types)
        capability_manager.register_api_schema_names(api_schemas)

        # Register Newton schema names that are PhysX capabilities
        newton_prim_types, newton_api_schemas = get_newton_schema_names()
        capability_manager.register_schema_type_names(newton_prim_types)
        capability_manager.register_api_schema_names(newton_api_schemas)

        # Register PhysX property widgets
        property.register_parent_schema(
            "PhysxSchema",
            "PhysX",
            physxschema.widgets,
            physxschema.property_builders,
            physxschema.property_order,
            physxschema.extensions,
            physxschema.extras,
            physxschema.ignore,
            physxschema.internal_extensions,
        )
        property.register_parent_schema(
            "physxSchemaAddition",
            "PhysX",
            physxschemaaddition.widgets,
            physxschemaaddition.property_builders,
            physxschemaaddition.property_order,
            physxschemaaddition.extensions,
            physxschemaaddition.extras,
            physxschemaaddition.ignore,
            physxschemaaddition.internal_extensions,
        )
        property.register_parent_schema(
            "omniUsdPhysicsDeformableSchema",
            "PhysX",
            usdphysicsdeformableschema.widgets,
            usdphysicsdeformableschema.property_builders,
            usdphysicsdeformableschema.property_order,
            usdphysicsdeformableschema.extensions,
            usdphysicsdeformableschema.extras,
            usdphysicsdeformableschema.ignore,
            usdphysicsdeformableschema.internal_extensions,
        )

        # Register Newton property widgets under the PhysX engine group
        property.register_parent_schema(
            "newton",
            "PhysX",
            NewtonPhysXUiDefinitions.widgets,
            NewtonPhysXUiDefinitions.property_builders,
            NewtonPhysXUiDefinitions.property_order,
            NewtonPhysXUiDefinitions.extensions,
            NewtonPhysXUiDefinitions.extras,
            NewtonPhysXUiDefinitions.ignore,
            NewtonPhysXUiDefinitions.internal_extensions,
        )

        # Newton contact props sit in "Advanced"; PhysX collision "Advanced" props are
        # relabeled "Advanced - PhysX" so users can tell which solver owns each setting.
        property.register_property_display_group("newton:contactMargin", "Advanced", collapsed=True)
        property.register_property_display_group("newton:contactGap", "Advanced", collapsed=True)
        for prop in _PHYSX_COLLISION_ADVANCED_PROPS:
            property.register_property_display_group(prop, "Advanced - PhysX", collapsed=True)
        property.register_display_group_order(["Advanced", "Advanced - PhysX", "Inactive"])

        # Register simulator-to-variant mappings and the parent schema group
        get_variant_switcher().register_simulator_variant("PhysX", "physx")

        # Register group to fit with the simulator name
        property.register_parent_schema_group(
            "PhysX",
            ["PhysxSchema", "physxSchemaAddition", "omniUsdPhysicsDeformableSchema", "newton"],
        )

    def on_shutdown(self):
        property.unregister_property_display_group("newton:contactMargin")
        property.unregister_property_display_group("newton:contactGap")
        for prop in _PHYSX_COLLISION_ADVANCED_PROPS:
            property.unregister_property_display_group(prop)
        property.unregister_display_group_order(["Advanced", "Advanced - PhysX", "Inactive"])

        # Unregister PhysX property widgets
        property.unregister_parent_schema_group("PhysX")
        property.unregister_parent_schema("newton")
        property.unregister_parent_schema("PhysxSchema")
        property.unregister_parent_schema("physxSchemaAddition")
        property.unregister_parent_schema("omniUsdPhysicsDeformableSchema")
