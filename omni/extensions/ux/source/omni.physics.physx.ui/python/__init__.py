# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = []

import omni.ext

from .physx_schemas import get_physx_schema_names
from .schemas import physxschema, physxschemaaddition, usdphysicsdeformableschema
from omni.physics.isaacsimready import get_capability_manager, get_variant_switcher
import omni.kit.property.physics as property


class PhysicsPhysxUIExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

    def on_startup(self, _ext_id):
        capability_manager = get_capability_manager()
    
        # Register all PhysX schema names
        prim_types, api_schemas = get_physx_schema_names()
        capability_manager.register_schema_type_names(prim_types)
        capability_manager.register_api_schema_names(api_schemas)

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

        # Register simulator-to-variant mappings and the parent schema group
        get_variant_switcher().register_simulator_variant("PhysX", "physx")

        # Register group to fit with the simulator name
        property.register_parent_schema_group(
            "PhysX",
            ["PhysxSchema", "physxSchemaAddition", "omniUsdPhysicsDeformableSchema"],
        )

    def on_shutdown(self):        
        # Unregister PhysX property widgets
        property.unregister_parent_schema_group("PhysX")
        property.unregister_parent_schema("PhysxSchema")
        property.unregister_parent_schema("physxSchemaAddition")
        property.unregister_parent_schema("omniUsdPhysicsDeformableSchema")
