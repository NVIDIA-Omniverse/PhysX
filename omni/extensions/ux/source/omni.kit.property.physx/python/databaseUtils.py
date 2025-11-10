# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdPhysics, Usd, PhysxSchema
from . import database
from collections import namedtuple
import omni.physx.bindings._physx as pxb


RefreshCache = namedtuple("RefreshCache", [
    "applied_schemas",
    "is_a_base_joint",
    "has_rigidbody_api",
    "has_collision_api",
    "canadd_filteredpairs",
    "canadd_massapi",
    "hasconflictingapis_RigidBodyAPI",
    "hasconflictingapis_CollisionAPI",
    "hasconflictingapis_ArticulationRoot",
    "over_subtree_limit",
])


def patch_refresh_cache_to_prim(p, lmt=100000):
    strings = p.GetAppliedSchemas()
    schemas = {s.split(":")[0] for s in strings}

    def has_api(api, applied_schemas):
        alias = database.schema_aliases[api]
        return alias in applied_schemas

    has_rigidbody_api = has_api(UsdPhysics.RigidBodyAPI, schemas)
    has_collision_api = has_api(UsdPhysics.CollisionAPI, schemas)
    is_a_base_joint = database.is_a_base_joint(p)
    canadd_filteredpairs = (
        has_rigidbody_api or has_collision_api or
        has_api(UsdPhysics.ArticulationRootAPI, schemas) or
        has_api(PhysxSchema.PhysxDeformableBodyAPI, schemas) or # DEPRECATED
        has_api(PhysxSchema.PhysxDeformableSurfaceAPI, schemas) or # DEPRECATED
        has_api(Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableBodyAPI"), schemas) or
        p.IsA(PhysxSchema.PhysxParticleSystem)
    )
    canadd_massapi = (
        has_rigidbody_api or has_collision_api or
        has_api(PhysxSchema.PhysxDeformableBodyAPI, schemas) or # DEPRECATED
        has_api(PhysxSchema.PhysxDeformableSurfaceAPI, schemas) or # DEPRECATED
        has_api(PhysxSchema.PhysxParticleAPI, schemas)
    )

    over_subtree_limit = pxb.isOverConflictingApisSubtreeLimit(p, lmt)
    if not over_subtree_limit:
        conflict_precompute = pxb.hasconflictingapis_Precompute(p)
        hasconflictingapis_CollisionAPI = conflict_precompute[0]
        hasconflictingapis_ArticulationRoot = conflict_precompute[1]
        hasconflictingapis_RigidBodyAPI = conflict_precompute[2]
    else:
        hasconflictingapis_CollisionAPI = False
        hasconflictingapis_ArticulationRoot = False
        hasconflictingapis_RigidBodyAPI = False


    # precompute some values to a monkeypatched attrib
    setattr(p, "_refresh_cache", RefreshCache(
        schemas,
        is_a_base_joint,
        has_rigidbody_api,
        has_collision_api,
        canadd_filteredpairs,
        canadd_massapi,
        hasconflictingapis_RigidBodyAPI,
        hasconflictingapis_CollisionAPI,
        hasconflictingapis_ArticulationRoot,
        over_subtree_limit,
    ))
