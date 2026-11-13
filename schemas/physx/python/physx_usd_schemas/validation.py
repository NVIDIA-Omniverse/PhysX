# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""OpenUSD validators shipped with the PhysX schema package."""

from __future__ import annotations

VALIDATOR_NAME = "physxSchemaValidators:MeshCollisionAPIChecker"
MESH_COLLISION_API_INVALID_PRIM = "MESH_COLLISION_API_INVALID_PRIM"
MESH_COLLISION_REQUIRES_COLLISION_API = "MESH_COLLISION_REQUIRES_COLLISION_API"

__all__ = [
    "MESH_COLLISION_API_INVALID_PRIM",
    "MESH_COLLISION_REQUIRES_COLLISION_API",
    "VALIDATOR_NAME",
    "register_validators",
]


def _error(prim, name: str, message: str):
    from pxr import UsdValidation

    return UsdValidation.ValidationError(
        name,
        UsdValidation.ValidationErrorType.Error,
        [UsdValidation.ValidationErrorSite(prim.GetStage(), prim.GetPath())],
        message,
    )


def _validate_mesh_collision_api(prim, _time_range):
    from pxr import Usd, UsdGeom, UsdPhysics

    if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        return []

    path = prim.GetPath()
    errors = []
    is_mesh = prim.IsA(UsdGeom.Mesh)
    if not is_mesh and Usd.SchemaRegistry.FindSchemaInfo("PhysxMeshMergeCollisionAPI") is None:
        raise RuntimeError(
            f"PhysX schema definitions are unavailable while validating '{path}'. "
            "Restart the process and import physx_usd_schemas before the first "
            "USD stage or schema query."
        )
    is_valid_placement = is_mesh or prim.HasAPI("PhysxMeshMergeCollisionAPI")
    if not is_valid_placement:
        errors.append(
            _error(
                prim,
                MESH_COLLISION_API_INVALID_PRIM,
                f"Prim '{path}' has MeshCollisionAPI but is not a UsdGeom Mesh "
                "nor a prim with PhysxMeshMergeCollisionAPI.",
            )
        )
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        errors.append(
            _error(
                prim,
                MESH_COLLISION_REQUIRES_COLLISION_API,
                f"Prim '{path}' has MeshCollisionAPI but does not have CollisionAPI.",
            )
        )
    return errors


def register_validators() -> bool:
    """Register bundled validators when the OpenUSD Python API supports it.

    Returns ``True`` when the validator is available. OpenUSD releases before
    26.05 do not expose Python registration; in that case this function returns
    ``False`` so the codeless schemas remain usable.
    """

    try:
        from pxr import UsdValidation
    except ImportError:
        return False

    registry_type = getattr(UsdValidation, "ValidationRegistry", None)
    metadata_type = getattr(UsdValidation, "ValidatorMetadata", None)
    if registry_type is None or metadata_type is None:
        return False

    if getattr(registry_type, "RegisterPrimValidator", None) is None:
        return False

    registry = registry_type()
    if registry.HasValidator(VALIDATOR_NAME):
        return True

    metadata = metadata_type(
        name=VALIDATOR_NAME,
        doc="Checks PhysX placement and dependencies for UsdPhysics.MeshCollisionAPI.",
        keywords=["physx", "collision"],
        schemaTypes=["UsdPhysicsMeshCollisionAPI"],
    )
    registry.RegisterPrimValidator(metadata, _validate_mesh_collision_api)
    return bool(registry.HasValidator(VALIDATOR_NAME))
