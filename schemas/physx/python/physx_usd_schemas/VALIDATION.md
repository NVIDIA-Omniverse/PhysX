<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PhysX schema validation

Call `physx_usd_schemas.register_validators()` to register the package's
validators through the OpenUSD `pxr.UsdValidation` Python API. Registration is
explicit and idempotent; the codeless schema resource plugins remain
`Type: resource`.

Validator availability requires OpenUSD 26.05 or newer. Older OpenUSD releases
can still import the package and use its schemas. On those releases,
`register_validators()` returns `False`.

Applications that use validators declared in Plug metadata must register those
plugins before calling `register_validators()`, because OpenUSD reads plugin
validator metadata when its global validation registry is first constructed.
Validators registered through the explicit Python API can still be added later.
The stock `usdchecker` command does not import this package or call the
registration function automatically; use a Python caller or host integration
that does so.

## MeshCollisionAPI placement

The native validator name is
`physxSchemaValidators:MeshCollisionAPIChecker`. Its `schemaTypes` metadata is
exactly `UsdPhysicsMeshCollisionAPI`. It ships from schema-wheel version
25.11.3.

The rule applies only to prims with `UsdPhysics.MeshCollisionAPI`. Other prims
produce no errors. An applicable prim must:

- be a `UsdGeom.Mesh` or have `PhysxMeshMergeCollisionAPI`; and
- also have `UsdPhysics.CollisionAPI`.

Failures are `Error` severity and identify the prim as their only site. Placement
is checked before the required API, so an invalid prim missing both obligations
returns these diagnostics in order:

- `MESH_COLLISION_API_INVALID_PRIM`: `Prim '<path>' has MeshCollisionAPI but is not a UsdGeom Mesh nor a prim with PhysxMeshMergeCollisionAPI.`
- `MESH_COLLISION_REQUIRES_COLLISION_API`: `Prim '<path>' has MeshCollisionAPI but does not have CollisionAPI.`

Checking the non-Mesh merge exception requires registered PhysX schema
definitions. If the schemas were imported after the process first read USD
schema definitions, the validator raises a `RuntimeError` with a process-restart
remedy instead of reporting the asset as invalid.

Unexpected callback exceptions propagate to Python callers. Consumers should
report them as validation-tool failures, not asset-rule failures. Validation is
read-only and does not modify stage serialization.

This native validator implements the check behind SimReady's
`PHYSX.COL.002` requirement. The requirement ID, downstream adoption, and
feature/profile selection remain owned by SimReady.

## Accepted example

```usda
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
)
{
}
```

## Rejected example

`MeshCollisionAPI` does not define collision by itself:

```usda
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsMeshCollisionAPI"]
)
{
}
```
