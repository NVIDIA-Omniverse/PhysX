<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# physx-usd-schemas

Codeless PhysX USD schemas for authoring and validation with a stock [OpenUSD](https://openusd.org/) runtime.

This package ships two USD schema modules and one helper module as pure-Python
subpackages (no compiled C++ bindings):

- **`physx_usd_schemas.PhysxSchema`** -- PhysX simulation schemas (rigid body, collision, joints, vehicles, particles, ...)
- **`physx_usd_schemas.OmniUsdPhysicsDeformableSchema`** -- Omni deformable-body simulation schemas
- **`physx_usd_schemas.PhysicsSchemaTools`** -- Schema utility helpers (e.g. `units_db`)

Importing `physx_usd_schemas` registers all bundled schemas with any stock `usd-core` runtime,
so you can author physics-annotated USD files without the full Omniverse runtime. On OpenUSD
26.05 or newer, call `physx_usd_schemas.register_validators()` to register the bundled
validators through `pxr.UsdValidation`.

This wheel supplies the NVIDIA physics schema extensions -- the `Physx*` API schemas and the
`OmniPhysics*` deformable schemas. The official `pxr.UsdPhysics` core physics schemas come from
`usd-core` (OpenUSD). The base wheel does not depend on `usd-core`, because a host application
may already provide a compatible OpenUSD runtime.

## Installation

For standalone Python, choose one optional OpenUSD provider:

```
pip install "physx-usd-schemas[usd]>=25.11.3"
pip install "physx-usd-schemas[usdex]>=25.11.3"
```

The `usd` extra installs `usd-core>=26.5`; the `usdex` extra installs
`usd-exchange>=3.0.1`. Do not combine them because both provide `pxr`.
Install `physx-usd-schemas` without either extra when the host application
already provides OpenUSD.

## Usage

### Register all schemas at once (recommended)

Importing `physx_usd_schemas` registers all bundled schemas automatically -- no explicit
call is needed:

```python
import physx_usd_schemas  # registers all schemas on import
from pxr import Usd

stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")
prim.ApplyAPI("PhysxRigidBodyAPI")
prim.ApplyAPI("PhysxCollisionAPI")
print(prim.GetAppliedSchemas())
```

`physx_usd_schemas.register_all()` is also available for explicit use and is safe to call again.
If the process already provides a schema plugin with the same name, for example the compiled
PhysX schema of a Kit application or a plugin found through `PXR_PLUGINPATH_NAME`, the package
keeps that plugin and skips its bundled copy.

### Validate `MeshCollisionAPI`

OpenUSD 26.05 or newer supports explicit Python validator registration. Register other
Plug-declared validators first, then call `register_validators()`. The callback checks
applicability itself, so direct validation and `ValidationContext` have the same verdict.
Import `physx_usd_schemas` before the process opens a USD stage or queries schema
definitions; late schema registration requires a process restart. The first validator ships
in `physx-usd-schemas` 25.11.3:

```python
import physx_usd_schemas
from pxr import Usd, UsdGeom, UsdPhysics, UsdValidation

assert physx_usd_schemas.register_validators()

stage = Usd.Stage.CreateInMemory()
mesh = UsdGeom.Mesh.Define(stage, "/World/Mesh")
UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

registry = UsdValidation.ValidationRegistry()
validator = registry.GetOrLoadValidatorByName(
    "physxSchemaValidators:MeshCollisionAPIChecker"
)
errors = validator.Validate(mesh.GetPrim())
assert errors == []
```

OpenUSD releases before 26.05 can still use the schemas; they do not expose Python validator
registration. `physx_usd_schemas.register_validators()` is safe to call again and returns
whether the validator is available. The installed wheel includes the complete rule contract,
diagnostics, and runnable USDA examples in `physx_usd_schemas/VALIDATION.md`.

### Register individual schemas

```python
from physx_usd_schemas import PhysxSchema
from physx_usd_schemas import OmniUsdPhysicsDeformableSchema

PhysxSchema.register()
OmniUsdPhysicsDeformableSchema.register()
```

### Use the pure-Python schema API

The schema modules expose a pure-Python API that mirrors the compiled bindings:

```python
from pxr import Usd
import physx_usd_schemas  # registers all schemas on import
from physx_usd_schemas import PhysxSchema

stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")
api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
api.CreateLinearDampingAttr(0.1)
```

### Discover schema resource paths

If you need the raw `plugInfo.json` directory paths for manual registration:

```python
import physx_usd_schemas
from pxr import Plug

paths = physx_usd_schemas.schema_paths()
Plug.Registry().RegisterPlugins([str(p) for p in paths])
```
