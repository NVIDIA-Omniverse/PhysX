<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# physx-usd-schemas

Codeless PhysX USD schemas for authoring and validation with a stock [OpenUSD](https://openusd.org/) runtime.

This package ships two USD schema modules as pure-Python subpackages (no compiled C++ bindings):

- **`physx_usd_schemas.PhysxSchema`** -- PhysX simulation schemas (rigid body, collision, joints, vehicles, particles, ...)
- **`physx_usd_schemas.OmniUsdPhysicsDeformableSchema`** -- Omni deformable-body simulation schemas
- **`physx_usd_schemas.PhysicsSchemaTools`** -- Schema utility helpers (e.g. `units_db`)

Importing `physx_usd_schemas` registers all bundled schemas with any stock `usd-core` runtime,
so you can author or validate physics-annotated USD files without the full Omniverse runtime.

This wheel supplies the NVIDIA physics schema extensions -- the `Physx*` API schemas and the
`OmniPhysics*` deformable schemas. The official `pxr.UsdPhysics` core physics schemas come from
`usd-core` (OpenUSD), which is why `usd-core` is required alongside this wheel.

## Installation

```
pip install physx-usd-schemas usd-core
```

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
