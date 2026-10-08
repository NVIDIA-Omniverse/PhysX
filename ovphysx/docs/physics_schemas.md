<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Physics Schemas

Physics behavior in a USD scene is described by **schemas**: typed sets of
attributes and relationships applied to prims. ovstage populates scenes authored
with these schemas into an `ovstage.Stage`; ovphysx attaches that stage and
simulates it. This page explains the schema layers ovphysx understands and how
to make the PhysX-specific ones available, both to ovstage when it populates a
scene and to a stock `usd-core` when you author or validate offline.

## Schema Layers

- **USD Physics Schema (`UsdPhysics`)** — the standard OpenUSD schema for
  annotating assets with physics: `PhysicsScene`, `PhysicsRigidBodyAPI`,
  `PhysicsCollisionAPI`, `PhysicsMassAPI`, `PhysicsMaterialAPI`, the joint types,
  and articulation APIs. It ships with stock `usd-core` and has typed Python
  bindings, for example `UsdPhysics.RigidBodyAPI.Apply(prim)`. Refer to the
  [OpenUSD UsdPhysics reference](https://openusd.org/release/api/usd_physics_page_front.html).
- **PhysX Schema (`Physx*`, codeless)** — extends `UsdPhysics` with PhysX-specific
  functionality and tuning: `PhysxSceneAPI` (solver, GPU settings),
  `PhysxRigidBodyAPI` (CCD, sleep, stabilization), `PhysxCollisionAPI` (contact
  and rest offsets), `PhysxForceAPI`, joint extensions such as
  `PhysxJointAxisAPI` (refer to
  [Joints](simulation_setup/joints.md#physx-joint-schema)), deformable extensions
  (refer to [Deformables](simulation_setup/deformables.md)), and more. ovphysx ships
  this schema as codeless artifacts (`plugInfo.json` plus `generatedSchema.usda`,
  no compiled library).
- **Omni Physics Deformable Schema (codeless)** — extends `UsdPhysics` with
  deformable bodies (`OmniPhysicsDeformableBodyAPI` and friends). Refer to
  [Deformables](simulation_setup/deformables.md).

The complete attribute set for each schema — types, defaults, and allowed values
— is authoritative in the schema definitions themselves and is rendered in the
[Omni Physics documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
and the [OpenUSD UsdPhysics reference](https://openusd.org/release/api/usd_physics_page_front.html).
This guide describes intent, not full attribute definitions, so the two do not
drift.

## How ovphysx Ships the PhysX Schemas

ovphysx exposes the PhysX USD schemas (`PhysxSchema` and
`OmniUsdPhysicsDeformableSchema`) as **codeless** artifacts: a root
`plugInfo.json` (with `Includes: ["*/resources/"]`) plus, per module, a
`<Module>/resources/` directory holding a `plugInfo.json` (`Type: resource`) and
a `generatedSchema.usda`, with no compiled library. They live under
`schemas/physx/` in the SDK and under `ovphysx/schemas/physx/` in the wheel.
Codeless schemas carry no typed helper class — there is no
`PhysxSchema.PhysxRigidBodyAPI` binding. You apply them by identifier and author
their attributes generically.

ovphysx never modifies the environment (`PXR_PLUGINPATH_NAME` or any other
plugin-path variable) and leaves registration to the application; it verifies at
`attach_ovstage` that the registration happened before the first population and
refuses the attach otherwise. It tells the application where the schemas are:

- **C**: `ovphysx_get_codeless_schema_root(ovphysx_string_t* out_root)` returns
  the schema root directory. The string is NUL-terminated, owned by ovphysx, and
  valid until the calling thread calls the function again. It returns
  `OVPHYSX_API_INVALID_ARGUMENT` for a NULL argument and `OVPHYSX_API_ERROR`
  (details in `ovphysx_get_last_error()`) when the schema tree is missing. It
  does not initialize ovphysx, load USD, acquire Carbonite, or modify the
  environment, and is safe to call before `ovphysx_create_instance()`.
- **Python**: `ovphysx.codeless_schema_root()` returns the same root as a
  `pathlib.Path`, and `ovphysx.codeless_schema_paths()` returns the per-module
  `resources` directories. Both are pure Python and never trigger native
  loading.

Because they are codeless, the core `UsdPhysics` typed APIs (rigid body,
collider, mass, scene) work out of the box with stock `usd-core`, while
`Physx*` attributes require registering the codeless schemas first.

## Making Schemas Available

The application owns the USD runtime(s) in its process and registers the
codeless schemas with each of them. There are two registration paths, depending
on which USD runtime is in play.

### Registering with ovstage

ovstage ingests USD scenes through its own internal namespaced OpenUSD runtime,
and that runtime knows nothing about the PhysX schemas until the application
registers them. Register them once per process, before the first population call
(`open_usd`, `apply_usd_changes`, or an export), by passing the schema root to
`ovstage.population.register_usd_schemas()`:

```python
import ovphysx
import ovstage

ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
stage = ovstage.Stage("scene")
```

In C, obtain the root with `ovphysx_get_codeless_schema_root()` and pass it to
`ovstage_population_register_usd_schemas()`:

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

static int register_physx_schemas(void)
{
    ovphysx_string_t root;
    if (ovphysx_get_codeless_schema_root(&root).status != OVPHYSX_API_SUCCESS)
    {
        return 0;
    }
    ovx_string_t path;
    path.ptr = root.ptr;
    path.length = root.length;
    return ovstage_population_register_usd_schemas(&path, 1) == OVSTAGE_OK;
}
```

The bundled samples do exactly this: the Python samples call
`register_usd_schemas()` in their `attach_scene` helper right before creating
the `ovstage.Stage`, and the C samples call
`ovphysx_sample_register_physx_schemas()` from
`tests/c_samples/common/ovstage_sample.h`.

The ordering is not advisory. USD assembles its schema registry once, on first
read, and ignores plugins registered afterwards; a registration that arrives
after ovstage's first schema read registers cleanly but contributes nothing,
and ovstage reports it as an error (`OVSTAGE_ERROR_OP_FAILED`, an
`ovstage.OvstageError` in Python). A registration that arrives after some other
USD consumer in the process read the schemas cannot be detected and fails
silently. No re-registration can repair the registry for the rest of the
process. Without the registration, population resolves only the properties
authored in the file rather than each prim's full schema-declared property set.
Registration is process-scoped and irreversible; registering the same root
twice is a no-op.

### Authoring with a stock `usd-core`

For a standalone Python authoring or validation tool, install the optional
stock USD package in that tool's environment before running an example that
imports `pxr`:

```bash
python -m pip install usd-core
```

`usd-core` is owned by the authoring tool. It is not an ovphysx package
dependency or the simulator's USD runtime, and ovphysx does not accept the
authoring tool's USD runtime directly. The public ingestion path is authored
USD -> ovstage population into an `ovstage.Stage` -> ovphysx attachment and
simulation.

PyPI currently provides neither a Linux aarch64 `usd-core` wheel nor a source
distribution. On Linux aarch64, hand-author `.usda`, author on a supported
host, or supply a compatible OpenUSD Python build.

The codeless PhysX USD schemas shipped in ovphysx can be registered with any USD
runtime — including stock `usd-core` from PyPI — without starting the simulator.
This is useful for offline authoring, validation, and non-interactive tooling:

```{literalinclude} ../tests/python_samples_extra/codeless_schemas/register_codeless_schemas.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

> **`RegisterPlugins()` must run before the process touches USD, and fails
> silently if it does not.** USD builds its *schema* registry lazily on first
> access and never rebuilds it. `Plug.Registry().RegisterPlugins()` populates
> the *plugin* registry — a different structure — and nothing propagates from
> there into an already-built schema registry. If **anything** in the process
> opened a stage or queried the schema registry first, registration cannot
> repair it, and no re-registration or explicit `plugin.Load()` recovers.
>
> The failure gives no warning: the registered-plugin count is still correct,
> `Tf.Type.FindByName("PhysxSchemaPhysxRigidBodyAPI")` still resolves, and the
> plugin still reports `isLoaded=True`. Only `ApplyAPI` fails, with
> `Tf.ErrorException: ApplyAPI: Cannot find a valid schema for the provided
> schema identifier 'PhysxRigidBodyAPI'`. There is no supported way to detect
> the state beforehand.
>
> The recipe above is therefore for processes you control from the first line —
> offline authoring, validation, and non-interactive tooling. If USD may
> already be initialised, use `PXR_PLUGINPATH_NAME` instead (below).

After registration, apply the PhysX APIs by identifier and set attributes with
USD's generic attribute API:

```python
import ovphysx
from pxr import Plug, Sdf, Usd, UsdPhysics

Plug.Registry().RegisterPlugins(
    [str(path) for path in ovphysx.codeless_schema_paths()]
)
stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")
UsdPhysics.RigidBodyAPI.Apply(prim)
UsdPhysics.CollisionAPI.Apply(prim)
if not prim.ApplyAPI("PhysxRigidBodyAPI"):
    raise RuntimeError("Failed to apply PhysxRigidBodyAPI")
attribute = prim.CreateAttribute(
    "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
)
if not attribute.Set(True):
    raise RuntimeError("Failed to set physxRigidBody:disableGravity")
if not stage.GetRootLayer().Export("physx_rigid_body.usda"):
    raise RuntimeError("Failed to export physx_rigid_body.usda")
```

This writes `physx_rigid_body.usda`, ready for ovstage to populate for ovphysx
simulation.

`codeless_schema_paths()` returns the per-module `resources` directories, and
`codeless_schema_root()` returns the directory that holds them. Both are
pure-Python and never trigger native loading, so they are safe to use in an
authoring-only process.

#### Host processes that already initialised USD

Inside a DCC host — Blender, Maya, Houdini, or any application embedding
`usd-core` — you cannot assume USD is untouched. A host **is** a USD
application: one File > Import > USD or File > Export > USD, or any other addon
that imports `pxr`, builds the schema registry before your code runs. The
`RegisterPlugins()` recipe above is then already too late, for the rest of the
session.

For these processes, point `PXR_PLUGINPATH_NAME` at the codeless `resources`
directories **before the host process launches**. USD reads that variable while
*constructing* the registry, which is the only moment early enough, and no
`RegisterPlugins()` call is needed at all:

```python
# Compute the paths in a launcher process, then start the host with them set.
import os
import subprocess

import ovphysx

env = dict(os.environ)
# Append rather than replace: the variable may already carry plugin roots that
# the host or another package needs.
entries = [str(path) for path in ovphysx.codeless_schema_paths()]
existing = env.get("PXR_PLUGINPATH_NAME")
if existing:
    entries.append(existing)
env["PXR_PLUGINPATH_NAME"] = os.pathsep.join(entries)
subprocess.run(["blender"], env=env)
```

or from a shell, likewise preserving any existing value:

```bash
SCHEMA_PATHS="$(python -c 'import os, ovphysx; print(os.pathsep.join(str(p) for p in ovphysx.codeless_schema_paths()))')"
export PXR_PLUGINPATH_NAME="$SCHEMA_PATHS${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"
blender
```

Setting `PXR_PLUGINPATH_NAME` from *inside* an already-running host has no
effect — by then the registry exists.

`PXR_PLUGINPATH_NAME` is stock USD's own plugin-path variable, read by the
host's USD runtime; ovphysx never sets it. It is not how the schemas reach
ovstage: register them with ovstage through `register_usd_schemas()` as
described in [Registering with ovstage](#registering-with-ovstage).

## Authoring Routes

The same physics content can be authored two ways; both produce a `.usda`/`.usd`
file that ovstage can populate for ovphysx simulation.

- **Hand-authored `.usda` text.** Apply schemas through the `apiSchemas`
  metadata list and set attributes directly. No Python or schema registration
  needed to write the file:

  ```usda
  #usda 1.0

  def Cube "box" (
      prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI"]
  )
  {
      bool physxRigidBody:disableGravity = 0
  }
  ```

- **Python with a USD runtime.** Use typed `UsdPhysics` bindings for core
  schemas and the codeless `ApplyAPI` pattern above for `Physx*` schemas.

> **Common pitfall.** `PhysxSchema.PhysxRigidBodyAPI` is unavailable with stock
> `usd-core`; there is no compiled `PhysxSchema` module. Register the codeless
> schemas, then apply `prim.ApplyAPI("PhysxRigidBodyAPI")` by identifier.

For a hands-on authoring walkthrough (scene, ground, rigid body, colliders), refer to
the bundled `ovphysx-usd-authoring` skill and the per-topic pages under
Simulation Setup.
