# Physics Schemas

Physics behavior in a USD scene is described by **schemas**: typed sets of
attributes and relationships applied to prims. ovphysx loads and simulates scenes
authored with these schemas. This page explains the schema layers ovphysx
understands and how to make the PhysX-specific ones available when you author
with a stock `usd-core`.

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

ovphysx exposes the PhysX USD schemas as **codeless** artifacts: a
`plugInfo.json` (`Type: resource`) plus a `generatedSchema.usda` per module, with
no compiled library. Codeless schemas carry no typed helper class — there is no
`PhysxSchema.PhysxRigidBodyAPI` binding. You apply them by identifier and author
their attributes generically.

Because they are codeless, the core `UsdPhysics` typed APIs (rigid body,
collider, mass, scene) work out of the box with stock `usd-core`, while
`Physx*` attributes require registering the codeless schemas first.

## Making Schemas Available

There are two registration paths, depending on which USD runtime is in play.

### ovphysx's bundled USD runtime

When you run the simulator, ovphysx uses its bundled OV namespaced USD runtime.
Importing `ovphysx` automatically calls `register_schema_paths()`, which appends
ovphysx's schema plugin path to `OV_PXR_PLUGINPATH_2511` before the first USD
stage open. In a process that mixes ovphysx with another USD-aware subsystem
(for example ovrtx), call `register_schema_paths()` (and the peer subsystem's
equivalent) explicitly before the first stage open so USD's schema registry sees
every plugin root. This ordering is not advisory: USD's schema registry is built
once, on first access, and a late call cannot repair it. Refer to the USD
coexistence notes in the [Overview](ovphysx_overview.md).

### Authoring with a stock `usd-core`

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
from pxr import Sdf

prim.ApplyAPI("PhysxRigidBodyAPI")
prim.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)
```

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
host's USD runtime. It is unrelated to `OV_PXR_PLUGINPATH_2511`, which applies
only to ovphysx's bundled namespaced runtime (refer to the "ovphysx's bundled
USD runtime" section above).

## Authoring Routes

The same physics content can be authored two ways; both produce a `.usda`/`.usd`
file that ovphysx loads identically.

- **Hand-authored `.usda` text.** Apply schemas through the `apiSchemas`
  metadata list and set attributes directly. No Python or schema registration
  needed to write the file:

  ```usda
  def Cube "box" (
      prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI"]
  )
  {
      bool physxRigidBody:disableGravity = 0
  }
  ```

- **Python with a USD runtime.** Use typed `UsdPhysics` bindings for core
  schemas and the codeless `ApplyAPI` pattern above for `Physx*` schemas.

> **Common pitfall.** Using a typed `PhysxSchema.*` class instead of the codeless
> `prim.ApplyAPI("Physx...")` pattern silently no-ops with stock `usd-core` —
> there is no compiled `PhysxSchema` module. Register the codeless schemas and
> apply by identifier.

For a hands-on authoring walkthrough (scene, ground, rigid body, colliders), refer to
the bundled `ovphysx-usd-authoring` skill and the per-topic pages under
Simulation Setup.
