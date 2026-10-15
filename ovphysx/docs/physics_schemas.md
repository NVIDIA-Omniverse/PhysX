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
  (refer to [Deformables](simulation_setup/deformables.md)), and more. The external provider supplies
  this schema as codeless artifacts (`plugInfo.json` plus `generatedSchema.usda`,
  no compiled library).
- **Omni Physics Deformable Schema (codeless)** — extends `UsdPhysics` with
  deformable bodies (`OmniPhysicsDeformableBodyAPI` and friends). Refer to
  [Deformables](simulation_setup/deformables.md).
- **Newton Schema (`newton:*`, not shipped)** — the
  [Newton USD schema](https://github.com/newton-physics/newton-usd-schemas), a
  subset of whose attributes ovphysx reads as fallbacks for the PhysX spellings.
  Install and register it yourself; refer to
  [Registering the Newton USD schema](#registering-the-newton-usd-schema).

The complete attribute set for each schema — types, defaults, and allowed values
— is authoritative in the schema definitions themselves and is rendered in the
[Omni Physics documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
and the [OpenUSD UsdPhysics reference](https://openusd.org/release/api/usd_physics_page_front.html).
This guide describes intent, not full attribute definitions, so the two do not
drift.

## Installing the External PhysX Schemas

Applications install **`physx-usd-schemas`** separately from ovphysx.
The same platform-independent wheel supplies Python and native applications on
Linux x86_64/aarch64 and Windows x64. The SDK and ovphysx wheel contain no schema
resources and expose no PhysX schema-location API. OVPhysX builds require neither the
`usd_ext_physics` Packman package nor generated PhysX schema headers.

For Python, install the provider from PyPI:

```bash
pip install physx-usd-schemas
```

The application-side `schema_support/` recipe shipped with the Python samples
uses `importlib.metadata.distribution("physx-usd-schemas")` and locates both
modules' `resources/` directories. It verifies
`plugInfo.json` and `generatedSchema.usda` before registration and reports how to
repair a missing or incomplete installation. It does not import the
provider: that package's top-level import registers with stock USD and needs
`pxr`. Metadata discovery loads neither USD nor the native ovphysx library.
Copy `schema_support/` alongside a copied sample, or use its installed sample
namespace as the examples below do. Applications can adapt this recipe to their
deployment; it is not a core ovphysx API.

For native applications, unpack the same wheel as an external data archive.
Examples including `common/ovstage_sample.h` require the SDK
`samples/c_samples/` directory on the compiler include path, or a copy of its
`common/` directory alongside the application sources.
The SDK's sample installer reads the artifact URL and SHA-256 from the adjacent
`common/physx_schemas.lock.json`, verifies the digest, and keeps the wheel's
distribution metadata:

```bash
python /path/to/sdk/samples/c_samples/common/fetch_physx_schemas.py --dest /path/to/app-deps/physx-schemas
export PHYSX_USD_SCHEMAS_ROOT=/path/to/app-deps/physx-schemas
```

On Windows, set `$env:PHYSX_USD_SCHEMAS_ROOT` to that extracted directory.
An offline installation can pass `--wheel /path/to/physx_usd_schemas-<version>-py3-none-any.whl`.
Python is needed only for this installation step; native schema discovery and
registration use no Python runtime. The directory contains:

```text
physx_usd_schemas-<version>.dist-info/METADATA
physx_usd_schemas/PhysxSchema/resources/{plugInfo.json,generatedSchema.usda}
physx_usd_schemas/OmniUsdPhysicsDeformableSchema/resources/{plugInfo.json,generatedSchema.usda}
```

Keep the provider outside the ovphysx SDK. After relocating the application and
provider, resolve its new directory and update `PHYSX_USD_SCHEMAS_ROOT`; Python
resolves the provider from its current installation metadata. `OVPHYSX_LIB`
selects the simulation library and never selects schemas.

Version selection follows the same policy as the Newton schema dependency:
Python applications declare compatibility constraints in `pyproject.toml` and
record the resolved version and hashes in `uv.lock`. The native sample installer
uses its artifact lock file for reproducible provisioning. Discovery checks the
required resource layout without enforcing a release number. Dependency
resolution reports unsatisfied version constraints; missing modules or resource
files are reported before registration. Resource discovery alone does not prove
semantic compatibility with every schema release.

ovphysx leaves schema registration to the application and never changes USD
plugin-path variables. The application registers the provider's resources with
its chosen USD runtime. The native attach checks for registration as described
under [Attach-time registration guard](#attach-time-registration-guard).

## Developing Schemas Locally

`--devschema` keeps the local schema development workflow. It builds the schema
project's own `physx-usd-schemas` wheel from locally generated data and extracts
a separate provider under the selected build directory, by default
`ovphysx/_build/test-deps/physx-usd-schemas-dev/<wheel-sha256>/`. The SDK and
ovphysx wheel still contain no schema resources.

From `ovphysx/`, select the local wheel with:

```bash
./build.sh --devschema
cmake -DDEV_SCHEMA=ON -P scripts/validate_all.cmake
```

On Windows use `build.bat --devschema`. Each devschema build regenerates the
local schema data before packaging the wheel. Switching this option changes no
native compiler inputs and requires no clean rebuild.

The build records the provider directory in `ovphysx_dev_schema.stamp` under
the selected build directory (`_build` by default). For a custom directory,
pass `-DBUILD_DIR=<path>` to CMake scripts or set `OVPHYSX_BUILD_DIR` for scripts
and `run_pytest.sh`/`.bat`; relative paths are resolved from `ovphysx/`.
Native test/sample/benchmark runners use it as `PHYSX_USD_SCHEMAS_ROOT`. Python
runtime, sample, and benchmark runners, including `run_pytest.sh`/`.bat`, prepend
it to `PYTHONPATH`, so metadata discovery and authoring utilities see the local
data even when a published wheel is installed in the test environment. A
missing local provider fails with a rebuild instruction instead of falling back
to published schemas. Rebuilding after local edits selects a new digest-named
provider even when the schema version has not changed. Normal builds clear the
selection and use the published provider again.

For an application launched outside these runners, install the separately built
schema wheel from `schemas/physx/_dist/`, or use the directory recorded in the
stamp: set `PHYSX_USD_SCHEMAS_ROOT` for a native application, or prepend that
directory to `PYTHONPATH` for Python. Both routes keep schema resources outside
ovphysx. Local providers retain package-identity, digest, and completeness checks
without being forced to match the published artifact's version. Applications
maintain their supported-version constraints and locks in dependency configuration.

## Making Schemas Available

The application owns the USD runtime(s) in its process and registers the
codeless schemas with each of them. There are two registration paths, depending
on which USD runtime is in play.

### Registering with ovstage

ovstage ingests USD scenes through its own internal namespaced OpenUSD runtime,
and that runtime knows nothing about the PhysX schemas until the application
registers them. Register them in each process that reads USD schemas, including
spawned workers, before the first schema read. Population (`open_usd` or
`apply_usd_changes`) and USD export both read and freeze the schema definitions.
Pass both provider resource directories to
`ovstage.population.register_usd_schemas()`:

```python
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage

ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("scene")
```

In C, the sample's `common/schema_support.h` validates the external root and
required resources. `common/ovstage_sample.h` registers both returned directories before
population and reports provider or registration errors:

```c
#include "common/ovstage_sample.h"

static int prepare_schema_registry(void)
{
    return ovphysx_sample_register_physx_schemas();
}
```

Call this before the first USD schema read, including population or export.
The native samples already call it from
`ovphysx_sample_attach_usd_with_ovstage()`. The helper is application code;
there is no schema-location call into libovphysx.

The ordering is not advisory. USD assembles its schema registry once, on first
read, and ignores plugins registered afterwards; a registration that arrives
after ovstage's first schema read registers cleanly but contributes nothing,
and ovstage reports it as an error (`OVSTAGE_ERROR_OP_FAILED`, an
`ovstage.OvstageError` in Python). A registration that arrives after some other
USD consumer in the process read the schemas cannot be detected and fails
silently. No re-registration can repair the registry for the rest of the
process. Without registration, population cannot resolve PhysX APIs and drops
their attributes, including authored values and schema fallbacks.
Registration is process-scoped and irreversible; registering the same root
twice is a no-op. Restart the process after a late registration and register
before populating or exporting again; creating another Stage or PhysX instance
is not enough.

### Attach-time registration guard

By default, native `ovphysx_attach_ovstage()` checks for missing plugin families,
`physxSchema` and `omniUsdPhysicsDeformableSchema`, through ovstage after its USD
schema registry has been built. If ovstage reports either family missing, attach
returns an `OVPHYSX_API_ERROR` admission receipt with no task identity. Python's
`PhysX.attach_ovstage()` raises `RuntimeError` before returning a Task. The guard
takes no Stage ownership and does not alter an existing attachment.

This refusal persists for the rest of the process because USD cannot rebuild
the registry. Restart and register both external resource directories before
populating or exporting. A late registration or a new instance does not recover
dropped PhysX settings. The check recognizes the registered plugin families regardless
of the provider's directory; it does not locate or install the provider.

A procedural Stage attached while USD's schema definitions are still unbuilt
skips the family check and leaves it active for later attaches. Population and
export both build the registry. After either operation, the check applies to
every attach in the process, including procedural Stages.
It cannot detect schema registration that another USD consumer made
unobservable to ovstage.

The guard also cannot identify a schema family registered late before its first
check. For example, populating without the schemas, then calling
`register_usd_schemas()` and catching its `OVSTAGE_ERROR_OP_FAILED` error can
make a later attach succeed even though population lost the schema data.
Always restart after a registration-ordering error; a successful attach is not
proof that the schemas were available during population.

The legacy Carbonite setting `/ovphysx/schemas/requireRegistration` defaults to
`true`. Set it to `false` at instance creation only when accepting this missing
registration, for example in a host whose registration ovstage cannot observe:

```python
from ovphysx import PhysX, PhysXConfig

physx = PhysX(config=PhysXConfig(carbonite_overrides={
    "/ovphysx/schemas/requireRegistration": False,
}))
```

This downgrades the refusal to a warning and permits attachment; it does not
register schemas or repair lost data. Native callers use the same key in the
legacy create-time Carbonite configuration. There is no scoped `GlobalConfigKey`
for this setting.

### Registering the Newton USD schema

ovphysx also reads a subset of the
[Newton USD schema](https://github.com/newton-physics/newton-usd-schemas)
(`NewtonJointAPI`, `NewtonArticulationRootAPI`, `NewtonSceneAPI`, ...).
`NewtonJointAPI` sets the velocity limit, armature, friction and damping for all
degrees of freedom of a joint. `PhysxJointAxisAPI` values take precedence. A
per-axis `maxJointVelocity` left unlimited (`inf` or `FLT_MAX`) counts as no PhysX
opinion and yields to an authored Newton limit; the per-attribute contract is in
[PhysxJointAxisAPI](population/PhysxJointAxisAPI.md). ovphysx does not ship this
schema. It is a separate package that the application installs and registers, in
the same call and under the same ordering rule as the PhysX schemas:

```bash
pip install newton-usd-schemas
```

```python
import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage

ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
```

The bundled Python samples register it this way; their project file declares
`newton-usd-schemas`, so `uv run` resolves it, and a plain `pip install ovphysx`
environment needs `pip install newton-usd-schemas` before running them.
`ovphysx.newton_schema_root()` returns the directory of the installed
`newton-usd-schemas` package without importing it (importing it requires `pxr`
and registers the schema with that stock USD runtime instead), and raises
`FileNotFoundError` with the install hint when the package is missing. Without
the pip package, download the schema from its
[GitHub repository](https://github.com/newton-physics/newton-usd-schemas) and pass
the directory holding its `plugInfo.json` to `register_usd_schemas()` (in C,
`ovstage_population_register_usd_schemas()`) yourself.

ovstage populates an attribute only when a registered schema defines it, so a
scene that authors `newton:*` attributes without this registration loses them
silently: a joint authoring only `newton:velocityLimit` simulates unlimited.
`PhysX.attach_ovstage()` therefore checks registration of the installed
`newton-usd-schemas` package once per process before the native attach and emits
a `RuntimeWarning` if ovstage reports late registration. When the
package is not installed at all it logs a warning on the `ovphysx` Python logger
instead: nothing is wrong for a scene without `newton:*` attributes, and a suite
that promotes warnings to errors keeps running. ovstage keys registration on the plugin
family, so a complete copy of the Newton schema registered from any directory
(a GitHub checkout) is recognized. The check probes only once USD has built its
schema definitions, whether through population or export; a stage authored
procedurally through `ovphysx.population` and attached while those definitions
remain unbuilt is left alone, so the check never registers a schema on the
application's behalf. A registration ovstage cannot observe (another USD
consumer in the process read
the schema definitions first) is not detected; silence the check in that case
with the legacy `PhysXConfig(carbonite_overrides={"/ovphysx/schemas/warnMissingNewtonSchema": False})`
(the string `"false"` is accepted too). There is no scoped `GlobalConfigKey` for this check. The C API cannot locate the package and
performs no such check.

This Newton warning is separate from the native PhysX registration guard above:
it does not check either PhysX schema family, and silencing it does not bypass
that guard.

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

The separately installed codeless PhysX USD schemas can be registered with any USD
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
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from pxr import Plug, Sdf, Usd, UsdPhysics

Plug.Registry().RegisterPlugins(
    [str(path) for path in physx_schema_paths()]
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

from ovphysx.samples.python_samples.schema_support import physx_schema_paths

env = dict(os.environ)
# Append rather than replace: the variable may already carry plugin roots that
# the host or another package needs.
entries = [str(path) for path in physx_schema_paths()]
existing = env.get("PXR_PLUGINPATH_NAME")
if existing:
    entries.append(existing)
env["PXR_PLUGINPATH_NAME"] = os.pathsep.join(entries)
subprocess.run(["blender"], env=env)
```

or from a shell, likewise preserving any existing value:

```bash
SCHEMA_PATHS="$(python -c 'import os; from ovphysx.samples.python_samples.schema_support import physx_schema_paths; print(os.pathsep.join(str(p) for p in physx_schema_paths()))')"
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
