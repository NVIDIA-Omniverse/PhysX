---
name: ovphysx-basic-workflow
description: Create an ovphysx instance, attach an ovstage scene, step simulation, and clean up. Use when starting a new ovphysx project, testing basic integration, or learning the minimal workflow.
license: Apache-2.0
compatibility: "ovphysx build with the task-completion API; Python examples require the ovphysx Python package, and C examples require the OVPhysX SDK plus the matching native OVStage package. Python examples require Python 3.10-3.13, the versions the ovphysx wheel is validated on."
allowed-tools: Read Shell
metadata:
  version: "0.1.11"
  author: NVIDIA Omniverse Physics
  tags: "ovphysx, physics, quickstart"
---

# Basic Workflow: Attach ovstage and Step

The smallest end-to-end ovphysx workflow.

## When to Use

Use this skill when a caller needs the smallest working ovphysx program, an installation smoke test, or a Python-to-C API mapping for instance creation, ovstage attachment, stepping, and cleanup.

## Instructions

1. Read the referenced sample for the caller's language before changing code.
2. Keep the create, populate ovstage, attach, step, and destroy order shown here unless the caller authors later ovstage edits that require an update drain.
3. Use Shell to run the Python sample or build and run the C sample after adapting the paths.

These Python examples require Python 3.10-3.13, the versions the ovphysx
wheel is validated on.

For Python, install `physx-usd-schemas` and `newton-usd-schemas` alongside
`ovphysx`. Register both PhysX and Newton schemas before the first population
call so authored `newton:*` values are preserved.

Import the PhysX schema recipe from
`ovphysx.samples.python_samples.schema_support`; for native samples, set
`PHYSX_USD_SCHEMAS_ROOT` to the extracted wheel. See
[Physics Schemas](../../docs/physics_schemas.md) for setup.

## Python

```bash
pip install ovphysx physx-usd-schemas newton-usd-schemas
```

```python
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX
import ovstage
from ovphysx.population import physics_population_descs

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "simple_physics_scene.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

physx = PhysX()
# Register external PhysX and Newton schemas before the first population.
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("ovphysx-basic")
try:
    ovstage.population.open_usd_with_desc(
        stage, str(usd_path), ordinal=1, time=ovstage.population.DEFAULT_TIME,
        descs=physics_population_descs(rendering=True))
    stage.advance_write_floor(ordinal=1).wait()
    physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
    physx.step_sync(1.0 / 60.0)
finally:
    try:
        physx.wait_task(physx.detach_ovstage())
    finally:
        physx.destroy()
        stage.destroy()
```

`step_sync()` steps and waits in one call, so step errors surface immediately and
teardown cannot race the step. Use bare async `step()` + an explicit
`wait_task(task)` / `wait_all()` only when overlapping CPU work with the running
step; `step()` returns the immutable `Task` receipt that `wait_task()` takes, and an
accepted step that fails during execution raises `TaskError` from that wait.
Rejected submissions raise immediately from `step()`.

Populate USD through the physics population description with RENDERING beside it,
the compatibility path for USD that may put colliders under native scene-graph
instances. Sealing composes world transforms with or without RENDERING; see
[Populating physics](../../docs/ovstage_integration.md#populating-physics-the-physics-population-description).

### Later ovstage edits

The drain only transfers a population change that was authored first. Calling
it against an unchanged population succeeds as a no-op.

For a wheel-only application, add a USD reference at a new absolute prim path
such as `/World/Spawned`, then apply and drain it at a fresh ordinal. The file
must have a `defaultPrim` selecting the content to add. Use an asset without a
`PhysicsScene`, and do not reference onto an existing simulated hierarchy:
those structural edits can recreate existing physics objects and reset state.

Read `samples/python_samples/population_update.py` in the installed wheel
(source: `tests/python_samples/population_update.py`). Import
`add_usd_file_and_drain` from `ovphysx.samples.python_samples.population_update`
for the addition above. Its `drain_population_change` helper is also available
when the caller has already authored the change.

`add_usd_reference()` authors the backing USD change; `apply_usd_changes()`
publishes it at the supplied ordinal.
`open_usd()` replaces the backing USD source and repopulates existing paths.
Use `add_usd_reference()` for the isolated addition shown here.
Applications with their own backing-USD integration may author there instead,
then call `drain_population_change()` with the edit's ordinal.

`apply_usd_changes()` waits for population work but does not seal the ordinal.
Use a fresh ordinal for each later change. Do not drain the initial
`read_ordinal` again; `attach_ovstage()` already parsed it. The descriptions the initial
`open_usd_with_desc()` call takes are the ones later `apply_usd_changes()` calls
use.
Before a structural add or remove, destroy cached tensor bindings when practical, then
create replacements after the update completes. Do not use a pre-update
binding's membership as the test oracle.

Full sample:
- `samples/python_samples/hello_world.py` (wheel)
- Source checkout: `tests/python_samples/hello_world.py`

## C

The C workflow follows one ordering; keep it as-is:

1. `ovphysx_initialize()` (process-wide; no Python analogue).
2. `ovphysx_create_instance(&handle)` with no create-args. Set process-global
   config with `ovphysx_set_global_config` before this call if needed; set
   `OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS` with `ovphysx_set_config` after create
   and before attach.
3. Install the external schema provider, then register its two resource
   directories before population, using the sample's schema-registration helper.
4. Populate an ovstage from USD, wait for the population op, advance and wait
   for the write floor, then submit `ovphysx_attach_ovstage(handle, stage, read_ordinal)`
   and wait successfully for its task.
5. `ovphysx_step_sync()` -- steps and waits in one call, so step errors are
   reported and teardown cannot race the step.
6. Submit detach and wait successfully before destroying the Stage. If detach
   fails, retain the Stage until `ovphysx_destroy_instance()` succeeds.
7. `ovphysx_shutdown()`.

Read the SDK's `samples/c_samples/basic_workflow_c/main.c` for this explicit
public-API sequence, then follow `samples/c_samples/basic_workflow_c/README.md`
to build and launch it with the adjacent `CMakeLists.txt`. In a source checkout,
these files are under `tests/c_samples/basic_workflow_c/`.

The sample contains the complete program and build recipe. It uses the SDK's
schema-registration helper and bundled `simple_physics_scene.usda`. Read the
README for dependency setup, Windows developer-shell requirements, and the
launcher path for the selected generator before adapting the sample.
The C sample requires the SDK and is not included in the Python wheel.

If you step with the async `ovphysx_step()` instead of `ovphysx_step_sync()`,
you must then call `ovphysx_wait_task()` with the returned receipt (the whole
`ovphysx_enqueue_result_t`), a concrete timeout, and a zero-initialized
`ovphysx_wait_task_result_t`; check the returned status and `num_errors`, then
release the result with `ovphysx_destroy_wait_task_result()` before teardown —
the enqueue status alone only means *accepted*, not *completed*. The same
receipt/wait rule applies to attach, update and detach, whether they run inline
or on the instance's worker thread. `is_stage_attached()` is only a synchronous
ownership snapshot.

## Key APIs

| Python | C |
|--------|---|
| _(none — implicit)_ | `ovphysx_initialize()` / `ovphysx_shutdown()` |
| `PhysX()` | `ovphysx_create_instance()` |
| `set_global_config` / `physx.set_config` | `ovphysx_set_global_config` / `ovphysx_set_config` |
| `physx.attach_ovstage(stage)` | `ovphysx_attach_ovstage()` |
| `physx.step_sync(dt)` | `ovphysx_step_sync()` |
| `physx.step(dt)` | `ovphysx_step()` |
| `physx.destroy()` | `ovphysx_destroy_instance()` |

The C API brackets everything with `ovphysx_initialize()` / `ovphysx_shutdown()`;
Python manages that lifecycle internally, so there is no Python analogue. Use
`physx.update_from_ovstage(from_ordinal, to_ordinal)` /
`ovphysx_update_from_ovstage()` only to drain later ovstage edits after the
initial attach (not needed for this minimal workflow).

## References

- Docs: `docs/tutorials/hello_world.md`
- Python sample: `samples/python_samples/hello_world.py` (wheel; source: `tests/python_samples/hello_world.py`)
- C sample: `samples/c_samples/basic_workflow_c/main.c` (SDK; source: `tests/c_samples/basic_workflow_c/main.c`)
- C build and launch recipe: `samples/c_samples/basic_workflow_c/README.md` and `samples/c_samples/basic_workflow_c/CMakeLists.txt` (SDK; source: `tests/c_samples/basic_workflow_c/`)
- Later population edits: `samples/python_samples/population_update.py` (wheel; source: `tests/python_samples/population_update.py`)
- C header: `include/ovphysx/ovphysx.h`
