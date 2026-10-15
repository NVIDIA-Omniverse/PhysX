---
name: basic-workflow
description: Create an ovphysx instance, attach an ovstage scene, step simulation, and clean up. Use when starting a new ovphysx project, testing basic integration, or learning the minimal workflow.
license: Apache-2.0
compatibility: "ovphysx build with the task-completion API; Python examples require the ovphysx Python package, and C examples require the OVPhysX SDK plus the matching native OVStage package."
allowed-tools: Read Shell
metadata:
  version: "0.1.9"
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
    ovstage.population.open_usd(stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL)
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

Use `ALL` when populating USD so nested world transforms are resolved by the
rendering domain. `PHYSICS` alone seeds local world matrices in OVStage 0.3;
see [Population domains](../../docs/ovstage_integration.md#population-domains).

### Later ovstage edits

The drain only transfers a population change that was authored first. Calling
it against an unchanged population succeeds as a no-op.

For a wheel-only application, add a USD reference at a new absolute prim path
such as `/World/Spawned`, then apply and drain it at a fresh ordinal. The file
must have a `defaultPrim` selecting the content to add. Use an asset without a
`PhysicsScene`, and do not reference onto an existing simulated hierarchy:
those structural edits can recreate existing physics objects and reset state.

```python
import ovstage


def drain_population_change(stage, physx, ordinal):
    ovstage.population.apply_usd_changes(stage, ordinal=ordinal)
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))


def add_usd_file_and_drain(stage, physx, usd_path, target_path, ordinal):
    ovstage.population.add_usd_reference(stage, str(usd_path), target_path)
    drain_population_change(stage, physx, ordinal)
```

`add_usd_reference()` authors the backing USD change; `apply_usd_changes()`
publishes it at the supplied ordinal.
`open_usd()` replaces the backing USD source and repopulates existing paths.
Use `add_usd_reference()` for the isolated addition shown here.
Applications with their own backing-USD integration may author there instead,
then call `drain_population_change()` with the edit's ordinal.

`apply_usd_changes()` waits for population work but does not seal the ordinal.
Use a fresh ordinal for each later change. Do not drain the initial
`read_ordinal` again; `attach_ovstage()` already parsed it. The initial `open_usd()`
call with `PopulationDomain.ALL` selects the domains used by later
`apply_usd_changes()` calls, including rendering for composed world transforms.
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
   directories before population, using the application sample helper below.
4. Populate an ovstage from USD, wait for the population op, advance and wait
   for the write floor, then submit `ovphysx_attach_ovstage(handle, stage, read_ordinal)`
   and wait successfully for its task.
5. `ovphysx_step_sync()` -- steps and waits in one call, so step errors are
   reported and teardown cannot race the step.
6. Submit detach and wait successfully before destroying the Stage. If detach
   fails, retain the Stage until `ovphysx_destroy_instance()` succeeds.
7. `ovphysx_shutdown()`.

This program uses public ovphysx / ovstage API and the SDK's schema-registration
helper. Follow the build and launch steps below. Run it from the extracted SDK
root so the bundled `samples/data/simple_physics_scene.usda` path resolves, or
replace that relative path with its absolute location:

```c
#include <ovphysx/ovphysx.h>
#include "common/ovstage_sample.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <stdio.h>
#include <string.h>

static int complete_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS)
        return 0;
    ovphysx_wait_task_result_t waited = {0};
    ovphysx_result_t result =
        ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    for (size_t i = 0; i < waited.num_errors; ++i) {
        ovphysx_string_t error = ovphysx_get_last_task_error(handle, waited.error_tasks[i]);
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
    }
    ovphysx_destroy_wait_task_result(&waited);
    return result.status == OVPHYSX_API_SUCCESS;
}

int main(void)
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to initialize ovphysx\n");
        return 1;
    }

    ovphysx_instance_handle_t handle = 0;
    if (ovphysx_create_instance(&handle).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to create instance\n");
        ovphysx_shutdown();
        return 1;
    }

    if (!ovphysx_sample_register_physx_schemas()) {
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    // Create an ovstage and populate it from USD (public ovstage API).
    ovstage_instance_desc_t stage_desc;
    memset(&stage_desc, 0, sizeof(stage_desc));
    stage_desc.name = "ovphysx-basic";
    ovstage_instance_t* stage = NULL;
    if (ovstage_create_instance(&stage_desc, &stage) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to create ovstage\n");
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    const char* usd_path = "samples/data/simple_physics_scene.usda";
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strnlen(usd_path, 4096);

    const uint64_t ordinal = 1;
    ovstage_population_enqueue_result_t open = ovstage_population_open_usd_from_file(
        stage, path, ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
    ovstage_population_op_wait_result_t open_wait;
    memset(&open_wait, 0, sizeof(open_wait));
    if (open.status != OVSTAGE_OK ||
        ovstage_population_wait_op(stage, open.op_index, OVSTAGE_TIMEOUT_INFINITE, &open_wait) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to populate ovstage\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    ovstage_write_floor_desc_t floor_desc;
    memset(&floor_desc, 0, sizeof(floor_desc));
    floor_desc.ordinal = ordinal;
    floor_desc.scope = OVSTAGE_SCOPE_ALL;
    ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floor_desc);
    ovstage_op_wait_result_t floor_wait;
    memset(&floor_wait, 0, sizeof(floor_wait));
    if (floor.status != OVSTAGE_OK ||
        ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait) != OVSTAGE_OK ||
        floor_wait.error_op_id_count != 0) {
        fprintf(stderr, "Failed to seal ovstage ordinal\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to release ovstage floor operation\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    if (!complete_task(handle, ovphysx_attach_ovstage(handle, stage, ordinal))) {
        fprintf(stderr, "Failed to attach ovstage\n");
        // Failed attach can retain ownership if its rollback fails.
        if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
            return 1;
        ovstage_destroy_instance(stage);
        ovphysx_shutdown();
        return 1;
    }

    // step_sync steps and waits; a nonzero status means the step failed.
    if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Simulation step failed\n");
        if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
            return 1;
        ovstage_destroy_instance(stage);
        ovphysx_shutdown();
        return 1;
    }

    int detached = complete_task(handle, ovphysx_detach_ovstage(handle));
    if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
        return 1;
    ovstage_destroy_instance(stage);
    ovphysx_shutdown();
    return detached ? 0 : 1;
}
```

If you step with the async `ovphysx_step()` instead of `ovphysx_step_sync()`,
you must then call `ovphysx_wait_task()` with the returned receipt (the whole
`ovphysx_enqueue_result_t`), a concrete timeout, and a zero-initialized
`ovphysx_wait_task_result_t`; check the returned status and `num_errors`, then
release the result with `ovphysx_destroy_wait_task_result()` before teardown —
the enqueue status alone only means *accepted*, not *completed*. The same
receipt/wait rule applies to attach, update and detach, even while they execute
inline. `is_stage_attached()` is only a synchronous ownership snapshot.

Full sample:
- `samples/c_samples/hello_world_c/main.c` (SDK)
- Source checkout: `tests/c_samples/hello_world_c/main.c`

The tested sample keeps its `main()` short by factoring the populate/attach and
stage teardown into local helpers (`ovphysx_sample_attach_usd_with_ovstage()` /
`ovphysx_sample_destroy_stage()` in `tests/c_samples/common/ovstage_sample.h`).
Those helpers are sample scaffolding, not public API — the sequence above is what
they wrap.

### Build and run the C sample

Save the program as `main.c` in the extracted SDK root. Copy
`samples/c_samples/common/` beside it, and save the following as `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(ovphysx_example LANGUAGES C)

find_package(ovphysx REQUIRED)
add_executable(my_app main.c)
target_link_libraries(my_app PRIVATE ovphysx::ovphysx ovphysx::ovstage)
if(WIN32)
    ovphysx_copy_runtime_dlls(my_app)
    ovphysx_set_ovstage_runtime_env(my_app)
endif()
```

Configure and build from the SDK root. Use `build-my-app` to keep this project
separate from the SDK README's `build` directory. For Ninja, add `-G Ninja`.
On Windows, use Visual Studio Developer PowerShell configured for x64 with
`-Arch amd64 -HostArch amd64` so the compiler, headers, and libraries are available;
see Microsoft's [developer shell setup](https://learn.microsoft.com/en-us/visualstudio/ide/reference/command-prompt-powershell#target-architecture-and-host-architecture).
Use a fresh build directory when changing generators:

```bash
cmake -B build-my-app -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"
cmake --build build-my-app --config Release
```

Run from the SDK root so `samples/data/simple_physics_scene.usda` resolves.
Set `PHYSX_USD_SCHEMAS_ROOT` to the extracted `physx-usd-schemas` wheel directory
that contains `physx_usd_schemas/`.

Linux:

```bash
export PHYSX_USD_SCHEMAS_ROOT="/path/to/physx-schemas"
./build-my-app/my_app
```

Windows PowerShell: set the variable in the shell that runs the launcher:

```powershell
$env:PHYSX_USD_SCHEMAS_ROOT = "C:\path\to\physx-schemas"
```

- Windows, Visual Studio generator: `.\build-my-app\Release\run_my_app.cmd`.
- Windows, Ninja generator: `.\build-my-app\run_my_app.cmd`.

On Windows, `ovphysx_copy_runtime_dlls()` copies the ovphysx DLLs and plugins
beside the executable. `ovphysx_set_ovstage_runtime_env()` generates
`run_my_app.cmd`, which adds
the resolved OVStage runtime directories to `PATH` before starting the program.
Use this launcher from a shell. Keep OVStage in its own package so it can find
its plugins.

`find_package(ovphysx)` uses a compatible OVStage package from `CMAKE_PREFIX_PATH`.
If none is available, it downloads and caches the pinned package; this requires
Python 3.8+ and network access on the first configure. To use that bootstrap,
set `CMAKE_PREFIX_PATH` to the OVPhysX SDK root alone. For offline setup, supply
OVStage separately and add `-DOVPHYSX_FETCH_OVSTAGE=OFF`.

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
- C sample: `samples/c_samples/hello_world_c/main.c` (SDK; source: `tests/c_samples/hello_world_c/main.c`)
- C header: `include/ovphysx/ovphysx.h`
