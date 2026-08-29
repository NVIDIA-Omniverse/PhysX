---
name: basic-workflow
description: Create an ovphysx instance, attach an ovstage scene, step simulation, and clean up. Use when starting a new ovphysx project, testing basic integration, or learning the minimal workflow.
compatibility: "ovphysx 0.4 wheel or SDK; Python examples require the ovphysx Python package, and C examples require the OVPhysX SDK plus the matching native OVStage package."
allowed-tools: Read Shell
metadata:
  version: "0.1.1"
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
3. Use Shell to run the Python sample or compile the C sample after adapting the paths.

## Python

```bash
pip install ovphysx
```

```python
from ovphysx import PhysX
import ovstage

physx = PhysX()
stage = ovstage.Stage("ovphysx-basic")
try:
    ovstage.population.open_usd(stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
    stage.advance_write_floor(ordinal=1).wait()
    physx.attach_ovstage(stage, read_ordinal=1)
    physx.step_sync(1.0 / 60.0)
finally:
    physx.detach_ovstage()
    stage.destroy()
    physx.release()
```

`step_sync()` steps and waits in one call, so step errors surface immediately and
teardown cannot race the step. Use bare async `step()` + an explicit
`wait_op()` / `wait_all()` only when overlapping CPU work with the running step.

The physics-only `domains` mask above is fine for this skill's non-instanced
sample USD. For arbitrary content prefer `ALL` (see
`docs/ovstage_integration.md`, "Population domains") -- `PHYSICS` alone can
silently omit colliders under native USD scene-graph instances.

### Later ovstage edits

When a running test changes the USD population after attach, use this exact
handoff:

```python
import ovstage


def drain_population_change(stage, physx, ordinal):
    ovstage.population.apply_usd_changes(stage, ordinal=ordinal)
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.update_from_ovstage(ordinal, ordinal)
```

`apply_usd_changes()` waits for population work but does not seal the ordinal.
Do not drain the initial `read_ordinal` again; `attach_ovstage()` already parsed
it. Before a structural add or remove, destroy cached tensor bindings when
practical, then create replacements after the update completes. Do not use a
pre-update binding's membership as the test oracle.

Full sample:
- `samples/python_samples/hello_world.py` (wheel)
- Source checkout: `tests/python_samples/hello_world.py`

## C

The C workflow follows one ordering; keep it as-is:

1. `ovphysx_initialize()` (process-wide; no Python analogue).
2. `ovphysx_create_instance()` with `OVPHYSX_CREATE_ARGS_DEFAULT`.
3. Populate an ovstage from USD, wait for the population op, advance and wait
   for the write floor, then `ovphysx_attach_ovstage(handle, stage, read_ordinal)`.
4. `ovphysx_step_sync()` — steps and waits in one call, so step errors are
   reported and teardown cannot race the step.
5. Detach and destroy the stage, then `ovphysx_destroy_instance()`.
6. `ovphysx_shutdown()`.

This program uses only public ovphysx / ovstage API, so it builds against the SDK
as shown under "Build the C sample" below:

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to initialize ovphysx\n");
        return 1;
    }

    ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = 0;
    if (ovphysx_create_instance(&create_args, &handle).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to create instance\n");
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

    const char* usd_path = "scene.usda";
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strnlen(usd_path, 4096);

    const uint64_t ordinal = 1;
    ovstage_population_enqueue_result_t open = ovstage_population_open_usd_from_file(
        stage, path, ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
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

    if (ovphysx_attach_ovstage(handle, stage, ordinal).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to attach ovstage\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    // step_sync steps and waits; a nonzero status means the step failed.
    if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Simulation step failed\n");
        ovphysx_detach_ovstage(handle);
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    ovphysx_detach_ovstage(handle);
    ovstage_destroy_instance(stage);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    return 0;
}
```

If you step with the async `ovphysx_step()` instead of `ovphysx_step_sync()`,
you must then call `ovphysx_wait_op(handle, step_result.op_index, ...)` and check
`num_errors` before teardown — the enqueue status alone only means *accepted*,
not *completed*.

Full sample:
- `samples/c_samples/hello_world_c/main.c` (SDK)
- Source checkout: `tests/c_samples/hello_world_c/main.c`

The tested sample keeps its `main()` short by factoring the populate/attach and
stage teardown into local helpers (`ovphysx_sample_attach_usd_with_ovstage()` /
`ovphysx_sample_destroy_stage()` in `tests/c_samples/common/ovstage_sample.h`).
Those helpers are sample scaffolding, not public API — the sequence above is what
they wrap.

Build the C sample with CMake:

```cmake
find_package(ovphysx REQUIRED)
add_executable(my_app main.c)
target_link_libraries(my_app PRIVATE ovphysx::ovphysx ovphysx::ovstage)
if(WIN32) # optionally copy ovphysx dlls into bin folder
    ovphysx_copy_runtime_dlls(my_app)
endif()
```

Configure and build:

```bash
cmake -B build \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"
cmake --build build
```

For prebuilt SDKs, download OVPhysX and the matching native OVStage archive
from their GitHub Releases pages and keep the extracted roots separate. Source
builds fetch OVStage automatically.

## Key APIs

| Python | C |
|--------|---|
| _(none — implicit)_ | `ovphysx_initialize()` / `ovphysx_shutdown()` |
| `PhysX()` | `ovphysx_create_instance()` |
| `physx.attach_ovstage(stage)` | `ovphysx_attach_ovstage()` |
| `physx.step_sync(dt)` | `ovphysx_step_sync()` |
| `physx.step(dt)` | `ovphysx_step()` |
| `physx.release()` | `ovphysx_destroy_instance()` |

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
