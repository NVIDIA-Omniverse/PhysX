---
name: clone-environments
description: Clone one USD environment subtree into many runtime-only PhysX environments for RL-style batched simulation. Use when a caller needs replicated physics environments, clone API ordering, or target transform guidance.
license: Apache-2.0
compatibility: "ovphysx build with the task-completion API; source USD must contain the source prim path before cloning."
allowed-tools: Read Shell
metadata:
  version: "0.1.7"
  author: NVIDIA Omniverse Physics
  tags: "ovphysx, physics, cloning, rl"
---

# Clone Environments

Use the clone API to replicate a USD source subtree into runtime-only PhysX copies.
The source hierarchy must exist in the loaded USD stage; live PhysX objects are keyed by
the target paths, but no USD or runtime-stage prims are authored.

Clone before `warmup()` or the first simulation step. Wait for the clone operation before creating bindings that should match cloned prims.

## When to Use

Use this skill when a caller needs many copies of one loaded environment for RL-style batched simulation, wants the clone API call order, or needs the target transform layout.

## Instructions

1. Read `docs/tutorials/cloning.md` and the sample for the caller's language before changing code.
2. Populate and seal an ovstage, attach it and collect the attach task, call clone before `warmup()` or the first simulation step, and wait for clone completion before creating bindings.
3. Use Shell to run the Python sample or compile the C sample after adapting the source and target paths.

For Python, install `physx-usd-schemas` and `newton-usd-schemas` alongside
`ovphysx`. Register both PhysX and Newton schemas before the first population
call so authored `newton:*` values are preserved.

Import the PhysX schema recipe from
`ovphysx.samples.python_samples.schema_support`; for native samples, set
`PHYSX_USD_SCHEMAS_ROOT` to the extracted wheel. See
[Physics Schemas](../../docs/physics_schemas.md) for setup.

## Python

```python
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX
import ovstage

PhysX.set_cpu_mode(True)
physx = PhysX()
# Register external PhysX and Newton schemas before the first population.
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("ovphysx-clone")
usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "basic_simulation.usda"
ovstage.population.open_usd(stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
anchor_transforms = [
    (4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
]
physx.clone("/World/envs/env0", targets, anchor_transforms=anchor_transforms)
physx.wait_all()

physx.wait_task(physx.detach_ovstage())
stage.destroy()
physx.destroy()
```

Use `ALL` when populating USD so nested world transforms are resolved by the
rendering domain. `PHYSICS` alone seeds local world matrices in OVStage 0.3;
see [Population domains](../../docs/ovstage_integration.md#population-domains).

Use `anchor_transforms` when each clone needs an explicit target-root world pose.
Provide one transform per target: Python uses a list with `len(anchor_transforms) == len(targets)`, and C uses a flat `num_targets * 7` float array.
Each transform is `(px, py, pz, qx, qy, qz, qw)`.
The transform anchors the exact target subtree root at its final absolute world
pose. For example, when cloning
`/env0/Robot` to `/env1/Robot`, supply the final world pose of `/env1/Robot`.
PhysX environment ids isolate clones only under GPU dynamics + GPU broadphase.
In CPU mode, use spatially disjoint anchor transforms as above. Co-located CPU
clones share one collision space and can push each other apart. The runtime logs
`EnvIds requested but gpu dynamic is disabled` when the requested isolation is
not in effect. That record goes to the Carbonite log, not Python `warnings`, so
observe it with `ovphysx.enable_python_logging()` (Python) or
`ovphysx_set_log_callback()` (C).

Full sample:
- `samples/python_samples/clone.py` (wheel)
- Source checkout: `tests/python_samples/clone.py`

## C

The clone + wait is the skill's subject; the ovstage populate/attach that precedes
it is the same public flow as `basic-workflow`. This fragment uses only public
ovphysx / ovstage API. The caller passes a stage it created with
`ovstage_create_instance` and the absolute path to the SDK's
`samples/data/basic_simulation.usda`, which contains `/World/envs/env0`.
Copy the SDK's `samples/c_samples/common/` directory beside the source file:

```c
#include <ovphysx/ovphysx.h>
#include "common/ovstage_sample.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int wait_for_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS)
        return 0;

    // The result must start zeroed; the wait owns any error text it collects
    // until ovphysx_destroy_wait_task_result() releases it.
    ovphysx_wait_task_result_t wait_result = {0};
    ovphysx_result_t wait_status =
        ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &wait_result);
    int ok = wait_status.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0;
    ovphysx_destroy_wait_task_result(&wait_result);
    return ok;
}

static int load_and_clone_envs(
    ovphysx_instance_handle_t handle, ovstage_instance_t* stage, const char* usd_path)
{
    // Populate the ovstage from USD, seal its ordinal, then attach (public API).
    const uint64_t ordinal = 1;
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strlen(usd_path);

    // Register external schemas before population.
    if (!ovphysx_sample_register_physx_schemas()) {
        return 0;
    }

    ovstage_population_enqueue_result_t open = ovstage_population_open_usd_from_file(
        stage, path, ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
    ovstage_population_op_wait_result_t open_wait;
    memset(&open_wait, 0, sizeof(open_wait));
    if (open.status != OVSTAGE_OK ||
        ovstage_population_wait_op(stage, open.op_index, OVSTAGE_TIMEOUT_INFINITE, &open_wait) != OVSTAGE_OK) {
        return 0;
    }

    // attach_ovstage() reads at a sealed ordinal.
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
        return 0;
    }
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK) {
        return 0;
    }

    if (!wait_for_task(handle, ovphysx_attach_ovstage(handle, stage, ordinal))) {
        return 0;
    }

    ovphysx_string_t targets[3] = {
        ovphysx_cstr("/World/envs/env1"),
        ovphysx_cstr("/World/envs/env2"),
        ovphysx_cstr("/World/envs/env3")
    };
    const float anchor_transforms[3 * 7] = {
        4.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        8.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };

    // Clone before warmup or the first step; wait before creating bindings.
    // A SUCCESS status is only admission: the clone's outcome is collected by
    // the wait on its receipt.
    ovphysx_enqueue_result_t clone_task = ovphysx_clone(
        handle,
        ovphysx_cstr("/World/envs/env0"),
        targets,
        3,
        anchor_transforms,
        NULL);  /* env_ids: automatic per-call numbering */
    if (clone_task.status != OVPHYSX_API_SUCCESS) {
        return 0;
    }

    return wait_for_task(handle, clone_task);
}
```

Full sample:
- `samples/c_samples/clone_c/main.c` (SDK)
- Source checkout: `tests/c_samples/clone_c/main.c`

## C++

The experimental C++ wrapper returns a move-only `WaitResult`. It calls
`ovphysx_destroy_wait_task_result()` when it leaves scope, including on an early
return. With the sample stage already attached, clone before warmup or stepping:

```cpp
#include <ovphysx/experimental/ovphysx.hpp>
#include <string>
#include <vector>

bool clone_two_envs(ovphysx::PhysX& physx)
{
    const std::vector<std::string> targets = {
        "/World/envs/env1", "/World/envs/env2"
    };
    const float anchor_transforms[2 * 7] = {
        4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };
    const ovphysx_enqueue_result_t task =
        physx.clone("/World/envs/env0", targets, anchor_transforms);
    if (task.status != OVPHYSX_API_SUCCESS) {
        return false;
    }
    ovphysx::physx::WaitResult result = physx.waitTask(task);
    return result.status() == OVPHYSX_API_SUCCESS;
}
```

Inspect `result.errorCount()` and `result.errorTaskAt(index)` before leaving scope
if the caller needs individual failures. Use each failed receipt with
`ovphysx_get_last_task_error(physx.handle(), receipt)` while the result is alive.

## Requirements

- Source path exists in the loaded USD stage.
- Target paths are unique and do not already exist.
- Clone before warmup or the first simulation step.
- Cross-environment collision isolation uses PhysX environment ids automatically
  (`/ovphysx/clone/useEnvIds`, default on; engages under GPU dynamics + GPU broadphase).
  Environment ids do not isolate CPU clones. Give CPU clones spatially disjoint
  `anchor_transforms`; co-located CPU clones share one collision space.
  When one logical environment is assembled from several clone calls (one call per source
  row), pass the optional per-target `env_ids` with the same ids in every call so
  same-environment objects keep colliding. USD collision groups/filtering remain available
  for finer-grained control.

## Key APIs

| Python | C |
|--------|---|
| `physx.clone(source, targets)` | `ovphysx_clone()` |
| `physx.wait_task(task)` | `ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &wait_result)` |
| `physx.wait_all()` | `ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &wait_result)` |

`clone()` returns a `Task` receipt in Python and an `ovphysx_enqueue_result_t` in
C. `wait_all()` collects every uncollected task; the C sample instead waits the
single clone task with `ovphysx_wait_task()` and the returned receipt. In Python
an accepted clone that fails during execution raises `TaskError` from its wait;
a rejected submission raises immediately from `clone()`.

## References

- Docs: `docs/tutorials/cloning.md`
- Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
- C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)
