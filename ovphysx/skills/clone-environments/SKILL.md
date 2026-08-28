---
name: clone-environments
description: Clone one USD environment subtree into many runtime-only PhysX environments for RL-style batched simulation. Use when a caller needs replicated physics environments, clone API ordering, or target transform guidance.
version: "0.1.0"
author: NVIDIA Omniverse Physics
tags:
  - ovphysx
  - physics
  - cloning
  - rl
tools:
  - Read
  - Shell
compatibility: "ovphysx >=0.5.4 wheel or SDK (six-argument ovphysx_clone with env_ids); source USD must contain the source prim path before cloning."
---

# Clone Environments

Use the clone API to replicate a USD source subtree into runtime-only PhysX copies.
The source hierarchy must exist in the loaded USD stage; live PhysX objects are keyed by
the target paths, but no USD or runtime-stage prims are authored.

Clone before `warmup_gpu()` or the first simulation step. Replication executes inline;
the returned operation index is already complete, and bindings created after the call
can match cloned physics objects immediately.

## When to Use

Use this skill when a caller needs many copies of one loaded environment for RL-style batched simulation, wants the clone API call order, or needs the target transform layout.

## Instructions

1. Read `docs/tutorials/cloning.md` and the sample for the caller's language before changing code.
2. Populate an ovstage, attach it at that ordinal, and call clone before `warmup_gpu()` or the first simulation step. The clone is complete when the call returns; `wait_op()` / `wait_all()` remains valid but returns immediately.
3. Use Shell to run the Python sample or compile the C sample after adapting the source and target paths.

## Python

```python
from ovphysx import PhysX
import ovstage

PhysX.set_cpu_mode(True)
physx = PhysX()
stage = ovstage.Stage("ovphysx-clone")
ovstage.population.open_usd(stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.attach_ovstage(stage, read_ordinal=1)

targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
physx.clone("/World/envs/env0", targets)
physx.wait_all()

physx.detach_ovstage()
stage.destroy()
physx.release()
```

The physics-only `domains` mask above is fine for this skill's non-instanced
sample USD. For arbitrary content prefer `ALL` -- see
`docs/ovstage_integration.md` ("Population domains"). Clones are runtime-only
PhysX environments and are not populated into the Stage, so a render consumer
does not see them.

Use `parent_transforms` when each clone needs an initial parent pose.
Provide one transform per target: Python uses a list with `len(parent_transforms) == len(targets)`, and C uses a flat `num_targets * 7` float array.
Each transform is `(px, py, pz, qx, qy, qz, qw)`.

Full sample:
- `samples/python_samples/clone.py` (wheel)
- Source checkout: `tests/python_samples/clone.py`

## C

The clone + wait is the skill's subject; the ovstage populate/attach that precedes
it is the same public flow as `basic-workflow`. This fragment uses only public
ovphysx / ovstage API (the caller passes a stage it created with
`ovstage_create_instance`):

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int wait_for_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index)
{
    const uint64_t kTenSecondsNs = 10ULL * 1000ULL * 1000ULL * 1000ULL;

    ovphysx_op_wait_result_t wait_result = {0};
    ovphysx_result_t wait_status =
        ovphysx_wait_op(handle, op_index, kTenSecondsNs, &wait_result);
    int ok = wait_status.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0;
    ovphysx_destroy_wait_result(&wait_result);
    return ok;
}

static int load_and_clone_envs(ovphysx_handle_t handle, ovstage_instance_t* stage)
{
    // Populate the ovstage from USD, wait for it, then attach (public API).
    const uint64_t ordinal = 1;
    const char* usd_path = "scene.usda";
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strnlen(usd_path, 4096);

    ovstage_population_enqueue_result_t open = ovstage_population_open_usd_from_file(
        stage, path, ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
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

    if (ovphysx_attach_ovstage(handle, stage, ordinal).status != OVPHYSX_API_SUCCESS) {
        return 0;
    }

    ovphysx_string_t targets[3] = {
        ovphysx_cstr("/World/envs/env1"),
        ovphysx_cstr("/World/envs/env2"),
        ovphysx_cstr("/World/envs/env3")
    };

    // Clone before the first step (or GPU warmup); wait before creating bindings.
    ovphysx_enqueue_result_t clone_result = ovphysx_clone(
        handle,
        ovphysx_cstr("/World/envs/env0"),
        targets,
        3,
        NULL,   /* parent_transforms: co-locate on the source */
        NULL);  /* env_ids: automatic per-call numbering */
    if (clone_result.status != OVPHYSX_API_SUCCESS) {
        return 0;
    }

    return wait_for_op(handle, clone_result.op_index);
}
```

Full sample:
- `samples/c_samples/clone_c/main.c` (SDK)
- Source checkout: `tests/c_samples/clone_c/main.c`

## Requirements

- Source path exists in the loaded USD stage.
- Target paths are unique and do not already exist.
- Clone before GPU warmup or the first simulation step.
- Cross-environment collision isolation uses PhysX environment ids automatically
  (`/ovphysx/clone/useEnvIds`, default on; engages under GPU dynamics + GPU broadphase).
  When one logical environment is assembled from several clone calls (one call per source
  row), pass the optional per-target `env_ids` with the same ids in every call so
  same-environment objects keep colliding. USD collision groups/filtering remain available
  for finer-grained control.

## Key APIs

| Python | C |
|--------|---|
| `physx.clone(source, targets)` | `ovphysx_clone()` |
| `physx.wait_all()` | `ovphysx_wait_op(handle, OVPHYSX_OP_INDEX_ALL, timeout_ns, &wait_result)` |

`wait_all()` drains all pending ops; the C sample instead waits the single clone
op with `ovphysx_wait_op(handle, clone_result.op_index, ...)`.

## References

- Docs: `docs/tutorials/cloning.md`
- Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
- C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)
