---
name: clone-environments
description: Clone one USD environment subtree into many runtime-only PhysX environments for RL-style batched simulation. Use when a caller needs replicated physics environments, clone API ordering, or target transform guidance.
license: Apache-2.0
compatibility: "ovphysx >=0.6.0 wheel or SDK (anchor_transforms naming); source USD must contain the source prim path before cloning."
allowed-tools: Read Shell
metadata:
  version: "0.1.4"
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
2. Populate an ovstage, attach it, drain the committed ordinal range, call clone before `warmup()` or the first simulation step, and wait for clone completion before creating bindings.
3. Use Shell to run the Python sample or compile the C sample after adapting the source and target paths.

## Python

```python
from ovphysx import PhysX, codeless_schema_root
import ovstage

PhysX.set_cpu_mode(True)
physx = PhysX()
# Register the codeless PhysX schemas before the first population call.
ovstage.population.register_usd_schemas([str(codeless_schema_root())])
stage = ovstage.Stage("ovphysx-clone")
ovstage.population.open_usd(stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.attach_ovstage(stage, read_ordinal=1)

targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
anchor_transforms = [
    (4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
]
physx.clone("/World/envs/env0", targets, anchor_transforms=anchor_transforms)
physx.wait_all()

physx.detach_ovstage()
stage.destroy()
physx.destroy()
```

The physics-only `domains` mask above is fine for this skill's non-instanced
sample USD. For arbitrary content prefer `ALL` — see
[Population domains](../../docs/ovstage_integration.md#population-domains).
Clones are runtime-only PhysX environments and are not populated into the
Stage, so a render consumer does not see them.

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
    const ovphysx_timeout_t kTenSecondsNs = 10ULL * 1000ULL * 1000ULL * 1000ULL;

    ovphysx_op_wait_result_t wait_result = {0};
    ovphysx_result_t wait_status =
        ovphysx_wait_op(handle, op_index, kTenSecondsNs, &wait_result);
    int ok = wait_status.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0;
    ovphysx_destroy_wait_result(&wait_result);
    return ok;
}

static int load_and_clone_envs(ovphysx_handle_t handle, ovstage_instance_t* stage)
{
    // Populate the ovstage from USD, seal its ordinal, then attach (public API).
    const uint64_t ordinal = 1;
    const char* usd_path = "scene.usda";
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strnlen(usd_path, 4096);

    // Register the codeless PhysX schemas before the first population call in the
    // process; attach_ovstage refuses a stage populated without them.
    ovphysx_string_t schema_root;
    if (ovphysx_get_codeless_schema_root(&schema_root).status != OVPHYSX_API_SUCCESS) {
        return 0;
    }
    ovx_string_t schema_path;
    schema_path.ptr = schema_root.ptr;
    schema_path.length = schema_root.length;
    if (ovstage_population_register_usd_schemas(&schema_path, 1) != OVSTAGE_OK) {
        return 0;
    }

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
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK) {
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
    const float anchor_transforms[3 * 7] = {
        4.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        8.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };

    // Clone before warmup or the first step; wait before creating bindings.
    ovphysx_enqueue_result_t clone_result = ovphysx_clone(
        handle,
        ovphysx_cstr("/World/envs/env0"),
        targets,
        3,
        anchor_transforms,
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
| `physx.wait_all()` | `ovphysx_wait_op(handle, OVPHYSX_OP_INDEX_ALL, OVPHYSX_TIMEOUT_INFINITE, &wait_result)` |

`wait_all()` drains all pending ops; the C sample instead waits the single clone
op with `ovphysx_wait_op()` and the returned operation index.

## References

- Docs: `docs/tutorials/cloning.md`
- Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
- C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)
