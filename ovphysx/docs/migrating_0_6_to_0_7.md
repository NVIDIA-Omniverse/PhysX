<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Migration from 0.6 to 0.7

This page is for an application that already runs against ovphysx 0.6.
A new project should start at the [Quickstart](tutorials/quickstart.md) and the
[read/write contract](read_write/index.md). Those pages describe 0.7 as it is.

Removal of recording-only startup settings is covered in the
[OVD migration guide](ovd_api_migration.md). The 0.7
[changelog](changelog.md) lists schema, contact-name, and bugfix detail this
page does not repeat.

## Status codes (C and Python)

0.7 removes `OVPHYSX_API_END_OF_ITERATION` / `ApiStatus.END_OF_ITERATION`
(formerly 9) and changes `OVPHYSX_API_INVALID_STATE` / `ApiStatus.INVALID_STATE`
from 10 to 9. Use the named status constants rather than hard-coded values;
`INVALID_STATE` is an error, not an iteration sentinel.

For `ovphysx_fetch_read_next` and `ovphysx_fetch_write_next`, check receipt
admission and successful task completion before inspecting the output.
Iteration ends when `*out_group` is `NULL` for reads or `out_group->group` is
`NULL` for writes. A failed task can also leave a null group and must be
handled as an error. In Python, `session.fetch_next().wait()` returns `None`
at end of iteration and raises on failure. See the
[read/write contract](read_write/index.md).

## Python

`PhysX()` and `step_sync()` are unchanged.
`destroy()` raises while an output-read array or a view of it is still alive,
and it collects outstanding task failures. Drop those arrays first.
The calls that break a typical 0.6 script are stage lifecycle, read/write,
waiting, and config.

| 0.6 | 0.7 |
|---|---|
| `attach_ovstage(stage, read_ordinal)`, `update_from_ovstage(first, last)`, `detach_ovstage()` return after checked execution | Each returns a `Task`; collect it with `physx.wait_task(task)` before relying on success |
| `reset_stage()` for cleanup or whole-Stage replacement | `physx.wait_task(physx.detach_ovstage())`; retain the Stage if teardown fails |
| `with physx.read(object_type, names, scope=)` as a context manager that fetches every group | `with physx.read_sync(object_type, names, scope=)` for that behavior. `physx.read(query, attributes)` returns an `Operation` on a completed `Query` and fetches nothing until `fetch_next()` |
| `read_tokens(object_type, tokens, scope=)` | `read_tokens_sync(...)`, or pass the tokens to `read(query, tokens)` |
| `write(object_type, one_name, scope=)` | `write_sync(...)`. `write(query, attributes)` accepts several attributes. `session.commit(group)` returns an `Operation` and does not check the outcome. Use `commit_sync(group)` or `.commit(group).wait()`. Leaving a `write_sync()` context collects releases, not that commit |
| `step()` returns an operation index; `wait_op(index)` | `step()` returns a `Task`. `wait_task(task)` collects only that task. `wait_all()` collects every uncollected task. An integer index is not accepted |
| `step_n_sync(n, dt)` | a loop of `step_sync(dt)` |
| `set_config(entry)`, `set_config_bool` / `set_config_int32` / `set_config_float`, `get_config_bool` / `get_config_int32` / `get_config_float` / `get_config_string` | `set_global_config(key, value)` and `get_global_config(key)` for process settings. `PhysX.set_config(key, value)` is the per-instance setting |
| `update_articulations_kinematic()` | the same method, with component flags and a selection (`scope`, `prim_paths`, or `pattern`) |
| `query_shared_dictionary(query)` | `query.dictionary`, or `ovstage.PathDictionary(stage)` on the stage you attached |
| `get_contact_report()` and the raw-report types | Read `contactEvent*` attributes through a `SimObjectType.CONTACT_SENSOR` query; see [Contact sensors](read_write/readable.md#contact-sensors) |

`prim_paths` and `pattern` select prims on `query()`, `read_sync()`,
`read_tokens_sync()`, and `write_sync()`. They are interned path handles when
passed as `prim_paths`, and they take precedence over a pattern.
`PhysX.read()` and `PhysX.write()` take the `Query` those helpers create.

`PhysX(config=PhysXConfig(...))` still applies unrelated legacy create-time config.
OVD startup fields and `omnipvd_recording_capable` are removed; use the global
capability followed by explicit recording start/stop.
`nvtx_enabled` is gone. Use `profiler_backend="nvtx"` for the old true case
and `profiler_backend="none"` for false. New code sets process keys with
`set_global_config()` before the first instance. Set
`ConfigKey.ACTIVE_CUDA_GPUS` on the instance after create and before attach.
`PhysX.set_cpu_mode(True)` remains a compatibility wrapper for
`GlobalConfigKey.CPU_ONLY`.

A session read of rigid-body positions:

```python
from ovphysx.types import ObjectScope, SimObjectType

with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
    for group in result.groups:
        positions = group.tensors[0]
```

The async form opens the query yourself and waits on each receipt. See
[Read/write](read_write/index.md).

### Stage lifecycle

In 0.7, `attach_ovstage()` requires an owner-aware `ovstage.Stage`.
Passing `None` or a raw native handle raises `TypeError`. Replace raw handles
accepted in 0.6 with the owning Stage object; raw-handle integrations use the
C API.

Attach, update and detach currently execute inline. Their task-returning API
allows execution to be deferred later: a successful submission is not proof
that execution succeeded. For example, with a populated and sealed Stage:

```python
from ovphysx import PhysX


def simulate_stage(stage, read_ordinal: int) -> None:
    physx = PhysX()
    try:
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=read_ordinal))
        physx.step_sync(1.0 / 60.0)
        physx.wait_task(physx.detach_ovstage())
    finally:
        # Destroy the instance before the caller destroys its Stage.
        physx.destroy()
```

Wait for update receipts too. Keep the Stage and the sealed data read by a
lifecycle task alive through completion. Keep an attached Stage alive through
successful detach or instance destruction; a failed teardown may retain it.
`is_stage_attached()` remains an immediate ownership check. It does not wait
for a pending task or establish readiness.

`reset_stage()` performed teardown, not initial-pose restoration. Applications
such as IsaacLab must replace cleanup and whole-Stage replacement calls with
detach plus a successful wait when adopting this SDK. Episode resets continue
to write poses, velocities and joint state. Migrate attach and update waits in
the same consumer change; changing only the old reset calls is insufficient.

## C

`ovphysx_handle_t` is still a deprecated typedef of `ovphysx_instance_handle_t`.
`ovphysx.h` still includes `ovphysx_legacy.h`, so an existing include keeps
seeing the deprecated families, with deprecation warnings.
`OVPHYSX_ATTR_*` names are in `ovphysx_attributes.h`. `ovphysx.h` does not
include that header, so C code that uses those names adds
`#include <ovphysx/ovphysx_attributes.h>`.

### Creation and config

`ovphysx_create_instance()` takes only the output handle. Set OVD capability
with `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` before that call (Python:
`set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)` before
`PhysX()`). The old `ovphysx_config_entry_omnipvd_recording_capable()` helper
and OVD startup keys are removed.
Unrelated legacy create-time arguments, including Carbonite tasking pool sizes, go to
`ovphysx_create_instance_with_args()` in the legacy header.

In 0.6, `ovphysx_set_global_config` took an `ovphysx_config_entry_t` from
`ovphysx_config_entry_bool`, `ovphysx_config_entry_int32`,
`ovphysx_config_entry_float`, `ovphysx_config_entry_string`, or a named helper
such as `ovphysx_config_entry_num_threads`. In 0.7,
`ovphysx_set_global_config(key, value)` and `ovphysx_get_global_config()` are
the process settings, so those 0.6 calls no longer compile. The drop-in is
`ovphysx_set_global_config_entry` in `ovphysx_legacy.h`, except OmniPVD startup
and capability entries, which now return `OVPHYSX_API_INVALID_ARGUMENT`.
The per-instance CUDA
GPU list uses `ovphysx_set_config()` / `ovphysx_get_config()` after create and
before attach. `ovphysx_config_entry_nvtx_enabled` is removed; select the
profiler backend with `OVPHYSX_PROFILER` or
`OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND`.

### Task receipts

In 0.6, `ovphysx_step`, `ovphysx_clone`, `ovphysx_reset_stage`, and
`ovphysx_add_user_task` returned `ovphysx_enqueue_result_t`. In 0.7 these also
return a receipt:

- `ovphysx_attach_ovstage`, `ovphysx_update_from_ovstage`, `ovphysx_detach_ovstage`
- `ovphysx_query`, `ovphysx_query_selection`, `ovphysx_query_contacts`
- `ovphysx_read`, `ovphysx_write`
- `ovphysx_fetch_read_next`, `ovphysx_fetch_write_next`
- `ovphysx_commit_group`
- `ovphysx_release_query`, `ovphysx_release_read`, `ovphysx_release_write`, `ovphysx_release_group`
- `ovphysx_update_articulations_kinematic`

The arguments changed too. `ovphysx_read` takes a trailing nullable summary.
`ovphysx_write` takes an attribute array and count, the output handle, and a
nullable summary. `ovphysx_fetch_write_next` fills
`ovphysx_fetched_write_group_t`; read `.group` after the wait.

Check admission on the receipt, then collect the outcome with
`ovphysx_wait_task()` or `ovphysx_wait_all()`. Destroy the wait result with
`ovphysx_destroy_wait_task_result()`. Execution failure text is
`ovphysx_get_last_task_error(instance, task)` until that destroy.
`ovphysx_wait_op`, `ovphysx_get_last_op_error`, `ovphysx_destroy_wait_result`,
and `ovphysx_add_user_task` are removed. Run the application callback after the
relevant wait.

The wait-result type, fields, and cleanup function changed together:

| 0.6 | 0.7 |
|---|---|
| `ovphysx_op_wait_result_t` | `ovphysx_wait_task_result_t` |
| `error_op_indices[i]` | `error_tasks[i]`, a full `ovphysx_enqueue_result_t` receipt |
| `lowest_pending_op_index` | `pending_task`, a full receipt; all zero if no task remains pending within that wait's scope |
| `ovphysx_destroy_wait_result(&result)` | `ovphysx_destroy_wait_task_result(&result)` |

`num_errors` still counts failed entries. Zero-initialize each result, process
its failed receipts and borrowed messages before destruction, and do not
shallow-copy it or free `error_tasks` separately. With a finite timeout,
`ovphysx_wait_all()` can return completed failures together with `TIMEOUT` and
`pending_task`; inspect the failures before destroying that result too.

This helper checks admission, waits for one task, prints retained execution
errors while their result is alive, and returns the wait status to its caller:

```c
#include <ovphysx/ovphysx.h>
#include <stdio.h>

static ovphysx_api_status_t collect_task(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS)
        return task.status;

    ovphysx_wait_task_result_t result = {0};
    const ovphysx_result_t status = ovphysx_wait_task(
        handle, task, OVPHYSX_TIMEOUT_INFINITE, &result);
    for (size_t i = 0; i < result.num_errors; ++i)
    {
        const ovphysx_string_t message =
            ovphysx_get_last_task_error(handle, result.error_tasks[i]);
        fwrite(message.ptr, 1, message.length, stderr);
        fputc('\n', stderr);
    }
    ovphysx_destroy_wait_task_result(&result);
    return status.status;
}
```

`ovphysx_reset_stage` is removed. Use `ovphysx_detach_ovstage` and collect its
task outcome before destroying or replacing the Stage. The sealed ordinal
data used by attach/update must remain readable through task completion.
Invalid submissions report an immediate error with no task; accepted runtime
failures are reported by the wait. A repeated detach is an accepted successful
no-op. `ovphysx_is_stage_attached` still returns an immediate status and writes
its boolean output without waiting or collecting any task.

`ovphysx_step_n_sync` is removed. Loop `ovphysx_step_sync`.

`ovphysx_update_articulations_kinematic` takes an articulation query and
position/velocity flags. `ovphysx_articulation_update_kinematic` is removed.

`ovphysx_subscribe_object_changes` and `ovphysx_unsubscribe_object_changes`
take the owning instance first.

`ovphysx_query` takes `ovphysx_query_target_t` and an optional summary.
`ovphysx_sim_object_type_t` is removed. Prim-set selection is
`ovphysx_query_selection`. Discovery that used `ovphysx_fetch_query_result` and
`ovphysx_query_shared_dictionary` is the query summary plus
`ovstage_get_path_dictionary` on the attached stage. Both ovphysx calls are
removed.

### CUDA read readiness

`ovphysx_cuda_stream_wait_event` is removed. Native consumers use CUDA's
`cuStreamWaitEvent` or `cudaStreamWaitEvent` with the read group's nonzero
`cuda_sync.wait_event` and check the returned CUDA status. Skip a zero event.
The event remains owned by the read session; do not destroy it. Finish external
consumers before releasing the session.

Python continues to order each fetched group's event onto the current Warp
stream before returning arrays. The bundled native runtime checks the CUDA
status without adding a Python CUDA package or blocking the host. If the wait
cannot be established, Python raises `RuntimeError` before exposing the arrays.
Host groups and zero events skip the wait.

### Removed lookups

`ovphysx_get_contact_report()` and its C structs are removed. Query
`OVPHYSX_QUERY_TARGET_CONTACT_SENSOR` and use `ovphysx_read()` with the
`contactEvent*` attributes for events, points and friction anchors; see
[Contact sensors](read_write/readable.md#contact-sensors). Results are
per-sensor arrays owned by the read session rather than the old flat borrowed
report, so adapt grouping, identities and lifetimes. A removed-shape event
cannot appear if no selected sensor can claim it. Detailed events remain
unavailable on DirectGPU. Net-force attributes do not replace event payloads.

`ovphysx_get_physx_ptr` is removed. Read the `physxPointer` column
(`OVPHYSX_ATTR_PHYSX_POINTER`) for the matching `ovphysx_query_target_t`.
The [PhysX interop](tutorials/physx_interop.md) tutorial is the 0.7 sequence.
Do not read that column from inside an object-change callback. Read it after
the triggering call, or its task wait, returns.

The `ovphysx_debug_render_*` family is removed. Debug drawing is a
`OVPHYSX_QUERY_TARGET_DEBUG_RENDER` query. Settings and geometry are ordinary
session attributes. A query does not require an attached stage. Names and
layouts are in [Readable](read_write/readable.md#debug-render) and
[Writable](read_write/writable.md#debug-render).

| 0.6 call | 0.7 attribute |
|---|---|
| `ovphysx_debug_render_enable` | `enabled` (`uint8`, nonzero on) |
| `ovphysx_debug_render_set_scale` / `get_scale` | `scale` |
| `ovphysx_debug_render_set_parameter` / `get_parameter` | one float per parameter: `worldAxes`, `bodyAxes`, `contactNormal`, `sdf`, and the other `OVPHYSX_ATTR_DEBUG_*` names |
| `ovphysx_debug_render_set_culling_box` | `cullingBox` |
| `ovphysx_debug_render_set_scope_tokens` | `scope` |
| `ovphysx_debug_render_get_points` | `pointPosition`, `pointColor` |
| `ovphysx_debug_render_get_lines` | `linePosition`, `lineColor` |
| `ovphysx_debug_render_get_triangles` | `trianglePosition`, `triangleColor` |

In C, pass `&OVPHYSX_NAME_DEBUG_*`. In Python, pass the string name.

### Still present, deprecated

These moved to `ovphysx_legacy.h` and still link. New code uses session
read/write.

- Tensor bindings, including `ovphysx_create_tensor_binding`, `ovphysx_rigid_body_view_wake_up`, and `ovphysx_rigid_body_view_sleep`. [Migrating off Tensor Bindings](tensor_binding_migration.md) maps each `TensorType` to session attributes.
- Contact bindings. A port from 0.6.0 through 0.6.3 gains `ovphysx_read_contact_net_normal_forces`, `ovphysx_read_contact_normal_force_matrix`, and `ovphysx_read_normal_contact_data`, which landed in 0.6.4. The older C names remain as aliases. Friction reads are separate. See the changelog.
- Scene queries (`ovphysx_raycast`, `ovphysx_sweep`, `ovphysx_overlap`)
- SDF views
- `ovphysx_get_object_type`

Old recording exports (`ovphysx_start_recording`, `ovphysx_stop_recording`,
`ovphysx_is_recording`) remain deprecated aliases of the `ovphysx_ovd_*` names.
They were not removed in 0.7.

## USD and schema

`PhysxJointAPI` is no longer parsed. Authored `physxJoint:maxJointVelocity`,
`physxJoint:armature`, and `physxJoint:jointFriction` are ignored.
`PrimBatch::applyPhysxJointAPI()` and `apply_physx_joint_api()` are removed.
Use `NewtonJointAPI` for a value that applies to every degree of freedom of a
joint, and `PhysxJointAxisAPI` for one axis. `physxJoint:jointFriction` has no
numeric equivalent. Retune with `newton:friction` and `newton:damping`, or the
per-axis friction attributes. The [changelog](changelog.md) records the
precedence change: a `PhysxJointAxisAPI` value wins over the Newton value.

Install `physx-usd-schemas` separately and register both resource directories
before the first USD schema read (population or export) in each process,
including workers. The native attach guard is restored in 0.7: if ovstage reports
that either required PhysX
schema family was missing when its registry was built, attach fails at admission
without a task or Stage ownership. Restart the process and register before
populating or exporting; a late registration cannot recover lost settings. The
legacy Carbonite setting `/ovphysx/schemas/requireRegistration = false` permits attach
with a warning but does not repair the data. See
[Physics Schemas](physics_schemas.md#attach-time-registration-guard).

Register `newton-usd-schemas` in the same call when the stage authors `newton:*`
attributes. Python's separate Newton warning remains; it does not check PhysX
schema registration or replace the native guard.
