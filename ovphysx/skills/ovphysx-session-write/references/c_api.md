# C Write-Session Lifecycle

Read this file for the C `ovphysx_write()` branch. Every call returns
`ovphysx_result_t`; check `.status == OVPHYSX_API_SUCCESS` and release on both
the success and error paths.

Before calling the helper, initialize ovphysx and attach a populated ovstage
instance as described in [Basic Workflow](../../basic-workflow/SKILL.md#c).
Complete a warmup or step before the first write. The helper requires
CPU-resident velocity columns and an `error_text` buffer with nonzero capacity.
Keep the instance and stage alive until it returns.

## Lifecycle

1. **Query** the simulated type (`ovphysx_query`), by type -- not schema or
   prim-path pattern.
2. **Open** a write session on ONE attribute (`ovphysx_write`). The attribute is
   an `ovx_string_or_token_t`: a semantic string such as `OVPHYSX_ATTR_POSITION`,
   or an interned token from discovery.
3. **Fetch, fill, commit** each group. `ovphysx_fetch_write_next` yields borrowed
   groups until `OVPHYSX_API_END_OF_ITERATION`. Any other non-success status is a
   real error, not exhaustion. Fill EVERY entry of `group->data.tensors`, then
   `ovphysx_commit_group`. A group never committed publishes nothing.
4. **Release** the write handle then the query handle.

Add this helper to `examples/session_write.c` in your SDK application. It
extends the attached-instance lifecycle; it does not replace initialization
or stage setup. Success returns `OVPHYSX_API_SUCCESS` after at least one commit.

```c
#include <ovphysx/ovphysx.h>
#include <stdio.h>

// Precondition: the write never auto-warms. On CPU / GPU-with-readback a pre-step
// write is applied. On DirectGPU, commit is refused until a first step has sized
// the GPU view -- warmup() or step first for a recipe that works on every mode.
// error_text must point to error_capacity writable bytes; capacity must be nonzero.
static ovphysx_result_t drive_all_bodies_x(ovphysx_handle_t handle, float vx,
                                         char* error_text, size_t error_capacity)
{
    ovphysx_query_handle_t query = 0;
    ovphysx_write_handle_t write = 0;
    const ovstage_map_group_t* group = NULL;
    int committed = 0;
    const ovx_string_or_token_t attr = {
        0, { OVPHYSX_ATTR_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_LINEAR_VELOCITY) - 1 }
    };
    error_text[0] = '\0';
    ovphysx_result_t r = ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query);
    if (r.status != OVPHYSX_API_SUCCESS)
        goto failed;
    r = ovphysx_write(handle, query, &attr, &write);
    if (r.status != OVPHYSX_API_SUCCESS)
        goto failed;

    for (;;) {
        r = ovphysx_fetch_write_next(handle, write, &group);
        if (r.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        if (r.status != OVPHYSX_API_SUCCESS)
            goto failed;
        // Fill every tensor. This helper expects CPU-resident float vec3 columns.
        for (uint32_t ti = 0; ti < group->data.tensor_count; ++ti) {
            const DLTensor* t = &group->data.tensors[ti];
            if (!t->data || t->device.device_type != kDLCPU) {
                r = (ovphysx_result_t){ OVPHYSX_API_DEVICE_MISMATCH };
                snprintf(error_text, error_capacity, "drive_all_bodies_x requires CPU tensors");
                goto cleanup;
            }
            const size_t lanes = t->dtype.lanes ? t->dtype.lanes : 1;
            const size_t rows = (size_t)t->shape[0];
            float* dst = (float*)t->data;
            for (size_t i = 0; i < rows; ++i) {
                dst[i * lanes + 0] = vx;
                for (size_t c = 1; c < lanes; ++c)
                    dst[i * lanes + c] = 0.0f;
            }
        }
        r = ovphysx_commit_group(handle, write, group, (ovstage_cuda_sync_t){ 0 });
        if (r.status != OVPHYSX_API_SUCCESS)
            goto failed;
        ++committed;
    }

    r = (ovphysx_result_t){ committed > 0 ? OVPHYSX_API_SUCCESS : OVPHYSX_API_ERROR };
    if (committed == 0)
        snprintf(error_text, error_capacity, "write produced no groups; nothing was driven");
    goto cleanup;

failed:
    {
        // Cleanup may replace the native error; copy it into caller-owned storage first.
        const ovphysx_string_t error = ovphysx_get_last_error();
        snprintf(error_text, error_capacity, "%.*s", (int)error.length, error.ptr ? error.ptr : "");
    }
cleanup:
    if (write)
        ovphysx_release_write(handle, write);
    if (query)
        ovphysx_release_query(handle, query);
    return r;
}
```

The caller retains the failure reason in `error_text` after the helper releases its
handles. On CPU or GPU-with-readback scenes, PhysX rejects this velocity setter on
standalone kinematic bodies, and commit reports the SDK error. Valid dynamic rows in
the same group can still be written, including rows after the rejected body; the
kinematic body's velocity stays unchanged. The failed group is spent, with no rollback
or applied/rejected row count. The default rigid-body query includes kinematic bodies;
there is no per-body selection or kinematic-body filter.

## Device and streams

The example writes a CPU session (`{0, 0}` asserts nothing is outstanding). For a
GPU scene the group tensors may be CUDA and the write may stage through the host;
inspect `group->data.tensors[i].device.device_type` per tensor, fill on the
matching device, and pass a real `ovstage_cuda_sync_t{stream, wait_event}` to
`ovphysx_commit_group` so the write is ordered after your producing work.

## Forces and wrenches

`OVPHYSX_ATTR_FORCE` is a vec3 at the centre of mass. `OVPHYSX_ATTR_WRENCH` is
nine wide, `[fx,fy,fz, tx,ty,tz, px,py,pz]` -- force, torque, WORLD application
point. Both are WRITE-ONLY: the solver consumes and clears them each step, so
there is nothing to read back -- assert the consequence.

## Validation

- Warm up or step before the first write for a recipe that works on DirectGPU too.
  On CPU and GPU-with-readback a pre-step write is applied; on DirectGPU it is
  refused. The write never auto-warms.
- Every group's tensors are fully filled before `ovphysx_commit_group`; there is
  no fill mask for skipping individual bodies. Queries select object types and
  attributes, not individual prims. Refer to
  [Read/Write Limitations](../../../docs/read_write/limitations.md) before
  choosing an API for per-body control.
- Both handles are released on the success AND every error path.
- The test observes a physical consequence, not the write-only input read back.

Read simulated results back with `ovphysx_read` -- see the [Output Read](../../ovphysx-output-read/SKILL.md)
skill. Worked write example (source checkout): the write loop in
`tests/c_unittests/test_joint_datamovement.cpp`.
