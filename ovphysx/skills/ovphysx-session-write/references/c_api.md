# C Write-Session Lifecycle

Read this file for the C `ovphysx_write()` branch. Every call returns
`ovphysx_result_t`; check `.status == OVPHYSX_API_SUCCESS` and release on both
the success and error paths.

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

```c
#include <ovphysx/ovphysx.h>

// Precondition: warm up or step first -- a write before the first step is
// REFUSED (the DirectGPU superset view does not exist yet), never auto-warmed.
static ovphysx_result_t drive_all_bodies_x(ovphysx_handle_t handle, float vx)
{
    ovphysx_query_handle_t query = 0;
    ovphysx_result_t r = ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query);
    if (r.status != OVPHYSX_API_SUCCESS)
        return r;

    const ovx_string_or_token_t attr = {
        0, { OVPHYSX_ATTR_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_LINEAR_VELOCITY) - 1 }
    };
    ovphysx_write_handle_t write = 0;
    r = ovphysx_write(handle, query, &attr, &write);
    if (r.status != OVPHYSX_API_SUCCESS) {
        ovphysx_release_query(handle, query);
        return r;
    }

    const ovstage_map_group_t* group = NULL;
    int committed = 0;
    for (ovphysx_result_t fw;
         (fw = ovphysx_fetch_write_next(handle, write, &group)).status != OVPHYSX_API_END_OF_ITERATION;) {
        if (fw.status != OVPHYSX_API_SUCCESS) {          // a real error, not exhaustion
            ovphysx_release_write(handle, write);
            ovphysx_release_query(handle, query);
            return fw;
        }
        // Fill EVERY tensor of the group: commit publishes the whole group, so an
        // unfilled tensor drives its prims with stale memory. This is a CPU write
        // session, so a non-CPU tensor is a setup error -- fail fast rather than skip
        // and commit a partially filled group (a GPU scene takes the device path
        // described below, not a continue).
        for (uint32_t ti = 0; group->data.tensors && ti < group->data.tensor_count; ++ti) {
            const DLTensor* t = &group->data.tensors[ti];
            if (!t->data || t->device.device_type != kDLCPU) {
                ovphysx_release_write(handle, write);
                ovphysx_release_query(handle, query);
                return (ovphysx_result_t){ OVPHYSX_API_DEVICE_MISMATCH };
            }
            const size_t lanes = t->dtype.lanes ? t->dtype.lanes : 1;   // vec3 -> 3
            const size_t rows = (size_t)t->shape[0];
            float* dst = (float*)t->data;
            for (size_t i = 0; i < rows; ++i) {
                dst[i * lanes + 0] = vx;                 // x
                for (size_t c = 1; c < lanes; ++c)
                    dst[i * lanes + c] = 0.0f;
            }
        }
        // ovstage_cuda_sync_t{} == {0, 0}: nothing outstanding on a host write.
        r = ovphysx_commit_group(handle, write, group, (ovstage_cuda_sync_t){ 0 });
        if (r.status != OVPHYSX_API_SUCCESS) {
            ovphysx_release_write(handle, write);
            ovphysx_release_query(handle, query);
            return r;
        }
        ++committed;
    }

    ovphysx_release_write(handle, write);
    ovphysx_release_query(handle, query);
    return committed > 0 ? (ovphysx_result_t){ OVPHYSX_API_SUCCESS }
                         : (ovphysx_result_t){ OVPHYSX_API_ERROR };     // no groups: nothing driven
}
```

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

- Warm up or step before the first write; a pre-step write is refused.
- Every group's tensors are fully filled before `ovphysx_commit_group`; to write
  fewer prims, query fewer.
- Both handles are released on the success AND every error path.
- The test observes a physical consequence, not the write-only input read back.

Read simulated results back with `ovphysx_read` -- see the `ovphysx-output-read`
skill. Worked write example (source checkout): the write loop in
`tests/c_unittests/test_joint_datamovement.cpp`.
