# C Write-Session Lifecycle

Read this file for the C `ovphysx_write()` branch. Session operations return
`ovphysx_enqueue_result_t`: admission is separate from execution success. Wait
for each receipt before using output handles/groups, and collect release tasks
on both success and error paths. Keep input arrays and output slots alive until
completion. GPU buffers retain their additional producer/consumer dependencies.

## Lifecycle

1. **Query** the simulated type (`ovphysx_query`), by type -- not schema. Use
   `ovphysx_query_selection` instead to narrow the type to explicit prim handles
   or a path pattern. The last argument of either is an optional discovery
   result; pass NULL when you do not need it.
2. **Open** a write session over an array of attributes (`ovphysx_write`). Each
   entry is an `ovx_string_or_token_t`: a semantic string such as
   `OVPHYSX_ATTR_LINEAR_VELOCITY`, or an interned token from the attached Stage's
   dictionary (a token wins over a string in the same entry). Every entry is
   validated before the session is published: an empty array is rejected, and a
   duplicate (including a string and a token naming the same attribute), a token
   that does not resolve, or an unsupported or read-only name fails the open task.
   The optional last argument receives an `ovphysx_write_result_t` with one row per
   request (canonical token and planned group count); pass NULL to skip it.
3. **Fetch, fill, commit** each group. `ovphysx_fetch_write_next` fills a
   caller-owned `ovphysx_fetched_write_group_t` until a successful wait yields a
   NULL `group` (its `attribute_index` is then `SIZE_MAX`). A failed task is a
   real error, not exhaustion. `attribute_index` is the group's position in the
   request array -- the only way to tell same-shaped attributes apart. Fill EVERY
   entry of `group->data.tensors`, then pass the same `group` pointer to
   `ovphysx_commit_group`. A group never committed publishes nothing.
4. **Release** the write handle then the query handle.

This helper writes two same-shaped vec3 attributes in one session:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
#include <stdio.h>

static ovphysx_result_t finish_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
                                    char* error_text, size_t error_capacity)
{
    ovphysx_result_t status = { task.status };
    ovphysx_wait_task_result_t result = { 0 };
    if (task.status == OVPHYSX_API_SUCCESS)
        status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &result);
    if (status.status != OVPHYSX_API_SUCCESS) {
        const ovphysx_string_t error = result.num_errors ?
            ovphysx_get_last_task_error(handle, result.error_tasks[0]) : ovphysx_get_last_error();
        if (error_text[0] == '\0')
            snprintf(error_text, error_capacity, "%.*s", (int)error.length, error.ptr ? error.ptr : "");
        else
            fprintf(stderr, "Cleanup also failed: %.*s\n", (int)error.length, error.ptr ? error.ptr : "");
    }
    ovphysx_destroy_wait_task_result(&result);
    return status;
}

// Precondition: the write never auto-warms. On CPU / GPU-with-readback a pre-step
// write is applied. On DirectGPU, commit is refused until a first step has sized
// the GPU view -- warmup() or step first for a recipe that works on every mode.
// error_text must point to error_capacity writable bytes; capacity must be nonzero.
// Sets every rigid body's linear velocity to (vx, 0, 0) and angular velocity to (0, 0, wz).
static ovphysx_result_t drive_all_bodies(ovphysx_instance_handle_t handle, float vx, float wz,
                                         char* error_text, size_t error_capacity)
{
    ovphysx_query_handle_t query = 0;
    ovphysx_write_handle_t write = 0;
    int committed = 0;
    const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_LINEAR_VELOCITY) - 1 } },
        { 0, { OVPHYSX_ATTR_ANGULAR_VELOCITY, sizeof(OVPHYSX_ATTR_ANGULAR_VELOCITY) - 1 } },
    };
    error_text[0] = '\0';
    ovphysx_result_t r = finish_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, NULL), error_text, error_capacity);
    if (r.status != OVPHYSX_API_SUCCESS)
        goto cleanup;
    r = finish_task(handle, ovphysx_write(handle, query, attrs, 2, &write, NULL), error_text, error_capacity);
    if (r.status != OVPHYSX_API_SUCCESS)
        goto cleanup;

    for (;;) {
        ovphysx_fetched_write_group_t fetched = { NULL, 0 };
        r = finish_task(handle, ovphysx_fetch_write_next(handle, write, &fetched), error_text, error_capacity);
        if (r.status == OVPHYSX_API_SUCCESS && !fetched.group)
            break;
        if (r.status != OVPHYSX_API_SUCCESS)
            goto cleanup;
        const ovstage_map_group_t* group = fetched.group;
        // Both columns are float vec3: only attribute_index says which request this is.
        const size_t axis = fetched.attribute_index == 0 ? 0 : 2;
        const float value = fetched.attribute_index == 0 ? vx : wz;
        // Fill every tensor. This helper expects CPU-resident float vec3 columns.
        for (uint32_t ti = 0; ti < group->data.tensor_count; ++ti) {
            const DLTensor* t = &group->data.tensors[ti];
            if (!t->data || t->device.device_type != kDLCPU) {
                r = (ovphysx_result_t){ OVPHYSX_API_DEVICE_MISMATCH };
                snprintf(error_text, error_capacity, "drive_all_bodies requires CPU tensors");
                goto cleanup;
            }
            const size_t lanes = t->dtype.lanes ? t->dtype.lanes : 1;
            const size_t rows = (size_t)t->shape[0];
            float* dst = (float*)t->data;
            for (size_t i = 0; i < rows; ++i)
                for (size_t c = 0; c < lanes; ++c)
                    dst[i * lanes + c] = c == axis ? value : 0.0f;
        }
        r = finish_task(handle, ovphysx_commit_group(handle, write, group, (ovstage_cuda_sync_t){ 0 }), error_text, error_capacity);
        if (r.status != OVPHYSX_API_SUCCESS)
            goto cleanup;
        ++committed;
    }

    r = (ovphysx_result_t){ committed > 0 ? OVPHYSX_API_SUCCESS : OVPHYSX_API_ERROR };
    if (committed == 0)
        snprintf(error_text, error_capacity, "write produced no groups; nothing was driven");

cleanup:
    if (write) {
        const ovphysx_result_t released = finish_task(handle, ovphysx_release_write(handle, write),
                                                     error_text, error_capacity);
        if (r.status == OVPHYSX_API_SUCCESS)
            r = released;
    }
    if (query) {
        const ovphysx_result_t released = finish_task(handle, ovphysx_release_query(handle, query),
                                                     error_text, error_capacity);
        if (r.status == OVPHYSX_API_SUCCESS)
            r = released;
    }
    return r;
}
```

The caller retains the failure reason in `error_text` after the helper submits its
releases. Accepted release transfers cleanup responsibility to the SDK even when
the task reports a failure; do not retry it. Rejected release leaves ownership
with the caller, and this helper does not return the still-owned handle: when it
returns a failure, stop using this session's resources and destroy the instance
before continuing. A completed commit does not by itself release GPU staging
buffers; release the write session after its device dependencies.

Commits are not transactional. Each accepted commit spends its group, commits run
in the order you issue them, and an earlier successful commit stays applied when a
later one fails. On CPU or GPU-with-readback scenes, PhysX rejects these velocity
setters on standalone kinematic bodies, and commit reports the SDK error. Valid
dynamic rows in the same group can still be written, including rows after the
rejected body; the kinematic body's velocity stays unchanged. The failed group is
spent, with no rollback or applied/rejected row count. The default rigid-body query
includes kinematic bodies. There is no kinematic-body filter; to write only some
bodies, open the query with `ovphysx_query_selection` over their prims.

## Device and streams

The example writes a CPU session (`{0, 0}` asserts nothing is outstanding). For a
GPU scene the group tensors may be CUDA and the write may stage through the host;
inspect `group->data.tensors[i].device.device_type` per tensor, fill on the
matching device, and pass a real `ovstage_cuda_sync_t{stream, wait_event}` to
`ovphysx_commit_group` so the write is ordered after your producing work. Both
fields name your PRODUCER work and do not select the stream physics uses; both
are honored when both are nonzero, and only `{0, 0}` asserts that nothing is
outstanding. A context-relative stream value such as `1` (the legacy default
stream) refers to the CUDA context current on the calling thread, so make the
producer's context current first. Keep the stream and event valid until the
commit task completes. The current implementation can block the host while it
completes these dependencies. A failure to establish them fails the commit task,
spends the group, and makes the session release report the retained storage. During
the wait, the public ovphysx API rejects overlapping query/read/write/fetch/commit/release
submissions with `OVPHYSX_API_INVALID_STATE`. Producer work must not depend on code
(such as a CUDA host callback) that needs one of those calls to succeed or waits
for this commit to finish -- that cyclic dependency can deadlock. Keep public API
calls serialized.

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
  no fill mask for skipping rows. Narrow the prims with `ovphysx_query_selection`
  instead.
- Each group's attribute is identified by `attribute_index`, never by tensor
  shape or fetch order.
- Both release calls are submitted on the success AND every error path. A
  rejected release keeps the handle owned by the caller; after a failed helper
  call, destroy the instance before continuing.
- The test observes a physical consequence, not the write-only input read back.

Read simulated results back with `ovphysx_read` -- see the `ovphysx-output-read`
skill. Worked write example (source checkout): the write loop in
`tests/c_unittests/test_joint_datamovement.cpp`.
