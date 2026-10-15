# C Query/Read Lifecycle

Read this file for the native C output-read branch.

## Handle Sequence

```text
ovphysx_query(type, scope, &query, &query_result or NULL) -> wait_task
  -> ovphysx_read(query, attributes, count, &read, &read_result or NULL) -> wait_task
  -> ovphysx_fetch_read_next -> wait_task; stop on a null group
       -> consume group
       -> ovphysx_release_group -> wait_task
  -> ovphysx_release_read -> wait_task
  -> ovphysx_release_query -> wait_task
```

Query, read, fetch-next and release return `ovphysx_enqueue_result_t`.
The receipt status reports admission, including accepted work that fails while
executing inline. Wait on the receipt before using output handles, results or
groups. A successful fetch with a null output group means normal exhaustion; a
failed wait is an error. An empty query has a nonzero query handle and produces no
groups. `ovphysx_query_selection` takes the same final optional result.

Release every fetched group, then the read and query handles, after external
consumers have completed, including on error paths. Releases also return tasks that must be collected. Accepted release
ends caller use even if cleanup fails; do not resubmit it. Rejected release
retains caller ownership. Task error strings belong to the wait result: copy or
report them before `ovphysx_destroy_wait_task_result`. Keep input arrays and
output slots alive through their task's completion.

## Discovery and Selector Semantics

A non-NULL `ovstage_query_result_t*` passed to `ovphysx_query` receives a
discovery snapshot taken at creation: the total prim count and the tokens of the
attributes the object type can read (its vocabulary, not a promise that each one
emits a group for this match). Its attribute array belongs to the query until its
release is accepted.
Its `query_result_id` and `all_handle` are zero and are not OVStage resources; never
pass them to OVStage. A non-NULL `ovphysx_read_result_t*` passed to `ovphysx_read`
receives one row per requested attribute, in request order: the canonical token
(`OVX_INVALID_TOKEN` for a name the type cannot read), the group count, and
`omission_reasons` bits (`OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE`,
`..._UNSUPPORTED_CONFIGURATION`, `..._NOT_READY`) that tell an empty match from
unsupported or not-yet-ready data. Its rows stay valid until the read release is
accepted. NULL skips either summary. A duplicate request, or a token that does not
resolve in the attached Stage's dictionary, fails the read task.

A group's `prim_list` and `attribute` are interned integers, not paths or names.
They belong to the path dictionary of the ovstage Stage you attached; get it with
`ovstage_get_path_dictionary(stage)` and do not free it. Resolve an emitted
attribute through the public dictionary API:

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovx/path_dictionary/path_dictionary.h>
#include <stdio.h>

static int print_group_attribute(ovstage_instance_t* stage, const ovstage_read_group_t* group)
{
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
        return 0;
    ovx_string_t name = { 0 };
    const ovx_api_result_t result =
        path_dictionary_get_strings_from_tokens(dict, &group->attribute, 1, &name);
    if (result.status != OVX_API_SUCCESS) {
        if (result.error.ptr)
            path_dictionary_release_error(dict, result.error);
        return 0;
    }
    printf("%.*s\n", (int)name.length, name.ptr);
    return 1;
}
```

The returned name belongs to the dictionary and must not be freed. To resolve
prim paths, fetch handles with `path_dictionary_get_paths_from_path_list`, obtain
their component tokens with `path_dictionary_get_tokens_from_paths`, and resolve
those tokens with `path_dictionary_get_strings_from_tokens`. These calls write
to caller-provided arrays; check their returned counts and release any returned
error with `path_dictionary_release_error`.

A whole-type query is a selector over simulated type and scope. Each read
session evaluates its membership and column values against the latest completed
step when it opens, so a step between query and read makes the read observe the
newer step. The optional query result is a creation-time snapshot and never
updates. A selected query (`ovphysx_query_selection`) keeps the membership it
resolved at creation. An open read session stays valid after its query is
released. Before using `OVPHYSX_SCOPE_ACTIVE` or relying on its discovery count,
read [Scope and layout constraints](scope_and_layout.md); support differs by
object type.

## Borrowed Groups

`ovphysx_fetch_read_next()` returns a producer-owned
`ovstage_read_group_t const*`. The group struct and stage-derived prim list remain
valid until the group or read session is released, or the attachment is retired,
whichever happens first. Numeric tensor, prim-index-map,
data-index-map, mask, and CUDA-event storage remains valid until the read session is released.

Preserve these fields when forwarding a group:

- `prims.list`, `prims.offset`, `prims.count`, and `prims.index_map`
- emitted `attribute`, `semantic`, `is_array`, and `is_delete`
- `data.tensors`, `data.tensor_count`, `data.index_map`, and `data.cuda_sync`

Forwarding `cuda_sync` is not the same as honouring it. On a DirectGPU scene the
covered types emit **device-resident** (`kDLCUDA`) tensors, handed over before their
producing work has necessarily completed.

Use CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent` to order your stream after
the group's nonzero event, and check the returned CUDA status. A zero event means
there is nothing to wait for. The event is borrowed; do not destroy it.

The loop below uses the CUDA runtime and a caller-provided kernel launcher.
Compile it as C++; the ovphysx calls themselves use the C API.

```cpp
#include <ovphysx/ovphysx.h>
#include <cuda_runtime_api.h>
#include <stdio.h>

static ovphysx_result_t finish_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_result_t status = { task.status };
    ovphysx_wait_task_result_t result = {};
    if (task.status == OVPHYSX_API_SUCCESS)
        status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &result);
    for (size_t i = 0; i < result.num_errors; ++i) {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, result.error_tasks[i]);
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr ? error.ptr : "");
    }
    if (status.status != OVPHYSX_API_SUCCESS && !result.num_errors) {
        const ovphysx_string_t error = ovphysx_get_last_error();
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr ? error.ptr : "");
    }
    ovphysx_destroy_wait_task_result(&result);
    return status;
}

/* Supplied by the caller: a CUDA kernel launcher and a host column consumer. */
extern void launch_my_kernel(const float* rows, int64_t rowCount, cudaStream_t stream);
extern void consume_host_rows(const DLTensor* t);

/* Consumes every group of one read, then releases the read and the query.
 * Returns 1 on success, -1 for an unproven consumer wait, or 0 for another failure.
 * The -1 result retains both handles for caller-owned recovery.
 * `handle` and `query` are yours; `read` is the session ovphysx_read() handed back. */
static int consume_read(ovphysx_instance_handle_t handle,
                        ovphysx_query_handle_t query,
                        ovphysx_read_handle_t read,
                        cudaStream_t myStream)
{
    int ok = 1;

    for (;;) {
        const ovstage_read_group_t* g = NULL;
        const ovphysx_result_t r = finish_task(handle, ovphysx_fetch_read_next(handle, read, &g));
        if (r.status == OVPHYSX_API_SUCCESS && !g)
            break;                       /* exhausted -- NOT an error */
        if (r.status != OVPHYSX_API_SUCCESS) {
            ok = 0;                      /* a real failure; do not treat it as end-of-data */
            break;
        }

        /* Ordered ONCE per group, not per tensor: cuda_sync describes the whole group, and a
         * group is device-uniform. Only the legacy default/null stream of the producing
         * context is already ordered after the producer work; any other stream needs this
         * wait. Wait, do not destroy: the event belongs to the read session. Skip a
         * zero wait_event because CUDA does not accept a null event. */
        int groupOk = 1;
        if (g->data.tensor_count > 0 && g->data.tensors[0].device.device_type == kDLCUDA &&
            g->data.cuda_sync.wait_event != 0) {
            const cudaError_t w = cudaStreamWaitEvent(
                myStream, (cudaEvent_t)g->data.cuda_sync.wait_event, 0);
            if (w != cudaSuccess)
                groupOk = 0;
        }

        /* A fixed group carries ONE stacked tensor; an array group carries one per prim.
         * Iterating tensor_count is the only form correct for both -- tensors[0] silently
         * consumes the first prim's row and drops the rest of an array group. */
        for (uint32_t ti = 0; groupOk && ti < g->data.tensor_count; ++ti) {
            const DLTensor* t = &g->data.tensors[ti];

            /* Dispatch on BOTH dtype and device. Columns are not all float32 -- shapeCount is
             * int32 and the actor flags are uint8 -- and a host column must not be handed to a
             * CUDA kernel. */
            if (t->device.device_type != kDLCUDA) {
                consume_host_rows(t);
                continue;
            }
            if (t->dtype.code != kDLFloat || t->dtype.bits != 32) {
                /* Device column of some other element type: handle it or skip it, but do not
                 * reinterpret it as float. */
                continue;
            }

            /* Dense (strides == NULL, byte_offset == 0): rows are contiguous, dtype.lanes wide. */
            const float* rows = (const float*)((const char*)t->data + t->byte_offset);
            launch_my_kernel(rows, t->shape[0], myStream);
        }

        /* Released on EVERY path out of the group, including the failure above: breaking out
         * first would leak this group's prim list for the life of the session. */
        if (finish_task(handle, ovphysx_release_group(handle, read, g->read_group_id)).status != OVPHYSX_API_SUCCESS)
            groupOk = 0;

        if (!groupOk) {
            ok = 0;
            break;
        }
    }

    /* THE CONSUMER MUST FINISH BEFORE THE SESSION GOES AWAY. A device column points into
     * session-owned memory, and ovphysx_release_read frees it. The wait above orders the
     * producer BEFORE your kernel; nothing orders your kernel before this free, so without
     * this synchronize the kernel reads memory the next line releases.
     *
     * Unconditional: a failed loop may still have launched kernels over columns that are about
     * to be freed. */
    if (cudaStreamSynchronize(myStream) != cudaSuccess)
        return -1;

    /* One cleanup path, and every status checked. Both run even after a failure -- the session
     * and query own memory regardless of how the loop ended. */
    if (finish_task(handle, ovphysx_release_read(handle, read)).status != OVPHYSX_API_SUCCESS)
        ok = 0;
    if (finish_task(handle, ovphysx_release_query(handle, query)).status != OVPHYSX_API_SUCCESS)
        ok = 0;

    return ok;
}
```

On `-1`, the helper keeps the read and query alive because consumer completion is unknown.
The caller still owns both handles and must establish completion before releasing the session
or destroying its instance. On `0`, consumers have finished but another operation failed;
stop using these resources and destroy the instance before continuing, since a rejected release
can retain caller ownership. Accepted releases must not be resubmitted.

Copying each column to your own memory inside the loop works equally well and lets the
session go earlier; the synchronize is the cheaper option when the read is consumed once.

Issue the CUDA wait with the consumer's context current. Context-relative default
stream handles refer to that context; ovphysx does not choose the consumer stream.

`cuda_sync.stream` is always `0` — the read never asks a consumer to drain a stream.
Release each group when done with it, then the session; a device column's memory
belongs to the session and must not be read after `ovphysx_release_read`.

The current output-read producer zero-initializes
`meta.layout_generation` and `meta.attribute_write_floor_ordinal`. Do not use
either field as an invalidation or sealing signal. The application owns ovstage
ordinal advancement, and known structural edits require an explicit rebuild of
cached prim order and shapes.

Fixed groups contain one stacked tensor. Array groups contain one tensor per
prim. DLPack `dtype.lanes` carries tuple width.

That difference decides how many tensors to read, and taking `tensors[0]` is wrong
for half of them:

```c
#include <ovphysx/ovphysx.h>

extern void consume(const DLTensor* tensor);

static void consume_group(const ovstage_read_group_t* group)
{
    for (uint32_t i = 0; i < group->data.tensor_count; ++i)
        consume(&group->data.tensors[i]);
}
```
