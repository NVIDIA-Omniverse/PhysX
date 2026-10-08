# C Query/Read Lifecycle

Read this file for the native C output-read branch.

## Handle Sequence

```text
ovphysx_query(type, scope)
  -> optional ovphysx_fetch_query_result
  -> optional ovphysx_query_shared_dictionary
  -> ovphysx_read(attributes)
  -> ovphysx_fetch_read_next until OVPHYSX_API_END_OF_ITERATION
       -> consume group
       -> ovphysx_release_group
  -> ovphysx_release_read
  -> ovphysx_release_query
```

`OVPHYSX_API_END_OF_ITERATION` is normal exhaustion. Every other non-success
status is an error. A successful query always has a nonzero handle, including an
empty match; an empty read reaches end-of-iteration immediately.

Release every fetched group before releasing its read session. Release the read
before its query on both success and error paths. Retrieve the same-thread error
string before cleanup if a failure must be reported.

## Discovery and Selector Semantics

`ovphysx_fetch_query_result()` reports the produced attribute tokens and total
prim count. Its arrays belong to the query until `ovphysx_release_query()`.
`ovphysx_query_shared_dictionary()` returns the runtime-owned ovstage path
dictionary that resolves the query's tokens and prim lists; the caller does not
free it.

The query is a lazy selector over simulated type and scope. Membership and
column values are evaluated against the latest completed step when discovery or
read occurs. A step between query and read makes the read observe the newer
step. Before using `OVPHYSX_SCOPE_ACTIVE` or relying on its discovery count,
read [Scope and layout constraints](scope_and_layout.md); support differs by
object type.

## Borrowed Groups

`ovphysx_fetch_read_next()` returns a producer-owned
`ovstage_read_group_t const*`. The group struct and stage-derived prim list remain
valid until the group or read session is released. Numeric tensor, prim-index-map,
data-index-map, mask, and CUDA-event storage remains valid until the read session is released.

Preserve these fields when forwarding a group:

- `prims.list`, `prims.offset`, `prims.count`, and `prims.index_map`
- emitted `attribute`, `semantic`, `is_array`, and `is_delete`
- `data.tensors`, `data.tensor_count`, `data.index_map`, and `data.cuda_sync`

Forwarding `cuda_sync` is not the same as honouring it. On a DirectGPU scene the
covered types emit **device-resident** (`kDLCUDA`) tensors, handed over before their
producing work has necessarily completed.

`ovphysx_cuda_stream_wait_event(stream, event)` is the ordering call, so honouring
the contract costs no CUDA dependency of your own — it routes through the driver
shim ovphysx already loads. Pass the stream as `uintptr_t`: `0` is the NULL stream,
`1` the legacy default, `2` the per-thread default, anything else a `CUstream`.

The loop below is `.cu`, not `.c` — the launch syntax is CUDA C++. Compile it as CUDA;
the ovphysx calls themselves are plain C and need no such thing.

```cuda
/* Compile as CUDA (.cu): the launch syntax below is CUDA C++. The ovphysx calls are plain C.
 *
 * Supplied by you: a kernel over one dense float column, and one over a host column. */
extern void launch_my_kernel(const float* rows, int64_t rowCount, cudaStream_t stream);
extern void consume_host_rows(const DLTensor* t);

/* Consumes every group of one read, then releases the read and the query. Returns 1 on success.
 * `handle` and `query` are yours; `read` is the session ovphysx_read() handed back. */
static int consume_read(ovphysx_handle_t handle,
                        ovphysx_query_handle_t query,
                        ovphysx_read_handle_t read,
                        cudaStream_t myStream)
{
    int ok = 1;

    for (;;) {
        const ovstage_read_group_t* g = NULL;
        const ovphysx_result_t r = ovphysx_fetch_read_next(handle, read, &g);
        if (r.status == OVPHYSX_API_END_OF_ITERATION)
            break;                       /* exhausted -- NOT an error */
        if (r.status != OVPHYSX_API_SUCCESS) {
            ok = 0;                      /* a real failure; do not treat it as end-of-data */
            break;
        }

        /* Ordered ONCE per group, not per tensor: cuda_sync describes the whole group, and a
         * group is device-uniform. Only needed when reading on your OWN stream -- the
         * default/null stream is already ordered after the producer work. Wait, do not destroy:
         * the event belongs to the read session. A zero wait_event is a no-op success. */
        int groupOk = 1;
        if (g->data.tensor_count > 0 && g->data.tensors[0].device.device_type == kDLCUDA) {
            const ovphysx_result_t w =
                ovphysx_cuda_stream_wait_event((uintptr_t)myStream, g->data.cuda_sync.wait_event);
            if (w.status != OVPHYSX_API_SUCCESS)
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
        if (ovphysx_release_group(handle, read, g->read_group_id).status != OVPHYSX_API_SUCCESS)
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
        ok = 0;

    /* One cleanup path, and every status checked. Both run even after a failure -- the session
     * and query own memory regardless of how the loop ended. */
    if (ovphysx_release_read(handle, read).status != OVPHYSX_API_SUCCESS)
        ok = 0;
    if (ovphysx_release_query(handle, query).status != OVPHYSX_API_SUCCESS)
        ok = 0;

    return ok;
}
```

Copying each column to your own memory inside the loop works equally well and lets the
session go earlier; the synchronize is the cheaper option when the read is consumed once.

The wait is issued in the calling thread's current CUDA context and ovphysx does not
push its own, so the context-relative sentinels `1` and `2` resolve against **your**
context — the one that queued the work being ordered.

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
if (g->is_array) {
    // One tensor per prim: tensor_count == prims.count, tensors[i] is prim i's own
    // variable-length array. A joint read over 16k joints is ONE group with 16k
    // tensors -- reading only tensors[0] gets the first joint and reports nothing.
    for (uint32_t i = 0; i < g->data.tensor_count; ++i)
        consume(&g->data.tensors[i]);
} else {
    // Every prim stacked into one tensor: tensor_count == 1, row i is prim i.
    consume(&g->data.tensors[0]);
}
```
