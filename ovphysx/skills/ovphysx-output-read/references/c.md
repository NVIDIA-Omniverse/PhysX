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
`ovstage_read_group_t const*`. The group, its tensors, prim list, and tokens
remain valid only until the group or read session is released.

Preserve these fields when forwarding a group:

- `prims.list`, `prims.offset`, `prims.count`, and `prims.index_map`
- emitted `attribute`, `semantic`, `is_array`, and `is_delete`
- `data.tensors`, `data.tensor_count`, `data.index_map`, and `data.cuda_sync`

The current output-read producer zero-initializes
`meta.layout_generation` and `meta.attribute_write_floor_ordinal`. Do not use
either field as an invalidation or sealing signal. The application owns ovstage
ordinal advancement, and known structural edits require an explicit rebuild of
cached prim order and shapes.

Fixed groups contain one stacked tensor. Array groups contain one tensor per
prim. DLPack `dtype.lanes` carries tuple width.
