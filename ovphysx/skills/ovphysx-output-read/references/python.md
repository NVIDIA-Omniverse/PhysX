# Python Output Reads

Read this file for the Python `PhysX.read()` or `PhysX.read_tokens()` branch.

## Open the read

Use semantic strings when the requested columns are known:

```python
from ovphysx.types import ObjectScope, SimObjectType

with physx.read(
    SimObjectType.RIGID_BODY,
    ["position", "orientation"],
    ObjectScope.ACTIVE,
) as result:
    for group in result.groups:
        column_copies = group.tensors
        prim_list = group.prim_list
        emitted_attribute = group.attribute
```

Use `read_tokens(object_type, attribute_tokens, scope)` when the caller already
has interned attribute tokens. Both entry points open the native query/read
sessions and drain all groups before returning the context-managed result.
Attributes that the selected type does not emit are skipped. A type with no
matching objects yields an empty `groups` list.

## Ownership

`group.tensors`, `group.index_map`, and `group.prim_index_map` are NumPy
copies and may outlive the context. The following values belong to the attached
Stage's shared dictionary and are used only while `ReadResult` is open:

- `group.prim_list`
- `group.attribute`
- `result.dictionary`

The context manager releases every native group, then the read and query
sessions. Closing the parent `PhysX` instance also invalidates the result.

## Scope and Layout

- The rigid-body `ACTIVE` example above is supported. Before using `ACTIVE` for
  another object type, or caching prim order and tensor shapes, read
  [Scope and layout constraints](scope_and_layout.md). In particular, do not
  wait for `group.layout_generation` to change before rebuilding after a known
  structural edit; the current output-read producer reports zero.
- Fixed groups contain one stacked tensor. Array groups contain one tensor per
  prim. The trailing NumPy dimension is the tuple width represented natively by
  DLPack `dtype.lanes`.
- Point-instancer `position` and `orientation` requests emit the attribute
  tokens `positions` and `orientations` in instancer-local space.
