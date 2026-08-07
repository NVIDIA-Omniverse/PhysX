# Scope and Layout Constraints

Read this file when using `ACTIVE` scope, consuming discovery counts for an
`ACTIVE` query, or caching prim order and tensor shapes across structural edits.

## `ACTIVE` Support

`ACTIVE` has two independently implemented behaviors: filtering the groups
returned by a read, and filtering the `total_prim_count` returned by discovery.
The ovphysx 0.5.2 baseline supports them as follows:

| Simulated object type | Read filters `ACTIVE` | Discovery filters `ACTIVE` |
| --- | --- | --- |
| rigid body | yes | yes |
| articulation link | yes | no |
| articulation joint | no | no |
| vehicle wheel | yes | yes |
| deformable volume or surface | yes | no |
| particle set | no | no |

Route scope choices conservatively:

- Use `ACTIVE` normally for rigid bodies and vehicle wheels, reopening the query
  after every completed step.
- For articulation links and deformables, an `ACTIVE` read is filtered but its
  discovery count is not. Use it only when the caller does not rely on
  `total_prim_count`; otherwise use `ALL`.
- Use `ALL` for articulation joints and particle sets because their read path
  currently ignores `ACTIVE`.

Check the installed API documentation before relaxing this routing for a newer
ovphysx version.

## Structural Rebuilds

The current output-read producer zero-initializes `layout_generation`; it does
not bump after an object add/remove, point-instancer count change, or other
topology change. Do not use it to decide whether cached state remains valid.

After a known structural edit, discard and rebuild cached queries, prim order,
index maps, tensor shapes, and destination layout before consuming another
frame. Ordinary value changes and simulation steps do not require this rebuild.
