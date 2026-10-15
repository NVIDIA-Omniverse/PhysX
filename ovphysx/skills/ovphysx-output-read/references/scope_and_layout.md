# Scope and Layout Constraints

Read this file when using `ACTIVE` scope, consuming discovery counts for an
`ACTIVE` query, or caching prim order and tensor shapes across structural edits.

## `ACTIVE` Support

`ACTIVE` has two independently implemented behaviors: filtering the groups
returned by a read, and filtering the `total_prim_count` returned by discovery
(the optional query or read result).
The current implementation supports them as follows:

| Simulated object type | Read filters `ACTIVE` | Discovery filters `ACTIVE` |
| --- | --- | --- |
| rigid body | yes | yes |
| articulation link | yes | yes |
| whole articulation | yes | yes |
| articulation joint | no | no |
| whole vehicle | yes | yes |
| vehicle wheel | yes | yes |
| deformable volume or surface | yes | yes |
| deformable material | n/a | n/a |
| particle set | n/a | n/a |
| fixed tendon | no | no |
| spatial tendon | no | no |
| character controller | n/a (no readable columns) | yes |

Route scope choices conservatively:

- Use `ACTIVE` normally for rigid bodies, articulation links, vehicles and vehicle wheels,
  which follow the scene's most recent active-actor report when available.
  Without reporting, rigid bodies use current awake, non-kinematic state;
  links use articulation awake state; vehicles and wheels use chassis awake state.
  Deformables use current awake state. The query can be reused; open a new
  read session after every completed step, since each session evaluates the
  active set when it opens. The query's own discovery result stays a
  creation-time count.
- Whole-articulation `ACTIVE` includes a root when any link is active, and its
  discovery count uses the same filter. Availability is checked per scene; a
  scene without active-actor reporting falls back to articulation awake state.
  DirectGPU currently disables sleeping, so ACTIVE is equivalent to ALL there.
- Character-controller `ACTIVE` follows its proxy actor's active-actor membership
  when reported, otherwise that actor's awake state. The type has no readable
  columns; read its pose through `RIGID_BODY` instead.
- Use `ALL` for articulation joints because their read path currently ignores
  `ACTIVE`. That is an implementation limitation, unlike the material and particle-set cases.
- Use `ALL` for tendons. Their properties are authoring-time values that a step
  never changes, so there is no active set to filter by and `ACTIVE` reads as `ALL`.
- Deformable materials have no active set at all — a material is not a simulated
  body and never sleeps — so `ACTIVE` and `ALL` are the same query. `n/a` above
  means exactly that, not "unimplemented": there is nothing a later version could
  filter by, so this row will not change.
- Particle sets are the same, for a different reason. A read's rows map to particle
  SETS, and a set is a `PxParticleBuffer` attached to a system — not a `PxActor`, so
  it has no sleep state to report. (The particle SYSTEM is an actor, but
  `PxPBDParticleSystem` exposes no sleep API either.) A set that is disabled is
  already omitted from an `ALL` read, so that is not an `ACTIVE`-only filter.
  This row rests on rows meaning sets: if a later version ever mapped them to
  particle systems, the question reopens and so does this row.

Check the installed API documentation before relaxing this routing for a newer
ovphysx version.

## Structural Rebuilds

The current output-read producer zero-initializes `layout_generation`; it does
not bump after an object add/remove, point-instancer count change, or other
topology change. Do not use it to decide whether cached state remains valid.

After a known structural edit, discard and rebuild cached queries, prim order,
index maps, tensor shapes, and destination layout before consuming another
frame. Ordinary value changes and simulation steps do not require this rebuild.

This applies with extra force to the whole-articulation inverse dynamics matrices
(`jacobian`, `jacobianShape`, `massMatrix`, `coriolisForce`, `gravityForce`,
`centroidalMomentum`).
They are emitted one group per cohort of structurally identical articulations, so a
structural edit can change how many groups a read returns and which rows each covers,
not just their contents. Cohort membership is re-derived every read, so the data is
never stale -- but a cached group index is. Identify a cohort by its prim list rather
than by position, and do not assume a group count carries across a structural edit.

## Names → prim paths

There is no names read. The deprecated `ovphysx_articulation_get_{dof,body,joint}_names` are replaced
by prim-path identity already carried by the read:

- **Link and joint names** are the leaf of each `ARTICULATION_LINK` / `ARTICULATION_JOINT` prim.
  Resolve a group's `prims.list` entry through the attached Stage's path dictionary (the same round-trip
  you use for any group's prim list) and take its leaf. Prim paths are the unique identity the legacy leaf-name
  strings only approximated: those were de-duplicated with a fabricated `_N` suffix on collision, which
  resolves back to no prim.

- **DOF identity** is `(joint prim, axis index)`, carried per DOF column by the whole-articulation
  `dofJointPath` / `dofAxisIndex` attributes. Entry `k` names the joint prim and axis that inverse
  dynamics matrix column `(isFixedBase ? 0 : 6) + k` belongs to, so a `jacobian` / `massMatrix` column
  is joinable to its joint. `dofJointPath` is a `uint64` interned prim-path handle, resolved the same
  way as `prims.list`; `0` (`OVX_INVALID_PRIMPATH`) is the trailing pad past `dofCount`.

Unlike the inverse dynamics matrices above, these two columns are emitted as ONE scene-wide group -- one
row per articulation, the per-DOF array padded to the widest selected articulation's `dofCount` -- like
the topology-metadata columns. So read them alongside `dofCount` / `isFixedBase` from that one group,
not per cohort, and use `dofCount` as each row's real length.
