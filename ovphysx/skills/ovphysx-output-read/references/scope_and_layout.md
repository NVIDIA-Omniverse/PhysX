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
(`jacobian`, `massMatrix`, `coriolisForce`, `gravityForce`, `centroidalMomentum`).
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

`dofAxisKind` is that same padded DOF column. Each entry is the PhysX joint-frame axis id
(0 twist, 1 swing1, 2 swing2, 3 x, 4 y, 5 z), not a USD axis token and not the unlocked-axis
ordinal in `dofAxisIndex`. A revolute about Y is 0 and a prismatic along Y is 3. A 0 inside
`dofCount` is twist; only entries past `dofCount` are padding. A locked twist leaves swing1 as
kind 1 and ordinal 0.

`dofJointType` is that same padded DOF column. Each entry is the PhysX joint type: 1 prismatic,
2 revolute, 3 revolute unwrapped, 4 spherical. A 0 past `dofCount` is padding. For unique joint leaf names, the legacy DOF
name is the leaf of `dofJointPath`, plus `:` and `dofAxisKind`, only when the type is spherical.
Colliding joint leaves receive numeric suffixes before the spherical axis suffix; use full prim paths for identity.

`linkPath` is the link prim for each `getLinkIndex()` slot, padded to the widest selected
articulation's `linkCount`, with 0 past that count. Slot 0 is the root link. The articulation row
prim may be that link, an ancestor Xform, or a fixed joint that is a child of slot 0. Fixed base:
link 0 has no Jacobian rows, and link L>0 occupies rows `6*(L-1)` through `6*L-1`. Floating base:
link L occupies rows `6*L` through `6*L+5`. Shape columns are link-major in this order, then
per-shape within the link. An `ARTICULATION_LINK` query stays in database order and is not
guaranteed to match `linkPath` order.

`linkInboundJoint` is the inbound joint prim of each `linkPath` slot. A 0 inside `linkCount`
means that link has no inbound joint. Slot 0 is 0. A fixed joint between two links is included.
`dofJointPath` skips zero-DOF joints. For unique joint leaf names, the legacy joint-name list is the leaf of each non-zero slot.
Legacy names disambiguate colliding leaves with numeric suffixes.

Tendon identity is four host columns, each in its own scene-wide group: `fixedTendonRoot` and
`fixedTendonInstance`, counted by `fixedTendonCount`, and `spatialTendonRoot` and
`spatialTendonInstance`, counted by `spatialTendonCount`. Slot T is the index
`getFixedTendons` / `getSpatialTendons` already uses. Each tendon is the root prim plus the
multi-apply instance name; a root prim alone is not unique. That index is not database order.
`FIXED_TENDON` and `SPATIAL_TENDON` rows carry `tendonInstance` in database order. Pair that token
with the row prim to find slot T. A zero-width group is omitted while its count column stays.
