# Scope and Layout Constraints

Read this file when using `ACTIVE` scope, consuming discovery counts for an
`ACTIVE` query, or caching prim order and tensor shapes across structural edits.

## `ACTIVE` Support

`ACTIVE` has two independently implemented behaviors: filtering the groups
returned by a read, and filtering the `total_prim_count` returned by discovery.
The ovphysx 0.6.0 baseline supports them as follows:

| Simulated object type | Read filters `ACTIVE` | Discovery filters `ACTIVE` |
| --- | --- | --- |
| rigid body | yes | yes |
| articulation link | yes | no |
| whole articulation | yes | yes |
| articulation joint | no | no |
| vehicle wheel | yes | yes |
| deformable volume or surface | yes | no |
| deformable material | n/a | n/a |
| particle set | n/a | n/a |
| fixed tendon | no | no |
| spatial tendon | no | no |

Route scope choices conservatively:

- Use `ACTIVE` normally for rigid bodies and vehicle wheels, reopening the query
  after every completed step.
- Whole-articulation `ACTIVE` includes a root when any link is active, and its
  discovery count uses the same filter. Availability is checked per scene; a
  scene without active-actor reporting falls back to articulation awake state.
  DirectGPU currently disables sleeping, so ACTIVE is equivalent to ALL there.
- For articulation links and deformables, an `ACTIVE` read is filtered but its
  discovery count is not. Use it only when the caller does not rely on
  `total_prim_count`; otherwise use `ALL`.
- Use `ALL` for articulation joints because their read path currently ignores
  `ACTIVE`. That is an implementation limitation, unlike the two `n/a` rows above.
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
