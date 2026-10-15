<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Data model and semantics

This page explains the important notions of the read and write API and provides all the information
required to *interpret* the values a read or write exchanges. 

## Groups and shape

A read or write returns its data as **groups**, with one attribute per group. Groups can be
partitioned by scene and, for inverse dynamics, by cohort. Every group carries the numeric column
and the associated prims. A **read** group (`ovstage_read_group_t`) also identifies the attribute
and its USD geometric role:

- **column**: the values (`tensors`).
- **prims**: the objects associated with the API call (`prim_list`), row *i* ↔ prim *i*.
- **attribute**: the corresponding `attribute`.
- **role**: the USD geometric `semantic` (see below).

The column has one row per object, in one of two shapes:

- **Fixed group** — rectangular: `N` objects give `N` rows, each row the value (a position is 3
  numbers, an orientation 4).
- **Array group** — each object's value is a list, and the lengths differ (e.g. a joint's number of
  DOFs), so it can't be rectangular: you get **one array per object**.

A read group indicates the shape with `is_array`. A **write** group (`ovstage_map_group_t`) has no
`attribute`, `semantic`, or `is_array` field: each fetch reports the group's `attribute_index` into
the attribute array the write session was opened with, and that object type and attribute determine
the layout described in [Writable data](writable.md).

The role is used mainly by ovstage to round-trip the data as the right type on a
write. Read groups use six roles:

| Role | Width | Meaning | Attributes |
|---|---|---|---|
| `POINT` | vec3 (3 × f32) | a position | `position`, `points`, `positions`, `restPoints`, `rootPosition`, `centerOfMassPosition`, `centerOfMassWorld`, `centerOfMassLocal`, `kinematicTargetPosition`, `groundHitPosition`, the contact positions (`filteredContactPoint`, `filteredFrictionPoint`, `rawContactPoint`, `contactEventPointPosition`, `contactEventAnchorPosition`) |
| `QUATERNION` | quat (4 × f32, **xyzw**) | a rotation | `orientation`, `orientations`, `rootOrientation`, `centerOfMassOrientation`, `kinematicTargetOrientation` |
| `VECTOR` | vec3 (3 × f32) | a free vector (no origin) | all linear/angular velocities and accelerations, the net contact forces, `filteredFrictionForce`, the contact event impulses; `suspensionForce`, `tireForce`, `tireLongitudinalDirection`, `tireLateralDirection` |
| `NORMAL` | vec3 (3 × f32) | a surface normal | `filteredContactNormal`, `rawContactNormal`, `contactEventPointNormal` |
| `MATRIX` | 9 × f32 (3×3, column-major) | a matrix | `inertia`, `inverseInertia` |
| `NONE` | scalar / flag / count / any width | no USD role | masses, per-shape and per-axis scalars, flags, counts, inverse dynamics matrices, indices, the deformable `kinematicTarget`, the contact force matrices, scalar contact forces and separations, contact event types and sides, prim-handle identities |

Two things to watch for:

- **The frame is not part of the role.** A `POINT` may be world (`position`), body-local
  (`centerOfMassPosition`), or mesh-local (deformable `points`). Read the per-attribute tables for
  the frame.
- **A pose is two columns, never one.** There is no fused transform role: a body pose is a `POINT`
  `position` plus a `QUATERNION` `orientation`; a root pose is `rootPosition` + `rootOrientation`.
  Writing one half [pre-reads the other](writable.md#write-timing-ordering-and-persistence) to
  preserve it.

## Signed scale and body orientation

For rigid bodies and articulation links returned in fixed groups, ovphysx
decomposes the composed **world matrix**, including ancestor transforms, to
obtain the physics pose at attach and when draining OVStage transform edits.
Its `orientation` is a proper rotation, not necessarily the authored
`xformOp:orient`. A matrix does not uniquely identify the authored scale signs
and rotation.

For a finite, nondegenerate, shear-free world matrix, ovphysx uses this rule:

- A negative determinant of the linear 3x3 part produces **three negative
  scale components**. A positive determinant produces three positive components.
- The rotation absorbs the remaining sign changes. For example, two negative
  authored scale components become positive scales and a different rotation.
- For OVStage's row-vector matrix, the scale magnitudes are the lengths of the
  first three rows' xyz components. Reconstruct the linear part as
  `diag(signed_scale) * R_row(orientation)`, with position in the last row.
  For column-vector math, use `R_column(orientation) * diag(signed_scale)`
  and position in the last column.

For example, a body with no transformed parent, a 45-degree Z rotation
(`xyzw = (0, 0, 0.3826834, 0.9238795)`) and scale `(-2, 3, 4)` reads an
orientation equivalent to `xyzw = (0.9238795, 0.3826834, 0, 0)` and pairs with
scale `(-2, -3, -4)`. That pair reconstructs the original matrix; pairing the
read orientation with the authored `(-2, 3, 4)` does not. Quaternions `q` and
`-q` represent the same rotation.

Orientation writes use this same physics frame. Decompose a desired world
transform with the same rule before writing its rotation. In the example above,
writing the authored quaternion back does not restore the authored transform.
Body-local values such as `centerOfMassPosition` are also expressed in the
decomposed body's frame.
The [OVStage output utilities](../ovstage_integration.md#c-utility-publish-fixed-world-transforms)
pair physics poses with this signed scale; they do not convert authored-frame
orientation writes. Pose plus diagonal scale does not preserve shear.

Point-instancer pose arrays use the [instancer-local representation](readable.md#rigid-bodies);
do not apply this sign conversion to those arrays.

## Articulation inverse dynamics cohorts

The whole-articulation inverse dynamics columns (`jacobian`,
`jacobianShape`, `massMatrix`, `coriolisForce`, `gravityForce`, `centroidalMomentum`) are a special case
as their size can be quite large. In order to be memory efficient, the notion of **cohort** has
been introduced. A **cohort** is a set of structurally identical articulations: the same fixed or
floating base type, joint types and names, DOF parentage, and link leaf names.

For a homogeneous scene, only one cohort is present, while for a heterogeneous scene,
the results are **cohort-partitioned** with one group per cohort.
The cohort can be identified by its prim list; the
group *index* is not promised stable across reads. Every other column has one fixed shape for the
whole read, so it needs no cohorts.

**Layout.** Matrices are row-major flattened. `massMatrix` is square (`M = numDofs + (fixedBase ? 0 :
6)`), so `M` is the square root of the lane count; `coriolisForce` and `gravityForce` are length `M`;
`centroidalMomentum` has six rows, so its column count is `lanes / 6`, and its last column is the
bias rather than a coordinate. `jacobian`'s two dimensions both vary, so they are published in
`jacobianShape` as `(rows, cols)`.

**Sign convention.** Generalized coordinates follow each USD joint's authored direction: positive when
`body0` is the articulation parent, negative when `body1` is. With `S_dof` the diagonal matrix of
those signs and `T = S_dof` (fixed base) or `T = diag(I₆, S_dof)` (floating base), the values returned
are `J = J_physx·T`, `M = T·M_physx·T`, `c = T·c_physx`, `g = T·g_physx`, and `[A|b] =
[A_physx·T | b_physx]` — so centroidal's six root columns and bias column are unchanged.

## Units

ovphysx follows one rule at the boundary: **the parse/population layer converts to engine-native
units at the boundary, and consumers never re-convert.** Positions are in the scene's length unit,
which is the USD stage unit — see [Stage units](../population/stage_units.md) for `metersPerUnit` /
`kilogramsPerUnit`.

Within the session API the angular convention is **not uniform** — it depends on the quantity:

| Quantity | Unit | Notes |
|---|---|---|
| positions, poses (`position`, `rootPosition`, COM) | scene length units | world unless the table says local |
| rigid/link `linearVelocity` | length / s | engine-native |
| rigid/link `angularVelocity` | **radians / s** | engine-native — **no** degree fold |
| rigid/link `linearAcceleration` / `angularAcceleration` | length/s² · rad/s² | read-only |
| joint DOF `jointPosition` / `jointVelocity` / targets | **degrees** (rotational axes) | as USD-authored; linear axes in length |
| joint DOF efforts (`jointActuationForce`, `jointMaxForce`) | force / torque | no angular fold |
| whole-articulation inverse dynamics (`jacobian`, `massMatrix`, …) | **radians** for angular generalized coordinates | no degree fold; linear dimensions keep their units |
| vehicle `steerAngle`, `wheelSteerAngle`, `rotationAngle`, `tireLateralSlip` | **radians** | command or telemetry as documented |
| vehicle `rotationSpeed`, `engineRotationSpeed` | **radians / s** | angular speeds |
| vehicle `gearSwitchTime`, `autoboxTimeSinceLastShift` | seconds | native transmission state |
| vehicle suspension/tire forces and raw-wheel torques | `mass * length / s^2` and `mass * length^2 / s^2` | stage-unit conventions |
| `mass`, `inertia` | mass, mass·length² | |
| contact forces (`net*ContactForce`, matrices, `filtered*Force`, `rawContactForce`) | force | impulse / the scene's step dt |
| contact event impulses (`contactEventPointImpulse`, `contactEventAnchorImpulse`) | force · time | as reported, not divided by dt |

> **Note.** The asymmetry is deliberate to call out: a rigid body's `angularVelocity` is **rad/s**,
> but an articulation joint's `jointVelocity` is in **degrees/s** (USD-authored). Read/write of the
> same quantity share one fold, so a read and a write on the same object never disagree — but two
> *different* object types do not share a convention.

## Availability and fallback

### Step-first precondition (DirectGPU)

On a DirectGPU scene (`eENABLE_DIRECT_GPU_API`) PhysX sizes its GPU structures during the **first
step** and rejects direct-GPU access until it has run. Therefore, before the first step:

- a **read** of a type whose device state the first step produces (rigid bodies, articulation links,
  whole articulations, joint DOF) emits **no groups** — it reports end-of-iteration immediately. This
  is not an error. The optional read summary marks each such request
  `OVPHYSX_READ_OMISSION_NOT_READY`, which tells it apart from a genuinely empty match (see
  [Session results](index.md#session-results)), and
- a **write** session still opens, but **commit is refused** (it is not auto-warmed).

On **CPU** and on **GPU with readback** (no `suppressReadback`) a pre-step **write commits and is
applied**. The write never auto-warms in any mode.

**Particles are the exception.** A particle read serves the **authored** `points` / `velocities` from
host staging before the first step — the sim uploads them to the device at the first step, and
`buildParticleGroups` patches each not-yet-uploaded set from its host values — so a pre-step particle
read returns groups, not end-of-iteration.

**Contact sensors** are served before the first step too: their fixed columns (net forces, force
matrices, filtered counts) read zero on the device, and their per-contact arrays are absent — there is no
contact yet.

You control the first step with `ovphysx_step` / `ovphysx_step_sync` / `ovphysx_warmup`. A **CPU
scene** *does* report authored initial state before the first step. Readiness is per-scene: a
multi-scene read returns ready partitions while omitting an unready DirectGPU scene.

### Missing and partial data

Reads are **dense** — an unavailable quantity is *omitted*, never NaN- or zero-filled to a full row:

- **Unknown or inapplicable attribute for the type** → no group is emitted for it; the drain ends
  with end-of-iteration (not an error), and the optional read summary row carries
  `OVX_INVALID_TOKEN` and `OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE`. Name dispatch is
  exhaustive — a name never falls through to a neighbouring attribute.
- **Attribute inapplicable to a particular object** (e.g. `mass` on an instanced body, a surface-only
  material field on a volume material, Standard transmission telemetry on a Basic vehicle,
  `collisionElementIndices` on a surface) → that object
  contributes no row/column; a group's prim set can be a **subset** of discovery. Always pair values
  to prims via the group's own `prims.list`.
- **Disabled rigid body** → on a **DirectGPU** scene it has no device state, so it is dropped from
  the group (fewer rows than discovery reports); on a **CPU** scene it stays in. A disabled
  point-instancer instance leaves its zero-filled slot.
- **Per-step element arrays** (contact detail, raw contacts, contact events) → a sensor with no
  element this step contributes no array, and a group with no element at all is not emitted.
- **Per-shape padding** past `shapeCount` is deterministic zero-fill, but `0.0` is a legal value —
  use `shapeCount` (not the zeros) as the terminator.
- There is **no per-object "pending vs current" fallback** in the read (that concept belongs to the
  write/kinematic path) — with one exception: a particle set with a **pending device upload**
  returns its host-staged `points` / `velocities`. This includes the authored values before the
  first step and later write-session values before the next step uploads them. Derived
  read-only columns (`jointProjectedForce`, `linkIncomingJointForce`) are recomputed each read, never
  stale.

The optional read summary reports, per request, whether selected objects were omitted because
the attribute is unsupported on the type, unsupported in this configuration, or not ready yet; see
[Session results](index.md#session-results). A backend failure already known when the session
opens fails the open task, and no session is published. A later fetch can still fail, for example
after its attachment is invalidated. A complete drain produces a successful fetch task with a null
group. Inspect that output only after the wait succeeds; a null output on a failed task does not
mean end of iteration.
