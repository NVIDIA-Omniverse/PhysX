<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Data model and semantics

This page explains the important notions of the read and write API and provides all the information
required to *interpret* the values a read or write exchanges. 

## Groups and shape

A read or write returns its data as **groups**, one per attribute. A group is a struct holding four
things:

- **column**: the values (`tensors`).
- **prims**: the objects associated with the API call (`prim_list`), row *i* ↔ prim *i*.
- **attribute**: the corresponding `attribute`.
- **role**: the USD geometric `semantic` (see below).

The column has one row per object, in one of two shapes:

- **Fixed group** — rectangular: `N` objects give `N` rows, each row the value (a position is 3
  numbers, an orientation 4).
- **Array group** — each object's value is a list, and the lengths differ (e.g. a joint's number of
  DOFs), so it can't be rectangular: you get **one array per object**.

The shape is indicated by `is_array`.

The role is used mainly by ovstage to round-trip the data as the right type on a
write. The read/write path uses five roles:

| Role | Width | Meaning | Attributes |
|---|---|---|---|
| `POINT` | vec3 (3 × f32) | a position | `position`, `points`, `positions`, `restPoints`, `rootPosition`, `centerOfMassPosition`, `centerOfMassWorld`, `centerOfMassLocal` |
| `QUATERNION` | quat (4 × f32, **xyzw**) | a rotation | `orientation`, `orientations`, `rootOrientation`, `centerOfMassOrientation` |
| `VECTOR` | vec3 (3 × f32) | a free vector (no origin) | all linear/angular velocities and accelerations |
| `MATRIX` | 9 × f32 (3×3, column-major) | a matrix | `inertia`, `inverseInertia` |
| `NONE` | scalar / flag / count / any width | no USD role | masses, per-shape and per-axis scalars, flags, counts, inverse dynamics matrices, indices |

Two things to watch for:

- **The frame is not part of the role.** A `POINT` may be world (`position`), body-local
  (`centerOfMassPosition`), or mesh-local (deformable `points`). Read the per-attribute tables for
  the frame.
- **A pose is two columns, never one.** There is no fused transform role: a body pose is a `POINT`
  `position` plus a `QUATERNION` `orientation`; a root pose is `rootPosition` + `rootOrientation`.
  Writing one half [pre-reads the other](writable.md#write-timing-ordering-and-persistence) to
  preserve it.

## Articulation inverse dynamics cohorts

The whole-articulation inverse dynamics columns (`jacobian`,
`jacobianShape`, `massMatrix`, `coriolisForce`, `gravityForce`, `centroidalMomentum`) are a special case
as their size can be quite large. In order to be memory efficient, the notion of **cohort** has
been introduced. A **cohort** is a set of structurally identical articulations, that is, same joint types and names, DOF
parentage, and link leaf names.

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
| vehicle `steerAngle` | **radians** | write-only control |
| `mass`, `inertia` | mass, mass·length² | |

> **Note.** The asymmetry is deliberate to call out: a rigid body's `angularVelocity` is **rad/s**,
> but an articulation joint's `jointVelocity` is in **degrees/s** (USD-authored). Read/write of the
> same quantity share one fold, so a read and a write on the same object never disagree — but two
> *different* object types do not share a convention.

## Availability and fallback

### Step-first precondition (DirectGPU)

On a DirectGPU scene (`eENABLE_DIRECT_GPU_API`) PhysX sizes its GPU structures during the **first
step** and rejects direct-GPU access until it has run. Therefore, before the first step:

- a **read** emits **no groups** for device-sourced types (it reports end-of-iteration immediately —
  not an error, and currently indistinguishable from a genuinely empty match), and
- a **write** is **refused** (it is not auto-warmed).

You control the first step with `ovphysx_step` / `ovphysx_step_sync` / `ovphysx_warmup`. A **CPU
scene** *does* report authored initial state before the first step. Readiness is per-scene: a
multi-scene read returns ready partitions while omitting an unready DirectGPU scene.

### Missing and partial data

Reads are **dense** — an unavailable quantity is *omitted*, never NaN- or zero-filled to a full row:

- **Unknown or inapplicable attribute for the type** → no group is emitted for it; the drain ends
  with end-of-iteration (not an error). Name dispatch is exhaustive — a name never falls through to a
  neighbouring attribute.
- **Attribute inapplicable to a particular object** (e.g. `mass` on an instanced body, a surface-only
  material field on a volume material, `collisionElementIndices` on a surface) → that object
  contributes no row/column; a group's prim set can be a **subset** of discovery. Always pair values
  to prims via the group's own `prims.list`.
- **Disabled rigid body** → on a **DirectGPU** scene it has no device state, so it is dropped from
  the group (fewer rows than discovery reports); on a **CPU** scene it stays in. A disabled
  point-instancer instance leaves its zero-filled slot.
- **Per-shape padding** past `shapeCount` is deterministic zero-fill, but `0.0` is a legal value —
  use `shapeCount` (not the zeros) as the terminator.
- There is **no per-object "pending vs current" fallback** in the read (that concept belongs to the
  write/kinematic path). Derived read-only columns (`jointProjectedForce`, `linkIncomingJointForce`)
  are recomputed each read, never stale.

A short set is distinguishable from a complete one by the **terminal status**: a failed backend
build/gather ends with an error, a complete drain ends with end-of-iteration.
