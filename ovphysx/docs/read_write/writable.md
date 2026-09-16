<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Writable data

The inventory of every quantity writable through the write API, per object type. Read-only quantities
are covered in [Readable data](readable.md); this page lists only what a write accepts.

Every table gives, per attribute, its **Meaning**, **dtype × lanes** (element type and tuple width),
and — where it applies — **Frame** and **Units**. Quaternions are xyzw. A `(write-only)` tag marks a
control the solver consumes then clears each step (not readable). Column residency follows the read:
device on DirectGPU, host on CPU — **except** joint DOF properties and particle `points`/`velocities`,
which are host-staged on **every** scene.

## Writability classes

To ask programmatically whether an `(object type, attribute)` pair is accepted:

- In C: `ovphysx_writability(object_type, attribute, &out)`
- In Python: `writability(object_type, attribute)`

It reports one of five classes — `OVPHYSX_WRITABILITY_<NAME>` in C (`ovphysx_writability_t`), the
matching integer in Python:

| Value | Class | Meaning |
|---|---|---|
| 0 | `UNCLASSIFIED` | the type does not accept this name; a write rejects it |
| 1 | `WRITABLE` | writable, no precondition beyond the object existing |
| 2 | `CONDITIONAL` | writable only under a stated condition; if unmet the write reports it and writes **nothing** (does not partially apply) |
| 3 | `WRITE_ONLY` | a control the solver consumes then clears each step; not readable |
| 4 | `READ_ONLY` | readable only; writing is an error naming the type, not a silent no-op |

The query is scene-independent (a hand-maintained snapshot of the runtime tables; string keys only;
the object type is bounds-checked). It classifies the *attribute*, not the object's current state.

## Write timing, ordering, and persistence

- **A first step is required.** A write before the scene's first step is refused (not auto-warmed).
- **Applied at commit, into live PhysX** — not deferred to the next step. A written value is
  observable through a read (or the raw PhysX pointer, `ovphysx_get_physx_ptr` / `get_physx_ptr`)
  before the next step. Commit orders the write against an in-flight step and honours the CUDA sync
  you pass it — the `write_done_sync` argument in C, the `cuda_stream=` keyword in Python.
- **Read-modify-write for split poses.** Writing `position` pre-reads and preserves `orientation`
  (and vice versa), because they are two slices of one atomic transform. Linear and angular velocity
  are independent (no pre-read). Particle/deformable point writes preserve the `.w` inverse-mass
  lane.
- **What the sim overwrites.** *State* (poses, velocities, root state, joint DOF state, point sets)
  is recomputed by the next solver step. *Inputs, targets, and properties* (drive targets, all joint
  and tendon properties, materials, mass/inertia/COM, flags, wheel controls) persist across steps
  until re-written. *Write-only controls* (`force`, `wrench`, `driveTorque`, `brakeTorque`,
  `steerAngle`) are consumed and cleared each step.
- **Persistence is runtime-only.** Every write publishes into live PhysX simulation state. **No
  write is authored back into the ovstage / USD source representation.** (A read gathers from the
  same live state, which is why a written value round-trips through a read.) Propagating results back
  into ovstage is a separate output-drain direction, not this write path — and its coverage differs
  from the writability query (see the last item in [Known limitations](limitations.md)).
- **No fill mask; one attribute per session.** A commit publishes the whole mapped group — every
  object of the queried type in scope. There is currently **no way to write a subset of prims**:
  a query cannot pick individual prims (per-prim selection is a planned extension).
  Rows are re-resolved at every commit, never cached across sessions or steps.
- **Commit consumes the group.** After a commit — `ovphysx_commit_group` in C, `session.commit(group)`
  in Python — the mapped buffers belong to physics (dereferencing the C pointers, or touching the
  Python tensor views, is undefined) and commit is not idempotent. A group left uncommitted when the
  session is released (or the Python `with` block exits) is discarded, not published.

## Rigid bodies

Rigid bodies (`RIGID_BODY`): all rigid **state** and **authored inputs** are writable; derived
quantities (`linearAcceleration`, `angularAcceleration`, `inverseMass`, `inverseInertia`,
`shapeCount`) are read-only.

| Attribute | Meaning | dtype × lanes | Frame | Units |
| --- | --- | --- | --- | --- |
| `position` | body position | f32 × 3 | world | length |
| `orientation` | rotation | f32 × 4 | world | — |
| `linearVelocity` | COM linear velocity | f32 × 3 | world | length/s |
| `angularVelocity` | angular velocity | f32 × 3 | world | rad/s |
| `mass` | body mass | f32 × 1 | — | mass |
| `inertia` | inertia tensor | f32 × 9 | COM | mass·length² |
| `centerOfMassPosition` | COM offset | f32 × 3 | body | length |
| `centerOfMassOrientation` | COM frame rotation | f32 × 4 | body | — |
| `disableGravity` | gravity-off flag | u8 × 1 | — | bool |
| `disableSimulation` | excluded-from-sim flag | u8 × 1 | — | bool |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — |
| `restitution` | per-shape restitution | f32 × per-shape | — | — |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length |
| `force` | external force at COM (write-only) | f32 × 3 | world | force |
| `wrench` | external force xyz + torque xyz + world application point (write-only) | f32 × 9 | world | force / torque |

Point-instancer instances are written in the same session as array groups (`position` / `orientation`
local, velocities world). On DirectGPU an instancer group holding a disabled instance is refused as a
whole (writes nothing) — asymmetric with standalone bodies, whose disabled rows are silently filtered.

Writing `disableSimulation` retains disabled rows in the group, so writing `0` re-enables a body.
This disables a **standalone** body only — a point-instancer **instance** cannot be disabled
individually (`disableSimulation` is not an instancer-writable column and there is no per-instance
route), which is unsupported in this release. **Do not toggle `eDISABLE_SIMULATION` directly on a
`PxRigidDynamic*` obtained from `ovphysx_get_physx_ptr`** — ovphysx does not observe the change, and
on a DirectGPU scene the next read or write may resolve a stale GPU index to the wrong body without
error. Write `disableSimulation` through the write API instead.

## Articulation links

Articulation links (`ARTICULATION_LINK`) share the rigid body's writable **properties**, with two
exceptions: pose and velocity are read-only (a link moves through its articulation's root pose plus
joint state), and so is `disableSimulation` (a link cannot be excluded from its articulation's
simulation individually). The table below is exactly the writable set.

| Attribute | Meaning | dtype × lanes | Frame | Units |
| --- | --- | --- | --- | --- |
| `mass` | link mass | f32 × 1 | — | mass |
| `inertia` | inertia tensor | f32 × 9 | COM | mass·length² |
| `centerOfMassPosition` | COM offset | f32 × 3 | body | length |
| `centerOfMassOrientation` | COM frame rotation | f32 × 4 | body | — |
| `disableGravity` | gravity-off flag | u8 × 1 | — | bool |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — |
| `restitution` | per-shape restitution | f32 × per-shape | — | — |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length |
| `force` | external force at COM (write-only) | f32 × 3 | world | force |
| `wrench` | external force xyz + torque xyz + world application point (write-only) | f32 × 9 | world | force / torque |

## Articulation joints — DOFs

Articulation joints (`ARTICULATION_JOINT`) write an array group, one value per enabled axis; the unit
fold is applied once by the setter. Rotational axes are in **degrees** (as USD-authored).

State and control:

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointPosition` | generalized position | f32 × per-axis | deg / length |
| `jointVelocity` | generalized velocity | f32 × per-axis | deg/s / length/s |
| `jointPositionTarget` | drive position target | f32 × per-axis | deg / length |
| `jointVelocityTarget` | drive velocity target | f32 × per-axis | deg/s / length/s |
| `jointActuationForce` | applied actuation effort | f32 × per-axis | force / torque |

Properties (authoring inputs; persist across steps):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointStiffness` | drive effort per unit position error | f32 × per-axis | effort/coord |
| `jointDamping` | drive effort per unit rate error | f32 × per-axis | effort/rate |
| `jointLimit` | (low, high) motion limit | f32 × 2 per axis | degrees / length; `FLT_MAX` = free |
| `jointMaxVelocity` | joint rate bound | f32 × per-axis | USD rate |
| `jointMaxForce` | drive effort bound | f32 × per-axis | force |
| `jointArmature` | added rotor inertia | f32 × per-axis | inertia |
| `jointStaticFriction` | break-away effort | f32 × per-axis | effort |
| `jointDynamicFriction` | sliding effort | f32 × per-axis | effort |
| `jointViscousFriction` | effort per unit rate | f32 × per-axis | effort/rate |
| `jointSpeedEffortGradient` | drive-envelope rate per effort | f32 × per-axis | USD |
| `jointMaxActuatorVelocity` | actuator rate bound | f32 × per-axis | USD rate |
| `jointVelocityDependentResistance` | envelope resistance per rate | f32 × per-axis | effort/rate |
| `jointDriveType` | 0 none / 1 force / 2 accel | u8 × per-axis | enum |

`jointLimit` is **conditional**: writing a *finite* limit onto a non-limited axis is refused and
writes nothing (PhysX forbids making an axis limited while its articulation is in a scene); a
`±FLT_MAX` (free) sentinel is a legal no-op. After writing `jointPosition`, call
`ovphysx_update_articulations_kinematic` (C) / `update_articulations_kinematic()` (Python) to refresh
dependent link transforms (a no-op on CPU — see [Known limitations](limitations.md)).

## Whole articulations

Whole articulations (`ARTICULATION`) accept root state only; COM, the inverse dynamics matrices, and per-shape
properties are read-only.

| Attribute | Meaning | dtype × lanes | Frame | Units |
| --- | --- | --- | --- | --- |
| `rootPosition` | root-link position | f32 × 3 | world | length |
| `rootOrientation` | root-link rotation | f32 × 4 | world | — |
| `rootLinearVelocity` | root linear velocity | f32 × 3 | world | length/s |
| `rootAngularVelocity` | root angular velocity | f32 × 3 | world | rad/s |

The rigid `position` / `velocity` names are refused on this type — use the `root*` names.

## Vehicle wheels

Vehicle wheels (`VEHICLE_WHEEL`) are **CPU-only**, and require a prior vehicle *read* on
the scene to cache the wheel enumeration. All three writable controls are write-only. The composed
wheel pose (`position` / `orientation`) is read-only.

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `driveTorque` | drive torque (write-only) | f32 × 1 | torque |
| `brakeTorque` | brake torque (write-only) | f32 × 1 | torque |
| `steerAngle` | steer angle (write-only) | f32 × 1 | radians |

## Deformable volumes / surfaces

Deformable volumes / surfaces (`DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE`) are **GPU-only,
device-resident**. `restPoints` and the connectivity indices are read-only; `kinematicTarget` is not
accepted by name (its buffer must outlive the call, and a write session's column is freed at release).

| Attribute | Meaning | dtype × lanes | Frame |
| --- | --- | --- | --- |
| `points` | current nodal positions (scale-corrected; `.w` inverse-mass lane preserved) | f32 × 3 | sim-mesh-local |
| `velocities` | nodal velocities | f32 × 3 | world |

## Particle sets

Particle sets (`PARTICLE_SET`) are **host-staged on every scene**, uploaded to the device at the next
step; readable before then.

| Attribute | Meaning | dtype × lanes | Frame |
| --- | --- | --- | --- |
| `points` | particle positions (scale-corrected; `.w` inverse-mass lane preserved) | f32 × 3 | particle-set-prim-local |
| `velocities` | particle velocities | f32 × 3 | world |

## Fixed / spatial tendons

Fixed / spatial tendons (`FIXED_TENDON`, `SPATIAL_TENDON`) are device-resident on DirectGPU, host
otherwise; all persist. `tendonLimit` and `tendonRestLength` are refused by name on a spatial tendon
(they live on the leaf attachment).

| Attribute | Meaning | dtype × lanes | Fixed | Spatial |
|---|---|---|---|---|
| `tendonStiffness` | stiffness | f32 × 1 | yes | yes |
| `tendonDamping` | damping | f32 × 1 | yes | yes |
| `tendonLimitStiffness` | limit-region stiffness | f32 × 1 | yes | yes |
| `tendonOffset` | offset | f32 × 1 | yes | yes |
| `tendonLimit` | (low, high) length limit | f32 × 2 | yes | — |
| `tendonRestLength` | rest length | f32 × 1 | yes | — |

## Deformable materials

Deformable materials (`DEFORMABLE_MATERIAL`) are host-resident, scene-less; all persist.
Bending/thickness fields apply to surface materials only (a volume material takes no row).

| Attribute | Meaning | dtype × lanes | Applies to |
|---|---|---|---|
| `deformableDynamicFriction` | dynamic friction | f32 × 1 | volume + surface |
| `deformableYoungsModulus` | Young's modulus | f32 × 1 | volume + surface |
| `deformablePoissonsRatio` | Poisson's ratio | f32 × 1 | volume + surface |
| `deformableElasticityDamping` | elasticity damping | f32 × 1 | volume + surface |
| `deformableBendingStiffness` | bending stiffness | f32 × 1 | surface only |
| `deformableThickness` | shell thickness | f32 × 1 | surface only |
| `deformableBendingDamping` | bending damping | f32 × 1 | surface only |
