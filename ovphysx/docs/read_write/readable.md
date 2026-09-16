<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Readable data

The inventory of every quantity readable through the read API, per object type.

Every table lists the readable attributes for one object type.
**dtype × lanes** is the element type and tuple width; **Residency** is *device* (`kDLCUDA` on a
DirectGPU scene, `kDLCPU` on CPU) or *host* (always `kDLCPU`). Quaternions are xyzw.

## Rigid bodies

Standalone bodies (`RIGID_BODY`) emit a **fixed** group, one row per body:

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency / validity |
| --- | --- | --- | --- | --- | --- |
| `position` | body position | f32 × 3 | world | length | device |
| `orientation` | rotation | f32 × 4 | world | — | device |
| `linearVelocity` | COM linear velocity | f32 × 3 | world | length/s | device |
| `angularVelocity` | angular velocity | f32 × 3 | world | rad/s | device |
| `linearAcceleration` | linear acceleration | f32 × 3 | world | length/s² | device |
| `angularAcceleration` | angular acceleration | f32 × 3 | world | rad/s² | device |
| `mass` | body mass | f32 × 1 | — | mass | host |
| `inverseMass` | 1/mass | f32 × 1 | — | 1/mass | host |
| `inertia` | inertia tensor | f32 × 9 | COM | mass·length² | host |
| `inverseInertia` | inverse inertia | f32 × 9 | COM | — | host |
| `centerOfMassPosition` | COM offset | f32 × 3 | body | length | host |
| `centerOfMassOrientation` | COM frame rotation | f32 × 4 | body | — | host |
| `disableGravity` | gravity-off flag | u8 × 1 | — | bool | host |
| `disableSimulation` | excluded-from-sim flag | u8 × 1 | — | bool | host |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — | host |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — | host |
| `restitution` | per-shape restitution | f32 × per-shape | — | — | host |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length | host |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length | host |
| `shapeCount` | real shapes per row | i32 × 1 | — | count | host |

Per-shape columns are padded to the widest *queried* body's shape count (varies per read — do not
cache); read `shapeCount` for the real length. A shape with several materials reports only its first
(material 0). On DirectGPU, disabled (`eDISABLE_SIMULATION`) bodies are omitted from fixed columns
while discovery keeps the full count.

**Point-instancer instances** read only the state attributes, as an **array** group (one tensor per
instancer, indexed by instance slot, holes zero-filled) under different names. The property and
per-shape columns have no instancer form.

| Standalone | Instancer array | Frame |
| --- | --- | --- |
| `position` | `positions` | instancer-local |
| `orientation` | `orientations` | instancer-local |
| `linearVelocity` | `velocities` | world |
| `angularVelocity` | `angularVelocities` | world |
| `linearAcceleration` | `accelerations` | world |
| `angularAcceleration` | `angularAccelerations` | world |

## Articulation links

Articulation links (`ARTICULATION_LINK`) are rigid bodies, so they read the full rigid-body attribute set, plus one link-only column:

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency |
| --- | --- | --- | --- | --- | --- |
| `position` | link position | f32 × 3 | world | length | device |
| `orientation` | link rotation | f32 × 4 | world | — | device |
| `linearVelocity` | COM linear velocity | f32 × 3 | world | length/s | device |
| `angularVelocity` | angular velocity | f32 × 3 | world | rad/s | device |
| `linearAcceleration` | linear acceleration | f32 × 3 | world | length/s² | device |
| `angularAcceleration` | angular acceleration | f32 × 3 | world | rad/s² | device |
| `mass` | link mass | f32 × 1 | — | mass | host |
| `inverseMass` | 1/mass | f32 × 1 | — | 1/mass | host |
| `inertia` | inertia tensor | f32 × 9 | COM | mass·length² | host |
| `inverseInertia` | inverse inertia | f32 × 9 | COM | — | host |
| `centerOfMassPosition` | COM offset | f32 × 3 | body | length | host |
| `centerOfMassOrientation` | COM frame rotation | f32 × 4 | body | — | host |
| `disableGravity` | gravity-off flag | u8 × 1 | — | bool | host |
| `disableSimulation` | excluded-from-sim flag | u8 × 1 | — | bool | host |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — | host |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — | host |
| `restitution` | per-shape restitution | f32 × per-shape | — | — | host |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length | host |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length | host |
| `shapeCount` | real shapes per row | i32 × 1 | — | count | host |
| `linkIncomingJointForce` | spatial force the inbound joint exerts on the link | f32 × 6 (force xyz, torque xyz) | joint | force / torque | device |

## Whole articulations

Whole articulations (`ARTICULATION`) emit one **fixed** group with one row per articulation in root-record order. Inverse dynamics columns are
[cohort-partitioned](data_model.md#articulation-inverse-dynamics-cohorts).

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency |
| --- | --- | --- | --- | --- | --- |
| `rootPosition` | root-link position | f32 × 3 | world | length | device |
| `rootOrientation` | root-link rotation | f32 × 4 | world | — | device |
| `rootLinearVelocity` | root linear velocity | f32 × 3 | world | length/s | device |
| `rootAngularVelocity` | root angular velocity | f32 × 3 | world | rad/s | device |
| `centerOfMassWorld` | articulation COM | f32 × 3 | world | length | device |
| `centerOfMassLocal` | articulation COM | f32 × 3 | root-link | length | device |
| `staticFriction` / `dynamicFriction` / `restitution` / `contactOffset` / `restOffset` | per-shape (all links) | f32 × per-shape | — | — | host |
| `shapeCount` | total shapes across links | i32 × 1 | — | count | host |
| `jacobian` | dense Jacobian | f32 × rows·cols | — | angular in radians | device |
| `jacobianShape` | (rows, cols) of `jacobian` | i32 × 2 | — | count | host |
| `massMatrix` | generalized mass matrix | f32 × M·M | — | — | device |
| `coriolisForce` | Coriolis + centrifugal compensation | f32 × M | — | — | device |
| `gravityForce` | gravity compensation | f32 × M | — | — | device |
| `centroidalMomentum` | centroidal momentum matrix (floating-base only; a fixed-base articulation contributes no row) | f32 × 6·(numDofs+7) | — | — | device |

Root state and COM are always in scene-world coordinates.

> **Note.** A group's tuple width is a 16-bit field, so a single inverse dynamics column wider than 65 535
> (roughly a 64-link / 189-DOF articulation's Jacobian, or a mass matrix past M ≈ 256) is dropped
> and the read ends with an error. This is a known boundary for very large articulations.

## Articulation joints — DOFs

Articulation joints (`ARTICULATION_JOINT`) emit an **array** group with one tensor per joint prim, one lane per unlocked DOF axis
(`jointLimit` is 2). A read may ask for state, control, or properties in any combination. Rotational
axes carry the joint's body-order sign; angular DOF quantities are in **degrees** (as USD-authored).

> **Note.** A requested attribute produces **one** array group whose `tensors[i]` holds joint *i*'s
> axis values — *not* one group per joint. Code that reads `tensors[0]` and advances to the next
> group sees only the first joint, and does so silently on a single-joint scene.

State / control (device on DirectGPU):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointPosition` | generalized position | f32 × per-axis | deg / length |
| `jointVelocity` | generalized velocity | f32 × per-axis | deg/s / length/s |
| `jointPositionTarget` | drive position target | f32 × per-axis | deg / length |
| `jointVelocityTarget` | drive velocity target | f32 × per-axis | deg/s / length/s |
| `jointActuationForce` | applied actuation effort | f32 × per-axis | force / torque |
| `jointProjectedForce` | inbound joint force on this axis | f32 × per-axis | force / torque (read-only, derived) |

Properties (authoring inputs — **host-resident even on DirectGPU**):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointStiffness` | drive effort per unit position error | f32 × per-axis | effort/coord |
| `jointDamping` | drive effort per unit rate error | f32 × per-axis | effort/rate |
| `jointLimit` | (low, high) motion limit | f32 × 2 per axis | degrees / length; `FLT_MAX` = free |
| `jointMaxVelocity` | joint rate bound | f32 × per-axis | USD rate |
| `jointMaxForce` | drive effort bound | f32 × per-axis | force |
| `jointArmature` | added rotor inertia | f32 × per-axis | inertia |
| `jointStaticFriction` / `jointDynamicFriction` | break-away / sliding effort | f32 × per-axis | effort |
| `jointViscousFriction` | effort per unit rate | f32 × per-axis | effort/rate |
| `jointSpeedEffortGradient` | drive-envelope rate per effort | f32 × per-axis | USD |
| `jointMaxActuatorVelocity` | actuator rate bound | f32 × per-axis | USD rate |
| `jointVelocityDependentResistance` | envelope resistance per rate | f32 × per-axis | effort/rate |
| `jointDriveType` | 0 none / 1 force / 2 accel | u8 × per-axis | enum |

`jointLimit` is reported as the authored `(lower, upper)` pair, in order, regardless of the joint's
body order — unlike `jointPosition`, which negates when `body0` is the child.

`jointProjectedForce` and `linkIncomingJointForce` (on articulation links) are **computed per read**
rather than gathered from stored state, so each costs several times a four-column pose/velocity read
of the same prims — request them in the read that needs them, not in a per-step pose read.
`jointProjectedForce` uses the same one-lane-per-unlocked-axis shape as the other joint columns: a
locked axis is **omitted** (not zero-filled), and a fixed joint — which has no DOF — contributes no
row.

## Vehicle wheels

Vehicle wheels (`VEHICLE_WHEEL`) emit a **fixed** group with one row per wheel. **Host-only** — a vehicle cannot attach to a
DirectGPU scene.

| Attribute | Meaning | dtype × lanes | Frame | Residency |
| --- | --- | --- | --- | --- |
| `position` | composed wheel world position | f32 × 3 | world | host |
| `orientation` | composed wheel world rotation | f32 × 4 | world | host |

The wheel pose is composed from chassis + suspension + steer, not gathered. `driveTorque` /
`brakeTorque` / `steerAngle` are write-only and not readable.

## Deformable volumes / surfaces

Deformable volumes / surfaces (`DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE`) emit an **array** group with one tensor per sim-mesh prim. **GPU-only** — with no CUDA context the read emits no
deformable groups.

| Attribute | Meaning | dtype × lanes | Frame | Residency | Applies to |
| --- | --- | --- | --- | --- | --- |
| `points` | current nodal positions | f32 × 3 | **sim-mesh-local** | device | volume + surface |
| `velocities` | nodal velocities | f32 × 3 | **world** | device | volume + surface |
| `restPoints` | rest nodal positions | f32 × 3 | sim-mesh-local | host | volume + surface |
| `simElementIndices` | sim-mesh connectivity | i32 × 4 (tet) / 3 (tri) | — | host | volume + surface |
| `collisionElementIndices` | collision-mesh connectivity | i32 × 4 (tet) | — | host | **volume only** |

A surface body's simulation mesh *is* its collision mesh, so it has no separate
`collisionElementIndices` — discovery omits it for surfaces, and a surface query for it returns no
group.

> **Note.** `points` and `restPoints` share the sim-mesh-local frame (their difference is
> meaningful), but `velocities` is in **world** space. Do not difference `points` against a
> time-integrated `velocities`.

## Particle sets

Particle sets (`PARTICLE_SET`) emit an **array** group with one tensor per particle-set prim. **GPU-only.**

| Attribute | Meaning | dtype × lanes | Frame | Residency |
| --- | --- | --- | --- | --- |
| `points` (alias `positions`) | particle positions | f32 × 3 | particle-set-prim-local | device |
| `velocities` | particle velocities | f32 × 3 | world | device |

`positions` is accepted as a legacy alias and echoed under the spelling you requested; discovery
advertises only `points`.

## Fixed / spatial tendons

Fixed / spatial tendons (`FIXED_TENDON`, `SPATIAL_TENDON`) emit a **fixed** group with one row per tendon-root prim. Authoring values, but **device-resident on GPU** /
host on CPU.

| Attribute | Meaning | dtype × lanes | Fixed | Spatial |
|---|---|---|---|---|
| `tendonStiffness` | stiffness | f32 × 1 | yes | yes |
| `tendonDamping` | damping | f32 × 1 | yes | yes |
| `tendonLimitStiffness` | limit-region stiffness | f32 × 1 | yes | yes |
| `tendonOffset` | offset | f32 × 1 | yes | yes |
| `tendonLimit` | (low, high) length limit | f32 × 2 | yes | — |
| `tendonRestLength` | rest length | f32 × 1 | yes | — |

## Deformable materials

Deformable materials (`DEFORMABLE_MATERIAL`) emit a **fixed** group with one row per material prim. **Host-only always.** Volume and surface materials share
this type; three fields apply to surface materials only, and a volume material contributes no row for
them (rather than reporting zero).

| Attribute | Meaning | dtype × lanes | Applies to |
|---|---|---|---|
| `deformableDynamicFriction` | dynamic friction | f32 × 1 | volume + surface |
| `deformableYoungsModulus` | Young's modulus | f32 × 1 | volume + surface |
| `deformablePoissonsRatio` | Poisson's ratio | f32 × 1 | volume + surface |
| `deformableElasticityDamping` | elasticity damping | f32 × 1 | volume + surface |
| `deformableBendingStiffness` | bending stiffness | f32 × 1 | **surface only** |
| `deformableThickness` | shell thickness | f32 × 1 | **surface only** |
| `deformableBendingDamping` | bending damping | f32 × 1 | **surface only** |
