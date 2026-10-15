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
| `isSleeping` | current sleep state: 1 asleep, 0 awake (read-only) | u8 × 1 | — | bool | host |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — | host |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — | host |
| `restitution` | per-shape restitution | f32 × per-shape | — | — | host |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length | host |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length | host |
| `shapeCount` | real shapes per row | i32 × 1 | — | count | host |
| `kinematicTargetPosition` | pending kinematic target position (else the body's pose) | f32 × 3 | world | length | host |
| `kinematicTargetOrientation` | pending kinematic target rotation (else the body's pose) | f32 × 4 | world | — | host |

Per-shape columns are padded to the widest *queried* body's shape count (varies per read — do not
cache); read `shapeCount` for the real length. A shape with several materials reports only its first
(material 0). On DirectGPU, disabled (`eDISABLE_SIMULATION`) bodies are omitted from fixed columns
while discovery keeps the full count.

`kinematicTargetPosition` / `kinematicTargetOrientation` return the **pending kinematic target** of a
kinematic body; a body with no target set (non-kinematic, or an undriven kinematic body) falls back
to its current world pose. They are host-authoritative and readable on a DirectGPU scene — read
`position` / `orientation` for a device-fresh pose.

`isSleeping` reports PhysX state when the read executes, independently of the last `wake`
command. Use `ALL` scope to inspect sleeping bodies; `ACTIVE` is a selection of bodies that
were active, not a sleep-state query. Zero velocity alone does not prove that a body is asleep.

- On CPU and GPU scenes with CPU readback, dynamic bodies report their live state. Kinematic
  bodies report PhysX's kinematic sleep state: setting a target wakes them, and they sleep
  after two consecutive simulation steps without a new target. Simulation-disabled bodies
  are included on these modes and report asleep when scene sleeping is enabled.
- On scenes with `eDISABLE_SLEEPING`, including DirectGPU, every emitted value is `0` because
  the scene disables sleeping. This follows PhysX's reported state even for a disabled body
  on a CPU/readback scene; use `disableSimulation` to determine whether it participates in
  simulation. This does not enable `wake` writes on DirectGPU.
- DirectGPU keeps its existing selection rules: no groups before the first completed step,
  and no rows for simulation-disabled bodies. A missing row is not an awake value.
- The host column is a read-session snapshot. Complete preceding simulation work before
  reading; a later step or write requires a new read to observe the changed state. Retained
  snapshots keep their original values under the usual read-buffer lifetime rules.

`isSleeping` has no point-instancer array form and is not emitted for whole-articulation,
vehicle-wheel, deformable, particle or tendon queries.

**Point-instancer instances** read only the state attributes listed below, as an **array** group (one tensor per
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
| `isSleeping` | current sleep state: 1 asleep, 0 awake (read-only) | u8 × 1 | — | bool | host |
| `staticFriction` | per-shape static friction | f32 × per-shape | — | — | host |
| `dynamicFriction` | per-shape dynamic friction | f32 × per-shape | — | — | host |
| `restitution` | per-shape restitution | f32 × per-shape | — | — | host |
| `contactOffset` | per-shape contact offset | f32 × per-shape | — | length | host |
| `restOffset` | per-shape rest offset | f32 × per-shape | — | length | host |
| `shapeCount` | real shapes per row | i32 × 1 | — | count | host |
| `linkIncomingJointForce` | spatial force the inbound joint exerts on the link | f32 × 6 (force xyz, torque xyz) | joint | force / torque | device |
| `kinematicTargetPosition` | world pose fallback (a link cannot be a kinematic target) | f32 × 3 | world | length | host |
| `kinematicTargetOrientation` | world pose fallback (a link cannot be a kinematic target) | f32 × 4 | world | — | host |
| `physxPointer` | borrowed `PxArticulationLink` address | u64 × 1 | — | — | host |

For `isSleeping`, every link reports the state of its **owning articulation**. Links do not
sleep independently; this read does not make per-link `wake` writes valid.

## Character controllers

`CHARACTER_CONTROLLER` has no readable columns. The controller prim is a row of
`RIGID_BODY` `position` and `orientation`: the backing kinematic actor's global
pose. That orientation includes the capsule-axis rotation and is not the
fixup-corrected orientation written back to the prim. `move` updates the controller
and its kinematic target immediately; that rigid-body pose changes on the next
simulation step, when the target is integrated.

## SDF shapes

`SDF_SHAPE` has no readable columns. Evaluate the shapes' signed distance fields at
your own points with `ovphysx_evaluate_sdf_shapes`; see [SDF shapes](index.md#sdf-shapes).

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
| `centerOfMassLocal` | articulation COM | f32 × 3 | root-link COM | length | device |
| `staticFriction` / `dynamicFriction` / `restitution` / `contactOffset` / `restOffset` | per-shape (all links) | f32 × per-shape | — | — | host |
| `shapeCount` | total shapes across links | i32 × 1 | — | count | host |
| `dofCount` | degrees of freedom | i32 × 1 | — | count | host |
| `linkCount` | articulation links (not standalone rigid bodies) | i32 × 1 | — | count | host |
| `jointCount` | inter-link joints | i32 × 1 | — | count | host |
| `fixedTendonCount` | fixed tendons on this articulation | i32 × 1 | — | count | host |
| `spatialTendonCount` | spatial tendons on this articulation | i32 × 1 | — | count | host |
| `isFixedBase` | 1 = fixed base, 0 = floating | i32 × 1 | — | — | host |
| `dofJointPath` | owning joint prim for each DOF column (interned prim-path handle; `0` pads) | u64 × dofCount (padded) | — | — | host |
| `dofAxisIndex` | each DOF's ordinal among its joint's unlocked axes | i32 × dofCount (padded) | — | — | host |
| `jacobian` | dense Jacobian | f32 × rows·cols | — | angular in radians | device |
| `jacobianShape` | (rows, cols) of `jacobian` | i32 × 2 | — | count | host |
| `massMatrix` | generalized mass matrix | f32 × M·M | — | — | device |
| `coriolisForce` | Coriolis + centrifugal compensation | f32 × M | — | — | device |
| `gravityForce` | gravity compensation | f32 × M | — | — | device |
| `centroidalMomentum` | centroidal momentum matrix (floating-base only; a fixed-base articulation contributes no row) | f32 × 6·(numDofs+7) | — | — | device |
| `physxPointer` | borrowed `PxArticulationReducedCoordinate` address | u64 × 1 | — | — | host |

Root state and `centerOfMassWorld` are in scene-world coordinates; `centerOfMassLocal` is in the root
link's **COM frame** — place it in the world with the root's mass-frame pose, not its actor/prim pose
(the two differ when the root has a non-identity mass pose).

The six int32 topology scalars — `dofCount`, `linkCount`, `jointCount`, `fixedTendonCount`,
`spatialTendonCount` and `isFixedBase` — replace the deprecated `ovphysx_get_articulation_metadata`
field-for-field. They are **host-resident on every device** like `shapeCount` (read from host-side
articulation state, so no device copy is needed) and emitted in the one scene-wide group even when base
type varies per row. `isFixedBase` reflects the **live** base flag, so a runtime
`setArticulationFlag(eFIX_BASE, …)` toggle is visible on the next read. The tendon counts are **per
articulation**, not the view-wide maxima the deprecated getter reported; `linkCount` counts
articulation **links** (the old struct's
mislabelled `body_count`), not standalone rigid bodies; and `isFixedBase` is an `int32` `0`/`1` flag.
The `dofJointPath` / `dofAxisIndex` descriptor is the non-binding successor to the DOF / body / joint
**name** lists: each is a padded per-DOF array (host-resident, one row per articulation, in cache-DOF
order) that maps inverse-dynamics column `(isFixedBase ? 0 : 6) + k` to its owning joint prim — an
interned prim-path handle, resolved through the attached Stage's path dictionary like `prims.list` (`0` pads) — and to
that DOF's ordinal among the joint's unlocked axes. Link and joint names are then the leaf of the
`kOvxArticulationLink` / `kOvxArticulationJoint` prim read by `ovphysx_read`. **Limitation:** only
DOF-bearing joints appear — a fully-locked (zero-DOF) joint has no read group or descriptor entry, so its
name/path is not recoverable through the read API; the deprecated `ovphysx_articulation_get_joint_names`
still lists those fixed joints.

> **Note.** A group's tuple width is a 16-bit field, so a single inverse dynamics column wider than 65 535
> (roughly a 64-link / 189-DOF articulation's Jacobian, or a mass matrix past M ≈ 256) is omitted
> for that cohort. The read succeeds, other supported columns remain available, and the optional
> summary reports `OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION` for the affected request.

## Articulation joints — DOFs

Articulation joints (`ARTICULATION_JOINT`) emit an **array** group with one tensor per joint prim. Each
tensor has **one row per unlocked axis**, in `PxArticulationAxis` order — one value per row
(`dtype.lanes` is 1), except `jointLimit`, whose row is the `(low, high)` pair (`dtype.lanes` is 2).
Don't confuse the axis count (the tensor's shape) with lanes. A read may ask for state, control, or
properties in any combination. Rotational axes carry the joint's body-order sign; angular DOF
quantities are in **degrees** (as USD-authored).

> **Note.** A requested attribute produces **one** array group whose `tensors[i]` holds joint *i*'s
> axis values — *not* one group per joint. Code that reads `tensors[0]` and advances to the next
> group sees only the first joint, and does so silently on a single-joint scene.

State / control (device on DirectGPU):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointPosition` | generalized position | f32 × 1 | deg / length |
| `jointVelocity` | generalized velocity | f32 × 1 | deg/s / length/s |
| `jointPositionTarget` | drive position target | f32 × 1 | deg / length |
| `jointVelocityTarget` | drive velocity target | f32 × 1 | deg/s / length/s |
| `jointActuationForce` | applied actuation effort | f32 × 1 | force / torque |
| `jointProjectedForce` | inbound joint force on this axis | f32 × 1 | force / torque (read-only, derived) |

Properties (authoring inputs — **host-resident even on DirectGPU**):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointStiffness` | drive effort per unit position error | f32 × 1 | effort/coord |
| `jointDamping` | drive effort per unit rate error | f32 × 1 | effort/rate |
| `jointLimit` | (low, high) motion limit | f32 × 2 | degrees / length; `FLT_MAX` = free |
| `jointMaxVelocity` | joint rate bound | f32 × 1 | USD rate |
| `jointMaxForce` | drive effort bound | f32 × 1 | force |
| `jointArmature` | added rotor inertia | f32 × 1 | inertia |
| `jointStaticFriction` / `jointDynamicFriction` | break-away / sliding effort | f32 × 1 | effort |
| `jointViscousFriction` | effort per unit rate | f32 × 1 | effort/rate |
| `jointSpeedEffortGradient` | drive-envelope rate per effort | f32 × 1 | USD |
| `jointMaxActuatorVelocity` | actuator rate bound | f32 × 1 | USD rate |
| `jointVelocityDependentResistance` | envelope resistance per rate | f32 × 1 | effort/rate |
| `jointDriveType` | 0 none / 1 force / 2 accel | u8 × 1 | enum |
| `physxPointer` | borrowed `PxArticulationJointReducedCoordinate` address | u64 × 1 | — |

`jointLimit` is reported as the authored `(lower, upper)` pair, in order, regardless of the joint's
body order — unlike `jointPosition`, which negates when `body0` is the child.

`jointProjectedForce` and `linkIncomingJointForce` (on articulation links) are **computed per read**
rather than gathered from stored state, so each costs several times a four-column pose/velocity read
of the same prims — request them in the read that needs them, not in a per-step pose read.
`jointProjectedForce` uses the same one-row-per-unlocked-axis shape as the other joint columns: a
locked axis is **omitted** (not a zero-filled row), and a fixed joint — which has no DOF — contributes
no row.

## Whole vehicles

Whole vehicles (`VEHICLE`) emit fixed, host-resident groups with one row per vehicle prim.
The chassis pose and velocity use a separate `RIGID_BODY` query on that same prim; wheel
attachments use `VEHICLE_WHEEL`. Vehicles are unavailable on DirectGPU scenes.

| Attribute | Meaning | dtype x lanes | Units | Applicability |
| --- | --- | --- | --- | --- |
| `accelerator` | persistent accelerator input | f32 x 1 | normalized [0,1] | DriveBasic / DriveStandard |
| `brake0`, `brake1` | persistent brake-channel input | f32 x 1 | normalized [0,1] | DriveBasic / DriveStandard |
| `steer` | persistent steering input | f32 x 1 | normalized [-1,1] | DriveBasic / DriveStandard |
| `clutch` | native clutch command state | f32 x 1 | normalized [0,1] | DriveStandard |
| `currentGear` | engaged gear | i32 x 1 | gear index | DriveStandard |
| `targetGear` | internal transmission target, distinct from the requested control | i32 x 1 | gear index | DriveStandard |
| `engineRotationSpeed` | engine angular speed | f32 x 1 | rad/s | DriveStandard |
| `automaticTransmission` | automatic transmission enabled | u8 x 1 | boolean | DriveStandard |
| `gearSwitchTime` | elapsed time in current gear change; negative when no change is active | f32 x 1 | seconds | DriveStandard |
| `autoboxTimeSinceLastShift` | automatic-gearbox time since the last shift; zero without an autobox | f32 x 1 | seconds | DriveStandard |

Gear indices use -1 for reverse, 0 for neutral and positive values for forward gears.
Before a recorded automatic shift, `autoboxTimeSinceLastShift` can hold the native float32-maximum
sentinel instead of an elapsed duration.
The readable controls use the same names as writes. An unconfigured brake channel reads zero;
writing it has no effect. `requestedGear`, tank `thrust0` / `thrust1`, and `resetToRestState` are
write-only. `targetGear` reports the transmission's internal state, not an echo of `requestedGear`.

Discovery lists readable attributes supported by at least one selected vehicle. Mixed reads emit
only applicable rows, with `UNSUPPORTED_CONFIGURATION` in the optional session summary for omitted
vehicles. Always use each group's prim list: a Standard-only column can have fewer rows than a
control column. A disabled vehicle remains in `ALL` membership.

## Vehicle wheels

Vehicle wheels (`VEHICLE_WHEEL`) emit fixed, host-resident groups with one row per wheel attachment.
The wheel pose is composed from chassis, suspension and steering. Other columns report native
vehicle state from its latest update. Vehicles are unavailable on DirectGPU scenes.

| Attribute | Meaning | dtype x lanes | Frame / units |
| --- | --- | --- | --- |
| `position` | composed wheel position | f32 x 3 | world; length |
| `orientation` | composed wheel rotation, xyzw | f32 x 4 | world |
| `rotationSpeed` | wheel angular speed | f32 x 1 | rad/s |
| `rotationAngle` | wheel rotation angle | f32 x 1 | radians |
| `wheelSteerAngle` | actual wheel steering angle | f32 x 1 | radians |
| `suspensionJounce` | suspension compression | f32 x 1 | length |
| `suspensionForce` | suspension force | f32 x 3 | world; force |
| `tireFriction` | effective tire friction | f32 x 1 | dimensionless |
| `tireLongitudinalDirection` | tire's longitudinal direction | f32 x 3 | world; unit vector |
| `tireLateralDirection` | tire's lateral direction | f32 x 3 | world; unit vector |
| `tireLongitudinalSlip` | longitudinal slip | f32 x 1 | dimensionless ratio |
| `tireLateralSlip` | lateral slip | f32 x 1 | radians |
| `tireForce` | tire force | f32 x 3 | world; force |
| `isOnGround` | validity of the current ground-contact sample | u8 x 1 | boolean |
| `groundPlane` | contact plane `(nx, ny, nz, d)`, `n dot p + d = 0` | f32 x 4 | world; d in length |
| `groundHitPosition` | contact hit position | f32 x 3 | world; length |
| `groundActorPath` | ground actor prim identity | u64 x 1 | attached Stage path dictionary |
| `groundShapePath` | ground shape prim identity | u64 x 1 | attached Stage path dictionary |
| `groundMaterialPath` | ground material prim identity | u64 x 1 | attached Stage path dictionary |

All these columns are read-only. The existing raw-wheel `driveTorque`, `brakeTorque` and
`steerAngle` controls are write-only; `wheelSteerAngle` is actual telemetry, separate from that
steering command.

`isOnGround == 0` means no valid current contact sample. It does not prove that a sleeping vehicle
is physically airborne. Adding or removing any runtime object record in the owning attachment
invalidates contact data until the next completed vehicle update, even when the changed object
is unrelated to that contact. Before the first update and for sleeping or disabled vehicles,
contact data can also be invalid. In that case `groundPlane`,
`groundHitPosition` and all three ground identities are zero. Other telemetry retains its native
last-update state; it is not forcibly cleared when contact becomes invalid.

A ground identity is an `ovx_primpath_t` represented as uint64. Resolve it with the attached Stage's
path dictionary, exactly as a group's prim identities. Zero means absent or unrepresentable,
including an implicit default material. These values are not pointers or private object IDs.
Length, force and torque follow the [stage's units](data_model.md#units).

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
| `kinematicTarget` | pending sim-node kinematic target (`.w` active-flag) | f32 × 4 | sim-mesh-local | device | **volume only** |

A surface body's simulation mesh *is* its collision mesh, so it has no separate
`collisionElementIndices` — discovery omits it for surfaces, and a surface query for it returns no
group.

`kinematicTarget` is served **only for bodies actually being driven** (through
`setKinematicTargetBufferD`); an undriven body contributes no tensor, so its group covers a subset of
the `points` prims. The `.w` lane is an active-flag with **inverted** polarity: `w == 0` drives the
node to its `xyz` target, `w != 0` leaves it free.

> **Note.** `points` and `restPoints` share the sim-mesh-local frame (their difference is
> meaningful), but `velocities` is in **world** space. Do not difference `points` against a
> time-integrated `velocities`.

## Particle sets

Particle sets (`PARTICLE_SET`) emit an **array** group with one tensor per particle-set prim. **GPU-only.**

| Attribute | Meaning | dtype × lanes | Frame | Residency |
| --- | --- | --- | --- | --- |
| `points` (alias `positions`) | particle positions | f32 × 3 | particle-set-prim-local | device |
| `velocities` | particle velocities | f32 × 3 | world | device |
| `physxPointer` | borrowed particle-set address | u64 × 1 | — | host |

`positions` is accepted as an alias and canonicalized to `points` in groups and session summaries;
discovery advertises only `points`. Requesting both spellings is a duplicate and fails the open.

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

## Debug render

Debug render (`DEBUG_RENDER`) is one instance-level target, not a prim. A query does not require an
attached stage. Settings are one row. Geometry columns are a snapshot of PhysX debug primitives for
scenes whose attach handle matches this instance. Position index `i` and color index `i` are the
same primitive. Shape 0 is an empty list. **Host-only always.**

**Geometry timing.** Complete simulation before reading. On CPU scenes, PhysX actor/shape
wireframes use poses from the start of each scene's latest simulation step, before integration.
With substepping, this is the start of that scene's last substep. Contact and friction
primitives describe contacts computed for that step. Reading copies the existing
primitives; a moving body's wireframe can therefore trail its rendered post-step pose.
The simulated body state is not delayed.

| Attribute | Meaning | dtype × lanes / Python shape | Residency |
|---|---|---|---|
| `enabled` | visualization on/off | u8 × 1 → `[1]` | host |
| `scale` | visualization scale | f32 × 1 → `[1]` | host |
| `worldAxes` … `sdf` | one float per visualization parameter (see `OVPHYSX_ATTR_DEBUG_*`) | f32 × 1 → `[1]` | host |
| `cullingBox` | cull box min xyz, max xyz; leading shape 0 means no box | f32 × 6 → `[1, 6]` or `[0, 6]` | host |
| `scope` | interned path tokens; length 0 draws everything | u64 × 1 → `[N]` | host |
| `pointPosition` | point positions | f32, shape `[N, 3]` | host |
| `pointColor` | point colors (`0xAARRGGBB`) | u32, shape `[N]` | host |
| `linePosition` | line endpoint positions | f32, shape `[N, 2, 3]` | host |
| `lineColor` | line endpoint colors | u32, shape `[N, 2]` | host |
| `trianglePosition` | triangle vertex positions | f32, shape `[N, 3, 3]` | host |
| `triangleColor` | triangle vertex colors | u32, shape `[N, 3]` | host |

In C, pass `&OVPHYSX_NAME_DEBUG_*`. In Python, pass the string names above. With a stage
attached, each returned group's attribute token resolves to that name. With no stage the
token is 0, because there is no path dictionary to intern it. A token-only read then fails.

## Contact sensors

Contact sensors (`CONTACT_SENSOR`) include rigid bodies and articulation links carrying
`PhysxContactReportAPI`, and every link of an articulation carrying it. On CPU simulation and GPU
dynamics with CPU readback, each collider carrying the schema is also a sensor. A body row aggregates
all of its shapes' contacts and each collider row reads only its own shape's, so the two **overlap**:
do not sum net forces or count contacts/events across a body and its own colliders, or they double-count.
Collider rows are host-only — a DirectGPU scene enumerates only reporting bodies (a selection naming
only a separate child collider discovers no rows there). Every attribute is **read-only** and describes
the last step; each scene's sensors come back as
their own groups, reduced with that scene's step dt. Three query forms select them — the whole set
(`ovphysx_query`), a prim-set selection (`ovphysx_query_selection`; `pattern=` / `prim_paths=` in
Python), and a [contact query](index.md#contact-queries) that pairs each sensor with F filter bodies.
`ACTIVE` scope behaves as `ALL`.

### Child collider sensors

On CPU simulation and GPU dynamics with CPU readback, a child collider can be selected as a contact
sensor when `PhysxContactReportAPI` is applied to that collider. This makes the sensor available; it
does not by itself enable contact reporting for its owning rigid body.

To enable reporting from that body, also apply `PhysxContactReportAPI` to the owning rigid-body prim.
For an articulation, reporting can be enabled on the owning link or on the articulation root, which
enables reporting for every link. A child sensor can also receive contacts reported by the other actor
in the pair. If neither actor reports matching contacts, the child remains a valid sensor but reads
zero force, even while touching another object.

For example, add this authoring fragment at the stage root and select `/Body/Collider` to read that
shape's contacts:

```usda
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxContactReportAPI"]
)
{
    float physxContactReport:threshold = 0

    def Cube "Collider" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxContactReportAPI"]
    )
    {
        double size = 0.5
    }
}
```

Configure reporting settings, including `physxContactReport:threshold` and
`physxContactReport:reportPairs`, on the reporting body, link, or articulation root. Settings on a
separate child collider do not configure its owner's reporting. The example enables reporting from
`/Body` without depending on the other actor; sleeping and other reporting conditions still apply.

**Deprecated contact bindings.** On CPU simulation and GPU dynamics with CPU readback, selecting
the owning body as a sensor in the same binding or another live contact binding on the same attachment
credits the contacts to the body and leaves the child sensor at zero. Wildcard patterns can select
both. For a child binding, enable reporting on `/Body` but select only `/Body/Collider` as a sensor,
and establish the bindings before stepping. Session contact reads support body and child rows together;
see the [Contact Binding tutorial](../tutorials/contact_binding.md) for the binding limitations.

A standalone static collider is its own rigid actor, so applying the schema to that prim enables
reporting directly on CPU simulation and GPU dynamics with CPU readback. On DirectGPU, a standalone
static collider remains a sensor row but reads zero force. DirectGPU session queries omit separate
child-collider rows; select a dynamic body or articulation link to read forces instead.

### Reading contact data

**Sleeping contacts.** On CPU and GPU-dynamics scenes with CPU readback, persistent
contacts stop being reported when neither body in the pair is active. A settled
sensor can therefore have zero forces and no contact arrays while still touching
the ground. This also applies to contact bindings;
environment IDs do not disable contact reporting. DirectGPU disables sleeping,
so its contact tensors can remain nonzero on the same settled scene.
For continuous reporting, disable sleeping before attachment (for example,
`physxScene:disableSleeping = true` on a scene with `PhysxSceneAPI`), or wake the
bodies and complete another simulation step before reading. Use
[`isSleeping`](#rigid-bodies) to inspect sleep state.

On a DirectGPU scene a sensor or a filter that is a point-instancer prototype stands for its **first
instance only**, as in the contact binding: every column reads that instance's contacts, where a CPU scene
reads every instance's. Another sensor's contacts with each instance are unaffected.

**Net forces** — a **fixed** group, one row per sensor, on every query form:

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency |
| --- | --- | --- | --- | --- | --- |
| `netNormalContactForce` | the sensor's contact-point impulses, summed, / dt | f32 × 3 | world | force | device |
| `netFrictionContactForce` | the sensor's friction-anchor impulses, summed, / dt | f32 × 3 | world | force | device |

Their sum is the total contact force. Friction anchor forces that oppose each other cancel, so a
friction torque does not show.

**Per filter** — on a contact query with filters only (F filters per sensor); a query without filters
does not serve these names:

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency |
| --- | --- | --- | --- | --- | --- |
| `normalContactForceMatrix` | normal force from each filter's body; filter f at lanes 3f..3f+2 | f32 × 3F | world | force | device |
| `frictionContactForceMatrix` | friction force from each filter's body, same layout | f32 × 3F | world | force | device |
| `filteredContactCount` | contact points from each filter's body | i32 × F | — | count | device |
| `filteredFrictionCount` | friction anchors from each filter's body | i32 × F | — | count | device |
| `filteredContactForce` | per point: its normal force (see below) | f32 × 1 (array) | — | force | device |
| `filteredContactPoint` | per point: its position | f32 × 3 (array) | world | length | device |
| `filteredContactNormal` | per point: its normal, as reported | f32 × 3 (array) | world | — | device |
| `filteredContactSeparation` | per point: its separation | f32 × 1 (array) | — | length | device |
| `filteredFrictionForce` | per anchor: its impulse / dt | f32 × 3 (array) | world | force | device |
| `filteredFrictionPoint` | per anchor: its position | f32 × 3 (array) | world | length | device |

The four matrix and count columns are fixed groups over every sensor. The six `filtered*` element
attributes are **array** groups: one array per sensor, its elements ordered by filter column —
`filteredContactCount` / `filteredFrictionCount` split it into per-filter runs, in lane order. A filter
that is a collider rather than a body is matched on a CPU scene, but reads zero on a DirectGPU scene, as
in the contact binding.

**Raw contacts** — **array** groups, one array per sensor, on every query form; every contact point of
every pair the sensor is in, whatever the other body:

| Attribute | Meaning | dtype × lanes | Frame | Units | Residency |
| --- | --- | --- | --- | --- | --- |
| `rawContactForce` | per point: its normal force (see below) | f32 × 1 | — | force | device |
| `rawContactPoint` | per point: its position | f32 × 3 | world | length | device |
| `rawContactNormal` | per point: its normal, as reported | f32 × 3 | world | — | device |
| `rawContactSeparation` | per point: its separation | f32 × 1 | — | length | device |
| `rawContactOtherBody` | per point: the other body's prim | u64 prim handle | — | — | device |

**Events** — **array** groups, one array per sensor, on every query form over a **CPU scene only**:
each contact-report pair the sensor is a side of — found, persisting or lost — as seen from the sensor,
in report order. A pair between two sensors appears under each.

| Attribute | Per | Meaning | dtype × lanes | Units |
| --- | --- | --- | --- | --- |
| `contactEventType` | event | 0 found, 1 lost, 2 persist | i32 × 1 | — |
| `contactEventSensorSide` | event | the pair side (0 or 1) the sensor is | i32 × 1 | — |
| `contactEventCollider` | event | the sensor's own collider prim | u64 prim handle | — |
| `contactEventOtherBody` | event | the other body's prim | u64 prim handle | — |
| `contactEventOtherCollider` | event | the other collider's prim | u64 prim handle | — |
| `contactEventProtoIndex` / `contactEventOtherProtoIndex` | event | the collider's instance index in its point instancer — its slot in `protoIndices`, not the prototype it selects — own / other (0xFFFFFFFF: not an instance) | u32 × 1 | — |
| `contactEventPointCount` / `contactEventAnchorCount` | event | the event's contact points / friction anchors | i32 × 1 | count |
| `contactEventPointPosition` | point | position (world) | f32 × 3 | length |
| `contactEventPointNormal` | point | normal, as reported (world) | f32 × 3 | — |
| `contactEventPointImpulse` | point | impulse, as reported (world) | f32 × 3 | force·s |
| `contactEventPointSeparation` | point | separation | f32 × 1 | length |
| `contactEventPointFaceIndex` / `contactEventPointOtherFaceIndex` | point | triangle-mesh face index, own / other | u32 × 1 | — |
| `contactEventPointMaterial` / `contactEventPointOtherMaterial` | point | material prim, own / other | u64 prim handle | — |
| `contactEventAnchorPosition` | anchor | position (world) | f32 × 3 | length |
| `contactEventAnchorImpulse` | anchor | impulse, as reported (world) | f32 × 3 | force·s |

A sensor's points (anchors) are concatenated in event order; `contactEventPointCount`
(`contactEventAnchorCount`), one value per event, splits them per event. A sensor has an array when it has
at least one element at the attribute's level this step, so a sensor whose only event is a lost pair, which
carries no point, has event arrays but no point or anchor array. Event values are as reported: impulses,
not forces, never negated.

How to read these values:

- **Force sign.** A force is the one the sensor receives: a point's normal force is its impulse
  divided by dt — the impulse's magnitude on a CPU scene, the solver's normal impulse on a DirectGPU
  scene — negated when the sensor is the pair's second body, and normals are never flipped. So
  `force × normal` is the force on the sensor.
- **Identities** are prim-path handles interned in the attached Stage's dictionary; obtain it through
  `stage->vtable->get_path_dictionary(stage->context)` in C and resolve through the ovx path-dictionary API,
  or use `ovstage.PathDictionary.path_to_string` in Python. An
  identity with no prim reads 0.
- **Exact sizing.** Each array holds the step's elements — no capacity, no truncation. A sensor with no
  element this step is absent from an array group, and the group is absent when no sensor has one.
- **Order.** On a CPU scene a sensor's elements keep report order (within a filter column for the
  filtered detail); on a DirectGPU scene their order is unspecified, as it is in the contact binding.
- **DirectGPU.** The filtered detail's arrays and the raw contacts synchronize the host once per read, to
  size them from the per-sensor counts; a read that asks for none of them does not, nor does one of the
  count columns alone. Events are not served
  (PhysX's GPU contact pairs carry no event type): the read completes without them. Before the first
  step the fixed columns read zero and the arrays are absent.

## Native PhysX pointers

These query types publish only `physxPointer`: a host `uint64` snapshot of a
borrowed PhysX address. `PHYSICS` is one row whose path is `/` and ignores
prim-set selection. The others have no active notion, so `ACTIVE` matches `ALL`.

| Query type | Address |
| --- | --- |
| `SCENE` | `PxScene` |
| `PHYSX_MATERIAL` | `PxMaterial` |
| `SHAPE` | `PxShape` |
| `COMPOUND_SHAPE` | compound-shape wrapper |
| `RIGID_ACTOR` | `PxRigidDynamic` or `PxRigidStatic` |
| `JOINT` | standalone `PxJoint` |
| `CUSTOM_JOINT` | plugin custom joint |
| `PARTICLE_SYSTEM` | particle system |
| `PHYSICS` | process-global `PxPhysics` |

The same column is also on articulation, articulation link, articulation joint,
and particle set, listed in those tables. A missing object is absent. Reading
the name on a type that does not publish it skips the name. See
[PhysX Pointer Interop](../developer_guide.md#physx-pointer-interop).
