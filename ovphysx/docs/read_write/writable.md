<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Writable data

See the [detailed I/O attribute reference](attributes/index.md) for exact request names,
layouts, conditional units and per-attribute restrictions.

The inventory of every quantity writable through the write API, per object type. Read-only quantities
are covered in [Readable data](readable.md); this page lists only what a write accepts.

Every table gives, per attribute, its **Meaning**, **dtype × lanes** (element type and tuple width),
and — where it applies — **Frame** and **Units**. Quaternions are xyzw. A `(write-only)` tag marks a
control that is **not readable** (no read-back); its persistence varies (see *Write timing, ordering,
and persistence*). Column residency follows the read:
device on DirectGPU, host on CPU — **except** joint DOF properties and particle `points`/`velocities`,
which are host-staged on **every** scene.

## Writability classes

To ask programmatically whether an `(object type, attribute)` pair is accepted:

- In C: `ovphysx_writability(object_type, attribute, &out)`
- In Python: `from ovphysx._bindings import writability`, then
  `writability(object_type, attribute)`. This helper currently lives in the private bindings module;
  it is not exported as `ovphysx.writability` or `PhysX.writability`.

It reports one of five classes — `OVPHYSX_WRITABILITY_<NAME>` in C (`ovphysx_writability_t`), the
matching integer in Python:

| Value | Class | Meaning |
|---|---|---|
| 0 | `UNCLASSIFIED` | the type does not accept this name; a write rejects it |
| 1 | `WRITABLE` | writable, no precondition beyond the object existing |
| 2 | `CONDITIONAL` | writable only under a stated condition; if unmet the write reports it and writes **nothing** (does not partially apply) |
| 3 | `WRITE_ONLY` | a control input; writable but **not readable** (no read-back). Whether it clears each step or persists is a separate per-attribute property (see persistence, below) |
| 4 | `READ_ONLY` | readable only; writing is an error naming the type, not a silent no-op |

The query is scene-independent (a hand-maintained snapshot of the runtime tables; string keys only;
the object type is bounds-checked). It classifies the *attribute*, not the object's current state.
Opening a write session remains the authoritative check for a live scene; the optional write
summary reports planned groups, not writability.

## Write timing, ordering, and persistence

- **Never auto-warmed.** The session write never steps for you. On CPU and on GPU with
  readback, a write before the scene's first step **commits and is applied**. On DirectGPU
  (`suppressReadback` / `eENABLE_DIRECT_GPU_API`), that same pre-step commit is **refused**
  until a first step has sized the GPU view. Call `ovphysx_warmup` / `step_sync` first for
  a recipe that works on every mode. See [data model — DirectGPU](data_model.md#step-first-precondition-directgpu).
- **Applied at commit, into live PhysX** — not deferred to the next step. A written value is
  observable through a read (or the borrowed `physxPointer` address)
  before the next step. Character-controller `move` updates its controller at commit, but the
  backing actor's rigid-body pose is updated on the next step. The `write_done_sync` you pass
  (`cuda_stream=` / `cuda_wait_event=` in
  Python) orders the commit after **your fill's** CUDA stream/event. A DirectGPU device group is
  ordered on the device, so its commit does not block the host; every other group waits for your
  fill on the host. See [CUDA synchronization](device.md#cuda-synchronization). The write runtime separately synchronizes
  commit with an outstanding PhysX step. Keep session and step calls serialized and
  complete a mid-session step before further scene-dependent accesses; this internal
  synchronization does not collect the step's outcome.
- **Read-modify-write for split poses.** Writing `position` pre-reads and preserves `orientation`
  (and vice versa), because they are two slices of one atomic transform. Linear and angular velocity
  are independent (no pre-read). Particle/deformable point writes preserve the `.w` inverse-mass
  lane.
- **What the sim overwrites.** *State* (poses, velocities, root state, joint DOF state, point sets)
  is recomputed by the next solver step. *Inputs, targets, and properties* (drive targets, all joint
  and tendon properties, materials, mass/inertia/COM, flags) persist across steps until re-written.
  This does not protect a live value from a pending OVStage parse recomputation; see
  [Mixing OVStage mass edits and session writes](#mixing-ovstage-mass-edits-and-session-writes).
  *Rigid external forces* (`force`, `wrench`) are the only write-only controls the solver consumes
  and clears each step. *Vehicle wheel commands* (`driveTorque`, `brakeTorque`, `steerAngle`) are
  also write-only but **persist until re-written** — the solver reads them every step without
  clearing, so a one-step command keeps applying until the caller resets it (write `0`).
  Whole-vehicle controls also persist. `resetToRestState` instead runs once at commit and is never
  retained as a per-step command.
  Character controller `move` is applied once at commit and is not replayed; see its section below.
- **Wake on write (sleeping articulations).** On CPU and GPU-with-readback scenes, a write wakes a
  sleeping articulation when it sets state or commands the mechanism, following PhysX's setters:
  `rootPosition`, `rootOrientation`, `rootLinearVelocity`, `rootAngularVelocity`, `jointPosition`,
  `jointVelocity`, `jointActuationForce`, `jointPositionTarget`, `jointVelocityTarget` and
  `tendonOffset` (the offset actuates the tendon by shifting its length, which changes its steady
  state). **Does not wake:** the drive gains and tuning properties — `jointStiffness`,
  `jointDamping`, `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonRestLength`,
  `tendonLimit`, and the other joint/tendon properties above. These commit and read back truthfully on
  the tendons and joints that accept them — `tendonLimit` and `tendonRestLength` exist only on fixed
  tendons and are refused by name on a spatial tendon (see the Fixed / spatial tendons section) — but
  the articulation stays asleep, so a property change that *would* alter its settled
  state (for example `tendonRestLength`, or `jointStiffness` when the joint/tendon settled under load)
  does not take visible effect until the articulation is awake. There is no whole-articulation wake command —
  the rigid-body `wake` attribute is refused on an articulation link (a link cannot wake
  independently) — so wake it with one of the waking writes above; re-sending the current
  `jointPositionTarget` or `rootPosition` is enough. This is by design (inherited from PhysX), not a
  dropped write. A DirectGPU scene has no sleeping, so none of this applies there.
- **Persistence is runtime-only.** Every write publishes into live PhysX simulation state. **No
  write is authored back into the ovstage / USD source representation.** (A read gathers from the
  same live state, which is why a written value round-trips through a read.) Propagating results back
  into ovstage is a separate output-drain direction, not this write path — and its coverage differs
  from the writability query (see the last item in [Known limitations](limitations.md)).
- **Several attributes per session.** A write session opens over an attribute array and validates
  all of it before exposing a group: an unsupported, read-only or duplicate name fails the open and
  publishes nothing. Each fetched group names its attribute through `attribute_index`, the
  position in that array (see [Session results](index.md#session-results)). Commits run in the
  order the caller issues them, whatever the fetch order; independent attributes are not promised
  to commute. `DEBUG_RENDER` does not take several attributes: each of its write sessions names
  exactly one (see [Debug render](#debug-render)).
- **Row masks.** A commit publishes the whole mapped group — every object the query selects. To
  write only some of its rows, commit with a row mask: `ovphysx_commit_group_masked` in C,
  `session.commit(group, mask=...)` in Python. The mask is a contiguous 1-D bool or uint8 tensor with
  one entry per group row, on the group's CUDA device, or in host memory for a host group; a nonzero
  entry selects the row. A row is one tensor of a deformable group, and otherwise one entry of the
  tensor's leading dimension: one DOF of a joint or joint-property group, so a mask selects single
  axes; for a point instancer, one slot of its instance array, holes included. An unselected row keeps its value, except that an unselected
  `force` or `wrench` row applies no force from this commit. On a DirectGPU scene a `force` or
  `wrench` commit sets the forces of every row of its group, so it also clears what an earlier
  commit in the same step applied to its unselected rows: of two commits with disjoint masks, only
  the second one's forces apply. Elsewhere forces add up within a step. On a DirectGPU scene an
  articulation root, joint or tendon commit writes every articulation of its group back, unselected
  ones with their current values: until the next step those read zero link accelerations and incoming
  joint forces, and a root whose centre of mass is offset can drift slightly with each such commit. A
  selected query's write does the same to the articulations outside its selection. A malformed mask
  is refused at commit and leaves the group live, as is a mask on a character controller, vehicle or
  particle group. A DirectGPU group reads the mask on the device: keep it valid until the work
  queued on the commit's stream has run, and for as long as a graph that captured the commit may be
  replayed. To write a fixed subset of prims, open the query with a
  [prim-set selection](index.md#selecting-a-subset-of-prims) (explicit prim handles or a path
  pattern) instead; the write then maps only the selected prims. Neither `ALL` nor `ACTIVE` scope is
  a kinematic-body filter. Rows are resolved for one write session and never reused by another.
- **Committing a live group consumes it.** After an accepted commit of a live group
  owned by this session -- `ovphysx_commit_group` in C, `session.commit(group)` in
  Python -- the mapped buffers belong to physics (dereferencing the C pointers, or touching the
  Python tensor views, is undefined) and commit is not idempotent. A group left uncommitted when the
  session is released (or the Python `with` block exits) is discarded, not published.
  Detach and instance destruction also invalidate write sessions; finish
  external consumers of their buffers before these transitions.
- **Read the failure reason.** Python raises `RuntimeError` for a rejected submission and
  `TaskError` (a `RuntimeError`) with the write's diagnostic when an operation is waited.
  Before it submits, `session.commit` validates ownership in Python: a group owned by another
  session, or any object that is not one of this session's fetched groups (`session.groups`),
  raises `ValueError`,
  and a group this session already committed raises `RuntimeError`. Both are raised
  synchronously at the call, with no Operation or Task to wait, as they are caught before
  native admission.
  In C, admission rejection uses `ovphysx_get_last_error()` on the calling thread;
  accepted execution failures are reported by the wait and
  `ovphysx_get_last_task_error()`. Read the cause before destroying its wait result.
  A rejected submission retains the group. Once execution accepts this session's
  live group for publication, it is spent even if publication fails. In C, where there is no
  such object-identity precheck, an unknown, foreign or already-spent non-null native group
  pointer is admitted and reports a task failure without consuming another valid group.
  Previously committed groups remain applied, and the failing group may also be
  partially applied. There is no rollback or applied/rejected row count; a failure
  guarantees zero writes only when its diagnostic says so.

## Mixing OVStage mass edits and session writes

Session commits and OVStage edits have different application times. A completed
`update_from_ovstage()` for an authored rigid mass-frame edit queues a combined
mass, inertia, and center-of-mass recomputation from the stage for the next step.
It does not make that recomputation observable immediately. A session commit
before that step updates live PhysX state, but does not cancel the queued work
or change its authored inputs. The next step can therefore replace the session
value even when the session commit was the later call. There is no global
"latest call wins" rule across these two routes.

For example, starting with mass 3, draining an OVStage `physics:mass = 6` edit
leaves the live mass at 3. Committing session `mass = 9` then makes a read return
9. The next step applies the queued recomputation and a read returns 6.

To retain a session override after that recomputation, seal and drain the
OVStage edit, complete a simulation step, and then commit the session override.
Further steps preserve the mass override until another write or a stage-driven
recomputation replaces it. The intervening step advances the simulation using
the stage-derived mass; it is not a flush-only operation. If that step must use
the desired mass, author the desired final mass in OVStage before draining
instead of relying on a session override before the step. A session mass write
does not itself rescale inertia; set the required inertia/COM as well when
overriding the mass frame.

See [OVStage edit timing](../ovstage_integration.md#deferred-mass-edits-and-session-overrides)
for the call sequence. Other drain coverage and device restrictions remain as
listed in [Known limitations](limitations.md).

## Rigid bodies

Rigid bodies (`RIGID_BODY`): all rigid **state** and **authored inputs** are writable; derived
quantities (`linearAcceleration`, `angularAcceleration`, `inverseMass`, `inverseInertia`,
`shapeCount`), the reported `isSleeping` state and the creation-time `worldScale` are read-only.

On CPU and GPU-with-readback scenes, PhysX rejects `linearVelocity` and
`angularVelocity` setters on standalone kinematic bodies. Commit reports the SDK
error, but valid dynamic rows in the same group can still be written, including rows
after the rejected body. The kinematic body's velocity stays unchanged. This SDK
restriction does not apply to DirectGPU velocity writes.

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
| `wake` | wake / sleep control — nonzero wakes, zero sleeps (write-only, CONDITIONAL: non-DirectGPU scenes) | u8 × 1 | — | bool |
| `kinematicTargetPosition` | kinematic target position — drives a kinematic body (CONDITIONAL: non-DirectGPU scenes) | f32 × 3 | world | length |
| `kinematicTargetOrientation` | kinematic target rotation — drives a kinematic body (CONDITIONAL: non-DirectGPU scenes) | f32 × 4 | world | — |

Point-instancer instances are written in the same session as array groups (`position` / `orientation`
local, velocities world). On DirectGPU an instancer group holding a disabled instance is refused as a
whole (writes nothing) — asymmetric with standalone bodies, whose disabled rows are silently filtered.

Writing `disableSimulation` retains disabled rows in the group, so writing `0` re-enables a body.
This disables a **standalone** body only — a point-instancer **instance** cannot be disabled
individually (`disableSimulation` is not an instancer-writable column and there is no per-instance
route), which is unsupported in this release. **Do not toggle `eDISABLE_SIMULATION` directly on a
`PxRigidDynamic*` read from `physxPointer`** — ovphysx does not observe the change, and
on a DirectGPU scene the next read or write may resolve a stale GPU index to the wrong body without
error. Write `disableSimulation` through the write API instead.

`kinematicTargetPosition` / `kinematicTargetOrientation` are **CONDITIONAL**: supported on **CPU
scenes and GPU scenes with CPU readback**. They drive a *kinematic* rigid body to a world pose;
the write is **refused outright on a DirectGPU scene** because PhysX never uploads the
CPU-integrated target to the device. Direct `position` / `orientation` writes are device-backed
on DirectGPU, but teleport the body without a kinematic target's step velocity; they are not an
equivalent way to carry a resting load. Preconditions are enforced
atomically: a disabled kinematic body, or a non-finite position or a non-finite/zero quaternion,
fails the whole commit naming the body. A non-kinematic body cannot be driven, but the session still
opens and its other rows land — that body is warned and skipped, not name-refused. Position and
orientation are one transform, so writing one preserves the other.

`wake` is a **write-only command**. Read [`isSleeping`](readable.md#rigid-bodies) separately to
observe the current state; it does not echo the command. Nonzero wakes the addressed rigid
dynamics, zero sleeps them, per prim in one commit. It is a **one-time command over the mapped
group**: every row is driven, so a zero row *actively sleeps* that body and there is no
leave-unchanged value. To leave a body untouched, leave it out of the commit's
[row mask](#write-timing-ordering-and-persistence), or out of the query with a
[prim-set selection](index.md#selecting-a-subset-of-prims), which can include sleeping bodies. It is **CONDITIONAL**, refused **only on a DirectGPU scene**
(`eENABLE_DIRECT_GPU_API`), because DirectGPU requires `eDISABLE_SLEEPING`. The command is
supported on CPU scenes and GPU scenes with CPU readback. Non-dynamic, kinematic, and disabled
bodies are skipped rather than driven,
and on an `eDISABLE_SLEEPING` scene the sleep half (`wake == 0`) is a no-op.

The per-shape material attributes (`staticFriction` / `dynamicFriction` / `restitution`) are carried by a
`PxMaterial` that several shapes may share (the authored default). Writing one gives the written shape its
**own** material, so the write is isolated — a sibling shape still bound to the original material is
unchanged. Other material settings, including compliant-contact acceleration mode, remain unchanged.
The trade-off: once a shape owns its material, a later edit to the originally-authored material
(an ovstage / USD material change) no longer reaches that shape; only shapes still bound to the authored
material follow.
The fork has no authored material identity: contact-point material handles report the scene's
default material prim when one is authored, or `0` otherwise.

## Articulation links

Articulation links (`ARTICULATION_LINK`) share the rigid body's writable **properties**, with these
exceptions: pose and velocity are read-only (a link moves through its articulation's root pose plus
joint state); `worldScale` is read-only as on a rigid body; `disableSimulation` is read-only (a
link cannot be excluded from its articulation's simulation individually); the `kinematicTarget*`
names are refused (a link cannot be a kinematic target); and `wake` is refused (a link cannot wake
independently — the whole articulation wakes as a unit). The table below is exactly the writable set.

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

On a **DirectGPU** scene, `force` and `wrench` apply per articulation as a whole per-link block, and
are write-only with no device readback to merge against: a `force` write replaces the matched
articulation's force block, a `wrench` write both its force and torque blocks. A later write naming any
link of the **same** articulation in the same step replaces that block for the whole articulation, so
two links of one articulation written in separate selected sessions before the step keep only the last
write — even though the links were selected separately. Set the links of one articulation in a single
write to apply them together. Writes to links of *different* articulations are preserved.

## Articulation joints — DOFs

Articulation joints (`ARTICULATION_JOINT`) write a fixed group with **one row per unlocked axis**, as
the read emits it: one tensor, a joint's rows consecutive in `PxArticulationAxis` order, and each row's
prim-list entry naming its joint — one value per row (`dtype.lanes` is 1), two for `jointLimit`. Fill
it as one array. The unit fold is applied once by the setter. Rotational axes are in **degrees** (as
USD-authored). `ovphysx_create_joint_arrays_from_map_group` (`<ovphysx/ovphysx_utils.h>`) and
`ovphysx.utils.split_joint_group` give one writable view per joint over the same column.

State and control:

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointPosition` | generalized position | f32 × 1 | deg / length |
| `jointVelocity` | generalized velocity | f32 × 1 | deg/s / length/s |
| `jointPositionTarget` | drive position target | f32 × 1 | deg / length |
| `jointVelocityTarget` | drive velocity target | f32 × 1 | deg/s / length/s |
| `jointActuationForce` | applied actuation effort | f32 × 1 | force / torque |

Properties (authoring inputs; persist across steps):

| Attribute | Meaning | dtype × lanes | Units |
|---|---|---|---|
| `jointStiffness` | drive effort per unit position error | f32 × 1 | effort/coord |
| `jointDamping` | drive effort per unit rate error | f32 × 1 | effort/rate |
| `jointLimit` | (low, high) motion limit | f32 × 2 | degrees / length; `FLT_MAX` = free |
| `jointMaxVelocity` | joint rate bound | f32 × 1 | USD rate |
| `jointMaxForce` | drive effort bound | f32 × 1 | force |
| `jointArmature` | added rotor inertia | f32 × 1 | inertia |
| `jointStaticFriction` | break-away effort | f32 × 1 | effort |
| `jointDynamicFriction` | sliding effort | f32 × 1 | effort |
| `jointViscousFriction` | effort per unit rate | f32 × 1 | effort/rate |
| `jointSpeedEffortGradient` | drive-envelope rate per effort | f32 × 1 | USD |
| `jointMaxActuatorVelocity` | actuator rate bound | f32 × 1 | USD rate |
| `jointVelocityDependentResistance` | envelope resistance per rate | f32 × 1 | effort/rate |
| `jointDriveType` | 0 none / 1 force / 2 accel | u8 × 1 | enum |

`jointLimit` is **conditional**: writing a *finite* limit onto a non-limited axis is refused and
writes nothing (PhysX forbids making an axis limited while its articulation is in a scene); a
`±FLT_MAX` (free) sentinel is a legal no-op. After writing `jointPosition`, call
`ovphysx_update_articulations_kinematic` (C) / `update_articulations_kinematic()` (Python) to refresh
dependent link transforms. Wait for the C task before consuming those transforms;
Python waits internally. On a DirectGPU scene, pass the stream you committed on, so
the refresh is ordered after the write on the device (see
[Articulation kinematic update](../api.md#articulation-kinematic-update)). The update supports CPU and GPU scenes and acts on the
selected articulations (see [Known limitations](limitations.md)).

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

## Whole vehicles

Whole vehicles (`VEHICLE`) accept fixed, host-resident columns with one row per selected vehicle
prim. No earlier read or step is required on a CPU scene. DirectGPU vehicles are unsupported.
Select the same prim as `RIGID_BODY` to set chassis pose or velocity; select its wheel attachment
prims as `VEHICLE_WHEEL` for raw-wheel controls.

| Attribute | Meaning | dtype x lanes | Range / applicability |
| --- | --- | --- | --- |
| `accelerator` | persistent accelerator input | f32 x 1 | finite [0,1]; DriveBasic / DriveStandard |
| `brake0`, `brake1` | persistent brake-channel input | f32 x 1 | finite [0,1]; DriveBasic / DriveStandard |
| `steer` | persistent steering input | f32 x 1 | finite [-1,1]; DriveBasic / DriveStandard |
| `requestedGear` | persistent gear request (write-only) | i32 x 1 | [-1,30] or 255; DriveBasic / DriveStandard |
| `thrust0`, `thrust1` | persistent track thrust (write-only) | f32 x 1 | finite [-1,1]; tank drive only |
| `resetToRestState` | execute native rest reset (write-only) | u8 x 1 | 0 leaves state unchanged; 1 resets; other values fail; all rows require eligible chassis |

All whole-vehicle controls are `CONDITIONAL` in `ovphysx_writability`, including the
write-only gear, thrust and reset commands: drive subtype or chassis state can refuse them.
Opening the write checks every selected vehicle and rejects incompatible rows. A mixed selection cannot silently
skip unsupported vehicles. At commit, every addressed vehicle and value is validated before
any setter runs. An invalid value leaves this entire group unchanged.

An unconfigured brake channel accepts a write with no effect and reads zero. On DriveBasic,
accepted negative / zero / positive gear requests select reverse / neutral / forward; 255 is
positive and therefore selects forward. On DriveStandard, 255 selects automatic control when an
autobox is configured. Without an autobox the request is accepted with a warning, and
`automaticTransmission` remains zero. Requests for unconfigured forward gears have no effect.
A successful request is not a promise that `currentGear` or the internal `targetGear` immediately
becomes that value. These controls persist until replaced.

### Reset ordering

`resetToRestState` resets each row containing 1 once when its group is committed. It is never
replayed on later steps. A zero row makes no mutation but still undergoes target eligibility
checks. Releasing a filled group without committing does nothing. The normal single-commit rule
prevents executing the same group twice.

Reset preserves chassis pose and existing controls. It clears chassis velocity and accumulated
forces, resets wheel dynamics, finishes a pending gear change at its internal target and restores
the configured engine idle speed. It is not a restoration of the originally authored scene.
A disabled vehicle controller can be reset, but any addressed chassis with `disableSimulation`
set rejects the group before mutation, including a row whose reset value is zero. Use the chassis
`isSleeping` read when the current sleep state matters.

Commits execute in caller order and are not a transaction. For a new driving episode, commit
reset first, then the desired chassis pose and velocity, then the desired controls. Resetting
after a velocity write intentionally clears that velocity. Preserved throttle can move the
vehicle again on its next step; write zero if that is not desired.

## Vehicle wheels

Vehicle wheels (`VEHICLE_WHEEL`) accept host-resident raw-wheel controls only when the vehicle
has no managed drive. DirectGPU vehicles are unsupported. No prior read is needed. All three
controls are write-only and persist until replaced; write zero to stop applying them.
The composed wheel pose and all [wheel telemetry](readable.md#vehicle-wheels) are read-only.

| Attribute | Meaning | dtype x lanes | Units / accepted values |
| --- | --- | --- | --- |
| `driveTorque` | persistent drive torque | f32 x 1 | torque; finite |
| `brakeTorque` | persistent brake torque | f32 x 1 | torque; finite and nonnegative |
| `steerAngle` | persistent steering command | f32 x 1 | radians; finite |

Incompatible wheels or an invalid value reject the group before any wheel changes. Select a
raw-wheel vehicle's attachments explicitly when a scene also contains managed vehicles.

## Character controllers

`CHARACTER_CONTROLLER` accepts one host column, `move`. It is **CONDITIONAL**:
refused on a DirectGPU scene (`eENABLE_DIRECT_GPU_API`), because
`PxController::move` updates the proxy through `setKinematicTarget` and that CPU
target is not uploaded. Pose is the controller prim's `RIGID_BODY` position and
orientation: the backing actor's global position and the controller prim's world
orientation, with the capsule-axis correction applied as in native prim write-back.

| Attribute | Meaning | dtype × lanes | Frame / values |
|---|---|---|---|
| `move` | world-space displacement applied once at commit | f32 × 3 | world |

Each successful commit calls `PxController::move` once with that displacement,
minimum distance 0, the scene timestep, and the default filters. A finite zero
vector is passed through too. A later step does not replay it. Two commits before
one step each apply.
Non-finite values fail before any controller is updated.
There is no teleport, local frame, or gravity control on this type. Collision
flags are not exposed.

### Riding a moving platform

The application must include gravity or a downward grounding displacement in its
`move` writes. Scene gravity is not automatically added to a character-controller
move. A zero displacement can carry the controller with a previously touched
platform, but repeated zero moves do **not** guarantee sustained following: the
supporting contact can be lost. A controller given no move commits does not update
its position to follow the platform either.

After settling the controller onto a kinematic platform, advance the platform
with a kinematic target, step the scene, and commit the controller's movement,
including its downward component, on every frame. For example, for a grounded
`/World/Rider` in a metre-scale, Y-up CPU scene running at 60 Hz, the following
commit after each platform step maintains contact while a level platform moves
horizontally at 1 m/s:

```python
import numpy as np
from ovphysx.types import SimObjectType

# Run after each platform target and physx.step_sync(1.0 / 60.0).
with physx.write_sync(
    SimObjectType.CHARACTER_CONTROLLER, "move", pattern="/World/Rider"
) as session:
    for group in session.groups:
        group.tensors[0].assign(np.array([[0.0, -0.01, 0.0]], dtype=np.float32))
        session.commit_sync(group)
```

The downward `0.01` is a displacement in scene length units, not a gravity setting;
choose it from the application's movement/gravity policy for its scale and
timestep. Add any desired lateral motion to the same vector. The rigid-body pose
read reflects the move after the next simulation step, including one final step
when checking the last move of a run.

## Deformable volumes / surfaces

Deformable volumes / surfaces (`DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE`) are **GPU-only,
device-resident**. `restPoints` and the connectivity indices are read-only. `kinematicTarget` (volume
only) drives sim-mesh nodes to a target through a **runtime-owned per-body buffer** whose lifetime
outlives any single write — so, unlike a freed-at-release write column, the solver keeps reading valid
memory. Opening the write sets up every body it covers, every node free; the `kinematicTarget` read then
lists those bodies. It is **CONDITIONAL**: refused on a volume already made kinematic another way
(`eKINEMATIC` / `kinematicBody`, or a tensor-binding view's `ePARTIALLY_KINEMATIC`), and pinning is
**monotonic** — a node a write marks free while it is pinned stays pinned at its last target, the commit
writes every other node and succeeds, and an error is logged later. Per node, the `.w` lane selects whether
it is pinned, with **inverted** polarity: `w == 0` drives the node to its `xyz` target, `w != 0` leaves it
free. On a DirectGPU scene the commit waits for nothing on the host and can be captured in a CUDA graph.

| Attribute | Meaning | dtype × lanes | Frame |
| --- | --- | --- | --- |
| `points` | current nodal positions (scale-corrected; `.w` inverse-mass lane preserved) | f32 × 3 | sim-mesh-local |
| `velocities` | nodal velocities | f32 × 3 | world |
| `kinematicTarget` | drive sim-node targets (`.w` active-flag; volume only, CONDITIONAL) | f32 × 4 | sim-mesh-local |

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

## Debug render

Debug render (`DEBUG_RENDER`) settings are writable without an attached stage. Geometry columns
(`pointPosition` / `pointColor`, `linePosition` / `lineColor`, `trianglePosition` / `triangleColor`)
are read-only. A write session names exactly one attribute. Any other count fails at open, before
the Python context body runs: it publishes no group and commits nothing. `enabled` and `scale` are
two sessions. Requesting `["enabled", "scale"]` in one session raises `TaskError`.

The opened tensor holds the current value, so committing it unchanged keeps that value. Fill the
tensor before committing to set a new one:

```python
import numpy as np
from ovphysx.types import SimObjectType

with physx.write_sync(SimObjectType.DEBUG_RENDER, ["enabled"]) as session:
    for group in session.groups:
        group.tensors[0].assign(np.array([1], dtype=np.uint8))
        session.commit_sync(group)

with physx.write_sync(SimObjectType.DEBUG_RENDER, ["scale"]) as session:
    for group in session.groups:
        group.tensors[0].assign(np.array([2.0], dtype=np.float32))
        session.commit_sync(group)
```

A commit pushes the instance's full settings onto scenes of this attach; a parameter
value of 0 turns that parameter off. A `scope` write is uint64 shape `[4097]`: element 0
is the count (at most 4096) and elements 1..count are the path tokens. Count 0 draws
everything. A `cullingBox` write is float32 shape `[7]`: element 0 is 1 to store the next
six floats (min xyz, max xyz, min ≤ max), or 0 to clear the box. Scope tokens are cleared
on detach because they belong to the stage path dictionary.

| Attribute | Meaning | dtype × lanes |
|---|---|---|
| `enabled` | visualization on/off | u8 × 1 |
| `scale` | visualization scale (finite, ≥ 0) | f32 × 1 |
| `worldAxes` … `sdf` | visualization parameter magnitude (finite, ≥ 0) | f32 × 1 |
| `cullingBox` | element 0 stores (1) or clears (0); the next six floats are min xyz, max xyz | f32 × 1, shape `[7]` |
| `scope` | element 0 is the token count; elements 1..count are interned path tokens | u64 × 1, shape `[4097]` |

In C, pass `&OVPHYSX_NAME_DEBUG_*`. In Python, pass the string name.
