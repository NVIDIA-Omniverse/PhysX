<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Known limitations and gaps

Current boundaries of the session read/write API and unsupported combinations. See [the session model](index.md) for the API itself.

- **Structural changes through a native PhysX pointer are not recommended.** A structural change adds,
  removes or re-inserts objects, moves them between scenes, or attaches or detaches shapes. ovphysx
  cannot detect such a change. After one, ovphysx reads and writes are not guaranteed to work until the
  stage is re-attached: a call may fail, read or write the wrong object without any error, or crash.
  Structural changes through ovstage and `update_from_ovstage`, clone, or a re-attach of the stage keep
  reads and writes working. See
  [Structural changes through a pointer](../developer_guide.md#structural-changes-through-a-pointer).
- **Removing an attribute while keeping its prim is not recommended.** After an attribute is removed
  from a prim that stays (with `ovstage_delete_attributes`, or by a USD edit that population applies),
  the PhysX value is undefined: depending on the attribute, it keeps its last value or returns to a
  default. Write the value you want instead.
- **A point-instancer write does not survive an articulation entering its scene.** Articulations added
  through `update_from_ovstage` or created at attach enter their scene at the next step. An instancer write
  opened before that step is refused at commit with nothing written; reopen it after the step.
- **Wake / sleep has no leave-unchanged value.** The `wake` write attribute is the
  non-deprecated successor to per-binding wake/sleep. It drives every row it commits: nonzero wakes,
  zero sleeps — so a zero row actively sleeps that body (except on an `eDISABLE_SLEEPING` scene,
  where the sleep half is a no-op). To leave bodies untouched, commit with a
  [row mask](writable.md#write-timing-ordering-and-persistence) that leaves them out, or narrow the
  query with a [prim-set selection](index.md#selecting-a-subset-of-prims); selected sleeping bodies
  remain included. Refused on a DirectGPU
  scene (`eENABLE_DIRECT_GPU_API`), since the SDK disables sleeping there. Read `isSleeping`
  separately for the actual state of emitted rigid bodies or articulation links; it has no
  point-instancer form. [Read semantics and mode restrictions](readable.md#rigid-bodies).
- **Articulation forward kinematics.** `ovphysx_update_articulations_kinematic` updates the
  articulations selected by an `ARTICULATION` query on CPU and GPU. POSITION and VELOCITY flags
  select the requested refresh; DirectGPU uses the SDK's all-component refresh. It does not
  perform collision or contact processing, and it never warms the scene up: on a DirectGPU scene
  that has not stepped yet it is refused.
- **Unsupported device combinations.** Vehicle controls and telemetry are host-resident and
  unavailable on DirectGPU; deformables and particles require a
  CUDA context (they emit nothing on a sim without one). Contact events and separate child-collider
  contact-sensor rows are available on CPU simulation and GPU dynamics with CPU readback. DirectGPU
  omits events (its raw contacts carry the points) and separate child-collider rows: a selection naming
  only such a collider discovers no rows. A standalone static collider remains a DirectGPU sensor row
  but reads zero force.
- **Contact limits.** A contact query takes at most 21845 filters per sensor (the force matrix's
  lanes). On a DirectGPU scene the per-contact order within a sensor (or filter column) is unspecified,
  and a sensor or filter that is a point-instancer prototype stands for its first instance only, as in
  the contact binding (CPU simulation and GPU dynamics with CPU readback cover every instance).
  For a lost contact pair whose sensor-side shape was removed the report drops that side's body id, so the
  pair reaches a sensor's events only through a collider id that still names it (a body that is its own
  collider).
- **SDF evaluation is DirectGPU-only, in mesh-local space.** `ovphysx_evaluate_sdf_shapes` refuses
  a scene without the DirectGPU API (CPU evaluation is not implemented) and a selection spanning
  more than one scene. One call evaluates at most 65535 shapes, the most a single PhysX launch addresses
  (one CUDA grid row per shape), and at most 2^31 - 1 points in all (N x Q, which PhysX indexes in 32-bit
  arithmetic); split a larger selection or point set. Points and gradients are in each shape's
  mesh-local space, with no world-space form, and every shape takes the same number of points per call.
  After physics objects are created, released, or transferred between scenes, evaluation waits for a simulation step, as PhysX uploads
  shape data to the GPU only when it simulates; any such change asks for the step, not only a change to an
  SDF shape.
- **Character-controller orientation retains native controller limitations.** A runtime
  `physxCharacterController:upAxis` edit does not refresh the creation-time capsule-axis
  correction, so later pose output can disagree with the new up direction. Authored rotations
  on controller prims are not retained by native controller simulation or its pose output.
- **Kinematic-target writes require CPU or GPU scenes with CPU readback.** Driving a rigid body
  through `kinematicTargetPosition` / `kinematicTargetOrientation` is refused on a DirectGPU scene
  (the CPU-integrated target is never uploaded to the device). Direct `position` / `orientation`
  writes are device-backed on DirectGPU, but teleport the body without a kinematic target's step
  velocity; they are not an equivalent way to carry a resting load.
- **`ovphysx_writability` is not the ovstage-drain coverage map.** It reports what the *imperative
  write session* accepts. The separate ovstage value-change drain deliberately does **not** apply
  some of those attributes (tendon knobs, the rigid mass frame, deformable bind-pose points,
  local-space-velocity bodies, multi-axis D6 joints), falling back to the parse path — and on a
  DirectGPU scene a tendon change via the drain does not reliably take effect. Do not treat the
  writability query as the drain's contract.
- **Deferred rigid mass-frame edits can replace later session writes.** A completed
  `update_from_ovstage()` queues the mass-frame recomputation for the next step.
  A session mass/inertia/COM commit before that step does not cancel it, even if
  an immediate read returns the session value. Complete the step before committing
  an override that must survive the pending recomputation. See
  [mass-edit ordering and persistence](writable.md#mixing-ovstage-mass-edits-and-session-writes)
  for an example and the effect of the intervening step.
- **DirectGPU live velocity edits require world space after the first step.** Once the scene's
  first simulation step initializes DirectGPU, `update_from_ovstage()` does not update solver
  velocities for dynamic (non-kinematic) rigid bodies with `physics:localSpaceVelocities = true`.
  Their `physics:velocity` and `physics:angularVelocity` edits reach unsupported PhysX setters;
  PhysX logs errors and the solver keeps its previous velocities, even if the drain completes.
  Initial local-space velocities and edits drained before that first step still apply.
  Kinematic bodies use a separate surface-velocity path. For live ovstage edits,
  configure world-space velocities before attaching and author world-space vectors. To drive a
  local-space-authored body at runtime, use the session write API's world-space
  [`linearVelocity` and `angularVelocity`](writable.md#rigid-bodies) instead.
  Session `angularVelocity` uses radians per second; USD `physics:angularVelocity` uses degrees
  per second, so convert by `pi / 180` as well as rotating the vector into world space.
- **Local-space velocity edits are not reframed for articulation roots.** `physics:localSpaceVelocities`
  is honored only for standalone rigid bodies and PointInstancer prototypes. On an articulation, the
  root's authored local-space velocity is reframed at load, but a later velocity edit — through
  `update_from_ovstage()` or a USD property change — is applied in its raw local numbers (no scale and
  rotation into world space), so it lands in the wrong frame; on a DirectGPU scene after its first step
  it is not applied at all. To drive an articulation root's velocity at runtime, use the session write
  API's world-space [`rootLinearVelocity` / `rootAngularVelocity`](writable.md#whole-articulations)
  (angular in rad/s, not the USD deg/s); a link's velocity is read-only.
- **Runtime scale edits do not reach every body.** Physics keeps the scale a body was created with,
  which [`worldScale`](readable.md#rigid-bodies) reports. A scale change that arrives as an edit (a
  USD edit, or an OVStage transform change, including one that animates a kinematic body) recreates
  a CPU-scene rigid body, kinematic or dynamic, with the new scale. Character controllers, DirectGPU
  rigid bodies, articulation links and kinematic bodies driven by time-sampled transforms do not
  apply it: they keep simulating, and reporting, the creation scale. The OVStage output helpers
  (`ovphysx::utils::writeWorldTransformsToOvstage()` and `ovphysx.utils.step_and_write_to_ovstage()`)
  compose each body's world matrix with that stored scale, so on every publication they overwrite an
  `omni:fabric:worldMatrix` that a hierarchy recomputation derived from the edit, and the edit appears
  to snap back. Their hierarchy computation at the output ordinal also folds that matrix into the
  local transform, so the edit is overwritten there too. To change such a body's scale, recreate it: remove and re-add the prim, or reattach
  the stage.
- **The OVStage output helpers publish a character controller in its capsule actor's frame.** A
  controller's `RIGID_BODY` `orientation` is its kinematic capsule actor's
  ([Character controllers](readable.md#character-controllers)), which PhysX rotates so the capsule
  axis follows the controller's up axis, while its `worldScale` keeps the controller prim's authored
  axes. The helpers compose the two, so the `omni:fabric:worldMatrix` they publish for a controller
  prim carries that up-axis rotation (none only for an X up axis), and a nonuniform scale lands on
  the wrong axes. Their hierarchy computation folds that matrix into the controller prim's local
  transform as well. Do not rely on the helpers' matrix for controller prims.

## See also

- [C API Reference](../api.md) — entry-point signatures and Doxygen
- [Python API Reference](../python_api.rst) — `PhysX.query` / `PhysX.read` / `PhysX.write`
- [ovphysx overview](../ovphysx_overview.md) — the simulation loop and core concepts
- [Population](../population/index.md) and [Stage units](../population/stage_units.md) — how a stage
  is populated from schema, fallbacks, and unit scaling
