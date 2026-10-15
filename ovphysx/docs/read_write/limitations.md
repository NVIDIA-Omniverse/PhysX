<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Known limitations and gaps

Current boundaries of the session read/write API and unsupported combinations. See [the session model](index.md) for the API itself.

- **Fixed-joint names.** The prim-path and DOF-column identity attributes replace body and
  DOF-bearing joint names, but fixed (zero-DOF) joint names still require the deprecated
  tensor-binding API. Whole-articulation topology and metadata are available through
  `ovphysx_read`; see [Whole articulations](readable.md#whole-articulations).
- **Wake / sleep is a whole-group command, not indexed.** The `wake` write attribute is the
  non-deprecated successor to per-binding wake/sleep, but it drives the *entire mapped group* in one
  commit: nonzero wakes, zero sleeps, with no leave-unchanged value — so a zero row actively sleeps
  that body (except on an `eDISABLE_SLEEPING` scene, where the sleep half is a no-op).
  To toggle a subset while leaving other bodies untouched, narrow the query with a
  [prim-set selection](index.md#selecting-a-subset-of-prims); selected sleeping bodies remain included.
  There is no per-row mask inside the mapped group. Refused on a DirectGPU
  scene (`eENABLE_DIRECT_GPU_API`), since the SDK disables sleeping there. Read `isSleeping`
  separately for the actual state of emitted rigid bodies or articulation links; it has no
  point-instancer form. [Read semantics and mode restrictions](readable.md#rigid-bodies).
- **Articulation forward kinematics.** `ovphysx_update_articulations_kinematic` updates the
  articulations selected by an `ARTICULATION` query on CPU and GPU. POSITION and VELOCITY flags
  select the requested refresh; DirectGPU uses the SDK's all-component refresh. It does not
  perform collision or contact processing, and the first GPU update may warm up the scene.
- **Very large articulation inverse dynamics.** A single inverse dynamics column wider than 65 535 lanes is dropped
  for its cohort. The read succeeds with an optional-summary
  `OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION` bit for the affected request; other
  supported columns remain available (see [Whole articulations](readable.md#whole-articulations)).
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
  After physics objects are created or released, evaluation waits for a simulation step, as PhysX uploads
  shape data to the GPU only when it simulates; any such change asks for the step, not only a change to an
  SDF shape.
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

## See also

- [C API Reference](../api.md) — entry-point signatures and Doxygen
- [Python API Reference](../python_api.rst) — `PhysX.query` / `PhysX.read` / `PhysX.write`
- [ovphysx overview](../ovphysx_overview.md) — the simulation loop and core concepts
- [Population](../population/index.md) and [Stage units](../population/stage_units.md) — how a stage
  is populated from schema, fallbacks, and unit scaling
