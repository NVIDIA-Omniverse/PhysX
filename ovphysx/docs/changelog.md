<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Changelog

All notable changes to `ovphysx` are documented in this file.

## [0.7.0] - Date 2026-10-14

> **API, ABI and package migration required.** Rebuild native consumers against
> the 0.7.0 headers. Lifecycle and session calls return task receipts whose
> outcomes must be collected; Python's synchronous read/write conveniences now
> use `*_sync` names. Replace `reset_stage()` with detach followed by a successful
> wait, and move raw contact-report consumers to contact sessions. Install and
> register `physx-usd-schemas` separately. See the
> [0.6 to 0.7 migration guide](migrating_0_6_to_0_7.md) and the breaking changes
> below for configuration, schema and removed-API replacements.

### Added

- **Contact sensing is available through query/read sessions.**
  `ovphysx_query_contacts()` and Python `ContactSelection` pair sensor and
  filter selections. Read net normal and friction forces, force matrices,
  filtered contact detail, raw contacts and `contactEvent*` payloads through
  `CONTACT_SENSOR` queries. Arrays fit the current step's contacts without a
  caller-supplied capacity; detailed events remain unavailable on DirectGPU.
  A reporting collider can have its own row on host scenes, overlapping its
  body's aggregate row: do not sum both. Apply `PhysxContactReportAPI` to a
  selected child collider and enable reporting on its owning body, link or
  articulation root. On DirectGPU, separate child rows are omitted and
  standalone static sensors read zero force. See
  [Contact sensors](read_write/readable.md#contact-sensors).
- **Read and write queries can select a subset of prims.** Use
  `ovphysx_query_selection()` or the `prim_paths` / `pattern` arguments of
  `PhysX.query()`, `read_sync()`, `read_tokens_sync()` and `write_sync()`.
  `prim_paths` contains interned path handles and takes precedence over a
  pattern. Selections include sleeping objects, cannot be combined with Python
  `scope=ObjectScope.ACTIVE`, and may validly match no objects.
- **Vehicle controls and telemetry use the session API.** Query whole vehicles
  for managed controls and transmission state, chassis rigid bodies for pose
  and velocity, and wheel attachments for telemetry or raw-wheel controls.
  `resetToRestState` is a one-time command; requested controls are distinct from
  actual state. Vehicle columns are host-resident and unavailable on DirectGPU.
- **Capsule character controllers accept session movement commands.** Write a
  world-space displacement to `move` on `CHARACTER_CONTROLLER`; each commit
  applies it once. A zero displacement still follows a moving platform, and
  rigid-body pose reads reflect the movement after the next simulation step.
  DirectGPU movement is unsupported.
- **Articulation reads expose topology and DOF identity.** Read `dofCount`,
  `linkCount`, `jointCount`, `fixedTendonCount`, `spatialTendonCount` and
  `isFixedBase`, including mixed articulation topologies. `dofJointPath` and
  `dofAxisIndex` map each DOF to its joint and axis; floating-base
  inverse-dynamics matrices place these DOFs after six root columns. DOF rows
  are padded to the widest articulation in the group.
- **Rigid-body and volume-deformable kinematic targets are accessible through
  sessions.** `kinematicTargetPosition` and `kinematicTargetOrientation` drive
  rigid bodies on CPU and GPU scenes with readback; updating one preserves the
  other, and reads without a pending target return the current pose. DirectGPU
  rigid-body writes are rejected, and articulation-link targets are read-only.
  Volume-deformable `kinematicTarget` provides
  simulation-mesh-local positions and an active flag per node, including on
  DirectGPU; freeing every node removes the drive. Reads include only driven
  bodies, and surface deformables do not support the attribute.
- **Sessions expose sleep state and wake/sleep commands.** Read `isSleeping`
  for rigid bodies or articulation links; links report their articulation's
  state, and scenes with sleeping disabled, including DirectGPU, report zero.
  Write `wake` on rigid bodies with nonzero to wake or zero to sleep. These
  writes reject DirectGPU and articulation links and skip kinematic or
  simulation-disabled bodies. Use all-object scope or an explicit selection
  to include sleeping bodies.
- **SDF shapes can be evaluated through a query.**
  `ovphysx_evaluate_sdf_shapes()` and `PhysX.evaluate_sdf_shapes()` write
  gradients and signed distances for caller-supplied points into a GPU buffer.
  Points and gradients use each shape's mesh space. This DirectGPU-only path
  replaces deprecated SDF views; a query must stay within one scene and each
  evaluation supports at most 65,535 shapes. Native callers include
  `ovphysx_geometry_query.h` explicitly.
- **Profiling supports Chrome trace output and covers every public C call.**
  Select `OVPHYSX_PROFILER=trace` or `nvtx` before creating the first instance,
  or set `GlobalConfigKey.PROFILER_BACKEND`. Set `OVPHYSX_PROFILER_OUTPUT` for
  the trace destination. Traces are written when the last instance shuts down
  and include API, runtime and SDK zones; SDK phases spanning threads are
  omitted under both backends.
- **Optional companion services support legacy physics extension workflows.**
  Services cover inspection, queries, events, interaction, copied-data cooking,
  mass and attachment authoring, and preview. Optional Python utilities use
  application-provided USD. Event polling reports lost history explicitly;
  custom joint and geometry access requires separate, build-matched SDK headers.

### Changed

- **The bundled simulation engine advances to PhysX 5.12.** Deformable collision
  filtering, attachments and material reuse have corrected behavior, alongside
  joint force/torque reporting and material-limit and scene-query fixes.
  Existing scenes can produce different collision and material responses.
- **TGS applies external forces every position iteration by default.**
  `physxScene:enableExternalForcesEveryIteration` now defaults to `true` and is
  deprecated. Scenes that did not author it can produce different trajectories,
  including more accurate free-fall displacement. Author `false` to retain
  once-per-step application while the attribute remains supported; PGS behavior
  is unchanged.
- **Several instances can keep independent stages attached.** Each instance
  holds at most one live stage, and the same stage cannot be attached twice.
  Reads and writes on one instance can complete while another instance steps.
  Detach and destruction preserve other instances. Queries, subscriptions and
  retained errors belong to their owner; subscriptions and debug-render settings
  survive its stage replacement, except stage-specific debug scope tokens.
  CPU-only policy, tolerance scale and recording transport remain process-wide.
- **Successful object-change unsubscription waits for callbacks to finish.**
  After `ovphysx_unsubscribe_object_changes()` succeeds, the subscription will
  no longer access `user_data`. Unsubscribing from that same callback returns
  `INVALID_STATE` and leaves it active. Drop cached native pointers before the
  matching destroy callback returns; refresh caches after cloning, which does
  not emit object-change notifications.
- **OVStage advances to the 0.3 line.** Python and native applications must use
  the matching wheel and SDK dependencies. Native SDK applications can use the
  supplied fetcher and shared download cache; Python installs the separately
  pinned wheel. Population shared with rendering retains the authored physics
  types and point-instancer prototype targets.
- **Additional legacy families emit deprecation warnings.** Scene queries,
  SDF views and `ovphysx_get_object_type()` join tensor and contact bindings in
  the legacy headers. `ovphysx.h` still includes those declarations, and Python
  methods remain available. Prefer session APIs where a replacement exists;
  see [Migrating off Tensor Bindings](tensor_binding_migration.md).
- **Recording exports use consistent OVD names.** Use
  `ovphysx_ovd_start_recording()`, `ovphysx_ovd_stop_recording()` and
  `ovphysx_ovd_is_recording()`. The old exports and `ovphysx_handle_t` remain
  deprecated aliases; new native code uses `ovphysx_instance_handle_t`.
  Python recording method names are unchanged.

### Fixed

- **Linux shutdown preserves host exit callbacks and buffered output.** A
  balanced `ovphysx_initialize()` / `ovphysx_shutdown()` lifecycle no longer
  skips earlier host exit callbacks or leaves buffered files empty. Destroy
  all instances before shutdown. Windows hosts, and Linux hosts that load
  independent libraries with terminating exit handlers, must still flush
  output explicitly.
- **Repeated SDK lifecycles preserve logging and clean up caches safely.**
  Log callbacks and source filtering continue to work after shutdown and
  reinitialization. Private cooking caches remain available until their
  background writers stop: Linux removes them at SDK shutdown, while Windows
  attempts removal at normal process exit. Windows cache cleanup remains
  best-effort and may require normal temporary-directory cleanup.
- **CPU-only stage population works without an available CUDA device.**
  Populating a CPU-only stage no longer enters a CUDA-dependent update path.
  GPU-backed stages continue to require CUDA.
- **Attach again rejects stages when ovstage reports missing PhysX schemas.**
  Register both resource directories from the external `physx-usd-schemas`
  package before the first USD schema read (population or export) in each process,
  including workers.
  A refusal requires a process restart with correct registration; the legacy
  `/ovphysx/schemas/requireRegistration = false` setting permits attach with a
  warning but does not restore dropped settings.
- **GPU contact generation avoids invalid memory access.** Sphere contacts
  at heightfield boundary edges no longer trigger CUDA errors. Exhausted
  contact or force buffers discard contacts that do not fit instead of
  publishing invalid data; overflow diagnostics remain.
- **CPU continuous collision detection preserves valid later impacts.** A
  fast-moving body no longer passes through a triangle mesh when an earlier
  grazing hit is rejected.
- **Live stage edits preserve unrelated simulated motion.** Moving a kinematic
  body no longer freezes falling bodies during GPU hierarchy updates. Adding
  visual children or refreshing unchanged prim types also preserves existing
  body poses and velocities.
- **Moving a body between physics scenes no longer duplicates read rows.**
  Whole-body position and velocity reads return each body once, including
  transfers into an initially empty scene.
- **Re-enabling gravity wakes sleeping articulations.** Applying
  `physxRigidBody:disableGravity = false` through a live stage update now
  starts the articulation moving; disabling gravity preserves sleep.
- **Live GPU particle position updates reach the simulation.** Vector arrays
  are no longer silently ignored when read from OVStage.
- **Instanced colliders use their prototype's authored density.** Mass and
  inertia calculations now honor collider density at attachment and after
  later mass-property edits. Explicit instance values retain precedence.
- **Deformables retain authored rest offsets with automatic contact offsets.**
  Volume and surface deformables no longer replace `physxCollision:restOffset`
  with its default in this case. Equal explicit contact and rest offsets
  preserve the rest offset and raise the contact offset with a diagnostic.
- **Local-space velocities on point-instanced bodies are mapped consistently.**
  With `physics:localSpaceVelocities` on a prototype, linear and angular
  instance velocities use the instance's current world frame at load and after
  edits. Linear velocity is scaled then rotated; angular velocity is rotated
  only. Writing velocity back to the instancer arrays uses the inverse mapping.
  Read-session velocities remain world-space. See
  [Local-space instance velocities](simulation_setup/rigid_bodies.md#local-space-instance-velocities).
- **Fixed-tendon gearing beyond ±π is converted instead of discarded.** A
  revolute fixed-tendon axis whose `gearing` exceeded ±π could make the
  articulation non-finite on its first step. Gearings now convert across the
  documented range; values too large to represent after degrees-to-radians
  conversion saturate with a warning identifying the axis and joint.
- **Selected material writes preserve other shapes and untouched properties.**
  Session friction or restitution writes no longer change unselected shapes
  sharing the material. Tensor-binding writes preserve damping, combine modes
  and material flags. At the material limit or without an active scene, writes
  report failure instead of crashing; affected shapes retain their materials,
  although earlier rows may already have changed.
- **Articulation link forces reach the intended links.** Indexed tensor writes
  handle mixed articulation sizes and reordered GPU indices correctly.
  Separate DirectGPU session writes to different articulations before a step
  no longer clear the forces or torques written by earlier sessions.
- **GPU tendon updates affect the next simulation step.** Fixed and spatial
  tendon stiffness, damping, limit stiffness and offset, along with attachment
  coefficients and rest lengths, no longer read back successfully while leaving
  the GPU solver unchanged. The earlier wake-up workaround is no longer needed.
- **Retained accelerations no longer corrupt subsequent TGS steps.** On GPU
  rigid bodies, one-time impulses and velocity changes are no longer replayed
  as persistent forces. CPU articulation links also retain the intended
  accelerations, correcting motion under gravity when acceleration retention
  and per-iteration external forces are enabled.
- **GPU articulation contacts and spherical-joint drives are corrected.**
  Scenes with more than 32 articulations and an articulation exceeding 64 links
  no longer suffer incorrect or nondeterministic contact responses from this
  case. Spherical-joint drives with performance envelopes use the correct joint
  effort, avoiding missed targets and incorrectly applied effort limits.
- **Failed SDK initialization leaves other live instances usable.** Invalid
  stage units and initialization failures produce errors instead of crashes or
  partially initialized state. Correct the stage and retry attachment after
  collecting the failed task's outcome.
- **Disabled deformables no longer crash filtered-pair creation.** Collision
  filtering skips disabled deformables that have no simulation object.
- **Raycasts and sphere sweeps tolerate invalid mesh material indices.** A hit
  whose triangle refers outside the shape's material table still returns the
  hit with an empty material identity and an error, instead of crashing.
- **Local linear velocity respects non-uniform scale before rotation.**
  Loading and live velocity edits now scale in local axes before converting to
  world space. Rotated, non-uniformly scaled bodies therefore move in the
  intended direction and at the intended speed.
- **Live Newton joint edits reach the simulation.** Changes to
  `newton:armature`, `newton:friction` and `newton:damping` were applied at load
  but ignored on a live stage. Per-axis `PhysxJointAxisAPI` values retain
  precedence, and removing that API at runtime restores the Newton values.
- **Cloned contact sensors follow the originals' eligibility rules.** The exact
  selected object must carry `PhysxContactReportAPI`; a tagged parent no longer
  qualifies an untagged cloned child or adds unintended wildcard sensors.
- **Contact-binding force reads stay correct after zero-duration steps.**
  Both `step()` and `step_sync()` retain the last simulated step's
  impulse-to-force conversion when `dt` is zero.
- **Invalidated tensor views fail cleanly.** Destroying an articulation
  invalidates rigid-body views of its links, and resets before simulation
  starts invalidate existing views. Incoming joint-force and tendon-damping
  reads through stale views report failure instead of crashing.
- **Articulation and tendon sessions handle initialization correctly.**
  Queries no longer index incomplete link metadata out of bounds or publish
  partial results when a later articulation is invalid. Articulation root,
  link and joint writes work before the first step on CPU and GPU scenes with
  readback. Reads refresh articulation and tendon membership after insertion,
  avoiding empty results left over from initialization. DirectGPU articulation
  and tendon commits still require a simulation step and now report that
  requirement explicitly.
- **Rejected kinematic targets report their cause and body path.** Disabled
  bodies, non-finite positions and invalid quaternions produce task errors even
  with logging disabled. Rejected batches leave their valid peers untouched.
- **Linux wildcard selections handle very long prim names without crashing.**
  Tensor bindings and session queries retain the existing pattern grammar and
  pattern-length limit without imposing a new prim-name limit.
- **Python destruction protects borrowed read arrays.** `PhysX.destroy()`
  refuses to release storage while a read array or a view of it remains alive.
  Release those borrows before retrying destruction.
- **Python debug-render sessions work with an attached stage.** Settings writes
  and geometry reads no longer fail because the query loses its attachment.
  Attribute tokens use the attached stage's dictionary; detaching still
  invalidates existing queries and sessions.
- **Python tensor transfers wait for CUDA producers before accessing buffers.**
  Full, indexed, masked and cached transfers honor pending Warp work and the
  CUDA DLPack stream handoff, preventing stale writes and read results being
  overwritten by unfinished buffer initialization. Callers must still
  synchronize raw `DLTensor` and pinned host buffers themselves. Empty GPU
  contact reads also clear caller buffers correctly when they belong to another
  CUDA context.
- **Writing CPU poses to GPU-backed OVStage matrices preserves the caller's
  CUDA context.** `writeWorldTransformsToOvstage()` restores the previous
  context, including when none was current, for both cached and uncached writes.
- **Asynchronous stepping accepts the same timestep range as synchronous
  stepping.** `step()` now accepts finite, nonnegative values above one second,
  as `step_sync()` already did. Neither path clamps nor subdivides the timestep;
  callers choose a value suitable for collision accuracy.
- **USD authoring utilities handle inactive transforms and invalid inputs.**
  Existing translate, orient and scale operations excluded from the effective
  transform stack are reactivated while preserving precision and reset state.
  Unit-conversion helpers warn and use a scale of one for invalid
  `metersPerUnit`; they do not repair stage metadata. `create_mesh_cone()`
  rejects invalid dimensions or tessellation before editing the stage, while
  zero radius remains supported.
- **Startup worker-pool overrides take effect before workers start.**
  Carbonite tasking and TBB settings now control their pools; invalid counts
  and late changes are rejected. Configure them through the first instance's
  legacy Carbonite overrides. The PhysX dispatcher count is a separate setting.
- **Copied examples find their installed dependencies and scene data.** Python
  examples import schema helpers and locate bundled scenes through the
  installed package. The guides distinguish source-checkout paths from installed
  wheel and SDK sample locations, and identify `scene.usda` as a caller-supplied
  file rather than a bundled scene. Shipped workflows register PhysX and Newton
  schemas before population and add USD references without replacing the scene.
  Bundled CPU and GPU variants now declare their intended units and up axis,
  correcting default-density masses and scene orientation.
- **Windows C sample launchers preserve the caller's environment and exit code.**
  Generated launchers restore the caller's `PATH` and return the application's
  exit status. The shipped instructions include schema setup and launcher use.
- **Simulation guides clarify scene defaults and state interpretation.** GPU
  dynamics and GPU broadphase are defaults; per-scene CPU setup must disable
  GPU dynamics and select a CPU broadphase. DirectGPU live velocity edits
  through OVStage require world-space bodies and values after the first step;
  use session writes for bodies authored with local-space velocities. Rendering
  guidance explains signed scale and why debug wireframes can trail post-step
  poses.

### Breaking Changes

- **Lifecycle and session operations return task receipts.** Check admission,
  then collect execution with `ovphysx_wait_task()` or `ovphysx_wait_all()`;
  acceptance alone does not establish success. This includes attach, update,
  detach, query/read/write creation, fetching, committing and releasing sessions.
  Replace `wait_op()` with `wait_task()` on the returned receipt. Each outcome
  is collected once; an earlier failure does not make a later successful task's
  wait fail. `wait_all()` collects outstanding outcomes in submission order.
  Timed-out waits remain retryable, and a repeated collected wait returns
  `NOT_FOUND`.
- **C wait results carry complete task receipts.** Replace
  `ovphysx_op_wait_result_t` with `ovphysx_wait_task_result_t`,
  `error_op_indices` with `error_tasks`, and `lowest_pending_op_index` with
  `pending_task`. The new fields hold full receipts instead of integer indices.
  Zero-initialize results and inspect completed failures even when a wait
  returns `TIMEOUT`. Read error messages before calling
  `ovphysx_destroy_wait_task_result()`; do not shallow-copy the result or free
  `error_tasks` separately.
- **Python read/write calls distinguish asynchronous and synchronous use.**
  `PhysX.read(query, attributes)` and `write(query, attributes)` return typed
  `Operation` objects on a completed query. Use `read_sync()`,
  `read_tokens_sync()` and `write_sync()` for the former convenience call
  shapes. Writes may request multiple attributes; fetched groups identify their
  original request. Collect commits with `commit_sync()` or
  `commit(group).wait()`; leaving a `write_sync()` context does not collect
  a commit's outcome.
- **Stage teardown uses detach and a successful wait.** `reset_stage()` and
  `ovphysx_reset_stage()` are removed. Keep the stage alive through successful
  detach or instance destruction, and sealed attach/update data through task
  completion. Python `attach_ovstage()` requires the owning `ovstage.Stage`;
  raw handles and `None` raise `TypeError`. Raw-handle integrations use the C
  API. Use `is_stage_attached()` for the immediate ownership check;
  public attachment identities are removed. Replace `step_n_sync()` with a
  loop of `step_sync()` calls, and removed user-task callbacks with application
  code run after the relevant wait.
- **Configuration separates process and instance settings.**
  `ovphysx_create_instance()` now takes only the output handle; unrelated
  legacy creation arguments use `ovphysx_create_instance_with_args()`.
  Set process keys through `set_global_config()` and instance keys through
  `PhysX.set_config()` or their C equivalents. Set `ACTIVE_CUDA_GPUS` after
  creation and before attach; matching environment variables override global
  setters. Carbonite-only settings still use `PhysXConfig.carbonite_overrides`,
  despite the legacy configuration warning. Replace removed OmniPVD startup
  settings with global `OVD_RECORDING_CAPABLE` before creation, followed by FILE or
  TCP recording after attach; see the [OVD migration guide](ovd_api_migration.md).
  Replace `OVPHYSX_NVTX` / `nvtx_enabled` with the profiler backend setting.
- **Schemas are installed separately from the wheel and SDK.** Install
  `physx-usd-schemas` and register both PhysX and deformable resource directories
  before population. Bundled schema resources, `ovphysx_get_codeless_schema_root()`
  and Python `codeless_schema_root()` / `codeless_schema_paths()` are removed.
  Native applications extract the provider wheel as data; Python applications
  locate it through distribution metadata. Native samples fetch the provider
  from public PyPI. Source builds using `--devschema` produce a separate local
  provider, including from the public source export.
  See [Physics Schemas](physics_schemas.md).
- **`PhysxJointAPI` is removed and its authored values are ignored silently.**
  Replace `physxJoint:maxJointVelocity` and `physxJoint:armature` with
  `NewtonJointAPI` values for all DOFs, or `PhysxJointAxisAPI` values per axis;
  per-axis values take precedence. The old joint-friction coefficient has no
  numeric replacement: retune with `newton:friction` / `newton:damping`, or
  per-axis `staticFrictionEffort`, `dynamicFrictionEffort` and
  `viscousFrictionCoefficient`. The population helpers
  `PrimBatch::applyPhysxJointAPI()` and `apply_physx_joint_api()` are also removed.
- **Dedicated contact-report, pointer and debug-render calls move to sessions.**
  Replace `ovphysx_get_contact_report()` and Python `get_contact_report()` with
  `contactEvent*` reads, adapting to per-sensor grouping and session-owned arrays.
  Removed-shape events cannot appear without a selected sensor that can claim
  them; net-force reads do not replace event payloads. Replace
  `ovphysx_get_physx_ptr()` with a `physxPointer` read and the
  `ovphysx_debug_render_*` family with `DEBUG_RENDER` queries. Debug settings and
  geometry are session attributes, and debug queries require no attached stage.
- **Native query and articulation-update signatures change.**
  `ovphysx_query_target_t` and `OVPHYSX_QUERY_TARGET_*` replace the old object
  enum and constants. Include `ovphysx_attributes.h` explicitly for
  `OVPHYSX_ATTR_*` names. Query summaries replace `ovphysx_fetch_query_result()`;
  use the attached stage's dictionary instead of `ovphysx_query_shared_dictionary()`.
  `ovphysx_update_articulations_kinematic()` takes a query and position/velocity
  flags, replacing the binding-based update. Wait before reading updated state;
  DirectGPU may refresh both components for either nonzero mask. Object-change
  subscribe/unsubscribe and `ovphysx_get_last_task_error()` take the owner first.
- **Native CUDA readiness waits use CUDA directly.**
  `ovphysx_cuda_stream_wait_event()` is removed. Use `cuStreamWaitEvent()` or
  `cudaStreamWaitEvent()` on a read group's nonzero borrowed event and check
  the result; never destroy that event. Finish consumers before releasing the
  session. Python orders readiness onto the current Warp stream and raises
  before exposing arrays if the wait fails, without a new Python CUDA dependency.
- **Status codes and fetch exhaustion change.** `END_OF_ITERATION` is removed,
  and `INVALID_STATE` changes from 10 to 9; use named constants. A successful
  fetch ends with a null group, or Python `None`. Check task completion first,
  because a failed fetch can also leave a null group.

### Performance

- **Repeated DirectGPU active-body reads cost less.** Reads with
  `ObjectScope.ACTIVE` reuse the eligible body selection until membership or
  kinematic state changes. CPU active reads are unchanged. Change kinematic
  and simulation-disable flags through supported stage or session APIs;
  mutation through a raw PhysX pointer does not maintain these selections.
- **CPU reads of multiple inverse-dynamics columns share their work.** Reading
  `jacobian`, `massMatrix`, `coriolisForce`, `gravityForce` and
  `centroidalMomentum` together avoids repeated calculations while preserving
  the values returned by separate reads.
- **Whole-scene GPU articulation refreshes have less overhead.** Queries
  covering every articulation use the scene-wide refresh, while partial
  queries retain their selection. Refresh failures are reported instead of
  allowing stale link state to appear successfully updated.
