<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Changelog

All notable changes to `ovphysx` are documented in this file.

## [0.8.0] - Date 2026-11-11

> **C APIs, tensor layouts, scene selection, OVStage population, and task lifetimes
> require migration.** Rebuild native clients, review scene ownership, update
> population and recording calls, and adapt joint and inverse-dynamics tensor
> consumers. Task submission does not imply completion, and explicitly closing a
> Python read invalidates its borrowed arrays. Follow the
> [migration steps](#breaking-changes) below.

### Added

- **Inspect task progress without waiting or collecting results.**
  `ovphysx_query_task_status()` reports an `ovphysx_task_status_t`; Python
  `PhysX.task_status()` returns `TaskStatus`. Finished tasks remain `SUCCEEDED`
  or `FAILED` until a wait collects them, then report `NOT_FOUND`. Continue
  collecting tasks with `wait_task()` or `wait_all()` to report execution errors
  and release their retained outcomes.

- **DirectGPU reads and writes support CUDA graph capture.** A held query can
  supply reads, policy work, writes, and articulation kinematic refreshes in one
  graph, replayed between simulation steps. Volume-deformable `kinematicTarget`
  commits also support capture. Capture requires a driver supporting CUDA 12.5
  or newer, preparation outside capture, and live read/write sessions throughout
  replay. Host and contact reads, populated deformable skin-mesh reads, and
  host-applied writes are excluded. Follow the [device and capture
  requirements](read_write/device.md#capturing-a-write-step-in-a-cuda-graph).
  `ovphysx_commit_group_masked()` and Python `commit(..., mask=)` select rows
  using one Boolean or unsigned 8-bit mask value per row on the group's device.
  Joint masks select degrees of freedom. Character controllers, vehicles, and
  particles do not support masked commits.

- **Experimental compiled stage cache for `attach_ovstage()`.**
  `GlobalConfigKey.COMPILED_STAGE_CACHE` selects reuse of compiled physics for
  later attaches and added subtrees. `AUTOMATIC` uses eligible persistent
  cooked-collider storage; `READ_WRITE` uses a configured directory or a
  process-private cache deleted at shutdown. Keep persistent storage private and
  local: cached results are checksummed, not authenticated. Failed restores fall
  back to ordinary loading; unsupported physics bypasses the cache. Restores
  preserve clone environment IDs and first-step results for affected rotated
  centers of mass. Triangle-mesh caches can be restored across processes without
  a first-step crash, and Debug builds can detach or destroy instances after
  cache hits without aborting. Refer to the [cache modes, eligibility, and
  directory requirements](developer_guide.md#compiled-stage-cache-experimental).

- **Population builders write more physics components.** The C++
  `ovphysx/population/Population.hpp` and Python `ovphysx.population` builders
  gained `PointInstancer`, `PhysxPhysicsJointInstancer`, `GeomSubset`, `Scope`,
  `CollectionAPI`, `PhysxForceAPI`, `PhysxMimicJointAPI`, the PhysX tendon APIs,
  `PhysxPBDMaterialAPI`, and the Newton scene, joint, collision, mesh-collision,
  articulation-root, and mimic APIs. Each has a reference page in the
  [population documentation](population/index.md). Half-precision columns,
  including instance orientations and joint-instance rotations, accept floats in
  C++ as in Python.

- **Articulation reads expose link, axis, and tendon identities.** New
  host-resident, read-only attributes include `linkPath`, `linkInboundJoint`,
  `dofAxisKind`, `dofJointType`, and fixed/spatial tendon root and instance
  identities. Tendon queries also expose `tendonInstance`. These let callers map
  simulation rows back to authored links, joints, and tendon instances;
  `dofAxisKind` identifies the PhysX axis, not the authored joint axis. Refer to
  the [articulation attribute reference](read_write/attributes/articulation.md).

- **Deformable skin-mesh output read.** The `DEFORMABLE_SKIN_MESH` query target
  (C: `OVPHYSX_QUERY_TARGET_DEFORMABLE_SKIN_MESH`, value 25; Python:
  `SimObjectType.DEFORMABLE_SKIN_MESH`) exposes the skinned render-mesh `points`
  of volume and surface deformables. Each skin mesh prim has one read-only,
  device-resident tensor in skin-mesh-local coordinates.

- **Attribute metadata is available without starting physics.**
  `ovphysx.attributes.get_attribute()` and `list_attributes()` expose immutable
  units, layout, residency, and capability metadata without loading PhysX, Warp,
  or USD. Typed C `OVPHYSX_NAME_*` attribute-name objects and an [attribute
  reference](read_write/attributes/index.md) also ship. Catalog support does not
  guarantee that a particular live session admits an operation.

- **OmniPVD can record selected objects and accept TCP receiver selections.**
  Python recording filters select supported rigid bodies, articulations,
  particles, and deformables, with optional secondary attributes. Use
  `wait_for_selection=True` for TCP discovery before detailed recording; FILE
  recording does not support this option. Owner simulation steps apply receiver
  requests; wait for the applied acknowledgment before using a new selection.
  The installed `ovphysx-ovd-investigation` skill and `omnipvd_inspection.py`
  sample explain capture, inspection, and comparison. Inspection requires the
  separate PhysX SDK `ovd-inspect` tool. See the [recording
  tutorial](tutorials/omnipvd_recording.md).

### Changed

- **Structural changes through native PhysX pointers are not recommended.** After
  changing scene membership or shape attachments this way, ovphysx reads and
  writes are not guaranteed until the stage is detached and reattached. Use
  OVStage updates with `update_from_ovstage()` or `clone()` for structural
  changes. See [native pointer guidance](developer_guide.md#structural-changes-through-a-pointer).

- **The bundled PhysX 5.13 engine raises deformable capacity.** A scene now
  supports up to 65,535 deformable volumes and 65,535 deformable surfaces,
  increased from 4,095 of each. Removed actors still count toward the limit
  until the next simulation step completes. The engine also reduces the cost of
  putting large simulation islands to sleep.

- **`update_from_ovstage()` applies eligible values and poses on the GPU.**
  Eligible rigid-body velocity and pose edits use device paths in DirectGPU
  mode. Tendon-level stiffness, damping, and enable-state edits, plus fixed
  tendon limits and rest lengths, also reach DirectGPU simulation. Per-axis and
  per-attachment tendon edits remain unsupported. Run this update outside CUDA
  graph capture. Blocking-stream capture is rejected with
  `OVPHYSX_API_INVALID_STATE` (Python `RuntimeError`); non-blocking-stream capture
  is not detected and executes the update only once, during capture. To trigger
  a pose edit, change the local transform and keep any resolved world matrix
  consistent; changing only `omni:fabric:worldMatrix` does not trigger an update.
  Follow the mode restrictions in [Transform
  inputs](ovstage_integration.md#transform-inputs).

- **Output helpers preserve body scale and publish deformable skin meshes.**
  Rigid-body and articulation-link reads expose `worldScale`, the scale used
  when creating the physics body. Output helpers compose their published
  transforms with this scale and update local transforms so later parent edits
  compose with the simulated pose. Unsupported live scale edits are overwritten;
  recreate or reattach those bodies to change their scale.
  `step_and_write_to_ovstage()` also adds `DEFORMABLE_SKIN_MESH: ("points",)` to
  its default outputs. Rendering population creates the skin prims needed for
  these `sim:points` writes; pass `outputs=` to choose the output set
  explicitly.

- **Independent instances can progress during lifecycle and recording calls.**
  Recording transitions coordinate earlier work across instances without
  collecting task results. Steps on another instance may wait for a lifecycle or
  recording call to finish. Finite waits can time out; retain their pending task
  receipts and retry. One instance still requires external serialization.

- **Volume-deformable target commits retain existing pins.** A pinned volume
  node cannot be freed through `kinematicTarget`. An attempted unpin retains the
  last target and is logged later without failing the commit; a successful
  commit does not mean that the node was unpinned.

- **Particle recordings include more motion data.** Recordings now include
  particle velocities and phase values; secondary data also includes diffuse
  particle positions, lifetimes, and velocities. These payloads increase
  recording readback. Select particle buffers and disable secondary data when
  appropriate, following the [recording
  options](tutorials/omnipvd_recording.md#select-objects-and-data).

- **The deprecated CPU and GPU tensor-binding agent skills are removed.** The
  compatibility APIs remain available, but new and maintained workflows should
  use `ovphysx-output-read` and `ovphysx-session-write` for simulation I/O.

- **Renamed the `basic-workflow` and `clone-environments` agent skills to
  `ovphysx-basic-workflow` and `ovphysx-clone-environments`, so every shipped
  skill now carries the `ovphysx-` prefix.** Update any agent configuration or
  script that selects these skills by name or by their `skills/` directory.

### Fixed

- **Companion picking and dragging work in GPU-dynamics scenes.** Raycast,
  sweep, overlap, force grabs, and D6 dragging are available when GPU dynamics
  retains CPU readback. DirectGPU scenes remain unsupported for these
  operations; read-only native SDK leases remain CPU-only.

- **Mesh-merge colliders work with OVStage population.**
  `PhysxMeshMergeCollisionAPI` produces one merged collider instead of separate
  shapes with ambiguous identities. Merged collision and query geometry honors
  left-handed or reflected members and scaled bounding approximations. Invalid
  members are skipped with a warning; a merge with no usable geometry creates
  no shape. Use `rendering=True` when a member is reached only by expanding
  another collection target. See the [mesh-merge population
  reference](population/PhysxMeshMergeCollisionAPI.md).

- **Population builders preserve vector and quaternion array shapes.** Python
  and C++ point-instancer and joint-instancer arrays now read back as tuples,
  such as `[N, 3]` positions and `[N, 4]` rotations, instead of flattened scalars.
  Prebuilt Warp arrays passed to Python builders must be contiguous; call
  `.contiguous()` on a strided array before passing it.

- **Invalid OVStage clone paths fail instead of hanging.** With the default
  stage implementation, clone requests reject relative or noncanonical paths.
  Use absolute, canonical source and destination paths and a non-root
  destination.

- **Sparse CPU hierarchy updates preserve unrelated GPU results.** Updating
  selected prims no longer discards current GPU transforms and bounds for
  untouched prims.

- **Point-instancer commits no longer crash after an articulation enters the
  scene.** A write opened before that step is refused without writing on CPU and
  DirectGPU. Reopen the write after the step.

- **Clones preserve rigid-body motion locks and cloned material flags.** Cloned
  rigid bodies keep locked translation or rotation axes. Materials copied with
  the hierarchy retain disabled strong friction and compliant-contact
  acceleration-spring settings.

- **A clone that cannot create its replicas fails without crashing.**
  Exhausting the PhysX material table now fails the clone task and removes
  partially created replicas. No replica is added to the scene, so the target
  can be retried after capacity becomes available. The material limit is
  unchanged.

- **Mirrored geometry uses correctly scaled collision and query shapes.**
  Negative-scale point instances now collide using correctly mirrored convex
  hull or convex decomposition shapes in USD and OVStage, regardless of instance
  order. Overlap and sweep queries also use valid box extents for negative-scale
  cubes.

- **Live rigid-body and scene edits update GPU simulation correctly.** Changes
  to lock flags, gravity, maximum depenetration velocity, and contact-report
  thresholds reach the next step. In DirectGPU scenes, edited or newly attached
  colliders use current poses, and collision detection waits for CUDA-ordered
  pose writes. Scene edits and pose writes no longer leave stale collision state
  or access invalid data in affected aggregate and scene-query-only shape cases.

- **Large GPU articulations and live articulation edits simulate correctly.**
  Fixes address incorrect joint forces and updates with 256 or more degrees of
  freedom, and invalid accesses when contacts or constraints share an ancestor
  at link index 64 or higher. Property and structural edits preserve root
  velocities. Floating-base articulation drives reach their requested velocities
  when articulation contacts are solved last. External forces applied per solver
  iteration remain associated with the correct articulation across sleep and
  removal.

- **Articulation mass matrices and CPU velocity limits are corrected.** Mass
  matrices are more accurate far from the world origin. CPU joint-velocity
  limits use the intended axis for prismatic joints, swing-axis revolute joints,
  and spherical joints with affected locked-axis combinations; related
  inverse-dynamics calculations are corrected too.

- **Reinserted particle systems use their own particles and offsets.** Removing
  and adding a particle system back to a scene no longer makes it simulate using
  another system's particle data or offsets.

- **Deformable creation failures and disabled objects are handled safely.**
  Failed body creation cleans up resources; failed skinning allocations disable
  skin output. Stepping, resetting, or picking disabled deformables avoids
  missing simulation objects, and queries no longer count disabled bodies that
  have no simulation actor.

- **Live joint-limit edits apply consistently.** Changing a fixed joint to a
  revolute joint no longer leaves a zero-width limit when limit columns are
  absent. One-sided limits remain effective, and bounds clamp consistently with
  joint creation. Updating both bounds no longer depends on delivery order or
  applies a later ordinal early; clones and rebuilds preserve delivered bounds.
  Supply the new type's required columns and APIs when changing joint type.
  Rebuild the joint when switching an axis between free and limited motion.

- **Legacy joint velocity limits are honored without the removed schema.**
  Authored `physxJoint:maxJointVelocity` values work even without `PhysxJointAPI`.
  Finite per-axis limits take precedence over finite `newton:velocityLimit`,
  which takes precedence over the legacy value. Migrate to `NewtonJointAPI`;
  changing the legacy attribute requires reattachment.

- **Disabled joints no longer warn that misaligned frames will snap bodies
  together.** The warning is checked against current frames when the joint is
  enabled.

- **Tendon access remains valid after removing its articulation.** Deleting an
  articulation's base link while tendon prims remain no longer leaves tendon
  query, read, write, or discovery operations accessing released objects.
  Released tendons return no rows.

- **Contact reads track sensor removal and enable-state changes.** Destroying a
  sensor invalidates its contact views, and disabling then re-enabling a GPU
  sensor no longer reports another body's contacts. Reads also avoid invalid
  accesses with unequal filter counts, newly enabled bodies, or added
  articulations after earlier GPU contact reads. Unavailable articulation
  sensors are diagnosed and read as zero.

- **New read views resolve replacement simulation objects.** Replacing bodies or
  articulations without changing their count, or inserting a static object, no
  longer leaves newly created views using stale objects or paths.

- **Retried scene updates preserve object additions and removals.** If applying
  ovstage changes fails, retries no longer lose pending deletions or leave
  behind objects added before the failure and removed before retry. Objects
  recreated at the same path remain live.

- **Invalid vehicle drive targets are diagnosed.** An unresolved drive target
  now reports both the vehicle and the target path so callers can correct the
  authored relationship.

- **A busy commit reports that no values were published.** The failure result no
  longer reports an unknown outcome for this failure.

- **GPU collision pairs remain correct after large resets or teleports.** Moving
  many objects at once no longer misses new collision pairs or retains stale
  interactions that increase contact work and GPU memory use.

- **Convex bodies contact mesh borders and heightfields correctly.** Affected
  boxes and convex shapes no longer get thrown away from the surface on GPU or
  pushed sideways and stopped by incorrect CPU contacts.

- **DirectGPU articulation access survives removal of the largest
  articulation.** Remaining articulations no longer return incorrect state or
  receive writes at the wrong locations. After adding a larger articulation,
  step before reading it and recreate stale simulation views as required.

- **Failed asynchronous steps preserve force scaling in contact bindings.** A
  failed step no longer replaces the duration of the last successful positive
  step used to convert contact impulses into forces.

- **Character-controller orientations match their authored frame.** Orientation
  and target-orientation reads no longer include an unwanted 90-degree capsule
  rotation. Replacing a controller also invalidates the old controller instead
  of keeping its state live.

- **Reads can recover from a transient GPU allocation failure.** A failed
  allocation no longer leaves later reads permanently reusing invalid state.

- **Configuration supplied before creating an instance takes effect.** Legacy
  configuration entries are retained until initialization. Set startup-only
  tasking options before creating the first instance. `NUM_THREADS` must be set
  before attachment; zero runs PhysX simulation tasks on the caller's thread.

- **Detaching during mesh cooking no longer crashes or hangs.** An in-flight
  mesh cook can be canceled safely when its stage is detached.

- **Python handle validation rejects NumPy Boolean values.** NumPy Booleans and
  objects that only implement integer indexing no longer pass as handles. Supply
  an accepted integer handle or the documented OVStage handle wrapper.

- **Population preserves more authored physics inputs.** Native USD scene-graph
  instances retain their colliders and resolved materials with physics-only
  population. Material binding strength and purpose, cylinder and cone convex
  margins, and affected collision, force, and default inputs are preserved.

- **Packaged documentation includes usable images and clearer loading
  guidance.** Shipped images no longer contain Git LFS pointer text. The loading
  and population guides explain schema registration, sealing, transform updates,
  and native-instance constraints, with contextual glossary links and complete
  workflow examples. Guidance also clarifies that joint velocity limits are
  solver constraints rather than exact clamps, and documents native PhysX
  helper-linking limitations. Removing an attribute while keeping its prim
  leaves the value used by physics undefined; write the intended value instead.

- **OmniPVD recordings retain current motion and contacts.** Particle systems
  created before recording starts now record motion every frame. CUDA-ordered
  writes record current values, DirectGPU joint forces persist until changed,
  and multiple continuous collision detection passes no longer drop unrelated
  contacts.

### Breaking Changes

- **PhysX now simulates every authored physics scene.** Scenes with Newton or
  other simulator APIs no longer require `PhysxSceneAPI` to include their bodies
  in simulation. The default-simulator setting no longer excludes scenes.
  Review scenes previously excluded this way and use `physics:simulationOwner`
  to assign bodies explicitly; objects without an owner join the first created
  scene.

- **Task-returning operations now execute asynchronously by default.**
  Successful submission means that work was accepted, not that it completed.
  Retain caller-owned storage for the documented operation lifetime and collect
  outcomes with `wait_task()` or `wait_all()` before consuming results or
  releasing that storage. Set `GlobalConfigKey.ASYNC_EXECUTION` to false before
  creating an instance to disable asynchronous execution. Queries, reads, and
  writes can queue behind earlier work. Recording transitions and environments
  without worker resources can execute tasks on the caller's thread. Refer to
  [task execution and ownership](developer_guide.md#execution-model).

- **C read and articulation-refresh calls accept CUDA synchronization.**
  `ovphysx_read()` adds an `ovstage_cuda_sync_t` argument before `out_read`;
  `ovphysx_update_articulations_kinematic()` adds it after `flags`. Rebuild and
  update callers; zero initialization preserves ordinary default-stream use.
  Call `warmup()` or step before the first DirectGPU kinematic refresh, which no
  longer warms the scene automatically. Before filling reused C write storage,
  order the producer against `data.cuda_sync.wait_event`; Python arranges this
  wait. Python reads accept `cuda_stream`, `cuda_wait_event`, and `event_owner`
  for explicit producer ordering and event lifetime.

- **Explicitly closing a Python read invalidates its borrowed arrays.**
  `ReadResult.close()`, `release()`, and context-manager exit end array validity
  even if Python references to those arrays remain. Copy data that must survive
  an explicit close. Merely dropping the result without closing it preserves the
  existing borrowed-array lifetime behavior.

- **OVStage 0.4 changes token types and sentinel values.** Replace
  `ovx_string_or_token_t` with `ovx_token_or_string_t` in C clients.
  `OVX_INVALID_TOKEN` is now `UINT64_MAX`; token zero represents the empty
  string. Use the named sentinel instead of assuming zero is invalid. A non-null
  string takes precedence when both a string and a token are supplied. Python
  attribute-token inputs reject both zero and `2**64 - 1`.

- **OVStage handles use typed structs and objects.** Native clients use
  `.raw_id` or the handle comparison and validity helpers. Python attribute
  results are `ovstage.Token` objects and prim lists use `ovstage.PrimPathList`;
  read a handle's `.raw_id` instead of calling `int(handle)`.

- **OVStage writes enforce an attribute's semantic.** An explicitly different
  semantic is rejected under `REJECT`, including adding a role to a roleless
  attribute. Use a compatible semantic, or `FORCE` when intentionally replacing
  the selected attribute's schema.

- **Native read groups contain an explicit column.** Move `attribute`, `data`,
  `is_array`, `semantic`, `meta`, and `ordinal` access to `group->columns[0]`.
  `prims` and `read_group_id` remain on the group; release it with
  `ovphysx_release_group()` as before. OVPhysX Python `ReadGroup` fields remain
  flattened; code reading OVStage groups directly must iterate `group.columns`.

- **Physics population uses explicit descriptions.** Replace the `PHYSICS` and
  `ALL` population modes with the `*_with_desc` calls and
  `ovphysx.population.physics_population_descs()`. Add `rendering=True` when
  sharing the stage with rendering or loading PointInstancer prototype geometry;
  native USD scene-graph instances support physics-only population. C callers
  use `<ovphysx/ovphysx_population.h>` and
  `ovphysx_get_physics_population_desc()`. Supply the required `time` argument
  to the population call, for example `time=ovstage.population.DEFAULT_TIME` in
  Python. Remove the old physics-only workaround that deletes
  `omni:fabric:worldMatrix` before sealing; sealing already composes world
  matrices, and vehicle-wheel output needs the column.
  See [OVStage integration](ovstage_integration.md).

- **OVStage ordinals are signed 64-bit values.** Use `ovstage_ordinal_t` and the
  named ordinal bounds instead of unsigned sentinels. OVPhysX attachment still
  requires an ordinal from 1 through `OVSTAGE_ORDINAL_MAX`; zero is reserved.
  Seal the initial population before attaching.

- **Articulation-joint state and property rows represent degrees of freedom.**
  Each group has one tensor row per unlocked degree of freedom, with repeated
  joint paths and a mask element per row. Prim selection still names joints;
  `physxPointer` remains one row per joint. Update tensor indexing and masks.
  For OVStage write-back, use `ovphysx.utils.split_joint_group` or the C
  joint-array helpers described in [joint reads](read_write/readable.md) and
  [joint writes](read_write/writable.md).

- **Inverse-dynamics reads expose dimensions as tensor axes.** Jacobians use
  `[objects, rows, columns]`, mass matrices `[objects, M, M]`, and force vectors
  `[objects, M]`, each with one dtype lane. Centroidal momentum uses `[objects,
  6, numDofs + 7]` for floating-base articulations. Stop requesting the removed
  `jacobianShape` attribute and read dimensions from each tensor's `shape`. See
  the [articulation reference](read_write/attributes/articulation.md).

- **Native recording start accepts an options argument.** Rebuild C clients and
  pass a third argument to `ovphysx_ovd_start_recording()`; `NULL` preserves
  unfiltered recording. Initialize non-null options with
  `OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT` before selecting objects or data.
  Python accepts `filter=` and `wait_for_selection=`.

- **DirectGPU recordings store joint values by axis.** Recorded `jointPosition`,
  `jointVelocity`, `jointForce`, `driveTarget`, and `driveVelocity` now have six
  axis slots, with zeros for locked axes. Update recording readers that assumed
  one value per unlocked degree of freedom. This concerns OmniPVD recordings;
  simulation read/write tensors use the layouts described above.

- **Execution calls reject callback reentry.** Calls that step, wait, change
  stage attachment, access simulation data, or start/stop recording return
  `OVPHYSX_API_INVALID_STATE` from runtime, object-change, and application log
  callbacks. Defer execution until the callback returns.

### Performance

- **Scenes using GPU broadphase use less CPU time to process large sets of
  pairs.** Overall simulation-step time may be unchanged.

- **Reusing queries reduces repeated read and write overhead.** Uncaptured reads
  and writes over a held query reuse preparation work. Selected joint reads
  return contiguous tensors and fetch only the relevant articulations where the
  requested attribute supports this.

- **Initial attachment avoids repeated work in mesh-heavy stages.** Stages with
  many meshes, filtered collision pairs, or visual PointInstancers can attach
  faster, including when the compiled stage cache is disabled.

- **GPU contact reads avoid excessive worker wakeups.** Contact-read scheduling
  reduces overhead in sustained workloads; the benefit depends on the workload.

- **Repeated scene edits avoid growing update overhead.** Small object additions
  and changing subsets of velocity updates avoid repeatedly processing all
  existing bodies. Benefits depend on scene size and the edited subset.

- **Linux CPU simulation recovers performance with the schedutil governor.** The
  bundled runtime dependency corrects a regression affecting this CPU frequency
  policy.
