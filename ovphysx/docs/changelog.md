<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Changelog

All notable changes to `ovphysx` are documented in this file.

## [0.6.3] - Date 2026-09-10

### Added
- **`ovphysx.utils` is now a USD authoring package.** It was a single module
  holding `step_and_write_to_ovstage()`; it is now a package of USD authoring
  helpers for shapes, meshes, joints, materials, planes, particles, deformables,
  collision filtering, transforms, paths and codeless schema access. Names stay
  flat, so `ovphysx.utils.add_rigid_box` and
  `ovphysx.utils.shapes.add_rigid_box` are the same function, and
  `step_and_write_to_ovstage` is still importable from `ovphysx.utils`. The
  package needs only `pxr` and the standard library and loads no native
  library. One deliberate behavior change: the six `add_rigid_*` helpers now
  always author a rigid body, including at `density=0.0`; use the matching
  `add_collider_*` helper for a static collider.
- **`ovphysx_get_object_type()` classifies standalone and custom joints.** A
  maximal-coordinate joint between plain rigid bodies, and a plugin-registered
  custom joint, simulated correctly but reported
  `OVPHYSX_OBJECT_TYPE_INVALID` -- the same value as a path with no object at
  all -- which made the call useless for identity or existence checks. They now
  report the new `OVPHYSX_OBJECT_TYPE_JOINT` / `ObjectType.JOINT` (6) and
  `OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT` / `ObjectType.CUSTOM_JOINT` (7).
  Articulation joints keep reporting `ARTICULATION_JOINT` (5). `INVALID` now
  means only that no classified simulation object lives at the path.
- **A physics population contract, with generated builders and per-component
  documentation.** The contract records, for every USD prim type and applied
  API schema that physics reads, the exact ovstage encoding of each column, its
  raw USD fallback, what population writes, what the parser assumes when the
  column is absent, and how the column shows up in the PhysX objects the
  runtime creates. It covers 19 prim types, 31 API schemas and 292 columns. A
  Warp-based `ovphysx.population` Python module and a header-only C++17 builder
  (`ovphysx/population/Population.hpp`) are generated from it, so an
  application can build a populated stage without hand-encoding column names
  and types. One documentation page per component ships under Physics Schemas.
  No runtime behavior changes.
- **Read and write data-contract documentation.** New pages describe the
  `ovphysx_read()` / `ovphysx_write()` data model, the readable and writable
  attribute sets, device placement, and the known limitations. The ovstage
  integration guide now links to them instead of carrying its own partial copy.
- **`OvStageOutputCache` reduces the cost of publishing output every frame.**
  `step_and_write_to_ovstage()` reads the current transform and
  point-instancer values on every call by default. Passing a cache
  (`with OvStageOutputCache(physx) as cache:`, then
  `step_and_write_to_ovstage(..., cache=cache)`) lets it own copies and reuse
  its CPU and CUDA output buffers across calls. Call `cache.refresh()` after a
  transform, point-instancer pose-array or topology change; the cache is bound
  to one ovstage attachment and cannot survive a detach and reattach.

### Changed
- **`step_and_write_to_ovstage()` publishes world transforms, not shadow pose
  attributes.** Poses used to land on `sim:<name>` shadow attributes that scene
  consumers did not read. For fixed rigid bodies, articulation links and
  vehicle wheels the helper now reads the current `omni:fabric:worldMatrix`,
  keeps its shear-free signed scale, and writes back a reconstructed world
  matrix; PhysX pose output itself carries no scale, so scale stays ovstage
  state. It never writes `omni:xform` and never changes
  `omni:resetXformStack`. Rigid-body point instancers get their native
  `positions` and `orientations` arrays written instead, preserving unsimulated
  slots and authored trailing rows; `scales` and prototype indices are
  untouched. Every other emitted attribute still goes to its `sim:<name>`
  shadow attribute. The sampled world matrix must already be current, so
  compute the hierarchy and advance its write floor before calling the helper.
  This helper does not make its output the prim's new local transform, and does
  not propagate to descendants.
- **`ovphysx_attach_ovstage()` refuses a stage populated without the PhysX USD
  schemas.** ovstage drops applied API schemas it cannot resolve, so a stage
  populated before
  `ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])`
  silently lost every `Physx*` setting -- self-collision flags, joint velocity
  limits and the rest. Simulation then diverged on configurations that depend
  on them, which could crash the process inside the first step. Attach now
  verifies the registration and fails with an error naming the missing call,
  before the runtime is touched. The refusal is sticky for the process:
  registering late fixes the plugin list but not USD's already-built schema
  registry. Your application still owns the registration; ovphysx only checks
  it. Flows that already register are unchanged.
- **The wheel and SDK no longer contain OmniClient.** ovphysx stopped copying
  OmniClient, the `omniverse_connection` library and their provenance file into
  its payloads, and no longer preloads OmniClient or rejects a mismatched
  version at `PhysX()` construction. OmniClient, the USD resolver and the USD
  runtime belong to the ovstage distribution your application supplies. Local
  USD population, attach and simulation are unaffected in either startup order.
- **The retired Fabric GPU plugins are gone from the package.** The unused
  `omni.cubric.plugin` and `omni.gpucompute-cuda.plugin` dependencies and the
  `plugins/gpu` search path are removed. PhysX GPU simulation is unchanged and
  still runs from `PhysXGpu_64` in the flat `plugins/` directory.
- **The bundled PhysX SDK moves to a build from the 0.6 release line.** It
  picks up solver batching fixes, better load balancing in the CPU-only
  articulation code, and a bounds check on the contact-manager index during
  narrowphase refresh that could otherwise fault.
- **A path pattern component longer than 4096 characters is now rejected.**
  Every entry point that takes a path pattern -- tensor binding `pattern` and
  `prim_paths`, SDF view `pattern`, contact binding `sensor_patterns` and
  `filter_patterns` -- returns `OVPHYSX_API_INVALID_ARGUMENT` (`RuntimeError`
  in Python) instead of accepting the input. Components are delimited the way
  the runtime tokenizes them, at `/` outside balanced parentheses. Real prim
  names and alternation lists stay far below the bound; a caller with a very
  long explicit list should pass literal paths.
- **Source builds fetch the pinned ovstage wheel from public PyPI.** The pinned
  version does not change, and the wheels are the same builds that were served
  internally. This affects only how a source build resolves its dependency.
- **A source build needs CMake 3.22 or newer.** The README, the local
  development guide and the source-link tutorial said 3.16, but the runtime the
  build pulls in has required 3.22 for some time. The `find_package(ovphysx)`
  flows against an installed SDK are unaffected and still work at 3.16.

### Performance
- **An incremental ovstage spawn costs a handful of round trips instead of
  hundreds.** Creating and draining one rigid body used to rebuild the whole
  scan context from cold on every drain -- whole-stage schema probes, a
  per-family stage query, a live ancestor walk for material bindings, and a
  re-intern of every well-known token -- so the cost was fixed per drain rather
  than proportional to what changed. The incremental scan now runs on the warm
  attached source, answers family and column probes from one cached stage
  vocabulary, prefetches bodies, shapes and ancestors once per spawn and reuses
  that window for the mass update and object creation. Simulation results are
  unchanged.
- **Repeated ovstage reads and writes no longer evict each other's row
  uploads.** The rigid view's ovstage row-list device cache held one slot, so a
  read/write loop whose row lists differ re-uploaded on every access. It is now
  a small LRU, and a stable read keeps its upload while a changing list cycles
  through the spare slots.
- **Attaching a scene with render-only scene-graph instances no longer runs
  reverse instancing queries during cooking.** The lookup is needed only
  outside the cooking pass; keeping it out removes a query per prototype from
  attach on scenes whose instances carry no physics.

### Fixed
- **Instance-proxy colliders keep their authored collision settings.** ovstage
  instance-proxy rows carry the logical collider path, but some collision
  values live only on the prototype's backing row. The runtime read the logical
  row alone, so an explicitly disabled collider came back enabled from the
  default value and authored contact and rest offsets were lost -- self
  collision then made colliders interact that were meant to be off. Collision
  enablement, the scalar `PhysxCollisionAPI` values and the contact margin and
  gap values now fall back to the nearest instance-root prototype backing when
  the logical row has no value. Logical identity, transforms, material
  bindings, relationships, instance overrides and value blocks stay
  authoritative. This applies at initial population and attach; it does not add
  live fan-out of prototype edits.
- **A very long path-pattern component no longer kills the process.** On Linux,
  a single `/`-separated component longer than roughly 58K characters
  overflowed the thread stack inside the regular-expression compiler and the
  process died with no exception and no log line. The matcher now bounds the
  token it compiles and warns instead, and the public entry points reject the
  input outright (see *Changed*). Windows was never affected.
- **A read taken right after a write no longer returns another scene's prim
  paths.** The rigid read cache is keyed by the scene pointer and shared with
  the write path. A write refreshed the entry's keys but left its resolved path
  handles in place, so when a new scene reused a destroyed scene's address with
  the same body count, the following read served the old scene's paths. A write
  that changes the key set now drops the stale handles; a steady-state write
  over an unchanged key set keeps its warm ones.
- **The public source drop configures with the documented CMake version.**
  `CMakePresets.json` declared preset schema version 6, which only CMake 3.25
  and newer parse, so the documented preset flow failed on stock CMake 3.22
  with an `Unrecognized "version" field` error that named neither CMake nor a
  version. The file uses nothing newer than schema 3 and now declares 3.
- **The public source drop no longer advertises `-n` / `--no-docker`.** Nothing
  in the drop read the variable behind that flag, so passing it did nothing and
  omitting it started no container. A developer who trusted the help text built
  against the host's glibc and was then rejected by the ABI check. The flag and
  the Docker claim are gone from the public tree, and the README states plainly
  that meeting the glibc 2.35 baseline is the build environment's job.
- **The public source drop ships `python/uv.lock`.** Its own documented entry
  point, `cmake -P scripts/validate_all.cmake`, needs that lock for the frozen
  `uv sync` its type-check step runs; without it the step silently skipped, so
  the stub tree was never type-checked outside this repository.

## [0.6.2] - Date 2026-09-05

> **`ovphysx_read_raw_contact_data()` changes its tensor count from seven to
> six.** The four per-contact value tensors (force, point, normal, separation)
> are unchanged, but the `counts` / `start_indices` / `sensor_actor_ids` /
> `other_actor_ids` tensors are replaced by two paired tensors: `sensor_layout`
> (`[S, 2]`: contact count, start index) and `actor_ids` (`[C, 2]`: reporting
> sensor's actor, other actor). Update call sites to the new signature; there is
> no versioned alternate symbol.

### Added
- **Auto deformable attachments now work without a live USD stage.** A
  `PhysxAutoDeformableAttachmentAPI` prim produced nothing on an ovstage attach
  with no backing USD stage, because the runtime only generated its attachment
  sub prims by authoring them into USD. The runtime now builds and keeps that
  set in memory when it cannot author, so the attachment simulates on both the
  USD and USD-free arms.

### Changed
- **The Python test run resolves USD from stock pip `usd-core`.** Source builds no
  longer fetch an internal USD package for the Python tests; they install stock
  `usd-core` from PyPI instead, the only supported source. ovphysx does not test
  OmniClient or remote-USD loading (the application owns USD and its resolver). On
  linux aarch64 (no PyPI `usd-core` wheel) the two mass-unit doc-contract checks
  skip. This changes only the repo's own test/dev flow; the shipped SDK and wheel
  carry no USD either way.
- **The pinned ovstage runtime moves to `0.2.0.377349`.**
- **ovphysx, including the bundled PhysX SDK, is now licensed under the Apache
  License 2.0**, replacing BSD-3-Clause. There is no source, ABI, or behavior
  change; the updated `LICENSE.txt` ships in the wheel and SDK package.
- **`ovphysx_read_raw_contact_data()` reports which actor pair produced each
  contact, in six tensors instead of seven.** See the breaking-change note
  above for the new shape. Resolve an id with `get_other_actor_paths_from_ids()`;
  a removed actor now resolves to an empty path instead of a stale one.

### Performance
- **ovstage change-feed drains scale with what changed, not with stage or scene
  size.** Reading and applying value, velocity, and transform updates from
  ovstage after a step, and the per-spawn/despawn cost as a scene grows, both
  used to carry fixed per-drain costs that grew with scene size. Measured
  scenarios show drain cost dropping roughly 3-5x, with the worst-case
  teleport and velocity-update paths dropping by more than an order of
  magnitude; population growth and shrink are also faster. No API or behavior
  change.

### Fixed
- **A GPU write to ovstage no longer intermittently poisons the CUDA context.**
  Writing rigid-body pose, velocity, or wrench data through DirectGPU could
  race an internal index build against PhysX's read of that index, which
  occasionally produced an illegal memory access that aborted the CUDA context
  for the whole process. The write path now waits for the index build to
  finish before PhysX reads it.
- **A read of a GPU-resident tensor binding no longer faults when
  `active_cuda_gpus` selects a non-default device.** The read staged through a
  buffer allocated and freed on whatever CUDA context happened to be current on
  the calling thread, which could be the wrong device once a non-zero ordinal
  was selected. Reads and writes now stage in the binding's own CUDA context.
- **Disabling one rigid body on DirectGPU no longer blinds ovstage reads and
  writes for the whole scene.** The shared read view used to be invalidated
  whenever any body was disabled, taking every other body's ovstage I/O down
  with it. Disabled bodies are now tracked without dropping the shared view,
  and `disableSimulation = 0` still re-enables normally.
- **`update_from_ovstage()` now applies rigid body, vehicle wheel, and tendon
  changes through the same write path as `ovphysx_write()`.** The previous
  per-object update loop was inert for several of these properties on a
  DirectGPU scene, so values written to ovstage did not reach physics. The
  change also turns a drain into one vectorized publish instead of one call per
  object.
- **A cooking crash from a released CUDA context is fixed.** Cooking on the GPU
  could crash with a pure-virtual call if the host released and recreated its
  CUDA context while a cook was still using the old one. Cooking now holds its
  own reference to the context for as long as it needs it.
- **A process that never calls `PhysX.destroy()` no longer leaves
  cooked-collider cache directories behind.** Exiting the interpreter without
  an explicit `destroy()` skipped the shutdown path that removes the
  process-private cache from the temp directory. A Python exit handler now
  runs that cleanup on normal exit and most interrupted exits; abrupt
  termination such as `SIGKILL` remains out of scope.
- **Rigid-body and articulation tensor views no longer warn about their own
  matches.** A path pattern that also matches a same-named object of a
  different type still produces one aggregate "no match" diagnostic when
  nothing valid is found; it no longer additionally warns once per wrong-type
  candidate along the way. Valid matches are unchanged.
- **The `clone.py`, `tensor_bindings.py`, and `omnipvd_recording.py` tutorial
  snippets now find their bundled USD scenes when copied into another
  project.** They resolved scenes relative to the running script's own
  directory, which only worked inside the installed samples tree. They now
  resolve relative to the installed `ovphysx` package.
- **The cloning tutorial and samples now document and demonstrate CPU clone
  collision isolation.** `PhysX.clone()` / `ovphysx_clone()` isolate cloned
  environments from each other's collisions only under GPU dynamics with GPU
  broadphase; on CPU, co-located clones share one collision space and can
  shove each other apart. This is now documented at the API and in the
  tutorial, and the shipped samples space clones apart instead of stacking
  them.
- **Docs and the `tensor-bindings-gpu` skill no longer claim GPU dynamics needs
  authoring.** `physxScene:enableGPUDynamics` defaults to `true` in the PhysX
  schema, so a scene that never authors it already runs GPU dynamics; two docs
  said otherwise.

## [0.6.1] - Date 2026-09-03

### Added
- **Writability is queryable.** `ovphysx_writability(object_type, attribute, &out)`
  reports whether `ovphysx_write()` accepts a pair, as `WRITABLE`, `CONDITIONAL`,
  `WRITE_ONLY`, `READ_ONLY` or `UNCLASSIFIED`. `CONDITIONAL` means writable only under a
  condition: `jointLimit` is refused on a free axis. It needs no instance, scene or
  step. An `object_type` outside `ovphysx_sim_object_type_t` returns
  `INVALID_ARGUMENT`, not `UNCLASSIFIED`.
- **`ovphysx_get_tensor_binding_native_device()` reports a binding's device.** A binding
  exposed its dtype and shape but not whether its storage is on the CPU or a CUDA
  device, so callers had to reproduce ovphysx's placement rules to allocate a tensor.
  Experimental C++ and Python wrappers are included. The tensor-binding API is
  deprecated (see *Deprecated*); prefer `ovphysx_read()` and `ovphysx_write()`.
- **Five hidden benchmark rows.** The opt-in C++ suite gains
  `WriteScalingHighN.velocity_{ovstage,tensor}_{8192,16384}_cpu` and
  `RuntimeSpawnScaling.collider_heavy_1280_cpu`. They run only when selected explicitly
  with `--hidden`, and are diagnostics with no pass/fail threshold.

### Performance
- **Attaching an ovstage scene is now faster than loading the same scene from USD.**
  On a 132k-joint tracked-vehicle scene attach went from 68.6 s to about 7 s, and on a
  512-environment instanced robot scene from 48.6 s to 4.3 s. USD load is not regressed
  and the resulting physics is unchanged. One upgrade cost: a multi-material mesh cooked
  as convex, decomposition or sphere fill gets a new cooking key, so it cooks once more
  on first use.
- **Attach cost no longer scales with scene content that physics does not use.** Attach
  scanned the whole stage even when few prims carried physics. On a benchmark holding
  five rigid bodies inside a large render-only scene it went from about 990 ms to about
  220 ms. Scenes that are all physics are unaffected.
- **The ovstage GPU output read reuses its device buffers.** On the DirectGPU path each
  read allocated and freed its device and staging buffers, which dominated the per-read
  cost of a loop that reads the same columns every step. Buffers are now pooled and
  reused. `PhysXConfig(ovstage_read_pool_max_mb=...)` /
  `ovphysx_config_entry_ovstage_read_pool_max_mb()` (carbonite
  `/physics/ovstageReadPoolMaxMB`) caps what is retained; default 256 MiB, `0` or
  negative disables pooling. It bounds retained memory, not a single read's peak.

### Changed
- **ovphysx has exactly one build, and it is USD-free.** The former USD-linked variant
  and its `OVPHYSX_NO_USD` option are gone, so the shipped configuration is now the one
  the unit tests exercise. Public APIs and physics behavior are unchanged, with one
  exception: the experimental Mineways voxel map (`InfiniteVoxelMapAPI`) is no longer
  supported. Prims applying it are ignored with a warning, and `IPhysx::setVoxelRange`
  returns `false`.
- **Every Python write tensor is a `warp.array`.** Host columns used to come back as
  NumPy views; now every group tensor is a Warp array on its native CPU or CUDA device.
  A non-empty tensor is still a mutable alias of runtime storage -- fill it in place,
  then commit -- while an empty one is a Warp-owned empty array. Each write tensor also
  reports its own residency, which can differ from the matching read when the write
  stages through the host on a GPU scene. The C ABI and device placement are unchanged.
- **Your application must register the PhysX USD schemas itself.** The pinned ovstage
  runtime no longer bundles or self-registers them. Call
  `ovstage_population_register_usd_schemas()` (Python:
  `ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])`)
  before the first population call, or the scene parses as if no physics schema had been
  applied. Scenes in non-default units keep their scaling either way.
- **The pinned ovstage runtime moves to `0.2.0.375783`.** ovphysx binds to one ovstage
  release's ABI; the wheel installs that exact version.
- **SDK packaging rejects a mismatched USD build.** The USD build ovphysx was linked
  against is recorded in the installed `config.toml` as `build_package`, and packaging
  now fails if the OVStage runtime supplies a different USD kit build. Two kit builds of
  one USD version share a library name, so the old filename check could not tell them
  apart.
- **CPU-only tensor bindings require host buffers.** The documented contract was not
  explicit: these bindings need `kDLCPU` or `kDLCUDAHost` source, destination, index and
  mask buffers even on a GPU simulation, and reject `kDLCUDA` and `kDLCUDAManaged`
  instead of staging them to the host. Documentation only; behavior is unchanged.

### Fixed
- **A `find_package(ovphysx)` consumer no longer fails to start with `libovstage.so:
  cannot open shared object file`.** On Linux the installed target emitted its rpath as
  `DT_RUNPATH`, which the loader does not apply to a dependency's own dependencies. It
  now links with `--disable-new-dtags`, fixing every consumer, not only the samples.
- **Linear BasisCurves work as surface-velocity curves.**
  `PhysxSplinesSurfaceVelocityAPI` ignored the BasisCurves `type` and treated every
  curve as cubic, so a `type = linear` conveyor path of three points was rejected for
  too few control points and longer polylines were smoothed off the authored path.
  Linear curves now follow the polyline exactly. Two-point curves of any basis are fixed
  too. A rejected curve names the cause -- no target, target not in the attached stage,
  or target not a BasisCurves -- instead of one generic message.
- **A vehicle is no longer created without its brakes.** Under the ovstage backend a
  sub-schema applied through an unqualified multi-apply name went undetected.
  `OmniPhysicsDeformablePoseAPI` was missed the same way and now applies.
- **A tendon whose attributes are all left at their defaults is no longer dropped.** The
  scan looked for authored attributes instead of the applied schema. Vehicle context,
  friction tables and shared components were skipped by the same kind of check.

### Deprecated
- **The tensor-binding API is deprecated in favor of read and write sessions.** The
  seven C entry points, the C++ `ovphysx::TensorBinding` and `createTensorBinding`, and
  the Python `TensorBinding` and `create_tensor_binding()` are marked deprecated, and
  the Python ones raise `DeprecationWarning`. Use `ovphysx_read()` / `PhysX.read()` and
  `ovphysx_write()` / `PhysX.write()`. Nothing is removed and behavior is unchanged;
  removal is not scheduled for 0.6.

### Removed
- **The `OVPHYSX_NO_USD` build option and the USD-linked variant.** There is one ovphysx
  build and it links no USD. Consumers that attached a native `UsdStage` must attach
  through ovstage instead.

## [0.6.0] - Date 2026-08-31

> **C ABI break.** `ovphysx_debug_render_set_parameter()` changes its third
> parameter from `bool` to `float`. A binary built against an earlier 0.5 header
> must be recompiled and relinked; source callers should pass `1.0f` for enabled
> and `0.0f` for disabled. This is independent of the `ovphysx_clone` ABI change
> introduced in 0.5.4.
>
> **Python source breaks.** `PhysX.clone()` renames the `parent_transforms`
> keyword to `anchor_transforms`. Positional callers, the C ABI, and runtime
> placement behavior are unchanged. The former `PhysX.release()` method and
> main-object context-manager protocol are removed. Replace `physx.release()`
> with `physx.destroy()`, and replace `with PhysX()` with explicit construction
> plus `destroy()` in a `finally` block.
>
> **C ABI break.** `ovphysx_contact_event_header_t.stageId` (`int64_t`) is
> renamed and retyped to `.attachHandle` (`uint64_t`). A
> stageless attach used to report `stageId == 0` indistinguishably from "no
> attach"; `attachHandle` is nonzero for every live attach, stageless or not,
> and never repeats across a detach/reattach pair. A binary built against an
> earlier header reading `.stageId` must be recompiled and relinked — the
> field keeps the same offset and the same 8-byte width, so a stale consumer
> silently reinterprets a live attach handle as a USD stage id instead of
> reading a mismatched-width neighbor field or plainly zero. Added
> `ovphysx_get_attach_handle()` / `PhysX.get_attach_handle()` to read an
> instance's current attach handle directly, without waiting for a contact
> event to report one.
>
> **C++ source break.** Many public runtime entry points move from
> `uint64_t stageId` to `AttachHandle`, and because
> `AttachHandle` is itself a `uint64_t` alias, a stale out-of-repo source
> caller can compile unchanged and get *wrong runtime behavior* rather than a
> build failure. Affected: `IPhysxSimulation::addForceAtPos`, `addTorque`,
> `wakeUp`, `putToSleep`, `isSleeping`, `subscribePhysicsTriggerReportEvents`,
> and their `*Instanced` counterparts; `IPhysxReplicator::registerReplicator`,
> `unregisterReplicator`, `replicate`, `isReplicatorStage`, and the
> `ReplicationAttachFn`/`ReplicationAttachEndFn` callback typedefs;
> `IPhysxCookingService`'s `PhysxCookingComputeRequest::attachHandle` field and
> its completion callback; and `TriggerEvent.h`'s `TriggerEventData::stageId`,
> renamed to `::attachHandle` in parallel with the ABI rename documented
> above. Pass `kActiveAttach` where a caller used to pass stage id 0 meaning
> "the current attach" — it resolves late, at the point of use, so a
> pre-attach subscription still works, but it only resolves while exactly one
> attach is live. Obtain a concrete handle for a specific attach from
> `IPhysxSimulation::getAttachHandle()`. A stale or otherwise unresolvable
> handle resolves to no attach rather than aliasing a different live one.
> Diagnostics on that path are not uniform across this surface — for example
> `registerReplicator()` emits `CARB_LOG_ERROR` for a stale or non-live handle,
> while other entry points here resolve silently — so a caller must not rely
> on a log appearing and should check return values/results to diagnose
> staleness itself.
> `subscribePhysicsTriggerReportEvents` no longer accepts `kNoAttach` (0) as
> an "all attaches" wildcard the way the old stage-id spelling did; `kNoAttach`
> is now rejected outright (returns `kInvalidSubscriptionId`). A consumer that
> wants every attach must subscribe once per attach, using each attach's own
> handle or `kActiveAttach`.
>
> **C++ source/binary break.** The runtime's public `IPhysx` interface no
> longer names an object by USD path. `ObjectCreationNotificationFn`,
> `ObjectDestructionNotificationFn`, `getObjectId()`, and `getPhysXPtr()` take
> `omni::physics::parse::ObjectKey` where they previously took an `SdfPath`;
> the two types are not layout-compatible, so a stale binary fails to compile
> or link rather than silently misinterpreting the parameter. This is a hard
> break with no compatibility shim — out-of-repo consumers of these entry
> points must rebuild against the new signatures. Resolve a path string to an
> `ObjectKey` (and back) only through the two boundary functions designated
> for that; no other public function takes or returns a path.
>
> **C ABI break.** `ovphysx_scene_query_hit_t`'s `collision`, `rigid_body`,
> and `material` fields (part of the same object-identity migration as the
> `IPhysx` break above) no longer hold a uint64-encoded `SdfPath`; they hold an opaque
> `omni::physics::parse::ObjectKey.handle` assigned by the runtime. The two
> encodings are not comparable — a consumer that reproduced ovphysx's old
> SdfPath bit-cast to match hit fields against known prim paths will silently
> compare against the wrong values instead of failing loudly. Struct layout,
> size, and field order are unchanged, so this is a behavioral break only, not
> an ABI-layout break — no recompile is required, but any comparison logic
> against the old encoding now silently misbehaves and must be removed. Use
> the new `ovphysx_scene_query_get_paths_from_ids()` (see *Added* below) to
> resolve these fields to a path instead.
>
> **C++ source/binary break.** `IPhysxSceneQuery.h`'s scene-query surface
> (the C++ counterpart of the `ovphysx_scene_query_hit_t` break above, part
> of the same object-identity migration) no longer names a shape, body, or material
> by USD path. `SphereShapeReportFn`, `BoxShapeReportFn`,
> `CapsuleShapeReportFn`, `ConeShapeReportFn`, `CylinderShapeReportFn`,
> `ConvexMeshShapeReportFn`, and `TriangleMeshShapeReportFn`'s leading
> identity parameter now take `omni::physics::parse::ObjectKey` in place of
> a `uint64_t`-encoded `SdfPath`; `SceneQueryHitObject::collision`/
> `rigidBody` and `SceneQueryHitLocation::material` retype the same way.
> `overlapMesh`, `overlapMeshAny`, `reportCollisionShapes`, `overlapShape`,
> `overlapShapeAny`, `sweepMeshClosest`, `sweepShapeClosest`, `sweepMeshAny`,
> `sweepShapeAny`, `sweepMeshAll`, and `sweepShapeAll` take `ObjectKey` in
> place of their mesh/gPrim/traversal-root path parameter. The primitive
> queries (raycast, sphere/box sweep and overlap) are unaffected — they
> never carried a path parameter. A stale binary fails to compile or link
> rather than silently misinterpreting the parameter.
>
> **C++ source/binary break.** `IPhysxCooking::precookMesh`'s `meshPath`
> parameter (part of the same object-identity migration) retypes and renames to
> `meshKey` (`omni::physics::parse::ObjectKey`), matching this same header's
> already-migrated `createConvexMesh`/`cookAutoDeformableBody`.
>
> **C++ source/binary break.** `TriggerEvent.h`'s
> `TriggerEventData::triggerColliderPrimId`, `otherColliderPrimId`,
> `triggerBodyPrimId`, and `otherBodyPrimId` (part of the same
> object-identity migration) retype and rename to `triggerColliderPrimKey`,
> `otherColliderPrimKey`, `triggerBodyPrimKey`, and `otherBodyPrimKey`
> (`omni::physics::parse::ObjectKey`) in place of a `uint64_t`-encoded
> `SdfPath`. Independent of this same struct's `stageId`→`attachHandle`
> retype — `TriggerEventData` has no
> public `ovphysx` C ABI mirror, so that retype is not separately called
> out elsewhere in this changelog.
>
> **C ABI struct-layout break.** `PhysxCookingComputeRequest`
> (`ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h`) drops its
> `DataInputMode dataInputMode` field, the `DataInputMode` enum
> (`eINPUT_MODE_FROM_PRIM_ID`, `eINPUT_MODE_FROM_PRIM_MESH_VIEW`), and `double
> primTimeCode`, shrinking the struct by 16 bytes (4-byte enum, 4 bytes of
> alignment padding, and the 8-byte `double`). The removed
> `eINPUT_MODE_FROM_PRIM_ID` mode let the cooking service resolve a bare
> `primStageId`/`primId` pair by reading USD directly; every request now
> carries a `PhysxCookingMeshView` that the caller fills through the same
> backend-agnostic `IPhysicsSource` path ovstage callers already used
> exclusively. `primStageId` remains on the struct but is now a caller-owned
> correlation key only, not a stage-lookup input. This is a public SDK API
> break with no compatibility shim: a binary built against the earlier header
> must be recompiled and relinked. A caller already submitting mesh-view
> requests needs no change; a caller that relied on the removed bare
> prim-id/stage-id default must build and populate a `PhysxCookingMeshView`
> before submitting.
>
> **Source break.** `ConvexDecomposition::applySphereApproximation(const
> char* primPath, uint32_t stageId)` is removed from the public
> `omni/convexdecomposition/ConvexDecomposition.h` header. It had zero
> callers anywhere in this repo; the retained
> `ConvexDecomposition::computeSphereApproximation`, which takes a
> caller-supplied `SimpleMesh` instead of a USD prim path and stage id,
> remains available for sphere-approximation authoring. A caller relying on
> the removed prim-path overload must read its own mesh data and call
> `computeSphereApproximation` directly — there is no drop-in replacement.
>
> **C++ source break.** `IPhysx::createD6JointAtPath()` changes from
> `(ObjectKey jointKey, ObjectKey body0, ObjectKey body1)` to
> `(const char* jointPath, ObjectKey body0, ObjectKey body1)`. This is the
> third narrow place a path string is allowed to cross the public API
> boundary: the call is create-shaped, so
> `resolveObjectKey`'s existence gate can never resolve `jointPath` before
> the joint exists, and `jointPath` is instead minted into an `ObjectKey`
> existence-independently via `AttachedStage::keyFor()`. A null `jointPath`,
> or no attached stage, does not fail the call -- the joint is still created
> and returned if the underlying `PxD6JointCreate` succeeds, but it is
> silently skipped for `ObjectKey` registration and stays unresolvable by
> key afterward.
>
> **C++ source/binary break.** `IPhysxSimulation::setSimulationOutputFlags()`,
> `addSimulationOutputFlags()`, and `removeSimulationOutputFlags()` drop their
> `paths`/`numPaths` parameters and become pure global on/off toggles for the
> given `SimulationOutputType`. The per-path variant bit-cast `SdfPath` as
> `uint64_t` instead of using the runtime's object-identity type and had no
> callers anywhere in this repo, including the `ovphysx` SDK; it is removed
> rather than migrated. The global mode — which gates real transform/velocity
> write-back and replicator skip-write behavior — is unchanged in behavior; it
> is simply the only mode now. A binary built against an earlier header must
> be recompiled and relinked.
>
> Selected configuration arguments on `PhysX()`, `wait_op()`, `wait_all()`,
> `attach_ovstage()`, `read()`, `read_tokens()`, `get_contact_report()`, and
> `enable_python_logging()` are now keyword-only. Positional use of those
> arguments raises `TypeError`; pass them by name. Primary operands remain
> positional.
>
> **Logging C ABI break.** Log levels now use `DEFAULT=0`, `VERBOSE=1`,
> `INFO=2`, `WARNING=3`, `ERROR=4`, `NONE=5`. The former multi-callback
> `ovphysx_register_log_callback` / `ovphysx_unregister_log_callback` API is
> replaced by the single-slot `ovphysx_set_log_callback`, whose callback also
> receives channel and Unix-epoch timestamp metadata. `ovphysx_set_log_level()`
> now changes only the named `omni_physx_sdk`, `omni.physx`, and
> `ovphysx_internal` Carbonite source policies instead of Carbonite's
> process-global threshold. Unnamed, host, and dependency sources remain
> unchanged; the application callback still observes and filters the process
> log stream. `OVPHYSX_LOG_NONE` is therefore not a whole-runtime or process
> mute. Every successful
> shutdown disables and drains the callback, including when live handles remain
> solely for explicit destruction. Recompile C/C++ callers.
>
> **Logging Python source change.** `enable_python_logging()` now accepts a
> minimum severity and channel filter and owns the sole native callback slot
> while enabled, replacing any C-level callback. The callback's channel and
> timestamp are exposed as `ovphysx_channel` and `ovphysx_timestamp` on each
> Python `LogRecord`. Calling `PhysX()` or `PhysX.destroy()` from that callback
> is rejected; retry after callback delivery returns. Concurrent construction
> during process initialization waits; it shares a successful initialization or
> retries after a known failure. Construction rejects during ambiguous rollback
> and while final shutdown is draining so callback dependencies cannot deadlock
> the drain. A callback must not synchronously wait for work that may
> emit into the same serialized callback registration.
> Successful final shutdown disables and drains the Python bridge and releases
> its callback owners.
>
> **Returned-string source change.** Empty `ovphysx_get_last_error()` and
> `ovphysx_get_last_op_error()` results now have a non-NULL pointer; test
> `length`, not `ptr`, to distinguish an empty result.

> **Python source break.** `ReadGroup` gains two trailing fields, `cuda_stream` and
> `cuda_wait_event`, taking it from 14 to 16. It is a `NamedTuple`, so its arity is part
> of its public shape: unpacking a whole group (`a, b, ... = group`), comparing one
> against a 14-tuple, or unpickling one written by an earlier version now fails. Both
> fields default to `0`, so keyword construction, attribute access, indexing and slicing
> are unaffected. Read `group.cuda_wait_event` rather than positionally unpacking.
>
> **Python source break.** `PhysX.read()` and `PhysX.read_tokens()` now return
> `warp.array`. Every non-empty tensor is a Warp array on the read's native CPU or
> CUDA device; every non-empty index map is a CPU `warp.array` of `uint32` on both
> backends. They no longer return CPU NumPy arrays or CUDA `ManagedDLTensor`
> wrappers. A native lane count above 1 becomes a trailing Warp dimension. Use the
> Warp array directly, or call `.numpy().copy()` when an independent host copy is
> needed.
>
> **Behavior change — disabling a rigid body, and the unsupported raw-pointer path.**
> On a DirectGPU scene, disabling one rigid body no longer invalidates the whole scene
> view: bulk **reads and writes** omit the disabled body and keep serving its enabled
> peers (query discovery still counts it; CPU output stays inclusive). Because that hard
> invalidation is gone, toggling `PxActorFlag::eDISABLE_SIMULATION` on a `PxRigidDynamic*`
> obtained from `ovphysx_get_physx_ptr()` is now **unsupported and undefined**: ovphysx does
> not observe the change, so the next DirectGPU read or write silently resolves a stale index
> to the wrong body — where a 0.5 build failed loudly with a whole-view invalidation. Disable a
> **standalone** rigid body through the ovstage `disableSimulation` attribute via `ovphysx_write()`
> instead. A point-instancer **instance** has no supported per-instance disable route in this
> release — `disableSimulation` is not an instancer-writable column and there is no per-instance
> tensor equivalent — so disabling an individual instance is unsupported.

### Added
- **Output read: full attribute coverage.** `ovphysx_read()` / `PhysX.read()` now serve the same
  attribute set as the tensor API.

  | object type | attributes added |
  |---|---|
  | `RIGID_BODY`, `ARTICULATION_LINK` | linear/angular acceleration; `mass`, `inverseMass`, `inertia`, `inverseInertia`, `centerOfMassPosition`, `centerOfMassOrientation`, `disableSimulation`, `disableGravity`; `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`, `shapeCount` |
  | `ARTICULATION` *(new)* | `rootPosition`, `rootOrientation`, `rootLinearVelocity`, `rootAngularVelocity`, `centerOfMassWorld`, `centerOfMassLocal`, the per-shape properties, `shapeCount`, and `jacobian`, `massMatrix`, `coriolisForce`, `gravityForce`, `centroidalMomentum`, `jacobianShape` |
  | `ARTICULATION_JOINT` | the full DOF set: position/velocity targets, actuation and projected force, stiffness, damping, limits, max velocity/force, armature, friction, drive model, `driveType` |
  | `FIXED_TENDON`, `SPATIAL_TENDON` *(new)* | `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonOffset`; plus `tendonLimit` and `tendonRestLength` on fixed tendons |
  | `DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE` | `restPoints`, `simElementIndices`, and `collisionElementIndices` (volume only) |
  | `DEFORMABLE_MATERIAL` *(new)* | `dynamicFriction`, `youngsModulus`, `poissonsRatio`, `elasticityDamping`, `bendingStiffness`, `thickness`, `bendingDamping` |

- **App-to-physics write.** `ovphysx_write()` /
  `ovphysx_fetch_write_next()` / `ovphysx_commit_group()` / `ovphysx_release_write()`,
  and `PhysX.write()` in Python, open a session over a query and hand back writable
  groups whose columns match what `ovphysx_read()` emits for the same query — same
  prims, same order, same residency. A column can be read, edited and written back
  with no repack, and on a GPU scene without leaving the device. Values are published
  through the tensor backend, not through per-actor USD authoring.

  A failed `ovphysx_commit_group()` says WHICH failure it was. A group that was never
  live — unknown session, or unknown, foreign or already committed — is rejected before
  any publish and reports that nothing was written. A live group whose publish failed
  does not: the scatter ran, a device scatter can fail partway, and commit is not
  retryable, so the error says the group is spent and that how much reached the solver
  is not reported at this layer. The two previously shared one message that claimed
  nothing was published.

  In Python a group's columns are MUTABLE ALIASES of the runtime's storage, never
  copies: fill them in place, then commit. Device columns come back as Warp arrays on
  their own device, matching what `read()` returns, so torch/cupy can take them
  zero-copy; host columns come back as NumPy views over the mapped pointer. Neither
  outlives its session.

  Coverage:
  - `OVPHYSX_OBJECT_RIGID_BODY` — `position`, `orientation`, `linearVelocity`,
    `angularVelocity`.
  - `OVPHYSX_OBJECT_ARTICULATION_JOINT` — `jointPosition`, `jointVelocity`.

  Contract notes:
  - **One attribute per session.** The ovstage map group carries no attribute field,
    so writing position and orientation over one prim set is two sessions.
  - **A group commits once**, identified by its address. Committing an unknown,
    foreign or already-committed group is refused rather than silently accepted.
  - **Uncommitted groups are discarded** on release, so a caller that fails mid-fill
    publishes nothing from the group it was filling.
  - **`position` and `orientation` are two slices of one transform**, so writing
    either preserves the other — at the cost of reading the current pose first. The
    two velocities are independent and pay no such cost. The same applies to joint
    DOFs: a write preserves the DOFs it does not address.
  - **Articulation links refuse per ATTRIBUTE, not per type.** A `PxArticulationLink`
    is a `PxRigidBody`, so `OVPHYSX_OBJECT_ARTICULATION_LINK` accepts `mass`,
    `inertia`, `centerOfMassPosition` / `centerOfMassOrientation`, `disableGravity`
    and the per-shape properties. It refuses `position`, `orientation`,
    `linearVelocity` and `angularVelocity` — PhysX has no link write for them — and
    refuses `disableSimulation`, which `PxActorFlag` supports on `PxRigidStatic` and
    `PxRigidDynamic` only. Each refusal names the attribute and the alternative.
  - **Articulation DOF properties and drive inputs are writable** on
    `OVPHYSX_OBJECT_ARTICULATION_JOINT`: the thirteen properties the read publishes
    — `jointStiffness`, `jointDamping`, `jointLimit`, `jointMaxVelocity`,
    `jointMaxForce`, `jointArmature`, the friction triple, the drive-envelope
    triple and `jointDriveType` — plus `jointPositionTarget`,
    `jointVelocityTarget` and `jointActuationForce`. Names are shared with the
    read, so the two cannot drift apart.

    Each write applies the exact inverse of the read's unit fold, per attribute
    rather than per object: on the same rotational axis the two targets carry the
    rad→deg fold and the body-order sign, while `jointActuationForce` carries only
    the sign — a joint effort is not an angular quantity.

    `jointLimit` is one attribute of two lanes, not two attributes. It is
    **refused** on an axis whose motion is not `eLIMITED`: PhysX does not allow an
    axis to become limited while its articulation is in a scene, so the limit
    would silently do nothing. The refusal names the condition and writes nothing
    at all, so a limit never half-lands. Writing back the `±FLT_MAX` sentinel the
    read reports for a free axis is not a finite limit and still succeeds, which
    keeps read-modify-write over an articulation with a mix of limited and free
    axes working.
  - **Fixed: host-only rigid properties failed to commit on a DirectGPU
    scene.** `mass`, `inertia`, `centerOfMassPosition` / `centerOfMassOrientation`,
    `disableGravity` and `disableSimulation` publish a host column even on a
    DirectGPU scene, and the commit was choosing which backend view to resolve
    rows through from that column's residency — so it asked for the CPU
    simulation view, which does not exist there, and the write failed while the
    matching read worked. The view now follows the scene's pipeline. Affects
    every DirectGPU scene, not only scenes with disabled bodies.
  - **Fixed: disabling one rigid body no longer blocks DirectGPU bulk reads and
    writes for its scene.** Device output and ordinary write groups omit disabled
    bodies while continuing to serve enabled peers. Query discovery still counts
    every matching prim, and `disableSimulation=0` becomes available for
    re-enabling a disabled body. CPU reads remain inclusive because the host
    actor state is still available.
  - **Deformable simulation-mesh state is writable** on
    `OVPHYSX_OBJECT_DEFORMABLE_VOLUME` and `OVPHYSX_OBJECT_DEFORMABLE_SURFACE`:
    `points` and `velocities`, one array group per body. `points` is accepted in
    the SIM-MESH local frame — the frame the read publishes it in — and
    `velocities` in world, matching the read on both.

    These columns are **device-resident**, unlike the particle and material
    ones: PhysX exposes sim-mesh state only as device buffers, so the write is a
    device scatter into PhysX's own memory followed by the `markDirty` its
    header calls mandatory. Volume and surface bodies expose different getters
    and different dirty-flag enums for the same two columns, and both are chosen
    from the object's concrete PhysX type.

    `restPoints` and `kinematicTarget` are refused by name. `restPoints` is
    authored geometry the solver never rewrites. `kinematicTarget` is set by
    handing PhysX a buffer it *keeps*, which a write session cannot supply
    because its column is freed at release — serving it needs an owned per-body
    buffer with a lifetime this API does not have.
  - **Deformable material properties are writable** on
    `OVPHYSX_OBJECT_DEFORMABLE_MATERIAL`: `deformableYoungsModulus`,
    `deformablePoissonsRatio`, `deformableDynamicFriction`,
    `deformableElasticityDamping`, plus the surface-only
    `deformableBendingStiffness`, `deformableThickness` and
    `deformableBendingDamping`. One fixed group stacking every material prim,
    one f32 each, host-resident on every scene — these are authored inputs PhysX
    never writes back, so there is no device copy to hand out.

    The three surface-only names address the SURFACE materials only, which is
    the subset the read publishes them over: PhysX puts them on
    `PxDeformableSurfaceMaterial` alone, and the read omits the row on a volume
    material rather than reporting `0.0`. Which kind a material is comes from
    PhysX's own concrete type, not from the record that names it.
  - **Particle points and velocities are writable** on
    `OVPHYSX_OBJECT_PARTICLE_SET`, as one array group per set. `points` is
    accepted in the set prim's LOCAL frame — the frame the read publishes it in
    — and `velocities` in world, matching the read on both counts.

    These columns are **host-resident even on a DirectGPU scene**, unlike the
    read's, which are device-resident. The destination is the reason: a particle
    write lands in the set's pinned host staging arrays and raises its upload
    flag, and PhysX copies that to the device buffer at the next step. It is the
    same mechanism USD authoring already publishes these two quantities through,
    which is what keeps a written column readable *before* the next step — the
    read serves the staging arrays exactly when that flag is raised.
  - **Point-instancer instances are writable**, on both devices. A `position`,
    `orientation`, `linearVelocity` or `angularVelocity` write on
    `OVPHYSX_OBJECT_RIGID_BODY` emits the standalone group *and* one array group
    per instancer, carrying that instancer's full instance array placed by index —
    the same shape the read emits, so a column can be read and written straight
    back. An index with no live instance is **skipped**: there is nothing to write
    to, which mirrors the read leaving such a slot as the caller zero-filled it.
    Note a **scaled** instancer round-trips only up to its scale: the local pair
    carries position and orientation, and USD keeps `scales` as its own array.
  - **Vehicle wheel controls are writable** on `OVPHYSX_OBJECT_VEHICLE_WHEEL`:
    `driveTorque`, `brakeTorque` and `steerAngle`, for vehicles with no drive,
    whose control surface is per wheel. `position` and `orientation` are refused
    on that type by name and permanently — a wheel's transform is composed each
    step from the chassis, its suspension and its steer angle, so a written value
    is overwritten by the next step. Vehicles are CPU-only, so these have no
    device path. Vehicles WITH a drive are accelerated and steered as a whole and
    their commands are not yet reachable.
  - **Force and wrench are writable**, and are the API's only **write-only** attributes.
    `force` is a vec3 applied at the centre of mass; `wrench` is `[N,9]` —
    force, torque, and the WORLD-space point the load applies at — deliberately
    not split, because the split rule exists so each half can be read back and
    preserved, and a control input the solver clears each step has no stored value
    to preserve. Both are accepted on `OVPHYSX_OBJECT_RIGID_BODY` and
    `OVPHYSX_OBJECT_ARTICULATION_LINK`. Neither is ever emitted by
    `ovphysx_read`: reading a force back would report what the solver did with
    it, not what was written. A force applies for one step and is then cleared,
    so it must be written before each step it should act on. On a link, note that
    a commit covers the articulation view as a whole: links the query did not
    match have their force zeroed for that step, because a write-only input has no
    stored value to preserve.
  - **Tendon properties are writable**, on `OVPHYSX_OBJECT_FIXED_TENDON` and
    `OVPHYSX_OBJECT_SPATIAL_TENDON`: `tendonStiffness`, `tendonDamping`,
    `tendonLimitStiffness` and `tendonOffset` on both kinds, plus `tendonLimit`
    (the `(low, high)` pair) and `tendonRestLength` on fixed tendons only. The two
    fixed-only names are refused on a spatial tendon by name — not because PhysX
    lacks a setter, but because the schema places both on the tendon's leaf
    attachment, which is the same reason the read refuses them. No unit conversion
    in either direction: the rad→deg fold lives in the gearing coefficient, so
    everything downstream of it is passthrough.
  - **The articulation itself is writable**, through the new
    `OVPHYSX_OBJECT_ARTICULATION` (`SimObjectType.ARTICULATION`, enum value 9), whose
    prims are the articulation root prims. It accepts `rootPosition`, `rootOrientation`,
    `rootLinearVelocity` and `rootAngularVelocity` — the same names the read uses for
    this type, not the bare rigid spellings — the articulation's ROOT state, and the
    documented way to place an articulation now that link poses are refused. The
    selector overlaps `OVPHYSX_OBJECT_ARTICULATION_LINK` on the root prim, which is
    fine: these are queries, not a partition, and the two reach different state on it.
  - **Link poses are not refreshed by a write.** After writing joint state or a root
    pose, link poses read back at their last-stepped values until the next step,
    matching the read's contract of reporting the most recently completed step. No
    `eUPDATE_KINEMATIC` is issued.

- **Articulation tendon output read.** Two new simulated object types,
  `OVPHYSX_OBJECT_FIXED_TENDON` and `OVPHYSX_OBJECT_SPATIAL_TENDON`
  (`SimObjectType.FIXED_TENDON` / `SPATIAL_TENDON` in Python), read through the
  same `ovphysx_query` / `ovphysx_read` lifecycle as every other type. Both serve
  `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness` and `tendonOffset`; fixed tendons also serve
  `tendonLimit` (a `(low, high)` pair, `dtype.lanes == 2`) and `tendonRestLength` — the
  spatial tendon has neither, because the schema places them on its leaf
  attachment. The prim reported for a tendon is the prim carrying its **root**
  API: the joint with `PhysxTendonAxisRootAPI`, or the link with
  `PhysxTendonAttachmentRootAPI`. A tendon's other axes and attachments do not
  produce rows. Groups are fixed (one row per tendon, all tendons stacked), and
  columns are device-resident on a DirectGPU scene like the other
  backend-sourced reads. Tendon properties are authoring-time values, so
  `OVPHYSX_SCOPE_ACTIVE` behaves as `OVPHYSX_SCOPE_ALL` for both types.
- **Output-read acceleration attributes.** `ovphysx_read()` serves
  `linearAcceleration` and `angularAcceleration` (`OVPHYSX_ATTR_LINEAR_ACCELERATION` /
  `OVPHYSX_ATTR_ANGULAR_ACCELERATION`, `vec3 f32`) for `OVPHYSX_OBJECT_RIGID_BODY` and
  `OVPHYSX_OBJECT_ARTICULATION_LINK`, on both devices, closing the gap against the tensor
  binding's `RIGID_BODY_ACCELERATION_F32` and `ARTICULATION_LINK_ACCELERATION_F32`.
  Point-instancer instances publish them as the array attributes `accelerations` and
  `angularAccelerations`. Values come from the same DirectGPU scratch buffers velocity
  uses, so a read asking for both pays one extra bulk read rather than a second gather
  path.
- **Output-read body properties.** `ovphysx_read()` serves `mass`, `inverseMass`, `inertia`,
  `inverseInertia`, `centerOfMassPosition`, `centerOfMassOrientation`, `disableGravity` and
  `disableSimulation` for `OVPHYSX_OBJECT_RIGID_BODY` and `OVPHYSX_OBJECT_ARTICULATION_LINK`,
  closing the `RIGID_BODY_MASS/INERTIA/COM/…` and `ARTICULATION_BODY_*` tensor-binding gap.
  Three things differ from the existing columns and a consumer must handle them:
  `disableGravity` / `disableSimulation` are **`uint8`** (`{kDLUInt, 8}`), not f32 — read
  `tensors[i].dtype`; all eight are **always host-resident** (`kDLCPU`), because they are
  simulation inputs PhysX never writes back, so on a DirectGPU scene one read can return
  `kDLCUDA` pose alongside `kDLCPU` mass — branch on `tensors[i].device.device_type`; and they
  have **no point-instancer array form**, so a read asking for one warns and omits instanced
  bodies from that column. `centerOfMass*` is in the body's local frame, unlike
  `position`/`orientation`.
- **Output-read per-shape properties.** `ovphysx_read()` serves `staticFriction`,
  `dynamicFriction`, `restitution`, `contactOffset`, `restOffset` and `shapeCount` for
  `OVPHYSX_OBJECT_RIGID_BODY` and `OVPHYSX_OBJECT_ARTICULATION_LINK`, completing rigid-body
  parity with the tensor binding apart from the write-only force/wrench pair. A body has a
  variable shape count, so these are a **padded fixed group**: one stacked tensor whose
  `dtype.lanes` is the widest body *in that read* — it varies between reads of the same
  attribute, so do not cache it. Row *i* holds `shapeCount[i]` real values then zeros; read
  `shapeCount` (int32) to find the end, since 0.0 is a legal offset and is not a terminator.
  Padding is zero-filled, unlike the tensor API's, which is uninitialised. Only a shape's
  **first** material is reported, matching the binding. Host-resident like the other properties.
  Note each material attribute costs its own `getMaterials()` pass over every shape, so reading all
  three walks them three times.

  Three things to check in your reader:

  - Take `dtype.lanes` from each **group**, not once per attribute. The inverse dynamics matrices are
    sized by each articulation's topology, so a scene with two robot types returns two groups per
    attribute; a uniform fleet returns one, as before.
  - A joint-state read returns one group per attribute, not one per joint prim. Deformable and
    particle reads changed the same way: a scene of N sets now yields **one** array group of N
    tensors where it used to yield N groups of one. Walk `tensor_count` and pair each tensor with
    the prim at the same index in the group's prim list.
  - One read can mix devices — a CUDA pose column beside a CPU property column. Branch on
    `device_type` per tensor. Deformable and particle `points` / `positions` / `velocities` are
    now `kDLCUDA` where they used to be host, with a `data.cuda_sync.wait_event` to wait on
    (`ReadGroup.cuda_wait_event` in Python). Point-instancer arrays are `kDLCUDA` on a GPU scene
    at either scope and for every attribute, including acceleration; they used to fall back to
    host for all but the simplest read. `PhysX.read` already handles this.

  Not served: `kinematicTarget`, which is simulation input rather than output. Omitted where it
  does not apply: `centroidalMomentum` on fixed-base articulations, `collisionElementIndices` on
  surface deformables, and `bendingStiffness` / `thickness` / `bendingDamping` on volume
  materials. A group can therefore cover a subset of its type's prims — pair values with the
  group's own prim list.

  Reads are also faster, at 8,192 environments: joint state ~0.31 ms (was ~728 ms), links ~0.28
  ms (was ~1.09 ms), and a four-attribute read over 163,860 rigid bodies ~0.65 ms on GPU, ~33
  ms on CPU (was ~52 ms). The same read over 163,840 point-instancer instances is ~0.7 ms on
  GPU (was ~5.5 ms) and ~19 ms on CPU (was ~21 ms). The particle read gained two orders of
  magnitude separately, by using the world-to-local matrix the runtime already maintains
  instead of resolving one out of USD per set on every read. A read that cannot produce a
  column now reports failure instead of returning short or zero-filled data.

- **Benchmark timing diagnostics.** `--timing-diagnostics=<path>` writes one
  schema-version-1 JSONL object per emitted row with the count, mean,
  population standard deviation, minimum, and maximum across every actual
  timed step; the existing trimmed report value is unchanged. A separate
  hidden DirectGPU row measures the first creation and spec lookup of the five
  bindings used by the 4,096-Cartpole control row. Stage loading, cloning,
  warmup, and caller CUDA tensor-buffer allocation are outside that timer.
- **OmniPVD TCP startup transport.** Four typed C and Python config fields select
  TCP, address, port, and millisecond send timeout at instance creation. FILE
  remains the default; TCP connects synchronously to an already-ready trusted
  plaintext listener.
- **Named native timeout contract.** Added the ABI-identical
  `ovphysx_timeout_t` alias plus `OVPHYSX_TIMEOUT_POLL` and
  `OVPHYSX_TIMEOUT_INFINITE`. Timeout-bearing C and experimental C++ APIs now
  use the named type and values; existing binaries and `uint64_t` source
  callers remain compatible. Python keeps its idiomatic `0` (poll), positive
  integer (finite nanoseconds), and `None` (infinite) spellings.
- **Logging and returned-string contracts.** Added single-callback replacement,
  longest-prefix channel filtering, serialized delivery, reentrancy guards, and
  a bounded barrier for callback deliveries already accepted by the ovphysx
  dispatcher. Every `ovphysx_string_t` successfully produced or
  delivered by ovphysx, including populated outputs and callback values,
  now has a non-NULL pointer and trailing NUL at `ptr[length]`; input strings
  remain length-prefixed views.
- **OmniPVD capture compatibility docs.** The
  OmniPVD recording tutorial now states the OVD format, OmniPVD stream version
  (0.4.0), the independent PhysX OVD integration version (3.1), Kit
  `omni.physx.pvd` as the canonical reader, and both reader compatibility
  checks.
- **NVTX profiling for Nsight Systems.** ovphysx can emit NVTX ranges for its API
  calls (`ovphysx` domain) and for the PhysX SDK profile zones, CPU and GPU
  (`PhysX` domain). Enable with `OVPHYSX_NVTX=1` in the environment or
  `PhysXConfig(nvtx_enabled=True)` / `ovphysx_config_entry_nvtx_enabled(true)`
  before instance creation; read the effective state with
  `ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, ...)` or
  `PhysX.get_config_bool(ConfigBool.NVTX_ENABLED)`. Instrumentation is compiled
  into release builds and the wheel, so profiling needs no rebuild, and is off by
  default. NVTX 3 is header-only: nothing extra is linked or shipped. See
  [NVTX Profiling With Nsight Systems](developer_guide.md#nvtx-profiling-with-nsight-systems).
- **Checked Python instance destruction.** `PhysX.destroy()` is now the
  canonical deterministic lifecycle operation. It is idempotent after the
  instance reaches terminal state and raises when native destruction or final
  process shutdown reports a failure.
- **Process-wide hard CPU-only mode is observable.** Added
  `ovphysx_get_cpu_mode` / `PhysX.get_cpu_mode()` / experimental `PhysX::getCpuMode()`
  to report the effective hard CPU-only policy (`ovphysx_set_cpu_mode(true)` or
  `OVPHYSX_DISABLE_GPU`). Successful `ovphysx_create_instance()` now emits an
  INFO line with `process_cpu_only=`, `cuda_available=`, and `active_cuda_gpus=`
  (`no_override` for empty create-args; explicit `"-1"` logs as `-1`).
  `OVPHYSX_DISABLE_GPU` is latched at `ovphysx_initialize` (live before that).
  This is hard-policy / create-intent observability only -- not attach-time
  resolved scene dynamics or ordinal.
- **`ovphysx_scene_query_get_paths_from_ids()` resolves scene-query hit
  identities to paths.** New C API (Python:
  `PhysX.get_scene_query_paths_from_ids()`) resolves the opaque
  `ObjectKey.handle` values in a raycast/sweep/overlap hit's `collision`,
  `rigid_body`, and `material` fields back to physics-object paths, mirroring
  `ovphysx_contact_binding_get_other_actor_paths_from_ids()` for contact
  bindings. Unlike that resolver, returned strings are not a per-call cache:
  each is owned by the currently attached physics source and stays valid
  until the next detach/re-attach, so a later call to this function (against
  the same attach) does not invalidate a pointer an earlier call returned.
  Unresolvable ids (the zero sentinel, an id from an object removed since the
  query, or no active attach) yield an empty path rather than an error. See
  `developer_guide.md`'s "Path Encoding" section under Scene Queries.
  **Open question, disclosed rather than silently decided:** this resolver is
  a compatibility bridge that restores the path-resolvability the
  object-identity migration above took away from the raycast/sweep/overlap
  family, not a statement about that family's long-term home. Whether that
  family is kept, removed, or relocated to a successor scene-query library is
  not yet decided.
- **`ovphysx_cuda_stream_wait_event(stream, event)` orders your stream after a
  read's work.** A device read column is handed over
  before the work producing it has necessarily finished; call this to order your own CUDA
  stream after the group's event, without taking a CUDA dependency of your own. An `event` of
  `0` is a no-op success.
- **`ovphysx_debug_render_set_scope_tokens()` filters debug visualization by
  exact OVStage path handles.** The scope API has no string-prefix or path-
  interning variant: callers expand a hierarchy to its exact object set and use
  the Stage's `path_dictionary_instance_t` to create `ovx_primpath_t` handles.
  Matching uses the runtime object's canonical source key, so it does not require
  exposed PhysX actor names. Handles require no individual release, remain valid
  only for their originating Stage dictionary, and the scope is cleared on detach.
- **Python type stubs (``.pyi``) and ``py.typed`` marker for IDE support.**
  The wheel and editable source tree now ship PEP 561 stubs for ``PhysX``,
  ``TensorBinding``, ``ContactBinding``, scene-query result types, DLPack
  structures, and module-level exports so VS Code, PyCharm, and Cursor can
  provide autocompletion and static type checking without loading native code.
  CI and ``validate_all`` run ``pyright`` on the stub tree via
  ``scripts/test_pyright.cmake``.
- **Kinematic support-geometry guidance and samples.** New C and Python samples
  demonstrate a translating support driven by ovstage transform updates, a
  stationary `PhysxSurfaceVelocityAPI` conveyor, and their additive combination
  with dynamic riders. The accompanying guide explains how kinematic transform
  changes become PhysX targets, distinguishes them from teleporting legacy
  tensor pose writes, and documents friction, sleeping, reset/readback, control
  ordinals, authored-scale preservation, local versus resolved world matrices,
  descendant world updates, and the DirectGPU limitations.
- **`ovphysx.utils.step_and_write_to_ovstage(physx, *, dt, output_ordinal,
  outputs=None)` utility.** Runs one `step_sync`, the matching `PhysX.read`
  output selections, and the OVStage write-back without expanding the core
  `PhysX` API. Each group's emitted attribute name is written to a shadow
  `sim:<name>` attribute (avoiding a type collision with the authored column),
  `prim_list` forwards directly, fixed-group tensors pass through as-is, and
  array-group tensors are lane-folded into OVStage's `dtype.lanes` vector
  representation; the whole write seals with one
  `advance_write_floor(ordinal=output_ordinal)`. `output_ordinal` must never be
  passed to `update_from_ovstage`, or physics would re-ingest its own output.

- **Three physics fixtures are added to the shipped sample data payload.**
  `empty_dynamic_boxes.usda`, `empty_dynamic_boxes_cpu.usda` and
  `simple_physics_scene_cpu.usda` land under `tests/data/`, which
  `install.cmake` and `build_wheel.cmake` copy into the SDK (`samples/data/`)
  and the wheel (`ovphysx/samples/data/`). The two `_cpu` layers are sublayer
  overlays declaring an explicit CPU solver and MBP broadphase over their base
  scene, for consumers needing the CPU pipeline rather than the GPU-dynamics
  default a `PhysicsScene` without `PhysxSceneAPI` selects. No existing fixture
  changed.

- **Sequential OmniPVD recording.** `ovphysx_start_recording()`,
  `ovphysx_stop_recording()` and `ovphysx_is_recording()` — `start_recording()`,
  `stop_recording()` and `is_recording()` in Python — record to an exact `.ovd`
  file or to a TCP listener that is already accepting, and can be started and
  stopped repeatedly while the runtime is live. Every session, whether it starts
  at startup or later, captures the current core PhysX state plus the full
  PhysXExtensions and PhysXVehicle schemas and their live objects. Recording is
  off unless it is asked for: the new creation-time `omnipvd_recording_capable`
  setting is false by default, and a runtime created without it (and without
  startup output) passes no `PxOmniPvd` to PhysX at all, so the default path
  carries no sampler, writer or stream cost. Opting in creates the provider and
  enables scene readback, which a DirectGPU scene pays for even before recording
  starts. Only one recording is active per shared runtime; starting a second one
  returns `INVALID_STATE` and leaves the first destination in place.

- **`newton:velocityLimit` is honored on joints.** The Newton `NewtonJointAPI`
  attribute maps onto `physxJoint:maxJointVelocity` with the same precedence as
  the other Newton attributes: an authored PhysX value wins, then an authored
  Newton value, then the PhysX default. Both are joint-level and share units —
  degrees per second for angular DOFs, distance per second for linear ones — so
  the value is used as authored. A per-axis `PhysxJointAxisAPI:maxJointVelocity`
  still overrides it, at parse time and on a live edit.

### Performance
- **A stage with several physics scenes no longer rebuilds its backend cache on every
  read.** The tensor backend's simulation data and its cached superset views were keyed
  on the attach handle alone, so two `PhysicsScene` prims under one attach shared a
  single entry: reading them in turn evicted one and rebuilt the other, re-running
  `GpuSimulationData::init()` -- every device allocation -- plus the superset view
  construction, on every read of every partition. Six alternating acquires produced six
  rebuilds where two suffice. Cache entries are now keyed on the attach *and* the scene,
  so each scene keeps its own. Single-scene stages are unaffected, and values were always
  correct -- only the cost changes.
- **Hidden persistent-contact step/read benchmarks.** Added separate CPU and
  conventional-GPU rows that measure one 1/60-second synchronous step plus the
  public raw contact-report pull over 512 isolated persistent reporter/static
  pairs. Both rows validate exact pair/cardinality and borrowed report contents
  outside the timer, fail on known GPU fallback or capacity warnings, and
  reject DirectGPU. The existing seventeen-row Authoring/WriteScaling L1B
  inventory is unchanged. Its six hidden requested-GPU Authoring diagnostics
  already failed on recognized CPU-fallback warnings and now also fail on GPU
  contact-capacity warnings observed during attachment or warm-up. This changes
  their pass/fail gate, not their names or the CPU KPI contract.
- **Vehicle wheel `position` and `orientation` share one transform composition.** Reading both now
  composes each wheel's world transform once -- `getGlobalPose`, `getCMassLocalPose` and either the
  shape's local pose or the vehicle SDK's -- instead of running the whole composition per column and
  discarding the half it did not publish. On 1,024 vehicles a two-column read costs **~149 us against
  ~278 us before (1.87x)**; a one-column read is unchanged. Values are identical -- only the cost
  changes.
- **Faster per-shape rigid-body reads.** A read asking for more than one of `staticFriction` /
  `dynamicFriction` / `restitution` / `contactOffset` / `restOffset` now walks each body's shapes
  ONCE for the whole set instead of once per column, and resolves the shape's material once per
  shape rather than once per shape per column. On 8,192 bodies (655,440 shape visits) a five-column
  read costs **~47.4 ms against ~76.1 ms before (1.60x)**; a one-column read is unchanged. Values are
  identical -- only the cost changes.

  Three of the five properties are read off the shape's material and two are not, so a
  `contactOffset` + `restOffset` read now touches no material at all, where per-column dispatch
  resolved one for every column that asked.
- **Faster whole-articulation root-state reads on CPU.** A read asking for more than one of
  `rootPosition` / `rootOrientation` / `rootLinearVelocity` / `rootAngularVelocity` now hands the
  whole requested set to one backend gather instead of dispatching per column. On 8,192
  articulations the four-column read costs **~1.01 ms against ~2.17 ms before (2.15x)**; a
  one-column read is unchanged. Values are identical -- only the cost changes.

  The four columns come from fewer sources than there are columns. On the host all four are filled
  by a single `copyInternalStateToCache` per articulation, the root-transform and root-velocity
  flags OR'd into one call, where per-column dispatch made four. On a DirectGPU scene one
  `eROOT_GLOBAL_POSE` copy now serves both pose columns, taking four device fetches to three;
  linear and angular velocity are distinct read types and still need one each. The device read
  measures the same as before, so its per-column cost is not the DirectGPU copy.
- **Hidden in-process Authoring and WriteScaling benchmark contract.** Added
  fail-closed C++ rows for population growth/churn, runtime writes,
  transform/velocity updates, and the 4,096-body write-scaling pair. The frozen
  contract is CPU-canonical (eleven CPU rows); six requested-GPU Authoring rows
  remain unscheduled diagnostics. Initial lower-is-better absolute-latency KPIs
  are `Authoring.population_add_drip_cpu` and
  `WriteScaling.velocity_ovstage_4096_cpu`; every other retained row is a
  diagnostic or comparator. A dedicated Linux CI job builds and installs the
  opt-in suite and requires exactly eleven positive CPU rows without adding
  benchmarks to the publish/security-critical SDK build path.
- **Tensor-binding creation and OVStage attach and write throughput restored.**
  A regression made tensor-binding creation and the OVStage population and
  velocity-write paths several times slower on large stages. Parse contexts now
  reuse their token set instead of rebuilding it per operation, object keys are
  resolved from the runtime databases before falling back to a source query so
  runtime-only clones cost no lookup, incremental OVStage updates reuse
  source-native token ids, and wildcard matches are converted to keys once while
  keeping their creation-order result ordering. In a local measurement,
  tensor-binding creation went from 5.93 s to 0.61 s and OVStage velocity writes
  from 306 ms to 110 ms.
### Changed
- **ovphysx is built USD-free by default, and no longer ships a USD runtime.**
  The build option that produced the USD-free library was experimental and off
  by default; it is now on by default and is renamed to `OVPHYSX_NO_USD`,
  dropping the `EXPERIMENTAL_` marker from its old name. `libovphysx` has no
  link-time dependency on any USD library, and the shipped SDK and wheel no
  longer contain the namespaced USD monolith at all — roughly 70 MB smaller.
  USD comes entirely from the ovstage runtime the library already requires, and
  ovphysx binds to that one copy.

  This closes a class of hard aborts: a process that reached two monolith
  images — ovstage's and ovphysx's — registered USD's process-wide singletons
  twice and died with `multiple debug symbol definitions`, even when the two
  files were identical. Packaging now verifies that the ovstage runtime
  provides a monolith matching the one ovphysx was built against, and fails the
  install rather than producing a payload with no USD behind it.

  Scene replication and cloning, previously the one capability that did not
  work in the USD-free configuration, now work. Set `-DOVPHYSX_NO_USD=OFF` to
  build the USD-linked variant, which is still supported for consumers that
  attach a native `UsdStage`.
- **Updated the pinned USD and Carbonite runtimes** to USD `0.25.11.kit.5` and
  Carbonite `214.0.0-pre`.
- **The bundled PhysX engine moves from 5.10 to 5.11.** The simulation engine
  inside the wheel and the SDK is a newer PhysX SDK release. Solver results can
  differ in the last bits from 0.5, so a test that pins exact trajectories may
  need rebaselining; no ovphysx API changes with it.
- **Articulation Jacobians, mass matrices and centroidal momentum now use the
  authored joint basis.** DOF state and generalized forces already followed each
  USD joint's authored body relationship, but these dense quantities were
  returned in PhysX's raw basis. For a joint whose `body1` is the articulation
  parent, related quantities on the same view therefore pointed in opposite
  directions. All of them now share one basis, in `ovphysx_read()` and in the
  tensor bindings, on CPU and DirectGPU. Articulations whose joints are all
  parent-first are unaffected. A caller that flipped signs itself for reversed
  joints — on Jacobian columns, mass-matrix rows and columns, or centroidal
  joint columns — must drop that correction. Floating-root coordinates, the
  mass-matrix root block and the centroidal bias are unchanged; Coriolis and
  gravity already carried the fold.
- **`ARTICULATION_MASS_CENTER_WORLD` is now reported relative to the view's
  subspace origin on the DirectGPU path.** The GPU tensor binding previously
  returned PhysX's raw computed centre of mass, while the CPU path already
  subtracted the subspace origin; the two disagreed for any view built with a
  non-zero origin. The GPU path now matches the CPU one, so the value is
  world-frame in the same sense on both devices.

  **This changes a shipped value.** A view whose subspace origin is zero — the
  default, and every view that does not opt into a subspace — is unaffected.
  A consumer that was subtracting the origin itself to work around the GPU
  behaviour will now double-subtract and must drop that correction. The
  ovstage output read's centre-of-mass column is scene-world and is not
  affected either way. The same read no longer depends on what the destination
  buffer held before the call, and its DirectGPU result is complete when the
  getter returns.
- **CPU-only tensor property APIs no longer silently stage GPU tensors
  Shape properties, disable-gravity/simulation flags, and
  wake/sleep end in CPU PhysX calls. Gpu*View helpers that copied caller GPU
  buffers to host (and the matching ovphysx write/read staging for those
  types) are removed: pass host tensors or the call fails. On GPU simulation,
  `eDISABLE_SIMULATION` removes DirectGPU rows — the parent simulation view
  is invalidated (`getValid() == false`) and callers must recreate bindings
  for the enabled set (no disabled-pose patch cache / escape hatch). DirectGPU
  `createRigidBodyView` omits already-disabled rigid dynamics so a wildcard
  pattern still yields a valid enabled-only view; re-enable is out of band,
  then recreate. Bad index tensors (GPU, wrong dtype, oversized, or
  out-of-range values) fail with no side effects on both CPU and GPU sims.
  `BaseArticulationView::setCOMs` now clears the COM cache used by GPU
  force-at-position (matching rigid bodies).
- **String config buffer accounting.** `ovphysx_get_global_config_string()`
  rejects NULL, zero-capacity, and capacities above `UINT32_MAX`. On
  `OVPHYSX_API_BUFFER_TOO_SMALL`, `value_out->length` remains the caller's
  writable capacity so the same descriptor can be reused after growing its
  buffer; `out_required_size` reports the required size including the NUL.
- **Build dependencies no longer come from the kit-kernel package.** Carbonite and
  Omni framework headers now come from `carb_sdk_static` (`carb_sdk_plugins`), python
  3.12 from the same package, and the namespaced USD monolith and gsl from
  `ovphysx/ovruntime`.  The USD version is taken from the `ovruntime_deps` package that
  ovruntime uses, so the build USD and the py312 USD used by the python tests cannot
  drift apart.  This removes the kit-kernel download from the ovphysx build; there is
  no change to the shipped SDK or wheel contents.
- **Clone target poses are now named `anchor_transforms`.** The C parameter,
  experimental C++ parameter, and Python keyword were renamed from
  `parent_transforms` (`parentTransforms` in C++) because every entry anchors the
  exact target subtree root at its final absolute world pose; it is not a parent
  pose. Python keyword callers must update to `anchor_transforms`. The C binary
  ABI and runtime placement behavior are unchanged.

- **`ovphysx_warmup_gpu` renamed to `ovphysx_warmup` and extended to CPU mode.**
  The warmup step (a 1ns simulate+fetchResults pass that initializes PhysX lazy
  structures and disables per-step Fabric sync overhead) now runs in CPU mode as
  well as GPU mode. The old function name is removed; callers must update to
  `ovphysx_warmup` (C) / `warmup()` (Python).
- **Statically linked the OmniPVD runtime.** SDK and wheel artifacts no longer contain PVDRuntime_64.dll or libPVDRuntime_64.so; recording APIs and behavior are unchanged.

### Fixed
- **`LogLevel.NONE` now silences the remaining PhysX runtime warnings.**
  `set_log_level(NONE)` no longer lets GPU-broadphase
  fallback or deformable CUDA-context warnings through. The GPU-broadphase
  fallback warning is emitted at most once, and rigid-only scenes no longer
  warn about a missing deformable CUDA context. Source-level updates also
  leave a host-globally disabled Carbonite log channel disabled, so
  `set_log_level(WARNING)` cannot reopen it.
- **OmniPVD recording works on Linux AArch64.** FILE and TCP recording were
  compiled out on that platform, left over from the days when PVDRuntime shipped
  as a separate shared library. They are now built and tested there, and no
  shared-library dependency is added.
- **OVStage change events are delivered once.**
  `update_from_ovstage()` now honors the consumed-ordinal cursor that attachment
  positions at its `read_ordinal`: ordinals at or below it are skipped, so a
  fully consumed range is a successful no-op and an overlapping range applies
  only its unread suffix. Object-created notifications for population authored
  after attachment are preserved. `ovphysx_attach_ovstage()` rejects
  `read_ordinal == 0` (`OVPHYSX_API_INVALID_ARGUMENT`); 0 is the runtime's
  internal skip-cursor sentinel and would leave attach replay unguarded.
- **Mimic joints on a D6 joint now act on the authored rotational axis.** The parse
  library encoded the resolved axis as a positional index (0/1/2) rather than the
  `JointAxis` enumerator (`eRotX` is 4), so the engine's axis lookup fell through and
  bound every D6 mimic joint to `PxArticulationAxis::eSWING2` regardless of the
  `PhysxMimicJointAPI:<axis>` instance applied. Nothing was logged on either CPU or
  GPU -- the scene simply simulated the wrong degree of freedom. Revolute and
  prismatic mimic joints, which carry no axis, were unaffected.
- **Fitted child mesh colliders now preserve authored mass frames.**
  `boundingSphere` and `boundingCube` fit offsets still place
  the collision shape, but no longer translate or rotate collider-local center
  of mass or inertia frames a second time during parent-body aggregation. USD
  and ovstage use the same corrected path.
- **The process-private cooked-collider cache cleanup at shutdown now retries
  briefly before giving up.** With `cooked_collider_cache_dir` unset, a cook
  completing immediately before process exit could race the cache
  directory's teardown: `wait_all()` only drains the cook compute queue, not
  the underlying datastore's on-disk write-back, so the last bytes could
  still be trickling out when the process-private temp directory was
  removed, leaving it (or part of it) behind. Cleanup now retries for up to
  ~80 ms, which narrows the window in practice for short write-backs; it
  remains best-effort, and a leftover directory after retries exhaust is now
  logged instead of silently abandoned. No effect on the common case where
  cleanup already succeeds on the first attempt.
- **Simulation-operation polling no longer blocks for completion.** A zero or
  finite `ovphysx_wait_op()` timeout now checks simulation readiness before
  calling the blocking result-finalization path. If the operation is not ready
  before the requested budget, the wait reports `OVPHYSX_API_TIMEOUT`, returns
  the lowest pending operation, and leaves it available for a later wait.
  Poll and finite waits use the generic tracked-operation path; the direct
  single-operation sync fast path remains available only to infinite waits.
  Once the final boundary check observes readiness, result finalization and its
  actual terminal result win even if finalization extends total call duration.
- **Unsealed articulation attachment now fails closed.**
  Unreadable initial articulation/joint schema data returns an error; seal and
  retry. Attribute-scoped seals remain valid.
- **Repeated native destroy rejection is side-effect-free.** A handle that is
  already absent now returns `OVPHYSX_API_ERROR` before teardown, so Python can
  retry after an ambiguous ctypes exception without clearing pending-operation
  state owned by a surviving instance.
- **Getting-started prerequisites now match the shipped packages.**
  Quickstart and Hello World point to the bundled
  `samples/data/` stages and distinguish prebuilt GPU runtime requirements from
  source-build requirements. GPU simulation with a prebuilt wheel or SDK needs
  a driver compatible with CUDA 12.8, not a CUDA Toolkit installation; the
  Toolkit remains a GPU-enabled source-build prerequisite.
- **Optional Python USD authoring now documents its `usd-core` prerequisite.**
  The public Physics Schemas page and shipped
  `ovphysx-usd-authoring` skill now direct external authoring and validation
  tools to install stock `usd-core` before importing `pxr`. It remains a
  tool-owned optional package, not an ovphysx dependency or simulator runtime;
  ovstage populates the authored USD before ovphysx attaches and simulates the
  resulting stage. The Linux aarch64 PyPI limitation and non-Python authoring
  alternatives are also documented.
- **The advertised `OVPHYSX_PHYSX_TYPE_PHYSICS` lookup is now reachable.**
  `ovphysx_get_physx_ptr()` accepts either zero-length
  string representation (`{ NULL, 0 }` or `{ "", 0 }`) for the pathless,
  process-global `PxPhysics` object. A non-empty `PHYSICS` selector now returns
  `OVPHYSX_API_INVALID_ARGUMENT` instead of falling through to
  `OVPHYSX_API_NOT_FOUND`; empty selectors for path-bound types remain invalid.
- **Tensor views are now invalidated when the stage is detached.** Detach releases every PhysX
  object for the stage but told the views nothing: `UsdLoad::detach` disables object-change
  notifications before the release, and `physXDetach` emits `eStopped` only once the simulation
  has been started -- which the `IPhysxSimulation` stepping path never sets. The view kept
  reporting valid while holding raw `PxRigidBody` pointers, so the next `get_transforms()`
  dispatched a virtual call through a freed actor (NvBugs 6583612). The tensors simulation-event
  listener now also handles `ePhysicsObjectsReleased`, which every bulk release emits, covering
  detach, `release_physics_objects()` and `force_load_physics_from_usd()`.
- **GPU tensor reads no longer rebuild the rigid-body GPU-index map on every read.** The map is
  rebuilt by calling `getGPUIndex()` on every body, and a dirty flag exists to skip that when no
  body has been enabled or disabled since the last rebuild. The flag was cleared only after an
  early return taken whenever no index had actually moved — which is the steady-state case — so it
  was cleared only on reads where something *had* changed, and stayed dirty otherwise. The rebuild
  therefore ran on every read of every GPU scene, disable-free or not, and was the single largest
  cost in a read: reading pose and velocity for 163,860 rigid bodies through a tensor binding went
  from ~11.2 ms to ~1.0 ms. Behavior is unchanged — while any body is disabled the flag still stays
  dirty so the map keeps refreshing until every index lands.
- **CPU tensor views no longer leak a `PxArticulationCache` per articulation.**
  `PxArticulationReducedCoordinate::release()` explicitly does not free caches created from it, so
  the view that called `createCache()` owns them, but `~CpuArticulationView` and
  `~CpuRigidBodyView` were empty. Every CPU articulation view leaked one cache per articulation,
  and every CPU rigid-body view one per articulation root, for the lifetime of the process. Both
  destructors now release them, guarded on the PhysX plugin still being loaded since `release()`
  frees through the foundation allocator.
- **Point instancers with `inactiveIds` no longer read out of bounds, and `inactiveIds` is now
  interpreted per the USD spec.** With `inactiveIds` authored, only the prototypes still referenced
  by an active instance were parsed, and their descriptors were appended rather than stored at their
  prototype index — so the instance loop, which indexes by `protoIndices`, read past the end of the
  list and dereferenced a garbage descriptor (NvBugs 6455958). Additionally, `inactiveIds` entries
  were treated as positional instance indices; they are ids into the optional `ids` attribute when
  it is authored, and positional only when it is not. An instancer with `ids = [100, 200]` and
  `inactiveIds = [200]` now deactivates instance 1 instead of writing out of range. Out-of-range
  `inactiveIds` entries and out-of-range `protoIndices` are skipped with a warning.
- **Tensor views are now invalidated when their physics scene is deleted.**
  Destroying the scene notifies subscribers as `ePTScene`, which the simulation
  view ignored -- it kept reporting valid while the GPU path held the freed
  `PxScene` as a raw pointer, so the next read jumped through a stale vtable
  (observed from an on-step callback via `GpuRigidBodyView::getVelocities`;
  NvBugs 6521047). The view now records the scene it is bound to and invalidates
  on its destruction, the same way it already did for bodies, links and shapes.

  Replacing a stage's physics scene is fixed along with it. The scene's cached
  simulation data is now dropped when the scene is destroyed, so a view created
  afterwards is built against the live scene rather than handed buffers, actor
  maps and a CUDA context belonging to the freed one.
- **Ragged articulation views no longer read past an articulation's own DOFs.** A view
  spanning articulations of different sizes reports `getMaxDofs()`/`getMaxLinks()` as the
  maximum over its entries and pads every row out to that width, but the per-articulation
  accessors are bounded by each articulation's own count. Several call sites walked to the
  maximum: the GPU articulation-view constructor queried the metatype for padding columns
  (logging an error per column), and the CPU Coriolis and gravity-compensation readers indexed
  the PhysX articulation cache past its end, producing out-of-bounds reads and sign-flipped
  values. Padding columns now read as `0`, matching the convention used elsewhere in the
  tensor API. `IArticulationView::getUsdDofPath`/`getUsdLinkPath` also returned a pointer into
  a path destroyed on return; the path is now held for the call's lifetime. The padding-null
  contract and the returned buffer's lifetime are documented on the view interfaces.
  Separately, a view mixing fixed- and floating-base articulations is now refused by the
  accessors whose row layout is derived from base type — `getCoriolisAndCentrifugal-
  CompensationForces`, `getGravityCompensationForces` and `getArticulationCentroidalMomentum`.
  They took that layout from the first entry and applied it to all of them, reading a
  fixed-base articulation's cache six values past its end when the first entry was
  floating-base. They now fail with an error naming the remedy: build one view per base type.
  Raggedness in DOF or link count alone is unaffected.
- **Index tensors are now bounded by the view's entry count.** The loop over an index tensor was
  driven entirely by the descriptor's declared shape; nothing cross-checked it against the view,
  and the real allocation size is not knowable from a descriptor. A shape larger than the view's
  entry count therefore walked off the end of the caller's buffer, which is how NvBugs 6504465
  ended up faulting inside the CUDA driver's reserved address range. Every index-taking entry
  point now rejects an oversized index tensor with a logged error, matching how the data tensor
  was already size-checked -- the articulation, rigid-body, GPU deformable-body and
  deformable-material views, the
  property setters implemented on the shared base classes, and the GPU staged setters, whose
  device-to-host copy is sized from the descriptor and so read past the caller's allocation
  before any bound applied.

  **Contract change.** An index tensor may now hold at most as many indices as the view has
  entries. Previously a CPU view silently accepted an oversized one, processing the in-range
  indices and skipping the rest, so a caller passing more indices than entities -- duplicates, or
  a deliberately padded index array -- worked and will now fail with an error. That set can never
  be legitimate: it is exactly the set these functions build for themselves when no index tensor
  is supplied. Out-of-range index *values* within a correctly sized tensor are unaffected and
  continue to be skipped.
- **Character controllers now load and update on an OVStage-backed stage.** A prim
  carrying `PhysxCharacterControllerAPI` had never produced a controller on any
  non-USD parse source: the OVStage walker had no `emitCct`, and its scan result's
  `ccts` list was not forwarded onto the scanned stage. An empty list is
  indistinguishable from a stage that authors no controller, so nothing reported the
  loss. Post-attach `physxCharacterController:*` property edits were dropped for the
  same reason on the change-feed side - `PhysxCharacterControllerAPI` was missing from
  the known-physics family lists that decide whether a path's changes are delivered at
  all, in the initial seed, the structural refresh, and the cached read path alike.
  A controller on a capsule with no authored `radius` now also falls back to the
  `UsdGeomCapsule` schema value of `0.5` rather than `1.0`, which had made an
  unauthored controller twice as wide as the USD path wherever the source has no
  backing USD stage to resolve the fallback through.
- **No spurious default physics scene on a scoped OVStage scan.** "This stage authors
  no `PhysicsScene`" is a whole-stage fact, but the OVStage walker concluded it from any
  scan whose result held no scene - including an incremental re-scan rooted at a single
  newly added prim. Each such scan published an extra synthetic
  `/__defaultPhysicsScene__`, reported as an additional object-created notification where
  the USD path reports none. Synthesis is now gated on a genuinely unscoped scan; a
  scoped initial load still gets the loader's own default-scene fallback. A scan scoped
  purely by excluded subtrees counts as scoped for the same reason, which closes the same
  hole on the direct scan entry point (stage loading itself always scans from the
  pseudo-root, so it was never reachable that way).
- **`UsdGeomPoints` prims answer OVStage type-identity questions consistently.**
  `Points` was listed under `Xformable` and `PointBased` but not under `Gprim`, so the
  same prim answered true to two of its base types and false to the third. A points
  cloud therefore satisfied the deformable-body root gate (`Xformable` and not
  `Gprim`) and was admitted as a deformable root. Note this does not give a points
  cloud a collision shape on OVStage - that dispatch does not exist yet.
- **Stale SDF views are rejected before any GPU work.**
  `evaluate()` on an `SdfView` whose stage was torn down by `reset_stage()` or
  `detach_ovstage()` already raised `RuntimeError` instead of crashing,
  but the validity check ran after the implicit warmup, so a stale
  handle could still trigger a simulation step against the newly attached stage
  before the error was returned. The handle is now resolved and validated first,
  and the stale call has no side effects.
- **Deformable skinning synchronization no longer crashes when the CUDA context
  is unavailable.** It now warns once and skips.
- **Clone transform documentation now matches the existing target-root placement
  behavior.** Each `anchor_transforms` entry is the final
  absolute world pose of the exact target subtree root. Descendants keep their
  poses relative to the source subtree root.
- **OmniPVD C-API recording config is independent of config-entry order.**
  `ovphysx_create_instance()` now applies the recording
  directory before either the typed or raw Carbonite output-enable trigger when
  reusing the process-wide runtime, so `[OUTPUT_ENABLED, OVD_RECORDING_DIRECTORY]`
  produces the requested `.ovd` capture just like the reverse order.
- **Bounding-sphere and bounding-cube colliders now honor the prim's world
  scale.** Both approximations copied unscaled mesh points into the merged mesh
  description, and unlike the cooked-mesh path there is no later scale to apply,
  so the resulting sphere or box was sized in mesh-local units. A gprim scaled by
  0.01 produced a collider 100 times too large — correctly placed and rotated,
  but overlapping everything around it from the first step.
- **Live transform edits reach colliders inside an instanceable prim.** Editing
  the transform of an instanceable prim's own root never reached the PhysX
  collider for its referenced subtree once the scene had cooked, so the collider
  stayed at its first-cook pose. The descendant walk now descends into instance
  proxies, and a static actor's pose is no longer re-derived from a source prim
  that resolves inside the shared prototype, which discarded the
  instance-specific pose. Affects assets whose instance root carries the
  collision API while the geometry lives on nested prims in the prototype.
- **Object-deletion callbacks now run while a simulation is shutting down.**
  Tensor simulation views subscribed with the default notification gate, which
  is deliberately closed as a simulation ends, so the handler that invalidates a
  view when its scene is released never ran on that path. Views could survive
  their scene until the next explicit detach.
- **Crash when a GPU scene was created while another was being torn down.** The
  scene-release path moved its scene map aside before deleting the scenes, so a
  GPU setup re-entering during that loop saw no live scenes and was free to
  release and recreate the CUDA context manager that the scenes still held. The
  round-robin cursor that picks a context manager was also used without being
  reduced, so it could index past the end of its vector and return a non-null
  garbage pointer that passed every null check.
- **Crash from a mass update on an unresolved object.** A rigid-body mass update
  carrying the invalid object id indexed the internal record array with it,
  reading far past the end of the array. The invalid id is now skipped, the
  object-creation result is checked before it is recorded, and entry ids are
  bounds-checked before they become record references.
- **Crash from an actor released during an active-actor callback.** Actors
  released while active-actor results were being processed left stale references
  behind. Released actors are now tracked and skipped.
- **Crash when a body's simulation owner changed and the body was then
  removed.** Changing the owner reassigned the actor's scene pointer without
  moving it between the scenes' actor lists, so the removal searched the wrong
  list, found nothing, and freed the actor anyway — leaving a dangling pointer
  that the next simulation reset dereferenced.
- **Crash when OmniPVD output was toggled with a scene attached.** Changing the
  recording setting recreated the PhysX SDK object even though live scenes still
  belonged to it. The recreation is now deferred while any scene is attached,
  and the requested setting takes effect at the next attach.
- **Crash from concurrent profile-statistics collection during replication.**
  Replication opened a profile scope in two lambdas that run concurrently, and
  both appended to the same unsynchronized statistics vector. Profiling is on by
  default, so this affected every session that cloned.
- **Removing the default-simulator setting no longer crashes.** A null value for
  that setting is handled instead of dereferenced.
- **Getting-started examples now step synchronously.** The Python quickstart,
  the READMEs and the shipped Hello World sample used the asynchronous
  `step()` for a single-step workflow, so Hello World could report success
  before the step finished and cleanup could lose the detailed operation error.
  They now use `step_sync()`. The advanced async samples are unchanged.
- **The Python tensor-bindings sample now shows a link that actually moves.**
  It applied articulation velocity targets but printed link 0, which the bundled
  fixture fixes to the world, so the output looked static while all 14 driven
  links moved. It now reports the chain tip, like the C sample, and checks that
  the displayed pose really changed before reporting success. The misleading
  X-Euler line is gone — the fixture's revolute joints turn about Y.

### Removed
- **pkg-config support (`ovphysx.pc`).** The generated `.pc` file could not
  describe the whole dependency chain: the public headers include ovstage
  headers, and ovstage publishes no pkg-config metadata, so a consumer still had
  to inject ovstage's include and library paths by hand. `find_package(ovphysx)`
  resolves the chain through `find_dependency(ovstage)` and is the supported
  integration path.
- **`ovphysx.dlpack.ManagedDLTensor` and its DLPack capsule provider.** The Python
  output read returns `warp.array`, so the hand-written wrapper and its
  `__dlpack__()` capsule export are gone rather than kept as a second result type
  The ctypes mirror of the DLPack structs remains for the
  compatibility APIs that still type their Python arguments as DLPack.

### Known limitations
- **Point-instancer rigid bodies are not available to TensorBindingsAPI.**
  `UsdGeom.PointInstancer` instances are simulated and available through the
  ovstage `RIGID_BODY` output-read path, but they are not exposed as individual
  rigid-body tensor-binding rows. With the default empty-binding behavior, a
  binding that targets only the point instancer has count zero. Read instance
  state through output read; control it by authoring the point-instancer arrays
  through ovstage.
