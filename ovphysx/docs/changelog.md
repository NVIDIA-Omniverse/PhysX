<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Changelog

All notable changes to `ovphysx` are documented in this file.

## [0.5.11] - Date 2026-08-21

### Changed

- **OVStage-backed articulation link and DOF indices now use stable path-derived ordering.**
  This fixes run-to-run tensor-column drift, but it may change which link or DOF
  occupies a given tensor column compared with 0.5.10. OVStage ordering is not
  guaranteed to match USD backend authoring order; consumers needing identity
  across upgrades or backends should use reported paths or names.
- **Windows source and SDK sample builds now require CMake 4.1 or newer.**
  Linux is unchanged at CMake 3.16 or newer. The prerequisite lists in the
  README, the quickstart, and the source-link build tutorial state the split.

### Fixed

- **OVStage attachment now honors explicit CUDA device selection.**
  `active_cuda_gpus` applies on every attach path, so a CUDA context left
  current by a renderer can no longer make PhysX pick a different device than
  the one you asked for. A single explicit ordinal also disables stale
  multi-GPU scene distribution for that attachment.
- **GPU-selection documentation matches the runtime contract.**
  Empty `active_cuda_gpus` means PhysX automatic selection, while GPU tensor
  callers should pass an explicit ordinal and reuse it in their DLPack device
  metadata. This is a documentation correction, not a runtime change, and
  supersedes the older 0.4.1 note that grouped empty input with GPU 0.
- **Wildcard tensor bindings now reach runtime clones through unauthored
  intermediate target paths.** Cloning `/World/envs/env0/robot` directly to
  `/World/envs/env1/robot` creates runtime physics without authoring
  `/World/envs/env1` in USD. Patterns such as `/World/envs/env*/robot` now match
  the source and every clone. Removing an authored ancestor also removes the
  clones below it, instead of leaving orphaned physics behind. Exact-path
  bindings work as before.
- **Collider cooking no longer writes `[Error] [omni.datastore]` lines when
  `cooked_collider_cache_dir` is unset.** The data store used to default to a
  path next to the Python interpreter, which a normal Linux venv cannot write.
  Simulation was correct, but every run printed errors. Cooking now uses a
  private directory under the OS temp directory and deletes it at process exit,
  so nothing is persisted. A configured directory that cannot be created or
  written falls back the same way. The configured directory takes effect at the
  first runtime bootstrap; changing it later in the same process does nothing.

## [0.5.10] - Date 2026-08-05

> **C ABI break.** `ovphysx_debug_render_set_parameter()` changes its third
> parameter from `bool` to `float`. A binary built against an earlier 0.5 header
> must be recompiled and relinked; source callers should pass `1.0f` for enabled
> and `0.0f` for disabled. This is independent of the `ovphysx_clone` ABI change
> introduced in 0.5.4.

### Added
- Added `ovphysx_debug_render_set_scope_tokens` for exact OVStage path-handle
  debug-visualization filtering. The scope API has no string-prefix or path-
  interning variant: callers expand a hierarchy to its exact object set and use
  the Stage's `path_dictionary_instance_t` to create `ovx_primpath_t` handles.
  Matching uses the runtime object's canonical source key, so it does not require
  exposed PhysX actor names. Handles require no individual release, remain valid
  only for their originating Stage dictionary, and the scope is cleared on detach.
- **Disabled debug rendering avoids per-step overhead.** While disabled (the
  default), debug visualization performs no per-step CPU or GPU visualization
  work, GPU-to-host visualization copies, or scratch-allocation growth. Scratch
  capacity acquired while enabled may remain reserved until scene teardown.
- **`OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL` tensor type.** New read/write `[N]` uint8/bool tensor that toggles `PxActorFlag::eDISABLE_GRAVITY` on the underlying `PxRigidActor` at runtime. Wired to `IRigidBodyView::set/getDisableGravities` (+ indexed and masked writes). Python: `TensorType.RIGID_BODY_DISABLE_GRAVITY`. Reads and writes reflect live PhysX actor flags only (no USD I/O). Runtime gravity suppression validated on CPU and GPU DirectGPU (`GpuRigidBodyDisableGravitySuppressesFall`).
- **`OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL` tensor type.** New read/write `[N, L]` uint8/bool tensor that toggles `PxActorFlag::eDISABLE_GRAVITY` per articulation link at runtime. Wired to `IArticulationView::set/getDisableGravities` (+ indexed and masked writes). Python: `TensorType.ARTICULATION_BODY_DISABLE_GRAVITY`. Zero-padded link columns beyond `numLinks` are ignored on write. Runtime gravity suppression validated on CPU and GPU DirectGPU (`GpuArticulationDisableGravitySuppressesFall`).
- **`OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8` tensor type.** New read-only `[N, D]` uint8 tensor reporting the drive type per articulation DOF (`0`=none, `1`=force, `2`=acceleration). Wired to `IArticulationView::getDriveTypes`. Python: `TensorType.ARTICULATION_DOF_DRIVE_TYPE`. Reads reflect the live `PxArticulationDrive::driveType` rather than authored USD. Padded DOF columns beyond `numDofs` now read as `0`; `BaseArticulationView::getDriveTypes` previously left them at whatever the caller's buffer held.

### Changed
- **The SDK package no longer ships ovstage.** The ovstage headers, library,
  runtime plugin tree, and bundled `python/ovstage` package are gone from the
  `.zip` / `.tar.gz`. Native SDK users download the matching OVStage archive
  separately and add both package roots to `CMAKE_PREFIX_PATH`. Source builds
  still fetch OVStage automatically, and the ovphysx wheel still depends on the
  separate ovstage wheel.
- **`ovphysx_debug_render_set_parameter()` takes a value.** 0 disables the
  geometry type and a positive value enables it. For parameters whose PhysX
  semantics define a magnitude, that value scales the geometry together with the
  master scale. `CONTACT_POINT` and `FRICTION_POINT` use a positive value only as
  an enable gate; their marker size follows the master scale. Geometry with an
  inherent shape is drawn at that shape's dimensions. The getter returns the last
  requested value. The C ABI migration is described in the release note above;
  the interim `_set_parameter_value()` is gone.
- **`PhysX.read()` / `PhysX.read_tokens()` now raise `TypeError` on an unsupported
  output-column dtype** instead of silently reinterpreting it as `float32`. Every
  dtype the runtime emits today decodes as before — float32/64, int8/32/64,
  uint8/32/64, and `kDLBool` (bits=8) — so only a genuinely unmapped DLPack
  `(code, bits)` raises, making a future native dtype-encoding drift fail loudly
  rather than return corrupted values.
- **Contact detail reads now report undersized flat buffers.**
  `ovphysx_read_contact_data()`, `ovphysx_read_friction_data()`, and
  `ovphysx_read_raw_contact_data()` return `OVPHYSX_API_BUFFER_TOO_SMALL`
  instead of success when `max_contact_data_count` cannot hold all entries.
  Count and start-index tensors contain the required layout for sizing a
  recreated binding on subsequent simulation steps; the overflowing read's
  payload tensors are not valid.

### Fixed
- **`basic-workflow` C sample compiles again (NVBug 6557378).** The fenced sample
  waited on enqueue results via `.op_id`, but the ovstage members are named
  `.op_index`. The two wait calls now use `.op_index`.
- **Shipped skills and docs now seal the ovstage write floor before attach
  (NVBug 6557378).** `open_usd()` does not advance the write floor;
  `attach_ovstage()` reads a sealed ordinal. Four skills
  (`clone-environments`, `tensor-bindings-cpu`, `tensor-bindings-gpu`,
  `ovphysx-usd-authoring`), the quickstart, developer-guide snippets,
  overview/physics-scene prose, and top-level README examples omitted
  `advance_write_floor()`, so following them as published attached
  successfully but silently dropped every articulation and joint while
  rigid bodies still loaded. The Python `attach_ovstage` docstring also
  misdescribed that failure as an empty scene; it now states the real
  partial-parse behaviour. Product samples and `basic-workflow` already
  had the seal call.
- **The documented `.usda` scene template now sets the mass unit (NVBug 6557394 /
  OMPE-104387).** The template in `simulation_setup/physics_scene.md` authored
  `kilogramsPerMass`, which is not a USD stage-metadata key: USD accepted the
  layer and dropped the field without a warning, so a reader who copied the
  template kept the default mass unit and every authored mass was off by the
  intended scale factor. The key is `kilogramsPerUnit`, as the same page already
  stated in prose. The reference scene the page links, `simple_physics_scene.usda`,
  carried the same invalid key and is corrected too.
- **`ovphysx_step_sync()` no longer hangs after repeated runtime prim additions
  (OMPE-104209).** Scoped OVStage updates no longer create duplicate default
  physics scenes.
- **Windows `--devphysx` source builds now compile with MSVC.** Descriptor
  deleter size and alignment constants use static storage duration, keeping the
  required deleter lambdas captureless without triggering MSVC C3493. Runtime
  values and behavior are unchanged.
- **Wheel builds no longer depend on developer-local state (NVBug 6543059 /
  OMPE-103947).** `cmake -P scripts/build_wheel.cmake` staged the whole
  `tests/python_samples` tree, including a gitignored `.venv` whose
  `bin/python` symlinks an interpreter outside the repository, so the wheel step
  aborted on any machine that had run the python-samples tests. That state is
  now excluded from the copy. The Linux prerequisites also name the `file`
  utility and `binutils`, which the install step requires, and the errors raised
  when they are missing name the packages to install.
- **Contact-binding docs now state the `PhysxContactReportAPI` requirement (NVBug
  6543101 / OMPE-103946).** The tutorial, the developer guide, the `ovphysx.h`
  Doxygen block, and the `create_contact_binding()` docstring all claimed no USD
  authoring was needed beyond the rigid bodies themselves. A contact binding
  additionally requires `PhysxContactReportAPI` on each authored USD prim named by
  `sensor_patterns` - on that exact prim, not a parent body or child collider - so
  a scene authored as documented produced no binding at all. Filter prims need no
  extra schema, and runtime-only clones inherit contact reporting from the source
  actor. The creation error, previously `no matching sensors?`, now leads with a
  neutral `no sensor entries were produced` and lists the common causes: an
  authored sensor prim without the schema, a pattern matching no object, and
  `filter_patterns` resolving to the wrong count.
- **Documented the codeless schema registration ordering hazard (NVBug 6530141
  / OMPE-103543).** `Plug.Registry().RegisterPlugins()` can only expose the
  codeless PhysX USD schemas in a process that has not yet opened a stage or
  queried USD's schema registry; that registry is built once, on first access,
  and a late call fails silently — the plugin count and
  `Tf.Type.FindByName` still report success while `ApplyAPI` raises. The "stock
  `usd-core`" section of the physics-schemas guide now states this and documents
  presetting `PXR_PLUGINPATH_NAME` before the host process launches as the
  supported route for DCC hosts and other processes that have already
  initialised USD. This is OpenUSD's registry lifecycle, not an ovphysx defect;
  the behavior is unchanged. A new `schema_registry_ordering.py` regression
  sample pins both the failing and the supported ordering.
- **Windows `--devphysx` builds no longer fail when the MSVC toolchain path
  contains spaces and 8.3 short names are available (NVBug 6543145 /
  OMPE-103943).** Under the Ninja generator, CMake 8.3-shortens the `-ccbin` path
  it passes to nvcc while `PATH` kept the long form, so nvcc rejected the
  mismatch with `cl.exe in PATH ... is different than one specified with -ccbin`
  and CUDA configuration failed. This hit source-drop builds using a local Visual
  Studio under `C:\Program Files\...`. On a volume with 8.3 name generation
  disabled the build now warns and asks you to move the toolchain to a path
  without spaces. Build with `build.bat --rebuild` when moving onto this
  version: CMake never re-detects the cached compiler, so an existing Windows
  build directory keeps the old path spelling. `GENERATOR` is also
  whitespace-tolerant.
- **Tensor binding caches no longer retain foreign DLPack capsules.** Repeated
  reads and writes now cache a Python-owned tensor descriptor for NumPy,
  PyTorch, and Warp storage, while direct `DLTensor` inputs retain their
  caller-owned descriptor and other DLPack providers are reacquired per call.
  This prevents a retained capsule from invoking a producer callback after its
  Python module has shut down.
- **Synchronous contact-force reads now use the current timestep (NVBug
  6519970 / OMPE-103224).** `ovphysx_step_sync()` and
  `ovphysx_step_n_sync()` now cache their timestep after successful completion,
  matching `ovphysx_step()` plus a wait. Contact binding reads therefore divide
  contact and friction impulses by the timestep that produced them instead of
  the initializer `1.0` or a stale earlier async-step value.
- **Articulation velocity-dependent dynamics queries now use live state
  (NVBug 6520292).** `ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE` and the bias
  column of `ARTICULATION_CENTROIDAL_MOMENTUM` previously used joint velocities
  retained in each CPU articulation view's creation-time cache. Reads through a
  persistent binding could therefore remain zero or stale after velocities changed
  through another binding or a simulation step. Both queries now refresh their
  joint velocities before computing. This affected CPU simulation and GPU dynamics
  with CPU tensor readback; the genuine DirectGPU tensor path already reads live
  device state and is unchanged.
- **Opaque object handles no longer alias across object kinds or instances
  (NVBug 6504951 / OMPE-102857).** Instance, tensor-binding, contact-binding and
  SDF-view handles previously each counted from 1 inside their own owner, so an
  instance handle and that instance's first tensor binding were both the number
  1, and a binding handle outlived by its instance matched the first binding of
  the next instance. For a valid `ovphysx_handle_t instance` and an
  `ovphysx_tensor_spec_t out_spec`,
  `ovphysx_get_tensor_binding_spec(instance, instance, &out_spec)` therefore
  returned an `ovphysx_result_t` with `.status == OVPHYSX_API_SUCCESS` and
  populated `out_spec` from the wrongly resolved first tensor binding. All four
  now come from one process-wide, never-reused nonzero sequence, so that
  specific lookup with a valid instance misses the tensor-binding map and its
  returned `ovphysx_result_t` has `.status == OVPHYSX_API_NOT_FOUND`. No public
  API signature, public struct layout, C ABI,
  or error-enum value changed; handles remain opaque and
  `OVPHYSX_INVALID_HANDLE` (0) remains the invalid sentinel. Numeric allocation
  is observably different: handle values are no longer allocated per object
  kind, and a particular kind's first handle is no longer guaranteed to be 1.
- **Python DLPack data type codes now support integer conversion (NVBug
  6507889).** Calling `int(tensor.dtype.code)` returns the numeric
  `DLDataTypeCode` value instead of trying to parse its raw `ctypes` bytes.
- **Fixed-base Jacobian row count corrected in the Tensor Bindings tutorial
  (NVBug 6504947 / OMPE-102860).** The tutorial now documents
  `R=(L-1)*6, C=D`, matching the public C header and runtime; it previously
  stated `R=L*6, C=D`.
- **Documented that changing `--devphysx` / `--devschema` requires a clean
  rebuild (NVBug 6542923 / OMPE-103941).** The flag combination selects a build
  flavor, and incremental builds across a flavor change are not supported:
  `--devphysx` caches `PHYSX_SDK_DIR` pointing at the local `physx/` source tree,
  so a later build without it fails with a `PhysXGpu_64.dll` copy error, "No such
  file or directory". The build options, `--help` output, and troubleshooting
  docs now state that `--rebuild` is required.
- **Population domains (`ALL` / `PHYSICS | RENDERING`) are now documented
  (NVBug 6532997 / OMPE-103566).** `PopulationDomain` is an OR-combinable
  bitmask; ovstage's `open_usd()` default is `RENDERING` (physics off). For
  arbitrary USD — including headless ovphysx-only apps — populate `ALL`
  (equivalently `PHYSICS | RENDERING`): under the currently pinned ovstage,
  `PHYSICS` alone can silently omit colliders that sit under native USD
  scene-graph instances. `PHYSICS` alone remains valid only for content known
  not to use native instancing (such as the shipped samples). See
  [Ovstage Integration](ovstage_integration.md#population-domains).
- **Standalone startup no longer reports 11 false missing-plugin warnings
  (NVBug 6504272).** Static Carbonite plugin registration now reuses the
  already-collected registry state instead of probing expected-absent plugins
  with the diagnostic `getPluginDesc()` API.
- **OVStage-backed scans retain the source that minted descriptor handles.**
  Token-valued descriptors now resolve correctly after ingestion, preventing
  spatial-tendon crashes with same-link intermediate attachments.
- **OVStage-backed joint frames now match native USD ingestion for scaled bodies.**
  Joint anchors are converted from relationship targets into the resolved body
  frame and body scale is baked into their translation, preventing attach-time
  shifts in scaled articulations.

### Performance
- **Significantly faster stage-attach for OVStage-backed scenes.** Attaching
  large, heavily-populated OVStage scenes is now substantially quicker, on par
  with the equivalent USD load. No behavioral change.
- **Render-only scene-graph prototypes no longer dominate OVStage attach time
  in common leaf-collider scenes (NVBug 6532970 / OMPE-103564).** OVPhysX
  queries instance-root mappings only for prototype roots proven to back
  physics collision shapes, rather than querying every prototype in the stage.
  Ambiguous non-leaf colliders conservatively retain complete expansion until
  OVStage provides the general batched resolver tracked by OMPE-100947. Public
  type and geometry reads for render-only instance proxies remain available
  through a cached, post-attach reverse lookup. Physics behavior and public APIs
  are unchanged.

---

## [0.5.9] - 2026-07-21

### Fixed
- **Linux wheel startup reuses already-loaded compatible OVStage/USD SONAMEs.**
  This prevents duplicate static USD registration when OVRTX-first applications
  later import ovphysx.

---

## [0.5.8] - 2026-07-21

### Fixed
- **`ARTICULATION_JOINT` output read now returns every unlocked joint DOF, with or
  without `JointStateAPI` (NvBugs 6481083 / OMPE-102219).** Previously the
  `jointPosition` / `jointVelocity` read only emitted a group for joint axes carrying
  an authored `JointStateAPI`, so a valid articulation whose joints had none returned
  zero groups, and a joint with `JointStateAPI` on only a subset of its DOF dropped the
  remaining unlocked DOF. The read now enumerates the joint's actual unlocked
  reduced-coordinate DOF (angular axes in degrees, linear in base units), honoring an
  authored per-axis `convertToDegrees` flag when present. Under `eENABLE_DIRECT_GPU_API`
  the per-DOF state is sourced live from the direct-GPU API instead of the frozen
  CPU-side accessor.
- **Fixed-base articulations reject centroidal-momentum tensor bindings at
  creation (NVBugs 6481094 / OMPE-102210).** `create_tensor_binding` for
  `ARTICULATION_CENTROIDAL_MOMENTUM` now fails with `INVALID_ARGUMENT` (raises
  `RuntimeError` in Python) when any matched articulation is fixed-base, instead
  of accepting the binding and only surfacing the failure later at read time.
  Centroidal momentum is defined for floating-base articulations only. The check
  inspects every matched articulation, so a pattern that resolves to a mix of
  fixed- and floating-base articulations is rejected too. Empty matches remain
  valid empty bindings.
- **Consumed operation indices now honor single-use wait semantics
  (NVBugs 6473891, 6481089, 6481092).** Explicit waits are tracked separately
  from internal synchronization, so a second blocking or polling `wait_op()`
  reports `NOT_FOUND` (and raises `RuntimeError` in Python) as documented, even
  after later stream operations synchronize pending work. The simulation wait
  fast path also defensively permits only one competing waiter to consume an
  index; callers must still serialize same-instance API use.
- **Wheel-bundled `OVPHYSX_LIB` selection preserves the paired OVStage runtime
  (NVBugs 6481086).** Setting the override to the installed wheel's own
  `libovphysx.so` or `ovphysx.dll` now retains cross-wheel dependency setup.
  Overrides that select an external SDK remain isolated from wheel dependencies.

---

## [0.5.7] - 2026-07-21

> **Versioned ABI break introduced in 0.5.4.** The 0.5 release line changes exported C ABI
> (`ovphysx_clone` gains a sixth `env_ids` parameter — see the cloning entry), so the wheel/SDK
> version was bumped to `0.5.4`. A binary built against a `0.5.3`-or-earlier header must be
> recompiled, and can pin `<0.5.4` to stay on the old
> five-argument surface. The internal `IPhysxReplicator::replicate` / `isReplicatorStage` /
> `IPhysxSimulation::cloneEnvironments` function-table shapes also change, but ovruntime is linked
> **statically** into ovphysx, so there is no independently-versioned ovruntime binary and this
> single `0.5.4` bump covers the whole shipped surface.

### Changed
- **Windows PE binaries codesigned before packaging.** The Windows build job now
  runs `repo_codesign` against `_install/` after install and before SDK/wheel/packman
  packaging, so all distributed `.dll` and `.pyd` files carry a production
  Authenticode signature.  The packman `.7z` is signed during packaging via
  `sign_binary_files` in `repo.toml`.  Linux `.so` files are not affected
  (`repo_codesign` is a PE/Authenticode-only tool).
- **`PhysX.overlap()` now raises `ValueError` for `SceneQueryMode.CLOSEST`.**
  CLOSEST has no meaning for overlap queries (no ray or sweep direction). The
  C API still treats CLOSEST as ALL; only the Python API rejects the mode.
  Closes NVBugs 6172863.
- **Contact report batched buffers are reclaimed on simulation reset/detach (NVBugs 6172770).**
  `PhysXScene` now deletes its owned `ContactReport` during teardown (previously
  leaked peak-capacity vectors). Documented in the developer guide.
- **ovstage compatibility pin centralized at `0.1.0.346039`.** ovstage is fetched as
  a single self-contained wheel (C++ SDK + python package) resolved from a PEP 503
  index selected by `fetch_ovstage_release.py --source` (default `internal`; the
  open-source drop uses the default public PyPI, pypi.org).

### Added
- **Documented x86_64 AVX CPU requirement and fail-fast init check (NVBugs 6447187).**
  Pre-built Linux and Windows x86_64 binaries require AVX; Linux aarch64 is
  unaffected. README, overview, quickstart, PyPI metadata, and agent guidance
  now state the requirement. `ovphysx_initialize()` (and Python `PhysX()`
  construction) now returns a clear error when x86_64 hosts lack AVX hardware/OS
  support instead of failing later with SIGILL at the first physics step. No
  non-AVX fallback is provided.
- **Simulation Setup, Physics Schemas, and Guides documentation.** New docs
  sections covering the physics content ovphysx simulates and how to author it in
  USD: `docs/simulation_setup/` (physics scene, colliders, rigid bodies, joints,
  articulations, deformables, particles), `docs/physics_schemas.md` (schema
  layers and codeless PhysX schema registration), and `docs/guides/`
  (performance, collision tuning, articulation stability, gripper tuning, known
  limitations). Adapted from the Omni PhysX developer guide and scoped to the
  ovphysx public API (codeless PhysX schemas + `PhysXConfig`).
- **`ovphysx-output-read` skill.** New agent skill under `skills/` for the
  ovstage-native physics output query/read API, available since ovphysx 0.5.0,
  covering Python and C reads,
  borrowed group lifetime, `ALL` versus `ACTIVE` scope, and closed-loop ovstage
  write-back with disjoint control/output ordinal lanes. Bundled Python, C, and
  closed-loop references keep standalone agent installs self-contained. Ships in
  the wheel and SDK with product-owned behavior and routing evals.
- **`ovphysx-usd-authoring` skill.** New agent/human skill under `skills/` for
  authoring USD physics content that ovphysx loads and simulates (physics scene,
  rigid bodies, colliders, mass), covering both the hand-authored `.usda` route
  and the Python route (stock `usd-core`; PhysX-specific attributes via the
  codeless schemas). Ships in the wheel and SDK alongside the existing skills.
- **TensorBindingsAPI deformable material extended properties.** Four additional
  deformable material tensor types: `DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32`
  (133, volume + surface), `DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32` (134),
  `DEFORMABLE_MATERIAL_THICKNESS_F32` (135), and
  `DEFORMABLE_MATERIAL_BENDING_DAMPING_F32` (136). The bending/thickness
  properties are surface-material-only -- reads return 0.0 for volume material
  entries and writes are silently skipped. Requires `IDeformableMaterialView`
  extension via new `getSurfaceProperty`/`setSurfaceProperty` helpers in
  `BaseDeformableMaterialView`.
- **SDF shape evaluation API.** New C `ovphysx_create_sdf_view()`,
  `ovphysx_evaluate_sdf()`, `ovphysx_destroy_sdf_view()`, and Python
  `PhysX.create_sdf_view()` / `SdfView.evaluate()`. Evaluates signed distance
  fields of PhysX collision shapes at caller-supplied query points. Input shape
  `[N, Q, 3]`, output shape `[N, Q, 4]` (gradient xyz + distance). GPU instances
  only (CPU SDF evaluation is not yet implemented). Uses the existing
  `ISdfShapeView` tensor backend.
- **Environment cloning API is now replicator-backed (Fabric-free).** `ovphysx_clone(handle,
  source_path_in_usd, target_paths[], num_target_paths, parent_transforms, env_ids)` (C) +
  Python `PhysX.clone(source_path, target_paths, parent_transforms=None, env_ids=None)` +
  experimental `PhysX::clone` keep the pre-0.5 clone *semantics* (the trailing optional `env_ids`
  is a versioned C ABI addition -- see the env-ids entry below), but the implementation now drives the PhysX SDK replicator
  (binary serialization) instead of the removed Fabric scenegraph copy -- so cloned articulations
  are **real articulations** (what IsaacLab-style batched RL needs), with no Fabric involved. Each
  copy is created at the caller's `target_paths[i]`; `parent_transforms[i]` (a flat
  `[num_target_paths * 7]` array: position + imaginary-first quaternion) positions the copy's
  parent, and every cloned body keeps its pose relative to the source's parent, so an at-origin
  source lands each body exactly at `parent_transforms[i]` (NULL co-locates every copy on the
  source pose). Behavior change vs the old Fabric clone: the clones exist as **runtime
  physics only** -- no USD/scenegraph prims are authored (scenegraph duplication is now an ovstage
  responsibility, `ovstage_clone` + instancing). env-id cross-environment collision filtering
  follows the `/ovphysx/clone/useEnvIds` setting (default on); with explicit per-clone transforms
  the copies are already spatially separated, so it is an optional add-on. Replication executes
  inline and returns an already-complete `op_index`; `wait_op()` remains valid and returns
  immediately (IsaacLab ClonePlan contract). Must be called before GPU
  warmup / the first step. Path-string API (no USD type crosses the boundary). Restores the
  `Lab.cartpole_*` / `Lab.anymal_*` GPU benchmarks. The clone's replicator registration is one-shot
  (released as soon as the clone completes, even on failure), so a later `reset_stage()` -> reattach
  -> clone works and no replicator state leaks; a clone that throws internally is reported as an error
  instead of crossing the C boundary. Multiple `clone()` calls on one attach are env-id-safe: env-ids are
  offset per batch, so distinct environments never alias to one id under GPU broadphase. When one
  logical environment is assembled from **several** clone calls (a heterogeneous ClonePlan: one call
  per source row), pass the new optional per-target `env_ids` (C `const uint32_t*`, Python
  `env_ids=[...]`, C++ `envIds`) with the same ids in every call: the same logical id always maps to
  the same runtime environment (`env_ids[i] + 1`; 0 stays the source's), so `/env1/Robot` and
  `/env1/Object` cloned by different calls keep colliding with each other while staying isolated
  from other environments — per-call automatic numbering would silently stop same-environment
  contacts. A later `env_ids=NULL` call numbers past every explicitly-placed id, so mixed usage
  cannot alias. `env_ids` values must be < `0x00FFFFFF` (PhysX supports at most 1<<24 environments
  and the runtime id is `env_ids[i] + 1`); the runtime also checks `setEnvironmentID`'s result and
  guards the automatic id base against exhaustion, warning rather than silently disabling isolation.
  **Breaking (C ABI):** `ovphysx_clone` gains the trailing `env_ids` parameter — existing C callers
  must add `NULL` and relink; the ovphysx wheel/SDK version is bumped to `0.5.4` for this (see the
  release-header note above).
  Python/C++ callers are unaffected (optional keyword/defaulted argument). Invalid
  targets are rejected up front: empty, equal to the source, duplicated in one call, or already
  cloned on the current attach. A rejected clone reports its error synchronously with no dangling
  operation, so a later attach / reset / clone is unaffected. Targets carrying an embedded NUL are
  also rejected (the C-string seam would otherwise truncate them to a different path, e.g. back to
  the source). A target that is already populated with physics from the initial parse (a real USD
  subtree carrying physics, not just a prior-clone target) is rejected by a runtime ObjectDb
  subtree check before the replicator runs, so cloning onto it can no longer add duplicate actors.
  The experimental C++ `PhysX::clone` completes the clone synchronously and now
  surfaces the operation index through an optional `outOpIndex` out-param, matching the C/Python
  forms. The Python wrapper also validates each
  `parent_transforms` entry is exactly seven finite floats (short/long tuples are rejected rather
  than causing an out-of-bounds native read or shifted poses). Co-located clones (null
  `parent_transforms`) are fully env-id-isolated: `attach_ovstage` enables creation-time
  environment-id assignment (`/physics/replicatorEnvIdsOnAttach`, following
  `/ovphysx/clone/useEnvIds`), so on GPU-dynamics + GPU-broadphase scenes the source environment's
  **dynamic** bodies and articulations are created holding environment id 0 (clones get 1..N) and
  never collide with their co-located copies. Ids are assigned at body creation — live scene
  objects are never removed and re-added, which corrupts DirectGPU scenes. **Static** bodies keep
  the collide-with-all default (`PX_INVALID_U32`) — statics never collide with each other, so this
  keeps a shared static ground plane colliding with every environment, so cloned envs keep resting
  on it. (Known limitation: a co-located clone stack whose envs each carry their own static
  obstacle is not isolated from those overlapping statics — use explicit transforms for that
  layout; a shared *dynamic* object without a scene-partition primvar lands in env 0.)

### Known limitations
- **Point-instancer rigid bodies are not available to TensorBindingsAPI.**
  `UsdGeom.PointInstancer` instances are simulated and available through the
  ovstage `RIGID_BODY` output-read path, but they are not exposed as individual
  rigid-body tensor-binding rows. With the default empty-binding behavior, a
  binding that targets only the point instancer has count zero. Read instance
  state through output read; control it by authoring the point-instancer arrays
  through ovstage. (NVBugs 6481085 / OMPE-102217.)
- **ovstage-authored clones are not available to TensorBindingsAPI in 0.5.**
  Draining an `ovstage.Stage.clone()` / `ovstage_clone()` delta with
  `update_from_ovstage()` creates the corresponding physics objects, but
  TensorBindingsAPI does not discover them. After the source Stage is populated
  and attached, and after any later committed source edits are drained,
  tensor-based multi-environment workloads must use `PhysX.clone()` /
  `ovphysx_clone()` and complete direct cloning before `warmup_gpu()` or the
  first simulation step. Direct cloning invalidates existing tensor and contact
  bindings; destroy and recreate them before use. Do not also drain a duplicate
  ovstage clone delta for the same targets. Complete support for ovstage-authored
  clones in TensorBindingsAPI is scheduled for ovstage after 0.5.

### Fixed
- **Object-change callbacks no longer fire for the initial stage population
  (NVBugs 6473870 / OMPE-102206).** A subscriber registered before an
  `attach_ovstage()` / `update_from_ovstage()` populated its scene received a
  spurious `on_object_created` callback for every actor and shape of the initial
  population, contradicting the documented contract that the initial population is
  not notified (the caller already has that state from setup). ovphysx subscribes
  with `stopCallbackWhenSimStopped=false` so it also observes reset/clone events
  while stopped; that flag used to bypass the runtime's initial-population
  suppression as well. The runtime now suppresses the initial population for every
  subscriber by default, independent of the simulation-stopped gate.
- **SDF mesh colliders no longer hang attach under CPU-only mode (NVBugs 6480595).**
  Authoring a dynamic rigid body with an SDF mesh collider
  (`PhysxSDFMeshCollisionAPI`, `physics:approximation="sdf"`) and attaching while
  in CPU-only mode (`PhysX.set_cpu_mode(True)`) previously hung `attach` forever:
  the synchronous attach-time SDF cook requested GPU cooking on the ujitso agent,
  which could not create a CUDA context, and attach then blocked on an infinite
  cook wait that the canceled build never signaled. SDF cooking now falls back to
  the CPU builder whenever no CUDA device is available, so the body cooks and
  simulates on CPU. GPU cooking is unchanged when a device is present.
- **`ovphysx_create_instance()` rejects a null config-entry array with a
  nonzero count (NVBug 6481076 / OMPE-102221).** With an active lifecycle, the
  C API now returns `OVPHYSX_API_INVALID_ARGUMENT` without changing the output
  handle or starting the runtime, instead of silently ignoring the pair and
  returning a live instance.
- **Malformed DLPack metadata is rejected before traversal
  (NVBug 6394727).** The Python `__dlpack__` ingestion path now raises
  `ValueError` when a producer reports a rank outside the supported range of
  1 through 8 or a null shape, before indexing its shape or stride metadata.
- **`ManagedDLTensor` cleanup waits for all DLPack borrowers
  (NVBug 6473877 / OMPE-102207).** The Python wrapper and every capsule exported
  from it now share the wrapper's lifetime. A capsule retains its producer until
  the capsule or its consumer releases the managed tensor, while the producer
  remains the sole owner of the user cleanup callback. The callback therefore
  runs exactly once after the wrapper and all borrowers are gone, preventing
  both premature cleanup and the previous double callback.
- **Large `clone()` batches retain every runtime target path (NVBugs 6471168).**
  The replicator rename callback previously received integer path handles encoded
  from temporary `SdfPath` objects that had already been destroyed. At large
  batch sizes, allocator reuse made some clones unavailable under their requested
  runtime paths and from tensor bindings even though all PhysX actors were created.
  The clone call now owns every target `SdfPath` until synchronous
  replication completes, so wildcard and explicit tensor bindings resolve the
  full batch deterministically.
- **Ovstage output reads now return live DirectGPU rigid-body and articulation-link
  state.** Position, orientation, linear velocity, and angular velocity are gathered
  from PhysX DirectGPU state, using the scene-wide articulation-link stride.
- **Live deformable output reads under DirectGPU.** The ovstage-native output
  read now refreshes volume and surface sim-mesh positions and velocities from
  the device when per-frame USD write-back is disabled. In scenes where
  sleeping is enabled, `ALL` scope also refreshes sleeping deformables instead
  of returning last-awake data. (NVBugs 6464833 / OMPE-101753.)
- **Tensor path matcher: same-name over-match, unanchored alternation, and `**` duplicates fixed.** Three corrections to the leaf-recursive pattern matching that shipped in 0.4.1 for `create_articulation_view()` / `create_rigid_body_view()` / `create_rigid_contact_view()`. (1) A leaf that recurs nested inside its own match (e.g. `Robot/Disc001/robot/Disc001` under pattern `/envs/*/robot/Disc001`) is now suppressed -- shallowest wins -- so a match is counted once; this fixes the IsaacLab 3.0 contact-sensor "expected 1, found 2" filter-count error. (2) An alternation leaf like `base_link|link_0` is anchored as `^(base_link|link_0)$`, so it no longer also matches `base_link_extra` / `prefix_link_0` and silently enlarges views. (3) After an explicit `**` the final leaf is direct-matched rather than re-recursed from every root, so contact `filterPaths` and SDF views (which do not pointer-dedup) no longer receive duplicate paths.
- **The resolver and OmniClient runtime now come from the exact OVStage package
  selected by the build.** The direct `omniusdresolver_ov_openusd_0.25.11_nopy`
  and `omni_client_library` Packman dependencies are removed. Native packaging
  stages the matched OVStage resolver, registry, OmniClient, and
  `omniverse_connection` set, while wheel packaging continues to leave the USD
  and resolver singleton to the paired OVStage wheel. OVStage-provided notices
  are preserved in the SDK and wheel license payloads. Windows wheel startup
  registers both OVStage runtime directories before loading `ovphysx.dll`, so
  the external USD monolith resolves without relying on `PATH`. Startup also verifies
  the loaded OmniClient version against the staged OVStage provider and fails
  closed instead of reusing a foreign Kit client.
- **Debug compilation now consistently uses Release runtime dependencies.**
  The published OVStage package provides a Release-only resolver, OmniClient,
  and `omniverse_connection` set. Internal and public builds therefore default
  to Release runtime dependencies, and an explicit true-Debug runtime request
  fails at configuration time instead of mixing incompatible runtime variants.
- **Wildcard tensor bindings over a top-level runtime clone now resolve the
  cloned bodies.** After `clone()`-ing a source subtree onto a brand-new
  top-level target (e.g. `/World` -> `/World_clone0`), a pattern binding such as
  `create_tensor_binding(pattern="/World_clone0/*")` resolved 0 bodies even
  though the clone materialized real bodies (explicit prim paths found them). The
  replicator registered the clone root under the absolute root `/` instead of the
  (empty) pseudo-root, so the internal-DB path matcher -- which selects roots by
  an empty parent -- never enumerated it. Nested clone targets under an existing
  parent (e.g. `/World/envs/env0` -> `/World/envs/env1`) were unaffected.
  (NVBugs 6421194 / OMPE-100487.)
- **Embedded NUL bytes in prim paths and patterns are rejected consistently
  across the whole API.** `get_object_type()`, `create_tensor_binding()`,
  `create_sdf_view()`, `get_physx_ptr()`, `clone()`, `create_contact_binding()`,
  and the scene-query SHAPE geometry (`sweep()` / `overlap()`) no longer silently
  mis-resolve paths carrying an
  embedded NUL byte (truncation, glob broadening, or wrong-object matches); they
  now return `OVPHYSX_API_INVALID_ARGUMENT`. To make this safe once, the
  remaining `const char*` path inputs at the C ABI were converted to the
  length-prefixed `ovphysx_string_t`: `ovphysx_get_object_type()`,
  `ovphysx_get_physx_ptr()`, and the `prim_path` field of
  `ovphysx_scene_query_geometry_desc_t.shape`. **Migration (C):** wrap literals
  with `OVPHYSX_LITERAL("/path")` or runtime strings with `ovphysx_cstr(str)`.
  The experimental C++ `PhysX::getPhysXPtr()` and Python `get_object_type()` /
  `sweep()` / `overlap()` signatures are unchanged. Closes NVBugs 6433621.
- **`clone()` now invalidates the tensor backend's cached per-stage simulation data (NVBug 6428316).**
  The DirectGPU tensor backend builds its `GpuSimulationData` lazily on the first
  `createSimulationView()` and snapshots the rigid-dynamic population at that instant --
  the actor->row map and the shared DirectGPU staging buffers (`mRdPoseDev`,
  `mRdLinearVelAccDev`, ...) are sized by the then-current rigid-dynamic count. A
  `create_tensor_binding()` issued *before* `clone()` locked that count in at the pre-clone
  value; `clone()` then added bodies without rebuilding the cache. Every binding created
  afterward logged `Internal error: Unresolved rigid dynamic index!` for the cloned actors
  and overflowed the undersized buffers, producing silent, non-deterministic partial writes
  (only a subset of environments received a batched `RIGID_BODY_VELOCITY` write, while
  `write()` still returned success) and an intermittent CUDA illegal memory access
  (`cuda_status=700`) on the `RIGID_BODY_POSE` read path. `clone()` now calls
  `resetStage()` so the next `createSimulationView()` rebuilds the cache against the
  post-clone population. Existing tensor and contact bindings are invalidated by
  `clone()` and must be recreated before use. Best practice remains
  `attach -> clone -> warmup_gpu -> create bindings`.
- **`clone()` after the first `step()` is now rejected on CPU as well as GPU, and duplicate `target_paths` are rejected.**
  Previously the after-step precondition only fired on GPU (`gpu_warmup_done` was
  never set in CPU mode), so code validated on CPU could clone after stepping
  with no error and then fail with a `RuntimeError` the first time it ran on
  GPU. `clone()` also silently collapsed a batch of duplicate `target_paths`
  into a single clone with no diagnostic. Both are now rejected with
  `OVPHYSX_API_INVALID_ARGUMENT` / `RuntimeError` in either mode. Closes NVBug
  6433668.
- **ovstage-backed loads now preserve child-collider poses and disabled state.**
  Collision shapes below a rigid-body prim retain their body-relative
  translation, rotation, and scale, and `physics:collisionEnabled=false` is
  honored. Previously, child colliders could be created at an identity pose and
  authored-disabled shapes could remain active, producing incorrect contact
  geometry and destabilizing constrained bodies.
- **Hard CPU-only mode rejects CUDA TensorBinding buffers before CUDA access.**
  When `ovphysx_set_cpu_mode(true)` is active or `OVPHYSX_DISABLE_GPU` is set,
  tensor reads and writes return `OVPHYSX_API_DEVICE_MISMATCH` for `kDLCUDA`
  and `kDLCUDAManaged` buffers without loading the optional CUDA interface or
  attempting a copy. Normal CPU/CUDA cross-device staging remains supported
  outside hard CPU-only mode.
- **ovstage attach/detach now preserves reusable runtime state.** Runtime
  attach failures are reported instead of being returned as success, and
  partial attachment state retains no pointer to the caller-owned Stage.
  Ovstage-only and nonresident-backing attachments keyed internally by zero
  now detach cleanly and can be reattached. Backing-stage query errors fail
  instead of silently becoming stageless attachments.
- **`ovphysx_reset_stage()` now releases TensorAPI per-stage simulation data.**
  Stage reset and ovstage detach now invalidate the stage's tensor views and release
  cached CPU/GPU simulation buffers. Previously this cleanup happened only when
  destroying the whole `PhysX` instance, so repeated reset-and-reload cycles
  could exhaust GPU memory.
- **Closed-loop output-read samples now verify ovstage control writes.** The
  Python sample now seals control and output ordinals before consuming or
  publishing them, and the C sample updates the existing
  `physics:velocity` attribute without replacing its semantic metadata. Both
  samples alternate the requested x velocity, fail if the simulation does not
  observe the change, and print the resulting velocity and displacement.
- **ovstage compatibility is pinned to `0.1.0.342061` (C++ build
  `0.1.0.342061.191b00ed`).** The integration uses the renamed
  `ovstage_api_status_t` and enqueue `op_index` fields, declares fixed versus
  array writes explicitly, and uses population-owned USD files instead of the
  removed caller-owned StageCache entry point. The artifact fetch rejects a
  wheel whose embedded build version does not match the C++ package, avoiding
  a silent mismatch at the moving `0.1.0` release URL. The SDK preserves the
  release's curated third-party notices as
  `ovstage-THIRD-PARTY-NOTICES.txt` at the package root.
- **ovstage consumers must rebuild against the pinned `342061` headers.** This
  release replaces `cuda_event` in the data and write payloads with
  `cuda_sync { stream, wait_event }`, changing the public read-group layout.
- **Repeated ovstage wholesale reloads no longer corrupt Fabric change
  tracking.** Build `342061` clears stale dirty-bucket IDs when wholesale
  population resets and recycles its buckets, so an `ALL -> reset -> ALL`
  sequence produces a valid change snapshot instead of crashing hierarchy
  consumers. Ovstage owns and loads the matching Fabric implementation from its
  released package.
- **Initial ovstage snapshots are no longer replayed after attach.**
  `ovphysx_attach_ovstage(..., read_ordinal)` already parses that ordinal, so
  the shipped helpers, samples, tests, and docs now call
  `ovphysx_update_from_ovstage()` only for later application-authored writes.
  This avoids treating the population's `usd-prim-type` columns as a second
  structural resync before the first DirectGPU step.
- **Whole-prim removals from ovstage now remove their runtime physics
  objects.** Build `342061` emits ranged-read tombstones for deleted
  `usd-prim-type` and `usd-schemas` metadata even when their write floors do not
  advance. The change feed uses those tombstones to trigger a final structural
  snapshot, so `ovphysx_update_from_ovstage()` destroys only paths that remain
  absent and preserves a prim deleted and recreated within the same range. The
  previous latest-state liveness probe is gone.
- **Runtime clones preserve contact-report registration after ovstage loads.**
  Replication now finalizes deferred source setup even when the source objects
  were already parsed by an ovstage update. This resolves pending contact-report
  actor pairs before cloning, so a clone created before the first simulation
  step reports contacts like its source body.
- **The namespaced USD runtime follows the ovrtx 0.4 / ovstage 0.1 release pin.**
  Source builds and Python runtime tests use `0.25.11.kit.4-gl.21081`, kept
  together to preserve ABI alignment.
- **ovstage's nested Cubric plugin is excluded from CPU-safe ovphysx
  packages.** Build `0.1.0.342061` keeps the plugin under
  `bin/plugins/omni.cubric/`; installation now removes that directory alongside
  the older layout so unused CUDA startup code is not exposed on CPU-only hosts.
- **`ovphysx_write_tensor_binding` now accepts int64 index tensors.** Previously
  only int32 was accepted; int64 indices are now staged down to int32 before the
  write. Fixes silent no-op on tendon and other indexed writes when callers
  (e.g. the IsaacSim umbrella adapter) supply int64 indices.
- **Windows CPU-only ovstage Stage creation is restored.** Build `342061`
  retains the GPU-optional bootstrap fix from Kit (MR 47029), so the Windows
  no-GPU CI again exercises both C++ `cpu_usd` ingestion and the Python Stage
  smoke instead of skipping them. On a GPU-capable host, ovstage Stage creation
  still activates a CUDA primary context before ovphysx attach; the separate
  hard CPU-only no-driver-touch test remains skipped at that upstream boundary.
- **Windows ovruntime teardown is a blocking gate again.** Build `342061`
  retains the corrected Cubric/usdrt plugin load order. The temporary
  `339558`-specific allow-failure and pin guard are removed; the complete
  Windows unit suite must now exit cleanly after its assertions pass.

### Removed
- **Fabric-based cloning implementation removed** (the `ovphysx_clone` / `PhysX.clone()` API is
  retained, reimplemented on the PhysX replicator -- see Added). Deleted the Fabric scenegraph-
  duplication path behind the old clone: the sidecar clone entry points, `ovphysxReplicator.cpp`,
  the `ovphysx_set_clone_env_root()` remnants, and the Fabric prim duplication itself. Part of the
  Fabric retirement (ADR-0009): this was the last consumer of Fabric-based runtime prim duplication
  in ovphysx. Data-plane duplication of the *scenegraph* is now an ovstage responsibility
  (`ovstage_clone` + instancing); ovphysx's `clone()` handles the *physics* instantiation via the
  replicator. **Behavior change:** the old Fabric path authored per-clone scenegraph prims; the new
  path does not -- clones exist as runtime physics only.
- **Vestigial Fabric data-movement layer (internal).** Deleted the orphaned
  `DataMovementImpl` Fabric stage-lifecycle / DLPack holder -- dead code superseded by the
  ovstage read/write API (0.5.1, ADR-0007); no public API or behavior change. Part of the
  Fabric retirement (ADR-0009).
- **Obsolete Fabric config knobs.** Dropped the internally-forced `/physics/fabricEnabled`
  setting and removed `/physics/fabricUpdateVelocities` from the config docs/examples/tests.
  Both had zero readers after the runtime Fabric removal (no-ops); they may still be passed
  via `carbonite_overrides` but no longer do anything.
- **Host Fabric/USDRT bootstrap payload removed.** Standalone Physics tests and
  packaged ovphysx artifacts no longer preload or flatten a second Fabric,
  scenegraph, hierarchy, or population stack from `ovruntime_deps`. ovstage owns
  and loads its matching population runtime from its module-relative plugin tree.
  ovphysx retains `ovruntime_deps` for unrelated services such as datastore,
  UJITSO, blobkey, Cubric, GPU compute, and USD support.

---

## [0.5.1] - Date TBD

### Added
- **Physics output read API (ovstage), ADR-0007.** New C surface to read simulation output the ovstage way and feed it straight back into the attached Stage with no repack: `ovphysx_query(type, scope)` → `ovphysx_fetch_query_result` (attribute / total-prim discovery) → `ovphysx_read(attrs…)` → loop `ovphysx_fetch_read_next(&group_ptr)` → `ovphysx_release_group` → `ovphysx_release_read` / `ovphysx_release_query`. Generic over simulated type (`ovphysx_sim_object_type_t`: rigid body incl. point-instancer arrays, articulation link, joint state, vehicle wheel, deformable volume/surface, particle set) and scope (`ovphysx_object_scope_t` all/active).
  - **ovstage-native types (no ovphysx mirror).** The public headers `#include` the ovstage/ovx headers and use ovstage's own types directly — a read group is `ovstage_read_group_t`, discovery is `ovstage_query_result_t`, attribute names are `ovx_string_or_token_t` (string name OR interned token, in one call), the attached Stage is an `ovstage_instance_t*`, and `ovphysx_update_from_ovstage` takes an `ovstage_ordinal_range_t`. This replaces the earlier ovphysx mirror structs (`ovphysx_read_group_t` / `ovphysx_query_result_t` / `ovphysx_attribute_semantic_t`) and the separate `ovphysx_read_tokens`. ovphysx ships the ovstage headers in its SDK/wheel alongside `ovphysx.h`.
  - **Producer-owned group.** `ovphysx_fetch_read_next` hands back a borrowed `const ovstage_read_group_t*` (producer-owned) instead of filling a caller struct; ovstage owns its allocation and lifetime. Consumers must compile against the exact pinned ovstage headers when the public struct layout changes. The group's borrowed `data.tensors` (tuple width in `dtype.lanes`), `prims.list` interned prim set, `attribute` token, and `data.index_map`/`mask` feed the ovstage write path verbatim.
  - **Distinct EOF vs. error end to end.** The lower ovstage-native `ovxFetchReadNext` returns a 3-valued `OvxReadStatus` (ok / end-of-iteration / error) instead of a bool, so the public `OVPHYSX_API_END_OF_ITERATION` is forwarded faithfully and a genuine fetch error is no longer reported as EOF.
  - **Empty match is a valid query.** A successful query with no matches returns a nonzero handle whose read reaches end-of-iteration immediately (`total_prim_count == 0`); a zero `out_query` means FAILURE only.
  - **Stable group lifetime.** A fetched group's borrowed storage (struct + tensors / index_map / prims.list) is valid until its `read_group_id` is released via `ovphysx_release_group` — fetching further groups, or an intervening `ovphysx_step`, does NOT invalidate earlier ones (the data path gathers into session-owned storage; the sidecar retains each live group and forwards `ovxReleaseGroup`).
  - **Point-instancer output forwards verbatim.** Rigid-body point-instancer output always emits the instancer's FULL instance array (by-index, no element-axis scatter) for both ALL and ACTIVE scope, so the group is directly assignable into the ovstage write path; ACTIVE scope only selects which instancers are emitted, not a sparse subset of instances.
  - Routes through the internal sidecar (which links the ovstage + runtime read symbols) via `g_sidecar*` function pointers — it now passes the ovstage types straight through (no translation). Requires an attached ovstage Stage (ovstage-only). Documented in the **ovstage Integration** docs section (ordinal-coupling principle: app→physics edits flow through `ovphysx_update_from_ovstage`; physics→app output is written at ordinals drain ranges never cover). Python: context-managed `PhysX.read(...)` / `PhysX.read_tokens(...)` → `ReadResult.groups` (`ReadGroup` exposes `tensors`/`index_map`/`semantic`/`ordinal`/`layout_generation` + the interned `prim_list`/`attribute`), valid for the `with` block.
- **Closed-loop ovstage output-read samples (`output_read.py`, `output_read_c/`).** New Python and C samples driving the full ADR-0007 round trip on the falling-boxes scene: each frame authors a `physics:velocity` control edit on every rigid body into ovstage (reusing the read group's interned `prim_list` directly — no path rebuild), drains only the control ordinal via `ovphysx_update_from_ovstage`, steps, then reads every `ovphysx_sim_object_type_t` back and writes each group into ovstage under `sim:<attribute>` at a separate output ordinal that is never drained. The C sample reuses the group's borrowed `tensors` verbatim as the write payload (true no-repack); both demonstrate the two-lane (control/output) ordinal scheme end to end.
- **Codeless PhysX USD schemas exposed for external authoring/validation (OMPE-86833).** ovphysx now ships the PhysX USD schemas as codeless schema artifacts (a `plugInfo.json` with `Type=resource` plus `generatedSchema.usda`, no compiled library) in both the wheel and the SDK package under a stable path: `<ovphysx>/schemas/physx/<module>/resources/`. The exposed set is derived at packaging time from the PhysX schema modules ovphysx actually ships (e.g. `PhysxSchema` and `OmniUsdPhysicsDeformableSchema`), so it always matches the packaged schema revision and tracks module additions/removals automatically. They are intended for external tooling that drives a stock `usd-core` from PyPI; ovphysx still has no dependency on any external schema package. New pure-Python helpers `ovphysx.codeless_schema_paths()` (per-module `resources/` directories, ready for `pxr.Plug.Registry().RegisterPlugins()`) and `ovphysx.codeless_schema_root()` discover them without triggering native loading. A reference example + regression test (`tests/python_samples_extra/codeless_schemas/`) registers the schemas into a stock `usd-core`, applies `PhysxRigidBodyAPI` by identifier, and asserts `prim.HasAPI(...)`. Codeless schemas carry no compiled C++/Python bindings; use USD's generic schema API and apply by schema identifier (e.g. `prim.ApplyAPI("PhysxRigidBodyAPI")`).
- **PhysX debug-visualization C API (`ovphysx_debug_render_*`).** Drives the standard PhysX debug-visualization pipeline (`omni::physx::IPhysxVisualization`) from hosts that step PhysX through ovphysx: `ovphysx_debug_render_enable()` authors the `eVISUALIZATION` flags + applies the selected params; `ovphysx_debug_render_set_parameter()` / `_get_parameter()` toggle/read one `ovphysx_debug_render_parameter_t` (a public enum mirroring `omni::physx::PhysXVisualizationParameter`, validated against `NONE`/out-of-range); `ovphysx_debug_render_set_scale()` / `_get_scale()` set/read the master scale (rejects non-finite/negative); `ovphysx_debug_render_set_culling_box()` (rejects non-finite / `min > max`); and `ovphysx_debug_render_get_points()` / `_lines()` / `_triangles()` hand back read-only buffers of `ovphysx_debug_point_t` / `_line_t` / `_triangle_t` (layout `static_assert`-locked to the omni::physx structs), valid until the next step or any stage/scene change. `_get_parameter` / `_get_scale` return the values last set through ovphysx (cached on the ovphysx side; no interface change). No high-level Python wrapper yet. Pairs with an omni.physx fix so a non-positive viewport gizmo scale no longer zeroes `eSCALE` (debug viz works in headless / minimal hosts without seeding any persistent setting).
- **`ovphysx_articulation_update_kinematic()` (OMPE-94459, _KINEMATIC_UPDATE_NOOP fix).** New C API that forces propagation of root + DOF state into every articulation link buffer in the binding by calling `PxArticulationReducedCoordinate::updateKinematic`. Flags map to `PxArticulationKinematicFlag::ePOSITION` / `eVELOCITY` via `OVPHYSX_ARTICULATION_KINEMATIC_POSITION` / `_VELOCITY` (bitmask). Closes the umbrella's `OvPhysxSimulationView.update_articulations_kinematic` no-op gap so dof-position / root-transform writes can be observed in link-pose reads without simulating. Routes through the internal sidecar (which links PhysX SDK headers); the main ovphysx target stays free of direct PhysX includes via a new `g_sidecarUpdateKinematic` function pointer. Python: callable directly via `_bindings._lib.ovphysx_articulation_update_kinematic(handle, binding_handle, flags)` with typed ctypes argtypes already registered; no high-level Python wrapper class method yet.
- **`OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32` tensor type (OMPE-94459, JointPerformanceEnvelope fix).** New read/write `[N, D, 3]` tensor wiring `IArticulationView::set/getDofDriveModelProperties`. Per-DOF triple is (`speedEffortGradient`, `maxActuatorVelocity`, `velocityDependentResistance`). Writes are applied only to DOFs with `PhysxDrivePerformanceEnvelopeAPI` applied in USD; the engine silently drops writes on other DOFs and logs a warning. Python: `TensorType.ARTICULATION_DOF_DRIVE_MODEL`.
- **`ovphysx_get_object_type()` (OMPE-94459 #13).** New C API + Python `PhysX.get_object_type(prim_path)` that classifies a USD prim by TensorAPI object type (`RIGID_BODY`, `ARTICULATION`, `ARTICULATION_LINK`, `ARTICULATION_ROOT_LINK`, `ARTICULATION_JOINT`, or `INVALID`). Mirrors `omni::physics::tensors::ObjectType`. Closes the umbrella adapter's missing `sim.get_object_type` surface.
- **GPU-mode shape-property and uint8-flag tensor writes (OMPE-94459 §B9).** Fixed articulation and rigid-body tensor views rejecting GPU tensors for material, contact-offset, rest-offset, and uint8 flag writes. The GPU view implementations now stage GPU tensors to host via cudaMemcpy before delegating to the shared runtime logic, and matching reads copy host-side results back to GPU destination tensors. Both indexed and masked GPU writes on the three articulation shape-property bindings (`ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION`, `ARTICULATION_CONTACT_OFFSET`, `ARTICULATION_REST_OFFSET`) and the rigid-body uint8 bindings now succeed instead of returning "Incompatible device".
- **`OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL` tensor type (OMPE-94459 §B5).** New read/write `[N]` uint8/bool tensor that toggles `PxActorFlag::eDISABLE_SIMULATION` on the underlying `PxRigidActor` so disabled bodies stop participating in the next solver step. Writes apply at runtime (no stage detach required) -- the underlying TensorAPI `IRigidBodyView::set/getDisableSimulations` was already wired engine-side, this just exposes it through the ovphysx binding. Both indexed and masked-write paths are supported. Python: `TensorType.RIGID_BODY_DISABLE_SIMULATION`. The umbrella adapter's `RigidBodyView.set_disable_simulations()` can now drive the engine directly instead of writing USD `physics:rigidBodyEnabled`. The C ABI dtype validator (`validateTensorShape`) gained a per-type dtype expectation so `DISABLE_SIMULATION` accepts uint8/bool while every other binding continues to require float32; `ovphysx_get_tensor_binding_spec()` reports the matching dtype.
- **Python `TensorBinding.dtype` and `TensorBinding.spec`.** The high-level Python `TensorBinding` now preserves the native `ovphysx_get_tensor_binding_spec()` dtype in addition to `shape` and `ndim`. `binding.dtype` returns a Python-owned `DLDataType` copy, and `binding.spec` returns `TensorBindingSpec(dtype, ndim, shape)`. Downstream view builders can allocate buffers from the native DLPack spec and no longer need to assume every `TensorType` is float32.
- **`ovphysx_read_raw_contact_data()` and `ovphysx_contact_binding_get_other_actor_paths_from_ids()` (OMPE-94459 #21).** Filter-less variant of `ovphysx_read_contact_data` -- returns every contact involving each sensor body plus a per-contact opaque actor id resolvable to a USD prim path. Shapes (C = max_contact_data_count, S = sensor_count): force/separation `[C, 1]`, point/normal `[C, 3]`, count/start_indices `[S]` (1D, no filter dim), other_actor_ids `[C]` int64/uint64. Pair with `ovphysx_contact_binding_get_other_actor_paths_from_ids(ids_tensor, out_paths, max, &count)` to translate ids to USD prim paths in bulk. Filter pattern length may be 0 when creating the binding (raw reads don't use a filter dim). Python: `ContactBinding.read_raw_contact_data(force, point, normal, separation, count, start_indices, ids)` + `get_other_actor_paths_from_ids(ids_array) -> list[str]`. Also extends the DLPack->TensorDesc converter (`DLPackConvert.h`) to accept int64/uint64 in addition to the previous float32/int32/uint32/uint8/bool set, required by the actor-id tensors.
- **Three new articulation tensor types for centroidal dynamics (OMPE-94459, §B0):** `OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32` (12), `OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32` (13), and `OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32` (14). Both mass-center variants return `[N, 3]` (per-articulation COM in world or root-local frame); the centroidal-momentum tensor returns `[N, 6, D+7]` packing the 6x(D+6) centroidal-momentum matrix plus a 6x1 bias column (D = `getMaxDofs()`). All three are read-only -- writes return `OVPHYSX_API_INVALID_ARGUMENT`. Centroidal momentum requires floating-base articulations; the tensor backend returns an error on fixed-base. Python mirror: `TensorType.ARTICULATION_MASS_CENTER_WORLD`, `..._MASS_CENTER_LOCAL`, `..._CENTROIDAL_MOMENTUM`.
- **`ovphysx_rigid_body_view_wake_up()` / `ovphysx_rigid_body_view_sleep()` (OMPE-94459 follow-up).** New C APIs + Python `TensorBinding.wake_up(indices=None)` / `TensorBinding.sleep(indices=None)` that wake or put to sleep rigid bodies in a binding, mirroring `PxRigidDynamic::wakeUp` / `putToSleep`. Optional int32 indices tensor for a subset; pass NULL/None to act on every body in the binding. Bodies with `RIGID_BODY_DISABLE_SIMULATION` set are silently skipped. `GpuRigidBodyView` overrides added so both calls work on the GPU pipeline (stages GPU indices to host then delegates to base impl).
- **`ovphysx_destroy_instance` resets the tensor SimulationBackend (OMPE-94459 follow-up).** The backend caches `mGpuSimData` / `mCpuSimData` as plugin-scope singletons and clears them only on stop events that ovphysx does not emit. With plugins resident across destroy/create cycles, the next instance + stage attach would inherit stale GPU data causing "Internal error: Unresolved rigid dynamic index!" and heap corruption. Fix: the destroy path now resets the tensor backend after per-binding cleanup so the next instance starts with fresh backend state.
- **GPU `link-incoming-joint-force` / `dof-projected-joint-forces` return zeros pre-simulate, matching CPU.** `GpuArticulationView::{getDofProjectedJointForces, getLinkIncomingJointForce}` now zero the destination tensor when `SimulationBackend::getStepCount() == 0`, mirroring CPU's existing `PxMemZero` on `dt == 0.0f`. Previously the GPU readback streamed uninitialised PhysX SDK buffer bytes to the projection kernel.
- **Disable/enable is now correct across multiple GPU rigid-body views over the same scene (OMPE-94459).** `GpuRigidBodyView`'s rd-GPU-index refresh was gated on a per-view dirty flag set only by that view's own `setDisableSimulations`. With two or more views covering the same body, a disable/enable issued through one view freed/reallocated the shared GPU island index without dirtying the sibling views, so those siblings kept reading the stale/invalid index. Fix: `setDisableSimulations` now bumps a scene-wide `mRdDisableEpoch` on the shared `GpuSimulationData`; `refreshRdGpuIndices()` compares it against a per-view last-seen value and rebuilds when a sibling toggled a body. The fast path stays a true no-op for the common single-view, disable-free case.
- **`ovphysx_set_cpu_mode(bool)` process-level CPU-only API.** Call before creating any instances to force the entire process into CPU-only mode: no CUDA driver is touched, all PhysX scenes use CPU dynamics regardless of their USD `physxScene:enableGPUDynamics` setting, and Fabric stages use CPU compute. Returns `OVPHYSX_API_ERROR` if any instances are currently active, or if attempting to revert after CPU mode has been applied (sticky for the process lifetime). C++: `PhysX::setCpuMode(bool)` (static). Python: `PhysX.set_cpu_mode(bool)` (static). For per-scene CPU control without this flag, author scenes explicitly with `physxScene:enableGPUDynamics = false` and `physxScene:broadphaseType = "MBP"`.

### Changed
- **`omni.physics.tensors` plugin removed; `TensorApi` now ships in the static PhysX runtime (OMPE-96492).** The standalone `omni.physics.tensors.plugin` (`TensorApi` + `BackendRegistry`) is deleted. Because PhysX is the only simulation backend and it already lives inside `OvruntimePhysX`, the backend registry was pure indirection: the static PhysX runtime now owns the `TensorApi` function table directly and `createSimulationView` / `reset` / `resetStage` call the in-runtime backend directly. `BackendRegistry` and `ISimulationBackend` are removed, and the now-meaningless per-call `backendName` selector is dropped from `createSimulationView` / `reset` / `resetStage`. Behavior-neutral for tensor API consumers; the `ISimulationView` / typed-view contract is unchanged. The Python `omni.physics.tensors` module and frontends were already retired upstream; this completes the removal.
- **`omni.physx.tensors` backend folded into the static PhysX runtime (OMPE-96492).** The standalone `omni.physx.tensors` Carbonite plugin is no longer built or packaged; the PhysX tensor backend now ships in the static `OvruntimePhysX` runtime linked into `libovphysx`.
- **Non-Fabric ovruntime interfaces no longer use Carbonite plugin metadata or publication.** Their existing function-table shapes are preserved, while ovphysx links the core `OvruntimePhysX` runtime statically and starts it directly after Carbonite has loaded the remaining dependency plugins. This removes the old Carbonite interface-acquisition and lifecycle path without changing public ovphysx C, C++, or Python APIs. `IPhysxFabric` is excluded because Fabric is being removed separately.
- **Tensor API path matcher no longer depends on Fabric/usdrt (OMPE-96492).** The tensor wildcard matcher no longer attaches a usdrt/Fabric stage. On fabric-replicated stages it resolves against the Omni PhysX internal path<->object database and restores numeric clone order via `ObjectId`; on USD-authored stages it uses the plain-USD matcher unchanged. The tensor backend is now Fabric-include-free, so the tensor API works in headless ovphysx without a populated Fabric stage. Behavior-preserving -- tensor-view row/env ordering is unchanged on both paths (validated bit-identical against trunk on the IsaacLab parity matrix).
- **Empty tensor bindings are quieter and better documented.** Rigid-body and articulation tensor bindings created from optional broad patterns that match zero prims remain valid zero-count bindings, but no longer emit error/warning-level native logs through ovphysx. Explicit `prim_paths` still surface no-match diagnostics for typos. Python `create_tensor_binding(..., raise_if_empty=True)` now keeps its opt-in `ValueError` while explaining that optional or broad readback should use the default `raise_if_empty=False` and check `binding.count`. The tensor-binding tutorial now calls out empty optional bindings explicitly.
- **Remaining top-level thread-safety docs now match the singleton runtime contract.** The C header, module-level Python docs, and overview no longer claim that multiple `PhysX` instances are fully thread-safe; they now match the developer guide's process-global runtime guidance.
- **`active_cuda_gpus` GPU ordinal selection applied when simulation first binds to a stage.** The ordinal specified in `ovphysx_create_args::active_cuda_gpus` is applied lazily, not at `create_instance` time. All stage bindings serialize through an internal mutex so concurrent instances with different ordinals do not stomp each other's setting.
- **Tensor backend per-stage view invalidation.** `SimulationBackend::resetStage(stageId)` now finds and invalidates all `ISimulationView` objects for that stage before releasing sim data, preventing stale-view use-after-free. `ovphysx_destroy_instance` calls `resetStage` automatically on teardown.

### Removed
- **Dead pre-ovstage USD-from-URI loader removed (internal).** The unused internal `omni_sdk_physx_load_usd` and its sidecar open path (`ovphysx_open_usd_stage` + `ovphysx_open_usd_stage_wrapper` + `g_sidecarOpenUsdStage`) — a leftover from when ovphysx opened USD itself — had no caller or public entrypoint and are gone. ovphysx never resolves a remote/local USD asset itself in the ovstage model (the application populates the Stage and attaches it). The sidecar's `ovphysx_close_usd_stage` is retained (the instance-destroy/unload path still erases the attached USD stage from `UsdUtilsStageCache`). No public-API change.
- **`ovphysx_configure_s3()` / `ovphysx_configure_azure_sas()` removed from the public interface.** Also gone: the C++ `PhysX::configureS3` / `PhysX::configureAzureSas` and the Python `configure_s3` / `configure_azure_sas`. They were thin forwarders to OmniClient's process-global `omniClientSetS3Configuration2` / `omniClientSetAzureSASToken` — a pre-ovstage vestige from when ovphysx loaded USD itself. In the ovstage model the **application** owns population (`ovstage.population.open_usd(...)`) and ovphysx only consumes an already-populated Stage, so remote-asset credentials belong on the asset layer (OmniClient), configured by the app before population — not on ovphysx. **Migration:** configure credentials directly on OmniClient (`omni.client` S3/Azure config in Python, or `omniClientSetS3Configuration2` / `omniClientSetAzureSASToken` in C/C++) before calling `open_usd`. See the **Remote USD Loading** developer-guide section.
- **`ovphysx_get_stage_id()` removed from the public interface.** The C API `ovphysx_get_stage_id`, the C++ wrapper `PhysX::getStageId`, and the Python `PhysX.get_stage_id()` are gone. The attached USD stage id was an internal correlation detail that leaked the runtime's stage-cache identity into the public surface; applications own their ovstage Stage and ordinals directly and have no need for it. **Migration:** there is no replacement — drop the call. Internal callers that still need the attached stage id read it from instance state directly.
- **Init-time joint-limit snap removed (OMPE-94459).** Articulation joints authored resting outside their limits are no longer snapped back into range at load (`addArticulationToScene`). The runtime no longer silently corrects authoring errors -- an out-of-limit joint now follows normal solver behavior. Assets must author joint state within limits; authoring-time validation will flag violations (tracked in a follow-up validation ticket).
- **`cpu_only` field removed from `ovphysx_create_args`.** Replaced by the process-level `ovphysx_set_cpu_mode()` API. The previous per-instance flag had process-wide effect on PhysX dynamics mode and was removed to make that scope explicit. **Migration:** replace `args.cpu_only = true` / `PhysX(cpu_only=True)` with a call to `ovphysx_set_cpu_mode(true)` / `PhysX.set_cpu_mode(True)` before creating any instances.
- **`DeviceType` Python enum and `PhysXDeviceError` exception removed.** Neither was raised or checked by any current code path.

### Fixed
- **Use-after-free in `BaseDeformableMaterialView::release()`.** `release()` nulled `mSim` before `delete this`, so the destructor skipped `mSim->_onChildRelease(this)` and the freed view was never removed from the parent `BaseSimulationView::mDeformableMaterialViews`. The dangling pointer was later dereferenced by `~BaseSimulationView`'s `_onParentRelease()` sweep, `check()`, and `release(recursive)`, corrupting the heap (intermittent SIGABRT, `unlink_chunk` hang, or segfault) after repeated deformable-material-view create/release cycles over a persistent per-stage_id `SimulationView`. Fix: drop `mSim = nullptr;` so `release()` matches the six sibling views and lets the destructor deregister normally.

## [0.5.0] - Date TBD

### Changed or Fixed
- **Breaking: stepping no longer takes an explicit simulation-time argument; the
  scene-teardown `reset()` is renamed to `reset_stage()` (mirrors ovrtx).**
  `step()`, `step_sync()`, and `step_n_sync()` (Python) and `ovphysx_step()`,
  `ovphysx_step_sync()`, `ovphysx_step_n_sync()` (C, plus the experimental
  `PhysX::step()` C++ wrapper) drop their `current_time` / `sim_time`
  parameter. Simulation time is now a private implementation detail: the library
  tracks it internally (starting at `0.0`, advancing by `dt` per step, by `n*dt`
  for `step_n_sync`) purely to feed engine/Fabric timestamps. This removes the
  per-call `sim_time += n*dt` boilerplate from RL training loops. The clock is
  owned by the application/ovstage, so it is not exposed through the ovphysx
  public API.
  - **The previous scene-teardown `reset()` is renamed to `reset_stage()` /
    `ovphysx_reset_stage()`** (mirrors `ovrtx_reset_stage()`); it clears the
    stage.

  **Migration:** callers of the old `reset()` (scene teardown) must switch to
  `reset_stage()`. The C++ wrapper's `reset(handle)` (RAII handle reset) is
  unchanged.

### Added
- **UJITSO cooked-collider cache is now enabled (local, in-process).** The kitless
  loader previously never loaded the UJITSO plugins, so collision cooking always ran
  uncached — every convex / triangle-mesh / SDF / convex-decomposition collider was
  re-cooked on every launch (the dominant cost in IsaacLab's repeated-launch
  workflow). ovphysx now loads `omni.blobkey`, `carb.datastore`,
  `carb.ujitsoagent`, and `carb.ujitso.default` before the runtime cooking service and persists
  cooked colliders to a content-addressed cache, so they are reused on the next launch.
  The cache key includes the mesh geometry, cooking parameters, cooking type, and a
  cooker version token, so a changed mesh/param or a different PhysX build is never
  served a stale collider.
  **Cache location is application-provided:** set
  `PhysXConfig(cooked_collider_cache_dir="<dir>")` (C:
  `OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY`) to the directory where cooked
  colliders should persist. ovphysx does not read environment variables and does not
  choose a location on the app's behalf; if no directory is configured, cooking still
  works but does not persist across runs. The cache grows without automatic eviction —
  delete the configured directory to reclaim space.
  **Scope:** strictly local and in-process — no Omni Hub, Nucleus, or GRPC backends
  (those datastore backends default to off; an app may opt in via
  `carbonite_overrides`). Disable cooking with
  `/physics/cooking/ujitsoCollisionCooking=false`. If collision cooking is enabled but
  the UJITSO plugins fail to load, initialization fails fast with a clear error rather
  than silently cooking uncached; a non-writable configured cache dir logs a warning
  and continues (no persistence). Addresses NVBugs 6262606.
- **Process-global lifecycle API.** New C entrypoints
  `ovphysx_initialize()` and `ovphysx_shutdown()` manage one active process
  lifecycle state. A second initialize before shutdown returns
  `OVPHYSX_API_ERROR`. C callers must initialize before
  `ovphysx_create_instance()`. `ovphysx_shutdown()` clears this lifecycle state
  but does not unload Carbonite, OmniClient, or the static PhysX runtime; those
  remain resident until process exit. CPU/GPU selection remains on
  `ovphysx_create_instance()`.
- **Release runtime-dependency selection for public Debug builds.** New CMake
  option `OVPHYSX_USE_RELEASE_RUNTIME_DEPS` selects release Packman runtime
  dependencies while preserving the requested compile build type. Internal
  checkouts default this option to `OFF`, so Debug builds keep matching Debug
  runtime dependencies unless explicitly opted in. Generated public source
  packages default it to `ON`, because public debug dependency packages are
  not shipped. The selected runtime-deps config is propagated through fetch,
  configure, install, and package steps; `cmake -P scripts/test_runtime_deps_config.cmake`
  validates the default matrix.
- **PhysX object-change notifications via `ovphysx_subscribe_object_changes()` / `ovphysx_unsubscribe_object_changes()` (C) and `ovphysx::subscribeObjectChanges()` returning an RAII `ObjectChangeSubscription` (experimental C++).** Subscribers receive optional `on_object_created`, `on_object_destroyed`, and `on_all_objects_destroyed` callbacks; the per-object callbacks deliver the prim path and `ovphysx_physx_type_t`, while `on_all_objects_destroyed` is a no-payload bulk-invalidation signal. Pair with `ovphysx_get_physx_ptr()` to refresh or invalidate cached pointers. Subscriptions are process-global; callbacks fire for events on every ovphysx instance in the process. **Limitations:** `ovphysx_clone()` does not currently emit creation notifications. After clone returns, refresh pointer caches explicitly. `ovphysx_reset_stage()` reliably emits `on_all_objects_destroyed` for bulk teardown. Not exposed in Python (the Python layer no longer surfaces raw PhysX pointers).
- **`PhysX.get_contact_report(copy=True)` returns Python-owned data safe to retain across steps.** The default `copy=False` keeps existing zero-copy ctypes-array semantics for back-compat. `copy=True` materializes `headers`, `points`, and `anchors` into `list[dict]` so callers (RL training loops, anything that holds contact data across a step) are no longer exposed to the silent-corruption hazard when internal C buffers are reallocated by the next `step()` / `step_sync()`. Docstring now carries a prominent lifetime warning. Closes NVBug 6172700.
- **`ResourceWarning` for forgotten `PhysX()` releases (Python).** Use `with PhysX():` or an explicit `release()` for deterministic cleanup. For instances that are garbage-collected mid-run, `__del__` emits a `ResourceWarning` (silent by default, surfaced under `python -W default::ResourceWarning` or in pytest) and then calls `release()` itself. The Python API no longer registers a process-exit `atexit` cleanup hook, so applications must not rely on interpreter shutdown to release live instances. `PhysX.release()` is unchanged (still idempotent). Closes NVBug 6172756.
- **Opt-in benchmark suite** under `tests/benchmarks/` (C++ harness, framework files copied near-verbatim from `omni.physx/tests/test.benchmarks/` so future updates merge cleanly) and `tests/python_benchmarks/` (`pytest-benchmark`-based). Both surfaces are off by default. Build the C++ binary with `cmake -DOVPHYSX_BUILD_BENCHMARKS=ON -P scripts/build.cmake` (or `./build.sh --benchmarks`); run the suite via `cmake -P scripts/test_benchmarks_cpp.cmake` (or `..._python.cmake`). Coverage: USD scene load, per-step simulation cost (CPU + GPU), `clone()` scaling at N = 64/256/1024, and tensor-binding read/write throughput. The C++ binary supports `--regenerate` to refresh baselines and `--slop=<pct>` for the regression bound. The CTest label is `benchmarks` and is excluded from `validate_all`. Heavy fixtures are generated and stored next to their generator scripts under `tests/benchmarks/data/`: `kapla_tower_sphere_collapse.usda` (1728 starting-asleep Kapla blocks + a sphere that fires in to collapse them) replaces the prior `boxes_falling`-based step benchmarks; `articulation_pileup.usda` (16 free-falling articulations of increasing link count over a dense scatter of dynamic obstacles, adapted from `omni.physxdemos.scenes.ArticulationDemo`) replaces the prior `links_chain` step benchmark. All benchmark fixtures now author `physxScene:timeStepsPerSecond = 240` so step-timing comparisons across fixtures are consistent.

### Changed or Fixed
- **ovphysx SDK and wheel builds now use static Carbonite as the only supported
  Carbonite mode.** The package no longer ships the core `libcarb` /
  `carb.dll` runtime library; `libovphysx` owns the single static Carbonite
  framework and packages only the remaining no-libcarb Carbonite bootstrap
  shims needed at runtime. Build and install RPATH handling was tightened so
  shipped binaries resolve bundled plugins without depending on a dynamic
  Carbonite SDK layout. TensorBindingsAPI Python acquisition now routes through
  the ovphysx-owned static framework instead of a module-local `CARB_BINDINGS`
  Carbonite entrypoint. The package verifier now hard-fails install and wheel
  builds if core Carbonite leaks back in as `libcarb.so`, `libcarb.so.*`,
  `carb.dll`, ELF `DT_NEEDED`/`DT_SONAME`, or a PE import; the per-plugin
  no-libcarb shims (`libcarb.<name>.plugin.so` / `carb.<name>.plugin.dll`) remain
  allowed. No caller migration is required for normal wheel or dynamically linked
  C/C++ SDK usage.
- **`clone()` after `warmup_gpu()` / first step now raises `RuntimeError` instead of silently corrupting state.** Previously `clone()` after GPU warmup logged a native `CARB_LOG_WARN` and proceeded, reallocating DirectGPU buffers and silently corrupting solver state — a documented hazard with no safe-fallback semantics. The C runtime (`ovphysx_clone`) now returns `OVPHYSX_API_INVALID_ARGUMENT` when `gpu_warmup_done` is set, with the message *"clone() must be called before warmup_gpu() and before the first step()…"*; Python `PhysX.clone()` propagates this as `RuntimeError`. Call `reset_stage()` first if you need to clone after warmup. Closes NVBug 6172717.
- **`ovphysx_subscribe_object_changes()` no longer retries per-call when invoked before instance creation; it returns `OVPHYSX_API_ERROR` immediately if the internal sidecar isn't loaded yet.** Previously the call retried symbol resolution on every attempt to accommodate subscribers that beat instance creation. The sidecar is now resolved eagerly during `ovphysx_create_instance`, so subscribe must be called *after* at least one instance exists. **Migration:** reorder callers to `ovphysx_create_instance` before any `ovphysx_subscribe_object_changes`; the error message names the missing sidecar load as the cause.
- **Multi-instance and threading documentation corrected to reflect the process-global runtime.** The developer guide previously implied that multiple `ovphysx` instances are safe to use concurrently across threads. They are not: every instance shares one attached scene process-wide. Callers must serialize across instances and treat the per-instance handle as bookkeeping only. The implementation has not changed; this is a spec correction. See [docs/developer_guide.md](../docs/developer_guide.md) Multi-Instance Support + Threading sections for the corrected guarantees.

### Removed
- **The legacy USD-handle load/unload API is removed**, superseded by the ovstage attach/update flow. `ovphysx_add_usd()` / `ovphysx_remove_usd()` (C), `PhysX::removeUsd()` (experimental C++), and `PhysX.remove_usd()` (Python) are gone, along with the per-instance `ovphysx_usd_handle_t`-keyed tracking. Load USD by populating an ovstage Stage and attaching it with `ovphysx_attach_ovstage()`; use `ovphysx_update_from_ovstage()` for later edits and clear the stage with `ovphysx_reset_stage()`. **Migration:** replace `add_usd` / `remove_usd` with the ovstage attach/update/reset flow.
- **Raw PhysX SDK pointers are no longer exposed through the Python API.** `PhysX.get_physx_ptr()` and the `ovphysx.PhysXType` enum are removed. The Python wrapper returned the pointer as a plain `int` address, so the only realistic use was passing it to a C/C++ extension and casting it back to a PhysX SDK type — a heavy migration for a Python-stable API to keep guaranteeing, with no in-Python use case that warranted it. The C (`ovphysx_get_physx_ptr()`, `ovphysx_physx_type_t`) and experimental C++ (`PhysX::getPhysXPtr()`) entrypoints are unchanged. **Migration:** this use case is no longer supported in Python. Callers that need raw PhysX SDK objects should call `ovphysx_get_physx_ptr()` from a native (C/C++) extension; the C and C++ surfaces continue to ship and remain the canonical interop path.

## [0.4.1] - 2026-05-12

### Breaking Changes
- **`gpu_index` replaced by `active_cuda_gpus` on `ovphysx_create_args`.** The `int32_t gpu_index` field is removed. Use `ovphysx_string_t active_cuda_gpus` instead: a comma-separated string of CUDA device ordinals. Single-GPU usage: `args.active_cuda_gpus = ovphysx_cstr("2")` (C), `PhysX(active_cuda_gpus="2")` (Python), `createArgs.setActiveCudaGpus("2")` (C++). Default (empty) selects GPU 0, matching the old `gpu_index=0` default. Multi-GPU patterns `"0,1,...,N-1"` (all GPUs, round-robin) and `"1,2,...,N-1"` (all except first) are now supported. The C++ `CreateArgs::setGpuIndex(int32_t)` is replaced by `setActiveCudaGpus(const std::string&)`.
- **Pre-loaded Carbonite PhysX stacks are rejected.** ovphysx can share a process with other OV libraries through namespaced USD reuse, but it no longer reuses a pre-existing Carbonite-published PhysX runtime stack. If another Carbonite user already registered incompatible PhysX plugins before `ovphysx_create_instance()`, creation fails with a clear error. Conflicting pre-set `/physics/cudaDevice` or `/physics/suppressReadback` values now fail instead of being silently accepted.
- **DirectGPU mode is now opt-in instead of auto-enabled for GPU instances.** Previously, `ovphysx_create_instance(device=GPU)` unconditionally set `/physics/suppressReadback=true` and `/physics/suppressFabricUpdate=true` (via two paths: a startup-time `CarboniteLoader::setStartupSuppressReadback` write before PhysX plugins loaded, and a per-instance write at GPU bootstrap), which routes PhysX into `eENABLE_DIRECT_GPU_API` mode. DirectGPU is incompatible with contact modification (per PhysX SDK guidance: contact-modify requires a CPU roundtrip), so any scene relying on `eMODIFY_CONTACTS` -- including the surface-velocity / conveyor pattern, custom contact callbacks, and contact-shaping filter rules -- silently failed to form kinematic-vs-dynamic broadphase pairs in GPU mode. **Both** write paths are removed: ovphysx no longer touches `/physics/suppressReadback` or `/physics/suppressFabricUpdate` at any point in its lifecycle. The previously-public `CarboniteLoader::setStartupSuppressReadback` static method is removed (internal API; no public callers). Hosts that want DirectGPU's tensor-pipeline performance (e.g. Isaac Lab) opt in by either (a) setting `/physics/suppressReadback=true` via Carbonite settings *before* `ovphysx_create_instance` if they have direct ISettings access, or (b) passing `ovphysx_config_entry_carbonite("/physics/suppressReadback", "true")` in `create_args.config_entries` (Python: `PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True})`). At startup ovphysx now logs a `[CarboniteLoader] /physics/suppressReadback=...` INFO line so misconfigurations are visible. See `ovphysx_create_args` in [`include/ovphysx/ovphysx_types.h`](../include/ovphysx/ovphysx_types.h) for the full trade-off documentation. **Migration:** GPU users who relied on suppressReadback being auto-enabled must set it explicitly via one of the routes above. **Perf note:** DirectGPU is the faster simulation path for tensor-pipeline workloads, so existing GPU consumers will see slower per-step times until they opt in -- the change here is correctness (contact-modify scenes now work in GPU mode), not a perf regression of the new default. Users who need contact-modify in GPU mode now get correct behavior with no code change.
- **`ovphysx`'s Python `import` now auto-registers ovphysx's namespaced USD schema/plugin path** by appending it to `OV_PXR_PLUGINPATH_2511` once at module load. Eliminates a class of silent-schema-drop bugs in mixed-process apps (`import ovphysx; import ovrtx` or vice versa) where USD's `PlugRegistry` is populated lazily on first stage open and never re-scans, so any subsystem that didn't get its plugin path published *before that moment* has its applied schemas silently dropped. The implicit registration in `CarboniteLoader` continues to cover the ovphysx-only-in-process case; this Python-side change is purely additive. The auto-call is pure-Python (env-var append only) and does not trigger native loading -- USD and `_bindings` remain deferred to first native-attribute access (`PhysX`, `ContactBinding`, ...). Failures (e.g. partially-installed checkout with no `plugins/usd` directory) are surfaced as a logged warning rather than raised so `import ovphysx` does not hard-fail. The existing public `register_schema_paths()` is unchanged and remains idempotent (already-called guard); apps that called it explicitly continue to work and the explicit call becomes a cheap no-op on success. C/C++ apps are unaffected -- still use `ovphysx_register_schema_paths()` explicitly. **Migration:** none required. Apps that need an env-var-pure import can `os.environ.pop("OV_PXR_PLUGINPATH_2511")` after `import ovphysx`.

### Added
- **`add_subdirectory()` support for source-link builds.** Users can now include ovphysx in their own CMake project via `add_subdirectory()` instead of `find_package()`, enabling an edit-rebuild workflow where changing ovphysx sources automatically recompiles them. Dependencies are auto-fetched at configure time (`OVPHYSX_FETCH_DEPS=ON`). See the [source-link tutorial](tutorials/source_link_build.md) and `tests/c_samples/hello_world_source_link/` for a complete example.
- **`ovphysx_attach_ovstage()` / `ovphysx_update_from_ovstage()` / `ovphysx_detach_ovstage()` for ovstage consumer integration.** Pair an `ovstage_instance_t*` with an ovphysx instance at the initial read ordinal, then explicitly drain subsequent committed edits by passing the producer-owned ordinal range to `ovphysx_update_from_ovstage(handle, from_ordinal, to_ordinal)`. Init-only attach rejects re-attach with `OVPHYSX_API_ERROR`; detach is idempotent. Python: `PhysX.attach_ovstage(stage)` accepts an `ovstage.Stage` or raw handle, `PhysX.update_from_ovstage(from_ordinal, to_ordinal)` drains later changes, and `PhysX.detach_ovstage()` detaches the caller-owned stage.
- **Cross-device staged reads AND writes in `ovphysx_read_tensor_binding` / `ovphysx_write_tensor_binding` / `ovphysx_write_tensor_binding_masked`.** A CPU-declared `src`/`dst`/`mask`/`index` tensor now works against a GPU-bound binding, and a GPU-declared tensor works against a CPU-bound binding: the call transparently allocates a staging buffer on the binding's device (or host) and routes through `IOptionalCuda::memcpyDtoH` / `memcpyHtoD`. Reads stage-and-copy after the simView read; writes copy-and-stage before the simView write. Same-device combinations are unchanged. Cross-GPU (different ordinals) still returns `OVPHYSX_API_DEVICE_MISMATCH`.
- **Schema path pre-registration API for multi-subsystem USD processes.** New C API `ovphysx_register_schema_paths()` and Python helper `ovphysx.register_schema_paths()` append ovphysx's namespaced USD schema/plugin root to `OV_PXR_PLUGINPATH_2511` before ovphysx initialization. Use this with peer subsystem equivalents such as `ovrtx_register_schema_paths()` before the first USD stage open or schema-registry access.
- **Safer behavior when loading into a process with another Carbonite-owning library.** Previously, ovphysx would blindly load its own USD-dependent plugin stack on top of any framework already bootstrapped in the process. The USD-population availability guard is now narrowed to fire only in full-host mode rather than any-usdrt-user, so partial hosts that provide population utilities but not the physics-schema plugins now trigger ovphysx's own physics-schema plugin load instead of silently skipping it. In-process coexistence remains subject to ABI compatibility of the shared deps that both libraries link against (Fabric interface version, Carbonite version, USD build-package); when those don't line up, the new drift diagnostics (see `OVPHYSX_COEXIST_DIAGNOSTICS` and `config.toml build_package` below, plus the refuse-branch in `CarboniteLoader::initialize()` under *Changed or Fixed*) convert the failure into a named, actionable error instead of a silent plugin-resolution cascade.
- **`OVPHYSX_COEXIST_DIAGNOSTICS=1` diagnostic env var.** Set to enable stderr-only diagnostic logging of the plugin-load and Fabric-acquire paths inside `ovphysx_create_instance` -- useful when triaging a "Dependency: [omni::physics::schema::IUsdPhysics v1.1] failed to be resolved" cascade or "Fabric interfaces unavailable" error in a coexistence context. Logs the Framework pointer, plugin-registry contents, `omni.physicsschema.plugin` registration state, and the Fabric interface-version list as seen by ovphysx. Unset: no behavior change.
- **`OVPHYSX_COEXIST_REFUSE=1` opt-out env var.** Restores the legacy fail-fast-at-load behavior for users who would rather not attempt coexistence with another Carbonite-owning library. With the default coexistence flip (see *Changed or Fixed* below), `OVPHYSX_COEXIST_REFUSE=1` is the explicit off-ramp: ovphysx detects a foreign Carbonite framework and refuses to initialize with a sharp error message instead of proceeding. Default unset: ovphysx proceeds with coexistence and lets per-phase interface probes name any version skew at the failing plugin.
- **Per-phase plugin-interface probes after static registration / `loadPlugins()`.** `CarboniteLoader::initialize()` now runs `tryAcquireInterface<>` against the primary interface of each core plugin loaded during static registration (`carb.dictionary`, `carb.settings`, `carb.tokens`, `carb.tasking`, `carb.filesystem`). If any returns null -- because a foreign-host plugin advertises an incompatible major/minor or because the plugin failed to load -- ovphysx fails fast with a single error line that names every plugin that did not satisfy ovphysx's expected interface version. Standalone ovphysx is unaffected. The probe converts what was previously an opaque `nullptr` cascade two layers down into a named, actionable error at load time, mirroring the existing `UsdVersionCheck` pattern for USD.
- **USD build-package drift diagnostic in `config.toml`.** `config.toml` now records the exact USD packman package identifier ovphysx was linked against (e.g. `build_package = "0.25.11.kit.2-gl.19811"`, auto-populated at build time from the resolved `_build/target-deps/usd/release` symlink). At runtime, `UsdVersionCheck` resolves the loaded USD's canonical path, extracts its packman id, and compares. If the two differ -- e.g. ovphysx built against `0.25.11.kit.2` but another library preloaded `0.25.11.kit.1` -- a single warning line names both ids and points at Carbonite-plugin-resolution failure as the probable downstream symptom. Same-build case is silent.
- **New `--devschema` flag** forwards to ovruntime for local physics schema builds.
- **Multi-GPU scene distribution.** Pass `active_cuda_gpus="0,1,2"` (all N GPUs) or `active_cuda_gpus="1,2"` (all except GPU 0) to distribute scenes round-robin across the specified devices. Internally maps to `/physics/sceneMultiGPUMode = eAll` or `eSkipFirst`. Other patterns (e.g. `"0,2"` on a 4-GPU machine) return `OVPHYSX_API_INVALID_ARGUMENT` with a descriptive error; arbitrary subset support can be added in a future release without API changes.
- **TensorBindingsAPI: standalone rigid body read-only queries** - `RIGID_BODY_ACCELERATION` (`[N,6]`), `RIGID_BODY_INV_MASS` (`[N]`), and `RIGID_BODY_INV_INERTIA` (`[N,9]`). The enum values use previously unused slots after `RIGID_BODY_WRENCH`; existing tensor enum values were not reordered.
- **Articulation kinematic update API.** New C `ovphysx_update_articulations_kinematic()`, Python `PhysX.update_articulations_kinematic()`, and experimental C++ `PhysX::updateArticulationsKinematic()` recompute articulation link poses from current DOF positions. In GPU mode the first call may perform the standard DirectGPU warmup step; after warmup, the FK refresh does not run a normal simulation step or collision/contact work.
- **TensorBinding row metadata for tensor bindings.** Python `TensorBinding.prim_paths` and C `ovphysx_tensor_binding_get_prim_paths()` return the resolved USD prim path for each rigid-body tensor row and the articulation root prim path for each articulation row.
- **TensorBindingsAPI deformable support.** Added volume and surface deformable
  body tensors (simulation positions, velocities, rest positions, simulation
  element indices) plus volume collision element indices
  (`DEFORMABLE_COLLISION_ELEMENT_INDICES_S32`, `[N,F,4]`, int32, read-only;
  K=4 matches the tetrahedral element connectivity exposed by
  `GpuVolumeDeformableBodyView`). Surface deformable tensors use
  `createSurfaceDeformableBodyView` at bind time and share the same
  read/write/masked-write paths as volume.
  All body tensors bind to runtime deformable body views and require DirectGPU
  mode. Added deformable material tensors for dynamic
  friction, Young's modulus, and Poisson's ratio; material tensors are
  CPU-backed. Fixed masked write for deformable material tensors
  (`setDynamicFrictionMasked`, `setYoungsModulusMasked`,
  `setPoissonsRatioMasked`). Simulation and collision element index tensors
  report signed int32 DLPack metadata.
- **Python tensor-binding empty-match guard.** `PhysX.create_tensor_binding(..., raise_if_empty=True)` now raises `ValueError` when the binding would match zero prims. The default remains `False`, preserving zero-count bindings for optional scene elements.
- **Python binding lifetime warnings.** `TensorBinding` and `ContactBinding` now emit `ResourceWarning` if garbage collection cleans them up without an explicit `destroy()` or context-manager exit. This does not affect normal read/write paths; create bindings once outside simulation loops and reuse or destroy them explicitly.
- **ContactBinding detailed contact/friction reads.** Added `sensor_paths`, `filter_paths`, `max_contact_data_count`, C `ovphysx_contact_binding_get_sensor_paths()`, `ovphysx_contact_binding_get_filter_paths()`, `ovphysx_get_contact_binding_capacity()`, `ovphysx_read_contact_data()`, `ovphysx_read_friction_data()`, and matching Python methods. The API uses flat `[C,*]` buffers with `[S,F]` count/start-index tensors. Detailed reads require filtered bindings (`filters_per_sensor > 0`); unfiltered bindings remain valid for aggregate net-force reads.
- **Experimental C++ `CreateArgs` type and `PhysX::create(out, CreateArgs)`.** `CreateArgs` is a safe C++ wrapper for `ovphysx_create_args` that default-constructs to `OVPHYSX_CREATE_ARGS_DEFAULT` and exposes setters for `device`, `active_cuda_gpus`, `bundled_deps_path`, and config entries. Replaces the previous `create(PhysX&, config_entries, count)` overload.
- **OmniPVD recording support via typed config.** Two new config entries enable `.ovd` recording: `omnipvd_output_enabled` (bool) and `omnipvd_ovd_recording_directory` (string). Both must be set at instance creation via `PhysXConfig` (Python) or `config_entries` on `ovphysx_create_args` (C/C++). These entries were previously available only via `carbonite_overrides`; they now have first-class typed fields. See [OmniPVD Recording tutorial](tutorials/omnipvd_recording.md).
- **Source builds now use namespaced USD.** ovphysx builds against namespaced monolithic USD (`PXR_NS=ov`) so it can coexist with host applications that already load another USD runtime. **Migration for source builds:** update build scripts to fetch namespaced USD packages; classic modular USD is no longer supported.

### Changed or Fixed
- **ContactBinding device-mismatch errors now explain the DirectGPU opt-in.**
  When a CPU-backed ContactBinding receives a CUDA output tensor, the error now
  points out that `device="gpu"` enables GPU dynamics only and that
  TensorAPI/ContactBinding CUDA views require `/physics/suppressReadback=true`
  before instance creation. Behavior is unchanged.
- **ovphysx startup now appends its namespaced USD schema path when `OV_PXR_PLUGINPATH_2511` is already set.** Previously `CarboniteLoader` only set the variable when it was empty, so pre-existing ovrtx or application paths could prevent ovphysx's own schema root from being published. Startup now shares the same append/dedupe logic as `ovphysx_register_schema_paths()` and does not modify `PXR_PLUGINPATH_NAME`. Startup and explicit pre-registration fail fast if ovphysx cannot find an existing `plugins/usd` directory, rather than publishing a missing path that USD silently ignores.
- **Improved automatic GPU selection on multi-GPU systems.** When the physics device is not set explicitly, ovphysx now prefers the GPU already in use by the renderer, and otherwise picks the discrete GPU with the most memory. Previously device 0 was always chosen, which could result in physics running on a different GPU than the renderer on multi-GPU workstations. Users on single-GPU machines are unaffected; users on multi-GPU systems may see a different device selected than before.
- **`ovphysx_clone()` now preserves `xformOp:scale` from source prims.** Previously, clones of prims with non-unit scale appeared at incorrect world-space size. The fix reads the source's scale (supporting both `double3` and `float3` attributes), writes it to each target, includes it in `xformOpOrder`, and incorporates it into the `localMatrix`.
- **Contact binding now matches runtime-cloned sensor bodies.** Runtime clones created with `ovphysx_clone()` can exist only in Fabric/usdrt, where copied USD API-schema metadata may not be visible. Contact binding now keeps the strict `PhysxContactReportAPI` requirement for real USD prims while allowing clone-only runtime paths to validate through their PhysX actor pointer, so cloned environments produce the same contact-binding rows as rigid-body tensor bindings.
- **`ovphysx_step_sync()` now calls `ensure_physics_attached()` before stepping**, matching the async `ovphysx_step()` path. Previously, the first `step_sync` after stage ingestion could run against an unattached stage, so features like OmniPVD recording finalization silently failed.
- **Config entries are now applied before PhysX plugin loading** so that settings read during `createPhysics()` (e.g., OmniPVD recording) are in place at initialization time.
- **TensorBindingsAPI shape property tensors now accept CPU buffers in GPU simulations.** Rigid-body and articulation material/contact/rest-offset tensors are PhysX parameter tensors and remain CPU-resident even when state tensors use CUDA buffers.
- **ovphysx wheel bootstrap no longer sets `PYTHONHOME`.** Fixes mis-resolution of `sys.prefix` in child processes that spawn venv interpreters (Windows; e.g. rerun.io).
- **Unified build flags.** `build.sh` / `build.bat` now accept the same flags as ovruntime: `-c`/`--clean`, `-x`/`--rebuild`, `-d`/`--debug`, `-r`/`--release` (default), `-t`/`--target <name>`, `-g`/`--generate`. **Note:** default is now release-only (was debug+release); `-t` is now `--target <name>` (use `--test-venv` for the old test-venv behavior).
- **`create_articulation_view()` and `create_rigid_body_view()` pattern matching extended to reach nested bodies.** Patterns now support `**` for recursive descent and -- by default -- match a named final component against descendants at any depth so patterns like `/World/envs/env_*/Robot/<name>` keep working on deeper hierarchies. Views dedup by backing PhysX pointer. Set `/physics/tensors/recursiveLeafPatternMatch` to `false` to restore strict per-level matching. **Migration:** existing patterns may pick up additional prims at deeper levels. If that changes behavior, either set the Carbonite flag to `false` or switch to explicit absolute paths.
- **`/ovphysx/latest/` on GitHub Pages now serves the docs directly** instead of redirecting to `/ovphysx/<version>/`. This fixes deep links such as the PyPI Changelog URL (`/latest/changelog.html`) which previously 404'd. Bookmarks to `/latest/` continue to work; bookmarks to versioned URLs are unchanged.

### Removed
- **Environment variables `OVPHYSX_ROOT`, `OVPHYSX_BIN_DIR`, `OVPHYSX_PLUGINS_DIR` removed.** All runtime paths are now derived from `OVPHYSX_LIB` (the single dev-mode override for the shared library path). **Migration:** replace any of the removed env vars with `OVPHYSX_LIB` pointing at the shared library.
- **`ovphysx_set_shutting_down()` removed.** Full teardown now happens automatically when the last instance is destroyed via `ovphysx_destroy_instance()` (C/C++) or `release()` (Python). **Migration:** remove any `ovphysx_set_shutting_down()` calls.
- **Legacy Python TensorAPI compatibility removed.** `ovphysx.tensors` is no longer shipped or exported. **Migration:** replace `ovphysx.tensors` imports with TensorBindingsAPI: use `physx.create_tensor_binding()` to create bindings, then `binding.read()` / `binding.write()` for data access.

## [0.3] - 2026-03-31

### Breaking Changes
- **Settings API replaced with typed config system.** The old string-based settings (`settings_keys`/`settings_values`/`settings_count` on `ovphysx_create_args`; `ovphysx_set_global_setting()`/`ovphysx_get_global_setting()`; Python `PhysX(settings=...)`, `set_setting()`, `get_setting()`; C++ `PhysX::setSetting()`/`getSetting()`) are all removed. **Migration:** C — use `ovphysx_config_entry_t` array on `config_entries`/`config_entry_count` with builders from `ovphysx_config.h`, runtime `ovphysx_set_global_config()` and typed getters `ovphysx_get_global_config_bool/int32/float/string()`. Python — use `PhysX(config=PhysXConfig(num_threads=4))`, runtime `set_config_bool()`/`set_config_int32()` and matching getters. Arbitrary Carbonite paths remain accessible via `ovphysx_config_entry_carbonite()` (C/C++) or `PhysXConfig(carbonite_overrides={...})` (Python).
- **`OVPHYSX_CONFIG_CUDA_DEVICE` removed** from `ovphysx_config_int32_t`. **Migration:** use `active_cuda_gpus` on `ovphysx_create_args` (C) or `PhysX(active_cuda_gpus=...)` (Python). Setting `/physics/cudaDevice` via `carbonite_overrides` now raises an error.
- **Removed typed config entries** for settings not in Physics Preferences: `OVPHYSX_CONFIG_PHYSX_DISPATCHER`, `OVPHYSX_CONFIG_OMNI_PVD_OUTPUT_ENABLED`, `OVPHYSX_CONFIG_UJITSO_COLLISION_COOKING` (bool); `OVPHYSX_CONFIG_PVD_RECORDING_DIRECTORY` (string). **Migration:** these settings remain accessible via `carbonite_overrides`.
- **Log level enum reordered to ascending severity** (matches ovrtx and industry convention). New values: `OVPHYSX_LOG_VERBOSE=0`, `OVPHYSX_LOG_INFO=1`, `OVPHYSX_LOG_WARNING=2` (unchanged), `OVPHYSX_LOG_ERROR=3`, `OVPHYSX_LOG_NONE=4`. **Migration:** code using symbolic names is unaffected; code using raw integer values must update.
- **Error handling switched from inline error strings to thread-local `get_last_error()` query.** `ovphysx_result_t` and `ovphysx_enqueue_result_t` no longer have an `error` field. **Migration:** on failure, call `ovphysx_get_last_error()` on the same thread to retrieve the error message (valid until the next ovphysx API call on that thread). For `wait_op`, iterate `error_op_indices` and call `ovphysx_get_last_op_error()` per failed index, then `ovphysx_destroy_wait_result(&result)`. Removed: `ovphysx_destroy_error()`, `ovphysx_destroy_errors()`. Python API is unaffected (errors are raised as exceptions).
- **`ContactEventHeader.stageId` type changed from `long` to `int64_t`** for cross-platform ABI stability. This changes the struct layout on Windows.
- **`ovphysx_get_contact_report()` C signature changed**: now returns typed pointers (`const ovphysx_contact_event_header_t**`, `const ovphysx_contact_point_t**`) instead of `const void**`, and gained 2 new parameters (`out_friction_anchors`, `out_num_friction_anchors`). **Migration:** update pointer types and append `, NULL, NULL` for friction anchor parameters. C++ callers are unaffected (new params default to `nullptr`).
- **Removed** `ovphysx_articulation_get_dof_count`, `_body_count`, `_joint_count`, `_is_fixed_base`, `_fixed_tendon_count`, `_spatial_tendon_count`. **Migration:** use `ovphysx_get_articulation_metadata(handle, binding, &meta)`. Python `TensorBinding` properties are unchanged.
- **`ovphysx_clone()` parameter `parent_positions_xyz` replaced with `parent_transforms`** (7 floats per target: px, py, pz, qx, qy, qz, qw). **Migration:** replace `parent_positions_xyz=[x, y, z]` with `parent_transforms=[px, py, pz, 0, 0, 0, 1]` (append identity quaternion).
- **Removed** `ovphysx_set_clone_env_root()` (C) and `set_clone_env_root()` (Python). **Migration:** no replacement needed — the first `clone()`, `simulate()`, or `warmup_gpu()` call triggers stage attach lazily.
- **`ovphysx_device_t` enum reordered:** `OVPHYSX_DEVICE_AUTO = 0` (was 2) so zero-initialized args default to AUTO. **Migration:** code using symbolic names is unaffected; code using raw integers must update (`0=AUTO, 1=GPU, 2=CPU`).
- **`OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_FORCE_F32` renamed** to `ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE` (enum value 72 unchanged). **Migration:** update references to the old name.
- Scene query enum constants disambiguated: `OVPHYSX_SCENE_QUERY_CLOSEST` → `OVPHYSX_SCENE_QUERY_MODE_CLOSEST` (and `_ANY`, `_ALL`); `OVPHYSX_SCENE_QUERY_SPHERE` → `OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE` (and `_BOX`, `_SHAPE`). **Migration:** update to the new constant names.
- **Python enum cleanup.** All `OVPHYSX_TENSOR_*_F32` bare-int constants removed (use `TensorType.*` members); `OVPHYSX_API_*`, `OVPHYSX_LOG_*`, `OVPHYSX_DEVICE_*` constants removed (use `ApiStatus.*`, `LogLevel.*`, `DeviceType.*`); `OVPHYSX_OP_INDEX_ALL` renamed to `OP_INDEX_ALL`; `kDLCPU`, `kDLCUDA`, `kDLInt`, `kDLUInt`, `kDLFloat` removed (use `DLDeviceType.kDLCPU` / `DLDataTypeCode.kDLFloat`).
- **Removed `ovphysx_finalize()`**. Use `ovphysx_shutdown()` for process lifecycle shutdown and `ovphysx_destroy_instance()` for handles. **Migration:** remove any `ovphysx_finalize()` calls.

### Added
- `ovphysx_articulation_metadata_t` struct and `ovphysx_get_articulation_metadata()` — fills 6 scalar topology fields in one call, one lock acquire, one stream fence.
- `OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE` (int32) config entry for `/physics/sceneMultiGPUMode`.
- `ConfigBool`, `ConfigInt32`, `ConfigFloat`, `ConfigString` IntEnums in Python for typed config key access.
- `PhysXConfig` dataclass in Python for typed initialization config.
- Remote USD loading through ovstage population accepts remote URIs (`omniverse://`, S3, Azure Blob). Use HTTPS virtual-hosted S3 URLs. Use `ovphysx_configure_s3()` / `configure_s3()` and `ovphysx_configure_azure_sas()` / `configure_azure_sas()` for credential setup before population.
- TensorBindingsAPI: DOF property tensors — stiffness, damping, limits, max velocity, max force, armature, friction properties (read/write, indexed, masked).
- TensorBindingsAPI: body property tensors — mass, center-of-mass pose, inertia tensor (read/write, indexed, masked).
- TensorBindingsAPI: link acceleration tensor (`[N, L, 6]`, read-only).
- TensorBindingsAPI: dynamics query tensors (read-only) — Jacobian, generalized mass matrix, Coriolis + centrifugal forces, gravity compensation, link incoming joint force, DOF projected joint forces.
- TensorBindingsAPI: standalone rigid body properties — mass, inertia, COM pose (read/write with indexed/masked write support). Articulation body inverse mass and inverse inertia (read-only).
- TensorBindingsAPI: fixed tendon properties — stiffness, damping, limit stiffness, limits, rest length, offset (read/write, indexed, masked).
- TensorBindingsAPI: spatial tendon properties — stiffness, damping, limit stiffness, offset (read/write, indexed, masked).
- TensorBindingsAPI: shape-level tensors — `RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION` (`[N,S,3]`), `RIGID_BODY_CONTACT_OFFSET` (`[N,S]`), `RIGID_BODY_REST_OFFSET` (`[N,S]`), and articulation equivalents. All support read, indexed write, and masked write.
- Articulation metadata name queries: `ovphysx_articulation_get_dof_names`, `get_body_names`, `get_joint_names`.
- Contact binding API: `ovphysx_create_contact_binding` / `destroy` / `get_spec` / `read_net_forces` / `read_force_matrix`. Python `ContactBinding` class for reading contact forces via DLPack tensors.
- PhysX object interop: `ovphysx_get_physx_ptr()` returns raw PhysX SDK pointers by USD prim path and type enum. SDK ships PhysX headers under `include/physx/`. See [PhysX Interop tutorial](tutorials/physx_interop.md).
- Contact report API: `ovphysx_get_contact_report()` exposes per-step contact data as typed C struct pointers. Python `PhysX.get_contact_report()` returns a dict with ctypes arrays.
- Scene query API: `ovphysx_raycast()`, `ovphysx_sweep()`, `ovphysx_overlap()` — raycast, geometry sweep, and overlap queries. Supports CLOSEST/ANY/ALL hit modes with sphere, box, and arbitrary-shape geometry.
- Synchronous stepping: `ovphysx_step_sync()` / `step_sync()` combines step + wait into one call. `ovphysx_step_n_sync()` / `step_n_sync()` batches N steps.
- `OVPHYSX_API_BUFFER_TOO_SMALL = 6` status code.
- CMake package config: `find_package(ovphysx)` provides `ovphysx::ovphysx` target and Windows `ovphysx_copy_runtime_dlls()` helper.
- Visual sample using [Rerun](https://rerun.io) for rigid body visualization ([Rendering Handoff](tutorials/render_handoff.md)).
- Linux aarch64 support.

### Changed or Fixed
- Physics scene parsing is deferred until the first simulation attach path. This avoids GPU buffer corruption when `clone()` adds environments after the initial scene update. Correct sequence: ovstage population + `attach_ovstage()` -> `clone()` -> `warmup_gpu()`/`step()`; call `update_from_ovstage()` only for later authored edits.
- A GPU and CUDA installation is no longer required for CPU-only simulation.
- Fixed a fatal TF_DEBUG crash when ovphysx and OVRTX co-loaded in Python due to a collision with USD libraries.
- Fixed various memory leaks across C, C++, and Python error-handling and cleanup paths.
- Contact binding read functions now validate dtype is float32 before dispatching.
- `ContactBinding.destroy()` now checks status and raises on failure; validates parent SDK is alive.
- `create_contact_binding` now validates `sensor_patterns` is non-empty and `filter_patterns` length.
- Wheel metadata corrected: `Root-Is-Purelib: false`, OS classifiers set to Linux and Windows.
- Fixed shutdown crash where stepper tasks could access freed CUDA context.
- Fixed process-exit crashes caused by non-deterministic DLL/SO unload ordering. `ovphysx_destroy_instance()` now performs full teardown when the last instance is destroyed.
- `ovphysx_remove_usd()` now validates `usd_handle` and returns `OVPHYSX_API_NOT_FOUND` for unknown handles.

### Removed
- `ovphysx.pc` (pkg-config file) removed; CMake is the supported integration path.
- `hdStorm` (Hydra Storm renderer) removed from SDK; not needed for headless physics simulation.

## [0.2] - 2026-03-03

First released version. No changelog was maintained prior to v0.3.
