---
name: ovphysx-output-read
description: Read ovphysx simulation output with the ovstage-native output-read API when results must retain ovstage identity or write back without rebuilding paths. Covers Python `PhysX.query()` + `PhysX.read()` operations and `PhysX.read_sync()`, C `ovphysx_query()` / `ovphysx_read()` with their optional results, emitted attribute tokens, borrowed groups, and closed-loop physics-to-ovstage write-back.
license: Apache-2.0
compatibility: "Requires matching ovphysx headers/library with the task-based session API and an attached ovstage Stage; supports the Python wheel and C SDK. Python session reads create Warp arrays and need a writable Warp cache directory. If the default cache location is not writable, set WARP_CACHE_PATH before Warp is initialized."
metadata:
  author: "NVIDIA Omniverse Physics Team"
  version: "0.4.0"
  tags: "ovphysx, ovstage, physics-output, query-read"
---

# Read ovphysx Simulation Output

Use the output-read API when results must retain ovstage prim identity, emitted
attribute tokens, and fixed/array column layout. The session write API
(`ovphysx_write` / `PhysX.write`) is the route for pushing caller-owned bulk buffers in --
see the `ovphysx-session-write` skill (the deprecated tensor-binding API also covered this);
raycast, sweep, and overlap are geometry queries.

The prim list and attribute token a result carries are **interned integers**, not paths or
names. Resolve them through the attached Stage's dictionary: `ovstage.PathDictionary(stage)` or
`result.dictionary` in Python — `get_path_strings(group.prim_list)` for the prim paths and
`token_to_string(group.attribute)` for the emitted attribute name (C:
`ovstage_get_path_dictionary(stage)` with `path_dictionary_get_paths_from_path_list` /
`path_dictionary_get_strings_from_tokens`). See `references/python.md` or
`references/c_api.md` for the selected language.

## Workflow

1. Select the language branch and read its bundled reference:

   - Python: [Python output reads](references/python.md)
   - C: [C query/read lifecycle](references/c_api.md)

   Read only the selected language reference. Select the simulated object type,
   semantic attributes, `ALL` or `ACTIVE` scope, and destination. When selecting
   `ACTIVE` or caching layout state, also read
   [Scope and layout constraints](references/scope_and_layout.md); support is
   type-dependent and the current layout metadata is not an invalidation signal.

   **Complete when:** the language, type, attributes, scope, and destination are
   explicit.

2. Keep an ovstage Stage attached and alive. When attachment and process
   lifecycle are not already established, invoke the `basic-workflow` skill by
   name.

   **Complete when:** the attached Stage owns the shared path dictionary for the
   whole read.

3. Complete the simulation work whose output will be read. Use `step_sync()` /
   `ovphysx_step_sync()`, or wait for the asynchronous step before opening the
   read.

   **Complete when:** the intended step has completed.

4. Open and drain the read exactly as the selected language reference describes.
   A zero-object match is successful and yields no groups. Request the optional
   read result (C `ovphysx_read_result_t*`, Python `metadata=True`) when the caller
   must tell an empty match from an unsupported or not-yet-ready attribute.

   **Complete when:** every returned group is consumed and normal exhaustion is
   distinguished from failure.

5. Consume each group according to its destination.

   - In Python, every non-empty numeric column is a borrowed `warp.array` on its
     native CPU or CUDA device. Use it directly in Warp; call `.numpy().copy()`
     when a caller-owned host copy is actually needed. Building a CUDA group's
     arrays orders its producer event onto the Warp stream current at that
     point (inside `read_sync()`, or at the first `wait()` / `result` of a
     `fetch_next()` operation), so no manual wait or import-time negotiation is
     needed.
   - For native C inspection, copy values that must outlive the read session.
   - For ovstage write-back, first read
     [Closed-loop ovstage write-back](references/closed_loop.md), then complete
     the write while the group's identity and borrowed data remain valid.

   **Complete when:** retained Warp arrays keep their session lease, retained
   copies are caller-owned, or the ovstage write completed before releasing its
   source read session.

6. Release resources and validate the selected branch. Python closes the
   `ReadResult` and any explicit `Query` with their context managers and collects
   the submitted releases with `wait_all()` after dropping retained arrays (a
   `read_sync()` result waits for the releases it submits on exit). C releases
   every fetched group, then the read and query
   handles on success and error paths. Exercise an integration test that checks
   values and, for write-back, ordinal separation.

   **Complete when:** every retained array is deliberate and dropped before
   `PhysX.destroy()`, supported `ACTIVE` reads are reopened per frame, structural
   changes trigger fresh query and downstream layout setup, and the test observes
   the expected output.

## Shared Contracts

- Native query/read/write, fetch, commit and release operations return task
  receipts. Check admission, wait for that receipt, then consume outputs. Fetch
  exhaustion is a successful wait with a null group. Copy task diagnostics before
  destroying wait-result storage. Python's default session calls return an
  `Operation`; `wait()` collects it and raises `TaskError` on failure, and
  `read_sync()` / `read_tokens_sync()` wait internally.
- Query by simulated type, not USD schema. To narrow a type to specific prims,
  add a prim-set selection (C `ovphysx_query_selection`, Python `prim_paths=` /
  `pattern=`); it always covers the whole selected set, so it takes no scope.
- Request semantic names such as `position`, `orientation`,
  `linearVelocity`, `angularVelocity`, `linearAcceleration`,
  `angularAcceleration`, `jointPosition`, `jointVelocity`, `points`, or
  `velocities`. Body properties (`mass`, `inertia`, `centerOfMassPosition`,
  `disableGravity`, ...) are also readable, but are always host-resident and
  the two flags are `uint8` -- check `dtype` and `device` per tensor rather
  than assuming device-resident f32. Per-shape properties (`contactOffset`,
  `staticFriction`, ...) are padded to the widest selected object in the read; pair them
  with `shapeCount` to find where each row's real values stop.
- Read `isSleeping` with `ALL` scope to inspect current rigid-body sleep state, including
  sleepers. It is a read-only host uint8 scalar: 1 asleep, 0 awake. Articulation links
  report their owning articulation's state; no point-instancer array form exists.
  CPU and GPU-with-readback scenes report PhysX state, including kinematics and disabled
  bodies. Sleeping-disabled scenes, including DirectGPU, report 0 for emitted rows;
  DirectGPU still omits pre-step partitions and disabled rows. Use `disableSimulation`
  separately for participation. ACTIVE membership, zero velocity and the last `wake`
  command do not determine sleep state. Re-read for fresh values; retained arrays are snapshots.
- Deformable bodies serve `points` and `velocities` as device columns on a
  DirectGPU scene, plus `restPoints` and the **int32** `simElementIndices`
  (`collisionElementIndices` too, on the volume type only). The topology three
  are authored arrays with no device copy, so they arrive host-resident in the
  same read as the device `points` -- branch on `device` per tensor.
- Particle sets serve `points` (or `positions`) and `velocities` as **device**
  columns, one array group per attribute carrying one tensor per set -- so walk
  `tensor_count` and pair each tensor with the prim at the same index, rather than
  assuming a group covers one set. `points` is prim-local and `velocities` is
  world, the same split deformables have. A read before the first step returns the
  authored positions. The type needs a CUDA context and emits nothing without one.
- Deformable materials are their own type, `SimObjectType.DEFORMABLE_MATERIAL`,
  keyed by the bound Material prim: `deformableDynamicFriction`, `deformableYoungsModulus`,
  `deformablePoissonsRatio`, `deformableElasticityDamping`, and on surface materials only
  `deformableBendingStiffness`, `deformableThickness` and `deformableBendingDamping`. Those last three OMIT
  volume materials rather than reporting 0.0, so pair values with prims through
  the group's own prim list -- its row count can be smaller than the query's.
- Character controllers use `SimObjectType.CHARACTER_CONTROLLER`, keyed by the
  capsule prim carrying `PhysxCharacterControllerAPI`. The type has no readable
  columns. Read the controller prim as `RIGID_BODY` `position` and `orientation`:
  the backing actor's global pose, which updates on the step after `move`.
- SDF collision shapes use `SimObjectType.SDF_SHAPE` and have no readable
  columns. Evaluate them with `physx.evaluate_sdf_shapes(points, out, pattern=...)`:
  points `[N, Q, 3]` in mesh-local space, out `[N, Q, 4]` (gradient, then
  distance), float32 on the CUDA device, DirectGPU only. Row *i* is the *i*-th
  matched shape in pattern order.
- In ovphysx 0.6.0 and later, whole articulations use
  `SimObjectType.ARTICULATION` and report the
  articulation-root API prim. They serve root pose/velocity as `rootPosition`,
  `rootOrientation`, `rootLinearVelocity` and `rootAngularVelocity` -- qualified
  because the row is keyed by the API prim, which may be an ancestor Xform rather
  than the root link -- plus `centerOfMassWorld`, `centerOfMassLocal`, and the
  padded per-shape set.
  `rootPosition` and world COM stay in scene world regardless of TensorBindings
  subspace roots; local COM uses the physical root link's center-of-mass (mass)
  frame. Root/COM columns follow the simulation device; shape columns remain on
  CPU.
- Articulation joints also serve the per-step control values
  (`jointPositionTarget`, `jointVelocityTarget`, `jointActuationForce`,
  `jointProjectedForce`) and the authored DOF properties (`jointStiffness`,
  `jointDamping`, `jointLimit`, `jointMaxVelocity`, `jointMaxForce`,
  `jointArmature`, `jointStaticFriction`, `jointDynamicFriction`,
  `jointViscousFriction`, `jointSpeedEffortGradient`,
  `jointMaxActuatorVelocity`, `jointVelocityDependentResistance`,
  `jointDriveType`). The properties behave like body properties -- always
  host-resident, so one GPU read mixes devices -- and `jointDriveType` is
  `uint8` (0 none, 1 force, 2 acceleration). `jointLimit` is one column of two
  lanes, `(lower, upper)`, authored-order regardless of the joint's body order;
  an unset bound reads back as `FLT_MAX` rather than a converted number.
  Articulation links additionally serve `linkIncomingJointForce`, six lanes of
  force xyz then torque xyz.
- On an angular axis, the unit follows the QUANTITY, not just the axis:
  coordinates and rates are degrees, gains and per-rate coefficients are per
  degree, and efforts, inertia and the drive type are unconverted. Do not
  apply a blanket radian conversion to a joint column.
- Whole articulations also serve `jacobian`, `massMatrix`, `coriolisForce`,
  `gravityForce` and `centroidalMomentum`, row-major flattened into `dtype.lanes`.
  These groups cover topology-based subsets of a type's rows: their width
  depends on each articulation's topology, so rows are split into cohorts of
  structurally identical articulations and each cohort is its own group. Read `lanes`
  per group -- one group for a homogeneous fleet, one per topology otherwise -- and
  match a cohort by its prim list. `massMatrix` is square, the two force columns are
  `[lanes]`, `centroidalMomentum` has six rows with the bias in its last column, and
  `jacobian` carries its `[rows, cols]` in the companion `jacobianShape`, which is
  emitted per cohort too -- same prims, same order -- so pair the two groups by prim
  list and read any row. `centroidalMomentum` is floating-base only and a fixed-base
  cohort simply omits it.
- Treat a whole-type query as a selector evaluated at each session open. A read
  observes the most recently completed step, not a snapshot captured when the
  query was opened. The optional query result is a creation-time snapshot, and a
  selected query keeps the membership it resolved at creation.
- Before using `ACTIVE` or caching prim order and shapes, apply the bundled
  [scope and layout constraints](references/scope_and_layout.md). Do not assume
  every object type filters `ACTIVE`, discovery counts always match an `ACTIVE`
  read, or `layout_generation` changes after structural edits.

## Installed API Sources

For the caller's installed ovphysx version, prefer:

- C SDK headers: `include/ovphysx/ovphysx.h`,
  `include/ovphysx/ovphysx_types.h`, and `include/ovphysx/ovphysx_attributes.h`
  (the `OVPHYSX_ATTR_*` names; `ovphysx.h` does not include it)
- Python docstrings: `ovphysx.api.PhysX`, `Operation`, `Query`, `ReadResult`,
  `ReadGroup`, and `ReadMetadata`

The public release documentation starts at
<https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>.
