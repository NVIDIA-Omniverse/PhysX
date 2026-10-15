---
name: ovphysx-session-write
description: Push caller-owned bulk control inputs and state into an ovphysx simulation with the session write API — Python `PhysX.write_sync()` or the `PhysX.query()` + `PhysX.write()` operations, C `ovphysx_write()`. Covers writing rigid-body and articulation state and control (poses, velocities, joint positions/velocities and their targets, forces and wrenches, mass properties) for one or more attributes per session by filling borrowed write groups, identifying each by its request index, and committing them, then stepping. The successor to the deprecated tensor-binding write; for reading simulation output back, use the ovphysx-output-read skill.
license: Apache-2.0
compatibility: "Requires matching ovphysx headers/library with the task-based session API and an attached ovstage Stage; supports the Python wheel and C SDK. Python session writes create Warp arrays and need a writable Warp cache directory. If the default cache location is not writable, set WARP_CACHE_PATH before Warp is initialized."
metadata:
  author: "NVIDIA Omniverse Physics Team"
  version: "0.3.0"
  tags: "ovphysx, ovstage, physics-write, control-input"
---

# Write Control Inputs into an ovphysx Simulation

Use the session write API to push caller-owned buffers — control inputs and
state — into the simulation: apply velocities, teleport poses, drive joints
(position / velocity and their targets), apply forces and wrenches, set mass
properties, or issue wake/sleep commands. It is the mirror of the output read and the successor to the
deprecated tensor-binding write. For reading simulation output back (borrowed
warp arrays, ovstage identity, closed-loop write-back), use the
`ovphysx-output-read` skill.

## Workflow

1. Select the language branch and read its bundled reference:

   - Python: [Python session writes](references/python.md)
   - C: [C write-session lifecycle](references/c_api.md)

   Read only the selected language reference. Select the simulated object type,
   the attributes to write, and the source buffer's device.

   **Complete when:** the language, object type, attributes, and source device
   are explicit.

2. Keep an ovstage Stage attached and alive. When attachment and process
   lifecycle are not already established, invoke the `basic-workflow` skill by
   name.

   **Complete when:** the attached Stage owns the shared path dictionary for the
   whole write.

3. Warm up or step BEFORE the first write if you need a recipe that also works
   on DirectGPU. On CPU and on GPU-with-readback a write issued before the
   scene's first step is APPLIED, not refused. On DirectGPU (`suppressReadback`)
   commit is REFUSED until a first step has sized the GPU scatter view. In every
   mode the write never auto-warms: it never steps for you. Call `warmup()` /
   `ovphysx_warmup()` or `step_sync()` first when you want one sequence for all
   three modes.

   **Complete when:** the caller has either warmed or stepped, or has accepted
   that CPU/GPU-with-readback will apply a pre-step write and DirectGPU will not.

4. Open a write session for the attributes, fill EVERY entry of every group, then
   commit each group. A session carries every attribute named when it is opened,
   and each fetched group reports its request position as `attribute_index` --
   identify a group's attribute by that index, never by tensor shape or fetch
   order. Group tensors are mutable views onto runtime-owned storage; fill them
   in place. Commit publishes the WHOLE group, so a partially filled group
   publishes whatever its unfilled entries contain -- fill every tensor of every
   group before committing. A group that is never committed is discarded and
   publishes nothing.

   **Complete when:** every group intended to publish is fully filled and
   committed, and any abandoned fill committed nothing.

5. Complete the simulation step that consumes the write, then read results back.
   Control inputs marked write-only (forces, wrenches) are consumed and cleared
   each step; read outcomes through a STATE attribute, not the control one.
   Invoke the `ovphysx-output-read` skill for the read.

   **Complete when:** the step has run and simulated results are read from a
   state attribute, not the control input.

6. Release resources and validate the selected branch. Python closes the
   `WriteSession` and any explicit `Query` with their context managers; a
   `write_sync()` session waits for its releases on exit, and the default
   operations' releases are collected with `wait_all()`. C releases the write and
   query handles on success and error paths. Exercise an integration test that
   writes, steps, and observes the intended CONSEQUENCE (motion, pose change)
   rather than reading a write-only input back.

   **Complete when:** every session is released before `PhysX.destroy()`, and
   the test observes the expected physical consequence.

## Shared Contracts

- Native query/read/write, fetch, commit and release operations return task
  receipts. Check admission, wait for that receipt, then consume outputs. Fetch
  exhaustion is a successful wait with a null group. Copy task diagnostics before
  destroying wait-result storage. Python's default session calls return an
  `Operation`; `wait()` collects it and raises `TaskError` on failure, and
  `write_sync()` / `commit_sync()` / `close_sync()` wait internally.
- Query by simulated type, not USD schema; narrow it to specific prims with a
  prim-set selection when needed. Identify objects through each write group's own
  prim list; read/write partitions and fetch order need not match.
- Several attributes per session: pass them in one list. The open validates all
  of them first -- a duplicate (including a string and a token naming the same
  attribute), a token that does not resolve in the attached Stage's dictionary,
  or an unsupported or read-only name fails the whole open. Commits are separate
  and not transactional: commit order is execution order, and an earlier
  successful commit stays applied when a later one fails.
- The writable set is listed by object type in **Writable attributes by object
  type** below. The writability query (`ovphysx_writability`) gives the
  authoritative classification of every (object type, attribute) pair as
  WRITABLE, WRITE_ONLY, READ_ONLY, or CONDITIONAL with no scene or step -- C
  callers should consult it rather than guessing whether a name is writable.
  It has no *public* Python binding (`hasattr(ovphysx, 'writability')` is
  `False`); `ovphysx._bindings.writability(object_type, attribute)` is a
  private ctypes wrapper around the same call (used by the project's own
  `test_writability.py`), not a supported/stable public API -- prefer the
  table below, or attempt `physx.write_sync(...)`, which raises `TaskError` (a
  `RuntimeError`) for an attribute that is not writable for the object type,
  rather than writing nothing silently.
- Fill EVERY mapped entry before committing. There is no fill mask on the group:
  a committed group publishes every entry, so an unfilled one publishes stale
  memory. To write only some bodies, select their prims with a prim-set query
  (C `ovphysx_query_selection`, Python `prim_paths=` / `pattern=`); there is no
  kinematic-body filter, and neither `ALL` nor `ACTIVE` scope provides one.
- Uncommitted groups are discarded when the session closes. A failed commit can
  partially apply its group and consumes it; previously committed groups remain
  applied. Copy the task diagnostic before destroying its wait result;
  Python includes it in the exception.
- `force` is a vec3 applied at the CENTRE OF MASS. `wrench` is nine wide --
  `[fx,fy,fz, tx,ty,tz, px,py,pz]` = force, torque, and a WORLD application
  point -- so use `wrench` (force at the point, torque zero) for a load away
  from the COM, and `wrench` (force zero, torque set) for a pure torque. Both
  are WORLD frame and WRITE-ONLY: the solver consumes and clears them each step,
  so there is nothing to read back -- assert their consequence.
- `wake` is a one-time command for standalone rigid dynamics: nonzero wakes, zero sleeps.
  Fill every mapped row; zero is not a leave-unchanged value. Use `scope=ALL` to include
  sleeping bodies. A prim-set selection can narrow the command to chosen bodies, including
  sleepers. CPU and GPU-with-readback scenes
  support it; DirectGPU refuses it. Kinematic and simulation-disabled bodies are skipped,
  and sleeping-disabled scenes ignore the sleep command. Articulation-link writes are refused.
  Read `isSleeping` separately to observe state (1 asleep, 0 awake); neither the last command
  nor ACTIVE membership proves current state. The command takes effect at commit.
- Angular joint columns are per-AXIS and in DEGREES (coordinates and rates);
  gains are per degree and efforts are unconverted. Do not apply a blanket
  radian conversion. A prismatic axis stays in the stage's base length unit.
- Inspect `tensor.device` per group. A GPU scene may stage a write through the
  host, so the write device can differ from the read; commit a CUDA group with
  its producer stream or event (`cuda_stream=` / `cuda_wait_event=` in Python,
  `ovstage_cuda_sync_t` in C). They name your fill work, not the stream physics
  uses; the default-stream value `1` refers to the CUDA context current during
  the commit, so make the producer's context current (Python `wp.ScopedDevice`).
- Read simulated results from a STATE attribute, not the control input you
  wrote: a velocity-target write reads back the target, not the physics outcome.

## Writable attributes by object type

Pass each attribute as a write string -- Python `physx.write_sync(type, ["name"])`
or `physx.write(query, ["name"])`, C the matching `OVPHYSX_ATTR_*` macro. This is
the current writable set; a
**write-only** input has no read-back (assert its consequence) and `jointLimit`
is **conditional** -- see the notes under the table.

| `SimObjectType` | Writable attributes |
|---|---|
| `RIGID_BODY` (incl. point-instancer instances) | State: `position`, `orientation`, `linearVelocity`, `angularVelocity`. Mass: `mass`, `inertia`, `centerOfMassPosition`, `centerOfMassOrientation`. Flags: `disableGravity`, `disableSimulation` (standalone bodies only -- not writable per point-instancer instance; see the note below the table). Per-shape: `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`. **Write-only:** `force`, `wrench`. **Conditional, standalone-only:** `wake` (uint8). **Conditional:** `kinematicTargetPosition`, `kinematicTargetOrientation`. |
| `ARTICULATION` (root) | `rootPosition`, `rootOrientation`, `rootLinearVelocity`, `rootAngularVelocity`. |
| `ARTICULATION_JOINT` (per DOF axis) | State/control: `jointPosition`, `jointVelocity`, `jointPositionTarget`, `jointVelocityTarget`, `jointActuationForce`. Drive props: `jointStiffness`, `jointDamping`, `jointMaxVelocity`, `jointMaxForce`, `jointArmature`, `jointStaticFriction`, `jointDynamicFriction`, `jointViscousFriction`, `jointSpeedEffortGradient`, `jointMaxActuatorVelocity`, `jointVelocityDependentResistance`, `jointDriveType`. **Conditional:** `jointLimit`. |
| `ARTICULATION_LINK` | Mass: `mass`, `inertia`, `centerOfMassPosition`, `centerOfMassOrientation`, `disableGravity`; per-shape `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`. **Write-only:** `force`, `wrench`. Link pose/velocity are READ-ONLY (derived from articulation state). |
| `PARTICLE_SET`, `DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE` | `points`, `velocities` (sim mesh). Volume deformables also accept conditional `kinematicTarget`. |
| `DEFORMABLE_MATERIAL` | `deformableDynamicFriction`, `deformableYoungsModulus`, `deformablePoissonsRatio`, `deformableElasticityDamping`; surface materials also `deformableBendingStiffness`, `deformableThickness`, `deformableBendingDamping`. |
| `VEHICLE` | **Conditional, DriveBasic/DriveStandard:** `accelerator`, `brake0`, `brake1`, `steer`. **Conditional, write-only:** `requestedGear` (managed drives), `thrust0` / `thrust1` (tanks), `resetToRestState` (uint8 command). Chassis pose/velocity use `RIGID_BODY` on the same prim. |
| `VEHICLE_WHEEL` | **Write-only controls, vehicles without a managed drive only:** `driveTorque`, `brakeTorque`, `steerAngle`. Wheel pose and telemetry are READ-ONLY. |
| `CHARACTER_CONTROLLER` | **Conditional:** `move`, a world-space displacement applied once at commit. Refused on a DirectGPU scene. Pose is the controller prim's rigid-body position and orientation. |
| `FIXED_TENDON` | `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonLimit`, `tendonRestLength`, `tendonOffset`. |
| `SPATIAL_TENDON` | `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonOffset`. |

- **WRITE-ONLY** describes readability, not persistence: `force` and `wrench` clear
  each step, vehicle controls persist until rewritten, and `wake` and `resetToRestState`
  are one-time commands. A character-controller `move` is applied once at commit and is not
  replayed. Read state to observe their consequence. Vehicle columns are
  host-resident and unavailable on DirectGPU; see the [vehicle write contract](../../docs/read_write/writable.md#whole-vehicles)
  for ranges, drive applicability and reset ordering.
- **CONDITIONAL** (`jointLimit`): a finite limit interval only lands on an axis
  whose motion is `eLIMITED`; PhysX cannot re-open a free axis in-scene, so a
  finite limit on a free axis is refused. Writing the read's unlimited sentinel
  (+/-`FLT_MAX`) back to a free axis is a legal no-op.
- `wake`, rigid-body kinematic targets and deformable `kinematicTarget` have additional
  scene/object conditions; use the [writable-data contract](../../docs/read_write/writable.md).
- **STANDALONE-ONLY** (`disableSimulation` on a point-instancer instance):
  `disableSimulation` is not an instancer-writable column and there is no
  per-instance route, so no supported path disables an individual instance -- it
  applies to standalone rigid bodies only.
- `ovphysx_writability(object_type, attribute)` is the runtime source of truth
  this table snapshots (C API only -- see the note above the table). The
  `OVPHYSX_ATTR_*` string constants it and `include/ovphysx/ovphysx_attributes.h`
  use are a C-only convenience; the wheel ships no `include/` directory and
  has no Python binding for them. A Python caller passes the same attribute
  name as a plain string (`physx.write_sync(SimObjectType.RIGID_BODY, ["position"])`),
  exactly as shown in the examples in this skill.

## Installed API Sources

For the caller's installed ovphysx version, prefer:

- C SDK headers: `include/ovphysx/ovphysx.h`,
  `include/ovphysx/ovphysx_types.h`, and `include/ovphysx/ovphysx_attributes.h`
  (the `OVPHYSX_ATTR_*` names; `ovphysx.h` does not include it)
- Python docstrings: `ovphysx.api.PhysX.write`, `PhysX.write_sync`, `Operation`,
  `WriteSession`, and `WriteGroup`
- The session read/write contract: `docs/ovstage_integration.md`

The public release documentation starts at
<https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>.
