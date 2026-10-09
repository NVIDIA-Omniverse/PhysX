---
name: ovphysx-session-write
description: Push caller-owned bulk control inputs and state into an ovphysx simulation with the session write API — Python `PhysX.write()`, C `ovphysx_write()`. Covers writing rigid-body and articulation state and control (poses, velocities, joint positions/velocities and their targets, forces and wrenches, mass properties) by filling borrowed write groups and committing them, then stepping. The successor to the deprecated tensor-binding write; for reading simulation output back, use the ovphysx-output-read skill.
license: Apache-2.0
compatibility: "Requires ovphysx >=0.6.0 and an attached ovstage Stage; supports the Python wheel and C SDK. Python PhysX.write() creates Warp arrays and needs a writable Warp cache directory. If the default cache location is not writable, set WARP_CACHE_PATH before Warp is initialized."
metadata:
  author: "NVIDIA Omniverse Physics Team"
  version: "0.1.1"
  tags: "ovphysx, ovstage, physics-write, control-input"
---

# Write Control Inputs into an ovphysx Simulation

Use the session write API to push caller-owned buffers — control inputs and
state — into the simulation: apply velocities, teleport poses, drive joints
(position / velocity and their targets), apply forces and wrenches, or set mass
properties. It is the mirror of the output read and the successor to the
deprecated tensor-binding write. For reading simulation output back (borrowed
warp arrays, ovstage identity, closed-loop write-back), use the
`ovphysx-output-read` skill.

## Workflow

1. Select the language branch and read its bundled reference:

   - Python: [Python session writes](references/python.md)
   - C: [C write-session lifecycle](references/c_api.md)

   Read only the selected language reference. Select the simulated object type,
   the single attribute to write, and the source buffer's device.

   **Complete when:** the language, object type, attribute, and source device
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

4. Open a write session on ONE attribute, fill EVERY entry of every group, then
   commit each group. A session carries exactly one attribute, named when it is
   opened. Group tensors are mutable views onto runtime-owned storage; fill them
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

6. Release resources and validate the selected branch. Python uses the
   `WriteSession` context manager. C releases the write and query handles on
   success and error paths. Exercise an integration test that writes, steps, and
   observes the intended CONSEQUENCE (motion, pose change) rather than reading a
   write-only input back.

   **Complete when:** every session is released before `PhysX.destroy()`, and
   the test observes the expected physical consequence.

## Shared Contracts

- Query by simulated type, not USD schema or prim-path pattern. The write groups
  mirror the read: same prims, same order.
- One attribute per session. Open a separate session for each attribute you
  write.
- The writable set is listed by object type in **Writable attributes by object
  type** below. The writability query (`ovphysx_writability`) gives the
  authoritative classification of every (object type, attribute) pair as
  WRITABLE, WRITE_ONLY, READ_ONLY, or CONDITIONAL with no scene or step -- C
  callers should consult it rather than guessing whether a name is writable.
  It has no *public* Python binding (`hasattr(ovphysx, 'writability')` is
  `False`); `ovphysx._bindings.writability(object_type, attribute)` is a
  private ctypes wrapper around the same call (used by the project's own
  `test_writability.py`), not a supported/stable public API -- prefer the
  table below, or attempt `physx.write(...)`, which raises `RuntimeError`
  immediately for an attribute that is not
  writable for the object type, rather than writing nothing silently.
- Fill EVERY mapped entry before committing. There is no fill mask on the group:
  a committed group publishes every entry, so an unfilled one publishes stale
  memory. Queries cannot select individual bodies or filter out kinematic bodies;
  neither `ALL` nor `ACTIVE` scope provides that filter.
- Uncommitted groups are discarded when the session closes. A failed commit can
  partially apply its group and consumes it; previously committed groups remain
  applied. Read the diagnostic before another C API call, including cleanup;
  Python includes it in the exception.
- `force` is a vec3 applied at the CENTRE OF MASS. `wrench` is nine wide --
  `[fx,fy,fz, tx,ty,tz, px,py,pz]` = force, torque, and a WORLD application
  point -- so use `wrench` (force at the point, torque zero) for a load away
  from the COM, and `wrench` (force zero, torque set) for a pure torque. Both
  are WORLD frame and WRITE-ONLY: the solver consumes and clears them each step,
  so there is nothing to read back -- assert their consequence.
- Angular joint columns are per-AXIS and in DEGREES (coordinates and rates);
  gains are per degree and efforts are unconverted. Do not apply a blanket
  radian conversion. A prismatic axis stays in the stage's base length unit.
- Inspect `tensor.device` per group. A GPU scene may stage a write through the
  host, so the write device can differ from the read; commit a CUDA group with
  its stream (`cuda_stream=` in Python, `ovstage_cuda_sync_t` in C).
- Read simulated results from a STATE attribute, not the control input you
  wrote: a velocity-target write reads back the target, not the physics outcome.

## Writable attributes by object type

Pass the attribute as the write string -- Python `physx.write(type, "name")`, C
the matching `OVPHYSX_ATTR_*` macro. This is the writable set in ovphysx 0.6; a
**write-only** load has no read-back (assert its consequence) and `jointLimit`
is **conditional** -- see the notes under the table.

| `SimObjectType` | Writable attributes |
|---|---|
| `RIGID_BODY` (incl. point-instancer instances) | State: `position`, `orientation`, `linearVelocity`, `angularVelocity`. Mass: `mass`, `inertia`, `centerOfMassPosition`, `centerOfMassOrientation`. Flags: `disableGravity`, `disableSimulation` (standalone bodies only -- not writable per point-instancer instance; see the note below the table). Per-shape: `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`. **Write-only:** `force`, `wrench`. |
| `ARTICULATION` (root) | `rootPosition`, `rootOrientation`, `rootLinearVelocity`, `rootAngularVelocity`. |
| `ARTICULATION_JOINT` (per DOF axis) | State/control: `jointPosition`, `jointVelocity`, `jointPositionTarget`, `jointVelocityTarget`, `jointActuationForce`. Drive props: `jointStiffness`, `jointDamping`, `jointMaxVelocity`, `jointMaxForce`, `jointArmature`, `jointStaticFriction`, `jointDynamicFriction`, `jointViscousFriction`, `jointSpeedEffortGradient`, `jointMaxActuatorVelocity`, `jointVelocityDependentResistance`, `jointDriveType`. **Conditional:** `jointLimit`. |
| `ARTICULATION_LINK` | Mass: `mass`, `inertia`, `centerOfMassPosition`, `centerOfMassOrientation`, `disableGravity`; per-shape `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`. **Write-only:** `force`, `wrench`. Link pose/velocity are READ-ONLY (derived from articulation state). |
| `PARTICLE_SET`, `DEFORMABLE_VOLUME`, `DEFORMABLE_SURFACE` | `points`, `velocities` (sim mesh). |
| `DEFORMABLE_MATERIAL` | `deformableDynamicFriction`, `deformableYoungsModulus`, `deformablePoissonsRatio`, `deformableElasticityDamping`; surface materials also `deformableBendingStiffness`, `deformableThickness`, `deformableBendingDamping`. |
| `VEHICLE_WHEEL` | **Write-only controls:** `driveTorque`, `brakeTorque`, `steerAngle`. Wheel pose is derived (READ-ONLY). |
| `FIXED_TENDON` | `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonLimit`, `tendonRestLength`, `tendonOffset`. |
| `SPATIAL_TENDON` | `tendonStiffness`, `tendonDamping`, `tendonLimitStiffness`, `tendonOffset`. |

- **WRITE-ONLY** (`force`, `wrench`, and the vehicle controls): the solver
  consumes and clears these each step, so there is nothing to read back -- assert
  the physical consequence, not the value.
- **CONDITIONAL** (`jointLimit`): a finite limit interval only lands on an axis
  whose motion is `eLIMITED`; PhysX cannot re-open a free axis in-scene, so a
  finite limit on a free axis is refused. Writing the read's unlimited sentinel
  (+/-`FLT_MAX`) back to a free axis is a legal no-op.
- **STANDALONE-ONLY** (`disableSimulation` on a point-instancer instance):
  `disableSimulation` is not an instancer-writable column and there is no
  per-instance route, so no supported path disables an individual instance -- it
  applies to standalone rigid bodies only.
- `ovphysx_writability(object_type, attribute)` is the runtime source of truth
  this table snapshots (C API only -- see the note above the table). The
  `OVPHYSX_ATTR_*` string constants it and `include/ovphysx/ovphysx_types.h`
  use are a C-only convenience; the wheel ships no `include/` directory and
  has no Python binding for them. A Python caller passes the same attribute
  name as a plain string (`physx.write(SimObjectType.RIGID_BODY, "position")`),
  exactly as shown in the examples in this skill.

## Installed API Sources

For the caller's installed ovphysx version, prefer:

- C SDK headers: `include/ovphysx/ovphysx.h` and
  `include/ovphysx/ovphysx_types.h`
- Python docstrings: `ovphysx.api.PhysX.write`, `WriteSession`, and `WriteGroup`
- The session read/write contract: `docs/ovstage_integration.md`

The public release documentation starts at
<https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>.
