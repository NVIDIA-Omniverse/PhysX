<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Experimental OvPhysX WASM64 CPU port

This target builds the original OvPhysX parser, runtime, sidecar and C API with
the PhysX CPU solver. OVStage owns USD ingestion and the application-owned stage.
It uses no custom descriptor-to-PhysX bridge and links no second OpenUSD image.
This is a feasibility experiment, not a supported platform or published package.

The baseline recipe uses Emscripten 5.0.7, memory64, pthreads, JavaScript C++
exceptions, RTTI and scalar PhysX math. The [performance configuration](integration/README.md#performance-build-options)
adds coherent standardized WASM exceptions, SSE2 math lowered to WASM SIMD,
mimalloc, optional solver workers, and a smaller generated loader. These remain
explicit build options. Node 24.19.0, Chrome 157 and Firefox 159 have exercised
real creation, attachment, stepping, reads, sealed OVStage pose publication,
detach, destruction and fresh-owner reset. The feature suite checks floor
contact, SDF triangle-mesh collision cooking and a two-link articulation.
Those fixtures do not establish full native feature parity.

## Build and test

The source baseline is `ovphysx-0.8.0`, commit
`7d6f6ac8b83e1610bdf715f1f2f1d6b718aee586`. Use emsdk revision
`708dcbcc9ee23bc7d2dad44fc90db377e38b771c` with Emscripten 5.0.7. The driver reads
the actual source commit and requires that release baseline in its ancestry; the
toolchain revision remains exact. It reads include/compiler settings from a completed native OVStage WASM Makefiles
build, then records source, object and archive hashes. Required external inputs
include matching OVStage/USDRT/Fabric/Carbonite headers and a pinned WASM64
OpenUSD/oneTBB installation. A public PhysX clone alone cannot rebuild that host.

Prepare PIC Abseil and OVStage with the helpers in
[Kit PR #6034](https://github.com/NVIDIA-Omniverse/kit/pull/6034), commit
`320688609f7dadf2119b9a833cc6fd995bb6ece7`. Then run from the PhysX repository root:

```sh
python3 ovphysx/experimental/wasm/build.py \
  --emsdk /path/to/emsdk --stage-build /path/to/ovstage-pic-build \
  --out /path/to/ovphysx-pic-build --sdk --runtime-objects --pic --jobs 4 \
  --components common,parse,ovstage,foundation,cuda_shim,cooking,convex,runtime,api,sidecar,bootstrap,closure,platform
python3 ovphysx/experimental/wasm/integration/build-population.py \
  --ovstage-build /path/to/ovstage-pic-build \
  --runtime-dir /path/to/ovphysx-pic-build --out /path/to/shared-runtime --shared
node ovphysx/experimental/wasm/integration/test-runtime.mjs \
  /path/to/shared-runtime/ovstage.mjs /path/to/shared-runtime/smoke.json \
  /path/to/shared-runtime/ovphysx.wasm
node ovphysx/experimental/wasm/integration/test-runtime-features.mjs \
  /path/to/shared-runtime/ovstage.mjs /path/to/shared-runtime/features.json \
  all /path/to/shared-runtime/ovphysx.wasm
```

Use Node 24 or newer. `--diagnostic-names` retains native stack names at extra
binary cost. The ordinary native `ovphysx/build.sh` workflow is unchanged; this
separate experimental target does not establish Linux/Windows/ARM regression
coverage for the guarded source changes.

The shared output contains `ovstage.mjs`, `ovstage.wasm` and `ovphysx.wasm`.
`--validate-artifact-abi` adds the matching `compiled-artifacts.mjs` helper;
distribute that complete four-file set together. Build receipts record the exact
sizes and hashes. Browser tests use a COOP/COEP static server, module workers
and a browser supporting memory64/SharedArrayBuffer. See the [integration
recipe](integration/README.md) and the [standalone demo and executed
receipts](https://github.com/NVIDIA-dev/rilei_isaacsim_next_web_standalone).

## Shared-library ownership

The OVStage main module owns the memory, allocator, function table and host.
The OvPhysX side module borrows the validated native stage pointer through the
original attach API. This is one shared memory across two libraries. Raw pointers
cannot cross independent memories. Dispose physics before its stage, then
terminate the owner worker; reset currently creates a fresh owner.

`AUTOLOAD_DYLIBS=0` lets the caller select `dynamicLibraries` at initialization.
Stage-only loading populates real USD without downloading physics. The modules
must come from a matched build set. Late hot loading, unloading, arbitrary
version mixing and a standalone physics module without OVStage are unsupported.
The JavaScript npm package ownership is maintained in the standalone repository.

Important integration details:

- Stage/Fabric, Abseil and physics inputs require position-independent code.
- Carbonite globals are deliberately local to each module. Bootstrap borrows the
  existing host and initializes its local framework and runtime callbacks.
- The main explicitly links the matching PIC C++ support archive with section
  garbage collection, so optional side definitions cannot become stage-only
  initialization dependencies.
- Known schema APIs are direct WASM imports. Their original registration checks
  and probes execute; native OS-library lookup is unchanged. General synchronous
  `dlopen` with live pthreads has an unqualified memory64 conversion path in the
  pinned toolchain.
- Official physics and rendering population descriptions and codeless schemas
  supply defaults, geometry, units and axes before attachment. Publication uses
  genuine CPU output and a separate sealed ordinal; it does not reset simulation.

## Named CPU controls

The shared build exposes four optional JavaScript control methods through
`@omniverse/ovphysx`. The package and its matching native artifacts are maintained
in the [standalone repository](https://github.com/NVIDIA-dev/rilei_isaacsim_next_web_standalone/tree/main/packages/ovphysx).
These methods borrow the existing attachment; they do not create another solver,
advance simulation time, or modify the authored USD layer.

| JavaScript method | Input/output contract | Native implementation |
| --- | --- | --- |
| `setJointVelocityTargets(targets)` | Named revolute-joint targets in radians/second. The joint must already have an angular drive with positive damping and force limit. Existing gains, stiffness and position targets are retained. | Transient OVStage drive-column writes, followed by the original OVPhysX property-update path. |
| `readJointStates(paths)` | Owned ordered results, including duplicate paths: position in radians and velocity in radians/second, with authored body1-relative-to-body0 sign. | Public articulation-joint state columns, or standalone-joint state derived from actual actor poses/angular velocities and the original joint frames. |
| `setKinematicTargets(targets)` | World positions in metres and normalized quaternions in `(x, y, z, w)` order. Targets integrate on the next explicit `physics.step(dt)`. | Public OVPhysX kinematic write sessions. The adapter converts metres to stage units. |
| `readBodyStates(paths)` | Named dynamic, kinematic, static or articulation-link states. World body-origin position is in metres; linear velocity is the centre-of-mass world velocity in metres/second; angular velocity is in radians/second. | Public body/link state columns. Static actors use their fixed configured world pose after native actor membership is verified; they have zero solver velocity. |

For standalone revolute joints, the adapter measures relative orientation and
angular velocity; it never reports the authored target as measured state. It
matches the native twist branch, including the `(-2π, 2π]` wrapping convention.
Joint endpoint frames are captured at attachment before simulation publication,
including supported relationships to transformed descendants of bodies. Public
articulation columns already encode authored endpoint sign and angular values
in degrees; the adapter converts units once. This is a CPU rigid-body/revolute
profile, not a general joint, deformable, vehicle or DirectGPU control interface.
Unsupported joint metadata remains a named-control rejection rather than making
an otherwise usable physics scene fail to attach.

Breakable joints are not qualified in the current Official WASM host. A real
joint-break event reaches the unimplemented host dictionary `setIntArray` path
and can abort the owner; dispose it and create a fresh owner. This is an existing
platform boundary, not parity with Nano's broken-joint rejection. Shared
relationship-frame tests use uniformly scaled body descendants; Nano rejects
indirect joint targets with nonuniform scale. These limits must remain explicit
when selecting assets or comparing the two implementations.

### Ordinals and ownership

A joint-target batch follows this sequence in the owner worker:

1. Validate every target and prepare the stage writes before changing a drive.
2. Write `drive:angular:physics:targetVelocity` at a new control ordinal, converting
   radians/second to the native USD degrees/second convention.
3. Seal that ordinal through the stage facade, then call
   `ovphysx_update_from_ovstage` for exactly that control ordinal.
4. Let the application call `physics.step(dt)` with `dt` in seconds and later
   `physics.publish()` at a separate higher output ordinal.

Output ordinals are excluded from subsequent control updates, so physics does
not consume its own published poses. Runtime control values may be visible in
OVStage reads; they are not authored USD edits and are absent from USD export
and authoring checkpoints. Disposal detaches physics and releases control
metadata; it does not restore runtime poses or drive values. Use a fresh stage
owner to restore the authored scene, as the standalone Stop/reset workflow does.
The initial native attachment ordinal must be positive.

The private C adapter is `integration/controls.inl`. Its packed-path calls back
four public operations: `ovp_wasm_set_joint_velocity_targets`,
`ovp_wasm_read_joint_states`, `ovp_wasm_set_kinematic_targets` and
`ovp_wasm_read_body_states`. Joint writes additionally use
`ovp_wasm_apply_controls` after the facade seal. The
`ovp_wasm_control_error_is_partial` flag distinguishes a rejected preflight from
an admitted operation or cleanup failure; the JavaScript owner retains its
failed state after the latter and requires disposal. No rollback is promised.
The C read-result buffer is borrowed until the next adapter operation; JavaScript
copies and validates it before returning an owned result. Public query-group
prim identities establish ownership; no SDK pointer value is dereferenced.

### Reproduce the package checks

Build a complete matched shared runtime with the recipes above, then follow the
standalone [distribution recipe](https://github.com/NVIDIA-dev/rilei_isaacsim_next_web_standalone/blob/main/docs/DISTRIBUTION.md)
to prepare the retained-ordinal profile and vendor that matched artifact pair.
From the standalone checkout, use Node 24 or newer:

```sh
npm ci
node --test tests/physics-controls-conformance-real.mjs
node --test tests/robot-arm-demo-real.mjs
```

Both commands exercise the same public scene/controller contract through the
Official and Nano factory pairs. The first checks named state, drive direction
and units, batch rejection, kinematic contact, endpoint frames, output ownership
and fresh-owner reset. The second checks physical pick-and-place replays and a
weak-grip negative control. These are correctness checks on the qualified profile;
they do not establish full native feature parity or a performance comparison.

## Host and capability limits

The target supplies implemented logging, settings/dictionary, event and task
storage services. Static adapters preserve original runtime startup and sidecar
calls while replacing native filesystem/plugin-directory loading. See
[host/README.md](host/README.md) for focused service behavior tests.

The profile requires `CPU_ONLY=true`, `ASYNC_EXECUTION=false` and disabled
UJITSO/compiled caches. Solver workers default to zero; the qualified runtime can
select two or four additional CPU workers with explicit pool-capacity checks
([configuration and lifecycle](integration/SOLVER_THREADS.md)). The build generates
`generated/cooking/TriangleMeshCookingTask.cpp` with one SDF construction thread.
The original SDK then computes the same SDF inline. Its native default of 16
threads can exhaust the preallocated browser pthread pool and block attachment
while waiting for a thread that cannot start. Serial SDF cooking avoids that
hang but increases initial loading time for detailed collision meshes; it does
not replace them with simpler shapes. Original runtime sources remain unchanged,
and the object receipt records both original and generated source hashes.
CUDA declarations remain consistent across SDK/runtime compilation, but driver loading and device
operations are unavailable. Generated GPU entry points throw unsupported errors;
CUDA adapters reject operations and preserve failure semantics. They do not
simulate successful GPU work. The focused boundary test is:

```sh
python3 ovphysx/experimental/wasm/platform/test.py --out /path/to/ovphysx-pic-build \
  --emsdk /path/to/emsdk --stage-build /path/to/ovstage-pic-build --node /path/to/node
```

GPU physics, native asynchronous execution, raw TCP recording, broad shape/joint
coverage and portable solver checkpoints remain unqualified. Actual CPU
collision cooking and solving execute. Browser and native workflow parity need
separate conformance tests.

## Distribution

Keep original SDK/OvPhysX notices and the licenses for external dependencies.
The generators read local external headers rather than vendoring proprietary
SDK payloads here. This directory commits source and recipes, not build caches
or compiled binaries. The standalone demo vendors a matched build separately.
No npm or wheel publication is performed by these scripts.
