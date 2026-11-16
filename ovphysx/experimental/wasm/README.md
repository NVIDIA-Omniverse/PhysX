<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Experimental OvPhysX WASM64 CPU port

This target builds the original OvPhysX parser, runtime, sidecar and C API with
the PhysX CPU solver. OVStage owns USD ingestion and the application-owned stage.
It uses no custom descriptor-to-PhysX bridge and links no second OpenUSD image.
This is a feasibility experiment, not a supported platform or published package.

The executed profile uses Emscripten 5.0.7, memory64, pthreads, JavaScript C++
exceptions and RTTI. PhysX uses scalar CPU paths. Node 24.19.0 and Chrome 157
have executed real creation, attachment, stepping, reads, sealed OVStage pose
publication, detach, destruction and fresh-owner reset. The feature suite checks
floor contact, SDF triangle-mesh collision cooking and a two-link articulation. Those fixtures do not establish full
native feature parity.

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
`93ee11e8352384368e37de21f0415b3d0f60a3a4`. Then run from the PhysX repository root:

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

The compact executed output is `ovstage.mjs` (5,465,710 bytes), `ovstage.wasm`
(20,290,201 bytes) and `ovphysx.wasm` (13,394,574 bytes). These are artifact sizes,
not heap measurements. Browser tests use a COOP/COEP static server, module workers
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

## Host and capability limits

The target supplies implemented logging, settings/dictionary, event and task
storage services. Static adapters preserve original runtime startup and sidecar
calls while replacing native filesystem/plugin-directory loading. See
[host/README.md](host/README.md) for focused service behavior tests.

The selected profile requires `CPU_ONLY=true`, `ASYNC_EXECUTION=false`, zero
solver scheduler workers and disabled UJITSO/compiled caches. The build generates
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
