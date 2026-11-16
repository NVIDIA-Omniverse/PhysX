# Official physics population in the shared WASM runtime

This is an executed prerequisite for the real OvPhysX port. It links the unchanged
`ovphysxPopulation.cpp` and its generated native description into the working
OVStage/OpenUSD/USDRT/Fabric WASM64 image. It does not link or replace the physics
simulator.

The generated bridge registers the official PhysX and deformable codeless schema
resources before population, and passes the official physics description alongside
RENDERING. The original schema resources are embedded with their relative layout.
The source checkouts and completed OVStage build are left unchanged.

```sh
python3 ovphysx/experimental/wasm/integration/build-population.py \
  --ovstage-build /tmp/ovstage-population-build \
  --out /tmp/ovphysx-stage-integration

/tmp/ovstage-wasm-emsdk/node/24.19.0_64bit/bin/node \
  ovphysx/experimental/wasm/integration/test-population.mjs \
  /tmp/ovphysx-stage-integration/ovstage-physics-population.mjs \
  /home/rilei/dev/kit_2/rendering/ovstage/experimental/wasm/dist/native/index.mjs \
  /tmp/ovphysx-stage-integration/result.json
```

The recipe reads compile/link metadata from a completed native OVStage Unix
Makefiles build. It checks that memory64/pthreads are selected and rejects changed
bridge anchors. `commands.json` records argv and working directories;
`build-info.json` records source/config/schema/artifact hashes. The existing build
must use the compatible current OVStage population API. This is an experimental
integration driver, not a replacement for the native PhysX build entry point.

The test checks real population output: authored mass and gravity, un-authored
rigid-body/collision/velocity defaults, PhysX damping defaults, applied schema
identity, and units/axis. Initial Node 24 run passed. Full initialization, attach,
stepping and physics read/write are separate acceptance gates.

The generated source also defines internal
`ovphysx_wasm_borrow_stage(uint32_t handle)`. It calls the existing stage handle
validator in the same translation unit and returns a borrowed pointer for runtime
attachment within the shared WASM memory. It is not exported to JavaScript, and does not change
stage lifetime. The final owner must detach physics before destroying its stage.


## Executed shared physics runtime

The population-only command above is a focused diagnostic. The full CPU runtime
uses `--runtime-dir /path/to/ovphysx-pic-build --shared` with a PIC stage build,
producing `ovstage.mjs`, `ovstage.wasm` and `ovphysx.wasm`. The [parent README](../README.md)
gives the actual compile, link and Node smoke/feature commands. Both Node and
Chrome have passed attachment, steps, output publication, floor contact,
articulation, SDF triangle-mesh cooking and fresh-owner reset using these separate libraries.

For browser features, copy `runtime-features-worker.mjs`, `feature-runner.mjs`,
`contact.usda`, `articulation.usda` and `sdf.usda` beside the matching binaries. Serve the
directory with COOP/COEP and the correct JavaScript/WASM MIME types, then run:

```sh
node ovphysx/experimental/wasm/integration/test-runtime-browser.mjs \
  http://127.0.0.1:8771/ /path/to/playwright-core/index.mjs \
  /path/to/shared-runtime/browser.json \
  /runtime-features-worker.mjs ./ovstage.mjs ./ovphysx.wasm
```

The runner checks the selected side download, isolation, UI responsiveness and
actual solver behavior. The host framework and allocator remain owned by the
main module; the side initializes its local Carbonite globals from that host.
Native schema verification uses direct imports in the Emscripten build. There
is no supported dynamic native-plugin discovery or cross-version module mixing.

The local `sdf.usda` fixture exercises actual CPU SDF cooking and settling contact
without network assets. The browser timeout catches the former pool-exhaustion
hang in the SDK thread-start loop; the same scenario runs in Node. Run only that
scenario in Node by replacing `all` with `sdf` in the feature command.
