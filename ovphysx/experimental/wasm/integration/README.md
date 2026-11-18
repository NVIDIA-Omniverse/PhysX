# Shared OVStage/OvPhysX WASM runtime

The current runtime builds the native PhysX CPU implementation as an optional
side module sharing OVStage's memory. Use the [parent build recipe](../README.md#build-and-test)
to compile the SDK/runtime objects, then link them with a matching PIC stage build
using `build-population.py --runtime-dir /path/to/ovphysx-pic-build --shared`.
The output is `ovstage.mjs`, `ovstage.wasm` and `ovphysx.wasm`; ABI validation adds
`compiled-artifacts.mjs`. Keep the complete matched artifact set together.

The link driver reads compile/link metadata from a completed native OVStage Unix
Makefiles build and checks memory64, pthreads and compatible dependency ABIs.
It registers official PhysX/deformable codeless schemas before population and
uses the native physics description alongside RENDERING. It leaves dependency
source checkouts and the completed stage build unchanged. `commands.json` records
argv and working directories; `build-info.json` records inputs and artifact hashes.
This experimental driver does not replace the native PhysX build entry point.

Node and browser tests exercise CPU attachment, stepping, output publication,
floor contact, articulation, SDF cooking and fresh-owner reset. These fixtures
do not establish full native feature parity.

## Browser runtime validation

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

## Population-only diagnostic

Omit `--runtime-dir` to link only the official physics population description and
codeless schemas. This mode isolates authored values, schema defaults and stage
metadata from solver attachment; retain it when investigating population failures.
It produces no physics simulator.

```sh
python3 ovphysx/experimental/wasm/integration/build-population.py \
  --ovstage-build /path/to/ovstage-build --out /path/to/population-diagnostic
node ovphysx/experimental/wasm/integration/test-population.mjs \
  /path/to/population-diagnostic/ovstage-physics-population.mjs \
  /path/to/kit/rendering/ovstage/experimental/wasm/dist/native/index.mjs \
  /path/to/population-diagnostic/result.json
```

The test checks authored mass/gravity, unauthored rigid-body/collision/velocity
and damping defaults, applied schema identities, units and up-axis. The internal
`ovphysx_wasm_borrow_stage(uint32_t handle)` validates the stage handle and returns
a borrowed pointer for runtime attachment in the shared memory. It is not
exported to JavaScript and does not change stage lifetime; detach physics before
destroying the stage.

## Output publication contract

Run the focused publication/lifetime regression against a matching shared runtime:

```sh
/tmp/ovstage-wasm-emsdk/node/24.19.0_64bit/bin/node \
  ovphysx/experimental/wasm/integration/test-runtime-output.mjs \
  /path/to/shared-runtime/ovstage.mjs \
  /path/to/shared-runtime/output-result.json \
  /path/to/shared-runtime/ovphysx.wasm
```

The local fixture checks repeated publication against current physics positions,
mirrored and nested scale, descendant hierarchy and parent-relative local
transforms. It also checks publication without advancing physics, rejected
ordinals and stage identities, failed attachment cleanup, physics reattachment,
and stage destruction/recreation in the same runtime. The output cache belongs
to one physics attachment and is released before detach; it only reuses native
buffer allocation and interned tokens, while every publication reads fresh poses
and runs the native hierarchy update.

## Performance build options

The browser performance experiment keeps the native API and fresh worker owner
boundary. Its measured candidates and limitations are recorded in the standalone
[historical performance investigation](https://github.com/NVIDIA-dev/rilei_isaacsim_next_web_standalone/blob/2e5800c2951eae7e5597e5e7fe853ed880b6f356/docs/official-performance-investigation.md).
See the [current implementation review](https://github.com/NVIDIA-dev/rilei_isaacsim_next_web_standalone/blob/main/docs/web-implementation-review.md)
for subsequent findings and qualification limits. Build flags remain opt-in;
these are not a new native OVPhysX build policy.

Rebuild OpenUSD, TBB, Abseil and every stage/runtime consumer with the same
exception ABI. `population/build.py --exceptions wasm` selects standardized
native WebAssembly exceptions; `wasm-legacy` selects the older native encoding.
The default remains JavaScript exceptions. The Kit recipe records the dependency
provenance and rejects mixed exception or Sdf handle layouts. A native exception
link flag alone is insufficient when any input was compiled for JavaScript
exceptions.

For the vector-math experiment, build both SDK and all runtime components with
`ovphysx/experimental/wasm/build.py --sdk-math sse2 --pic`. Use `--sdk` for the
SDK and `--runtime-objects` with the complete component list from the parent
README for the runtime. This lowers the existing SSE2 implementation to WASM
SIMD; it does not enable fast-math or change the configured solver policy.
The compiler's metadata typedef compatibility overlay is pinned and applied
without editing the SDK source checkout. Keep `--runtime-opt 1`; the matched O3
experiment did not establish a pyramid stepping benefit.

The final shared link can select these independently:

```sh
python3 ovphysx/experimental/wasm/integration/build-population.py \
  --ovstage-build /path/to/matched-stage-build \
  --runtime-dir /path/to/matched-sse2-runtime \
  --shared --pthread-pool-size 16 --stage-only-pthread-pool-size 8 \
  --allocator mimalloc --side-private-functions --prune-side-stubs \
  --validate-artifact-abi \
  --out /path/to/fresh-shared-output
```

`--side-private-functions` derives a conservative symbol policy from the actual
main/side link commands. It protects every main export and definition in explicit
main inputs, all data/globals and the public physics exports. It changes only
function visibility metadata in copied side objects/archives, then relinks only
the side. Main bytes, original inputs, shared imports and exported identities
must pass the audit before installation. `private-functions/` retains the policy,
changed-byte receipts and previous side image. Use a fresh output directory; a
prior proof is never silently overwritten. This policy is qualified for one
main module and its optional physics side, not arbitrary later `dlopen` modules.
The tool queries Emscripten's configured Node and verifies actual memory64/table64
compilation support; it does not silently select the system Node.

`--prune-side-stubs` is an additional opt-in that requires
`--side-private-functions`. After the native side audit, it removes only the
obsolete generated JavaScript function stubs and their import-table entries.
The actual main and optimized side must no longer import or export those names;
public/shared symbols and all remaining source references are protected. Unknown
generated syntax, changed inputs and extra side modules reject the build. This
phase leaves both WASM images byte-identical and performs no second main link.
`loader-pruning/` retains the unpruned loader, derived source and audit. The
separate `loaderPruning` receipt describes this phase; the preceding native
visibility receipt's unchanged-main claim applies before loader pruning.
ABI wrapping and final artifact hashes are produced after both phases.

`--validate-artifact-abi` adds a bounded ABI custom section to both final WASM
files and a JavaScript wrapper that compiles and validates the requested artifacts
before creating native memory, workers or providers. It checks the exact build
pair, exception encoding and USD header/handle ABI. It then creates fresh native
instances and passes the compiled code to the existing pthread bootstrap. No
native owner or compiled-code cache survives stage disposal. The distribution
adds `compiled-artifacts.mjs` beside `ovstage.mjs`; the optional physics binary
stays in its own package and its URL is supplied by `physicsPlugin()`. Compatibility
metadata is not a cryptographic signature. The build receipt contains the raw
artifact identities, emitted memory/table types and final four-file closure.

`--stage-only-pthread-pool-size 8 --pthread-pool-size 16` independently selects
eight startup workers when no side libraries are requested and sixteen when the
physics side is declared. It requires shared mode, a stage-only value of at least
eight and no more than the maximum. The supported Emscripten pool expression
reads the declared `dynamicLibraries` list at initialization; it does not infer
solver settings, resize a live pool or bypass idle-capacity checks. Without this
option the existing fixed-pool behavior remains. The receipt records both counts
and the exact selection expression in `pthreadPoolPolicy`.

The public physics option is `createPhysics(stage, { solverWorkers: 0 | 2 | 4 })`.
It defaults to zero additional workers. More workers helped the active pyramid
but added overhead to a light grid; they are not universally faster. The owner
preflights idle pool capacity before attachment. See [solver threads](SOLVER_THREADS.md)
for lifetime, browser and feature checks. Use the original eight-bit Sdf region
layout; smaller regions did not establish a useful Chrome resident-memory gain
and regressed the measured pyramid tick. Publication selector caching likewise
showed no gain and is not part of the build. The performance investigation retains
the comparison results.
