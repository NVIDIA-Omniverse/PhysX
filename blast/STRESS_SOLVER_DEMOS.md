# Blast Stress Solver Demo Quickstart

This note captures the exact steps needed to build and run the current Rust and Node.js stress solver demos in this workspace. It reflects the additional defines required when compiling on non-x86 Linux (SIMD disabled) and the toolchain prerequisites observed during verification.

## Prerequisites

- C++17 toolchain (the dev container already provides `c++`).
- Rust toolchain (`cargo` 1.90+).
- Emscripten `emcc` (validated with version 4.0.15).
- Node.js 18+ and `npm` (not present in the stock container; install via your preferred method, e.g. `apt`, `nvm`, or NodeSource).

## Rust demo

The Rust build requires forcing the scalar stress solver backend and skipping SIMD feature detection. These defines are already baked into `blast/rust_stress_example/build.rs`:

```text
STRESS_SOLVER_FORCE_SCALAR
STRESS_SOLVER_NO_SIMD
STRESS_SOLVER_NO_DEVICE_QUERY
```

Run the demo:

```bash
cargo run --release --manifest-path blast/rust_stress_example/Cargo.toml
```

Expected output is the three-bond truss simulation showing frame-by-frame stresses and bond removals.

## Node.js / WASM demo

This flow compiles the same solver bridge through Emscripten and runs the analytics in JavaScript.

1. Ensure Node.js and `npm` are installed (`node --version`, `npm --version`).
2. From the repository root:

```bash
cd blast/js_stress_example
npm run demo
```

`npm run demo` invokes the build script, compiling `stress_bridge.cpp` and `stress.cpp` with the same scalar flags, emitting `dist/stress_solver.{cjs,wasm}`, and then executes `demo.js` to log the frame analysis. On systems without Node.js, install it before running the command.

## Troubleshooting

- **`xmmintrin.h` or `cpuid.h` missing**: occurs on non-x86 targets. The scalar build flags above prevent those includes.
- **`npm: command not found`**: install Node.js/`npm`; Emscripten alone is not sufficient.
- **WASM build errors about SIMD**: confirm the `-DSTRESS_SOLVER_FORCE_SCALAR=1` and `-DSTRESS_SOLVER_NO_SIMD=1` flags are present (they are baked into `scripts/build.js`).


