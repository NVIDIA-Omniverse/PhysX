# AGENTS.md -- ovphysx

This file is for AI agents and automation. It describes how to build, test, and safely use ovphysx.

ovphysx is a self-contained C API and Python library for USD-based physics simulation with DLPack tensor interoperability.
It lives at `ovphysx/` inside the Omniverse Physics monorepo.

Capabilities:
- Load USD scenes and simulate rigid bodies / articulations via PhysX
- Load from local paths, Omniverse Nucleus, S3 (HTTPS), or Azure Blob — see [Remote USD Loading](docs/developer_guide.md#remote-usd-loading)
- Exchange simulation state via CPU / GPU tensors (DLPack interop with NumPy, PyTorch, etc.)
- Clone environments for batched RL workloads (1000s of parallel instances)

Supported platforms: Linux x86_64, Linux aarch64 (arm64), Windows x64. x86_64 builds require AVX at runtime; `ovphysx_initialize()` fails fast when AVX is missing (documented; no non-AVX fallback). GPU features require CUDA and an NVIDIA GPU.
All code should support all platforms, use as few platform-specific sections as possible.

## Build and test

All commands run from `ovphysx/`.

**Build:**
```bash
./build.sh                                        # bootstrap + compile (the newcomer entrypoint)
```

**Full local validation (build + install + wheel + all tests):**

```bash
cmake -P scripts/validate_all.cmake               # single command: build + install + wheel + all tests
cmake -DSKIP_GLIBC_CHECK=ON -P scripts/validate_all.cmake  # local/agent run; skips CI ABI baseline check
```

> **Heads up on the glibc/libstdc++ baseline check.** Install/wheel/validate steps verify that shipped binaries don't pull in symbols newer than the ovphysx baseline. On a host with newer glibc than the baseline (most desktop distros), the check fails fast with a readelf-driven error. For local dev, skip it:
> ```bash
> SKIP_GLIBC_CHECK=ON cmake -P scripts/validate_all.cmake     # env-var form — propagates through validate_all's subprocess chain
> cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake        # -D form — works for direct cmake -P invocations
> ```
> Do **not** ship binaries built with the check skipped — CI runs it unconditionally on the baseline image.

**Agent from-scratch validation checklist:**

- For requests like "checkout, clean, build from scratch, build the wheel, and run all ovphysx tests", use `validate_all.cmake` as the source of truth. It performs the build, SDK install, wheel build, and all non-benchmark CTest suites in one pipeline.
- After `git clean -dxf`, verify `_build/`, `_install/`, `_dist/`, and `_repo/` were actually removed. On Windows, `git clean` can skip nested dependency repos under `_build/`; if the user asked for a real clean build, remove the resolved `_build/` path explicitly before validating.
- Do not replace `validate_all.cmake` with separate install/wheel/test invocations unless you are debugging a failed stage after the aggregate run.
- Make `uv --version` pass in the same shell before starting validation. Installing `uv` is not enough if its Scripts directory is not on `PATH`.
- `validate_all.cmake` runs CTest with `-LE benchmarks`; benchmarks are opt-in and require a benchmark-enabled rebuild.

**Windows agent/default validation recipe:**
```powershell
# Confirm uv is on PATH before launching validation. If missing, install it and
# prepend the user Scripts directory for this shell.
py -3 -m pip install --user uv
$uvScripts = Join-Path (& py -3 -c "import site; print(site.USER_BASE)") "Scripts"
$env:Path = "$uvScripts;C:\Windows\System32;C:\Windows\System32\WindowsPowerShell\v1.0;$env:Path"
uv --version

# Prefer Ninja for full local validation; Visual Studio/MSBuild custom targets can
# lose PATH entries needed by nested cmake scripts (notably powershell.exe and uv.exe).
$env:GENERATOR = "ninja"
cmake -DSKIP_GLIBC_CHECK=ON -P scripts\validate_all.cmake
```

When reporting a validation result, include the branch/commit, exact command and key environment (`GENERATOR`, `PATH` changes, `uv --version`), whether build/install/wheel succeeded, the wheel path, the final CTest pass/fail summary, and every failed CTest bucket with its inner failure line. For example, report a specific GTest case, process exit code, missing-file message, or post-pytest access violation instead of only saying "validate_all failed."

**Fast Python iteration (after one build + install):**
```bash
cd tests/python_tests && uv run pytest            # edit .py, re-run, no rebuild needed
```

**Advanced: native CMake targets** (same install + wheel + tests as `validate_all.cmake`; run `build.sh` or `cmake -P scripts/build.cmake` first):
```bash
cmake --build _build --target validate_all        # install + wheel + all tests
cmake --build _build --target validate_runtime    # install + C++ and Python runtime tests only
cmake --build _build --target validate_wheel      # install + wheel + wheel tests only
```

**Advanced: CMake presets** (after `_build` exists and dependencies are fetched, e.g. via `build.sh`):
```bash
cmake --preset host-release                       # configure _build (preset cache vars)
cmake --build --preset validate-all               # builds validate_all in _build (validate-all preset → host-release binaryDir)
```

**Individual scripts (for debugging or CI):**
```bash
cmake -P scripts/install.cmake                    # assemble _install/
cmake -P scripts/build_wheel.cmake                # build wheel into _dist/
cmake -P scripts/test_cpp.cmake                   # C++ unit tests
cmake -P scripts/test_python_runtime.cmake        # Python runtime tests
cmake -P scripts/test_cpp_samples.cmake           # C++ sample apps
cmake -P scripts/test_python_wheel.cmake          # wheel smoke (python -m ovphysx)
cmake -P scripts/test_python_samples.cmake        # Python sample apps
cmake -P scripts/package_sdk.cmake                # SDK archive into _dist/
```


**Build options:**
```bash
./build.sh --clean                                # clean artifacts only (no build)
./build.sh --rebuild                              # clean then rebuild
./build.sh --debug                                # debug build
./build.sh --release                              # release build (default)
./build.sh --target <name>                        # build specific CMake target
./build.sh --generate                             # configure only (no build)
./build.sh --devphysx                             # build against local PhysX SDK source
./build.sh --devschema                            # use locally-built physics schema
uv run pytest cpu_tests/                          # CPU-mode tests (separate invocation required)
```

**`--devphysx` / `--devschema` cannot be changed incrementally.** The flag
combination selects a build flavor, and incremental builds across a flavor change
are unsupported. `--devphysx` builds PhysX from the local `physx/` source tree and
caches `PHYSX_SDK_DIR` pointing at it; a later configure without `--devphysx`
keeps that cached path, so the `PhysXGpu_64.dll` copy step looks under the source
tree instead of the packaged SDK and fails. Add `--rebuild` whenever the flag
combination differs from the previous build.


**CI vs local:** `validate_all.cmake` reproduces the full build + test pipeline locally.

**VS Code / Cursor test explorer:** run `uv sync` once in `tests/python_tests/` to create the `.venv`, then select the `.venv` interpreter via "Python: Select Interpreter" in the command palette. Tests appear in the Testing sidebar and can be debugged via the launch configurations. The `conftest.py` auto-detects `_install/` and handles all native library preloading.

The build (`build.sh`/`build.bat`) has two phases:
1. **ovruntime** (`ovphysx/ovruntime/`): built first via its own `repo build` system (premake). Produces the PhysX runtime extension plugins. An incremental hash stamp skips this step when ovruntime sources are unchanged.
2. **ovphysx** (`ovphysx/`): built second via CMake. Compiles `libovphysx`, the internal sidecar, and C unit tests. Ovruntime binaries land in `_build/release/`; `--devphysx` builds PhysX in its `checked` config but redirects the output back to the same `release/` directory.

Changes to files under `ovphysx/ovruntime/` therefore require a rebuild from `ovphysx/` to take effect.

Prerequisites: CMake 3.16+ on Linux (4.1+ on Windows), C++17 compiler, [uv](https://docs.astral.sh/uv/) (Astral's Python package manager).
Dependencies auto-download during build via packman.

## Updating the pinned ovstage version

ovphysx pins one exact ovstage release. The version's source of truth is the
`OVSTAGE_VERSION` constant in `scripts/fetch_ovstage_release.py` (a numeric version,
e.g. `0.1.0.344313`).

ovstage ships a single self-contained wheel per platform that carries BOTH the C++
SDK (headers, `libovstage.so`, cmake config, runtime plugins) and the importable
python package. `fetch_ovstage_release.py` downloads that wheel, extracts its
`ovstage/` tree into the build's `OVSTAGE_DIR`, and stages the wheel for the python
tests. It resolves the wheel from the PEP 503 index hardcoded in its `INDEX_URL`.


## Public API

ovphysx is pre-release software. API stability:
- C API (`ovphysx.h` / `ovphysx_types.h`): primary supported ABI surface (still pre-release; compatibility is best-effort until 1.0).
- C++ API (`experimental/ovphysx.hpp`): experimental, may change without notice.
- Python API (`ovphysx.api.PhysX`): pre-release, surface may change between minor versions.

- **C**: `include/ovphysx/ovphysx.h` (C ABI) and `include/ovphysx/ovphysx_types.h`
- **C++** (experimental): `include/ovphysx/experimental/ovphysx.hpp`
- **Python**: `python/ovphysx/api.py` -- the `PhysX` class

Recommended API surface:
- **TensorBindingsAPI**: the primary public data I/O API for RL-style loops and bulk state exchange (bindings + DLPack tensors). Exposed in C and via `ovphysx.api.PhysX` in Python.

Async/execution model:
- Some calls enqueue work and return an `op_index`. Synchronization via `wait_op()`/`wait_all()` is only needed for out-of-stream operations.

## Navigation

- Samples: `tests/python_samples/`, `tests/c_samples/`
- Tutorials: `docs/tutorials/`
- Developer guide: `docs/developer_guide.md`
- Skills (agent playbooks): `SKILLS.md` and `skills/`
- Shipped source samples (SDK/wheel): `samples/` (C samples, Python samples, and sample USD data)
- Benchmarks (opt-in): `tests/benchmarks/` (C++) and `tests/python_benchmarks/` (Python). See [`tests/benchmarks/README.md`](tests/benchmarks/README.md). Off by default; build with `./build.sh --benchmarks` (or `--rebuild --benchmarks`) and run via `cmake -P scripts/test_benchmarks_cpp.cmake` / `..._python.cmake`. `validate_all` excludes them (CTest label `benchmarks`); `validate_benchmarks` runs them.
- Rendered docs: https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html

## MR hygiene

- If a change in `ovphysx/` or `ovphysx/ovruntime` causes user-facing changes for ovphysx, update `docs/changelog.md` in the same MR.
- **On a release branch, the changelog's top section is `VERSION` + 0.0.1.** Releases are cut from `release/ovphysx/*`, where `ovphysx/VERSION` holds the last *released* version -- so the section every in-flight entry goes into is `## [<VERSION + 0.0.1>] - Date TBD`. Read `VERSION`; do not assume the top section is current. Keep exactly one unreleased section: no `[Unreleased]` heading, no second unreleased section, and no release date on a section that has not shipped. The release bump moves `VERSION` and dates that section; the next entry then opens the following one.
- **On `trunk`, `VERSION` matches the top changelog section.** Nothing is released from `trunk`, so `VERSION` there is the version under development rather than the last released one, and it should equal the version of the top changelog section. That is the expected state, not drift, and the `VERSION` + 0.0.1 rule above does not apply.
- **Do not cherry-pick a changelog entry from a release branch to `trunk`.** Each branch has its own `VERSION`, so an entry written on `release/ovphysx/*` names a version that does not exist on `trunk`. Carrying it over files one fix under two different versions and drags the branches' section structures out of sync. When cherry-picking a release-branch commit to `trunk`, drop the `docs/changelog.md` hunk and take the rest. If the change warrants a trunk changelog entry in its own right, write a fresh one into trunk's own top section rather than copying the release-branch text.

## Common footguns

- **`op_index` is single-use.** A wait consumes every completed or failed index it reaches, including unconsumed indices below the requested index; a timeout leaves the still-pending index unconsumed. Reusing a consumed index raises `NOT_FOUND` (`RuntimeError` in Python). Polling is supported via `timeout_ns = 0`. See `docs/developer_guide.md`.
- **Settings and hard CPU-only mode are per-process.** Carbonite settings are global; different ovphysx instances in the same process share the same settings. Normal CPU/GPU dynamics are authored per scene in USD. If `ovphysx_set_cpu_mode(true)` is enabled, the override is process-wide and cannot be reverted in that process.
- **Namespaced USD runtime.** ovphysx uses the OV namespaced monolithic USD runtime. If another OV library loads a compatible namespaced USD first, ovphysx should detect and reuse it; classic host USD is kept separate and is not validated as the ovphysx runtime. See `docs/developer_guide.md`.
- **Thread safety.** A single instance is NOT thread-safe -- serialize access externally. Multiple instances also share one underlying runtime singleton and one attached stage, so cross-instance simulation calls must also be serialized. Do not wait on the same `op_index` from multiple threads.
- **Single attached stage per process.** All ovphysx instances share one runtime singleton; only one USD stage can be attached at a time process-wide. Multiple instance handles give per-handle bookkeeping (error queue, tensor bindings, lifetime), not independent simulations. Multiple `UsdPhysicsScene` prims in a single stage become separate PhysX scenes but step together via `ovphysx_step` (per-scene stepping is not yet surfaced). For isolated stages or parallel sims, use separate subprocesses.
- **Error string ownership.** C API: on failure, call `ovphysx_get_last_error()` on the same thread to retrieve the error string. For `ovphysx_wait_op()` failures, call `ovphysx_get_last_op_error()` per failed op index, then `ovphysx_destroy_wait_result()`. Python: errors are raised as exceptions (automatic cleanup).
- **Error/lifecycle checks should be C-first.** When feasible, enforce lifecycle safety and argument validation in the C API so all frontends get the same guarantees. Keep Python thin and limited to minimal exception translation or ergonomics.

## Code style

- C API uses `ovphysx_` prefix; all public symbols are in `ovphysx.h`.
- Python API is the `PhysX` class in `ovphysx.api`.
- Source files (`.cpp`, `.h`, `.py`, `.cmake`, `.sh`, `.bat`, etc.) must be
  ASCII-only in code and comments. Use `--` or `-`, not em dashes or other
  Unicode punctuation. Markdown docs under `docs/` are exempt from this rule.

## Doc authoring conventions

Docs under `docs/` are authored in Markdown and serve two audiences: Sphinx (for the HTML build) and plain Markdown viewers (GitHub, SDK, wheel, LLMs). Follow these rules to keep them readable everywhere:

- Use **standard Markdown** for links, headings, blockquotes, and code blocks. Do not use MyST-only directives like `{doc}`, `{tab-set}`, `{note}`, or colon-fence syntax.
- **`{literalinclude}` is the one allowed MyST directive** for including source code from test files. The `scripts/preprocess_markdown.py` script resolves it at SDK/wheel packaging time. Supported options: `:language:`, `:lines:`, `:start-after:`, and `:end-before:`. Prefer `:start-after:` / `:end-before:` with `[tutorial-start]` / `[tutorial-end]` markers in source files over `:lines:` ranges, which silently drift when code is edited.
- Doc-to-doc cross-references should use standard Markdown links: `[Link Text](relative/path.md)`.
- New docs must be added to the hidden `{toctree}` at the bottom of `docs/index.md`.
- `docs/api.md` is the only file that should contain `{eval-rst}` (for Doxygen rendering). The preprocessor strips these blocks from shipped copies.
