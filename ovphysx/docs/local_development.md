# Local Development

This guide covers the two ways to build an application against ovphysx, plus
the Python wheel workflow for rapid iteration.

## Building Against the Installed SDK

The simplest approach: build ovphysx once, install it, and point your app at the
install tree with `find_package()`.

```bash
# 1. Build ovphysx (fetches dependencies automatically)
cd ovphysx
./build.sh            # Linux
build.bat             # Windows

# 2. Install the SDK into _install/
cmake --build _build --target install_sdk

# 3. Build your app against the installed SDK
cmake -S my_app -B my_app/_build \
    -DCMAKE_PREFIX_PATH="$(pwd)/_install;$(pwd)/ovruntime/_build/target-deps/ovstage/ovstage"
cmake --build my_app/_build
```

The source build fetches the pinned OVStage wheel automatically. The second
prefix above points CMake at the native package extracted from that wheel.

Your app's `CMakeLists.txt` uses `find_package()`:

```cmake
find_package(ovphysx REQUIRED)
target_link_libraries(my_app PRIVATE ovphysx::ovphysx ovphysx::ovstage)
```

Refer to `tests/c_samples/hello_world_c/` for a complete example.

## Building Against the Source Checkout

If you need to iterate on ovphysx code and have your app automatically
recompile changed sources, use `add_subdirectory()` instead.

```bash
# No prior build step needed — dependencies are fetched at configure time.
cmake -S my_app -B my_app/_build
cmake --build my_app/_build
```

Your app's `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Point at the ovphysx source tree
set(OVPHYSX_SOURCE_DIR "/path/to/ovphysx")
add_subdirectory("${OVPHYSX_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/ovphysx")

add_executable(my_app main.c)
target_link_libraries(my_app PRIVATE ovphysx::ovphysx)

# Set up RPATH / DLL copying so the executable finds libraries at runtime
ovphysx_setup_source_link_runtime(my_app)
```

The first `cmake` configure will automatically fetch packman dependencies
(controlled by `OVPHYSX_FETCH_DEPS`, default `ON`). Subsequent configures
skip the fetch if the dependencies already exist.

Edit-rebuild cycle:

```bash
# Edit ovphysx source.
vim ovphysx/src/ovphysx/ovphysx.cpp

# Rebuild — only changed ovphysx sources recompile
cmake --build my_app/_build
```

`ovphysx_setup_source_link_runtime()` configures RPATH (Linux) or copies DLLs
(Windows) so that the executable can load `libovphysx.so` and its direct
dependencies.

**Running the executable:** The ovphysx runtime also needs Carbonite and PhysX
plugins at runtime (loaded through `dlopen`). The simplest approach is to build the
installed SDK once and point `OVPHYSX_LIB` at the installed ovphysx library so
the runtime can derive the matching config, schema, and plugin paths:

```bash
# One-time setup: build and install the SDK
cd ovphysx && ./build.sh && cmake --build _build --target install_sdk

# Run with runtime layout from the installed SDK
OVPHYSX_LIB=ovphysx/_install/lib/libovphysx.so ./my_app/_build/my_app
```

On Windows, use `OVPHYSX_LIB=ovphysx\_install\bin\ovphysx.dll`.

Refer to `tests/c_samples/hello_world_source_link/` for a complete example.

## Python Wheel Workflow

For Python users, the ovphysx wheel contains the physics runtime plus the
OVStage-provided OmniClient and connection libraries needed for PhysX-first
startup. It declares an exact `ovstage` wheel dependency, which supplies the
matched OVStage runtime, USD resolver and registry, and namespaced USD runtime
without packaging duplicate resolver/USD singletons in the ovphysx wheel.

```bash
cd ovphysx

# Build everything and create the wheel in _dist/
./build.sh
cmake --build _build --target build_wheel

# Create and activate a virtual environment, then install the wheel
uv venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
uv pip install _dist/ovphysx-*.whl
```

For fast iteration without rebuilding the wheel, point `OVPHYSX_LIB` at the
build tree:

```bash
export OVPHYSX_LIB="$(pwd)/_build/linux-x86_64/release/libovphysx.so"
python my_script.py
```

## Configuration Reference

### CMake Cache Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `OVPHYSX_FETCH_DEPS` | `ON` | Automatically fetch packman dependencies at configure time |
| `LOCAL_TARGET_DEPS` | `<ovphysx>/_build/target-deps` | Override the directory where packman dependencies are stored |
| `OVPHYSX_DEV_PHYSX` | `OFF` | Build PhysX SDK from source instead of using prebuilt binaries |
| `OVPHYSX_DEV_SCHEMA` | `OFF` | Use locally-built physics schema |
| `OVPHYSX_WARNINGS_AS_ERRORS` | `ON` in CI, `OFF` otherwise | Treat compiler warnings as errors |
| `OVPHYSX_BIN_DIR` | *(empty)* | Advanced build-time helper path for Kit/PhysX plugin linkage; not a runtime environment variable |

### Environment Variables

| Variable | Description |
|----------|-------------|
| `OVPHYSX_LIB` | Path to `libovphysx.so` / `ovphysx.dll`. Python loads this library directly; native/source-link runs use its directory to find `config.toml`, plugins, and USD schema paths. |

## Troubleshooting

Common from-source build and first-run failures:

- **Startup aborts with `This CPU does not support AVX...`** — x86_64 binaries
  require AVX and have no fallback, so `ovphysx_initialize()` / `PhysX()` fails
  fast. Check with `grep -qw avx /proc/cpuinfo`; use an AVX-capable x86_64 CPU, or
  the aarch64 wheel on ARM Linux.
- **Configure or compile fails on C++17 or unknown-flag errors** — the toolchain
  is below the compiler and CMake versions required by ovphysx and the PhysX SDK.
  Match those requirements; on Windows, `set GENERATOR=ninja` to use Ninja instead
  of the default Visual Studio generator.
- **packman download or connection error on the first configure** — a dependency
  fetch failed. Retry (transient failures are common); the cache lives under
  `_build/target-deps/` (see `OVPHYSX_FETCH_DEPS` and `LOCAL_TARGET_DEPS` in the
  [Configuration Reference](#cmake-cache-variables)).
- **Source-path mismatch or stale cache after moving the checkout** — delete
  `_build/` and reconfigure, or `build.sh --rebuild`. Use `--clean` to clean only
  and `--generate` to configure without compiling.
- **`Error copying file ... PhysXGpu_64.dll ... No such file or directory` after
  changing `--devphysx` or `--devschema`** — incremental builds across a flavor
  change are not supported. `--devphysx` caches `PHYSX_SDK_DIR` pointing at the
  local `physx/` source tree; a later configure without it keeps that cached
  path, so the copy step looks for `PhysXGpu_64.dll` under the source tree
  instead of the packaged SDK. Rebuild cleanly with `--rebuild` (`build.sh
  --rebuild <flags>` or `build.bat --rebuild <flags>`).
- **Debug configure fails with `OVPHYSX_USE_RELEASE_RUNTIME_DEPS=OFF`** — Debug
  builds require the Release runtime dependencies (published OVStage ships Release
  only); leave the flag at its default `ON`.
- **Install/wheel/validate fails with a readelf / glibc baseline error on a newer
  distro** — Linux SDK and wheel builds target a glibc 2.35 (`manylinux_2_35`) ABI
  baseline. On a host with newer glibc the check fails fast; pass
  `SKIP_GLIBC_CHECK=ON` for local development, e.g.
  `cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake` (also accepted as an
  environment variable through `validate_all`).
- **Install or wheel build fails with `'file' not found` on Linux** — both steps
  shell out to the OS `file` utility (libmagic) to find unstripped ELF binaries,
  and to `strip`/`readelf` from `binutils`. Minimal container images ship
  neither: `apt-get install file binutils`.
- **`No CMAKE_CUDA_COMPILER could be found`** — `nvcc` is not on `PATH`, or the
  CUDA Toolkit does not match the tested version. Install a compatible CUDA Toolkit
  (see the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md))
  with `nvcc` on `PATH` or set `CUDACXX`; CPU-only builds do not need CUDA.
