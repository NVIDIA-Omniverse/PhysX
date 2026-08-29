# PhysX FetchContent Integration

Integrate PhysX into CMake projects via `FetchContent` with zero manual setup.

## Requirements

- **CMake** 3.21 or later
- **C++14**-capable compiler (GCC 7+, Clang 5+, MSVC 2017+)
- **GPU builds (optional):** CUDA toolkit and a compatible NVIDIA GPU

## How to Use

### Basic Usage with Presets (Recommended)
```cmake
cmake_minimum_required(VERSION 3.21)
project(MyPhysXApp LANGUAGES CXX)

include(FetchContent)

# Configure PhysX using a preset (easy!)
set(PHYSX_PRESET "vc17win64" CACHE STRING "" FORCE)      # Windows
# set(PHYSX_PRESET "linux-gcc" CACHE STRING "" FORCE)    # Linux  
# set(PHYSX_PRESET "vc17win64-cpu-only" CACHE STRING "" FORCE)  # No GPU

# Download and configure PhysX (pin GIT_TAG to a release tag or commit you need)
FetchContent_Declare(
    physx_lib
    GIT_REPOSITORY https://github.com/NVIDIA-Omniverse/PhysX.git
    GIT_TAG        origin/main  # Or pin to a specific release tag/commit
    SOURCE_SUBDIR  physx
)

FetchContent_MakeAvailable(physx_lib)

# Create your application
add_executable(my_app main.cpp)

# Link with PhysX (automatically includes headers, libraries, definitions)
target_link_libraries(my_app PRIVATE physx_lib)
```

### Manual Configuration (Advanced)
```cmake
# Instead of presets, set options explicitly before FetchContent_Declare:
set(PX_GENERATE_GPU_PROJECTS OFF CACHE BOOL "" FORCE)
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "" FORCE)
set(PX_BUILDSNIPPETS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    physx_lib
    GIT_REPOSITORY https://github.com/NVIDIA-Omniverse/PhysX.git
    GIT_TAG        origin/main  # Or pin to a specific release tag/commit
    SOURCE_SUBDIR  physx
)
FetchContent_MakeAvailable(physx_lib)
```

### Available Presets

PhysX provides pre-configured build presets for common platforms and use cases:

#### Windows (Visual Studio)

| Preset | Compiler | GPU | Libraries | Use Case |
|--------|----------|-----|-----------|----------|
| `vc17win64` | VS2022 | ✅ Enabled | Shared | **Most common Windows setup** |
| `vc17win64-cpu-only` | VS2022 | ❌ CPU only | Shared | **CPU-only Windows builds** |
| `vc16win64` | VS2019 | ✅ Enabled | Shared | VS2019 compatibility |
| `vc16win64-cpu-only` | VS2019 | ❌ CPU only | Shared | VS2019 CPU-only |

#### Linux (GCC/Clang)

| Preset | Compiler | GPU | Libraries | Use Case |
|--------|----------|-----|-----------|----------|
| `linux-gcc` | GCC | ✅ Enabled | Static | **Most common Linux setup** |
| `linux-gcc-cpu-only` | GCC | ❌ CPU only | Static | **CPU-only Linux builds** |
| `linux-clang` | Clang | ✅ Enabled | Static | Clang preference |
| `linux-clang-cpu-only` | Clang | ❌ CPU only | Static | Clang CPU-only |

#### Linux ARM64

| Preset | Compiler | GPU | Libraries | Use Case |
|--------|----------|-----|-----------|----------|
| `linux-aarch64-gcc` | GCC | ✅ Enabled | Static | **ARM64 Linux with GPU** |
| `linux-aarch64-gcc-cpu-only` | GCC | ❌ CPU only | Static | ARM64 Linux CPU-only |
| `linux-aarch64-clang` | Clang | ✅ Enabled | Static | ARM64 Clang with GPU |
| `linux-aarch64-clang-cpu-only` | Clang | ❌ CPU only | Static | ARM64 Clang CPU-only |

#### Choosing a Preset
```cmake
# Platform-specific recommendations:
if(WIN32)
    set(PHYSX_PRESET "vc17win64")              # Windows + GPU
    # set(PHYSX_PRESET "vc17win64-cpu-only")   # Windows, no GPU needed
elseif(UNIX AND NOT APPLE)
    set(PHYSX_PRESET "linux-gcc")              # Linux + GPU
    # set(PHYSX_PRESET "linux-gcc-cpu-only")   # Linux, no GPU needed
endif()
```

### Manual Customization Options
```cmake
# Control library types
set(PX_GENERATE_STATIC_LIBRARIES ON)        # Use static libraries
# WARNING: Do NOT enable PX_GENERATE_GPU_STATIC_LIBRARIES when PX_GENERATE_STATIC_LIBRARIES is ON.
# Building both PhysX and PhysXGpu as static causes duplicate symbol linker errors.
# Keep PhysXGpu shared (the default) when using static PhysX libraries.
# set(PX_GENERATE_GPU_STATIC_LIBRARIES ON)  # Not recommended with static PhysX

# Control features
set(PX_GENERATE_GPU_PROJECTS OFF)           # Disable GPU support
set(PX_BUILDPVDRUNTIME ON)                  # Include PVD runtime
# set(PX_BUILDSNIPPETS ON)                  # NOT recommended: +8min build, requires OpenGL/GLUT

# Then make available
FetchContent_MakeAvailable(physx_lib)
```

### Advanced Configuration

#### Custom User Presets

For reusable configurations, create a preset XML file in `physx/buildtools/presets/` (use `.user.xml` suffix to keep it out of version control):

```xml
<?xml version="1.0" encoding="utf-8"?>
<preset name="my-project" comment="Custom configuration for my application">
  <platform targetPlatform="win64" compiler="vc17" />
  <CMakeSwitches>
    <cmakeSwitch name="PX_GENERATE_GPU_PROJECTS" value="True" />
    <cmakeSwitch name="PX_GENERATE_STATIC_LIBRARIES" value="False" />
    <cmakeSwitch name="PX_BUILDPVDRUNTIME" value="True" />
    <cmakeSwitch name="NV_USE_STATIC_WINCRT" value="False" />
  </CMakeSwitches>
</preset>
```

Then reference it by filename (without `.xml`):
```cmake
set(PHYSX_PRESET "my-project.user" CACHE STRING "" FORCE)
```

#### Command-Line Configuration

Pass options directly without modifying CMakeLists.txt:
```bash
cmake -B build -DPHYSX_PRESET=vc17win64-cpu-only                    # Use a preset
cmake -B build -DPX_GENERATE_GPU_PROJECTS=OFF -DPX_GENERATE_STATIC_LIBRARIES=ON  # Manual flags
cmake -B build -DPHYSX_PRESET= -DPX_GENERATE_GPU_PROJECTS=OFF       # Bypass preset
```

#### Windows Runtime Library

Public presets use `/MD` (dynamic CRT) to match CMake defaults. For `/MT` (static CRT):
```cmake
set(NV_USE_STATIC_WINCRT ON CACHE BOOL "" FORCE)
```

### Build Configuration Types (Important for Visual Studio / Xcode)

PhysX uses custom build configuration types: `debug`, `checked`, `profile`, and `release` — not the standard CMake `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel`. These custom configs are required by the PhysX SDK (e.g., `checked` enables validation and performance profiling that doesn't map to any standard CMake config).

When PhysX is included via FetchContent, it sets `CMAKE_CONFIGURATION_TYPES` to these custom values. This has the following implications for **multi-config generators** (Visual Studio, Xcode):

- Use `--config release` (lowercase) instead of `--config Release` when building:
  ```bash
  cmake --build build --config release
  ```
- The Visual Studio configuration dropdown will show `debug`, `checked`, `profile`, `release` instead of the standard names
- CMake generator expressions like `$<CONFIG:Release>` still work — they are case-insensitive since CMake 3.19, and this MR requires CMake 3.21+
- Output directories follow PhysX conventions (e.g., `bin/win.x86_64.vc143/release/`)

**Single-config generators** (Makefiles, Ninja) are unaffected — set `CMAKE_BUILD_TYPE` as usual:
```bash
cmake -DCMAKE_BUILD_TYPE=release ../physx
```

### What the `physx_lib` target provides

`FetchContent_MakeAvailable(physx_lib)` creates an INTERFACE target named `physx_lib`. It is an INTERFACE target (not STATIC/SHARED) because PhysX can be built as either — the INTERFACE target abstracts this away. It aggregates:

- **Libraries**: PhysX, PhysXCommon, PhysXFoundation, PhysXCooking, PhysXCharacterKinematic, PhysXVehicle, PhysXExtensions, PhysXPvdSDK (and PhysXGpu when GPU is enabled)
- **Include directories**: All public PhysX headers
- **Compile definitions**: GPU/CPU mode flags, platform defines

`target_link_libraries(my_app PRIVATE physx_lib)` is all you need.

### Installing and using find_package

PhysX supports the standard CMake install + `find_package` workflow for production deployments:

```bash
# Build and install PhysX (top-level, not via FetchContent)
cmake -B build physx -DCMAKE_BUILD_TYPE=release -DCMAKE_INSTALL_PREFIX=/opt/physx \
      -DPX_GENERATE_GPU_PROJECTS=OFF -DPX_BUILDPVDRUNTIME=ON
cmake --build build --parallel
cmake --install build
```

Then consume from another project:
```cmake
cmake_minimum_required(VERSION 3.21)
project(my_app LANGUAGES CXX)

find_package(PhysX REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE ${PhysX_LIBRARIES})
target_include_directories(my_app PRIVATE ${PhysX_INCLUDE_DIRS})
```

Configure with:
```bash
cmake -B build -DCMAKE_PREFIX_PATH=/opt/physx
```

The installed package provides both legacy variables (`PhysX_LIBRARIES`, `PhysX_INCLUDE_DIRS`) and modern namespaced targets (`PhysX::PhysX`, `PhysX::PhysXCommon`, etc.).

### Debug and development options
```cmake
set(FETCHCONTENT_QUIET OFF)  # Verbose FetchContent output

# Use a local PhysX repo instead of downloading
# (path must be the repo root containing the "physx/" subdirectory)
set(FETCHCONTENT_SOURCE_DIR_PHYSX_LIB "C:/dev/physics-repo")   # Windows
set(FETCHCONTENT_SOURCE_DIR_PHYSX_LIB "/home/dev/physics-repo") # Linux
```

## Reference

### Key Files

| File | Role |
|---|---|
| `physx/CMakeLists.txt` | Root entry point — preset loading, `project()` with CUDA handling, `physx_lib` interface target, install/export logic |
| `physx/buildtools/cmake/PhysXPresets.cmake` | Two-phase preset system (switches before `project()`, params after) |
| `physx/buildtools/cmake/PhysXConfigurationTypes.cmake` | Custom build configs (debug/checked/profile/release) |
| `physx/source/compiler/cmake/PhysXConfig.cmake.in` | `find_package(PhysX)` config template for post-install consumption |
| `physx/documentation/fetchcontent/` | User guide, CI guide, working example app |

### Key Design Decisions

1. **Two-phase presets** — GPU/CUDA options resolved *before* `project()` so CUDA is only enabled when needed
2. **Subproject CUDA handling** — Uses `enable_language(CUDA)` instead of declaring it in `project()` to avoid conflicts with parent project languages
3. **`physx_lib` INTERFACE target** — Single aggregator with curated include paths; does not propagate individual target includes to avoid exposing internal source tree paths
4. **`PX_ENABLE_INSTALL`** — TRUE for top-level, FALSE for subprojects, gating all install rules
5. **Snippets and PVD forced OFF** — Snippets require OpenGL/GLUT and add ~8 min build time
6. **RPATH for shared libraries** — `$ORIGIN`-based RPATH so shared libs find each other at runtime
