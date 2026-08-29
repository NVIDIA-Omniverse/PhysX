# FetchContent CI Testing Guide

> **Note**: This document is for CI maintainers. For user integration, see [`../FetchContent.md`](../FetchContent.md).

## Key Files

| File | Role |
|---|---|
| `ci/physx/fetchcontent_tests.yml` | Full CI matrix: Linux/Windows/ARM64, CPU/GPU, shared/static, CMake 3.21+3.28, install+find_package |
| `ci/helpers/fetchcontent_test_template.cmake` | Subdirectory simulation test template |
| `ci/helpers/find_package_test_template.cmake` | Post-install `find_package` validation template |
| `physx/CMakeLists.txt` | Root FetchContent entry point (what CI tests exercise) |
| `physx/documentation/fetchcontent/examples/fetchcontent/` | Example app validated by CI |

## Key Design Decisions

1. **Three test scenarios per job** — Each CI job validates direct CMake build, subdirectory usage (simulating FetchContent), and the user-facing example project, ensuring all consumption paths work
2. **CMake version coverage** — Tests run against both the minimum supported version (3.21) and latest (3.28) to catch compatibility regressions at both ends
3. **Explicit CMake download** — CI downloads a known CMake version rather than relying on system CMake, ensuring reproducible results across runner images
4. **GPU static + PhysX static excluded** — This combination causes duplicate symbol linker errors; CI enforces `PX_GENERATE_GPU_STATIC_LIBRARIES=OFF` when `PX_GENERATE_STATIC_LIBRARIES=ON`
5. **Install + find_package validation** — A dedicated job builds, installs, then consumes PhysX via `find_package(PhysX)` to verify the full export/config workflow end-to-end
6. **Windows GPU tests disabled** — No Windows CI runners with CUDA toolkit available; GPU validation relies on Linux runners

## CI Pipeline Integration

The FetchContent functionality is automatically tested via `ci/physx/fetchcontent_tests.yml` whenever a source distro is generated.

### Test Matrix

| Platform | Configuration | CMake | GPU | Libraries |
|----------|---------------|-------|-----|-----------|
| Ubuntu 22.04 x86_64 | Release | Latest | ❌ | Shared |
| Ubuntu 22.04 x86_64 | Release | Latest | ✅ | Static |
| Ubuntu 24.04 x86_64 | Debug | Latest | ❌ | Shared |
| Ubuntu 24.04 x86_64 | Release | Latest | ✅ | All Features |
| Ubuntu 22.04 aarch64 | Release | Latest | ❌ | Static |
| Windows VC16 | Release | Latest | ❌ | Shared |
| Windows VC17 (GPU) | Release | Latest | ⚠️ Skipped | Shared *(no CUDA on runners)* |
| Windows VC17 | Debug | Latest | ❌ | Shared |
| Ubuntu 22.04 x86_64 | Release | 3.21 | ❌ | Shared |
| Ubuntu 24.04 x86_64 | Release | Latest | ❌ | Install + find_package |

### Test Scenarios Per Job

Each CI job validates three consumption paths:

1. **Direct CMake Build**: Builds `physx/CMakeLists.txt` directly — validates the root entry point
2. **Subdirectory Usage**: Uses `ci/helpers/fetchcontent_test_template.cmake` to simulate `add_subdirectory()` / `FetchContent_MakeAvailable()` — validates subproject integration
3. **Example Project**: Builds the user-facing example in `documentation/fetchcontent/examples/fetchcontent/` — validates the documented workflow

## Local Testing

### Helper Scripts

```bash
# Linux/macOS
./ci/helpers/test_fetchcontent.sh cpu     # CPU-only test
./ci/helpers/test_fetchcontent.sh gpu     # GPU-enabled test  
./ci/helpers/test_fetchcontent.sh static  # Static libraries
./ci/helpers/test_fetchcontent.sh debug   # Debug build
./ci/helpers/test_fetchcontent.sh install # Installation test
```

```batch
REM Windows
ci\helpers\test_fetchcontent.bat cpu     
ci\helpers\test_fetchcontent.bat gpu     
```

### Manual Testing

```bash
# Direct build test
mkdir test_direct && cd test_direct
cmake /path/to/physx -DCMAKE_BUILD_TYPE=Release -DPX_GENERATE_GPU_PROJECTS=OFF
cmake --build . --parallel $(nproc)

# Subdirectory test
mkdir test_subdir && cd test_subdir
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.21)
project(TestApp LANGUAGES CXX)
add_subdirectory(/path/to/physx physx_build)
add_executable(test_app /path/to/physx/documentation/fetchcontent/examples/fetchcontent/main.cpp)
target_link_libraries(test_app PRIVATE physx_lib)
EOF
cmake . -DCMAKE_BUILD_TYPE=Release -DPX_GENERATE_GPU_PROJECTS=OFF
cmake --build . --parallel
```

## Adding New Test Configurations

1. Edit `ci/physx/fetchcontent_tests.yml`
2. Add new job extending existing templates:

   ```yaml
   my_new_test:
     extends:
       - .fetchcontent-tests-common
       - .linux-fetchcontent-template
     variables:
       CMAKE_BUILD_TYPE: "Release"
       CMAKE_OPTIONS: "-DPX_MY_OPTION=ON"
   ```

## Troubleshooting

### Common Issues

| Error | Solution |
|-------|----------|
| CMake version too old | Upgrade to 3.21+ (3.28+ recommended) |
| CUDA not found | Disable GPU: `-DPX_GENERATE_GPU_PROJECTS=OFF` |
| FetchContent download fails | Use local source: `-DFETCHCONTENT_SOURCE_DIR_PHYSX_LIB=/path/to/repo` (path = directory that contains the `physx` subdirectory) |
| Compiler not compatible | Use GCC 7+, Clang 5+, or MSVC 2017+ |

### Debug Commands

```bash
# Enable verbose output
cmake .. -DFETCHCONTENT_QUIET=OFF -DCMAKE_VERBOSE_MAKEFILE=ON

# Check CMake cache
grep -E "(PX_|CUDA|PhysX)" CMakeCache.txt

# List targets
cmake --build . --target help | grep -i physx
```
