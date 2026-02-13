# PhysX Packman Dependency Removal - Progress Log

## Overall Goal
Remove dependency on NVIDIA's proprietary package manager 'packman' from the PhysX SDK project located in `/home/jtwenty10/github/PhysX/physx/buildtools/packman`.

## Strategy
Replace packman-managed dependencies with system-provided packages, starting with the most straightforward dependencies and working toward more complex ones.

## Current Status Summary

- ✅ **Phase 1 Complete**: OpenGL/GLUT replaced with system packages
- ✅ **Phase 2 Complete**: Full audit of packman dependencies
- ✅ **Phase 3 Complete**: RapidJSON replaced with system package
- ✅ **Phase 4 Complete**: System build tools (CMake, Clang, Make) working without packman
- 🔲 **Phase 5 Future**: Remove metadata generation feature (large surgery)

### 🎉 Major Milestone Achieved!

**PhysX can now be built for Linux WITHOUT PACKMAN** for standard use cases. Use the new `generate_projects_no_packman.sh` script to build completely independently of packman. The only remaining packman dependency is the metadata generation tool (clang-physxmetadata), which is scheduled for removal.

### Packman Dependencies Status

| Package | Platform | Status | Action |
|---------|----------|--------|--------|
| opengl-linux | Linux | ✅ DONE | Replaced with system OpenGL/GLUT |
| rapidjson | All | ✅ DONE | Replaced with `rapidjson-dev` package |
| clang-physxmetadata | All | ⚠️ DEFER | NVIDIA proprietary, remove with metadata feature |
| freeglut-windows | Windows | ⏸️ SKIP | Not needed for Linux |
| VsWhere | Windows | ⏸️ SKIP | Not needed for Linux |

### Linux Build Prerequisites (NEW)

Required system packages:
```bash
sudo apt-get install -y \
  libglut-dev \
  libglu1-mesa-dev \
  libopengl-dev \
  rapidjson-dev
```

---

## Phase 1: Replace Packman OpenGL with System OpenGL ✅ COMPLETE

### Problem
PhysX was using an ancient OpenGL package pulled via packman instead of the system's native OpenGL libraries.

### Solution Implemented

#### 1. Updated SnippetRender.cmake
**File**: `snippets/compiler/cmake/linux/SnippetRender.cmake`

Changes made:
- Added `FIND_PACKAGE(OpenGL REQUIRED)` to use system OpenGL (line 32)
- Set GLUT_LIB to lowercase "glut" for modern Linux (line 49)
- Configured platform linked libs to use: `GL GLU ${GLUT_LIB}` (line 51)

#### 2. Fixed GLUT Capitalization Bug
**Issue**: Linker was failing with error: `/usr/bin/ld: cannot find -lGLUT: No such file or directory`

**Root Cause**: Two template files had faulty conditionals that only set lowercase "glut" for aarch64, but used uppercase "GLUT" for x86_64:
```cmake
IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    SET(GLUT_LIB "glut")
ELSE()
    SET(GLUT_LIB "GLUT")  # WRONG for modern Linux!
ENDIF()
```

**Files Fixed**:
- `snippets/compiler/cmake/linux/SnippetTemplate.cmake` (lines 49-54)
- `snippets/compiler/cmake/linux/SnippetVehicleTemplate.cmake` (lines 49-54)

**Fix Applied**:
```cmake
# Modern Linux uses lowercase glut library name
SET(GLUT_LIB "glut")
```

#### 3. Build Verification
- Regenerated CMake configuration in `compiler/linux-clang-cpu-only-checked/`
- Build completed successfully (100%) with all snippets building
- No linker errors related to OpenGL/GLUT

### Build Configuration Notes
- **Primary build dir**: `compiler/linux-clang-cpu-only-checked/`
- **Faster config for testing**: Use "checked" instead of "release"
- **Build command**: `cd compiler/linux-clang-cpu-only-checked && make -j4`
- **After CMake changes**: Run `cmake .` to regenerate, then `make`

---

## Phase 2: Audit Packman Dependencies ✅ COMPLETE

### Packman Configuration
- **Config file**: `buildtools/packman/config.packman.xml`
- **Remote repos**: NVIDIA CloudFront and URM (artifactory)
- **Cache location**: `~/.cache/packman/`
- **Bootstrap script**: `generate_projects.sh` calls packman to pull dependencies

### Current Packman-Managed Packages

Found in `dependencies.xml`:

1. **clang-physxmetadata** (version 4.0.0.32489833_1)
   - Custom LLVM/Clang 4.0.0 modified for NVIDIA PhysX metadata generation
   - Required for: `requiredForDistro requiredForMetaGen`
   - Platform: Linux
   - **Status**: ⚠️ PROPRIETARY - Must stay with packman (NVIDIA-specific)

2. **VsWhere** (version 2.7.3111.17308_1.0)
   - Visual Studio locator tool
   - Platform: Windows + cross-compile toolchains
   - **Status**: ⏸️ Windows-only, skip for Linux

3. **freeglut-windows** (version 3.4_1.1)
   - FreeGLUT for Windows
   - Platform: Windows only (vc15win64, vc16win64, vc17win64)
   - **Status**: ⏸️ Windows-only, skip for Linux

4. **rapidjson** (version 1.1.0-67fac85-073453e1)
   - Fast JSON parser/generator for C++
   - Platform: All
   - **Status**: ✅ CAN REPLACE - Available as `rapidjson-dev` system package

5. **opengl-linux** (version 2017.5.19.1) - ANCIENT!
   - Old OpenGL/GLUT/MESA from 2017
   - Platform: Linux
   - **Status**: ✅ ALREADY REPLACED - Using system OpenGL

### PM_ Environment Variables in Use

All PM_ environment variables found in the codebase:

**Linux-relevant:**
- `PM_rapidjson_PATH` - RapidJSON include path
- `PM_cmake_PATH` - CMake binary (build tool)
- `PM_ninja_PATH` - Ninja build tool
- `PM_clang_PATH` - Clang compiler for builds
- `PM_CLANGCROSSCOMPILE_PATH` - Cross-compilation toolchain
- `PM_clangMetadata_PATH` - PhysX metadata generator
- `PM_CUDA_PATH` - CUDA toolkit path
- `PM_PACKAGES_ROOT` - Packman cache root
- `PM_PATHS` - Combined package paths for CMake

**Windows-only (can ignore for Linux):**
- `PM_freeglut_PATH` - FreeGLUT path
- `PM_OpenGL_VERSION` - OpenGL version
- `PM_winsdk_PATH` - Windows SDK
- `PM_MinGW_PATH` - MinGW toolchain
- `PM_SECURELOADLIBRARY_PATH` - Security library

### Packman Usage in Build System

**Entry point**: `generate_projects.sh`
```bash
# Pulls dependencies from dependencies.xml
source packman pull dependencies.xml --platform <platform>
# Then runs CMake generation
python cmake_generate_projects.py <platform>
```

**CMake integration**: `buildtools/cmake_generate_projects.py`
- Sets `CMAKE_PREFIX_PATH` to `PM_PATHS` (all packman packages)
- Uses `PM_cmake_PATH` for CMake binary
- Uses `PM_ninja_PATH` for Ninja builds
- Uses `PM_clang_PATH` for cross-compilation

### Currently Cached Packman Packages

Found in `~/.cache/packman/chk/`:
- `7za/22.01-1` - Archive tool (used by packman itself)
- `clang-physxmetadata/4.0.0.32489833_1` - ⚠️ Must keep
- `opengl-linux/2017.5.19.1` - ✅ Can delete (already replaced)
- `rapidjson/1.1.0-67fac85-073453e1` - ✅ Will replace

### System Packages Already Installed

Verified on this system:
```
✅ libglut-dev, libglut3.12 - GLUT library
✅ libopengl-dev, libopengl0 - OpenGL
✅ libglu1-mesa, libglu1-mesa-dev - GLU
✅ freeglut3-dev - FreeGLUT
```

Available but not installed:
```
🔲 rapidjson-dev - RapidJSON (need to install)

---

## Phase 3: Replace RapidJSON with System Package ✅ COMPLETE

### Goal
Replace packman's RapidJSON (version 1.1.0 from 2018) with system package `rapidjson-dev`.

### Current Usage
- **Used by**: Vehicle template snippets
- **CMake variable**: `PM_RAPIDJSON_PATH_INTERNAL` → `PM_RAPIDJSON_INCLUDE_PATH`
- **Referenced in**: `snippets/compiler/cmake/SnippetVehicleTemplate.cmake`

### Implementation Completed

1. ✅ **Installed system package**:
   ```bash
   sudo apt-get install -y rapidjson-dev
   # Installed version: 1.1.0+dfsg2-7.2
   ```

2. ✅ **Located system headers**:
   ```
   Headers installed at: /usr/include/rapidjson/
   ```

3. ✅ **Updated CMake files**:
   - Modified `snippets/compiler/cmake/SnippetVehicleTemplate.cmake`
   - Linux now uses `/usr/include` for RapidJSON
   - Windows/Mac fall back to packman version if needed
   - Changes use conditional: `IF(UNIX AND NOT APPLE)`

4. ✅ **Tested build**:
   - Regenerated CMake successfully
   - Built `SnippetVehicleFourWheelDrive` - SUCCESS
   - Built `SnippetVehicleTankDrive` - SUCCESS
   - All vehicle snippets compile with system RapidJSON

5. 🔲 **Update dependencies.xml** (Optional cleanup):
   - Can remove rapidjson dependency for Linux platforms later
   - For now, keeping it for Windows compatibility

### System vs Packman Comparison
- **Packman**: RapidJSON 1.1.0 (from 2018, commit 67fac85-073453e1)
- **System**: RapidJSON 1.1.0+dfsg2-7.2 (Ubuntu noble/universe)
- **Result**: ✅ Same version, fully compatible
- **Risk**: Low - RapidJSON is header-only and stable

---

## Phase 4: Use System Build Tools Instead of Packman ✅ COMPLETE

### Goal
Replace packman-managed build tools (CMake, Ninja, Clang) with system-installed versions.

### System Tool Versions Verified

- **CMake**: 3.28.3 (system: `/usr/bin/cmake`)
- **Clang**: 18.1.3 Ubuntu (system: `/usr/bin/clang++`)
- **Make**: 4.3 GNU (system: `/usr/bin/make`)
- **Ninja**: Not installed (not needed for Make-based builds)

### Implementation Completed

1. ✅ **Checked system tool versions** - All compatible
2. ✅ **Created packman-free generator** - `generate_projects_no_packman.sh`
3. ✅ **Bypassed packman** - Direct Python invocation with minimal env vars
4. ✅ **Tested build successfully** - All configurations generate and build correctly
5. ✅ **Updated documentation** - See below

### Key Discovery

The `cmake_generate_projects.py` script already has fallback logic:
```python
if os.environ.get('PM_cmake_PATH') is not None:
    cmakeExec = os.environ['PM_cmake_PATH'] + '/bin/cmake'
else:
    cmakeExec = 'cmake'  # Uses system cmake!
```

By simply not running packman and setting minimal environment variables, the system automatically uses system tools.

### New Build Script

Created `generate_projects_no_packman.sh` which:
- Sets `PHYSX_ROOT_DIR` to current directory
- Sets `PM_PATHS="/usr"` for system package locations
- Calls `cmake_generate_projects.py` directly
- Uses system CMake, Clang, and Make
- No packman invocation required

**Usage**:
```bash
./generate_projects_no_packman.sh linux-clang-cpu-only
cd compiler/linux-clang-cpu-only-checked
make -j$(nproc)
```

### Verification

- ✅ Generated all 4 build configurations (debug, checked, profile, release)
- ✅ CMake found system OpenGL libraries
- ✅ RapidJSON found at `/usr/include`
- ✅ Build completes successfully
- ✅ Snippets link and execute correctly

---

## Phase 5: Remove Metadata Feature (FUTURE - Large Surgery)

### Dependencies We MUST Keep (For Now)

1. **clang-physxmetadata** ⚠️
   - Custom NVIDIA tool for PhysX-specific code generation
   - Not available elsewhere
   - **USER NOTE**: This is an old feature for fast binary serialization/loading
   - **USER NOTE**: We don't need this metadata feature anymore
   - **FUTURE WORK**: This entire feature needs to be removed from the codebase
   - **STATUS**: Bigger surgery throughout codebase - defer until after RapidJSON
   - For now: Keep packman dependency for this, remove later

### Build Tools Assessment

Need to determine if we want to:
- **Use system CMake/Ninja/Clang** - Simpler, standard approach
- **Keep packman versions** - Reproducible builds, but dependency remains

Questions:
1. Does PhysX require specific CMake/Ninja versions?
2. Can we build with system compilers, or do we need packman's clang?
3. For cross-compilation, is packman's toolchain mandatory?

### Ultimate Goal Options

**Option A: Minimal Packman**
- Keep only clang-physxmetadata
- Use system packages for everything else
- Modify build scripts to conditionally use packman only for metadata tool

**Option B: Packman-Free (Stretch Goal)**
- Find alternative for metadata generation
- Or skip metadata generation if not needed for user's use case
- Completely remove packman dependency

---

## Important Notes & Lessons Learned

### Build System Notes
- PhysX uses CMake with template files in `snippets/compiler/cmake/`
- Platform-specific configs in subdirectories: `linux/`, `windows/`, etc.
- After modifying templates, must regenerate with `cmake .` before building

### Linux OpenGL/GLUT Library Names
- Modern Linux uses **lowercase** library names: `glut`, not `GLUT`
- This applies to both x86_64 and aarch64 architectures
- Use `FIND_PACKAGE(OpenGL REQUIRED)` for proper CMake integration

### Session Persistence
- **Always use tmux** to prevent losing work on disconnection
- Run `tmux attach -t work` or `tmux new -s work` before starting Claude Code
- Document progress in this file before long-running builds

### Git Status (as of last check)
```
Modified files:
- README.md
- dependencies.xml
- snippets/compiler/cmake/linux/SnippetRender.cmake
- snippets/compiler/cmake/linux/SnippetTemplate.cmake  (newly modified)
- snippets/compiler/cmake/linux/SnippetVehicleTemplate.cmake  (newly modified)
```

---

## User Requirements (Answered)

1. **Metadata Generation**: ✅ ANSWERED
   - The PhysX metadata feature (clang-physxmetadata) is OLD and NOT NEEDED
   - It was for fast binary serialization/loading
   - Will need to be removed entirely (bigger surgery throughout codebase)
   - Defer metadata removal until after other packman dependencies removed

2. **Build Tool Preferences**: ❓ TO BE DETERMINED
   - System CMake/Ninja/Clang (standard, easier to maintain)
   - Packman's versions (reproducible, but keeps dependency)

3. **Cross-compilation**: ❓ TO BE DETERMINED
   - Do you need cross-compilation support?
   - If YES: May need packman's cross-compile toolchains
   - If NO: Can simplify significantly

4. **Windows builds**: ❓ TO BE DETERMINED (likely NO for this work)
   - If NO: Can remove all Windows-specific packman dependencies from consideration

5. **Target platforms**: ❓ TO BE DETERMINED
   - Linux x86_64 only?
   - Linux aarch64?
   - Others?

## Removal Roadmap

Based on user requirements, the order of operations:

1. ✅ **Phase 1**: Replace OpenGL/GLUT - COMPLETE
2. ✅ **Phase 2**: Audit dependencies - COMPLETE
3. 🔄 **Phase 3**: Replace RapidJSON - IN PROGRESS
4. 🔲 **Phase 4**: Replace/remove other dependencies (build tools, etc.)
5. 🔲 **Phase 5**: Remove metadata generation feature (large codebase surgery)
6. 🔲 **Phase 6**: Remove packman infrastructure entirely

---

## File Change Log

### 2026-02-13 - Phase 1: OpenGL System Integration
- ✅ Modified: `snippets/compiler/cmake/linux/SnippetRender.cmake`
- ✅ Modified: `snippets/compiler/cmake/linux/SnippetTemplate.cmake`
- ✅ Modified: `snippets/compiler/cmake/linux/SnippetVehicleTemplate.cmake` (GLUT fix)
- ✅ Verified: Build completes successfully in checked configuration

### 2026-02-13 - Phase 3: RapidJSON System Integration
- ✅ Installed: System package `rapidjson-dev` (version 1.1.0+dfsg2-7.2)
- ✅ Modified: `snippets/compiler/cmake/SnippetVehicleTemplate.cmake`
  - Added conditional to use `/usr/include` for Linux
  - Kept packman fallback for Windows/Mac
- ✅ Modified: `README.md` - Updated prerequisites and packman notes
- ✅ Verified: Vehicle snippets build successfully with system RapidJSON

### 2026-02-13 - Phase 4: System Build Tools Integration
- ✅ Verified system tools: CMake 3.28.3, Clang 18.1.3, Make 4.3
- ✅ Created: `generate_projects_no_packman.sh` - Packman-free build script
  - Uses system CMake, Clang, and Make
  - Sets minimal environment variables (PHYSX_ROOT_DIR, PM_PATHS)
  - No packman invocation required
- ✅ Tested: All 4 build configurations generate and build successfully
- ✅ Verified: Complete build works without packman dependencies

---

*Last updated: 2026-02-13 19:30 UTC*
*Session: Running in tmux session 'work'*
*Status: Phase 4 complete - **PACKMAN-FREE BUILDS NOW POSSIBLE!***
