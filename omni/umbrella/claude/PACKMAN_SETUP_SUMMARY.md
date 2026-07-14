# Packman Setup Summary

## Overview

The Physics Umbrella library now uses packman to fetch pybind11, doctest, and Python dependencies, matching the setup used in physics_hydra project.

## Changes Made

### 1. Created Packman Configuration

**`deps/target-deps.packman.xml`**:
```xml
<project toolsVersion="6.11" chroniclePath="../_build/chronicles">
    <dependency name="cmake" linkPath="../_build/target-deps/cmake">
        <package name="cmake-windows-x86_64" version="3.25.1" platforms="windows-x86_64"/>
        <package name="cmake-linux-x86_64" version="3.25.1" platforms="linux-x86_64"/>
    </dependency>

    <dependency name="pybind11" linkPath="../_build/target-deps/pybind11">
        <package name="pybind11" version="2.11.1-0" />
    </dependency>

    <dependency name="doctest" linkPath="../_build/target-deps/doctest">
        <package name="doctest" version="2.4.5+nv1-3" />
    </dependency>

    <dependency name="python-3.11" linkPath="../_build/target-deps/python-3.11">
        <package name="python" version="3.11.11+nv1-${platform}" />
    </dependency>
</project>
```

### 2. Copied Packman Tools

Copied packman tools from physics_hydra to `umbrella/tools/packman/`:
- `packman.cmd` (Windows)
- `packman` (Linux/Mac)
- `packmanconf.py`
- `config.packman.xml`
- `bootstrap/` directory

### 3. Created Dependency Pull Scripts

**`pull_dependencies.bat`** (Windows):
```batch
call tools\packman\packman.cmd pull deps\target-deps.packman.xml --platform windows-x86_64
```

**`pull_dependencies.sh`** (Linux/Mac):
```bash
./tools/packman/packman pull deps/target-deps.packman.xml --platform linux-x86_64
```

### 4. Updated Build Scripts

Both `build.bat` and `build.sh` now call `pull_dependencies` first:
```batch
echo Pulling dependencies...
call pull_dependencies.bat
```

### 5. Updated CMake Configuration

**`bindings/CMakeLists.txt`**:
- Checks for pybind11 at `${UMBRELLA_ROOT_DIR}/../_build/target-deps/pybind11`
- Uses packman-fetched Python from `../_build/target-deps/python`
- Sets Python version to 3.12 (actual version in packman)
- Falls back to system pybind11 if packman version not found
- Provides clear instructions to run `pull_dependencies` if not found

**`tests/cpp/CMakeLists.txt`**:
- Uses doctest from `${UMBRELLA_ROOT_DIR}/../_build/target-deps/doctest`
- Provides instructions to run `pull_dependencies` if not found

## Dependencies Fetched

When `pull_dependencies` is run, packman fetches:

1. **pybind11 2.11.1-0**
   - Location: `_build/target-deps/pybind11/`
   - Used for: Python bindings

2. **doctest 2.4.5+nv1-3**
   - Location: `_build/target-deps/doctest/`
   - Used for: C++ unit tests

3. **Python 3.12.12+nv3**
   - Location: `_build/target-deps/python/`
   - Used for: Building and running Python bindings

4. **CMake 3.25.1** (optional)
   - Location: `_build/target-deps/cmake/`
   - Used for: Build system

## Usage

### Initial Setup

```bash
# Windows
cd umbrella
pull_dependencies.bat

# Linux/Mac
cd umbrella
chmod +x pull_dependencies.sh
./pull_dependencies.sh
```

### Building

Dependencies are automatically pulled by build scripts:

```bash
# Windows
build.bat

# Linux/Mac
./build.sh
```

### Manual Dependency Pull

If you want to pull dependencies without building:

```bash
# Windows
pull_dependencies.bat

# Linux/Mac
./pull_dependencies.sh
```

## Directory Structure

```
umbrella/
├── deps/
│   └── target-deps.packman.xml       # Packman dependencies
├── tools/
│   └── packman/                      # Packman tools (copied from physics_hydra)
│       ├── packman.cmd               # Windows packman
│       ├── packman                   # Linux/Mac packman
│       ├── packmanconf.py            # Configuration
│       ├── config.packman.xml        # Packman config
│       └── bootstrap/                # Bootstrap files
├── pull_dependencies.bat             # Windows dependency pull script
├── pull_dependencies.sh              # Linux/Mac dependency pull script
├── build.bat                         # Updated to pull deps first
└── build.sh                          # Updated to pull deps first

../_build/target-deps/                # Packman-fetched dependencies (shared)
├── pybind11/                         # pybind11 2.11.1-0
├── doctest/                          # doctest 2.4.5+nv1-3
├── python/                           # Python 3.12.12+nv3
└── cmake/                            # CMake 3.25.1 (optional)
```

## Benefits

1. **No Manual Installation**: No need to `pip install pybind11` or install doctest manually
2. **Consistent Versions**: Everyone uses the same versions of dependencies
3. **Hermetic Builds**: Dependencies are isolated from system installations
4. **Matches physics_hydra**: Same setup as physics_hydra for consistency
5. **Offline Capable**: Once fetched, dependencies are cached locally

## Build Output

With packman dependencies, the build now produces:

- **C++ Library**: `PhysicsUmbrella_d.dll` (Debug) / `PhysicsUmbrella.dll` (Release)
- **Python Bindings**: `_physics_umbrella.cp312-win_amd64.pyd` (Windows) or `_physics_umbrella.cpython-312-x86_64-linux-gnu.so` (Linux)
- **C++ Tests**: `test_umbrella_basic.exe`, `test_umbrella_suite.exe`

## CMake Output

When configured, CMake now reports:

```
-- Using pybind11 from packman: R:/src/physics/omni/umbrella/../_build/target-deps/pybind11
-- pybind11 v2.11.1
-- Found PythonInterp: R:/src/physics/omni/_build/target-deps/python/python.exe
-- Found PythonLibs: R:/src/physics/omni/umbrella/../_build/target-deps/python/libs/python312.lib
-- Building Python bindings with pybind11
-- Building C++ tests
-- Found doctest at R:/src/physics/omni/umbrella/../_build/target-deps/doctest/include
```

## Fallback Behavior

If packman dependencies are not found, CMake falls back to system installations:

1. Searches for system `pybind11` via `find_package(pybind11 CONFIG)`
2. If not found, provides instructions:
   ```
   Run: ./pull_dependencies.bat (or .sh) to fetch pybind11 via packman
   Or install pybind11: pip install pybind11
   ```

## Troubleshooting

### Packman Warnings

```
packman(WARNING): Remote 'user:cloudfront' defined but not listed in 'remotes' attribute
```
**Solution**: These warnings are harmless and can be ignored.

### Python Version Mismatch

If you see Python 3.11 vs 3.12 errors:
- Packman fetches Python 3.12, not 3.11
- CMakeLists.txt has been updated to use Python 3.12
- The symlink `python-3.11` in target-deps.packman.xml points to Python 3.12

### Dependencies Not Found

```
pybind11 not found - Python bindings will not be built
```
**Solution**: Run `pull_dependencies.bat` (or `.sh`)

### Permission Denied (Linux/Mac)

```
permission denied: ./pull_dependencies.sh
```
**Solution**: `chmod +x pull_dependencies.sh build.sh tools/packman/packman`

## Comparison with Manual Setup

| Aspect | Manual (Before) | Packman (After) |
|--------|-----------------|-----------------|
| pybind11 | `pip install pybind11` | Fetched by packman |
| doctest | Manual download | Fetched by packman |
| Python | System Python | Packman Python 3.12 |
| Version Control | Varies by user | Fixed versions |
| Setup Steps | Multiple | Single `pull_dependencies` |
| Consistency | ❌ Varies | ✅ Consistent |
| Offline | ❌ Requires network | ✅ Cached locally |

## Next Steps

1. Run `pull_dependencies.bat` (or `.sh`)
2. Build with `build.bat` (or `.sh`)
3. Python bindings will be at: `_build/windows-x86_64/lib/Debug/_physics_umbrella.cp312-win_amd64.pyd`
4. Copy bindings to `bindings/` directory to run Python tests

## See Also

- `deps/target-deps.packman.xml` - Dependency configuration
- `tools/packman/` - Packman tools
- `pull_dependencies.bat/sh` - Dependency pull scripts
- `bindings/CMakeLists.txt` - Python bindings configuration
- `tests/cpp/CMakeLists.txt` - C++ tests configuration
