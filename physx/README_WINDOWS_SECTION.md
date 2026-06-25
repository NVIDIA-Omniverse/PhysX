# README.md Windows Section Update

## Add this section to README.md after the Linux build instructions

---

## Building PhysX for Windows (vcpkg - No Packman)

### Option 1: Using vcpkg (Recommended - No Packman Required)

PhysX can now be built on Windows without NVIDIA's packman using Microsoft's vcpkg package manager.

#### Prerequisites

1. **Install vcpkg**
   ```powershell
   # Open PowerShell as Administrator
   cd C:\
   git clone https://github.com/Microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat
   ```

   Optionally, set the VCPKG_ROOT environment variable:
   ```powershell
   [Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "User")
   ```

2. **Install Required Packages**
   ```powershell
   cd C:\vcpkg
   .\vcpkg.exe install rapidjson:x64-windows freeglut:x64-windows
   ```

3. **Install Build Tools**
   - Visual Studio 2019 or newer (with C++ development tools)
   - CMake 3.14 or newer
   - Python 3.6 or newer

#### Build Instructions

1. **Generate Project Files**
   ```cmd
   cd physx
   generate_projects_vcpkg.bat windows-vc16
   ```

   The script will:
   - Verify vcpkg is installed
   - Check that required packages are available
   - Generate Visual Studio solutions using vcpkg dependencies
   - Provide clear error messages if anything is missing

2. **Build with CMake (Command Line)**
   ```cmd
   cd compiler\windows-vc16-release
   cmake --build . --config Release -j8
   ```

3. **Or Build with Visual Studio**
   ```cmd
   # Open the generated solution
   start compiler\windows-vc16-release\PhysXSDK.sln
   ```
   Then build from within Visual Studio (Ctrl+Shift+B)

4. **Run Examples**
   ```cmd
   cd bin\windows.x86_64\release
   .\SnippetHelloWorld.exe
   .\SnippetVehicleFourWheelDrive.exe
   ```

#### Available Build Configurations

- `windows-vc16-debug` - Debug build with symbols
- `windows-vc16-checked` - Optimized with assertions
- `windows-vc16-profile` - Optimized with profiling
- `windows-vc16-release` - Full optimization

### Option 2: Using Packman (Traditional Method)

If you prefer to use NVIDIA's packman, the traditional build process still works:

```cmd
cd physx
generate_projects.bat windows-vc16
```

This will download dependencies via packman and generate projects as before.

---

## Comparison: vcpkg vs Packman

| Feature | vcpkg | Packman |
|---------|-------|---------|
| Package Manager | Microsoft vcpkg | NVIDIA proprietary |
| Installation | One-time setup | Automatic per-project |
| Dependencies | Open source packages | NVIDIA-curated packages |
| Updates | Manual (`vcpkg upgrade`) | Automatic via packman |
| Offline Builds | Yes (after initial install) | No (requires internet) |
| Platform Support | Windows, Linux, macOS | Limited |

**Recommendation:** Use vcpkg for modern Windows development. Use packman only if you need exact NVIDIA package versions or have existing packman-based workflows.

---

## Troubleshooting

### vcpkg Issues

**Error: "vcpkg not found"**
- Ensure vcpkg is installed at C:\vcpkg or set VCPKG_ROOT environment variable
- Verify vcpkg.exe exists: `C:\vcpkg\vcpkg.exe version`

**Error: "Missing required vcpkg packages"**
- Install missing packages: `C:\vcpkg\vcpkg.exe install rapidjson:x64-windows freeglut:x64-windows`
- Verify installation: `C:\vcpkg\vcpkg.exe list`

**Build Error: "Cannot find rapidjson/document.h"**
- Ensure rapidjson is installed via vcpkg
- Check CMake is using vcpkg toolchain file
- Regenerate projects: delete `compiler/` and run `generate_projects_vcpkg.bat` again

**Build Error: "Cannot find freeglut libraries"**
- Ensure freeglut is installed: `C:\vcpkg\vcpkg.exe list freeglut`
- Install if missing: `C:\vcpkg\vcpkg.exe install freeglut:x64-windows`

### General Build Issues

**Error: "Python not found"**
- Install Python 3.6+: https://www.python.org/downloads/
- Ensure Python is in PATH

**Error: CMake version too old**
- Update CMake: https://cmake.org/download/
- Minimum required: CMake 3.14

---

## Benefits of vcpkg Builds

✅ **No proprietary package manager** - Uses Microsoft's official vcpkg
✅ **Faster setup** - No packman bootstrap required
✅ **Better caching** - vcpkg packages cached locally
✅ **Open source** - All dependencies from public repositories
✅ **Cross-platform** - Same approach works on Linux (with system packages)
✅ **Modern tooling** - Integrates with Visual Studio and CMake

---
