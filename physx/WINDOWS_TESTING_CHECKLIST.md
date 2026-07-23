# Windows vcpkg Implementation - Testing Checklist

## Pre-Implementation Setup

### 1. Install vcpkg
```powershell
# Open PowerShell as Administrator
cd C:\
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Set environment variable (optional but recommended)
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "User")
```

**Verify:**
- [ ] vcpkg.exe runs: `C:\vcpkg\vcpkg.exe version`
- [ ] vcpkg is in PATH or VCPKG_ROOT is set

### 2. Install Required Packages
```powershell
cd C:\vcpkg
.\vcpkg.exe install rapidjson:x64-windows
.\vcpkg.exe install freeglut:x64-windows
```

**Verify:**
- [ ] `.\vcpkg.exe list rapidjson` shows package installed
- [ ] `.\vcpkg.exe list freeglut` shows package installed

## Implementation Steps

### 3. Apply CMake Changes
Use the instructions from `CMAKE_VCPKG_CHANGES.md`:

**Files to modify:**
- [ ] `snippets/compiler/cmake/SnippetVehicleTemplate.cmake`
- [ ] `snippets/compiler/cmake/windows/SnippetVehicleTemplate.cmake`
- [ ] `snippets/compiler/cmake/SnippetTemplate.cmake` (if needed)
- [ ] `snippets/compiler/cmake/windows/SnippetTemplate.cmake` (if needed)
- [ ] `snippets/compiler/cmake/SnippetRender.cmake` (if needed)
- [ ] `snippets/compiler/cmake/windows/SnippetRender.cmake` (if needed)

**Verify:**
- [ ] vcpkg detection logic added
- [ ] Backward compatibility with packman maintained
- [ ] Error messages clear if packages missing

### 4. Test Prerequisites Script
```cmd
cd C:\<path-to-repo>\physx
generate_projects_vcpkg.bat
```

**Expected behavior:**
- [ ] Script detects vcpkg installation
- [ ] Script verifies rapidjson is installed
- [ ] Script verifies freeglut is installed
- [ ] Script provides clear error if packages missing

**Test negative case:**
- [ ] Temporarily rename vcpkg directory - script should error clearly
- [ ] Uninstall rapidjson - script should list missing package
- [ ] Restore everything before continuing

### 5. Test CMake Generation
```cmd
generate_projects_vcpkg.bat windows-vc16
```

**Verify:**
- [ ] CMake runs without errors
- [ ] Detects vcpkg toolchain file
- [ ] Finds rapidjson via vcpkg
- [ ] Finds freeglut via vcpkg
- [ ] Projects generated in `compiler/` directory
- [ ] No packman calls in CMake output

**Check CMake output for:**
- [ ] "Using vcpkg RapidJSON at: ..." message
- [ ] "Using vcpkg FreeGLUT" message
- [ ] No packman-related errors

### 6. Test Build - Debug Configuration
```cmd
cd compiler\windows-vc16-debug
cmake --build . --config Debug -j8
```

**Verify:**
- [ ] Build starts without errors
- [ ] Vehicle snippets compile (SnippetVehicleFourWheelDrive, etc.)
- [ ] Links successfully
- [ ] No missing rapidjson headers error
- [ ] No missing freeglut errors

### 7. Test Build - Release Configuration
```cmd
cd ..\windows-vc16-release
cmake --build . --config Release -j8
```

**Verify:**
- [ ] Build completes successfully
- [ ] All snippets build
- [ ] Executables created in bin/

### 8. Test Executables
```cmd
cd bin\windows.x86_64\release
.\SnippetVehicleFourWheelDrive.exe
```

**Verify:**
- [ ] Snippet runs without errors
- [ ] No missing DLL errors
- [ ] Window opens (if applicable)
- [ ] Closes cleanly

**Test multiple snippets:**
- [ ] SnippetVehicleFourWheelDrive
- [ ] SnippetVehicleDirectDrive
- [ ] SnippetHelloWorld (non-vehicle snippet)

### 9. Backward Compatibility Test
Test that packman still works if vcpkg is not used:

```cmd
# Don't use generate_projects_vcpkg.bat
# Use the original script instead
generate_projects.bat windows-vc16
```

**Verify:**
- [ ] Falls back to packman
- [ ] Builds successfully with packman
- [ ] No vcpkg-related errors

## Documentation Updates

### 10. Update README.md
Add Windows vcpkg instructions:

**Sections to add:**
- [ ] Prerequisites for Windows (vcpkg installation)
- [ ] Package installation commands
- [ ] Build instructions using vcpkg
- [ ] Note that packman is still supported

### 11. Update PACKMAN_REMOVAL_PROGRESS.md
- [ ] Mark Phase 6 as complete
- [ ] Document test results
- [ ] Note any issues encountered
- [ ] Update status from "IN PROGRESS" to "COMPLETE"

## Final Verification

### 12. Clean Build Test
```cmd
# Delete compiler directory
rmdir /s /q compiler

# Regenerate and rebuild from scratch
generate_projects_vcpkg.bat windows-vc16
cd compiler\windows-vc16-release
cmake --build . --config Release -j8
```

**Verify:**
- [ ] Clean generation works
- [ ] Clean build completes
- [ ] All tests pass

## Commit and Push

### 13. Git Operations
```cmd
git status
git add .
git commit -m "Add vcpkg support for Windows builds"
git push origin remove-packman-dependencies
```

**Verify:**
- [ ] All new files committed
- [ ] CMake changes committed
- [ ] Documentation updated
- [ ] Pushed successfully

## Known Issues / Notes

Document any issues encountered:

```
[Add notes here during Windows testing session]

Example:
- Issue: vcpkg find_package didn't work for X
- Solution: Used find_path instead
- Reason: ...
```

## Success Criteria

✅ **Phase 6 Complete When:**
- [ ] vcpkg script works on Windows
- [ ] CMake detects and uses vcpkg packages
- [ ] Build completes successfully
- [ ] Executables run
- [ ] Backward compatibility with packman maintained
- [ ] Documentation updated
- [ ] Changes committed and pushed

---

**Estimated Time:** 1-2 hours for full implementation and testing
**Priority Issues:** Focus on rapidjson and freeglut first - these are critical dependencies
