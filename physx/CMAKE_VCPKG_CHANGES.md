# CMake Changes for vcpkg Support

## Overview
Update CMake files to detect and use vcpkg-provided packages on Windows while maintaining backward compatibility with packman.

## Detection Strategy

Detect vcpkg by checking if `CMAKE_TOOLCHAIN_FILE` contains "vcpkg":
```cmake
if(DEFINED CMAKE_TOOLCHAIN_FILE AND CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg")
    set(USING_VCPKG TRUE)
else()
    set(USING_VCPKG FALSE)
endif()
```

## Files to Modify

### 1. snippets/compiler/cmake/SnippetVehicleTemplate.cmake

**Current code (lines 34-44):**
```cmake
# Use system RapidJSON instead of packman version
IF(UNIX AND NOT APPLE)
	# Linux: Use system-installed rapidjson-dev package
	SET(PM_RAPIDJSON_INCLUDE_PATH "/usr/include")
ELSE()
	# Windows/Mac: Fall back to packman if PM_rapidjson_PATH is set
	IF(NOT PM_RAPIDJSON_PATH_INTERNAL)
		SET(PM_RAPIDJSON_PATH_INTERNAL $ENV{PM_rapidjson_PATH} CACHE INTERNAL "rapidjson package path")
	ENDIF()
	SET(PM_RAPIDJSON_INCLUDE_PATH ${PM_RAPIDJSON_PATH_INTERNAL}/include)
ENDIF()
```

**Proposed changes:**
```cmake
# Detect vcpkg usage
if(DEFINED CMAKE_TOOLCHAIN_FILE AND CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg")
    set(USING_VCPKG TRUE)
else()
    set(USING_VCPKG FALSE)
endif()

# Configure RapidJSON include path based on platform and package manager
IF(UNIX AND NOT APPLE)
	# Linux: Use system-installed rapidjson-dev package
	SET(PM_RAPIDJSON_INCLUDE_PATH "/usr/include")
ELSEIF(USING_VCPKG)
	# Windows/Mac with vcpkg: Use vcpkg-provided rapidjson
	# vcpkg automatically sets up include paths via CMAKE_TOOLCHAIN_FILE
	# RapidJSON is header-only, so we just need to find it
	find_path(RAPIDJSON_INCLUDE_DIR rapidjson/rapidjson.h)
	if(RAPIDJSON_INCLUDE_DIR)
		SET(PM_RAPIDJSON_INCLUDE_PATH ${RAPIDJSON_INCLUDE_DIR})
		message(STATUS "Using vcpkg RapidJSON at: ${PM_RAPIDJSON_INCLUDE_PATH}")
	else()
		message(FATAL_ERROR "RapidJSON not found via vcpkg. Install with: vcpkg install rapidjson")
	endif()
ELSE()
	# Windows/Mac: Fall back to packman if PM_rapidjson_PATH is set
	IF(NOT PM_RAPIDJSON_PATH_INTERNAL)
		SET(PM_RAPIDJSON_PATH_INTERNAL $ENV{PM_rapidjson_PATH} CACHE INTERNAL "rapidjson package path")
	ENDIF()
	SET(PM_RAPIDJSON_INCLUDE_PATH ${PM_RAPIDJSON_PATH_INTERNAL}/include)
ENDIF()
```

### 2. snippets/compiler/cmake/windows/SnippetVehicleTemplate.cmake

**Current code (lines 31-33):**
```cmake
IF(NOT FREEGLUT_PATH)
	SET(FREEGLUT_PATH $ENV{PM_freeglut_PATH} CACHE INTERNAL "Freeglut package path")
ENDIF()
```

**Proposed changes:**
```cmake
# Detect vcpkg usage
if(DEFINED CMAKE_TOOLCHAIN_FILE AND CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg")
    set(USING_VCPKG TRUE)
else()
    set(USING_VCPKG FALSE)
endif()

IF(USING_VCPKG)
	# Use vcpkg-provided FreeGLUT
	find_package(GLUT REQUIRED)
	# vcpkg's FindGLUT will set GLUT_LIBRARIES and GLUT_INCLUDE_DIR
	SET(FREEGLUT_LIB ${GLUT_LIBRARIES})
	message(STATUS "Using vcpkg FreeGLUT")
ELSE()
	# Use packman FreeGLUT
	IF(NOT FREEGLUT_PATH)
		SET(FREEGLUT_PATH $ENV{PM_freeglut_PATH} CACHE INTERNAL "Freeglut package path")
	ENDIF()

	# Keep existing packman FREEGLUT_LIB configuration
	SET(FREEGLUT_LIB
		$<$<CONFIG:debug>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglutd.lib>
		$<$<CONFIG:checked>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
		$<$<CONFIG:profile>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
		$<$<CONFIG:release>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
	)
ENDIF()
```

### 3. Similar Changes for Other Templates

Apply similar logic to:
- `snippets/compiler/cmake/SnippetTemplate.cmake`
- `snippets/compiler/cmake/windows/SnippetTemplate.cmake`
- `snippets/compiler/cmake/SnippetRender.cmake`
- `snippets/compiler/cmake/windows/SnippetRender.cmake`

## Testing on Windows

After making these changes, test with:

```bash
# With vcpkg
generate_projects_vcpkg.bat windows-vc16
cmake --build compiler\windows-vc16-release\ --config Release

# With packman (backward compatibility test)
generate_projects.bat windows-vc16
```

Both should work!

## Key Points

1. **Backward compatible**: Packman still works if vcpkg not detected
2. **Automatic detection**: No manual flags needed, detects vcpkg toolchain file
3. **Clear errors**: If vcpkg is used but packages missing, fails with clear message
4. **Platform-specific**: Linux uses apt-get, Windows can use vcpkg, both supported

## Next Steps for Windows Session

1. Install vcpkg on Windows
2. Apply these CMake changes
3. Test generation and build
4. Update README.md with Windows instructions
5. Commit and push
