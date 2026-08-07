## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

cmake_minimum_required(VERSION 3.21)

# ===================================================================
# EARLY OPTIONS - Need to be declared before project() call
# ===================================================================

set(PHYSX_PRESET "" CACHE STRING "PhysX build preset (e.g., vc17win64, linux-gcc, vc17win64-cpu-only)")

# GPU options must be set before project() to determine CUDA language requirement
option(PX_GENERATE_GPU_PROJECTS "Generate GPU projects" ON)

# Apply preset switches (PX_GENERATE_GPU_PROJECTS, etc.) BEFORE project() so that
# cpu-only presets can disable CUDA without requiring a separate command-line flag.
# Parameters (CMAKE_INSTALL_PREFIX) are applied in phase 2 after project().
if(PHYSX_PRESET)
  include("${CMAKE_CURRENT_LIST_DIR}/buildtools/cmake/PhysXPresets.cmake")
  physx_apply_preset_switches(${PHYSX_PRESET})
endif()

# Detect top-level before project() - PROJECT_IS_TOP_LEVEL is only available
# after project() is called, so we check manually here.
# When used as a subproject via FetchContent, we use enable_language(CUDA)
# instead of declaring it in project() to avoid conflicts with the parent's languages.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(_PHYSX_IS_TOP_LEVEL_FOR_PROJECT TRUE)
else()
  set(_PHYSX_IS_TOP_LEVEL_FOR_PROJECT FALSE)
endif()

# Read version from PxPhysicsVersion.h - single source of truth for the SDK version.
# This avoids version drift between the header and the CMake project() declaration.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/include/foundation/PxPhysicsVersion.h" _PX_VERSION_LINES
     REGEX "#define PX_PHYSICS_VERSION_(MAJOR|MINOR|BUGFIX)")
foreach(_line IN LISTS _PX_VERSION_LINES)
  if(_line MATCHES "#define PX_PHYSICS_VERSION_MAJOR[ \t]+([0-9]+)")
    set(_PX_VER_MAJOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "#define PX_PHYSICS_VERSION_MINOR[ \t]+([0-9]+)")
    set(_PX_VER_MINOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "#define PX_PHYSICS_VERSION_BUGFIX[ \t]+([0-9]+)")
    set(_PX_VER_BUGFIX "${CMAKE_MATCH_1}")
  endif()
endforeach()
if(NOT DEFINED _PX_VER_MAJOR OR NOT DEFINED _PX_VER_MINOR OR NOT DEFINED _PX_VER_BUGFIX)
  message(FATAL_ERROR
    "Failed to parse PhysX version from include/foundation/PxPhysicsVersion.h.\n"
    "Expected PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, and PX_PHYSICS_VERSION_BUGFIX defines.")
endif()
set(_PX_VERSION "${_PX_VER_MAJOR}.${_PX_VER_MINOR}.${_PX_VER_BUGFIX}")
message(STATUS "PhysX version: ${_PX_VERSION} (from PxPhysicsVersion.h)")

if(PX_GENERATE_GPU_PROJECTS)
  if(_PHYSX_IS_TOP_LEVEL_FOR_PROJECT)
    # Top-level: declare all languages in project() - this is the normal case
    project(PhysX VERSION ${_PX_VERSION} LANGUAGES C CXX CUDA)
    message(STATUS "CUDA enabled for GPU projects (top-level project)")
  else()
    # Subproject: inherit parent's languages, then add CUDA separately.
    # enable_language(CUDA) will produce a clear error if CUDA is not found.
    project(PhysX VERSION ${_PX_VERSION})
    enable_language(CUDA)
    message(STATUS "CUDA enabled for GPU projects (subproject - used enable_language)")
  endif()

  # Post-project() CUDA validation: give a clear, actionable error if the CUDA
  # compiler was not found. project(LANGUAGES CUDA) and enable_language(CUDA)
  # already error on missing CUDA, but this provides a more user-friendly message.
  if(NOT CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR
      "PX_GENERATE_GPU_PROJECTS is ON but no CUDA compiler was found.\n"
      "Either install the CUDA toolkit, or disable GPU support:\n"
      "  -DPX_GENERATE_GPU_PROJECTS=OFF\n"
      "  or use a cpu-only preset (e.g., linux-gcc-cpu-only, vc17win64-cpu-only)")
  endif()
else()
  project(PhysX VERSION ${_PX_VERSION} LANGUAGES C CXX)
  message(STATUS "CUDA disabled - building CPU-only configuration")
endif()

# Installation helper functions
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set PHYSX_ROOT_DIR to the location of this file
set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL "Path to PhysX root directory")
set(PHYSX_CMAKE_MODULES_PATH "${PHYSX_ROOT_DIR}/source/compiler/cmake/modules" CACHE INTERNAL "")
list(APPEND CMAKE_MODULE_PATH ${PHYSX_CMAKE_MODULES_PATH})

# ===================================================================
# CUSTOM BUILD CONFIGURATIONS - PhysX uses CHECKED and PROFILE
# ===================================================================
# Shared module: sets CMAKE_CONFIGURATION_TYPES, linker flags, and compiler flags
# for PhysX's custom configs (debug, checked, profile, release).
# Also included by physx/source/compiler/cmake/CMakeLists.txt (standalone builds).
include("${CMAKE_CURRENT_LIST_DIR}/buildtools/cmake/PhysXConfigurationTypes.cmake")

# ===================================================================
# PRESET PHASE 2 - Parameters that require project() to have been called
# ===================================================================

# Phase 1 (switches) was applied before project() so PX_GENERATE_GPU_PROJECTS
# could influence CUDA language selection. Phase 2 applies CMAKE_INSTALL_PREFIX
# (needs CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) and FetchContent overrides.
if(PHYSX_PRESET)
  physx_apply_preset_params()
endif()

# ===================================================================
# MSVC RUNTIME LIBRARY - Must be set AFTER presets are applied
# ===================================================================
# Set CMAKE_MSVC_RUNTIME_LIBRARY for CRT selection (static /MT vs dynamic /MD)
# This must be set before targets are created. NvidiaBuildOptions.cmake also sets this,
# but as a non-cached variable that won't propagate to consumers.
# For FetchContent, we set it as CACHE so consumers inherit the same CRT setting.
#
# IMPORTANT: This is placed AFTER preset application so that preset values for
# NV_USE_STATIC_WINCRT take effect. Command-line -DNV_USE_STATIC_WINCRT=ON also works.
if(MSVC)
  if(NV_USE_STATIC_WINCRT)
    set(_PX_MSVC_RUNTIME "MultiThreaded$<$<CONFIG:debug>:Debug>")
  else()
    set(_PX_MSVC_RUNTIME "MultiThreaded$<$<CONFIG:debug>:Debug>DLL")
  endif()

  if(_PHYSX_IS_TOP_LEVEL_FOR_PROJECT)
    # Top-level: FORCE to ensure PhysX's CRT choice takes effect
    set(CMAKE_MSVC_RUNTIME_LIBRARY "${_PX_MSVC_RUNTIME}" CACHE STRING "MSVC runtime library" FORCE)
  else()
    # Subproject: respect parent's CRT choice if already set
    if(NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY OR CMAKE_MSVC_RUNTIME_LIBRARY STREQUAL "")
      set(CMAKE_MSVC_RUNTIME_LIBRARY "${_PX_MSVC_RUNTIME}" CACHE STRING "MSVC runtime library")
    endif()
  endif()
  message(STATUS "MSVC runtime: ${CMAKE_MSVC_RUNTIME_LIBRARY}")
endif()

# ===================================================================
# BUILD OPTIONS - Can be used standalone or overridden by presets
# ===================================================================

# Mimic Python preset parameters via options and environment variables
# Note: PX_GENERATE_GPU_PROJECTS already declared above before project() call
option(PX_GENERATE_GPU_PROJECTS_ONLY "Generate ONLY GPU projects" OFF)
option(PX_GENERATE_STATIC_LIBRARIES "Build static PhysX libs" OFF)
option(PX_GENERATE_GPU_STATIC_LIBRARIES "Build static PhysXGpu" OFF)
option(PX_BUILDSNIPPETS "Generate the snippets" OFF)
option(PX_BUILDPVDRUNTIME "Generate the OmniPVD project" OFF)
option(PX_CMAKE_SUPPRESS_REGENERATION "Disable zero_check projects" OFF)

# Guard against invalid configuration: static PhysX + static PhysXGpu causes
# duplicate symbol linker errors (e.g., generateContacts compiled in both libraries).
if(PX_GENERATE_STATIC_LIBRARIES AND PX_GENERATE_GPU_STATIC_LIBRARIES)
  message(FATAL_ERROR
    "PX_GENERATE_STATIC_LIBRARIES=ON and PX_GENERATE_GPU_STATIC_LIBRARIES=ON is not supported.\n"
    "This combination causes duplicate symbol linker errors.\n"
    "Keep PhysXGpu as shared when building PhysX as static:\n"
    "  -DPX_GENERATE_STATIC_LIBRARIES=ON -DPX_GENERATE_GPU_STATIC_LIBRARIES=OFF")
endif()

# CRITICAL: Set NV_USE_DEBUG_WINCRT to ON for FetchContent builds
# This ensures debug builds get /D "_DEBUG" instead of /D "NDEBUG"
# NvidiaBuildOptions.cmake uses this to set WINCRT_DEBUG variable,
# which is then used by windows/CMakeLists.txt to construct CMAKE_CXX_FLAGS_DEBUG
option(NV_USE_DEBUG_WINCRT "Use debug CRT for debug builds (/D_DEBUG)" ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Installation control: reuse _PHYSX_IS_TOP_LEVEL_FOR_PROJECT (set before project())
# to avoid a second, redundant top-level check.
if(_PHYSX_IS_TOP_LEVEL_FOR_PROJECT)
  set(PX_IS_TOP_LEVEL_PROJECT TRUE)
  set(PX_ENABLE_INSTALL TRUE)
  set(PX_OUTPUT_LIB_DIR "${PHYSX_ROOT_DIR}/lib" CACHE PATH "")
  set(PX_OUTPUT_BIN_DIR "${PHYSX_ROOT_DIR}/bin" CACHE PATH "")
  set(PX_OUTPUT_DLL_DIR "${PHYSX_ROOT_DIR}/bin" CACHE PATH "")
else()
  set(PX_IS_TOP_LEVEL_PROJECT FALSE)
  set(PX_ENABLE_INSTALL FALSE)
  set(PX_BUILDSNIPPETS FALSE)
  set(PX_BUILDPVDRUNTIME FALSE)
  set(PX_OUTPUT_LIB_DIR "${CMAKE_BINARY_DIR}/physx" CACHE PATH "" FORCE)
  set(PX_OUTPUT_BIN_DIR "${CMAKE_BINARY_DIR}/physx" CACHE PATH "" FORCE)
  set(PX_OUTPUT_DLL_DIR "${CMAKE_BINARY_DIR}/physx" CACHE PATH "" FORCE)
endif()

# Allow overriding some parameters from the command line for preset-like behavior.
if(NOT DEFINED TARGET_BUILD_PLATFORM)
if(WIN32)
    set(TARGET_BUILD_PLATFORM "windows" CACHE INTERNAL "Target build platform")
elseif(APPLE)
    set(TARGET_BUILD_PLATFORM "mac" CACHE INTERNAL "Target build platform")
  elseif(UNIX)
    set(TARGET_BUILD_PLATFORM "linux" CACHE INTERNAL "Target build platform")
  endif()
endif()

if(NOT DEFINED PX_OUTPUT_ARCH)
  if(TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "mac")
    set(PX_OUTPUT_ARCH "x86" CACHE INTERNAL "Output architecture")
  elseif(TARGET_BUILD_PLATFORM STREQUAL "linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      set(PX_OUTPUT_ARCH "arm" CACHE INTERNAL "Output architecture")
    else()
      set(PX_OUTPUT_ARCH "x86" CACHE INTERNAL "Output architecture")
    endif()
  endif()
endif()

# Allow defining the compiler and generator from the command-line (mimicking preset xml)
if(NOT DEFINED COMPILER)
  set(COMPILER "" CACHE STRING "Preset compiler")
endif()
if(NOT DEFINED GENERATOR)
  set(GENERATOR "" CACHE STRING "Preset generator")
endif()

message(STATUS "Target platform: ${TARGET_BUILD_PLATFORM}")
message(STATUS "Output architecture: ${PX_OUTPUT_ARCH}")
message(STATUS "Compiler: ${COMPILER}")
message(STATUS "Generator: ${GENERATOR}")

# Toolchain and compiler configuration (legacy preset compatibility).
# NOTE: CMAKE_TOOLCHAIN_FILE must be set before the first project() to take effect.
# Since project() was already called above, these settings only apply to subsequent
# sub-projects or if CMake re-configures. For cross-compilation, pass
# -DCMAKE_TOOLCHAIN_FILE on the initial cmake invocation instead.
if(TARGET_BUILD_PLATFORM STREQUAL "win64")
  if(COMPILER STREQUAL "x86_64-w64-mingw32-g++")
    set(CMAKE_TOOLCHAIN_FILE "${PHYSX_CMAKE_MODULES_PATH}/linux/WindowsCrossToolchain.linux-unknown-x86_64.cmake"
         CACHE FILEPATH "Toolchain file")
  endif()
elseif(TARGET_BUILD_PLATFORM STREQUAL "linux")
  if(COMPILER STREQUAL "clang-crosscompile")
    set(CMAKE_TOOLCHAIN_FILE "${PHYSX_CMAKE_MODULES_PATH}/linux/LinuxCrossToolchain.x86_64-unknown-linux-gnu.cmake"
         CACHE FILEPATH "Toolchain file")
    if(DEFINED ENV{PM_MinGW_PATH})
      set(CMAKE_MAKE_PROGRAM "$ENV{PM_MinGW_PATH}/bin/mingw32-make.exe")
    endif()
  elseif(COMPILER STREQUAL "clang")
    if(DEFINED ENV{PM_clang_PATH})
      set(CMAKE_C_COMPILER "$ENV{PM_clang_PATH}/bin/clang")
      set(CMAKE_CXX_COMPILER "$ENV{PM_clang_PATH}/bin/clang++")
    endif()
  endif()
elseif(TARGET_BUILD_PLATFORM STREQUAL "linuxAarch64")
  if(COMPILER STREQUAL "clang-crosscompile")
    set(CMAKE_TOOLCHAIN_FILE "${PHYSX_CMAKE_MODULES_PATH}/linux/LinuxCrossToolchain.aarch64-unknown-linux-gnueabihf.cmake"
         CACHE FILEPATH "Toolchain file")
    if(DEFINED ENV{PM_MinGW_PATH})
      set(CMAKE_MAKE_PROGRAM "$ENV{PM_MinGW_PATH}/bin/mingw32-make.exe")
    endif()
  elseif(COMPILER STREQUAL "gcc")
    # Optionally adjust settings for Packman's compiler here.
    # ...additional configuration if needed...
  elseif(COMPILER STREQUAL "clang")
    if(DEFINED ENV{PM_clang_PATH})
      set(CMAKE_C_COMPILER "$ENV{PM_clang_PATH}/bin/clang")
      set(CMAKE_CXX_COMPILER "$ENV{PM_clang_PATH}/bin/clang++")
    endif()
  endif()
endif()

# Common parameters (like the Python getCommonParams)
# Append PM_PATHS to CMAKE_PREFIX_PATH instead of overwriting to preserve caller's paths
if(DEFINED ENV{PM_PATHS})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{PM_PATHS}")
endif()

if(DEFINED ENV{GENERATE_SOURCE_DISTRO} AND "$ENV{GENERATE_SOURCE_DISTRO}" STREQUAL "1")
  set(PX_GENERATE_SOURCE_DISTRO 1)
endif()

# CUDA Integration (GPU enabled, not cross-compiling, Windows or Linux only)
if(PX_GENERATE_GPU_PROJECTS AND NOT CMAKE_CROSSCOMPILING AND (TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux"))
  # Set CUDA Toolkit root before searching (if provided via environment)
  if(DEFINED ENV{PM_CUDA_PATH})
    set(CUDAToolkit_ROOT "$ENV{PM_CUDA_PATH}" CACHE PATH "CUDA Toolkit root directory")
    message(STATUS "Using CUDA Toolkit at: $ENV{PM_CUDA_PATH}")
  endif()
  
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    message(STATUS "Found CUDAToolkit: ${CUDAToolkit_VERSION}")
    message(STATUS "CUDAToolkit_LIBRARY_DIR: ${CUDAToolkit_LIBRARY_DIR}")
  else()
    message(WARNING "CUDAToolkit not found - GPU builds may have linking issues. Ensure CUDA is installed.")
  endif()
  
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75 CACHE STRING "CUDA architectures")
  endif()
  message(STATUS "CUDA enabled with architecture ${CMAKE_CUDA_ARCHITECTURES}")
  
  if(DEFINED ENV{PM_CUDA_PATH})
    if((COMPILER STREQUAL "vc15" OR COMPILER STREQUAL "vc16" OR COMPILER STREQUAL "vc17") AND NOT GENERATOR STREQUAL "ninja")
      # Additional CUDA flags could be passed here if required.
    elseif(COMPILER STREQUAL "clang" AND DEFINED ENV{PM_clang_PATH})
      set(CMAKE_CUDA_HOST_COMPILER "$ENV{PM_clang_PATH}/bin/clang++")
    endif()
  endif()
endif()

# Platform/arch already set earlier via if(NOT DEFINED ...) guards
message(STATUS "Final configuration - Platform: ${TARGET_BUILD_PLATFORM}, Arch: ${PX_OUTPUT_ARCH}")

# Output directories are set in the if(_PHYSX_TOP_LEVEL) / else() block above


# Dependencies and FetchContent integration (as before)
include(FetchContent)
message(STATUS "Setting up dependencies...")

# Platform-specific dependencies
if(TARGET_BUILD_PLATFORM STREQUAL "linux" AND PX_BUILDSNIPPETS)
  find_package(OpenGL REQUIRED)
  message(STATUS "OpenGL found: ${OPENGL_FOUND}")
endif()

set(PUBLIC_RELEASE 1 CACHE INTERNAL "")
if(PX_CMAKE_SUPPRESS_REGENERATION)
  set(CMAKE_SUPPRESS_REGENERATION true)
endif()

# Create interface library for clients
add_library(physx_lib INTERFACE)

# Add PhysX source directories.
# When used as a subproject, EXCLUDE_FROM_ALL prevents the inner INSTALL(FILES ...)
# commands (headers, PDBs) from leaking into the consumer's install prefix.
# The root CMakeLists.txt handles its own install rules via PX_ENABLE_INSTALL.
message(STATUS "Adding PhysX SDK source...")
if(PX_ENABLE_INSTALL)
  add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_source_bin)
else()
  add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_source_bin EXCLUDE_FROM_ALL)
endif()

# Conditionally add GPU source if GPU projects are enabled
if(PX_GENERATE_GPU_PROJECTS)
  message(STATUS "Adding PhysX GPU source...")
  if(PX_ENABLE_INSTALL)
    add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmakegpu ${CMAKE_CURRENT_BINARY_DIR}/sdk_gpu_source_bin)
  else()
    add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmakegpu ${CMAKE_CURRENT_BINARY_DIR}/sdk_gpu_source_bin EXCLUDE_FROM_ALL)
  endif()

  # Link PhysXGpu against CUDA driver library
  if(TARGET PhysXGpu)
    message(STATUS "=== Configuring PhysXGpu CUDA linking ===")
    
    # Use PUBLIC linkage for static libs (dependencies must propagate to consumers)
    # Use PRIVATE linkage for shared libs (dependencies are internal)
    if(PX_GENERATE_GPU_STATIC_LIBRARIES)
      set(CUDA_LINK_SCOPE PUBLIC)
      message(STATUS "PhysXGpu is STATIC - using PUBLIC linkage for CUDA libs")
    else()
      set(CUDA_LINK_SCOPE PRIVATE)
      message(STATUS "PhysXGpu is SHARED - using PRIVATE linkage for CUDA libs")
    endif()
    
    # Link against CUDA driver API (libcuda.so on Linux, cuda.lib on Windows)
    # Background: libcuda.so.1 is part of the NVIDIA GPU driver, not the CUDA Toolkit.
    # The CUDA Toolkit includes stub libraries specifically for building without GPU drivers.
    # This is the standard approach for CI/CD environments and cross-compilation scenarios.
    
    # Strategy:
    # 1. Try to find the real driver library first (dev machines with NVIDIA driver installed)
    # 2. Fall back to stub library (CI/Docker without GPU driver, cross-compilation)
    # 3. Error if neither is found
    
    # Try to find the real driver library first (requires NVIDIA driver to be installed)
    find_library(CUDA_DRIVER_LIBRARY_REAL 
      NAMES cuda 
      PATHS /usr/lib /usr/lib64 /usr/local/cuda ${CUDAToolkit_LIBRARY_DIR}
      PATH_SUFFIXES lib64 lib/x64 lib lib64/nvidia lib/nvidia nvidia-cuda-toolkit/lib64
      NO_DEFAULT_PATH
    )
    
    # Fallback to stub library (designed for linking without GPU driver present)
    if(NOT CUDA_DRIVER_LIBRARY_REAL)
      find_library(CUDA_DRIVER_LIBRARY_STUB
        NAMES cuda
        PATHS /usr/local/cuda ${CUDAToolkit_LIBRARY_DIR}
        PATH_SUFFIXES lib64/stubs lib/stubs stubs lib/x64/stubs
      )
      if(CUDA_DRIVER_LIBRARY_STUB)
        set(CUDA_DRIVER_LIBRARY ${CUDA_DRIVER_LIBRARY_STUB})
        message(STATUS "[OK] Using CUDA driver stub library: ${CUDA_DRIVER_LIBRARY}")
        message(STATUS "  (Stub libraries are for build-time linking; real driver needed at runtime)")
      endif()
    else()
      set(CUDA_DRIVER_LIBRARY ${CUDA_DRIVER_LIBRARY_REAL})
      message(STATUS "[OK] Found real CUDA driver library: ${CUDA_DRIVER_LIBRARY}")
    endif()
    
    # Link the library we found
    if(CUDA_DRIVER_LIBRARY)
      target_link_libraries(PhysXGpu ${CUDA_LINK_SCOPE} ${CUDA_DRIVER_LIBRARY})
    else()
      message(FATAL_ERROR 
        "[ERROR] CUDA driver library (libcuda) not found - required for GPU builds.\n"
        "  Solutions:\n"
        "  1. Install NVIDIA driver (provides libcuda.so.1)\n"
        "  2. Install CUDA Toolkit (provides stub libraries in lib64/stubs/)\n"
        "  3. Disable GPU projects: -DPX_GENERATE_GPU_PROJECTS=OFF\n"
        "  Searched paths:\n"
        "    - /usr/lib, /usr/lib64, /usr/local/cuda\n"
        "    - ${CUDAToolkit_LIBRARY_DIR}")
    endif()
    
    # Also link CUDA runtime if available
    if(TARGET CUDA::cudart)
      message(STATUS "[OK] Linking PhysXGpu with CUDA::cudart")
      target_link_libraries(PhysXGpu ${CUDA_LINK_SCOPE} CUDA::cudart)
    else()
      message(STATUS "[WARN] CUDA::cudart target not available")
    endif()
    message(STATUS "=== PhysXGpu CUDA linking complete ===")
  endif()

  if(WIN32)
    # Prevent the duplicate "_64" on windows platforms by clearing the auto-appended postfixes
    if(TARGET PhysXGpu)
      set_target_properties(PhysXGpu PROPERTIES
        DEBUG_POSTFIX   ""
        RELEASE_POSTFIX ""
        PROFILE_POSTFIX ""
        CHECKED_POSTFIX ""
      )
    endif()

  endif()
endif()

# Record the two build-output directories as CMake variables
set(PHYSX_SDK_SOURCE_BIN_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_source_bin"   CACHE INTERNAL "PhysX SDK core binary dir")
if(PX_GENERATE_GPU_PROJECTS)
  set(PHYSX_SDK_GPU_SOURCE_BIN_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_gpu_source_bin" CACHE INTERNAL "PhysX SDK GPU binary dir")
else()
  set(PHYSX_SDK_GPU_SOURCE_BIN_DIR "" CACHE INTERNAL "PhysX SDK GPU binary dir (disabled)")
endif()

# Attach them as public target properties on the physx_lib target
set_target_properties(physx_lib PROPERTIES
  INTERFACE_PHYSX_SDK_SOURCE_BIN_DIR "${PHYSX_SDK_SOURCE_BIN_DIR}"
  INTERFACE_PHYSX_SDK_GPU_SOURCE_BIN_DIR "${PHYSX_SDK_GPU_SOURCE_BIN_DIR}"
)

if(PX_BUILDPVDRUNTIME)
  if(TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux")
    message(STATUS "Adding PVD Runtime...")
    add_subdirectory(${PHYSX_ROOT_DIR}/pvdruntime/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/pvdruntime_bin)
  endif()
endif()
if(PX_BUILDSNIPPETS)
  message(STATUS "Adding Snippets...")
  add_subdirectory(${PHYSX_ROOT_DIR}/snippets/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_snippets_bin)
endif()

# Link against PhysX libraries if available
set(PHYSX_LIBS "")
foreach(lib IN ITEMS PhysX PhysXCommon PhysXFoundation PhysXExtensions PhysXPvdSDK PhysXCooking PhysXCharacterKinematic PhysXVehicle)
  if(TARGET ${lib})
    message(STATUS "Linking physx_lib with ${lib}")
    target_link_libraries(physx_lib INTERFACE ${lib})
    list(APPEND PHYSX_LIBS ${lib})
    
    # Add DISABLE_CUDA_PHYSX to each library's interface when GPU is disabled
    # This ensures consumers (both FetchContent and find_package) get the definition
    if(NOT PX_GENERATE_GPU_PROJECTS)
      target_compile_definitions(${lib} INTERFACE DISABLE_CUDA_PHYSX)
    endif()
  endif()
endforeach()

# Public API include paths for physx_lib consumers.
# Individual PhysX targets set their includes as PRIVATE, so transitive propagation
# only exposes PhysXFoundation's PUBLIC ${PHYSX_ROOT_DIR}/include. The remaining
# paths must be listed explicitly here:
#   include/                   - PhysXFoundation public API (Px*.h, foundation/*.h)
#   source/foundation/include  - Foundation internals needed by consumers (PsAllocator.h, etc.)
#   source/physx/include       - PhysX module headers (PxPhysics.h forward decls)
#   source/common/include      - Shared types used across modules (CmPhysXCommon.h)
# If a new public include path is ever added to a PhysX library, it must be added here.
target_include_directories(physx_lib INTERFACE
  $<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/include>
  $<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/source/foundation/include>
  $<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/source/physx/include>
  $<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/source/common/include>
  $<INSTALL_INTERFACE:include>
)

# CRT selection (static /MT vs dynamic /MD) is handled by NvidiaBuildOptions.cmake
# via CMAKE_MSVC_RUNTIME_LIBRARY property based on NV_USE_STATIC_WINCRT option.
# Consumers linking against physx_lib will inherit the same CRT setting automatically.

foreach(lib IN LISTS PHYSX_LIBS)
  # Use get_property with DEFINED to properly detect if properties exist
  # (get_target_property returns "NOTFOUND" string which if() treats as true)
  
  # Definitions
  get_property(DEFS_DEFINED TARGET ${lib} PROPERTY INTERFACE_COMPILE_DEFINITIONS DEFINED)
  if(DEFS_DEFINED)
    get_property(DEFS TARGET ${lib} PROPERTY INTERFACE_COMPILE_DEFINITIONS)
    foreach(_d IN LISTS DEFS)
      if(_d MATCHES "(^NOTFOUND$|-NOTFOUND$)")
        message(FATAL_ERROR "Invalid INTERFACE_COMPILE_DEFINITIONS entry '${_d}' on target ${lib}")
      endif()
    endforeach()
  if(DEFS)
    target_compile_definitions(physx_lib INTERFACE ${DEFS})
    endif()
  endif()

  # Options
  get_property(OPTS_DEFINED TARGET ${lib} PROPERTY INTERFACE_COMPILE_OPTIONS DEFINED)
  if(OPTS_DEFINED)
    get_property(OPTS TARGET ${lib} PROPERTY INTERFACE_COMPILE_OPTIONS)
    foreach(_o IN LISTS OPTS)
      if(_o MATCHES "(^NOTFOUND$|-NOTFOUND$)")
        message(FATAL_ERROR "Invalid INTERFACE_COMPILE_OPTIONS entry '${_o}' on target ${lib}")
      endif()
    endforeach()
  if(OPTS)
    target_compile_options(physx_lib INTERFACE ${OPTS})
    endif()
  endif()

  # Features
  get_property(FEATS_DEFINED TARGET ${lib} PROPERTY INTERFACE_COMPILE_FEATURES DEFINED)
  if(FEATS_DEFINED)
    get_property(FEATS TARGET ${lib} PROPERTY INTERFACE_COMPILE_FEATURES)
  if(FEATS)
    target_compile_features(physx_lib INTERFACE ${FEATS})
    endif()
  endif()
  
  # Note: Deliberately NOT propagating INTERFACE_INCLUDE_DIRECTORIES from individual targets
  # to avoid source directory path issues. We set our own include paths above.
endforeach()

# GPU and CUDA integration
if(NOT CMAKE_CROSSCOMPILING AND (TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux"))
  if(TARGET PhysXGpu)
    message(STATUS "Linking physx_lib with PhysXGpu")
    target_link_libraries(physx_lib INTERFACE PhysXGpu)
    list(APPEND PHYSX_LIBS PhysXGpu)
  endif()
  
  # Link CUDA libraries to physx_lib for consumers
  # Note: PhysXGpu uses PRIVATE linkage for shared libs, so deps don't propagate
  # Consumers need to link against CUDA libs directly when using shared PhysXGpu
  if(PX_GENERATE_GPU_PROJECTS)
    if(CUDAToolkit_FOUND)
      message(STATUS "Linking physx_lib with CUDA::cudart")
      target_link_libraries(physx_lib INTERFACE CUDA::cudart)
    endif()
    
    if(CUDA_DRIVER_LIBRARY)
      message(STATUS "Linking physx_lib with CUDA driver library: ${CUDA_DRIVER_LIBRARY}")
      target_link_libraries(physx_lib INTERFACE ${CUDA_DRIVER_LIBRARY})
    else()
      message(WARNING "CUDA driver library not available for physx_lib consumers")
    endif()
  endif()
endif()

message(STATUS "PhysX CMake configuration complete")

# Only execute installation commands when appropriate
if(PX_ENABLE_INSTALL)
  message(STATUS "PhysX installation enabled")

  # Install PhysX headers
  install(DIRECTORY ${PHYSX_ROOT_DIR}/include/
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
          FILES_MATCHING PATTERN "*.h*")

  # Set RPATH on shared libraries so they can find each other after installation
  foreach(_lib IN LISTS PHYSX_LIBS)
    if(TARGET ${_lib})
      get_target_property(_lib_type ${_lib} TYPE)
      if(_lib_type STREQUAL "SHARED_LIBRARY")
        set_target_properties(${_lib} PROPERTIES INSTALL_RPATH "$ORIGIN")
      endif()
    endif()
  endforeach()

  # Install PhysX library targets and the physx_lib INTERFACE aggregator
  install(TARGETS ${PHYSX_LIBS} physx_lib
      EXPORT PhysXTargets
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # Export the targets for find_package
install(EXPORT PhysXTargets 
    FILE PhysXTargets.cmake
    NAMESPACE PhysX::
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysX)

  # Generate the version file for the configuration
  write_basic_package_version_file(
      "${CMAKE_CURRENT_BINARY_DIR}/PhysXConfigVersion.cmake"
      VERSION ${PROJECT_VERSION}
      COMPATIBILITY AnyNewerVersion)

  # Configure the config file
  configure_package_config_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/source/compiler/cmake/PhysXConfig.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/PhysXConfig.cmake"
      INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysX)

  # Install the config files
  install(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/PhysXConfig.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/PhysXConfigVersion.cmake"
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysX)
else()
  message(STATUS "PhysX installation disabled (not top-level project)")
  message(STATUS "However, shared library targets will still be available for linking.")

  # We're excluding PhysXCharacterKinematic and PhysXExtensions here since they're always static

  # Create empty lists for shared libraries
  set(SHARED_LIBS_TO_INSTALL "")

  # Add GPU library if it's not static
  if(NOT PX_GENERATE_GPU_STATIC_LIBRARIES AND TARGET PhysXGpu)
    list(APPEND SHARED_LIBS_TO_INSTALL PhysXGpu)
  endif()

  # Add core libraries if they're not static
  if(NOT PX_GENERATE_STATIC_LIBRARIES)
    foreach(lib IN ITEMS PhysX PhysXCommon PhysXCooking PhysXFoundation)
      if(TARGET ${lib})
        list(APPEND SHARED_LIBS_TO_INSTALL ${lib})
      endif()
    endforeach()
  endif()

  # Only set properties and install if we have shared libraries
  if(SHARED_LIBS_TO_INSTALL)
    # Install with proper RPATH settings so libraries can find each other
    set_target_properties(${SHARED_LIBS_TO_INSTALL}
        PROPERTIES
        INSTALL_RPATH "$ORIGIN"
        BUILD_WITH_INSTALL_RPATH TRUE)

    install(TARGETS ${SHARED_LIBS_TO_INSTALL}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        # Note: for shared libraries on Windows, CMake also generates DLL import
        # libraries (.lib) and installs them via the ARCHIVE destination.
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
  endif()
endif()
