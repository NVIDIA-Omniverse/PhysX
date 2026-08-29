# PhysX Preset Support for FetchContent Integration
# 
# This module provides functionality to apply PhysX build presets from XML files,
# similar to how generate_projects.py processes presets but integrated into CMake.
#
# Presets are applied in two phases so that switches (like PX_GENERATE_GPU_PROJECTS)
# take effect before project() and can control CUDA language selection, while
# parameters (like CMAKE_INSTALL_PREFIX) are applied after project() where
# CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT is available.
#
# Usage:
#   include("${CMAKE_CURRENT_LIST_DIR}/buildtools/cmake/PhysXPresets.cmake")
#   physx_apply_preset_switches(${PHYSX_PRESET})   # before project()
#   project(MyProject ...)
#   physx_apply_preset_params()                     # after project()

# IMPORTANT: Capture CMAKE_CURRENT_LIST_DIR at file parse time, not inside functions.
# Inside functions, CMAKE_CURRENT_LIST_DIR refers to the file calling the function!
set(_PHYSX_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE INTERNAL "PhysX root directory")
set(_PHYSX_PRESETS_DIR "${_PHYSX_ROOT_DIR}/buildtools/presets" CACHE INTERNAL "")

# Function to display available presets
function(physx_list_available_presets)
    message(STATUS "Available PhysX presets:")
    message(STATUS "")
    message(STATUS "Windows (Visual Studio):")
    message(STATUS "  vc17win64              - VS2022, GPU enabled, shared libraries")
    message(STATUS "  vc17win64-cpu-only     - VS2022, CPU only, shared libraries") 
    message(STATUS "  vc16win64              - VS2019, GPU enabled, shared libraries")
    message(STATUS "  vc16win64-cpu-only     - VS2019, CPU only, shared libraries")
    message(STATUS "")
    message(STATUS "Linux (GCC/Clang):")
    message(STATUS "  linux-gcc              - GCC, GPU enabled, static libraries")
    message(STATUS "  linux-gcc-cpu-only     - GCC, CPU only, static libraries")
    message(STATUS "  linux-clang            - Clang, GPU enabled, static libraries")
    message(STATUS "  linux-clang-cpu-only   - Clang, CPU only, static libraries")
    message(STATUS "")
    message(STATUS "Linux ARM64:")
    message(STATUS "  linux-aarch64-gcc      - ARM64 GCC, GPU enabled, static libraries")
    message(STATUS "  linux-aarch64-clang    - ARM64 Clang, GPU enabled, static libraries")
    message(STATUS "")
    message(STATUS "Usage: set(PHYSX_PRESET \"vc17win64\" CACHE STRING \"\")")
endfunction()

# Phase 1 - call BEFORE project().
# Reads the preset XML and applies boolean switches (PX_GENERATE_GPU_PROJECTS, etc.)
# so they are available when the project() call decides whether to enable CUDA.
function(physx_apply_preset_switches preset_name)
    _physx_resolve_preset_file("${preset_name}")

    message(STATUS "Applying preset switches from: ${_PHYSX_PRESET_FILE}")
    file(READ "${_PHYSX_PRESET_FILE}" PRESET_CONTENT)
    physx_parse_cmake_switches("${PRESET_CONTENT}")
    message(STATUS "PhysX preset '${preset_name}' switches applied (phase 1 - before project)")
endfunction()

# Phase 2 - call AFTER project().
# Applies string parameters (CMAKE_INSTALL_PREFIX needs CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
# and overrides options that are incompatible with FetchContent subproject builds.
function(physx_apply_preset_params)
    if(NOT _PHYSX_PRESET_FILE)
        return()
    endif()

    file(READ "${_PHYSX_PRESET_FILE}" PRESET_CONTENT)
    physx_parse_cmake_params("${PRESET_CONTENT}")
    physx_override_fetchcontent_incompatible_options()
    message(STATUS "PhysX preset params applied (phase 2 - after project)")
endfunction()

# Internal helper: resolve and cache the preset file path.
function(_physx_resolve_preset_file preset_name)
    set(PUBLIC_PRESET_PATH "${_PHYSX_PRESETS_DIR}/public/${preset_name}.xml")
    set(INTERNAL_PRESET_PATH "${_PHYSX_PRESETS_DIR}/${preset_name}.xml")

    if(EXISTS "${PUBLIC_PRESET_PATH}")
        set(_PHYSX_PRESET_FILE "${PUBLIC_PRESET_PATH}" CACHE INTERNAL "Resolved preset file" FORCE)
        message(STATUS "Using public preset: ${preset_name}")
    elseif(EXISTS "${INTERNAL_PRESET_PATH}")
        set(_PHYSX_PRESET_FILE "${INTERNAL_PRESET_PATH}" CACHE INTERNAL "Resolved preset file" FORCE)
        message(STATUS "Using internal preset: ${preset_name}")
    else()
        physx_list_available_presets()
        message(FATAL_ERROR "Preset '${preset_name}' not found. See available presets listed above.")
    endif()
endfunction()

# Function to parse CMake switches from preset XML content
function(physx_parse_cmake_switches xml_content)
    # Common PhysX CMake switches that we support
    set(SUPPORTED_SWITCHES
        "PX_BUILDSNIPPETS"
        "PX_BUILDPVDRUNTIME"
        "PX_GENERATE_STATIC_LIBRARIES"
        "PX_GENERATE_GPU_PROJECTS"
        "PX_GENERATE_GPU_PROJECTS_ONLY"
        "PX_GENERATE_GPU_STATIC_LIBRARIES"
        "PX_SCALAR_MATH"
        "NV_USE_STATIC_WINCRT"
        "NV_USE_DEBUG_WINCRT"
        "PX_FLOAT_POINT_PRECISE_MATH"
        "PX_GENERATE_GPU_REDUCED_ARCHITECTURES"
    )
    
    foreach(SWITCH_NAME ${SUPPORTED_SWITCHES})
        # Look for the switch in XML content using regex
        # Pattern: <cmakeSwitch name="SWITCH_NAME" value="True|False"
        string(REGEX MATCH "cmakeSwitch name=\"${SWITCH_NAME}\" value=\"([^\"]+)\"" SWITCH_MATCH "${xml_content}")
        
        if(SWITCH_MATCH)
            # Extract the value (True/False)
            string(REGEX REPLACE ".*value=\"([^\"]+)\".*" "\\1" SWITCH_VALUE "${SWITCH_MATCH}")
            
            # Convert to CMake boolean and set the cache variable.
            # Use set(... CACHE BOOL ...) WITHOUT FORCE so that explicit
            # command-line -D flags take precedence over preset values.
            # CMake's cache semantics: if the variable is already in the cache
            # (from -D), set() without FORCE is a no-op, preserving the user's choice.
            if("${SWITCH_VALUE}" STREQUAL "True")
                set(${SWITCH_NAME} ON CACHE BOOL "Set by preset ${PHYSX_PRESET}")
                message(STATUS "  ${SWITCH_NAME} = ${${SWITCH_NAME}} (preset default: ON)")
            elseif("${SWITCH_VALUE}" STREQUAL "False")
                set(${SWITCH_NAME} OFF CACHE BOOL "Set by preset ${PHYSX_PRESET}")
                message(STATUS "  ${SWITCH_NAME} = ${${SWITCH_NAME}} (preset default: OFF)")
            endif()
        endif()
    endforeach()
endfunction()

# Function to parse CMake parameters from preset XML content
function(physx_parse_cmake_params xml_content)
    # Common PhysX CMake parameters that we support
    set(SUPPORTED_PARAMS
        "CMAKE_INSTALL_PREFIX"
    )
    
    foreach(PARAM_NAME ${SUPPORTED_PARAMS})
        # Look for the parameter in XML content
        # Pattern: <cmakeParam name="PARAM_NAME" value="some_value"
        string(REGEX MATCH "cmakeParam name=\"${PARAM_NAME}\" value=\"([^\"]+)\"" PARAM_MATCH "${xml_content}")
        
        if(PARAM_MATCH)
            # Extract the value
            string(REGEX REPLACE ".*value=\"([^\"]+)\".*" "\\1" PARAM_VALUE "${PARAM_MATCH}")
            
            # For CMAKE_INSTALL_PREFIX, make it relative to PhysX root if it's a relative path
            if("${PARAM_NAME}" STREQUAL "CMAKE_INSTALL_PREFIX")
                if(NOT IS_ABSOLUTE "${PARAM_VALUE}")
                    # Use captured _PHYSX_ROOT_DIR, NOT CMAKE_CURRENT_LIST_DIR (wrong inside functions!)
                    set(PARAM_VALUE "${_PHYSX_ROOT_DIR}/${PARAM_VALUE}")
                endif()
                # Only apply preset value if user hasn't explicitly set CMAKE_INSTALL_PREFIX.
                # CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT is TRUE when the prefix is still
                # at platform default (/usr/local or C:/Program Files/...), indicating user
                # hasn't overridden it via -DCMAKE_INSTALL_PREFIX=... or cache.
                # Note: The old check (NOT DEFINED CMAKE_INSTALL_PREFIX) never worked because
                # CMake always defines this variable with a platform default after project().
                if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
                    set(${PARAM_NAME} "${PARAM_VALUE}" CACHE PATH "Set by preset ${PHYSX_PRESET}" FORCE)
                    message(STATUS "  ${PARAM_NAME} = ${PARAM_VALUE} (from preset)")
                else()
                    message(STATUS "  ${PARAM_NAME}: Respecting user's value: ${CMAKE_INSTALL_PREFIX}")
                endif()
            else()
                # Set the cache variable for non-CMAKE_INSTALL_PREFIX params
                set(${PARAM_NAME} "${PARAM_VALUE}" CACHE STRING "Set by preset ${PHYSX_PRESET}" FORCE)
                message(STATUS "  ${PARAM_NAME} = ${PARAM_VALUE} (from preset)")
            endif()
        endif()
    endforeach()
endfunction()

# Function to override preset options that are incompatible with FetchContent
function(physx_override_fetchcontent_incompatible_options)
    # Disable snippets - they require FreeGLUT which we don't support in FetchContent
    if(PX_BUILDSNIPPETS)
        set(PX_BUILDSNIPPETS OFF CACHE BOOL "Disabled for FetchContent (requires FreeGLUT)" FORCE)
        message(STATUS "  PX_BUILDSNIPPETS = OFF (overridden for FetchContent compatibility)")
    endif()
    
    # Disable PVD Runtime by default for simpler builds
    if(PX_BUILDPVDRUNTIME)
        set(PX_BUILDPVDRUNTIME OFF CACHE BOOL "Disabled for FetchContent (optional component)" FORCE) 
        message(STATUS "  PX_BUILDPVDRUNTIME = OFF (overridden for FetchContent compatibility)")
    endif()
    
    # For cpu-only presets, ensure GPU-only mode is also disabled
    if(PHYSX_PRESET MATCHES "cpu-only")
        set(PX_GENERATE_GPU_PROJECTS_ONLY OFF CACHE BOOL "Disabled for CPU-only preset" FORCE)
    endif()
endfunction()

