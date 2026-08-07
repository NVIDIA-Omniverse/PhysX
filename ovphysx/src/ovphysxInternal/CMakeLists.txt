# ovphysx Internal Sidecar Library
# Separates USD/omni.physx-dependent functionality into a loadable plugin.
# This solves the Windows chicken-and-egg problem: main library loads without USD,
# downloads extensions, then loads this sidecar with matching USD version.

cmake_minimum_required(VERSION 3.16)

# Create shared library for internal sidecar functionality
add_library(ovphysx_internal SHARED
    ovphysxInternal.cpp
    ovphysxInternalBootstrap.cpp
    ovphysxInternalInterop.cpp
    ovphysxInternalObjectChange.cpp
    ovphysxInternalStage.cpp
    # Compiled directly into the sidecar (not linked from ovphysx, whose symbols
    # are not exported) so ovphysxInternalStage.cpp can resolve the bundled USD
    # plugin directory and register it with PlugRegistry before the first stage
    # open. getLibraryDirectory() uses this module's address, which lives in the
    # same directory as the plugins/ tree, so the resolved path is correct.
    ${CMAKE_CURRENT_SOURCE_DIR}/../UsdSchemaPaths/UsdSchemaPaths.cpp
)

# Link USD and Carbonite (sidecar needs both)
# Use USD_LINK_DIR set by parent CMakeLists.txt (either extensions or target-deps USD)
if(USD_LINK_DIR)
    target_link_directories(ovphysx_internal PRIVATE ${USD_LINK_DIR})
    message(STATUS "Internal sidecar using USD from: ${USD_LINK_DIR}")
endif()

# The sidecar is a peer DSO (ovrtx pattern: owner uses
# CARB_STATIC_BINARY_GLOBALS, peers use CARB_GLOBALS + setter injection).
# It does NOT define CARB_STATIC_PLUGIN_BUILD and does NOT link carb.static;
# the main library injects framework + OmniCore built-ins at load.
if(NOT DEFINED CARB_STATIC_INCLUDE_DIR OR CARB_STATIC_INCLUDE_DIR STREQUAL "")
    message(FATAL_ERROR "ovphysx_internal requires CARB_STATIC_INCLUDE_DIR for Carbonite headers")
endif()
target_include_directories(ovphysx_internal BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(ovphysx_internal PRIVATE
    carb_ErrorApi=carb_ErrorApi_latest
    carb_Framework=carb_Framework_latest
    carb_filesystem_IFileSystem=carb_filesystem_IFileSystem_latest
    carb_dictionary_IDictionary=carb_dictionary_IDictionary_latest
)
if(WIN32)
    target_link_libraries(ovphysx_internal PRIVATE shlwapi pathcch winmm)
    target_compile_options(ovphysx_internal PRIVATE /wd4211)
endif()

# Link namespaced monolithic USD and the main library.
target_link_libraries(ovphysx_internal PRIVATE
    ${OVPHYSX_USD_MS_LIB}
    ovphysx
)
if(WIN32)
    target_link_libraries(ovphysx_internal PRIVATE "${OVPHYSX_TBB_LIBRARY}")
endif()

# Include directories: Carbonite, PhysX, and USD headers
# PYTHON_INCLUDE_DIRS is set by the parent CMakeLists.txt via glob

target_include_directories(ovphysx_internal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/include
)
# Third-party headers as SYSTEM to suppress warnings we don't control.
target_include_directories(ovphysx_internal SYSTEM PRIVATE
    ${CARB_SDK_INCLUDE_DIR}
    ${USD_TDEP_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${OMNI_PHYSICS_DIR}/include
    ${KIT_SDK_DIR}/dev/include
    ${LOCAL_TARGET_DEPS}/gsl/include
    # PhysX SDK headers -- needed by the kinematic-update entry point that calls
    # PxArticulationReducedCoordinate::updateKinematic directly. Other sidecar
    # entry points only deal with omni::physx interfaces, not PhysX SDK types.
    ${OVRUNTIME_DIR}/_build/target-deps/physx/include
    # ovstage headers (ovx/, ovstage/ovstage_api/, dlpack/) pulled in by
    # omni/physx/IOvxPhysicsRead.h for the physics output read entry points.
    # Use the exact provider resolved by OvstageDependency.cmake.
    ${OVSTAGE_INCLUDE_DIRS}
)

ovphysx_apply_compiler_defaults(ovphysx_internal
    GCC_SUPPRESSIONS
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-enum-compare>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-class-memaccess>
)

# Enable symbol export (defines OVPHYSX_INTERNAL_API_EXPORTS when building this DLL)
target_compile_definitions(ovphysx_internal PRIVATE
    OVPHYSX_INTERNAL_API_EXPORTS
)

# Windows-specific defines to prevent macro conflicts
if(WIN32)
    target_compile_definitions(ovphysx_internal PRIVATE
        NOMINMAX            # Prevent Windows.h min/max macros from breaking STL
        WIN32_LEAN_AND_MEAN # Reduce Windows.h bloat
    )
endif()

# Platform-specific output configuration
if(WIN32)
    set_target_properties(ovphysx_internal PROPERTIES
        OUTPUT_NAME "ovphysx_internal"
        RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
    )
else()
    set_target_properties(ovphysx_internal PROPERTIES
        OUTPUT_NAME "ovphysx_internal"
        LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
        VERSION ${OVPHYSX_VERSION_NUMERIC}
        SOVERSION ${OVPHYSX_VERSION_MAJOR}
    )
endif()

# On Linux, set RPATH so the sidecar can find Carbonite and USD at both
# build time (via absolute paths) and install time (via $ORIGIN-relative paths).
if(NOT WIN32)
    set_target_properties(ovphysx_internal PROPERTIES
        BUILD_RPATH "${USD_LINK_DIR}"
        INSTALL_RPATH "\$ORIGIN:\$ORIGIN/../plugins:\$ORIGIN/../plugins/gpu"
        BUILD_RPATH_USE_ORIGIN TRUE
        SKIP_BUILD_RPATH FALSE
        LINK_FLAGS "-Wl,--disable-new-dtags"
    )
endif()

message(STATUS "Internal sidecar will be built as separate shared library")
