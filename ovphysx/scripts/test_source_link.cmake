# ovphysx Source-Link Sample Test
# Configures, builds, and runs the hello_world_source_link sample which uses
# add_subdirectory() to build against the ovphysx source tree.
#
# The build verifies that add_subdirectory() integration works (configure,
# compile, link). For runtime execution, OVPHYSX_LIB points discovery at the
# installed SDK layout since it has the complete flattened plugin tree.
#
# Usage: cmake [-DBUILD_TYPE=Release] -P scripts/test_source_link.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

set(SAMPLE_DIR "${PROJECT_ROOT}/tests/c_samples/hello_world_source_link")
set(SAMPLE_BUILD_DIR "${PROJECT_ROOT}/_build/sample_tests/c_samples/hello_world_source_link")
set(SAMPLE_NAME "hello_world_source_link")
set(INSTALL_PATH "${PROJECT_ROOT}/_install")
set(INSTALL_LIB_PATH "${INSTALL_PATH}/${INSTALL_RUNTIME_SUBDIR}/${SHARED_LIB_PREFIX}ovphysx${SHARED_LIB_SUFFIX}")

message(STATUS "")
message(STATUS "=== Source-Link Sample Test (testing add_subdirectory workflow) ===")
message(STATUS "Sample: ${SAMPLE_DIR}")
message(STATUS "Build dir: ${SAMPLE_BUILD_DIR}")
message(STATUS "Config: ${BUILD_TYPE}")

# Verify the installed SDK exists (needed for runtime plugins)
if(NOT EXISTS "${INSTALL_PATH}/plugins")
    message(FATAL_ERROR "Installed SDK not found at ${INSTALL_PATH}.\n"
                        "Run: cmake -P scripts/install.cmake")
endif()
if(NOT EXISTS "${INSTALL_LIB_PATH}")
    message(FATAL_ERROR "Installed ovphysx library not found at ${INSTALL_LIB_PATH}.\n"
                        "Run: cmake -P scripts/install.cmake")
endif()

set(_SOURCE_LINK_OVSTAGE_ARGS)
if(DEFINED OVSTAGE_DIR AND EXISTS "${OVSTAGE_DIR}")
    list(APPEND _SOURCE_LINK_OVSTAGE_ARGS "-DOVSTAGE_DIR=${OVSTAGE_DIR}")
elseif(EXISTS "${PROJECT_ROOT}/_build/CMakeCache.txt")
    file(STRINGS "${PROJECT_ROOT}/_build/CMakeCache.txt" _OVSTAGE_CACHE_LINE REGEX "^OVSTAGE_DIR(:[^=]*)?=")
    if(_OVSTAGE_CACHE_LINE)
        list(GET _OVSTAGE_CACHE_LINE 0 _OVSTAGE_CACHE_LINE_FIRST)
        string(REGEX REPLACE "^[^=]+=" "" _OVSTAGE_CACHE_DIR "${_OVSTAGE_CACHE_LINE_FIRST}")
        if(EXISTS "${_OVSTAGE_CACHE_DIR}")
            list(APPEND _SOURCE_LINK_OVSTAGE_ARGS "-DOVSTAGE_DIR=${_OVSTAGE_CACHE_DIR}")
        endif()
    endif()
endif()

# Clean and recreate build directory
file(REMOVE_RECURSE "${SAMPLE_BUILD_DIR}")
file(MAKE_DIRECTORY "${SAMPLE_BUILD_DIR}")

# Build with the packaged toolchain the SDK was built with, not whatever Visual
# Studio the machine has.
set(_SOURCE_LINK_TOOLCHAIN_ARGS "")
ovphysx_pin_packaged_host_toolchain(_SOURCE_LINK_TOOLCHAIN_ARGS)

# Configure — deps should already be fetched by the main build, so skip auto-fetch
message(STATUS "Configuring ${SAMPLE_NAME}...")
execute_process(
    COMMAND ${CMAKE_COMMAND}
        "${SAMPLE_DIR}"
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
        -DOVPHYSX_FETCH_DEPS=OFF
        ${_SOURCE_LINK_OVSTAGE_ARGS}
        ${_SOURCE_LINK_TOOLCHAIN_ARGS}
    WORKING_DIRECTORY "${SAMPLE_BUILD_DIR}"
    RESULT_VARIABLE CONFIG_RESULT
)
if(NOT CONFIG_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} configuration failed (exit code: ${CONFIG_RESULT})")
endif()

# Build (default target — includes ovphysx, ovruntime plugins, and the sample).
# Bound parallelism: this rebuilds the full ovruntime/PhysX stack from source,
# so an unbounded `--parallel` (== make -j) exhausts RAM and thrashes the box.
ovphysx_compute_build_jobs(_SAMPLE_JOBS)
message(STATUS "Building ${SAMPLE_NAME} (parallel ${_SAMPLE_JOBS})...")
execute_process(
    COMMAND ${CMAKE_COMMAND} --build . --parallel ${_SAMPLE_JOBS} --config ${BUILD_TYPE}
    WORKING_DIRECTORY "${SAMPLE_BUILD_DIR}"
    RESULT_VARIABLE BUILD_RESULT
)
if(NOT BUILD_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} build failed (exit code: ${BUILD_RESULT})")
endif()

# Find the executable
file(GLOB_RECURSE SAMPLE_EXECUTABLE_LIST "${SAMPLE_BUILD_DIR}/*${SAMPLE_NAME}${EXE_SUFFIX}")
if(NOT SAMPLE_EXECUTABLE_LIST)
    message(FATAL_ERROR "${SAMPLE_NAME} executable not found in ${SAMPLE_BUILD_DIR}")
endif()
list(GET SAMPLE_EXECUTABLE_LIST 0 SAMPLE_EXECUTABLE)
message(STATUS "Found executable: ${SAMPLE_EXECUTABLE}")

# Run using the installed SDK layout for runtime config/schema/plugin discovery.
# The executable still finds its linked build-tree libraries via RPATH/DLL copy;
# OVPHYSX_LIB only anchors ovphysx runtime discovery at the installed SDK.
if(WIN32)
    # ovstage is not installed; use the external runtime on PATH.
    ovphysx_resolve_ovstage_paths()
    get_filename_component(SAMPLE_EXECUTABLE_DIR "${SAMPLE_EXECUTABLE}" DIRECTORY)
    set(_WINDOWS_PATH_SEGMENTS
        "${SAMPLE_EXECUTABLE_DIR}"
        "${INSTALL_PATH}/${INSTALL_RUNTIME_SUBDIR}"
        "${INSTALL_PATH}/plugins"
        "${INSTALL_PATH}/plugins/bin/deps"
        "${OVPHYSX_OVSTAGE_RUNTIME_DIR}"
        "$ENV{SystemRoot}\\System32"
        "$ENV{SystemRoot}"
    )
    file(GLOB_RECURSE _SOURCE_LINK_DLLS "${SAMPLE_BUILD_DIR}/*.dll")
    foreach(_SOURCE_LINK_DLL ${_SOURCE_LINK_DLLS})
        get_filename_component(_SOURCE_LINK_DLL_DIR "${_SOURCE_LINK_DLL}" DIRECTORY)
        list(APPEND _WINDOWS_PATH_SEGMENTS "${_SOURCE_LINK_DLL_DIR}")
    endforeach()
    if(DEFINED ENV{CUDA_PATH})
        list(APPEND _WINDOWS_PATH_SEGMENTS "$ENV{CUDA_PATH}\\bin")
    endif()
    list(REMOVE_DUPLICATES _WINDOWS_PATH_SEGMENTS)
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    string(REPLACE ";" "\\;" _WINDOWS_PATH_VALUE_ESCAPED "${_WINDOWS_PATH_VALUE}")
    set(_ENV_OVERRIDES
        "PATH=${_WINDOWS_PATH_VALUE_ESCAPED}"
        "OVPHYSX_LIB=${INSTALL_LIB_PATH}"
    )
else()
    # Clear LD_LIBRARY_PATH to verify the executable finds libs via RPATH alone.
    set(_ENV_OVERRIDES "LD_LIBRARY_PATH=" "OVPHYSX_LIB=${INSTALL_LIB_PATH}")
endif()

message(STATUS "Running ${SAMPLE_NAME} (plugins from installed SDK)...")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${_ENV_OVERRIDES} "${SAMPLE_EXECUTABLE}"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    TIMEOUT 60
    RESULT_VARIABLE RUN_RESULT
)

if(NOT RUN_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} failed (exit code: ${RUN_RESULT})")
endif()

message(STATUS "")
message(STATUS "[PASS] ${SAMPLE_NAME} passed")
message(STATUS "")
message(STATUS "Source-link sample test: PASSED")
