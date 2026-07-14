# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Benchmark framework sources ---
set(BENCHMARK_FRAMEWORK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/framework/BmGlobals.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/framework/BmOutput.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/framework/BmTime.cpp
)

# --- Benchmark harness + registration ---
set(BENCHMARK_HARNESS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Harness.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BenchmarkList.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/TestHelpers.cpp
)

# --- Individual benchmark tests ---
set(BENCHMARK_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/MLIRPolicy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsAttributeChanges.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsCompetitive.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsKinematicBodies.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsReplicator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsSimulate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsUSDSimulate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics/PhysicsVehicles.cpp
)

# --- Shared test helpers (under tests/common/) ---
set(BENCHMARK_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysicsChangeTemplate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysicsReplicatorTemplate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/TestVehicleFactory.cpp
)

# --- Create benchmark executable ---
add_executable(OvruntimeBenchmarks
    ${BENCHMARK_FRAMEWORK_SOURCES}
    ${BENCHMARK_HARNESS_SOURCES}
    ${BENCHMARK_TEST_SOURCES}
    ${BENCHMARK_COMMON_SOURCES}
)

set_property(TARGET OvruntimeBenchmarks PROPERTY FOLDER "Tests")

set_target_properties(OvruntimeBenchmarks PROPERTIES
    OUTPUT_NAME "ovruntime.test.benchmarks"
)

# RTTI and exceptions enabled
if(NOT WIN32)
    target_compile_options(OvruntimeBenchmarks PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimeBenchmarks
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}                     # Harness.h, BenchmarkList.h
        ${CMAKE_CURRENT_SOURCE_DIR}/framework           # BmBenchmark.h, etc.
        ${CMAKE_CURRENT_SOURCE_DIR}/common              # TestHelpers.h
        ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/physics  # MLIRPolicy.h, Utils.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../common           # shared test helpers (PhysicsChangeTemplate, etc.)
        ${OVRUNTIME_ROOT_DIR}/include                    # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private            # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch                 # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include      # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeBenchmarks)

# Compile definitions
target_compile_definitions(OvruntimeBenchmarks PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    "DISABLE_ONNX=1"
)

# USD schema plugin path (physxSchema, physxSchemaAddition plugInfo.json registration)
# Windows: relative to exe dir (CMAKE_BINARY_DIR/<config>/) — portable across CI agents.
#   placeholder substitutes for the config subdir since file(RELATIVE_PATH) has no genex support.
# Linux: absolute paths are fine; build and test share the same filesystem layout.
if(EXISTS "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
    if(WIN32)
        file(RELATIVE_PATH _USD_EXT_REL "${CMAKE_BINARY_DIR}/placeholder"
            "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
        set(_USD_EXT_PLUGIN_PATH "${_USD_EXT_REL}")
    else()
        set(_USD_EXT_PLUGIN_PATH "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
    endif()
    target_compile_definitions(OvruntimeBenchmarks PRIVATE
        "USD_EXT_PHYSICS_PLUGIN_PATH=\"${_USD_EXT_PLUGIN_PATH}\""
    )
endif()

# Plugin search paths for Carbonite's plugin loader.
# Windows: relative to exe dir (CMAKE_BINARY_DIR/<config>/) — portable across CI agents.
#   placeholder substitutes for the config subdir since file(RELATIVE_PATH) has no genex support.
# Linux: absolute paths are fine; RPATH handles shared library discovery.
if(DEFINED CARB_SDK_DIR)
    if(WIN32)
        file(RELATIVE_PATH _CARB_REL "${CMAKE_BINARY_DIR}/placeholder" "${CARB_SDK_DIR}/_build/windows-x86_64")
        set(_CARB_PLUGIN_DIR "${_CARB_REL}/$<LOWER_CASE:$<CONFIG>>")
    else()
        set(_CARB_PLUGIN_DIR "${CARB_SDK_DIR}/_build/linux-x86_64/release")
    endif()
    target_compile_definitions(OvruntimeBenchmarks PRIVATE
        "CARB_SDK_SEARCH_PATH=\"${_CARB_PLUGIN_DIR}\""
    )
    # Python scripting plugin lives in a subdirectory (e.g. scripting-python-3.12/)
    if(EXISTS "${CARB_SDK_DIR}/_build/linux-x86_64/release/scripting-python-3.12" OR WIN32)
        if(WIN32)
            set(_CARB_SCRIPTING_DIR "${_CARB_REL}/$<LOWER_CASE:$<CONFIG>>/scripting-python-3.12")
        else()
            set(_CARB_SCRIPTING_DIR "${CARB_SDK_DIR}/_build/linux-x86_64/release/scripting-python-3.12")
        endif()
        target_compile_definitions(OvruntimeBenchmarks PRIVATE
            "CARB_SDK_SCRIPTING_SEARCH_PATH=\"${_CARB_SCRIPTING_DIR}\""
        )
    endif()
endif()

# ovruntime_deps plugins (carb.stats.plugin etc. — lives in plugins/carb.stats/ subdirectory)
if(DEFINED OVRUNTIME_DEPS_DIR)
    if(WIN32)
        file(RELATIVE_PATH _RTX_REL "${CMAKE_BINARY_DIR}/placeholder" "${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64")
        set(_RTX_PLUGINS_BASE "${_RTX_REL}/$<LOWER_CASE:$<CONFIG>>/plugins")
    else()
        set(_RTX_PLUGINS_BASE "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins")
    endif()
    target_compile_definitions(OvruntimeBenchmarks PRIVATE
        "RTX_PLUGINS_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/carb.stats\""
        "RTX_FABRIC_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/omni.fabric\""
        "RTX_CUBRIC_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/omni.cubric\""
        "RTX_USD_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/omni.usd\""
        "RTX_SCENEGRAPH_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/scenegraph\""
        "RTX_USDRT_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/usdrt\""
        "RTX_UJITSO_AGENT_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/carb.ujitsoagent\""
        "RTX_UJITSO_DEFAULT_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/carb.ujitso.default\""
        "RTX_BLOBKEY_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/omni.blobkey\""
        "RTX_DATASTORE_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/carb.datastore\""
        "RTX_GPUCOMPUTE_SEARCH_PATH=\"${_RTX_PLUGINS_BASE}/gpucompute\""
    )
endif()

# Link dependencies
target_link_libraries(OvruntimeBenchmarks
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimeBenchmarks PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimeBenchmarks PRIVATE carb)
endif()

# USD + TBB + Python libraries (handles both monolithic and non-monolithic USD)
ovruntime_link_usd_deps(OvruntimeBenchmarks)

# physicsSchemaTools + physxSchema libraries
if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimeBenchmarks PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimeBenchmarks PRIVATE physicsSchemaTools physxSchema)
endif()

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimeBenchmarks FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimeBenchmarks PRIVATE rt)
    # CUDA driver API (used by PhysicsCompetitive benchmark)
    if(DEFINED CUDA_DIR)
        target_link_directories(OvruntimeBenchmarks PRIVATE
            ${CUDA_DIR}/lib64
        )
        # Pass stubs/ as a bare -L so CMake does NOT add it to RUNPATH.
        # Prevents stub libcuda.so from shadowing the real driver at runtime.
        target_link_options(OvruntimeBenchmarks PRIVATE
            -L${CUDA_DIR}/lib64/stubs
        )
        target_link_libraries(OvruntimeBenchmarks PRIVATE cuda)
    endif()
else()
    if(DEFINED CUDA_DIR)
        target_link_directories(OvruntimeBenchmarks PRIVATE ${CUDA_DIR}/lib/x64)
        target_link_libraries(OvruntimeBenchmarks PRIVATE cuda delayimp)
        target_link_options(OvruntimeBenchmarks PRIVATE /DELAYLOAD:nvcuda.dll)
    endif()
endif()

# Precompiled header
target_precompile_headers(OvruntimeBenchmarks PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# RPATH configuration (Linux: bake library paths into binary; Windows: VS_DEBUGGER_ENVIRONMENT + CTest)
if(NOT WIN32)
    set(_BM_RPATH_DIRS "")
    if(DEFINED CARB_SDK_DIR)
        list(APPEND _BM_RPATH_DIRS "${_CARB_LIB_DIR}/release")
        if(EXISTS "${_CARB_LIB_DIR}/release/scripting-python-3.12")
            list(APPEND _BM_RPATH_DIRS "${_CARB_LIB_DIR}/release/scripting-python-3.12")
        endif()
    endif()
    if(DEFINED USD_DIR)
        list(APPEND _BM_RPATH_DIRS "${USD_DIR}/release/lib")
    endif()
    if(DEFINED PYTHON_DIR)
        list(APPEND _BM_RPATH_DIRS "${PYTHON_DIR}/lib")
    endif()
    if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        list(APPEND _BM_RPATH_DIRS "${USD_EXT_PHYSICS_DIR}/lib")
    endif()
    if(DEFINED OVRUNTIME_DEPS_DIR)
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.stats")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.fabric")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.cubric")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.usd")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/scenegraph")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/usdrt")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.ujitsoagent")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.ujitso.default")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.blobkey")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.datastore")
        list(APPEND _BM_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/gpucompute")
    endif()

    set_target_properties(OvruntimeBenchmarks PROPERTIES
        BUILD_RPATH "${_BM_RPATH_DIRS}"
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )
    target_link_options(OvruntimeBenchmarks PRIVATE "LINKER:--disable-new-dtags")
else()
    # Windows: set PATH for Visual Studio debugging and CTest
    set(_WIN_PATH_DIRS "")
    if(DEFINED CARB_SDK_DIR)
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${CARB_SDK_DIR}/_build/windows-x86_64/${OVRUNTIME_CARB_CONFIG}")
    endif()
    if(DEFINED USD_DIR)
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${USD_DIR}/release/lib;${USD_DIR}/release/bin")
    endif()
    if(DEFINED PYTHON_DIR)
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${PYTHON_DIR}")
    endif()
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/usd_ext_physics/release/lib")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_TARGET_DEPS}/usd_ext_physics/release/lib")
    endif()
    if(DEFINED OVRUNTIME_DEPS_DIR)
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/carb.stats")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/omni.fabric")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/omni.cubric")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/omni.usd")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/scenegraph")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/usdrt")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/carb.ujitsoagent")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/carb.ujitso.default")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/omni.blobkey")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/carb.datastore")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64/release/plugins/gpucompute")
    endif()
    set_target_properties(OvruntimeBenchmarks PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:OvruntimeBenchmarks>"
        VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%${_WIN_PATH_DIRS}"
    )
endif()

# CTest integration (disabled by default; enable with -DOVRUNTIME_ENABLE_BENCHMARK_CTEST=ON)
if(OVRUNTIME_ENABLE_BENCHMARK_CTEST)
    add_test(NAME ovruntime.test.benchmarks COMMAND OvruntimeBenchmarks)
    # Threading limits (match Kit CI defaults, OMPE-59303)
    set_tests_properties(ovruntime.test.benchmarks PROPERTIES
        ENVIRONMENT "PXR_WORK_THREAD_LIMIT=16;OPENBLAS_NUM_THREADS=16"
    )
    if(WIN32)
        set(_BM_CTEST_ENV "")
        if(DEFINED CARB_SDK_DIR)
            list(APPEND _BM_CTEST_ENV "PATH=path_list_prepend:${CARB_SDK_DIR}/_build/windows-x86_64/release")
        endif()
        if(DEFINED USD_DIR)
            list(APPEND _BM_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/lib")
            list(APPEND _BM_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/bin")
        endif()
        if(DEFINED PYTHON_DIR)
            list(APPEND _BM_CTEST_ENV "PATH=path_list_prepend:${PYTHON_DIR}")
        endif()
        if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
            list(APPEND _BM_CTEST_ENV "PATH=path_list_prepend:${USD_EXT_PHYSICS_DIR}/lib")
        endif()
        set_tests_properties(ovruntime.test.benchmarks PROPERTIES
            ENVIRONMENT_MODIFICATION "${_BM_CTEST_ENV}"
        )
    endif()
endif()
