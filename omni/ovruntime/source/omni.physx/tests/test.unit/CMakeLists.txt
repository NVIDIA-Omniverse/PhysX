# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Unit test sources ---
set(UNIT_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/PhysicsTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestArticulations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestArticulationD6Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestArticulationPrismaticJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestArticulationRevoluteJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestArticulationSphericalJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestCharacterController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestCollision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestCollisionRepresentation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestCollisionShapeQuery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestContactReport.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestCooking.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestD6Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestDeformables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestDistanceJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestFabricChanges.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestFabricDirectGPU.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestFabricOutputs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestGearJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestMaterial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestMeshMergeCollision.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestObjectChangeNotifications.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestParticles.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestPhysics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestRackAndPinionJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestReplicator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestRevoluteJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestRigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestTendons.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestTransformationChanges.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestVehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/physics/TestNewtonSchema.cpp
)

# Shared test helpers (under tests/common/)
set(UNIT_TEST_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysicsChangeTemplate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysicsReplicatorTemplate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/TestVehicleFactory.cpp
)

# --- Create test executable ---
add_executable(OvruntimeUnitTests
    ${UNIT_TEST_SOURCES}
    ${UNIT_TEST_COMMON_SOURCES}
)

set_property(TARGET OvruntimeUnitTests PROPERTY FOLDER "Tests")

set_target_properties(OvruntimeUnitTests PROPERTIES
    OUTPUT_NAME "ovruntime.test.unit"
)

# RTTI and exceptions enabled
if(NOT WIN32)
    target_compile_options(OvruntimeUnitTests PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimeUnitTests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}                    # test.unit root (common/TestHelpers.h)
        ${CMAKE_CURRENT_SOURCE_DIR}/physics            # PhysicsTools.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../common          # shared test helpers
        ${OVRUNTIME_ROOT_DIR}/include                   # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private           # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch                # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include     # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeUnitTests)

# Compile definitions
target_compile_definitions(OvruntimeUnitTests PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
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
    target_compile_definitions(OvruntimeUnitTests PRIVATE
        "USD_EXT_PHYSICS_PLUGIN_PATH=\"${_USD_EXT_PLUGIN_PATH}\""
    )
endif()

# Test resources path (plugInfo.json for Kit prim metadata: hide_in_stage_window, no_delete)
target_compile_definitions(OvruntimeUnitTests PRIVATE
    "UNIT_TEST_RESOURCES_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/resources\""
)

# Newton USD schema plugin path (newton_prebundle/newton_usd_schemas contains plugInfo.json)
# Windows: relative to exe dir (same pattern as USD_EXT_PHYSICS_PLUGIN_PATH above).
# Linux: absolute paths are fine.
if(DEFINED NEWTON_SCHEMAS_DIR AND EXISTS "${NEWTON_SCHEMAS_DIR}/newton_usd_schemas/plugInfo.json")
    if(WIN32)
        file(RELATIVE_PATH _NEWTON_REL "${CMAKE_BINARY_DIR}/placeholder"
            "${NEWTON_SCHEMAS_DIR}/newton_usd_schemas")
        set(_NEWTON_PLUGIN_PATH "${_NEWTON_REL}")
    else()
        set(_NEWTON_PLUGIN_PATH "${NEWTON_SCHEMAS_DIR}/newton_usd_schemas")
    endif()
    target_compile_definitions(OvruntimeUnitTests PRIVATE
        "NEWTON_SCHEMA_PLUGIN_PATH=\"${_NEWTON_PLUGIN_PATH}\""
    )
    target_include_directories(OvruntimeUnitTests PRIVATE
        ${OVRUNTIME_ROOT_DIR}/_build/generated  # NewtonSchemaTokens.h
    )
endif()

# Plugin search paths for Carbonite's plugin loader.
# Windows: relative to exe dir (CMAKE_BINARY_DIR/<config>/) — portable across CI agents.
#   placeholder substitutes for the config subdir since file(RELATIVE_PATH) has no genex support.
# Linux: absolute paths are fine; RPATH handles shared library discovery.
if(DEFINED CARB_SDK_DIR)
    if(WIN32)
        file(RELATIVE_PATH _CARB_REL "${CMAKE_BINARY_DIR}/placeholder" "${CARB_SDK_DIR}/_build/windows-x86_64")
        set(_CARB_PLUGIN_DIR "${_CARB_REL}/${OVRUNTIME_CARB_CONFIG}")
    else()
        set(_CARB_PLUGIN_DIR "${CARB_SDK_DIR}/_build/linux-x86_64/release")
    endif()
    target_compile_definitions(OvruntimeUnitTests PRIVATE
        "CARB_SDK_SEARCH_PATH=\"${_CARB_PLUGIN_DIR}\""
    )
endif()

# ovruntime_deps plugins (carb.stats.plugin etc. — lives in plugins/carb.stats/ subdirectory)
if(DEFINED OVRUNTIME_DEPS_DIR)
    if(WIN32)
        file(RELATIVE_PATH _RTX_REL "${CMAKE_BINARY_DIR}/placeholder" "${OVRUNTIME_DEPS_DIR}/_build/windows-x86_64")
        set(_RTX_PLUGINS_BASE "${_RTX_REL}/${OVRUNTIME_CARB_CONFIG}/plugins")
    else()
        set(_RTX_PLUGINS_BASE "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins")
    endif()
    target_compile_definitions(OvruntimeUnitTests PRIVATE
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
target_link_libraries(OvruntimeUnitTests
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimeUnitTests PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimeUnitTests PRIVATE carb)
endif()

# USD + TBB + Python libraries (handles both monolithic and non-monolithic USD)
ovruntime_link_usd_deps(OvruntimeUnitTests)

# physicsSchemaTools + physxSchema libraries
if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimeUnitTests PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimeUnitTests PRIVATE physicsSchemaTools physxSchema)
endif()

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimeUnitTests FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimeUnitTests PRIVATE rt)
    # CUDA driver API stubs — the foundation plugin calls cuInit at runtime.
    # --no-as-needed forces libcuda.so into NEEDED so the symbol is available
    # to plugins loaded via dlopen.  The stubs are only used to satisfy the
    # link-time symbol (via -L as a link option, NOT target_link_directories,
    # which would add stubs/ to BUILD_RPATH and cause the runtime loader to
    # pick the stub libcuda.so.1 instead of the real system CUDA driver).
    if(DEFINED CUDA_DIR)
        target_link_options(OvruntimeUnitTests PRIVATE "-L${CUDA_DIR}/lib64/stubs")
        target_link_libraries(OvruntimeUnitTests PRIVATE
            -Wl,--no-as-needed cuda -Wl,--as-needed
        )
    endif()
else()
    # CUDA driver API (cuInit etc. referenced transitively via physics test helpers)
    if(DEFINED CUDA_DIR)
        target_link_directories(OvruntimeUnitTests PRIVATE ${CUDA_DIR}/lib/x64)
        target_link_libraries(OvruntimeUnitTests PRIVATE cuda delayimp)
        target_link_options(OvruntimeUnitTests PRIVATE /DELAYLOAD:nvcuda.dll)
    endif()
endif()

# Precompiled header
target_precompile_headers(OvruntimeUnitTests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# RPATH configuration (Linux: bake library paths into binary; Windows: VS_DEBUGGER_ENVIRONMENT + CTest)
if(NOT WIN32)
    set(_UNIT_RPATH_DIRS "")
    if(DEFINED CARB_SDK_DIR)
        list(APPEND _UNIT_RPATH_DIRS "${_CARB_LIB_DIR}/release")
        if(EXISTS "${_CARB_LIB_DIR}/release/scripting-python-3.12")
            list(APPEND _UNIT_RPATH_DIRS "${_CARB_LIB_DIR}/release/scripting-python-3.12")
        endif()
    endif()
    if(DEFINED USD_DIR)
        list(APPEND _UNIT_RPATH_DIRS "${USD_DIR}/release/lib")
    endif()
    if(DEFINED PYTHON_DIR)
        list(APPEND _UNIT_RPATH_DIRS "${PYTHON_DIR}/lib")
    endif()
    if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        list(APPEND _UNIT_RPATH_DIRS "${USD_EXT_PHYSICS_DIR}/lib")
    endif()
    if(DEFINED OVRUNTIME_DEPS_DIR)
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.stats")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.fabric")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.cubric")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.usd")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/scenegraph")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/usdrt")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.ujitsoagent")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.ujitso.default")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/omni.blobkey")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/carb.datastore")
        list(APPEND _UNIT_RPATH_DIRS "${OVRUNTIME_DEPS_DIR}/_build/linux-x86_64/release/plugins/gpucompute")
    endif()

    set_target_properties(OvruntimeUnitTests PROPERTIES
        BUILD_RPATH "${_UNIT_RPATH_DIRS}"
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )
    target_link_options(OvruntimeUnitTests PRIVATE "LINKER:--disable-new-dtags")
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
    set_target_properties(OvruntimeUnitTests PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:OvruntimeUnitTests>"
        VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%${_WIN_PATH_DIRS}"
    )
endif()

# CTest integration
# Run from the exe dir so Carbonite resolves "." and relative search paths correctly.
add_test(NAME ovruntime.test.unit
    COMMAND OvruntimeUnitTests --reporters=junit --out=${PX_OUTPUT_BIN_DIR}/doctest_unit_results.xml
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:OvruntimeUnitTests>"
)

# Threading limits (match Kit CI defaults, OMPE-59303).
# Without these, USD/TBB create threads based on hardware_concurrency()
# which can be very high on CI runners, causing hangs.
set_tests_properties(ovruntime.test.unit PROPERTIES
    ENVIRONMENT "PXR_WORK_THREAD_LIMIT=16;OPENBLAS_NUM_THREADS=16"
)
if(WIN32)
    # Windows: prepend DLL dirs to PATH for ctest runs (ENVIRONMENT_MODIFICATION, CMake 3.22+).
    # Each entry must be a single path — path_list_prepend cannot contain semicolons.
    set(_UNIT_CTEST_ENV "")
    if(DEFINED CARB_SDK_DIR)
        list(APPEND _UNIT_CTEST_ENV "PATH=path_list_prepend:${CARB_SDK_DIR}/_build/windows-x86_64/release")
    endif()
    if(DEFINED USD_DIR)
        list(APPEND _UNIT_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/lib")
        list(APPEND _UNIT_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/bin")
    endif()
    if(DEFINED PYTHON_DIR)
        list(APPEND _UNIT_CTEST_ENV "PATH=path_list_prepend:${PYTHON_DIR}")
    endif()
    if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        list(APPEND _UNIT_CTEST_ENV "PATH=path_list_prepend:${USD_EXT_PHYSICS_DIR}/lib")
    endif()
    set_tests_properties(ovruntime.test.unit PROPERTIES
        ENVIRONMENT_MODIFICATION "${_UNIT_CTEST_ENV}"
    )
endif()
# Coverage builds (debug + instrumentation) are significantly slower
if(OVRUNTIME_ENABLE_COVERAGE)
    set_tests_properties(ovruntime.test.unit PROPERTIES TIMEOUT 1800)
else()
    set_tests_properties(ovruntime.test.unit PROPERTIES TIMEOUT 300)
endif()
