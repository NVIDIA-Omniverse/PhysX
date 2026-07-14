# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Unit test sources ---
set(COOKING_UNIT_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CookingTestTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TestCookingService.cpp
    ${OVRUNTIME_ROOT_DIR}/source/omni.physx/tests/test.unit/physics/TestDeformableCookingUtility.cpp
)

# --- Create test executable ---
add_executable(OvruntimeCookingUnitTests
    ${COOKING_UNIT_TEST_SOURCES}
)

set_property(TARGET OvruntimeCookingUnitTests PROPERTY FOLDER "Tests")

set_target_properties(OvruntimeCookingUnitTests PROPERTIES
    OUTPUT_NAME "ovruntime.cooking.test.unit"
)

# RTTI and exceptions enabled
if(NOT WIN32)
    target_compile_options(OvruntimeCookingUnitTests PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
# Shared test helpers (TestHelpers.h, Tools.h) live in the omni.physx test tree
set(_PHYSX_TEST_UNIT_DIR ${OVRUNTIME_ROOT_DIR}/source/omni.physx/tests/test.unit)
set(_PHYSX_TEST_COMMON_DIR ${OVRUNTIME_ROOT_DIR}/source/omni.physx/tests/common)

target_include_directories(OvruntimeCookingUnitTests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}                    # CookingTestTools.h
        ${_PHYSX_TEST_UNIT_DIR}                        # common/TestHelpers.h
        ${_PHYSX_TEST_COMMON_DIR}                      # Tools.h
        ${OVRUNTIME_ROOT_DIR}/include                   # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private           # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch                # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include     # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeCookingUnitTests)

# Compile definitions
target_compile_definitions(OvruntimeCookingUnitTests PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# USD schema plugin path (physxSchema, physxSchemaAddition plugInfo.json registration)
# Windows: relative to exe dir (CMAKE_BINARY_DIR/<config>/) — portable across CI agents.
#   placeholder substitutes for the config subdir since file(RELATIVE_PATH) has no genex support.
# Linux: absolute paths are fine; build and test share the same filesystem layout.
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
    if(WIN32)
        file(RELATIVE_PATH _USD_EXT_REL "${CMAKE_BINARY_DIR}/placeholder"
            "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
        set(_USD_EXT_PLUGIN_PATH "${_USD_EXT_REL}")
    else()
        set(_USD_EXT_PLUGIN_PATH "${USD_EXT_PHYSICS_DIR}/share/usd/plugins")
    endif()
    target_compile_definitions(OvruntimeCookingUnitTests PRIVATE
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
        set(_CARB_PLUGIN_DIR "${_CARB_REL}/${OVRUNTIME_CARB_CONFIG}")
    else()
        set(_CARB_PLUGIN_DIR "${CARB_SDK_DIR}/_build/linux-x86_64/release")
    endif()
    target_compile_definitions(OvruntimeCookingUnitTests PRIVATE
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
    target_compile_definitions(OvruntimeCookingUnitTests PRIVATE
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
    )
endif()

# Link dependencies
target_link_libraries(OvruntimeCookingUnitTests
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimeCookingUnitTests PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimeCookingUnitTests PRIVATE carb)
endif()

# USD + TBB + Python libraries (handles both monolithic and non-monolithic USD)
ovruntime_link_usd_deps(OvruntimeCookingUnitTests)

# physicsSchemaTools + physxSchema libraries (for createMeshCapsule etc.)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimeCookingUnitTests PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimeCookingUnitTests PRIVATE physicsSchemaTools physxSchema)
endif()

# PhysX static libraries (test creates PxFoundation/PxPhysics directly for validation)
ovruntime_link_physx_static_libs(OvruntimeCookingUnitTests FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimeCookingUnitTests PRIVATE rt)
    # CUDA driver API stubs — the foundation plugin calls cuInit at runtime.
    # --no-as-needed forces libcuda.so into NEEDED so the symbol is available
    # to plugins loaded via dlopen.  The stubs are only used to satisfy the
    # link-time symbol (via -L as a link option, NOT target_link_directories,
    # which would add stubs/ to BUILD_RPATH and cause the runtime loader to
    # pick the stub libcuda.so.1 instead of the real system CUDA driver).
    if(DEFINED CUDA_DIR)
        target_link_options(OvruntimeCookingUnitTests PRIVATE "-L${CUDA_DIR}/lib64/stubs")
        target_link_libraries(OvruntimeCookingUnitTests PRIVATE
            -Wl,--no-as-needed cuda -Wl,--as-needed
        )
    endif()
endif()

# Precompiled header
target_precompile_headers(OvruntimeCookingUnitTests PRIVATE
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
    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
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
    endif()

    set_target_properties(OvruntimeCookingUnitTests PROPERTIES
        BUILD_RPATH "${_UNIT_RPATH_DIRS}"
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )
    target_link_options(OvruntimeCookingUnitTests PRIVATE "LINKER:--disable-new-dtags")
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
    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        set(_WIN_PATH_DIRS "${_WIN_PATH_DIRS};${USD_EXT_PHYSICS_DIR}/lib")
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
    endif()
    set_target_properties(OvruntimeCookingUnitTests PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:OvruntimeCookingUnitTests>"
        VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%${_WIN_PATH_DIRS}"
    )
endif()

# CTest integration
# Run from the exe dir so Carbonite resolves "." and relative search paths correctly.
add_test(NAME ovruntime.cooking.test.unit
    COMMAND OvruntimeCookingUnitTests --reporters=junit --out=${PX_OUTPUT_BIN_DIR}/doctest_cooking_results.xml
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:OvruntimeCookingUnitTests>"
)
if(WIN32)
    set(_COOKING_CTEST_ENV "")
    if(DEFINED CARB_SDK_DIR)
        list(APPEND _COOKING_CTEST_ENV "PATH=path_list_prepend:${CARB_SDK_DIR}/_build/windows-x86_64/release")
    endif()
    if(DEFINED USD_DIR)
        list(APPEND _COOKING_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/lib")
        list(APPEND _COOKING_CTEST_ENV "PATH=path_list_prepend:${USD_DIR}/release/bin")
    endif()
    if(DEFINED PYTHON_DIR)
        list(APPEND _COOKING_CTEST_ENV "PATH=path_list_prepend:${PYTHON_DIR}")
    endif()
    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        list(APPEND _COOKING_CTEST_ENV "PATH=path_list_prepend:${USD_EXT_PHYSICS_DIR}/lib")
    endif()
    set_tests_properties(ovruntime.cooking.test.unit PROPERTIES
        ENVIRONMENT_MODIFICATION "${_COOKING_CTEST_ENV}"
    )
endif()
# Coverage builds (debug + instrumentation) are significantly slower
if(OVRUNTIME_ENABLE_COVERAGE)
    set_tests_properties(ovruntime.cooking.test.unit PROPERTIES TIMEOUT 1800)
else()
    set_tests_properties(ovruntime.cooking.test.unit PROPERTIES TIMEOUT 600)
endif()
