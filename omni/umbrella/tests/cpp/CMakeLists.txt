# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# C++ unit tests for umbrella library

# Require Carbonite SDK for headers
if(NOT DEFINED CARB_SDK_DIR)
    message(WARNING "Carbonite SDK not available - skipping C++ tests")
    return()
endif()

# SimulationRegistry source (compiled directly into test binaries)
set(SIMULATION_REGISTRY_SRC
    ${UMBRELLA_ROOT_DIR}/source/SimulationRegistry.cpp
)

# Find doctest from packman
set(DOCTEST_INCLUDE_DIR "${UMBRELLA_ROOT_DIR}/_build/target-deps/doctest/include")

if(NOT EXISTS "${DOCTEST_INCLUDE_DIR}/doctest/doctest.h")
    message(WARNING "doctest not found at ${DOCTEST_INCLUDE_DIR}")
    message(WARNING "Run: ./pull_dependencies.bat (or .sh) to fetch doctest via packman")
    message(WARNING "Skipping doctest-based tests")
    message(STATUS "Building basic test only")

    # Build basic test without doctest
    add_executable(test_umbrella_basic
        test_umbrella_basic.cpp
        ${SIMULATION_REGISTRY_SRC}
    )

    target_include_directories(test_umbrella_basic
        PRIVATE ${UMBRELLA_ROOT_DIR}/include ${UMBRELLA_ROOT_DIR}/source
    )
    target_include_directories(test_umbrella_basic SYSTEM
        PRIVATE ${CARB_SDK_DIR}/include
    )

    # Tests require C++ exceptions (try/catch in test harness)
    if(MSVC)
        target_compile_options(test_umbrella_basic PRIVATE /EHsc)
        target_compile_definitions(test_umbrella_basic PRIVATE _HAS_EXCEPTIONS=1)
    endif()

    # RPATH / PATH for carb runtime (Linux: bake into binary; Windows: VS debugger + CTest)
    if(NOT WIN32)
        set_target_properties(test_umbrella_basic PROPERTIES
            BUILD_RPATH "${CARB_SDK_DIR}/_build/linux-x86_64/release"
        )
        target_link_options(test_umbrella_basic PRIVATE "LINKER:--disable-new-dtags")
    else()
        set_target_properties(test_umbrella_basic PROPERTIES
            VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%;${CARB_SDK_DIR}/_build/windows-x86_64/$<LOWER_CASE:$<CONFIG>>"
        )
    endif()

    add_test(NAME UmbrellaBasicTest COMMAND test_umbrella_basic)
    if(WIN32)
        set_tests_properties(UmbrellaBasicTest PROPERTIES
            ENVIRONMENT_MODIFICATION
                "PATH=path_list_prepend:${CARB_SDK_DIR}/_build/windows-x86_64/release"
        )
    endif()

    return()
endif()

message(STATUS "Found doctest at ${DOCTEST_INCLUDE_DIR}")

# Build comprehensive test suite with doctest
add_executable(test_umbrella_suite
    doctest_main.cpp
    TestSimulationInterface.cpp
    TestSimulationRegistry.cpp
    TestSimulatorSimulation.cpp
    TestSimulatorStageUpdate.cpp
    TestSimulatorSceneQuery.cpp
    TestSimulatorInteraction.cpp
    TestSimulatorBenchmark.cpp
    TestRegistryEvents.cpp
    TestMultiSimulationSubscriptions.cpp
    ${SIMULATION_REGISTRY_SRC}
)

target_include_directories(test_umbrella_suite
    PRIVATE ${UMBRELLA_ROOT_DIR}/include ${UMBRELLA_ROOT_DIR}/source
)
target_include_directories(test_umbrella_suite SYSTEM
    PRIVATE ${CARB_SDK_DIR}/include ${DOCTEST_INCLUDE_DIR}
)

# Build basic test (no doctest dependency)
add_executable(test_umbrella_basic
    test_umbrella_basic.cpp
    ${SIMULATION_REGISTRY_SRC}
)

target_include_directories(test_umbrella_basic
    PRIVATE ${UMBRELLA_ROOT_DIR}/include ${UMBRELLA_ROOT_DIR}/source
)
target_include_directories(test_umbrella_basic SYSTEM
    PRIVATE ${CARB_SDK_DIR}/include
)

# Tests require C++ exceptions (doctest REQUIRE uses exceptions, test_basic has try/catch)
if(MSVC)
    target_compile_options(test_umbrella_suite PRIVATE /EHsc)
    target_compile_definitions(test_umbrella_suite PRIVATE _HAS_EXCEPTIONS=1)
    target_compile_options(test_umbrella_basic PRIVATE /EHsc)
    target_compile_definitions(test_umbrella_basic PRIVATE _HAS_EXCEPTIONS=1)
endif()

# RPATH / PATH for carb runtime (Linux: bake into binary; Windows: VS debugger + CTest)
if(NOT WIN32)
    set(_UMBRELLA_TEST_RPATH "${CARB_SDK_DIR}/_build/linux-x86_64/release")
    foreach(_tgt IN ITEMS test_umbrella_suite test_umbrella_basic)
        set_target_properties(${_tgt} PROPERTIES
            BUILD_RPATH "${_UMBRELLA_TEST_RPATH}"
        )
        target_link_options(${_tgt} PRIVATE "LINKER:--disable-new-dtags")
    endforeach()
else()
    foreach(_tgt IN ITEMS test_umbrella_suite test_umbrella_basic)
        set_target_properties(${_tgt} PROPERTIES
            VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%;${CARB_SDK_DIR}/_build/windows-x86_64/$<LOWER_CASE:$<CONFIG>>"
        )
    endforeach()
endif()

# Add tests
add_test(NAME UmbrellaBasicTest COMMAND test_umbrella_basic)
add_test(NAME UmbrellaSuiteTest COMMAND test_umbrella_suite --reporters=junit --out=${PX_OUTPUT_BIN_DIR}/doctest_results.xml)
if(WIN32)
    set(_UMBRELLA_CTEST_ENV
        "PATH=path_list_prepend:${CARB_SDK_DIR}/_build/windows-x86_64/release"
    )
    set_tests_properties(UmbrellaBasicTest UmbrellaSuiteTest PROPERTIES
        ENVIRONMENT_MODIFICATION "${_UMBRELLA_CTEST_ENV}"
    )
endif()
