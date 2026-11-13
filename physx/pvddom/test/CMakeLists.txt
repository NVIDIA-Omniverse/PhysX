## Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
## Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0



if(PVDDOM_BUILD_TOOLS AND CMAKE_VERSION VERSION_LESS 3.21)
    message(FATAL_ERROR "Installed-tool tests require CMake 3.21; disable PVDDOM_BUILD_TESTS or PVDDOM_BUILD_TOOLS")
endif()

if(MSVC)
    add_compile_options(/EHsc)
endif()

include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Exercise the SDK's Linux exception policy in the standalone runtime tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(PVDRuntime PRIVATE -fno-exceptions)
endif()
set(PVDDOM_TEST_SOURCES test_pvddom.cpp test_reader.cpp test_reader_diagnostics.cpp)
if(PVDDOM_BUILD_INSPECTION)
    list(APPEND PVDDOM_TEST_SOURCES test_document.cpp test_inspection.cpp test_numeric_inspection.cpp)
endif()
add_executable(pvddom_tests ${PVDDOM_TEST_SOURCES})
if(PVDDOM_BUILD_INSPECTION)
    target_include_directories(pvddom_tests PRIVATE ../../include ../src)
endif()
target_link_libraries(pvddom_tests PRIVATE PvdDom gtest_main)
target_compile_definitions(pvddom_tests PRIVATE PVDDOM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

if(CMAKE_VERSION VERSION_LESS 3.18)
    add_test(NAME pvddom_tests COMMAND pvddom_tests)
else()
    include(GoogleTest)
    gtest_discover_tests(pvddom_tests DISCOVERY_MODE PRE_TEST)
endif()

if(PVDDOM_BUILD_INSPECTION)
    add_executable(pvddom_allocation_failures test_allocation_failures.cpp)
    target_link_libraries(pvddom_allocation_failures PRIVATE PvdDom)
    target_compile_definitions(pvddom_allocation_failures PRIVATE PVDDOM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
    add_test(NAME pvddom_allocation_failures COMMAND pvddom_allocation_failures)
endif()
if(PVDDOM_BUILD_TOOLS)
    add_executable(ovd_annotation_file_tests test_annotations.cpp)
    add_test(NAME ovd_annotation_file_failures COMMAND ovd_annotation_file_tests "${CMAKE_CURRENT_BINARY_DIR}")

    if(UNIX)
        add_executable(ovd_new_file_tests test_new_file.cpp)
        add_test(NAME ovd_new_file_permissions COMMAND ovd_new_file_tests "${CMAKE_CURRENT_BINARY_DIR}")
    elseif(WIN32)
        add_executable(ovd_new_file_windows_tests test_new_file_windows.cpp)
        add_test(NAME ovd_new_file_windows COMMAND ovd_new_file_windows_tests)
    endif()
endif()
add_executable(pvddom_reader_allocation_failures test_reader_allocation_failures.cpp)
target_link_libraries(pvddom_reader_allocation_failures PRIVATE PvdDom)
add_test(NAME pvddom_reader_allocation_failures COMMAND pvddom_reader_allocation_failures)

# OVD file generator tool
add_executable(generate_test_ovd generate_test_ovd.cpp)
target_link_libraries(generate_test_ovd PRIVATE PvdDom)
set(PVDDOM_GENERATED_TEST_FILE "${CMAKE_CURRENT_BINARY_DIR}/generated_scene.ovd")
add_custom_command(OUTPUT "${PVDDOM_GENERATED_TEST_FILE}"
    COMMAND $<TARGET_FILE:generate_test_ovd> "${PVDDOM_GENERATED_TEST_FILE}"
    DEPENDS generate_test_ovd
    VERBATIM)
add_custom_target(generate_test_scene DEPENDS "${PVDDOM_GENERATED_TEST_FILE}")
add_dependencies(pvddom_tests generate_test_scene)
target_compile_definitions(pvddom_tests PRIVATE PVDDOM_GENERATED_TEST_FILE="${PVDDOM_GENERATED_TEST_FILE}")

if(PVDDOM_BUILD_INSPECTION)
    set(PVDDOM_NUMERIC_TEST_FILE "${CMAKE_CURRENT_BINARY_DIR}/numeric_values.ovd")
    add_custom_command(OUTPUT "${PVDDOM_NUMERIC_TEST_FILE}"
        COMMAND $<TARGET_FILE:generate_test_ovd> "${PVDDOM_NUMERIC_TEST_FILE}" --numeric-values
        DEPENDS generate_test_ovd
        VERBATIM)
    add_custom_target(generate_numeric_test_scene DEPENDS "${PVDDOM_NUMERIC_TEST_FILE}")
    add_dependencies(pvddom_tests generate_numeric_test_scene)
    target_compile_definitions(pvddom_tests PRIVATE PVDDOM_NUMERIC_TEST_FILE="${PVDDOM_NUMERIC_TEST_FILE}")
endif()

# Standalone OVD command-stream decoder (diagnostic tool)
add_executable(dump_ovd dump_ovd.cpp)
target_link_libraries(dump_ovd PRIVATE PvdDom)

if(PVDDOM_BUILD_TOOLS)
    add_test(NAME ovd_cli_installed COMMAND ${CMAKE_COMMAND}
        -DBUILD=${CMAKE_BINARY_DIR} -DCONFIG=$<CONFIG>
        -DFIXTURE=${PVDDOM_GENERATED_TEST_FILE}
        -DGENERATOR_TOOL=$<TARGET_FILE:generate_test_ovd>
        -DREQUESTED_DOM_VERSION=${_pvddom_sdk_version}
        -DSCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/cli.cmake
        -DCONSUMER=${CMAKE_CURRENT_SOURCE_DIR}/consumer
        -DGENERATOR=${CMAKE_GENERATOR}
        "-DCXX_FLAGS=${CMAKE_CXX_FLAGS}" "-DLINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/install_cli.cmake)
endif()
