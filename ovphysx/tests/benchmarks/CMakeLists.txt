# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx benchmark suite (opt-in)
#
# @implements REQ-CAPI-BENCHMARK-002
# @covers AC-1 AC-2 AC-3 AC-4
#
# @implements REQ-CAPI-BENCHMARK-003
# @covers AC-1 AC-2
#
# @implements REQ-CAPI-BENCHMARK-004
# @covers AC-1 AC-2 AC-5
#
# Built only when OVPHYSX_BUILD_BENCHMARKS=ON. The harness and framework files
# under framework/ are imported from omni.physx/tests/test.benchmarks/ to keep
# the diff trivial when re-syncing. The benchmarks under benchmarks/ are
# ovphysx-specific.
#
# Linkage mirrors the c_unittests pattern. It links ovphysx (the public library)
# plus static Carbonite for the framework files' logging/file helpers, and
# points RPATH at _install/ so the binary runs against the installed SDK.
#
# tests/ holds the stdlib-only Python inventory contract for the hidden L1B
# (Authoring/WriteScaling) row set plus focused C++ failure-matching coverage.
# The inventory tests run without a build via
# `python3 -m unittest discover -s tests/benchmarks/tests -p "test_*.py"`, and
# are driven against the built executable by
# ../../scripts/test_benchmark_contract.cmake (registered as the
# `benchmark-contract` CTest under the `benchmarks` label).

cmake_minimum_required(VERSION 3.16)

set(OVPHYSX_BENCHMARK_FRAMEWORK_SRCS
    framework/BmTime.cpp
    framework/BmOutput.cpp
    framework/BmGlobals.cpp
)

set(OVPHYSX_BENCHMARK_HARNESS_SRCS
    Harness.cpp
    BenchmarkFailure.cpp
    BenchmarkList.cpp
    TimingDiagnostics.cpp
)

set(OVPHYSX_BENCHMARK_SUITE_SRCS
    benchmarks/Smoke.cpp
    benchmarks/UsdLoad.cpp
    benchmarks/InstancingAttach.cpp
    benchmarks/StepCpu.cpp
    benchmarks/StepGpu.cpp
    benchmarks/Clone.cpp
    benchmarks/OutputRead.cpp
    benchmarks/OutputWrite.cpp
    benchmarks/TensorBindings.cpp
    benchmarks/LowLoad.cpp
    benchmarks/LabCartpole.cpp
    benchmarks/LabAnymal.cpp
    benchmarks/AuthoringPopulationAdd.cpp
    benchmarks/AuthoringTeleport.cpp
    benchmarks/AuthoringDrain.cpp
    benchmarks/AuthoringPopulationChurn.cpp
    benchmarks/AuthoringPopulationScale.cpp
    benchmarks/AuthoringRuntimeWriteSpawn.cpp
    benchmarks/WriteScaling.cpp
    benchmarks/ContactReport.cpp
)

add_executable(ovphysx_benchmarks
    ${OVPHYSX_BENCHMARK_HARNESS_SRCS}
    ${OVPHYSX_BENCHMARK_FRAMEWORK_SRCS}
    ${OVPHYSX_BENCHMARK_SUITE_SRCS}
)

add_executable(benchmark_failure_tests
    tests/BenchmarkFailureTest.cpp
    BenchmarkFailure.cpp
)
target_link_libraries(benchmark_failure_tests PRIVATE ovphysx ovphysx::ovstage)
target_include_directories(benchmark_failure_tests BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(benchmark_failure_tests PRIVATE CARB_STATIC_PLUGIN_BUILD=1)
ovphysx_apply_compiler_defaults(benchmark_failure_tests)
set_target_properties(benchmark_failure_tests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
add_test(NAME benchmark-failure-unit COMMAND benchmark_failure_tests)
set_tests_properties(benchmark-failure-unit PROPERTIES
    LABELS "benchmark-unit;benchmarks"
)

add_executable(benchmark_timing_diagnostics_tests
    tests/TimingDiagnosticsTest.cpp
    TimingDiagnostics.cpp
)
ovphysx_apply_compiler_defaults(benchmark_timing_diagnostics_tests)
set_target_properties(benchmark_timing_diagnostics_tests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
add_test(NAME benchmark-timing-diagnostics-unit
    COMMAND benchmark_timing_diagnostics_tests
        "$<TARGET_FILE_DIR:benchmark_timing_diagnostics_tests>/benchmark_timing_diagnostics_unit.jsonl"
)
set_tests_properties(benchmark-timing-diagnostics-unit PROPERTIES
    LABELS "benchmark-unit;benchmarks"
)

target_link_libraries(ovphysx_benchmarks PRIVATE ovphysx ovphysx::ovstage)

if(TARGET ovphysx_benchmark_cuda)
    message(STATUS "ovphysx Cartpole benchmark: native CUDA tensors enabled")
    target_compile_definitions(ovphysx_benchmarks PRIVATE OVPHYSX_BENCHMARK_HAS_CUDA=1)
    target_link_libraries(ovphysx_benchmarks PRIVATE ovphysx_benchmark_cuda)
else()
    message(STATUS "ovphysx Cartpole benchmark: native CUDA tensors disabled explicitly")
endif()

# Framework files use carb defines (CARB_PLATFORM_*) and carb::filesystem
# directly. The benchmark executable links static Carbonite for its own
# logging/file helpers.
target_include_directories(ovphysx_benchmarks BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(ovphysx_benchmarks PRIVATE CARB_STATIC_PLUGIN_BUILD=1)
target_link_libraries(ovphysx_benchmarks PRIVATE ${CARB_STATIC_LIBRARY})
if(WIN32)
    target_link_libraries(ovphysx_benchmarks PRIVATE shlwapi pathcch winmm)
    target_compile_options(ovphysx_benchmarks PRIVATE /wd4211)
endif()

target_include_directories(ovphysx_benchmarks
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/ovruntime/source/common/include
        ${CARB_SDK_INCLUDE_DIR}
)

if(NOT WIN32)
    target_link_libraries(ovphysx_benchmarks PRIVATE ${CMAKE_DL_LIBS})
endif()

ovphysx_apply_compiler_defaults(ovphysx_benchmarks)

set_target_properties(ovphysx_benchmarks PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Run against the installed SDK (same recipe as c_unittests). cmake -P
# scripts/install.cmake must be run before invoking the benchmark binary.
if(NOT WIN32)
    set(INSTALL_ROOT "${PROJECT_SOURCE_DIR}/_install")
    set(BENCHMARK_RPATH_DIRS
        "${INSTALL_ROOT}/lib"
        "${INSTALL_ROOT}/plugins"
        "${INSTALL_ROOT}/plugins/bin/deps"
    )

    set(PYTHON_LIB_DIR "${PROJECT_SOURCE_DIR}/_build/target-deps/python/lib")
    if(EXISTS "${PYTHON_LIB_DIR}")
        list(APPEND BENCHMARK_RPATH_DIRS "${PYTHON_LIB_DIR}")
    endif()

    list(JOIN BENCHMARK_RPATH_DIRS ":" BENCHMARK_RPATH)

    set_target_properties(ovphysx_benchmarks PROPERTIES
        BUILD_RPATH "${BENCHMARK_RPATH}"
        INSTALL_RPATH "\$ORIGIN"
        SKIP_BUILD_RPATH FALSE
    )
endif()
