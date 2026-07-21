# ovphysx benchmark suite (opt-in)
#
# Built only when OVPHYSX_BUILD_BENCHMARKS=ON. The harness and framework files
# under framework/ are imported from omni.physx/tests/test.benchmarks/ to keep
# the diff trivial when re-syncing. The benchmarks under benchmarks/ are
# ovphysx-specific.
#
# Linkage mirrors the c_unittests pattern: link ovphysx (the public library)
# plus static Carbonite for the framework files' logging/file helpers, and
# point RPATH at _install/ so the binary runs against the installed SDK.

cmake_minimum_required(VERSION 3.16)

set(OVPHYSX_BENCHMARK_FRAMEWORK_SRCS
    framework/BmTime.cpp
    framework/BmOutput.cpp
    framework/BmGlobals.cpp
)

set(OVPHYSX_BENCHMARK_HARNESS_SRCS
    Harness.cpp
    BenchmarkList.cpp
)

set(OVPHYSX_BENCHMARK_SUITE_SRCS
    benchmarks/Smoke.cpp
    benchmarks/UsdLoad.cpp
    benchmarks/StepCpu.cpp
    benchmarks/StepGpu.cpp
    benchmarks/Clone.cpp
    benchmarks/TensorBindings.cpp
    benchmarks/LowLoad.cpp
    benchmarks/LabCartpole.cpp
    benchmarks/LabAnymal.cpp
)

add_executable(ovphysx_benchmarks
    ${OVPHYSX_BENCHMARK_HARNESS_SRCS}
    ${OVPHYSX_BENCHMARK_FRAMEWORK_SRCS}
    ${OVPHYSX_BENCHMARK_SUITE_SRCS}
)

target_link_libraries(ovphysx_benchmarks PRIVATE ovphysx ovphysx::ovstage)

# Framework files use carb defines (CARB_PLATFORM_*) and carb::filesystem
# directly. The benchmark executable links static Carbonite for its own
# logging/file helpers.
target_include_directories(ovphysx_benchmarks BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(ovphysx_benchmarks PRIVATE CARB_STATIC_PLUGIN_BUILD=1)
target_link_libraries(ovphysx_benchmarks PRIVATE ${CARB_STATIC_LIBRARY})
if(WIN32)
    target_link_libraries(ovphysx_benchmarks PRIVATE shlwapi pathcch winmm)
    target_link_libraries(ovphysx_benchmarks PRIVATE "${OVPHYSX_TBB_LIBRARY}")
    target_compile_options(ovphysx_benchmarks PRIVATE /wd4211)
endif()

target_include_directories(ovphysx_benchmarks
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
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
        "${INSTALL_ROOT}/plugins/gpu"
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

# Copy config.toml next to the binary, matching the c_unittests pattern so the
# benchmark binary can be launched standalone.
add_custom_command(TARGET ovphysx_benchmarks POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_BINARY_DIR}/config.toml"
        "$<TARGET_FILE_DIR:ovphysx_benchmarks>/config.toml"
    COMMENT "Copying config.toml next to ovphysx_benchmarks"
)
