# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)

# ============================================================================
# GPU Tests Option
# ============================================================================
# GPU tests are enabled by default. Without a CUDA driver the test binary still
# builds and runs, and the GPU tests skip at runtime.
#
# CI and release validation can require CUDA at runtime by setting:
#   OVPHYSX_TEST_REQUIRE_CUDA=1
option(OVPHYSX_ENABLE_GPU_TESTS "Enable GPU/CUDA tests (requires CUDA driver API)" ON)

if(OVPHYSX_ENABLE_GPU_TESTS)
    message(STATUS "GPU tests ENABLED (will skip at runtime if CUDA unavailable)")
else()
    message(STATUS "GPU tests DISABLED (compile-time)")
endif()

# Fetch GoogleTest automatically if not available
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ""
)
# Configure GoogleTest options before making it available
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)  # Don't install GoogleTest
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)    # Don't build GMock (not needed)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # Windows: Prevent CRT conflicts
FetchContent_MakeAvailable(googletest)

file(GLOB C_UNITTEST_SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "test_*.cpp")

add_executable(c_unittests ${C_UNITTEST_SOURCES})

# Link against gtest rather than gtest_main, because test_main.cpp provides main.
# USD and Python dependencies are loaded dynamically at runtime by the internal sidecar.
target_link_libraries(c_unittests
    PRIVATE
        gtest
        ovphysx
        ovphysx::ovstage
        # The DefaultAllocator regression test links the build-tree static allocator
        # directly. The OmniPVD cases remain the installed-SDK integration coverage.
        OvruntimeCommon
)

# Parse produced captures with the same OmniPVD reader that owns the format.
get_filename_component(_OVPHYSX_TEST_PHYSX_ROOT "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}" DIRECTORY)
target_include_directories(c_unittests PRIVATE "${_OVPHYSX_TEST_PHYSX_ROOT}/pvdruntime/include")
if(TARGET PVDRuntime)
    target_link_libraries(c_unittests PRIVATE PVDRuntime)
else()
    target_link_directories(c_unittests PRIVATE
        "${_OVPHYSX_TEST_PHYSX_ROOT}/bin/${PHYSX_PLATFORM_BIN}/$<IF:$<CONFIG:Debug>,debug,checked>")
    target_link_libraries(c_unittests PRIVATE PVDRuntime_static_64)
endif()
if(WIN32)
    target_link_libraries(c_unittests PRIVATE ws2_32)
endif()

# Tests consume ovphysx through the C API. ovphysx owns the static Carbonite framework.
target_compile_definitions(c_unittests PRIVATE CARB_STATIC_PLUGIN_BUILD=1)

# Feature-gated tests call GTEST_SKIP() inside #ifndef guards on the SDK/GPU
# capability defines. When a define is absent the body after GTEST_SKIP is
# unreachable, which MSVC reports as C4702 and the project-wide /WX promotes to
# an error. Linux and clang do not warn, so C4702 is silenced for the test TU only.
if(MSVC)
    target_compile_options(c_unittests PRIVATE /wd4702)
endif()

if(OVPHYSX_ENABLE_GPU_TESTS)
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_ENABLE_GPU_TESTS=1)
else()
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_ENABLE_GPU_TESTS=0)
endif()

# Whether the PhysXGpu module the tests load contains the DirectGPU debug-viz
# features (friction-anchor emission and the contact scope gate). Either the
# TARGET PhysXGpu exists because PhysXGpu is built from this tree's source (the
# prebuilt path links raw *_static_64 libs and creates no target), or the prebuilt
# physxsdk package is probed for a literal the feature embeds ("drawFrictionAnchors:",
# the error string in PxgGpuNarrowphaseCore::drawFrictionAnchors). The probe re-arms
# the hard assertions once the package pin contains the feature. Without it the
# documented skip would hide a regression in the shipped PhysXGpu indefinitely.
# Tests hard-assert when either holds and carry a documented skip otherwise.
set(_OVPHYSX_TEST_GPU_VIZ_CAPABLE FALSE)
if(TARGET PhysXGpu)
    set(_OVPHYSX_TEST_GPU_VIZ_CAPABLE TRUE)
else()
    # Probe the packman physx package, which is the module the tests load in both
    # non-source-GPU configurations. PHYSX_SDK_DIR is not usable as the base: on
    # prebuilt builds it is a directory-scope variable inside the ovruntime
    # subdirectory (not DEFINED here), and on devphysx it is CACHE-set to the physx
    # source tree, which has no bin/. The platform dir is globbed instead of using
    # PHYSX_PLATFORM_BIN, which has the same scope problem.
    if(WIN32)
        set(_physxgpu_probe_name "PhysXGpu_64.dll")
    else()
        set(_physxgpu_probe_name "libPhysXGpu_64.so")
    endif()
    file(GLOB _physxgpu_probe_candidates
        "${OVRUNTIME_TARGET_DEPS}/physx/bin/*/checked/${_physxgpu_probe_name}"
        "${OVRUNTIME_TARGET_DEPS}/physx/bin/*/debug/${_physxgpu_probe_name}"
        "${OVRUNTIME_TARGET_DEPS}/physx/bin/*/release/${_physxgpu_probe_name}")
    if(_physxgpu_probe_candidates)
        list(GET _physxgpu_probe_candidates 0 _physxgpu_probe)
        file(STRINGS "${_physxgpu_probe}" _physxgpu_viz_marker REGEX "drawFrictionAnchors:" LIMIT_COUNT 1)
        if(_physxgpu_viz_marker)
            set(_OVPHYSX_TEST_GPU_VIZ_CAPABLE TRUE)
        endif()
    endif()
endif()
if(_OVPHYSX_TEST_GPU_VIZ_CAPABLE)
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_TEST_PHYSXGPU_DEBUG_VIZ=1)
    message(STATUS "c_unittests: PhysXGpu provides the DirectGPU debug-viz features -- hard-asserting emission")
else()
    message(STATUS "c_unittests: prebuilt PhysXGpu predates the DirectGPU debug-viz features -- emission tests skip")
endif()

# The contact-scope rule is implemented in the CPU PhysX SDK. Source builds use
# this tree directly and must always assert it, even when PhysXGpu is unavailable.
# For prebuilt SDKs, the GPU marker above identifies packages built from the same
# PhysX change and re-arms the CPU assertion once that package pin rolls forward.
set(_OVPHYSX_TEST_CPU_SCOPE_VIZ_CAPABLE FALSE)
if(TARGET PhysX OR _OVPHYSX_TEST_GPU_VIZ_CAPABLE)
    set(_OVPHYSX_TEST_CPU_SCOPE_VIZ_CAPABLE TRUE)
endif()
if(_OVPHYSX_TEST_CPU_SCOPE_VIZ_CAPABLE)
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_TEST_PHYSXSDK_CONTACT_SCOPE_VIZ=1)
    message(STATUS "c_unittests: PhysX SDK provides contact-scope visualization -- hard-asserting scope")
else()
    message(STATUS "c_unittests: prebuilt PhysX SDK predates contact-scope visualization -- scope test skips")
endif()

if(NOT WIN32)
    target_link_libraries(c_unittests PRIVATE ${CMAKE_DL_LIBS})
    # c_unittests does not link USD/TBB (the internal sidecar loads them dynamically
    # at runtime), so the ovstage/USD shared libs it links carry transitively
    # undefined symbols such as tbb::detail::r1::*. Those are deferred to runtime
    # resolution, the same pattern OvruntimePhysX uses for CUDA. Stricter linkers
    # (notably aarch64 bfd ld) otherwise fail the link with "undefined reference to
    # tbb::detail::r1::..." from libovstage.so/libov_ms.so. The x86_64 linker
    # tolerates them by default.
    target_link_options(c_unittests PRIVATE -Wl,--allow-shlib-undefined)
    # Use old-style DT_RPATH (not DT_RUNPATH) so the test's RPATH also resolves the
    # transitive deps of the ovstage/USD libs it links. libovstage.so needs
    # libtbb.so.12, which install.cmake stages into _install/plugins/ (on the
    # BUILD_RPATH below). DT_RUNPATH does not propagate to a dependency's own
    # dependencies, so without this the test aborts at load time with
    # "libtbb.so.12: cannot open shared object file". Mirrors OvruntimeBenchmarks.
    target_link_options(c_unittests PRIVATE "LINKER:--disable-new-dtags")
endif()

# The top-level project selects this from the exact PhysX dependency configured
# for ovruntime and reuses it for SDK installation. Fail here too so a standalone
# test-target configuration reports the missing/mismatched dependency clearly.
if(NOT DEFINED _OVPHYSX_PHYSX_INCLUDE_SOURCE OR
   NOT EXISTS "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}/PxScene.h")
    message(FATAL_ERROR
        "c_unittests requires PxScene.h from the PhysX SDK selected by the "
        "ovphysx build, but _OVPHYSX_PHYSX_INCLUDE_SOURCE is missing or invalid: "
        "'${_OVPHYSX_PHYSX_INCLUDE_SOURCE}'")
endif()
set(_OVPHYSX_TEST_PHYSX_INCLUDE_DIR "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}")

target_include_directories(c_unittests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/include
        ${OMNI_PHYSICS_DIR}/include
        ${CARB_SDK_INCLUDE_DIR}
        ${_OVPHYSX_TEST_PHYSX_INCLUDE_DIR}
)

# Pointer-interop tests include PhysX declarations from the matching SDK above,
# but c_unittests does not link PhysX directly. PhysX objects and implementation
# remain owned by ovphysx. The target links ovstage for its public attachment API;
# neither ovphysx nor its internal sidecar loads USD or Python. USD is reached
# only through libovstage, which resolves its own monolith from OVStage's plugin
# directory (also on the test RPATH below).

# Add compile definitions for test data paths
# Relative paths assume cwd = PROJECT_ROOT (ovphysx/), enforced by test_cpp.cmake
# WORKING_DIRECTORY and by bootstrap_test_environment() error messages in test_main.cpp
target_compile_definitions(c_unittests
    PRIVATE
        OVPHYSX_SOURCE_DIR="."
        OVPHYSX_INSTALL_DIR="./_install"
)

ovphysx_apply_compiler_defaults(c_unittests)

# Set C++ standard to match main project
set_target_properties(c_unittests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# AddressSanitizer is disabled because it causes CUDA out-of-memory failures. Use Valgrind instead.
# if(NOT WIN32)
#     message(STATUS "AddressSanitizer ENABLED for c_unittests (DEBUG MODE)")
#     target_compile_options(c_unittests PRIVATE -fsanitize=address -fno-omit-frame-pointer -g)
#     target_link_options(c_unittests PRIVATE -fsanitize=address)
# endif()

# The RPATH points at the installed SDK in _install/, not the build tree, so the
# tests validate the shipped layout. Run cmake -P scripts/install.cmake first.
if(NOT WIN32)
    set(INSTALL_ROOT "${PROJECT_SOURCE_DIR}/_install")
    # The installed SDK ships no USD or TBB libraries (REQ-PACKAGING-USDFREE-001 AC-4);
    # those live in OVStage's plugin tree and are reached through libovstage's
    # own RUNPATH. The OVStage plugin directory stays on the test RPATH so a
    # test that resolves one of them directly still finds the same image.
    set(TEST_RPATH_DIRS
        "${INSTALL_ROOT}/lib"
        "${INSTALL_ROOT}/plugins"
        "${INSTALL_ROOT}/plugins/bin/deps"
        "${_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR}/plugins"
    )

    list(JOIN TEST_RPATH_DIRS ":" TEST_RPATH)
    
    set_target_properties(c_unittests PROPERTIES
        BUILD_RPATH "${TEST_RPATH}"
        INSTALL_RPATH "\$ORIGIN"
        SKIP_BUILD_RPATH FALSE
    )
    message(STATUS "Test RPATH set to _install/ (tests require install.cmake first)")
else()
    # Windows: DLLs must be on PATH before process launch (no RPATH equivalent).
    # test_cpp.cmake sets PATH. Standalone runs require the user to set it.
endif()

# Isolated sidecar tests inject a fake visualization accessor. Keep them
# out of c_unittests so no other test process can observe those process-global seams.
add_executable(sidecar_token_scope_unittests
    ${PROJECT_SOURCE_DIR}/tests/internal_unittests/test_token_scope_sidecar.cpp
)
target_link_libraries(sidecar_token_scope_unittests
    PRIVATE
        gtest_main
        ovphysx_internal
        ovphysx::ovstage
)
target_include_directories(sidecar_token_scope_unittests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/include
        ${OMNI_PHYSICS_DIR}/include
        ${CARB_SDK_INCLUDE_DIR}
)
target_include_directories(sidecar_token_scope_unittests SYSTEM PRIVATE
    ${OVSTAGE_INCLUDE_DIRS}
)
ovphysx_apply_compiler_defaults(sidecar_token_scope_unittests)
set_target_properties(sidecar_token_scope_unittests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
if(NOT WIN32)
    target_link_options(sidecar_token_scope_unittests PRIVATE
        -Wl,--allow-shlib-undefined
        "LINKER:--disable-new-dtags"
    )
    set_target_properties(sidecar_token_scope_unittests PROPERTIES
        BUILD_RPATH "${TEST_RPATH}"
        INSTALL_RPATH "\$ORIGIN"
        SKIP_BUILD_RPATH FALSE
    )
endif()

# ============================================================================
# C API Compatibility Test
# ============================================================================
# This test compiles as pure C (not C++) to validate C API compatibility. It
# catches C++ keywords (nullptr, delete, etc.) in the headers and missing
# typedefs for struct types. A compile failure here means the C API is broken.
#
# NOTE: Disabled on Windows because the MSVC C compiler does not handle the
# nested function pointer types. The Linux build validates C compatibility.

if(NOT WIN32)
    add_executable(c_api_compatibility_test
        test_c_api_compatibility.c
    )

    # The C standard (not C++) is what makes this test meaningful.
    set_target_properties(c_api_compatibility_test PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
    )
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(c_api_compatibility_test PRIVATE
            $<$<COMPILE_LANGUAGE:C>:-Wno-unused-but-set-variable>
            $<$<COMPILE_LANGUAGE:C>:-Wno-unused-variable>
        )
    endif()

    target_include_directories(c_api_compatibility_test
        PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/src/include
    )
    # The public ovphysx headers include <ovstage/...> and <ovx/...> directly. This
    # compile-only test does not link ovphysx, so it does not inherit the PUBLIC
    # ovstage include interface. The ovstage include dirs are added explicitly
    # (SYSTEM, matching how the ovphysx target consumes them).
    target_include_directories(c_api_compatibility_test SYSTEM PRIVATE
        ${OVSTAGE_INCLUDE_DIRS}
    )

    # Compile-only test, nothing is linked. It validates header syntax, not runtime behavior.

    # Always compiled as part of the ovphysx build.
    add_dependencies(ovphysx c_api_compatibility_test)

    message(STATUS "C API compatibility test enabled (compiles as pure C11)")
else()
    message(STATUS "C API compatibility test skipped on Windows (MSVC C compiler limitation - validated on Linux)")
endif()
