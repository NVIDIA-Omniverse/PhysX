cmake_minimum_required(VERSION 3.16)

# ============================================================================
# GPU Tests Option
# ============================================================================
# GPU tests are ENABLED by default.
# Developers without a CUDA driver/GPU can still build and run the test binary; GPU
# tests will be skipped at runtime when CUDA is unavailable.
#
# CI/release validation can additionally enforce CUDA availability at runtime by setting:
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

# Create the test executable
add_executable(c_unittests ${C_UNITTEST_SOURCES})

# Link against gtest (not gtest_main - we provide our own main) and ovphysx
# Note: USD/Python dependencies are handled by the internal sidecar which loads dynamically at runtime
target_link_libraries(c_unittests
    PRIVATE
        gtest
        ovphysx
        ovphysx::ovstage
)

# Tests consume ovphysx through the C API; ovphysx owns the static Carbonite framework.
target_compile_definitions(c_unittests PRIVATE CARB_STATIC_PLUGIN_BUILD=1)

# Feature-gated tests skip via GTEST_SKIP() inside #ifndef guards on the SDK/GPU
# capability defines; when a define is absent the body after the (return-like)
# GTEST_SKIP is unreachable, which MSVC flags as C4702 and the project-wide /WX
# promotes to an error. The skip-then-dead-code is intentional and documented
# per test, and Linux/clang do not warn, so silence C4702 for the test TU only.
if(MSVC)
    target_compile_options(c_unittests PRIVATE /wd4702)
endif()

if(OVPHYSX_ENABLE_GPU_TESTS)
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_ENABLE_GPU_TESTS=1)
else()
    target_compile_definitions(c_unittests PRIVATE OVPHYSX_ENABLE_GPU_TESTS=0)
endif()

# Whether the PhysXGpu module the tests will load contains the DirectGPU debug-viz
# features added by this tree (friction-anchor emission + the contact scope gate).
# Two ways to have them:
#   - PhysXGpu built from THIS tree's source (devphysx with CUDA): TARGET PhysXGpu
#     exists (the prebuilt path links raw *_static_64 libs and creates no target).
#   - A prebuilt physxsdk package NEW ENOUGH to contain them: probe the packaged
#     binary for a literal the feature embeds ("drawFrictionAnchors:", the error
#     string in PxgGpuNarrowphaseCore::drawFrictionAnchors). This re-arms the
#     hard assertions automatically once the package pin rolls past the merge --
#     without it, the tests' documented skip would hide a future trunk regression
#     in the shipped PhysXGpu forever (review).
# Tests hard-assert when either holds and carry a documented skip otherwise.
set(_OVPHYSX_TEST_GPU_VIZ_CAPABLE FALSE)
if(TARGET PhysXGpu)
    set(_OVPHYSX_TEST_GPU_VIZ_CAPABLE TRUE)
else()
    # Probe the packman physx package -- the module the tests actually load in both
    # non-source-GPU configurations. PHYSX_SDK_DIR is the WRONG base for this
    # (review): on prebuilt builds it is a directory-scope variable inside the
    # ovruntime subdirectory (not DEFINED here), and on devphysx it is CACHE-set to
    # the physx SOURCE tree, which has no bin/. Glob the platform dir instead of
    # requiring PHYSX_PLATFORM_BIN, which has the same scope problem.
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
    # c_unittests deliberately does NOT link USD/TBB (the internal sidecar loads
    # them dynamically at runtime), so the ovstage/USD shared libs it links carry
    # transitively-undefined symbols (e.g. tbb::detail::r1::*). Defer those to
    # runtime resolution — the same pattern OvruntimePhysX uses for CUDA. Without
    # this, stricter linkers (notably aarch64 bfd ld) fail the link with
    # "undefined reference to tbb::detail::r1::..." from libovstage.so/libov_ms.so;
    # x86_64's linker already tolerates them by default.
    target_link_options(c_unittests PRIVATE -Wl,--allow-shlib-undefined)
    # Use old-style DT_RPATH (not DT_RUNPATH) so the test's RPATH resolves the
    # TRANSITIVE deps of the ovstage/USD libs it links. libovstage.so needs
    # libtbb.so.12, which install.cmake stages into _install/plugins/ (on the
    # BUILD_RPATH below); DT_RUNPATH does not propagate to a dependency's own
    # dependencies, so without this the test aborts at load time with
    # "libtbb.so.12: cannot open shared object file". Mirrors OvruntimeBenchmarks.
    target_link_options(c_unittests PRIVATE "LINKER:--disable-new-dtags")
endif()

# Set include directories
target_include_directories(c_unittests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/include
        ${OMNI_PHYSICS_DIR}/include
        ${CARB_SDK_INCLUDE_DIR}
)

# Note: The test binary only links ovphysx. USD/Python symbols are loaded
# dynamically by the internal sidecar at runtime, so we avoid linking USD here.
# Python libraries are added to RPATH below so the plugin can resolve them.

# Add compile definitions for test data paths
# Relative paths assume cwd = PROJECT_ROOT (ovphysx/), enforced by test_cpp.cmake
# WORKING_DIRECTORY and by bootstrap_test_environment() error messages in test_main.cpp
target_compile_definitions(c_unittests
    PRIVATE
        OVPHYSX_SOURCE_DIR="."
        OVPHYSX_INSTALL_DIR="./_install"
)

ovphysx_apply_compiler_defaults(c_unittests)
ovphysx_apply_windows_tbb_config(c_unittests)

# Set C++ standard to match main project
set_target_properties(c_unittests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# AddressSanitizer DISABLED - use Valgrind instead (ASAN causes CUDA out of memory)
# if(NOT WIN32)
#     message(STATUS "AddressSanitizer ENABLED for c_unittests (DEBUG MODE)")
#     target_compile_options(c_unittests PRIVATE -fsanitize=address -fno-omit-frame-pointer -g)
#     target_link_options(c_unittests PRIVATE -fsanitize=address)
# endif()

# Copy config.toml to test executable directory for runtime access
add_custom_command(TARGET c_unittests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_BINARY_DIR}/config.toml"
        "$<TARGET_FILE_DIR:c_unittests>/config.toml"
    COMMENT "Copying config.toml to test directory"
)

# Set RPATH to run tests against INSTALLED SDK (not build tree)
# Tests validate the actual shipped layout in _install/
# Requires: cmake -P scripts/install.cmake before running tests
if(NOT WIN32)
    # Linux: Point RPATH directly at _install/ directories
    set(INSTALL_ROOT "${PROJECT_SOURCE_DIR}/_install")
    set(TEST_RPATH_DIRS
        "${INSTALL_ROOT}/lib"
        "${INSTALL_ROOT}/plugins"
        "${INSTALL_ROOT}/plugins/gpu"
        "${INSTALL_ROOT}/plugins/bin/deps"
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
    # test_cpp.cmake sets PATH; standalone runs require the user to set it.
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
    ${USD_TDEP_INCLUDE_DIR}
    ${OVSTAGE_INCLUDE_DIRS}
)
ovphysx_apply_compiler_defaults(sidecar_token_scope_unittests)
ovphysx_apply_windows_tbb_config(sidecar_token_scope_unittests)
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
# This test MUST compile as pure C (not C++) to validate C API compatibility.
# It ensures that headers don't contain C++ keywords (nullptr, delete, etc.)
# and that typedefs are properly defined for struct types.
#
# If this test fails to compile, the C API is broken and needs fixing.
#
# NOTE: Disabled on Windows due to MSVC C compiler limitations with complex
# nested function pointer types. Linux build validates true C compatibility.

if(NOT WIN32)
    add_executable(c_api_compatibility_test
        test_c_api_compatibility.c
    )

    # CRITICAL: Set C standard (not C++) to ensure true C compatibility
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

    # Include directories for C test
    target_include_directories(c_api_compatibility_test
        PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/src/include
    )
    # The public ovphysx headers #include <ovstage/...>/<ovx/...> directly. This
    # compile-only test does not link ovphysx, so it does not inherit ovphysx's
    # PUBLIC ovstage include interface — add the ovstage include dirs explicitly
    # (SYSTEM, matching how the ovphysx target consumes them) so the shipped
    # public headers resolve at build time.
    target_include_directories(c_api_compatibility_test SYSTEM PRIVATE
        ${OVSTAGE_INCLUDE_DIRS}
    )

    # No linking required - this is a compile-only test
    # The test validates header syntax, not runtime behavior

    # Add as a build target (always compiled)
    add_dependencies(ovphysx c_api_compatibility_test)

    message(STATUS "C API compatibility test enabled (compiles as pure C11)")
else()
    message(STATUS "C API compatibility test skipped on Windows (MSVC C compiler limitation - validated on Linux)")
endif()
