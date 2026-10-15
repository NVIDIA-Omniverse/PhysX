# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTRESULT-001
# @covers AC-1 AC-2 AC-3

# @implements REQ-CAPI-THREADS-001
# @covers AC-1 AC-2
#
# ovphysx C++ unit tests.
# Runs the GTest-based C++ unit tests against the installed SDK.
# Usage: cmake -P scripts/test_cpp.cmake
#
# The device mode (CPU vs GPU) and lifecycle refs are process-global. Tests run
# in separate processes so those locks and refs start clean for each pass:
#   Attach-time CUDA selection test (isolated process)         clean selector
#   Default GPU dynamics      (filter: DefaultGpuDynamicsTest) readback path
#   Remaining GPU tests       (filter: *GpuTest*)               GPU lock
#   Global lifecycle tests    (filter: GlobalLifecycle.*)       clean refs
#   Non-GPU tests             (filter: derived from isolate_from_cpu_pass) CPU lock
#
# Prerequisite: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/fetch_test_schemas.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")

if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR "_install directory does not exist.\nRun: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake")
endif()

if(NOT EXISTS "${INSTALL_PLUGINS_DIR}")
    message(FATAL_ERROR "_install/plugins directory does not exist.\nRun: cmake -P scripts/install.cmake")
endif()

# The test executables stay in _build. Only the SDK under test is installed.
set(GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/c_unittests${EXE_SUFFIX}")
set(SIDECAR_GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/sidecar_token_scope_unittests${EXE_SUFFIX}")

if(NOT EXISTS "${GTEST_EXECUTABLE}")
    message(FATAL_ERROR "C++ test executable not found at: ${GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()
if(NOT EXISTS "${SIDECAR_GTEST_EXECUTABLE}")
    message(FATAL_ERROR "Sidecar C++ test executable not found at: ${SIDECAR_GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()

# On Windows the DLLs must be on PATH before process launch, because the loader
# resolves them before main(). On Linux RPATH handles everything. Other env vars
# (PYTHONPATH) are set by test_main.cpp::bootstrap_test_environment() on both platforms.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    ovphysx_resolve_ovstage_paths()
    # OVStage's plugins dir carries the USD monolith, which the SDK does not
    # ship (REQ-PACKAGING-USDFREE-001 AC-4) and sidecar_token_scope_unittests imports
    # directly through ovphysx_internal.
    set(BASE_TEST_PATH "${INSTALL_BIN_DIR}${PATH_SEP}${INSTALL_PLUGINS_DIR}${PATH_SEP}${INSTALL_BIN_DEPS_DIR}${PATH_SEP}${TARGET_PYTHON_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}/plugins${PATH_SEP}$ENV{PATH}")
    set(ENV{PATH} "${BASE_TEST_PATH}")
    set(ENV{OVPHYSX_LIB} "${INSTALL_BIN_DIR}/ovphysx.dll")
endif()

message(STATUS "")
message(STATUS "=== C++ Unit Tests (using INSTALLED SDK) ===")
message(STATUS "  Project root: ${PROJECT_ROOT}")
message(STATUS "  Install dir: ${INSTALL_DIR}")
message(STATUS "  Plugins dir: ${INSTALL_PLUGINS_DIR}")
message(STATUS "  Test executable: ${GTEST_EXECUTABLE}")

# -------------------------------------------------------------------------
# Helper functions: run one GTest pass, print the tail, check the XML and exit code.
# -------------------------------------------------------------------------
set(TEST_RESULTS_DIR "${BUILD_PATH}/test_results")
file(MAKE_DIRECTORY "${TEST_RESULTS_DIR}")

# Passes with documented fully-skipped outcomes. A test can perform assertions
# before GTEST_SKIP; zero passed cases does not imply that no assertions ran.
# Listing an exception preserves the deliberate skip and warns that its full
# scenario remains unverified; unexpected fully-skipped passes are hard errors.
# Delete an entry the moment its blocker clears -- an entry outliving its reason
# is the silence this list exists to prevent.
#
#   cpu-no-cuda-context: the current OVStage limitation can cause a late skip
#   after useful CPU-mode assertions: ovstage_create_instance activates the
#   CUDA primary context before ovphysx receives the stage. The full CPU-only
#   no-driver-touch contract remains unverified. This test also deliberately
#   skips without a usable CUDA device; see test_cpu_no_cuda_context.cpp.
set(GTEST_PASSES_KNOWN_FULLY_SKIPPED "cpu-no-cuda-context")

function(parse_gtest_results _P_LABEL _P_XML)
    file(READ "${_P_XML}" _P_XML_CONTENT)

    foreach(_P_ATTR tests failures disabled skipped)
        string(REGEX MATCH "<testsuites[^>]*${_P_ATTR}=\"([0-9]+)\"" _P_MATCH "${_P_XML_CONTENT}")
        if(NOT _P_MATCH AND _P_ATTR STREQUAL "skipped")
            # GoogleTest reports skipped counts on each child suite, rather
            # than on its aggregate testsuites element. All children contribute:
            # using only the first can make an all-skipped pass look healthy.
            set(_P_skipped 0)
            string(REGEX MATCHALL "<testsuite[ \t\r\n][^>]*>" _P_SUITES "${_P_XML_CONTENT}")
            foreach(_P_SUITE IN LISTS _P_SUITES)
                string(REGEX MATCH "[ \t\r\n]skipped=\"([0-9]+)\"" _P_SUITE_SKIPPED "${_P_SUITE}")
                if(_P_SUITE_SKIPPED)
                    math(EXPR _P_skipped "${_P_skipped} + ${CMAKE_MATCH_1}")
                endif()
            endforeach()
            continue()
        endif()
        if(NOT _P_MATCH)
            string(REGEX MATCH "<testsuite[^>]*${_P_ATTR}=\"([0-9]+)\"" _P_MATCH "${_P_XML_CONTENT}")
        endif()
        if(_P_MATCH)
            string(REGEX REPLACE ".*${_P_ATTR}=\"([0-9]+)\".*" "\\1" _P_${_P_ATTR} "${_P_MATCH}")
        elseif(_P_ATTR STREQUAL "tests")
            message(FATAL_ERROR "GTest (${_P_LABEL}) results XML is malformed: missing '${_P_ATTR}' count")
        else()
            set(_P_${_P_ATTR} 0)
        endif()
    endforeach()

    if(_P_tests EQUAL 0)
        message(FATAL_ERROR "GTest (${_P_LABEL}) ran zero tests")
    endif()

    math(EXPR _P_passed "${_P_tests} - ${_P_failures} - ${_P_disabled} - ${_P_skipped}")
    if(_P_passed LESS 0)
        message(FATAL_ERROR "GTest (${_P_LABEL}) results XML has inconsistent counts")
    endif()

    message(STATUS
        "GTest ${_P_LABEL}: ${_P_tests} tests, ${_P_passed} passed, ${_P_skipped} skipped, "
        "${_P_disabled} disabled, ${_P_failures} failed")

    if(_P_failures GREATER 0)
        message(FATAL_ERROR "C++ unit tests had failures in ${_P_LABEL} pass")
    endif()

    # No case completed successfully. The zero-tests guard cannot see this:
    # cases were selected but skipped or disabled. Some assertions may have
    # run before a skip; the result does not validate the complete scenario.
    # Qualified full-driver passes must distinguish this from a healthy result,
    # while preserving the documented exceptions below.
    if(_P_passed EQUAL 0)
        # This isolated case intentionally self-skips without two visible CUDA
        # devices. Match its identity AND exact prerequisite diagnostic, not
        # just its pass label: unrelated skips must still fail closed.
        string(REGEX MATCHALL "<testcase[ >]" _P_CASES "${_P_XML_CONTENT}")
        list(LENGTH _P_CASES _P_CASE_COUNT)
        string(REGEX MATCH "<testcase[^>]*>" _P_CASE "${_P_XML_CONTENT}")
        if(_P_LABEL STREQUAL "cuda-selection-nonzero-ordinal"
           AND _P_tests EQUAL 1 AND _P_skipped EQUAL 1 AND _P_disabled EQUAL 0
           AND _P_CASE_COUNT EQUAL 1
           AND _P_CASE MATCHES " name=\"DirectGpuHostReadOnNonZeroOrdinal\""
           AND _P_CASE MATCHES " classname=\"ActiveCudaGpusAttachTest\""
           AND _P_XML_CONTENT MATCHES "<skipped message=\"[^\"]*:[0-9]+&#x0A;needs at least two CUDA devices to select a non-zero ordinal&#x0A;\"")
            message(WARNING
                "GTest (${_P_LABEL}) skipped for its hardware prerequisite: "
                "at least two visible CUDA devices are required. "
                "The non-zero-ordinal scenario was not exercised on this runner.")
            return()
        endif()
        # Solver-count coverage requires a tasking pool large enough for the
        # requested count (REQ-CAPI-THREADS-001 AC-1). Keep smaller-host skips
        # limited to the corresponding isolated case and exact diagnostic.
        if(_P_LABEL MATCHES "^solver-threads-([1248])$")
            set(_P_WORKERS "${CMAKE_MATCH_1}")
            if(_P_tests EQUAL 1 AND _P_skipped EQUAL 1 AND _P_disabled EQUAL 0
               AND _P_CASE_COUNT EQUAL 1
               AND _P_CASE MATCHES " name=\"Workers${_P_WORKERS}\""
               AND _P_CASE MATCHES " classname=\"SolverThreadCount\""
               AND _P_XML_CONTENT MATCHES "<skipped message=\"[^\"]*:[0-9]+&#x0A;Requested count exceeds this host&apos;s default tasking capacity&#x0A;\"")
                message(WARNING
                    "GTest (${_P_LABEL}) skipped for its hardware prerequisite: "
                    "the host's default tasking capacity is below ${_P_WORKERS}. "
                    "The ${_P_WORKERS}-worker scenario was not exercised on this runner.")
                return()
            endif()
        endif()
        list(FIND GTEST_PASSES_KNOWN_FULLY_SKIPPED "${_P_LABEL}" _P_KNOWN_HOLE)
        if(_P_KNOWN_HOLE EQUAL -1)
            message(FATAL_ERROR
                "GTest (${_P_LABEL}) passed 0 of ${_P_tests} tests "
                "(${_P_skipped} skipped, ${_P_disabled} disabled): "
                "no cases completed successfully. If that is genuinely expected, "
                "add "
                "'${_P_LABEL}' to GTEST_PASSES_KNOWN_FULLY_SKIPPED with the reason.")
        endif()
        message(WARNING
            "GTest (${_P_LABEL}) passed 0 of ${_P_tests} tests (${_P_skipped} "
            "skipped) - known fully-skipped exception, see "
            "GTEST_PASSES_KNOWN_FULLY_SKIPPED in scripts/test_cpp.cmake. "
            "The complete test scenario remains unverified.")
    endif()
endfunction()

# These passes fail when no CUDA driver is present. OVPHYSX_SKIP_GPU_PASSES
# leaves them unlaunched. cpu-no-cuda-context stays: it exits 0 without a driver.
set(OVPHYSX_GTEST_PASSES_REQUIRING_CUDA
    cuda-selection-attach
    cuda-selection-nonzero-ordinal
    default-gpu
    session-gpu-dynamics
    gpu
)
if(NOT DEFINED OVPHYSX_SKIP_GPU_PASSES AND DEFINED ENV{OVPHYSX_SKIP_GPU_PASSES})
    set(OVPHYSX_SKIP_GPU_PASSES "$ENV{OVPHYSX_SKIP_GPU_PASSES}")
endif()

function(run_gtest_pass _P_LABEL _P_FILTER)
    if(OVPHYSX_SKIP_GPU_PASSES AND "${_P_LABEL}" IN_LIST OVPHYSX_GTEST_PASSES_REQUIRING_CUDA)
        message(STATUS "Skipping GTest (${_P_LABEL}): no CUDA driver")
        return()
    endif()
    set(_P_EXECUTABLE "${GTEST_EXECUTABLE}")
    set(_P_TIMEOUT 300)
    if(ARGC GREATER 2)
        set(_P_EXECUTABLE "${ARGV2}")
    endif()
    if(ARGC GREATER 3)
        set(_P_TIMEOUT "${ARGV3}")
    endif()
    set(_P_XML "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}_results.xml")
    # One combined log with stdout and stderr interleaved, for easier triage.
    set(_P_LOG "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}.log")
    # GLOBAL so nested cmake function() calls still reach the leak scan.
    set_property(GLOBAL APPEND PROPERTY OVPHYSX_GTEST_PASS_LABELS "${_P_LABEL}")
    file(REMOVE "${_P_XML}" "${_P_LOG}")

    message(STATUS "")
    message(STATUS "--- ${_P_LABEL} pass (filter: ${_P_FILTER}) ---")

    set(_P_ENV)
    set(_P_ORIGINAL_PATH "$ENV{PATH}")
    if("${_P_LABEL}" STREQUAL "gpu" OR "${_P_LABEL}" STREQUAL "default-gpu"
       OR "${_P_LABEL}" STREQUAL "session-gpu-dynamics")
        list(APPEND _P_ENV "OVPHYSX_TEST_REQUIRE_CUDA=1")
    elseif("${_P_LABEL}" MATCHES "^lifecycle")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        list(APPEND _P_ENV "OVPHYSX_TEST_LIFECYCLE_OWNS_INIT=1")
    elseif("${_P_LABEL}" STREQUAL "cpu" OR "${_P_LABEL}" MATCHES "^(profiler-(trace|backend|bind-)|solver-threads-|omnipvd-)")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        if(OS_NAME STREQUAL "windows")
            set(ENV{PATH} "${BASE_TEST_PATH}")
        endif()
    elseif("${_P_LABEL}" STREQUAL "sidecar-token-scope")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        if(OS_NAME STREQUAL "windows")
            set(ENV{PATH} "${BASE_TEST_PATH}")
        endif()
    endif()
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_P_ENV} ${_P_EXECUTABLE}
                "--gtest_output=xml:${_P_XML}"
                "--gtest_filter=${_P_FILTER}"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT ${_P_TIMEOUT}
        RESULT_VARIABLE _P_RC
        OUTPUT_VARIABLE _P_STDOUT
        ERROR_VARIABLE  _P_STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )
    if(OS_NAME STREQUAL "windows")
        set(ENV{PATH} "${_P_ORIGINAL_PATH}")
    endif()

    file(WRITE "${_P_LOG}" "${_P_STDOUT}${_P_STDERR}")

    message(STATUS "Exit code (${_P_LABEL}): ${_P_RC}")

    # The tail is printed for quick CI triage. The full output is in the log file.
    if(EXISTS "${_P_LOG}")
        file(READ "${_P_LOG}" _P_LOG_ALL)
        string(LENGTH "${_P_LOG_ALL}" _P_LOG_LEN)
        math(EXPR _P_LOG_START "(${_P_LOG_LEN} - 8192)")
        if(_P_LOG_START LESS 0)
            set(_P_LOG_START 0)
        endif()
        string(SUBSTRING "${_P_LOG_ALL}" ${_P_LOG_START} -1 _P_LOG_TAIL)
        if(_P_LOG_TAIL)
            message(STATUS "GTest ${_P_LABEL} output tail (last ~8KB):\n${_P_LOG_TAIL}")
        endif()
    endif()

    if("${_P_RC}" MATCHES "[Tt]imeout")
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) timed out after ${_P_TIMEOUT}s; log: ${_P_LOG}")
    endif()

    # The XML proves the test process reached the result writer.
    if(EXISTS "${_P_XML}")
        parse_gtest_results("${_P_LABEL}" "${_P_XML}")
    else()
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) produced no XML output. "
            "The process likely crashed or failed to start (exit code: ${_P_RC})")
    endif()

    # A non-zero exit with valid XML points at a teardown failure.
    if(NOT "${_P_RC}" STREQUAL "0")
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) wrote valid XML but exited non-zero afterward. "
            "This may be a teardown crash (exit code: ${_P_RC})")
    endif()
endfunction()

# Register a gtest pattern the shared cpu pass must not run. Call this in the
# same comment block as the isolated run_gtest_pass that owns the pattern --
# not on the cpu filter line -- so two MRs adding suites land in disjoint hunks.
# Do not copy another pass's full filter here: negative sub-patterns would
# re-include those cases in cpu. Patterns that already match *GpuTest* do not
# need a second registration.
function(isolate_from_cpu_pass _PATTERN)
    get_property(_LATCHED GLOBAL PROPERTY OVPHYSX_CPU_PASS_LATCHED)
    if(_LATCHED)
        message(FATAL_ERROR
            "isolate_from_cpu_pass(\"${_PATTERN}\") must run before the cpu pass; "
            "a later call would leave the suite in both an isolated process and "
            "the shared cpu process")
    endif()
    if("${_PATTERN}" STREQUAL "")
        message(FATAL_ERROR "isolate_from_cpu_pass() requires a gtest pattern")
    endif()
    set_property(GLOBAL APPEND PROPERTY OVPHYSX_CPU_PASS_EXCLUSIONS "${_PATTERN}")
endfunction()

# Discover cases so new tests cannot be excluded from every pass. Each case
# needs its own process; the prefix selects initialization ownership.
function(run_isolated_gtest_cases _SUITE _PREFIX)
    execute_process(
        COMMAND "${GTEST_EXECUTABLE}" --gtest_list_tests "--gtest_filter=${_SUITE}.*"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 60
        RESULT_VARIABLE _LIST_RC
        OUTPUT_VARIABLE _LIST
        ERROR_VARIABLE _LIST_ERROR)
    if(NOT "${_LIST_RC}" STREQUAL "0")
        message(FATAL_ERROR "Cannot discover ${_SUITE} tests: ${_LIST_RC}: ${_LIST_ERROR}")
    endif()
    string(REGEX MATCHALL "\n  [A-Za-z_][A-Za-z0-9_]*" _CASES "${_LIST}")
    if(NOT _CASES)
        message(FATAL_ERROR "No ${_SUITE} tests discovered")
    endif()
    foreach(_CASE IN LISTS _CASES)
        string(STRIP "${_CASE}" _CASE)
        if(_CASE MATCHES "^DISABLED_")
            continue()
        endif()
        run_gtest_pass("${_PREFIX}-${_CASE}" "${_SUITE}.${_CASE}")
    endforeach()
endfunction()

# -------------------------------------------------------------------------
# Attach-time CUDA selection runs in a fresh process so the selector starts at
# automatic (-1) before the OVStage attach under test.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("ActiveCudaGpusAttachTest.*")
run_gtest_pass(
    "cuda-selection-attach"
    "ActiveCudaGpusAttachTest.*:-ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal")

# -------------------------------------------------------------------------
# The non-zero-ordinal DirectGPU read needs a process with no earlier GPU attach.
# PhysX latches its CUDA device at the first GPU attach in the process, so an
# attach on ordinal 0 pins the device and the later /physics/cudaDevice = 1 write
# does not move it. Sharing the pass above makes the case run on ordinal 0.
# Self-skips on single-GPU runners.
# -------------------------------------------------------------------------
run_gtest_pass(
    "cuda-selection-nonzero-ordinal"
    "ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal")

# -------------------------------------------------------------------------
# Default GPU dynamics without suppressReadback / DirectGPU. This is a fresh
# process because Carbonite settings and the selected simulation path are
# process-global.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("DefaultGpuDynamicsTest.*")
run_gtest_pass("default-gpu" "DefaultGpuDynamicsTest.*")

# -------------------------------------------------------------------------
# Session-API default GPU dynamics (ovphysx_read / ovphysx_write). Same fresh
# process as DefaultGpuDynamicsTest above: Carbonite settings and the selected
# simulation path are process-global.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("SessionGpuDynamicsTest.*")
run_gtest_pass("session-gpu-dynamics" "SessionGpuDynamicsTest.*")

# -------------------------------------------------------------------------
# Remaining GPU tests. The CPU-no-CUDA-context test also requires a fresh
# process and is routed to its own pass below. *GpuTest* also keeps
# CpuNoCudaContextGpuTest.* out of the shared cpu process.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("*GpuTest*")
run_gtest_pass("gpu" "*GpuTest*:-CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Global lifecycle tests run in their own process so runtime refs start clean.
# The callback-drain regressions get a separate short-timeout process. A broken
# drain or lock hoist would otherwise wedge callback and shutdown threads, so
# in-process cleanup cannot safely recover.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("GlobalLifecycle.*")
isolate_from_cpu_pass("HostExit.*")
run_gtest_pass("lifecycle-host-exit" "HostExit.*" "${GTEST_EXECUTABLE}" 60)
run_gtest_pass(
    "lifecycle"
    "GlobalLifecycle.*:-GlobalLifecycle.ShutdownDrainsAcceptedCallback:GlobalLifecycle.LiveInstanceShutdownDrainDoesNotHoldInstanceMapLock:GlobalLifecycle.PreInitLoggingConfigSurvivesLifecycleCycles")
# Logging configuration must be stored before the first Carbonite bootstrap,
# then survive explicit teardown without retaining the old application callback.
run_gtest_pass(
    "lifecycle-preinit-logging"
    "GlobalLifecycle.PreInitLoggingConfigSurvivesLifecycleCycles"
    "${GTEST_EXECUTABLE}"
    60)
run_gtest_pass(
    "lifecycle-lock-drain"
    "GlobalLifecycle.ShutdownDrainsAcceptedCallback:GlobalLifecycle.LiveInstanceShutdownDrainDoesNotHoldInstanceMapLock"
    "${GTEST_EXECUTABLE}"
    60)

# Provider creation is process-latched, so this exclusion regression needs a
# fresh process before any other test can create PxPhysics.
isolate_from_cpu_pass("OmniPvdColdCreation.*")
run_gtest_pass("omnipvd-cold" "OmniPvdColdCreation.*")

# Scoped recording capability is startup-only. Each case needs fresh pending
# settings and ownership of initialize/shutdown, including before first create.
isolate_from_cpu_pass("OmniPvdGlobalConfig.*")
run_isolated_gtest_cases("OmniPvdGlobalConfig" "lifecycle-omnipvd")

# Recording capability cannot change after the first successful create. Each
# recording case configures the modern flag before its first instance, so run
# it in a fresh process instead of sharing the CPU pass's frozen default.
isolate_from_cpu_pass("OmniPvdRecording.*")
run_isolated_gtest_cases("OmniPvdRecording" "omnipvd-recording")

# The chrome trace is written by the last-loader shutdown, so any other live
# instance in the process -- the shared CPU one every other case uses -- would
# defer the write past the end of the test. Own process, nothing else in it.
isolate_from_cpu_pass("ProfilerTraceBackend.*")
run_gtest_pass("profiler-trace" "ProfilerTraceBackend.*")

# The side-effect comparison needs one arm with nothing bound. A backend binds at
# most once per loader epoch and is released only by the last loader, so in the
# shared process an earlier case's binding would still be live and both arms would
# be profiled. Own process, nothing else in it.
isolate_from_cpu_pass("ProfilerSideEffects.*")
run_gtest_pass("profiler-side-effects" "ProfilerSideEffects.*")

# Carbonite locks the profiler interface on the first successful acquire, and the
# getter reports that bound backend (or none, when a host-owned profiler won).
# Each of these cases assumes it is the first bind in the process: one expects
# trace to win, the other expects NVTX. Earlier ProfilerBackend cases in the
# shared cpu pass acquire NVTX and would decide the result for both. They also
# poison each other, so they do not share a process.
isolate_from_cpu_pass("ProfilerBackend.CarbBackendBindsOnALaterInstance")
run_gtest_pass(
    "profiler-bind-later"
    "ProfilerBackend.CarbBackendBindsOnALaterInstance")
isolate_from_cpu_pass("ProfilerBackend.CarbBackendBindingIsIdempotent")
run_gtest_pass(
    "profiler-bind-idempotent"
    "ProfilerBackend.CarbBackendBindingIsIdempotent")

# The remaining ProfilerBackend cases read the bound backend. The shared cpu
# pass keeps an instance alive, so an NVTX bind from an earlier case in this
# suite stays bound and a later "none" or unknown OVPHYSX_PROFILER value still
# reads back as NVTX. Own process, with the two first-bind cases above left out
# because they decide each other's result.
isolate_from_cpu_pass("ProfilerBackend.*")
run_gtest_pass(
    "profiler-backend"
    "ProfilerBackend.*-ProfilerBackend.CarbBackendBindsOnALaterInstance:ProfilerBackend.CarbBackendBindingIsIdempotent")

# -------------------------------------------------------------------------
# Worker-pool sizing is decided once, when the first instance starts
# carb.tasking, and cannot be resized afterwards. Each case therefore needs a
# process where no earlier test has started the pool: the default case would
# otherwise observe another test's override, and the override case could not
# start the pool at all. The "lifecycle" label prefix also hands them
# OVPHYSX_TEST_LIFECYCLE_OWNS_INIT so the gtest environment does not initialize
# ovphysx before the case sets its own create-time configuration.
# -------------------------------------------------------------------------
isolate_from_cpu_pass("WorkerPoolDefaultTest.*")
run_gtest_pass("lifecycle-worker-pool-default" "WorkerPoolDefaultTest.*")
isolate_from_cpu_pass("WorkerPoolOverrideTest.*")
run_gtest_pass("lifecycle-worker-pool-override" "WorkerPoolOverrideTest.*")
isolate_from_cpu_pass("WorkerPoolInvalidTest.*")
run_gtest_pass("lifecycle-worker-pool-invalid" "WorkerPoolInvalidTest.*")

# -------------------------------------------------------------------------
# Each requested dispatcher count needs a fresh runtime and settings store.
isolate_from_cpu_pass("SolverThreadCount.*")
foreach(_THREAD_COUNT 1 2 4 8)
    run_gtest_pass("solver-threads-${_THREAD_COUNT}"
        "SolverThreadCount.Workers${_THREAD_COUNT}" "${GTEST_EXECUTABLE}" 60)
endforeach()

# -------------------------------------------------------------------------
# Non-GPU tests. Filter is every isolate_from_cpu_pass pattern, joined. Latch
# so a later isolate_from_cpu_pass cannot silently double-run a suite.
# -------------------------------------------------------------------------
get_property(_CPU_EXCLUSIONS GLOBAL PROPERTY OVPHYSX_CPU_PASS_EXCLUSIONS)
if(NOT _CPU_EXCLUSIONS)
    message(FATAL_ERROR "cpu pass has no isolate_from_cpu_pass() registrations")
endif()
list(JOIN _CPU_EXCLUSIONS ":" _CPU_FILTER)
set_property(GLOBAL PROPERTY OVPHYSX_CPU_PASS_LATCHED TRUE)
run_gtest_pass("cpu" "-${_CPU_FILTER}")

# -------------------------------------------------------------------------
# CPU-no-CUDA-context contract. On a GPU box, a cpu_only instance must not open
# a CUDA context. Runs in a fresh process so the primary-context probe starts
# from a clean state. Self-skips on CPU-only runners.
# -------------------------------------------------------------------------
run_gtest_pass("cpu-no-cuda-context" "CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Isolated checked token-scope sidecar bridge tests.
# -------------------------------------------------------------------------
run_gtest_pass("sidecar-token-scope" "*" "${SIDECAR_GTEST_EXECUTABLE}")

# -------------------------------------------------------------------------
# Check for simulation stage leaks across all passes.
# -------------------------------------------------------------------------
set(GTEST_ALL_OUTPUT "")
get_property(_LEAK_LABELS GLOBAL PROPERTY OVPHYSX_GTEST_PASS_LABELS)
if(NOT _LEAK_LABELS)
    message(FATAL_ERROR "no gtest pass labels recorded for the leak scan")
endif()
foreach(_LABEL IN LISTS _LEAK_LABELS)
    set(_LEAK_LOG "${TEST_RESULTS_DIR}/gtest_${_LABEL}.log")
    if(NOT EXISTS "${_LEAK_LOG}")
        message(FATAL_ERROR
            "leak scan: missing log for pass '${_LABEL}': ${_LEAK_LOG}")
    endif()
    file(READ "${_LEAK_LOG}" _LEAK_CONTENTS)
    string(APPEND GTEST_ALL_OUTPUT "${_LEAK_CONTENTS}")
endforeach()
if(GTEST_ALL_OUTPUT MATCHES "outstanding SimStageWithHistory")
    string(REGEX MATCH "had ([0-9]+) outstanding" LEAK_MATCH "${GTEST_ALL_OUTPUT}")
    if(CMAKE_MATCH_1)
        message(FATAL_ERROR "Simulation stage leak detected: ${CMAKE_MATCH_1} outstanding SimStageWithHistory(s) at shutdown")
    else()
        message(FATAL_ERROR "Simulation stage leak detected (see test output)")
    endif()
endif()

message(STATUS "")
message(STATUS "C++ unit tests: PASSED")
