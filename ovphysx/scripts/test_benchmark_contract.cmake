# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4 AC-5
#
# @implements REQ-CAPI-BENCHMARK-002
# @covers AC-1 AC-2 AC-3
#
# @implements REQ-CAPI-BENCHMARK-003
# @covers AC-1 AC-2
#
# @implements REQ-CAPI-BENCHMARK-004
# @covers AC-1 AC-4 AC-5
#
# @implements REQ-CAPI-BENCHMARK-005
# @covers AC-1 AC-2 AC-3 AC-4
#
# ovphysx benchmark producer contract.
#
# This is a producer contract, not a performance run. It does not compare
# timings against a baseline.
#
# The high-scale section at the end covers the separate five-row hidden
# inventory (tests/benchmarks/tests/high_scale_inventory.json): a
# collision-free union with the L1B and ContactReport inventories, the exact
# hidden --list, vacuity without --hidden, and one positive run that publishes
# exactly the five rows. That run is OPT-IN under
# OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1, because a 16,384-body scene takes
# minutes to populate. Step 1's unchanged L1B list check doubles as the proof
# that those rows stay outside the frozen `Authoring.*:WriteScaling.*` selection.
#
# Steps 1-9 cover the L1B (Authoring / WriteScaling) frozen inventory: that the
# executable's registered row set matches tests/benchmarks/tests/
# producer_inventory.json exactly, that every frozen row is hidden (absent from
# default wildcard execution), that the inventory's CPU subset runs and
# publishes numbers, that a row pointed at missing data fails closed with no
# published metric, that the same family filter without --hidden selects nothing
# at all, and that a device-gated row the harness selects but never executes
# publishes no record.
#
# Step 10 covers OutputRead, which is a performance family rather than a frozen
# inventory and so is gated on the producer contract alone. A CPU row exits
# cleanly and publishes (step 10), and a row pointed at missing data publishes
# nothing and exits non-zero (step 10b). Step 10c applies the same clean-exit
# contract to a DirectGPU row and is OPT-IN. It runs only under
# OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU=1, which the benchmark CI job exports, so
# it gates in CI and is skipped by a plain local run even on a GPU host.
# Step 10d adds the one thing behaviour cannot check: that the registered row
# set is exactly outputread_inventory.json and that every row there names an
# owner, so the family cannot grow anonymously.
#
# The regenerating steps wrap the shared developer baseline at
# <app dir>/../data/benchmarkData/_baseline.txt in a crash-recoverable
# transaction (scripts/benchmark_baseline_txn.cmake), because --regenerate
# rewrites it with only the rows the narrowly filtered step selected.
#
# tests/benchmarks/tests/verify_inventory.py (stdlib-only) does the parsing
# and comparison. This script only drives the executable and the row counts and
# filters it reads back from producer_inventory.json, so the total/CPU row
# counts are never hard-coded here. producer_inventory.json is the source of truth.
#
# Usage:
#   cmake -P scripts/test_benchmark_contract.cmake
#
# Prerequisite:
#   ./build.sh --benchmarks && \
#   cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# The family filter is fixed. It is the scope of L1B, not a per-task-derived
# value. producer_inventory.json's row set (read below) is what varies by
# task, and that is what drives every count and per-device filter used here.
set(L1B_FAMILY_FILTER "Authoring.*:WriteScaling.*")

set(INVENTORY_TESTS_DIR "${PROJECT_ROOT}/tests/benchmarks/tests")
set(VERIFY_INVENTORY_PY "${INVENTORY_TESTS_DIR}/verify_inventory.py")
set(VERIFY_CONTACT_INVENTORY_PY "${INVENTORY_TESTS_DIR}/verify_contact_report_inventory.py")
set(VERIFY_DIAGNOSTICS_PY "${INVENTORY_TESTS_DIR}/verify_benchmark_diagnostics.py")
set(PRODUCER_INVENTORY_JSON "${INVENTORY_TESTS_DIR}/producer_inventory.json")
set(OUTPUT_READ_INVENTORY_JSON "${INVENTORY_TESTS_DIR}/outputread_inventory.json")
set(CONTACT_INVENTORY_JSON "${INVENTORY_TESTS_DIR}/contact_report_inventory.json")
set(CONTACT_FAMILY_FILTER "ContactReport.*")
set(HIGH_SCALE_INVENTORY_JSON "${INVENTORY_TESTS_DIR}/high_scale_inventory.json")
# Two families on purpose. Neither matches the frozen L1B family filter above.
set(HIGH_SCALE_FAMILY_FILTER "WriteScalingHighN.*:RuntimeSpawnScaling.*")
set(CARTPOLE_SOURCE "${PROJECT_ROOT}/tests/benchmarks/benchmarks/LabCartpole.cpp")
set(HARNESS_SOURCE "${PROJECT_ROOT}/tests/benchmarks/Harness.cpp")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(BENCHMARK_BINARY "${STANDARD_OUTPUT_DIR}/ovphysx_benchmarks${EXE_SUFFIX}")
set(RESULTS_DIR "${PROJECT_ROOT}/_build/benchmark_contract_results")
file(MAKE_DIRECTORY "${RESULTS_DIR}")

if(NOT EXISTS "${VERIFY_INVENTORY_PY}")
    message(FATAL_ERROR "Inventory parser not found: ${VERIFY_INVENTORY_PY}")
endif()
if(NOT EXISTS "${VERIFY_CONTACT_INVENTORY_PY}")
    message(FATAL_ERROR "ContactReport inventory parser not found: ${VERIFY_CONTACT_INVENTORY_PY}")
endif()
if(NOT EXISTS "${VERIFY_DIAGNOSTICS_PY}")
    message(FATAL_ERROR "Timing diagnostics verifier not found: ${VERIFY_DIAGNOSTICS_PY}")
endif()
if(NOT EXISTS "${PRODUCER_INVENTORY_JSON}")
    message(FATAL_ERROR "Inventory fixture not found: ${PRODUCER_INVENTORY_JSON}")
endif()
if(NOT EXISTS "${CONTACT_INVENTORY_JSON}")
    message(FATAL_ERROR "ContactReport inventory fixture not found: ${CONTACT_INVENTORY_JSON}")
endif()
if(NOT EXISTS "${HIGH_SCALE_INVENTORY_JSON}")
    message(FATAL_ERROR "High-scale inventory fixture not found: ${HIGH_SCALE_INVENTORY_JSON}")
endif()
if(NOT EXISTS "${CARTPOLE_SOURCE}")
    message(FATAL_ERROR "Cartpole benchmark source not found: ${CARTPOLE_SOURCE}")
endif()
if(NOT EXISTS "${HARNESS_SOURCE}")
    message(FATAL_ERROR "Benchmark harness source not found: ${HARNESS_SOURCE}")
endif()
if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR
        "_install/ not found.\n"
        "Run: cmake -DOVPHYSX_BUILD_BENCHMARKS=ON -P scripts/build.cmake && cmake -P scripts/install.cmake")
endif()
if(NOT EXISTS "${BENCHMARK_BINARY}")
    message(FATAL_ERROR
        "ovphysx_benchmarks not built at: ${BENCHMARK_BINARY}\n"
        "Configure with -DOVPHYSX_BUILD_BENCHMARKS=ON and rebuild.")
endif()

# On Windows the DLLs must be on PATH. On Linux RPATH handles it. Same recipe
# as test_benchmarks_cpp.cmake.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    ovphysx_resolve_ovstage_paths()
    # ovstage.dll statically imports the USD monolith, which lives in OVStage's
    # plugins/ dir and which the SDK does not ship (REQ-PACKAGING-USDFREE-001 AC-4).
    # Linux resolves it via ovstage's RUNPATH ($ORIGIN/plugins). Windows needs
    # both dirs on PATH, after the _install entries so installed DLLs still win.
    set(ENV{PATH} "${INSTALL_BIN_DIR}${PATH_SEP}${INSTALL_PLUGINS_DIR}${PATH_SEP}${INSTALL_BIN_DEPS_DIR}${PATH_SEP}${TARGET_PYTHON_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}/plugins${PATH_SEP}$ENV{PATH}")
    set(ENV{OVPHYSX_LIB} "${INSTALL_BIN_DIR}/ovphysx.dll")
endif()

# ---------------------------------------------------------------------------
# Helper: invoke verify_inventory.py and fail the whole contract with its
# diagnostic on a non-zero exit. Captures stdout in VERIFY_STDOUT (parent
# scope) for callers that need to read a value back (counts, filter).
# ---------------------------------------------------------------------------
function(run_verify_inventory)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${VERIFY_INVENTORY_PY}" ${ARGN}
        OUTPUT_VARIABLE _OUT
        ERROR_VARIABLE _ERR
        RESULT_VARIABLE _RC
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _RC EQUAL 0)
        message(FATAL_ERROR "verify_inventory.py ${ARGN} failed:\n${_OUT}\n${_ERR}")
    endif()
    if(_OUT)
        message(STATUS "verify_inventory.py ${ARGN}: ${_OUT}")
    endif()
    set(VERIFY_STDOUT "${_OUT}" PARENT_SCOPE)
endfunction()

# Invoke the strict two-row ContactReport inventory and suppression verifier.
function(run_verify_contact_inventory)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${VERIFY_CONTACT_INVENTORY_PY}" ${ARGN}
        OUTPUT_VARIABLE _OUT ERROR_VARIABLE _ERR RESULT_VARIABLE _RC
    )
    if(NOT _RC EQUAL 0)
        message(FATAL_ERROR "verify_contact_report_inventory.py ${ARGN} failed:\n${_OUT}\n${_ERR}")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Helper: invoke the REQ-CAPI-BENCHMARK-002 artifact/source verifier.
# ---------------------------------------------------------------------------
function(run_verify_diagnostics)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${VERIFY_DIAGNOSTICS_PY}" ${ARGN}
        OUTPUT_VARIABLE _OUT
        ERROR_VARIABLE _ERR
        RESULT_VARIABLE _RC
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _RC EQUAL 0)
        message(FATAL_ERROR "verify_benchmark_diagnostics.py ${ARGN} failed:\n${_OUT}\n${_ERR}")
    endif()
    if(_OUT)
        message(STATUS "verify_benchmark_diagnostics.py ${ARGN}: ${_OUT}")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Read the row counts and per-device filters from producer_inventory.json.
# Nothing below hard-codes the inventory counts.
# ---------------------------------------------------------------------------
run_verify_inventory(counts "${PRODUCER_INVENTORY_JSON}")
set(_COUNTS_OUTPUT "${VERIFY_STDOUT}")
string(REGEX MATCH "TOTAL=([0-9]+)" _ "${_COUNTS_OUTPUT}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "Could not parse TOTAL= from verify_inventory.py counts output:\n${_COUNTS_OUTPUT}")
endif()
set(INVENTORY_TOTAL "${CMAKE_MATCH_1}")
string(REGEX MATCH "CPU=([0-9]+)" _ "${_COUNTS_OUTPUT}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "Could not parse CPU= from verify_inventory.py counts output:\n${_COUNTS_OUTPUT}")
endif()
set(INVENTORY_CPU "${CMAKE_MATCH_1}")

run_verify_inventory(filter "${PRODUCER_INVENTORY_JSON}" --device cpu)
set(CPU_ROW_FILTER "${VERIFY_STDOUT}")
if(NOT CPU_ROW_FILTER)
    message(FATAL_ERROR "Inventory produced an empty CPU filter")
endif()

run_verify_inventory(filter "${PRODUCER_INVENTORY_JSON}" --device gpu)
set(GPU_ROW_FILTER "${VERIFY_STDOUT}")
if(NOT GPU_ROW_FILTER)
    message(FATAL_ERROR "Inventory produced an empty GPU filter")
endif()
string(REPLACE ":" ";" GPU_ROW_NAMES "${GPU_ROW_FILTER}")

# Keep contact rows separate and collision-free against the frozen L1B inventory.
run_verify_contact_inventory(
    check-definition "${CONTACT_INVENTORY_JSON}" "${PRODUCER_INVENTORY_JSON}"
)
set(CONTACT_CPU_ROW "ContactReport.persistent_pairs_512_step_read_cpu")
set(CONTACT_GPU_ROW "ContactReport.persistent_pairs_512_step_read_gpu")
set(CONTACT_CPU_COMMAND "${CONTACT_CPU_ROW}")
set(CONTACT_GPU_COMMAND "${CONTACT_GPU_ROW}_GPU")

# ---------------------------------------------------------------------------
# --regenerate rewrites the shared developer baseline at
# <app dir>/../data/benchmarkData/_baseline.txt (BmOutput's destructor), and it
# writes only the rows the run selected. Every regenerating step below is
# scoped to a narrow filter, so leaving that write in place would truncate a
# developer's broad baseline, or create a narrow one where none existed, and
# silently disable baseline comparison for every row this contract does not
# run.
#
# benchmark_baseline_txn.cmake wraps each regenerating step in a crash-
# recoverable transaction. The "did it exist?" answer lives on disk, so a run
# killed while the child owns the narrow baseline stays recoverable and the
# next invocation recovers it before it snapshots anything. commit() also
# checks that what came back is byte-identical to what was found.
# scripts/test_benchmark_baseline_txn.cmake covers the protocol against a
# scratch directory. Step 0 below runs it.
# ---------------------------------------------------------------------------
include("${SCRIPT_DIR}/benchmark_baseline_txn.cmake")

get_filename_component(DEVELOPER_BASELINE "${STANDARD_OUTPUT_DIR}/../data/benchmarkData/_baseline.txt" ABSOLUTE)
set(BASELINE_TXN_TARGET "${DEVELOPER_BASELINE}")
set(BASELINE_TXN_BACKUP "${RESULTS_DIR}/_baseline.developer.bak")
set(BASELINE_TXN_MARKER "${RESULTS_DIR}/_baseline.txn")

message(STATUS "")
message(STATUS "=== ovphysx benchmark contract (hidden Authoring/WriteScaling inventory) ===")
message(STATUS "  Project root:     ${PROJECT_ROOT}")
message(STATUS "  Binary:           ${BENCHMARK_BINARY}")
message(STATUS "  Results:          ${RESULTS_DIR}")
message(STATUS "  Family filter:    ${L1B_FAMILY_FILTER}")
message(STATUS "  Inventory total:  ${INVENTORY_TOTAL}")
message(STATUS "  Inventory CPU:    ${INVENTORY_CPU}")
message(STATUS "  CPU row filter:   ${CPU_ROW_FILTER}")

# ---------------------------------------------------------------------------
# Step 0: prove the baseline transaction protocol still holds, then complete
# any transaction a previous run left open. Both have to happen before the
# first --regenerate step, because that is the first thing that can destroy a
# developer baseline.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 0: baseline transaction protocol and recovery ---")
execute_process(
    COMMAND "${CMAKE_COMMAND}" -P "${SCRIPT_DIR}/test_benchmark_baseline_txn.cmake"
    TIMEOUT 120
    RESULT_VARIABLE _TXN_RC
    OUTPUT_VARIABLE _TXN_STDOUT
    ERROR_VARIABLE _TXN_STDERR
)
file(WRITE "${RESULTS_DIR}/baseline_txn.log" "${_TXN_STDOUT}${_TXN_STDERR}")
if(NOT _TXN_RC EQUAL 0)
    message(FATAL_ERROR
        "baseline transaction protocol regression failed (exit ${_TXN_RC}):\n${_TXN_STDOUT}\n${_TXN_STDERR}")
endif()
message(STATUS "Baseline transaction protocol: verified (log: ${RESULTS_DIR}/baseline_txn.log)")

baseline_txn_recover()

# ---------------------------------------------------------------------------
# Step 1: the scoped --list must name exactly the inventory's rows, and every
# one of them must be marked hidden. --list always passes showHidden=true
# internally (see Harness.cpp), so this works without --hidden.
# ---------------------------------------------------------------------------
set(LIST_OUTPUT_FILE "${RESULTS_DIR}/list_output.txt")
file(REMOVE "${LIST_OUTPUT_FILE}")
message(STATUS "")
message(STATUS "--- step 1: scoped --list vs producer_inventory.json ---")
execute_process(
    COMMAND "${BENCHMARK_BINARY}" "--list" "--filter=${L1B_FAMILY_FILTER}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 120
    RESULT_VARIABLE _LIST_RC
    OUTPUT_VARIABLE _LIST_STDOUT
    ERROR_VARIABLE _LIST_STDERR
)
file(WRITE "${LIST_OUTPUT_FILE}" "${_LIST_STDOUT}")
message(STATUS "Exit code (list): ${_LIST_RC}")
if(NOT _LIST_RC EQUAL 0)
    message(FATAL_ERROR "ovphysx_benchmarks --list failed (exit ${_LIST_RC}):\n${_LIST_STDOUT}\n${_LIST_STDERR}")
endif()

run_verify_inventory(check-list "${PRODUCER_INVENTORY_JSON}" "${LIST_OUTPUT_FILE}")

# ---------------------------------------------------------------------------
# Step 1b: the binding-creation row is a separate hidden L1 probe rather than
# part of the frozen L1B producer inventory. This pins its executable inventory
# name and hidden marker, then statically pins the GPU/non-GPU one-shot defaults
# and the guard that rejects a second timed call. The five-binding timer itself
# is exercised separately on a CUDA-capable DirectGPU host.
# ---------------------------------------------------------------------------
set(CONTROL_ROW "Probe.cartpole_4096_control_step")
set(BINDING_ROW "Probe.cartpole_4096_tensor_binding_create")
set(BINDING_LIST_OUTPUT "${RESULTS_DIR}/binding_list_output.txt")
file(REMOVE "${BINDING_LIST_OUTPUT}")
message(STATUS "")
message(STATUS "--- step 1b: hidden one-shot Cartpole binding row contract ---")
execute_process(
    COMMAND "${BENCHMARK_BINARY}" "--list" "--filter=${BINDING_ROW}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 120
    RESULT_VARIABLE _BINDING_LIST_RC
    OUTPUT_VARIABLE _BINDING_LIST_STDOUT
    ERROR_VARIABLE _BINDING_LIST_STDERR
)
file(WRITE "${BINDING_LIST_OUTPUT}" "${_BINDING_LIST_STDOUT}")
if(NOT _BINDING_LIST_RC EQUAL 0)
    message(FATAL_ERROR
        "binding-row --list failed (exit ${_BINDING_LIST_RC}):\n${_BINDING_LIST_STDOUT}\n${_BINDING_LIST_STDERR}")
endif()
run_verify_diagnostics(check-binding-list "${BINDING_LIST_OUTPUT}")
run_verify_diagnostics(check-binding-source "${CARTPOLE_SOURCE}" "${HARNESS_SOURCE}")

# ---------------------------------------------------------------------------
# Step 2: run the inventory's CPU subset with --hidden --regenerate. The
# positive run must produce exactly the expected positive rows, with no missing
# row and no duplicate.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 2: CPU subset positive run (--hidden --regenerate) ---")
set(POSITIVE_REPORT "${RESULTS_DIR}/positive_cpu_report.txt")
file(REMOVE "${POSITIVE_REPORT}")
baseline_txn_begin()
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--hidden"
        "--regenerate"
        "--filter=${CPU_ROW_FILTER}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${POSITIVE_REPORT}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 1800
    RESULT_VARIABLE _POS_RC
    OUTPUT_VARIABLE _POS_STDOUT
    ERROR_VARIABLE _POS_STDERR
)
baseline_txn_commit()
file(WRITE "${RESULTS_DIR}/positive_cpu.log" "${_POS_STDOUT}${_POS_STDERR}")
message(STATUS "Exit code (positive CPU subset): ${_POS_RC}")
if(NOT _POS_RC EQUAL 0)
    message(FATAL_ERROR "positive CPU subset run failed (exit ${_POS_RC}); log: ${RESULTS_DIR}/positive_cpu.log")
endif()

run_verify_inventory(check-report "${PRODUCER_INVENTORY_JSON}" "${POSITIVE_REPORT}" --device cpu)

# ---------------------------------------------------------------------------
# Step 3: the primary KPI row against a definitely missing data directory
# must fail closed, with a non-zero exit and no successful metric published for
# that row (bmRecordFailure() suppresses its emit()).
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 3: primary KPI row against missing data directory (negative) ---")
set(MISSING_DATA_DIR "${PROJECT_ROOT}/_build/benchmark_contract_results/definitely_missing_data_dir")
if(EXISTS "${MISSING_DATA_DIR}")
    file(REMOVE_RECURSE "${MISSING_DATA_DIR}")
endif()
set(NEGATIVE_REPORT "${RESULTS_DIR}/negative_report.txt")
set(NEGATIVE_DIAGNOSTICS "${RESULTS_DIR}/negative_diagnostics.jsonl")
file(REMOVE "${NEGATIVE_REPORT}")
file(REMOVE "${NEGATIVE_DIAGNOSTICS}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--hidden"
        "--filter=Authoring.population_add_drip_cpu"
        "--data=${MISSING_DATA_DIR}"
        "--report=${NEGATIVE_REPORT}"
        "--timing-diagnostics=${NEGATIVE_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _NEG_RC
    OUTPUT_VARIABLE _NEG_STDOUT
    ERROR_VARIABLE _NEG_STDERR
)
file(WRITE "${RESULTS_DIR}/negative.log" "${_NEG_STDOUT}${_NEG_STDERR}")
message(STATUS "Exit code (negative data-dir run): ${_NEG_RC}")
if(_NEG_RC EQUAL 0)
    message(FATAL_ERROR
        "negative run against a missing data directory unexpectedly exited 0; "
        "Authoring.population_add_drip_cpu must fail closed. log: ${RESULTS_DIR}/negative.log")
endif()
if(NOT _NEG_STDOUT MATCHES "=== 1 benchmark failure\\(s\\) ===")
    message(FATAL_ERROR
        "negative run did not record exactly one benchmark failure; later runs must stop after "
        "the row first fails. log: ${RESULTS_DIR}/negative.log")
endif()

run_verify_inventory(check-absent "${NEGATIVE_REPORT}" "Authoring.population_add_drip_cpu")
run_verify_diagnostics(check-empty "${NEGATIVE_DIAGNOSTICS}")

# ---------------------------------------------------------------------------
# Step 4: vacuity control. The same L1B family filter WITHOUT --hidden must
# select nothing at all, because every frozen row is hidden. An empty
# selection also exits 0, so this step asserts the zero-row shape explicitly
# rather than trusting the exit code alone. A run that silently selected
# zero rows must never be read as a passing contract.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 4: vacuity control (family filter without --hidden) ---")
set(VACUITY_REPORT "${RESULTS_DIR}/vacuity_report.txt")
set(VACUITY_DIAGNOSTICS "${RESULTS_DIR}/vacuity_diagnostics.jsonl")
file(REMOVE "${VACUITY_REPORT}")
file(REMOVE "${VACUITY_DIAGNOSTICS}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=${L1B_FAMILY_FILTER}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${VACUITY_REPORT}"
        "--timing-diagnostics=${VACUITY_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _VAC_RC
    OUTPUT_VARIABLE _VAC_STDOUT
    ERROR_VARIABLE _VAC_STDERR
)
file(WRITE "${RESULTS_DIR}/vacuity.log" "${_VAC_STDOUT}${_VAC_STDERR}")
message(STATUS "Exit code (vacuity control): ${_VAC_RC}")
if(NOT _VAC_RC EQUAL 0)
    message(FATAL_ERROR
        "vacuity control run exited non-zero (${_VAC_RC}); expected a clean run that simply "
        "selected nothing. log: ${RESULTS_DIR}/vacuity.log")
endif()

run_verify_inventory(check-vacuity "${PRODUCER_INVENTORY_JSON}" "${VACUITY_REPORT}")
run_verify_diagnostics(check-empty "${VACUITY_DIAGNOSTICS}")

# ---------------------------------------------------------------------------
# Step 5: unexecuted rows must publish nothing. The inventory's GPU rows are
# device-gated (isValid() is false in this CPU pass), so the harness never runs
# them. That is not a failure and the run must still exit 0, but the row must
# also not reach BmOutput::emit(), which would otherwise print a zero-valued
# metric line and, under --regenerate, write a zero baseline record. A zero
# baseline is permanent fail-open. BmOutput::performanceDelta() returns 0
# whenever either side is 0, so that row's comparison can never fail again.
#
# --regenerate is deliberately used here rather than a cheaper compare run,
# because it is the mode FrameCore runs and the mode that persists the damage.
# Step 2 above already proves this same binary does publish rows it executes,
# so "nothing published" cannot be read as a vacuous pass.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 5: device-gated rows publish no record (unexecuted-row suppression) ---")
set(SKIPPED_REPORT "${RESULTS_DIR}/skipped_gpu_report.txt")
set(SKIPPED_DIAGNOSTICS "${RESULTS_DIR}/skipped_gpu_diagnostics.jsonl")
file(REMOVE "${SKIPPED_REPORT}")
file(REMOVE "${SKIPPED_DIAGNOSTICS}")
baseline_txn_begin()
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--hidden"
        "--regenerate"
        "--filter=${GPU_ROW_FILTER}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${SKIPPED_REPORT}"
        "--timing-diagnostics=${SKIPPED_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _SKIP_RC
    OUTPUT_VARIABLE _SKIP_STDOUT
    ERROR_VARIABLE _SKIP_STDERR
)
baseline_txn_commit()
file(WRITE "${RESULTS_DIR}/skipped_gpu.log" "${_SKIP_STDOUT}${_SKIP_STDERR}")
message(STATUS "Exit code (device-gated rows): ${_SKIP_RC}")
if(NOT _SKIP_RC EQUAL 0)
    message(FATAL_ERROR
        "device-gated rows exited non-zero (${_SKIP_RC}); belonging to the other pass is a skip, "
        "not a failure. log: ${RESULTS_DIR}/skipped_gpu.log")
endif()

# "No row published a metric" is also true of a run that selected nothing, so
# the other half is proven first. Every requested GPU row was constructed and
# then gated out, once each, as shown by the harness diagnostic per exact row.
run_verify_inventory(check-skipped "${RESULTS_DIR}/skipped_gpu.log" ${GPU_ROW_NAMES})
run_verify_inventory(check-unpublished "${SKIPPED_REPORT}" ${GPU_ROW_NAMES})
run_verify_diagnostics(check-empty "${SKIPPED_DIAGNOSTICS}")

# ---------------------------------------------------------------------------
# Step 6: exercise the built harness's all-sample path with a cheap benchmark.
# Twenty measured steps across five runs produce exactly 100 sidecar samples.
# The dummy warm-up must not be counted. --threads=2 deliberately gives the
# row a postfix, proving the JSONL key is the exact decorated report name.
# --regenerate keeps the canonical report at its four-field shape, and the
# baseline transaction prevents this narrow Smoke run from replacing a
# developer's shared baseline.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 6: Smoke all-sample diagnostics and four-field report ---")
set(SMOKE_REPORT "${RESULTS_DIR}/smoke_diagnostics_report.txt")
set(SMOKE_DIAGNOSTICS "${RESULTS_DIR}/smoke_diagnostics.jsonl")
set(SMOKE_DECORATED_CMD "Smoke.no_op_2T")
file(REMOVE "${SMOKE_REPORT}" "${SMOKE_DIAGNOSTICS}")
baseline_txn_begin()
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--regenerate"
        "--filter=Smoke.no_op"
        "--steps=20"
        "--runs=5"
        "--threads=2"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${SMOKE_REPORT}"
        "--timing-diagnostics=${SMOKE_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _SMOKE_RC
    OUTPUT_VARIABLE _SMOKE_STDOUT
    ERROR_VARIABLE _SMOKE_STDERR
)
baseline_txn_commit()
file(WRITE "${RESULTS_DIR}/smoke_diagnostics.log" "${_SMOKE_STDOUT}${_SMOKE_STDERR}")
if(NOT _SMOKE_RC EQUAL 0)
    message(FATAL_ERROR
        "Smoke timing-diagnostics run failed (exit ${_SMOKE_RC}); "
        "log: ${RESULTS_DIR}/smoke_diagnostics.log")
endif()
run_verify_diagnostics(
    check-result
    "${SMOKE_REPORT}"
    "${SMOKE_DIAGNOSTICS}"
    --expected-cmd "${SMOKE_DECORATED_CMD}"
    --expected-count 100
)

# ---------------------------------------------------------------------------
# Step 7: an unusable diagnostics path is a process failure, not a silent
# fallback to report-only operation. A regular file in the parent position is
# a deterministic invalid path on every supported platform.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 7: invalid diagnostics path fails closed ---")
set(INVALID_DIAGNOSTICS_PARENT "${RESULTS_DIR}/invalid_diagnostics_parent")
set(INVALID_DIAGNOSTICS "${INVALID_DIAGNOSTICS_PARENT}/diagnostics.jsonl")
set(INVALID_DIAGNOSTICS_REPORT "${RESULTS_DIR}/invalid_diagnostics_report.txt")
file(REMOVE_RECURSE "${INVALID_DIAGNOSTICS_PARENT}")
file(WRITE "${INVALID_DIAGNOSTICS_PARENT}" "regular file, deliberately not a directory\n")
file(REMOVE "${INVALID_DIAGNOSTICS_REPORT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=Smoke.no_op"
        "--steps=1"
        "--runs=3"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${INVALID_DIAGNOSTICS_REPORT}"
        "--timing-diagnostics=${INVALID_DIAGNOSTICS}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _INVALID_DIAGNOSTICS_RC
    OUTPUT_VARIABLE _INVALID_DIAGNOSTICS_STDOUT
    ERROR_VARIABLE _INVALID_DIAGNOSTICS_STDERR
)
file(WRITE "${RESULTS_DIR}/invalid_diagnostics.log"
    "${_INVALID_DIAGNOSTICS_STDOUT}${_INVALID_DIAGNOSTICS_STDERR}")
if(_INVALID_DIAGNOSTICS_RC EQUAL 0)
    message(FATAL_ERROR
        "invalid timing-diagnostics path unexpectedly exited 0; "
        "log: ${RESULTS_DIR}/invalid_diagnostics.log")
endif()
set(_INVALID_DIAGNOSTICS_LOG "${_INVALID_DIAGNOSTICS_STDOUT}${_INVALID_DIAGNOSTICS_STDERR}")
if(NOT _INVALID_DIAGNOSTICS_LOG MATCHES "<timing-diagnostics>" OR
   NOT _INVALID_DIAGNOSTICS_LOG MATCHES "could not open timing diagnostics file")
    message(FATAL_ERROR
        "invalid timing-diagnostics path did not report the diagnostics open failure; "
        "log: ${RESULTS_DIR}/invalid_diagnostics.log")
endif()
if(EXISTS "${INVALID_DIAGNOSTICS_REPORT}")
    message(FATAL_ERROR
        "invalid timing-diagnostics path published a report unexpectedly: ${INVALID_DIAGNOSTICS_REPORT}")
endif()
if(EXISTS "${INVALID_DIAGNOSTICS}")
    message(FATAL_ERROR
        "invalid timing-diagnostics path unexpectedly created a sidecar: ${INVALID_DIAGNOSTICS}")
endif()

# ---------------------------------------------------------------------------
# Step 7b: report and diagnostics must not resolve to the same file, even when
# their path strings differ. Two independent writers would otherwise truncate
# or interleave the canonical report.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 7b: report and diagnostics path collision fails closed ---")
set(COLLIDING_OUTPUT "${RESULTS_DIR}/colliding_report_and_diagnostics.txt")
set(COLLIDING_OUTPUT_ALIAS "${RESULTS_DIR}/./colliding_report_and_diagnostics.txt")
file(REMOVE "${COLLIDING_OUTPUT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=Smoke.no_op"
        "--steps=1"
        "--runs=3"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${COLLIDING_OUTPUT}"
        "--timing-diagnostics=${COLLIDING_OUTPUT_ALIAS}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _COLLIDING_OUTPUT_RC
    OUTPUT_VARIABLE _COLLIDING_OUTPUT_STDOUT
    ERROR_VARIABLE _COLLIDING_OUTPUT_STDERR
)
file(WRITE "${RESULTS_DIR}/colliding_report_and_diagnostics.log"
    "${_COLLIDING_OUTPUT_STDOUT}${_COLLIDING_OUTPUT_STDERR}")
if(_COLLIDING_OUTPUT_RC EQUAL 0)
    message(FATAL_ERROR
        "colliding report and diagnostics paths unexpectedly exited 0; "
        "log: ${RESULTS_DIR}/colliding_report_and_diagnostics.log")
endif()
set(_COLLIDING_OUTPUT_LOG "${_COLLIDING_OUTPUT_STDOUT}${_COLLIDING_OUTPUT_STDERR}")
if(NOT _COLLIDING_OUTPUT_LOG MATCHES
   "--timing-diagnostics and --report must name different files")
    message(FATAL_ERROR
        "colliding output paths did not report the path conflict; "
        "log: ${RESULTS_DIR}/colliding_report_and_diagnostics.log")
endif()
if(EXISTS "${COLLIDING_OUTPUT}")
    message(FATAL_ERROR
        "colliding output paths unexpectedly created a file: ${COLLIDING_OUTPUT}")
endif()

if(UNIX)
    message(STATUS "")
    message(STATUS "--- step 7c: dangling symlink output collision fails closed ---")
    set(SYMLINK_TARGET "${RESULTS_DIR}/colliding_symlink_target.txt")
    set(SYMLINK_ALIAS "${RESULTS_DIR}/colliding_symlink_alias.txt")
    file(REMOVE "${SYMLINK_TARGET}" "${SYMLINK_ALIAS}")
    file(CREATE_LINK "${SYMLINK_TARGET}" "${SYMLINK_ALIAS}" SYMBOLIC RESULT _SYMLINK_CREATE_RESULT)
    if(NOT _SYMLINK_CREATE_RESULT STREQUAL "0")
        message(FATAL_ERROR "could not create path-collision test symlink: ${_SYMLINK_CREATE_RESULT}")
    endif()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--filter=Smoke.no_op"
            "--steps=1"
            "--runs=3"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${SYMLINK_TARGET}"
            "--timing-diagnostics=${SYMLINK_ALIAS}"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 300
        RESULT_VARIABLE _SYMLINK_COLLISION_RC
        OUTPUT_VARIABLE _SYMLINK_COLLISION_STDOUT
        ERROR_VARIABLE _SYMLINK_COLLISION_STDERR
    )
    file(WRITE "${RESULTS_DIR}/colliding_symlink.log"
        "${_SYMLINK_COLLISION_STDOUT}${_SYMLINK_COLLISION_STDERR}")
    if(_SYMLINK_COLLISION_RC EQUAL 0)
        message(FATAL_ERROR
            "symlinked report and diagnostics paths unexpectedly exited 0; "
            "log: ${RESULTS_DIR}/colliding_symlink.log")
    endif()
    set(_SYMLINK_COLLISION_LOG "${_SYMLINK_COLLISION_STDOUT}${_SYMLINK_COLLISION_STDERR}")
    if(NOT _SYMLINK_COLLISION_LOG MATCHES
       "--timing-diagnostics and --report must name different files")
        message(FATAL_ERROR
            "symlinked output paths did not report the path conflict; "
            "log: ${RESULTS_DIR}/colliding_symlink.log")
    endif()
    if(EXISTS "${SYMLINK_TARGET}")
        message(FATAL_ERROR
            "symlinked output paths unexpectedly created their target: ${SYMLINK_TARGET}")
    endif()
    file(REMOVE "${SYMLINK_ALIAS}")
endif()

# ---------------------------------------------------------------------------
# Step 8: the binding row cannot be run as a normal CPU benchmark. This is the
# locally portable runtime half of its DirectGPU contract. The first-use timing
# and override failures remain GPU-only tests.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 8: binding row rejects a non-DirectGPU invocation ---")
set(BINDING_CPU_REPORT "${RESULTS_DIR}/binding_cpu_report.txt")
set(BINDING_CPU_DIAGNOSTICS "${RESULTS_DIR}/binding_cpu_diagnostics.jsonl")
file(REMOVE "${BINDING_CPU_REPORT}" "${BINDING_CPU_DIAGNOSTICS}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--hidden"
        "--filter=${BINDING_ROW}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${BINDING_CPU_REPORT}"
        "--timing-diagnostics=${BINDING_CPU_DIAGNOSTICS}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 300
    RESULT_VARIABLE _BINDING_CPU_RC
    OUTPUT_VARIABLE _BINDING_CPU_STDOUT
    ERROR_VARIABLE _BINDING_CPU_STDERR
)
file(WRITE "${RESULTS_DIR}/binding_cpu.log" "${_BINDING_CPU_STDOUT}${_BINDING_CPU_STDERR}")
if(_BINDING_CPU_RC EQUAL 0)
    message(FATAL_ERROR
        "binding row unexpectedly ran without --forceGpu --directGpu; "
        "log: ${RESULTS_DIR}/binding_cpu.log")
endif()
if(EXISTS "${BINDING_CPU_REPORT}")
    message(FATAL_ERROR
        "rejected non-DirectGPU binding row unexpectedly published a report: ${BINDING_CPU_REPORT}")
endif()
run_verify_diagnostics(check-empty "${BINDING_CPU_DIAGNOSTICS}")

# ---------------------------------------------------------------------------
# Step 9 (opt-in): the dedicated benchmark CI runner has a GPU, while this
# contract is also useful on CPU-only developer hosts. When explicitly
# enabled, it proves that the real DirectGPU control row publishes its default
# 100 samples. Three fresh processes then prove the one-shot binding row: one
# successful default invocation, followed by step and run overrides that
# request a second timed call and must fail before either output publishes a
# row.
# ---------------------------------------------------------------------------
if("$ENV{OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU}" STREQUAL "1")
    message(STATUS "")
    message(STATUS "--- step 9: DirectGPU Cartpole control and binding contracts ---")
    set(CONTROL_GPU_CMD "${CONTROL_ROW}_GPU")
    set(CONTROL_GPU_REPORT "${RESULTS_DIR}/control_gpu_report.txt")
    set(CONTROL_GPU_DIAGNOSTICS "${RESULTS_DIR}/control_gpu_diagnostics.jsonl")
    file(REMOVE "${CONTROL_GPU_REPORT}" "${CONTROL_GPU_DIAGNOSTICS}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--hidden"
            "--regenerate"
            "--forceGpu"
            "--directGpu"
            "--filter=${CONTROL_ROW}"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${CONTROL_GPU_REPORT}"
            "--timing-diagnostics=${CONTROL_GPU_DIAGNOSTICS}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 1800
        RESULT_VARIABLE _CONTROL_GPU_RC
        OUTPUT_VARIABLE _CONTROL_GPU_STDOUT
        ERROR_VARIABLE _CONTROL_GPU_STDERR
    )
    baseline_txn_commit()
    file(WRITE "${RESULTS_DIR}/control_gpu.log" "${_CONTROL_GPU_STDOUT}${_CONTROL_GPU_STDERR}")
    if(NOT _CONTROL_GPU_RC EQUAL 0)
        message(FATAL_ERROR
            "DirectGPU control run failed (exit ${_CONTROL_GPU_RC}); "
            "log: ${RESULTS_DIR}/control_gpu.log")
    endif()
    run_verify_diagnostics(
        check-result
        "${CONTROL_GPU_REPORT}"
        "${CONTROL_GPU_DIAGNOSTICS}"
        --expected-cmd "${CONTROL_GPU_CMD}"
        --expected-count 100
    )

    set(BINDING_GPU_CMD "${BINDING_ROW}_GPU")
    set(BINDING_GPU_REPORT "${RESULTS_DIR}/binding_gpu_report.txt")
    set(BINDING_GPU_DIAGNOSTICS "${RESULTS_DIR}/binding_gpu_diagnostics.jsonl")
    file(REMOVE "${BINDING_GPU_REPORT}" "${BINDING_GPU_DIAGNOSTICS}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--hidden"
            "--regenerate"
            "--forceGpu"
            "--directGpu"
            "--filter=${BINDING_ROW}"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${BINDING_GPU_REPORT}"
            "--timing-diagnostics=${BINDING_GPU_DIAGNOSTICS}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 1800
        RESULT_VARIABLE _BINDING_GPU_RC
        OUTPUT_VARIABLE _BINDING_GPU_STDOUT
        ERROR_VARIABLE _BINDING_GPU_STDERR
    )
    baseline_txn_commit()
    file(WRITE "${RESULTS_DIR}/binding_gpu.log" "${_BINDING_GPU_STDOUT}${_BINDING_GPU_STDERR}")
    if(NOT _BINDING_GPU_RC EQUAL 0)
        message(FATAL_ERROR
            "DirectGPU binding positive run failed (exit ${_BINDING_GPU_RC}); "
            "log: ${RESULTS_DIR}/binding_gpu.log")
    endif()
    run_verify_diagnostics(
        check-result
        "${BINDING_GPU_REPORT}"
        "${BINDING_GPU_DIAGNOSTICS}"
        --expected-cmd "${BINDING_GPU_CMD}"
        --expected-count 1
    )

    foreach(_OVERRIDE IN ITEMS "steps=2" "runs=3")
        string(REPLACE "=" "_" _OVERRIDE_LABEL "${_OVERRIDE}")
        set(_OVERRIDE_REPORT "${RESULTS_DIR}/binding_gpu_${_OVERRIDE_LABEL}_report.txt")
        set(_OVERRIDE_DIAGNOSTICS "${RESULTS_DIR}/binding_gpu_${_OVERRIDE_LABEL}_diagnostics.jsonl")
        set(_OVERRIDE_LOG "${RESULTS_DIR}/binding_gpu_${_OVERRIDE_LABEL}.log")
        file(REMOVE "${_OVERRIDE_REPORT}" "${_OVERRIDE_DIAGNOSTICS}")
        execute_process(
            COMMAND "${BENCHMARK_BINARY}"
                "--hidden"
                "--forceGpu"
                "--directGpu"
                "--filter=${BINDING_ROW}"
                "--${_OVERRIDE}"
                "--data=${PROJECT_ROOT}/tests/data"
                "--report=${_OVERRIDE_REPORT}"
                "--timing-diagnostics=${_OVERRIDE_DIAGNOSTICS}"
                "--verbose"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            TIMEOUT 1800
            RESULT_VARIABLE _OVERRIDE_RC
            OUTPUT_VARIABLE _OVERRIDE_STDOUT
            ERROR_VARIABLE _OVERRIDE_STDERR
        )
        file(WRITE "${_OVERRIDE_LOG}" "${_OVERRIDE_STDOUT}${_OVERRIDE_STDERR}")
        if(_OVERRIDE_RC EQUAL 0)
            message(FATAL_ERROR
                "DirectGPU binding --${_OVERRIDE} override unexpectedly exited 0; log: ${_OVERRIDE_LOG}")
        endif()
        set(_OVERRIDE_COMBINED_LOG "${_OVERRIDE_STDOUT}${_OVERRIDE_STDERR}")
        if(NOT _OVERRIDE_COMBINED_LOG MATCHES "must run once per process")
            message(FATAL_ERROR
                "DirectGPU binding --${_OVERRIDE} did not fail through the second-measurement guard; "
                "log: ${_OVERRIDE_LOG}")
        endif()
        run_verify_diagnostics(check-no-result "${_OVERRIDE_REPORT}" --expected-cmd "${BINDING_GPU_CMD}")
        run_verify_diagnostics(check-empty "${_OVERRIDE_DIAGNOSTICS}")
    endforeach()
else()
    message(STATUS "")
    message(STATUS
        "--- step 9: DirectGPU Cartpole runtime contracts skipped "
        "(set OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU=1 on a GPU host) ---")
endif()

# ---------------------------------------------------------------------------
# Step 10: the OutputRead smoke contract.
#
# OutputRead rows are performance lanes, not a frozen inventory, so this does
# not compare them against producer_inventory.json. What it gates is the
# producer contract they share with every other family. A row that completes
# publishes a number and the process exits cleanly, and a row that cannot do
# its work publishes NOTHING and the process exits non-zero.
#
# That second half is the one worth a test. A failed output read returns early,
# so it times FASTER than a successful one, exactly the shape a regression
# gate must never mistake for an optimization.
#
# --steps/--runs are cut right down. This is a contract, not a measurement.
# ---------------------------------------------------------------------------
set(OUTPUT_READ_CPU_ROW "OutputRead.queryread_rb_1024_cpu")

message(STATUS "")
message(STATUS "--- step 10: OutputRead smoke, clean exit and published row (positive) ---")
set(OUTPUT_READ_REPORT "${RESULTS_DIR}/output_read_report.txt")
set(OUTPUT_READ_DIAGNOSTICS "${RESULTS_DIR}/output_read_diagnostics.jsonl")
file(REMOVE "${OUTPUT_READ_REPORT}" "${OUTPUT_READ_DIAGNOSTICS}")
baseline_txn_begin()
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--regenerate"
        "--filter=${OUTPUT_READ_CPU_ROW}"
        "--steps=3"
        "--runs=3"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${OUTPUT_READ_REPORT}"
        "--timing-diagnostics=${OUTPUT_READ_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 900
    RESULT_VARIABLE _OR_RC
    OUTPUT_VARIABLE _OR_STDOUT
    ERROR_VARIABLE _OR_STDERR
)
baseline_txn_commit()
file(WRITE "${RESULTS_DIR}/output_read.log" "${_OR_STDOUT}${_OR_STDERR}")
message(STATUS "Exit code (OutputRead positive): ${_OR_RC}")

# The exit code IS the assertion here. bmTerminate() runs after the report is
# written, so a teardown fault shows up as a non-zero exit on a run whose
# numbers were all fine. Nothing else in the suite covers that.
if(NOT _OR_RC EQUAL 0)
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} did not exit cleanly (exit ${_OR_RC}); the row's work may still have "
        "succeeded -- check whether the failure is in teardown. log: ${RESULTS_DIR}/output_read.log")
endif()
if(NOT _OR_STDOUT MATCHES "${OUTPUT_READ_CPU_ROW}")
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} selected nothing; a contract that runs zero rows must not read as a "
        "pass. log: ${RESULTS_DIR}/output_read.log")
endif()
if(_OR_STDOUT MATCHES "benchmark failure\\(s\\)")
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} recorded a failure on a run that should be clean; "
        "log: ${RESULTS_DIR}/output_read.log")
endif()
if(NOT EXISTS "${OUTPUT_READ_REPORT}")
    message(FATAL_ERROR "OutputRead positive run wrote no report: ${OUTPUT_READ_REPORT}")
endif()
file(READ "${OUTPUT_READ_REPORT}" _OR_REPORT_TEXT)
if(NOT _OR_REPORT_TEXT MATCHES "${OUTPUT_READ_CPU_ROW}")
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} exited 0 but published no metric; report: ${OUTPUT_READ_REPORT}")
endif()

message(STATUS "")
message(STATUS "--- step 10b: OutputRead against missing data directory (negative) ---")
set(OUTPUT_READ_NEG_REPORT "${RESULTS_DIR}/output_read_negative_report.txt")
set(OUTPUT_READ_NEG_DIAGNOSTICS "${RESULTS_DIR}/output_read_negative_diagnostics.jsonl")
file(REMOVE "${OUTPUT_READ_NEG_REPORT}" "${OUTPUT_READ_NEG_DIAGNOSTICS}")
if(EXISTS "${MISSING_DATA_DIR}")
    file(REMOVE_RECURSE "${MISSING_DATA_DIR}")
endif()
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=${OUTPUT_READ_CPU_ROW}"
        "--steps=3"
        "--runs=3"
        "--data=${MISSING_DATA_DIR}"
        "--report=${OUTPUT_READ_NEG_REPORT}"
        "--timing-diagnostics=${OUTPUT_READ_NEG_DIAGNOSTICS}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 900
    RESULT_VARIABLE _OR_NEG_RC
    OUTPUT_VARIABLE _OR_NEG_STDOUT
    ERROR_VARIABLE _OR_NEG_STDERR
)
file(WRITE "${RESULTS_DIR}/output_read_negative.log" "${_OR_NEG_STDOUT}${_OR_NEG_STDERR}")
message(STATUS "Exit code (OutputRead negative): ${_OR_NEG_RC}")
if(_OR_NEG_RC EQUAL 0)
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} against a missing data directory unexpectedly exited 0; a read that "
        "did no work must fail closed. log: ${RESULTS_DIR}/output_read_negative.log")
endif()
# One entry, not one per run. The row records its first failure and stays quiet
# after it, so a count above one means the guard stopped holding.
if(NOT _OR_NEG_STDOUT MATCHES "=== 1 benchmark failure\\(s\\) ===")
    message(FATAL_ERROR
        "${OUTPUT_READ_CPU_ROW} did not record exactly one benchmark failure; "
        "log: ${RESULTS_DIR}/output_read_negative.log")
endif()
run_verify_inventory(check-absent "${OUTPUT_READ_NEG_REPORT}" "${OUTPUT_READ_CPU_ROW}")
run_verify_diagnostics(check-empty "${OUTPUT_READ_NEG_DIAGNOSTICS}")

# ---------------------------------------------------------------------------
# Step 10c (opt-in): the same clean-exit contract on a DirectGPU scene, where
# the read serves device-resident columns and waits on a completion event.
# ---------------------------------------------------------------------------
if("$ENV{OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU}" STREQUAL "1")
    message(STATUS "")
    message(STATUS "--- step 10c: OutputRead DirectGPU smoke ---")
    set(OUTPUT_READ_GPU_ROW "OutputRead.queryread_rb_1024_gpu")
    set(OUTPUT_READ_GPU_REPORT "${RESULTS_DIR}/output_read_gpu_report.txt")
    set(OUTPUT_READ_GPU_DIAGNOSTICS "${RESULTS_DIR}/output_read_gpu_diagnostics.jsonl")
    file(REMOVE "${OUTPUT_READ_GPU_REPORT}" "${OUTPUT_READ_GPU_DIAGNOSTICS}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--hidden"
            "--regenerate"
            "--forceGpu"
            "--directGpu"
            "--filter=${OUTPUT_READ_GPU_ROW}"
            "--steps=3"
            "--runs=3"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${OUTPUT_READ_GPU_REPORT}"
            "--timing-diagnostics=${OUTPUT_READ_GPU_DIAGNOSTICS}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 1800
        RESULT_VARIABLE _OR_GPU_RC
        OUTPUT_VARIABLE _OR_GPU_STDOUT
        ERROR_VARIABLE _OR_GPU_STDERR
    )
    baseline_txn_commit()
    file(WRITE "${RESULTS_DIR}/output_read_gpu.log" "${_OR_GPU_STDOUT}${_OR_GPU_STDERR}")
    message(STATUS "Exit code (OutputRead DirectGPU): ${_OR_GPU_RC}")
    if(NOT _OR_GPU_RC EQUAL 0)
        message(FATAL_ERROR
            "${OUTPUT_READ_GPU_ROW} did not exit cleanly (exit ${_OR_GPU_RC}); "
            "log: ${RESULTS_DIR}/output_read_gpu.log")
    endif()
    if(_OR_GPU_STDOUT MATCHES "benchmark failure\\(s\\)")
        message(FATAL_ERROR
            "${OUTPUT_READ_GPU_ROW} recorded a failure on a run that should be clean; "
            "log: ${RESULTS_DIR}/output_read_gpu.log")
    endif()
    file(READ "${OUTPUT_READ_GPU_REPORT}" _OR_GPU_REPORT_TEXT)
    if(NOT _OR_GPU_REPORT_TEXT MATCHES "${OUTPUT_READ_GPU_ROW}")
        message(FATAL_ERROR
            "${OUTPUT_READ_GPU_ROW} exited 0 but published no metric; report: ${OUTPUT_READ_GPU_REPORT}")
    endif()
else()
    message(STATUS "")
    message(STATUS
        "--- step 10c: OutputRead DirectGPU smoke skipped "
        "(set OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU=1 on a GPU host) ---")
endif()

# ---------------------------------------------------------------------------
# Step 10d: every registered OutputRead row is in outputread_inventory.json,
# and every inventory row names an owner.
#
# Steps 10/10b/10c gate BEHAVIOUR: a row exits cleanly, publishes, and fails
# closed on missing data. None of them can see a row being ADDED. The check
# that keeps the family from growing unnoticed is not a count but the
# requirement that a new row arrive with someone's name on it. The inventory
# sets require_owner, so a row added without an owner fails to load, and a row
# added without an inventory entry fails to match.
#
# The hidden flag is compared too: OutputRead is legitimately mixed (CPU rows
# visible, DirectGPU rows hidden), and a GPU row that stopped being hidden
# would run on every default developer pass and fail with no CUDA device.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "--- step 10d: registered OutputRead rows vs outputread_inventory.json ---")
set(OUTPUT_READ_LIST_OUTPUT "${RESULTS_DIR}/output_read_list_output.txt")
file(REMOVE "${OUTPUT_READ_LIST_OUTPUT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}" "--list" "--filter=OutputRead.*"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    TIMEOUT 120
    RESULT_VARIABLE _OR_LIST_RC
    OUTPUT_VARIABLE _OR_LIST_STDOUT
    ERROR_VARIABLE _OR_LIST_STDERR
)
file(WRITE "${OUTPUT_READ_LIST_OUTPUT}" "${_OR_LIST_STDOUT}")
message(STATUS "Exit code (OutputRead list): ${_OR_LIST_RC}")
if(NOT _OR_LIST_RC EQUAL 0)
    message(FATAL_ERROR
        "ovphysx_benchmarks --list --filter=OutputRead.* failed (exit ${_OR_LIST_RC}):\n"
        "${_OR_LIST_STDOUT}\n${_OR_LIST_STDERR}")
endif()

run_verify_inventory(check-list-owned "${OUTPUT_READ_INVENTORY_JSON}" "${OUTPUT_READ_LIST_OUTPUT}")

message(STATUS "")
message(STATUS "ovphysx benchmark contract: PASSED (${INVENTORY_TOTAL} total / ${INVENTORY_CPU} CPU rows)")

# ContactReport execution helpers. Positive runs protect the developer
# baseline and use the shared exact report/diagnostics grammar. Wrong-device
# runs prove both selection and suppression without regenerating a baseline.
function(run_contact_positive LABEL ROW EXPECTED_COMMAND)
    set(_REPORT "${RESULTS_DIR}/contact_${LABEL}_report.txt")
    set(_DIAGNOSTICS "${RESULTS_DIR}/contact_${LABEL}_diagnostics.jsonl")
    set(_LOG "${RESULTS_DIR}/contact_${LABEL}.log")
    file(REMOVE "${_REPORT}" "${_DIAGNOSTICS}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}" "--hidden" "--regenerate" ${ARGN}
            "--filter=${ROW}"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${_REPORT}"
            "--timing-diagnostics=${_DIAGNOSTICS}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 1800
        RESULT_VARIABLE _RC OUTPUT_VARIABLE _STDOUT ERROR_VARIABLE _STDERR
    )
    baseline_txn_commit()
    file(WRITE "${_LOG}" "${_STDOUT}${_STDERR}")
    if(NOT _RC EQUAL 0)
        message(FATAL_ERROR "ContactReport ${LABEL} run failed (exit ${_RC}); log: ${_LOG}")
    endif()
    run_verify_diagnostics(check-result "${_REPORT}" "${_DIAGNOSTICS}"
        --expected-cmd "${EXPECTED_COMMAND}" --expected-count 100)
endfunction()

function(run_contact_wrong_device LABEL ROW ROW_DEVICE PASS_DEVICE)
    set(_REPORT "${RESULTS_DIR}/contact_${LABEL}_report.txt")
    set(_LOG "${RESULTS_DIR}/contact_${LABEL}.log")
    file(REMOVE "${_REPORT}")
    execute_process(
        COMMAND "${BENCHMARK_BINARY}" "--hidden" ${ARGN}
            "--filter=${ROW}"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${_REPORT}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 300
        RESULT_VARIABLE _RC OUTPUT_VARIABLE _STDOUT ERROR_VARIABLE _STDERR
    )
    file(WRITE "${_LOG}" "${_STDOUT}${_STDERR}")
    if(NOT _RC EQUAL 0)
        message(FATAL_ERROR
            "wrong-device ContactReport ${LABEL} run failed instead of skipping (exit ${_RC}); log: ${_LOG}")
    endif()
    run_verify_contact_inventory(check-skipped "${CONTACT_INVENTORY_JSON}" "${_LOG}"
        --row-device "${ROW_DEVICE}" --pass-device "${PASS_DEVICE}")
    run_verify_contact_inventory(check-no-metrics "${_REPORT}")
endfunction()

message(STATUS "")
message(STATUS "=== ovphysx ContactReport benchmark contract (separate two-row inventory) ===")

# The executable list must exactly match the separate two-row hidden inventory.
message(STATUS "--- contact A: exact hidden two-row executable inventory ---")
set(CONTACT_LIST_OUTPUT "${RESULTS_DIR}/contact_list_output.txt")
file(REMOVE "${CONTACT_LIST_OUTPUT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}" "--list" "--filter=${CONTACT_FAMILY_FILTER}"
    WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 120
    RESULT_VARIABLE _CONTACT_LIST_RC
    OUTPUT_FILE "${CONTACT_LIST_OUTPUT}" ERROR_VARIABLE _CONTACT_LIST_STDERR
)
if(NOT _CONTACT_LIST_RC EQUAL 0)
    message(FATAL_ERROR
        "ContactReport --list failed (exit ${_CONTACT_LIST_RC}):\n${_CONTACT_LIST_STDERR}")
endif()
run_verify_contact_inventory(check-list "${CONTACT_INVENTORY_JSON}" "${CONTACT_LIST_OUTPUT}")

# Omitting --hidden must select no contact row even though an empty run exits zero.
message(STATUS "--- contact B: hidden-row vacuity control ---")
set(CONTACT_VACUITY_REPORT "${RESULTS_DIR}/contact_vacuity_report.txt")
file(REMOVE "${CONTACT_VACUITY_REPORT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=${CONTACT_FAMILY_FILTER}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${CONTACT_VACUITY_REPORT}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 300
    RESULT_VARIABLE _CONTACT_VACUITY_RC
    OUTPUT_VARIABLE _CONTACT_VACUITY_STDOUT ERROR_VARIABLE _CONTACT_VACUITY_STDERR
)
if(NOT _CONTACT_VACUITY_RC EQUAL 0)
    message(FATAL_ERROR
        "ContactReport vacuity control failed (exit ${_CONTACT_VACUITY_RC}):\n"
        "${_CONTACT_VACUITY_STDOUT}${_CONTACT_VACUITY_STDERR}")
endif()
run_verify_contact_inventory(check-no-metrics "${CONTACT_VACUITY_REPORT}")

message(STATUS "--- contact C: positive CPU row and 100 timed calls ---")
run_contact_positive(cpu "${CONTACT_CPU_ROW}" "${CONTACT_CPU_COMMAND}")

message(STATUS "--- contact D: GPU row is selected and suppressed in a CPU pass ---")
run_contact_wrong_device(gpu_on_cpu "${CONTACT_GPU_ROW}" gpu cpu)

if("$ENV{OVPHYSX_BENCHMARK_CONTRACT_GPU}" STREQUAL "1")
    message(STATUS "--- contact E: positive conventional-GPU row and 100 timed calls ---")
    run_contact_positive(gpu "${CONTACT_GPU_ROW}" "${CONTACT_GPU_COMMAND}" --forceGpu)

    message(STATUS "--- contact F: CPU row is selected and suppressed in a GPU pass ---")
    run_contact_wrong_device(cpu_on_gpu "${CONTACT_CPU_ROW}" cpu gpu --forceGpu)

    # DirectGPU is unsupported for this report path and must fail without a metric.
    message(STATUS "--- contact G: DirectGPU is rejected without publishing ---")
    set(CONTACT_DIRECT_GPU_REPORT "${RESULTS_DIR}/contact_direct_gpu_report.txt")
    file(REMOVE "${CONTACT_DIRECT_GPU_REPORT}")
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--hidden" "--forceGpu" "--directGpu"
            "--filter=${CONTACT_GPU_ROW}"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${CONTACT_DIRECT_GPU_REPORT}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 300
        RESULT_VARIABLE _CONTACT_DIRECT_GPU_RC
        OUTPUT_VARIABLE _CONTACT_DIRECT_GPU_STDOUT ERROR_VARIABLE _CONTACT_DIRECT_GPU_STDERR
    )
    if(_CONTACT_DIRECT_GPU_RC EQUAL 0)
        message(FATAL_ERROR
            "DirectGPU ContactReport refusal unexpectedly exited zero")
    endif()
    set(_CONTACT_DIRECT_GPU_LOG_TEXT "${_CONTACT_DIRECT_GPU_STDOUT}${_CONTACT_DIRECT_GPU_STDERR}")
    if(NOT _CONTACT_DIRECT_GPU_LOG_TEXT MATCHES "reject DirectGPU")
        message(FATAL_ERROR
            "DirectGPU ContactReport refusal did not report its mode error:\n${_CONTACT_DIRECT_GPU_LOG_TEXT}")
    endif()
    run_verify_contact_inventory(check-no-metrics "${CONTACT_DIRECT_GPU_REPORT}" --allow-missing)
else()
    message(STATUS
        "--- contact E-G: conventional-GPU and DirectGPU-refusal contracts skipped "
        "(set OVPHYSX_BENCHMARK_CONTRACT_GPU=1 on a GPU host) ---")
endif()

message(STATUS "ovphysx ContactReport benchmark contract: PASSED (2 hidden rows; CPU plus conventional GPU)")

# ---------------------------------------------------------------------------
# High-scale diagnostics: the separate five-row hidden inventory
# (REQ-CAPI-BENCHMARK-005). Step 1 already proved that the frozen L1B
# `Authoring.*:WriteScaling.*` list is exactly producer_inventory.json, which is
# the isolation half of this contract. The WriteScalingHighN.* and
# RuntimeSpawnScaling.* families are invisible to the L1B selection.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=== ovphysx high-scale diagnostics contract (separate five-row inventory) ===")

# All four inventories share one strstr()-matched baseline lookup, so their
# union must be free of duplicate and substring-colliding names, including
# names that would only collide once the harness decorates a record.
message(STATUS "--- high-scale A: inventory union is collision-free ---")
run_verify_inventory(check-union
    "${PRODUCER_INVENTORY_JSON}" "${OUTPUT_READ_INVENTORY_JSON}"
    "${HIGH_SCALE_INVENTORY_JSON}" "${CONTACT_INVENTORY_JSON}")

run_verify_inventory(counts "${HIGH_SCALE_INVENTORY_JSON}")
set(_HIGH_SCALE_COUNTS "${VERIFY_STDOUT}")
string(REGEX MATCH "TOTAL=([0-9]+)" _ "${_HIGH_SCALE_COUNTS}")
set(HIGH_SCALE_TOTAL "${CMAKE_MATCH_1}")
string(REGEX MATCH "CPU=([0-9]+)" _ "${_HIGH_SCALE_COUNTS}")
set(HIGH_SCALE_CPU "${CMAKE_MATCH_1}")
if(NOT HIGH_SCALE_TOTAL OR NOT HIGH_SCALE_TOTAL STREQUAL HIGH_SCALE_CPU)
    message(FATAL_ERROR
        "high-scale inventory must be CPU-only (TOTAL=${HIGH_SCALE_TOTAL}, CPU=${HIGH_SCALE_CPU}):\n"
        "${_HIGH_SCALE_COUNTS}")
endif()
run_verify_inventory(filter "${HIGH_SCALE_INVENTORY_JSON}" --device cpu)
set(HIGH_SCALE_ROW_FILTER "${VERIFY_STDOUT}")
if(NOT HIGH_SCALE_ROW_FILTER)
    message(FATAL_ERROR "High-scale inventory produced an empty CPU filter")
endif()
message(STATUS "  Inventory rows:   ${HIGH_SCALE_TOTAL} (all CPU)")
message(STATUS "  Row filter:       ${HIGH_SCALE_ROW_FILTER}")

# The scoped --list must name exactly the inventory's rows, every one hidden.
message(STATUS "--- high-scale B: exact hidden executable inventory ---")
set(HIGH_SCALE_LIST_OUTPUT "${RESULTS_DIR}/high_scale_list_output.txt")
file(REMOVE "${HIGH_SCALE_LIST_OUTPUT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}" "--list" "--filter=${HIGH_SCALE_FAMILY_FILTER}"
    WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 120
    RESULT_VARIABLE _HIGH_SCALE_LIST_RC
    OUTPUT_FILE "${HIGH_SCALE_LIST_OUTPUT}" ERROR_VARIABLE _HIGH_SCALE_LIST_STDERR
)
if(NOT _HIGH_SCALE_LIST_RC EQUAL 0)
    message(FATAL_ERROR
        "high-scale --list failed (exit ${_HIGH_SCALE_LIST_RC}):\n${_HIGH_SCALE_LIST_STDERR}")
endif()
run_verify_inventory(check-list "${HIGH_SCALE_INVENTORY_JSON}" "${HIGH_SCALE_LIST_OUTPUT}")

# Omitting --hidden must select no high-scale row even though an empty run
# exits zero.
message(STATUS "--- high-scale C: hidden-row vacuity control ---")
set(HIGH_SCALE_VACUITY_REPORT "${RESULTS_DIR}/high_scale_vacuity_report.txt")
file(REMOVE "${HIGH_SCALE_VACUITY_REPORT}")
execute_process(
    COMMAND "${BENCHMARK_BINARY}"
        "--filter=${HIGH_SCALE_FAMILY_FILTER}"
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${HIGH_SCALE_VACUITY_REPORT}"
        "--verbose"
    WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 300
    RESULT_VARIABLE _HIGH_SCALE_VACUITY_RC
    OUTPUT_VARIABLE _HIGH_SCALE_VACUITY_STDOUT ERROR_VARIABLE _HIGH_SCALE_VACUITY_STDERR
)
file(WRITE "${RESULTS_DIR}/high_scale_vacuity.log" "${_HIGH_SCALE_VACUITY_STDOUT}${_HIGH_SCALE_VACUITY_STDERR}")
if(NOT _HIGH_SCALE_VACUITY_RC EQUAL 0)
    message(FATAL_ERROR
        "high-scale vacuity control exited non-zero (${_HIGH_SCALE_VACUITY_RC}); expected a clean run that "
        "simply selected nothing. log: ${RESULTS_DIR}/high_scale_vacuity.log")
endif()
run_verify_inventory(check-vacuity "${HIGH_SCALE_INVENTORY_JSON}" "${HIGH_SCALE_VACUITY_REPORT}")

# The positive run (OPT-IN) must publish exactly the five rows once each. Every
# row carries its own untimed correctness gate (written velocity on every body
# for the WriteScalingHighN.* rows; realized rigid body at the exact authored
# path for the collider row), so a published row is a validated row.
#
# --steps/--runs are cut right down, as step 10 does for OutputRead. This is a
# contract, not a measurement. Even so the run is opt-in under
# OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1. Populating and attaching a
# 16,384-body scene costs minutes per run, so even the reduced shape is a
# nightly-sized workload that does not fit a per-MR job. The scheduled
# FrameCore run is where the default 20-by-5 statistic is produced and
# validated. Run this step locally before changing these rows.
if("$ENV{OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE}" STREQUAL "1")
    message(STATUS "--- high-scale D: positive five-row run (--hidden --regenerate, reduced steps/runs) ---")
    set(HIGH_SCALE_REPORT "${RESULTS_DIR}/high_scale_positive_report.txt")
    file(REMOVE "${HIGH_SCALE_REPORT}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BENCHMARK_BINARY}"
            "--hidden"
            "--regenerate"
            "--filter=${HIGH_SCALE_ROW_FILTER}"
            "--steps=1"
            "--runs=3"
            "--data=${PROJECT_ROOT}/tests/data"
            "--report=${HIGH_SCALE_REPORT}"
            "--verbose"
        WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 7200
        RESULT_VARIABLE _HIGH_SCALE_RC
        OUTPUT_VARIABLE _HIGH_SCALE_STDOUT ERROR_VARIABLE _HIGH_SCALE_STDERR
    )
    baseline_txn_commit()
    file(WRITE "${RESULTS_DIR}/high_scale_positive.log" "${_HIGH_SCALE_STDOUT}${_HIGH_SCALE_STDERR}")
    message(STATUS "Exit code (high-scale positive run): ${_HIGH_SCALE_RC}")
    if(NOT _HIGH_SCALE_RC EQUAL 0)
        message(FATAL_ERROR
            "high-scale positive run failed (exit ${_HIGH_SCALE_RC}); log: ${RESULTS_DIR}/high_scale_positive.log")
    endif()
    if(_HIGH_SCALE_STDOUT MATCHES "benchmark failure\\(s\\)")
        message(FATAL_ERROR
            "high-scale positive run recorded a failure on a run that should be clean; "
            "log: ${RESULTS_DIR}/high_scale_positive.log")
    endif()
    run_verify_inventory(check-report "${HIGH_SCALE_INVENTORY_JSON}" "${HIGH_SCALE_REPORT}" --device cpu)
else()
    message(STATUS
        "--- high-scale D: positive five-row run skipped "
        "(set OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1 to run it; minutes per 16,384-body scene) ---")
endif()

message(STATUS "ovphysx high-scale diagnostics contract: PASSED (${HIGH_SCALE_TOTAL} hidden CPU rows)")
