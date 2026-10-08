# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-4
#
# Minimal `cmake -P` CLI around the recover()/begin()/commit() functions of
# scripts/benchmark_baseline_txn.cmake, for callers that are not cmake scripts (for example the
# measure-benchmark-change skill's shell workflow) but need the same crash-recoverable
# snapshot/restore protocol that scripts/test_benchmark_contract.cmake uses.
#
# Usage:
#   cmake -DBASELINE_TXN_ACTION=recover|begin|commit \
#         -DBASELINE_TXN_TARGET=<file to protect> \
#         -DBASELINE_TXN_BACKUP=<snapshot path> \
#         -DBASELINE_TXN_MARKER=<marker path> \
#         -P scripts/benchmark_baseline_txn_cli.cmake
#
# Run `recover` once before the first `begin` of a session. It is idempotent and a no-op when no
# transaction was left open by an interrupted run. Run `begin` before overwriting the target and
# `commit` afterward on every exit path, success or failure, to restore the developer's own file.

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
include("${SCRIPT_DIR}/benchmark_baseline_txn.cmake")

if(NOT BASELINE_TXN_ACTION)
    message(FATAL_ERROR "BASELINE_TXN_ACTION is required: recover, begin, or commit")
endif()

if(BASELINE_TXN_ACTION STREQUAL "recover")
    baseline_txn_recover()
elseif(BASELINE_TXN_ACTION STREQUAL "begin")
    baseline_txn_begin()
elseif(BASELINE_TXN_ACTION STREQUAL "commit")
    baseline_txn_commit()
else()
    message(FATAL_ERROR "Unknown BASELINE_TXN_ACTION '${BASELINE_TXN_ACTION}': expected recover, begin, or commit")
endif()
