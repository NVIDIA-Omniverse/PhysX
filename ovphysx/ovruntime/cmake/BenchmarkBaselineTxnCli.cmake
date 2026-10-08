# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Minimal `cmake -P` CLI around cmake/BenchmarkBaselineTxn.cmake's recover()/begin()/commit()
# functions, for callers that are not themselves a cmake script (e.g. the measure-benchmark-change
# skill's shell workflow) but still need the same crash-recoverable snapshot/restore protocol.
# Sibling of ovphysx's scripts/benchmark_baseline_txn_cli.cmake.
#
# Usage:
#   cmake -DBASELINE_TXN_ACTION=recover|begin|commit \
#         -DBASELINE_TXN_TARGET=<file to protect> \
#         -DBASELINE_TXN_BACKUP=<snapshot path> \
#         -DBASELINE_TXN_MARKER=<marker path> \
#         -P cmake/BenchmarkBaselineTxnCli.cmake
#
# Always run `recover` once before the first `begin` of a session (idempotent, no-op if there is no
# open transaction from a prior interrupted run), `begin` before overwriting the target, and
# `commit` afterward -- on every exit path, success or failure -- to put the developer's own file
# back exactly.

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
include("${SCRIPT_DIR}/BenchmarkBaselineTxn.cmake")

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
