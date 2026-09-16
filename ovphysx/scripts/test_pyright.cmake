# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-6
#
# Static type-check of the ovphysx Python stubs (PEP 561 .pyi + py.typed).
# Does not import native code or require ovstage, only the python/ dev group.
# Usage: cmake -P scripts/test_pyright.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/build_common.cmake")

set(PYTHON_PKG_DIR "${PROJECT_ROOT}/python")

# `uv sync --frozen` below is a hard error without a lock, not a no-op, so a tree
# without one skips instead of aborting. Both the internal tree and the public drop
# carry python/uv.lock, so this is a fallback rather than the normal path.
if(NOT EXISTS "${PYTHON_PKG_DIR}/uv.lock")
    message(STATUS "  [SKIP] pyright: python/uv.lock is not in this source tree")
    return()
endif()

if(NOT OVPHYSX_UV_COMMAND)
    message(FATAL_ERROR
        "uv not found. Install it: https://docs.astral.sh/uv/getting-started/installation/")
endif()

execute_process(
    COMMAND "${OVPHYSX_UV_COMMAND}" --version
    OUTPUT_VARIABLE _UV_VERSION
    ERROR_QUIET
    RESULT_VARIABLE _UV_CHECK
)
if(NOT _UV_CHECK EQUAL 0)
    message(FATAL_ERROR "uv --version failed (exit ${_UV_CHECK})")
endif()
string(STRIP "${_UV_VERSION}" _UV_VERSION)
message(STATUS "Pyright: using uv (${_UV_VERSION})")

set(UV_CACHE_DIR_PATH "${PROJECT_ROOT}/_build/.uv_cache")
file(MAKE_DIRECTORY "${UV_CACHE_DIR_PATH}")
set(UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    "UV_HTTP_TIMEOUT=300"
)

# Sync pinned dev deps only. Installing the ovphysx project would pull the exact
# ovstage wheel pair, and pyright only needs the checked-in .pyi tree.
# --frozen is required because ci_validate runs before fetch_deps. Without a local
# ovstage wheel, uv would re-resolve ovstage from CI's Artifactory index and fail.
#
# --locked does not work here. On every CI runner its freshness check invalidates
# the lock at this stage and re-resolves, reaching the index and failing on ovstage
# even though the lock and pyproject.toml agree. Agreement between
# python/pyproject.toml and python/uv.lock is enforced by a checked-in test
# instead, which reads both as text.
message(STATUS "Syncing pinned pyright dev dependencies (no ovphysx/ovstage install)")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV}
        "${OVPHYSX_UV_COMMAND}" sync --only-group dev --no-install-project --frozen
    WORKING_DIRECTORY "${PYTHON_PKG_DIR}"
    RESULT_VARIABLE _SYNC_RESULT
    OUTPUT_VARIABLE _SYNC_STDOUT
    ERROR_VARIABLE _SYNC_STDERR
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
)
if(NOT _SYNC_RESULT EQUAL 0)
    # uv also fails here for reasons unrelated to resolution, such as a malformed lock,
    # so the message points at uv's own output rather than asserting one cause.
    message(FATAL_ERROR "uv sync --only-group dev failed (exit ${_SYNC_RESULT}); see the "
        "uv output above -- an unsatisfiable ovstage requirement there means uv "
        "re-resolved instead of reading python/uv.lock.")
endif()

message(STATUS "Running pinned pyright on ovphysx stub tree")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV}
        "${OVPHYSX_UV_COMMAND}" run --no-sync pyright
    WORKING_DIRECTORY "${PYTHON_PKG_DIR}"
    RESULT_VARIABLE _PYRIGHT_RESULT
    OUTPUT_VARIABLE _PYRIGHT_STDOUT
    ERROR_VARIABLE _PYRIGHT_STDERR
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
)
if(NOT _PYRIGHT_RESULT EQUAL 0)
    message(FATAL_ERROR "pyright failed (exit ${_PYRIGHT_RESULT})")
endif()

message(STATUS "  [OK] pyright passed")
