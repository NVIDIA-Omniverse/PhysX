# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED TEST_ROOT OR NOT DEFINED HELPERS)
    message(FATAL_ERROR "TEST_ROOT and HELPERS are required")
endif()

include("${HELPERS}")

set(_src "${TEST_ROOT}/python_samples")
set(_dst "${TEST_ROOT}/staged/python_samples")

file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY "${_src}/.venv/bin" "${_src}/__pycache__" "${_src}/.pytest_cache")
file(WRITE "${_src}/hello_world.py" "print('hello')\n")
file(WRITE "${_src}/uv.lock" "version = 1\n")
file(WRITE "${_src}/.venv/pyvenv.cfg" "home = /nowhere\n")
file(WRITE "${_src}/__pycache__/hello_world.pyc" "junk")
file(WRITE "${_src}/.pytest_cache/CACHEDIR.TAG" "Signature: 8a477f597d28d172\n")
# The reported failure (NVBug 6543059): .venv/bin/python resolves outside the
# repository, so copying it aborts the wheel build.  Symlink creation needs a
# privilege Windows does not grant by default, so it is best-effort here; the
# assertion below holds on every platform.
file(CREATE_LINK "${TEST_ROOT}/no-such-interpreter" "${_src}/.venv/bin/python"
    SYMBOLIC RESULT _link_result)

stage_python_samples_tree("${_src}" "${_dst}")

file(GLOB_RECURSE _staged RELATIVE "${_dst}" "${_dst}/*")
if(NOT "${_staged}" STREQUAL "hello_world.py")
    message(FATAL_ERROR "Unexpected wheel staging result: ${_staged}")
endif()
