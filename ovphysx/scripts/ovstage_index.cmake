# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-2, AC-3
#
# uv environment for OVStage resolution with UV_NO_CONFIG=1 (ADR-0033).
#
# UV_NO_CONFIG ignores uv's configuration files, but a project's own explicit
# [[tool.uv.index]] entries and [tool.uv.sources] are still honored. A project run
# (`uv run`, `uv sync`) therefore needs no index from the environment: its pyproject
# routes ovstage and physx-usd-schemas to the explicit index and everything else
# comes from PyPI. The index must not be exported as an extra index. uv asks every
# extra index about every package it resolves, build-system requirements included,
# so a 5xx from an index that never hosted setuptools or wheel still fails the run.
# uv also reads UV_INDEX, which a runner image may export, so that is cleared too.
#
# PACKAGE_ONLY is for `uv pip`, which ignores tool.uv.sources and so does need the
# index location. It exposes only the ovstage package page, because exposing the whole
# index could shadow unrelated PyPI packages.
#
# PROJECT selects the pyproject that names the index (default: tests/python_tests).
# Both modes require it to name the `ovstage` index, so a project run cannot lose it silently.
#
# This sets package-source policy only. uv's retry count is deliberately left to the
# environment: retries are opt-in (ADR-0033), and CI sets UV_HTTP_RETRIES next to
# OVSTAGE_FETCH_RETRY_TIMEOUT. The resolver that bumps the OVStage pin draws the same
# line between package-source policy, which it clears, and transport settings, which
# it keeps.

function(ovphysx_ovstage_index_env out_var)
    cmake_parse_arguments(_ovsi "PACKAGE_ONLY" "PROJECT" "" ${ARGN})
    if(NOT _ovsi_PROJECT)
        set(_ovsi_PROJECT "tests/python_tests")
    endif()
    set(_pyproject "${PROJECT_ROOT}/${_ovsi_PROJECT}/pyproject.toml")
    if(NOT EXISTS "${_pyproject}")
        message(FATAL_ERROR "ovstage index source not found: ${_pyproject}")
    endif()

    # Match the url line of the [[tool.uv.index]] block named "ovstage".
    file(READ "${_pyproject}" _content)
    string(REGEX MATCH "\\[\\[tool\\.uv\\.index\\]\\][ \t\r\n]*name = \"ovstage\"[ \t\r\n]*url = \"([^\"]+)\""
           _matched "${_content}")
    if(NOT CMAKE_MATCH_1)
        message(FATAL_ERROR
            "Could not read the [[tool.uv.index]] named 'ovstage' from ${_pyproject}. "
            "It is the single source for the ovstage wheel index (ADR-0033).")
    endif()

    # Empty values also clear indexes inherited from the caller's environment.
    set(_env "UV_INDEX_URL=https://pypi.org/simple" "UV_DEFAULT_INDEX=" "UV_EXTRA_INDEX_URL=" "UV_INDEX=")
    if(_ovsi_PACKAGE_ONLY)
        string(REGEX REPLACE "/+$" "" _index "${CMAKE_MATCH_1}")
        list(APPEND _env "UV_FIND_LINKS=${_index}/ovstage/")
    endif()
    set(${out_var} ${_env} PARENT_SCOPE)
endfunction()
