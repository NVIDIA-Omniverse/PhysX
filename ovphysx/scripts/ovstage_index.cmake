# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-2, AC-3
#
# uv environment for OVStage resolution with UV_NO_CONFIG=1 (ADR-0033).
# PACKAGE_ONLY limits unlocked installs to the ovstage package page; uv pip ignores
# tool.uv.sources, so exposing the whole index could shadow unrelated PyPI packages.
# PROJECT selects the index source (default: tests/python_tests).

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

    set(_env "UV_INDEX_URL=https://pypi.org/simple")
    if(_ovsi_PACKAGE_ONLY)
        string(REGEX REPLACE "/+$" "" _index "${CMAKE_MATCH_1}")
        list(APPEND _env "UV_EXTRA_INDEX_URL=" "UV_FIND_LINKS=${_index}/ovstage/")
    else()
        list(APPEND _env "UV_EXTRA_INDEX_URL=${CMAKE_MATCH_1}")
    endif()
    set(${out_var} ${_env} PARENT_SCOPE)
endfunction()
