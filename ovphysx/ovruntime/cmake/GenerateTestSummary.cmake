# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# ---------------------------------------------------------------------------
# C++ doctest catalogues (TEST_SUMMARY.md)
# ---------------------------------------------------------------------------
# The per-tree TEST_SUMMARY.md catalogues are a build artifact, NOT a checked-in
# file: a committed generated file produced a merge conflict on nearly every MR
# that touched a test. Instead we regenerate them once per build into
# _build/generated/test-summaries/<slug>/TEST_SUMMARY.md. Only the analysis
# side-car (tools/repoman/test_summary_analysis.json) stays source-controlled.

set(_ts_script "${CMAKE_CURRENT_SOURCE_DIR}/tools/repoman/get_test_summary.py")
set(_ts_out_dir "${CMAKE_CURRENT_SOURCE_DIR}/_build/generated/test-summaries")

if(WIN32)
    set(_ts_python "${PYTHON_DIR}/python.exe")
else()
    set(_ts_python "${PYTHON_DIR}/python")
endif()

if(EXISTS "${_ts_script}" AND EXISTS "${_ts_python}")
    # A custom target (no OUTPUT/BYPRODUCTS) is always considered out of date, so it
    # runs exactly once per build — a single invocation regenerates all three trees.
    # It has no build dependencies (the catalogue derives from the .cpp sources, not
    # from compiled artifacts), so it runs in parallel and never gates the test build.
    add_custom_target(generate_test_summary ALL
        COMMAND "${_ts_python}" "${_ts_script}" --out-dir "${_ts_out_dir}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Generating C++ test catalogues (TEST_SUMMARY.md) into _build/generated/test-summaries"
        VERBATIM
    )
else()
    if(NOT EXISTS "${_ts_python}")
        message(WARNING "Python not found (${_ts_python}); skipping TEST_SUMMARY.md generation")
    endif()
endif()
