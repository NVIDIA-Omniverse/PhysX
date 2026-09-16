# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ---------------------------------------------------------------------------
# C++ benchmark catalogue (BENCHMARK_SUMMARY.md)
# ---------------------------------------------------------------------------
# Sibling of GenerateTestSummary.cmake, for the omni.physx C++ benchmark suite. Same rationale: the
# catalogue is a build artifact, NOT a checked-in file, regenerated once per build into
# _build/generated/benchmark-summaries/omni.physx/BENCHMARK_SUMMARY.md. Only the analysis side-car
# (tools/repoman/benchmark_summary_analysis.json) stays source-controlled.

set(_bs_script "${CMAKE_CURRENT_SOURCE_DIR}/tools/repoman/get_benchmark_summary.py")
set(_bs_out_dir "${CMAKE_CURRENT_SOURCE_DIR}/_build/generated/benchmark-summaries")

if(WIN32)
    set(_bs_python "${PYTHON_DIR}/python.exe")
else()
    set(_bs_python "${PYTHON_DIR}/python")
endif()

if(EXISTS "${_bs_script}" AND EXISTS "${_bs_python}")
    # A custom target (no OUTPUT/BYPRODUCTS) is always considered out of date, so it runs exactly
    # once per build. It has no build dependencies (the catalogue derives from the .cpp sources, not
    # from compiled artifacts), so it runs in parallel and never gates the test build.
    #
    # Namespaced (not bare "generate_benchmark_summary"): ovphysx adds this directory via
    # add_subdirectory() and defines its own bare-named target of the same kind when
    # OVPHYSX_BUILD_BENCHMARKS=ON. Both being enabled in one configure would otherwise collide.
    # --check is part of the normal generation path (not opt-in): a registered row missing an
    # analysis note, or a side-car key that no longer matches a registered row, fails the build
    # here instead of shipping a silently drifted catalogue.
    add_custom_target(ovruntime_generate_benchmark_summary ALL
        COMMAND "${_bs_python}" "${_bs_script}" --check --out-dir "${_bs_out_dir}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Generating C++ benchmark catalogue (BENCHMARK_SUMMARY.md) into _build/generated/benchmark-summaries"
        VERBATIM
    )
else()
    if(NOT EXISTS "${_bs_python}")
        message(WARNING "Python not found (${_bs_python}); skipping BENCHMARK_SUMMARY.md generation")
    endif()
endif()
