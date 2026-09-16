# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-MATH-001
# @covers AC-8
#
# CI gate: the `gfmath` region of MatrixTools.{h,cpp} is a bit-exact
# transcription of third-party OpenUSD source living inside an otherwise
# original-NVIDIA file (see `docs/internal/third_party_openusd_gfmath.md`
# and REQ-MATH-001 AC-8). It must stay bracketed by REUSE-style SPDX-Snippet
# markers that record that region's upstream provenance, so an automated
# SPDX/REUSE scanner -- not just a human reading the banner prose -- can see
# the attribution Apache-2.0 Section 4(c)/(d) requires. This
# gate turns "someone edits the file later and drops the markers" into a build
# error instead of a future review re-discovery.
#
# Invoked as:
#   cmake -DSPDX_SNIPPET_FILES=<;-separated list> -P spdx_snippet_check.cmake

if(NOT DEFINED SPDX_SNIPPET_FILES)
    message(FATAL_ERROR "SPDX_SNIPPET_FILES must be defined")
endif()

set(_offenders "")
foreach(_f IN LISTS SPDX_SNIPPET_FILES)
    if(NOT EXISTS "${_f}")
        message(FATAL_ERROR "spdx_snippet_check: file does not exist: ${_f}")
    endif()
    file(READ "${_f}" _content)

    if(NOT _content MATCHES "SPDX-SnippetBegin")
        list(APPEND _offenders "${_f}: missing SPDX-SnippetBegin")
        continue()
    endif()
    if(NOT _content MATCHES "SPDX-SnippetEnd")
        list(APPEND _offenders "${_f}: missing SPDX-SnippetEnd")
        continue()
    endif()
    if(NOT _content MATCHES "SPDX-License-Identifier: Apache-2.0")
        list(APPEND _offenders "${_f}: missing SPDX-License-Identifier: Apache-2.0 snippet")
        continue()
    endif()

    # The markers must bracket `namespace gfmath ... } // namespace gfmath`:
    # Begin before the namespace opens, End after it closes.
    string(FIND "${_content}" "SPDX-SnippetBegin" _begin_idx)
    string(FIND "${_content}" "namespace gfmath" _ns_open_idx)
    string(FIND "${_content}" "} // namespace gfmath" _ns_close_idx)
    string(FIND "${_content}" "SPDX-SnippetEnd" _end_idx)

    if(_ns_open_idx EQUAL -1 OR _ns_close_idx EQUAL -1)
        list(APPEND _offenders "${_f}: could not locate the gfmath namespace boundaries")
        continue()
    endif()

    if(NOT (_begin_idx LESS _ns_open_idx AND _ns_close_idx LESS _end_idx))
        list(APPEND _offenders "${_f}: SPDX-Snippet markers do not bracket namespace gfmath")
    endif()
endforeach()

if(_offenders)
    message("ERROR: the OpenUSD-derived gfmath transcription lost its SPDX-Snippet attribution markers:")
    foreach(_o IN LISTS _offenders)
        message("  ${_o}")
    endforeach()
    message(FATAL_ERROR "SPDX-Snippet markers missing or misplaced around namespace gfmath")
endif()

message(STATUS "OK: the gfmath transcription's SPDX-Snippet attribution markers are present")
