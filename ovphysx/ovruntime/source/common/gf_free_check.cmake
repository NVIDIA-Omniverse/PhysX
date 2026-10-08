# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-MATH-001
# @covers AC-12
#
# CI gate: files that must not (re)acquire pxr/Gf. Two modes, because the two
# kinds of file being guarded have different budgets.
#
# ---------------------------------------------------------------------------
# Mode 1 -- GF_FREE_FILES: no pxr AT ALL.
#
# `MatrixTools.{h,cpp}` and `DeformableCookingTransform.{h,cpp}` are the runtime's
# replacements for pxr's Gf math -- MatrixTools' `gfmath` namespace is a verbatim,
# bit-exact transcription of the pxr routines whose outputs are hashed into the
# persistent Ujitso cooking cache key. Reintroducing a real Gf call anywhere in
# them would both re-link USD and, because Gf and the transcription are only
# byte-identical while nobody edits either, quietly reopen the cache-rekey risk
# these files exist to close. This gate makes that reintroduction a build error
# rather than a review question.
#
# The pattern deliberately matches UNQUALIFIED `GfXxx` / `VtArray` spellings as
# well as `pxr/` and `PXR_NS::`: these files are compiled with the UsdPCH.h force
# include in some configurations, so a bare `GfMatrix4d` compiles without any
# visible pxr token. Matching only `pxr/` would report a false pass.
#
# ---------------------------------------------------------------------------
# Mode 2 -- GF_SYMBOL_FREE_FILES (+ optional GF_ALLOWED_SYMBOLS): no Gf.
#
# For USD-facing runtime files that legitimately keep the rest of pxr (`UsdPrim`,
# `SdfPath`, `TfToken`, `VtArray`) but must stay off Gf, because Gf is what makes
# `OvruntimePhysX` link the USD math library that the USD-free build has to drop.
# `pxr/` and `PXR_NS` are therefore NOT violations here -- only `GfXxx`.
#
# `GF_ALLOWED_SYMBOLS` is the escape hatch for Gf *value types* that are element
# layout only and pull no Gf library symbol of their own: a `VtArray<GfVec3f>`
# written straight to a USD attribute costs Vt and Usd symbols, not Gf ones.
# Keep the list minimal, and shrink it -- an entry is a statement that the file
# still has USD authoring that has not moved behind the write sink yet.
#
# Invoked as:
#   cmake -DGF_FREE_FILES=<;-separated list> -P gf_free_check.cmake
#   cmake -DGF_SYMBOL_FREE_FILES=<list> [-DGF_ALLOWED_SYMBOLS=<list>] -P gf_free_check.cmake

if(NOT DEFINED GF_FREE_FILES AND NOT DEFINED GF_SYMBOL_FREE_FILES)
    message(FATAL_ERROR "GF_FREE_FILES or GF_SYMBOL_FREE_FILES must be defined")
endif()

# Find the index of the unescaped closing quote/apostrophe that terminates a
# string or character literal starting at index _start (the first character
# inside the literal, i.e. right after the opening quote). Honors backslash
# escapes so `\"` / `\\` do not end the literal early. Returns -1 if the
# literal is never closed before EOF.
function(_gf_scan_literal _content _start _quote _out)
    string(LENGTH "${_content}" _len)
    set(_search ${_start})
    set(_found -1)
    while(_search LESS _len)
        string(SUBSTRING "${_content}" ${_search} -1 _tail)
        string(FIND "${_tail}" "${_quote}" _rel)
        if(_rel EQUAL -1)
            break()
        endif()
        math(EXPR _idx "${_search} + ${_rel}")
        set(_bs_count 0)
        set(_probe ${_idx})
        while(_probe GREATER ${_start})
            math(EXPR _probe "${_probe} - 1")
            string(SUBSTRING "${_content}" ${_probe} 1 _ch)
            if(_ch STREQUAL "\\")
                math(EXPR _bs_count "${_bs_count} + 1")
            else()
                break()
            endif()
        endwhile()
        math(EXPR _bs_parity "${_bs_count} % 2")
        if(_bs_parity EQUAL 0)
            set(_found ${_idx})
            break()
        endif()
        math(EXPR _search "${_idx} + 1")
    endwhile()
    set(${_out} ${_found} PARENT_SCOPE)
endfunction()

# Read a source file and strip its comments: every one of these files documents
# what its Gf original was called, and that prose is the point, not a violation.
#
# A single-pass, state-aware scan rather than a pair of regexes: a POSIX ERE
# `/\*...*\/` cannot be told to stop at the FIRST `*/`, and a bare `//` regex
# cannot tell a real line comment from one spelled inside a string literal.
# String and character literals are copied through untouched -- including any
# Gf spelling they contain -- because only comment-boundary detection is being
# fixed here, not string-content stripping.
#
# Known gaps, both absent from every currently gated file: C++11 raw string
# literals (`R"(...)"`) are not special-cased, so a `)"` inside one that looks
# like a delimiter would desync the scan; and an unmatched digit-separator `'`
# is treated as an unterminated char literal, which fails closed (over-reports
# rather than silently missing a violation).
function(_gf_read_code _path _out)
    if(NOT EXISTS "${_path}")
        message(FATAL_ERROR "gf_free_check: file does not exist: ${_path}")
    endif()
    file(READ "${_path}" _content)
    string(LENGTH "${_content}" _len)

    set(_code "")
    set(_pos 0)
    while(_pos LESS _len)
        string(SUBSTRING "${_content}" ${_pos} -1 _rest)
        string(FIND "${_rest}" "//" _i_line)
        string(FIND "${_rest}" "/*" _i_block)
        string(FIND "${_rest}" "\"" _i_dq)
        string(FIND "${_rest}" "'" _i_sq)

        set(_min -1)
        foreach(_cand IN ITEMS ${_i_line} ${_i_block} ${_i_dq} ${_i_sq})
            if(_cand GREATER -1 AND (_min EQUAL -1 OR _cand LESS _min))
                set(_min ${_cand})
            endif()
        endforeach()

        if(_min EQUAL -1)
            string(APPEND _code "${_rest}")
            set(_pos ${_len})
        else()
            if(_min GREATER 0)
                string(SUBSTRING "${_rest}" 0 ${_min} _prefix)
                string(APPEND _code "${_prefix}")
            endif()
            math(EXPR _delim_pos "${_pos} + ${_min}")

            if(_i_block GREATER -1 AND _i_block EQUAL _min)
                # Block comment: terminate at the FIRST legal "*/", searched from
                # just after the opening "/*" -- the failure mode this replaces
                # let a `/***/` comment's own ending be skipped past.
                string(SUBSTRING "${_content}" ${_delim_pos} -1 _from_open)
                string(SUBSTRING "${_from_open}" 2 -1 _after_open)
                string(FIND "${_after_open}" "*/" _rel_close)
                if(_rel_close EQUAL -1)
                    set(_pos ${_len})
                else()
                    math(EXPR _pos "${_delim_pos} + 2 + ${_rel_close} + 2")
                endif()
            elseif(_i_line GREATER -1 AND _i_line EQUAL _min)
                string(SUBSTRING "${_content}" ${_delim_pos} -1 _from_open)
                string(FIND "${_from_open}" "\n" _rel_nl)
                if(_rel_nl EQUAL -1)
                    set(_pos ${_len})
                else()
                    string(APPEND _code "\n")
                    math(EXPR _pos "${_delim_pos} + ${_rel_nl} + 1")
                endif()
            else()
                # String or char literal: a `//`/`/*` look-alike inside one must
                # not be read as a comment start.
                if(_i_dq GREATER -1 AND _i_dq EQUAL _min)
                    set(_quote "\"")
                else()
                    set(_quote "'")
                endif()
                string(APPEND _code "${_quote}")
                math(EXPR _lit_start "${_delim_pos} + 1")
                _gf_scan_literal("${_content}" ${_lit_start} "${_quote}" _close_idx)
                if(_close_idx EQUAL -1)
                    string(SUBSTRING "${_content}" ${_lit_start} -1 _lit_body)
                    string(APPEND _code "${_lit_body}")
                    set(_pos ${_len})
                else()
                    math(EXPR _lit_len "${_close_idx} - ${_lit_start} + 1")
                    string(SUBSTRING "${_content}" ${_lit_start} ${_lit_len} _lit_body)
                    string(APPEND _code "${_lit_body}")
                    math(EXPR _pos "${_close_idx} + 1")
                endif()
            endif()
        endif()
    endwhile()

    set(${_out} "${_code}" PARENT_SCOPE)
endfunction()

set(_offenders "")
foreach(_f IN LISTS GF_FREE_FILES)
    _gf_read_code("${_f}" _code)
    if(_code MATCHES "pxr/"
       OR _code MATCHES "PXR_NS"
       OR _code MATCHES "PXR_NAMESPACE"
       OR _code MATCHES "(^|[^A-Za-z0-9_])Gf[A-Z][A-Za-z0-9_]*"
       OR _code MATCHES "(^|[^A-Za-z0-9_])VtArray")
        list(APPEND _offenders "${_f}")
    endif()
endforeach()

if(_offenders)
    message("ERROR: pxr/Gf spellings found in the ported Gf math, which must stay pxr-free:")
    foreach(_f IN LISTS _offenders)
        message("  ${_f}")
    endforeach()
    message(FATAL_ERROR "Gf reintroduced into common/foundation math")
endif()

set(_symbol_offenders "")
foreach(_f IN LISTS GF_SYMBOL_FREE_FILES)
    _gf_read_code("${_f}" _code)
    # Collect every Gf spelling, then subtract the allowlist. Normalize every
    # non-identifier separator to a single space first: MATCHALL captures the
    # leading delimiter inside each match, and CMake stores matches as a
    # ;-separated list, so when the delimiter itself is a ';' the match
    # becomes a two-element list whose empty half bypasses the allowlist
    # check below and gets reported with a blank symbol name.
    string(REGEX REPLACE "[^A-Za-z0-9_]" " " _normalized_code "${_code}")
    string(REGEX MATCHALL "(^| )Gf[A-Z][A-Za-z0-9_]*" _hits "${_normalized_code}")
    foreach(_hit IN LISTS _hits)
        string(REGEX REPLACE "^.*(Gf[A-Z][A-Za-z0-9_]*)$" "\\1" _sym "${_hit}")
        # list(FIND) rather than IN_LIST: script mode (`cmake -P`) runs without
        # a project()'s policy defaults, so CMP0057 is not set here.
        list(FIND GF_ALLOWED_SYMBOLS "${_sym}" _allowed_idx)
        if(_allowed_idx EQUAL -1)
            list(APPEND _symbol_offenders "${_f}: ${_sym}")
        endif()
    endforeach()
endforeach()

if(_symbol_offenders)
    list(REMOVE_DUPLICATES _symbol_offenders)
    message("ERROR: Gf spellings found in files that must stay Gf-free "
            "(USD authoring belongs behind the omni.physics.usd write sink):")
    foreach(_o IN LISTS _symbol_offenders)
        message("  ${_o}")
    endforeach()
    message(FATAL_ERROR "Gf reintroduced into a Gf-free runtime file")
endif()

if(GF_FREE_FILES)
    message(STATUS "OK: the ported Gf math (MatrixTools, DeformableCookingTransform) is pxr-free")
endif()
if(GF_SYMBOL_FREE_FILES)
    message(STATUS "OK: the guarded runtime files are Gf-free")
endif()
