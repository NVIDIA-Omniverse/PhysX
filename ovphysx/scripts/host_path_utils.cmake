# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Host Path Helpers
#
# Windows toolchain roots are discovered as native paths (vswhere, ProgramFiles),
# so they carry backslashes and usually spaces. Both trip up tools that receive a
# path on an unquoted command line.

include_guard(GLOBAL)

# Convert a path to CMake's forward-slash form.
function(ovphysx_normalize_host_path _path _out_var)
    file(TO_CMAKE_PATH "${_path}" _normalized)
    set(${_out_var} "${_normalized}" PARENT_SCOPE)
endfunction()

# Convert a path to the forward-slashed 8.3 short spelling Windows resolves for
# it, which is what CMake emits for a path it has to pass on a command line.
#
# The contract is "the spelling CMake will use", not "no spaces": when only some
# components have an 8.3 alias the result is mixed, and returning the long form
# instead would disagree with CMake just as badly. A result that still contains a
# space is therefore kept, with a warning.
#
# Falls back to the long form (also with a warning) when no short name resolves -
# 8.3 generation can be disabled per volume. That fallback is best effort and
# untested: a spaced path may still fail later, which is why the warning says how
# to get out of the situation rather than claiming it is handled.
function(ovphysx_space_free_host_path _path _out_var)
    ovphysx_normalize_host_path("${_path}" _normalized)

    if(NOT _normalized MATCHES " " OR NOT CMAKE_HOST_WIN32)
        set(${_out_var} "${_normalized}" PARENT_SCOPE)
        return()
    endif()

    file(TO_NATIVE_PATH "${_normalized}" _native)
    execute_process(
        COMMAND cmd /c for %I in ("${_native}") do @echo %~sI
        OUTPUT_VARIABLE _short
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _short_result
        ERROR_QUIET
    )

    if(NOT _short_result EQUAL 0 OR NOT _short)
        message(WARNING
            "No 8.3 short name available for:\n"
            "  ${_normalized}\n"
            "Falling back to the long form, which is not known to work for a path "
            "with spaces. If the CUDA configure fails, move the toolchain to a path "
            "without spaces or enable 8.3 name generation on this volume.")
        set(${_out_var} "${_normalized}" PARENT_SCOPE)
        return()
    endif()

    ovphysx_normalize_host_path("${_short}" _short_normalized)

    # Only take a short name that resolves to the same place.
    if(NOT EXISTS "${_short_normalized}")
        message(WARNING
            "8.3 short name does not resolve:\n"
            "  ${_normalized}\n"
            "  -> ${_short_normalized}\n"
            "Using the long form.")
        set(${_out_var} "${_normalized}" PARENT_SCOPE)
        return()
    endif()

    if(_short_normalized MATCHES " ")
        message(WARNING
            "8.3 short name still contains a space:\n"
            "  ${_short_normalized}\n"
            "Using it anyway, because it is the spelling CMake resolves too. If the "
            "CUDA configure fails, move the toolchain to a path without spaces.")
    endif()

    set(${_out_var} "${_short_normalized}" PARENT_SCOPE)
endfunction()
