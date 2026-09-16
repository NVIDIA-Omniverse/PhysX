# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2

# Collect public portable-documentation paths relative to DOCS_SRC.
# Markdown is preprocessed. RST and image assets are copied verbatim.
function(ovphysx_collect_public_doc_paths DOCS_SRC OUT_MARKDOWN OUT_RAW)
    file(GLOB_RECURSE _PUBLIC_DOCS_MD RELATIVE "${DOCS_SRC}"
        "${DOCS_SRC}/*.md"
    )
    file(GLOB_RECURSE _PUBLIC_DOCS_RAW RELATIVE "${DOCS_SRC}"
        "${DOCS_SRC}/*.rst"
        "${DOCS_SRC}/*.png"
        "${DOCS_SRC}/*.jpg"
    )

    list(FILTER _PUBLIC_DOCS_MD EXCLUDE REGEX "(^|/)internal/")
    list(FILTER _PUBLIC_DOCS_RAW EXCLUDE REGEX "(^|/)internal/")
    list(SORT _PUBLIC_DOCS_MD)
    list(SORT _PUBLIC_DOCS_RAW)

    set(${OUT_MARKDOWN} "${_PUBLIC_DOCS_MD}" PARENT_SCOPE)
    set(${OUT_RAW} "${_PUBLIC_DOCS_RAW}" PARENT_SCOPE)
endfunction()

# Preprocess and copy public portable documentation. Resolves {literalinclude},
# strips MyST-only blocks ({toctree}, {eval-rst}), fails on unhandled directives,
# and preserves RST/image paths. Called by install.cmake and build_wheel.cmake.
#
# Arguments:
#   PYTHON        - path to Python interpreter
#   PROJECT_ROOT  - project root (for --project-root boundary check)
#   DOCS_SRC      - source docs directory (e.g. ${PROJECT_ROOT}/docs)
#   DOCS_DST      - destination directory for preprocessed docs
function(preprocess_public_docs PYTHON PROJECT_ROOT DOCS_SRC DOCS_DST)
    set(PREPROCESS_SCRIPT "${PROJECT_ROOT}/scripts/preprocess_markdown.py")

    file(MAKE_DIRECTORY "${DOCS_DST}")

    # Remove the docs/markdown/ directory left behind by the previous layout.
    if(IS_DIRECTORY "${DOCS_DST}/markdown")
        file(REMOVE_RECURSE "${DOCS_DST}/markdown")
        message(STATUS "  Removed stale docs/markdown/ directory")
    endif()

    ovphysx_collect_public_doc_paths(
        "${DOCS_SRC}" _PUBLIC_DOCS_MD _PUBLIC_DOCS_RAW)

    foreach(REL_MD IN LISTS _PUBLIC_DOCS_MD)
        get_filename_component(_REL_MD_DIR "${REL_MD}" DIRECTORY)
        if(_REL_MD_DIR)
            file(MAKE_DIRECTORY "${DOCS_DST}/${_REL_MD_DIR}")
        endif()
        execute_process(
            COMMAND "${PYTHON}" "${PREPROCESS_SCRIPT}"
                "${DOCS_SRC}/${REL_MD}" "${DOCS_DST}/${REL_MD}"
                --project-root "${PROJECT_ROOT}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE _PREPROCESS_RESULT
        )
        if(NOT _PREPROCESS_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to preprocess ${REL_MD}")
        endif()
    endforeach()

    foreach(REL_RAW IN LISTS _PUBLIC_DOCS_RAW)
        copy_file_if_different(
            "${DOCS_SRC}/${REL_RAW}" "${DOCS_DST}/${REL_RAW}")
    endforeach()
endfunction()
