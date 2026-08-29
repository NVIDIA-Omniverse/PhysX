# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Shared function: preprocess and copy public Markdown docs.
# Resolves {literalinclude}, strips MyST-only blocks ({toctree}, {eval-rst}),
# and errors on unhandled directives. Called by install.cmake and build_wheel.cmake.
#
# Arguments:
#   PYTHON        - path to Python interpreter
#   PROJECT_ROOT  - project root (for --project-root boundary check)
#   DOCS_SRC      - source docs directory (e.g. ${PROJECT_ROOT}/docs)
#   DOCS_DST      - destination directory for preprocessed docs
function(preprocess_public_docs PYTHON PROJECT_ROOT DOCS_SRC DOCS_DST)
    set(PREPROCESS_SCRIPT "${PROJECT_ROOT}/scripts/preprocess_markdown.py")

    file(MAKE_DIRECTORY "${DOCS_DST}")

    # Remove stale docs/markdown/ from previous layout.
    if(IS_DIRECTORY "${DOCS_DST}/markdown")
        file(REMOVE_RECURSE "${DOCS_DST}/markdown")
        message(STATUS "  Removed stale docs/markdown/ directory")
    endif()

    file(GLOB _PUBLIC_DOCS_MD RELATIVE "${DOCS_SRC}"
        "${DOCS_SRC}/*.md"
        "${DOCS_SRC}/tutorials/*.md"
    )

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
endfunction()
