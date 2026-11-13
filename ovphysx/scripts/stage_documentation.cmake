# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2 AC-3

if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    cmake_minimum_required(VERSION 3.16)
endif()
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/crossplatform_helpers.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/preprocess_docs.cmake")

# Shared by SDK installation, wheel staging, and the source-only CI check.
# ARTIFACT is sdk or wheel; HTML_DOCS may be empty for a local portable-only run.
function(stage_documentation_payload PYTHON PROJECT_ROOT PACKAGE_ROOT ARTIFACT HTML_DOCS REQUIRE_HTML)
    if(NOT ARTIFACT STREQUAL "sdk" AND NOT ARTIFACT STREQUAL "wheel")
        message(FATAL_ERROR "Unknown documentation artifact: ${ARTIFACT}")
    endif()
    preprocess_public_docs("${PYTHON}" "${PROJECT_ROOT}"
        "${PROJECT_ROOT}/docs" "${PACKAGE_ROOT}/docs")
    copy_file_if_different("${PROJECT_ROOT}/SKILLS.md" "${PACKAGE_ROOT}/SKILLS.md")
    copy_tree_if_different("${PROJECT_ROOT}/skills" "${PACKAGE_ROOT}/skills")

    if(IS_DIRECTORY "${HTML_DOCS}")
        copy_tree_if_different("${HTML_DOCS}" "${PACKAGE_ROOT}/docs/html")
    elseif(REQUIRE_HTML)
        message(FATAL_ERROR "Required rendered documentation not found: ${HTML_DOCS}")
    elseif(NOT HTML_DOCS STREQUAL "")
        message(WARNING "Rendered documentation not found at ${HTML_DOCS}; this artifact contains portable documentation only")
    else()
        message(STATUS "Rendered documentation omitted; staging portable documentation only")
    endif()

    set(SAMPLES_DST "${PACKAGE_ROOT}/samples")
    file(MAKE_DIRECTORY "${SAMPLES_DST}")
    if(ARTIFACT STREQUAL "sdk")
        execute_process(
            COMMAND "${PYTHON}" "${PROJECT_ROOT}/scripts/preprocess_markdown.py"
                "${PROJECT_ROOT}/SDK_README.md" "${PACKAGE_ROOT}/README.md"
                --project-root "${PROJECT_ROOT}"
            RESULT_VARIABLE _README_RESULT)
        if(NOT _README_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to preprocess SDK_README.md")
        endif()
        # Preserve the SDK's existing removal of stale Python samples.
        if(EXISTS "${SAMPLES_DST}/python_samples")
            file(REMOVE_RECURSE "${SAMPLES_DST}/python_samples")
        endif()
        copy_tree_if_different("${PROJECT_ROOT}/tests/c_samples" "${SAMPLES_DST}/c_samples"
            EXCLUDE_DIRS _build)
    else()
        stage_python_samples_tree("${PROJECT_ROOT}/tests/python_samples" "${SAMPLES_DST}/python_samples")
    endif()
    copy_tree_if_different("${PROJECT_ROOT}/tests/data" "${SAMPLES_DST}/data")
endfunction()

# Script-mode entry point used by test_documentation_package.py. Including this
# file from a packaging script only defines the shared function above.
if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    foreach(_REQUIRED PYTHON PROJECT_ROOT PACKAGE_ROOT ARTIFACT REQUIRE_HTML)
        if(NOT DEFINED ${_REQUIRED})
            message(FATAL_ERROR "Missing -D${_REQUIRED}")
        endif()
    endforeach()
    stage_documentation_payload("${PYTHON}" "${PROJECT_ROOT}" "${PACKAGE_ROOT}"
        "${ARTIFACT}" "${HTML_DOCS}" "${REQUIRE_HTML}")
    if(ARTIFACT STREQUAL "sdk")
        include("${PROJECT_ROOT}/cmake/public_headers.cmake")
        ovphysx_public_header_sources("${PROJECT_ROOT}" _HEADER_DIRS _HEADER_PATTERNS)
        file(COPY ${_HEADER_DIRS} DESTINATION "${PACKAGE_ROOT}/include"
            FILES_MATCHING ${_HEADER_PATTERNS})
    endif()
endif()
