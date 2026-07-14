# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Locate pip packages that were pre-installed into OVRUNTIME_TARGET_DEPS by
# pull_dependencies (ovruntime standalone) or pip_fetch.py (ovexts / ovphysx).
# This file does not fetch anything — all pip dependencies are pulled during
# the dependency-fetch phase, before CMake configure runs.

# ---------------------------------------------------------------------------
# Newton USD Schemas
# ---------------------------------------------------------------------------
# pull_dependencies (standalone) and pip_fetch.py (ovexts) both install
# newton-usd-schemas into OVRUNTIME_TARGET_DEPS/newton_prebundle.
if(EXISTS "${OVRUNTIME_TARGET_DEPS}/newton_prebundle")
    set(NEWTON_SCHEMAS_DIR "${OVRUNTIME_TARGET_DEPS}/newton_prebundle")
    message(STATUS "Found Newton USD schemas: ${NEWTON_SCHEMAS_DIR}")
else()
    message(WARNING "Newton USD schemas not found. Run pull_dependencies first.")
endif()

# ---------------------------------------------------------------------------
# Generate PXR token header from the Newton schema
# ---------------------------------------------------------------------------
# Runs once at configure time.  The generator only writes the file when
# content actually changes, so incremental builds are not affected.
set(NEWTON_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_build/generated")
set(NEWTON_SCHEMA_TOKENS_HEADER "${NEWTON_GENERATED_DIR}/NewtonSchemaTokens.h")

if(NEWTON_SCHEMAS_DIR)
    set(_newton_usda "${NEWTON_SCHEMAS_DIR}/newton_usd_schemas/generatedSchema.usda")
    set(_gen_script "${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_newton_tokens.py")

    if(EXISTS "${_newton_usda}" AND EXISTS "${_gen_script}")
        if(NOT EXISTS "${NEWTON_SCHEMA_TOKENS_HEADER}")
            message(STATUS "Generating Newton schema tokens header ...")
        endif()

        # Resolve the Python executable
        if(WIN32)
            set(_gen_python "${PYTHON_DIR}/python.exe")
        else()
            set(_gen_python "${PYTHON_DIR}/python")
        endif()

        execute_process(
            COMMAND "${_gen_python}" "${_gen_script}" "${_newton_usda}"
                    -o "${NEWTON_SCHEMA_TOKENS_HEADER}"
            RESULT_VARIABLE _gen_result
        )
        if(_gen_result EQUAL 0)
            message(STATUS "Newton schema tokens header: ${NEWTON_SCHEMA_TOKENS_HEADER}")
        else()
            message(WARNING "Newton schema token generation failed (exit ${_gen_result})")
        endif()
    else()
        if(NOT EXISTS "${_newton_usda}")
            message(WARNING "Newton schema USDA not found at ${_newton_usda}")
        endif()
    endif()
endif()
