# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Keep test schemas outside the product package.
include("${CMAKE_CURRENT_LIST_DIR}/dev_schema_provider.cmake")
if(OVPHYSX_DEV_SCHEMA_PROVIDER)
    return()
endif()
# The installer needs only stdlib; the active CI interpreter is sufficient.
find_package(Python3 3.10 COMPONENTS Interpreter REQUIRED)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_ROOT}/tests/c_samples/common/fetch_physx_schemas.py"
            --cache-dir "${BUILD_PATH}/test-deps"
    RESULT_VARIABLE _SCHEMA_RESULT
    OUTPUT_VARIABLE _SCHEMA_PROVIDER
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT _SCHEMA_RESULT EQUAL 0)
    message(FATAL_ERROR "External PhysX schema provider installation failed")
endif()
set(ENV{PHYSX_USD_SCHEMAS_ROOT} "${_SCHEMA_PROVIDER}")
message(STATUS "Using locked schema provider: ${_SCHEMA_PROVIDER}")
