# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-6

execute_process(
    COMMAND "${TARGET_PYTHON}" "${SCHEMA_DIR}/scripts/build_wheel.py"
    WORKING_DIRECTORY "${SCHEMA_DIR}"
    RESULT_VARIABLE _DEV_SCHEMA_WHEEL_RESULT
)
if(NOT _DEV_SCHEMA_WHEEL_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to build the separate devschema provider wheel")
endif()
file(GLOB _DEV_SCHEMA_WHEELS "${SCHEMA_DIR}/_dist/physx_usd_schemas-*.whl")
list(LENGTH _DEV_SCHEMA_WHEELS _DEV_SCHEMA_WHEEL_COUNT)
if(NOT _DEV_SCHEMA_WHEEL_COUNT EQUAL 1)
    message(FATAL_ERROR "Expected one locally built physx-usd-schemas wheel, found ${_DEV_SCHEMA_WHEEL_COUNT}")
endif()
list(GET _DEV_SCHEMA_WHEELS 0 _DEV_SCHEMA_WHEEL)
file(SHA256 "${_DEV_SCHEMA_WHEEL}" _DEV_SCHEMA_SHA256)
# Digest-specific paths distinguish same-version edits and preserve live data.
set(_DEV_SCHEMA_PROVIDER "${BUILD_PATH}/test-deps/physx-usd-schemas-dev/${_DEV_SCHEMA_SHA256}")
execute_process(
    COMMAND "${TARGET_PYTHON}" "${PROJECT_ROOT}/tests/c_samples/common/fetch_physx_schemas.py"
            --wheel "${_DEV_SCHEMA_WHEEL}" --sha256 "${_DEV_SCHEMA_SHA256}"
            --dest "${_DEV_SCHEMA_PROVIDER}"
    RESULT_VARIABLE _DEV_SCHEMA_INSTALL_RESULT
)
if(NOT _DEV_SCHEMA_INSTALL_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to install the separate devschema provider")
endif()
file(WRITE "${OVPHYSX_DEV_SCHEMA_STAMP}" "${_DEV_SCHEMA_PROVIDER}\n")
