# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-6

# PYTHONPATH gives the --devschema provider precedence over the installed wheel.
set(OVPHYSX_DEV_SCHEMA_PROVIDER "")
set(_DEV_SCHEMA_STAMP "${BUILD_PATH}/ovphysx_dev_schema.stamp")
if(EXISTS "${_DEV_SCHEMA_STAMP}")
    file(READ "${_DEV_SCHEMA_STAMP}" OVPHYSX_DEV_SCHEMA_PROVIDER)
    string(STRIP "${OVPHYSX_DEV_SCHEMA_PROVIDER}" OVPHYSX_DEV_SCHEMA_PROVIDER)
    file(GLOB _DEV_SCHEMA_METADATA "${OVPHYSX_DEV_SCHEMA_PROVIDER}/physx_usd_schemas-*.dist-info/METADATA")
    list(LENGTH _DEV_SCHEMA_METADATA _DEV_SCHEMA_METADATA_COUNT)
    if(NOT IS_ABSOLUTE "${OVPHYSX_DEV_SCHEMA_PROVIDER}" OR NOT _DEV_SCHEMA_METADATA_COUNT EQUAL 1)
        message(FATAL_ERROR "Missing local devschema provider. Rebuild with --devschema; refusing to use published schemas instead.")
    endif()
    foreach(_MODULE PhysxSchema OmniUsdPhysicsDeformableSchema)
        foreach(_FILE plugInfo.json generatedSchema.usda)
            if(NOT EXISTS "${OVPHYSX_DEV_SCHEMA_PROVIDER}/physx_usd_schemas/${_MODULE}/resources/${_FILE}")
                message(FATAL_ERROR "Incomplete local devschema provider. Rebuild with --devschema; missing ${_MODULE}/resources/${_FILE}.")
            endif()
        endforeach()
    endforeach()
    set(ENV{PHYSX_USD_SCHEMAS_ROOT} "${OVPHYSX_DEV_SCHEMA_PROVIDER}")
    if(CMAKE_HOST_WIN32)
        set(_DEV_SCHEMA_PATH_SEP ";")
    else()
        set(_DEV_SCHEMA_PATH_SEP ":")
    endif()
    set(ENV{PYTHONPATH} "${OVPHYSX_DEV_SCHEMA_PROVIDER}${_DEV_SCHEMA_PATH_SEP}$ENV{PYTHONPATH}")
    message(STATUS "Using separate local schema provider: ${OVPHYSX_DEV_SCHEMA_PROVIDER}")
endif()
