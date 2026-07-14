# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# omni.usd.schema.physx extension — copies schema libraries and resources
# from the physics schema package (usd_ext_physics).

set(EXT_NAME "omni.usd.schema.physx")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")

# Static extension content
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"          "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"            "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"            "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts"  "${EXT_DIR}/usd/schema/physx/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/tests"    "${EXT_DIR}/usd/schema/physx/tests")

ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/usd/schema/physx/__init__.py")

# Schema package resources and libraries.
# USD_EXT_PHYSICS_DIR is set by the root CMakeLists.txt (always concrete, points to release).
# USD_EXT_PHYSICS_BASE is the parent dir for config-dependent native libraries.
# For multi-config generators, we copy per-config native DLLs from the matching package.
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}")
    set(_SCHEMA_DIR "${USD_EXT_PHYSICS_DIR}")

    # Determine which configs to deploy native libraries for
    if(CMAKE_CONFIGURATION_TYPES AND DEFINED USD_EXT_PHYSICS_BASE)
        set(_SCHEMA_DEPLOY_CONFIGS ${CMAKE_CONFIGURATION_TYPES})
    else()
        set(_SCHEMA_DEPLOY_CONFIGS "")
    endif()

    # USD plugin resources (static, identical across configs — use release)
    ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/share/usd/plugins/PhysxSchema/resources/"
        "${EXT_DIR}/plugins/PhysxSchema/resources")
    ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/share/usd/plugins/PhysxSchemaAddition/resources/"
        "${EXT_DIR}/plugins/PhysxSchemaAddition/resources")
    ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/share/usd/plugins/OmniUsdPhysicsDeformableSchema/resources/"
        "${EXT_DIR}/plugins/OmniUsdPhysicsDeformableSchema/resources")

    # Python schema modules and shared libraries — config-dependent for multi-config.
    # For multi-config, copy from each config's schema package into the per-config ext dir.
    if(_SCHEMA_DEPLOY_CONFIGS)
        foreach(_cfg IN LISTS _SCHEMA_DEPLOY_CONFIGS)
            string(TOLOWER "${_cfg}" _cfg_lower)
            set(_CFG_SCHEMA "${USD_EXT_PHYSICS_BASE}/${_cfg_lower}")
            # Resolve per-config EXT_DIR (replace genex-based OVEXTS_BIN_DIR with concrete path)
            set(_CFG_EXT_DIR "${OVEXTS_ROOT_DIR}/_build/${_PLATFORM_ARCH}/${_cfg_lower}/extsPhysics/${EXT_NAME}")
            if(NOT EXISTS "${_CFG_SCHEMA}")
                # Fall back to release schema if this config's package wasn't pulled
                set(_CFG_SCHEMA "${_SCHEMA_DIR}")
            endif()
            ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/python/PhysicsSchemaTools/"
                "${_CFG_EXT_DIR}/pxr/PhysicsSchemaTools")
            ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/python/PhysxSchema/"
                "${_CFG_EXT_DIR}/pxr/PhysxSchema")
            if(WIN32)
                ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/physxSchema.dll" "${_CFG_EXT_DIR}/bin")
                ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/physicsSchemaTools.dll" "${_CFG_EXT_DIR}/bin")
            else()
                ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/libphysxSchema.so" "${_CFG_EXT_DIR}/bin")
                ovexts_prebuild_copy_dir("${_CFG_SCHEMA}/lib/libphysicsSchemaTools.so" "${_CFG_EXT_DIR}/bin")
            endif()
        endforeach()
    else()
        # Single-config: copy from USD_EXT_PHYSICS_DIR directly
        ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/python/PhysicsSchemaTools/"
            "${EXT_DIR}/pxr/PhysicsSchemaTools")
        ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/python/PhysxSchema/"
            "${EXT_DIR}/pxr/PhysxSchema")
        if(WIN32)
            ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/physxSchema.dll" "${EXT_DIR}/bin")
            ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/physicsSchemaTools.dll" "${EXT_DIR}/bin")
        else()
            ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/libphysxSchema.so" "${EXT_DIR}/bin")
            ovexts_prebuild_copy_dir("${_SCHEMA_DIR}/lib/libphysicsSchemaTools.so" "${EXT_DIR}/bin")
        endif()
    endif()
else()
    message(WARNING "USD_EXT_PHYSICS_DIR not set or not found — ${EXT_NAME} schema files will not be copied")
endif()

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
