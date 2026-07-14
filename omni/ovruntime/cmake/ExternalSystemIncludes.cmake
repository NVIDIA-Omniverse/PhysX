# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Adds all external (third-party) dependency include directories to the given
# target as SYSTEM includes.  This suppresses compiler warnings originating in
# headers we cannot modify (Carbonite SDK, PhysX SDK, USD, CUDA, Kit SDK, etc.)
#
# Usage:  ovruntime_add_external_system_includes(<target>)

function(ovruntime_add_external_system_includes _target)
    set(_SYSTEM_DIRS "")

    # Carbonite SDK
    if(DEFINED CARB_SDK_DIR)
        list(APPEND _SYSTEM_DIRS "${CARB_SDK_DIR}/include")
    endif()

    # PhysX SDK
    if(DEFINED PHYSX_SDK_DIR)
        list(APPEND _SYSTEM_DIRS "${PHYSX_SDK_DIR}/include")
        # Parent of packman physx dir — resolves <physx/include/...> and
        # <cxxopts/include/...> style includes from target-deps
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_TARGET_DEPS}")
        if(OVRUNTIME_DEV_PHYSX)
            # When building from source, also add the repo root so
            # <physx/include/...> resolves against the source tree
            get_filename_component(_PHYSX_PARENT "${PHYSX_SDK_DIR}" DIRECTORY)
            list(APPEND _SYSTEM_DIRS "${_PHYSX_PARENT}")
        endif()
        if(EXISTS "${PHYSX_SDK_DIR}/pvdruntime/include")
            list(APPEND _SYSTEM_DIRS "${PHYSX_SDK_DIR}/pvdruntime/include")
        endif()
        if(EXISTS "${PHYSX_SDK_DIR}/pvddom/include")
            list(APPEND _SYSTEM_DIRS "${PHYSX_SDK_DIR}/pvddom/include")
        endif()
    endif()

    # USD headers — config-dependent (debug/release have separate include trees).
    # Library paths (debug/lib vs release/lib) are handled separately by
    # UsdLinkDependencies.cmake with generator expressions.
    if(DEFINED USD_DIR)
        if(CMAKE_CONFIGURATION_TYPES)
            # Multi-config (VS): add both so MSVC can resolve for either config.
            # The actual headers are identical but packman installs per-config.
            foreach(_usd_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
                string(TOLOWER "${_usd_cfg}" _usd_cfg_lower)
                if(EXISTS "${USD_DIR}/${_usd_cfg_lower}/include")
                    list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg_lower}/include")
                    list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg_lower}/include/pxr/external")
                    if(NOT WIN32)
                        list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg_lower}/include/boost")
                    endif()
                endif()
            endforeach()
        else()
            # Single-config: use the matching config, fall back to release.
            string(TOLOWER "${CMAKE_BUILD_TYPE}" _usd_cfg)
            if(NOT EXISTS "${USD_DIR}/${_usd_cfg}/include")
                set(_usd_cfg "release")
            endif()
            list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg}/include")
            list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg}/include/pxr/external")
            if(NOT WIN32)
                list(APPEND _SYSTEM_DIRS "${USD_DIR}/${_usd_cfg}/include/boost")
            endif()
        endif()
    endif()

    # CUDA
    if(DEFINED CUDA_DIR)
        list(APPEND _SYSTEM_DIRS "${CUDA_DIR}/include")
    endif()

    # Physics schema (physxSchema, physicsSchemaTools)
    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/include")
        list(APPEND _SYSTEM_DIRS "${USD_EXT_PHYSICS_DIR}/include")
    endif()

    # Python
    if(DEFINED PYTHON_DIR)
        list(APPEND _SYSTEM_DIRS "${PYTHON_DIR}/include")
        if(NOT WIN32)
            file(GLOB _PY_DIRS "${PYTHON_DIR}/include/python3.*")
            if(_PY_DIRS)
                list(GET _PY_DIRS 0 _PY_DIR)
                list(APPEND _SYSTEM_DIRS "${_PY_DIR}")
            endif()
        endif()
    endif()

    # ovruntime_deps (RTX/fabric plugin headers: carb/ujitso, omni/fabric, omni/blobkey, etc.)
    if(DEFINED OVRUNTIME_DEPS_DIR AND EXISTS "${OVRUNTIME_DEPS_DIR}/include")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_DEPS_DIR}/include")
    endif()

    # Kit-kernel dev headers (omni/kit, omni/ext, omni/log, etc.)
    if(DEFINED OVRUNTIME_KIT_SDK_DIR AND EXISTS "${OVRUNTIME_KIT_SDK_DIR}/dev/include")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_KIT_SDK_DIR}/dev/include")
    endif()

    # Fabric headers (omni/utils/HashTypes.h, omni/utils/OperationResult.h, etc.)
    if(DEFINED OVRUNTIME_KIT_SDK_DIR AND EXISTS "${OVRUNTIME_KIT_SDK_DIR}/dev/fabric/include")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_KIT_SDK_DIR}/dev/fabric/include")
    endif()

    # ImGui
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/imgui")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_TARGET_DEPS}/imgui")
    endif()

    # GSL
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/gsl/include")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_TARGET_DEPS}/gsl/include")
    endif()

    # Boost preprocessor
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/boost_preprocessor")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_TARGET_DEPS}/boost_preprocessor")
    endif()

    # Client library (OmniClient)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/client-library/include")
        list(APPEND _SYSTEM_DIRS "${OVRUNTIME_TARGET_DEPS}/client-library/include")
    endif()

    # pybind11
    if(DEFINED PYBIND11_DIR)
        list(APPEND _SYSTEM_DIRS "${PYBIND11_DIR}/include")
    endif()

    # doctest
    if(DEFINED DOCTEST_DIR)
        list(APPEND _SYSTEM_DIRS "${DOCTEST_DIR}/include")
    endif()

    # cxxopts
    if(DEFINED CXXOPTS_DIR)
        list(APPEND _SYSTEM_DIRS "${CXXOPTS_DIR}/include")
    endif()

    if(_SYSTEM_DIRS)
        target_include_directories(${_target} SYSTEM PRIVATE ${_SYSTEM_DIRS})
    endif()
endfunction()
