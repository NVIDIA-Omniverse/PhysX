# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.vehicle")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxvehicle")

# ---------------------------------------------------------------------------
# Prebuild: copy/link static content
# ---------------------------------------------------------------------------
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")

# ---------------------------------------------------------------------------
# Plugin: omni.physx.vehicle.plugin
# ---------------------------------------------------------------------------
ovexts_add_carbonite_plugin(omni.physx.vehicle.plugin
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXVehicle.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VehicleManager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VehicleController.cpp
    OUTPUT_NAME "omni.physx.vehicle.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)

# USD dependencies (includes, libs, ~50 USD libraries)
ovruntime_link_usd_deps(omni.physx.vehicle.plugin)

# PhysX SDK static libraries
ovruntime_link_physx_static_libs(omni.physx.vehicle.plugin FULL)

# Kit SDK includes
target_include_directories(omni.physx.vehicle.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
)

# Link OvruntimeCommon (built from ovruntime subproject) + schema libs
target_link_libraries(omni.physx.vehicle.plugin PRIVATE
    OvruntimeCommon
    physxSchema
    omni.usd
)

# Schema library directory
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.vehicle.plugin PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
endif()

# Kit SDK omni.usd.core lib directory
ovexts_link_kit_ext_libs(omni.physx.vehicle.plugin "omni.usd.core")

# ---------------------------------------------------------------------------
# Python bindings: _physxVehicle
# ---------------------------------------------------------------------------
ovexts_add_python_bindings(omni.physx.vehicle.python
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXVehiclePython.cpp
    OUTPUT_NAME "_physxVehicle"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
