# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.vehicle.tests")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxvehicletests")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/audio/physics/vehicle"                   "${EXT_DIR}/data/audio")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/usd/tests/Physics/Vehicle_Schema_Tests"  "${EXT_DIR}/data/tests/Vehicle_Schema_Tests")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.vehicle.tests.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.vehicle.tests.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physx.vehicle.tests.plugin)
ovruntime_link_physx_static_libs(omni.physx.vehicle.tests.plugin FULL)
target_include_directories(omni.physx.vehicle.tests.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)
target_link_libraries(omni.physx.vehicle.tests.plugin PRIVATE
    physxSchema
    omni.usd
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.vehicle.tests.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physx.vehicle.tests.plugin "omni.usd.core")

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.vehicle.tests.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxVehicleTests"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
