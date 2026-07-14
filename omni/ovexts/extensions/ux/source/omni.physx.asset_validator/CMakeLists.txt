# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.asset_validator")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_REPO_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxassetvalidator")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/tests"   "${EXT_DIR}/${EXT_PYTHON_NS}/tests")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/usd/tests/Physics/AssetValidator" "${EXT_DIR}/testdata")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.asset_validator.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.asset_validator.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physx.asset_validator.plugin)
ovruntime_link_physx_static_libs(omni.physx.asset_validator.plugin FULL)
target_include_directories(omni.physx.asset_validator.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
    ${OVEXTS_KIT_SDK_DIR}/dev/fabric/include
)
target_link_libraries(omni.physx.asset_validator.plugin PRIVATE
    carb
    physxSchema
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.asset_validator.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.asset_validator.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxAssetValidator"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)
target_include_directories(omni.physx.asset_validator.python PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME} DIR "extsPhysicsRepo")
ovexts_define_etm_test_experience(${EXT_NAME})
