# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.stageupdate")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxstageupdate")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/tests"   "${EXT_DIR}/${EXT_PYTHON_NS}/tests")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.stageupdate.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.stageupdate.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physx.stageupdate.plugin)
target_include_directories(omni.physx.stageupdate.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(omni.physx.stageupdate.plugin PRIVATE dl)
endif()

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.stageupdate.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxStageUpdateNode"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
