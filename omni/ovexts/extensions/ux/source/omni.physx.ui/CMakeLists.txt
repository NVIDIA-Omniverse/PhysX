# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.ui")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxui")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/tests"   "${EXT_DIR}/${EXT_PYTHON_NS}/tests")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/icons/omni.physx.ui" "${EXT_DIR}/icons")
ovexts_prebuild_link("${OVRUNTIME_DIR}/source/common/python/windowmenuitem" "${EXT_DIR}/omni/physxuicommon/windowmenuitem")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physxui.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physxui.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physxui.plugin)
ovruntime_link_physx_static_libs(omni.physxui.plugin FULL)
ovexts_link_omniui_deps(omni.physxui.plugin)
ovexts_link_imgui_deps(omni.physxui.plugin)
target_include_directories(omni.physxui.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
    ${OVEXTS_KIT_SDK_DIR}/dev/fabric/include
    ${OVEXTS_TARGET_DEPS}/glm
    ${OVEXTS_TARGET_DEPS}/gsl/include
)
target_link_libraries(omni.physxui.plugin PRIVATE
    OvruntimeCommon
    physicsSchemaTools
    physxSchema
    omni.usd
    omni.ui
    omni.ui.scene
    carb
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physxui.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physxui.plugin "omni.usd.core")

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physxui.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxUI"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)
ovruntime_link_usd_deps(omni.physxui.python)
ovexts_link_omniui_deps(omni.physxui.python)
target_include_directories(omni.physxui.python PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)
target_link_libraries(omni.physxui.python PRIVATE omni.ui.scene)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
