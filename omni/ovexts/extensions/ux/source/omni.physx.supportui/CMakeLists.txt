# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.supportui")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxsupportui")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/resources"      "${EXT_DIR}/resources")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/images/omni.physx.placement" "${EXT_DIR}/icons")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.supportui.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.supportui.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physx.supportui.plugin)
ovruntime_link_physx_static_libs(omni.physx.supportui.plugin FULL)
ovexts_link_omniui_deps(omni.physx.supportui.plugin)
ovexts_link_imgui_deps(omni.physx.supportui.plugin)
target_include_directories(omni.physx.supportui.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
    ${OVEXTS_KIT_SDK_DIR}/dev/fabric/include
    ${OVEXTS_TARGET_DEPS}/gsl/include
)
target_link_libraries(omni.physx.supportui.plugin PRIVATE
    OvruntimeCommon
    physxSchema
    omni.usd
    carb
    omni.ui
    omni.ui.scene
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.supportui.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physx.supportui.plugin "omni.usd.core")

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.supportui.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxSupportUi"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)
ovruntime_link_usd_deps(omni.physx.supportui.python)
ovexts_link_omniui_deps(omni.physx.supportui.python)
target_include_directories(omni.physx.supportui.python PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)
target_link_libraries(omni.physx.supportui.python PRIVATE omni.ui omni.ui.scene)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME} DIR "extsPhysicsRepo")
ovexts_define_etm_test_experience(${EXT_NAME})
