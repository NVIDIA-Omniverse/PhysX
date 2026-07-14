# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.usdphysics.ui")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/usdphysicsui")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/icons/omni.usdphysics.ui" "${EXT_DIR}/icons")

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.usdphysicsui.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.usdphysicsui.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.usdphysicsui.plugin)
ovexts_link_imgui_deps(omni.usdphysicsui.plugin)
target_include_directories(omni.usdphysicsui.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
    ${OVEXTS_KIT_SDK_DIR}/dev/fabric/include
    ${OVEXTS_TARGET_DEPS}/gsl/include
)
target_link_libraries(omni.usdphysicsui.plugin PRIVATE
    OvruntimeCommon
    omni.usd
    carb
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.usdphysicsui.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
    if(CMAKE_CONFIGURATION_TYPES AND DEFINED USD_EXT_PHYSICS_BASE)
        foreach(_ext_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
            string(TOLOWER "${_ext_cfg}" _ext_cfg_lower)
            if(EXISTS "${USD_EXT_PHYSICS_BASE}/${_ext_cfg_lower}/lib")
                target_link_directories(omni.usdphysicsui.plugin PRIVATE
                    "${USD_EXT_PHYSICS_BASE}/${_ext_cfg_lower}/lib")
            endif()
        endforeach()
    endif()
endif()
ovexts_link_kit_ext_libs(omni.usdphysicsui.plugin "omni.usd.core")

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.usdphysicsui.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_usdphysicsUI"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)
ovruntime_link_usd_deps(omni.usdphysicsui.python)
target_include_directories(omni.usdphysicsui.python PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
