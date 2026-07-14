# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.pvd")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxpvd")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config" "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"   "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"   "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")

# PVDRuntime — the extension.py on_startup() preloads this DLL from its own bin/.
# In premake builds it was copied by repo_build.copy_to_targetdir; replicate here.
set(_GPU_EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/omni.physx.gpu")
if(WIN32)
    set(_PVDRUNTIME_NAME "PVDRuntime_64.dll")
else()
    set(_PVDRUNTIME_NAME "libPVDRuntime_64.so")
endif()
add_custom_target(omni.physx.pvd.pvdruntime_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_GPU_EXT_DIR}/bin/${_PVDRUNTIME_NAME}"
        "${EXT_DIR}/bin/${_PVDRUNTIME_NAME}"
    COMMENT "Copying ${_PVDRUNTIME_NAME} to omni.physx.pvd extension"
    DEPENDS OvruntimePhysXGpu
)

# Plugin
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.pvd.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.pvd.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)
ovruntime_link_usd_deps(omni.physx.pvd.plugin)
ovruntime_link_physx_static_libs(omni.physx.pvd.plugin FULL)
target_include_directories(omni.physx.pvd.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/client-library/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../physx/pvddom/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../physx/pvdruntime/include
)
# PvdDom — DOM parser library extracted from this extension
if(NOT TARGET PvdDom)
    set(PVDRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../physx/pvdruntime" CACHE PATH "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../physx/pvddom ${CMAKE_CURRENT_BINARY_DIR}/pvddom_bin)
endif()

target_link_libraries(omni.physx.pvd.plugin PRIVATE
    OvruntimeCommon
    physxSchema
    omni.usd
    PvdDom
)
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.pvd.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physx.pvd.plugin "omni.usd.core")

# Suppress pointer-arith warning in PVD parser code (matches premake)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(omni.physx.pvd.plugin PRIVATE -Wno-pointer-arith)
endif()

# Bindings
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.pvd.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxPvd"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
