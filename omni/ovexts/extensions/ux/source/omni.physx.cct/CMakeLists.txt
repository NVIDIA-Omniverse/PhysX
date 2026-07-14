# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.cct")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxcct")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/nodes"          "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/nodes")

# ---------------------------------------------------------------------------
# OGN code generation (Python-only nodes — no C++ headers needed for plugin)
# ---------------------------------------------------------------------------
ovexts_generate_ogn(omni.physx.cct
    NODES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nodes"
    EXTENSION "omni.physx.cct"
    MODULE "omni.physxcct"
    PYTHON_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn"
    DOCS_DIR "${EXT_DIR}/ogn/docs"
    TESTS_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/tests"
    USD_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/tests/usd"
)

# ---------------------------------------------------------------------------
# Plugin: omni.physx.cct.plugin
# ---------------------------------------------------------------------------
file(GLOB_RECURSE _PLUGIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
ovexts_add_carbonite_plugin(omni.physx.cct.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.cct.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)

# USD dependencies
ovruntime_link_usd_deps(omni.physx.cct.plugin)

# PhysX SDK — CCT only needs PhysXCharacterKinematic + core libs
ovruntime_link_physx_static_libs(omni.physx.cct.plugin FULL)

target_include_directories(omni.physx.cct.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
)

# Link libraries
target_link_libraries(omni.physx.cct.plugin PRIVATE
    physxSchema
    omni.usd
)

if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.cct.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physx.cct.plugin "omni.usd.core")

# ---------------------------------------------------------------------------
# Python bindings: _physxCct
# ---------------------------------------------------------------------------
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.cct.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxCct"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
