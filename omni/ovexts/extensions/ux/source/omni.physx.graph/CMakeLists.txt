# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.graph")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxgraph")

# Prebuild
ovexts_prebuild_copy("${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"           "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"           "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"         "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/nodes"          "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/nodes")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/scripts" "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python/tests"   "${EXT_DIR}/${EXT_PYTHON_NS}/tests")

# ---------------------------------------------------------------------------
# OGN code generation
# ---------------------------------------------------------------------------
ovexts_generate_ogn(omni.physx.graph
    NODES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nodes"
    EXTENSION "omni.physx.graph"
    MODULE "omni.physxgraph"
    PYTHON_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn"
    DOCS_DIR "${EXT_DIR}/ogn/docs"
    TESTS_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/tests"
    USD_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/ogn/tests/usd"
)

# ---------------------------------------------------------------------------
# Plugin: omni.physx.graph.plugin
# ---------------------------------------------------------------------------
file(GLOB_RECURSE _PLUGIN_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/*.cpp"
)
ovexts_add_carbonite_plugin(omni.physx.graph.plugin
    SOURCES ${_PLUGIN_SRCS}
    OUTPUT_NAME "omni.physx.graph.plugin"
    OUTPUT_DIR "${EXT_DIR}/bin"
)

# Depend on OGN generation (headers must be generated before compilation)
add_dependencies(omni.physx.graph.plugin omni.physx.graph_ogn)

# USD dependencies
ovruntime_link_usd_deps(omni.physx.graph.plugin)

# PhysX SDK static libraries
ovruntime_link_physx_static_libs(omni.physx.graph.plugin FULL)

# OGN generated headers + Kit SDK includes
target_include_directories(omni.physx.graph.plugin PRIVATE
    ${OVEXTS_KIT_SDK_INCLUDES}
    ${OVEXTS_TARGET_DEPS}/gsl/include
    ${OVEXTS_OGN_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes    # for relative includes like ../plugins/ from node subdirs
)

# Kit SDK omni.graph.core headers (for NodeTypeRegistrar, OgnHelpers, etc.)
# These are loaded at runtime via Carbonite plugin system, not linked directly.

# Link libraries
target_link_libraries(omni.physx.graph.plugin PRIVATE
    OvruntimeCommon
    physxSchema
    physicsSchemaTools
    omni.usd
)

if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(omni.physx.graph.plugin PRIVATE ${USD_EXT_PHYSICS_DIR}/lib)
endif()
ovexts_link_kit_ext_libs(omni.physx.graph.plugin "omni.usd.core")

# ---------------------------------------------------------------------------
# Python bindings: _physxGraph
# ---------------------------------------------------------------------------
file(GLOB _BINDINGS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp")
ovexts_add_python_bindings(omni.physx.graph.python
    SOURCES ${_BINDINGS_SRCS}
    OUTPUT_NAME "_physxGraph"
    OUTPUT_DIR "${EXT_DIR}/${EXT_PYTHON_NS}/bindings"
)

# Bindings also need USD deps (for boost::python on Windows)
ovruntime_link_usd_deps(omni.physx.graph.python)

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
ovexts_define_etm_test_experience(${EXT_NAME})
