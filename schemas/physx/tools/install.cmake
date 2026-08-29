# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# cmake -P script: install codeless schema artifacts to _build/schema/.
# Replaces the premake prebuild_copy steps in the old premake5.lua files.
#
# Usage (called from build.sh / build.bat):
#   cmake -P tools/install.cmake
#
# SCHEMA_ROOT : absolute path to schemas/physx  (default: parent of this file's directory)

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SCHEMA_ROOT)
    get_filename_component(SCHEMA_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

set(INSTALL_DIR "${SCHEMA_ROOT}/_build/schema")
file(REMOVE_RECURSE "${INSTALL_DIR}")
message(STATUS "[install] schema")

# install_schema_module(<source dir> <Module> <includeDir>)
#
# Installs one codeless schema module. Headers are taken from the generator's
# codeless_headers.manifest (per-class + umbrella) plus the two fixed-name
# headers, NOT a *.h glob -- so a header orphaned by a deleted schema class is
# never republished. Python and USD data are explicit allowlists.
function(install_schema_module _SRC _MODULE _INCDIR)
    file(INSTALL
        "${_SRC}/generatedSchema.usda"
        "${_SRC}/schema.usda"
        "${_SRC}/plugInfo.json"
        DESTINATION "${INSTALL_DIR}/share/usd/plugins/${_MODULE}/resources"
    )

    set(_HEADERS "${_SRC}/tokens.h")
    if(EXISTS "${_SRC}/axisInstanceTokens.h")
        list(APPEND _HEADERS "${_SRC}/axisInstanceTokens.h")
    endif()
    set(_MANIFEST "${_SRC}/codeless_headers.manifest")
    if(NOT EXISTS "${_MANIFEST}")
        message(FATAL_ERROR "install: ${_MANIFEST} not found; run gen_codeless first.")
    endif()
    file(STRINGS "${_MANIFEST}" _GEN_HEADERS)
    foreach(_h ${_GEN_HEADERS})
        if(_h)
            list(APPEND _HEADERS "${_SRC}/${_h}")
        endif()
    endforeach()
    file(INSTALL ${_HEADERS} DESTINATION "${INSTALL_DIR}/include/${_INCDIR}")

    file(INSTALL
        "${_SRC}/__init__.py"
        "${_SRC}/codeless_api.py"
        "${_SRC}/_tokens.py"
        DESTINATION "${INSTALL_DIR}/lib/python/${_MODULE}"
    )
endfunction()

install_schema_module("${SCHEMA_ROOT}/source/physxSchema" "PhysxSchema" "physxSchema")
install_schema_module("${SCHEMA_ROOT}/source/omniUsdPhysicsDeformableSchema"
                      "OmniUsdPhysicsDeformableSchema" "omniUsdPhysicsDeformableSchema")
