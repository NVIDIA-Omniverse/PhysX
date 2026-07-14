# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# OGN (OmniGraph Node) code generation for ovexts.
#
# Generates Database headers (C++) and Python wrappers from .ogn node
# description files using the generate_node.py tool from omni.graph.tools.
#
# Outputs:
#   OVEXTS_OGN_PYTHON           - Python executable for running the generator
#   OVEXTS_OGN_GENERATOR        - Path to generate_node.py
#   OVEXTS_OGN_CONFIG_DIR       - OGN configuration directory (type definitions)
#   OVEXTS_OGN_AVAILABLE        - TRUE if all OGN tool dependencies are found

# ---------------------------------------------------------------------------
# Discover the OGN toolchain
# ---------------------------------------------------------------------------

# Python from target-deps
set(OVEXTS_OGN_PYTHON "")
if(WIN32)
    set(_ogn_python_candidate "${OVEXTS_TARGET_DEPS}/python/python.exe")
else()
    set(_ogn_python_candidate "${OVEXTS_TARGET_DEPS}/python/bin/python3")
endif()
if(EXISTS "${_ogn_python_candidate}")
    set(OVEXTS_OGN_PYTHON "${_ogn_python_candidate}")
endif()

# generate_node.py from omni.graph.tools Kit extension.
# Check three locations in priority order:
#   1. Precached build extensions (populated by build.py precache step)
#   2. Kit SDK bundled extensions
#   3. Kit SDK deps extensions
# Uses _CONFIGURE variants for configure-time EXISTS checks (no genexes).
set(OVEXTS_OGN_GENERATOR "")
foreach(_exts_dir
    "${OVEXTS_KIT_SDK_EXTS_BUILD_CONFIGURE}"
    "${OVEXTS_KIT_SDK_EXTS_LINKED_CONFIGURE}"
    "${OVEXTS_KIT_SDK_EXTS_DEPS_CONFIGURE}"
)
    set(_gen_candidate "${_exts_dir}/omni.graph.tools/omni/graph/tools/generate_node.py")
    if(EXISTS "${_gen_candidate}")
        set(OVEXTS_OGN_GENERATOR "${_gen_candidate}")
        break()
    endif()
endforeach()

# OGN config directory — type definition JSON files.
# Check the omni.graph.tools extension's ogn/ subdirectory first,
# then fall back to the _build/ogn/config/ directory (premake legacy).
set(OVEXTS_OGN_CONFIG_DIR "")
foreach(_exts_dir
    "${OVEXTS_KIT_SDK_EXTS_BUILD_CONFIGURE}"
    "${OVEXTS_KIT_SDK_EXTS_LINKED_CONFIGURE}"
    "${OVEXTS_KIT_SDK_EXTS_DEPS_CONFIGURE}"
)
    set(_cfg_candidate "${_exts_dir}/omni.graph.tools/ogn")
    if(EXISTS "${_cfg_candidate}/TypeConfigurationPod.json")
        set(OVEXTS_OGN_CONFIG_DIR "${_cfg_candidate}")
        break()
    endif()
endforeach()
if(NOT OVEXTS_OGN_CONFIG_DIR AND EXISTS "${OVEXTS_ROOT_DIR}/_build/ogn/config/TypeConfigurationPod.json")
    set(OVEXTS_OGN_CONFIG_DIR "${OVEXTS_ROOT_DIR}/_build/ogn/config")
endif()

# Check availability
set(OVEXTS_OGN_AVAILABLE FALSE)
if(OVEXTS_OGN_PYTHON AND OVEXTS_OGN_GENERATOR AND OVEXTS_OGN_CONFIG_DIR)
    set(OVEXTS_OGN_AVAILABLE TRUE)
    message(STATUS "OGN generator found: ${OVEXTS_OGN_GENERATOR}")
    message(STATUS "OGN config dir: ${OVEXTS_OGN_CONFIG_DIR}")
else()
    if(NOT OVEXTS_OGN_PYTHON)
        message(STATUS "OGN: Python not found in target-deps")
    endif()
    if(NOT OVEXTS_OGN_GENERATOR)
        message(STATUS "OGN: generate_node.py not found in Kit extensions")
    endif()
    if(NOT OVEXTS_OGN_CONFIG_DIR)
        message(STATUS "OGN: config directory not found")
    endif()
endif()

# OGN output directories
set(OVEXTS_OGN_INCLUDE_DIR "${CMAKE_BINARY_DIR}/ogn/include")
set(OVEXTS_OGN_INTERMEDIATE_DIR "${CMAKE_BINARY_DIR}/ogn/intermediate")

# ---------------------------------------------------------------------------
# ovexts_generate_ogn(<ext_name>
#     NODES_DIR <dir>            # Directory containing .ogn files (searched recursively)
#     EXTENSION <name>           # Extension name (e.g. "omni.physx.graph")
#     MODULE <name>              # Python module name (e.g. "omni.physx.graph")
#     [PYTHON_DIR <dir>]         # Output directory for Python OGN wrappers
#     [DOCS_DIR <dir>]           # Output directory for documentation
#     [TESTS_DIR <dir>]          # Output directory for test files
#     [USD_DIR <dir>]            # Output directory for USD test templates
# )
#
# Generates C++ Database headers and optional Python/docs/test outputs from
# .ogn files. The C++ headers go to ${OVEXTS_OGN_INCLUDE_DIR}.
#
# Sets in parent scope:
#   ${ext_name}_OGN_OUTPUTS - list of generated output paths (headers + stamp files)
# ---------------------------------------------------------------------------
function(ovexts_generate_ogn ext_name)
    cmake_parse_arguments(ARG "" "NODES_DIR;EXTENSION;MODULE;PYTHON_DIR;DOCS_DIR;TESTS_DIR;USD_DIR" "" ${ARGN})

    if(NOT OVEXTS_OGN_AVAILABLE)
        message(FATAL_ERROR "ovexts_generate_ogn(${ext_name}): OGN toolchain not available. "
            "Run build.sh --fetch-only or ensure omni.graph.tools extension is resolved.")
    endif()

    if(NOT ARG_NODES_DIR)
        message(FATAL_ERROR "ovexts_generate_ogn(${ext_name}): NODES_DIR is required")
    endif()
    if(NOT ARG_EXTENSION)
        message(FATAL_ERROR "ovexts_generate_ogn(${ext_name}): EXTENSION is required")
    endif()
    if(NOT ARG_MODULE)
        set(ARG_MODULE "${ARG_EXTENSION}")
    endif()

    # Discover all .ogn files
    file(GLOB_RECURSE _ogn_files "${ARG_NODES_DIR}/*.ogn")
    if(NOT _ogn_files)
        message(WARNING "ovexts_generate_ogn(${ext_name}): No .ogn files found in ${ARG_NODES_DIR}")
        return()
    endif()

    file(MAKE_DIRECTORY "${OVEXTS_OGN_INCLUDE_DIR}")
    file(MAKE_DIRECTORY "${OVEXTS_OGN_INTERMEDIATE_DIR}")

    # Create __init__.py in the Python OGN output directory (package marker).
    # The generator creates tests/__init__.py but not the parent ogn/__init__.py.
    # Use _ovexts_resolve_multiconfig_paths to handle genex paths at configure time.
    if(ARG_PYTHON_DIR)
        _ovexts_resolve_multiconfig_paths("${ARG_PYTHON_DIR}" _python_dirs)
        foreach(_pd IN LISTS _python_dirs)
            file(MAKE_DIRECTORY "${_pd}")
            file(TOUCH "${_pd}/__init__.py")
        endforeach()
    endif()

    set(_generated_outputs)

    foreach(_ogn_file IN LISTS _ogn_files)
        # Derive the node class name from filename: OgnFoo.ogn -> OgnFoo
        get_filename_component(_ogn_name "${_ogn_file}" NAME_WE)

        # Detect Python-language nodes at configure time.
        # Python nodes don't generate C++ headers, so we use a stamp file instead.
        file(READ "${_ogn_file}" _ogn_content)
        string(FIND "${_ogn_content}" "\"language\": \"Python\"" _python_pos)
        string(FIND "${_ogn_content}" "\"language\":\"Python\"" _python_pos2)
        if(_python_pos GREATER -1 OR _python_pos2 GREATER -1)
            set(_is_python TRUE)
        else()
            set(_is_python FALSE)
        endif()

        # Use the actual generated file as the OUTPUT so CMake regenerates
        # if the output is deleted (e.g. by a clean Docker build).
        if(_is_python AND ARG_PYTHON_DIR)
            set(_output_file "${ARG_PYTHON_DIR}/${_ogn_name}Database.py")
        elseif(_is_python)
            set(_output_file "${OVEXTS_OGN_INTERMEDIATE_DIR}/${_ogn_name}.ogn.stamp")
        else()
            set(_output_file "${OVEXTS_OGN_INCLUDE_DIR}/${_ogn_name}Database.h")
        endif()

        # Build the generator command arguments
        set(_gen_args
            "${OVEXTS_OGN_GENERATOR}"
            --configDirectory "${OVEXTS_OGN_CONFIG_DIR}"
            --extension "${ARG_EXTENSION}"
            --module "${ARG_MODULE}"
            --intermediate "${OVEXTS_OGN_INTERMEDIATE_DIR}"
            --nodeFile "${_ogn_file}"
        )
        # Only request C++ output for C++ nodes
        if(NOT _is_python)
            list(APPEND _gen_args --cpp "${OVEXTS_OGN_INCLUDE_DIR}")
        endif()
        if(ARG_PYTHON_DIR)
            list(APPEND _gen_args --python "${ARG_PYTHON_DIR}")
        endif()
        if(ARG_DOCS_DIR)
            list(APPEND _gen_args --docs "${ARG_DOCS_DIR}")
        endif()
        if(ARG_TESTS_DIR)
            list(APPEND _gen_args --tests "${ARG_TESTS_DIR}")
        endif()
        if(ARG_USD_DIR)
            list(APPEND _gen_args --usdPath "${ARG_USD_DIR}")
        endif()

        add_custom_command(
            OUTPUT "${_output_file}"
            COMMAND "${OVEXTS_OGN_PYTHON}" ${_gen_args}
            DEPENDS "${_ogn_file}"
            WORKING_DIRECTORY "${ARG_NODES_DIR}"
            COMMENT "OGN [${ARG_EXTENSION}]: ${_ogn_name}.ogn"
            VERBATIM
        )

        list(APPEND _generated_outputs "${_output_file}")
    endforeach()

    # Create a custom target that all generated outputs depend on
    add_custom_target(${ext_name}_ogn ALL DEPENDS ${_generated_outputs})
    set_property(TARGET ${ext_name}_ogn PROPERTY FOLDER "OGN")

    # Export the generated output list to parent scope
    set(${ext_name}_OGN_OUTPUTS ${_generated_outputs} PARENT_SCOPE)
endfunction()
