# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Helper functions for ovexts CMake build.
# Provides equivalents of the Premake5 helpers in premake5-public.lua.

# ---------------------------------------------------------------------------
# ovexts_set_output_directory(<target> <dir>)
#
# Set the output directory for a target across all configurations.
# Uses config-specific properties to override the global CMAKE_*_OUTPUT_DIRECTORY_*
# set by NvidiaBuildOptions/SetOutputPaths.
# ---------------------------------------------------------------------------
function(ovexts_set_output_directory target dir)
    set_target_properties(${target} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${dir}"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${dir}"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${dir}"
        LIBRARY_OUTPUT_DIRECTORY_CHECKED "${dir}"
        RUNTIME_OUTPUT_DIRECTORY "${dir}"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${dir}"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${dir}"
        RUNTIME_OUTPUT_DIRECTORY_CHECKED "${dir}"
        ARCHIVE_OUTPUT_DIRECTORY "${dir}"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${dir}"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${dir}"
        ARCHIVE_OUTPUT_DIRECTORY_CHECKED "${dir}"
    )
endfunction()

# ---------------------------------------------------------------------------
# _ovexts_resolve_multiconfig_paths(<path> <out_var>)
#
# For multi-config generators, OVEXTS_BIN_DIR contains a generator expression
# that cannot be used in configure-time commands (file, configure_file, etc.).
# This helper replaces the genex portion with each concrete config directory
# from OVEXTS_BIN_DIR_CONFIGS and returns a list of resolved paths.
# For single-config generators, returns <path> unchanged.
# ---------------------------------------------------------------------------
function(_ovexts_resolve_multiconfig_paths path out_var)
    if(NOT CMAKE_CONFIGURATION_TYPES)
        set(${out_var} "${path}" PARENT_SCOPE)
        return()
    endif()
    set(_result "")
    foreach(_bin IN LISTS OVEXTS_BIN_DIR_CONFIGS)
        string(REPLACE "${OVEXTS_BIN_DIR}" "${_bin}" _resolved "${path}")
        list(APPEND _result "${_resolved}")
    endforeach()
    set(${out_var} "${_result}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# ovexts_prebuild_link(<source> <dest>)
#
# Create a symbolic link from <source> to <dest> at configure time.
# For multi-config generators, expands to one link per configuration.
# Equivalent of repo_build.prebuild_link.
# ---------------------------------------------------------------------------
function(ovexts_prebuild_link source dest)
    if(NOT EXISTS "${source}")
        message(WARNING "ovexts_prebuild_link: source does not exist: ${source}")
        return()
    endif()
    _ovexts_resolve_multiconfig_paths("${dest}" _dests)
    foreach(_d IN LISTS _dests)
        if(EXISTS "${_d}")
            if(WIN32)
                file(TO_NATIVE_PATH "${_d}" _native_d_rm)
                execute_process(COMMAND cmd /c rmdir "${_native_d_rm}" COMMAND_ERROR_IS_FATAL ANY)
            else()
                file(REMOVE "${_d}")
            endif()
        endif()
        get_filename_component(_parent "${_d}" DIRECTORY)
        file(MAKE_DIRECTORY "${_parent}")
        if(WIN32)
            file(TO_NATIVE_PATH "${_d}" _native_d)
            file(TO_NATIVE_PATH "${source}" _native_src)
            execute_process(COMMAND cmd /c mklink /J "${_native_d}" "${_native_src}" COMMAND_ERROR_IS_FATAL ANY)
        else()
            file(CREATE_LINK "${source}" "${_d}" SYMBOLIC)
        endif()
    endforeach()
endfunction()

# ---------------------------------------------------------------------------
# ovexts_prebuild_copy(<source> <dest>)
#
# Copy <source> file to <dest> at configure time.
# For multi-config generators, expands to one copy per configuration.
# Equivalent of repo_build.prebuild_copy.
# ---------------------------------------------------------------------------
function(ovexts_prebuild_copy source dest)
    if(NOT EXISTS "${source}")
        message(WARNING "ovexts_prebuild_copy: source does not exist: ${source}")
        return()
    endif()
    _ovexts_resolve_multiconfig_paths("${dest}" _dests)
    foreach(_d IN LISTS _dests)
        get_filename_component(_parent "${_d}" DIRECTORY)
        file(MAKE_DIRECTORY "${_parent}")
        configure_file("${source}" "${_d}" COPYONLY)
    endforeach()
endfunction()

# ---------------------------------------------------------------------------
# ovexts_prebuild_copy_dir(<source_dir> <dest_dir>)
#
# Copy a directory tree at configure time (wraps file(COPY)).
# For multi-config generators, expands to one copy per configuration.
# ---------------------------------------------------------------------------
function(ovexts_prebuild_copy_dir source dest)
    if(NOT EXISTS "${source}")
        message(WARNING "ovexts_prebuild_copy_dir: source does not exist: ${source}")
        return()
    endif()
    _ovexts_resolve_multiconfig_paths("${dest}" _dests)
    foreach(_d IN LISTS _dests)
        file(COPY "${source}" DESTINATION "${_d}")
    endforeach()
endfunction()

# ---------------------------------------------------------------------------
# ovexts_add_carbonite_plugin(<target_name>
#     SOURCES <src1> [<src2> ...]
#     OUTPUT_NAME <name>
#     OUTPUT_DIR <dir>
# )
#
# Creates a shared library target with Carbonite plugin conventions.
# Equivalent of carbonitePlugin{} in premake5-public.lua.
# ---------------------------------------------------------------------------
function(ovexts_add_carbonite_plugin target_name)
    cmake_parse_arguments(ARG "" "OUTPUT_NAME;OUTPUT_DIR" "SOURCES" ${ARGN})

    add_library(${target_name} SHARED ${ARG_SOURCES})

    set_property(TARGET ${target_name} PROPERTY FOLDER "UX/${target_name}")

    if(ARG_OUTPUT_NAME)
        set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "${ARG_OUTPUT_NAME}")
    endif()

    if(ARG_OUTPUT_DIR)
        ovexts_set_output_directory(${target_name} "${ARG_OUTPUT_DIR}")
    endif()

    # Standard include directories for all Carbonite plugins
    target_include_directories(${target_name} PRIVATE
        ${OVEXTS_OVRUNTIME_INCLUDE_DIR}
        ${OVEXTS_UMBRELLA_INCLUDE_DIR}
        ${OVEXTS_OVRUNTIME_COMMON_INCLUDE_DIR}
        ${OVEXTS_OVRUNTIME_PCH_DIR}
    )

    # Standard compile definitions
    target_compile_definitions(${target_name} PRIVATE
        "PX_PHYSX_STATIC_LIB"
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    # External dependencies as SYSTEM includes (suppress third-party warnings)
    ovruntime_add_external_system_includes(${target_name})

    # Precompiled header (UsdPCH.h from ovruntime)
    target_precompile_headers(${target_name} PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${OVEXTS_OVRUNTIME_PCH_DIR}/UsdPCH.h>"
    )

    # Link carb
    if(DEFINED CARB_SDK_DIR)
        target_link_directories(${target_name} PRIVATE
            ${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/${OVEXTS_CARB_CONFIG}
        )
        target_link_libraries(${target_name} PRIVATE carb)
    endif()

    # RTTI and exceptions
    if(NOT WIN32)
        target_compile_options(${target_name} PRIVATE -fexceptions -frtti -fPIC)
    else()
        target_compile_options(${target_name} PRIVATE /EHsc)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# ovexts_add_python_bindings(<target_name>
#     SOURCES <src1> [<src2> ...]
#     OUTPUT_NAME <name>
#     OUTPUT_DIR <dir>
# )
#
# Creates a Python bindings shared library (.pyd on Windows, .so on Linux).
# Equivalent of carboniteBindingsPython{} in premake5-public.lua.
# ---------------------------------------------------------------------------
function(ovexts_add_python_bindings target_name)
    cmake_parse_arguments(ARG "" "OUTPUT_NAME;OUTPUT_DIR" "SOURCES" ${ARGN})

    add_library(${target_name} SHARED ${ARG_SOURCES})

    set_property(TARGET ${target_name} PROPERTY FOLDER "UX/${target_name}")

    if(ARG_OUTPUT_NAME)
        set_target_properties(${target_name} PROPERTIES
            OUTPUT_NAME "${ARG_OUTPUT_NAME}"
            PREFIX ""
        )
    endif()

    # Python extension suffix.
    # UX bindings use bare suffix (.so / .pyd) because Kit's custom importer loads them
    # during extension startup. Using the cpython suffix would make them findable by
    # Python's standard importer too, causing pybind11 double-registration when
    # omni.graph.tools tries to import OGN submodules.
    if(WIN32)
        set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
    else()
        set_target_properties(${target_name} PROPERTIES SUFFIX ".so")
    endif()

    if(ARG_OUTPUT_DIR)
        ovexts_set_output_directory(${target_name} "${ARG_OUTPUT_DIR}")
    endif()

    # Standard include directories (including PCH for UsdPCH.h)
    target_include_directories(${target_name} PRIVATE
        ${OVEXTS_OVRUNTIME_INCLUDE_DIR}
        ${OVEXTS_UMBRELLA_INCLUDE_DIR}
        ${OVEXTS_OVRUNTIME_COMMON_INCLUDE_DIR}
        ${OVEXTS_OVRUNTIME_PCH_DIR}
    )

    # Standard compile definitions
    target_compile_definitions(${target_name} PRIVATE
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    # External dependencies as SYSTEM includes (suppress third-party warnings)
    ovruntime_add_external_system_includes(${target_name})

    # RTTI and exceptions
    if(NOT WIN32)
        target_compile_options(${target_name} PRIVATE -fexceptions -frtti -fPIC)
    else()
        target_compile_options(${target_name} PRIVATE /EHsc)
    endif()

    # Link carb
    if(DEFINED CARB_SDK_DIR)
        target_link_directories(${target_name} PRIVATE
            ${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/${OVEXTS_CARB_CONFIG}
        )
        target_link_libraries(${target_name} PRIVATE carb)
    endif()

    # Python + USD boost::python bindings dependencies
    ovruntime_link_bindings_deps(${target_name})
endfunction()

# ---------------------------------------------------------------------------
# ovexts_link_omniui_deps(<target>)
#
# Add omni.ui and omni.ui.scene link directories from Kit SDK.
# Equivalent of extension_omniui_deps() in premake5-public.lua.
# ---------------------------------------------------------------------------
function(ovexts_link_omniui_deps target)
    ovexts_link_kit_ext_libs(${target} "omni.ui" "omni.ui.scene")
endfunction()

# ---------------------------------------------------------------------------
# ovexts_link_imgui_deps(<target>)
#
# Add imgui includes, defines, and link directories from Kit SDK.
# Equivalent of extension_imgui_deps() in premake5-public.lua.
# ---------------------------------------------------------------------------
function(ovexts_link_imgui_deps target)
    target_compile_definitions(${target} PRIVATE IMGUI_NVIDIA)
    if(EXISTS "${OVEXTS_TARGET_DEPS}/imgui")
        target_include_directories(${target} PRIVATE "${OVEXTS_TARGET_DEPS}/imgui")
    endif()
    ovexts_link_kit_ext_libs(${target} "omni.kit.renderer.imgui")
    target_link_libraries(${target} PRIVATE imgui)
endfunction()
