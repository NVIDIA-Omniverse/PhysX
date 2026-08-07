# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Adds USD, TBB, and system library link dependencies to a target.
# This keeps the historical helper name used by ovruntime targets, but the
# dependency set is namespaced-USD-only and py-less.
#
# Usage:
#   ovruntime_link_usd_deps(<target>)

# ---------------------------------------------------------------------------
# usd_links(<target> <lib1> [<lib2> ...])
#
# Link the namespaced monolithic USD library to a target.
# Equivalent of the Premake5 usd_links{} helper in premake5-public.lua.
# ---------------------------------------------------------------------------
function(usd_links target)
    if(NOT DEFINED USD_DIR)
        return()
    endif()

    # We only need the *usd_ms library NAME (e.g. ov_25.11usd_ms), which is
    # identical in release/lib and debug/lib — only the binary differs. The
    # actual debug-vs-release selection at link time is handled by
    # _ovruntime_add_usd_link_dirs() via target_link_directories generator
    # expressions. So search whichever config dir is staged, preferring the
    # matching one so debug-only builds (which don't stage release/) succeed.
    if(DEFINED OVRUNTIME_RUNTIME_DEPS_CONFIG AND NOT OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "per-config")
        set(_usd_ms_cfg "${OVRUNTIME_RUNTIME_DEPS_CONFIG}")
    elseif(CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" _usd_ms_cfg)
    else()
        set(_usd_ms_cfg "release")
    endif()
    set(_usd_ms_search_order "${_usd_ms_cfg}")
    foreach(_alt release debug)
        if(NOT _alt STREQUAL _usd_ms_cfg)
            list(APPEND _usd_ms_search_order "${_alt}")
        endif()
    endforeach()
    set(_usd_ms_libs "")
    foreach(_cfg ${_usd_ms_search_order})
        if(WIN32)
            file(GLOB _usd_ms_libs "${USD_DIR}/${_cfg}/lib/*usd_ms.lib")
        else()
            file(GLOB _usd_ms_libs "${USD_DIR}/${_cfg}/lib/lib*usd_ms.so")
        endif()
        if(_usd_ms_libs)
            break()
        endif()
    endforeach()

    if(NOT _usd_ms_libs)
        message(FATAL_ERROR
            "ovruntime source builds require namespaced monolithic USD in ${USD_DIR}/{release,debug}/lib, "
            "but no *usd_ms library was found.")
    endif()

    list(GET _usd_ms_libs 0 _usd_ms_path)
    get_filename_component(_usd_ms_name "${_usd_ms_path}" NAME)
    if(WIN32)
        string(REGEX REPLACE "\\.lib$" "" _usd_ms_name "${_usd_ms_name}")
    else()
        string(REGEX REPLACE "\\.so$" "" _usd_ms_name "${_usd_ms_name}")
        string(REGEX REPLACE "^lib" "" _usd_ms_name "${_usd_ms_name}")
    endif()
    target_link_libraries(${target} PRIVATE ${_usd_ms_name})
endfunction()

function(_ovruntime_add_usd_link_dirs _target)
    if(NOT DEFINED USD_DIR)
        return()
    endif()

    set(_USD_RELEASE_LIB_DIR "${USD_DIR}/release/lib")
    set(_USD_DEBUG_LIB_DIR "${USD_DIR}/debug/lib")

    if(DEFINED OVRUNTIME_RUNTIME_DEPS_CONFIG AND NOT OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "per-config")
        set(_USD_SELECTED_LIB_DIR "${USD_DIR}/${OVRUNTIME_RUNTIME_DEPS_CONFIG}/lib")
        if(EXISTS "${_USD_SELECTED_LIB_DIR}")
            target_link_directories(${_target} PRIVATE "${_USD_SELECTED_LIB_DIR}")
        else()
            message(FATAL_ERROR
                "Selected USD library directory does not exist: ${_USD_SELECTED_LIB_DIR}. "
                "Fetch ${OVRUNTIME_RUNTIME_DEPS_CONFIG} runtime dependencies before configuring.")
        endif()
    elseif(EXISTS "${_USD_RELEASE_LIB_DIR}" AND EXISTS "${_USD_DEBUG_LIB_DIR}")
        target_link_directories(${_target} PRIVATE
            $<$<CONFIG:Debug>:${_USD_DEBUG_LIB_DIR}>
            $<$<NOT:$<CONFIG:Debug>>:${_USD_RELEASE_LIB_DIR}>
        )
    elseif(EXISTS "${_USD_DEBUG_LIB_DIR}")
        target_link_directories(${_target} PRIVATE "${_USD_DEBUG_LIB_DIR}")
    elseif(EXISTS "${_USD_RELEASE_LIB_DIR}")
        target_link_directories(${_target} PRIVATE "${_USD_RELEASE_LIB_DIR}")
    endif()
endfunction()

function(_ovruntime_link_tbb _target)
    if(WIN32 AND DEFINED USD_DIR)
        set(_TBB_RELEASE_LIB "")
        foreach(_CANDIDATE "${USD_DIR}/release/lib/tbb.lib" "${USD_DIR}/release/lib/tbb12.lib")
            if(EXISTS "${_CANDIDATE}")
                set(_TBB_RELEASE_LIB "${_CANDIDATE}")
                break()
            endif()
        endforeach()

        set(_TBB_DEBUG_LIB "")
        foreach(_CANDIDATE "${USD_DIR}/debug/lib/tbb_debug.lib" "${USD_DIR}/debug/lib/tbb12_debug.lib")
            if(EXISTS "${_CANDIDATE}")
                set(_TBB_DEBUG_LIB "${_CANDIDATE}")
                break()
            endif()
        endforeach()

        if(DEFINED OVRUNTIME_RUNTIME_DEPS_CONFIG AND NOT OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "per-config")
            if(OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "debug" AND _TBB_DEBUG_LIB)
                target_link_libraries(${_target} PRIVATE "${_TBB_DEBUG_LIB}")
                return()
            endif()
            if(OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "release" AND _TBB_RELEASE_LIB)
                target_link_libraries(${_target} PRIVATE "${_TBB_RELEASE_LIB}")
                return()
            endif()
        endif()

        if(_TBB_RELEASE_LIB AND _TBB_DEBUG_LIB)
            target_link_libraries(${_target} PRIVATE
                $<$<CONFIG:Debug>:${_TBB_DEBUG_LIB}>
                $<$<NOT:$<CONFIG:Debug>>:${_TBB_RELEASE_LIB}>
            )
            return()
        endif()
        if(_TBB_DEBUG_LIB)
            target_link_libraries(${_target} PRIVATE "${_TBB_DEBUG_LIB}")
            return()
        endif()
        if(_TBB_RELEASE_LIB)
            target_link_libraries(${_target} PRIVATE "${_TBB_RELEASE_LIB}")
            return()
        endif()
    endif()

    # Linux: link TBB from the selected runtime-deps config.
    if(DEFINED OVRUNTIME_RUNTIME_DEPS_CONFIG AND NOT OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "per-config")
        if(OVRUNTIME_RUNTIME_DEPS_CONFIG STREQUAL "debug")
            target_link_libraries(${_target} PRIVATE tbb_debug)
        else()
            target_link_libraries(${_target} PRIVATE tbb)
        endif()
        return()
    elseif(CMAKE_CONFIGURATION_TYPES)
        target_link_libraries(${_target} PRIVATE
            $<$<CONFIG:Debug>:tbb_debug>
            $<$<NOT:$<CONFIG:Debug>>:tbb>
        )
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_libraries(${_target} PRIVATE tbb_debug)
    else()
        target_link_libraries(${_target} PRIVATE tbb)
    endif()
endfunction()

function(ovruntime_link_usd_deps _target)
    # USD library directories
    _ovruntime_add_usd_link_dirs(${_target})

    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        target_link_directories(${_target} PRIVATE
            ${USD_EXT_PHYSICS_DIR}/lib
        )
        # Multi-config: also add debug lib dir so debug builds find debug schema libs.
        if(CMAKE_CONFIGURATION_TYPES AND DEFINED USD_EXT_PHYSICS_BASE)
            foreach(_ext_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
                string(TOLOWER "${_ext_cfg}" _ext_cfg_lower)
                if(EXISTS "${USD_EXT_PHYSICS_BASE}/${_ext_cfg_lower}/lib")
                    target_link_directories(${_target} PRIVATE
                        "${USD_EXT_PHYSICS_BASE}/${_ext_cfg_lower}/lib"
                    )
                endif()
            endforeach()
        endif()
    endif()

    # Kit SDK omni.usd.core (provides omni.usd runtime library)
    if(DEFINED OVRUNTIME_KIT_SDK_DIR AND EXISTS "${OVRUNTIME_KIT_SDK_DIR}/exts/omni.usd.core/bin")
        target_link_directories(${_target} PRIVATE
            ${OVRUNTIME_KIT_SDK_DIR}/exts/omni.usd.core/bin
        )
    endif()

    # Link the namespaced USD monolith.
    usd_links(${_target}
        ar arch gf js kind pcp plug
        sdf tf ts trace usd usdGeom
        usdSkel usdShade vt work pxOsd
        hdx hd usdImaging usdLux usdUtils
        usdPhysics
    )

    # TBB
    _ovruntime_link_tbb(${_target})

    # Linux runtime plugins still need the usual system loader/thread libs even
    # when the shipped package is otherwise py-less.
    if(NOT WIN32)
        target_link_libraries(${_target} PRIVATE dl pthread)
    endif()
endfunction()
