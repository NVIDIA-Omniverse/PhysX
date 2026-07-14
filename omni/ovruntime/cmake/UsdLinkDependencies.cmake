# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Adds USD, TBB, Python, and system library link dependencies to a target.
# This matches what the premake extension_usd_deps() function provides.
#
# Usage:  ovruntime_link_usd_deps(<target>)

# ---------------------------------------------------------------------------
# usd_links(<target> <lib1> [<lib2> ...])
#
# Link USD libraries to a target, handling monolithic vs. non-monolithic USD.
# Equivalent of the Premake5 usd_links{} helper in premake5-public.lua.
#
# For monolithic builds (namespaced USD), a single *usd_ms library is present
# in the USD lib dir; link that instead of individual libs.
# For non-monolithic builds, each name is prefixed with "usd_" and linked.
# ---------------------------------------------------------------------------
function(usd_links target)
    if(NOT DEFINED USD_DIR)
        return()
    endif()

    if(WIN32)
        file(GLOB _usd_ms_libs "${USD_DIR}/release/lib/*usd_ms.lib")
    else()
        file(GLOB _usd_ms_libs "${USD_DIR}/release/lib/lib*usd_ms.so")
    endif()

    if(_usd_ms_libs)
        # Monolithic: link the single *usd_ms library (e.g. ov_25.11usd_ms).
        list(GET _usd_ms_libs 0 _usd_ms_path)
        get_filename_component(_usd_ms_name "${_usd_ms_path}" NAME)
        if(WIN32)
            string(REGEX REPLACE "\\.lib$" "" _usd_ms_name "${_usd_ms_name}")
        else()
            string(REGEX REPLACE "\\.so$" "" _usd_ms_name "${_usd_ms_name}")
            string(REGEX REPLACE "^lib" "" _usd_ms_name "${_usd_ms_name}")
        endif()
        target_link_libraries(${target} PRIVATE ${_usd_ms_name})
    else()
        # Non-monolithic: prefix each lib name with "usd_" and link individually.
        set(_prefixed_libs "")
        foreach(_lib IN LISTS ARGN)
            list(APPEND _prefixed_libs "usd_${_lib}")
        endforeach()
        target_link_libraries(${target} PRIVATE ${_prefixed_libs})
    endif()
endfunction()

function(_ovruntime_add_usd_link_dirs _target)
    if(NOT DEFINED USD_DIR)
        return()
    endif()

    set(_USD_RELEASE_LIB_DIR "${USD_DIR}/release/lib")
    set(_USD_DEBUG_LIB_DIR "${USD_DIR}/debug/lib")

    if(EXISTS "${_USD_RELEASE_LIB_DIR}" AND EXISTS "${_USD_DEBUG_LIB_DIR}")
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

    # Linux: link tbb_debug for debug, tbb for release (matching premake add_tbb_deps).
    if(CMAKE_CONFIGURATION_TYPES)
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

    # Python library directory (Windows: libs/python312.lib, Linux: lib/libpython3.12.so)
    if(DEFINED PYTHON_DIR)
        if(WIN32)
            target_link_directories(${_target} PRIVATE ${PYTHON_DIR}/libs)
        else()
            target_link_directories(${_target} PRIVATE ${PYTHON_DIR}/lib)
        endif()
    endif()

    # Link USD libraries — handles monolithic vs. non-monolithic automatically.
    # Python symbols are included in the monolithic lib; usd_python is only linked
    # for non-monolithic builds.
    usd_links(${_target}
        ar arch gf js kind pcp plug
        sdf tf ts trace usd usdGeom
        usdSkel usdShade vt work pxOsd
        hdx hd usdImaging usdLux usdUtils
        usdPhysics boost python
    )

    # TBB
    _ovruntime_link_tbb(${_target})

    # Platform-specific: Python, dl, pthread on Linux
    if(NOT WIN32)
        target_link_libraries(${_target} PRIVATE python3.12 dl pthread)
    endif()
endfunction()

# Adds Python + USD boost::python link dependencies to a bindings target.
# Bindings include UsdPCH.h which pulls in boost::python symbols from libusd_python.so.
#
# Usage:  ovruntime_link_bindings_deps(<target>)

function(ovruntime_link_bindings_deps _target)
    if(WIN32)
        # On Windows, pybind11 .pyd modules must have all DLL imports resolved at link time.
        # Reuse the full USD dependency set (same as plugin targets), then add physxSchema.
        ovruntime_link_usd_deps(${_target})
        if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
            target_link_libraries(${_target} PRIVATE physicsSchemaTools physxSchema)
        endif()
    else()
        if(DEFINED PYTHON_DIR)
            target_link_directories(${_target} PRIVATE ${PYTHON_DIR}/lib)
            # Derive the link library from PYTHON_DIR so that overrides
            # (e.g. py311 pass from ovphysx) link the matching libpython.
            file(GLOB _pyinc_dirs "${PYTHON_DIR}/include/python3.*")
            if(_pyinc_dirs)
                list(GET _pyinc_dirs 0 _pyinc)
                get_filename_component(_pyver "${_pyinc}" NAME)
                target_link_libraries(${_target} PRIVATE ${_pyver})
            else()
                message(WARNING "Could not detect Python version from ${PYTHON_DIR}/include; assuming 3.12")
                target_link_libraries(${_target} PRIVATE python3.12)
            endif()
        endif()
        _ovruntime_add_usd_link_dirs(${_target})
        if(DEFINED USD_DIR)
            # Link pxr_boost::python symbols: usd_python for non-monolithic builds,
            # *usd_ms for monolithic (namespaced) builds.  Without an explicit link
            # the namespaced ovInternal_v0_25_N::pxr_boost symbols are unresolved
            # at dlopen time even if *usd_ms.so is already in the process image.
            usd_links(${_target} python)
        endif()
        # TBB: bindings include UsdPCH.h which pulls TBB symbols via tbb::detail::r1.
        # Without an explicit link, the .so has an undefined TBB symbol but no
        # NEEDED entry, so Python dlopen fails unless libtbb is already loaded.
        _ovruntime_link_tbb(${_target})
    endif()
endfunction()
