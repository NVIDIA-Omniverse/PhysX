# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Windows host-toolchain helpers, shared by build.cmake and the standalone
# consumer builds (C++ samples, source-link test).

# Resolve the include/lib/bin layout of an MSVC + WinSDK pair (packaged or
# local VS). Sets in the caller's scope: MSVC_TOOLS_DIR, MSVC_BIN_DIR,
# MSVC_INCLUDE, MSVC_LIB, WINSDK_BIN_DIR, WINSDK_UCRT_INCLUDE,
# WINSDK_UM_INCLUDE, WINSDK_SHARED_INCLUDE, WINSDK_UCRT_LIB, WINSDK_UM_LIB.
function(ovphysx_resolve_msvc_layout MSVC_ROOT WINSDK_ROOT)
    # MSVC tools version (e.g. 14.29.30133); use the newest.
    file(GLOB _msvc_tools_dirs "${MSVC_ROOT}/VC/Tools/MSVC/*")
    if(NOT _msvc_tools_dirs)
        message(FATAL_ERROR "MSVC tools not found in ${MSVC_ROOT}/VC/Tools/MSVC/")
    endif()
    list(SORT _msvc_tools_dirs)
    list(GET _msvc_tools_dirs -1 _msvc_tools_dir)
    message(STATUS "  Found MSVC tools: ${_msvc_tools_dir}")

    if(EXISTS "${WINSDK_ROOT}/Include/ucrt")
        # Flat structure (packaged WinSDK)
        set(_winsdk_include_base "${WINSDK_ROOT}/Include")
        set(_winsdk_lib_base "${WINSDK_ROOT}/Lib")
        set(_winsdk_bin_base "${WINSDK_ROOT}/bin")
        message(STATUS "  Found WinSDK (flat structure)")
    else()
        # Versioned structure (full WinSDK installation); use the newest.
        file(GLOB _winsdk_version_dirs "${WINSDK_ROOT}/Include/10.*")
        if(NOT _winsdk_version_dirs)
            message(FATAL_ERROR "WinSDK include not found in ${WINSDK_ROOT}/Include/")
        endif()
        list(SORT _winsdk_version_dirs)
        list(GET _winsdk_version_dirs -1 _winsdk_version_dir)
        get_filename_component(_winsdk_version "${_winsdk_version_dir}" NAME)
        set(_winsdk_include_base "${WINSDK_ROOT}/Include/${_winsdk_version}")
        set(_winsdk_lib_base "${WINSDK_ROOT}/Lib/${_winsdk_version}")
        set(_winsdk_bin_base "${WINSDK_ROOT}/bin/${_winsdk_version}")
        message(STATUS "  Found WinSDK version: ${_winsdk_version}")
    endif()

    set(MSVC_TOOLS_DIR "${_msvc_tools_dir}" PARENT_SCOPE)
    set(MSVC_BIN_DIR "${_msvc_tools_dir}/bin/Hostx64/x64" PARENT_SCOPE)
    set(MSVC_INCLUDE "${_msvc_tools_dir}/include" PARENT_SCOPE)
    set(MSVC_LIB "${_msvc_tools_dir}/lib/x64" PARENT_SCOPE)
    set(WINSDK_BIN_DIR "${_winsdk_bin_base}/x64" PARENT_SCOPE)
    set(WINSDK_UCRT_INCLUDE "${_winsdk_include_base}/ucrt" PARENT_SCOPE)
    set(WINSDK_UM_INCLUDE "${_winsdk_include_base}/um" PARENT_SCOPE)
    set(WINSDK_SHARED_INCLUDE "${_winsdk_include_base}/shared" PARENT_SCOPE)
    set(WINSDK_UCRT_LIB "${_winsdk_lib_base}/ucrt/x64" PARENT_SCOPE)
    set(WINSDK_UM_LIB "${_winsdk_lib_base}/um/x64" PARENT_SCOPE)
endfunction()

# Pin a standalone CMake configure to the packaged host toolchain, appending
# generator/compiler args to the list named by OUT_ARGS and exporting
# INCLUDE/LIB/PATH (execute_process children inherit them).
#
# Those configures pass no generator, so CMake picks its default: the newest
# Visual Studio the VS Installer knows about, or NMake Makefiles when there is
# none. That builds the samples with whatever VS the machine happens to have
# instead of the packaged MSVC, and fails outright on a runner with no VS.
#
# No-op off Windows and without the packaged toolchain (public source drop),
# where a local Visual Studio is the intended toolchain.
function(ovphysx_pin_packaged_host_toolchain OUT_ARGS)
    if(NOT WIN32)
        return()
    endif()

    set(_msvc_root "${PROJECT_ROOT}/_build/host-deps/msvc")
    set(_winsdk_root "${PROJECT_ROOT}/_build/host-deps/winsdk")
    set(_ninja "${PROJECT_ROOT}/_build/host-deps/ninja/ninja.exe")
    if(NOT EXISTS "${_msvc_root}/VC/Tools/MSVC"
       OR NOT EXISTS "${_winsdk_root}/Include"
       OR NOT EXISTS "${_ninja}")
        message(STATUS "  Packaged host toolchain not present; "
                       "using the local Visual Studio for standalone builds")
        return()
    endif()

    ovphysx_resolve_msvc_layout("${_msvc_root}" "${_winsdk_root}")
    if(NOT EXISTS "${MSVC_BIN_DIR}/cl.exe")
        message(FATAL_ERROR "cl.exe not found at ${MSVC_BIN_DIR}\n"
                            "Run scripts/fetch_deps.bat to download host dependencies.")
    endif()

    # cl.exe needs its own directory on PATH (link.exe, mspdbcore.dll, the
    # toolset CRT) and the WinSDK bin for rc.exe/mt.exe.
    get_filename_component(_ninja_dir "${_ninja}" DIRECTORY)
    set(ENV{PATH} "${_ninja_dir};${MSVC_BIN_DIR};${WINSDK_BIN_DIR};$ENV{PATH}")
    set(ENV{INCLUDE} "${MSVC_INCLUDE};${WINSDK_UCRT_INCLUDE};${WINSDK_UM_INCLUDE};${WINSDK_SHARED_INCLUDE}")
    set(ENV{LIB} "${MSVC_LIB};${WINSDK_UCRT_LIB};${WINSDK_UM_LIB}")

    # --no-warn-unused-cli: a C-only sample never reads CMAKE_CXX_COMPILER.
    set(_args "${${OUT_ARGS}}")
    list(APPEND _args
        -G Ninja
        --no-warn-unused-cli
        "-DCMAKE_MAKE_PROGRAM=${_ninja}"
        "-DCMAKE_C_COMPILER=${MSVC_BIN_DIR}/cl.exe"
        "-DCMAKE_CXX_COMPILER=${MSVC_BIN_DIR}/cl.exe"
    )
    set(${OUT_ARGS} "${_args}" PARENT_SCOPE)
    message(STATUS "  Standalone builds pinned to packaged MSVC: ${MSVC_TOOLS_DIR}")
endfunction()
