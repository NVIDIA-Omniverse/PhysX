# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Include guard — safe to include from multiple CMakeLists files.
if(DEFINED _OVRUNTIME_CUDA_UTILS_INCLUDED)
    return()
endif()
set(_OVRUNTIME_CUDA_UTILS_INCLUDED 1)

if(NOT DEFINED CUDA_DIR)
    message(WARNING "CudaUtils: CUDA_DIR not set — CUDA targets will not build correctly")
    return()
endif()

# --- Global CUDA setup ---
# CUDA architectures targeted by all ovruntime CUDA builds.
# Every listed arch is compiled to SASS. The highest arch additionally gets a
# PTX entry on the Windows custom-command path (see ovruntime_target_add_cuda_sources
# below); on Linux, CMAKE_CUDA_ARCHITECTURES with plain numbers already emits PTX
# for every arch. The PTX entry is what lets the driver JIT-compile for GPUs
# released after this list was last updated (forward compatibility). Mirrors the
# gencode pattern used by PhysX in physx/source/compiler/cmakegpu/CMakeLists.txt.
# Update this list when a new compute capability ships and bump the highest arch
# accordingly. Requires nvcc >= 12.8 for sm_100 / sm_120 (Blackwell).
set(OVRUNTIME_CUDA_ARCHITECTURES 52 53 60 61 62 70 72 75 80 86 87 89 90 100 120)

if(WIN32)
    # On Windows with the Visual Studio generator, enable_language(CUDA) requires the VS
    # CUDA toolkit integration which is not present when CUDA comes from packman.
    # Instead, .cu files are compiled via add_custom_command (direct nvcc invocation).
    #
    # --use-local-env is required because nvcc resolves vcvars64.bat relative to the
    # -ccbin cl.exe path. The packaged MSVC toolchain includes vcvars64.bat but it fails
    # to execute without a full VS installation. build.cmake already sets INCLUDE, LIB,
    # PATH, and VCINSTALLDIR correctly, so --use-local-env tells nvcc to use the
    # inherited environment instead of trying to run vcvars64.bat.
    set(_OVRUNTIME_NVCC "${CUDA_DIR}/bin/nvcc.exe")
else()
    set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc" CACHE FILEPATH "" FORCE)
    enable_language(CUDA)
    set(CMAKE_CUDA_ARCHITECTURES ${OVRUNTIME_CUDA_ARCHITECTURES})
endif()

# --- ovruntime_target_add_cuda_sources() ---
#
# Adds CUDA source files to a target, handling the platform difference:
#   Linux:   adds .cu files as native CMake CUDA-language sources and sets
#            CUDA_SEPARABLE_COMPILATION + -Xcompiler=-fPIC.
#   Windows: compiles each .cu via direct nvcc invocation (add_custom_command),
#            bypassing the VS CUDA toolkit integration, then links the resulting
#            .obj files into the target.
#
# The standard ovruntime include paths (CUDA, Carbonite, PhysX, ovruntime headers)
# are added automatically.  Use INCLUDE_DIRS for target-specific extras.
#
# Usage:
#   ovruntime_target_add_cuda_sources(<target>
#       SOURCES     <file.cu>...
#       [INCLUDE_DIRS <dir>...]
#   )
function(ovruntime_target_add_cuda_sources _target)
    cmake_parse_arguments(ARG "" "" "SOURCES;INCLUDE_DIRS" ${ARGN})

    if(WIN32)
        # Build -gencode flags: SASS (code=sm_X) for every listed arch, plus a
        # PTX (code=compute_X) entry for the highest arch so the driver can
        # JIT-compile for newer GPUs not in the list (forward compatibility).
        # Without this PTX fallback, anything newer than the highest sm_X here
        # fails the first kernel launch with cudaErrorNoKernelImageForDevice.
        set(_arch_flags "")
        foreach(_sm IN LISTS OVRUNTIME_CUDA_ARCHITECTURES)
            list(APPEND _arch_flags "-gencode" "arch=compute_${_sm},code=sm_${_sm}")
        endforeach()
        list(GET OVRUNTIME_CUDA_ARCHITECTURES -1 _highest_sm)
        list(APPEND _arch_flags "-gencode"
            "arch=compute_${_highest_sm},code=compute_${_highest_sm}")

        # Standard ovruntime include paths — present for every CUDA target.
        set(_std_incs
            "${OVRUNTIME_ROOT_DIR}/include"
            "${OVRUNTIME_ROOT_DIR}/include/private"
            "${OVRUNTIME_ROOT_DIR}/source/common/include"
            "${CUDA_DIR}/include"
        )
        if(DEFINED CARB_SDK_DIR)
            list(APPEND _std_incs "${CARB_SDK_DIR}/include")
        endif()
        if(DEFINED PHYSX_SDK_DIR)
            list(APPEND _std_incs "${PHYSX_SDK_DIR}/include")
        endif()
        if(DEFINED OVRUNTIME_DEPS_DIR AND EXISTS "${OVRUNTIME_DEPS_DIR}/include")
            list(APPEND _std_incs "${OVRUNTIME_DEPS_DIR}/include")
        endif()
        if(DEFINED OVRUNTIME_KIT_SDK_DIR AND EXISTS "${OVRUNTIME_KIT_SDK_DIR}/dev/include")
            list(APPEND _std_incs "${OVRUNTIME_KIT_SDK_DIR}/dev/include")
        endif()
        if(DEFINED OVRUNTIME_KIT_SDK_DIR AND EXISTS "${OVRUNTIME_KIT_SDK_DIR}/dev/fabric/include")
            list(APPEND _std_incs "${OVRUNTIME_KIT_SDK_DIR}/dev/fabric/include")
        endif()

        set(_inc_flags "")
        foreach(_dir IN LISTS _std_incs ARG_INCLUDE_DIRS)
            list(APPEND _inc_flags "-I" "${_dir}")
        endforeach()

        foreach(_cu IN LISTS ARG_SOURCES)
            get_filename_component(_cu_name "${_cu}" NAME_WE)
            # Flat output path — nvcc is the only COMMAND so its exit code drives build failure.
            set(_obj "${CMAKE_CURRENT_BINARY_DIR}/${_cu_name}_$<CONFIG>.obj")
            add_custom_command(
                OUTPUT "${_obj}"
                COMMAND "${_OVRUNTIME_NVCC}"
                    -c "${_cu}" -o "${_obj}"
                    -ccbin "${CMAKE_CXX_COMPILER}"
                    --use-local-env
                    --std=c++17
                    --compiler-options=/std:c++17
                    # Suppress benign nvcc warnings from Windows SDK headers:
                    #   174-D: expression has no effect         (winnt.h TpInitializeCallbackEnviron)
                    #   108-D: signed bit field of length 1     (winuser.h TITLEBARINFO::fBarFocused)
                    #  1835-D: attribute "dllimport" does not apply (wincrypt.h PFN_CERT_IS_WEAK_HASH)
                    --diag-suppress=174,108,1835
                    # Suppress deprecated-gpu-targets warning for pre-sm_75 architectures
                    # (kept for broad HW compatibility; will be removed when min CUDA is bumped)
                    -Wno-deprecated-gpu-targets
                    ${_arch_flags}
                    ${_inc_flags}
                    "$<$<CONFIG:Debug>:-G;-g;-O0;--compiler-options=/MDd>"
                    "$<$<OR:$<CONFIG:Release>,$<CONFIG:checked>>:--compiler-options=/MD;-DNDEBUG>"
                DEPENDS "${_cu}"
                COMMENT "Compiling CUDA: ${_cu_name}.cu ($<CONFIG>)"
                COMMAND_EXPAND_LISTS
            )
            target_sources(${_target} PRIVATE "${_obj}")
        endforeach()
    else()
        target_sources(${_target} PRIVATE ${ARG_SOURCES})
        set_target_properties(${_target} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
        target_compile_options(${_target} PRIVATE
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-fPIC>
        )
    endif()
endfunction()
