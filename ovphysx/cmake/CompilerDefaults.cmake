# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Compiler-specific settings, aligned with the ovruntime premake (omni.physics.plugin).

if(MSVC)
    # Character set: Unicode (matches premake)
    add_compile_definitions(UNICODE _UNICODE)

    # Warning level 4, treat warnings as errors.
    # The COMPILE_LANGUAGE:C,CXX guard keeps these MSVC-specific flags away from
    # nvcc, which reads them as filenames and fails with "A single input file is required".
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:/W4>
        $<$<COMPILE_LANGUAGE:C,CXX>:/WX>
    )

    # RTTI enabled (matches premake /GR)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:/GR>)

    # Multi-processor compilation, debug info
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:/MP>
        $<$<COMPILE_LANGUAGE:C,CXX>:/Zi>
    )

    # Additional flags matching premake
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:/bigobj>
        $<$<COMPILE_LANGUAGE:C,CXX>:/Zc:__cplusplus>
        $<$<COMPILE_LANGUAGE:C,CXX>:/guard:cf>
        $<$<COMPILE_LANGUAGE:C,CXX>:/utf-8>
        $<$<COMPILE_LANGUAGE:C,CXX>:/permissive->
    )

    # Preprocessor definitions
    add_compile_definitions(
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        # Suppress the TBB auto-link pragma (#pragma comment(lib, "tbb12[_debug].lib")).
        # The USD packman only ships release TBB libs (no tbb12_debug.lib), so the
        # explicit tbb link in ovruntime_link_usd_deps() is used instead.
        __TBB_NO_IMPLICIT_LINKAGE=1
    )

    # Release-specific optimizations (function-level linking, intrinsics, string pooling)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Gy /Oi /GF")

    # Release linker: dead-code elimination, COMDAT folding, full PDB
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /OPT:REF /OPT:ICF /DEBUG:FULL")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /OPT:REF /OPT:ICF /DEBUG:FULL")

    # Debug linker: full PDB
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG:FULL")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG:FULL")

    # Suppress benign linker warnings:
    #   LNK4098: LIBCMT vs MSVCRT CRT conflict. Pre-built PhysX static libs use /MT
    #            and the ovphysx DLLs use /MD. The linker picks the correct one.
    #   LNK4199: /DELAYLOAD ignored. Targets link cuda.lib for optional GPU support
    #            but may not reference nvcuda.dll symbols directly.
    add_link_options(/ignore:4098 /ignore:4199)

    # Subsystem: Windows (matches premake)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SUBSYSTEM:WINDOWS")

elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # GCC/Clang compiler flags. The COMPILE_LANGUAGE:C,CXX guard keeps them away
    # from nvcc, which does not understand GCC warning flags.
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wextra>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Werror>
        $<$<COMPILE_LANGUAGE:C,CXX>:-fvisibility=hidden>
    )
endif()

# Optimization flags (CRT handled by NvidiaBuildOptions via CMAKE_MSVC_RUNTIME_LIBRARY)
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()
