# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# PhysX SDK dependency management for ovruntime.
#
# When OVRUNTIME_DEV_PHYSX is ON, PhysX is built from source via add_subdirectory.
# When OFF (default), pre-built PhysX static libraries from packman are used.
#
# Provides:
#   ovruntime_link_physx_static_libs(<target> [FULL|MINIMAL])
#     Links PhysX static libraries to <target>.
#     FULL (default): all PhysX libs (Extensions, Vehicle, CharacterKinematic, Cooking, etc.)
#     MINIMAL: only PhysX, PhysXCommon, PhysXFoundation

if(OVRUNTIME_DEV_PHYSX AND NOT TARGET PhysX)
    # --- Build PhysX from source ---
    set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../physx" CACHE PATH "PhysX SDK root directory")
    get_filename_component(PHYSX_ROOT_DIR "${PHYSX_ROOT_DIR}" ABSOLUTE)

    if(EXISTS "${PHYSX_ROOT_DIR}/compiler/internal/CMakeLists.txt")
        set(_OVRUNTIME_PHYSX_SOURCE_LAYOUT "internal")
    elseif(EXISTS "${PHYSX_ROOT_DIR}/source/compiler/cmake/CMakeLists.txt")
        set(_OVRUNTIME_PHYSX_SOURCE_LAYOUT "public")
    else()
        message(FATAL_ERROR "OVRUNTIME_DEV_PHYSX is ON but PhysX source not found at ${PHYSX_ROOT_DIR}")
    endif()

    message(STATUS "OVRUNTIME_DEV_PHYSX: Building PhysX from ${_OVRUNTIME_PHYSX_SOURCE_LAYOUT} source at ${PHYSX_ROOT_DIR}")

    # Public release validation must prove that the shipped public PhysX source
    # tree works. Internal developers may still use the monorepo PhysX layout,
    # but OSS CI sets this flag so an accidental internal-layout build fails
    # loudly instead of giving a false public-readiness signal.
    set(_OVRUNTIME_REQUIRE_PUBLIC_PHYSX OFF)
    if(DEFINED OVRUNTIME_REQUIRE_PUBLIC_PHYSX)
        set(_OVRUNTIME_REQUIRE_PUBLIC_PHYSX "${OVRUNTIME_REQUIRE_PUBLIC_PHYSX}")
    elseif(DEFINED ENV{OVPHYSX_REQUIRE_PUBLIC_PHYSX})
        set(_OVRUNTIME_REQUIRE_PUBLIC_PHYSX "$ENV{OVPHYSX_REQUIRE_PUBLIC_PHYSX}")
    endif()
    if(_OVRUNTIME_REQUIRE_PUBLIC_PHYSX AND _OVRUNTIME_PHYSX_SOURCE_LAYOUT STREQUAL "internal")
        message(FATAL_ERROR
            "OVRUNTIME_DEV_PHYSX is using the internal PhysX layout at ${PHYSX_ROOT_DIR}, "
            "but public PhysX source was required. Use the official PhysX public source "
            "distro zip from generate_and_package_public_distro for open-source validation."
        )
    endif()

    set(PHYSX_CMAKE_MODULES_PATH "${PHYSX_ROOT_DIR}/source/compiler/cmake/modules" CACHE INTERNAL "Path to PhysX CMake Modules" FORCE)
    list(APPEND CMAKE_MODULE_PATH "${PHYSX_CMAKE_MODULES_PATH}")

    # PhysX needs the CUDA compiler for GPU projects.  Point cmake at the
    # packman-provided nvcc before PhysX calls ENABLE_LANGUAGE(CUDA).
    if(DEFINED CUDA_DIR)
        if(WIN32)
            if(NOT CMAKE_CUDA_COMPILER)
                set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc.exe" CACHE FILEPATH "" FORCE)
            endif()
            # Packaged MSVC's vcvars64.bat can fail when nvcc invokes it — observed
            # in Ninja/CI when the runner PATH contains entries that break batch
            # parsing. --use-local-env tells nvcc to use the surrounding environment
            # (INCLUDE/LIB/PATH/VCINSTALLDIR) instead of running vcvars64.bat. ovphysx
            # build.cmake seeds those vars for Ninja before CMake runs; the VS generator
            # uses -T cuda= and CMAKE_VS_SDK_* for MSBuild, but configure-time CUDA
            # detection still invokes nvcc, so apply this on all Windows generators
            # (CudaUtils.cmake).
            if(CMAKE_CXX_COMPILER AND NOT CMAKE_CUDA_HOST_COMPILER)
                set(CMAKE_CUDA_HOST_COMPILER "${CMAKE_CXX_COMPILER}" CACHE FILEPATH "" FORCE)
            endif()
            if(NOT CMAKE_CUDA_FLAGS MATCHES "--use-local-env")
                if(CMAKE_CUDA_FLAGS)
                    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use-local-env" CACHE STRING "" FORCE)
                else()
                    set(CMAKE_CUDA_FLAGS "--use-local-env" CACHE STRING "" FORCE)
                endif()
            endif()
        elseif(NOT CMAKE_CUDA_COMPILER)
            set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc" CACHE FILEPATH "" FORCE)
        endif()
        # Help PhysX's FIND_PACKAGE(CUDAToolkit) resolve the same packman toolkit.
        set(CUDAToolkit_ROOT "${CUDA_DIR}" CACHE PATH "" FORCE)
        set(CUDAToolkit_ROOT_DIR "${CUDA_DIR}" CACHE PATH "" FORCE)
    endif()
    if(DEFINED ENV{PM_SECURELOADLIBRARY_PATH} AND EXISTS "$ENV{PM_SECURELOADLIBRARY_PATH}/src/nvSecureLoadLibrary.c")
        set(SECURELOADLIBRARYPATH "$ENV{PM_SECURELOADLIBRARY_PATH}" CACHE INTERNAL "Secure load library path" FORCE)
    endif()

    # Set PhysX cmake options matching the carbonite presets
    set(PX_BUILDUNITTESTS OFF CACHE BOOL "" FORCE)
    set(PX_BUILDBENCHMARKS OFF CACHE BOOL "" FORCE)
    set(PX_BUILDVISUALTESTS OFF CACHE BOOL "" FORCE)
    set(PX_BUILDPVDRUNTIME ON CACHE BOOL "" FORCE)
    set(PX_BUILDSNIPPETS OFF CACHE BOOL "" FORCE)
    set(PX_BUILDVHACD ON CACHE BOOL "" FORCE)
    set(PX_CMAKE_SUPPRESS_REGENERATION OFF CACHE BOOL "" FORCE)
    set(PX_SCALAR_MATH OFF CACHE BOOL "" FORCE)
    set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "" FORCE)

    # GPU build is gated on CUDA being available. PhysX's add_subdirectory entry
    # points used below (compiler/cmake and compiler/cmakegpu) bypass
    # physx/CMakeLists.txt, which is where the standalone-build flow enables
    # CUDA and resolves the toolkit. On Windows, cmakegpu/windows/PhysXGpu.cmake
    # links against CUDA::cuda_driver via /DELAYLOAD:nvcuda.dll, so the IMPORTED
    # target must exist by the time cmakegpu is added.
    #
    # Detect CUDA without hard-failing when it's absent (e.g. the public
    # source distro strips CUDA from packman deps). Pattern:
    #   1. check_language(CUDA) probes for an nvcc compiler without enabling.
    #   2. If found, enable_language(CUDA) so FindCUDAToolkit can derive
    #      CUDAToolkit_BIN_DIR from CMAKE_CUDA_COMPILER_TOOLKIT_ROOT.
    #   3. find_package(CUDAToolkit QUIET) to create the IMPORTED targets.
    #   4. Gate PX_GENERATE_GPU_PROJECTS on whether CUDA::cuda_driver
    #      materialized — a partial CUDA install missing the driver stub
    #      shouldn't enable a build that links against that target.
    # Sources: packman-managed CUDA via CUDA_DIR (set by ovruntime/CMakeLists)
    # in local --devphysx, or a host CUDA Toolkit install on PATH/CUDA_PATH.
    include(CheckLanguage)
    check_language(CUDA)
    set(_OVRUNTIME_DEV_PHYSX_GPU_DEFAULT OFF)
    set(_OVRUNTIME_DEV_PHYSX_CUDA_STATUS "no CUDA compiler found")
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        find_package(CUDAToolkit QUIET)
        if(TARGET CUDA::cuda_driver)
            set(_OVRUNTIME_DEV_PHYSX_GPU_DEFAULT ON)
            set(_OVRUNTIME_DEV_PHYSX_CUDA_STATUS "CUDAToolkit ${CUDAToolkit_VERSION} at ${CUDAToolkit_LIBRARY_DIR}")
        else()
            set(_OVRUNTIME_DEV_PHYSX_CUDA_STATUS "CUDAToolkit found but CUDA::cuda_driver target missing (incomplete install?)")
        endif()
    endif()

    # Detection-driven default; not FORCE'd so a user can override via
    # -DPX_GENERATE_GPU_PROJECTS=ON|OFF on the cmake command line. The sanity
    # check below catches "=ON without CUDA::cuda_driver available" so they
    # don't fall into a confusing target-not-found error inside
    # cmakegpu/windows/PhysXGpu.cmake.
    set(PX_GENERATE_GPU_PROJECTS ${_OVRUNTIME_DEV_PHYSX_GPU_DEFAULT} CACHE BOOL "")
    if(PX_GENERATE_GPU_PROJECTS AND NOT TARGET CUDA::cuda_driver)
        message(FATAL_ERROR
            "PX_GENERATE_GPU_PROJECTS=ON but CUDA::cuda_driver is not available "
            "(${_OVRUNTIME_DEV_PHYSX_CUDA_STATUS}). "
            "Install a CUDA Toolkit (Windows: full NVIDIA installer; Linux: "
            "system package or set CMAKE_CUDA_COMPILER), or pass "
            "-DPX_GENERATE_GPU_PROJECTS=OFF to build CPU-only."
        )
    endif()
    if(PX_GENERATE_GPU_PROJECTS)
        message(STATUS "OVRUNTIME_DEV_PHYSX: GPU build enabled — ${_OVRUNTIME_DEV_PHYSX_CUDA_STATUS}")
    else()
        message(STATUS "OVRUNTIME_DEV_PHYSX: GPU build disabled (CUDA: ${_OVRUNTIME_DEV_PHYSX_CUDA_STATUS})")
    endif()
    set(PX_GENERATE_GPU_PROJECTS_ONLY OFF CACHE BOOL "" FORCE)
    set(PX_GENERATE_GPU_STATIC_LIBRARIES OFF CACHE BOOL "" FORCE)
    set(PX_GENERATE_GPU_REDUCED_ARCHITECTURES OFF CACHE BOOL "" FORCE)
    set(NV_APPEND_CONFIG_NAME OFF CACHE BOOL "" FORCE)
    set(PUBLIC_RELEASE OFF CACHE BOOL "" FORCE)

    # Platform-specific preset options
    if(WIN32)
        set(NV_USE_STATIC_WINCRT OFF CACHE BOOL "" FORCE)
        set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "" FORCE)
        set(NV_FORCE_64BIT_SUFFIX OFF CACHE BOOL "" FORCE)
        set(NV_FORCE_32BIT_SUFFIX OFF CACHE BOOL "" FORCE)
        # Keep all embedded PhysX build products under ovruntime/_build.
        # PhysX's low-level PDB export path writes to ${PHYSX_ROOT_DIR}/release,
        # which pollutes the sibling physx source tree in --devphysx mode.
        set(PX_EXPORT_LOWLEVEL_PDB OFF CACHE BOOL "" FORCE)
        set(PX_COPY_EXTERNAL_DLL OFF CACHE BOOL "" FORCE)
        set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "" FORCE)
        set(PX_USE_NVTX OFF CACHE BOOL "" FORCE)
    else()
        set(NV_USE_STATIC_WINCRT ON CACHE BOOL "" FORCE)
        set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "" FORCE)
        set(PX_EXPORT_LOWLEVEL_PDB OFF CACHE BOOL "" FORCE)
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
            set(NV_FORCE_64BIT_SUFFIX ON CACHE BOOL "" FORCE)
        else()
            set(NV_FORCE_64BIT_SUFFIX OFF CACHE BOOL "" FORCE)
        endif()
        set(NV_FORCE_32BIT_SUFFIX OFF CACHE BOOL "" FORCE)
    endif()

    # Map ovruntime Release → PhysX "checked" config.
    # PhysX "release" strips PVD support and assertions; "checked" is the correct
    # config for development (NDEBUG + PX_CHECKED=1 + PVD), matching the packman
    # pre-built libraries that ovruntime links against in the non-devphysx case.
    # CMAKE_BUILD_TYPE is temporarily set to "checked" for the PhysX add_subdirectory
    # call only, then restored so that the parent project (ovruntime) continues
    # to see the original type.  This avoids polluting parent logic such as output
    # directory computation with PhysX's internal build-type choice.
    set(_OVRUNTIME_SAVED_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        message(STATUS "OVRUNTIME_DEV_PHYSX: Remapping CMAKE_BUILD_TYPE Release → checked for PhysX subproject")
        set(CMAKE_BUILD_TYPE "checked" CACHE STRING "" FORCE)

        # Define "checked" compiler/linker flags mirroring Release (cmake only
        # knows Debug/Release/RelWithDebInfo/MinSizeRel out of the box).
        set(CMAKE_C_FLAGS_CHECKED          "${CMAKE_C_FLAGS_RELEASE}"          CACHE STRING "" FORCE)
        set(CMAKE_CXX_FLAGS_CHECKED        "${CMAKE_CXX_FLAGS_RELEASE}"        CACHE STRING "" FORCE)
        set(CMAKE_CUDA_FLAGS_CHECKED       "${CMAKE_CUDA_FLAGS_RELEASE}"       CACHE STRING "" FORCE)
        set(CMAKE_EXE_LINKER_FLAGS_CHECKED "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
        set(CMAKE_SHARED_LINKER_FLAGS_CHECKED "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
        set(CMAKE_STATIC_LINKER_FLAGS_CHECKED "${CMAKE_STATIC_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
        set(CMAKE_MODULE_LINKER_FLAGS_CHECKED "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)

        # Redirect output directories so binaries still land in "release/" (not
        # "checked/").  NvidiaBuildOptions already set *_CHECKED paths to checked/;
        # override them with the release paths so tests and scripts find the
        # binaries in the expected location.  Must use regular variables (not CACHE)
        # because NvidiaBuildOptions sets them as regular variables which take
        # precedence over cache entries in cmake.
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_CHECKED "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_CHECKED "${CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE}")
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_CHECKED "${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}")
        set(PX_EXE_OUTPUT_DIRECTORY_CHECKED        "${PX_EXE_OUTPUT_DIRECTORY_RELEASE}" CACHE INTERNAL "" FORCE)
    endif()

    if(_OVRUNTIME_PHYSX_SOURCE_LAYOUT STREQUAL "internal")
        # Include PhysX via its internal CMake entry point when available. This
        # preserves the in-repo development path used by internal branches.
        add_subdirectory(
            "${PHYSX_ROOT_DIR}/compiler/internal"
            "${CMAKE_CURRENT_BINARY_DIR}/physx_sdk"
            EXCLUDE_FROM_ALL
        )
    else()
        # The public PhysX source distro intentionally omits compiler/internal.
        # Include the public source projects directly so --devphysx validates the
        # same source layout that will be published.
        #
        # PUBLIC_RELEASE=ON matches the public layout's own default (set in
        # physx/CMakeLists.txt). The freeglut/DLL-copy logic gated by it in
        # source/compiler/cmake/windows/CMakeLists.txt only affects snippet,
        # unit-test, and visualtest targets, all of which we disable above
        # (PX_BUILDSNIPPETS / PX_BUILDUNITTESTS / PX_BUILDVISUALTESTS = OFF).
        # The earlier reason for using compiler/internal (avoid PUBLIC_RELEASE=1
        # pulling freeglut) does not apply once those targets are off.
        set(PUBLIC_RELEASE ON CACHE BOOL "" FORCE)
        add_subdirectory(
            "${PHYSX_ROOT_DIR}/source/compiler/cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/physx_sdk"
            EXCLUDE_FROM_ALL
        )
        if(PX_GENERATE_GPU_PROJECTS OR PX_GENERATE_GPU_PROJECTS_ONLY)
            add_subdirectory(
                "${PHYSX_ROOT_DIR}/source/compiler/cmakegpu"
                "${CMAKE_CURRENT_BINARY_DIR}/physx_gpu"
                EXCLUDE_FROM_ALL
            )
        endif()
        if(PX_BUILDPVDRUNTIME AND EXISTS "${PHYSX_ROOT_DIR}/pvdruntime/compiler/cmake/CMakeLists.txt")
            add_subdirectory(
                "${PHYSX_ROOT_DIR}/pvdruntime/compiler/cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/physx_pvdruntime"
                EXCLUDE_FROM_ALL
            )
        endif()
    endif()

    if(TARGET PhysXCharacterKinematic AND TARGET PhysXExtensions)
        # Source-built PhysX CCT uses PxMeshOverlapUtil from Extensions but the
        # upstream target only exposes Foundation. Static ovphysx links need the
        # real dependency so libovphysx closes over Extensions.
        target_link_libraries(PhysXCharacterKinematic PUBLIC PhysXExtensions)
    endif()

    # Restore CMAKE_BUILD_TYPE so the parent project is not affected by the
    # PhysX-internal Release → checked remapping above.
    set(CMAKE_BUILD_TYPE "${_OVRUNTIME_SAVED_BUILD_TYPE}" CACHE STRING "" FORCE)
    unset(_OVRUNTIME_SAVED_BUILD_TYPE)

    # Fix output names: PhysX cmake uses OUTPUT_NAME "PhysXGpu" / "PVDRuntime"
    # (no _64 suffix), which sets the SONAME to libPhysXGpu.so / libPVDRuntime.so.
    # PxLoadPhysxGPUModule (compiled into the PhysX static lib) does
    # dlopen("libPhysXGpu_64.so") at runtime.  On Linux, dlopen matches by SONAME
    # when checking already-loaded libraries, so the SONAME must include _64 for the
    # lookup to succeed — especially from Python where the calling library has no
    # RPATH pointing to the bin directory.
    if(TARGET PhysXGpu)
        set_target_properties(PhysXGpu PROPERTIES OUTPUT_NAME "PhysXGpu_64")
    endif()
    if(TARGET PVDRuntime)
        set_target_properties(PVDRuntime PROPERTIES OUTPUT_NAME "PVDRuntime_64")
    endif()

    # --- Release→checked compile-def patch (VS and single-config Linux) ---
    # Two cases require this patch:
    #   1. Multi-config generators (VS): CMAKE_BUILD_TYPE is empty, so VS "Release"
    #      matches PhysX's $<CONFIG:release> generator expressions which set
    #      PX_SUPPORT_PVD=0 and PX_SUPPORT_OMNI_PVD=0, completely disabling PVD.
    #   2. Single-config Linux Release: the temporary CMAKE_BUILD_TYPE=checked remap
    #      above does NOT prevent $<$<CONFIG:release>:...> from firing because
    #      generator expressions are evaluated at generation time, after
    #      CMAKE_BUILD_TYPE has been restored to "Release".
    # Fix by patching PhysX target compile definitions: replace the release-config
    # defs with the checked-config defs so that Release builds get PVD support,
    # matching the behaviour of the pre-built (packman) PhysX checked libraries.
    get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(_is_multi_config OR CMAKE_BUILD_TYPE STREQUAL "Release")
        message(STATUS "OVRUNTIME_DEV_PHYSX: Remapping PhysX Release compile defs → checked (multi_config=${_is_multi_config}, build_type=${CMAKE_BUILD_TYPE})")

        # PhysX's platform cmake (e.g. windows/CMakeLists.txt) wrote these cache
        # variables during add_subdirectory; they contain the already-expanded
        # preprocessor definition lists for each configuration.
        if(WIN32)
            set(_PX_RELEASE_DEFS "${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS}")
            set(_PX_CHECKED_DEFS "${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS}")
        else()
            set(_PX_RELEASE_DEFS "${PHYSX_LINUX_RELEASE_COMPILE_DEFS}")
            set(_PX_CHECKED_DEFS "${PHYSX_LINUX_CHECKED_COMPILE_DEFS}")
        endif()

        # Build the genex substrings to search for / replace with.
        # PhysX target cmake files are inconsistent: most use a trailing semicolon
        # before the closing bracket (e.g. ...DEFS;>) but PhysXCommon omits it
        # (e.g. ...DEFS>).  Handle both variants.
        set(_OLD_GENEX_SEMI "$<$<CONFIG:release>:${_PX_RELEASE_DEFS};>")
        set(_NEW_GENEX_SEMI "$<$<CONFIG:release>:${_PX_CHECKED_DEFS};>")
        set(_OLD_GENEX_BARE "$<$<CONFIG:release>:${_PX_RELEASE_DEFS}>")
        set(_NEW_GENEX_BARE "$<$<CONFIG:release>:${_PX_CHECKED_DEFS}>")

        # All PhysX targets (CPU and GPU) that carry the per-config defs.
        # GPU targets must match CPU targets so that shared headers like
        # PxsSimulationController.h produce the same vtable layout
        # (PX_SUPPORT_OMNI_PVD guards virtual methods).
        set(_PHYSX_TARGETS
            PhysX PhysXCommon PhysXFoundation PhysXCooking PhysXExtensions
            PhysXPvdSDK PhysXCharacterKinematic PhysXVehicle PhysXTask
            LowLevel LowLevelAABB LowLevelDynamics SceneQuery SimulationController
            PhysXGpu PhysXGpuDependencies
            PhysXArticulationGpu PhysXBroadphaseGpu PhysXCommonGpu
            PhysXCudaContextManager PhysXNarrowphaseGpu
            PhysXSimulationControllerGpu PhysXSolverGpu
        )
        foreach(_t IN LISTS _PHYSX_TARGETS)
            if(TARGET ${_t})
                get_property(_defs TARGET ${_t} PROPERTY COMPILE_DEFINITIONS)
                string(REPLACE "${_OLD_GENEX_SEMI}" "${_NEW_GENEX_SEMI}" _defs "${_defs}")
                string(REPLACE "${_OLD_GENEX_BARE}" "${_NEW_GENEX_BARE}" _defs "${_defs}")
                set_property(TARGET ${_t} PROPERTY COMPILE_DEFINITIONS "${_defs}")
            endif()
        endforeach()
    endif()

    # Set PHYSX_SDK_DIR for include path resolution (points to source tree)
    set(PHYSX_SDK_DIR "${PHYSX_ROOT_DIR}" CACHE PATH "" FORCE)
endif()

if(OVRUNTIME_DEV_PHYSX)
    # Mark that we have source-built PhysX targets available
    set(OVRUNTIME_PHYSX_FROM_SOURCE TRUE)
    # Ensure PHYSX_SDK_DIR points to source tree (may be re-included from subdirectory)
    if(NOT DEFINED PHYSX_ROOT_DIR)
        set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../physx" CACHE PATH "PhysX SDK root directory")
        get_filename_component(PHYSX_ROOT_DIR "${PHYSX_ROOT_DIR}" ABSOLUTE)
    endif()
    set(PHYSX_SDK_DIR "${PHYSX_ROOT_DIR}" CACHE PATH "" FORCE)
else()
    set(OVRUNTIME_PHYSX_FROM_SOURCE FALSE)
endif()


# Helper function: link PhysX static libraries to a target
# Usage:
#   ovruntime_link_physx_static_libs(MyTarget)          # links all PhysX libs
#   ovruntime_link_physx_static_libs(MyTarget FULL)     # same as above
#   ovruntime_link_physx_static_libs(MyTarget MINIMAL)  # links only PhysX, PhysXCommon, PhysXFoundation
function(ovruntime_link_physx_static_libs _target)
    set(_mode "FULL")
    if(ARGC GREATER 1)
        set(_mode "${ARGV1}")
    endif()

    if(OVRUNTIME_PHYSX_FROM_SOURCE)
        # Link directly to cmake targets built from source
        if(_mode STREQUAL "MINIMAL")
            target_link_libraries(${_target} PRIVATE
                PhysX
                PhysXCommon
                PhysXFoundation
            )
        else()
            target_link_libraries(${_target} PRIVATE
                PhysXVehicle
                PhysXCharacterKinematic
                PhysXExtensions
                PhysX
                PhysXPvdSDK
                PhysXCooking
                PhysXCommon
                PhysXFoundation
            )
        endif()
        if(NOT WIN32)
            target_link_libraries(${_target} PRIVATE dl pthread)
        endif()
    else()
        # Link to pre-built static libraries from packman
        if(NOT DEFINED PHYSX_SDK_DIR)
            return()
        endif()

        set(_PHYSX_LIB_DIR "${PHYSX_SDK_DIR}/bin/${PHYSX_PLATFORM_BIN}")
        target_link_directories(${_target} PRIVATE
            ${_PHYSX_LIB_DIR}/$<IF:$<CONFIG:Debug>,debug,checked>
        )

        if(_mode STREQUAL "MINIMAL")
            if(NOT WIN32)
                target_link_libraries(${_target} PRIVATE
                    -Wl,--start-group
                    PhysX_static_64
                    PhysXCommon_static_64
                    PhysXFoundation_static_64
                    -Wl,--end-group
                )
            else()
                target_link_libraries(${_target} PRIVATE
                    PhysX_static_64
                    PhysXCommon_static_64
                    PhysXFoundation_static_64
                )
            endif()
        else()
            if(NOT WIN32)
                target_link_libraries(${_target} PRIVATE
                    -Wl,--start-group
                    PhysXExtensions_static_64
                    PhysX_static_64
                    PhysXPvdSDK_static_64
                    PhysXVehicle_static_64
                    PhysXCharacterKinematic_static_64
                    PhysXCooking_static_64
                    PhysXCommon_static_64
                    PhysXFoundation_static_64
                    -Wl,--end-group
                    dl pthread
                )
            else()
                target_link_libraries(${_target} PRIVATE
                    PhysXVehicle_static_64
                    PhysXExtensions_static_64
                    PhysXCharacterKinematic_static_64
                    PhysX_static_64
                    PhysXPvdSDK_static_64
                    PhysXCooking_static_64
                    PhysXCommon_static_64
                    PhysXFoundation_static_64
                )
            endif()
        endif()
    endif()
endfunction()
