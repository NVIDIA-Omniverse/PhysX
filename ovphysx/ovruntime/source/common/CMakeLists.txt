# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Collect header files (public API)
set(COMMON_HEADERS
    # foundation
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Algorithms.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Allocator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/CarbPhysXCast.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/DeformableCookingTransform.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/MatrixTools.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Preprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/TransformedExtent.h
    # foundation: TypeCast.h (the pxr-typed PhysX<->Gf bridge) moved to
    # source/omni.physics.usd/ -- every consumer is USD-only. It is no longer
    # installed: a pxr-including header in the shipped SDK include/ tree is not
    # self-contained without USD.
    # ui: ImguiDrawingUtils.{h,cpp} moved to source/omni.physics.usd/ (it is
    # pxr-typed; common/ is the USD-free library) and is no longer installed.
    # utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CoreUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CudaHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Defer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/LockingList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryMacros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/OmniPhysXUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PhysXErrorCallback.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimHierarchyStorage.h
    # utilities: SdfPathEncoding.h and UsdToolsTestHelpers.h were never built or
    # installed from here -- both are test-only and pxr-typed, so they now live
    # next to their callers in source/omni.physx/tests/test.unit/physics/.
    # utilities: PrimUtilities.{h,cpp} moved to source/omni.physics.usd/ (it is
    # pxr-typed; common/ is the USD-free library), same relocation as
    # ImguiDrawingUtils above. UsdMaterialParsing.{h,cpp} deleted outright --
    # dead code, superseded by OvstageSource::getMaterialBinding(ObjectKey).
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Utilities.h
)
source_group(include FILES ${COMMON_HEADERS})

# Collect source files
set(COMMON_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/Allocator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/DeformableCookingTransform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/MatrixTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/TransformedExtent.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/OptionalCudaShim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/PhysXErrorCallback.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/PrimHierarchyStorage.cpp
)
source_group(source FILES ${COMMON_SOURCE})

# Create the static library (matching old premake "foundation" target)
add_library(OvruntimeCommon STATIC
    ${COMMON_HEADERS}
    ${COMMON_SOURCE}
)

set_property(TARGET OvruntimeCommon PROPERTY FOLDER "Runtime/Common")

# RTTI enabled (matching old premake config)
set_target_properties(OvruntimeCommon PROPERTIES
    OUTPUT_NAME "OvruntimeCommon"
)

# Include directories
target_include_directories(OvruntimeCommon
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include             # <common/foundation/...>, <common/utilities/...>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities
        ${OVRUNTIME_ROOT_DIR}/include                   # <omni/physx/...>, <omni/physics/...>
        ${OVRUNTIME_ROOT_DIR}/include/private           # <private/omni/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings.
# common/ is the pxr-free base library, compiled without the USD/physxSchema
# header dirs, so a stray pxr include is a build error.
ovruntime_add_external_system_includes(OvruntimeCommon NO_USD)

# Platform-specific settings
if(WIN32)
    # Disable vectorized STL algorithms for maximum MSVC toolset compatibility.
    target_compile_definitions(OvruntimeCommon PRIVATE _USE_STD_VECTOR_ALGORITHMS=0)
else()
    # Match old premake: exceptions on, fPIC, suppress some warnings
    target_compile_options(OvruntimeCommon PRIVATE
        -fexceptions -fPIC
        # aarch64's baseline FMA support makes GCC's default -ffp-contract=fast
        # fuse multiply-add into one rounding step, unlike x86_64 (no FMA
        # without explicit -mfma, so the default is a no-op). That is a real
        # source of ULP-level divergence for the gfmath bit-exact
        # transcriptions in MatrixTools.cpp (factor/_Jacobi3/composeWithPivot/
        # rotation-multiply/deformable-cooking-transform). Mirrors the
        # identical fix already applied to the PhysX SDK build (see
        # physx/source/compiler/cmake/linux/CMakeLists.txt).
        # This only keeps OvruntimeCommon's OWN object code free of fused
        # multiply-add; it has no effect on the FP model of the prebuilt,
        # vendored Gf binary TestMatrixTools.cpp links as its parity oracle,
        # which this build does not compile. See ADR-0018, open question 5.
        # @implements ADR-0018 (usd-free-runtime-build), open question 1
        $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>:-ffp-contract=off>
    )
endif()

# Install rules - public headers (preserving directory structure)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Algorithms.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Allocator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/CarbPhysXCast.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/DeformableCookingTransform.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/MatrixTools.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Preprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/TransformedExtent.h
    DESTINATION include/common/foundation
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CoreUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CudaHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Defer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/LockingList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryMacros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/OmniPhysXUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PhysXErrorCallback.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimHierarchyStorage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Utilities.h
    DESTINATION include/common/utilities
)

install(
    TARGETS OvruntimeCommon
    EXPORT OvruntimeCommon
    ARCHIVE DESTINATION lib
)

# CI grep gate: the ported Gf math must stay pxr-free.
add_custom_target(common_gf_free_check ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Checking the ported Gf math for pxr spellings..."
    COMMAND ${CMAKE_COMMAND}
        "-DGF_FREE_FILES=${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/MatrixTools.h;${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/MatrixTools.cpp;${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/DeformableCookingTransform.h;${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/DeformableCookingTransform.cpp;${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/TransformedExtent.h;${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/TransformedExtent.cpp"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gf_free_check.cmake
    VERBATIM
)

# CI gate: the OpenUSD-derived `gfmath` transcription in MatrixTools.{h,cpp}
# must keep its SPDX-Snippet attribution markers (see
# docs/internal/third_party_openusd_gfmath.md and REQ-MATH-001 AC-8).
add_custom_target(common_spdx_snippet_check ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Checking the gfmath transcription for SPDX-Snippet attribution markers..."
    COMMAND ${CMAKE_COMMAND}
        "-DSPDX_SNIPPET_FILES=${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/MatrixTools.h;${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/MatrixTools.cpp"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/spdx_snippet_check.cmake
    VERBATIM
)
