# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# omni.physics.usd — test-only USD parse/scan backends and seams (ADR-0027, static)
#
# Implements IPhysicsSource (read) for UsdStages, the USD-backed IPhysicsDataWrite
# sink, and the ADR-0027 seams (reparse, backing/attachment authoring, stage
# lifecycle, opaque pxr-handle op tables). Installed into the ADR-0005 registries
# through the C entry omniPhysicsUsdInstallBackends(). Configured only under
# OVRUNTIME_ENABLE_TESTS, linked only by test executables, never installed: it is
# the sole pxr-linking library target in the tree and has no dependency on
# OvruntimePhysX.

set(PARSE_USD_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdXformHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/XformOpResetStorage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdPhysicsDataWrite.h
    ${CMAKE_CURRENT_SOURCE_DIR}/DescriptorConverters.h
    ${CMAKE_CURRENT_SOURCE_DIR}/NativeWalker.h
    # Relocated from source/common/: pxr-typed headers whose consumers are all
    # USD-only. common/ is pxr-free.
    ${CMAKE_CURRENT_SOURCE_DIR}/ImguiDrawingUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/TypeCast.h
    ${CMAKE_CURRENT_SOURCE_DIR}/PrimUtilities.h
    # Relocated from common/utilities/Utilities.h (ADR-0027, REQ-PUBLICAPI-HANDLE-001):
    # the SdfPath<->uint64 bit encoders and the session-layer edit guard.
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdPathEncoding.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ScopedLayerEdit.h
    ${CMAKE_CURRENT_SOURCE_DIR}/OpaquePxrHandleOpsUsd.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/CustomTokens.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/StandaloneParser.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/StageScan.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdDeformableAttachmentWrite.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdParseBackend.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdReparse.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdBackingAuthor.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdAttachmentAuthor.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdStageLifecycle.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdScanBackend.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdBackendInstall.h
)
source_group(include FILES ${PARSE_USD_HEADERS})

set(PARSE_USD_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdChangeFeed.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdXformHelpers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/XformOpResetStorage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdPhysicsDataWrite.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CustomTokens.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DescriptorConverters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StandaloneParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StageScan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NativeWalker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdDeformableAttachmentWrite.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdParseBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdReparse_usd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdBackingAuthor_usd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdAttachmentAuthor_usd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdStageLifecycle_usd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdBackendInstall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ImguiDrawingUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PrimUtilities.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OpaquePxrHandleOps_usd.cpp
)
source_group(source FILES ${PARSE_USD_SOURCE})

add_library(OvruntimePhysicsUsd STATIC
    ${PARSE_USD_HEADERS}
    ${PARSE_USD_SOURCE}
)

set_property(TARGET OvruntimePhysicsUsd PROPERTY FOLDER "Tests/Parse")

set_target_properties(OvruntimePhysicsUsd PROPERTIES
    OUTPUT_NAME "OvruntimePhysicsUsd"
)

target_include_directories(OvruntimePhysicsUsd
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}     # <UsdSource.h> for the test executables
        ${OVRUNTIME_ROOT_DIR}/include   # <omni/physics/usd/StandaloneParser.h> for consumers
    PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include/private  # <private/omni/physics/schema/IUsdPhysics.h>
        # <common/foundation/MatrixTools.h> — the PxMat44d toolkit used by the
        # shared private headers and their call sites in NativeWalker.cpp.
        ${OVRUNTIME_ROOT_DIR}/source/common/include
)

# Static library: UsdBackendInstall.h must not expand to dllimport on MSVC.
target_compile_definitions(OvruntimePhysicsUsd PUBLIC OMNI_PHYSICS_USD_STATIC)

# Link against the core parse library (registry slots, descriptors, ScannedStage)
target_link_libraries(OvruntimePhysicsUsd
    PUBLIC OvruntimePhysicsParse
    # Only for MatrixTools.o (decomposeMatrix / makeMatrix / affineInverse).
    PRIVATE OvruntimeCommon
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysicsUsd)

# The only pxr-linking library target: USD, TBB, Python. Propagates to the linking
# test executables through the static archive's link interface.
ovruntime_link_usd_deps(OvruntimePhysicsUsd)

if(NOT WIN32)
    target_compile_options(OvruntimePhysicsUsd PRIVATE -fexceptions -fPIC -Wno-deprecated)
endif()

# No install() rules: test scaffolding, not a product artifact (ADR-0027).
