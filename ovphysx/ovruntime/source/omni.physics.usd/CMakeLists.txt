# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# omni.physics.usd — USD backend for the source-agnostic physics data layer (static)
#
# This library depends on USD and implements IPhysicsSource (read) for USD
# stages; it also hosts the USD-backed IPhysicsDataWrite (write) sink.

set(PARSE_USD_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdXformHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdPhysicsDataWrite.h
    ${CMAKE_CURRENT_SOURCE_DIR}/DescriptorConverters.h
    ${CMAKE_CURRENT_SOURCE_DIR}/NativeWalker.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/CustomTokens.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/StandaloneParser.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/StageScan.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdDeformableAttachmentWrite.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/usd/UsdParseBackend.h
)
source_group(include FILES ${PARSE_USD_HEADERS})

set(PARSE_USD_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdChangeFeed.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdXformHelpers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdPhysicsDataWrite.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CustomTokens.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DescriptorConverters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StandaloneParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StageScan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NativeWalker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdDeformableAttachmentWrite.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsdParseBackend.cpp
)
source_group(source FILES ${PARSE_USD_SOURCE})

add_library(OvruntimePhysicsUsd STATIC
    ${PARSE_USD_HEADERS}
    ${PARSE_USD_SOURCE}
)

set_property(TARGET OvruntimePhysicsUsd PROPERTY FOLDER "Runtime/Parse")

set_target_properties(OvruntimePhysicsUsd PROPERTIES
    OUTPUT_NAME "OvruntimePhysicsUsd"
)

target_include_directories(OvruntimePhysicsUsd
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}     # <UsdSource.h> for sibling code
        ${OVRUNTIME_ROOT_DIR}/include   # <omni/physics/usd/StandaloneParser.h> for consumers
    PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include/private  # <private/omni/physics/schema/IUsdPhysics.h>
)

# Link against the core parse library
target_link_libraries(OvruntimePhysicsUsd
    PUBLIC OvruntimePhysicsParse
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysicsUsd)

if(NOT WIN32)
    target_compile_options(OvruntimePhysicsUsd PRIVATE -fexceptions -fPIC -Wno-deprecated)
endif()

install(
    TARGETS OvruntimePhysicsUsd
    EXPORT OvruntimePhysicsUsd
    ARCHIVE DESTINATION lib
)
