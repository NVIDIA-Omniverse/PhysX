# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# omni.physics.ovstage — ovstage backend for the source-agnostic physics data
# layer (static). Implements IPhysicsSource (read) + the walker over an ovstage
# instance. Always built on this branch; USD vs ovstage is a runtime choice via
# the registered parse/scan backends (not a build flag). It is NOT installed as
# the default backend — a consumer/test selects it via
# setParseBackend(makeOvstageParseBackend()) + setScanBackend(makeOvstageScanBackend()).

set(PARSE_OVSTAGE_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageChangeFeed.h
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageWalker.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/ovstage/OvstageScan.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/ovstage/OvstageParseBackend.h
    ${OVRUNTIME_ROOT_DIR}/include/omni/physics/ovstage/OvstageOutput.h
)
source_group(include FILES ${PARSE_OVSTAGE_HEADERS})

set(PARSE_OVSTAGE_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageChangeFeed.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageWalker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageScan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageParseBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OvstageOutput.cpp
)
source_group(source FILES ${PARSE_OVSTAGE_SOURCE})

add_library(OvruntimePhysicsOvstage STATIC
    ${PARSE_OVSTAGE_HEADERS}
    ${PARSE_OVSTAGE_SOURCE}
)

set_property(TARGET OvruntimePhysicsOvstage PROPERTY FOLDER "Runtime/Parse")

set_target_properties(OvruntimePhysicsOvstage PROPERTIES
    OUTPUT_NAME "OvruntimePhysicsOvstage"
)

target_include_directories(OvruntimePhysicsOvstage
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}     # <OvstageSource.h> for sibling code
        ${OVRUNTIME_ROOT_DIR}/include   # <omni/physics/ovstage/OvstageParseBackend.h>
)

# ovstage public headers (ovstage/*, ovx/*) as SYSTEM — they come from the
# package resolved by cmake/OvstageDependency.cmake. PUBLIC so the test target
# that includes OvstageSource.h inherits them.
target_include_directories(OvruntimePhysicsOvstage SYSTEM PUBLIC ${OVSTAGE_INCLUDE_DIRS})

# Link against the USD-free parse core + the ovstage runtime only. ovstage is a
# peer of omni.physics.usd, NOT a dependent of it: scanStageOvstage produces a
# `parse::ScannedStage` via the parse core's `makeScannedStageFromSource`
# builder and registers a `parse::IScanBackend`, so there is no link to the USD
# backend library. (A USD consumer wraps the result in
# `omni::physics::usd::ScannedStage` on its own side for SdfPath resolution.)
target_link_libraries(OvruntimePhysicsOvstage
    PUBLIC OvruntimePhysicsParse ${OVSTAGE_LIBRARY}
)

# External dependencies — SYSTEM includes to suppress third-party warnings.
ovruntime_add_external_system_includes(OvruntimePhysicsOvstage)

if(NOT WIN32)
    target_compile_options(OvruntimePhysicsOvstage PRIVATE -fexceptions -fPIC -Wno-deprecated)
endif()

install(
    TARGETS OvruntimePhysicsOvstage
    EXPORT OvruntimePhysicsOvstage
    ARCHIVE DESTINATION lib
)
