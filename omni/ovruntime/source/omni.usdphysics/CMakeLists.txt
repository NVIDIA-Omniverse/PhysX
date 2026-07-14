# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Plugin headers (private) ---
set(USDPHYSICS_PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Articulation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Attachment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Collision.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CollisionGroup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/DeformableBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FilteredPairs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Joint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/RigidBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Scene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/UsdLoad.h
)
source_group(plugins FILES ${USDPHYSICS_PLUGIN_HEADERS})

# --- Plugin sources ---
set(USDPHYSICS_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Articulation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Attachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Collision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CollisionGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/DeformableBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FilteredPairs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/UsdLoad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/UsdPhysics.cpp
)
source_group(source FILES ${USDPHYSICS_PLUGIN_SOURCE})

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimeUsdPhysics SHARED
    ${USDPHYSICS_PLUGIN_HEADERS}
    ${USDPHYSICS_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimeUsdPhysics PROPERTY FOLDER "Runtime/UsdPhysics")

set_target_properties(OvruntimeUsdPhysics PROPERTIES
    OUTPUT_NAME "omni.usdphysics.plugin"
)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimeUsdPhysics PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimeUsdPhysics
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # <omni/physics/schema/...>
        ${OVRUNTIME_ROOT_DIR}/include/private         # <private/omni/physics/schema/...>
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeUsdPhysics)

# Compile definitions
target_compile_definitions(OvruntimeUsdPhysics PRIVATE
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimeUsdPhysics
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimeUsdPhysics PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimeUsdPhysics PRIVATE carb)
endif()

# physicsSchemaTools library
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimeUsdPhysics PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimeUsdPhysics PRIVATE physicsSchemaTools)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimeUsdPhysics)

# Precompiled header
target_precompile_headers(OvruntimeUsdPhysics PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# --- Python bindings (optional) ---
if(OVRUNTIME_ENABLE_PYTHON_BINDINGS)
    set(USDPHYSICS_BINDINGS_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsUsdPhysicsPython.cpp
    )
    source_group(bindings FILES ${USDPHYSICS_BINDINGS_SOURCE})

    add_library(OvruntimeUsdPhysicsBindings SHARED
        ${USDPHYSICS_BINDINGS_SOURCE}
    )

    set_property(TARGET OvruntimeUsdPhysicsBindings PROPERTY FOLDER "Runtime/UsdPhysics")

    set_target_properties(OvruntimeUsdPhysicsBindings PROPERTIES
        OUTPUT_NAME "_usdphysics"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(OvruntimeUsdPhysicsBindings PROPERTIES SUFFIX ".pyd")
    else()
        set_target_properties(OvruntimeUsdPhysicsBindings PROPERTIES SUFFIX ".so")
    endif()

    target_include_directories(OvruntimeUsdPhysicsBindings PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include
        ${OVRUNTIME_ROOT_DIR}/include/private
        ${OVRUNTIME_ROOT_DIR}/source/pch
        ${OVRUNTIME_ROOT_DIR}/source/common/include
    )

    # External dependencies — SYSTEM includes to suppress third-party warnings
    ovruntime_add_external_system_includes(OvruntimeUsdPhysicsBindings)

    # physicsSchemaTools (bindings link this too)
    if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        target_link_directories(OvruntimeUsdPhysicsBindings PRIVATE
            ${USD_EXT_PHYSICS_DIR}/lib
        )
        target_link_libraries(OvruntimeUsdPhysicsBindings PRIVATE physicsSchemaTools)
    endif()

    target_compile_definitions(OvruntimeUsdPhysicsBindings PRIVATE
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    if(NOT WIN32)
        target_compile_options(OvruntimeUsdPhysicsBindings PRIVATE -fexceptions -frtti -fPIC)
    endif()

    if(DEFINED CARB_SDK_DIR)
        target_link_directories(OvruntimeUsdPhysicsBindings PRIVATE
            ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
        )
        target_link_libraries(OvruntimeUsdPhysicsBindings PRIVATE carb)
    endif()

    # Python + USD boost::python libraries
    ovruntime_link_bindings_deps(OvruntimeUsdPhysicsBindings)

    # Precompiled header
    target_precompile_headers(OvruntimeUsdPhysicsBindings PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
    )

    # Install bindings
    install(
        TARGETS OvruntimeUsdPhysicsBindings
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
    )
endif()

# Install rules
install(
    TARGETS OvruntimeUsdPhysics
    EXPORT OvruntimeUsdPhysics
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)
