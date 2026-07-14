# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Plugin sources ---
set(PHYSICS_TENSORS_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysicsTensors.cpp
)
source_group(plugins FILES ${PHYSICS_TENSORS_PLUGIN_SOURCE})

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimePhysicsTensors SHARED
    ${PHYSICS_TENSORS_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysicsTensors PROPERTY FOLDER "Runtime/PhysicsTensors")

set_target_properties(OvruntimePhysicsTensors PROPERTIES
    OUTPUT_NAME "omni.physics.tensors.plugin"
)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimePhysicsTensors PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimePhysicsTensors
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private         # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysicsTensors)

# Compile definitions
target_compile_definitions(OvruntimePhysicsTensors PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimePhysicsTensors
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysicsTensors PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysicsTensors PRIVATE carb)
endif()

# physxSchema library
if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimePhysicsTensors PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimePhysicsTensors PRIVATE physxSchema)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimePhysicsTensors)

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimePhysicsTensors FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimePhysicsTensors PRIVATE rt)
endif()

# Precompiled header
target_precompile_headers(OvruntimePhysicsTensors PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# --- Python bindings (optional) ---
if(OVRUNTIME_ENABLE_PYTHON_BINDINGS)
    set(PHYSICS_TENSORS_BINDINGS_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/python/PhysicsTensorsBindingsPython.cpp
    )
    source_group(bindings FILES ${PHYSICS_TENSORS_BINDINGS_SOURCE})

    add_library(OvruntimePhysicsTensorsBindings SHARED
        ${PHYSICS_TENSORS_BINDINGS_SOURCE}
    )

    set_property(TARGET OvruntimePhysicsTensorsBindings PROPERTY FOLDER "Runtime/PhysicsTensors")

    set_target_properties(OvruntimePhysicsTensorsBindings PROPERTIES
        OUTPUT_NAME "_physicsTensors"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(OvruntimePhysicsTensorsBindings PROPERTIES SUFFIX ".pyd")
    else()
        set_target_properties(OvruntimePhysicsTensorsBindings PROPERTIES SUFFIX ".so")
    endif()

    target_include_directories(OvruntimePhysicsTensorsBindings PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include
        ${OVRUNTIME_ROOT_DIR}/include/private
        ${OVRUNTIME_ROOT_DIR}/source/pch
        ${OVRUNTIME_ROOT_DIR}/source/common/include
    )

    # External dependencies — SYSTEM includes to suppress third-party warnings
    ovruntime_add_external_system_includes(OvruntimePhysicsTensorsBindings)

    target_compile_definitions(OvruntimePhysicsTensorsBindings PRIVATE
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    if(NOT WIN32)
        target_compile_options(OvruntimePhysicsTensorsBindings PRIVATE -fexceptions -frtti -fPIC)
    endif()

    if(DEFINED CARB_SDK_DIR)
        target_link_directories(OvruntimePhysicsTensorsBindings PRIVATE
            ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
        )
        target_link_libraries(OvruntimePhysicsTensorsBindings PRIVATE carb)
    endif()

    # Python + USD boost::python libraries
    ovruntime_link_bindings_deps(OvruntimePhysicsTensorsBindings)

    # Precompiled header
    target_precompile_headers(OvruntimePhysicsTensorsBindings PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
    )

    # Install bindings
    install(
        TARGETS OvruntimePhysicsTensorsBindings
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
    )
endif()

# Install rules
install(
    TARGETS OvruntimePhysicsTensors
    EXPORT OvruntimePhysicsTensors
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)
