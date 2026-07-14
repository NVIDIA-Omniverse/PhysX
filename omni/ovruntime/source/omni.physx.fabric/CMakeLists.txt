# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Plugin headers (private) ---
set(FABRIC_PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/DirectGpuHelper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricKernels.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricMapper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SurfaceDeformableBodyManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VolumeDeformableBodyManager.h
)
source_group(plugins\\headers FILES ${FABRIC_PLUGIN_HEADERS})

# --- Plugin sources (C++) ---
set(FABRIC_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/DirectGpuHelper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricMapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXFabric.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SurfaceDeformableBodyManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VolumeDeformableBodyManager.cpp
)
source_group(plugins\\source FILES ${FABRIC_PLUGIN_SOURCE})

# --- CUDA sources ---
set(FABRIC_CUDA_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FabricKernels.cu
)
source_group(plugins\\cuda FILES ${FABRIC_CUDA_SOURCE})

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimePhysXFabric SHARED
    ${FABRIC_PLUGIN_HEADERS}
    ${FABRIC_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysXFabric PROPERTY FOLDER "Runtime/PhysXFabric")

set_target_properties(OvruntimePhysXFabric PROPERTIES
    OUTPUT_NAME "omni.physx.fabric.plugin"
)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimePhysXFabric PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fexceptions -frtti -fPIC>
    )
endif()

# Include directories
target_include_directories(OvruntimePhysXFabric
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private         # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXFabric)

# Compile definitions
target_compile_definitions(OvruntimePhysXFabric PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "$<$<COMPILE_LANGUAGE:CXX>:carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)>"
)

# Link dependencies
target_link_libraries(OvruntimePhysXFabric
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysXFabric PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysXFabric PRIVATE carb)
endif()

# physicsSchemaTools + physxSchema libraries
if(DEFINED USD_EXT_PHYSICS_DIR AND EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimePhysXFabric PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimePhysXFabric PRIVATE physicsSchemaTools physxSchema)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimePhysXFabric)

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimePhysXFabric FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimePhysXFabric PRIVATE rt)
endif()

# CUDA libraries
if(DEFINED CUDA_DIR)
    if(WIN32)
        target_link_directories(OvruntimePhysXFabric PRIVATE ${CUDA_DIR}/lib/x64)
        target_link_libraries(OvruntimePhysXFabric PRIVATE cuda cudart_static)
    else()
        target_link_directories(OvruntimePhysXFabric PRIVATE
            ${CUDA_DIR}/lib64
        )
        # Pass stubs/ as a bare -L so CMake does NOT add it to RUNPATH.
        # Prevents stub libcuda.so (SONAME libcuda.so.1) from shadowing the
        # real CUDA driver at runtime (CUDA_ERROR_STUB_LIBRARY / error 34).
        target_link_options(OvruntimePhysXFabric PRIVATE
            -L${CUDA_DIR}/lib64/stubs
        )
        target_link_libraries(OvruntimePhysXFabric PRIVATE cuda cudart_static)
    endif()
endif()

# CUDA sources — platform-adaptive: native CUDA language (Linux) or direct nvcc
# invocation via add_custom_command (Windows). See cmake/CudaUtils.cmake.
ovruntime_target_add_cuda_sources(OvruntimePhysXFabric
    SOURCES     ${FABRIC_CUDA_SOURCE}
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/plugins"
)

# Precompiled header (CXX only — CUDA files don't use PCH)
target_precompile_headers(OvruntimePhysXFabric PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# --- Python bindings (optional) ---
if(OVRUNTIME_ENABLE_PYTHON_BINDINGS)
    set(FABRIC_BINDINGS_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXFabricPython.cpp
    )
    source_group(bindings FILES ${FABRIC_BINDINGS_SOURCE})

    add_library(OvruntimePhysXFabricBindings SHARED
        ${FABRIC_BINDINGS_SOURCE}
    )

    set_property(TARGET OvruntimePhysXFabricBindings PROPERTY FOLDER "Runtime/PhysXFabric")

    set_target_properties(OvruntimePhysXFabricBindings PROPERTIES
        OUTPUT_NAME "_physxFabric"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(OvruntimePhysXFabricBindings PROPERTIES SUFFIX ".pyd")
    else()
        set_target_properties(OvruntimePhysXFabricBindings PROPERTIES SUFFIX ".so")
    endif()

    target_include_directories(OvruntimePhysXFabricBindings PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include
        ${OVRUNTIME_ROOT_DIR}/include/private
        ${OVRUNTIME_ROOT_DIR}/source/pch
        ${OVRUNTIME_ROOT_DIR}/source/common/include
    )

    # External dependencies — SYSTEM includes to suppress third-party warnings
    ovruntime_add_external_system_includes(OvruntimePhysXFabricBindings)

    target_compile_definitions(OvruntimePhysXFabricBindings PRIVATE
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    if(NOT WIN32)
        target_compile_options(OvruntimePhysXFabricBindings PRIVATE -fexceptions -frtti -fPIC)
    endif()

    if(DEFINED CARB_SDK_DIR)
        target_link_directories(OvruntimePhysXFabricBindings PRIVATE
            ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
        )
        target_link_libraries(OvruntimePhysXFabricBindings PRIVATE carb)
    endif()

    # Python + USD boost::python libraries
    ovruntime_link_bindings_deps(OvruntimePhysXFabricBindings)

    # Precompiled header
    target_precompile_headers(OvruntimePhysXFabricBindings PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
    )

    # Install bindings
    install(
        TARGETS OvruntimePhysXFabricBindings
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
    )
endif()

# Install rules
install(
    TARGETS OvruntimePhysXFabric
    EXPORT OvruntimePhysXFabric
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)
