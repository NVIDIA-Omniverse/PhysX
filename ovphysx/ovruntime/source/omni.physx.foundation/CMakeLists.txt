# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# --- Internal CUDA driver shim source ---
set(PHYSX_CUDA_SHIM_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cuda_shim/CudaShim.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cuda_shim/CudaShim.cpp
)
source_group(source FILES ${PHYSX_CUDA_SHIM_SOURCE})

add_library(OvruntimePhysXCudaShim STATIC
    ${PHYSX_CUDA_SHIM_SOURCE}
)

set_property(TARGET OvruntimePhysXCudaShim PROPERTY FOLDER "Runtime/PhysXFoundation")

set_target_properties(OvruntimePhysXCudaShim PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    INSTALL_RPATH_USE_LINK_PATH FALSE
)

if(NOT WIN32)
    target_compile_options(OvruntimePhysXCudaShim PRIVATE -fexceptions -frtti -fPIC)
endif()

target_include_directories(OvruntimePhysXCudaShim
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins              # cuda_shim/CudaShim.h
        ${OVRUNTIME_ROOT_DIR}/include                    # <omni/physx/IOptionalCuda.h>
)

ovruntime_add_external_system_includes(OvruntimePhysXCudaShim)

if(DEFINED CUDA_DIR)
    target_include_directories(OvruntimePhysXCudaShim SYSTEM PUBLIC ${CUDA_DIR}/include)
endif()

if(WIN32)
    target_link_libraries(OvruntimePhysXCudaShim PUBLIC setupapi)
else()
    target_link_libraries(OvruntimePhysXCudaShim PUBLIC dl)
endif()

# --- Internal foundation service source ---
set(PHYSXFOUNDATION_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXFoundation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXFoundation.cpp
)
source_group(source FILES ${PHYSXFOUNDATION_PLUGIN_SOURCE})

# --- Create static internal library ---
add_library(OvruntimePhysXFoundation STATIC
    ${PHYSXFOUNDATION_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysXFoundation PROPERTY FOLDER "Runtime/PhysXFoundation")

set_target_properties(OvruntimePhysXFoundation PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    INSTALL_RPATH_USE_LINK_PATH FALSE
)

# RTTI and exceptions enabled, no PCH (matching old premake: flags { "NoPCH" })
if(NOT WIN32)
    target_compile_options(OvruntimePhysXFoundation PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimePhysXFoundation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins              # internal PhysXFoundation.h
        ${OVRUNTIME_ROOT_DIR}/include                    # <omni/physx/IPhysxFoundation.h>, <omni/physx/IOptionalCuda.h>
    PRIVATE
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/Defer.h>, <common/utilities/CudaHelpers.h>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXFoundation)

# Compile definitions
target_compile_definitions(OvruntimePhysXFoundation PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimePhysXFoundation
    PRIVATE OvruntimeCommon
    PRIVATE OvruntimePhysXCudaShim
    PUBLIC OvruntimePhysXGpu
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysXFoundation PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysXFoundation PRIVATE carb)
endif()

# PhysX static libraries provide the PxCreateCudaContextManager trampoline used
# by PxPhysXGpuModuleLoader. Do not also link the PhysXGpu import library here:
# on Windows that duplicates the symbol already present in PhysX_static_64.
ovruntime_link_physx_static_libs(OvruntimePhysXFoundation MINIMAL)

# Install rules
install(FILES
    ${OVRUNTIME_ROOT_DIR}/include/omni/physx/IPhysxFoundation.h
    DESTINATION include/omni/physx
)
