# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Plugin headers (private) ---
set(TENSORS_PLUGIN_HEADERS
    # Root
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ArticulationMetatype.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CommonTypes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/GlobalsAreBad.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysicsTypes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SimulationBackend.h
    # Base
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseVolumeDeformableBodyView.h
    # CPU
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuVolumeDeformableBodyView.h
    # GPU
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/CudaCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/CudaKernels.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuVolumeDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/ThrustUtils.h
)
source_group(plugins\\headers FILES ${TENSORS_PLUGIN_HEADERS})

# --- Plugin sources (C++) ---
set(TENSORS_PLUGIN_SOURCE
    # Root
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ArticulationMetatype.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysxTensors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SimulationBackend.cpp
    # Base
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/base/BaseVolumeDeformableBodyView.cpp
    # CPU
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSimulationData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/cpu/CpuVolumeDeformableBodyView.cpp
    # GPU (C++ only — .cu handled separately)
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/CudaCommon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSimulationData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/GpuVolumeDeformableBodyView.cpp
)
source_group(plugins\\source FILES ${TENSORS_PLUGIN_SOURCE})

# --- CUDA sources ---
set(TENSORS_CUDA_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/gpu/CudaKernels.cu
)
source_group(plugins\\cuda FILES ${TENSORS_CUDA_SOURCE})

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimePhysXTensors SHARED
    ${TENSORS_PLUGIN_HEADERS}
    ${TENSORS_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysXTensors PROPERTY FOLDER "Runtime/PhysXTensors")

set_target_properties(OvruntimePhysXTensors PROPERTIES
    OUTPUT_NAME "omni.physx.tensors.plugin"
)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimePhysXTensors PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fexceptions -frtti -fPIC>
    )
endif()

# Include directories
target_include_directories(OvruntimePhysXTensors
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private         # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
        ${OVRUNTIME_TARGET_DEPS}                      # <physx/include/...> style includes
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXTensors)

# Compile definitions
target_compile_definitions(OvruntimePhysXTensors PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "$<$<COMPILE_LANGUAGE:CXX>:carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)>"
)

# Link dependencies
target_link_libraries(OvruntimePhysXTensors
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysXTensors PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysXTensors PRIVATE carb)
endif()

# physicsSchemaTools + physxSchema libraries
if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimePhysXTensors PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimePhysXTensors PRIVATE physicsSchemaTools physxSchema)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimePhysXTensors)

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimePhysXTensors FULL)
if(NOT WIN32)
    target_link_libraries(OvruntimePhysXTensors PRIVATE rt)
endif()

# CUDA libraries
if(DEFINED CUDA_DIR)
    if(WIN32)
        target_link_directories(OvruntimePhysXTensors PRIVATE ${CUDA_DIR}/lib/x64)
        target_link_libraries(OvruntimePhysXTensors PRIVATE cuda cudart_static)
    else()
        target_link_directories(OvruntimePhysXTensors PRIVATE
            ${CUDA_DIR}/lib64
        )
        # Pass stubs/ as a bare -L so CMake does NOT add it to RUNPATH.
        # Prevents stub libcuda.so (SONAME libcuda.so.1) from shadowing the
        # real CUDA driver at runtime (CUDA_ERROR_STUB_LIBRARY / error 34).
        target_link_options(OvruntimePhysXTensors PRIVATE
            -L${CUDA_DIR}/lib64/stubs
        )
        target_link_libraries(OvruntimePhysXTensors PRIVATE cuda cudart_static)
    endif()
endif()

# CUDA sources — platform-adaptive: native CUDA language (Linux) or direct nvcc
# invocation via add_custom_command (Windows). See cmake/CudaUtils.cmake.
ovruntime_target_add_cuda_sources(OvruntimePhysXTensors
    SOURCES     ${TENSORS_CUDA_SOURCE}
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/plugins"
)

# Precompiled header (CXX only — CUDA files don't use PCH)
target_precompile_headers(OvruntimePhysXTensors PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# Install rules
install(
    TARGETS OvruntimePhysXTensors
    EXPORT OvruntimePhysXTensors
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)
