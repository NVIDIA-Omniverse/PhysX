# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# --- Internal loader source ---
set(PHYSXGPU_LOADER_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXGpuLoader.h
)
source_group(headers FILES ${PHYSXGPU_LOADER_HEADERS})

set(PHYSXGPU_LOADER_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXGpuLoader.cpp
)
source_group(source FILES ${PHYSXGPU_LOADER_SOURCE})

# --- Create static internal loader library ---
add_library(OvruntimePhysXGpu STATIC
    ${PHYSXGPU_LOADER_HEADERS}
    ${PHYSXGPU_LOADER_SOURCE}
)

set_property(TARGET OvruntimePhysXGpu PROPERTY FOLDER "Runtime/PhysXGpu")

set_target_properties(OvruntimePhysXGpu PROPERTIES
    OUTPUT_NAME "OvruntimePhysXGpu"
)

# RTTI and exceptions enabled, no PCH (matching premake: flags { "NoPCH" })
if(NOT WIN32)
    target_compile_options(OvruntimePhysXGpu PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimePhysXGpu
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # PhysXGpuLoader.h
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXGpu)

# Link dependencies
if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysXGpu PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysXGpu PRIVATE carb)
endif()

# Platform-specific: link dl on Linux (for dynamic library loading)
if(NOT WIN32)
    target_link_libraries(OvruntimePhysXGpu PRIVATE dl)
endif()
