# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# --- Plugin source ---
set(PHYSXGPU_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXGpu.cpp
)
source_group(source FILES ${PHYSXGPU_PLUGIN_SOURCE})

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimePhysXGpu SHARED
    ${PHYSXGPU_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysXGpu PROPERTY FOLDER "Runtime/PhysXGpu")

set_target_properties(OvruntimePhysXGpu PROPERTIES
    OUTPUT_NAME "omni.physx.gpu.plugin"
)

# RTTI and exceptions enabled, no PCH (matching premake: flags { "NoPCH" })
if(NOT WIN32)
    target_compile_options(OvruntimePhysXGpu PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimePhysXGpu
    PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include                 # <omni/physx/IPhysxGpu.h>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXGpu)

# Compile definitions
target_compile_definitions(OvruntimePhysXGpu PRIVATE
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

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

# PhysXGpu, PVDRuntime, and (Windows-only) PhysXDevice64 are loaded at runtime.
# Copy them to the build output directory.
if(WIN32)
    set(_PHYSXGPU_LIB_NAME "PhysXGpu_64.dll")
    set(_PVDRUNTIME_LIB_NAME "PVDRuntime_64.dll")
else()
    set(_PHYSXGPU_LIB_NAME "libPhysXGpu_64.so")
    set(_PVDRUNTIME_LIB_NAME "libPVDRuntime_64.so")
endif()

set(_GPU_BIN_DIR "bin")
set(_PHYSX_BIN_DIR "bin")

if(OVRUNTIME_PHYSX_FROM_SOURCE)
    # When building PhysX from source, copy the GPU shared library from the PhysX build output.
    # EXCLUDE_FROM_ALL on the PhysX subdirectory means these targets won't build automatically —
    # add_dependencies ensures they are built when OvruntimePhysXGpu is built.
    if(TARGET PhysXGpu)
        add_dependencies(OvruntimePhysXGpu PhysXGpu)
        add_custom_command(TARGET OvruntimePhysXGpu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:PhysXGpu>"
                "$<TARGET_FILE_DIR:OvruntimePhysXGpu>/${_PHYSXGPU_LIB_NAME}"
            COMMENT "Copying ${_PHYSXGPU_LIB_NAME} from source build to bin directory"
        )
        install(FILES "$<TARGET_FILE:PhysXGpu>" DESTINATION ${_GPU_BIN_DIR} RENAME "${_PHYSXGPU_LIB_NAME}")
        install(FILES "$<TARGET_FILE:PhysXGpu>" DESTINATION ${_PHYSX_BIN_DIR} RENAME "${_PHYSXGPU_LIB_NAME}")
    endif()
    if(TARGET PVDRuntime)
        add_dependencies(OvruntimePhysXGpu PVDRuntime)
        add_custom_command(TARGET OvruntimePhysXGpu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:PVDRuntime>"
                "$<TARGET_FILE_DIR:OvruntimePhysXGpu>/${_PVDRUNTIME_LIB_NAME}"
            COMMENT "Copying ${_PVDRUNTIME_LIB_NAME} from source build to bin directory"
        )
        install(FILES "$<TARGET_FILE:PVDRuntime>" DESTINATION ${_PHYSX_BIN_DIR} RENAME "${_PVDRUNTIME_LIB_NAME}")
    endif()
elseif(DEFINED PHYSX_SDK_DIR)
    # Copy from packman package
    set(_PHYSXGPU_LIB_DIR "${PHYSX_SDK_DIR}/bin/${PHYSX_PLATFORM_BIN}")
    set(_PHYSXGPU_LIB "${_PHYSXGPU_LIB_DIR}/$<IF:$<CONFIG:Debug>,debug,checked>/${_PHYSXGPU_LIB_NAME}")
    set(_PVDRUNTIME_LIB "${_PHYSXGPU_LIB_DIR}/$<IF:$<CONFIG:Debug>,debug,checked>/${_PVDRUNTIME_LIB_NAME}")

    add_custom_command(TARGET OvruntimePhysXGpu POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_PHYSXGPU_LIB}"
            "$<TARGET_FILE_DIR:OvruntimePhysXGpu>/${_PHYSXGPU_LIB_NAME}"
        COMMENT "Copying ${_PHYSXGPU_LIB_NAME} to bin directory"
    )
    install(FILES "${_PHYSXGPU_LIB}" DESTINATION ${_GPU_BIN_DIR})
    install(FILES "${_PHYSXGPU_LIB}" DESTINATION ${_PHYSX_BIN_DIR})

    add_custom_command(TARGET OvruntimePhysXGpu POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_PVDRUNTIME_LIB}"
            "$<TARGET_FILE_DIR:OvruntimePhysXGpu>/${_PVDRUNTIME_LIB_NAME}"
        COMMENT "Copying ${_PVDRUNTIME_LIB_NAME} to bin directory"
    )
    install(FILES "${_PVDRUNTIME_LIB}" DESTINATION ${_PHYSX_BIN_DIR})
endif()

# PhysXDevice64 (Windows-only, always from packman)
if(WIN32)
    set(_PHYSX_DEVICE_DLL "${OVRUNTIME_TARGET_DEPS}/physxdevice/bin/x86/PhysXDevice64.dll")
    if(EXISTS "${_PHYSX_DEVICE_DLL}")
        add_custom_command(TARGET OvruntimePhysXGpu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_PHYSX_DEVICE_DLL}"
                "$<TARGET_FILE_DIR:OvruntimePhysXGpu>/PhysXDevice64.dll"
            COMMENT "Copying PhysXDevice64.dll to bin directory"
        )
        install(FILES "${_PHYSX_DEVICE_DLL}" DESTINATION ${_GPU_BIN_DIR})
    endif()
endif()

# Install rules
install(
    TARGETS OvruntimePhysXGpu
    EXPORT OvruntimePhysXGpu
    RUNTIME DESTINATION ${_GPU_BIN_DIR}
    LIBRARY DESTINATION ${_GPU_BIN_DIR}
    ARCHIVE DESTINATION lib
)
