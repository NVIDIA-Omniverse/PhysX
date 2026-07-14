# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physx")

# Direct ovruntime plugin output to extension bin/ directory
ovexts_set_output_directory(OvruntimePhysX "${EXT_DIR}/bin")
add_dependencies(ovexts_runtime_libs OvruntimePhysX)

# PVDRuntime is built/copied by OvruntimePhysXGpu into omni.physx.gpu/bin/,
# but omni.physx extension.toml declares it as a native.library in its own bin/.
set(_GPU_EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/omni.physx.gpu")
if(WIN32)
    set(_PVDRUNTIME_NAME "PVDRuntime_64.dll")
else()
    set(_PVDRUNTIME_NAME "libPVDRuntime_64.so")
endif()
add_custom_target(omni.physx.pvdruntime_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_GPU_EXT_DIR}/bin/${_PVDRUNTIME_NAME}"
        "${EXT_DIR}/bin/${_PVDRUNTIME_NAME}"
    COMMENT "Copying ${_PVDRUNTIME_NAME} to omni.physx extension"
    DEPENDS OvruntimePhysXGpu
)

# Direct ovruntime Python bindings to extension bindings/ directory
if(TARGET OvruntimePhysXBindings)
    ovexts_set_output_directory(OvruntimePhysXBindings "${EXT_DIR}/${EXT_PYTHON_NS}/bindings")
    add_dependencies(ovexts_runtime_libs OvruntimePhysXBindings)
endif()

# Static extension content
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config" "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"   "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"   "${EXT_DIR}/docs")

# Python source from ovruntime
set(_OVRUNTIME_PYTHON "${OVRUNTIME_DIR}/source/${EXT_NAME}/python")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/__init__.py" "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_link("${_OVRUNTIME_PYTHON}/scripts"     "${EXT_DIR}/${EXT_PYTHON_NS}/scripts")
ovexts_prebuild_link("${_OVRUNTIME_PYTHON}/tests"        "${EXT_DIR}/${EXT_PYTHON_NS}/tests")

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME})
