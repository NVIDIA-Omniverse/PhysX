# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.cooking")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxcooking")

# Direct ovruntime plugin output to extension bin/ directory
ovexts_set_output_directory(OvruntimePhysXCooking "${EXT_DIR}/bin")
add_dependencies(ovexts_runtime_libs OvruntimePhysXCooking)

# Direct ovruntime Python bindings to extension bindings/ directory
if(TARGET OvruntimePhysXCookingBindings)
    ovexts_set_output_directory(OvruntimePhysXCookingBindings "${EXT_DIR}/${EXT_PYTHON_NS}/bindings")
    add_dependencies(ovexts_runtime_libs OvruntimePhysXCookingBindings)
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
