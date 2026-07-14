# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physics.tensors")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physics/tensors")

# Direct ovruntime plugin output to extension bin/ directory
ovexts_set_output_directory(OvruntimePhysicsTensors "${EXT_DIR}/bin")
add_dependencies(ovexts_runtime_libs OvruntimePhysicsTensors)

# Direct ovruntime Python bindings to extension bindings/ directory
if(TARGET OvruntimePhysicsTensorsBindings)
    ovexts_set_output_directory(OvruntimePhysicsTensorsBindings "${EXT_DIR}/${EXT_PYTHON_NS}/bindings")
    add_dependencies(ovexts_runtime_libs OvruntimePhysicsTensorsBindings)
endif()

# Static extension content
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config" "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/data"   "${EXT_DIR}/data")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"   "${EXT_DIR}/docs")

# Python source from ovruntime — flat directory with many individual files
set(_OVRUNTIME_PYTHON "${OVRUNTIME_DIR}/source/${EXT_NAME}/python")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/__init__.py"        "${EXT_DIR}/${EXT_PYTHON_NS}/__init__.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/api.py"             "${EXT_DIR}/${EXT_PYTHON_NS}/api.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/extension.py"       "${EXT_DIR}/${EXT_PYTHON_NS}/extension.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/frontend_base.py"   "${EXT_DIR}/${EXT_PYTHON_NS}/frontend_base.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/frontend_np.py"     "${EXT_DIR}/${EXT_PYTHON_NS}/frontend_np.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/frontend_tf.py"     "${EXT_DIR}/${EXT_PYTHON_NS}/frontend_tf.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/frontend_torch.py"  "${EXT_DIR}/${EXT_PYTHON_NS}/frontend_torch.py")
ovexts_prebuild_copy("${_OVRUNTIME_PYTHON}/frontend_warp.py"   "${EXT_DIR}/${EXT_PYTHON_NS}/frontend_warp.py")
ovexts_prebuild_link("${_OVRUNTIME_PYTHON}/tests"              "${EXT_DIR}/${EXT_PYTHON_NS}/tests")
