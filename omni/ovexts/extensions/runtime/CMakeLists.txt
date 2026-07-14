# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Runtime extensions: redirect ovruntime target outputs into the extension layout.
# These extensions do NOT compile any C++ — they package pre-built ovruntime binaries
# alongside Python source, config, and documentation.

# Fix Python extension suffix on ovruntime binding targets.
# ovruntime sets a bare ".so" (Linux) / ".pyd" (Windows) suffix, but pybind11 bindings
# need the full cpython suffix (e.g. .cpython-312-x86_64-linux-gnu.so) so that
# repo stubgen can discover them for .pyi type stub generation.
foreach(_bindings_target
    OvruntimePhysXBindings
    OvruntimePhysXCookingBindings
    OvruntimePhysXFabricBindings
    OvruntimeUsdPhysicsBindings
    OvruntimeConvexDecompositionBindings
    OvruntimePhysicsTensorsBindings
)
    if(TARGET ${_bindings_target})
        set_target_properties(${_bindings_target} PROPERTIES SUFFIX "${OVEXTS_PYTHON_EXT_SUFFIX}")
    endif()
endforeach()

# Force-build ovruntime targets that were included with EXCLUDE_FROM_ALL
add_custom_target(ovexts_runtime_libs ALL)

add_subdirectory(source/omni.convexdecomposition)
add_subdirectory(source/omni.usd.schema.physx)
add_subdirectory(source/omni.usdphysics)
add_subdirectory(source/omni.physx.cooking)
add_subdirectory(source/omni.physx.gpu)
add_subdirectory(source/omni.physx)
add_subdirectory(source/omni.physx.fabric)
add_subdirectory(source/omni.physx.tensors)
add_subdirectory(source/omni.physics.tensors)
