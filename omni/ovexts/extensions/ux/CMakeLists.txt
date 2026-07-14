# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# UX extensions: compile C++ plugins and Python bindings for Omniverse Kit integration.

# C++ plugin + bindings extensions
add_subdirectory(source/omni.physx.camera)
add_subdirectory(source/omni.physx.pvd)
add_subdirectory(source/omni.physx.stageupdate)
add_subdirectory(source/omni.physx.ui)
add_subdirectory(source/omni.physx.supportui)
add_subdirectory(source/omni.usdphysics.ui)
add_subdirectory(source/omni.physx.vehicle)
add_subdirectory(source/omni.physx.vehicle.tests)
add_subdirectory(source/omni.physx.asset_validator)
# OGN extensions — require omni.graph.tools for code generation
if(OVEXTS_OGN_AVAILABLE)
    add_subdirectory(source/omni.physx.graph)
    add_subdirectory(source/omni.physx.cct)
else()
    message(WARNING "OGN toolchain not available — skipping omni.physx.graph and omni.physx.cct. "
        "Run 'build.sh --fetch-only' to resolve omni.graph.tools extension.")
endif()

# Python-only extensions
add_subdirectory(source/omni.kit.property.physics)
add_subdirectory(source/omni.physx.bundle)
add_subdirectory(source/omni.physx.tests)
add_subdirectory(source/omni.physx.tests.visual)
add_subdirectory(source/omni.physx.demos)
add_subdirectory(source/omni.physx.commands)
add_subdirectory(source/omni.usd.metrics.assembler.physics)
add_subdirectory(source/omni.physics.ui)
add_subdirectory(source/omni.physics.isaacsimready)
add_subdirectory(source/omni.physics.physx.ui)
