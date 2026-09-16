## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

# Single source of truth for the Windows-platform PhysXGpu resource files.
# Included by:
#  1. cmakegpu/windows/PhysXGpu.cmake when PX_GENERATE_GPU_PROJECTS is ON.
#  2. cmake/windows/CMakeLists.txt source-distro pass on non-GPU Windows
#     presets (e.g. windows-crosscompile, used by --distro_name=public on
#     Linux hosts), so the public source distro is complete on Linux hosts
#     that cannot run the CUDA toolchain.
#
# This file MUST contain only SET commands referencing existing source paths.
# It MUST NOT define targets, enable languages, find packages, or otherwise
# require the CUDA toolchain - the source-distro pass on non-GPU Windows
# presets walks it without ever entering cmakegpu/CMakeLists.txt.

SET(PHYSXGPU_RESOURCE
	${PHYSX_SOURCE_DIR}/compiler/windows/resource/PhysXGpu.rc
	${PHYSX_SOURCE_DIR}/compiler/windows/resource/resource.h
)
