## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

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
