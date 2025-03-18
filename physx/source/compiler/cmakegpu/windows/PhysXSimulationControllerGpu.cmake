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
## Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

#
# Build PhysXSimulationControllerGpu
#

IF(PX_GENERATE_GPU_STATIC_LIBRARIES)
	SET(PHYSXSIMULATIONCONTROLLERGPU_LIBTYPE OBJECT)
ELSE()
	SET(PHYSXSIMULATIONCONTROLLERGPU_LIBTYPE STATIC)
ENDIF()

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSXSIMULATIONCONTROLLERGPU_COMPILE_DEFS

	# Common to all configurations
	PRIVATE ${PHYSX_WINDOWS_COMPILE_DEFS};PX_PHYSX_STATIC_LIB;${PHYSXGPU_LIBTYPE_DEFS}

	PRIVATE $<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	PRIVATE $<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	PRIVATE $<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	PRIVATE $<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

IF(PHYSXSIMULATIONCONTROLLERGPU_LIBTYPE STREQUAL "STATIC")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_DEBUG "PhysXSimulationControllerGpu_static${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_CHECKED "PhysXSimulationControllerGpu_static${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_PROFILE "PhysXSimulationControllerGpu_static${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_RELEASE "PhysXSimulationControllerGpu_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_DEBUG "PhysXSimulationControllerGpu${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_CHECKED "PhysXSimulationControllerGpu${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_PROFILE "PhysXSimulationControllerGpu${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSXSCGPU_COMPILE_PDB_NAME_RELEASE "PhysXSimulationControllerGpu${CMAKE_RELEASE_POSTFIX}")
ENDIF()