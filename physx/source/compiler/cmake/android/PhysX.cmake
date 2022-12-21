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
## Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#
# Build PhysX (PROJECT not SOLUTION)
#

SET(PHYSX_PLATFORM_INCLUDES
	${NVTOOLSEXT_INCLUDE_DIRS}
)

SET(PHYSX_PLATFORM_OBJECT_FILES
	$<TARGET_OBJECTS:LowLevel>
	$<TARGET_OBJECTS:LowLevelAABB>
	$<TARGET_OBJECTS:LowLevelDynamics>
	$<TARGET_OBJECTS:PhysXTask>
	$<TARGET_OBJECTS:SceneQuery>
	$<TARGET_OBJECTS:SimulationController>	
)

# Required for some callbacks taking CUstream as an argument even when building with PX_SUPPORT_GPU_PHYSX = 0
SET(PHYSX_CUDATYPES_GPU_HEADERS
	${PHYSX_ROOT_DIR}/include/cudamanager/PxCudaTypes.h
)
SOURCE_GROUP(include\\cudamanager FILES ${PHYSX_CUDATYPES_GPU_HEADERS})

SET(PHYSX_PLATFORM_SRC_FILES
	${PX_SOURCE_DIR}/device/linux/PhysXIndicatorLinux.cpp
	${PX_SOURCE_DIR}/gpu/PxGpu.cpp
	${PX_SOURCE_DIR}/gpu/PxPhysXGpuModuleLoader.cpp 
	
	${PHYSX_CUDATYPES_GPU_HEADERS}
	
	${PHYSX_PLATFORM_OBJECT_FILES}
)

INSTALL(FILES ${PHYSX_CUDATYPES_GPU_HEADERS} DESTINATION include/cudamanager)

SET(PHYSX_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_ANDROID_COMPILE_DEFS};PX_PHYSX_CORE_EXPORTS
	
	$<$<CONFIG:debug>:${PHYSX_ANDROID_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_ANDROID_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_ANDROID_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_ANDROID_RELEASE_COMPILE_DEFS};>
)

IF(PX_GENERATE_STATIC_LIBRARIES)
	SET(PHYSX_LIBTYPE STATIC)
ELSE()
	SET(PHYSX_LIBTYPE SHARED)
ENDIF()

