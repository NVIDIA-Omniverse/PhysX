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

IF(NOT DEFINED PHYSX_LINUX_COMPILE_DEFS)
	MESSAGE(FATAL ERROR "Snippets uses the PhysX compile defs, and they're not defined when they need to be.")
ENDIF()

IF (NOT DEFINED PHYSX_CXX_FLAGS)
	MESSAGE(FATAL ERROR "Snippets uses the PhysX CXX flags, and they're not defined when they need to be.")
ENDIF()

# Get the CXX Flags from the Cached variables set by the PhysX CMakeLists
SET(CMAKE_CXX_FLAGS "${PHYSX_CXX_FLAGS}")

SET(CMAKE_CXX_FLAGS_DEBUG   "${PHYSX_CXX_FLAGS_DEBUG}")
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

# CUDA flags specific to Linux:
IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	SET(MSSE2_OPTIONS "")
	# Add the diagnostic suppression flag specifically for aarch64
	SET(AARCH64_DIAG_SUPPRESS_FLAGS "-diag-suppress 20054")
ELSE()
	SET(MSSE2_OPTIONS "-msse2,-mfpmath=sse,-m64,")
	SET(AARCH64_DIAG_SUPPRESS_FLAGS "")
ENDIF()

# CUDA flags specific to linux:
SET(CUDA_SUPPRESS_WARNINGS "-Wno-unused-function -Wno-inconsistent-missing-override -Wno-parentheses -Wno-unknown-warning-option")

SET(PHYSX_CUDA_FLAGS "${PHYSX_CUDA_FLAGS} ${CUDA_SUPPRESS_WARNINGS} ${AARCH64_DIAG_SUPPRESS_FLAGS}")

SET(PHYSX_CUDA_FLAGS_DEBUG   "${CUDA_DEBUG_FLAG} --compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden -G -g" CACHE INTERNAL "PhysX Debug CUDA Flags")
SET(PHYSX_CUDA_FLAGS_CHECKED "${CUDA_NDEBUG_FLAG} -lineinfo --compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden" CACHE INTERNAL "PhysX Checked CUDA Flags")
SET(PHYSX_CUDA_FLAGS_PROFILE "${CUDA_NDEBUG_FLAG} -lineinfo --compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden" CACHE INTERNAL "PhysX Profile CUDA Flags")
SET(PHYSX_CUDA_FLAGS_RELEASE "${CUDA_NDEBUG_FLAG} -lineinfo --compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden" CACHE INTERNAL "PhysX Release CUDA Flags")

# These flags are local to the directory the CMakeLists.txt is in, so don't get carried over to OTHER CMakeLists.txt (thus the CACHE variables above)
SET(CMAKE_CUDA_FLAGS ${PHYSX_CUDA_FLAGS})

SET(CMAKE_CUDA_FLAGS_DEBUG   ${PHYSX_CUDA_FLAGS_DEBUG})
SET(CMAKE_CUDA_FLAGS_CHECKED ${PHYSX_CUDA_FLAGS_CHECKED})
SET(CMAKE_CUDA_FLAGS_PROFILE ${PHYSX_CUDA_FLAGS_PROFILE})
SET(CMAKE_CUDA_FLAGS_RELEASE ${PHYSX_CUDA_FLAGS_RELEASE})


# SET(GENERATED_GPU_CUDA_FILES "")

# Include all of the projects
INCLUDE(PhysXBroadphaseGpu.cmake)
INCLUDE(PhysXCommonGpu.cmake)
INCLUDE(PhysXNarrowphaseGpu.cmake)
INCLUDE(PhysXSimulationControllerGpu.cmake)
INCLUDE(PhysXSolverGpu.cmake)
INCLUDE(PhysXCudaContextManager.cmake)
INCLUDE(PhysXArticulationGpu.cmake)
INCLUDE(PhysXGpuDependencies.cmake)
INCLUDE(PhysXGpu.cmake)	 # must be last

SET_PROPERTY(TARGET PhysXBroadphaseGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXCommonGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXNarrowphaseGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXSimulationControllerGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXArticulationGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXSolverGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXGpuDependencies PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXGpu PROPERTY FOLDER "PhysX SDK/GPU")
SET_PROPERTY(TARGET PhysXCudaContextManager PROPERTY FOLDER "PhysX SDK/GPU")	

SET(PHYSXDISTRO_LIBS PhysXGpu)

INSTALL(
	TARGETS ${PHYSXDISTRO_LIBS}
	EXPORT PhysXSDK
	DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>	
)
	
