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
# Build PhysXGPUExtensions
#

IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	FIND_PACKAGE(CUDAToolkit REQUIRED)
	SET(MSSE2_OPTIONS "")
ELSE()
	SET(MSSE2_OPTIONS "-msse2,-mfpmath=sse,-m64,")
ENDIF()

SET(LRT_OPTION "-lrt")

SET(CUDA_COMPILER_OPTION_DEBUG "--compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden")
SET(CUDA_COMPILER_OPTION_CHECKED "--compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden")
SET(CUDA_COMPILER_OPTION_PROFILE "--compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden")
SET(CUDA_COMPILER_OPTION_RELEASE "--compiler-options=-Wall,-O3,-fPIC,${MSSE2_OPTIONS}-fvisibility=hidden")

SET(PHYSX_GPU_EXTENSIONS_PLATFORM_INCLUDES
	PRIVATE ${PHYSX_SOURCE_DIR}/Common/src/windows
)

SET(PHYSX_GPU_EXTENSIONS_PLATFORM_SRC_FILES
	${PHYSX_GPU_EXTENSIONS_PLATFORM_OBJECT_FILES}
)

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSX_GPU_EXTENSIONS_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_LINUX_COMPILE_DEFS};${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS};${PHYSX_GPU_EXTENSIONS_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

IF(PX_GENERATE_GPU_PROJECTS)
	# gwoolery: copied from other similar instances where CUDA_CUDA_LIBRARY is used. TC will fail without this.
	# preist@: FIND_PACKAGE will not find the fallback stub/libcuda.so in case that the
	# machine does not have graphics drivers installed (that come with libcuda.so)
	# and linking will fail. So use stub fallback explicitly here:
	IF(${CUDA_CUDA_LIBRARY} STREQUAL "CUDA_CUDA_LIBRARY-NOTFOUND")
		find_library(CUDA_CUDA_LIBRARY cuda PATHS ${CUDA_TOOLKIT_TARGET_DIR}/lib64/stubs)
	ENDIF()
ENDIF()

SET(PHYSX_GPU_EXTENSIONS_PRIVATE_PLATFORM_LINKED_LIBS
	${CUDA_CUDA_LIBRARY}
	${LRT_OPTION}
)

SET(PHYSX_GPU_EXTENSIONS_LIBTYPE STATIC)
