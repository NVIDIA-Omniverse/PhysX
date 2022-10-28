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


SET(CUDA_COMPILER_OPTION_DEBUG "${CUDA_SUPPRESS_WARNINGS} --compiler-options=/W3,/nologo,/Zi,/Od,/RTC1,${CUDA_CRT_COMPILE_OPTIONS_DEBUG}")
SET(CUDA_COMPILER_OPTION_CHECKED "-lineinfo ${CUDA_SUPPRESS_WARNINGS} --compiler-options=/W3,/nologo,/Ot,/Ox,/Zi,${CUDA_CRT_COMPILE_OPTIONS_NDEBUG}")
SET(CUDA_COMPILER_OPTION_PROFILE "-lineinfo ${CUDA_SUPPRESS_WARNINGS} --compiler-options=/W3,/nologo,/Ot,/Ox,/Zi,${CUDA_CRT_COMPILE_OPTIONS_NDEBUG}")
SET(CUDA_COMPILER_OPTION_RELEASE "-lineinfo ${CUDA_SUPPRESS_WARNINGS} --compiler-options=/W3,/nologo,/Ot,/Ox,/Zi,${CUDA_CRT_COMPILE_OPTIONS_NDEBUG}")



SET(PHYSX_GPU_EXTENSIONS_PLATFORM_INCLUDES
	PRIVATE ${PHYSX_SOURCE_DIR}/Common/src/windows
)

SET(PHYSX_GPU_EXTENSIONS_PLATFORM_OBJECT_FILES
	$<TARGET_OBJECTS:FastXml>
)

SET(PHYSX_GPU_EXTENSIONS_PLATFORM_SRC_FILES
	${PHYSX_GPU_EXTENSIONS_PLATFORM_OBJECT_FILES}
)

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSX_GPU_EXTENSIONS_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

SET(PHYSX_GPU_EXTENSIONS_PRIVATE_PLATFORM_LINKED_LIBS
	${CUDA_CUDA_LIBRARY}
)

IF(NOT MSVC OR MSVC_TOOLSET_VERSION GREATER 140)
SET(PX_GPU_ARCH_FLAGS
		"-gencode arch=compute_50,code=sm_50\
		 -gencode arch=compute_60,code=sm_60\
		 -gencode arch=compute_70,code=sm_70\
		 -gencode arch=compute_75,code=sm_75
		 -gencode arch=compute_80,code=sm_80\
		 -gencode arch=compute_86,code=compute_86")
ELSE()
	SET(PX_GPU_ARCH_FLAGS
		"-gencode arch=compute_50,code=sm_50\
		 -gencode arch=compute_60,code=sm_60\
		 -gencode arch=compute_70,code=sm_70\
		 -gencode arch=compute_75,code=compute_75")
ENDIF()


SET(PHYSX_GPU_EXTENSIONS_LIBTYPE STATIC)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSX_GPU_EXTENSIONS_LIBTYPE STREQUAL "STATIC")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_DEBUG	  "PhysXGPUExtensions_static${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_CHECKED "PhysXGPUExtensions_static${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_PROFILE "PhysXGPUExtensions_static${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_RELEASE "PhysXGPUExtensions_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_DEBUG   "PhysXGPUExtensions${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_CHECKED "PhysXGPUExtensions${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_PROFILE "PhysXGPUExtensions${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_RELEASE "PhysXGPUExtensions${CMAKE_RELEASE_POSTFIX}")
ENDIF()

IF(PHYSX_GPU_EXTENSIONS_LIBTYPE STREQUAL "SHARED")
	INSTALL(FILES $<TARGET_PDB_FILE:PhysXGPUExtensions> 
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
ELSE()	
	INSTALL(FILES ${PHYSX_ROOT_DIR}/$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>/$<$<CONFIG:debug>:${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_DEBUG}>$<$<CONFIG:checked>:${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_CHECKED}>$<$<CONFIG:profile>:${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_PROFILE}>$<$<CONFIG:release>:${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_RELEASE}>.pdb
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
ENDIF()
