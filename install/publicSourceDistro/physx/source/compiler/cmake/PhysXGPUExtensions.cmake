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
# Build PhysXGPUExtensions common
#

FIND_PACKAGE(CUDA $ENV{PM_CUDA_Version} REQUIRED)

# CUDA!
SET(CUDA_NVCC_FLAGS "-use_fast_math -ftz=true -prec-div=false -prec-sqrt=false ${PX_GPU_ARCH_FLAGS} -D_CONSOLE -D_WIN32_WINNT=0x0501")

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxgpuextensions/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXGPUExtensions.cmake)

SET(CUDA_NVCC_FLAGS_DEBUG   "${CUDA_DEBUG_FLAG}  -D_CONSOLE -D_WIN32_WINNT=0x0501 ${CUDA_COMPILER_OPTION_DEBUG} -G -g")
SET(CUDA_NVCC_FLAGS_CHECKED "${CUDA_NDEBUG_FLAG} -D_CONSOLE -D_WIN32_WINNT=0x0501 ${CUDA_COMPILER_OPTION_CHECKED}")
SET(CUDA_NVCC_FLAGS_PROFILE "${CUDA_NDEBUG_FLAG} -D_CONSOLE -D_WIN32_WINNT=0x0501 ${CUDA_COMPILER_OPTION_PROFILE}")
SET(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NDEBUG_FLAG} -D_CONSOLE -D_WIN32_WINNT=0x0501 ${CUDA_COMPILER_OPTION_RELEASE}")

SET(PHYSX_GPU_EXTENSIONS_SOURCE
	${LL_SOURCE_DIR}/ExtIsosurfaceExtraction.cpp
	${LL_SOURCE_DIR}/ExtAnisotropy.cpp
	${LL_SOURCE_DIR}/ExtSparseGrid.cpp
	${LL_SOURCE_DIR}/ExtAlgorithms.cpp
    ${LL_SOURCE_DIR}/ExtUtility.cpp
	${LL_SOURCE_DIR}/ExtParticleExt.cpp
	${LL_SOURCE_DIR}/ExtParticleClothCooker.cpp
    ${LL_SOURCE_DIR}/ExtGpuMemory.cpp
	${LL_SOURCE_DIR}/ExtSoftBodyGPUExt.cpp
    ${LL_SOURCE_DIR}/ExtKernelManager.cpp
    ${LL_SOURCE_DIR}/ExtGpuExtensions.cpp
)


SOURCE_GROUP(src FILES ${PHYSX_GPU_EXTENSIONS_SOURCE})

SET(PHYSX_GPU_EXTENSIONS_HEADERS
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxAnisotropy.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxSmoothing.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxParticleNeighborhoodProvider.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxMultiCallback.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxAnisotropyData.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxAlgorithms.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxAlgorithmsData.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxGpuMemory.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxMDExt.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxIsosurfaceData.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxIsosurfaceExtraction.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxSparseGrid.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxUtility.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxParticleExt.h
	${PHYSX_ROOT_DIR}/include/gpuextensions/PxParticleClothCooker.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxDenseGridData.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxSparseGridData.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxSoftBodyGPUExt.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxKernelManager.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxKernelRegister.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxKernelModules.h
    ${PHYSX_ROOT_DIR}/include/gpuextensions/PxGpuExtensionsAPI.h
)

SOURCE_GROUP("extensions include" FILES ${PHYSX_GPU_EXTENSIONS_HEADERS})
SET(PHYSX_GPU_EXTENSIONS_CUDA_KERNELS
	${LL_SOURCE_DIR}/CUDA/extAnisotropy.cu
	${LL_SOURCE_DIR}/CUDA/ExtMDExt.cu
	${LL_SOURCE_DIR}/CUDA/extIsosurfaceExtraction.cu
	${LL_SOURCE_DIR}/CUDA/extSparseGrid.cu
	${LL_SOURCE_DIR}/CUDA/extAlgorithms.cu
    ${LL_SOURCE_DIR}/CUDA/extUtility.cu
    ${LL_SOURCE_DIR}/CUDA/extSoftBodyGPU.cu
    ${LL_SOURCE_DIR}/CUDA/denseGrid.cuh
    ${LL_SOURCE_DIR}/CUDA/sparseGrid.cuh
)
SOURCE_GROUP("extensions kernels/CUDA" FILES ${PHYSX_GPU_EXTENSIONS_CUDA_KERNELS})
SET(PHYSX_GPU_EXTENSIONS_CUDA_INCLUDE
)

SOURCE_GROUP(include FILES ${PHYSX_GPU_EXTENSIONS_HEADERS})



IF(PX_GENERATE_GPU_STATIC_LIBRARIES)
	LIST(APPEND GENERATED_GPU_CUDA_FILES ${GENERATED_CUDA_FILES})
	SET(GENERATED_CUDA_FILES "")
ENDIF()

CUDA_INCLUDE_DIRECTORIES(

	${PHYSXFOUNDATION_INCLUDES}

	${PHYSX_ROOT_DIR}/include
	${PHYSX_SOURCE_DIR}/common/include
	${PHYSX_SOURCE_DIR}/common/src
	${PHYSX_SOURCE_DIR}/geomutils/include
	${PHYSX_SOURCE_DIR}/geomutils/src
	${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	${PHYSX_SOURCE_DIR}/lowlevel/api/include
	${PHYSX_SOURCE_DIR}/lowlevel/software/include
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	${PHYSX_SOURCE_DIR}/lowleveldynamics/include
	${PHYSX_SOURCE_DIR}/lowleveldynamics/src
	${PHYSX_SOURCE_DIR}/physxgpu/src/common	
	${PHYSX_SOURCE_DIR}/physxgpu/include
	${PHYSX_SOURCE_DIR}/gpubroadphase/include
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include
	${PHYSX_SOURCE_DIR}/gpusolver/include
	${PHYSX_SOURCE_DIR}/gpucommon/include
	${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include	
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/src/CUDA
	${PHYSX_SOURCE_DIR}/gpuarticulation/include 

	${PHYSX_SOURCE_DIR}/physxgpu/src/CUDA	

	${CUDA_INCLUDE_DIRS}
	
	${LL_SOURCE_DIR}/include
	${LL_SOURCE_DIR}/src
)

CUDA_COMPILE(GENERATED_CUDA_FILES
	${PHYSX_GPU_EXTENSIONS_CUDA_KERNELS}
)


ADD_LIBRARY(PhysXGPUExtensions ${PHYSX_GPU_EXTENSIONS_LIBTYPE} 
	${PHYSX_GPU_EXTENSIONS_PLATFORM_SRC_FILES}
	
	${PHYSX_GPU_EXTENSIONS_SOURCE}
	
	${PHYSX_GPU_EXTENSIONS_HEADERS}

	${PHYSX_GPU_EXTENSIONS_CUDA_INCLUDE}

	${PHYSX_GPU_EXTENSIONS_CUDA_KERNELS}

	# Link in the CUDA files
	${GENERATED_CUDA_FILES}	
)

# Add the headers to the install
INSTALL(FILES ${PHYSX_GPU_EXTENSIONS_HEADERS} DESTINATION include/gpuextensions)

TARGET_INCLUDE_DIRECTORIES(PhysXGPUExtensions 

	PRIVATE ${PHYSX_GPU_EXTENSIONS_PLATFORM_INCLUDES}
	PRIVATE ${CUDA_INCLUDE_DIRS}

	PRIVATE ${PHYSX_ROOT_DIR}/include

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh

	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/core/include
	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/extensions/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/Xml
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/Binary
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/File

	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src

	PRIVATE ${PHYSX_SOURCE_DIR}/scenequery/include
)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(PhysXGPUExtensions 
	PRIVATE ${PHYSX_GPU_EXTENSIONS_COMPILE_DEFS}
)

TARGET_LINK_LIBRARIES(PhysXGPUExtensions
	PRIVATE ${PHYSX_GPU_EXTENSIONS_PRIVATE_PLATFORM_LINKED_LIBS}
	PUBLIC PhysXFoundation
	PUBLIC PhysXPvdSDK 
	PUBLIC PhysX
)



SET_TARGET_PROPERTIES(PhysXGPUExtensions PROPERTIES
	OUTPUT_NAME PhysXGPUExtensions
)





IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSX_GPU_EXTENSIONS_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXGPUExtensions PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG	"PhysXGPUExtensions_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXGPUExtensions_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXGPUExtensions_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXGPUExtensions_static"
	)
ENDIF()

IF(PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXGPUExtensions PROPERTIES 
		COMPILE_PDB_NAME_DEBUG	 ${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSX_GPU_EXTENSIONS_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)	
	# LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_GPU_EXTENSIONS_PLATFORM_SRC_FILES})	
	# LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_GPU_EXTENSIONS_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_GPU_EXTENSIONS_HEADERS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXGPUExtensions PROPERTIES POSITION_INDEPENDENT_CODE TRUE)