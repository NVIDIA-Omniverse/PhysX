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
# Build PhysXCudaContextManager common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/cudamanager)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXCudaContextManager.cmake)

SET(CUDACONTEXTMANAGER_HEADERS
	${PHYSX_ROOT_DIR}/include/cudamanager/PxCudaContextManager.h
	${PHYSX_ROOT_DIR}/include/cudamanager/PxCudaContext.h
	${PHYSX_ROOT_DIR}/include/cudamanager/PxCudaTypes.h
)
SOURCE_GROUP(include FILES ${CUDACONTEXTMANAGER_HEADERS})

SET(CUDACONTEXTMANAGER_KERNELS
)
SOURCE_GROUP("src kernels" FILES ${CUDACONTEXTMANAGER_KERNELS})

IF(PUBLIC_RELEASE)
    SET(CUDACONTEXTMANAGER_SOURCE
        ${LL_SOURCE_DIR}/src/CudaContextManager.cpp
        ${LL_SOURCE_DIR}/src/CudaKernelWrangler.cpp
    )

    SET(CUDACONTEXTMANAGER_SOURCE_HEADERS
        ${LL_SOURCE_DIR}/include/CudaContextManager.h
        ${LL_SOURCE_DIR}/include/CudaKernelWrangler.h
        ${LL_SOURCE_DIR}/include/PxgMemoryTracker.h
        ${PHYSX_SOURCE_DIR}/physx/src/opensource/cudamanager/include/PhysXDeviceSettings.h
    )
ELSE()
    SET(CUDACONTEXTMANAGER_SOURCE
        ${LL_SOURCE_DIR}/src/CudaContextManager.cpp
        ${LL_SOURCE_DIR}/src/CudaKernelWrangler.cpp
        ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/src/PhysXDeviceSettings.cpp
    )

    SET(CUDACONTEXTMANAGER_SOURCE_HEADERS
        ${LL_SOURCE_DIR}/include/CudaContextManager.h
        ${LL_SOURCE_DIR}/include/CudaKernelWrangler.h
        ${LL_SOURCE_DIR}/include/PxgMemoryTracker.h
        ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/include/PhysXDevice.h
        ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/include/PhysXDeviceSettings.h
    )
ENDIF()

SOURCE_GROUP("src\\src" FILES ${CUDACONTEXTMANAGER_SOURCE} ${CUDACONTEXTMANAGER_SOURCE_HEADERS})

ADD_LIBRARY(PhysXCudaContextManager ${CUDACONTEXTMANAGER_LIBTYPE}
	${CUDACONTEXTMANAGER_HEADERS}
	${CUDACONTEXTMANAGER_SOURCE}
	${CUDACONTEXTMANAGER_SOURCE_HEADERS}
	${CUDACONTEXTMANAGER_KERNELS}
	${CUDACONTEXTMANAGER_PLATFORM_SOURCES}
)

INSTALL(FILES ${CUDACONTEXTMANAGER_HEADERS} DESTINATION include/cudamanager)

# Target specific compile options

TARGET_INCLUDE_DIRECTORIES(PhysXCudaContextManager
	PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
	PRIVATE ${PHYSX_SOURCE_DIR}/task/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
	PRIVATE ${LL_SOURCE_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${CUDACONTEXTMANAGER_PLAFORM_HEADERS}
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
)

# Add CUDA manager include directory
IF(PUBLIC_RELEASE)
    TARGET_INCLUDE_DIRECTORIES(PhysXCudaContextManager PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/opensource/cudamanager/include)
ELSE()
    TARGET_INCLUDE_DIRECTORIES(PhysXCudaContextManager PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/include)
ENDIF()

TARGET_COMPILE_DEFINITIONS(PhysXCudaContextManager
	PRIVATE ${CUDACONTEXTMANAGER_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXCudaContextManager PROPERTIES
	OUTPUT_NAME PhysXCudaContextManager
)

SET_TARGET_PROPERTIES(PhysXCudaContextManager PROPERTIES
	ARCHIVE_NAME PhysXCudaContextManager
)

IF(CUDACONTEXTMANAGER_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXCudaContextManager PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXCudaContextManager_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXCudaContextManager_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXCudaContextManager_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXCudaContextManager_static"
	)
ENDIF()

IF(CUDACONTEXTMANAGER_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXCudaContextManager PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${CUDACONTEXTMANAGER_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${CUDACONTEXTMANAGER_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${CUDACONTEXTMANAGER_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${CUDACONTEXTMANAGER_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${CUDACONTEXTMANAGER_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${CUDACONTEXTMANAGER_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${CUDACONTEXTMANAGER_SOURCE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${CUDACONTEXTMANAGER_KERNELS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${CUDACONTEXTMANAGER_PLATFORM_SOURCES})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXCudaContextManager PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)

