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
# Build SnippetRender
#

IF(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
	find_package(OpenGL $ENV{PM_OpenGL_VERSION} CONFIG REQUIRED) # Pull in OpenGL and GLUT
ENDIF()
IF(NOT PUBLIC_RELEASE)
	find_package(CUDA $ENV{PM_CUDA_Version} REQUIRED)
ENDIF()

SET(SNIPPETRENDER_COMPILE_DEFS
	# Common to all configurations

	${PHYSX_LINUX_COMPILE_DEFS};

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

# vreutskyy: copied from SdkUnitTest.cmake
# preist@: FIND_PACKAGE will not find the fallback stub/libcuda.so in case that the
# machine does not have graphics drivers installed (that come with libcuda.so)
# and linking will fail. So use stub fallback explicitly here:
IF(NOT PUBLIC_RELEASE)
	IF(${CUDA_CUDA_LIBRARY} STREQUAL "CUDA_CUDA_LIBRARY-NOTFOUND")
		find_library(CUDA_CUDA_LIBRARY cuda PATHS ${CUDA_TOOLKIT_TARGET_DIR}/lib64/stubs)
	ENDIF()
	SET(CUDA_LIBS ${CUDA_CUDA_LIBRARY})
ENDIF()

SET(SNIPPETRENDER_PLATFORM_INCLUDES)

# gwoolery: aarch64 requires glut library to be lower case, for whatever reason
IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	SET(GLUT_LIB "glut")
ELSE()
	SET(GLUT_LIB "GLUT")
ENDIF()

SET(SNIPPETRENDER_PLATFORM_LINKED_LIBS GL GLU ${GLUT_LIB})

IF(NOT PUBLIC_RELEASE)
	LIST(APPEND SNIPPETRENDER_PLATFORM_INCLUDES ${CUDA_INCLUDE_DIRS})
	LIST(APPEND SNIPPETRENDER_PLATFORM_LINKED_LIBS ${CUDA_LIBS})
ENDIF()
