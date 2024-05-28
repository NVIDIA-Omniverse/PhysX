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
## Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#
# Build SnippetRender
#

IF(NOT FREEGLUT_PATH)
	SET(FREEGLUT_PATH $ENV{PM_freeglut_PATH} CACHE INTERNAL "Freeglut package path")
ENDIF()

SET(SNIPPETRENDER_COMPILE_DEFS
	# Common to all configurations

	${PHYSX_WINDOWS_COMPILE_DEFS};PX_PHYSX_STATIC_LIB;GLUT_NO_LIB_PRAGMA;${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

SET(SNIPPETRENDER_PLATFORM_FILES ${FREEGLUT_PATH}/include/GL/freeglut.h)

# Include OpenGL
SET(SNIPPETRENDER_PLATFORM_INCLUDES ${FREEGLUT_PATH}/include)
SET(SNIPPETRENDER_PLATFORM_LINKED_LIBS opengl32.lib glu32.lib)

IF(NOT PUBLIC_RELEASE)
	LIST(APPEND SNIPPETRENDER_PLATFORM_INCLUDES ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
	LIST(APPEND SNIPPETRENDER_PLATFORM_LINKED_LIBS CUDA::cuda_driver)
ENDIF()

