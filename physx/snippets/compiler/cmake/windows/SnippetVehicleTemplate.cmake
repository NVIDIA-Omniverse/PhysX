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
# Build Snippet win template
#

IF(NOT FREEGLUT_PATH)
	# Detect if using vcpkg or packman
	IF(DEFINED ENV{VCPKG_ROOT} OR CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg")
		# Using vcpkg mode
		IF(DEFINED ENV{VCPKG_ROOT})
			FILE(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}/installed/x64-windows" FREEGLUT_PATH)
			SET(FREEGLUT_PATH "${FREEGLUT_PATH}" CACHE INTERNAL "Freeglut package path (vcpkg)")
		ELSE()
			# Extract vcpkg root from CMAKE_TOOLCHAIN_FILE
			GET_FILENAME_COMPONENT(VCPKG_ROOT_FROM_TOOLCHAIN "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
			GET_FILENAME_COMPONENT(VCPKG_ROOT_FROM_TOOLCHAIN "${VCPKG_ROOT_FROM_TOOLCHAIN}" DIRECTORY)
			GET_FILENAME_COMPONENT(VCPKG_ROOT_FROM_TOOLCHAIN "${VCPKG_ROOT_FROM_TOOLCHAIN}" DIRECTORY)
			FILE(TO_CMAKE_PATH "${VCPKG_ROOT_FROM_TOOLCHAIN}/installed/x64-windows" FREEGLUT_PATH)
			SET(FREEGLUT_PATH "${FREEGLUT_PATH}" CACHE INTERNAL "Freeglut package path (vcpkg)")
		ENDIF()
	ELSE()
		# Using packman mode
		SET(FREEGLUT_PATH $ENV{PM_freeglut_PATH} CACHE INTERNAL "Freeglut package path")
	ENDIF()
ENDIF()

SET(SNIPPET_COMPILE_DEFS
	# Common to all configurations

	${PHYSX_WINDOWS_COMPILE_DEFS};RENDER_SNIPPET;${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

SET(SNIPPET_PLATFORM_SOURCES
	${PHYSX_ROOT_DIR}/snippets/snippetcommon/ClassicMain.cpp
)

SET(SNIPPET_PLATFORM_INCLUDES

)

# Set freeglut library paths - different for vcpkg vs packman
IF(DEFINED ENV{VCPKG_ROOT} OR CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg")
	# vcpkg mode: libraries in lib/ and debug/lib/ without win64 subdirectory
	SET(FREEGLUT_LIB
		$<$<CONFIG:debug>:${FREEGLUT_PATH}/debug/lib/freeglutd.lib>
		$<$<CONFIG:checked>:${FREEGLUT_PATH}/lib/freeglut.lib>
		$<$<CONFIG:profile>:${FREEGLUT_PATH}/lib/freeglut.lib>
		$<$<CONFIG:release>:${FREEGLUT_PATH}/lib/freeglut.lib>
	)
ELSE()
	# packman mode: libraries in lib/win${LIBPATH_SUFFIX}/ subdirectory
	SET(FREEGLUT_LIB
		$<$<CONFIG:debug>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglutd.lib>
		$<$<CONFIG:checked>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
		$<$<CONFIG:profile>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
		$<$<CONFIG:release>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
	)
ENDIF()

SET(SNIPPET_PLATFORM_LINKED_LIBS
	SnippetRender ${FREEGLUT_LIB}
)

IF(PX_GENERATE_GPU_STATIC_LIBRARIES)
	LIST(APPEND SNIPPET_PLATFORM_LINKED_LIBS PhysXGpu)
ENDIF()

IF(NOT PX_GENERATE_STATIC_LIBRARIES)
	LIST(APPEND SNIPPET_PLATFORM_LINKED_LIBS PhysXTask)
ENDIF()
