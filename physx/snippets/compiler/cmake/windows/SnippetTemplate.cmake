## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build Snippet win template
#

IF(NOT FREEGLUT_PATH)
	SET(FREEGLUT_PATH $ENV{PM_freeglut_PATH} CACHE INTERNAL "Freeglut package path")
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
	${PHYSX_ROOT_DIR}/snippets/snippetcommon/SnippetPrint.h
	${PHYSX_ROOT_DIR}/snippets/snippetcommon/SnippetPVD.h
)

SET(SNIPPET_PLATFORM_INCLUDES
	#adding PhysXGpu include for configs that don't add link target PhysXGpu
	${PHYSX_ROOT_DIR}/include/cudamanager
	${FREEGLUT_PATH}/include
)

#LINK_DIRECTORIES(${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX})
SET(FREEGLUT_LIB
	$<$<CONFIG:debug>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglutd.lib>
	$<$<CONFIG:checked>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
	$<$<CONFIG:profile>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
	$<$<CONFIG:release>:${FREEGLUT_PATH}/lib/win${LIBPATH_SUFFIX}/freeglut.lib>
)

IF(PX_GENERATE_STATIC_LIBRARIES)
	SET(SNIPPET_PLATFORM_LINKED_LIBS
		SnippetRender ${FREEGLUT_LIB}
	)
ELSE()
	SET(SNIPPET_PLATFORM_LINKED_LIBS
		SnippetRender SceneQuery ${FREEGLUT_LIB}
	)
ENDIF()

IF(${SNIPPET_NAME} STREQUAL "ConvexDecomposition")
	LIST(APPEND SNIPPET_PLATFORM_LINKED_LIBS
		VHACD
	)
ENDIF()

IF(PX_GENERATE_GPU_STATIC_LIBRARIES)
	LIST(APPEND SNIPPET_PLATFORM_LINKED_LIBS PhysXGpu)
ENDIF()
