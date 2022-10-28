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
# Build LowLevel
#

SET(LOWLEVEL_PLATFORM_INCLUDES
	${PHYSX_SOURCE_DIR}/Common/src/windows
	${PHYSX_SOURCE_DIR}/LowLevel/software/include/windows
	${PHYSX_SOURCE_DIR}/LowLevelDynamics/include/windows
	${PHYSX_SOURCE_DIR}/LowLevel/common/include/pipeline/windows
)

IF(PX_GENERATE_STATIC_LIBRARIES)
	SET(LOWLEVEL_LIBTYPE OBJECT)
ELSE()
	SET(LOWLEVEL_LIBTYPE STATIC)
ENDIF()

SET(LOWLEVEL_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND LOWLEVEL_LIBTYPE STREQUAL "STATIC")
	SET(LL_COMPILE_PDB_NAME_DEBUG "LowLevel_static${CMAKE_DEBUG_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_CHECKED "LowLevel_static${CMAKE_CHECKED_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_PROFILE "LowLevel_static${CMAKE_PROFILE_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_RELEASE "LowLevel_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(LL_COMPILE_PDB_NAME_DEBUG "LowLevel${CMAKE_DEBUG_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_CHECKED "LowLevel${CMAKE_CHECKED_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_PROFILE "LowLevel${CMAKE_PROFILE_POSTFIX}")
	SET(LL_COMPILE_PDB_NAME_RELEASE "LowLevel${CMAKE_RELEASE_POSTFIX}")
ENDIF()

IF(PX_EXPORT_LOWLEVEL_PDB)
	INSTALL(FILES ${PHYSX_ROOT_DIR}/$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>/$<$<CONFIG:debug>:${LL_COMPILE_PDB_NAME_DEBUG}>$<$<CONFIG:checked>:${LL_COMPILE_PDB_NAME_CHECKED}>$<$<CONFIG:profile>:${LL_COMPILE_PDB_NAME_PROFILE}>$<$<CONFIG:release>:${LL_COMPILE_PDB_NAME_RELEASE}>.pdb
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)	
ENDIF()

SET(LOWLEVEL_PLATFORM_LINK_FLAGS "/MAP")
