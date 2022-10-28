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
# Build LowLevelDynamics
#

SET(LOWLEVELDYNAMICS_PLATFORM_INCLUDES
	${PHYSX_SOURCE_DIR}/common/src/windows
	${PHYSX_SOURCE_DIR}/lowlevel/software/include/windows
	${PHYSX_SOURCE_DIR}/lowleveldynamics/include/windows
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline/windows
)

IF(PX_GENERATE_STATIC_LIBRARIES)
	SET(LOWLEVELDYNAMICS_LIBTYPE OBJECT)
ELSE()
	SET(LOWLEVELDYNAMICS_LIBTYPE STATIC)
ENDIF()

# Use generator expressions to set config specific preprocessor definitions
SET(LOWLEVELDYNAMICS_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND LOWLEVELDYNAMICS_LIBTYPE STREQUAL "STATIC")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_DEBUG "LowLevelDynamics_static${CMAKE_DEBUG_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_CHECKED "LowLevelDynamics_static${CMAKE_CHECKED_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_PROFILE "LowLevelDynamics_static${CMAKE_PROFILE_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_RELEASE "LowLevelDynamics_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(LLDYNAMICS_COMPILE_PDB_NAME_DEBUG "LowLevelDynamics${CMAKE_DEBUG_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_CHECKED "LowLevelDynamics${CMAKE_CHECKED_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_PROFILE "LowLevelDynamics${CMAKE_PROFILE_POSTFIX}")
	SET(LLDYNAMICS_COMPILE_PDB_NAME_RELEASE "LowLevelDynamics${CMAKE_RELEASE_POSTFIX}")
ENDIF()

IF(PX_EXPORT_LOWLEVEL_PDB)
	INSTALL(FILES ${PHYSX_ROOT_DIR}/$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>/$<$<CONFIG:debug>:${LLDYNAMICS_COMPILE_PDB_NAME_DEBUG}>$<$<CONFIG:checked>:${LLDYNAMICS_COMPILE_PDB_NAME_CHECKED}>$<$<CONFIG:profile>:${LLDYNAMICS_COMPILE_PDB_NAME_PROFILE}>$<$<CONFIG:release>:${LLDYNAMICS_COMPILE_PDB_NAME_RELEASE}>.pdb
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)	
ENDIF()

