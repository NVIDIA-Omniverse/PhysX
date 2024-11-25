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

IF(NOT DEFINED PHYSX_WINDOWS_COMPILE_DEFS)
	MESSAGE(FATAL ERROR "Snippets uses the PhysX compile defs, and they're not defined when they need to be.")
ENDIF()

IF (NOT DEFINED PHYSX_CXX_FLAGS)
	MESSAGE(FATAL ERROR "Snippets uses the PhysX CXX flags, and they're not defined when they need to be.")
ENDIF()

IF(MSVC_VERSION LESS_EQUAL 1900)
	SET(SNIPPET_WARNING_DISABLES "/wd4714")
ENDIF()

# Get the CXX Flags from the Cached variables set by the PhysX CMakeLists
SET(CMAKE_CXX_FLAGS "${PHYSX_CXX_FLAGS} ${SNIPPET_WARNING_DISABLES} /EHsc")

SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

# Get the C Flags from the Cached variables set by the PhysX CMakeLists
SET(CMAKE_C_FLAGS "${PHYSX_C_FLAGS} ${SNIPPET_WARNING_DISABLES}")

SET(CMAKE_C_FLAGS_DEBUG   ${PHYSX_C_FLAGS_DEBUG})
SET(CMAKE_C_FLAGS_CHECKED ${PHYSX_C_FLAGS_CHECKED})
SET(CMAKE_C_FLAGS_PROFILE ${PHYSX_C_FLAGS_PROFILE})
SET(CMAKE_C_FLAGS_RELEASE ${PHYSX_C_FLAGS_RELEASE})

# Build PDBs for all configurations
SET(CMAKE_SHARED_LINKER_FLAGS "/DEBUG")

SET(SLN_PHYSXDEVICE_PATH "$ENV{PM_PhysXDevice_PATH}/bin/x86/")

# Include ConfigureFileMT to expose that function
INCLUDE(ConfigureFileMT)
	
SET(SNIPPET_RENDER_ENABLED 1)

# Include all of the projects
SET(PLATFORM_SNIPPETS_LIST Convert LoadCollection DelayLoadHook)

IF(PX_BUILDVHACD)
	LIST(APPEND PLATFORM_SNIPPETS_LIST ConvexDecomposition)
ENDIF()
