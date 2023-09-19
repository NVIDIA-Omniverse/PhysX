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
## Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

# Define the options up front

OPTION(NV_APPEND_CONFIG_NAME "Append config (DEBUG, CHECKED, PROFILE or '' for release) to outputted binaries." OFF)
OPTION(NV_USE_STATIC_WINCRT "Use the statically linked windows CRT" OFF)
OPTION(NV_USE_DEBUG_WINCRT "Use the debug version of the CRT" OFF)
OPTION(NV_FORCE_64BIT_SUFFIX "Force a 64 bit suffix for platforms that don't register properly." OFF)
OPTION(NV_FORCE_32BIT_SUFFIX "Force a 32 bit suffix for platforms that don't register properly." OFF)

INCLUDE(SetOutputPaths)


IF(NV_FORCE_32BIT_SUFFIX AND NV_FORCE_64BIT_SUFFIX)
    MESSAGE(FATAL_ERROR "Cannot specify both NV_FORCE_64BIT_SUFFIX and NV_FORCE_32BIT_SUFFIX. Choose one.")
ENDIF()

IF(SUPPRESS_SUFFIX)
    MESSAGE("Suppressing binary suffixes.")
    SET(LIBPATH_SUFFIX "NONE")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    SET(EXE_SUFFIX "")
ELSEIF(NV_FORCE_32BIT_SUFFIX)
    MESSAGE("Forcing binary suffixes to 32 bit.")
    SET(LIBPATH_SUFFIX "32")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    SET(EXE_SUFFIX "_32")
ELSEIF(NV_FORCE_64BIT_SUFFIX)
    MESSAGE("Forcing binary suffixes to 64 bit.")
    SET(LIBPATH_SUFFIX "64")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    SET(EXE_SUFFIX "_64")
ELSE()
    # New bitness suffix
    IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
        SET(LIBPATH_SUFFIX "64")
        # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
        SET(EXE_SUFFIX "_64")
    ELSE()
        SET(LIBPATH_SUFFIX "32")
        # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
        SET(EXE_SUFFIX "_32")
    ENDIF()
ENDIF()

IF (NOT DEFINED PX_OUTPUT_LIB_DIR)
    MESSAGE(FATAL_ERROR "When using the GameWorks output structure you must specify PX_OUTPUT_LIB_DIR as the base")
ENDIF()

IF (NOT DEFINED PX_OUTPUT_BIN_DIR)
    MESSAGE(FATAL_ERROR "When using the GameWorks output structure you must specify PX_OUTPUT_BIN_DIR as the base")
ENDIF()

# Set the WINCRT_DEBUG and WINCRT_NDEBUG variables for use in project compile settings
# Really only relevant to windows

SET(DISABLE_ITERATOR_DEBUGGING "/D \"_HAS_ITERATOR_DEBUGGING=0\" /D \"_ITERATOR_DEBUG_LEVEL=0\"")
SET(DISABLE_ITERATOR_DEBUGGING_CUDA "-D_HAS_ITERATOR_DEBUGGING=0 -D_ITERATOR_DEBUG_LEVEL=0")
SET(CRT_DEBUG_FLAG "/D \"_DEBUG\"")
SET(CRT_NDEBUG_FLAG "/D \"NDEBUG\"")

# Need a different format for CUDA
SET(CUDA_DEBUG_FLAG "-DNDEBUG ${DISABLE_ITERATOR_DEBUGGING_CUDA}")
SET(CUDA_NDEBUG_FLAG "-DNDEBUG")

SET(CUDA_CRT_COMPILE_OPTIONS_NDEBUG "")
SET(CUDA_CRT_COMPILE_OPTIONS_DEBUG "")


IF(NV_USE_STATIC_WINCRT)
    SET(WINCRT_NDEBUG "/MT ${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    SET(CUDA_CRT_COMPILE_OPTIONS_NDEBUG "/MT")

    IF (NV_USE_DEBUG_WINCRT)
        SET(CUDA_DEBUG_FLAG "-D_DEBUG")
        SET(WINCRT_DEBUG "/MTd ${CRT_DEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
        SET(CUDA_CRT_COMPILE_OPTIONS_DEBUG "/MTd")
    ELSE()
        SET(WINCRT_DEBUG "/MT ${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
        SET(CUDA_CRT_COMPILE_OPTIONS_DEBUG "/MT")
    ENDIF()
ELSE()
    SET(WINCRT_NDEBUG "/MD ${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}")
    SET(CUDA_CRT_COMPILE_OPTIONS_NDEBUG "/MD")

    IF(NV_USE_DEBUG_WINCRT)
        SET(CUDA_DEBUG_FLAG "-D_DEBUG")
        SET(WINCRT_DEBUG "/MDd ${CRT_DEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
        SET(CUDA_CRT_COMPILE_OPTIONS_DEBUG "/MDd")
    ELSE()
        SET(WINCRT_DEBUG "/MD ${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
        SET(CUDA_CRT_COMPILE_OPTIONS_DEBUG "/MD")
    ENDIF()
ENDIF()

INCLUDE(GetCompilerAndPlatform)

GetPlatformBinName(PLATFORM_BIN_NAME ${LIBPATH_SUFFIX})


SET(PX_ROOT_LIB_DIR "bin/${PLATFORM_BIN_NAME}" CACHE INTERNAL "Relative root of the lib output directory")
SET(PX_ROOT_EXE_DIR "bin/${PLATFORM_BIN_NAME}" CACHE INTERNAL "Relative root dir of the exe output directory")

IF (NOT DEFINED PX_OUTPUT_ARCH)  # platforms with fixed arch like ps4 dont need to have arch defined
    SET(EXE_SUFFIX "")
ENDIF()

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/debug"   )
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_PROFILE "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/profile" )
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_CHECKED "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/checked" )
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/release" )

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/debug"   )
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_PROFILE "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/profile" )
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_CHECKED "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/checked" )
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PX_OUTPUT_LIB_DIR}/${PX_ROOT_LIB_DIR}/release" )

# RFC 108, we're doing EXEs as the special case since there will be presumable be less of those.
SET(PX_EXE_OUTPUT_DIRECTORY_DEBUG 		"${PX_OUTPUT_BIN_DIR}/${PX_ROOT_EXE_DIR}/debug"	  CACHE INTERNAL "Directory to put debug exes in")
SET(PX_EXE_OUTPUT_DIRECTORY_PROFILE  	"${PX_OUTPUT_BIN_DIR}/${PX_ROOT_EXE_DIR}/profile" CACHE INTERNAL "Directory to put profile exes in")
SET(PX_EXE_OUTPUT_DIRECTORY_CHECKED 	"${PX_OUTPUT_BIN_DIR}/${PX_ROOT_EXE_DIR}/checked" CACHE INTERNAL "Directory to put checked exes in")
SET(PX_EXE_OUTPUT_DIRECTORY_RELEASE  	"${PX_OUTPUT_BIN_DIR}/${PX_ROOT_EXE_DIR}/release" CACHE INTERNAL "Directory to put release exes in")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG 	${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}			)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_PROFILE  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_PROFILE}		)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_CHECKED 	${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_CHECKED}  		)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}		)


IF(NV_APPEND_CONFIG_NAME)
	SET(CMAKE_DEBUG_POSTFIX   "DEBUG_${LIBPATH_SUFFIX}")
	SET(CMAKE_PROFILE_POSTFIX "PROFILE_${LIBPATH_SUFFIX}")
	SET(CMAKE_CHECKED_POSTFIX "CHECKED_${LIBPATH_SUFFIX}")
	SET(CMAKE_RELEASE_POSTFIX "_${LIBPATH_SUFFIX}")
ELSE()
	IF (DEFINED PX_OUTPUT_ARCH)  # platforms with fixed arch like ps4 dont need to have arch defined, then dont add bitness
		SET(CMAKE_DEBUG_POSTFIX "_${LIBPATH_SUFFIX}")
		SET(CMAKE_PROFILE_POSTFIX "_${LIBPATH_SUFFIX}")
		SET(CMAKE_CHECKED_POSTFIX "_${LIBPATH_SUFFIX}")
		SET(CMAKE_RELEASE_POSTFIX "_${LIBPATH_SUFFIX}")
	ENDIF()
ENDIF()

# Can no longer just use LIBPATH_SUFFIX since it depends on build type
IF(CMAKE_CL_64)
	SET(RESOURCE_LIBPATH_SUFFIX "x64")
ELSE(CMAKE_CL_64)
	SET(RESOURCE_LIBPATH_SUFFIX "x86")
ENDIF(CMAKE_CL_64)


# removes characters from the version string and leaves just numbers
FUNCTION(StripPackmanVersion IN_VERSION _OUTPUT_VERSION)

  STRING(REGEX REPLACE "([^0-9.])" ""
    OUT_VERSION ${IN_VERSION})

  STRING(REPLACE ".." "."
    OUT_V2 ${OUT_VERSION})

  SET(${_OUTPUT_VERSION} ${OUT_V2} PARENT_SCOPE)
ENDFUNCTION(StripPackmanVersion)