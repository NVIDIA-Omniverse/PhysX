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
## Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

# Define the options up front

option(NV_APPEND_CONFIG_NAME "Append config (DEBUG, CHECKED, PROFILE or '' for release) to outputted binaries." OFF)
option(NV_USE_STATIC_WINCRT "Use the statically linked windows CRT" OFF)
option(NV_USE_DEBUG_WINCRT "Use the debug version of the CRT" OFF)
option(NV_FORCE_64BIT_SUFFIX "Force a 64 bit suffix for platforms that don't register properly." OFF)
option(NV_FORCE_32BIT_SUFFIX "Force a 32 bit suffix for platforms that don't register properly." OFF)

include(SetOutputPaths)


if(NV_FORCE_32BIT_SUFFIX AND NV_FORCE_64BIT_SUFFIX)
    message(FATAL_ERROR "Cannot specify both NV_FORCE_64BIT_SUFFIX and NV_FORCE_32BIT_SUFFIX. Choose one.")
endif()

if(SUPPRESS_SUFFIX)
    message("Suppressing binary suffixes.")
    set(LIBPATH_SUFFIX "NONE")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    set(EXE_SUFFIX "")
elseif(NV_FORCE_32BIT_SUFFIX)
    message("Forcing binary suffixes to 32 bit.")
    set(LIBPATH_SUFFIX "32")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    set(EXE_SUFFIX "_32")
elseif(NV_FORCE_64BIT_SUFFIX)
    message("Forcing binary suffixes to 64 bit.")
    set(LIBPATH_SUFFIX "64")
    # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
    set(EXE_SUFFIX "_64")
else()
    # New bitness suffix
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(LIBPATH_SUFFIX "64")
        # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
        set(EXE_SUFFIX "_64")
    else()
        set(LIBPATH_SUFFIX "32")
        # Set default exe suffix. Unset on platforms that don't need it. Include underscore since it's optional
        set(EXE_SUFFIX "_32")
    endif()
endif()

if (NOT DEFINED PX_OUTPUT_LIB_DIR)
    message(FATAL_ERROR "When using the GameWorks output structure you must specify PX_OUTPUT_LIB_DIR as the base")
endif()

if (NOT DEFINED PX_OUTPUT_BIN_DIR)
    message(FATAL_ERROR "When using the GameWorks output structure you must specify PX_OUTPUT_BIN_DIR as the base")
endif()

# Set the WINCRT_DEBUG and WINCRT_NDEBUG variables for use in project compile settings
# Really only relevant to windows

set(DISABLE_ITERATOR_DEBUGGING "/D \"_HAS_ITERATOR_DEBUGGING=0\" /D \"_ITERATOR_DEBUG_LEVEL=0\"")
set(DISABLE_ITERATOR_DEBUGGING_CUDA "-D_HAS_ITERATOR_DEBUGGING=0 -D_ITERATOR_DEBUG_LEVEL=0")
set(CRT_DEBUG_FLAG "/D \"_DEBUG\"")
set(CRT_NDEBUG_FLAG "/D \"NDEBUG\"")

# Need a different format for CUDA
set(CUDA_DEBUG_FLAG "${DISABLE_ITERATOR_DEBUGGING_CUDA}")
set(CUDA_NDEBUG_FLAG "-DNDEBUG")


if(NV_USE_STATIC_WINCRT)
    set(WINCRT_NDEBUG "${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:debug>:Debug>")
    set(CMAKE_CUDA_RUNTIME_LIBRARY "Static$<$<CONFIG:debug>:Debug>$<$<CONFIG:release>:Release>")

    if (NV_USE_DEBUG_WINCRT)
        set(CUDA_DEBUG_FLAG "-D_DEBUG")
        set(WINCRT_DEBUG "${CRT_DEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    else()
        set(WINCRT_DEBUG "${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    endif()
else()
    set(WINCRT_NDEBUG "${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:debug>:Debug>DLL")
    set(CMAKE_CUDA_RUNTIME_LIBRARY "Shared$<$<CONFIG:debug>:Debug>$<$<CONFIG:release>:Release>")

    if(NV_USE_DEBUG_WINCRT)
        set(CUDA_DEBUG_FLAG "-D_DEBUG")
        set(WINCRT_DEBUG "${CRT_DEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    else()
        set(WINCRT_DEBUG "${DISABLE_ITERATOR_DEBUGGING} ${CRT_NDEBUG_FLAG}" CACHE INTERNAL "Windows CRT build setting")
    endif()
endif()

if (NOT DEFINED PX_OUTPUT_ARCH)  # platforms with fixed arch like ps4 dont need to have arch defined
    set(EXE_SUFFIX "")
endif()

# Build output directories: ${PX_OUTPUT_LIB_DIR}/<config>
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${PX_OUTPUT_LIB_DIR}/debug"   )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_PROFILE "${PX_OUTPUT_LIB_DIR}/profile" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_CHECKED "${PX_OUTPUT_LIB_DIR}/checked" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PX_OUTPUT_LIB_DIR}/release" )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${PX_OUTPUT_LIB_DIR}/debug"   )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_PROFILE "${PX_OUTPUT_LIB_DIR}/profile" )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_CHECKED "${PX_OUTPUT_LIB_DIR}/checked" )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PX_OUTPUT_LIB_DIR}/release" )

set(PX_EXE_OUTPUT_DIRECTORY_DEBUG   "${PX_OUTPUT_BIN_DIR}/debug"   CACHE INTERNAL "Directory to put debug exes in")
set(PX_EXE_OUTPUT_DIRECTORY_PROFILE "${PX_OUTPUT_BIN_DIR}/profile" CACHE INTERNAL "Directory to put profile exes in")
set(PX_EXE_OUTPUT_DIRECTORY_CHECKED "${PX_OUTPUT_BIN_DIR}/checked" CACHE INTERNAL "Directory to put checked exes in")
set(PX_EXE_OUTPUT_DIRECTORY_RELEASE "${PX_OUTPUT_BIN_DIR}/release" CACHE INTERNAL "Directory to put release exes in")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG    ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}   )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_PROFILE  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_PROFILE} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_CHECKED  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_CHECKED} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE} )


if(NV_APPEND_CONFIG_NAME)
	set(CMAKE_DEBUG_POSTFIX   "DEBUG_${LIBPATH_SUFFIX}")
	set(CMAKE_PROFILE_POSTFIX "PROFILE_${LIBPATH_SUFFIX}")
	set(CMAKE_CHECKED_POSTFIX "CHECKED_${LIBPATH_SUFFIX}")
	set(CMAKE_RELEASE_POSTFIX "_${LIBPATH_SUFFIX}")
else()
	if (DEFINED PX_OUTPUT_ARCH)  # platforms with fixed arch like ps4 dont need to have arch defined, then dont add bitness
		set(CMAKE_DEBUG_POSTFIX "_${LIBPATH_SUFFIX}")
		set(CMAKE_PROFILE_POSTFIX "_${LIBPATH_SUFFIX}")
		set(CMAKE_CHECKED_POSTFIX "_${LIBPATH_SUFFIX}")
		set(CMAKE_RELEASE_POSTFIX "_${LIBPATH_SUFFIX}")
	endif()
endif()

# Can no longer just use LIBPATH_SUFFIX since it depends on build type
if(CMAKE_CL_64)
	set(RESOURCE_LIBPATH_SUFFIX "x64")
else(CMAKE_CL_64)
	set(RESOURCE_LIBPATH_SUFFIX "x86")
endif(CMAKE_CL_64)


# removes characters from the version string and leaves just numbers
function(StripPackmanVersion IN_VERSION _OUTPUT_VERSION)

  string(REGEX REPLACE "([^0-9.])" ""
    OUT_VERSION ${IN_VERSION})

  string(REPLACE ".." "."
    OUT_V2 ${OUT_VERSION})

  set(${_OUTPUT_VERSION} ${OUT_V2} PARENT_SCOPE)
endfunction(StripPackmanVersion)
