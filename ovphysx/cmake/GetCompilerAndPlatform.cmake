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

function(CompilerDumpVersion _OUTPUT_VERSION)

  execute_process(COMMAND ${CMAKE_CXX_COMPILER}
    ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
    OUTPUT_VARIABLE COMPILER_VERSION
  )
  string(REGEX REPLACE "([0-9])\\.([0-9])(\\.[0-9])?" "\\1\\2"
    COMPILER_VERSION ${COMPILER_VERSION})

  set(${_OUTPUT_VERSION} ${COMPILER_VERSION})
endfunction()

function(GetCompiler _ret)
	set(COMPILER_SUFFIX "UNKNOWN")

	if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel"
		OR CMAKE_CXX_COMPILER MATCHES "icl"
		OR CMAKE_CXX_COMPILER MATCHES "icpc")
		if(WIN32)
			set (COMPILER_SUFFIX "iw")
		else()
			set (COMPILER_SUFFIX "il")
		endif()
	elseif (GHSMULTI)
		set(COMPILER_SUFFIX "ghs")
	elseif(MSVC_VERSION GREATER_EQUAL 1930)
		set(COMPILER_SUFFIX "vc143")
	elseif (MSVC_VERSION GREATER_EQUAL 1920)
		set(COMPILER_SUFFIX "vc142")
	elseif (MSVC_VERSION GREATER_EQUAL 1910)
		set(COMPILER_SUFFIX "vc141")
	elseif (MSVC14)
		set(COMPILER_SUFFIX "vc140")
	elseif (MSVC12)
		set(COMPILER_SUFFIX "vc120")
	elseif (MSVC11)
		set(COMPILER_SUFFIX "vc110")
	elseif (MSVC10)
		set(COMPILER_SUFFIX "vc100")
	elseif (MSVC90)
		set(COMPILER_SUFFIX "vc90")
	elseif (MSVC80)
		set(COMPILER_SUFFIX "vc80")
	elseif (MSVC71)
		set(COMPILER_SUFFIX "vc71")
	elseif (MSVC70) # Good luck!
		set(COMPILER_SUFFIX "vc7") # yes, this is correct
	elseif (MSVC60) # Good luck!
		set(COMPILER_SUFFIX "vc6") # yes, this is correct
	elseif (BORLAND)
		set(COMPILER_SUFFIX "bcb")
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "SunPro")
		set(COMPILER_SUFFIX "sw")
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
		set(COMPILER_SUFFIX "xlc")
	elseif (MINGW)
		CompilerDumpVersion(_COMPILER_VERSION)
		set(COMPILER_SUFFIX "mgw${_COMPILER_VERSION}")
	elseif (UNIX)
		if (CMAKE_COMPILER_IS_GNUCXX)
			CompilerDumpVersion(_COMPILER_VERSION)
			if(APPLE)
		      		# on Mac OS X/Darwin is "xgcc".
		      		set(COMPILER_SUFFIX "xgcc${_COMPILER_VERSION}")
			else()
		  		set(COMPILER_SUFFIX "gcc${_COMPILER_VERSION}")
			endif()
		endif()
	else()
		# add clang!
		set(COMPILER_SUFFIX "")
	endif()

	set(${_ret} ${COMPILER_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetStaticCRTString _ret)
	if(NOT TARGET_BUILD_PLATFORM STREQUAL "windows")
		return()
	endif()

	if (NV_USE_STATIC_WINCRT)
		set(CRT_STRING "mt")
	else()
		set(CRT_STRING "md")
	endif()

	set(${_ret} ${CRT_STRING} PARENT_SCOPE)
endfunction()

function (GetPlatformBinName PLATFORM_BIN_NAME LIBPATH_SUFFIX)
	set(RETVAL "UNKNOWN")

	GetCompiler(COMPILER)

	if(TARGET_BUILD_PLATFORM STREQUAL "windows")
		GetStaticCRTString(CRT_STRING)
		set(RETVAL "win.x86_${LIBPATH_SUFFIX}.${COMPILER}.${CRT_STRING}")
	elseif(TARGET_BUILD_PLATFORM STREQUAL "mac")
		set(RETVAL "mac.x86_${LIBPATH_SUFFIX}")
	elseif(TARGET_BUILD_PLATFORM STREQUAL "switch")
		if (${CMAKE_GENERATOR_PLATFORM} STREQUAL "NX32")
			set(RETVAL "switch32")
		elseif (${CMAKE_GENERATOR_PLATFORM} STREQUAL "NX64")
		set(RETVAL "switch64")
		endif()
	elseif(TARGET_BUILD_PLATFORM STREQUAL "linux")
		if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
			set(RETVAL "linux.x86_64")
		elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
			set(RETVAL "linux.aarch64")
		endif()
	endif()

	set(${PLATFORM_BIN_NAME} ${RETVAL} PARENT_SCOPE)
endfunction()
