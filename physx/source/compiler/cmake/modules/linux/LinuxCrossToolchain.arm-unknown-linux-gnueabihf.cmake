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

IF(NOT $ENV{LINUX_ROOT} EQUAL "")
	SET(CMAKE_SYSTEM_NAME Linux)

	# FIXME: fix Linux toolchains to support architectures
	SET(LINUX_ROOT $ENV{UE_SDKS_ROOT}/HostWin64/Linux_x64/arm-unknown-linux-gnueabihf_v5_clang-3.5.0-ld-2.23.1-glibc-2.13/toolchain)
	STRING(REGEX REPLACE "\\\\" "/" LINUX_ROOT ${LINUX_ROOT})

	message (STATUS "LINUX_ROOT is '${LINUX_ROOT}'")
	SET(ARCHITECTURE_TRIPLE arm-unknown-linux-gnueabihf)
	SET(CMAKE_SYSTEM_PROCESSOR aarch64)

	SET(CMAKE_CROSSCOMPILING TRUE)
	SET(CMAKE_SYSTEM_NAME Linux)
	SET(CMAKE_SYSTEM_VERSION 1)

	# sysroot
	SET(CMAKE_SYSROOT ${LINUX_ROOT})

	SET(CMAKE_LIBRARY_ARCHITECTURE ${ARCHITECTURE_TRIPLE})

	# specify the cross compiler
	SET(CMAKE_C_COMPILER   ${CMAKE_SYSROOT}/bin/clang.exe)
	SET(CMAKE_C_COMPILER_TARGET ${ARCHITECTURE_TRIPLE})
	SET(CMAKE_C_FLAGS   "-target ${ARCHITECTURE_TRIPLE}  --sysroot ${LINUX_ROOT} ")

	SET(CMAKE_CXX_COMPILER   ${CMAKE_SYSROOT}/bin/clang++.exe)
	SET(CMAKE_CXX_COMPILER_TARGET ${ARCHITECTURE_TRIPLE})
	SET(CMAKE_CXX_FLAGS   "-target ${ARCHITECTURE_TRIPLE} --sysroot ${LINUX_ROOT} ")

	SET(CMAKE_FIND_ROOT_PATH  ${LINUX_ROOT})
	#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)	# hoping to force it to use ar
	#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
	#set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
ELSE()
	MESSAGE("LINUX_ROOT environment variable not defined!")
ENDIF()


