## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

IF(NOT $ENV{PM_PACKAGES_ROOT} EQUAL "")
	# CMake needs this for cross-compiling: the default try_compile() builds an
	# executable, which cannot be linked before the sysroot below is configured.
	# see: https://cmake.org/cmake/help/latest/module/CMakeForceCompiler.html
	# and https://cmake.org/cmake/help/latest/variable/CMAKE_TRY_COMPILE_TARGET_TYPE.html
	SET(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

	SET(LINUX_ROOT $ENV{PM_CLANGCROSSCOMPILE_PATH}/aarch64-unknown-linux-gnueabi)
	STRING(REGEX REPLACE "\\\\" "/" LINUX_ROOT ${LINUX_ROOT})

	MESSAGE(STATUS "LINUX_ROOT is '${LINUX_ROOT}'")
	SET(ARCHITECTURE_TRIPLE aarch64-unknown-linux-gnueabi)
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

	SET(CMAKE_CXX_COMPILER   ${CMAKE_SYSROOT}/bin/clang++.exe)
	SET(CMAKE_CXX_COMPILER_TARGET ${ARCHITECTURE_TRIPLE})

	SET(CMAKE_FIND_ROOT_PATH  ${LINUX_ROOT})
ELSE()
	MESSAGE("PM_PACKAGES_ROOT  variable not defined!")
ENDIF()


