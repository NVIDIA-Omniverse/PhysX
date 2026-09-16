## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

IF(NOT DEFINED PHYSX_LINUX_COMPILE_DEFS)
	MESSAGE(FATAL ERROR "PVDRuntime uses the PhysX compile defs, and they're not defined when they need to be.")
ENDIF()

IF (NOT DEFINED PHYSX_CXX_FLAGS)
	MESSAGE(FATAL ERROR "PVDRuntime uses the PhysX CXX flags, and they're not defined when they need to be.")
ENDIF()

# Get the CXX Flags from the Cached variables set by the PhysX CMakeLists
SET(CMAKE_CXX_FLAGS "${PHYSX_CXX_FLAGS}")

SET(CMAKE_CXX_FLAGS_DEBUG   "${PHYSX_CXX_FLAGS_DEBUG}")
SET(CMAKE_CXX_FLAGS_CHECKED "${PHYSX_CXX_FLAGS_CHECKED}")
SET(CMAKE_CXX_FLAGS_PROFILE "${PHYSX_CXX_FLAGS_PROFILE}")
SET(CMAKE_CXX_FLAGS_RELEASE "${PHYSX_CXX_FLAGS_RELEASE}")

# Include all of the projects
INCLUDE(PVDRuntime.cmake)

