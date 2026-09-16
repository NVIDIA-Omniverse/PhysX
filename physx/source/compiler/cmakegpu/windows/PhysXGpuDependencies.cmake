## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build LowLevelShared
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)

SET(GPUDEPENDENCIES_PLATFORM_INCLUDES
)

SET(GPUDEPENDENCIES_PLATFORM_SOURCE
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsAtomic.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsMutex.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsSync.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsThread.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsPrintString.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsSList.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsSocket.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsTime.cpp
	${PHYSX_SOURCE_DIR}/foundation/windows/FdWindowsFPU.cpp
)
SOURCE_GROUP("foundation\\windows" FILES ${GPUDEPENDENCIES_PLATFORM_SOURCE})

SET(GPUDEPENDENCIES_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};${PHYSXGPU_LIBTYPE_DEFS};PX_PHYSX_STATIC_LIB;

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

