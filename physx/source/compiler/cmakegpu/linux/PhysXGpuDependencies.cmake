## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build LowLevelShared
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)

SET(GPUDEPENDENCIES_PLATFORM_INCLUDES
)

SET(GPUDEPENDENCIES_PLATFORM_SOURCE
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixAtomic.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixMutex.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixSync.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixThread.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixPrintString.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixSList.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixSocket.cpp
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixTime.cpp	
	${PHYSX_SOURCE_DIR}/foundation/unix/FdUnixFPU.cpp
)
SOURCE_GROUP("foundation\\linux" FILES ${GPUDEPENDENCIES_PLATFORM_SOURCE})

SET(GPUDEPENDENCIES_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_LINUX_COMPILE_DEFS};

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

