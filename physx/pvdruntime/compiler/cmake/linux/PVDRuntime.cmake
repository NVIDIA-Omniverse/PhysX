## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build ServerTest template
#


SET(PVDRUNTIME_COMPILE_DEFS

	# Common to all configurations	
	${PHYSX_LINUX_COMPILE_DEFS};${PHYSX_LIBTYPE_DEFS};

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

SET(PVDRUNTIME_PLATFORM_LINKED_LIBS
)
