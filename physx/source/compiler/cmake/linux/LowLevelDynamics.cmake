## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build LowLevelDynamics
#


SET(LOWLEVELDYNAMICS_PLATFORM_INCLUDES
	${PHYSX_SOURCE_DIR}/common/src/linux
	${PHYSX_SOURCE_DIR}/lowlevel/software/include/linux
	${PHYSX_SOURCE_DIR}/lowleveldynamics/include/linux
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline/linux
)

# Use generator expressions to set config specific preprocessor definitions
SET(LOWLEVELDYNAMICS_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_LINUX_COMPILE_DEFS};PX_PHYSX_STATIC_LIB

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

SET(LOWLEVELDYNAMICS_LIBTYPE OBJECT)

