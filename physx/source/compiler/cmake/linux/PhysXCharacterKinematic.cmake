## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build PhysXCharacterKinematic
#

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSXCHARACTERKINEMATICS_COMPILE_DEFS 

	# Common to all configurations
	${PHYSX_LINUX_COMPILE_DEFS};

	$<$<CONFIG:debug>:${PHYSX_LINUX_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_LINUX_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_LINUX_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_LINUX_RELEASE_COMPILE_DEFS};>
)

SET(PHYSXCHARACTERKINEMATIC_LIBTYPE STATIC)


# enable -fPIC so we can link static libs with the editor
#SET_TARGET_PROPERTIES(PhysXCharacterKinematic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

