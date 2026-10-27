// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SOLVER_FLAGS_H
#define PXG_SOLVER_FLAGS_H

namespace physx
{
struct PxgSolverContactFlags
{
	enum Enum
	{
		eHAS_FORCE_THRESHOLDS = 1 << 0,

		// This flag enables target velocities being read from the friction anchor contact points.
		// It will get set when contact modification sets a target velocity on contact points.
		eHAS_TARGET_VELOCITY = 1 << 1,

		// This flag disables correlation of contact patches with friction patches from the previous frame
		// and sets the friction constraint bias (geometric error) multiplier to 0.
		//
		// Two scenarios will raise this flag:
		// - strong/sticky friction is disabled
		// - contact modification sets a target velocity on contact points
		eDISABLE_STRONG_FRICTION = 1 << 2,

		eDISABLE_FRICTION = 1 << 3,
		eCOMPLIANT_ACCELERATION_SPRING = 1 << 4,

		eLAST
	};
};
PX_COMPILE_TIME_ASSERT(PxgSolverContactFlags::eLAST <= ((1 << 7) + 1)); // we store these Flags as PxU8

} // namespace physx

#endif