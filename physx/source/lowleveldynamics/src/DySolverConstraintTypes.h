// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONSTRAINT_TYPES_H
#define DY_SOLVER_CONSTRAINT_TYPES_H

#include "foundation/PxSimpleTypes.h"
#include "PxPhysXConfig.h"

namespace physx
{

enum SolverConstraintType
{
	DY_SC_TYPE_NONE = 0,

	DY_SC_TYPE_RB_CONTACT,				// RB-only contact
	DY_SC_TYPE_RB_1D,					// RB-only 1D constraint
	DY_SC_TYPE_EXT_CONTACT,				// contact involving articulations
	DY_SC_TYPE_EXT_1D,					// 1D constraint involving articulations
	DY_SC_TYPE_STATIC_CONTACT,			// RB-only contact where body b is static
	DY_SC_TYPE_BLOCK_RB_CONTACT,
	DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT,
	DY_SC_TYPE_BLOCK_1D,

	DY_SC_CONSTRAINT_TYPE_COUNT	//Count of the number of different constraint types in the solver
};

enum SolverConstraintFlags
{
	DY_SC_FLAG_OUTPUT_FORCE			= (1<<1),
	DY_SC_FLAG_KEEP_BIAS			= (1<<2),
	DY_SC_FLAG_ROT_EQ				= (1<<3),
	DY_SC_FLAG_ORTHO_TARGET			= (1<<4),
	DY_SC_FLAG_SPRING				= (1<<5),
	DY_SC_FLAG_INEQUALITY			= (1<<6),
	DY_SC_FLAG_ACCELERATION_SPRING	= (1<<7)
};

}

#endif
