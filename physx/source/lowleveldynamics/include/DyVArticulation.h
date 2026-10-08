// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_V_ARTICULATION_H
#define DY_V_ARTICULATION_H

#include "DyArticulationJointCore.h"

namespace physx
{
	struct PxsBodyCore;

	namespace Dy
	{
		struct Constraint;

		typedef PxU64 ArticulationBitField;

		struct ArticulationLoopConstraint
		{
		public:
			PxU32 linkIndex0;
			PxU32 linkIndex1;
			Dy::Constraint* constraint;
		};

#define DY_ARTICULATION_LINK_NONE 0xffffffff

		struct ArticulationLink
		{
			PxU32					mPathToRootStartIndex;
			PxU32					mChildrenStartIndex;
			PxU16					mPathToRootCount;
			PxU16					mNumChildren;
			PxsBodyCore*			bodyCore;
			ArticulationJointCore*	inboundJoint;
			PxU32					parent;
			PxReal					cfm;

			PX_FORCE_INLINE	void	initBody(PxsBodyCore* core)
			{
				bodyCore				= core;
				mPathToRootStartIndex	= 0;
				mPathToRootCount		= 0;
				mChildrenStartIndex		= 0xffffffff;
				mNumChildren			= 0;
			}

			PX_FORCE_INLINE	void	initJoint(ArticulationJointCore* core, PxU32 parentIndex)
			{
				inboundJoint	= core;
				parent			= parentIndex;
			}
		};
	}
}

#endif
