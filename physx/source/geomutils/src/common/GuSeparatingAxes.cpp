// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuSeparatingAxes.h"

using namespace physx;

bool Gu::SeparatingAxes::addAxis(const PxVec3& axis)
{
	PxU32 numAxes = getNumAxes();
	const PxVec3* PX_RESTRICT axes = getAxes();
	const PxVec3* PX_RESTRICT axes_end = axes + numAxes;
	while(axes<axes_end)
	{
		if(PxAbs(axis.dot(*axes))>0.9999f)
			return false;
		axes++;
	}

#ifdef SEP_AXIS_FIXED_MEMORY
	if(mNbAxes<SEP_AXIS_FIXED_MEMORY)
	{
		mAxes[mNbAxes++] = axis;
		return true;
	}

	return false;
#else
	mAxes.pushBack(axis);
	return true;
#endif
}
