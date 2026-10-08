// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "CctSweptVolume.h"

using namespace physx;
using namespace Cct;

SweptVolume::SweptVolume()
{
	mType = SweptVolumeType::eLAST;
}

SweptVolume::~SweptVolume()
{
}

void Cct::computeTemporalBox(PxExtendedBounds3& _box, float radius, float height, float contactOffset, float maxJumpHeight, const PxVec3& upDirection, const PxExtendedVec3& center, const PxVec3& direction)
{
	const float r = radius + contactOffset;
	PxVec3 extents(r);
	const float halfHeight = height*0.5f;
	extents.x += fabsf(upDirection.x)*halfHeight;
	extents.y += fabsf(upDirection.y)*halfHeight;
	extents.z += fabsf(upDirection.z)*halfHeight;

	PxExtendedBounds3 box;
	setCenterExtents(box, center, extents);

	{
		PxExtendedBounds3 destBox;
		PxExtendedVec3 tmp = center;
		add(tmp, direction);
		setCenterExtents(destBox, tmp, extents);
		add(box, destBox);
	}

	if(maxJumpHeight!=0.0f)
	{
		PxExtendedBounds3 destBox;
		PxExtendedVec3 tmp = center;
		sub(tmp, upDirection * maxJumpHeight);
		setCenterExtents(destBox, tmp, extents);
		add(box, destBox);
	}

	_box = box;
}
