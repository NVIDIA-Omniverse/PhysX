// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "CctSweptBox.h"
#include "CctCharacterController.h"

using namespace physx;
using namespace Cct;

SweptBox::SweptBox()
{
	mType = SweptVolumeType::eBOX;
}

SweptBox::~SweptBox()
{
}

void SweptBox::computeTemporalBox(const SweepTest& test, PxExtendedBounds3& box, const PxExtendedVec3& center, const PxVec3& direction) const
{
	const float radius = PxMax(mExtents.y, mExtents.z);
	const float height = 2.0f * mExtents.x;
	Cct::computeTemporalBox(box, radius, height, test.mUserParams.mContactOffset, test.mUserParams.mMaxJumpHeight, test.mUserParams.mUpDirection, center, direction);
}
