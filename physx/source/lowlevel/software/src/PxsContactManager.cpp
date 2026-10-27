// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxsContactManager.h"

using namespace physx;

PxsContactManager::PxsContactManager(PxU32 index) : mFlags(0), mCmIndex(index)
{
	// PT: TODO: any reason why we don't initialize all members here, e.g. shapeCore pointers?
	// PT: it might be because of the way we preallocate contact managers in the pipeline, and release the ones
	// we filtered out. Maybe properly initializing everything "for no reason" in that case is costly.
	// Still, it is unclear why we initialize *some* of the members there then.
	mNpUnit.mRigidCore0			= NULL;
	mNpUnit.mRigidCore1			= NULL;
	mNpUnit.mRestDistance		= 0;
	mNpUnit.mFrictionDataPtr	= NULL;
	mNpUnit.mFrictionPatchCount	= 0;

	mNpUnit.mFlags = 0;
	mNpUnit.setDominance0(1u);
	mNpUnit.setDominance1(1u);
}

PxsContactManager::~PxsContactManager()
{
}

void PxsContactManager::setCCD(bool enable)
{
	PxU32 flags = mFlags & (~PXS_CM_CCD_CONTACT);
	if (enable)
		flags |= PXS_CM_CCD_LINEAR;
	else
		flags &= ~PXS_CM_CCD_LINEAR;

	mFlags = flags;
}

