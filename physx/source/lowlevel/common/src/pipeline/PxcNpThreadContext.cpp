// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxcConstraintBlockStream.h"
#include "PxcNpThreadContext.h"

using namespace physx;

PxcNpThreadContext::PxcNpThreadContext(PxcNpContext* params) : 
	mRenderOutput						(params->mRenderBuffer),
	mContactBlockStream					(params->mNpMemBlockPool),
	mNpCacheStreamPair					(params->mNpMemBlockPool),
	mNarrowPhaseParams					(0.0f, params->mMeshContactMargin, params->mToleranceLength),
	mPCM								(false),
	mContactCache						(false),
	mCreateAveragePoint					(false),
#if PX_ENABLE_SIM_STATS
	mCompressedCacheSize				(0),
	mNbDiscreteContactPairsWithCacheHits(0),
	mNbDiscreteContactPairsWithContacts	(0),
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	mMaxPatches							(0),
	mContactStreamPool					(params->mContactStreamPool),
	mPatchStreamPool					(params->mPatchStreamPool),
	mForceAndIndiceStreamPool			(params->mForceAndIndiceStreamPool),
	mFrictionPatchStreamPool			(params->mFrictionPatchStreamPool),
	mMaterialManager					(params->mMaterialManager),
	mLocalNewTouchCount					(0), 
	mLocalLostTouchCount				(0)
{
#if PX_ENABLE_SIM_STATS
	clearStats();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

PxcNpThreadContext::~PxcNpThreadContext()
{
}

#if PX_ENABLE_SIM_STATS
void PxcNpThreadContext::clearStats()
{
	PxMemSet(mDiscreteContactPairs, 0, sizeof(mDiscreteContactPairs));
	PxMemSet(mModifiedContactPairs, 0, sizeof(mModifiedContactPairs));
	mCompressedCacheSize					= 0;
	mNbDiscreteContactPairsWithCacheHits	= 0;
	mNbDiscreteContactPairsWithContacts		= 0;
}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

void PxcNpThreadContext::reset(PxU32 cmCount)
{
	mContactBlockStream.reset();
	mNpCacheStreamPair.reset();

	mLocalChangeTouch.clear();
	mLocalChangeTouch.resize(cmCount);
	mLocalNewTouchCount = 0;
	mLocalLostTouchCount = 0;
}
