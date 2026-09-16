// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxPhysXConfig.h"

#ifndef PXG_FRICTION_PATCH_H
#define PXG_FRICTION_PATCH_H

namespace physx
{

struct PxgFrictionPatch	
{
	float4	body0Normal;
	float4	body1Normal;

	float4	body0Anchors[2];
	float4	body1Anchors[2];

	PxU32	anchorCount;
	PxU32	broken;
	PxU32	contactID[2];

	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	operator = (const PxgFrictionPatch& other)
	{
		broken = other.broken;
		anchorCount = other.anchorCount;
		body0Normal = other.body0Normal;
		body1Normal = other.body1Normal;
		body0Anchors[0] = other.body0Anchors[0];   
		body0Anchors[1] = other.body0Anchors[1];
		body1Anchors[0] = other.body1Anchors[0];
		body1Anchors[1] = other.body1Anchors[1];
		contactID[0] = other.contactID[0];
		contactID[1] = other.contactID[1];
	}
};  

PX_COMPILE_TIME_ASSERT(sizeof(PxgFrictionPatch)==112);

struct PxgBlockFrictionPatch
{
	PX_ALIGN(256, float4 body0Normal[32]);
	PX_ALIGN(256, float4 body1Normal[32]);

	PX_ALIGN(128, PxU32 anchorCount[32]);
	PX_ALIGN(128, PxU32 broken[32]);
	PX_ALIGN(128, PxU32 contactID[2][32]);

	PX_ALIGN(256, float4 anchorPoints[2][32]);
	PX_ALIGN(128, PxU32 patchIndex[32]);
};

struct PxgBlockFrictionAnchorPatch
{
	PX_ALIGN(256, float4 body0Anchors[2][32]);	//1024	1024
	PX_ALIGN(256, float4 body1Anchors[2][32]);	//2048	1024
};

struct PxgFrictionAnchorPatch
{
	float4 body0Anchors[2];	
	float4 body1Anchors[2];	
};

struct PxgFrictionPatchGPU
{
	static const PxU32 MAX_ANCHORS = 2;	//!< Patch friction anchor max count
	PxVec3 points[MAX_ANCHORS];			//!< Patch friction anchors points
	PxVec3 impulses[MAX_ANCHORS];		//!< Patch friction impulses at anchors
	PxU32 anchors;						//!< Patch friction anchor count
};

/**
This class is used for friction correlation using the block friction format. The idea is simple - we have an array of these block friction index objects.
These objects contain a pointer to the block patch and the index that this particular constraint is in that structure.
This allows us to allocate individual block friction patches per-block constraint and then index into them.

Advantage - we can use colaesced memory access patterns for the friction patches, broken flags etc. Can remove the need for multiple pointers to friction patches in constraint descs.
Disadvantage - one extra level of indirection to access previous friction patches. No guarantees that accessing previous friction patches won't diverge (in practice,
they should be similar but they could still diverge if new constraints are introduced that change the layout of constraints within a given partition).
*/
struct PxgBlockFrictionIndex
{
	PxU64 mPatchIndex_threadIdxLow;

	PX_CUDA_CALLABLE PxU64 getPatchIndex() const { return mPatchIndex_threadIdxLow >> 5ull; }
	PX_CUDA_CALLABLE PxU32 getThreadIdx() const { return PxU32(mPatchIndex_threadIdxLow&31); }

	PX_CUDA_CALLABLE void createPatchIndex(const PxU32 patchIndex, const PxU32 threadIndexInWarp)
	{
		mPatchIndex_threadIdxLow = (PxU64(patchIndex) << 5ull) | threadIndexInWarp;
	}
};


}

#endif