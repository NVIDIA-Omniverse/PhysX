// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __MANIFOLD_CUH__
#define __MANIFOLD_CUH__

#include "foundation/PxTransform.h"
#include "PxgPersistentContactManifold.h"
#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"

#define MULTIMANIFOLD_MAX_MANIFOLDS	4
#define SUBMANIFOLD_MAX_CONTACTS	6

namespace physx
{

__device__ static inline bool refreshManifolds(
	const PxTransform & aToB,
	PxReal projectBreakingThreshold,
	PxgPersistentContactMultiManifold * PX_RESTRICT multiManifold
	)
{
	const PxU32 threadIdxInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxReal sqProjectBreakingThreshold = projectBreakingThreshold * projectBreakingThreshold;
	const PxU32 numManifolds = multiManifold->mNbManifolds;

	PxU32 i = threadIdxInWarp;
	{
		bool lostContact = false;

		PxU8 curManifoldIdx = i / SUBMANIFOLD_MAX_CONTACTS;		

		// refreshContactPoints
		if (curManifoldIdx < numManifolds)
		{
			PxU32 nbContacts = multiManifold->mNbContacts[curManifoldIdx];
			PxU8 curContactIdx = i % SUBMANIFOLD_MAX_CONTACTS;

			if (curContactIdx < nbContacts)
			{
				PxReal penNew;
				PxgContact & curContact = multiManifold->mContacts[curManifoldIdx][curContactIdx];

				PxVec3 pointA = curContact.pointA;
				PxVec3 pointB = curContact.pointB;

				const PxVec3 localAInB = aToB.transform(pointA); // from a to b
				const PxVec3 v = localAInB - pointB;

				PxVec3 normal = curContact.normal;
				PxReal penOld = curContact.penetration;
				penNew = v.dot(normal);

				const PxVec3 projectedPoint = localAInB - normal * penOld;
				const PxVec3 projectedDifference = pointB - projectedPoint;

				const PxReal distance2d = projectedDifference.magnitudeSquared();

				curContact.penetration = penNew;

				if (distance2d > sqProjectBreakingThreshold)
				{
					lostContact = true;
				}
			}
		}

		// If we lost any contact - we do the full contact gen, we don't need any contacts then
		if (__any_sync(FULL_MASK, (PxU32)lostContact))
		{
			if(threadIdxInWarp == 0)
				multiManifold->mNbManifolds = 0;
			return true;
		}
	}

	return false;
}

__device__  static inline bool invalidateManifold(const PxTransform & aToB, PxgPersistentContactMultiManifold& multiManifold, 
	const PxReal minMargin, const PxReal ratio)
{
	PxAlignedTransform prevRelativeTransform;
	PxAlignedTransform_ReadWarp(prevRelativeTransform, &multiManifold.mRelativeTransform);

	const PxReal thresholdP = minMargin * ratio;
	const PxReal thresholdQ = 0.9998f; //about 1 degree

	const PxVec3 maxTransfPosDelta_delta = aToB.p - PxVec3(prevRelativeTransform.p.x, prevRelativeTransform.p.y, prevRelativeTransform.p.z);
	const PxVec3 maxTransfPosDelta_absDelta(PxAbs(maxTransfPosDelta_delta.x), PxAbs(maxTransfPosDelta_delta.y), PxAbs(maxTransfPosDelta_delta.z));

	const PxReal deltaP = PxMax(maxTransfPosDelta_absDelta.x, PxMax(maxTransfPosDelta_absDelta.y, maxTransfPosDelta_absDelta.z));
	const PxReal deltaQ = aToB.q.dot((PxQuat)prevRelativeTransform.q);

	if ((deltaP > thresholdP) || (deltaQ < thresholdQ))
		return true;

	return false;
}

} // namespace physx

#endif