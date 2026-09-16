// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "DyThresholdTable.h"
#include "foundation/PxUtilities.h"

namespace physx
{
	namespace Dy
	{
		bool ThresholdTable::check(const ThresholdStreamElement* stream, PxU32 streamSize, const PxU32 nodeIndexA, const PxU32 nodeIndexB, PxReal dt)
		{
			PxU32* PX_RESTRICT hashes = mHash;
			PxU32* PX_RESTRICT nextIndices = mNexts;
			Pair* PX_RESTRICT pairs = mPairs;

			/*const PxsRigidBody* b0 = PxMin(body0, body1);
			const PxsRigidBody* b1 = PxMax(body0, body1);*/

			const PxU32 nA = PxMin(nodeIndexA, nodeIndexB);
			const PxU32 nB = PxMax(nodeIndexA, nodeIndexB);

			PxU32 hashKey = computeHashKey(nodeIndexA, nodeIndexB, mHashSize);

			PxU32 pairIndex = hashes[hashKey];
			while(NO_INDEX != pairIndex)
			{
				Pair& pair = pairs[pairIndex];
				const PxU32 thresholdStreamIndex = pair.thresholdStreamIndex;
				PX_ASSERT(thresholdStreamIndex < streamSize);
				PX_UNUSED(streamSize);
				const ThresholdStreamElement& otherElement = stream[thresholdStreamIndex];
				if(otherElement.nodeIndexA.index()==nA && otherElement.nodeIndexB.index()==nB)
					return (pair.accumulatedForce > (otherElement.threshold * dt));
				pairIndex = nextIndices[pairIndex];
			}
			return false;
		}
	}
}
