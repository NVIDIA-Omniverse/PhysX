// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DEFORMABLE_ELEMENT_FILTER_CUH__
#define __DEFORMABLE_ELEMENT_FILTER_CUH__

#include "foundation/PxSimpleTypes.h"
#include "PxFiltering.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgParticleSystem.h"

namespace physx
{

static __device__ bool find(const PxgRigidFilterPair* pairs, const PxU32 nbPairs, PxgRigidFilterPair& pair, bool ignoreIndex2)
{
	//Binary search...
	PxU32 l = 0, r = nbPairs;

	while (l < r)
	{
		PxU32 mid = (l + r) / 2;
		const PxgRigidFilterPair& p = pairs[mid];
		PxI32 cmp = pair.compare(p);

		if (cmp == 0 || (ignoreIndex2 && pair.index0 == p.index0 && pair.index1 == p.index1))
			return true;
		if (cmp < 0)
			r = mid;
		else
			l = mid + 1;

	}
	return false;
}

static __device__ bool find(const PxgRigidFilterPair* pairs, const PxU32 nbPairs, PxU64 rigidId, PxU32 compressedParticleId)
{
	PxgRigidFilterPair pair;
	pair.index0 = rigidId;
	pair.index1 = compressedParticleId;
	pair.index2 = 0;
	return find(pairs, nbPairs, pair, true);
}



static __device__ PxU32 findRange(PxU32 value, PxU32* values, PxU32 length)
{
	if (length == 0)
		return 0;

	PxU32 l = 0, r = length;
	while (l < r)
	{
		PxU32 m = (l + r) / 2;
		if (values[m] > value)
			r = m;
		else
			l = m + 1;

	}
	return r-1;
}

static __device__ bool find(const PxgNonRigidFilterPair* pairs, const PxU32 nbPairs, PxgNonRigidFilterPair& pair, bool ignoreIndex2)
{
	//Binary search...
	PxU32 l = 0, r = nbPairs;

	while (l < r)
	{
		PxU32 mid = (l + r) / 2;
		const PxgNonRigidFilterPair& p = pairs[mid];
		PxI32 cmp = pair.compare(p);

		if (cmp == 0 || (ignoreIndex2 && pair.index0 == p.index0 && pair.index1 == p.index1))
			return true;
		if (cmp < 0)
			r = mid;
		else
			l = mid + 1;

	}
	return false;
}

static __device__ bool find(const PxgNonRigidFilterPair* pairs, const PxU32 nbPairs, const PxU32 compressedId0, const PxU32 compressedId1)
{
	PxgNonRigidFilterPair pair;
	pair.index0 = compressedId0;
	pair.index1 = compressedId1;
	pair.index2 = 0;
	return find(pairs, nbPairs, pair, true);
}

static __device__ bool find(const PxgNonRigidFilterPair* pairs, const PxU32 nbPairs, const PxU32 compressedId0, const PxU32 compressedId1, PxU32 uniqueParticleUserBufferId)
{
	PxgNonRigidFilterPair pair;
	pair.index0 = compressedId0;
	pair.index1 = compressedId1;
	pair.index2 = uniqueParticleUserBufferId;

	return find(pairs, nbPairs, pair, false);
}

static __device__ bool find(const PxgParticleSystem& particleSystem, const PxgNonRigidFilterPair* pairs, const PxU32 nbPairs, const PxU32 compressedParticleId, const PxU32 compressedId1)
{
	PxU32 particleId = PxGetParticleIndex(compressedParticleId);

	PxU32 id = findRange(particleId, particleSystem.mParticleBufferRunsum, particleSystem.mCommonData.mNumParticleBuffers);

	PxgParticleSimBuffer& particleBuffer = particleSystem.mParticleSimBuffers[id];
	return find(pairs, nbPairs, compressedParticleId, compressedId1, particleBuffer.mUniqueId);
}

} // namespace physx

#endif // __DEFORMABLE_ELEMENT_FILTER_CUH__