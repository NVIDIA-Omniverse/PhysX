// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef __DEFORMABLE_ELEMENT_FILTER_CUH__
#define __DEFORMABLE_ELEMENT_FILTER_CUH__

#include "foundation/PxSimpleTypes.h"
#include "PxFiltering.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgParticleSystem.h"

using namespace physx;

static __device__ bool find(PxParticleRigidFilterPair* pairs, const PxU32 nbPairs, PxParticleRigidFilterPair& pair)
{
	//Binary search...
	PxU32 l = 0, r = nbPairs;

	while (l < r)
	{
		PxU32 mid = (l + r) / 2;
		const PxParticleRigidFilterPair& p = pairs[mid];

		if (pair == p)
			return true;
		else if (pair < p)
			r = mid;
		else
			l = mid + 1;
	}
	return false;
}

static __device__ bool find(PxParticleRigidFilterPair* pairs, const PxU32 nbPairs, PxU64 rigidId, PxU32 particleId)
{
	PxParticleRigidFilterPair pair;
	pair.mID0 = rigidId;
	pair.mID1 = particleId;
	return find(pairs, nbPairs, pair);
}

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

static __device__ bool find(PxgParticleSystem& particleSystem, PxU64 rigidId, PxU64 compressedParticleId)
{
	// this is the ID into the flat uber buffer
	PxU32 particleId = PxGetParticleIndex(compressedParticleId);

	PxU32 id = findRange(particleId, particleSystem.mParticleBufferRunsum, particleSystem.mCommonData.mNumParticleBuffers);
	
	PxgParticleSimBuffer& particleBuffer = particleSystem.mParticleSimBuffers[id];	
	return find(particleBuffer.mFilterPairs, particleBuffer.mNumFilterPairs, rigidId, particleId - particleSystem.mParticleBufferRunsum[id]);
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

#endif // __DEFORMABLE_ELEMENT_FILTER_CUH__