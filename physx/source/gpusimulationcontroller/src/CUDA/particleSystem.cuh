// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __PARTICLE_SYSTEM_CUH__
#define __PARTICLE_SYSTEM_CUH__

#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "utils.cuh"
#include "reduction.cuh"
#include "PxParticleGpu.h"
#include "PxSparseGridParams.h"
#include "PxgParticleSystem.h"

namespace physx
{
	__device__ inline PxVec3 getSubgridDomainSize(const PxSparseGridParams& params)
	{
		const PxReal dx = params.gridSpacing;
		return PxVec3(dx * (params.subgridSizeX - 2 * params.haloSize), dx * (params.subgridSizeY - 2 * params.haloSize), dx * (params.subgridSizeZ - 2 * params.haloSize));
	}

	__device__ inline bool tryFindSubgridHashkey(
		const PxU32* const PX_RESTRICT		sortedHashkey,
		const PxU32							numSubgrids,
		const PxU32							hashToFind,
		PxU32&								result)
	{
		result = binarySearch(sortedHashkey, numSubgrids, hashToFind);
		return sortedHashkey[result] == hashToFind;
	}
	
	__device__ inline float sqr(PxReal x) { return x * x; }

	__device__ inline float W(const PxReal x, const PxReal kSpiky1, const PxReal kInvRadius)
	{
		return kSpiky1 * sqr(1.0f - x * kInvRadius);
	}

	__device__ inline float dWdx(const PxReal x, const PxReal kSpiky2, const PxReal kInvRadius)
	{
		return -kSpiky2 * (1.0f - x * kInvRadius);
	}

	// aerodynamics model in Frozen
	PX_FORCE_INLINE __device__ PxVec3 disneyWindModelEffect(const PxVec3& x0, const PxVec3& x1, const PxVec3& x2, const PxVec3& vel0, const PxVec3& vel1, const PxVec3& vel2, 
		const PxVec3& wind, PxReal inverseMass, PxReal drag, PxReal lift, PxReal dt, PxReal airDensity)
	{
		const PxVec3 x01 = x1 - x0;
		const PxVec3 x02 = x2 - x0;
		PxVec3 n = x01.cross(x02);

		// airDensity: 1.225 kg / m3, reference: https://en.wikipedia.org/wiki/Density_of_air
		const PxVec3 v = (vel0 + vel1 + vel2) * 0.3333f;
		const PxVec3 vrel = wind - v;

		if(vrel.dot(n) < 0.f)
		{
			n *= -1.f;
		}

		// option 1. using current (deformed) triangle area
		const PxReal coef = 0.25f * airDensity * dt * inverseMass;

		//// optoin 2. using rest (undeformed) triangle area
		// const PxReal area = femCloth.mTriangleAreas[triIndex];
		// n.normalize();
		// const PxReal coef = 0.5 * shFEMCloth.mAirDensity * area * dt * inverseMass;

		return coef * ((drag - lift) * vrel.dot(n) * vrel + lift * vrel.magnitudeSquared() * n);
	}

	// finds the bufferIndex for a given UniqueID.
	static __device__ PxU32 findBufferIndexFromUniqueId(const PxgParticleSystem& particleSystem, PxU32 uniqueBufferId)
	{
		const PxU32 length = particleSystem.mCommonData.mNumParticleBuffers;
		if (length == 0)
			return 0;

		const PxU32* values = particleSystem.mParticleBufferSortedUniqueIds;	

		PxU32 l = 0, r = length;
		while (l < r)
		{
			PxU32 m = (l + r) / 2;
			if (values[m] > uniqueBufferId)
				r = m;
			else
				l = m + 1;
		}
		return particleSystem.mParticleBufferSortedUniqueIdsOriginalIndex[r - 1];
	}

}

#endif
