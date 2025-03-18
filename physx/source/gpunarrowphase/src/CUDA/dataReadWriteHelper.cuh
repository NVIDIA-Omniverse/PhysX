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

#ifndef __DATA_RW_HELPER_CUH__
#define __DATA_RW_HELPER_CUH__

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxBounds3.h"
#include "PxgContactManager.h"
#include "AlignedTransform.h"
#include "PxsTransformCache.h"
#include "PxgConvexConvexShape.h"
#include "convexNpCommon.h"

#include "PxgCommonDefines.h"
#include "geometry/PxGeometry.h"
#include "utils.cuh"
#include "PxgSolverCoreDesc.h"
#include "PxgArticulationCoreDesc.h"

using namespace physx;

struct PxgVelocityPackPGS
{
	PxVec3 linVel;
	PxVec3 angVel;
};

struct PxgVelocityPackTGS
{
	PxVec3 linVel;
	PxVec3 angVel;
	PxVec3 linDelta;
	PxVec3 angDelta;
};

union PxgVelocityPack
{
	PxgVelocityPackTGS tgs;
	PxgVelocityPackPGS pgs;

	PX_FORCE_INLINE __device__ PxgVelocityPack() {}
};

struct PxgVelocityReader
{
	//Shared
	const PxU32* const solverBodyIndices;

	//Articulation
	const float4* const PX_RESTRICT artiLinkVelocities;
	const PxU32 maxLinks;
	const PxU32 nbArticulations;
	const PxU32 artiOffset;

	//Rigid
	const PxU32 numSolverBodies;
	const float4* const PX_RESTRICT solverBodyDeltaVel;
	const float4* const PX_RESTRICT initialVel;
	
	template <typename IterativeData>
	PX_FORCE_INLINE __device__ PxgVelocityReader(const PxgPrePrepDesc* preDesc, const PxgSolverCoreDesc* solverCoreDesc, const PxgArticulationCoreDesc* artiCoreDesc,
		const PxgSolverSharedDesc<IterativeData>* sharedDesc, PxU32 numSolverBodies) :
		solverBodyIndices(preDesc->solverBodyIndices),
		artiLinkVelocities(solverCoreDesc->outArtiVelocity),
		maxLinks(artiCoreDesc->mMaxLinksPerArticulation),
		nbArticulations(artiCoreDesc->nbArticulations),
		artiOffset(maxLinks * nbArticulations),
		solverBodyDeltaVel(sharedDesc->iterativeData.solverBodyVelPool + solverCoreDesc->accumulatedBodyDeltaVOffset),
		initialVel(solverCoreDesc->outSolverVelocity),
		numSolverBodies(numSolverBodies)
	{
	}

	template<typename Pack>
	PX_FORCE_INLINE __device__ PxU32 readVelocitiesPGS(const PxNodeIndex rigidId, Pack& result)
	{
		const PxU32 solverBodyIdx = rigidId.isStaticBody() ? 0 : solverBodyIndices[rigidId.index()];

		if (rigidId.isArticulation())
		{
			const PxU32 index = solverBodyIdx * maxLinks;
			const float4* vels = &artiLinkVelocities[index];
			const PxU32 linkID = rigidId.articulationLinkId();

			result.linVel = PxLoad3(vels[linkID]);
			result.angVel = PxLoad3(vels[linkID + artiOffset]);			
		}
		else
		{
			result.linVel = PxLoad3(initialVel[solverBodyIdx]) + PxLoad3(solverBodyDeltaVel[solverBodyIdx]);
			result.angVel = PxLoad3(initialVel[solverBodyIdx + numSolverBodies]) + PxLoad3(solverBodyDeltaVel[solverBodyIdx + numSolverBodies]);			
		}
		return solverBodyIdx;
	}

	PX_FORCE_INLINE __device__ PxU32 readVelocitiesTGS(const PxNodeIndex rigidId, PxgVelocityPackTGS& result)
	{
		const PxU32 solverBodyIdx = rigidId.isStaticBody() ? 0 : solverBodyIndices[rigidId.index()];

		if (rigidId.isArticulation())
		{
			const PxU32 index = solverBodyIdx * maxLinks;
			const float4* vels = &artiLinkVelocities[index];
			const PxU32 linkID = rigidId.articulationLinkId();

			float4 linearVelocity = vels[linkID];
			float4 angularVelocity = vels[linkID + artiOffset];
			float4 linDelta = vels[linkID + artiOffset + artiOffset];
			float4 angDelta = vels[linkID + artiOffset + artiOffset + artiOffset];

			result.linVel = PxVec3(linearVelocity.x, linearVelocity.y, linearVelocity.z);
			result.angVel = PxVec3(angularVelocity.x, angularVelocity.y, angularVelocity.z);
			result.linDelta = PxVec3(linDelta.x, linDelta.y, linDelta.z);
			result.angDelta = PxVec3(angDelta.x, angDelta.y, angDelta.z);						
		}
		else
		{
			float4 vel0 = solverBodyDeltaVel[solverBodyIdx];
			float4 vel1 = solverBodyDeltaVel[solverBodyIdx + numSolverBodies];
			float4 vel2 = solverBodyDeltaVel[solverBodyIdx + numSolverBodies + numSolverBodies];

			result.linVel = PxVec3(vel0.x, vel0.y, vel0.z);
			result.angVel = PxVec3(vel0.w, vel1.x, vel1.y);
			result.linDelta = PxVec3(vel1.z, vel1.w, vel2.x);
			result.angDelta = PxVec3(vel2.y, vel2.z, vel2.w);
		}
		return solverBodyIdx;
	}
};

__device__ inline static
void PxgContactManagerInput_ReadWarp(PxgContactManagerInput & res, const PxgContactManagerInput * ptr, PxU32 offset)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	PxU32 input;
	if (laneIdx < 4)
		input = reinterpret_cast<const PxU32 *>(ptr + offset)[laneIdx];

	res.shapeRef0 = __shfl_sync(FULL_MASK, (input), 0);
	res.shapeRef1 = __shfl_sync(FULL_MASK, (input), 1);
	res.transformCacheRef0 = __shfl_sync(FULL_MASK, (input), 2);
	res.transformCacheRef1 = __shfl_sync(FULL_MASK, (input), 3);
}

__device__ inline static
void PxAlignedTransform_ReadWarp(PxAlignedTransform& res, const PxAlignedTransform* ptr)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	PxReal input;
	if (laneIdx < 8)
		input = reinterpret_cast<const PxReal *>(ptr)[laneIdx];

	res.q.q.x = __shfl_sync(FULL_MASK, input, 0);
	res.q.q.y = __shfl_sync(FULL_MASK, input, 1);
	res.q.q.z = __shfl_sync(FULL_MASK, input, 2);
	res.q.q.w = __shfl_sync(FULL_MASK, input, 3);
	res.p.x = __shfl_sync(FULL_MASK, input, 4);
	res.p.y = __shfl_sync(FULL_MASK, input, 5);
	res.p.z = __shfl_sync(FULL_MASK, input, 6);
	res.p.w = __shfl_sync(FULL_MASK, input, 7);
}

__device__ inline static
void PxsCachedTransform_ReadWarp(PxsCachedTransform& res, const PxsCachedTransform* ptr)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	PxReal input;
	if (laneIdx < sizeof(PxsCachedTransform)/sizeof(PxReal))
		input = reinterpret_cast<const PxReal *>(ptr)[laneIdx];

	res.transform.q.x = __shfl_sync(FULL_MASK, input, 0);
	res.transform.q.y = __shfl_sync(FULL_MASK, input, 1);
	res.transform.q.z = __shfl_sync(FULL_MASK, input, 2);
	res.transform.q.w = __shfl_sync(FULL_MASK, input, 3);
	res.transform.p.x = __shfl_sync(FULL_MASK, input, 4);
	res.transform.p.y = __shfl_sync(FULL_MASK, input, 5);
	res.transform.p.z = __shfl_sync(FULL_MASK, input, 6);
	res.flags = __shfl_sync(FULL_MASK, __float_as_int(input), 7);
}

__device__ inline static
void PxBounds3_ReadWarp(PxBounds3& res, const PxBounds3* ptr)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	PxReal input;
	if (laneIdx < sizeof(PxBounds3) /sizeof(PxReal))
		input = reinterpret_cast<const PxReal *>(ptr)[laneIdx];

	res.minimum.x = __shfl_sync(FULL_MASK, input, 0);
	res.minimum.y = __shfl_sync(FULL_MASK, input, 1);
	res.minimum.z = __shfl_sync(FULL_MASK, input, 2);
	res.maximum.x = __shfl_sync(FULL_MASK, input, 3);
	res.maximum.y = __shfl_sync(FULL_MASK, input, 4);
	res.maximum.z = __shfl_sync(FULL_MASK, input, 5);

}

__device__ inline static
void PxsCachedTransform_WriteWarp(PxsCachedTransform* ptr, const PxsCachedTransform& inp)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	if (laneIdx < 8)
	{
		uint input = reinterpret_cast<const uint *>(&inp)[laneIdx];
		reinterpret_cast<uint *>(ptr)[laneIdx] = input;
	}
}

__device__ inline static
void PxAlignedTransform_WriteWarp(PxAlignedTransform* ptr, const PxAlignedTransform& inp)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	if (laneIdx < 8)
	{
		uint input = reinterpret_cast<const uint *>(&inp)[laneIdx];
		reinterpret_cast<uint *>(ptr)[laneIdx] = input;
	}
}

__device__ inline static
void PxgShape_ReadWarp(PxgShape& res, const PxgShape* ptr)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	PxReal input;
	if (laneIdx < 11 + (sizeof(size_t) == 8) * 1)
	{
		input = reinterpret_cast<const PxReal *>(ptr)[laneIdx];
	}

	res.scale.scale.x = __shfl_sync(FULL_MASK, input, 0);
	res.scale.scale.y = __shfl_sync(FULL_MASK, input, 1);
	res.scale.scale.z = __shfl_sync(FULL_MASK, input, 2);
	res.scale.rotation.x = __shfl_sync(FULL_MASK, input, 3);
	res.scale.rotation.y = __shfl_sync(FULL_MASK, input, 4);
	res.scale.rotation.z = __shfl_sync(FULL_MASK, input, 5);
	res.scale.rotation.w = __shfl_sync(FULL_MASK, input, 6);
	res.materialIndex = __shfl_sync(FULL_MASK, __float_as_uint(input), 7);

	size_t hullOrMeshPtr = __shfl_sync(FULL_MASK, __float_as_uint(input), 8);

	if (sizeof(size_t) == 8)
	{
		const PxU64 hullPtr_hi = __shfl_sync(FULL_MASK, __float_as_uint(input), 9);
		hullOrMeshPtr |= hullPtr_hi << 32ll;
	}
	res.hullOrMeshPtr = hullOrMeshPtr;

	res.type = __shfl_sync(FULL_MASK, __float_as_uint(input), 9 + (sizeof(size_t) == 8) * 1);
	res.particleOrSoftbodyId = __shfl_sync(FULL_MASK, __float_as_uint(input), 10 + (sizeof(size_t) == 8) * 1);
}

/*
 * Loads a PxgShape pair and corresponding transform cache refs.
 * Loads ExpectedType0 into shape0. It is assumed that one of the shapes is actually of ExpectedType0.
 * The function asserts that the shape0 result actually is of type ExpectedType0 and that shape1 is of type
 * ExpectedType1 if it was specified.
 */
template <PxGeometryType::Enum ExpectedType0, PxGeometryType::Enum ExpectedType1 = PxGeometryType::eINVALID>
PX_FORCE_INLINE __device__ void LoadShapePair(
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxU32 cmIdx,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgShape& shape0, //output
	PxU32& transformCacheRef0, //output
	PxgShape& shape1, //output
	PxU32& transformCacheRef1 //output
)
{
	PxgContactManagerInput cmInput = cmInputs[cmIdx];
	const PxgShape* srcShape0 = gpuShapes + cmInput.shapeRef0;
	const PxgShape* srcShape1 = gpuShapes + cmInput.shapeRef1;
	PxU32 srcCacheRef0 = cmInput.transformCacheRef0;
	PxU32 srcCacheRef1 = cmInput.transformCacheRef1;

	if (ExpectedType0 != ExpectedType1)
	{
		PxGeometryType::Enum type0 = PxGeometryType::Enum(srcShape0->type);
		if (type0 != ExpectedType0)
		{
			PxSwap(srcShape0, srcShape1);
			PxSwap(srcCacheRef0, srcCacheRef1);
		}
	}

	shape0 = *srcShape0;
	shape1 = *srcShape1;
	transformCacheRef0 = srcCacheRef0;
	transformCacheRef1 = srcCacheRef1;

	assert(shape0.type == ExpectedType0);
	assert(ExpectedType1 == PxGeometryType::eINVALID || shape1.type == ExpectedType1);
}

/*
* Loads a PxgShape pair and corresponding transform cache refs. Full Warp needs to participate.
* Loads ExpectedType0 into shape0. It is assumed that one of the shapes is actually of ExpectedType0.
* The function asserts that the shape0 result actually is of type ExpectedType0 and that shape1 is of type
* ExpectedType1 if it was specified.
*/
template <PxGeometryType::Enum ExpectedType0, PxGeometryType::Enum ExpectedType1 = PxGeometryType::eINVALID>
PX_FORCE_INLINE __device__ void LoadShapePairWarp(
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxU32 cmIdx,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgShape& shape0, //output
	PxU32& transformCacheRef0, //output
	PxgShape& shape1, //output
	PxU32& transformCacheRef1 //output
)
{
	PxgContactManagerInput cmInput;
	PxgContactManagerInput_ReadWarp(cmInput, cmInputs, cmIdx);
	const PxgShape* srcShape0 = gpuShapes + cmInput.shapeRef0;
	const PxgShape* srcShape1 = gpuShapes + cmInput.shapeRef1;
	PxU32 srcCacheRef0 = cmInput.transformCacheRef0;
	PxU32 srcCacheRef1 = cmInput.transformCacheRef1;

	if (ExpectedType0 != ExpectedType1)
	{
		PxGeometryType::Enum type0 = PxGeometryType::Enum(srcShape0->type);
		if (type0 != ExpectedType0)
		{
			PxSwap(srcShape0, srcShape1);
			PxSwap(srcCacheRef0, srcCacheRef1);
		}
	}

	PxgShape_ReadWarp(shape0, srcShape0);
	PxgShape_ReadWarp(shape1, srcShape1);
	transformCacheRef0 = srcCacheRef0;
	transformCacheRef1 = srcCacheRef1;

	assert(shape0.type == ExpectedType0);
	assert(ExpectedType1 == PxGeometryType::eINVALID || shape1.type == ExpectedType1);
}

__device__ inline static
void ConvexTriNormalAndIndex_WriteWarp(ConvexTriNormalAndIndex* outp, const ConvexTriNormalAndIndex& inp)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	if (laneIdx < 4)
	{
		uint input = reinterpret_cast<const uint *>(&inp)[laneIdx];
		reinterpret_cast<uint *>(outp)[laneIdx] = input;
	}

}

__device__ inline static
void ConvexMeshPair_WriteWarp(ConvexMeshPair* outp, const ConvexMeshPair& inp)
{
	const PxU32 laneIdx = threadIdx.x & (WARP_SIZE - 1);

	if (laneIdx < (sizeof(ConvexMeshPair)/sizeof(uint)))
	{
		uint input = reinterpret_cast<const uint *>(&inp)[laneIdx];
		reinterpret_cast<uint *>(outp)[laneIdx] = input;
	}

}

#endif
