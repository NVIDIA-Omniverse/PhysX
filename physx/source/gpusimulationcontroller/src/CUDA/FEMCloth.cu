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

#include "FEMClothUtil.cuh"
#include "PxgNpKernelIndices.h"
#include "utils.cuh"
#include "atomic.cuh"

using namespace physx;

extern "C" __host__ void initFEMClothKernels1() {}

#define USE_BLOCK_COPY 1

/*******************************************************************************
 *
 * 
 * Global functions
 * 
 * 
 ******************************************************************************/

extern "C" __global__ __launch_bounds__(PxgFEMClothKernelBlockDim::CLOTH_PREINTEGRATION, 1)
void cloth_preIntegrateLaunch(
	PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32* activeIds,
	PxVec3 gravity,
	PxReal dt,
	bool isTGS,
	PxReal* speculativeCCDContactOffset,
	bool externalForcesEveryTgsIterationEnabled,
	bool adaptiveCollisionPairUpdate, 
	PxU8* updateContactPairs,
	PxU32* totalFemContactCounts)
{
	const PxU32 id = activeIds[blockIdx.y];
	PxgFEMCloth& femCloth = femCloths[id];

	// 32 warps in one block
	__shared__ PxReal maxMagnitudeVelSq[32];

	const PxU32 nbVerts = femCloth.mNbVerts;

	float4* PX_RESTRICT velocities = femCloth.mVelocity_InvMass;
	float4* PX_RESTRICT positions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT posDelta = femCloth.mAccumulatedDeltaPos;
	const PxReal velocityDamping = femCloth.mLinearDamping;

	const PxU32 threadIdxInWarp = threadIdx.x;
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalThreadIndex = threadIdxInWarp + WARP_SIZE * warpIndex + blockDim.x * blockDim.y * blockIdx.x;

	if(globalThreadIndex == 0)
	{
		femCloth.mIsActive = false;

		if (blockIdx.y == 0)
		{
			// Reset cloth contact counts if adaptive updates are not used.
			if (!adaptiveCollisionPairUpdate)
			{
				*totalFemContactCounts = 0;
			}

			// For TGS, updateContactPairs is updated in "cloth_stepLaunch"
			*updateContactPairs = static_cast<PxU8>(!isTGS);
		}
	}

	PxReal maxMagVelSq = 0.0f;
	if(globalThreadIndex < nbVerts)
	{
		float4 tPos = positions[globalThreadIndex];
		float4 oldVel = velocities[globalThreadIndex];

		// External forces
		PxVec3 vel = PxVec3(oldVel.x, oldVel.y, oldVel.z);
		if(!externalForcesEveryTgsIterationEnabled && tPos.w != 0.f)
		{
			if(!(femCloth.mActorFlags & PxActorFlag::eDISABLE_GRAVITY))
			{
				vel += gravity * dt;
			}
			PxReal damping = (1.f - PxMin(1.f, velocityDamping * dt));
			vel = vel * damping;
		}

		// Max velocity clamping
		const PxReal maxVel = femCloth.mMaxLinearVelocity;
		const PxReal velMagSq = vel.magnitudeSquared();

		vel = (maxVel != PX_MAX_REAL && maxVel * maxVel < velMagSq) ? maxVel / PxSqrt(velMagSq) * vel : vel;
		maxMagVelSq = vel.magnitudeSquared();

		if(isTGS)
		{
			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);
			posDelta[globalThreadIndex] = make_float4(0.f, 0.f, 0.f, tPos.w);
		}
		else
		{
			float4* PX_RESTRICT prevPositions = femCloth.mPrevPosition_InvMass;
			prevPositions[globalThreadIndex] = tPos;

			const PxVec3 delta = vel * dt;

			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);
			posDelta[globalThreadIndex] = make_float4(delta.x, delta.y, delta.z, tPos.w);

			PxVec3 oldPos(tPos.x, tPos.y, tPos.z);
			PxVec3 pos = oldPos + delta;
			positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);
		}
	}

	// Query the maximum velocity and speculativeCCDContactOffset per cloth.
	if(adaptiveCollisionPairUpdate)
	{
		if(warpIndex == 0) // Initialize shared memory to prevent reading garbage values.
			maxMagnitudeVelSq[threadIdxInWarp] = 0.0f;

		__syncthreads(); // Ensure all threads initialize shared memory

		maxMagnitudeVelSq[warpIndex] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxMagVelSq);

		__syncthreads();

		// Max vertex velocity in a block
		if(warpIndex == 0)
		{
			PxReal maxMagInBlock = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxMagnitudeVelSq[threadIdxInWarp]);

			if (threadIdxInWarp == 0)
			{
				// If externalForcesEveryTgsIterationEnabled is true, also include the effect of gravity to make the bound more conservative.
				maxMagInBlock = externalForcesEveryTgsIterationEnabled ? (PxSqrt(maxMagInBlock) + gravity.magnitude()) * dt
																	   : PxSqrt(maxMagInBlock) * dt;
				AtomicMax(&speculativeCCDContactOffset[id], maxMagInBlock);
			}
		}
	}
}

extern "C" __global__
void cloth_stepLaunch(
	PxgFEMCloth* PX_RESTRICT femCloths, 
	PxU32* activeId, 
	PxReal dt,
	PxVec3 gravity,
	bool externalForcesEveryTgsIterationEnabled,
	bool forceUpdateClothContactPairs,
	PxU8* updateContactPairs)
{
	const PxU32 id = activeId[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];

	const PxU32 nbVerts = femCloth.mNbVerts;

	float4* PX_RESTRICT velocities = femCloth.mVelocity_InvMass;
	float4* PX_RESTRICT positions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT prevPositions = femCloth.mPrevPosition_InvMass;
	float4* PX_RESTRICT posDelta = femCloth.mAccumulatedDeltaPos;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex == 0 && blockIdx.y == 0)
	{
		*updateContactPairs = static_cast<PxU8>(forceUpdateClothContactPairs);
	}

	if(globalThreadIndex < nbVerts)
	{
		float4 tPos = positions[globalThreadIndex];
		float4 tVel = velocities[globalThreadIndex];
		PxVec3 vel = PxVec3(tVel.x, tVel.y, tVel.z);
		PxVec3 oldPos(tPos.x, tPos.y, tPos.z);

		if (externalForcesEveryTgsIterationEnabled)
		{
			if (tPos.w != 0.f) 
			{
				if(!(femCloth.mActorFlags & PxActorFlag::eDISABLE_GRAVITY))
				{
					vel += gravity * dt;
				}
				const PxReal velocityDamping = femCloth.mLinearDamping;
				const PxReal damping = 1.f - PxMin(1.f, velocityDamping * dt);

				vel *= damping;
			}

			// Max velocity clamping
			const PxReal maxVel = femCloth.mMaxLinearVelocity;
			const PxReal velMagSq = vel.magnitudeSquared();
			vel = (maxVel != PX_MAX_REAL && maxVel * maxVel < velMagSq) ? maxVel / PxSqrt(velMagSq) * vel : vel;

			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tVel.w);
		}

		PxVec3 delta = vel * dt;
		PxVec3 pos = oldPos + delta;

		prevPositions[globalThreadIndex] = tPos;
		positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);
		posDelta[globalThreadIndex] += make_float4(delta.x, delta.y, delta.z, 0.f);
	}
}

extern "C" __global__ 
void cloth_rewindLaunch(
	const PxgFEMCloth* PX_RESTRICT femCloths, 
	const PxU32* activeId)
{
	const PxU32 id = activeId[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];

	const PxU32 nbVerts = femCloth.mNbVerts;

	const float4* PX_RESTRICT const prevPositions = femCloth.mPrevPosition_InvMass;
	float4* PX_RESTRICT positions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT accumDelta = femCloth.mAccumulatedDeltaPos;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if(globalThreadIndex < nbVerts)
	{
		const float4 curPos = positions[globalThreadIndex];
		if(curPos.w == 0.0f)
			return;

		const float4 prevPos = prevPositions[globalThreadIndex];
		const float4 deltaPos = curPos - prevPos;

		// Rewind accumulated delta
		accumDelta[globalThreadIndex] -= deltaPos;

		// Rewind position
		positions[globalThreadIndex] = prevPos;
	}
}

extern "C" __global__
void cloth_subStepLaunch(
	const PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32* activeId,
	const PxReal nbCollisionSubstepsInv,
	const PxReal dt)
{
	const PxU32 id = activeId[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];

	const PxU32 nbVerts = femCloth.mNbVerts;

	float4* PX_RESTRICT velocities = femCloth.mVelocity_InvMass;
	float4* PX_RESTRICT positions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT posDelta = femCloth.mAccumulatedDeltaPos;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if(globalThreadIndex < nbVerts)
	{
		const float4 curPos = positions[globalThreadIndex];

		if(curPos.w == 0.0f)
			return;

		float4 vel = velocities[globalThreadIndex];
		vel.w = 0.0f;

		const float4 deltaPos = (nbCollisionSubstepsInv * vel) * dt;

		// Accumulate delta
		posDelta[globalThreadIndex] += deltaPos;

		// Advance position
		positions[globalThreadIndex] = curPos + deltaPos;
	}
}

// each block refits one of the FEM cloth tree bounds, each warp has 32 threads
extern "C" __global__ __launch_bounds__(1024, 1)
void cloth_refitBoundLaunch(
	PxgFEMCloth* gFemClothes,
	const PxU32* activeFemClothes,
	const PxU32 nbActiveFemClothes,
	PxReal* contactDists,
	PxReal* speculativeCCDContactOffset,
	PxBounds3* boundArray,
	const PxU32* elemIndex,
	const PxReal nbCollisionPairUpdatesPerTimestep,
	const PxU8* updateContactPairs)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[FEM_MIDPHASE_SCRATCH_SIZE];

	const PxU32 idx = threadIdx.x;
	const PxU32 warpIndex = threadIdx.y;

	femClothRefitMidphaseScratch* s_warpScratch = reinterpret_cast<femClothRefitMidphaseScratch*>(scratchMem);

	const PxU32 femClothId = activeFemClothes[blockIdx.x];

	PxgFEMCloth& gFemCloth = gFemClothes[femClothId];

	PxU8 * triMeshGeomPtr = reinterpret_cast<PxU8 *>(gFemCloth.mTriMeshData);

	const uint4 nbVerts_nbTriangle_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(triMeshGeomPtr);
	triMeshGeomPtr += sizeof(uint4);

	const PxU32 maxDepth = nbVerts_nbTriangle_maxDepth_nbBv32TreeNodes.z;

	PxBounds3* sPackedNodeBounds = gFemCloth.mPackedNodeBounds;

	if (idx == 0 && warpIndex == 0)
	{
		s_warpScratch->meshVerts = gFemCloth.mPosition_InvMass;
		s_warpScratch->meshVertsIndices = gFemCloth.mTriangleVertexIndices;

		const PxU32 & nbBv32PackedNodes = nbVerts_nbTriangle_maxDepth_nbBv32TreeNodes.w;

		Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(triMeshGeomPtr);
		s_warpScratch->bv32PackedNodes = bv32PackedNodes;
	
		triMeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbBv32PackedNodes;

		Gu::BV32DataDepthInfo* bv32DepthInfo = reinterpret_cast<Gu::BV32DataDepthInfo*>(triMeshGeomPtr);
		s_warpScratch->bv32DepthInfo = bv32DepthInfo;
		triMeshGeomPtr += sizeof(const Gu::BV32DataDepthInfo) * maxDepth;

		PxU32* remapPackedNodeIndex = reinterpret_cast<PxU32*>(triMeshGeomPtr);
		s_warpScratch->bv32RemapPackedNodeIndex = remapPackedNodeIndex;
		triMeshGeomPtr += sizeof(PxU32) * nbBv32PackedNodes;
	}

	__syncthreads();

	// depth buffer will be all the node index
	const Gu::BV32DataDepthInfo* depthInfo = s_warpScratch->bv32DepthInfo;
	const PxU32* remapPackedNodeIndex = s_warpScratch->bv32RemapPackedNodeIndex;

	// each warp to deal with one node
	for (PxU32 i = maxDepth; i > 0; i--)
	{
		const  Gu::BV32DataDepthInfo& info = depthInfo[i - 1];

		const PxU32 offset = info.offset;
		const PxU32 count = info.count;

		for (PxU32 j = warpIndex; j < count; j += SB_REFIT_WAPRS_PER_BLOCK)
		{
			const PxU32 nodeIndex = remapPackedNodeIndex[j + offset];
			Gu::BV32DataPacked& currentNode = const_cast<Gu::BV32DataPacked&>(s_warpScratch->bv32PackedNodes[nodeIndex]);
			const PxU32 nbChildren = currentNode.mNbNodes;

			// compute the bitMask for all the leaf node
			PxU32 resultWarp = __ballot_sync(FULL_MASK, idx < nbChildren && currentNode.isLeaf(idx));
			PxU32 offset = warpScanExclusive(resultWarp, idx);
			PxU32 validCount = __popc(resultWarp);

			PxVec3 min(PX_MAX_F32);
			PxVec3 max(-PX_MAX_F32);

			// using one warp to compute all leaf node's under the same current node's min and max 
			for (PxU32 k = resultWarp; k; k = clearLowestSetBit(k))
			{
				const PxU32 indexInWarp = (k == 0) ? 0 : lowestSetIndex(k);

				// primitive: triangle
				const PxU32 nbPrimitives = currentNode.getNbReferencedPrimitives(indexInWarp);
				const PxU32 startIndex = currentNode.getPrimitiveStartIndex(indexInWarp);
				
				PxVec3 tMin = min;
				PxVec3 tMax = max;

				//each thread in a warp deal with one primitive (maximum 32 primitives)
				const PxU32 primitiveIndex = idx + startIndex;

				if (idx < nbPrimitives)
				{
					const uint4 primitiveIdx = s_warpScratch->meshVertsIndices[primitiveIndex];

					const PxVec3 worldV0 = PxLoad3(s_warpScratch->meshVerts[primitiveIdx.x]);
					const PxVec3 worldV1 = PxLoad3(s_warpScratch->meshVerts[primitiveIdx.y]);
					const PxVec3 worldV2 = PxLoad3(s_warpScratch->meshVerts[primitiveIdx.z]);
					
					PxReal tx = PxMin(worldV0.x, worldV1.x);
					PxReal ty = PxMin(worldV0.y, worldV1.y);
					PxReal tz = PxMin(worldV0.z, worldV1.z);

					tx = PxMin(tx, worldV2.x);
					ty = PxMin(ty, worldV2.y);
					tz = PxMin(tz, worldV2.z);

					tMin.x = PxMin(tMin.x, tx);
					tMin.y = PxMin(tMin.y, ty);
					tMin.z = PxMin(tMin.z, tz);

					tx = PxMax(worldV0.x, worldV1.x);
					ty = PxMax(worldV0.y, worldV1.y);
					tz = PxMax(worldV0.z, worldV1.z);

					tx = PxMax(tx, worldV2.x);
					ty = PxMax(ty, worldV2.y);
					tz = PxMax(tz, worldV2.z);

					tMax.x = PxMax(tMax.x, tx);
					tMax.y = PxMax(tMax.y, ty);
					tMax.z = PxMax(tMax.z, tz);
				}

				min = warpShuffleMin(tMin);
				max = warpShuffleMax(tMax);
			}

			if (idx < nbChildren && !currentNode.isLeaf(idx))
			{
				const PxU32 childOffset = currentNode.getChildOffset(idx);

				min = sPackedNodeBounds[childOffset].minimum;
				max = sPackedNodeBounds[childOffset].maximum;
			}

			if (idx < nbChildren)
			{
				// We already updated the bounds in the previous iterations
				reinterpret_cast<float4&>(currentNode.mMin[idx]) = make_float4(min.x, min.y, min.z, 0.f);
				reinterpret_cast<float4&>(currentNode.mMax[idx]) = make_float4(max.x, max.y, max.z, 0.f);
			}


			sPackedNodeBounds[nodeIndex].minimum = warpShuffleMin(min);
			sPackedNodeBounds[nodeIndex].maximum = warpShuffleMax(max);

		}

		__syncthreads();
	}

	//update the bound phase bound

	if (warpIndex == 0)
	{
		const PxU32 index = elemIndex[femClothId];

		// Expand the bound for broad phase.
		// Unlike soft bodies, the contact distance remains unchanged; otherwise, contact buffers easily overflow for cloth.
		const PxReal contactDist = gFemCloth.mOriginalContactOffset + speculativeCCDContactOffset[femClothId];
		const PxReal broadPhaseExpansion = PxMax(nbCollisionPairUpdatesPerTimestep, 1.0f) * contactDist;
		float* resBound = reinterpret_cast<float*>(&boundArray[index].minimum.x);

		PxBounds3& root = sPackedNodeBounds[0];
		//compute min(0-2) max(3-5)
		if (idx < 6)
		{
			float* r = reinterpret_cast<float*>(&root.minimum.x);
			float value = idx < 3 ? (r[idx] - broadPhaseExpansion) : (r[idx] + broadPhaseExpansion);
			resBound[idx] = value;
		}
	}

}

extern "C" __global__ 
void cloth_averageTrianglePairVertsLaunch(
	PxgFEMCloth* gFEMCloths, 
	const PxU32* activeFEMCloths,	
	const PxReal invDt,
	bool isSharedPartition)
{

#if USE_BLOCK_COPY

	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& femCloth = gFEMCloths[femClothId];

	uint2* sFEMCloth = reinterpret_cast<uint2*>(&femCloth);
	uint2* dFEMCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dFEMCloth, sFEMCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shFEMCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

#else

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& shFEMCloth = gFEMCloths[femClothId];

#endif

	if (!isSharedPartition && !shFEMCloth.mNonSharedTriPair_hasActiveBending)
		return;

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
	const PxU32 nbVerts = shFEMCloth.mNbVerts;

	if(groupThreadIdx < nbVerts)
	{
		float4* PX_RESTRICT positions = shFEMCloth.mPosition_InvMass;
		float4* PX_RESTRICT velocities = shFEMCloth.mVelocity_InvMass;
		float4* PX_RESTRICT accumulatedDeltaPos = shFEMCloth.mAccumulatedDeltaPos;

		float4* PX_RESTRICT triPairPositions = shFEMCloth.mPosition_InvMassCP;
		const PxU32* PX_RESTRICT triPairAccumulatedCopies =
			isSharedPartition ? shFEMCloth.mSharedTriPairAccumulatedCopiesCP : shFEMCloth.mNonSharedTriPairAccumulatedCopiesCP;

		const PxU32 nbTrianglePairs = isSharedPartition ? shFEMCloth.mNbSharedTrianglePairs : shFEMCloth.mNbNonSharedTrianglePairs;
		
		const PxU32 triPairOffset = nbTrianglePairs * 4;
		float4* triPairAccumulatedPoses = &triPairPositions[triPairOffset];

		const float4 pos_invMass = positions[groupThreadIdx];

		if(pos_invMass.w != 0.f && nbTrianglePairs > 0)
		{
			const PxU32 triPairStartInd = groupThreadIdx == 0 ? 0 : triPairAccumulatedCopies[groupThreadIdx - 1];
			const PxU32 triPairEndInd = triPairAccumulatedCopies[groupThreadIdx];
			const PxU32 triPairNbCopies = triPairEndInd - triPairStartInd;

			if (triPairNbCopies > 0)
			{
				float4 triPairDiff = triPairAccumulatedPoses[triPairStartInd] - pos_invMass;

				// Resetting the used triPairAccumulatedPoses by setting its w component to zero.
				triPairAccumulatedPoses[triPairStartInd].w = 0.0f;
				for (PxU32 j = triPairStartInd + 1; j < triPairEndInd; ++j)
				{
					triPairDiff += triPairAccumulatedPoses[j] - pos_invMass;
					triPairAccumulatedPoses[j].w = 0.0f;
				}

				const PxReal triPairScale = 1.f / triPairNbCopies;
				float4 delta = triPairDiff * triPairScale;
				delta.w = 0.0f;

				float4 pos = pos_invMass + delta;
				float4 vel = velocities[groupThreadIdx] + delta * invDt;
				float4 accumDelta = accumulatedDeltaPos[groupThreadIdx] + delta;

				// Max velocity clamping
				const PxReal maxVel = shFEMCloth.mMaxLinearVelocity;
				if (maxVel != PX_MAX_REAL)
				{
					const PxReal dt = 1.0f / invDt;
					const float4 prevPos = shFEMCloth.mPrevPosition_InvMass[groupThreadIdx];
					velocityClamping(pos, vel, accumDelta, maxVel, dt, prevPos);
				}

				positions[groupThreadIdx] = pos;
				velocities[groupThreadIdx] = vel;
				accumulatedDeltaPos[groupThreadIdx] = accumDelta;
			}
		}
	}
}

// Solve triangle constraints in triangle-constraint-only partitions (non-shared triangle partitions).
static __device__ inline void updateNonSharedTriangle(PxgFEMCloth& shFEMCloth,
	const PxsDeformableSurfaceMaterialData* PX_RESTRICT clothMaterials, PxU32 index,
	PxReal dt,
	bool isTGS)
{
	float4* PX_RESTRICT positions = shFEMCloth.mPosition_InvMass;
	float4* PX_RESTRICT velocities = shFEMCloth.mVelocity_InvMass;
	float4* PX_RESTRICT accumulatedDeltaPos = shFEMCloth.mAccumulatedDeltaPos;

	const uint4 vertexIndex = shFEMCloth.mOrderedNonSharedTriangleVertexIndices_triIndex[index];
	const PxU16 globalMaterialIndex = shFEMCloth.mMaterialIndices[vertexIndex.w];
	const PxsDeformableSurfaceMaterialData& material = clothMaterials[globalMaterialIndex];

	float4 x0 = positions[vertexIndex.x];
	float4 x1 = positions[vertexIndex.y];
	float4 x2 = positions[vertexIndex.z];

	const float4 xcp0 = x0;
	const float4 xcp1 = x1;
	const float4 xcp2 = x2;

	const float4& QInv = shFEMCloth.mOrderedNonSharedTriangleRestPoseInv[index];

	membraneEnergySolvePerTriangle(shFEMCloth, x0, x1, x2, dt, material, QInv, 1.0f, 1.0f, 1.0f, index, false, isTGS);

	float4 delta0 = x0 - xcp0;
	float4 delta1 = x1 - xcp1;
	float4 delta2 = x2 - xcp2;

	float4 accumDelta0 = accumulatedDeltaPos[vertexIndex.x];
	float4 accumDelta1 = accumulatedDeltaPos[vertexIndex.y];
	float4 accumDelta2 = accumulatedDeltaPos[vertexIndex.z];

	float4 vel0 = velocities[vertexIndex.x];
	float4 vel1 = velocities[vertexIndex.y];
	float4 vel2 = velocities[vertexIndex.z];

	const float invDt = 1.0f / dt;
	vel0 += delta0 * invDt;
	vel1 += delta1 * invDt;
	vel2 += delta2 * invDt;

	accumDelta0 += delta0;
	accumDelta1 += delta1;
	accumDelta2 += delta2;

	// Max velocity clamping
	const PxReal maxVel = shFEMCloth.mMaxLinearVelocity;
	if (maxVel != PX_MAX_REAL)
	{
		const float4 prevPos0 = shFEMCloth.mPrevPosition_InvMass[vertexIndex.x];
		velocityClamping(x0, vel0, accumDelta0, maxVel, dt, prevPos0);

		const float4 prevPos1 = shFEMCloth.mPrevPosition_InvMass[vertexIndex.y];
		velocityClamping(x1, vel1, accumDelta1, maxVel, dt, prevPos1);

		const float4 prevPos2 = shFEMCloth.mPrevPosition_InvMass[vertexIndex.z];
		velocityClamping(x2, vel2, accumDelta2, maxVel, dt, prevPos2);
	}

	positions[vertexIndex.x] = x0;
	positions[vertexIndex.y] = x1;
	positions[vertexIndex.z] = x2;

	accumulatedDeltaPos[vertexIndex.x] = accumDelta0;
	accumulatedDeltaPos[vertexIndex.y] = accumDelta1;
	accumulatedDeltaPos[vertexIndex.z] = accumDelta2;

	velocities[vertexIndex.x] = vel0;
	velocities[vertexIndex.y] = vel1;
	velocities[vertexIndex.z] = vel2;
}

extern "C" __global__ 
__launch_bounds__(PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 1)
void cloth_solveNonSharedTriangleEnergyLaunch(PxgFEMCloth* PX_RESTRICT gFEMCloths,
	const PxU32* PX_RESTRICT activeFEMCloths,
	const PxsDeformableSurfaceMaterialData* PX_RESTRICT clothMaterials,
	PxReal dt,
	PxU32 partitionId,
	bool isTGS)
{

#if USE_BLOCK_COPY

	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& femCloth = gFEMCloths[femClothId];

	uint2* sFEMCloth = reinterpret_cast<uint2*>(&femCloth);
	uint2* dFEMCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dFEMCloth, sFEMCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shFEMCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

#else

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& shFEMCloth = gFEMCloths[femClothId];

#endif

	if(partitionId < shFEMCloth.mNbNonSharedTriPartitions)
	{
		const PxU32 startInd = partitionId > 0 ? shFEMCloth.mNonSharedTriAccumulatedPartitionsCP[partitionId - 1] : 0;
		const PxU32 endInd = shFEMCloth.mNonSharedTriAccumulatedPartitionsCP[partitionId];

		const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 workIndex = startInd + groupThreadIdx;

		if(workIndex < endInd)
		{
			updateNonSharedTriangle(shFEMCloth, clothMaterials, workIndex, dt, isTGS);
		}
	}
}

// Solving partitions with small elements (< PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL) in a single kernel launch.
// Note: gridDim.x is always 1.
extern "C" __global__
void cloth_solveNonSharedTriangleEnergyClusterLaunch(PxgFEMCloth * PX_RESTRICT gFEMCloths,
	const PxU32 * PX_RESTRICT activeFEMCloths,
	const PxsDeformableSurfaceMaterialData * PX_RESTRICT clothMaterials,
	PxReal dt,
	PxU32 clusterStartId,
	bool isTGS)
{
#if USE_BLOCK_COPY

	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& femCloth = gFEMCloths[femClothId];

	uint2* sFEMCloth = reinterpret_cast<uint2*>(&femCloth);
	uint2* dFEMCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dFEMCloth, sFEMCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shFEMCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

#else

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& shFEMCloth = gFEMCloths[femClothId];

#endif

	for(PxU32 partitionId = clusterStartId; partitionId < shFEMCloth.mNbNonSharedTriPartitions; ++partitionId)
	{
		const PxU32 startInd = partitionId > 0 ? shFEMCloth.mNonSharedTriAccumulatedPartitionsCP[partitionId - 1] : 0;
		const PxU32 endInd = shFEMCloth.mNonSharedTriAccumulatedPartitionsCP[partitionId];

		const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 workIndex = startInd + groupThreadIdx;

		if(workIndex < endInd)
		{
			updateNonSharedTriangle(shFEMCloth, clothMaterials, workIndex, dt, isTGS);
		}

		__syncthreads();
	}
}

// Solve triangle-pair constraints in shared or non-shared triangle pair partitions.
static __device__ inline void updateTrianglePairs(PxgFEMCloth& shFEMCloth, const PxsDeformableSurfaceMaterialData* PX_RESTRICT clothMaterials,
												  PxU32 workIndex, PxReal dt, bool isShared, bool isTGS)
{
	PxU32* PX_RESTRICT writeIndices;
	PxU32* PX_RESTRICT triPairAccumulatedCopies;
	uint4 vertexIndex;
	PxU32 nbTrianglePairs;

	if (isShared)
	{
		writeIndices = shFEMCloth.mSharedTriPairRemapOutputCP;
		triPairAccumulatedCopies = shFEMCloth.mSharedTriPairAccumulatedCopiesCP;
		vertexIndex = shFEMCloth.mOrderedSharedTrianglePairVertexIndices[workIndex];
		nbTrianglePairs = shFEMCloth.mNbSharedTrianglePairs;
	}
	else
	{
		writeIndices = shFEMCloth.mNonSharedTriPairRemapOutputCP;
		triPairAccumulatedCopies = shFEMCloth.mNonSharedTriPairAccumulatedCopiesCP;
		vertexIndex = shFEMCloth.mOrderedNonSharedTrianglePairVertexIndices[workIndex];
		nbTrianglePairs = shFEMCloth.mNbNonSharedTrianglePairs;
	}

	float4* PX_RESTRICT curTriPairPositions = shFEMCloth.mPosition_InvMassCP;
	const float4* PX_RESTRICT positions = shFEMCloth.mPosition_InvMass;

	const PxU32* vertexIndexPtr = reinterpret_cast<const PxU32*>(&vertexIndex.x);

	const PxU32 remapIndex1 = workIndex + nbTrianglePairs;
	const PxU32 remapIndex2 = remapIndex1 + nbTrianglePairs;
	const PxU32 remapIndex3 = remapIndex2 + nbTrianglePairs;

	float4 x0 = curTriPairPositions[workIndex];
	float4 x1 = curTriPairPositions[remapIndex1];
	float4 x2 = curTriPairPositions[remapIndex2];
	float4 x3 = curTriPairPositions[remapIndex3];

	// If the w component of 'curTriPairPositions' is zero, the position values are not yet assigned; use 'positions' for the first assignment.
	// If w is non-zero, reset the used 'curTriPairPositions' by setting its w component to zero.
	if(x0.w == 0.0f)
		x0 = positions[vertexIndex.x];
	else
		curTriPairPositions[workIndex].w = 0.0f; 

	if(x1.w == 0.0f)
		x1 = positions[vertexIndex.y];
	else
		curTriPairPositions[remapIndex1].w = 0.0f;

	if(x2.w == 0.0f)
		x2 = positions[vertexIndex.z];
	else
		curTriPairPositions[remapIndex2].w = 0.0f;

	if(x3.w == 0.0f)
		x3 = positions[vertexIndex.w];
	else
		curTriPairPositions[remapIndex3].w = 0.0f;

	float4 vertexReferenceCounts;
	float* vertexReferenceCountPtr = reinterpret_cast<float*>(&vertexReferenceCounts.x);

#pragma unroll
	for (PxU32 i = 0; i < 4; ++i)
	{
		const PxU32 vi = vertexIndexPtr[i];
		const PxU32 triPairStartInd = vi == 0 ? 0 : triPairAccumulatedCopies[vi - 1];
		const PxU32 triPairEndInd = triPairAccumulatedCopies[vi];
		vertexReferenceCountPtr[i] = static_cast<float>(triPairEndInd - triPairStartInd);
	}

	if (isShared)
	{
		clothSharedEnergySolvePerTrianglePair(shFEMCloth, x0, x1, x2, x3, vertexReferenceCounts, clothMaterials, dt, workIndex, isTGS);
	}
	else
	{
		bendingEnergySolvePerTrianglePair(shFEMCloth, x0, x1, x2, x3, vertexReferenceCounts, dt, workIndex, false, isTGS);
	}

	PxU32 writeIndex0 = writeIndices[workIndex];
	PxU32 writeIndex1 = writeIndices[remapIndex1];
	PxU32 writeIndex2 = writeIndices[remapIndex2];
	PxU32 writeIndex3 = writeIndices[remapIndex3];

	curTriPairPositions[writeIndex0] = x0;
	curTriPairPositions[writeIndex1] = x1;
	curTriPairPositions[writeIndex2] = x2;
	curTriPairPositions[writeIndex3] = x3;
}

extern "C" __global__ 
__launch_bounds__(PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 8)
void cloth_solveTrianglePairEnergyLaunch(
	PxgFEMCloth* PX_RESTRICT gFEMCloths,
	const PxU32* PX_RESTRICT activeFEMCloths, 
	const PxsDeformableSurfaceMaterialData * PX_RESTRICT clothMaterials,
	PxReal dt,
	PxU32 partitionId, 
	bool isSharedPartition,
	bool isTGS)
{
#if USE_BLOCK_COPY

	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& femCloth = gFEMCloths[femClothId];

	uint2* sFEMCloth = reinterpret_cast<uint2*>(&femCloth);
	uint2* dFEMCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dFEMCloth, sFEMCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shFEMCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

#else

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& shFEMCloth = gFEMCloths[femClothId];

#endif

	if (!isSharedPartition && !shFEMCloth.mNonSharedTriPair_hasActiveBending)
		return;

	const PxU32 maxPartitions = isSharedPartition ? shFEMCloth.mNbSharedTriPairPartitions : shFEMCloth.mNbNonSharedTriPairPartitions;

	if(partitionId < maxPartitions)
	{
		const PxU32* const PX_RESTRICT accumulatedPartitions = isSharedPartition ? shFEMCloth.mSharedTriPairAccumulatedPartitionsCP : shFEMCloth.mNonSharedTriPairAccumulatedPartitionsCP;

		const PxU32 startInd = partitionId > 0 ? accumulatedPartitions[partitionId - 1] : 0;
		const PxU32 endInd = accumulatedPartitions[partitionId];

		const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 workIndex = startInd + groupThreadIdx;

		if(workIndex < endInd)
		{
			updateTrianglePairs(shFEMCloth, clothMaterials, workIndex, dt, isSharedPartition, isTGS);
		}
	}
}



extern "C" __global__
void cloth_applyAccumulatedDeltaVelocity(
	PxgFEMCloth * PX_RESTRICT femCloths,
	const PxU32 * activeId)
{
	const PxU32 id = activeId[blockIdx.y];
	PxgFEMCloth& femCloth = femCloths[id];

	float4* PX_RESTRICT velDelta = femCloth.mAccumulatedDeltaVel;
	float4* PX_RESTRICT velocity = femCloth.mVelocity_InvMass;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	const PxU32 nbVertices = femCloth.mNbVerts;

	if (globalThreadIndex < nbVertices)
	{
		PxVec3 deltaVel = PxLoad3(velDelta[globalThreadIndex]);
		PxVec3 vel = PxLoad3(velocity[globalThreadIndex]);		

		PxVec3 sum = vel + deltaVel;

		velocity[globalThreadIndex].x = sum.x;
		velocity[globalThreadIndex].y = sum.y;
		velocity[globalThreadIndex].z = sum.z;
		velDelta[globalThreadIndex] = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
	}
}

extern "C" __global__
void cloth_accumulateInPlaneDampingDeltaVelocity(
	PxgFEMCloth * PX_RESTRICT femCloths,
	const PxU32 * activeId,
	const PxsDeformableSurfaceMaterialData * PX_RESTRICT clothMaterials,
	PxReal dt)
{
	const PxU32 id = activeId[blockIdx.y];
	PxgFEMCloth& femCloth = femCloths[id];

	float4* PX_RESTRICT velDelta = femCloth.mAccumulatedDeltaVel;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	const PxU32 nbTriangles = femCloth.mNbTriangles;

	if (globalThreadIndex < nbTriangles)
	{
		const PxU16 materialIndex = femCloth.mMaterialIndices[globalThreadIndex];
		const PxsDeformableSurfaceMaterialData& material = clothMaterials[materialIndex];

		PxReal dampingFactor = PxClamp(material.elasticityDamping * dt, 0.0f, 1.0f);
		if (dampingFactor == 0.0f)
			return;

		//if (threadIdx.x == 0)
		//	printf("damping: %f\n", dampingFactor);

		uint4 triangle = femCloth.mTriangleVertexIndices[globalThreadIndex];

		PxReal invMassA, invMassB, invMassC;
		PxVec3 pA = PxLoad3(femCloth.mPosition_InvMass[triangle.x], invMassA);
		PxVec3 pB = PxLoad3(femCloth.mPosition_InvMass[triangle.y], invMassB);
		PxVec3 pC = PxLoad3(femCloth.mPosition_InvMass[triangle.z], invMassC);
		
		PxVec3 vA = PxLoad3(femCloth.mVelocity_InvMass[triangle.x]);
		PxVec3 vB = PxLoad3(femCloth.mVelocity_InvMass[triangle.y]);
		PxVec3 vC = PxLoad3(femCloth.mVelocity_InvMass[triangle.z]);
	
		PxVec3 linearVelocity = (1.0f / 3.0f) * (vA + vB + vC);
		PxVec3 centroid = (1.0f / 3.0f) * (pA + pB + pC); //Or use center of mass?

		PxVec3 normal = (pB - pA).cross(pC - pA);
		if (normal.normalize() < 1e-6f)
			return;

		//Only consider velocities in the triangle plane
		PxVec3 vAProj = projectVectorOntoPlane(vA - linearVelocity, normal);
		PxVec3 vBProj = projectVectorOntoPlane(vB - linearVelocity, normal);
		PxVec3 vCProj = projectVectorOntoPlane(vC - linearVelocity, normal);

		PxVec3 rigidVelDirA = normal.cross(pA - centroid);
		PxVec3 rigidVelDirB = normal.cross(pB - centroid);
		PxVec3 rigidVelDirC = normal.cross(pC - centroid);

		PxReal distCenterA = rigidVelDirA.normalize();
		PxReal distCenterB = rigidVelDirB.normalize();
		PxReal distCenterC = rigidVelDirC.normalize();

		PxReal rotVelA = rigidVelDirA.dot(vAProj) / distCenterA;
		PxReal rotVelB = rigidVelDirB.dot(vBProj) / distCenterB;
		PxReal rotVelC = rigidVelDirC.dot(vCProj) / distCenterC;

		PxReal rotVel = (1.0f / 3.0f) * (rotVelA + rotVelB + rotVelC);

		PxVec3 deltaVelA = rotVel * distCenterA * rigidVelDirA - vAProj;
		PxVec3 deltaVelB = rotVel * distCenterB * rigidVelDirB - vBProj;
		PxVec3 deltaVelC = rotVel * distCenterC * rigidVelDirC - vCProj;

		//Mass weights according to formula 33 from the paper "Detailed Rigid Body Simulation with Extended Position Based Dynamics"
		PxReal weightFactor = 1.0f / (invMassA + invMassB + invMassC);

		//deltaVelA *= invMassA * weightFactor * dampingFactor;
		//deltaVelB *= invMassB * weightFactor * dampingFactor;
		//deltaVelC *= invMassC * weightFactor * dampingFactor;

		//Make sure that the sum of the deltas does not produce sideway forces
		PxVec3 deltaAvg = (deltaVelA + deltaVelB + deltaVelC) * (1.0f / 3.0f);
		deltaVelA -= deltaAvg;
		deltaVelB -= deltaAvg;
		deltaVelC -= deltaAvg;
		//Now the following condition should be true: (deltaVelA + deltaVelB + deltaVelC)  == PxVec3(0)
		//PxVec3 debug = (deltaVelA + deltaVelB + deltaVelC);
		//printf("debug: %f %f %f\n", debug.x, debug.y, debug.z);

		deltaVelA *= invMassA * weightFactor * dampingFactor;
		deltaVelB *= invMassB * weightFactor * dampingFactor;
		deltaVelC *= invMassC * weightFactor * dampingFactor;

		//TODO: Apply deltaRotVelDamp in a more balanced way using inverse masses
		if (invMassA != 0.0f)
			AtomicAdd(velDelta, triangle.x, deltaVelA /** dampingFactor * invMassA * weightFactor*/);
		if (invMassB != 0.0f)
			AtomicAdd(velDelta, triangle.y, deltaVelB /** dampingFactor * invMassB * weightFactor*/);
		if (invMassC != 0.0f)
			AtomicAdd(velDelta, triangle.z, deltaVelC /** dampingFactor * invMassC * weightFactor*/);
	}
}

extern "C" __global__
void cloth_accumulateBendingDampingDeltaVelocity(
	PxgFEMCloth * PX_RESTRICT femCloths,
	const PxU32 * activeId,
	PxReal dt)
{
	const PxU32 id = activeId[blockIdx.y];
	PxgFEMCloth& femCloth = femCloths[id];

	float4* PX_RESTRICT velDelta = femCloth.mAccumulatedDeltaVel;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;


	uint4 triangle;
	PxReal bendingDamping;
	PxReal bendingStiffness;
	if (globalThreadIndex < femCloth.mNbTrianglePairs)
	{
		if (globalThreadIndex < femCloth.mNbSharedTrianglePairs)
		{
			PxU32 sharedTriPairIndex = globalThreadIndex;
			triangle = femCloth.mOrderedSharedTrianglePairVertexIndices[sharedTriPairIndex];
			const float4 bendingCoeffs = femCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping[sharedTriPairIndex];
			bendingDamping = bendingCoeffs.z;
			bendingStiffness = bendingCoeffs.y;
		}
		else
		{
			PxU32 nonSharedTriPairIndex = globalThreadIndex - femCloth.mNbSharedTrianglePairs;
			triangle = femCloth.mOrderedNonSharedTrianglePairVertexIndices[nonSharedTriPairIndex];
			const float4 bendingCoeffs = femCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping[nonSharedTriPairIndex];
			bendingDamping = bendingCoeffs.z;
			bendingStiffness = bendingCoeffs.y;
		}

		PxReal dampingFactor = PxClamp(bendingDamping * dt, 0.0f, 1.0f);
		if (dampingFactor == 0.0f || bendingStiffness == 0.0f)
			return;

		//if (threadIdx.x == 0)
		//	printf("bending damping: %f\n", dampingFactor);

		//uint4 triangle = femCloth.mTrianglePairVertexIndices[globalThreadIndex];

		PxVec3 velA = PxLoad3(femCloth.mVelocity_InvMass[triangle.z]);
		PxVec3 velB = PxLoad3(femCloth.mVelocity_InvMass[triangle.w]);
		PxVec3 velC = PxLoad3(femCloth.mVelocity_InvMass[triangle.x]);
		PxVec3 velC2 = PxLoad3(femCloth.mVelocity_InvMass[triangle.y]);

		float invMassA, invMassB, invMassC, invMassC2;
		PxVec3 posA = PxLoad3(femCloth.mPosition_InvMass[triangle.z], invMassA);
		PxVec3 posB = PxLoad3(femCloth.mPosition_InvMass[triangle.w], invMassB);
		PxVec3 posC = PxLoad3(femCloth.mPosition_InvMass[triangle.x], invMassC);
		PxVec3 posC2 = PxLoad3(femCloth.mPosition_InvMass[triangle.y], invMassC2);


		PxVec3 linearVelocity = 0.5f * (velA + velB); //Approximate the linear motion - could also take the average over all 4 velocities

		const PxVec3& start = posA;

		PxVec3 edgeDir = posB - posA;
		if (edgeDir.normalize() < 1e-6f)
			return;

		PxVec3 relVelTip1 = velC - linearVelocity;
		PxVec3 relVelTip2 = velC2 - linearVelocity;

		PxVec3 startToTip1 = posC - start;
		PxVec3 startToTip2 = posC2 - start;

		PxVec3 tip1Dir = edgeDir.cross(startToTip1);
		PxReal distanceToTip1 = tip1Dir.normalize(); //Works because edgeDir has unit length

		PxVec3 tip2Dir = edgeDir.cross(startToTip2);
		PxReal distanceToTip2 = tip2Dir.normalize(); //Works because edgeDir has unit length

		PxReal rotVelTip1 = tip1Dir.dot(relVelTip1) / distanceToTip1;
		PxReal rotVelTip2 = tip2Dir.dot(relVelTip2) / distanceToTip2;

		PxReal deltaRotVel = rotVelTip2 - rotVelTip1;

		PxReal deltaRotVelDamp = deltaRotVel * dampingFactor;

		PxVec3 deltaVelTip1 = deltaRotVelDamp * distanceToTip1 * tip1Dir;
		PxVec3 deltaVelTip2 = -deltaRotVelDamp * distanceToTip2 * tip2Dir;
		PxVec3 deltaVelA = PxVec3(0.0f);
		PxVec3 deltaVelB = PxVec3(0.0f);
		
		//Mass weights according to formula 33 from the paper "Detailed Rigid Body Simulation with Extended Position Based Dynamics"
		PxReal weightFactor = 1.0f / (invMassA + invMassB + invMassC + invMassC2);

		/*deltaVelTip1 *= invMassC * weightFactor;
		deltaVelTip2 *= invMassC2 * weightFactor;
		deltaVelA *= invMassA * weightFactor;
		deltaVelB *= invMassB * weightFactor;*/

		//Make sure that the sum of the deltas does not produce sideway forces
		PxVec3 deltaAvg = (deltaVelA + deltaVelB + deltaVelTip1 + deltaVelTip2) * (0.25f);
		deltaVelA -= deltaAvg;
		deltaVelB -= deltaAvg;
		deltaVelTip1 -= deltaAvg;
		deltaVelTip2 -= deltaAvg;
		//Now the following condition should be true: weightFactor * (deltaVelA  + deltaVelB  + deltaVelTip1 + deltaVelTip2) == PxVec3(0)
		//PxVec3 debug = (deltaVelA + deltaVelB + deltaVelTip1 + deltaVelTip2;
		//printf("debug: %f %f %f\n", debug.x, debug.y, debug.z);

		deltaVelTip1 *= invMassC * weightFactor;
		deltaVelTip2 *= invMassC2 * weightFactor;
		deltaVelA *= invMassA * weightFactor;
		deltaVelB *= invMassB * weightFactor;

		if (invMassA != 0.0f)
			AtomicAdd(velDelta, triangle.z, deltaVelA /** invMassA * weightFactor*/);
		if (invMassB != 0.0f)
			AtomicAdd(velDelta, triangle.w, deltaVelB /** invMassB * weightFactor*/);
		if (invMassC != 0.0f)
			AtomicAdd(velDelta, triangle.x, deltaVelTip1 /** invMassC * weightFactor*/);
		if (invMassC2 != 0.0f)
			AtomicAdd(velDelta, triangle.y, deltaVelTip2 /** invMassC2 * weightFactor*/);
	}
}



extern "C" __global__
void cloth_solveTrianglePairEnergyClusterLaunch(
	PxgFEMCloth * PX_RESTRICT gFEMCloths,
	const PxU32 * PX_RESTRICT activeFEMCloths,
	const PxsDeformableSurfaceMaterialData * PX_RESTRICT clothMaterials,
	PxReal dt,
	PxU32 firstClusterId,
	bool isSharedPartition,
	bool isTGS)
{
#if USE_BLOCK_COPY

	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& femCloth = gFEMCloths[femClothId];

	uint2* sFEMCloth = reinterpret_cast<uint2*>(&femCloth);
	uint2* dFEMCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dFEMCloth, sFEMCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shFEMCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

#else

	const PxU32 femClothId = activeFEMCloths[blockIdx.y];
	PxgFEMCloth& shFEMCloth = gFEMCloths[femClothId];

#endif

	if (!isSharedPartition && !shFEMCloth.mNonSharedTriPair_hasActiveBending)
		return;

	const PxU32 maxPartitions = isSharedPartition ? shFEMCloth.mNbSharedTriPairPartitions : shFEMCloth.mNbNonSharedTriPairPartitions;
	const PxU32* const PX_RESTRICT accumulatedPartitions =
		isSharedPartition ? shFEMCloth.mSharedTriPairAccumulatedPartitionsCP : shFEMCloth.mNonSharedTriPairAccumulatedPartitionsCP;

	for (PxU32 partitionId = firstClusterId; partitionId < maxPartitions; ++partitionId)
	{
		const PxU32 startInd = partitionId > 0 ? accumulatedPartitions[partitionId - 1] : 0;
		const PxU32 endInd = accumulatedPartitions[partitionId];

		const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 workIndex = startInd + groupThreadIdx;

		if (workIndex < endInd)
		{
			updateTrianglePairs(shFEMCloth, clothMaterials, workIndex, dt, isSharedPartition, isTGS);
		}

		__syncthreads();
	}
}

extern "C" __global__ 
void cloth_finalizeVelocitiesLaunch(
	PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32* activeId,
	const PxReal invTotalDt,
	const PxReal dt,
	const bool alwaysRunVelocityAveraging)
{
	__shared__ bool isAwake[PxgFEMClothKernelBlockDim::CLOTH_STEP/32];
	const PxU32 id = activeId[blockIdx.y];
	PxgFEMCloth& femCloth = femCloths[id];

	const PxU32 nbVerts = femCloth.mNbVerts;

	float4* PX_RESTRICT vels = femCloth.mVelocity_InvMass;
	float4* PX_RESTRICT posDelta = femCloth.mAccumulatedDeltaPos;
	float4* PX_RESTRICT prevPosDelta = femCloth.mPrevAccumulatedDeltaPos;
	float4* PX_RESTRICT positions = femCloth.mPosition_InvMass;

	const PxReal sleepDamping = 1.f - PxMin(1.f, femCloth.mSettlingDamping*dt);

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	bool awake = false;

	if(globalThreadIndex < nbVerts)
	{
		const PxReal settleTolerance = femCloth.mSettlingThreshold*dt;
		const PxReal tolerance = femCloth.mSleepThreshold*dt;
		const PxReal sleepDamping = 1.f - PxMin(1.f, femCloth.mSettlingDamping*dt);

		const float4 delta = posDelta[globalThreadIndex];
		PxVec3 tDelta = PxLoad3(delta);
		PxVec3 deltaVel = tDelta / dt;
		float4 pos = positions[globalThreadIndex];

		float4 prevDelta = prevPosDelta[globalThreadIndex];

		// After updating collision pairs, vertex movement is delta - prevDelta.
		// To maintain this difference as delta resets to zero in the next step, prevDelta is adjusted accordingly.
		prevDelta -= delta;
		prevDelta.w = 1.0f; // Mark prevDelta as set.
		prevPosDelta[globalThreadIndex] = prevDelta;

		if (pos.w != 0.0f) // skip kinematic and  infinite mass particles
		{
			PxReal velocityScaling = 1.0f;

			const PxReal magSq = tDelta.magnitudeSquared();
			if (magSq < settleTolerance*settleTolerance)
			{
				awake = magSq >= tolerance * tolerance;

				velocityScaling = sleepDamping;
			}
			else
			{
				awake = true;
			}

			if (alwaysRunVelocityAveraging || velocityScaling != 1.0f)
			{
				float4 vel = vels[globalThreadIndex];
				PxVec3 tVel = PxLoad3(vel);

				PxReal deltaVelMagSqr = deltaVel.magnitudeSquared();
				PxReal velMagSqr = tVel.magnitudeSquared();
				if (alwaysRunVelocityAveraging && deltaVelMagSqr < tVel.magnitudeSquared())
				{
					tVel = tVel * PxSqrt(deltaVelMagSqr / velMagSqr);
				}

				vels[globalThreadIndex] = make_float4(velocityScaling * tVel.x, velocityScaling * tVel.y, velocityScaling * tVel.z, vel.w);
			}
		}
	}

	awake = __any_sync(FULL_MASK, awake);

	if ((threadIdx.x & 31) == 0)
		isAwake[threadIdx.x / 32] = awake;

	__syncthreads();

	if (threadIdx.x < 32)
	{
		awake = isAwake[threadIdx.x];
		awake = __any_sync(FULL_MASK, awake);

		if (awake)
		{
			atomicOr(&femCloth.mIsActive, 1u);
		}
	}
}

extern "C" __global__ void cloth_sleeping(
	PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32 numActiveCloths,
	const PxU32* activeId,
	const PxReal dt,
	const PxReal resetCounter,
	PxReal* wakeCounters,
	PxU32* stateChangedMask)
{
	const PxU32 NumWarps = PxgFEMClothKernelBlockDim::CLOTH_STEP / 32;
	__shared__ PxU32 shMasks[NumWarps];
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	if (globalThreadIdx >= numActiveCloths)
		return;
	
	const PxU32 id = activeId[globalThreadIdx];
	PxgFEMCloth& femCloth = femCloths[id];

	bool reset = femCloth.mIsActive;

	PxReal counter = wakeCounters[id];
	bool wasActive = counter > 0.f;

	if (reset)
		counter = resetCounter;
	else
		counter = PxMax(0.f, counter - dt);

	bool isActive = counter > 0.f;

	PxU32 mask = __ballot_sync(FULL_MASK, isActive^wasActive);

	wakeCounters[id] = counter;
	if ((threadIdx.x & 31) == 0)
		shMasks[threadIdx.x / 32] = mask;

	__syncthreads();

	const PxU32 startIdx = (blockIdx.x*blockDim.x) / 32;
	if (threadIdx.x < NumWarps)
		stateChangedMask[startIdx + threadIdx.x] = shMasks[threadIdx.x];
}

