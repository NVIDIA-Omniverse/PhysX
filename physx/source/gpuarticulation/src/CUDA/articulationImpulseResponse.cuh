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


#ifndef	__ARTI_IMPULSE_RESPONSE_CUH__
#define	__ARTI_IMPULSE_RESPONSE_CUH__

#include "PxgSolverBody.h"
#include "PxSpatialMatrix.h"
#include "PxgArticulation.h"
#include "PxgArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "foundation/PxPreprocessor.h"
#include "solver/PxSolverDefs.h"
#include "MemoryAllocator.cuh"
#include "articulationDynamic.cuh"
#include "DyFeatherstoneArticulation.h"
#include "solverBlock.cuh"

using namespace physx;

static __device__ PX_FORCE_INLINE PxU32 articulationLowestSetBit(ArticulationBitField val)
{
	return val == 0 ? 0 : __ffsll(val) - 1;
}


static __device__ PX_FORCE_INLINE PxU32 articulationHighestSetBit(ArticulationBitField val)
{
	const PxU32 nbZeros = __clzll(val);
	return 63 - nbZeros;
}

static __device__ Cm::UnAlignedSpatialVector propagateAccelerationWNoJVelUpdate(
	const PxgArticulationBlockLinkData& linkData,
	const PxgArticulationBlockDofData* const dofData,
	const PxU32 dofCount,
	const Cm::UnAlignedSpatialVector& hDeltaV,
	const PxU32 threadIndexInWarp)
{
	const float c2px = linkData.mRw_x[threadIndexInWarp];
	const float c2py = linkData.mRw_y[threadIndexInWarp];
	const float c2pz = linkData.mRw_z[threadIndexInWarp];
	Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(PxVec3(-c2px, -c2py, -c2pz), hDeltaV); //parent velocity change

	float3 invStIsT[3];
	PxReal tJointDelta[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector isW[3];
	Cm::UnAlignedSpatialVector motionMatrix[3];

// the split into three separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			isW[ind] = loadSpatialVector(dofData[ind].mIsW, threadIndexInWarp);
			invStIsT[ind] = make_float3(dofData[ind].mInvStIsT_x[threadIndexInWarp], dofData[ind].mInvStIsT_y[threadIndexInWarp], dofData[ind].mInvStIsT_z[threadIndexInWarp]);
			tJointDelta[ind] = dofData[ind].mDeferredQstZ[threadIndexInWarp];
			motionMatrix[ind] = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
		}
	}

#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			tJointDelta[ind] -= isW[ind].innerProduct(pDeltaV);
		}
	}

#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const PxReal jDelta = invStIsT[ind].x * tJointDelta[0] + invStIsT[ind].y * tJointDelta[1]
				+ invStIsT[ind].z * tJointDelta[2];

			pDeltaV += motionMatrix[ind] * jDelta;
		}
	}

	return pDeltaV;
}


static __device__ PX_FORCE_INLINE void averageLinkImpulsesAndPropagate(uint2* PX_RESTRICT isSlabDirty, const Cm::UnAlignedSpatialVector* PX_RESTRICT impulses,
	PxgArticulationBlockData& PX_RESTRICT articulationBlock, PxgArticulationBlockLinkData* PX_RESTRICT artiLinks, 
	PxgArticulationBlockDofData* const PX_RESTRICT artiDofs,
	const PxU32 articulationId, const PxU32 maxLinks, const PxU32 nbArticulations, const PxU32 nbSlabs,
	const PxU32 nbLinks, const PxU32 threadIndexInWarp, PxReal scale = 0.0f)
{
	const PxU32 slabStepSize = maxLinks * nbArticulations;

	if (scale == 0.0f) // When scale is not input, compute the scale here. This method is not momentum-conserving.
	{
		scale = 1.0f;
		if (nbSlabs > 1)
		{
			PxReal count = 0.0f;
			for (PxU32 s = 0, slabDirtyIndex = articulationId; s < nbSlabs; ++s, slabDirtyIndex += nbArticulations)
			{
				const uint2 dirty = isSlabDirty[slabDirtyIndex];
				if (dirty.x != 0xFFFFFFFF || dirty.y != 0xFFFFFFFF)
				{
					count += 1.0f;
				}
			}

			scale = count > 1.0f ? 1.0f / count : 1.0f;
		}
	}

	PxU32 maxIndex = articulationBlock.mLinkWithDeferredImpulse[threadIndexInWarp];
	for (PxU32 s = 0, slabOffset = articulationId, slabDirtyIndex = articulationId; s < nbSlabs; ++s, slabOffset += slabStepSize, slabDirtyIndex += nbArticulations)
	{
		const uint2 dirtyIndex2 = isSlabDirty[slabDirtyIndex];
		for(PxU32 i = 0, dirtyIndex = dirtyIndex2.x; i < 2; ++i, dirtyIndex = dirtyIndex2.y)
		{
			//PxU32 dirtyIndex = i == 0 ? dirtyIndex2.x : dirtyIndex2.y;
			if (dirtyIndex != 0xFFFFFFFF)
			{
				const Cm::UnAlignedSpatialVector preloadedScratchImpulse = loadSpatialVector(artiLinks[dirtyIndex].mScratchImpulse, threadIndexInWarp);

				//Get the index of the dirty slab...
				const PxU32 deltaIdx = slabOffset + dirtyIndex * nbArticulations;

				//Cm::UnAlignedSpatialVector impulse = impulses[deltaIdx] * scale;
				Cm::UnAlignedSpatialVector impulse;

				float4* f4;
				float2* f2;

				size_t ptr = reinterpret_cast<size_t>(&impulses[deltaIdx]);

				if (ptr & 0xf)
				{
					f2 = reinterpret_cast<float2*>(ptr);
					f4 = reinterpret_cast<float4*>(f2 + 1);

					const float2 v0 = *f2;
					const float4 v1 = *f4;

					impulse.top.x = v0.x; impulse.top.y = v0.y; impulse.top.z = v1.x;
					impulse.bottom.x = v1.y; impulse.bottom.y = v1.z; impulse.bottom.z = v1.w;
				}
				else
				{
					f4 = reinterpret_cast<float4*>(ptr);
					f2 = reinterpret_cast<float2*>(f4 + 1);
					const float4 v0 = *f4;
					const float2 v1 = *f2;
					impulse.top.x = v0.x; impulse.top.y = v0.y; impulse.top.z = v0.z;
					impulse.bottom.x = v0.w; impulse.bottom.y = v1.x; impulse.bottom.z = v1.y;
				}

				*f4 = make_float4(0.f);
				*f2 = make_float2(0.f);

				//impulses[deltaIdx] = Cm::UnAlignedSpatialVector::Zero();
				storeSpatialVector(artiLinks[dirtyIndex].mScratchImpulse, preloadedScratchImpulse + impulse*scale, threadIndexInWarp);
				maxIndex = PxMax(dirtyIndex, maxIndex);
			}
		}
		isSlabDirty[slabDirtyIndex] = make_uint2(0xFFFFFFFF, 0xFFFFFFFF);
	}

	{
		const Cm::UnAlignedSpatialVector preloadedRootDeferredZ = loadSpatialVector(articulationBlock.mRootDeferredZ, threadIndexInWarp);

		if (maxIndex)
		{
			PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[artiLinks[maxIndex].mJointOffset[threadIndexInWarp]];

			PxU32 nextParent = artiLinks[maxIndex].mParents[threadIndexInWarp];
			PxU32 nextDofs = artiLinks[maxIndex].mDofs[threadIndexInWarp];
			float nextChild2Parentx = artiLinks[maxIndex].mRw_x[threadIndexInWarp];
			float nextChild2Parenty = artiLinks[maxIndex].mRw_y[threadIndexInWarp];
			float nextChild2Parentz = artiLinks[maxIndex].mRw_z[threadIndexInWarp];
			//Cm::UnAlignedSpatialVector nextDeferredZ = loadSpatialVector(artiLinks[maxIndex].mDeferredZ, threadIndexInWarp);

			for (PxU32 linkID = maxIndex; linkID > 0; linkID--)
			{
				PxgArticulationBlockLinkData& linkData = artiLinks[linkID];

				//Can't preload because this could have been written to
				const Cm::UnAlignedSpatialVector Z = loadSpatialVector(linkData.mScratchImpulse, threadIndexInWarp);
				const Cm::UnAlignedSpatialVector solverSpatialImpulse = loadSpatialVector(linkData.mSolverSpatialImpulse, threadIndexInWarp);

				const PxU32 parent = nextParent;
				const Cm::UnAlignedSpatialVector parentScratchImpulse = loadSpatialVector(artiLinks[parent].mScratchImpulse, threadIndexInWarp);
				const float child2Parentx = nextChild2Parentx;
				const float child2Parenty = nextChild2Parenty;
				const float child2Parentz = nextChild2Parentz;
				const PxU32 dofCount = nextDofs;
				//Cm::UnAlignedSpatialVector deferredZ = nextDeferredZ;

				if (linkID > 1)
				{
					PxU32 nextIndex = linkID - 1;
					nextParent = artiLinks[nextIndex].mParents[threadIndexInWarp];
					nextDofs = artiLinks[nextIndex].mDofs[threadIndexInWarp];
					nextChild2Parentx = artiLinks[nextIndex].mRw_x[threadIndexInWarp];
					nextChild2Parenty = artiLinks[nextIndex].mRw_y[threadIndexInWarp];
					nextChild2Parentz = artiLinks[nextIndex].mRw_z[threadIndexInWarp];
					//nextDeferredZ = loadSpatialVector(artiLinks[nextIndex].mDeferredZ, threadIndexInWarp);
				}

				const Cm::UnAlignedSpatialVector propagatedZ = propagateImpulseW_0(PxVec3(child2Parentx, child2Parenty, child2Parentz),
					dofData, Z,
					dofCount, threadIndexInWarp);

				//Accumulate the solver impulses applied to this link.
				storeSpatialVector(linkData.mSolverSpatialImpulse, solverSpatialImpulse + Z, threadIndexInWarp);

				//KS - we should be able to remove mImpulses once we are 100% certain that we will not have any deferredZ residuals 
				storeSpatialVector(artiLinks[parent].mScratchImpulse, parentScratchImpulse + propagatedZ, threadIndexInWarp);
				storeSpatialVector(linkData.mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);

				dofData -= nextDofs;

			}
		}
		// PT: we can't preload artiLinks[0].mScratchImpulse, as it's modified by the above loop
		const Cm::UnAlignedSpatialVector preloadedRootScratchImpulse = loadSpatialVector(artiLinks[0].mScratchImpulse, threadIndexInWarp);
		storeSpatialVector(artiLinks[0].mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(articulationBlock.mRootDeferredZ, preloadedRootScratchImpulse + preloadedRootDeferredZ, threadIndexInWarp);
	}
}

//This method works out the largest index (furthest down the tree) link that we can propagate to such that we can accumulate
//all impulses acting on the articulation into a single impulse value. We mark this link as dirty and terminate.
//The subsequent kernel requesting velocities uses this cached information to compute the velocity for the desired link.
//It is hoped that spatial locality of contacts in many cases will result in this avoiding doing a brute force propagate
//to all links in the articulation
static __device__ void averageLinkImpulsesAndPropagate2(uint2* PX_RESTRICT isSlabDirty, Cm::UnAlignedSpatialVector* PX_RESTRICT impulses,
	PxgArticulationBlockData& PX_RESTRICT articulationBlock, PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockDofData* const PX_RESTRICT artiDofs,
	const PxU32 articulationId, const PxU32 maxLinks, const PxU32 nbArticulations, const PxU32 nbSlabs,
	const PxU32 nbLinks, const PxU32 threadIndexInWarp, const PxReal scale,
	PxgArticulationBitFieldStackData* PX_RESTRICT pathToRootPerPartition, const PxU32 wordSize,
	const PxU32 commonNode, const PxU32 dirtyFlags)
{

	if (dirtyFlags & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
	{
		const PxU32 slabStepSize = maxLinks * nbArticulations;

		for (PxU32 s = 0, slabOffset = articulationId, slabDirtyIndex = articulationId; s < nbSlabs; ++s, slabOffset += slabStepSize, slabDirtyIndex += nbArticulations)
		{
			const uint2 dirtyIndex2 = isSlabDirty[slabDirtyIndex];
			for (PxU32 i = 0, dirtyIndex = dirtyIndex2.x; i < 2; ++i, dirtyIndex = dirtyIndex2.y)
			{
				//PxU32 dirtyIndex = i == 0 ? dirtyIndex2.x : dirtyIndex2.y;
				if (dirtyIndex != 0xFFFFFFFF)
				{
					const Cm::UnAlignedSpatialVector preloadedScratchImpulse = loadSpatialVector(artiLinks[dirtyIndex].mScratchImpulse, threadIndexInWarp);

					//Get the index of the dirty slab...
					const PxU32 deltaIdx = slabOffset + dirtyIndex * nbArticulations;
					Cm::UnAlignedSpatialVector impulse;

					float4* f4;
					float2* f2;

					size_t ptr = reinterpret_cast<size_t>(&impulses[deltaIdx]);

					if (ptr & 0xf)
					{
						f2 = reinterpret_cast<float2*>(ptr);
						f4 = reinterpret_cast<float4*>(f2 + 1);

						const float2 v0 = *f2;
						const float4 v1 = *f4;

						impulse.top.x = v0.x; impulse.top.y = v0.y; impulse.top.z = v1.x;
						impulse.bottom.x = v1.y; impulse.bottom.y = v1.z; impulse.bottom.z = v1.w;
					}
					else
					{
						f4 = reinterpret_cast<float4*>(ptr);
						f2 = reinterpret_cast<float2*>(f4 + 1);
						const float4 v0 = *f4;
						const float2 v1 = *f2;
						impulse.top.x = v0.x; impulse.top.y = v0.y; impulse.top.z = v0.z;
						impulse.bottom.x = v0.w; impulse.bottom.y = v1.x; impulse.bottom.z = v1.y;
					}

					*f4 = make_float4(0.f);
					*f2 = make_float2(0.f);

					impulse = impulse*scale;

					storeSpatialVector(artiLinks[dirtyIndex].mScratchImpulse, preloadedScratchImpulse + impulse, threadIndexInWarp);
				}
			}
			isSlabDirty[slabDirtyIndex] = make_uint2(0xFFFFFFFF, 0xFFFFFFFF);
		}
	}

	// Traverse up from last to front...
	for (PxI32 j = wordSize-1, bitOffset = (wordSize-1)*64; j >= 0; j--, bitOffset -= 64)
	{
		PxU64 word = pathToRootPerPartition[j].bitField[threadIndexInWarp];
		if (word != 0)
		{
			while (word)
			{
				PxU32 bitIndex = articulationHighestSetBit(word);
				
				const PxU32 index = bitIndex + bitOffset;

				if (index == commonNode)
					break; //We reached the common node so terminate the traversal

				word &= (~(1ull << bitIndex)); //Clear this bit

				PxgArticulationBlockLinkData& linkData = artiLinks[index];

				PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[linkData.mJointOffset[threadIndexInWarp]];

				const PxU32 parent = linkData.mParents[threadIndexInWarp];
				const float child2Parentx = linkData.mRw_x[threadIndexInWarp];
				const float child2Parenty = linkData.mRw_y[threadIndexInWarp];
				const float child2Parentz = linkData.mRw_z[threadIndexInWarp];
				const PxU32 dofCount = linkData.mDofs[threadIndexInWarp];

				const Cm::UnAlignedSpatialVector Z = loadSpatialVector(linkData.mScratchImpulse, threadIndexInWarp);
				const Cm::UnAlignedSpatialVector parentScratchImpulse = loadSpatialVector(artiLinks[parent].mScratchImpulse, threadIndexInWarp);

				const Cm::UnAlignedSpatialVector propagatedZ = propagateImpulseW_0(PxVec3(child2Parentx, child2Parenty, child2Parentz),
					dofData, Z,
					dofCount, threadIndexInWarp);

				//KS - we should be able to remove mImpulses once we are 100% certain that we will not have any deferredZ residuals 
				storeSpatialVector(artiLinks[parent].mScratchImpulse, parentScratchImpulse + propagatedZ, threadIndexInWarp);
				storeSpatialVector(linkData.mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
			}
		}
	}

	//(1) Compute updated link velocity...
	PxSpatialMatrix mat;
	loadSpatialMatrix(artiLinks[commonNode].mSpatialResponseMatrix, threadIndexInWarp, mat);

	const Cm::UnAlignedSpatialVector deltaV = mat * (-loadSpatialVector(artiLinks[commonNode].mScratchImpulse, threadIndexInWarp));
	// It is important that we DO NOT reset the artiLinks[commonNode].mScratchImpulse to zero here because it has not been propagated 
	// to the root (contrary to the subtree of the commonNode, whose impulses we just propagated).
	// The resetting of the artiLinks[commonNode].mScratchImpulse will be done eventually by the averageLinkImpulsesAndPropagate function
	// that goes all the way to the root.

	storeSpatialVector(articulationBlock.mCommonLinkDeltaVelocity, deltaV, threadIndexInWarp);
	articulationBlock.mLinkWithDeferredImpulse[threadIndexInWarp] = commonNode;
}

#endif