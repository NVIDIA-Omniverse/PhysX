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

#include "foundation/PxBounds3.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxHeightFieldSample.h"

#include "PxNodeIndex.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCoreKernelIndices.h"

#include "PxsTransformCache.h"

#include <vector_types.h>

#include "PxgCommonDefines.h"
#include "heightfieldUtil.cuh"
#include "dataReadWriteHelper.cuh"
#include "gridCal.cuh"
#include "particleCollision.cuh"

#include "assert.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels12() {}

PX_ALIGN_PREFIX(16)
struct PsHeightfieldScratch
{
	PxTransform heightfieldTransform;
	PxgShape heightfieldShape;

	PxgParticleContactInfo* PX_RESTRICT oneWayContactInfos;
	PxNodeIndex* PX_RESTRICT oneWayContactNodeIndices;
	PxU32* PX_RESTRICT oneWayContactCounts;

	float4* currentPositions;
	float4* predictedPositions;
	PxNodeIndex rigidId;		//heightfield
	PxU32 transformCacheRef;	//heightfield
	PxU32 numParticles;
	PxU32 particleSystemId;
	PxReal contactDist;
	PxReal restDist;
	bool enableCCD;
}PX_ALIGN_SUFFIX(16);


__device__ static inline void psHeightfieldMidphaseCollision(
	PsHeightfieldScratch* sScratch,
	float4* PX_RESTRICT sPerParticleTempContacts,
	const PxU32 numParticles,
	const PxU32 particleIndex,
	const bool isDiffuse,
	const bool isTGS,
	PxgParticleContactWriter& writer
)
{
	const PxReal contactDist = sScratch->contactDist;
	const PxReal restDist = sScratch->restDist;
	
	PxVec3 currentPos = PxLoad3(sScratch->currentPositions[particleIndex]);
	PxVec3 cVolumePos;
	PxReal cVolumeRadius;
	if (sScratch->enableCCD)
	{
		PxVec3 predictedPos = PxLoad3(sScratch->predictedPositions[particleIndex]);
		cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos,
			currentPos, predictedPos, contactDist, isDiffuse, isTGS);
	}
	else
	{
		cVolumePos = currentPos;
		cVolumeRadius = contactDist;
	}

	const PxVec3 cVolumeRadius3(cVolumeRadius);
	const PxVec3 min = cVolumePos - cVolumeRadius3;
	const PxVec3 max = cVolumePos + cVolumeRadius3;
	PxBounds3 worldBound(min, max);

	PxTransform heightfieldTransform = sScratch->heightfieldTransform;

	PxgShape heightfieldShape = sScratch->heightfieldShape;
	const PxReal oneOverHeightScale = 1.f / heightfieldShape.scale.scale.y;
	const PxReal oneOverRowScale = 1.f / PxAbs(heightfieldShape.scale.scale.x);
	const PxReal oneOverlColScale = 1.f / PxAbs(heightfieldShape.scale.scale.z);

	//bound is in world space, we need to transform the bound to the local space of height field
	PxBounds3 localBound = PxBounds3::transformFast(heightfieldTransform.getInverse(), worldBound);

	localBound.minimum.x *= oneOverRowScale;
	localBound.minimum.y *= oneOverHeightScale;
	localBound.minimum.z *= oneOverlColScale;

	localBound.maximum.x *= oneOverRowScale;
	localBound.maximum.y *= oneOverHeightScale;
	localBound.maximum.z *= oneOverlColScale;

	//row scale
	if (heightfieldShape.scale.scale.x < 0.f)
	{
		//swap min and max row scale
		const PxReal temp = localBound.minimum.x;
		localBound.minimum.x = localBound.maximum.x;
		localBound.maximum.x = temp;
	}

	//col scale
	if (heightfieldShape.scale.scale.z < 0.f)
	{
		PxReal swap = localBound.minimum.z;
		localBound.minimum.z = localBound.maximum.z;
		localBound.maximum.z = swap;
	}

	PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightfieldShape.hullOrMeshPtr);
	const PxU32 nbRows = heightfieldData[0];
	const PxU32 nbCols = heightfieldData[1];
	PxHeightFieldSample* samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);

	if ((localBound.minimum.x > nbRows - 1) || (localBound.minimum.z > nbCols - 1)
		|| (localBound.maximum.x < 0) || (localBound.maximum.z < 0))
	{
		return;
	}

	PxU32 minRow = getMinRow(localBound.minimum.x, nbRows);
	PxU32 maxRow = getMaxRow(localBound.maximum.x, nbRows);
	PxU32 minColumn = getMinColumn(localBound.minimum.z, nbCols);
	PxU32 maxColumn = getMaxColumn(localBound.maximum.z, nbCols);

	if ((2 * (maxColumn - minColumn) * (maxRow - minRow)) == 0)
	{
		return;
	}

	const PxVec3 cVolumePosShapeSpace = heightfieldTransform.transformInv(cVolumePos);
	const PxVec3 particlePosShapeSpace = heightfieldTransform.transformInv(currentPos);

	const PxReal miny = localBound.minimum.y;
	const PxReal maxy = localBound.maximum.y;

	const PxU32 columnSpan = maxColumn - minColumn;

	//How many static contacts this particle already has
	PxU32 contactCounts = 0;

	//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
	const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;

	for (PxU32 i = 0; i < totalNumProcessed; ++i)
	{
		PxU32 triangleIdx = 0xFFffFFff;

		const PxU32 index = i / 2;
		const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
		assert(isValidVertex(vertexIndex, nbRows, nbCols));
		PxReal h0 = getHeight(vertexIndex, samples);
		PxReal h1 = getHeight(vertexIndex + 1, samples);
		PxReal h2 = getHeight(vertexIndex + nbCols, samples);
		PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
		const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
		const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;

		if (!(con0 || con1))
		{
			const PxHeightFieldSample& sample = getSample(vertexIndex, samples);

			const bool isMaterial1 = (i & 1) ? 1 : 0;
			PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
			if (material != PxHeightFieldMaterial::eHOLE)
			{
				triangleIdx = isMaterial1 ? ((vertexIndex << 1) + 1) : (vertexIndex << 1);

				PxVec3 triV0, triV1, triV2;
				getTriangle(triV0, triV1, triV2, NULL, triangleIdx, heightfieldShape.scale, nbRows, nbCols, samples);
				
				PxVec3 normalShapeSpace;
				PxReal distance;
				bool intersect = particleTriangleTest(normalShapeSpace, distance,
					particlePosShapeSpace, cVolumePosShapeSpace, cVolumeRadius, triV0, triV1, triV2, sScratch->enableCCD);
				
				//this code still misses a logic from the triangle mesh near phase, which also selects the best contact
				//if there are more than MaxStaticContactsPerMesh
				if (intersect && contactCounts < PxgParticleContactInfo::MaxStaticContactsPerMesh)
				{
					PxVec3 normal = heightfieldTransform.rotate(normalShapeSpace);
					sPerParticleTempContacts[contactCounts++] = make_float4(normal.x, normal.y, normal.z, distance - restDist);
				}
			}
		}
	}//end of totalNumProcessed

	contactCounts = PxMin(contactCounts, PxgParticleContactInfo::MaxStaticContactsPerMesh);
	PxU32 contactWritten = 0;
	if (sScratch->rigidId.isStaticBody())
	{
		if (contactCounts)
		{
			PxU32 contactIndex = (PxU32)(atomicAdd((PxI32*)&sScratch->oneWayContactCounts[particleIndex], (PxI32)contactCounts));
			PxU32 endIndex = PxMin(contactIndex + contactCounts, PxgParticleContactInfo::MaxStaticContactsPerParticle);
			for (PxU32 i = contactIndex, src = 0; i < endIndex; i++, src++)
			{
				const PxU32 outputIndex = particleIndex + i * numParticles;
				sScratch->oneWayContactInfos[outputIndex].mNormal_PenW = sPerParticleTempContacts[src];
				sScratch->oneWayContactNodeIndices[outputIndex] = sScratch->rigidId;
			}
		}
		contactWritten = PxMin(contactCounts, PxgParticleContactInfo::MaxStaticContactsPerParticle);
		sScratch->oneWayContactCounts[particleIndex] = contactWritten;

	}

	PxU32 remaining = contactCounts - contactWritten;
	if ((remaining > 0) && !isDiffuse)
	{
		PxU64 compressedParticleIndex = PxEncodeParticleIndex(sScratch->particleSystemId, particleIndex);
		PxU32 contactStartIndex = atomicAdd(writer.numTotalContacts, remaining);
		PxU32 contactEndIndex = contactStartIndex + remaining;
		for (PxU32 contactIndex = contactStartIndex, src = 0; contactIndex < contactEndIndex; contactIndex++, src++)
		{
			float4 contact = sPerParticleTempContacts[src];
			writer.writeContact(contactIndex, PxVec4(contact.x, contact.y, contact.z, contact.w), compressedParticleIndex, sScratch->rigidId);
		}
	}
}


extern "C" __global__ void ps_heightfieldCollisonLaunch(
	const bool isTGS,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgParticleSystem* PX_RESTRICT particleSystems,
	PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	PxgParticleContactWriter writer
)
{
	__shared__ __align__(16) PxU8 sData[sizeof(PsHeightfieldScratch)];
	PsHeightfieldScratch& sHeightfieldScratch = reinterpret_cast<PsHeightfieldScratch&>(sData);

	__shared__ float4 sTempContacts[PxgParticleSystemKernelBlockDim::PS_HEIGHTFIELD_COLLISION][PxgParticleContactInfo::MaxStaticContactsPerMesh];

	unsigned int cmIdx = blockIdx.y;

	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const bool isDiffuseParticlesThread = blockIdx.z == 1;
	
	if (warpIndex == 0)
	{
		const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

		PxgShape particleShape, heightfieldShape;
		PxU32 particleCacheRef, heightfieldCacheRef;
		LoadShapePairWarp<PxGeometryType::ePARTICLESYSTEM, PxGeometryType::eHEIGHTFIELD>(cmInputs, cmIdx, gpuShapes,
			particleShape, particleCacheRef, heightfieldShape, heightfieldCacheRef);

		PxsCachedTransform heightfieldTransformCache;
		PxsCachedTransform_ReadWarp(heightfieldTransformCache, transformCache + heightfieldCacheRef);

		const PxReal contactDist = contactDistance[particleCacheRef] + contactDistance[heightfieldCacheRef];

		const PxNodeIndex rigidId = shapeToRigidRemapTable[heightfieldCacheRef];

		const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;
		PxgParticleSystem& particleSystem = particleSystems[particleSystemId];

		if (threadIndexInWarp == 0)
		{
			sHeightfieldScratch.heightfieldTransform = heightfieldTransformCache.transform;
			sHeightfieldScratch.heightfieldShape = heightfieldShape;
			sHeightfieldScratch.rigidId = rigidId;
			sHeightfieldScratch.transformCacheRef = heightfieldCacheRef;
			sHeightfieldScratch.contactDist = contactDist;
			sHeightfieldScratch.restDist = restDistances[cmIdx];
			sHeightfieldScratch.enableCCD = (particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;

			if (isDiffuseParticlesThread)
			{
				// If there is no diffuse particles or is not using PBD, disactivate collision handling from this kernel
				const bool isDiffuseParticlesActive = (particleSystem.mCommonData.mMaxDiffuseParticles > 0);
				sHeightfieldScratch.currentPositions = particleSystem.mDiffuseSortedOriginPos_LifeTime;
				sHeightfieldScratch.predictedPositions = particleSystem.mDiffuseSortedPos_LifeTime;
				sHeightfieldScratch.numParticles = (isDiffuseParticlesActive) ? *particleSystem.mNumDiffuseParticles : 0;
				sHeightfieldScratch.oneWayContactInfos = particleSystem.mDiffuseOneWayContactInfos;
				sHeightfieldScratch.oneWayContactNodeIndices = particleSystem.mDiffuseOneWayNodeIndex;
				sHeightfieldScratch.oneWayContactCounts = particleSystem.mDiffuseOneWayContactCount;
				sHeightfieldScratch.particleSystemId = particleSystemId;
			}
			else
			{
				sHeightfieldScratch.currentPositions = particleSystem.mSortedOriginPos_InvMass;
				sHeightfieldScratch.predictedPositions = particleSystem.mSortedPositions_InvMass;
				sHeightfieldScratch.numParticles = particleSystem.mCommonData.mNumParticles;
				sHeightfieldScratch.oneWayContactInfos = particleSystem.mOneWayContactInfos;
				sHeightfieldScratch.oneWayContactNodeIndices = particleSystem.mOneWayNodeIndex;
				sHeightfieldScratch.oneWayContactCounts = particleSystem.mOneWayContactCount;
				sHeightfieldScratch.particleSystemId = particleSystemId;
			}
		}
	}

	__syncthreads();
	
	const PxU32 numParticles = sHeightfieldScratch.numParticles;
	
	if (isDiffuseParticlesThread && numParticles == 0)
		return;

	for (PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x; globalThreadIndex < numParticles; globalThreadIndex += gridDim.x * blockDim.x)
	{
		psHeightfieldMidphaseCollision(
			&sHeightfieldScratch,
			sTempContacts[threadIdx.x],
			numParticles,
			globalThreadIndex,
			isDiffuseParticlesThread,
			isTGS,
			writer
			);
	}
}
