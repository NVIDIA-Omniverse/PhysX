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

#include <stdio.h>
#include <stdint.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "GuIntersectionTriangleBoxRef.h"

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxGeometry.h"

#include "cudaNpCommon.h"
#include "PxgPersistentContactManifold.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxsContactManagerState.h"
#include "PxsTransformCache.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCore.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgNpKernelIndices.h"
#include "PxsMaterialCore.h"

#include "PxgCommonDefines.h"
#include "utils.cuh"
#include "deformableElementFilter.cuh"
#include "PxNodeIndex.h"
#include "sdfCollision.cuh"
#include "gridCal.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels11() {}

#include "warpHelpers.cuh"
#include "manifold.cuh"
#include "vector_functions.h"

#include "reduction.cuh"
#include "PxgSimulationCoreDesc.h"

#include "bv32Traversal.cuh"
#include "triangleMesh.cuh"
#include "particleCollision.cuh"


PX_ALIGN_PREFIX(16)
struct PsMidphaseScratch
{
	const float4* PX_RESTRICT trimeshVerts;
	const uint4* PX_RESTRICT trimeshTriIndices;

	PxTransform meshToWorld;
	PxMeshScale trimeshScale;
	
	//bv32 tree
	const Gu::BV32DataPacked* bv32PackedNodes;

	//stack for traversal
	int sBv32Nodes[192]; //6 depth of the bv32 tree
	PxU32 sBv32ActiveParticles[192];
	PxVec3 shLocalContacts[32][PxgParticleContactInfo::MaxStaticContactsPerMesh];
	PxReal shLocalContactDepths[32][PxgParticleContactInfo::MaxStaticContactsPerMesh];

	float4* currentPositions;
	float4* predictedPositions;
	bool enableCCD;
	PxU8 pad[7];
}PX_ALIGN_SUFFIX(16);


struct PsTreeContactGenTraverser
{
	PsMidphaseScratch* s_warpScratch;
	const PxU32 globalParticleIndex;
	const PxU32 numParticles;
	PxVec3 cVolumePosVertexSpace;
	PxReal cVolumeRadius;
	PxVec3 cVolumePosShapeSpace;
	PxVec3 particlePosShapeSpace;
	PxU32 currentNodeActiveParticles;
	PxU32 contactCount;

	PX_FORCE_INLINE __device__ PsTreeContactGenTraverser(
		PsMidphaseScratch* s_warpScratch,
		const PxU32 numParticles,
		const PxU32 globalParticleIndex,
		const bool isDiffuse,
		const bool isTGS,
		const PxReal contactDist
		)
	: s_warpScratch(s_warpScratch)
	, numParticles(numParticles)
	, globalParticleIndex(globalParticleIndex)
	, contactCount(0)
	{
		if (globalParticleIndex < numParticles)
		{
			const PxMeshScale trimeshScale = s_warpScratch->trimeshScale;
			const PxTransform meshToWorld = s_warpScratch->meshToWorld;
			const PxVec3 currentPos = PxLoad3(s_warpScratch->currentPositions[globalParticleIndex]);
			particlePosShapeSpace = meshToWorld.transformInv(currentPos);
			
			if (s_warpScratch->enableCCD)
			{
				const PxVec3 predictedPos = PxLoad3(s_warpScratch->predictedPositions[globalParticleIndex]);
				PxVec3 cVolumePos;
				cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos, currentPos, predictedPos, contactDist, isDiffuse, isTGS);
				cVolumePosShapeSpace = meshToWorld.transformInv(cVolumePos);
			}
			else
			{
				cVolumePosShapeSpace = particlePosShapeSpace;
				cVolumeRadius = contactDist;
			}
			cVolumePosVertexSpace = shape2Vertex(cVolumePosShapeSpace, trimeshScale.scale, trimeshScale.rotation);
		}

		//intiailize all valid particles as active particle
		currentNodeActiveParticles = __ballot_sync(FULL_MASK, globalParticleIndex < numParticles);
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& cMin, const PxVec3& cMax) const
	{
		const PxMeshScale trimeshScale = s_warpScratch->trimeshScale;

		PxVec3 sCVolumePosVertexSpace;
		PxReal sCVolumeRadius;

		PxU32 newActiveParticles = 0;
		for (PxU32 i = currentNodeActiveParticles; i; i = clearLowestSetBit(i))
		{
			const PxU32 pIndexInWarp = (i == 0) ? 0 : lowestSetIndex(i);

			sCVolumePosVertexSpace.x = __shfl_sync(FULL_MASK, cVolumePosVertexSpace.x, pIndexInWarp);
			sCVolumePosVertexSpace.y = __shfl_sync(FULL_MASK, cVolumePosVertexSpace.y, pIndexInWarp);
			sCVolumePosVertexSpace.z = __shfl_sync(FULL_MASK, cVolumePosVertexSpace.z, pIndexInWarp);
			sCVolumeRadius = __shfl_sync(FULL_MASK, cVolumeRadius, pIndexInWarp);

			if (hasBox)
			{
				const PxVec3 closestVertexSpace = cMin.maximum(cMax.minimum(sCVolumePosVertexSpace));
				const PxVec3 distVecVertexSpace = closestVertexSpace - sCVolumePosVertexSpace;
				const PxVec3 distVec = vertex2Shape(distVecVertexSpace, trimeshScale.scale, trimeshScale.rotation);
				const PxReal distSq = distVec.magnitudeSquared();

				if (distSq < sCVolumeRadius*sCVolumeRadius)
				{
					newActiveParticles |= 1 << pIndexInWarp;
				}
			}
		}
		return newActiveParticles != 0;
	}

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		const PxMeshScale trimeshScale = s_warpScratch->trimeshScale;
		const PxTransform meshToWorld = s_warpScratch->meshToWorld;

		PxVec3 sParticlePosShapeSpace;
		PxVec3 sCVolumePosShapeSpace;
		PxReal sCVolumeRadius;

		for (PxU32 i = currentNodeActiveParticles; i; i = clearLowestSetBit(i))
		{
			const PxU32 pIndexInWarp = (i == 0) ? 0 : lowestSetIndex(i);

			sParticlePosShapeSpace.x = __shfl_sync(FULL_MASK, particlePosShapeSpace.x, pIndexInWarp);
			sParticlePosShapeSpace.y = __shfl_sync(FULL_MASK, particlePosShapeSpace.y, pIndexInWarp);
			sParticlePosShapeSpace.z = __shfl_sync(FULL_MASK, particlePosShapeSpace.z, pIndexInWarp);

			sCVolumePosShapeSpace.x = __shfl_sync(FULL_MASK, cVolumePosShapeSpace.x, pIndexInWarp);
			sCVolumePosShapeSpace.y = __shfl_sync(FULL_MASK, cVolumePosShapeSpace.y, pIndexInWarp);
			sCVolumePosShapeSpace.z = __shfl_sync(FULL_MASK, cVolumePosShapeSpace.z, pIndexInWarp);
			sCVolumeRadius = __shfl_sync(FULL_MASK, cVolumeRadius, pIndexInWarp);

			bool intersect = false;
			PxVec3 normal(0.f);
			PxReal distance = PX_MAX_F32;
			PxU32 triangleIdx = 0xffffffff;

			if (primitiveIndex != 0xFFFFFFFF)
			{
				triangleIdx = primitiveIndex;

				uint4 triIdx = s_warpScratch->trimeshTriIndices[triangleIdx];

				PxVec3 triV0, triV1, triV2;
				triV0 = vertex2Shape(PxLoad3(s_warpScratch->trimeshVerts[triIdx.x]), trimeshScale.scale, trimeshScale.rotation);
				triV1 = vertex2Shape(PxLoad3(s_warpScratch->trimeshVerts[triIdx.y]), trimeshScale.scale, trimeshScale.rotation);
				triV2 = vertex2Shape(PxLoad3(s_warpScratch->trimeshVerts[triIdx.z]), trimeshScale.scale, trimeshScale.rotation);

				MeshScaling meshScale(s_warpScratch->trimeshScale.scale, s_warpScratch->trimeshScale.rotation);

				if (meshScale.flipNormal)
					PxSwap(triV1, triV2);

				PxVec3 normalShapeSpace;
				intersect = particleTriangleTest(normalShapeSpace, distance,
					sParticlePosShapeSpace, sCVolumePosShapeSpace, sCVolumeRadius, triV0, triV1, triV2, s_warpScratch->enableCCD);

				if (intersect)
				{
					normal = meshToWorld.rotate(normalShapeSpace);
				}
			}

			//push the triangle into stack ptr
			PxU32 resultWarp = __ballot_sync(FULL_MASK, intersect);
			PxU32 offset = warpScanExclusive(resultWarp, idxInWarp);
			PxU32 validCount = __popc(resultWarp);

			//However many contacts the particle already had...
			PxU32 contactStartIndex = __shfl_sync(FULL_MASK, contactCount, pIndexInWarp);

			if (idxInWarp == pIndexInWarp)
				contactCount += validCount;

			bool failedWrite = false;
			if (intersect)
			{
				const PxU32 index = contactStartIndex + offset;

				if (index < PxgParticleContactInfo::MaxStaticContactsPerMesh)
				{
					//If we have space for this contact (otherwise, we drop it!)
					s_warpScratch->shLocalContacts[pIndexInWarp][index] = normal;
					s_warpScratch->shLocalContactDepths[pIndexInWarp][index] = distance;
				}
				else
					failedWrite = true;
			}

			PxU32 failMask = __ballot_sync(FULL_MASK, failedWrite);

			if (failMask)
			{
				PxReal outSep = -PX_MAX_F32;
				if (idxInWarp < PxgParticleContactInfo::MaxStaticContactsPerMesh)
					outSep = s_warpScratch->shLocalContactDepths[pIndexInWarp][idxInWarp];

				for (PxU32 i = failMask; i != 0; i = clearLowestSetBit(i))
				{
					PxU32 index = lowestSetIndex(i);
					PxReal depth = __shfl_sync(FULL_MASK, distance, index);

					PxU32 candidateSwapMask = __ballot_sync(FULL_MASK, depth < outSep);
					if (candidateSwapMask)
					{
						PxReal bestVal;
						PxU32 bestIndex = maxIndex(outSep, FULL_MASK, bestVal);

						if (idxInWarp == index)
						{
							s_warpScratch->shLocalContacts[pIndexInWarp][bestIndex] = normal;
							s_warpScratch->shLocalContactDepths[pIndexInWarp][bestIndex] = distance;
						}
						if (idxInWarp == bestIndex)
							outSep = depth;
					}

				}

				if (idxInWarp == pIndexInWarp)
					contactCount = PxgParticleContactInfo::MaxStaticContactsPerMesh;
			}
		}
	}

	PX_FORCE_INLINE __device__ void finalizeFullWarp(
		PxU32 idxInWarp,
		const PxU32 particleSystemId,
		const PxNodeIndex rigidId,
		PxgParticleContactInfo* PX_RESTRICT oneWayContactInfos,		//output
		PxNodeIndex* PX_RESTRICT oneWayContactNodeIndices,			//output
		PxU32* PX_RESTRICT oneWayContactCounts,						//output
		PxgParticleContactWriter& writer,		
		const bool isDiffuseParticlesThread,
		const PxReal restDist
		)
	{
		//__syncwarp();

		PxU32 id = 0;
		if (globalParticleIndex < numParticles)
		{
			if (rigidId.isStaticBody())
			{
				if (contactCount != 0)
				{
					PxU32 startIndex = PxU32(atomicAdd((PxI32*)&oneWayContactCounts[globalParticleIndex], PxI32(contactCount)));

					PxU32 endIndex = PxMin(PxgParticleContactInfo::MaxStaticContactsPerParticle, startIndex + contactCount);

					for (PxU32 i = startIndex; i < endIndex; i++, id++)
					{
						const PxVec3& normal = s_warpScratch->shLocalContacts[threadIdx.x][id];
						PxReal penetration = s_warpScratch->shLocalContactDepths[threadIdx.x][id] - restDist;
						const PxU32 outputIndex = globalParticleIndex + i * numParticles;
						oneWayContactInfos[outputIndex].mNormal_PenW = make_float4(normal.x, normal.y, normal.z, penetration);
						oneWayContactNodeIndices[outputIndex] = rigidId;
					}
				}
			}
		}

		PxU32 remaining = contactCount - id;

		if (__any_sync(FULL_MASK, remaining))
		{

			PxU32 count = warpScan<AddOpPxU32, PxU32>(FULL_MASK, remaining);

			PxU32 startIndex = 0xFFFFFFFF;
			if (idxInWarp == 31)
			{
				startIndex = atomicAdd(writer.numTotalContacts, count);
			}

			startIndex = __shfl_sync(FULL_MASK, startIndex, 31);


			if (globalParticleIndex < numParticles)
			{
				startIndex = startIndex + count - remaining;

				if (remaining && !isDiffuseParticlesThread)
				{
					PxU64 compressedParticleIndex = PxEncodeParticleIndex(particleSystemId, globalParticleIndex);

					for (PxU32 i = id, index = startIndex; i < contactCount; ++i, ++index)
					{
						const PxVec3& normal = s_warpScratch->shLocalContacts[idxInWarp][i];
						PxReal penetration = s_warpScratch->shLocalContactDepths[idxInWarp][i] - restDist;
						writer.writeContact(index, PxVec4(normal, penetration), compressedParticleIndex, rigidId);
					}
				}
			}
		}
	}
};



__device__ static inline void psSdfMeshMidphaseCollision(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgParticleSystem* particleSystems,
	PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxgParticleContactWriter& writer
)
{
	__shared__ __align__(16) char sMesh[sizeof(PxgTriangleMesh)];
	PxgTriangleMesh& triangleMesh = reinterpret_cast<PxgTriangleMesh&>(*sMesh);

	__shared__ __align__(16) char sSdfTexture[sizeof(SparseSDFTexture)];
	SparseSDFTexture& sdfTexture = reinterpret_cast<SparseSDFTexture&>(*sSdfTexture);

	__shared__ PxU32 triangleIndicesS[1024];


	unsigned int cmIdx = blockIdx.x;
	const bool isDiffuseParticlesThread = blockIdx.y == 1;


	PxgShape trimeshShape, shape;
	PxU32 trimeshTransformId, particleTransformId;
	LoadShapePair<PxGeometryType::eTRIANGLEMESH, PxGeometryType::ePARTICLESYSTEM>(cmInputs, cmIdx, gpuShapes,
		trimeshShape, trimeshTransformId, shape, particleTransformId);

	PxsCachedTransform trimeshTransformCached;
	PxsCachedTransform_ReadWarp(trimeshTransformCached, transformCache + trimeshTransformId);


	const PxReal contactDist = contactDistance[trimeshTransformId] + contactDistance[particleTransformId];
	const PxReal restDist = restDistances[cmIdx];

	const PxNodeIndex rigidId = shapeToRigidRemapTable[trimeshTransformId];

	const PxU32 particleSystemId = shape.particleOrSoftbodyId;
	PxgParticleSystem& particleSystem = particleSystems[particleSystemId];

	// If there is no diffuse particles or is not using PBD, skip this function
	if (isDiffuseParticlesThread && (particleSystem.mCommonData.mMaxDiffuseParticles == 0))
		return;

	
	if (threadIdx.x < 32)
	{
		readTriangleMesh(trimeshShape, triangleMesh);
		__syncwarp();
	}

	float4* positions;
	float4* positionsPrev;
	if (isDiffuseParticlesThread)
	{
		positions = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedPos_LifeTime);
		positionsPrev = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedOriginPos_LifeTime);
	}
	else
	{
		positions = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);
		positionsPrev = reinterpret_cast<float4*>(particleSystem.mSortedOriginPos_InvMass);
	}
	

	if (threadIdx.x == 0)
		sdfTexture.initialize(triangleMesh); //The texture is stored in shared memory - only one threads needs to initialize it
	
	__syncthreads();

	PxU32 numParticles;
	PxgParticleContactInfo* PX_RESTRICT oneWayContactInfos;
	PxNodeIndex* PX_RESTRICT oneWayNodeIndices;
	PxU32* PX_RESTRICT oneWayContactCounts;

	if (isDiffuseParticlesThread)
	{
		numParticles = *particleSystem.mNumDiffuseParticles;
		oneWayContactInfos = particleSystem.mDiffuseOneWayContactInfos;
		oneWayNodeIndices = particleSystem.mDiffuseOneWayNodeIndex;
		oneWayContactCounts = particleSystem.mDiffuseOneWayContactCount;
	}
	else
	{
		numParticles = particleSystem.mCommonData.mNumParticles;
		oneWayContactInfos = particleSystem.mOneWayContactInfos;
		oneWayNodeIndices = particleSystem.mOneWayNodeIndex;
		oneWayContactCounts = particleSystem.mOneWayContactCount;
	}

	const PxU32* PX_RESTRICT gridParticleIndex = particleSystem.mSortedToUnsortedMapping;

	PxReal cullScale = contactDist;
	cullScale /= trimeshShape.scale.scale.abs().minElement();

	const PxTransform& trimeshToWorld = trimeshTransformCached.transform;
	const PxTransform particleToTrimeshTransform = trimeshToWorld.getInverse();

	bool enableCCD = (particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;

	for (PxU32 i = 0; i < numParticles;)
	{
		PxU32 nbFoundVertices = findInterestingVertices<32, 1024>(numParticles, positions,
			trimeshShape.scale, cullScale, sdfTexture, particleToTrimeshTransform, i, triangleIndicesS);

		bool candidateContact = false;
		PxVec3 pos;
		PxVec3 normal;
		PxReal distance;
		PxReal finalSep;
		PxVec3 posPrev;
		PxReal startDist; //distPrev

		PxU32 ind = threadIdx.x < nbFoundVertices ? triangleIndicesS[threadIdx.x] : numParticles;

		if (ind < numParticles)
		{
			pos = particleToTrimeshTransform.transform(PxLoad3(positions[ind]));
			PxVec3 v = shape2Vertex(pos, trimeshShape.scale.scale, trimeshShape.scale.rotation);

			posPrev = particleToTrimeshTransform.transform(PxLoad3(positionsPrev[ind]));


			PxVec3 dirPrev;
			startDist = doVertexSDFCollision(sdfTexture, shape2Vertex(posPrev, trimeshShape.scale.scale, trimeshShape.scale.rotation), dirPrev, cullScale);
			PxVec3 dir;
			distance = doVertexSDFCollision(sdfTexture, v, dir, cullScale);

			if (distance < cullScale || startDist < cullScale)
			{
				dir = vertex2ShapeNormalVector(dir, trimeshShape.scale.scale, trimeshShape.scale.rotation);
				dirPrev = vertex2ShapeNormalVector(dirPrev, trimeshShape.scale.scale, trimeshShape.scale.rotation);

				PxReal m = dir.magnitudeSquared();
				if (m > 0.0f)
				{
					m = 1.0f / PxSqrt(m);
					distance = distance * m;
					startDist = startDist * m;
					dir = dir * m;	
					dirPrev = dirPrev * m;
				}

				/*candidateContact = distance < contactDist;

				if (candidateContact)
				{
					sep = sep;
					normal = dir;
				}*/

				if (startDist < contactDist)
				{
					normal = dirPrev;
					finalSep = startDist - restDist;					
					candidateContact = true;
				}
				if (!candidateContact && distance > 0.f)
				{
					if (distance < contactDist)
					{
						normal = dir;
						finalSep = (distance - restDist) + normal.dot(posPrev - pos);
						candidateContact = true;
					}
				}
				if (enableCCD && !candidateContact)
				{
					if (distance < 0.f)
					{
						PxVec3 d = (pos - posPrev).getNormalized();

						PxReal displacement = -PxMax(0.f, startDist - restDist) / (dir.dot(d));

						PxVec3 pt = posPrev + d * displacement;

						distance = doVertexSDFCollision(sdfTexture, shape2Vertex(particleToTrimeshTransform.transform(pt), trimeshShape.scale.scale, trimeshShape.scale.rotation), dir, cullScale);
						dir = vertex2ShapeNormalVector(dir, trimeshShape.scale.scale, trimeshShape.scale.rotation);
						if (m > 0.0f)
						{
							distance = distance * m;
							dir = dir * m;
						}

						if (distance < contactDist)
						{
							normal = dir;
							finalSep = (distance - restDist) + normal.dot(posPrev - pt);
							candidateContact = true;
						}
					}
				}
			}

			const PxU64 particleMask = PxEncodeParticleIndex(particleSystemId, gridParticleIndex[ind]);

			if (!isDiffuseParticlesThread && find(particleSystem, rigidId.getInd(), particleMask))
				candidateContact = false;
		}
		

		if (rigidId.isStaticBody())
		{
			if (candidateContact)
			{
				PxU32 startIndex = PxU32(atomicAdd((PxI32*)&oneWayContactCounts[ind], 1));

				const PxU32 outputIndex = ind + (startIndex) * numParticles;
				if (startIndex < PxgParticleContactInfo::MaxStaticContactsPerParticle)
				{
					oneWayContactInfos[outputIndex].mNormal_PenW = make_float4(normal.x, normal.y, normal.z, finalSep/*sep - restDist + normal.dot(posPrev - pos)*/);
					oneWayNodeIndices[outputIndex] = rigidId;
					candidateContact = false; //Contact is stored, so it is not a candidate contact anymore
				}
			}
		}
		

		PxU32 contactIndex = globalScanExclusive<32>(candidateContact, writer.numTotalContacts);

		if (candidateContact && !isDiffuseParticlesThread)
		{
			normal = trimeshToWorld.rotate(normal);

			PxVec4 normalPen(normal.x, normal.y, normal.z, finalSep/*sep - restDist + normal.dot(posPrev - pos)*/);

			writer.writeContact(contactIndex, normalPen,
				PxEncodeParticleIndex(particleSystemId, ind), rigidId);
		}
	}
}




template<unsigned int WarpsPerBlock>
__device__ static inline void psMeshMidphaseCollision(
	const bool isTGS,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PsMidphaseScratch*	s_warpScratch,
	PxgParticleSystem* particleSystems,
	PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	PxgParticleContactWriter& writer
)
{
	//thread index in warp
	const unsigned int idxInWarp = threadIdx.x;
	//wrap index
	const unsigned int warpIdx = threadIdx.y;//(idx >> LOG2_WARP_SIZE);
											 //wrap index in block
											 //const unsigned int idx = idxInWarp + warpIdx * WARP_SIZE;
	
	unsigned int cmIdx = blockIdx.y;
	const bool isDiffuseParticlesThread = blockIdx.z == 1;
	

	PxgShape trimeshShape, shape;
	PxU32 trimeshTransformId, particleTransformId;
	LoadShapePair<PxGeometryType::eTRIANGLEMESH, PxGeometryType::ePARTICLESYSTEM>(cmInputs, cmIdx, gpuShapes,
		trimeshShape, trimeshTransformId, shape, particleTransformId);

	PxsCachedTransform trimeshTransformCached;
	PxsCachedTransform_ReadWarp(trimeshTransformCached, transformCache + trimeshTransformId);

	const PxReal contactDist = contactDistance[trimeshTransformId] + contactDistance[particleTransformId];
	const PxReal restDist = restDistances[cmIdx];
		
	const PxNodeIndex rigidId = shapeToRigidRemapTable[trimeshTransformId];

	const PxU32 particleSystemId = shape.particleOrSoftbodyId;
	PxgParticleSystem& particleSystem = particleSystems[particleSystemId];

	// If there is no diffuse particles or is not using PBD, skip this function
	if (isDiffuseParticlesThread && (particleSystem.mCommonData.mMaxDiffuseParticles == 0))
		return;

	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	if (idxInWarp == 0)
	{
		s_warpScratch->meshToWorld = trimeshTransformCached.transform;
		s_warpScratch->trimeshScale = trimeshShape.scale;
		
		readTriangleMesh(trimeshGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->trimeshVerts, s_warpScratch->trimeshTriIndices);
		
		if (isDiffuseParticlesThread)
		{
			s_warpScratch->currentPositions = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedOriginPos_LifeTime);
			s_warpScratch->predictedPositions = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedPos_LifeTime);
		}
		else
		{
			s_warpScratch->currentPositions = reinterpret_cast<float4*>(particleSystem.mSortedOriginPos_InvMass);
			s_warpScratch->predictedPositions = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);
		}
		s_warpScratch->enableCCD = (particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;
	}
		
	__syncwarp();

	PxU32 numParticles;
	PxgParticleContactInfo* PX_RESTRICT oneWayContactInfos;
	PxNodeIndex* PX_RESTRICT oneWayNodeIndices;
	PxU32* PX_RESTRICT oneWayContactCounts;

	if (isDiffuseParticlesThread)
	{
		numParticles = *particleSystem.mNumDiffuseParticles;
		oneWayContactInfos = particleSystem.mDiffuseOneWayContactInfos;
		oneWayNodeIndices = particleSystem.mDiffuseOneWayNodeIndex;
		oneWayContactCounts = particleSystem.mDiffuseOneWayContactCount;
	}
	else
	{
		numParticles = particleSystem.mCommonData.mNumParticles;
		oneWayContactInfos = particleSystem.mOneWayContactInfos;
		oneWayNodeIndices = particleSystem.mOneWayNodeIndex;
		oneWayContactCounts = particleSystem.mOneWayContactCount;
	}

	const PxU32 nbWarpsRequired = (numParticles + WARP_SIZE - 1) / WARP_SIZE;
	const PxU32 totalNumWarps = PxgParticleSystemKernelGridDim::PS_MESH_COLLISION * PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK;
	const PxU32 nbIterationsPerWarps = (nbWarpsRequired + totalNumWarps - 1) / totalNumWarps;
	const PxU32* PX_RESTRICT gridParticleIndex = particleSystem.mSortedToUnsortedMapping;

	for (PxU32 i = 0; i < nbIterationsPerWarps; ++i)
	{
		const PxU32 workIndex = i + (warpIdx + WarpsPerBlock * blockIdx.x) * nbIterationsPerWarps;
		
		if (workIndex < nbWarpsRequired)
		{
			const PxU32 particleIndex = idxInWarp + workIndex * WARP_SIZE;
			const PxU64 particleMask = PxEncodeParticleIndex(particleSystemId, gridParticleIndex[particleIndex]);

			if (!isDiffuseParticlesThread && find(particleSystem, rigidId.getInd(), particleMask))
				return;

			PsTreeContactGenTraverser traverser(
				s_warpScratch,
				numParticles,
				particleIndex,
				isDiffuseParticlesThread,
				isTGS,
				contactDist
			);
			bv32TreeTraversal<PsTreeContactGenTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
			
			traverser.finalizeFullWarp(idxInWarp, particleSystemId,
				rigidId,
				oneWayContactInfos,
				oneWayNodeIndices,
				oneWayContactCounts,
				writer,				
				isDiffuseParticlesThread,
				restDist
			);
		}
	}
}


extern "C" __global__
__launch_bounds__(1024, 1)
void ps_sdfMeshCollisonLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistances,
	const PxgShape* PX_RESTRICT					gpuShapes,

	PxgParticleSystem* PX_RESTRICT				particleSystems,						//output
	PxNodeIndex* PX_RESTRICT					shapeToRigidRemapTable,

	PxgParticleContactWriter writer
)
{
	psSdfMeshMidphaseCollision(
		tolerenceLength,
		cmInputs,
		transformCache,
		contactDistance,
		restDistances,
		gpuShapes,
		particleSystems,
		shapeToRigidRemapTable,
		writer
		);
}



extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void ps_meshCollisonLaunch(
	const bool									isTGS,
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistances,
	const PxgShape* PX_RESTRICT					gpuShapes,
	PxgParticleSystem* PX_RESTRICT				particleSystems,						//output
	PxNodeIndex* PX_RESTRICT					shapeToRigidRemapTable,
	PxgParticleContactWriter					writer
)
{
	__shared__ __align__(16) PxU8 scratchMem[PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK][sizeof(PsMidphaseScratch)];

	psMeshMidphaseCollision<PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK>(
		isTGS,
		tolerenceLength,
		cmInputs,
		transformCache,
		contactDistance,
		restDistances,
		gpuShapes,
		(PsMidphaseScratch*)&scratchMem[threadIdx.y],
		particleSystems,
		shapeToRigidRemapTable,
		writer
		);
}
