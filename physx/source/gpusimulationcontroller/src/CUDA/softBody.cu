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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  



#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMathUtils.h"
#include "PxgSoftBody.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgFEMCloth.h"
#include "PxgNpKernelIndices.h"
#include "PxgBodySim.h"
#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "shuffle.cuh"
#include "atomic.cuh"
#include "gridCal.cuh"
#include "stdio.h"
#include "PxgSolverBody.h"
#include "PxNodeIndex.h"
#include "PxgArticulationBlockData.h"
#include "assert.h"
#include "GuBV32.h"
#include "sbMidphaseScratch.cuh"
#include "copy.cuh"
#include "PxgSimulationCoreDesc.h"
#include "PxgSolverCoreDesc.h"
#include "PxgArticulationCoreDesc.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "particleSystem.cuh"
#include "utils.cuh"
#include "deformableUtils.cuh"

using namespace physx;

extern "C" __host__ void initSoftBodyKernels0() {}


//static __device__ __forceinline__
//PxVec3 shuffleMin(physx::PxVec3 v)
//{
//	for (PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
//	{
//		v.x = fminf(v.x, __shfl_xor_sync(FULL_MASK, v.x, reductionRadius));
//		v.y = fminf(v.y, __shfl_xor_sync(FULL_MASK, v.y, reductionRadius));
//		v.z = fminf(v.z, __shfl_xor_sync(FULL_MASK, v.z, reductionRadius));
//	}
//
//	return v;
//}
//
//static __device__ __forceinline__
//PxVec3 shuffleMax(physx::PxVec3 v)
//{
//	for (PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
//	{
//		v.x = fmaxf(v.x, __shfl_xor_sync(FULL_MASK, v.x, reductionRadius));
//		v.y = fmaxf(v.y, __shfl_xor_sync(FULL_MASK, v.y, reductionRadius));
//		v.z = fmaxf(v.z, __shfl_xor_sync(FULL_MASK, v.z, reductionRadius));
//	}
//
//	return v;
//}

//each block refit one of the soft body tree bound, each warp has 32 threads
extern "C" __global__
__launch_bounds__(1024, 1)
void sb_refitBoundLaunch(
	PxgSoftBody* gBoftbodies,
	const PxU32* activeSoftbodies,
	const PxU32 nbActiveSoftBodies,
	PxReal* bpContactDistance,
	PxReal* npContactDistance,
	PxReal* speculativeCCDContactOffset,
	PxBounds3* boundArray,
	const PxU32* elemIndex)
{
	//const PxU32 maxPackedNode = 128;
	__shared__ PxU32 scratchMem[WARP_SIZE * 7];
	//__shared__ PxBounds3 sPackedNodeBounds[maxPackedNode];

	const PxU32 idx = threadIdx.x;
	const PxU32 warpIndex = threadIdx.y;

	//PxgSoftBody& sSoftBody = sSoftBodies[warpIndex];
	sbMidphaseScratch* s_warpScratch = reinterpret_cast<sbMidphaseScratch*>(scratchMem);

	const PxU32 softbodyId = activeSoftbodies[blockIdx.x];
	
	PxgSoftBody& gSoftbody = gBoftbodies[softbodyId];

	PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8 *>(gSoftbody.mTetMeshData);

	const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
	tetmeshGeomPtr += sizeof(uint4);

	const PxU32 maxDepth = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z;

	PxBounds3* sPackedNodeBounds = gSoftbody.mPackedNodeBounds;

	if (idx == 0 && warpIndex == 0)
	{
		s_warpScratch->tetmeshVerts = gSoftbody.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices = gSoftbody.mTetIndices;

		//const PxU32 & numVerts = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.x;
		//const PxU32 & numTets = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;
		
		const PxU32 & nbBv32PackedNodes = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

		//assert(nbBv32PackedNodes <= maxPackedNode);

		//printf("maxDepth %i numVerts %i numTets %i nbBv32TreeNodes %i\n", maxDepth, numVerts, numTets, nbBv32TreeNodes);

		Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
		s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		tetmeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbBv32PackedNodes;


		Gu::BV32DataDepthInfo* bv32DepthInfo = reinterpret_cast<Gu::BV32DataDepthInfo*>(tetmeshGeomPtr);
		s_warpScratch->bv32DepthInfo = bv32DepthInfo;
		tetmeshGeomPtr += sizeof(const Gu::BV32DataDepthInfo) * maxDepth;

		PxU32* remapPackedNodeIndex = reinterpret_cast<PxU32*>(tetmeshGeomPtr);
		s_warpScratch->bv32RemapPackedNodeIndex = remapPackedNodeIndex;
		tetmeshGeomPtr += sizeof(PxU32) * nbBv32PackedNodes;
	}

	__syncthreads();


	//Depth Buffer will be all the node index
	const Gu::BV32DataDepthInfo* depthInfo = s_warpScratch->bv32DepthInfo;

	const PxU32* remapPackedNodeIndex = s_warpScratch->bv32RemapPackedNodeIndex;

		
	//each warp to deal with one node
	for (PxU32 i = maxDepth; i > 0; i--)
	{
		const Gu::BV32DataDepthInfo& info = depthInfo[i-1];

		const PxU32 offset = info.offset;
		const PxU32 count = info.count;

		/*if (warpIndex == 0 && idx == 0 )
		{
			printf("blockIdx.x %i depth %i offset %i count %i\n", blockIdx.x, i, offset, count);
		}
*/
		
		for (PxU32 j = warpIndex; j < count; j += SB_REFIT_WAPRS_PER_BLOCK)
		{
			const PxU32 nodeIndex = remapPackedNodeIndex[j + offset];

			Gu::BV32DataPacked& currentNode = s_warpScratch->bv32PackedNodes[nodeIndex];

			const PxU32 nbChildren = currentNode.mNbNodes;

			//compute the bitMask for all the leaf node
			PxU32 resultWarp = __ballot_sync(FULL_MASK, idx < nbChildren && currentNode.isLeaf(idx));
			PxU32 offset = warpScanExclusive(resultWarp, idx);
			PxU32 validCount = __popc(resultWarp);

			PxVec3 min(PX_MAX_F32);
			PxVec3 max(-PX_MAX_F32);

			//using one warp to compute all leaf node's under the same current node's min and max 
			for (PxU32 k = resultWarp; k; k = clearLowestSetBit(k))
			{
				const PxU32 indexInWarp = (k == 0) ? 0 : lowestSetIndex(k);

				const PxU32 nbTets = currentNode.getNbReferencedPrimitives(indexInWarp);
				const PxU32 startIndex = currentNode.getPrimitiveStartIndex(indexInWarp);

				/*if (i == 1 && idx == 0)
					printf("nodeIndex %i indexInWarp %i nbTets %i startIndex %i\n", nodeIndex, indexInWarp, nbTets, startIndex);*/

				PxVec3 tMin = min;
				PxVec3 tMax = max;

				//each thread in a warp deal with on tetrahedron(maximum 32 tetrahderon)
				const PxU32 tetIndex = idx + startIndex;

				if (idx < nbTets)
				{
					const uint4 tetIdx = s_warpScratch->tetmeshTetIndices[tetIndex];

					const PxVec3 worldV0 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.x]);
					const PxVec3 worldV1 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.y]);
					const PxVec3 worldV2 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.z]);
					const PxVec3 worldV3 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.w]);

					PxReal tx = PxMin(worldV0.x, worldV1.x);
					PxReal ty = PxMin(worldV0.y, worldV1.y);
					PxReal tz = PxMin(worldV0.z, worldV1.z);

					PxReal tx2 = PxMin(worldV2.x, worldV3.x);
					PxReal ty2 = PxMin(worldV2.y, worldV3.y);
					PxReal tz2 = PxMin(worldV2.z, worldV3.z);

					tx = PxMin(tx, tx2);
					ty = PxMin(ty, ty2);
					tz = PxMin(tz, tz2);

					tMin.x = PxMin(tMin.x, tx);
					tMin.y = PxMin(tMin.y, ty);
					tMin.z = PxMin(tMin.z, tz);

					tx = PxMax(worldV0.x, worldV1.x);
					ty = PxMax(worldV0.y, worldV1.y);
					tz = PxMax(worldV0.z, worldV1.z);

					tx2 = PxMax(worldV2.x, worldV3.x);
					ty2 = PxMax(worldV2.y, worldV3.y);
					tz2 = PxMax(worldV2.z, worldV3.z);


					tx = PxMax(tx, tx2);
					ty = PxMax(ty, ty2);
					tz = PxMax(tz, tz2);

					tMax.x = PxMax(tMax.x, tx);
					tMax.y = PxMax(tMax.y, ty);
					tMax.z = PxMax(tMax.z, tz);
				}

				min = warpShuffleMin(tMin);
				max = warpShuffleMax(tMax);
			}

			/*if (i == 1 && idx == 0)
			{
				printf("leaf min(%f, %f, %f), max(%f, %f, %f)\n", min.x, min.y, min.z,
					max.x, max.y, max.z);
			}*/


			if (idx < nbChildren && !currentNode.isLeaf(idx))
			{
				const PxU32 childOffset = currentNode.getChildOffset(idx);

				min = sPackedNodeBounds[childOffset].minimum;
				max = sPackedNodeBounds[childOffset].maximum;
				/*
				if (i == 1)
				{
					printf("idx %i childOffset %i min(%f, %f, %f), max(%f, %f, %f)\n", idx, childOffset, min.x, min.y, min.z,
						max.x, max.y, max.z);
				}*/
			}

			if (idx < nbChildren)
			{
				//We already updated the bounds in the previous iterations
				reinterpret_cast<float4&>(currentNode.mMin[idx]) = make_float4(min.x, min.y, min.z, 0.f);
				reinterpret_cast<float4&>(currentNode.mMax[idx]) = make_float4(max.x, max.y, max.z, 0.f);
			}

			
			sPackedNodeBounds[nodeIndex].minimum = warpShuffleMin(min);
			sPackedNodeBounds[nodeIndex].maximum = warpShuffleMax(max);

		}

		__syncthreads();
	}

	//update the broad phase bound

	if (warpIndex == 0)
	{
		const PxU32 index = elemIndex[softbodyId];
		
		//we don't need to add on contactDist because the broad phase will add contact dist automatically
		//we need to update the contact distance to reflect CCD changes
		if ((gSoftbody.mBodyFlags & PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD) && idx == 0)
		{
			const PxReal contactDist = speculativeCCDContactOffset[softbodyId] + gSoftbody.mOriginalContactOffset;
			bpContactDistance[index] = contactDist;
			npContactDistance[index] = contactDist;
		}
	
		float* resBound = reinterpret_cast<float*>(&boundArray[index].minimum.x);

		PxBounds3& root = sPackedNodeBounds[0];
		//compute min(0-2) max(3-5)
		if (idx < 6)
		{
			float* r = reinterpret_cast<float*>(&root.minimum.x);
			resBound[idx] = r[idx];
		}

		/*if (idx == 0)
		{
			const PxReal mx0 = root.minimum.x - contactDist;
			const PxReal my0 = root.minimum.y - contactDist;
			const PxReal mz0 = root.minimum.z - contactDist;

			const PxReal mx1 = root.maximum.x + contactDist;
			const PxReal my1 = root.maximum.y + contactDist;
			const PxReal mz1 = root.maximum.z + contactDist;

			printf("root min(%f, %f, %f)\n", mx0, my0, mz0);
			printf("root max(%f, %f, %f)\n", mx1, my1, mz1);
		}*/
	}

}

struct sbScratch
{
	const float4 * PX_RESTRICT tetmeshVerts;
	const uint4 * PX_RESTRICT tetmeshTetIndices;
	PxMat33* restPoses;
};

//extern "C" __global__ void sb_computeRestPosesLaunch(
//	PxgSoftBody* newBoftbodies,
//	const PxU32 nbNewSoftBodies)
//{
//
//	//every two blocks to deal with one soft body
//	const PxU32 numThreadsPerSoftbody = 2048;
//
//	const PxU32 numThreadsRequired = numThreadsPerSoftbody * nbNewSoftBodies;
//
//	const PxU32 totalNumThreads = PxgSoftBodyKernelGridDim::SB_COMPUTERESTPOSES * PxgSoftBodyKernelBlockDim::SB_COMPUTERESTPOSES;
//
//	const PxU32 nbIterations = (numThreadsRequired + totalNumThreads - 1) / totalNumThreads;
//
//	/*if(blockIdx.x == 0 && threadIdx.x == 0)
//		printf("nbIterations %i nbNewSoftBodies %i \n", nbIterations, nbNewSoftBodies);*/
//
//	for (PxU32 i = 0.; i < nbIterations; ++i)
//	{
//		const PxU32 workIndex = i*blockDim.x * gridDim.x + (threadIdx.x + blockIdx.x * blockDim.x);
//
//		if (workIndex >= numThreadsRequired)
//			break;
//
//		//printf("workIndex %i \n", workIndex);
//
//		const PxU32 softbodyInd = workIndex / numThreadsPerSoftbody;
//
//		//printf("workIndex %i softbodyInd %i\n", workIndex, softbodyInd);
//
//		PxgSoftBody softbody = newBoftbodies[softbodyInd];
//
//		//PxU8* meshGeomPtr= reinterpret_cast<PxU8 *>(softbody.mTetMeshData1);
//
//		const uint4 nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(softbody.mTetMeshData);
//
//		const float4 * tetmeshVerts = softbody.mPosition_InvMass;
//		const uint4 * tetmeshTetIndices = softbody.mTetIndices;
//
//		PxMat33* restPoses = softbody.mTetraRestPoses;
//		
//		const PxU32 numTets = nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes.y;
//
//		const PxU32 nbIter = (numTets + numThreadsPerSoftbody - 1) / numThreadsPerSoftbody;
//
//		for (PxU32 j = 0; j < nbIter; ++j)
//		{
//			const PxU32 wrappedThreadIndex = workIndex % numThreadsPerSoftbody;
//
//			const PxU32 tetrahedronIdx = wrappedThreadIndex + numThreadsPerSoftbody * j;
//
//			if (tetrahedronIdx >= numTets)
//				break;
//
//			//printf("workIndex %i tetrahedronIdx %i\n", workIndex, tetrahedronIdx);
//
//			const uint4 tetInd = tetmeshTetIndices[tetrahedronIdx];
//			const float4 v0 = tetmeshVerts[tetInd.x];
//			const float4 v1 = tetmeshVerts[tetInd.y];
//			const float4 v2 = tetmeshVerts[tetInd.z];
//			const float4 v3 = tetmeshVerts[tetInd.w];
//
//			const PxVec3 u1 = PxLoad3(v1 - v0);
//			const PxVec3 u2 = PxLoad3(v2 - v0);
//			const PxVec3 u3 = PxLoad3(v3 - v0);
//
//			const PxMat33 D = PxMat33(u1, u2, u3);
//			restPoses[tetrahedronIdx] = D.getInverse();
//
//			/*if (tetrahedronIdx == 311)
//			{
//				PxMat33 inverse = D.getInverse();
//				printf("tetInd(%i, %i, %i, %i)\n", tetInd.x, tetInd.y, tetInd.z, tetInd.w);
//				printf("u0(%f, %f, %f)\n", u0.x, u0.y, u0.z);
//				printf("u1(%f, %f, %f)\n", u1.x, u1.y, u1.z);
//				printf("u2(%f, %f, %f)\n", u2.x, u2.y, u2.z);
//
//				printf("restPoses 0(%f, %f, %f)\n", inverse.column0.x, inverse.column0.y, inverse.column0.z);
//				printf("restPoses 1(%f, %f, %f)\n", inverse.column1.x, inverse.column1.y, inverse.column1.z);
//				printf("restPoses 2(%f, %f, %f)\n", inverse.column2.x, inverse.column2.y, inverse.column2.z);
//			}*/
//		}
//	}
//}

////This code is based on Matthias Muller's paper: A robust method to extract the rotational part of deoformations
////Basically, this is another way to extract a rotational matrix from deformation gradient instead of using polar
////decomposition 
//__device__ inline void sb_extractRotation(const PxMat33 &A, PxQuat& q, int maxIter)
//{
//	const PxReal eps = 1.0e-6f;
//	for (int iter = 0; iter < maxIter; iter++)
//	{
//		PxMat33 R(q);
//		PxVec3 omega = R.column0.cross(A.column0) + R.column1.cross(A.column1) + R.column2.cross(A.column2);
//		// (Cross(R.cols[0], A.cols[0]) + Cross(R.cols[1], A.cols[1]) + Cross(R.cols[2], A.cols[2]));
//
//		//omega *= 1.0f / (fabsf(Dot(R.cols[0], A.cols[0]) + Dot(R.cols[1], A.cols[1]) + Dot(R.cols[2], A.cols[2])) + 1.0e-6f);
//		omega *= 1.0f / (PxAbs(R.column0.dot(A.column0) + R.column1.dot(A.column1) + R.column2.dot(A.column2)) + eps);
//
//		float w = omega.magnitude();
//		if (w < eps)
//			break;
//
//		const PxVec3 axis = ((1.0f / w) * omega).getNormalized();
//		const PxQuat tempQ = PxQuat(w, axis);
//		q = tempQ * q;
//		q = q.getNormalized();
//	}
//}



//__device__ void computeTetrahedronRotations(const uint4* tetmeshTetIndices, const PxU32 tetrahedronIdx,
//	const float4* tetmeshVerts, const PxMat33* restPoses, float4* tetRotations, const PxQuat initialRotation)
//{
//	const uint4 tetInd = tetmeshTetIndices[tetrahedronIdx];
//	const float4 v0 = tetmeshVerts[tetInd.x];
//	const float4 v1 = tetmeshVerts[tetInd.y];
//	const float4 v2 = tetmeshVerts[tetInd.z];
//	const float4 v3 = tetmeshVerts[tetInd.w];
//
//	/*if (sourceInd == testInd)
//	{
//		printf("vertInd(%i, %i, %i %i)\n", tetInd.x, tetInd.y, tetInd.z, tetInd.w);
//		printf("x0(%.10f, %.10f, %.10f, %.10f)\n", v0.x, v0.y, v0.z, v0.w);
//		printf("x1(%.10f, %.10f, %.10f, %.10f)\n", v1.x, v1.y, v1.z, v1.w);
//		printf("x2(%.10f, %.10f, %.10f, %.10f)\n", v2.x, v2.y, v2.z, v2.w);
//		printf("x3(%.10f, %.10f, %.10f, %.10f)\n", v3.x, v3.y, v3.z, v3.w);
//	}*/
//
//	//compute displacement field
//	const PxVec3 u1 = PxLoad3(v1 - v0);
//	const PxVec3 u2 = PxLoad3(v2 - v0);
//	const PxVec3 u3 = PxLoad3(v3 - v0);
//
//	// load rest pose
//	const PxMat33 Qinv = restPoses[tetrahedronIdx];
//
//	/*if (sourceInd == testInd)
//		printf("%i Qinv c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
//			Qinv.column0.x, Qinv.column0.y, Qinv.column0.z,
//			Qinv.column1.x, Qinv.column1.y, Qinv.column1.z,
//			Qinv.column2.x, Qinv.column2.y, Qinv.column2.z);*/
//
//
//	const PxMat33 P = PxMat33(u1, u2, u3);
//
//	/*if (tetrahedronIdx == 0)
//		printf("%i P c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
//			P.column0.x, P.column0.y, P.column0.z,
//			P.column1.x, P.column1.y, P.column1.z,
//			P.column2.x, P.column2.y, P.column2.z);*/
//
//			// calculate deformation gradient
//	PxMat33 F = P * Qinv;
//
//	/*if (sourceInd == testInd)
//		printf("%i F c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
//			F.column0.x, F.column0.y, F.column0.z,
//			F.column1.x, F.column1.y, F.column1.z,
//			F.column2.x, F.column2.y, F.column2.z);*/
//
//	// Co-rotational strain
//	float4 tetR = tetRotations[tetrahedronIdx];
//
//	PxQuat q(tetR.x, tetR.y, tetR.z, tetR.w);
//
//
//	// initialize to identity quat if this is the first iteration (zero initialized memory)
//	if (q.magnitudeSquared() == 0.0f)
//		q = initialRotation;
//
//	/*if (sourceInd == testInd)
//		printf("before tetRot[%i](%.10f, %.10f, %.10f, %.10f)\n", tetrahedronIdx, q.x, q.y, q.z, q.w);*/
//
//	sb_extractRotation(F, q, 100);
//
//	tetRotations[tetrahedronIdx] = make_float4(q.x, q.y, q.z, q.w);
//}



__device__ void computeBarycentric(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, 
	const PxVec3& p, PxReal& v, PxReal& w, PxReal& x)
{
	const PxVec3 ba = b - a;
	const PxVec3 ca = c - a;
	const PxVec3 da = d - a;
	const PxVec3 pa = p - a;


	//PxMat33 bcd(ba, ca, da);
	//const PxReal detBcd = bcd.getDeterminant();
	const PxReal detBcd = ba.dot(ca.cross(da));

	/*if (fabsf(detBcd) <= 1.e-9f)
	{
		printf("Degenerated!!!\n");
		printf("a(%f, %f, %f)\n", a.x, a.y, a.z);
		printf("b(%f, %f, %f)\n", b.x, b.y, b.z);
		printf("c(%f, %f, %f)\n", c.x, c.y, c.z);
		printf("d(%f, %f, %f)\n", d.x, d.y, d.z);
	}*/

	/*PxMat33 pcd(pa, ca, da);
	const PxReal detPcd = pcd.getDeterminant();*/
	const PxReal detPcd = pa.dot(ca.cross(da));

	v = detPcd / detBcd;

	/*PxMat33 bpd(ba, pa, da);
	const PxReal detBpd = bpd.getDeterminant();*/
	const PxReal detBpd = ba.dot(pa.cross(da));
	w = detBpd / detBcd;

	/*PxMat33 bcp(ba, ca, pa);
	const PxReal detBcp = bcp.getDeterminant();*/
	const PxReal detBcp = ba.dot(ca.cross(pa));

	x = detBcp / detBcd;

	/*if (fabsf(detBcd) <= 1.e-9f)
	{
		printf("v %f w %f x %f\n", v, w, x);
	}*/
}

__device__ PxReal computeTriInvMass(const uint4 triVertIdx, const float4* position_invmass, const float4 barycentric)
{
	const float4 a = position_invmass[triVertIdx.x];
	const float4 b = position_invmass[triVertIdx.y];
	const float4 c = position_invmass[triVertIdx.z];

	const PxReal invMass = a.w * barycentric.x + b.w * barycentric.y + c.w * barycentric.z;

	return invMass;
}

extern "C" __global__ void sb_rigidContactPrepareLaunch(
	PxgSoftBody*					softbodies,
	float4*							contacts_restW,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemOtherContactInfo*			contactInfos,
	PxU32*							numContacts,
	PxgDbRigidContactBlock*			contactBlocks,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	PxReal*							rigidAppliedForces,
	const PxReal					invDt,
	PxgSolverSharedDescBase*		sharedDesc,
	const bool						isTGS
)
{
	const PxU32 tNumContacts = *numContacts;

	PxU32* solverBodyIndices = preDesc->solverBodyIndices;	
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		rigidAppliedForces[workIndex] = 0.0f;

		PxgFemOtherContactInfo contactInfo = contactInfos[workIndex];
		PxgDbRigidContactBlock& block = contactBlocks[workIndex/32];

		// First actor: rigid body
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxNodeIndex& rigidId = reinterpret_cast<const PxNodeIndex&>(pairInd0);

		// Second actor: soft body
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);

		PxgSoftBody& softbody = softbodies[PxGetSoftBodyId(pairInd1)];
		const PxU32 tetInd = PxGetSoftBodyElementIndex(pairInd1);

		if (tetInd == 0xfffff)
			continue;

		const uint4 tetrahedronIdx = softbody.mSimTetIndices[tetInd];

		float4* delta_invMass = softbody.mSimDeltaPos;
		const float4 barycentric = barycentrics[workIndex];

		const float4 contact_restW = contacts_restW[workIndex];
		const float4 normal_pen = normalPens[workIndex];

		const PxVec3 p(contact_restW.x, contact_restW.y, contact_restW.z);
		const PxReal rest = contact_restW.w;

		const float4 delta_invMass1 = barycentricProjectTet(tetrahedronIdx, delta_invMass, barycentric);
		const PxVec3 delta(delta_invMass1.x, delta_invMass1.y, delta_invMass1.z);

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal pen = normal_pen.w - rest;

		prepareDbRigidContact(block, normal, sharedDesc, p, pen, delta, rigidId, barycentric, prepareDesc, solverBodyIndices, softbody.mPenBiasClamp, invDt, isTGS);
	}
}



extern "C" __global__ void sb_particleContactPrepareLaunch(
	PxgSoftBody*					softbodies,
	PxgParticleSystem*				particlesystems,
	float4*							contacts,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemOtherContactInfo*			contactInfos,
	PxU32*							numContacts,
	PxgDbParticleContactBlock*		contactBlocks,
	float2*							softBodyAppliedForces,
	float2*							particleAppliedForces
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x&31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		//initialize appliedForces to be zero
		softBodyAppliedForces[workIndex] = make_float2(0.f, 0.f);
		particleAppliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgFemOtherContactInfo contactInfo = contactInfos[workIndex];
		PxgDbParticleContactBlock& block = contactBlocks[workIndex/32];

		PxU64 pairInd0 = contactInfo.pairInd0;

		//pairInd0 is a particle system
		const PxU32 tParticleSystemId = PxGetParticleSystemId(pairInd0);
		const PxU32 tParticleIndex = PxGetParticleIndex(pairInd0);
		
		//second one will be soft body
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		PxgSoftBody& softbody = softbodies[PxGetSoftBodyId(pairInd1)];
		const PxU32 tetInd = PxGetSoftBodyElementIndex(pairInd1);

		/*printf("workIndex %i particleSystemId %i particleIndex %i\n", workIndex, tParticleSystemId, tParticleIndex);
		printf("workIndex %i softbodyId %i tetInd %i\n", workIndex, pairInd1.getSoftBodyId(), tetInd);*/

		const uint4 tetrahedronIdx = softbody.mSimTetIndices[tetInd];
		float4* delta_invMassTet = softbody.mSimDeltaPos;

		/*	printf("workIndex %i tetrahedronId(%i, %i, %i, %i)\n", workIndex, tetrahedronIdx.x, tetrahedronIdx.y,
		tetrahedronIdx.z, tetrahedronIdx.w);*/

		const float4 normal_pen = normalPens[workIndex];

		const float4 barycentric = barycentrics[workIndex];

		// The narrowphase reports pen at the predicted position; re-reference it to the start-of-step
		// configuration, the same frame the solver's live relLinDelta is measured from, so the two agree.
		// Unlike the soft/cloth side (delta 0 at prep in TGS), the particle side integrates in both modes,
		// so its delta is live in TGS too.
		const PxVec3 tetDelta = PxLoad3(barycentricProjectTet(tetrahedronIdx, delta_invMassTet, barycentric));

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

		PxgParticleSystem& particleSystem = particlesystems[tParticleSystemId];
		const float4 delta_invMass = particleSystem.mSortedDeltaP[tParticleIndex];
		const PxVec3 particleDelta(delta_invMass.x, delta_invMass.y, delta_invMass.z);
		const PxReal pen = normal_pen.w - (particleDelta - tetDelta).dot(normal) - softbody.mRestDistance;

		block.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		block.barycentric[threadIndexInWarp] = barycentric;
		block.maxPenBiasClamp[threadIndexInWarp] = PxMax(softbody.mPenBiasClamp, particleSystem.mData.mPenBiasClamp);
	}
}


//contact prep for self collision/soft body vs soft body contacts
extern "C" __global__ void sb_softbodyContactPrepareLaunch(
	PxgSoftBody*						softbodies,
	float4*								contacts,
	float4*								normalPens,
	float4*								barycentrics0,
	float4*								barycentrics1,
	PxgFemFemContactInfo*				contactInfos,
	PxU32*								numContacts,
	PxgDbDbContactBlock*				contactBlocks,
	float2*								softBodyAppliedForces,
	const PxU32							maxContacts
)
{
	const PxU32 tNumContacts = PxMin(maxContacts, *numContacts);

	if (threadIdx.x == 0 && blockIdx.x == 0)
		*numContacts = tNumContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x&31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		//initialize appliedForces to be zero
		softBodyAppliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgFemFemContactInfo contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex/32];

		PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
	
		//first pairInd is soft body
		PxgSoftBody& softbody0 = softbodies[PxGetSoftBodyId(pairInd0)];
		const PxU32 tetInd0 = PxGetSoftBodyElementIndex(pairInd0);

		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);

		//second pairInd is soft body
		PxgSoftBody& softbody1 = softbodies[PxGetSoftBodyId(pairInd1)];
		const PxU32 tetInd1 = PxGetSoftBodyElementIndex(pairInd1);

		const PxReal rest = softbody0.mRestDistance + softbody1.mRestDistance;

		const float4 normal_pen = normalPens[workIndex];

		const float4 barycentric0 = barycentrics0[workIndex];
		const float4 barycentric1 = barycentrics1[workIndex];

		// The narrowphase reports pen at the predicted position; re-reference it to the start-of-step
		// configuration, the same frame the solver's live relLinDelta is measured from, so the two agree
		// (cf. sb_clothContactPrepareLaunch). No-op in TGS (no predicted delta yet -- mSimDeltaPos is 0 at
		// prep). ssNormal is the negated narrowphase normal (vol-vol reports db1->db0), so the compensation
		// adds (sb0-sb1).ssNormal.
		const uint4 sb0TetVert = softbody0.mSimTetIndices[tetInd0];
		const uint4 sb1TetVert = softbody1.mSimTetIndices[tetInd1];
		const float4 sb0Delta = barycentricProjectTet(sb0TetVert, softbody0.mSimDeltaPos, barycentric0);
		const float4 sb1Delta = barycentricProjectTet(sb1TetVert, softbody1.mSimDeltaPos, barycentric1);
		const PxVec3 ssNormal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal ssPen = (normal_pen.w - rest) + PxLoad3(sb0Delta - sb1Delta).dot(ssNormal);

		block.barycentric0[threadIndexInWarp] = barycentric0;
		block.barycentric1[threadIndexInWarp] = barycentric1;
		block.normal_pen[threadIndexInWarp] = make_float4(ssNormal.x, ssNormal.y, ssNormal.z, ssPen);

		block.maxPenBiasClamp[threadIndexInWarp] = PxMax(softbody0.mPenBiasClamp, softbody1.mPenBiasClamp);
	}
}


  
//! 
//! \brief    : prep cloth vs. soft body collision
//! 

extern "C" __global__ void sb_clothContactPrepareLaunch(
	PxgSoftBody*						softbodies,
	PxgFEMCloth*						clothes,
	float4*								normalPens,
	float4*								barycentrics0,
	float4*								barycentrics1,
	PxgFemFemContactInfo*				contactInfos,
	PxU32*								numContacts,
	PxgDbDbContactBlock*				contactBlocks,
	float2*								appliedForces,
	const PxU32							maxContacts
)
{
	const PxU32 tNumContacts = PxMin(maxContacts, *numContacts);

	if(threadIdx.x == 0 && blockIdx.x == 0)
		*numContacts = tNumContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		appliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgFemFemContactInfo contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex / 32];

		// Side 0: cloth (triangle or vertex)
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 clothId = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 triInd = PxGetClothElementIndex(pairInd0);

		// Side 1: softbody tet
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetInd = PxGetSoftBodyElementIndex(pairInd1);

		const float4 clothBC = barycentrics0[workIndex];
		const float4 softBodyBC = barycentrics1[workIndex];

		// The narrowphase reports pen at the predicted position; re-reference it to the start-of-step
		// configuration, the same frame the solver's live relLinDelta is measured from (cf.
		// sb_softbodyContactPrepareLaunch SS, which applies the same shift against its negated normal).
		// No-op in TGS (no predicted delta yet -- soft/cloth delta buffers are 0 at prep).
		const float4* PX_RESTRICT const clothAccumulatedDelta = cloth.mAccumulatedDeltaPos;
		const float4 clothDelta = clothAccumulatedDelta[triInd];

		const uint4 tetVertInd = softbody.mSimTetIndices[tetInd];
		const float4* PX_RESTRICT const softbodyAccumulatedDelta = softbody.mSimDeltaPos;
		const float4 softbodyDelta = barycentricProjectTet(tetVertInd, softbodyAccumulatedDelta, softBodyBC);

		const PxReal thickness = cloth.mRestDistance + softbody.mRestDistance;
		const float4 normal_pen = normalPens[workIndex];
		// normal stored directly (cloth->SB = db0->db1); SS negates -- vol-vol reports the opposite sense.
		const PxVec3 normal(normal_pen.x, normal_pen.y, normal_pen.z);

		const PxReal pen = normal_pen.w - thickness + PxLoad3(clothDelta - softbodyDelta).dot(normal);

		block.barycentric0[threadIndexInWarp] = clothBC;
		block.barycentric1[threadIndexInWarp] = softBodyBC;
		block.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);

		// Pair max of side 0 (cloth) and side 1 (softbody) bias clamps.
		block.maxPenBiasClamp[threadIndexInWarp] = PxMax(cloth.mPenBiasClamp, softbody.mPenBiasClamp);
	}
}

#define kGlobalRelax  0.125f


// SB-particle contact pre-count pass. For each active SP contact, bumps
// softbody.mSimDelta[v].w by 1 per touched tet vertex. The SP solve consumes
// .w for XPBD mass-splitting; the finalize divides .xyz by max(.w, 1) and zeros
// both. Activation is decided by solveDbDbContact with checkOnlyActivity=true
// and recorded via markInCollision so contact reports see the same state.
extern "C" __global__ void sb_querySBParticleContactReferenceCountLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particlesystems,
	PxgFemOtherContactInfo*						contactInfos,
	PxgDbParticleContactBlock*					contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt
)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbParticleContactBlock& block = contactBlocks[workIndex / 32];

		// pairInd0 = particle, pairInd1 = softbody tet (matches narrowphase).
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);
		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		// Kinematic particles (invMass0 == 0) are inactive end-to-end:
		// sb_solveOutputSPDeltaVLaunch skips the whole solve body. Match here.
		if(invMass0 == 0.f)
		{
			contactInfo.markInCollision(false);
			continue;
		}

		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

		const float4 sbBcF4 = block.barycentric[threadIndexInWarp];

		// Side convention matches sb_solveOutputSPDeltaVLaunch:
		// side 0 = SB tet, side 1 = particle. The prep stores state.normal
		// as -narrowphase to match this db0 -> db1 convention.
		PxgDeformablePart<PxVec4> sbPart;
		sbPart.bc = PxVec4(sbBcF4.x, sbBcF4.y, sbBcF4.z, sbBcF4.w);
		sbPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
		sbPart.readSoftBody(softbody, tetId, sbBcF4, NULL, /*checkOnlyActivity*/ true);

		PxgDeformablePart<PxVec3> particlePart;
		particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
		particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
		particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		const bool isActive = solveDbDbContact(sbPart, particlePart, state,
												 /*appliedNormalLambdaRef*/ 0.0f,
												 /*appliedTanLambdaRef*/ 0.0f,
												 /*frictionCoefficient*/ 0.0f,
												 dt,
												 /*checkOnlyActivity*/ true);

		contactInfo.markInCollision(isActive);

		if(isActive)
		{
			// Integer-count refcount bump per touched tet vertex (gated |bc| > 1e-3).
			const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
			bumpDbRefCountTet(softbody.mSimDelta, tetrahedronId, sbBcF4.x, sbBcF4.y, sbBcF4.z, sbBcF4.w);

#if PX_DB_PARTICLE_MASS_SPLIT
			// Particle-side mass-split count (dual of the SB-vertex refCount).
			atomicAdd(&particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
#endif
		}
	}
}


// SB-particle contact solve, softbody side of the split writeback.
// Side 0 = SB (PxVec4 tet), side 1 = particle (PxVec3, a single vertex with bc=(1,0,0) inline). The
// prep stores state.normal as -narrowphase, which matches solveDbDbContact's
// db0 -> db1 convention. Particles have no pen-bias, so the particle side
// uses the -1e32 sentinel.
//
// Mass-splitting: sb_querySBParticleContactReferenceCountLaunch pre-populates
// softbody.mSimDelta[v].w with the refcount; readSoftBody multiplies
// vertexInvMasses by .w; scatter via writeSoftBody does NOT touch .w
// so it survives to the finalize's max(.w, 1) divide.
extern "C" __global__ void sb_solveOutputSPDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particlesystems,
	PxgFemOtherContactInfo*						contactInfos,
	PxgDbParticleContactBlock*					contactBlocks,
	PxU32*										numContacts,
	float2*										appliedForces,				//output
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x&31;


	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbParticleContactBlock& block = contactBlocks[workIndex/32];

		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if (invMass0 != 0.f)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 sbBcF4 = block.barycentric[threadIndexInWarp];
			const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
			PxgSoftBody& softbody = softbodies[softbodyId];
			const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

			// SB side. checkOnlyActivity=false consumes the refCount in .w
			// (populated by sb_querySBParticleContactReferenceCountLaunch) to
			// mass-split vertexInvMasses, and reads materials for friction.
			PxgDeformablePart<PxVec4> sbPart;
			sbPart.bc = PxVec4(sbBcF4.x, sbBcF4.y, sbBcF4.z, sbBcF4.w);
			sbPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			sbPart.readSoftBody(softbody, tetId, sbBcF4, materials, /*checkOnlyActivity*/ false);

			PxgDeformablePart<PxVec3> particlePart;
			particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
#if PX_DB_PARTICLE_MASS_SPLIT
			// Inflate by the contact count so both solve halves compute the same lambda.
			const PxReal particleRefCount = PxMax(particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
			particlePart.vertexInvMasses = PxVec3(invMass0 * particleRefCount, 0.0f, 0.0f);
#else
			particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
#endif
			particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			particlePart.friction = psMat.friction;

			PxgDbContactState state;
			state.readContactPrepDbDb(block, threadIndexInWarp);

			const PxReal frictionCoefficient = (sbPart.friction + particlePart.friction) * 0.5f;

			float2 appliedForce = appliedForces[workIndex];
			solveDbDbContact(sbPart, particlePart, state,
							   appliedForce.x, appliedForce.y, frictionCoefficient, dt);

			if (state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f)
			{
				PxVec3 deltaPos;
				state.computeDeformableDelta(deltaPos, dt);

				// SB side scatter via writeSoftBody: .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
				sbPart.writeSoftBody(softbody, tetId, sbBcF4, /*elemIsVertex*/ false, deltaPos);
			}

			appliedForce.x = state.accumulatedDeltaLambdaN;
			appliedForce.y += state.deltaLambdaT;
			appliedForces[workIndex] = appliedForce;
		}
	}
}



// SB-particle contact solve, particle side of the split writeback. Math
// matches sb_solveOutputSPDeltaVLaunch above; only the scatter target
// differs (deltaP[] vs. mSimDelta atomics).
extern "C" __global__ void sb_solveOutputParticleDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particlesystems,
	PxgFemOtherContactInfo*						contactInfos,
	PxgDbParticleContactBlock*					contactBlocks,
	PxU32*										numContacts,
	float4*										deltaP,			//output
	float2*										appliedForces,	//output
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x&31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbParticleContactBlock& block = contactBlocks[workIndex/32];

		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if (invMass0 != 0.f)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 sbBcF4 = block.barycentric[threadIndexInWarp];
			const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
			PxgSoftBody& softbody = softbodies[softbodyId];
			const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

			PxgDeformablePart<PxVec4> sbPart;
			sbPart.bc = PxVec4(sbBcF4.x, sbBcF4.y, sbBcF4.z, sbBcF4.w);
			sbPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			// Read full (not activity-only) so both halves see the same invMass + friction and
			// compute the identical lambda -- the impulse stays equal-and-opposite. Side-effect
			// free here: this kernel scatters only to deltaP[], never to mSimDelta.
			sbPart.readSoftBody(softbody, tetId, sbBcF4, materials, /*checkOnlyActivity*/ false);

			PxgDeformablePart<PxVec3> particlePart;
			particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
#if PX_DB_PARTICLE_MASS_SPLIT
			// Inflate by the contact count so both solve halves compute the same lambda.
			const PxReal particleRefCount = PxMax(particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
			particlePart.vertexInvMasses = PxVec3(invMass0 * particleRefCount, 0.0f, 0.0f);
#else
			particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
#endif
			particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			particlePart.friction = psMat.friction;

			PxgDbContactState state;
			state.readContactPrepDbDb(block, threadIndexInWarp);

			const PxReal frictionCoefficient = (sbPart.friction + particlePart.friction) * 0.5f;

			float2 appliedForce = appliedForces[workIndex];
			solveDbDbContact(sbPart, particlePart, state,
							   appliedForce.x, appliedForce.y, frictionCoefficient, dt);

			// Particle-side scatter into deltaP[] (aggregated into mDeltaP later); .w flags a live contact.
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);
			const PxVec3 deltaV = -deltaPos * particlePart.vertexInvMasses.x;
			const PxReal w = (state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f) ? 1.f : 0.f;

			deltaP[workIndex] = make_float4(deltaV.x, deltaV.y, deltaV.z, w);

			appliedForce.x = state.accumulatedDeltaLambdaN;
			appliedForce.y += state.deltaLambdaT;
			appliedForces[workIndex] = appliedForce;
		}
	}
}


// SB-SB contact pre-count pass. Runs once per outer position-iter before
// sb_solveOutputSSDeltaVLaunch. For each active contact, bumps
// softbody{0,1}.mSimDelta[v].w by 1 per touched tet vertex; the solve
// consumes .w for XPBD mass-splitting and the finalize divides .xyz by
// max(.w, 1) and zeros both. Activation is decided by solveDbDbContact
// with checkOnlyActivity=true; markInCollision is sticky across iters so
// the .w refcount stays consistent if the contact toggles activity.
extern "C" __global__ void sb_querySSContactReferenceCountLaunch(
	PxgSoftBody*								softbodies,
	PxgFemFemContactInfo*						contactInfos,
	PxgDbDbContactBlock*						contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= tNumContacts)
			return;

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex / 32];

		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId0 = PxGetSoftBodyId(pairInd0);
		const PxU32 softbodyId1 = PxGetSoftBodyId(pairInd1);
		const PxU32 tetId0 = PxGetSoftBodyElementIndex(pairInd0);
		const PxU32 tetId1 = PxGetSoftBodyElementIndex(pairInd1);

		PxgSoftBody& softbody0 = softbodies[softbodyId0];
		PxgSoftBody& softbody1 = softbodies[softbodyId1];

		const float4 barycentric0 = block.barycentric0[threadIndexInWarp];
		const float4 barycentric1 = block.barycentric1[threadIndexInWarp];

		PxgDeformablePart<PxVec4> db0;
		PxgDeformablePart<PxVec4> db1;
		// checkOnlyActivity=true: skip the refcount multiply (the pre-count
		// hasn't populated .w yet) and the friction read (only the activation
		// predicate is consumed below).
		db0.readSoftBody(softbody0, tetId0, barycentric0, NULL, /*checkOnlyActivity*/ true);
		db1.readSoftBody(softbody1, tetId1, barycentric1, NULL, /*checkOnlyActivity*/ true);

		db0.readContactPrepDbSide0(block, threadIndexInWarp);
		db1.readContactPrepDbSide1(block, threadIndexInWarp);

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		const bool isActive = solveDbDbContact(db0, db1, state,
												 /*appliedNormalLambdaRef*/ 0.0f,
												 /*appliedTanLambdaRef*/ 0.0f,
												 /*frictionCoefficient*/ 0.0f,
												 dt,
												 /*checkOnlyActivity*/ true);

		contactInfo.markInCollision(isActive);

		if(isActive)
		{
			// Integer-count refcount bump per touched tet vertex (gated |bc| > 1e-3).
			const uint4 tetrahedronId0 = softbody0.mSimTetIndices[tetId0];
			const uint4 tetrahedronId1 = softbody1.mSimTetIndices[tetId1];
			bumpDbRefCountTet(softbody0.mSimDelta, tetrahedronId0, barycentric0.x, barycentric0.y, barycentric0.z, barycentric0.w);
			bumpDbRefCountTet(softbody1.mSimDelta, tetrahedronId1, barycentric1.x, barycentric1.y, barycentric1.z, barycentric1.w);
		}
	}
}

//solve soft bodies contacts, store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSSDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFemFemContactInfo*						contactInfos,
	PxgDbDbContactBlock*						contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt,
	float2*										appliedForces,				//output
	PxsDeformableVolumeMaterialData*			materials
)
{
	// SS contact body. Single kernel serves both PGS and TGS -- the math is
	// position-domain with no live-velocity read, so the PGS / TGS distinction
	// (linDelta = 0 on PGS for rigid-vs-deformable) has no analog here.
	//
	// Mass-splitting: sb_querySSContactReferenceCountLaunch pre-populates
	// softbody.mSimDelta[v].w with the refcount; readSoftBody multiplies
	// vertexInvMasses by .w; scatter via writeSoftBody does NOT touch .w
	// so it survives to the finalize's max(.w, 1) divide. Inactive contacts
	// skip scatter; their pre-counted .w averages cleanly at finalize.

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex / 32];

		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 softbodyId0 = PxGetSoftBodyId(pairInd0);
		PxgSoftBody& softbody0 = softbodies[softbodyId0];
		const PxU32 tetId0 = PxGetSoftBodyElementIndex(pairInd0);

		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId1 = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody1 = softbodies[softbodyId1];
		const PxU32 tetId1 = PxGetSoftBodyElementIndex(pairInd1);

		const float4 barycentric0 = block.barycentric0[threadIndexInWarp];
		const float4 barycentric1 = block.barycentric1[threadIndexInWarp];

		PxgDeformablePart<PxVec4> db0;
		PxgDeformablePart<PxVec4> db1;
		db0.readSoftBody(softbody0, tetId0, barycentric0, materials, /*checkOnlyActivity*/ false);
		db1.readSoftBody(softbody1, tetId1, barycentric1, materials, /*checkOnlyActivity*/ false);

		const PxReal frictionCoefficient = (db0.friction + db1.friction) * 0.5f;

		db0.readContactPrepDbSide0(block, threadIndexInWarp);
		db1.readContactPrepDbSide1(block, threadIndexInWarp);

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		float2 appliedForce = appliedForces[workIndex]; // (lambdaN_accum, lambdaT_accum)

		solveDbDbContact(db0, db1, state, appliedForce.x, appliedForce.y, frictionCoefficient, dt);

		if (state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f)
		{
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);

			// Scatter .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
			db0.writeSoftBody(softbody0, tetId0, barycentric0, /*elemIsVertex*/ false, deltaPos);
			db1.writeSoftBody(softbody1, tetId1, barycentric1, /*elemIsVertex*/ false, -deltaPos);
		}

		appliedForce.x = state.accumulatedDeltaLambdaN;
		appliedForce.y += state.deltaLambdaT;
		appliedForces[workIndex] = appliedForce;
	}
}

//! 
//! \brief    : solve cloth vs. soft body collision
//! 

// SB-cloth contact pre-count pass. For each active SC contact, bumps
// cloth.mDeltaPos[v].w and softbody.mSimDelta[v].w by 1 per touched vertex.
// The SC solve consumes both .w buffers for XPBD mass-splitting; each
// side's finalize divides .xyz by max(.w, 1) and zeros both.
extern "C" __global__ void sb_querySCContactReferenceCountLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMCloth*								clothes,
	PxgFemFemContactInfo*						contactInfos,
	PxgDbDbContactBlock*						contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt
)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= tNumContacts)
			return;

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex / 32];

		// Side 0: cloth (triangle or vertex per clothBC.w).
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 clothId = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 clothElemIdx = PxGetClothElementIndex(pairInd0);
		const float4 clothBC = block.barycentric0[threadIndexInWarp];

		// Side 1: softbody tet.
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);
		const float4 sbBC = block.barycentric1[threadIndexInWarp];

		PxgDeformablePart<PxVec3> dbCloth;
		PxgDeformablePart<PxVec4> dbSb;
		dbCloth.readCloth(cloth, clothElemIdx, clothBC, NULL, /*countReferenceOnly*/ true);
		dbSb.readSoftBody(softbody, tetId, sbBC, NULL, /*checkOnlyActivity*/ true);

		dbCloth.readContactPrepDbSide0(block, threadIndexInWarp);
		dbSb.readContactPrepDbSide1(block, threadIndexInWarp);

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		const bool isActive = solveDbDbContact(dbCloth, dbSb, state,
												 /*appliedNormalLambdaRef*/ 0.0f,
												 /*appliedTanLambdaRef*/ 0.0f,
												 /*frictionCoefficient*/ 0.0f,
												 dt,
												 /*checkOnlyActivity*/ true);

		contactInfo.markInCollision(isActive);

		if(isActive)
		{
			// Integer-count refcount bump per side (gated |bc| > 1e-3).
			// Cloth-triangle (clothBC.w == 0): bumps per touched triangle
			// vertex; cloth-vertex (clothBC.w != 0): bumps the lone vertex.
			if(clothBC.w == 0.0f)
			{
				const uint4 triVertId = cloth.mTriangleVertexIndices[clothElemIdx];
				bumpDbRefCountTri(cloth.mDeltaPos, triVertId, clothBC.x, clothBC.y, clothBC.z);
			}
			else
			{
				bumpDbRefCountVtx(cloth.mDeltaPos, clothElemIdx);
			}

			const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
			bumpDbRefCountTet(softbody.mSimDelta, tetrahedronId, sbBC.x, sbBC.y, sbBC.z, sbBC.w);
		}
	}
}

extern "C" __global__ void sb_solveOutputSCDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMCloth*								clothes,
	PxgFemFemContactInfo*						contactInfos,
	PxgDbDbContactBlock*						contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt,
	float2*										appliedForces, //output
	PxsDeformableVolumeMaterialData*			sbMaterials,
	PxsDeformableSurfaceMaterialData*			clothMaterials
)
{
	// SC contact body: PxgDeformablePart<PxVec3> (cloth) + <PxVec4> (softbody)
	// through solveDbDbContact. Single kernel serves both PGS and TGS -- the
	// math is position-domain with no live-velocity read on either side.
	//
	// appliedForces[workIndex] is (lambdaN_accum, lambdaT_accum) in
	// impulse-per-time units, per-step zero-initialized in the prep.
	//
	// Mass-splitting: sb_querySCContactReferenceCountLaunch pre-populates
	// cloth.mDeltaPos[v].w and softbody.mSimDelta[v].w; readCloth/readSoftBody
	// multiply vertexInvMasses by the refcount. Scatter via writeCloth /
	// writeSoftBody does NOT touch .w so it survives to each side's finalize.

	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgDbDbContactBlock& block = contactBlocks[workIndex / 32];

		// Side 0: cloth (triangle or vertex).
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 clothId = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 clothElemIdx = PxGetClothElementIndex(pairInd0);
		const float4 clothBC = block.barycentric0[threadIndexInWarp];

		// Side 1: softbody tet.
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);
		const float4 sbBC = block.barycentric1[threadIndexInWarp];

		PxgDeformablePart<PxVec3> dbCloth;
		PxgDeformablePart<PxVec4> dbSb;
		dbCloth.readCloth(cloth, clothElemIdx, clothBC, clothMaterials, /*countReferenceOnly*/ false);
		dbSb.readSoftBody(softbody, tetId, sbBC, sbMaterials, /*checkOnlyActivity*/ false);

		const PxReal frictionCoefficient = (dbCloth.friction + dbSb.friction) * 0.5f;

		dbCloth.readContactPrepDbSide0(block, threadIndexInWarp);
		dbSb.readContactPrepDbSide1(block, threadIndexInWarp);

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		float2 appliedForce = appliedForces[workIndex]; // (lambdaN_accum, lambdaT_accum)

		solveDbDbContact(dbCloth, dbSb, state, appliedForce.x, appliedForce.y, frictionCoefficient, dt);

		if (state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f)
		{
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);

			// Scatter .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
			dbCloth.writeCloth(cloth, clothElemIdx, clothBC, /*elemIsVertex*/ clothBC.w != 0.0f, deltaPos);
			dbSb.writeSoftBody(softbody, tetId, sbBC, /*elemIsVertex*/ false, -deltaPos);
		}

		appliedForce.x = state.accumulatedDeltaLambdaN;
		appliedForce.y += state.deltaLambdaT;
		appliedForces[workIndex] = appliedForce;
	}
}

static __device__ void queryRigidSoftBodyContactReferenceCount(
	PxgSoftBody* softbodies,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool wasActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgDbRigidContactBlock& block = contactBlocks[workIndex / 32];
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = block.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: soft body
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

		if(tetId < PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			const bool checkOnlyActivity = true; // Check only if the constraint is active, without evaluating forces/impulses.
			bool isActive = false;

			PxgSoftBody& softbody = softbodies[softbodyId];

			PxgRigidPart rigid;
			PxgDeformablePart<PxVec4> db;
			PxgDbContactState state;

			const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, artiCoreDesc->mMaxLinksPerArticulation);

			db.readSoftBody(softbody, tetId, bc, NULL, checkOnlyActivity);

			if(wasActive)
			{
				// Once activated, keep the contact pair active to maintain a conservative reference count.
				isActive = true;
			}
			else
			{
				rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, NULL, NULL);
				rigid.readContactPrep(block, threadIndexInWarp);
				db.readContactPrep(block, threadIndexInWarp);
				state.readContactPrep(block, threadIndexInWarp);
				rigid.readVelocity(velocityReader, rigidId, isTGS);
				isActive = solveRbDbContact(rigid, db, state, appliedForces[workIndex], dt, wasActive, checkOnlyActivity);

				if(isActive)
				{
					contactInfo.markInCollision(true);
				}
			}

			if(isActive)
			{
				// Update soft body
				bumpDbRefCountTet(softbody.mSimDelta, softbody.mSimTetIndices[tetId], bc.x, bc.y, bc.z, bc.w);

				// Update rigidbody
				if(globalRigidBodyId != -1 && fricTan0_invMass0.w != 0.0f)
				{
					// Increment the reference count of the rigid body.
					atomicAdd(&rigidBodyRefCounts[globalRigidBodyId], 1);
				}
			}
		}
	}
}

extern "C" __global__
void sb_queryRigidSoftContactReferenceCountLaunch(
	PxgSoftBody* softbodies,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	bool isTGS)
{
	queryRigidSoftBodyContactReferenceCount(softbodies, contactInfos, contactBlocks, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
											solverBodyVelPool, dt, appliedForces, rigidBodyRefCounts, isTGS);
}

static __device__ void solveRigidSoftBodyContact(
	PxgSoftBody* softbodies,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	float4* rigidDeltaVel,
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	const PxReal dt,
	PxsDeformableVolumeMaterialData* materials,
	const PxsMaterialData* PX_RESTRICT rigidBodyMaterials,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool isActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgDbRigidContactBlock& block = contactBlocks[workIndex / 32];
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = block.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: soft body
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

		if(tetId < PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			const bool checkOnlyActivity = false;

			PxgRigidPart rigid;
			PxgDeformablePart<PxVec4> db;
			PxgDbContactState state;

			const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, artiCoreDesc->mMaxLinksPerArticulation);

			if(isActive)
			{
				PxgSoftBody& softbody = softbodies[softbodyId];

				rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, rigidBodyRefCounts,
						   &rigidBodyMaterials[contactInfo.getRigidMaterialIndex()]);
				db.readSoftBody(softbody, tetId, bc, materials, checkOnlyActivity);

				rigid.readContactPrep(block, threadIndexInWarp);
				db.readContactPrep(block, threadIndexInWarp);
				state.readContactPrep(block, threadIndexInWarp);
				rigid.readVelocity(velocityReader, rigidId, isTGS);
				solveRbDbContact(rigid, db, state, appliedForces[workIndex], dt, isActive, checkOnlyActivity);

				// Compute soft body delta
				PxVec3 deltaPos;
				appliedForces[workIndex] = state.computeDeformableDelta(deltaPos, dt);

				// Update soft body: scatter .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
				db.writeSoftBody(softbody, tetId, bc, /*elemIsVertex*/ false, deltaPos);
			}

			// Update rigid body. Must write every iteration even when !isActive (the
			// accumulate scan reads every slot); writeContactDeltas zeroes the no-op cases.
			rigid.writeContactDeltas(rigidDeltaVel, rigidId, state, workIndex, workIndex + tNumContacts);
		}
	}
}

//solve collision between soft body and primitives based on the sorted contact by rigid id
//store new velocity to rigid body buffer
extern "C" __global__ void sb_solveRigidSoftCollisionLaunch(
	PxgSoftBody*								softbodies,
	PxgFemOtherContactInfo*						contactInfos,
	PxgDbRigidContactBlock*						contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	float4*										solverBodyVelPool,
	float4*										rigidDeltaVel,				//output
	PxReal*										appliedForces,				//output
	PxU32*										rigidBodyRefCounts,
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials,
	const PxsMaterialData * PX_RESTRICT			rigidBodyMaterials,
	bool										isTGS)
{
	solveRigidSoftBodyContact(softbodies, contactInfos, contactBlocks, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool,
		rigidDeltaVel, appliedForces, rigidBodyRefCounts, dt, materials, rigidBodyMaterials, isTGS);
}
