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



#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMathUtils.h"
#include "PxgSoftBodyCore.h"
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
#include "PxgArticulation.h"
#include "assert.h"
#include "GuBV32.h"
#include "sbMidphaseScratch.cuh"
#include "copy.cuh"
#include "softBody.cuh"
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
		const  Gu::BV32DataDepthInfo& info = depthInfo[i-1];

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

__device__ PxReal computeInvMass(const uint4 tetrahedronIdx, const float4* position_invmass, 
	const PxVec3& p, float4& barycentric)
{
	const float4 a = position_invmass[tetrahedronIdx.x];
	const float4 b = position_invmass[tetrahedronIdx.y];
	const float4 c = position_invmass[tetrahedronIdx.z];
	const float4 d = position_invmass[tetrahedronIdx.w];

	//printf("Idx %i, invMass(%f, %f, %f, %f)\n", threadIdx.x, a.w, b.w, c.w, d.w);

	//compute contact point mass
	PxReal v, w, x;
	computeBarycentric(PxLoad3(a), PxLoad3(b), PxLoad3(c), PxLoad3(d), p,
		v, w, x);
	const PxReal u = 1.f - v - w - x;
	const PxReal invMass = a.w * u + b.w * v + c.w * w + d.w * x;
	

	barycentric = make_float4(u, v, w, x);

	//After clamping the barycentrics might not sum up to 1 anymore
	//Clamping should be done because otherwise slightly negative barycentrics can lead to a negative inverse mass, e. g.
	//when 3 vertices of a tetrahedron have inverse mass zero and the forth vertex happens to get a negative barycentric coordinate
	barycentric.x = PxClamp(barycentric.x, 0.0f, 1.0f);
	barycentric.y = PxClamp(barycentric.y, 0.0f, 1.0f);
	barycentric.z = PxClamp(barycentric.z, 0.0f, 1.0f);
	barycentric.w = PxClamp(barycentric.w, 0.0f, 1.0f);

	return invMass;
}

__device__ PxReal computeInvMass(const uint4 tetrahedronIdx, const float4* position_invmass, const float4 barycentric)
{
	const float4 a = position_invmass[tetrahedronIdx.x];
	const float4 b = position_invmass[tetrahedronIdx.y];
	const float4 c = position_invmass[tetrahedronIdx.z];
	const float4 d = position_invmass[tetrahedronIdx.w];

	const PxReal invMass = a.w * barycentric.x + b.w * barycentric.y + c.w * barycentric.z + d.w * barycentric.w;

	return getSoftBodyInvMass(invMass, barycentric);
}

__device__ PxReal computeTriInvMass(const uint4 triVertIdx, const float4* position_invmass, const float4 barycentric)
{
	const float4 a = position_invmass[triVertIdx.x];
	const float4 b = position_invmass[triVertIdx.y];
	const float4 c = position_invmass[triVertIdx.z];

	const PxReal invMass = a.w * barycentric.x + b.w * barycentric.y + c.w * barycentric.z;

	return invMass;
}

__device__ float4 barycentricProjectTri(const uint4 triVertIdx, const float4* position_invmass, const float4 barycentric)
{
	const float4 a = position_invmass[triVertIdx.x];
	const float4 b = position_invmass[triVertIdx.y];
	const float4 c = position_invmass[triVertIdx.z];

	return a * barycentric.x + b * barycentric.y + c * barycentric.z;
}

__device__ float4 barycentricProject(const uint4 tetrahedronIdx, const float4* position_invmass, const float4 barycentric)
{
	const float4 a = position_invmass[tetrahedronIdx.x];
	const float4 b = position_invmass[tetrahedronIdx.y];
	const float4 c = position_invmass[tetrahedronIdx.z];
	const float4 d = position_invmass[tetrahedronIdx.w];

	return a * barycentric.x + b * barycentric.y + c * barycentric.z + d * barycentric.w;
}

__device__ inline void tetTriCollisionConstraint(PxVec3& cDx0, PxVec3& cDx1, PxVec3& cDx2, PxVec3& sDx0, PxVec3& sDx1,
                                                 PxVec3& sDx2, PxVec3& sDx3, float& lambdaN, float denomInv,
                                                 float4 normalPen, PxReal frictionCoefficient, const float4& cDelta0,
                                                 const float4& cDelta1, const float4& cDelta2, const float4& sDelta0,
                                                 const float4& sDelta1, const float4& sDelta2, const float4& sDelta3,
                                                 const float4& clothBC, const float4& softBodyBC)
{
	assert(denomInv != 0.f);
	const PxReal threshold = 1.0e-14f;

	PxReal pen;
	const PxVec3 normal = PxLoad3(normalPen, pen);

	PxReal cw0, cw1, cw2;
	PxReal sw0, sw1, sw2, sw3;
	const PxVec3 cd0 = PxLoad3(cDelta0, cw0);
	const PxVec3 cd1 = PxLoad3(cDelta1, cw1);
	const PxVec3 cd2 = PxLoad3(cDelta2, cw2);

	const PxVec3 sd0 = PxLoad3(sDelta0, sw0);
	const PxVec3 sd1 = PxLoad3(sDelta1, sw1);
	const PxVec3 sd2 = PxLoad3(sDelta2, sw2);
	const PxVec3 sd3 = PxLoad3(sDelta3, sw3);

	const PxVec3 relDelta =
	    (clothBC.x * cd0 + clothBC.y * cd1 + clothBC.z * cd2) -
	    (softBodyBC.x * sd0 + softBodyBC.y * sd1 + softBodyBC.z * sd2 + softBodyBC.w * sd3); // cloth - softbody

	//! collision constraint in normal direction
	const PxReal CN = pen + relDelta.dot(normal);

	PxReal deltaLambdaN = PxMax(-CN * denomInv, -lambdaN);
	lambdaN += deltaLambdaN;

	PxVec3 commonVec = deltaLambdaN * normal;

	//! friction constraint in tangent direction
	//! friction constraint is computed after collision constraint
	PxVec3 tanDir = (relDelta - relDelta.dot(normal) * normal);
	const PxReal tanMagSq = tanDir.magnitudeSquared();

	if(frictionCoefficient != 0.f && tanMagSq > threshold) // if tangential displacement is too little, friction is not
	                                                       // applied.
	{
		const PxReal tanMag = PxSqrt(tanMagSq);
		tanDir /= tanMag;
		PxReal CT = relDelta.dot(tanDir);

		PxReal deltaLambdaT = PxMax(0.f, CT * denomInv); // (-) sign is added in the next line
		deltaLambdaT = -PxMin(deltaLambdaT, frictionCoefficient * PxAbs(deltaLambdaN));

		assert(deltaLambdaT <= 0.f);

		const PxReal tanScale = 0.5f; // arbitary sacling factor.
		commonVec += tanScale * deltaLambdaT * tanDir;
	}

	cDx0 = cw0 * clothBC.x * commonVec;
	cDx1 = cw1 * clothBC.y * commonVec;
	cDx2 = cw2 * clothBC.z * commonVec;

	sDx0 = -sw0 * softBodyBC.x * commonVec;
	sDx1 = -sw1 * softBodyBC.y * commonVec;
	sDx2 = -sw2 * softBodyBC.z * commonVec;
	sDx3 = -sw3 * softBodyBC.w * commonVec;
}

__device__ inline void tetVertexCollisionConstraint(PxVec3& dx0, PxVec3& dx1, PxVec3& dx2, PxVec3& dx3, PxVec3& dx4,
                                                    float& lambdaN, float denomInv, float4 normalPen,
                                                    PxReal frictionCoefficient, const float4& delta0,
                                                    const float4& delta1, const float4& delta2, const float4& delta3,
                                                    const float4& delta4, const float4& bc)
{
	assert(denomInv != 0.f);
	const PxReal threshold = 1.0e-14f;

	PxReal pen;
	const PxVec3 normal = PxLoad3(normalPen, pen);

	PxReal w0, w1, w2, w3, w4;
	const PxVec3 d0 = PxLoad3(delta0, w0);
	const PxVec3 d1 = PxLoad3(delta1, w1);
	const PxVec3 d2 = PxLoad3(delta2, w2);
	const PxVec3 d3 = PxLoad3(delta3, w3);
	const PxVec3 d4 = PxLoad3(delta4, w4);

	const PxVec3 relDelta = d0 - (bc.x * d1 + bc.y * d2 + bc.z * d3 + bc.w * d4); // cloth - softbody

	//! collision constraint in normal direction
	const PxReal CN = pen + relDelta.dot(normal);

	PxReal deltaLambdaN = PxMax(-CN * denomInv, -lambdaN);
	lambdaN += deltaLambdaN;

	PxVec3 commonVec = deltaLambdaN * normal;

	//! friction constraint in tangent direction
	//! friction constraint is computed after collision constraint
	PxVec3 tanDir = (relDelta - relDelta.dot(normal) * normal);
	const PxReal tanMagSq = tanDir.magnitudeSquared();

	if (frictionCoefficient != 0.f && tanMagSq > threshold) // if tangential displacement is too little, friction is not applied.
	{
		const PxReal tanMag = PxSqrt(tanMagSq);
		tanDir /= tanMag;
		PxReal CT = relDelta.dot(tanDir);

		PxReal deltaLambdaT = PxMax(0.f, CT * denomInv); // (-) sign is added in the next line
		deltaLambdaT = -PxMin(deltaLambdaT, frictionCoefficient * PxAbs(deltaLambdaN));

		assert(deltaLambdaT <= 0.f);

		const PxReal tanScale = 0.5f; // arbitary scaling factor.
		commonVec += tanScale * deltaLambdaT * tanDir;
	}

	dx0 += w0 * commonVec;
	dx1 -= w1 * bc.x * commonVec;
	dx2 -= w2 * bc.y * commonVec;
	dx3 -= w3 * bc.z * commonVec;
	dx4 -= w4 * bc.w * commonVec;
}

extern "C" __global__ void sb_rigidContactPrepareLaunch(
	PxgSoftBody*					softbodies,
	float4*							contacts_restW,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemVsRigidContactInfo*		contactInfos,
	PxU32*							numContacts,
	PxgFemRigidConstraintBlock*		primitiveConstraints,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	float4*							rigidAppliedForces,
	float4*							softBodyAppliedForces,
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

		rigidAppliedForces[workIndex] = make_float4(0.f, 0.f, 0.f, 0.f);
		softBodyAppliedForces[workIndex] = make_float4(0.f, 0.f, 0.f, 0.f);

		PxgFemVsRigidContactInfo contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = primitiveConstraints[workIndex/32];

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

		const float4 delta_invMass1 = barycentricProject(tetrahedronIdx, delta_invMass, barycentric);
		const PxVec3 delta(delta_invMass1.x, delta_invMass1.y, delta_invMass1.z);

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal pen = normal_pen.w - rest;

		prepareFEMContacts(constraint, normal, sharedDesc, p, pen, delta, rigidId, barycentric, prepareDesc, solverBodyIndices, softbody.mPenBiasClamp, invDt, isTGS);
	}
}



extern "C" __global__ void sb_particleContactPrepareLaunch(
	PxgSoftBody*					softbodies,
	PxgParticleSystem*				particlesystems,
	float4*							contacts,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemContactInfo*				contactInfos,
	PxU32*							numContacts,
	PxgFEMParticleConstraintBlock*	spConstraints, //soft body particle constraint
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

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgFEMParticleConstraintBlock& constraint = spConstraints[workIndex/32];

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

		//get out the contact point
		const float4 contact = contacts[workIndex];
		const float4 normal_pen = normalPens[workIndex];

		/*printf("workIndex %i normal_pen(%f, %f, %f, %f)\n", workIndex, normal_pen.x, normal_pen.y,
		normal_pen.z, normal_pen.w);*/

		const PxVec3 p(contact.x, contact.y, contact.z);

		/*float4 barycentric;
		float invMass1 = computeInvMass(tetrahedronIdx, position_invmass, p, barycentric);*/
		float4 barycentric = barycentrics[workIndex];
		float4 tetDelta_invMass = barycentricProject(tetrahedronIdx, delta_invMassTet, barycentric);
		const PxVec3 tetDelta(tetDelta_invMass.x, tetDelta_invMass.y, tetDelta_invMass.z);

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

		PxgParticleSystem& particleSystem = particlesystems[tParticleSystemId];
		const float4 delta_invMass = particleSystem.mSortedDeltaP[tParticleIndex];
		const PxVec3 particleDelta(delta_invMass.x, delta_invMass.y, delta_invMass.z);
		const PxReal invMass0 = delta_invMass.w;
		const PxReal pen = normal_pen.w - (particleDelta - tetDelta).dot(normal) - softbody.mRestDistance;

		const PxReal sbMass = getSoftBodyInvMass(tetDelta_invMass.w, barycentric);

		const float unitResponse = invMass0 + sbMass;
		//KS - perhaps we don't need the > 0.f check here?
		const float velMultiplier = (unitResponse > 0.f) ? (1.f / unitResponse) : 0.f;

		//PxReal biasedErr = PxMin(-0.5f * pen * invDt, 5.f)*velMultiplier;
		//PxReal biasedErr = (-0.5f * pen * invDt)*velMultiplier;
		//printf("biasedErr %f, pen %f, invDt %f, velMultiplier %f\n", biasedErr, pen, invDt, velMultiplier);
		constraint.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		constraint.barycentric[threadIndexInWarp] = barycentric;
		constraint.velMultiplier[threadIndexInWarp] = velMultiplier;
	}
}


//contact prep for self collision/soft body vs soft body contacts
extern "C" __global__ void sb_softbodyContactPrepareLaunch(
	PxgSoftBody*						softbodies,
	float4*								contacts,
	float4*								normalPens,
	float4*								barycentrics0,
	float4*								barycentrics1,
	PxgFemContactInfo*					contactInfos,
	PxU32*								numContacts,
	PxgSoftBodySoftBodyConstraintBlock*	constraints,
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

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgSoftBodySoftBodyConstraintBlock& constraint = constraints[workIndex/32];

		PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
	
		//first pairInd is soft body
		PxgSoftBody& softbody0 = softbodies[PxGetSoftBodyId(pairInd0)];
		const PxU32 tetInd0 = PxGetSoftBodyElementIndex(pairInd0);

		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);

		//second pairInd is soft body
		PxgSoftBody& softbody1 = softbodies[PxGetSoftBodyId(pairInd1)];
		const PxU32 tetInd1 = PxGetSoftBodyElementIndex(pairInd1);

		const PxReal rest = softbody0.mRestDistance + softbody1.mRestDistance;

		const uint4 tetrahedronIdx0 = softbody0.mSimTetIndices[tetInd0];
		float4* position_invmass0 = softbody0.mSimPosition_InvMass;
		const uint4 tetrahedronIdx1 = softbody1.mSimTetIndices[tetInd1];
		float4* position_invmass1 = softbody1.mSimPosition_InvMass;

		const float4 normal_pen = normalPens[workIndex];

		const float4 barycentric0 = barycentrics0[workIndex];
		const float4 barycentric1 = barycentrics1[workIndex];
		float invMass0 = computeInvMass(tetrahedronIdx0, position_invmass0, barycentric0);
		float invMass1 = computeInvMass(tetrahedronIdx1, position_invmass1, barycentric1);
		
		/*const PxReal eps = 1.5f;
		if (PxAbs(barycentric0.x) > eps || PxAbs(barycentric0.y) > eps || PxAbs(barycentric0.z) > eps || PxAbs(barycentric0.w) > eps
			|| PxAbs(barycentric1.x) > eps || PxAbs(barycentric1.y) > eps || PxAbs(barycentric1.z) > eps || PxAbs(barycentric1.w) > eps)
		{
			printf("tetInd0 %i, barycentric(%f, %f, %f, %f)\n", tetInd0, barycentric0.x, barycentric0.y, barycentric0.z, barycentric0.w);
			printf("tetInd1 %i, barycentric(%f, %f, %f, %f)\n", tetInd1, barycentric1.x, barycentric1.y, barycentric1.z, barycentric1.w);
		}*/

		constraint.barycentric0[threadIndexInWarp] = barycentric0;
		constraint.barycentric1[threadIndexInWarp] = barycentric1;
		constraint.normal_pen[threadIndexInWarp] = make_float4(-normal_pen.x, -normal_pen.y, -normal_pen.z, -rest + normal_pen.w);
		//Avoid division by zero since cases where invMass0 and invMass1 are zero do exist. The contact generation kernel still generates a contact even if all nodes have invMass 0.
		constraint.velMultiplier[threadIndexInWarp] = 1.f / PxMax(1e-16f, invMass0 + invMass1); 
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
	PxgFemContactInfo*					contactInfos,
	PxU32*								numContacts,
	PxgSoftBodySoftBodyConstraintBlock*	constraints,
	float*								lambdaNs,
	const PxU32							maxContacts,
	PxsDeformableVolumeMaterialData*	softbodyMaterials,
	PxsDeformableSurfaceMaterialData*	clothMaterials
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

		// initialize lambdaN (accumulated delta lambdaN)
		lambdaNs[workIndex] = 0.f;

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgSoftBodySoftBodyConstraintBlock& constraint = constraints[workIndex / 32];

		// first actor: cloth vertex or triangle
		PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 clothId = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 triInd = PxGetClothElementIndex(pairInd0);

		const float4* PX_RESTRICT const clothAccumulatedDelta = cloth.mAccumulatedDeltaPos;
		const float4 clothDelta = clothAccumulatedDelta[triInd];

		const float4 clothBC = barycentrics0[workIndex];
		PxReal clothFriction = 0.f;
		PxReal clothDenom = 0.f;

		if (clothBC.w == 0.f) // colliding with cloth triangle
		{
			const PxU16 globalMaterialIndex = cloth.mMaterialIndices[triInd];
			clothFriction = clothMaterials[globalMaterialIndex].dynamicFriction;

			const uint4 triVertId = cloth.mTriangleVertexIndices[triInd];
			const PxReal clothW0 = clothAccumulatedDelta[triVertId.x].w;
			const PxReal clothW1 = clothAccumulatedDelta[triVertId.y].w;
			const PxReal clothW2 = clothAccumulatedDelta[triVertId.z].w;

			clothDenom =
			    clothBC.x * clothBC.x * clothW0 + clothBC.y * clothBC.y * clothW1 + clothBC.z * clothBC.z * clothW2;
		}
		else // colliding with cloth vertex
		{
			clothFriction = cloth.mDynamicFrictions[triInd];
			const PxReal clothW0 = clothAccumulatedDelta[triInd].w;

			clothDenom = clothW0;
		}

		// second actor: softbody tet
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetInd = PxGetSoftBodyElementIndex(pairInd1);
		const uint4 tetVertInd = softbody.mSimTetIndices[tetInd];		
		const float4 softBodyBC = barycentrics1[workIndex];

		const float4* PX_RESTRICT const softbodyAccumulatedDelta = softbody.mSimDeltaPos; 
		const float4 softbodyDelta = barycentricProject(tetVertInd, softbodyAccumulatedDelta, softBodyBC);

		const PxReal softBodyW0 = softbodyAccumulatedDelta[tetVertInd.x].w;
		const PxReal softBodyW1 = softbodyAccumulatedDelta[tetVertInd.y].w;
		const PxReal softBodyW2 = softbodyAccumulatedDelta[tetVertInd.z].w;
		const PxReal softBodyW3 = softbodyAccumulatedDelta[tetVertInd.w].w;

		const PxReal softBodyDenom = softBodyBC.x * softBodyBC.x * softBodyW0 + softBodyBC.y * softBodyBC.y * softBodyW1 +
		                             softBodyBC.z * softBodyBC.z * softBodyW2 + softBodyBC.w * softBodyBC.w * softBodyW3;

		const PxReal softbodyFriction = softbodyMaterials[softbody.mMaterialIndices[tetInd]].dynamicFriction;

		// common
		const PxReal thickness = cloth.mRestDistance + softbody.mRestDistance;
		const float4 normal_pen = normalPens[workIndex];
		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

		const PxReal pen = normal_pen.w - thickness - PxLoad3(clothDelta - softbodyDelta).dot(normal);
		const PxReal denom = clothDenom + softBodyDenom;

		const uint4 triVertId = cloth.mTriangleVertexIndices[triInd];
		float4 clothPos = clothBC.x * cloth.mPosition_InvMass[triVertId.x] +
		                  clothBC.y * cloth.mPosition_InvMass[triVertId.y] +
		                  clothBC.z * cloth.mPosition_InvMass[triVertId.z];
		float4 softBodyPos = softBodyBC.x * softbody.mSimPosition_InvMass[tetVertInd.x] +
		                     softBodyBC.y * softbody.mSimPosition_InvMass[tetVertInd.y] +
		                     softBodyBC.z * softbody.mSimPosition_InvMass[tetVertInd.z] +
		                     softBodyBC.w * softbody.mSimPosition_InvMass[tetVertInd.w];

		constraint.barycentric0[threadIndexInWarp] = clothBC;
		constraint.barycentric1[threadIndexInWarp] = softBodyBC;
		constraint.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		constraint.velMultiplier[threadIndexInWarp] = (denom == 0.f) ? 0.f : 1.f / denom;
		constraint.friction[threadIndexInWarp] = 0.5f * (clothFriction + softbodyFriction);
	}
}

#define kGlobalRelax  0.125f


//solve soft body vs particle contacts, store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSPDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particlesystems,
	PxgFemContactInfo*							contactInfos,
	PxgFEMParticleConstraintBlock*				constraints,
	PxU32*										numContacts,
	float2*										appliedForces,				//output
	const PxReal								biasCoefficient,
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

		/*if (workIndex == 0)
		{
		printf("tNumContacts %i\n", tNumContacts);
		}*/

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex/32];

		
		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		//first pairInd0 is particle
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);
		
		const float4 delta0 = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = delta0.w;
		
		if (invMass0 != 0)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			//const PxgCommonMaterial* PX_RESTRICT  psMaterials = particleSystem.mCommonMaterials;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;
			
			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 barycentric = constraint.barycentric[threadIndexInWarp];
			const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
			PxgSoftBody& softbody = softbodies[softbodyId];
			const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);
			const uint4 tetrahedronId1 = softbody.mSimTetIndices[tetId];

			const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
			const PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];
			float4 invMasses1;
			const float4 delta1 = computeTetraContact(softbody.mSimDeltaPos, tetrahedronId1, barycentric, invMasses1);

			const PxReal dynamicFriction0 = psMat.friction;
			const PxReal dynamicFriction1 = material.dynamicFriction;
			const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;
			const PxVec3 linDelta0(delta0.x, delta0.y, delta0.z);
			const PxVec3 linDelta1(delta1.x, delta1.y, delta1.z);

			const PxVec3 delta = linDelta1 - linDelta0;

			const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
			const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

			/*printf("workIndex %i baryCenteric(%f, %f, %f, %f)\n", baryCentric.x, baryCentric.y,
			baryCentric.z, baryCentric.w);*/

			/*if (workIndex == 0)
			printf("workIndex %i linVel(%f, %f, %f)\n", workIndex, linVel1.x, linVel1.y, linVel1.z);*/

			float2 appliedForce = appliedForces[workIndex];

			const PxReal normalDelta = delta.dot(normal);

			PxVec3 tanDir = delta - normal * normalDelta;
			const PxReal fricDelta = tanDir.normalize();

			const PxReal error = normal_pen.w + normalDelta;

			//const PxReal relaxation = kGlobalRelax;

											//KS - clamp the maximum force
											//Normal force can only be +ve!
			const float deltaF = PxMax(-appliedForce.x, -error * velMultiplier);/// *relaxation;
			//const float deltaF = PxMax(0.f, (biasedErr - normalVel) * velMultiplier);//*relaxation;
			appliedForce.x += deltaF;

			//printf("workIndex %i biasedErr = %f, normalVel = %f, deltaF = %f\n", workIndex, biasedErr, normalVel, deltaF);
			
			const PxReal friction = appliedForce.x* frictionCoefficient;

			//printf("tetId %i friction %f deltaF %f frictionCoefficient %f \n", tetId, friction, deltaF, frictionCoefficient);

			PxReal requiredForce = fricDelta * velMultiplier;

			/*printf("tetId %i requiredF0Force %f, requiredF1Force %f tanVel0 %f, tanVel1 %f, vmF0 %f, vmF1 %f \n", tetId, requiredF0Force, requiredF1Force, tanVel0, tanVel1,
			vmF0, vmF1);*/

			//requiredForce is always positive!
			PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
			appliedForce.y += deltaFr;

			const PxVec3 deltaPos = ((normal * deltaF) - tanDir * deltaFr)*biasCoefficient;

			//updateTetraPosDelta(invMasses0, barycentric0, tetrahedronId0, deltaPos, softbody0.mDelta);
			if (deltaF != 0.f || deltaFr != 0.f)
				updateTetraPosDelta(invMasses1, barycentric, tetrahedronId1, deltaPos, softbody.mSimDelta);
			

			appliedForces[workIndex] = appliedForce;
		}
	}
}



//solve soft body vs particle contacts, store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputParticleDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particlesystems,
	PxgFemContactInfo*							contactInfos,
	PxgFEMParticleConstraintBlock*				constraints,
	PxU32*										numContacts,
	float4*										deltaP,			//output
	float2*										appliedForces,	//output
	const PxReal								relaxation,
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

		/*if (workIndex == 0)
		{
		printf("tNumContacts %i\n", tNumContacts);
		}*/

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex/32];

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if (invMass0 != 0)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;
			
			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 barycentric = constraint.barycentric[threadIndexInWarp];
			const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
			PxgSoftBody& softbody = softbodies[softbodyId];
			const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);
			const uint4 tetrahedronId1 = softbody.mSimTetIndices[tetId];

			const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
			PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];
			float4 invMasses1;
			const float4 delta1 = computeTetraContact(softbody.mSimDeltaPos, tetrahedronId1, barycentric, invMasses1);

			const PxReal dynamicFriction0 = psMat.friction;
			const PxReal dynamicFriction1 = material.dynamicFriction;
			const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;
			const PxVec3 linDelta0(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			const PxVec3 linDelta1(delta1.x, delta1.y, delta1.z);

			const PxVec3 delta = linDelta1 - linDelta0;

			const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
			const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

			//printf("normal_pen(%f, %f, %f, %f)\n", normal_pen.x, normal_pen.y, normal_pen.z, normal_pen.w);

			/*if (workIndex == 0)
			printf("workIndex %i linVel(%f, %f, %f)\n", workIndex, linVel1.x, linVel1.y, linVel1.z);*/

			float2 appliedForce = appliedForces[workIndex];

			const float normalDelta = delta.dot(normal);

			PxVec3 tanDir = delta - normal * normalDelta;
			const PxReal fricDelta = tanDir.normalize();

			const PxReal error = normal_pen.w + normalDelta;

			//printf("%i: normal_penw = (%f, %f, %f, %f)\n", workIndex, normal_pen.x, normal_pen.y, normal_pen.z, normal_pen.w);

			//const PxReal relaxation = kGlobalRelax;

											//KS - clamp the maximum force
											//Normal force can only be +ve!
			const float deltaF = PxMax(-appliedForce.x, -error * velMultiplier);
			//const float deltaF = PxMax(0.f, (biasedErr - normalVel) * velMultiplier);//*relaxation;

			//printf("%i: BiasedErr = %f, normalVel = %f, velMultiplier = %f\n", workIndex, biasedErr, normalVel, velMultiplier);
			appliedForce.x += deltaF;

			//printf("workIndex %i particle %i: biasedErr = %f, normalVel = %f, deltaF = %f\n", workIndex, particleIndex, biasedErr, normalVel, deltaF);
			
			const PxReal friction = appliedForce.x* frictionCoefficient;

			//printf("tetId %i friction %f deltaF %f frictionCoefficient %f \n", tetId, friction, deltaF, frictionCoefficient);

			PxReal requiredForce = fricDelta * velMultiplier;

			/*printf("tetId %i requiredF0Force %f, requiredF1Force %f tanVel0 %f, tanVel1 %f, vmF0 %f, vmF1 %f \n", tetId, requiredF0Force, requiredF1Force, tanVel0, tanVel1,
			vmF0, vmF1);*/

			//requiredForce is always positive!
			PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
			appliedForce.y += deltaFr;

			const PxVec3 deltaV = ((-normal * deltaF) + tanDir * deltaFr) * invMass0 * relaxation;
			PxReal w = 0.f;
			if (deltaF != 0.f || deltaFr != 0.f)
				w = 1.f;

			deltaP[workIndex] = make_float4(deltaV.x, deltaV.y, deltaV.z, w);

			//updateTetraPosDelta(invMasses0, barycentric0, tetrahedronId0, deltaPos, softbody0.mDelta);
			//updateTetraPosDelta(invMasses1, barycentric, tetrahedronId1, -deltaPos, softbody.mDelta);

			appliedForces[workIndex] = appliedForce;
		}
	}
}


//solve soft bodies contacts, store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSSDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFemContactInfo*							contactInfos,
	PxgSoftBodySoftBodyConstraintBlock*			constraints,
	PxU32*										numContacts,
	const PxReal								dt,
	const PxReal								biasCoefficient,
	float2*										appliedForces,				//output
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

		/*if (workIndex == 0)
		{
			printf("tNumContacts %i\n", tNumContacts);
		}*/

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgSoftBodySoftBodyConstraintBlock& constraint = constraints[workIndex/32];


		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const float4 barycentric0 = constraint.barycentric0[threadIndexInWarp];
		const PxU32 softbodyId0 = PxGetSoftBodyId(pairInd0);
		PxgSoftBody& softbody0 = softbodies[softbodyId0];
		const PxU32 tetId0 = PxGetSoftBodyElementIndex(pairInd0);
		const uint4 tetrahedronId0 = softbody0.mSimTetIndices[tetId0];

		float4 invMasses0;
		const float4 pos0 = computeTetraContact(softbody0.mSimPosition_InvMass, tetrahedronId0, barycentric0, invMasses0);
		const float4 vel0 = computeTetraContact(softbody0.mSimVelocity_InvMass, tetrahedronId0, barycentric0, invMasses0);
		
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const float4 barycentric1 = constraint.barycentric1[threadIndexInWarp];
		const PxU32 softbodyId1 = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody1 = softbodies[softbodyId1];
		const PxU32 tetId1 = PxGetSoftBodyElementIndex(pairInd1);
		const uint4 tetrahedronId1 = softbody1.mSimTetIndices[tetId1];

		float4 invMasses1;
		const float4 pos1 = computeTetraContact(softbody1.mSimPosition_InvMass, tetrahedronId1, barycentric1, invMasses1);

		const float4 vel1 = computeTetraContact(softbody1.mSimVelocity_InvMass, tetrahedronId1, barycentric1, invMasses1);

		const PxU16 globalMaterialIndex0 = softbody0.mMaterialIndices[tetId0];
		const PxU16 globalMaterialIndex1 = softbody1.mMaterialIndices[tetId1];
		const PxsDeformableVolumeMaterialData& material0 = materials[globalMaterialIndex0];
		const PxsDeformableVolumeMaterialData& material1 = materials[globalMaterialIndex1];

		const PxReal dynamicFriction0 = material0.dynamicFriction;
		const PxReal dynamicFriction1 = material1.dynamicFriction;
		const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;
		const PxVec3 linVel0(vel0.x, vel0.y, vel0.z);
		const PxVec3 linVel1(vel1.x, vel1.y, vel1.z);

		const PxVec3 relVel = linVel1 - linVel0;

		const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
		const PxVec3 normal(normal_pen.x, normal_pen.y, normal_pen.z);
		const PxReal rest = normal_pen.w;

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		const float4 relPos = pos1 - pos0;

		const PxReal projectedErr = PxVec3(relPos.x, relPos.y, relPos.z).dot(normal) + rest;

		//printf("ProjectedErr = %f, err = %f\n", projectedErr, normal_pen.w);
		
		/*printf("workIndex %i baryCenteric(%f, %f, %f, %f)\n", baryCentric.x, baryCentric.y,
		baryCentric.z, baryCentric.w);*/

		/*if (workIndex == 0)
		printf("workIndex %i linVel(%f, %f, %f)\n", workIndex, linVel1.x, linVel1.y, linVel1.z);*/

		float2 appliedForce = appliedForces[workIndex];

		

		const PxReal normalVel = relVel.dot(normal);

		PxVec3 tanDir = relVel - normal * normalVel;
		const PxReal fricVel = tanDir.normalize();

		//const PxReal biasedErr = -normal_pen.w * biasCoefficient;
		const PxReal biasedErr = -projectedErr * biasCoefficient;

		//const PxReal relaxation = 0.75f;// kGlobalRelax;

		//KS - clamp the maximum force
		//Normal force can only be +ve!
		const float deltaF = PxMax(-appliedForce.x, (biasedErr - normalVel) * velMultiplier);// *relaxation;
		appliedForce.x += deltaF;
	

		const PxReal friction = appliedForce.x * frictionCoefficient;

		//printf("tetId %i friction %f deltaF %f frictionCoefficient %f \n", tetId, friction, deltaF, frictionCoefficient);

		PxReal requiredForce = fricVel * velMultiplier;

		/*printf("tetId %i requiredF0Force %f, requiredF1Force %f tanVel0 %f, tanVel1 %f, vmF0 %f, vmF1 %f \n", tetId, requiredF0Force, requiredF1Force, tanVel0, tanVel1,
		vmF0, vmF1);*/

		//requiredForce is always positive!
		PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
		appliedForce.y += deltaFr;

		if (deltaF != 0.f || deltaFr != 0.f)
		{
			const PxVec3 deltaPos = (-(normal * deltaF) + tanDir * deltaFr) * dt;

			updateTetraPosDelta(invMasses0, barycentric0, tetrahedronId0, deltaPos, softbody0.mSimDelta);
			updateTetraPosDelta(invMasses1, barycentric1, tetrahedronId1, -deltaPos, softbody1.mSimDelta);
		}
		
		appliedForces[workIndex] = appliedForce;
	}
}

//! 
//! \brief    : solve cloth vs. soft body collision
//! 

extern "C" __global__ void sb_solveOutputSCDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMCloth*								clothes,
	PxgFemContactInfo*							contactInfos,
	PxgSoftBodySoftBodyConstraintBlock*			constraints,
	PxU32*										numContacts,
	PxReal*										lambdaNs //output
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

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgSoftBodySoftBodyConstraintBlock& constraint = constraints[workIndex / 32];

		PxReal denomInv = constraint.velMultiplier[threadIndexInWarp]; // maybe we can remove velMultiplier from
		                                                               // constraint block, and compute it on the fly.
		if (denomInv == 0.f) continue;

		// first actor: cloth vertex (currently)
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const float4 clothBC = constraint.barycentric0[threadIndexInWarp];
		const PxU32 clothId = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 triId = PxGetClothElementIndex(pairInd0);

		// second actor: softbody tet
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const float4 softBodyBC = constraint.barycentric1[threadIndexInWarp];
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);
		const uint4 tetVertId = softbody.mSimTetIndices[tetId];

		float4 normal_pen = constraint.normal_pen[threadIndexInWarp];

		const float4* PX_RESTRICT const clothAccumDelta = cloth.mAccumulatedDeltaPos;

		const float4* PX_RESTRICT const softBodyAccumDelta = softbody.mSimDeltaPos;
		const float4 softBodyDelta0 = softBodyAccumDelta[tetVertId.x];
		const float4 softBodyDelta1 = softBodyAccumDelta[tetVertId.y];
		const float4 softBodyDelta2 = softBodyAccumDelta[tetVertId.z];
		const float4 softBodyDelta3 = softBodyAccumDelta[tetVertId.w];

		PxVec3 softBodyDx0(0.f), softBodyDx1(0.f), softBodyDx2(0.f), softBodyDx3(0.f);

		const PxReal frictionCoefficient = constraint.friction[threadIndexInWarp];		
		PxReal lambaN = lambdaNs[workIndex];

		if (clothBC.w == 0.f) // colliding with cloth triangle
		{
			PxVec3 clothDx0(0.f), clothDx1(0.f), clothDx2(0.f);
			const uint4 triVertId = cloth.mTriangleVertexIndices[triId];

			const float4 clothDelta0 = clothAccumDelta[triVertId.x];
			const float4 clothDelta1 = clothAccumDelta[triVertId.y];
			const float4 clothDelta2 = clothAccumDelta[triVertId.z];

			tetTriCollisionConstraint(clothDx0, clothDx1, clothDx2, softBodyDx0, softBodyDx1, softBodyDx2,
				softBodyDx3, lambaN, denomInv, normal_pen, frictionCoefficient, clothDelta0,
				clothDelta1, clothDelta2, softBodyDelta0, softBodyDelta1, softBodyDelta2,
				softBodyDelta3, clothBC, softBodyBC);

			if(clothDelta0.w != 0.f && clothBC.x > 1e-6f)
				AtomicAdd(cloth.mDeltaPos[triVertId.x], clothDx0, 1.f);

			if(clothDelta1.w != 0.f && clothBC.y > 1e-6f)
				AtomicAdd(cloth.mDeltaPos[triVertId.y], clothDx1, 1.f);

			if(clothDelta2.w != 0.f && clothBC.z > 1e-6f)
				AtomicAdd(cloth.mDeltaPos[triVertId.z], clothDx2, 1.f);

			if(softBodyDelta0.w != 0.f && softBodyBC.x > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.x], softBodyDx0, 1.f);

			if(softBodyDelta1.w != 0.f && softBodyBC.y > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.y], softBodyDx1, 1.f);

			if(softBodyDelta2.w != 0.f && softBodyBC.z > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.z], softBodyDx2, 1.f);

			if(softBodyDelta3.w != 0.f && softBodyBC.w > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.w], softBodyDx3, 1.f);
		}
		else // colliding with cloth vertex
		{
			PxVec3 clothDx0(0.f);
			const float4 clothDelta0 = clothAccumDelta[triId];

			tetVertexCollisionConstraint(clothDx0, softBodyDx0, softBodyDx1, softBodyDx2, softBodyDx3, lambaN, denomInv,
			                             normal_pen, frictionCoefficient, clothDelta0, softBodyDelta0, softBodyDelta1,
			                             softBodyDelta2, softBodyDelta3, softBodyBC);

			if(clothDelta0.w != 0.f)
				AtomicAdd(cloth.mDeltaPos[triId], clothDx0, 1.f);

			if(softBodyDelta0.w != 0.f && softBodyBC.x > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.x], softBodyDx0, 1.f);

			if(softBodyDelta1.w != 0.f && softBodyBC.y > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.y], softBodyDx1, 1.f);

			if(softBodyDelta2.w != 0.f && softBodyBC.z > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.z], softBodyDx2, 1.f);

			if(softBodyDelta3.w != 0.f && softBodyBC.w > 1e-6f)
				AtomicAdd(softbody.mSimDelta[tetVertId.w], softBodyDx3, 1.f);
		}

		lambdaNs[workIndex] = lambaN;
	}
}

//solve soft bodies contacts, store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSSDeltaVLaunchTGS(
	PxgSoftBody*								softbodies,
	PxgFemContactInfo*							contactInfos,
	PxgSoftBodySoftBodyConstraintBlock*			constraints,
	PxU32*										numContacts,
	const PxReal								dt,
	const PxReal								biasCoefficient,
	float2*										appliedForces,			//output
	PxsDeformableVolumeMaterialData*			materials
)
{

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	//const PxReal invDt = 1.0f / dt;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		/*if (workIndex == 0)
		{
		printf("tNumContacts %i\n", tNumContacts);
		}*/

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgSoftBodySoftBodyConstraintBlock& constraint = constraints[workIndex / 32];


		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const float4 barycentric0 = constraint.barycentric0[threadIndexInWarp];
		const PxU32 softbodyId0 = PxGetSoftBodyId(pairInd0);
		PxgSoftBody& softbody0 = softbodies[softbodyId0];
		const PxU32 tetId0 = PxGetSoftBodyElementIndex(pairInd0);
		const uint4 tetrahedronId0 = softbody0.mSimTetIndices[tetId0];

		float4 invMasses0;
		const float4 pos0 = computeTetraContact(softbody0.mSimDeltaPos, tetrahedronId0, barycentric0, invMasses0);

		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const float4 barycentric1 = constraint.barycentric1[threadIndexInWarp];
		const PxU32 softbodyId1 = PxGetSoftBodyId(pairInd1);
		PxgSoftBody& softbody1 = softbodies[softbodyId1];
		const PxU32 tetId1 = PxGetSoftBodyElementIndex(pairInd1);
		const uint4 tetrahedronId1 = softbody1.mSimTetIndices[tetId1];

		float4 invMasses1;
		const float4 pos1 = computeTetraContact(softbody1.mSimDeltaPos, tetrahedronId1, barycentric1, invMasses1);

		const PxU16 globalMaterialIndex0 = softbody0.mMaterialIndices[tetId0];
		const PxU16 globalMaterialIndex1 = softbody1.mMaterialIndices[tetId1];

		const PxsDeformableVolumeMaterialData& material0 = materials[globalMaterialIndex0];
		const PxsDeformableVolumeMaterialData& material1 = materials[globalMaterialIndex1];
		const PxReal dynamicFriction0 = material0.dynamicFriction;
		const PxReal dynamicFriction1 = material1.dynamicFriction;
		const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;

		const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
		const PxVec3 normal(normal_pen.x, normal_pen.y, normal_pen.z);
		const PxReal rest = normal_pen.w;

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		const float4 relPos = pos1 - pos0;

		const PxReal projectedErr = PxVec3(relPos.x, relPos.y, relPos.z).dot(normal) + rest;

		//printf("ProjectedErr = %f, err = %f\n", projectedErr, normal_pen.w);

		/*printf("workIndex %i baryCenteric(%f, %f, %f, %f)\n", baryCentric.x, baryCentric.y,
		baryCentric.z, baryCentric.w);*/

		/*if (workIndex == 0)
		printf("workIndex %i linVel(%f, %f, %f)\n", workIndex, linVel1.x, linVel1.y, linVel1.z);*/

		float2 appliedForce = appliedForces[workIndex];

		const PxVec3 rPos(relPos.x, relPos.y, relPos.z);

		const PxReal normalPos = rPos.dot(normal);

		PxVec3 tanDir = rPos - normal * normalPos;
		const PxReal tanDelta = tanDir.normalize();

		//const PxReal biasedErr = -normal_pen.w * biasCoefficient;
		const PxReal biasedErr = -projectedErr;

		//const PxReal relaxation = 0.75f;// kGlobalRelax;

		//KS - clamp the maximum force
		//Normal force can only be +ve!
		const float deltaF = PxMax(-appliedForce.x, (biasedErr)* velMultiplier);// *relaxation;
		appliedForce.x += deltaF;

		
		const PxReal friction = appliedForce.x* frictionCoefficient;

		//printf("tetId %i friction %f deltaF %f frictionCoefficient %f \n", tetId, friction, deltaF, frictionCoefficient);

		PxReal requiredForce = tanDelta * velMultiplier;

		/*printf("tetId %i requiredF0Force %f, requiredF1Force %f tanVel0 %f, tanVel1 %f, vmF0 %f, vmF1 %f \n", tetId, requiredF0Force, requiredF1Force, tanVel0, tanVel1,
		vmF0, vmF1);*/

		//requiredForce is always positive!
		PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
		appliedForce.y += deltaFr;

		if (deltaF != 0.f || deltaFr != 0.f)
		{
			const PxVec3 deltaPos = (-(normal * deltaF) + tanDir * deltaFr) *biasCoefficient * dt;

			updateTetraPosDelta(invMasses0, barycentric0, tetrahedronId0, deltaPos, softbody0.mSimDelta);
			updateTetraPosDelta(invMasses1, barycentric1, tetrahedronId1, -deltaPos, softbody1.mSimDelta);
		}
		
		appliedForces[workIndex] = appliedForce;
	}
}

template <typename IterativeData>
static __device__ void queryRigidSoftBodyContactReferenceCount(
	PxgSoftBody* softbodies,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemVsRigidContactInfo& contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: soft body
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

		if(tetId < PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			PxgSoftBody& softbody = softbodies[softbodyId];

			const PxVec4 bcVec(bc.x, bc.y, bc.z, bc.w);
			PxU32 globalRigidBodyId;
			bool isActive;
			PxVec4 femInvMass;

			if(isTGS)
			{
				FEMCollisionTGS<PxVec4> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, NULL, NULL);
				femCollision.readSoftBody(softbody, tetId, bc, NULL, true); // Friction info not required when querying reference counts
				isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt,
												   bcVec, true);
				globalRigidBodyId = femCollision.globalRigidBodyId;
				femInvMass = femCollision.deformableVertexInvMasses;
			}
			else
			{
				FEMCollisionPGS<PxVec4> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, NULL, NULL);
				femCollision.readSoftBody(softbody, tetId, bc, NULL, true); // Friction info not required when querying reference counts
				isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt,
												   bcVec, true);
				globalRigidBodyId = femCollision.globalRigidBodyId;
				femInvMass = femCollision.deformableVertexInvMasses;
			}

			if(isActive)
			{
				// Update soft body
				{
					// Increment the reference count of the four vertices.
					const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
					if(femInvMass.x > 0.0f && PxAbs(bc.x) > 1e-6f)
						atomicAdd(&softbody.mSimDelta[tetrahedronId.x].w, 1.0f);

					if(femInvMass.y > 0.0f && PxAbs(bc.y) > 1e-6f)
						atomicAdd(&softbody.mSimDelta[tetrahedronId.y].w, 1.0f);

					if(femInvMass.z > 0.0f && PxAbs(bc.z) > 1e-6f)
						atomicAdd(&softbody.mSimDelta[tetrahedronId.z].w, 1.0f);

					if(femInvMass.w > 0.0f && PxAbs(bc.w) > 1e-6f)
						atomicAdd(&softbody.mSimDelta[tetrahedronId.w].w, 1.0f);
				}

				// Update rigidbody
				if(!rigidId.isStaticBody() && fricTan0_invMass0.w != 0.0f)
				{
					// Increment the reference count of the rigid body.
					atomicAdd(&rigidBodyReferenceCounts[globalRigidBodyId], 1);
				}
			}
		}
	}
}

extern "C" __global__
void sb_queryRigidSoftContactReferenceCountLaunch(
	PxgSoftBody* softbodies,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = false;
	queryRigidSoftBodyContactReferenceCount(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
											sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}

template <typename IterativeData>
static __device__ void solveRigidSoftBodyContact(
	PxgSoftBody* softbodies,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	float4* rigidDeltaVel,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts,
	const PxReal dt,
	PxsDeformableVolumeMaterialData* materials,
	bool updateRigid,
	const PxsMaterialData* PX_RESTRICT rigidBodyMaterials,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemVsRigidContactInfo& contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: soft body
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 softbodyId = PxGetSoftBodyId(pairInd1);
		const PxU32 tetId = PxGetSoftBodyElementIndex(pairInd1);

		if(tetId < PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			PxgSoftBody& softbody = softbodies[softbodyId];
			const PxVec4 bcVec(bc.x, bc.y, bc.z, bc.w);

			if(isTGS)
			{
				FEMCollisionTGS<PxVec4> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, rigidBodyReferenceCounts,
										   &rigidBodyMaterials[contactInfo.rigidMatInd]);
				femCollision.readSoftBody(softbody, tetId, bc, materials, false);
				const bool isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId,
															  velocityReader, dt, bcVec, false);

				if(updateRigid) // Update rigid body
				{
					PxVec3 deltaLinVel0, deltaAngVel0;
					appliedForces[workIndex] = femCollision.computeRigidChange(deltaLinVel0, deltaAngVel0, fricTan0_invMass0.w, rigidId);
					femCollision.writeRigidBody(rigidDeltaVel, deltaLinVel0, deltaAngVel0, isActive, workIndex, workIndex + tNumContacts);
				}
				else if(isActive) // Update soft body
				{
					PxVec3 deltaPos;
					appliedForces[workIndex] = femCollision.computeFEMChange(deltaPos, dt);
					femCollision.writeSoftBody(softbody, tetId, bc, deltaPos);
				}
			}
			else // PGS
			{
				FEMCollisionPGS<PxVec4> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, rigidBodyReferenceCounts,
										   &rigidBodyMaterials[contactInfo.rigidMatInd]);
				femCollision.readSoftBody(softbody, tetId, bc, materials, false);
				const bool isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId,
															  velocityReader, dt, bcVec, false);

				if(updateRigid) // Update rigid body
				{
					PxVec3 deltaLinVel0, deltaAngVel0;
					appliedForces[workIndex] = femCollision.computeRigidChange(deltaLinVel0, deltaAngVel0, fricTan0_invMass0.w, rigidId);
					femCollision.writeRigidBody(rigidDeltaVel, deltaLinVel0, deltaAngVel0, isActive, workIndex, workIndex + tNumContacts);
				}
				else if(isActive) // Update soft body
				{
					PxVec3 deltaPos;
					appliedForces[workIndex] = femCollision.computeFEMChange(deltaPos, dt);
					femCollision.writeSoftBody(softbody, tetId, bc, deltaPos);
				}
			}
		}
	}
}


//solve collision between softbody and primitives
//store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSRDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFemVsRigidContactInfo*							contactInfos,
	PxgFemRigidConstraintBlock*					constraints,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	float4*										rigidDeltaVel,				//output
	float4*										appliedForces,				//output
	PxU32*										rigidBodyReferenceCounts,
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = false; // Update soft body
	const bool isTGS = false;

	solveRigidSoftBodyContact(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc,
							  rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, dt, materials, updateRigid, rigidBodyMaterials, isTGS);
}

//solve collision between soft body and primitives based on the sorted contact by rigid id
//store new velocity to rigid body buffer
extern "C" __global__ void sb_solveOutputRigidDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFemVsRigidContactInfo*					contactInfos,
	PxgFemRigidConstraintBlock*					constraints,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	float4*										rigidDeltaVel,				//output
	float4*										appliedForces,				//output
	PxU32*										rigidBodyReferenceCounts,
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = true;
	const bool isTGS = false;

	solveRigidSoftBodyContact(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc,
		rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, dt, materials, updateRigid, rigidBodyMaterials, isTGS);
}


///////////////////////

//TGS
extern "C" __global__
void sb_queryRigidSoftContactReferenceCountLaunchTGS(
	PxgSoftBody* softbodies,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = true;
	queryRigidSoftBodyContactReferenceCount(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
		sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}


//solve collision between softbody and primitives
//store positional change to soft body buffer
extern "C" __global__ void sb_solveOutputSRDeltaVLaunchTGS(
	PxgSoftBody*								softbodies,
	PxgFemVsRigidContactInfo*							contactInfos,
	PxgFemRigidConstraintBlock*					constraints,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>*	sharedDesc,
	float4*										rigidDeltaVel,				//output
	float4*										appliedForces,				//output
	PxU32*										rigidBodyReferenceCounts,
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials,
	const PxsMaterialData * PX_RESTRICT			rigidBodyMaterials)
{
	const bool updateRigid = false; // Update soft body
	const bool isTGS = true;

	solveRigidSoftBodyContact(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc,
		rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, dt, materials, updateRigid, rigidBodyMaterials, isTGS);
}

//solve collision between soft body and primitives based on the sorted contact by rigid id
//store new velocity to rigid body buffer
extern "C" __global__ void sb_solveOutputRigidDeltaVLaunchTGS(
	PxgSoftBody*								softbodies,
	PxgFemVsRigidContactInfo*							contactInfos,
	PxgFemRigidConstraintBlock*					constraints,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	float4*										rigidDeltaVel,				//output
	float4*										appliedForces,				//output
	PxU32*										rigidBodyReferenceCounts,
	const PxReal								dt,
	PxsDeformableVolumeMaterialData*			materials,
	const PxsMaterialData * PX_RESTRICT			rigidBodyMaterials
)
{
	const bool updateRigid = true;
	const bool isTGS = true;

	solveRigidSoftBodyContact(softbodies, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc,
		rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, dt, materials, updateRigid, rigidBodyMaterials, isTGS);
}
