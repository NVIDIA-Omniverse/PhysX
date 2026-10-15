// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxQuat.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxHeightFieldSample.h"
#include "geometry/PxMeshScale.h"

#include "GuDistancePointTriangle.h"
#include "GuDistancePointTetrahedron.h"

#include "PxNodeIndex.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgDeformableContactWriters.h"
#include "PxgFEMCloth.h"
#include "PxgNpKernelIndices.h"
#include "PxgParticleSystem.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSoftBody.h"

#include "PxsCachedTransform.h"

#include "convexNpCommon.h"
#include "cudaNpCommon.h"

#include <vector_types.h>

#include "dataReadWriteHelper.cuh"
#include "heightfieldUtil.cuh"
#include "deformableElementFilter.cuh"
#include "vector.cuh"

#include "utils.cuh"
#include "capsuleTriangle.cuh"
#include "deformableCollision.cuh"

using namespace schlock;
#include "gjk.cuh"
#include "epa.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels14() {}

struct sbTetDataScratch
{
	const float4 * PX_RESTRICT tetmeshVerts;
	const uint4 * PX_RESTRICT tetmeshTetIndices;
};

static __device__ bool computeTriangleBarycentric(PxVec3* verts, const PxVec3 p, float4& barycentric)
{
	const PxReal minEps = -1e-5f;
	const PxReal maxEps = 1.f + 1e-5f;
	const PxVec3 a = verts[0];
	const PxVec3 v0 = verts[1] - a;
	const PxVec3 v1 = verts[2] - a;
	const PxVec3 v2 = p - a;

	const PxReal d00 = v0.dot(v0);// V3Dot(v0, v0);
	const PxReal d01 = v0.dot(v1);// V3Dot(v0, v1);
	const PxReal d11 = v1.dot(v1);// V3Dot(v1, v1);
	const PxReal d20 = v2.dot(v0);// V3Dot(v2, v0);
	const PxReal d21 = v2.dot(v1);// V3Dot(v2, v1);
	const PxReal denom =1.f / (d00 * d11 - d01 * d01); // FRecip(FSub(FMul(d00, d11), FMul(d01, d01)));
	const PxReal v = (d11 * d20 - d01 * d21) * denom; //  FMul(FSub(FMul(d11, d20), FMul(d01, d21)), denom);
	const PxReal w = (d00 * d21 - d01 * d20) * denom; //  FMul(FSub(FMul(d00, d21), FMul(d01, d20)), denom);

	const PxReal u = 1.f - v - w;

	barycentric.x = u; barycentric.y = v; barycentric.z = w; barycentric.w = 0.f;

	return (u >= minEps && u <= maxEps) &&
		(v >= minEps && v <= maxEps) &&
		(w >= minEps && w <= maxEps);
}

__device__ bool isTetrahedronValid(PxVec3* verts)
{
	const PxVec3 a = verts[0];
	const PxVec3 ba = verts[1] - a;
	const PxVec3 ca = verts[2] - a;
	const PxVec3 da = verts[3] - a;

	//determinant
	const PxReal detBcd = ba.dot(ca.cross(da));

	return (PxAbs(detBcd) > 1e-6f);
}

__device__ static inline PxReal computeInnerSphere(const PxVec3& v0, const PxVec3& v1, 
	const PxVec3& v2, const PxVec3& v3, PxVec3& tetCenter)
{
	const PxVec3 center = (v0 + v1 + v2 + v3) * 0.25f;
	tetCenter = center;
	//plane n = (p1 - p0).cross(p2 - p0).getNormalized();


	//0 2 1
	PxVec3 n = (v2 - v0).cross(v1 - v0).getNormalized();
	PxReal innerSphere = PxAbs((center - v0).dot(n));

	
	{
		//1 2 3
		n = (v2 - v1).cross(v3 - v1).getNormalized();
		PxReal d = (center - v1).dot(n);
		innerSphere = PxMin(PxAbs(d), innerSphere);
	}

	//0 1 3
	{
		n = (v1 - v0).cross(v3 - v0).getNormalized();
		PxReal d = (center - v0).dot(n);
		innerSphere = PxMin(PxAbs(d), innerSphere);
	}

	//0 3 2
	{
		n = (v3 - v0).cross(v2 - v0).getNormalized();
		PxReal d = (center - v0).dot(n);
		innerSphere = PxMin(PxAbs(d), innerSphere);
	}

	return innerSphere;
}

__device__ PxVec3 computeSurfaceNormal(const PxVec3& v0, const PxVec3& v1,
	const PxVec3& v2, const PxVec3& v3, const PxVec3& dir, const PxU8 surfaceHint)
{
	//0111 - (0, 1, 2)
	PxVec3 n;
	PxReal bestD = -PX_MAX_F32;
	if (surfaceHint & (1 << 0))
	{
		n = (v2 - v0).cross(v2 - v1).getNormalized();

		PxReal d = (v3 - v0).dot(n);
		//flip normal
		if (d > 0.f)
			n = -n;

		bestD = dir.dot(n);
	}
	
	if (surfaceHint & (1<<1))
	{
		//1011 - (0, 1, 3)

		PxVec3 nor = (v3 - v0).cross(v3 - v1).getNormalized();
		PxReal d = (v3 - v0).dot(nor);
		if (d > 0.f)
			nor = -nor;
		PxReal absD = dir.dot(nor);

		if (absD > bestD)
		{
			//flip normal
			bestD = absD;
			n = nor;
		}
	}
	if (surfaceHint & (1<<2))
	{
		//1101 -(0, 2, 3)

		PxVec3 nor = (v3 - v0).cross(v3 - v2).getNormalized();
		PxReal d = (v3 - v0).dot(n);
		if (d > 0.f)
			nor = -nor;
		PxReal absD = dir.dot(nor);
		if (absD > bestD)
		{
			//flip normal
			
			bestD = absD;
			n = nor;
		}
	}
	
	if (surfaceHint & (1 << 3))
	{
		//1110 - (1, 2, 3)

		PxVec3 nor = (v3 - v1).cross(v3 - v2).getNormalized();
		PxReal d = (v3 - v1).dot(n);
		if (d > 0.f)
			nor = -nor;
		PxReal absD = dir.dot(nor);
		if (absD > bestD)
		{
			//flip normal
			if (d > 0.f)
				n = -n;
			n = nor;
		}
	}

	return n;

}

__device__ static inline PxReal computeInnerSphere(const float v, PxVec3& tetCenter)
{
	//12 threads has value but we use the full mask
	const PxU32 mask = FULL_MASK;
	const PxVec3 v0(__shfl_sync(mask, v, 0), __shfl_sync(mask, v, 1), __shfl_sync(mask, v, 2));
	const PxVec3 v1(__shfl_sync(mask, v, 3), __shfl_sync(mask, v, 4), __shfl_sync(mask, v, 5));
	const PxVec3 v2(__shfl_sync(mask, v, 6), __shfl_sync(mask, v, 7), __shfl_sync(mask, v, 8));
	const PxVec3 v3(__shfl_sync(mask, v, 9), __shfl_sync(mask, v, 10), __shfl_sync(mask, v, 11));

	return computeInnerSphere(v0, v1, v2, v3, tetCenter);
}

__device__ static inline void tetPlaneCollide(
	TetCollideScratch&			ss_scratch,
	const PxNodeIndex			rigidId,
	const PxU32					softbodyId,
	const PxU32					tetrahedronIdx,
	PxTransform&				planeTransform,
	const PxReal				contactDist,
	const PxReal				restDist,
	PxgRigidDbContactWriter&	writer,
	PxU32						rigidBodyMaterialId
)
{
	const PxU32 idxInWarp = threadIdx.x;

	bool intersect = false;
	PxVec3 worldPoint;
	PxVec3 worldNormal;
	PxReal pen;
	//a tetrahedron has 4 points
	if(idxInWarp < 4)
	{
		PxVec3 pInPlaneSpace = planeTransform.transformInv(ss_scratch.vB[idxInWarp]);

		//if (blockIdx.x == 0 && warpIndex == 0)
			//printf("pInPlaneSpace[%i](%f, %f, %f)\n", idxInWarp, pInPlaneSpace.x, pInPlaneSpace.y, pInPlaneSpace.z);

		if (contactDist >= pInPlaneSpace.x)
		{
			intersect = true;

			worldPoint = ss_scratch.vB[idxInWarp];
			//get the plane normal
			worldNormal = planeTransform.q.getBasisVector0();

			pen = pInPlaneSpace.x;
		}
	}

	const PxU32 index = globalScanExclusiveSingleWarp(intersect, writer.totalContactCount);

	if (intersect && index < writer.maxNumContacts)
	{
		PxU64 rigidRef = rigidId.getInd();
		PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronIdx);

		writer.writeVolumeContact(index, make_float4(worldPoint.x, worldPoint.y, worldPoint.z, restDist), make_float4(worldNormal.x, worldNormal.y, worldNormal.z, pen),
			rigidRef, rigidBodyMaterialId, tetRef);
	}
}

// Full warp equivalent of PxgRigidDbContactWriter::writeVolumeContact, and it claims the contact index
// rather than taking one. It is not a method because the two float4 stores are split across four lanes and
// the writer header is also compiled for the host.
PX_FORCE_INLINE __device__ void writeVolumeContactFullWarp(PxgRigidDbContactWriter& writer, bool intersect, const float* closestPointA, const float* closestPointB, PxReal penOffset,
	const PxReal* normalPtr, bool flipOutputNormal, PxReal restDist, PxU64 rigidRef, PxU32 rigidBodyMaterialId, PxU64 tetRef)
{
	int32_t index = 0xFFffFFff;
	if (threadIdx.x == 0 && intersect)
	{
		//printf("cmIdx %i tetrahedronIdx %i globalWarpIndex %i\n", cmIdx, tetrahedronIdx, globalWarpIndex);

		index = atomicAdd(writer.totalContactCount, 1);

		if (index >= writer.maxNumContacts)
			index = 0xFFffFFff;
		//printf("%i: StartIndex = %i nbToLock = %i\n", workIndex, startIndex, nbToLock);
	}

	index = __shfl_sync(FULL_MASK, index, 0);
	if (index != 0xFFffFFff)
	{
		writer.writeSortKey(index, rigidRef);

		PxReal witnessA = 0.f;
		PxReal witnessB = 0.f;
		PxReal normal = 0.f;
		if (threadIdx.x < 3)
		{
			witnessA = closestPointA[threadIdx.x];
			witnessB = closestPointB[threadIdx.x];
			normal = normalPtr[threadIdx.x];
		}
		PxReal pen = (witnessB - witnessA) * normal;

		pen = __shfl_sync(FULL_MASK, pen, 0) + __shfl_sync(FULL_MASK, pen, 1) + __shfl_sync(FULL_MASK, pen, 2) + penOffset;

		if (threadIdx.x < 4)
		{
			float* point = reinterpret_cast<float*>(&writer.outPoint[index]);
			float* normalPen = reinterpret_cast<float*>(&writer.outNormalPen[index]);

			const PxReal outP = threadIdx.x < 3 ? witnessB : restDist;
			point[threadIdx.x] = outP;

			const PxReal outNP = threadIdx.x < 3 ? (flipOutputNormal ? -normal: normal) : pen;
			normalPen[threadIdx.x] = outNP;

			if (threadIdx.x == 0)
			{
				PxgDeformablePairInfo* ptr = &writer.outContactInfo[index];
				ptr->ref0 = rigidRef;
				ptr->ref1 = tetRef;
				writer.outRigidMaterialIndex[index] = PxU16(rigidBodyMaterialId);
			}
		}
	}
}

__device__ static inline void sbPrimitiveCollision(
	PxU32										globalWarpIndex,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4									curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	sbTetDataScratch*							s_warpScratch,
	squawk::EpaScratch&							ss_epa_scratch,
	TetCollideScratch&							ss_scratch,
	PxgRigidDbContactWriter&					writer
	)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 tetrahedronIdx = curPair.y;

	/*if(threadIdx.x == 0)
		printf("cmIdx %i tetrahedronIdx %i\n", cmIdx, tetrahedronIdx);*/

	PxgShape softbodyShape, rigidShape;
	PxU32 softbodyCacheRef, rigidCacheRef;
	LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH>(cmInputs, cmIdx, gpuShapes,
		softbodyShape, softbodyCacheRef, rigidShape, rigidCacheRef);

	PxGeometryType::Enum rigidType = PxGeometryType::Enum(rigidShape.type);

	PxsCachedTransform rigidTransformCache;
	PxsCachedTransform_ReadWarp(rigidTransformCache, transformCache + rigidCacheRef);

	PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[rigidCacheRef];
	const PxReal restDist = restDistance[cmIdx];

	const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];
	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];

	// A filter pair for this rigid and tetrahedron suppresses contact generation between them.
	if (findRigidVolumeFilter(softbody.mRigidVolumeFilterPairs, softbody.mNumRigidVolumeFilterPairs,
		rigidId, rigidCacheRef, softbodyId, tetrahedronIdx))
		return;

	if (threadIdx.x == 0)
	{
		s_warpScratch->tetmeshVerts = softbody.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices = softbody.mTetIndices;
	}

	__syncwarp();

	const uint4 tetIdx = s_warpScratch->tetmeshTetIndices[tetrahedronIdx];

	const uint* tetIdxf = reinterpret_cast<const uint*>(&tetIdx);
	float v;
	if (threadIdx.x < 12)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = tetIdxf[elemId]; // tetIdx.x, tetIdx.y, tetIdx.z, tetIdx.w

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[index]);
		
		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];

		//load the verts into shared memory
		float* vB = reinterpret_cast<float*>(&ss_scratch.vB);
		vB[threadIdx.x] = v;
	}

	/*float v;
	if (threadIdx.x == 0)
	{
		const float4* verts = s_warpScratch->tetmeshVerts;

		const float4 v0 = verts[tetIdx.x];
		ss_scratch.vB[0] = PxVec3(v0.x, v0.y, v0.z);

		const float4 v1 = verts[tetIdx.y];
		ss_scratch.vB[1] = PxVec3(v1.x, v1.y, v1.z);

		const float4 v2 = verts[tetIdx.z];
		ss_scratch.vB[2] = PxVec3(v2.x, v2.y, v2.z);

		const float4 v3 = verts[tetIdx.w];
		ss_scratch.vB[3] = PxVec3(v3.x, v3.y, v3.z);
	}*/

	__syncwarp();
	
	if (rigidType == PxGeometryType::ePLANE)
	{
		
		tetPlaneCollide(ss_scratch, rigidId, softbodyId, tetrahedronIdx, rigidTransformCache.transform, 
			cDistance, restDist, writer, rigidShape.materialIndex);
	}
	else
	{
		PxVec3 tetCenter;
		const PxReal innerSphere = computeInnerSphere(v, tetCenter);

		size_t hullPtr = rigidShape.hullOrMeshPtr;
		const PxU8* convexPtr = (PxU8*)hullPtr + sizeof(float4);
		
		PxMeshScale rigidScale = rigidShape.scale;
	
		PxReal radius = 0.f;
		if (rigidType == PxGeometryType::eSPHERE || rigidType == PxGeometryType::eCAPSULE)
		{
			radius = rigidScale.scale.y;
		}

		if (threadIdx.x == 0)
		{
			if (rigidType == PxGeometryType::eSPHERE)
			{
				ss_scratch.nbVertices0 = 1;
				ss_scratch.inSphereRadius0 = 0;
				ss_scratch.vA[0] = rigidTransformCache.transform.p;

				ss_scratch.contactDistance = cDistance + rigidScale.scale.x;
				
			}
			else if (rigidType == PxGeometryType::eCAPSULE)
			{
				ss_scratch.nbVertices0 = 2;
				ss_scratch.inSphereRadius0 = 0;
				const PxVec3 xHalfHeight(rigidScale.scale.x, 0.f, 0.f);
				const PxVec3 wHalfHeight = rigidTransformCache.transform.rotate(xHalfHeight);
				ss_scratch.vA[0] = rigidTransformCache.transform.p + wHalfHeight;
				ss_scratch.vA[1] = rigidTransformCache.transform.p - wHalfHeight;

				ss_scratch.contactDistance = cDistance + rigidScale.scale.y;
			}
			else
			{
				const float4 extents = *((float4*)(convexPtr + sizeof(uint4)));
				const uint4 tmp = *((uint4*)convexPtr);
				const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;

				ss_scratch.nbVertices0 = getNbVerts(polyData0_NbEdgesNbHullVerticesNbPolygons);
				assert(ss_scratch.nbVertices0 <= CONVEX_MAX_VERTICES_POLYGONS);
				
				PxReal minimumScale = 1.f;
				if (rigidType == PxGeometryType::eBOX)
				{
					minimumScale = PxMin(rigidScale.scale.x, rigidScale.scale.y);
					minimumScale = PxMin(rigidScale.scale.z, minimumScale);
					//ss_scratch.nbVertices0 = 20;
				}
				ss_scratch.inSphereRadius0 = extents.w * minimumScale;

				ss_scratch.contactDistance = cDistance;
			}
			
			ss_scratch.scale0 = rigidScale.scale;
			ss_scratch.rot0 = rigidScale.rotation;

			//tetrahedron data
			ss_scratch.nbVertices1 = 4;
			ss_scratch.inSphereRadius1 = innerSphere;
			ss_scratch.searchDir = rigidTransformCache.transform.p - tetCenter;

			ss_scratch.cachedData.size = 0;
		}

		__syncwarp(); //ss_scratch.nbVertices0 is written above and read below

		if (rigidType == PxGeometryType::eCONVEXMESH || rigidType == PxGeometryType::eBOX)
		{
			const float4* pVertices0 = reinterpret_cast<const float4*>(convexPtr + sizeof(uint4) + sizeof(float4));

			/*if (rigidType == PxGeometryType::eBOX)
			{
				prepareBoxVertices(transfCache0.transform, shape0.scale.scale, shape0.scale.rotation, ss_scratch.nbVertices0,
					pVertices0, ss_scratch.vA);
			}
			else*/
			{
				prepareVertices(rigidTransformCache.transform, rigidScale.scale, rigidScale.rotation, ss_scratch.nbVertices0,
					pVertices0, ss_scratch.vA);
			}
		}

		__syncwarp();


		GjkResult::Enum result = tetPrimitivesCollide2(ss_epa_scratch, ss_scratch, 
			globalWarpIndex);

		bool intersect = (result == GjkResult::eOVERLAP || result == GjkResult::eCLOSE);


		PxU64 rigidRef = rigidId.getInd();
		PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronIdx);

		writeVolumeContactFullWarp(writer, intersect, &ss_scratch.gjkOutput.closestPointA.x, &ss_scratch.gjkOutput.closestPointB.x, -radius,
			&ss_scratch.gjkOutput.direction.x, false, restDist, rigidRef, rigidShape.materialIndex, tetRef);
	}
}

extern "C" __global__
void sb_primitiveContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	const PxU32									stackSizeBytes,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgRigidDbContactWriter						writer
)
{
	const PxU32 warpIndex = threadIdx.y;
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 7];

	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * MIDPHASE_WARPS_PER_BLOCK];
	__shared__ char sScratch[sizeof(TetCollideScratch) * MIDPHASE_WARPS_PER_BLOCK];
	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	TetCollideScratch* scratch = reinterpret_cast<TetCollideScratch*>(sScratch);

	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];
	TetCollideScratch& ss_scratch = scratch[warpIndex];

	sbTetDataScratch* s_warpScratch = (sbTetDataScratch*)scratchMem[threadIdx.y];

	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);
	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		/*if (globalWarpIdx != 21)
			return;*/

		const uint4 curPair = pairs[i];
		sbPrimitiveCollision(
			i,
			tolerenceLength,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			softbodies,
			shapeToRigidRemapTable,
			s_warpScratch,
			ss_epa_scratch,
			ss_scratch,
			writer
			);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}


struct sbsbScratch
{
	const float4 * PX_RESTRICT tetmeshVerts[2];
	const uint4 * PX_RESTRICT tetmeshTetIndices[2];
	const PxU8*	PX_RESTRICT tetmeshSurfaceHints[2];
};
PX_COMPILE_TIME_ASSERT(sizeof(sbsbScratch) <= WARP_SIZE * sizeof(PxU32) );

struct sbHeightfieldScratch
{
	const float4 * PX_RESTRICT tetmeshVerts;
	const uint4 * PX_RESTRICT tetmeshTetIndices;

	PxTransform heightfieldTransform;
	
	PxU32 nbRows;
	PxU32 nbCols;
	PxHeightFieldSample* samples;

};

PX_COMPILE_TIME_ASSERT(sizeof(sbHeightfieldScratch) <= WARP_SIZE * sizeof(PxU32));


__device__ static inline void sbMeshCollision(
	PxU32										globalWarpIndex,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4									curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	sbsbScratch*								s_warpScratch,
	squawk::EpaScratch&							ss_epa_scratch,
	TetCollideScratch&							ss_scratch,
	PxgRigidDbContactWriter&					writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 tetrahedronInd = curPair.y; //tetrahedron index
	const PxU32 triangleInd = curPair.z; //triangle index

											 /*if(threadIdx.x == 0)
											 printf("cmIdx %i tetrahedronIdx %i\n", cmIdx, tetrahedronIdx);*/

	PxgShape softbodyShape, trimeshShape;
	PxU32 softbodyCacheRef, trimeshCacheRef;
	LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		softbodyShape, softbodyCacheRef, trimeshShape, trimeshCacheRef);

	PxsCachedTransform trimeshTransformCache;
	PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);

	const PxTransform& trimeshToWorld = trimeshTransformCache.transform;
	const PxMeshScale& trimeshScale = trimeshShape.scale;

	PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[trimeshCacheRef];

	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];

	const PxNodeIndex rigidId = shapeToRigidRemapTable[trimeshCacheRef];

	// A filter pair for this rigid and tetrahedron suppresses contact generation between them.
	if (findRigidVolumeFilter(softbody.mRigidVolumeFilterPairs, softbody.mNumRigidVolumeFilterPairs,
		rigidId, trimeshCacheRef, softbodyId, tetrahedronInd))
		return;

	if (threadIdx.x == 0)
	{
		s_warpScratch->tetmeshVerts[0] = softbody.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices[0] = softbody.mTetIndices;
		s_warpScratch->tetmeshSurfaceHints[0] = softbody.mTetMeshSurfaceHint;

		PxU8 * trimeshGeomPtr = reinterpret_cast<PxU8*>(trimeshShape.hullOrMeshPtr);

		const uint4 nbVerts_nbTri_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
		trimeshGeomPtr += sizeof(uint4) + sizeof(const Gu::BV32DataPacked)* nbVerts_nbTri_maxDepth_nbBv32TreeNodes.w;

		s_warpScratch->tetmeshVerts[1] = reinterpret_cast<const float4 *>(trimeshGeomPtr);

		trimeshGeomPtr += sizeof(float4) * nbVerts_nbTri_maxDepth_nbBv32TreeNodes.x;

		s_warpScratch->tetmeshTetIndices[1] = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	}

	__syncwarp();

	const uint4 tetIdx = s_warpScratch->tetmeshTetIndices[0][tetrahedronInd];

	const uint* tetIdxf = reinterpret_cast<const uint*>(&tetIdx);
	float v;
	if (threadIdx.x < 12)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = tetIdxf[elemId]; // tetIdx.x, tetIdx.y, tetIdx.z, tetIdx.w

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[0][index]);

		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];

		//load the verts into shared memory
		float* vA = reinterpret_cast<float*>(&ss_scratch.vA);
		vA[threadIdx.x] = v;
	}

	PxVec3 tetCenter;
	const PxReal innerSphere = computeInnerSphere(v, tetCenter);

	const uint4 triIdx = s_warpScratch->tetmeshTetIndices[1][triangleInd];
	const uint* triIdxf = reinterpret_cast<const uint*>(&triIdx);
	if (threadIdx.x < 9)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = triIdxf[elemId]; // triIdxf.x, triIdxf.y, triIdxf.z

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[1][index]);

		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];
	}

	float s = 0.f, p = 0.f;
	if (threadIdx.x < 3)
	{
		const float* scale = reinterpret_cast<const float*>(&trimeshScale.scale.x);
		s = scale[threadIdx.x];

		const float* pp = reinterpret_cast<const float*>(&trimeshToWorld.p.x);
		p = pp[threadIdx.x];
	}

	float v0, v1, v2;
	{
		//vertex2Shape
		float m = v;
		v0 = rotateR(m, trimeshScale.rotation, threadIdx.x);
		v0 *= s;
		v0 = rotateInvR(v0, trimeshScale.rotation, threadIdx.x);

		//shfl 3, 4, 5 value to 0, 1, 2
		m = __shfl_sync(FULL_MASK, v, threadIdx.x + 3);
		v1 = rotateR(m, trimeshScale.rotation, threadIdx.x);
		v1 *= s;
		v1 = rotateInvR(v1, trimeshScale.rotation, threadIdx.x);

		
		//shfl 6, 7, 8 value to 0, 1, 2
		m = __shfl_sync(FULL_MASK, v, threadIdx.x + 6);
		v2 = rotateR(m, trimeshScale.rotation, threadIdx.x);
		v2 *= s;
		v2 = rotateInvR(v2, trimeshScale.rotation, threadIdx.x);
	}

	{
		//shapeToWorld
		v0 = rotateR(v0, trimeshToWorld.q, threadIdx.x) + p;
		v1 = rotateR(v1, trimeshToWorld.q, threadIdx.x) + p;
		v2 = rotateR(v2, trimeshToWorld.q, threadIdx.x) + p;
	}


	if (threadIdx.x < 3)
	{
		//load the verts into shared memory
		float* vB = reinterpret_cast<float*>(&ss_scratch.vB);
		vB[threadIdx.x] = v0;
		vB[threadIdx.x + 3] = v1;
		vB[threadIdx.x + 6] = v2;
	}

	/*PxVec3 vv0 = PxVec3(__shfl_sync(FULL_MASK, v0, 0), __shfl_sync(FULL_MASK, v0, 1), __shfl_sync(FULL_MASK, v0, 2));
	PxVec3 vv1 = PxVec3(__shfl_sync(FULL_MASK, v1, 0), __shfl_sync(FULL_MASK, v1, 1), __shfl_sync(FULL_MASK, v1, 2));
	PxVec3 vv2 = PxVec3(__shfl_sync(FULL_MASK, v2, 0), __shfl_sync(FULL_MASK, v2, 1), __shfl_sync(FULL_MASK, v2, 2));
*/
	//compute triangle world center
	const PxReal avg = 1.f / 3.f;
	PxReal triWorldCenter = (v0 + v1 + v2) * avg;
	PxVec3 triCenter = PxVec3( __shfl_sync(FULL_MASK, triWorldCenter, 0) 
						, __shfl_sync(FULL_MASK, triWorldCenter, 1) 
						, __shfl_sync(FULL_MASK, triWorldCenter, 2));

	

	/*PxVec3 t0 = PxVec3(__shfl_sync(FULL_MASK, v, 0), __shfl_sync(FULL_MASK, v, 1), __shfl_sync(FULL_MASK, v, 2));
	PxVec3 t1 = PxVec3(__shfl_sync(FULL_MASK, v, 3), __shfl_sync(FULL_MASK, v, 4), __shfl_sync(FULL_MASK, v, 5));
	PxVec3 t2 = PxVec3(__shfl_sync(FULL_MASK, v, 6), __shfl_sync(FULL_MASK, v, 7), __shfl_sync(FULL_MASK, v, 8));

	t0 = meshToWorld.transform(vertex2Shape(t0, trimeshScale.scale, trimeshScale.rotation));
	t1 = meshToWorld.transform(vertex2Shape(t1, trimeshScale.scale, trimeshScale.rotation));
	t2 = meshToWorld.transform(vertex2Shape(t2, trimeshScale.scale, trimeshScale.rotation));
	
	const PxVec3 triCenter1 = (t0 + t1 + t2) * avg;
	assert(triCenter1.x == triCenter.x && triCenter1.y == triCenter.y && triCenter1.z == triCenter.z);*/
	
	if (threadIdx.x == 0)
	{
		ss_scratch.tetIndex0 = tetrahedronInd;
		ss_scratch.tetIndex1 = triangleInd;

		ss_scratch.nbVertices0 = 4;
		ss_scratch.inSphereRadius0 = innerSphere;

		ss_scratch.nbVertices1 = 3;
		ss_scratch.inSphereRadius1 = 0.000001f;

		ss_scratch.searchDir = tetCenter - triCenter;

		ss_scratch.contactDistance = cDistance;

		ss_scratch.cachedData.size = 0;
	}
	

	__syncwarp();

	const PxVec3 v20(ss_scratch.vB[2] - ss_scratch.vB[0]);
	const PxVec3 v21(ss_scratch.vB[1] - ss_scratch.vB[0]);
	const PxVec3 n = (v20.cross(v21)).getNormalized();

	GjkResult::Enum result = tetPrimitivesCollide2(ss_epa_scratch, ss_scratch, globalWarpIndex);

	bool intersect = (result == GjkResult::eOVERLAP || result == GjkResult::eCLOSE);

	if (intersect)
	{
		intersect = intersect && ss_scratch.gjkOutput.direction.dot(n) >= 0.f;
	}

	PxU64 rigidRef = rigidId.getInd();
	PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronInd);

	const PxVec3 normal = ss_scratch.gjkOutput.direction.getNormalized();
	writeVolumeContactFullWarp(writer, intersect, &ss_scratch.gjkOutput.closestPointA.x, &ss_scratch.gjkOutput.closestPointB.x,
		0, &normal.x, true, restDistance[cmIdx], rigidRef, trimeshShape.materialIndex, tetRef);
}

extern "C" __global__
void sb_meshContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	const PxU32									stackSizeBytes,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgRigidDbContactWriter						writer
)
{

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	const PxU32 warpIndex = threadIdx.y;
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE];

	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * MIDPHASE_WARPS_PER_BLOCK];
	__shared__ char sScratch[sizeof(TetCollideScratch) * MIDPHASE_WARPS_PER_BLOCK];
	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	TetCollideScratch* scratch = reinterpret_cast<TetCollideScratch*>(sScratch);


	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];
	TetCollideScratch& ss_scratch = scratch[warpIndex];

	sbsbScratch* s_warpScratch = (sbsbScratch*)scratchMem[threadIdx.y];

	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;


	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);
	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		const uint4 curPair = pairs[i];

		sbMeshCollision(
			i,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			softbodies,
			shapeToRigidRemapTable,
			s_warpScratch,
			ss_epa_scratch,
			ss_scratch,
			writer
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}


__device__ static inline void sbHeightfieldCollision(
	PxU32										globalWarpIndex,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4									curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,

	sbHeightfieldScratch*						s_warpScratch,
	squawk::EpaScratch&							ss_epa_scratch,
	TetCollideScratch&							ss_scratch,
	PxgRigidDbContactWriter&					writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 tetrahedronInd = curPair.y; //tetrahedron index
	const PxU32 triangleIdx = curPair.z; //triangle index

										 /*if(threadIdx.x == 0)
										 printf("cmIdx %i tetrahedronIdx %i\n", cmIdx, tetrahedronIdx);*/

	PxgShape softbodyShape, heightfieldShape;
	PxU32 softbodyCacheRef, heightfieldCacheRef;
	LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eHEIGHTFIELD>(cmInputs, cmIdx, gpuShapes,
		softbodyShape, softbodyCacheRef, heightfieldShape, heightfieldCacheRef);

	PxsCachedTransform heightfieldCache;
	PxsCachedTransform_ReadWarp(heightfieldCache, transformCache + heightfieldCacheRef);

	const PxTransform& heightfieldToWorld = heightfieldCache.transform;
	
	PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[heightfieldCacheRef];

	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];

	const PxNodeIndex rigidId = shapeToRigidRemapTable[heightfieldCacheRef];

	// A filter pair for this rigid and tetrahedron suppresses contact generation between them.
	if (findRigidVolumeFilter(softbody.mRigidVolumeFilterPairs, softbody.mNumRigidVolumeFilterPairs,
		rigidId, heightfieldCacheRef, softbodyId, tetrahedronInd))
		return;

	if (threadIdx.x == 0)
	{
		s_warpScratch->tetmeshVerts = softbody.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices = softbody.mTetIndices;

		s_warpScratch->heightfieldTransform = heightfieldToWorld;

		PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightfieldShape.hullOrMeshPtr);
		s_warpScratch->nbRows = heightfieldData[0];
		s_warpScratch->nbCols = heightfieldData[1];
		s_warpScratch->samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);
	}

	__syncwarp();

	const uint4 tetIdx = s_warpScratch->tetmeshTetIndices[tetrahedronInd];

	const uint* tetIdxf = reinterpret_cast<const uint*>(&tetIdx);
	float v;
	if (threadIdx.x < 12)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = tetIdxf[elemId]; // tetIdx.x, tetIdx.y, tetIdx.z, tetIdx.w

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[index]);

		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];

		//load the verts into shared memory
		float* vA = reinterpret_cast<float*>(&ss_scratch.vA);
		vA[threadIdx.x] = v;
	}

	PxVec3 tetCenter;
	const PxReal innerSphere = computeInnerSphere(v, tetCenter);

	const PxU32 nbRows = s_warpScratch->nbRows;
	const PxU32 nbCols = s_warpScratch->nbCols;
	PxHeightFieldSample* samples = s_warpScratch->samples;

	PxVec3 triLocV0, triLocV1, triLocV2;
	getTriangle(triLocV0, triLocV1, triLocV2, NULL, triangleIdx, heightfieldShape.scale, nbRows, nbCols, samples);

	float p = 0.f, v0, v1, v2;
	if (threadIdx.x < 3)
	{
		const float* pp = reinterpret_cast<const float*>(&heightfieldToWorld.p.x);
		p = pp[threadIdx.x];

		const float* vv0 = reinterpret_cast<const float*>(&triLocV0.x);
		v0 = vv0[threadIdx.x];

		const float* vv1 = reinterpret_cast<const float*>(&triLocV1.x);
		v1 = vv1[threadIdx.x];

		const float* vv2 = reinterpret_cast<const float*>(&triLocV2.x);
		v2 = vv2[threadIdx.x];
	}

	//world space 
	v0 = rotateR(v0, heightfieldToWorld.q, threadIdx.x) + p;
	v1 = rotateR(v1, heightfieldToWorld.q, threadIdx.x) + p;
	v2 = rotateR(v2, heightfieldToWorld.q, threadIdx.x) + p;

	if (threadIdx.x < 3)
	{
		//load the verts into shared memory
		float* vB = reinterpret_cast<float*>(&ss_scratch.vB);
		vB[threadIdx.x] = v0;
		vB[threadIdx.x + 3] = v1;
		vB[threadIdx.x + 6] = v2;
	}

	/*PxVec3 vv0 = PxVec3(__shfl_sync(FULL_MASK, v0, 0), __shfl_sync(FULL_MASK, v0, 1), __shfl_sync(FULL_MASK, v0, 2));
	PxVec3 vv1 = PxVec3(__shfl_sync(FULL_MASK, v1, 0), __shfl_sync(FULL_MASK, v1, 1), __shfl_sync(FULL_MASK, v1, 2));
	PxVec3 vv2 = PxVec3(__shfl_sync(FULL_MASK, v2, 0), __shfl_sync(FULL_MASK, v2, 1), __shfl_sync(FULL_MASK, v2, 2));
	*/
	//compute triangle world center
	const PxReal avg = 1.f / 3.f;
	PxReal triWorldCenter = (v0 + v1 + v2) * avg;
	PxVec3 triCenter = PxVec3(__shfl_sync(FULL_MASK, triWorldCenter, 0)
		, __shfl_sync(FULL_MASK, triWorldCenter, 1)
		, __shfl_sync(FULL_MASK, triWorldCenter, 2));

	/*PxVec3 t0 = PxVec3(__shfl_sync(FULL_MASK, v, 0), __shfl_sync(FULL_MASK, v, 1), __shfl_sync(FULL_MASK, v, 2));
	PxVec3 t1 = PxVec3(__shfl_sync(FULL_MASK, v, 3), __shfl_sync(FULL_MASK, v, 4), __shfl_sync(FULL_MASK, v, 5));
	PxVec3 t2 = PxVec3(__shfl_sync(FULL_MASK, v, 6), __shfl_sync(FULL_MASK, v, 7), __shfl_sync(FULL_MASK, v, 8));

	t0 = meshToWorld.transform(vertex2Shape(t0, trimeshScale.scale, trimeshScale.rotation));
	t1 = meshToWorld.transform(vertex2Shape(t1, trimeshScale.scale, trimeshScale.rotation));
	t2 = meshToWorld.transform(vertex2Shape(t2, trimeshScale.scale, trimeshScale.rotation));

	const PxVec3 triCenter1 = (t0 + t1 + t2) * avg;
	assert(triCenter1.x == triCenter.x && triCenter1.y == triCenter.y && triCenter1.z == triCenter.z);*/

	if (threadIdx.x == 0)
	{
		ss_scratch.tetIndex0 = tetrahedronInd;
		ss_scratch.tetIndex1 = triangleIdx;

		ss_scratch.nbVertices0 = 4;
		ss_scratch.inSphereRadius0 = innerSphere;

		ss_scratch.nbVertices1 = 3;
		ss_scratch.inSphereRadius1 = 0.000001f;

		ss_scratch.searchDir = tetCenter - triCenter;

		ss_scratch.contactDistance = cDistance;

		ss_scratch.cachedData.size = 0;
	}


	__syncwarp();


	GjkResult::Enum result = tetPrimitivesCollide2(ss_epa_scratch, ss_scratch, globalWarpIndex);

	bool intersect = (result == GjkResult::eOVERLAP || result == GjkResult::eCLOSE);

	PxU64 rigidRef = rigidId.getInd();
	PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronInd);

	const PxVec3 v20(ss_scratch.vB[2] - ss_scratch.vB[0]);
	const PxVec3 v21(ss_scratch.vB[1] - ss_scratch.vB[0]);
	const PxVec3 n = (v20.cross(v21)).getNormalized();

	writeVolumeContactFullWarp(writer, intersect, &ss_scratch.gjkOutput.closestPointA.x, &ss_scratch.gjkOutput.closestPointB.x,
		0, &n.x, true, restDistance[cmIdx], rigidRef, heightfieldShape.materialIndex, tetRef);
}

extern "C" __global__
void sb_heightfieldContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,

	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	const PxU32									stackSizeBytes,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgRigidDbContactWriter						writer
)
{
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	const PxU32 warpIndex = threadIdx.y;
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE];
	
	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * MIDPHASE_WARPS_PER_BLOCK];
	__shared__ char sScratch[sizeof(TetCollideScratch) * MIDPHASE_WARPS_PER_BLOCK];
	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	TetCollideScratch* scratch = reinterpret_cast<TetCollideScratch*>(sScratch);
	   
	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];
	TetCollideScratch& ss_scratch = scratch[warpIndex];

	sbHeightfieldScratch* s_warpScratch = (sbHeightfieldScratch*)scratchMem[threadIdx.y];

	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;


	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);
	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		const uint4 curPair = pairs[i];

		sbHeightfieldCollision(
			i,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			softbodies,

			shapeToRigidRemapTable,
			s_warpScratch,
			ss_epa_scratch,
			ss_scratch,
			writer
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}

__device__ static inline void sbClothCollision(
	PxU32										globalWarpIndex,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4									curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	sbsbScratch*								s_warpScratch,
	squawk::EpaScratch&							ss_epa_scratch,
	TetCollideScratch&							ss_scratch,
	PxgDbDbContactWriter&						writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 tetrahedronInd = curPair.y; //tetrahedron index
	const PxU32 triangleInd = curPair.z; //triangle index

	PxgShape softbodyShape, clothShape;
	PxU32 softbodyCacheRef, clothCacheRef;
	LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		softbodyShape, softbodyCacheRef, clothShape, clothCacheRef);

	PxsCachedTransform clothTransformCache;
	PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);

	const PxTransform& clothToWorld = clothTransformCache.transform;
	//const PxMeshScale& trimeshScale = trimeshShape.scale;

	/*PxTransform meshToWorld = PxTransform(PxVec3(0.711670f, 1.497430f, 1.737538f), PxQuat(0.796422f, -0.477444f, 0.280923f, -0.242574f));
	PxMeshScale trimeshScale;
	trimeshScale.scale = PxVec3(1.f, 2.f, 4.f);
	trimeshScale.rotation = PxQuat(PxIdentity);*/

	PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[clothCacheRef];

	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];
	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	const PxgFEMCloth& cloth = clothes[clothId];


	if (threadIdx.x == 0)
	{
		s_warpScratch->tetmeshVerts[0] = softbody.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices[0] = softbody.mTetIndices;
		s_warpScratch->tetmeshSurfaceHints[0] = softbody.mTetMeshSurfaceHint;


		s_warpScratch->tetmeshVerts[1] = cloth.mPosition_InvMass;
		s_warpScratch->tetmeshTetIndices[1] = cloth.mTriangleVertexIndices_ownedEdges;

	}

	__syncwarp();

	const PxgDeformablePairInfo* filterPairs = softbody.mVolumeSurfaceFilterPairs;
	const PxU32 nbFilterPairs = softbody.mNumVolumeSurfaceFilterPairs;

	// Filters this tetrahedron against the whole surface, then against the input triangle. Also checks
	// whether the triangle is filtered against any tetrahedron.
	if (findVolumeSurfaceTriangleFilter(filterPairs, nbFilterPairs, softbodyId, tetrahedronInd, clothId, PxgDeformableRef::ANY_FEATURE) ||
		findVolumeSurfaceTriangleFilter(filterPairs, nbFilterPairs, softbodyId, tetrahedronInd, clothId, triangleInd) ||
		findVolumeSurfaceTriangleFilter(filterPairs, nbFilterPairs, softbodyId, PxgDeformableRef::ANY_FEATURE, clothId, triangleInd))
	{
		return;
	}

	const uint4 tetIdx = s_warpScratch->tetmeshTetIndices[0][tetrahedronInd];

	const uint* tetIdxf = reinterpret_cast<const uint*>(&tetIdx);
	float v;
	if (threadIdx.x < 12)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = tetIdxf[elemId]; // tetIdx.x, tetIdx.y, tetIdx.z, tetIdx.w

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[0][index]);

		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];

		//load the verts into shared memory
		float* vA = reinterpret_cast<float*>(&ss_scratch.vA);
		vA[threadIdx.x] = v;
	}

	PxVec3 tetCenter;
	const PxReal innerSphere = computeInnerSphere(v, tetCenter);

	const uint4 triIdx = s_warpScratch->tetmeshTetIndices[1][triangleInd];
	const uint* triIdxf = reinterpret_cast<const uint*>(&triIdx);
	if (threadIdx.x < 9)
	{
		const PxU32 elemId = threadIdx.x / 3;
		const PxU32 index = triIdxf[elemId]; // triIdxf.x, triIdxf.y, triIdxf.z

		const float* verts = reinterpret_cast<const float*>(&s_warpScratch->tetmeshVerts[1][index]);

		const PxU32 ind = threadIdx.x % 3;
		v = verts[ind];
	}

	float /*s = 0.f, */p = 0.f;
	if (threadIdx.x < 3)
	{
		//const float* scale = reinterpret_cast<const float*>(&trimeshScale.scale.x);
		//s = scale[threadIdx.x];

		const float* pp = reinterpret_cast<const float*>(&clothToWorld.p.x);
		p = pp[threadIdx.x];
	}

	//float v0, v1, v2;
	//{
	//	//vertex2Shape
	//	float m = v;
	//	v0 = rotateR(m, trimeshScale.rotation, threadIdx.x);
	//	//v0 *= s;
	//	v0 = rotateInvR(v0, trimeshScale.rotation, threadIdx.x);

	//	//shfl 3, 4, 5 value to 0, 1, 2
	//	m = __shfl_sync(mask, v, threadIdx.x + 3);
	//	v1 = rotateR(m, trimeshScale.rotation, threadIdx.x);
	//	//v1 *= s;
	//	v1 = rotateInvR(v1, trimeshScale.rotation, threadIdx.x);


	//	//shfl 6, 7, 8 value to 0, 1, 2
	//	m = __shfl_sync(mask, v, threadIdx.x + 6);
	//	v2 = rotateR(m, trimeshScale.rotation, threadIdx.x);
	//	//v2 *= s;
	//	v2 = rotateInvR(v2, trimeshScale.rotation, threadIdx.x);
	//}

	float v0, v1, v2;
	{
		//shapeToWorld
		float m = v;
		v0 = rotateR(m, clothToWorld.q, threadIdx.x) + p;

		//shfl 3, 4, 5 value to 0, 1, 2
		m = __shfl_sync(FULL_MASK, v, threadIdx.x + 3);

		v1 = rotateR(m, clothToWorld.q, threadIdx.x) + p;

		//shfl 6, 7, 8 value to 0, 1, 2
		m = __shfl_sync(FULL_MASK, v, threadIdx.x + 6);
		v2 = rotateR(m, clothToWorld.q, threadIdx.x) + p;
	}


	if (threadIdx.x < 3)
	{
		//load the verts into shared memory
		float* vB = reinterpret_cast<float*>(&ss_scratch.vB);
		vB[threadIdx.x] = v0;
		vB[threadIdx.x + 3] = v1;
		vB[threadIdx.x + 6] = v2;
	}

	/*PxVec3 vv0 = PxVec3(__shfl_sync(FULL_MASK, v0, 0), __shfl_sync(FULL_MASK, v0, 1), __shfl_sync(FULL_MASK, v0, 2));
	PxVec3 vv1 = PxVec3(__shfl_sync(FULL_MASK, v1, 0), __shfl_sync(FULL_MASK, v1, 1), __shfl_sync(FULL_MASK, v1, 2));
	PxVec3 vv2 = PxVec3(__shfl_sync(FULL_MASK, v2, 0), __shfl_sync(FULL_MASK, v2, 1), __shfl_sync(FULL_MASK, v2, 2));
*/
//compute triangle world center
	const PxReal avg = 1.f / 3.f;
	PxReal triWorldCenter = (v0 + v1 + v2) * avg;
	PxVec3 triCenter = PxVec3(__shfl_sync(FULL_MASK, triWorldCenter, 0)
		, __shfl_sync(FULL_MASK, triWorldCenter, 1)
		, __shfl_sync(FULL_MASK, triWorldCenter, 2));



	/*PxVec3 t0 = PxVec3(__shfl_sync(FULL_MASK, v, 0), __shfl_sync(FULL_MASK, v, 1), __shfl_sync(FULL_MASK, v, 2));
	PxVec3 t1 = PxVec3(__shfl_sync(FULL_MASK, v, 3), __shfl_sync(FULL_MASK, v, 4), __shfl_sync(FULL_MASK, v, 5));
	PxVec3 t2 = PxVec3(__shfl_sync(FULL_MASK, v, 6), __shfl_sync(FULL_MASK, v, 7), __shfl_sync(FULL_MASK, v, 8));

	t0 = meshToWorld.transform(vertex2Shape(t0, trimeshScale.scale, trimeshScale.rotation));
	t1 = meshToWorld.transform(vertex2Shape(t1, trimeshScale.scale, trimeshScale.rotation));
	t2 = meshToWorld.transform(vertex2Shape(t2, trimeshScale.scale, trimeshScale.rotation));

	const PxVec3 triCenter1 = (t0 + t1 + t2) * avg;
	assert(triCenter1.x == triCenter.x && triCenter1.y == triCenter.y && triCenter1.z == triCenter.z);*/

	if (threadIdx.x == 0)
	{
		ss_scratch.tetIndex0 = tetrahedronInd;
		ss_scratch.tetIndex1 = triangleInd;

		ss_scratch.nbVertices0 = 4;
		ss_scratch.inSphereRadius0 = innerSphere;

		ss_scratch.nbVertices1 = 3;
		ss_scratch.inSphereRadius1 = 0.000001f;

		ss_scratch.searchDir = tetCenter - triCenter;

		ss_scratch.contactDistance = cDistance;

		ss_scratch.cachedData.size = 0;
	}


	__syncwarp();


	GjkResult::Enum result = tetPrimitivesCollide2(ss_epa_scratch, ss_scratch, globalWarpIndex);

	bool intersect = (result == GjkResult::eOVERLAP || result == GjkResult::eCLOSE);
	float4 barycentric0, barycentric1;
	if (intersect)
	{
		intersect = computeTetBarycentric(ss_scratch.vA, ss_scratch.gjkOutput.closestPointA, barycentric1);
		if (intersect)
		{
			intersect = computeTriangleBarycentric(ss_scratch.vB, ss_scratch.gjkOutput.closestPointB, barycentric0);
		}
	}

	if (intersect)
	{
		////compute triangle normal
		//const PxVec3 v20(ss_scratch.vB[2] - ss_scratch.vB[0]);
		//const PxVec3 v21(ss_scratch.vB[1] - ss_scratch.vB[0]);
		//const PxVec3 n = (v20.cross(v21)).getNormalized();

		//Triangle rejection...
		const PxU8 tetMask = s_warpScratch->tetmeshSurfaceHints[0][tetrahedronInd];

		bool keep = false;
		if (threadIdx.x < 4)
		{
			
			if (tetMask & (1 << threadIdx.x))
			{
				PxU32 startIndex = PxU32(0 - threadIdx.x) & 3; //0, 3, 2, 1
				PxVec3 v0 = ss_scratch.vA[startIndex];
				PxVec3 v1 = ss_scratch.vA[(startIndex+1)&3];
				PxVec3 v2 = ss_scratch.vA[(startIndex+2) & 3];
				PxVec3 otherV = ss_scratch.vA[(startIndex + 3) & 3];

				//T0: 0,1,2, T1: 3,0,1, T2: 2,3,0 T3: 1,2,3

				PxVec3 normal = (v1 - v0).cross(v2 - v0);
				if ((otherV - v0).dot(normal) < 0.f)
					normal = -normal;
				
				//Now compare with 
				PxVec3 n = ss_scratch.gjkOutput.direction;

				if (normal.dot(n) < 0.f)
					keep = true;
			}
		}

		intersect = __any_sync(FULL_MASK, keep);

		int32_t index = 0xffffffff;
		if (threadIdx.x == 0 && intersect)
		{
			//printf("cmIdx %i tetrahedronIdx %i globalWarpIndex %i\n", cmIdx, tetrahedronIdx, globalWarpIndex);

			index = atomicAdd(writer.totalContactCount, 1);
			if (index >= writer.maxContacts)
				index = 0xffffffff;

			//printf("%i: StartIndex = %i nbToLock = %i\n", workIndex, startIndex, nbToLock);
		}

		index = __shfl_sync(FULL_MASK, index, 0);

		if (index != 0xffffffff)
		{

			PxU64 triRef = PxgEncodeDeformableTriangle(clothId, triangleInd);
			PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronInd);

			/*if (threadIdx.x == 0)
			{
				const PxU32 newTriInd = tetRef.getFemTriId();
				printf("%i softbodyId %i tetrahedronInd %i clothId %i triangleId %i newTriInd %i\n", index, softbodyId, tetrahedronInd, clothId, triangleInd, newTriInd);
			}*/

			PxReal witnessA = 0.f;
			PxReal witnessB = 0.f;
			PxReal normal = 0.f;

			if (threadIdx.x < 3)
			{
				const float* closestPointA = &ss_scratch.gjkOutput.closestPointA.x; //soft body
				const float* closestPointB = &ss_scratch.gjkOutput.closestPointB.x; //cloth
				const float* direction = &ss_scratch.gjkOutput.direction.x;
				witnessA = closestPointA[threadIdx.x];
				witnessB = closestPointB[threadIdx.x];
				normal = direction[threadIdx.x];
				//normal = surfaceNormal[threadIdx.x];
				//normal = n[threadIdx.x];
			}
			PxReal pen = (witnessB - witnessA) * normal;

			pen = __shfl_sync(FULL_MASK, pen, 0) + __shfl_sync(FULL_MASK, pen, 1) + __shfl_sync(FULL_MASK, pen, 2);


			if (threadIdx.x < 4)
			{
				float* point = reinterpret_cast<float*>(&writer.outPoint[index]);
				float* normalPen = reinterpret_cast<float*>(&writer.outNormalPen[index]);
				float* rBarycentric0 = reinterpret_cast<float*>(&writer.outBarycentric0[index]);

				// The point is the soft body's witness, which the remap converts into a simulation mesh
				// tetrahedron and the corresponding barycentric. The cloth's barycentric is written here
				// because it needs no remapping.
				const PxReal outP = threadIdx.x < 3 ? witnessA : 0.f;
				point[threadIdx.x] = outP;

				const PxReal outNP = threadIdx.x < 3 ? (-normal) : pen;
				normalPen[threadIdx.x] = outNP;

				const float* tBarycentric0 = reinterpret_cast<float*>(&barycentric0.x);
				rBarycentric0[threadIdx.x] = tBarycentric0[threadIdx.x];

				if (threadIdx.x == 0)
				{
					writer.outContactInfo[index].ref0 = triRef; //cloth
					writer.outContactInfo[index].ref1 = tetRef; //soft body

					//if (index == 0)
					//{
					//	printf("%i cB0(%f, %f, %f, %f), sB1(%f, %f, %f, %f)\n", index, barycentric0.x, barycentric0.y, barycentric0.z, barycentric0.w,
					//		barycentric1.x, barycentric1.y, barycentric1.z, barycentric1.w);

					//	PxVec3 pB = ss_scratch.gjkOutput.closestPointB;
					//	PxVec3 pA = ss_scratch.gjkOutput.closestPointA;

					//	PxVec3 norm = ss_scratch.gjkOutput.direction;
					//	const PxVec3 nn = norm.getNormalized();

					//	//const PxVec3 cpA = pB - nn * pen;
					//	printf("clothPoint(%f, %f, %f), SoftPoint(%f, %f, %f) norm(%f, %f, %f, %f)\n", pB.x, pB.y, pB.z, pA.x, pA.y, pA.z,
					//		norm.x, norm.y, norm.z, pen);


					//	printf("triangleInd %i, tetInd %i\n", triangleInd, tetrahedronInd);

					//}


				}


				//normalPen[threadIdx.x] = threadIdx.x < 3 ? witnessA : 0.f;
			}
		}
	}
}

extern "C" __global__
void sb_clothContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	const PxU32									stackSizeBytes,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgDbDbContactWriter						writer
)
{
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	const PxU32 warpIndex = threadIdx.y;
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE];
	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * MIDPHASE_WARPS_PER_BLOCK];
	__shared__ char sScratch[sizeof(TetCollideScratch) * MIDPHASE_WARPS_PER_BLOCK];
	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	TetCollideScratch* scratch = reinterpret_cast<TetCollideScratch*>(sScratch);


	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];
	TetCollideScratch& ss_scratch = scratch[warpIndex];

	sbsbScratch* s_warpScratch = (sbsbScratch*)scratchMem[threadIdx.y];

	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;


	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);
	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		const uint4 curPair = pairs[i];

		sbClothCollision(
			i,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			gpuShapes,
			softbodies,
			clothes,
			s_warpScratch,
			ss_epa_scratch,
			ss_scratch,
			writer
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}

extern "C" __global__
void fem_reorderRigidContactsLaunch(
	const float4* contacts,
	const float4* normalpens,
	const float4* barycentrics,
	PxgDeformablePairInfo* contactInfos,
	const PxU16* contactMaterials,
	const PxU32* numContacts,
	const PxU32* remapByRigid,
	float4* sortedContacts,
	float4* sortedNormalPens,
	float4* sortedBarycentrics,
	PxgDeformablePairInfo* sortedContactInfos,
	PxU16* sortedContactMaterials,
	PxU64*	sortedRigidIds
)
{
	const PxU32 totalNumContacts = *numContacts;

	const PxU32 numIternations = (totalNumContacts + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;

	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;
		if (index >= totalNumContacts)
			return;

		const PxU32 remapIndex = remapByRigid[index];

		sortedContacts[index] = contacts[remapIndex];
		sortedNormalPens[index] = normalpens[remapIndex];
		sortedBarycentrics[index] = barycentrics[remapIndex];

		const PxgDeformablePairInfo contactInfo = contactInfos[remapIndex];

		PxU64 rigidId = contactInfo.ref0;
		//assert(reinterpret_cast<const IG::NodeIndex&>(rigidId).isStaticBody());

		sortedContactInfos[index] = contactInfo;
		sortedContactMaterials[index] = contactMaterials[remapIndex];
		sortedRigidIds[index] = rigidId;
	}
}

extern "C" __global__
void sb_reorderPSContactsLaunch(
	const float4* contacts,
	const float4* normalpens,
	const float4* barycentrics,
	PxgDeformablePairInfo* contactInfos,
	const PxU32* numContacts,
	const PxU32* remapByRigid,
	float4* sortedContacts,
	float4* sortedNormalPens,
	float4* sortedBarycentrics,
	PxgDeformablePairInfo* sortedContactInfos
)
{
	const PxU32 totalNumContacts = *numContacts;

	const PxU32 numIternations = (totalNumContacts + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;

	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;
		if (index >= totalNumContacts)
			return;

		const PxU32 remapIndex = remapByRigid[index];

		sortedContacts[index] = contacts[remapIndex];
		sortedNormalPens[index] = normalpens[remapIndex];
		sortedBarycentrics[index] = barycentrics[remapIndex];
		sortedContactInfos[index] = contactInfos[remapIndex];
	}
}



static inline __device__ float4 PointOutsideOfPlane4(const PxVec3& p, const PxVec3& _a, const PxVec3& _b, 
	const PxVec3& _c, const PxVec3& _d)
{
	const PxVec3 ap = p - _a;
	const PxVec3 ab = _b - _a;
	const PxVec3 ac = _c - _a;
	const PxVec3 ad = _d - _a;

	const PxVec3 v0 = ab.cross(ac);
	const float signa0 = v0.dot(ap);
	const float signd0 = v0.dot(ad);// V3Dot(v0, _d);

	const PxVec3 v1 = ac.cross(ad);
	const float signa1 = v1.dot(ap);
	const float signd1 = v1.dot(ab);

	const PxVec3 v2 = ad.cross(ab);
	const float signa2 = v2.dot(ap);
	const float signd2 = v2.dot(ac);// V3Dot(v2, _c);

	const PxVec3 bd = _d - _b;
	const PxVec3 bc = _c - _b;
	
	const PxVec3 v3 = bd.cross(bc); 
	const float signd3 = v3.dot(p - _b);
	const float signa3 = v3.dot(_a - _b);

	//if combined signDist is less than zero, p is outside of that face
	float4 result = make_float4(signa0 * signd0, signa1 * signd1, signa2 * signd2, signa3 * signd3);
	
	return result;
}

__device__ PxVec3 getClosestPtPointTetrahedron(const PxVec3& p, const PxVec3& _a, const PxVec3& _b, const PxVec3& _c, const PxVec3& _d, const float4& result)
{
	//point is outside of this face
	PxVec3 bestClosestPt(0.f, 0.f, 0.f);
	PxReal bestSqDist = PX_MAX_F32;

	PxVec3 ab = _b - _a;
	PxVec3 ac = _c - _a;
	PxVec3 ad = _d - _a;

	if (result.x < 0.f)
	{
		// 0, 1, 2
		bestClosestPt = Gu::closestPtPointTriangle2(p, _a, _b, _c, ab, ac);
		PxVec3 diff = bestClosestPt - p;
		bestSqDist = diff.dot(diff);
	}

	if (result.y < 0.f)
	{
		// 0, 2, 3
		const PxVec3 closestPt = Gu::closestPtPointTriangle2(p, _a, _c, _d, ac, ad);
		PxVec3 diff = closestPt - p;
		const PxReal sqDist = diff.dot(diff);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.z < 0.f)
	{
		// 0, 3, 1
		const PxVec3 closestPt = Gu::closestPtPointTriangle2(p, _a, _d, _b, ad, ab);
		PxVec3 diff = closestPt - p;
		const PxReal sqDist = diff.dot(diff);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.w < 0.f)
	{
		// 1, 3, 2
		const PxVec3 closestPt = Gu::closestPtPointTriangle2(p, _b, _d, _c, _d-_b, _c-_b);
		PxVec3 diff = closestPt - p;
		const PxReal sqDist = diff.dot(diff);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	return bestClosestPt;
}

__device__ bool checkNormals(const PxVec3& n, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxU8 mask)
{
	if (mask & (1 << 0))
	{
		PxVec3 normal = (b - a).cross(c - a);
		if (normal.dot(d - a) < 0.f)
			normal = -normal;
		if (normal.dot(n) < 0.f)
			return true;
	}

	if (mask & (1 << 1))
	{
		PxVec3 normal = (b - a).cross(d - a);
		if (normal.dot(c - a) < 0.f)
			normal = -normal;
		if (normal.dot(n) < 0.f)
			return true;
	}

	if (mask & (1 << 2))
	{
		PxVec3 normal = (c - a).cross(d - a);
		if (normal.dot(b - a) < 0.f)
			normal = -normal;
		if (normal.dot(n) < 0.f)
			return true;
	}

	if (mask & (1 << 3))
	{
		PxVec3 normal = (c - b).cross(d - b);
		if (normal.dot(a - c) < 0.f)
			normal = -normal;
		if (normal.dot(n) < 0.f)
			return true;
	}

	return false;
}

//Closest point between a query point p and a tetrahedron
__device__ static void closestPtTetrahedron(const PxVec3& point, const PxU8 hint,
	const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, PxVec3& normal,
	PxVec3& closestP, PxReal& sqDist)
{
	const PxVec3 ac = (c - a);
	const PxVec3 ab = (b - a);
	const PxVec3 ad = (d - a);

	if (hint & 1) //0111
	{
		const PxVec3 tPoint = Gu::closestPtPointTriangle2(point, a, b, c, ab, ac);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ac.cross(ab)).getNormalized();
			if (ad.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 2)//1011
	{
		const PxVec3 tPoint = Gu::closestPtPointTriangle2(point, a, b, d, ab, ad);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ab.cross(ad)).getNormalized();
			if (ac.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 4)//1101
	{
		const PxVec3 tPoint = Gu::closestPtPointTriangle2(point, a, c, d, ac, ad);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ac.cross(ad)).getNormalized();
			if (ab.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 8)//1110
	{

		const PxVec3 bd = (d - b);
		const PxVec3 bc = (c - b);
		const PxVec3 tPoint = Gu::closestPtPointTriangle2(point, b, c, d, bc, bd);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (bc.cross(bd)).getNormalized();
			if (ab.dot(normal) > 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}
}


__device__ static inline void sbParticleCollision(
	const PxReal									toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT		cmInputs,
	const uint4										curPair,
	const PxsCachedTransform* PX_RESTRICT			transformCache,
	const PxReal* PX_RESTRICT						contactDistance,
	const PxgShape* PX_RESTRICT						gpuShapes,
	const PxgParticleSystem* PX_RESTRICT			particleSystems,
	const PxgSoftBody* PX_RESTRICT					softbodies,
	PxgParticleDbContactWriter&						writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 particleIndex = curPair.y;
	const PxU32 tetrahedronIdx = curPair.z;

	//printf("idx %i cmIdx %i particleIndex %i tetrahedronIdx %i\n", threadIdx.x, cmIdx, particleIndex, tetrahedronIdx);
		
	PxgShape particleShape, softbodyShape;
	PxU32 particleCacheRef, softbodyCacheRef;
	LoadShapePair<PxGeometryType::ePARTICLESYSTEM, PxGeometryType::eTETRAHEDRONMESH>(cmInputs, cmIdx, gpuShapes,
		particleShape, particleCacheRef, softbodyShape, softbodyCacheRef);

	const PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[softbodyCacheRef];

	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];

	const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;
	const PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
	
	const float4* sortedPose = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);

	
	const float4* tetmeshVerts = softbody.mPosition_InvMass;

	const uint4 tetIdx = softbody.mTetIndices[tetrahedronIdx];

	PxU8 surfaceMask = softbody.mTetMeshSurfaceHint[tetrahedronIdx];

	if (surfaceMask == 0)
		return;

	const float4 a_ = tetmeshVerts[tetIdx.x];
	const float4 b_ = tetmeshVerts[tetIdx.y];
	const float4 c_ = tetmeshVerts[tetIdx.z];
	const float4 d_ = tetmeshVerts[tetIdx.w];

	const PxVec3 a(a_.x, a_.y, a_.z);
	const PxVec3 b(b_.x, b_.y, b_.z);
	const PxVec3 c(c_.x, c_.y, c_.z);
	const PxVec3 d(d_.x, d_.y, d_.z);

	const float4 p_ = sortedPose[particleIndex];

	const PxVec3 p(p_.x, p_.y, p_.z);

	const float4 result = PointOutsideOfPlane4(p, a, b, c, d);

	/*if(threadIdx.x == 16)
		printf("idx %i result(%f, %f, %f, %f)\n", threadIdx.x, result.x, result.y, result.z, result.w);*/

	PxVec3 closestP;
	PxVec3 n2;
	PxReal sqDist = PX_MAX_F32;
	closestPtTetrahedron(p, surfaceMask, a, b, c, d, n2, closestP, sqDist);

	PxVec3 n = -(p - closestP);

	PxReal pen;

	//this means the p is inside for the tetrahedron
	if (result.x >= 0.f && result.y >= 0.f && result.z >= 0.f && result.w >= 0.f)
	{
		pen = n.dot(n2);
		n = n2;
	}
	else
	{
		
		const PxReal sqContactDistance = cDistance * cDistance;

		if (sqDist > sqContactDistance)
			return;

		
		pen = n.normalize();
		if (sqDist < 1e-8f || n.dot(n2) < 0.8f)
		{
			n = n2;
		}
	}

	int32_t index = atomicAdd(writer.totalContactCount, 1);
	if (index >= writer.maxNumContacts)
		return;

	{
		PxU64 particleRef = PxgEncodeParticleRef(particleSystemId, particleIndex);
		PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronIdx);

		writer.writeVolumeContact(index, make_float4(p.x, p.y, p.z, 0.f), make_float4(n.x, n.y, n.z, pen),
			particleRef, tetRef);
	}
}

extern "C" __global__
void sb_psContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgParticleSystem* PX_RESTRICT		particleSystems,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxU32									stackSizeBytes,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,	//output
	PxgParticleDbContactWriter					writer
)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const uint4* PX_RESTRICT pairs = reinterpret_cast<uint4*>(stackPtr);

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalThreadIdx; i < numPairs; i += gridDim.x * blockDim.x)
	{
		const uint4 curPair = pairs[i];

		sbParticleCollision(tolerenceLength, cmInputs,
			curPair, transformCache, contactDistance, gpuShapes,
			particleSystems, softbodies,
			writer
		);
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4)); 
	}
}




extern "C" __global__
void sb_other_contact_remap_to_simLaunch(
	const PxgSoftBody* PX_RESTRICT	softbodies,
	const PxU32						maxNumContacts,
	const float4*					point,
	float4*							outBarycentric,		//output
	PxgDeformablePairInfo *			outContactInfo,		//output
	PxU32*							totalContactCount,	//output
	const PxU32*					prevContactCount
)
{
	const PxU32 warpIdx = threadIdx.y;
	const PxU32 idxInWarp = threadIdx.x;

	const PxU32 globalWarpIdx = warpIdx + blockIdx.x * blockDim.y;

	PxU32 numContacts = *totalContactCount;

	if (numContacts > maxNumContacts)
	{
		numContacts = maxNumContacts;
		if (idxInWarp == 0 && globalWarpIdx == 0)
			*totalContactCount = maxNumContacts;
	}
	
	const PxU32 startIndex = prevContactCount ? *prevContactCount : 0;

	//each warp do remap from contacts in collision mesh to simulation mesh
	for (PxU32 contactIndex = globalWarpIdx + startIndex; contactIndex < numContacts; contactIndex += gridDim.x * blockDim.y)
	{
		const PxVec3 contact = PxLoad3(point[contactIndex]);

		const PxgDeformablePairInfo& contactInfo = outContactInfo[contactIndex];

		const PxU32 softbodyId = PxgGetDeformableId(contactInfo.ref1);
		const PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetInd = PxgGetDeformableFeatureId(contactInfo.ref1);

		const uint4* simTetrahedrons = softbody.mSimTetIndices;
		const float4* simVerts = softbody.mSimPosition_InvMass;
		const PxU32* tetRemapColToSim = softbody.mTetsRemapColToSim;
		const PxU32* tetAccumulatedRemap = softbody.mTetsAccumulatedRemapColToSim;

		const PxU32 endInd = tetAccumulatedRemap[tetInd];
		const PxU32 startInd = (tetInd != 0) ? tetAccumulatedRemap[tetInd - 1] : 0;
		const PxU32 range = endInd - startInd;

		const PxU32 nbIterations = (range + WARP_SIZE - 1) / WARP_SIZE;
		assert(nbIterations > 0);

		// result variables
		PxU64 simTetRef = PxgEncodeInvalidDeformableRef(softbodyId);
		float4 barycentric;
		PxReal sqDist = PX_MAX_F32;
		bool valid = false;

		for (PxU32 j = 0; j < nbIterations; ++j)
		{
			bool intersect = false;

			const PxU32 workIndex = idxInWarp + j * WARP_SIZE;
			if (workIndex < range)
			{
				const PxU32 index = startInd + workIndex;
				PxU32 tmpSimTetInd = tetRemapColToSim[index];
				const uint4 simTet = simTetrahedrons[tmpSimTetInd];
				const float4 v0 = simVerts[simTet.x];
				const float4 v1 = simVerts[simTet.y];
				const float4 v2 = simVerts[simTet.z];
				const float4 v3 = simVerts[simTet.w];

				const PxVec3 a(v0.x, v0.y, v0.z);
				const PxVec3 b(v1.x, v1.y, v1.z);
				const PxVec3 c(v2.x, v2.y, v2.z);
				const PxVec3 d(v3.x, v3.y, v3.z);

				float4 tmpBarycentric;

				intersect = computeTetBarycentric(a, b, c, d, contact, tmpBarycentric);

				if (!intersect)
				{
					PxVec3 tmpClosest = Gu::closestPtPointTetrahedron(contact, a, b, c, d);
					const PxVec3 v = contact - tmpClosest;
					PxReal tmpDist = v.dot(v);
					if (tmpDist < sqDist)
					{
						sqDist = tmpDist;
						simTetRef = PxgEncodeDeformableTetrahedron(softbodyId, tmpSimTetInd);
						computeTetBarycentric(a, b, c, d, tmpClosest, barycentric);
					}
				}
				else
				{
					simTetRef = PxgEncodeDeformableTetrahedron(softbodyId, tmpSimTetInd);
					barycentric = tmpBarycentric;
					sqDist = 0.f;
				}
			}

			PxU32 resultWarp = __ballot_sync(FULL_MASK, intersect);

			// If any of the candidate sim tets overlap, we take the first one and abort
			if (resultWarp != 0)
			{
				PxU32 firstThreadIntersect = lowestSetIndex(resultWarp);
				if (idxInWarp == firstThreadIntersect)
				{
					outContactInfo[contactIndex].ref1 = simTetRef;
					outBarycentric[contactIndex] = barycentric;
				}

				valid = true;
				break;
			}
		}

		if (!valid)
		{
			//if none of the contact intersect, we keep the shortest distance
			PxU32 winnerLane = 0xffffffff;
			sqDist = warpReduction<MinOpFloat, PxReal>(FULL_MASK, sqDist, winnerLane);

			if (idxInWarp == winnerLane)
			{
				barycentric.x = PxClamp(barycentric.x, 0.f, 1.f);
				barycentric.y = PxClamp(barycentric.y, 0.f, 1.f);
				barycentric.z = PxClamp(barycentric.z, 0.f, 1.f);
				barycentric.w = PxClamp(barycentric.w, 0.f, 1.f);
				
				outContactInfo[contactIndex].ref1 = simTetRef;
				outBarycentric[contactIndex] = barycentric;
			}
		}
	}

}


extern "C" __global__
__launch_bounds__(512, 1)
void sb_fem_contact_remap_to_simLaunch(
	const PxgSoftBody* PX_RESTRICT	softbodies,
	const float4*					point,
	float4*							outBarycentric0,	//output
	float4*							outBarycentric1,	//output
	PxgDeformablePairInfo*			outContactInfo,		//output
	PxU32*							totalContactCount,	//output
	PxU32*							prevContactCount,
	const PxU32						maxNbContacts
)
{
	const PxU32 warpIdx = threadIdx.y;
	const PxU32 idxInWarp = threadIdx.x;
	// One row per grid: the first remaps side 1, the second side 0.
	// softbodyFemContactApplyCollisionToSimMeshMapping launches the second row for a volume pair alone.
	const PxU32 pairIndex = blockIdx.y;

	const PxU32 globalWarpIdx = warpIdx + blockIdx.x * blockDim.y;

	PxU32 numContacts = *totalContactCount;
	if (numContacts > maxNbContacts)
	{
		numContacts = maxNbContacts;
		if (idxInWarp == 0 && globalWarpIdx == 0)
			*totalContactCount = maxNbContacts;
	}
	const PxU32 startOffset = *prevContactCount;

	// Side 0 is a collision mesh vertex, whose embedding in the simulation mesh is already available, so
	// this row is a lookup per contact rather than a search per warp. It shares a launch with the row
	// below only because both remap this stream, and could be a kernel of its own running beside it.
	if (pairIndex == 1)
	{
		const PxU32 globalThreadIdx = idxInWarp + blockDim.x * globalWarpIdx;
		const PxU32 nbThreads = gridDim.x * blockDim.x * blockDim.y;

		for (PxU32 contactIndex = globalThreadIdx + startOffset; contactIndex < numContacts; contactIndex += nbThreads)
		{
			const PxU64 vertexRef = outContactInfo[contactIndex].ref0;
			PX_ASSERT(PxgGetDeformableFeatureType(vertexRef) == PxgDeformableRef::eVERTEX);

			const PxU32 softbodyId = PxgGetDeformableId(vertexRef);
			const PxgSoftBody& softbody = softbodies[softbodyId];
			const PxU32 vertIdx = PxgGetDeformableFeatureId(vertexRef);

			outContactInfo[contactIndex].ref0 =
				PxgEncodeDeformableTetrahedron(softbodyId, softbody.mVertsRemapInGridModel[vertIdx]);
			outBarycentric0[contactIndex] = softbody.mVertsBarycentricInGridModel[vertIdx];
		}

		return;
	}

	// Side 1 is a tetrahedron met at the contact point, so its simulation tetrahedron has to be searched
	// for. Each warp takes one contact.
	for (PxU32 contactIndex = globalWarpIdx + startOffset; contactIndex < numContacts; contactIndex += gridDim.x * blockDim.y)
	{
		PxgDeformablePairInfo contactInfo = outContactInfo[contactIndex];

		PxU64 featureRef = contactInfo.ref1;
		float4* remapBarycentric = outBarycentric1;

		const float4 witness = point[contactIndex];
		const PxVec3 contact(witness.x, witness.y, witness.z);
	
		const PxU32 softbodyId = PxgGetDeformableId(featureRef);
		const PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU32 tetInd = PxgGetDeformableFeatureId(featureRef);

		const uint4* simTetrahedrons = softbody.mSimTetIndices;
		const float4* simVerts = softbody.mSimPosition_InvMass;
		const PxU32* tetRemapColToSim = softbody.mTetsRemapColToSim;
		const PxU32* tetAccumulatedRemap = softbody.mTetsAccumulatedRemapColToSim;

		const PxU32 endInd = tetAccumulatedRemap[tetInd];
		const PxU32 startInd = (tetInd != 0) ? tetAccumulatedRemap[tetInd - 1] : 0;
		const PxU32 range = endInd - startInd;

		const PxU32 nbIterations = (range + WARP_SIZE - 1) / WARP_SIZE;

		PxU64 simTetRef = PxgEncodeInvalidDeformableRef(softbodyId);
		float4 barycentric;
		PxReal sqDist = PX_MAX_F32;
		PxU32 winnerLane = 0xffffffff;
		bool valid = false;
		bool hasTet = false;
		for (PxU32 j = 0; j < nbIterations; ++j)
		{
			bool intersect = false;
		
			const PxU32 workIndex = idxInWarp + j * WARP_SIZE;
			if (workIndex < range)
			{
				const PxU32 index = startInd + workIndex;
				PxU32 tmpSimTetInd = tetRemapColToSim[index];
				const uint4 simTet = simTetrahedrons[tmpSimTetInd];
				const float4 v0 = simVerts[simTet.x];
				const float4 v1 = simVerts[simTet.y];
				const float4 v2 = simVerts[simTet.z];
				const float4 v3 = simVerts[simTet.w];

				const PxVec3 a(v0.x, v0.y, v0.z);
				const PxVec3 b(v1.x, v1.y, v1.z);
				const PxVec3 c(v2.x, v2.y, v2.z);
				const PxVec3 d(v3.x, v3.y, v3.z);

				if (isValidTet(a, b, c, d))
				{
					hasTet = true;
					float4 tmpBarycentric;

					intersect = computeTetBarycentric(a, b, c, d, contact, tmpBarycentric);

					if (!intersect)
					{
						PxVec3 tmpClosest = Gu::closestPtPointTetrahedron(contact, a, b, c, d);
						const PxVec3 v = contact - tmpClosest;
						PxReal tmpDist = v.dot(v);
						if (tmpDist < sqDist)
						{
							sqDist = tmpDist;
							simTetRef = PxgEncodeDeformableTetrahedron(softbodyId, tmpSimTetInd);
							computeTetBarycentric(a, b, c, d, tmpClosest, barycentric);
						}
					}
					else
					{
						simTetRef = PxgEncodeDeformableTetrahedron(softbodyId, tmpSimTetInd);
						barycentric = tmpBarycentric;
						//sqDist = 0.f;
					}
				}
			}

			PxU32 resultWarp = __ballot_sync(FULL_MASK, intersect);

			if (resultWarp != 0)
			{
				PxU32 firstThreadIntersect = lowestSetIndex(resultWarp);
				if (idxInWarp == firstThreadIntersect)
				{
					outContactInfo[contactIndex].ref1 = simTetRef;
					remapBarycentric[contactIndex] = barycentric;
				}

				valid = true;
				break;
			}
		}

		if (__ballot_sync(FULL_MASK, hasTet) == 0)
		{
			// No candidate simulation tetrahedron is geometrically valid, so this participant has no
			// feature and simTetRef still holds the invalid reference it started with.
			outContactInfo[contactIndex].ref1 = simTetRef;

			remapBarycentric[contactIndex] = make_float4(0.f, 0.f, 0.f, 0.f);
		}
		else if (!valid)
		{
			//if none of the contact intersect, we keep the shorest distance
			sqDist = warpReduction<MinOpFloat, PxReal>(FULL_MASK, sqDist, winnerLane);

			if (idxInWarp == winnerLane)
			{
				/*barycentric.x = PxClamp(barycentric.x, 0.f, 1.f);
				barycentric.y = PxClamp(barycentric.y, 0.f, 1.f);
				barycentric.z = PxClamp(barycentric.z, 0.f, 1.f);
				barycentric.w = PxClamp(barycentric.w, 0.f, 1.f);*/

				outContactInfo[contactIndex].ref1 = simTetRef;

				remapBarycentric[contactIndex] = barycentric;
			}
		}
	}

}




__device__ static inline void sbClothVertCollision(
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4									curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	PxgDbDbContactWriter&						writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 particleIndex = curPair.y;
	const PxU32 tetrahedronIdx = curPair.z;
	
	PxgShape softbodyShape, clothShape;
	PxU32 softbodyCacheRef, clothCacheRef;
	LoadShapePair<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx,	gpuShapes, 
		softbodyShape, softbodyCacheRef, clothShape, clothCacheRef);

	const PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[clothCacheRef];

	const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
	const PxgSoftBody& softbody = softbodies[softbodyId];

	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	const PxgFEMCloth& cloth = clothes[clothId];

	const PxgDeformablePairInfo* filterPairs = softbody.mVolumeSurfaceFilterPairs;
	const PxU32 nbFilterPairs = softbody.mNumVolumeSurfaceFilterPairs;

	// A filter pair for this tetrahedron and this cloth vertex suppresses contact generation between
	// them. Also checks whether either of the two is filtered against any.
	if(findVolumeSurfaceVertexFilter(filterPairs, nbFilterPairs, softbodyId, PxgDeformableRef::ANY_FEATURE, clothId, particleIndex) ||
	   findVolumeSurfaceVertexFilter(filterPairs, nbFilterPairs, softbodyId, tetrahedronIdx, clothId, PxgDeformableRef::ANY_FEATURE) ||
	   findVolumeSurfaceVertexFilter(filterPairs, nbFilterPairs, softbodyId, tetrahedronIdx, clothId, particleIndex))
	{
		return;
	}

	const float4* positions = cloth.mPosition_InvMass;
	
	const uint4 tetIdx = softbody.mTetIndices[tetrahedronIdx];
	const float4* tetmeshVerts = softbody.mPosition_InvMass;

	PxU8 surfaceMask = softbody.mTetMeshSurfaceHint[tetrahedronIdx];

	if (surfaceMask)
	{
		const float4 a_ = tetmeshVerts[tetIdx.x];
		const float4 b_ = tetmeshVerts[tetIdx.y];
		const float4 c_ = tetmeshVerts[tetIdx.z];
		const float4 d_ = tetmeshVerts[tetIdx.w];

		const PxVec3 a(a_.x, a_.y, a_.z);
		const PxVec3 b(b_.x, b_.y, b_.z);
		const PxVec3 c(c_.x, c_.y, c_.z);
		const PxVec3 d(d_.x, d_.y, d_.z);

		const float4 p_ = positions[particleIndex];

		const PxVec3 p(p_.x, p_.y, p_.z);

		const float4 result = PointOutsideOfPlane4(p, a, b, c, d);

		PxVec3 closestP;
		PxVec3 n2;
		PxReal sqDist = PX_MAX_F32;
		closestPtTetrahedron(p, surfaceMask, a, b, c, d, n2, closestP, sqDist);

		PxVec3 v = closestP - p;
		const float m = v.normalize();

		PxVec3 n;
		PxReal pen;
		const PxVec3 contact = closestP;

		//this means the p is inside for the tetrahedron
		if (result.x >= 0.f && result.y >= 0.f && result.z >= 0.f && result.w >= 0.f)
		{
			n = n2;
			pen = -m;
		}
		else
		{
			const PxReal sqContactDistance = cDistance * cDistance;

			if (sqDist > sqContactDistance)
			{
				return;
			}

			pen = m;
			n = m > 1e-10f && v.dot(n2) > 0.707f ? v : n2;
		}

		int32_t index = 0xffffffff;
		// writing contact if there is space left
		{
			index = atomicAdd(writer.totalContactCount, 1);
			if (index >= writer.maxContacts)
			{
				index = 0xffffffff;
			}
		}

		if (index != 0xffffffff)
		{
			PxU64 vertRef = PxgEncodeDeformableVertex(clothId, particleIndex);
			PxU64 tetRef = PxgEncodeDeformableTetrahedron(softbodyId, tetrahedronIdx);

			writer.writeSurfaceVolumeContact(index, make_float4(contact.x, contact.y, contact.z, 0.f),
				make_float4(n.x, n.y, n.z, pen), vertRef, tetRef);
		}
	}
}

extern "C" __global__
void sb_clothVertContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	const PxU32									stackSizeBytes,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgDbDbContactWriter						writer
)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const uint4* PX_RESTRICT pairs = reinterpret_cast<uint4*>(stackPtr);
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	//each thread does collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalThreadIdx; i < numPairs; i += gridDim.x * blockDim.x)
	{
		const uint4 curPair = pairs[i];

		sbClothVertCollision(tolerenceLength, cmInputs,
			curPair, transformCache, contactDistance, gpuShapes,
			clothes, softbodies,
			writer);
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}



