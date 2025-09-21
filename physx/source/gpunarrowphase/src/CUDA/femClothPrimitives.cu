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

#include "foundation/PxBasicTemplates.h"
#include "foundation/PxMat33.h"
#include "foundation/PxPlane.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxQuat.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxHeightFieldSample.h"

#include "PxNodeIndex.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgFEMCore.h"
#include "PxgNpKernelIndices.h"
#include "PxgParticleSystem.h"
#include "PxgSimulationCoreDesc.h"

#include "PxsTransformCache.h"

#include "cudaNpCommon.h"
#include "GuDistancePointTriangle.h"
#include "schlockShared.h"

#include <vector_types.h>

#include "capsuleTriangle.cuh"
#include "convexTriangle.cuh"
#include "dataReadWriteHelper.cuh"
#include "epa.cuh"
#include "heightfieldUtil.cuh"
#include "utils.cuh"
#include "reduction.cuh"
#include "deformableElementFilter.cuh"
#include "sphereCollision.cuh"
#include "deformableCollision.cuh"
#include "sdfCollision.cuh"
#include "reduction.cuh"

using namespace physx;
using namespace schlock;

extern "C" __host__ void initNarrowphaseKernels19() {}


struct PxgOutputIndex
{
	const PxU32 mask;
	PxU32 startIndex;


	//Must be called by a full warp
	PX_FORCE_INLINE __device__ PxgOutputIndex(const PxU32 mask, PxU32* totalContactCount) : mask(mask), startIndex(0xFFffFFff)
	{
		const PxU32 numContactsGenerated = __popc(mask);
		const PxU32 threadIndexInWarp = threadIdx.x & 31;
		if (threadIndexInWarp == 0 && numContactsGenerated > 0)
		{
			startIndex = atomicAdd(totalContactCount, numContactsGenerated);
		}
		startIndex = __shfl_sync(FULL_MASK, startIndex, 0);
	}

	PX_FORCE_INLINE __device__ bool hasContact()
	{
		//return mask & (1 << threadIdx.x);
		const PxU32 threadIndexInWarp = threadIdx.x & 31;
		return mask & (1 << threadIndexInWarp);
	}

	//Returns only a valid index if the thread contributes an element by setting the appropriate bit in mask (see constructor) to 1
	PX_FORCE_INLINE __device__ PxU32 getIndex()
	{
		const PxU32 threadIndexInWarp = threadIdx.x & 31;
		PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);
		return startIndex + offset;
	}
};


static __device__ bool computeBarycentric(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3 p, float4& barycentric)
{
	const PxReal minEps = -1e-4f;
	const PxReal maxEps = 1.f + 1e-4f;
	const PxVec3 v0 = b - a;
	const PxVec3 v1 = c - a;
	const PxVec3 v2 = p - a;

	const float d00 = v0.dot(v0);
	const float d01 = v0.dot(v1);
	const float d11 = v1.dot(v1);
	const float d20 = v2.dot(v0);
	const float d21 = v2.dot(v1);

	const float denom = d00 * d11 - d01 * d01;
	const float v = (d11 * d20 - d01 * d21) / denom;
	const float w = (d00 * d21 - d01 * d20) / denom;
	const float u = 1.f - v - w;

	barycentric.x = u; barycentric.y = v; barycentric.z = w;
	barycentric.w = 0.f;

	return (u >= minEps && u <= maxEps) &&
		(v >= minEps && v <= maxEps) &&
		(w >= minEps && w <= maxEps);
}

__device__ static inline PxU32 vertPlaneCollide(
	const PxVec3& a,
	const PxTransform& planeTransform,
	const PxReal contactDist,
	float4& normalPens						//output
)
{
	const PxVec3 pInPlaneSpace = planeTransform.transformInv(a);

	//get the plane normal
	const PxVec3 normal = planeTransform.q.getBasisVector0();

	if (contactDist >= pInPlaneSpace.x)
	{
		normalPens.x = normal.x;
		normalPens.y = normal.y;
		normalPens.z = normal.z;
		normalPens.w = pInPlaneSpace.x;
		return 1;
	}

	return 0;
}

__device__ static inline PxU32 triSphereCollide(
	const PxVec3& a,
	const PxVec3& b,
	const PxVec3& c,
	const PxVec3& sphereCenter,
	const PxReal radius,
	const PxReal contactDist,
	float4*	contacts,						//output
	float4* normalPens,						//output
	float4*	barycentrics					//output
)
{
	PxU32 count = 0;

	if (0) // three vertices of triangle vs. sphere contacts
	{
	
		count += spheresphere(a, sphereCenter, 0.f,
			radius, contactDist, normalPens[count]);
		
		count += spheresphere(b, sphereCenter, 0.f,
			radius, contactDist, normalPens[count]);

		count += spheresphere(c, sphereCenter, 0.f,
			radius, contactDist, normalPens[count]);
		
		//this means we will generate the vert contacts in other kernels so we don't need to generate contacts in here
		if(count == 3)
			return 0;

	}

	
	count = 0;

	const PxReal inflatedRadius = radius + contactDist;
	const PxReal sqInflatedRadius = inflatedRadius * inflatedRadius;

	PxReal s, t;
	const PxVec3 cloestPt = closestPtPointTriangle(sphereCenter, a, b, c, s, t);

	//const PxVec3 cloestPt2 = Gu::closestPtPointTriangle2(sphereCenter, a, b, c, ab, ac);

	//float4 tBarycentric;

	//bool intersect = computeBarycentric(a, b, c, cloestPt2, tBarycentric);


	const PxVec3 v = cloestPt - sphereCenter;
	const PxReal sqDist = v.dot(v);

	

	if (sqInflatedRadius > sqDist)
	{
		PxVec3 n;
		const PxReal dist = PxSqrt(sqDist);

		if (dist > 1e-8f)
		{
			const PxReal recipMag = 1.f / dist;
			n = v * recipMag;
		}
		else
		{
			const PxVec3 ab = b - a;
			const PxVec3 ac = c - a;

			//triangle surface normal
			n = ab.cross(ac).getNormalized();
		}

		const PxVec3 contact = sphereCenter + n * radius;

		contacts[count].x = contact.x;
		contacts[count].y = contact.y;
		contacts[count].z = contact.z;
		contacts[count].w = 0.f;

		normalPens[count].x = n.x;
		normalPens[count].y = n.y;
		normalPens[count].z = n.z;
		normalPens[count].w = dist - radius;
		barycentrics[count++] = make_float4(1.f - s - t, s, t, 0.f);
	}

	return count;
	
}


//return number of contacts, we assume mPosition_InvMass is world space. We need to make sure this works
//as local space mesh
__device__ static inline PxU32 fcSphereCollision(
	const PxsCachedTransform& sphereTransformCache,
	const PxsCachedTransform& clothTransformCache,
	const PxReal radius,
	const PxReal cDistance,
	const PxgFEMCloth& femCloth,
	const PxU32 triangleIdx,

	float4*						outPoint,									//output
	float4*						outNormalPen,								//output
	float4*						outBarycentric
)
{
	float4* position_invMass = femCloth.mPosition_InvMass;
	uint4* vertIndices = femCloth.mTriangleVertexIndices;

	const uint4 triIdx = vertIndices[triangleIdx];
	const float4 af = position_invMass[triIdx.x];
	const float4 bf = position_invMass[triIdx.y];
	const float4 cf = position_invMass[triIdx.z];

	const PxVec3 a(af.x, af.y, af.z);
	const PxVec3 b(bf.x, bf.y, bf.z);
	const PxVec3 c(cf.x, cf.y, cf.z);

	const PxVec3 sphereCenter = sphereTransformCache.transform.p;

	return triSphereCollide(a, b, c, sphereCenter, radius,
		cDistance, outPoint, outNormalPen, outBarycentric);

}

__device__ PxVec3 closestPtPointSegment(const PxVec3& p0, const PxVec3& dir, const PxReal len, const PxVec3& point)
{
	PxReal diff = (point - p0).dot(dir);
	diff = PxClamp(diff, 0.f, len);
	return p0 + dir * diff;
}

__device__ static inline PxU32 fcCapsuleCollision(
	const PxsCachedTransform& capsuleTransformCache,
	const PxsCachedTransform& clothTransformCache,
	const PxgShape& capsuleShape, 
	const PxgShape& clothShape,
	const PxReal cDistance,
	const PxgFEMCloth& femCloth,
	const PxU32 triangleIdx,

	float4*						outPoint,									//output
	float4*						outNormalPen,								//output
	float4*						outBarycentric
)
{
	float4* position_invMass = femCloth.mPosition_InvMass;
	uint4* vertIndices = femCloth.mTriangleVertexIndices;

	const uint4 triIdx = vertIndices[triangleIdx];
	const float4 af = position_invMass[triIdx.x];
	const float4 bf = position_invMass[triIdx.y];
	const float4 cf = position_invMass[triIdx.z];
	const PxVec3 a(af.x, af.y, af.z);
	const PxVec3 b(bf.x, bf.y, bf.z);
	const PxVec3 c(cf.x, cf.y, cf.z);

	//Geometries : Capsule
	const PxTransform meshToCapsule = capsuleTransformCache.transform.transformInv(clothTransformCache.transform);

	const PxReal halfHeight = capsuleShape.scale.scale.x;
	const PxReal capsuleRadius = capsuleShape.scale.scale.y;

	//Capsule in triangle mesh space
	const PxVec3 dir = capsuleTransformCache.transform.q.getBasisVector0();
	const PxVec3 tmp = dir * halfHeight;
	//const PxVec3 capsuleCenterInMesh = clothTransformCache.transform.transformInv(capsuleTransformCached.transform.p);
	//const PxVec3 capsuleDirInMesh = clothTransformCache.transform.rotateInv(tmp);
	//Geometries : Capsule in mesh local space
	const PxVec3 capsuleCenterInMesh = capsuleTransformCache.transform.p;
	const PxVec3 p = capsuleTransformCache.transform.p + tmp;
	const PxVec3 q = capsuleTransformCache.transform.p - tmp;

	// Triangle scaling, triangle verts in shape space
	//const PxVec3 v0 = vertex2Shape(a, clothShape.scale.scale, clothShape.scale.rotation);
	//const PxVec3 v1 = vertex2Shape(b, clothShape.scale.scale, clothShape.scale.rotation);
	//const PxVec3 v2 = vertex2Shape(c, clothShape.scale.scale, clothShape.scale.rotation);

	//const PxVec3 triNormal = (b - a).cross(c - a).getNormalized();
	const PxReal inflatedRadius = capsuleRadius + cDistance;

	const PxReal sqInflatedRadius = inflatedRadius * inflatedRadius;

	PxU32 numContacts = 0;

	if (0) // three vertices of triangle vs. capsule contacts
	{
		const PxReal len = halfHeight * 2.f;
		const PxVec3 closest0 = closestPtPointSegment(q, dir, len, a);
		const PxVec3 closest1 = closestPtPointSegment(q, dir, len, b);
		const PxVec3 closest2 = closestPtPointSegment(q, dir, len, c);

		PxVec3 nor0 = a - closest0;
		PxVec3 nor1 = b - closest1;
		PxVec3 nor2 = c - closest2;

		PxReal dist0 = nor0.normalize();
		PxReal dist1 = nor1.normalize();
		PxReal dist2 = nor2.normalize();
		
		if (dist0 < inflatedRadius)
		{			
			outPoint[numContacts] = make_float4(closest0.x, closest0.y, closest0.z, 0.f);
			outNormalPen[numContacts] = make_float4(nor0.x, nor0.y, nor0.z, dist0 - capsuleRadius);
			outBarycentric[numContacts] = make_float4(1.f, 0.f, 0.f, 0.f);
			numContacts++;
		}

		if (dist1 < inflatedRadius)
		{
			outPoint[numContacts] = make_float4(closest1.x, closest1.y, closest1.z, 0.f);
			outNormalPen[numContacts] = make_float4(nor1.x, nor1.y, nor1.z, dist1 - capsuleRadius);
			outBarycentric[numContacts] = make_float4(0.f, 1.f, 0.f, 0.f);
			numContacts++;
		}

		if (dist2 < inflatedRadius)
		{
			outPoint[numContacts] = make_float4(closest2.x, closest2.y, closest2.z, 0.f);
			outNormalPen[numContacts] = make_float4(nor2.x, nor2.y, nor2.z, dist2 - capsuleRadius);
			outBarycentric[numContacts] = make_float4(0.f, 0.f, 1.f, 0.f);
			numContacts++;
		}

		if (numContacts == 3)
			return 0;
	}

	if(0) // cloth triangle vs. two axis points of capsule
	{
		PxReal s0, t0;
		PxVec3 closestPt0 = closestPtPointTriangle(p, a, b, c, s0, t0);
		PxReal s1, t1;
		PxVec3 closestPt1 = closestPtPointTriangle(q, a, b, c, s1, t1);

		PxVec3 nor0 = closestPt0 - p;
		PxVec3 nor1 = closestPt1 - q;

		PxReal dist0 = nor0.normalize();
		PxReal dist1 = nor1.normalize();

		if (dist0 <= inflatedRadius)
		{
			PxReal pen = dist0 - capsuleRadius;
			//const PxVec3 worldP = clothTransformCache.transform.transform(outContacts[i]);
			outPoint[numContacts] = make_float4(closestPt0.x, closestPt0.y, closestPt0.z, 0.f);
			outNormalPen[numContacts] = make_float4(nor0.x, nor0.y, nor0.z, pen);
			outBarycentric[numContacts++] = make_float4(1 - s0 - t0, s0, t0, 0.f);
		}

		if (dist1 <= inflatedRadius)
		{
			PxReal pen = dist1 - capsuleRadius;
			//const PxVec3 worldP = clothTransformCache.transform.transform(outContacts[i]);
			outPoint[numContacts] = make_float4(closestPt1.x, closestPt1.y, closestPt1.z, 0.f);
			outNormalPen[numContacts] = make_float4(nor1.x, nor1.y, nor1.z, pen);
			outBarycentric[numContacts++] = make_float4(1 - s1 - t1, s1, t1, 0.f);
		}

	}

	if(1)
	{

		PxReal t, u, v;
		PxReal sqDist = distanceSegmentTriangleSquared(p, q, a, b, c, t, u, v);

		//capsule overlap with triangles
		if (sqInflatedRadius > sqDist)
		{

			const PxVec3 pq = q - p;
			const PxVec3 pointOnSegment = p + pq * t;
			const PxReal w = 1.f - u - v;
			const PxVec3 pointOnTriangle = a * w + b * u + c * v;
			const PxVec3 normal = (pointOnTriangle - pointOnSegment).getNormalized();

			//point on triangle
			const PxVec3 worldP = pointOnTriangle;
			float4 barycetric = make_float4(w, u, v, 0.f);

			const PxReal dist = PxSqrt(sqDist);

			PxReal pen = dist - capsuleRadius;
			
			const PxVec3 pointOnCapsule = pointOnSegment + capsuleRadius * normal;
			outPoint[numContacts] = make_float4(pointOnCapsule.x, pointOnCapsule.y, pointOnCapsule.z, 0.f);
			outNormalPen[numContacts] = make_float4(normal.x, normal.y, normal.z, pen);
			outBarycentric[numContacts++] = barycetric;
		}

		return numContacts;
	}

}


__device__ static inline PxU32 vertCapsuleCollide(
	const PxVec3&				a,
	const PxsCachedTransform& capsuleTransformCache,
	const PxsCachedTransform& clothTransformCache,
	const PxgShape& capsuleShape,
	const PxgShape& clothShape,
	const PxReal cDistance,

	float4&						outPoint,									//output
	float4&						outNormalPen								//output
)
{

	//Geometries : Capsule
	//const PxTransform meshToCapsule = capsuleTransformCache.transform.transformInv(trimeshTransformCache.transform);

	const PxReal halfHeight = capsuleShape.scale.scale.x;
	const PxReal capsuleRadius = capsuleShape.scale.scale.y;

	//Capsule in triangle mesh space
	const PxVec3 dir = capsuleTransformCache.transform.q.getBasisVector0();
	const PxVec3 tmp = dir * halfHeight;
	//const PxVec3 capsuleCenterInMesh = clothTransformCache.transform.transformInv(capsuleTransformCache.transform.p);
	//const PxVec3 capsuleDirInMesh = clothTransformCache.transform.rotateInv(tmp);
	//Geometries : Capsule in mesh local space
	const PxVec3 capsuleCenterInMesh = capsuleTransformCache.transform.p;
	const PxVec3 p = capsuleTransformCache.transform.p + tmp;
	const PxVec3 q = capsuleTransformCache.transform.p - tmp;

	// Triangle scaling, triangle verts in shape space
	//const PxVec3 v0 = vertex2Shape(a, clothShape.scale.scale, clothShape.scale.rotation);
	//const PxVec3 v1 = vertex2Shape(b, clothShape.scale.scale, clothShape.scale.rotation);
	//const PxVec3 v2 = vertex2Shape(c, clothShape.scale.scale, clothShape.scale.rotation);

	//const PxVec3 triNormal = (b - a).cross(c - a).getNormalized();
	const PxReal inflatedRadius = capsuleRadius + cDistance;
	
	const PxReal len = halfHeight * 2.f;
	const PxVec3 closest0 = closestPtPointSegment(q, dir, len, a);

	PxVec3 nor0 = a - closest0;
		
	PxReal dist0 = nor0.normalize();
	

	if (dist0 < inflatedRadius)
	{

		outPoint = make_float4(a.x, a.y, a.z, 0.f);
		outNormalPen = make_float4(nor0.x, nor0.y, nor0.z, dist0 - capsuleRadius);
		return 1;
	}

	return 0;
}


extern "C" __global__
void cloth_SphereContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,

	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,

	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,

	PxgRigidFilterPair*							filterPairs,
	const PxU32									nbFilterPairs,
	const PxU32									stackSizeBytes,
	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,
	PxgFEMContactWriter							writer
)
{
	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x*blockDim.x;

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 midPhasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(midPhasePairsFound, stackSize);

	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);

	const PxU32 numWarpsRequired = (numPairs + (WARP_SIZE - 1)) / WARP_SIZE;
	const PxU32 numThreadsRequired = numWarpsRequired * WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

#if 0
	if (globalThreadIdx == 0)
	{
		const PxU32 sizeofBlock = gridDim.x * blockDim.x;
		printf("numWarpsRequired %i numThreadsRequired %i sizeofBlock %i numPairs %i\n", numWarpsRequired, numThreadsRequired, sizeofBlock, numPairs);
	}
#endif

	float4 contacts[5];
	float4 normalPens[5];
	float4 tBarycentric[5];

	//each thread do collision detection between a triangle and sphere/plane
	for (PxU32 i = globalThreadIdx; i < numThreadsRequired; i += gridDim.x * blockDim.x)
	{
		PxU32 numContacts = 0;
		PxNodeIndex rigidId;
		PxU32 femClothId = 0xffffffff;
		PxU32 triangleId = 0xffffffff;
		PxU32 cmIdx;
		
		PxU32 rigidMaterialIndex = 0xFFFFFFFF;
		if (i < numPairs)
		{
			const uint4 curPair = pairs[i];
			cmIdx = curPair.x;
			triangleId = curPair.y;

#if 0
			if (threadIdx.x == 0)
				printf("cmIdx %i tetrahedronIdx %i\n", cmIdx, tetrahedronIdx);
#endif
			PxgShape clothShape, rigidShape;
			PxU32 clothCacheRef, rigidCacheRef;
			LoadShapePair<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
				clothShape, clothCacheRef, rigidShape, rigidCacheRef);
			
			PxGeometryType::Enum rigidType = PxGeometryType::Enum(rigidShape.type);

			rigidId = shapeToRigidRemapTable[rigidCacheRef];
			femClothId = clothShape.particleOrSoftbodyId;
			rigidMaterialIndex = rigidShape.materialIndex;

			const PxgFEMCloth& femCloth = femClothes[femClothId];
			uint4* vertIndices = femCloth.mTriangleVertexIndices;

			const uint4 triIdx = vertIndices[triangleId];

			const PxU32 clothMask0 = PxEncodeClothIndex(femClothId, triIdx.x);
			const PxU32 clothMask1 = PxEncodeClothIndex(femClothId, triIdx.y);
			const PxU32 clothMask2 = PxEncodeClothIndex(femClothId, triIdx.z);

			//If we find all verts in the filter pair list, we don't need to generate contacts between the rigid shape and the triangle
			bool skipTriangle = find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask0) && 
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask1) &&
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask2);
			
			if (!skipTriangle)
			{
				const PxsCachedTransform& rigidTransformCache = transformCache[rigidCacheRef];
				PxReal cDistance = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
			
				if(rigidType == PxGeometryType::eSPHERE)
				{
					const PxsCachedTransform& clothTransformCache = transformCache[clothCacheRef];

					const PxMeshScale& rigidScale = rigidShape.scale;

					numContacts = fcSphereCollision(
						rigidTransformCache,
						clothTransformCache,
						rigidScale.scale.y,
						cDistance,
						femCloth,
						triangleId,
						contacts,
						normalPens,
						tBarycentric
					);
				}
				else 
				{
					assert(rigidType == PxGeometryType::eCAPSULE);

					const PxsCachedTransform& trimeshTransformCache = transformCache[clothCacheRef];

					numContacts = fcCapsuleCollision(
						rigidTransformCache,
						trimeshTransformCache,
						rigidShape,
						clothShape,
						cDistance,
						femCloth,
						triangleId,
						contacts,
						normalPens,
						tBarycentric
					);
				}
			}
		}

		//Now we need to do a runsum...
		PxU32 inclusiveSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, numContacts);

		PxU32 totalCount = __shfl_sync(FULL_MASK, inclusiveSum, 31);
		inclusiveSum -= numContacts;

		PxU32 startIndex = 0xFFffFFff;
		if (threadIndexInWarp == 0 && totalCount > 0)
		{
			startIndex = atomicAdd(writer.totalContactCount, totalCount);
		}
		
		startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

#if 0
		if (startIndex == 0)
			printf("%i %i globalThreadIdx %i numContacts %i totalCount %i startIndex %i inclusiveSum %i \n", threadIndexInWarp, i, globalThreadIdx, numContacts, totalCount, startIndex, inclusiveSum);
#endif

		//first 8 bits for cloth, second 24 bits for triangle index

		if (numContacts > 0)
		{
			PxU64 pairInd0 = rigidId.getInd();
			PxU32 pairInd1 = PxEncodeClothIndex(femClothId, triangleId);

			const PxReal rest = restDistance[cmIdx];

			for (PxU32 j = 0; j < numContacts; ++j)
			{
				const PxU32 index = startIndex + inclusiveSum + j;
				float4 cont = contacts[j];
				cont.w = rest;
				writer.writeRigidVsDeformableContact(index, cont, normalPens[j], tBarycentric[j], pairInd0, pairInd1, rigidMaterialIndex, rigidId);
			}
		}
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, midPhasePairsFound * sizeof(uint4));
	}
}

extern "C" __global__
//__launch_bounds__(WARP_SIZE, 16)
void cloth_SphereVertexContactGenLaunch(
	PxU32 numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgFEMCloth* PX_RESTRICT femClothes,

	const PxNodeIndex* PX_RESTRICT			shapeToRigidRemapTable,

	PxgRigidFilterPair*						filterPairs,
	const PxU32								nbFilterPairs,

	PxgFEMContactWriter					writer
)
{
	//one block deal with one test
	unsigned int cmIdx = blockIdx.x;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	PxU32 rigidMaterialIndex = 0xFFFFFFFF;
	if (cmIdx < numWorkItems)
	{
		PxgShape clothShape, rigidShape;
		PxU32 clothCacheRef, rigidCacheRef;
		LoadShapePair<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape, clothCacheRef, rigidShape, rigidCacheRef);

		PxGeometryType::Enum rigidType = PxGeometryType::Enum(rigidShape.type);

		const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];

		const PxU32 femClothId = clothShape.particleOrSoftbodyId;

		PxgFEMCloth& femCloth = femClothes[femClothId];

		const float4* verts = femCloth.mPosition_InvMass;

		const PxU32 numVerts = femCloth.mNbVerts;

		const PxsCachedTransform& rigidTransformCache = transformCache[rigidCacheRef];

		PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		rigidMaterialIndex = rigidShape.materialIndex;

		float4 contact;
		float4 normalPen;
		PxU64 pairInd0;
		PxU32 pairInd1;
		//float4 tBarycentric;

		//const PxU32 numRequiredWarps = (numVerts + WARP_SIZE - 1) / WARP_SIZE;
		//const PxU32 numRequiredThreads = numRequiredWarps * WARP_SIZE;

		const PxU32 numRequiredThreads = (numVerts + 31) & (~31);

		const PxU32 threadIndexInBlock = threadIdx.x + threadIdx.y * WARP_SIZE;

		const PxU32 threadIndexInWarp = threadIdx.x;

#if 0
		if (threadIndexInBlock == 0)
			printf("numRequiredWarps %i numRequiredThreads %i numVerts %i totalThreadInABlock %i\n", numRequiredWarps, numRequiredThreads, numVerts, blockDim.x * blockDim.y);
#endif

		//each thread do collision detection between a triangle and sphere/plane
		for (PxU32 i = threadIndexInBlock; i < numRequiredThreads; i += blockDim.x * blockDim.y)
		{
			//numContacts will be either 0 or 1
			PxU32 numContacts = 0;

			if (i < numVerts)
			{
				const PxU32 clothMask = PxEncodeClothIndex(femClothId, i);

				//If we find the vert index in the filter pairs, we don't need to generate contacts between the rigid shape and the vertex
				bool skipVertex = find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask);

				if (!skipVertex)
				{
					contact = verts[i];
					//tBarycentric = make_float4(0.f, 0.f, 0.f, 1.f);
					const PxVec3 p(contact.x, contact.y, contact.z);

					if (rigidType == PxGeometryType::eSPHERE)
					{

						const PxMeshScale& scale = rigidShape.scale;
						const PxReal radius = scale.scale.y;

						numContacts = spheresphere(p, rigidTransformCache.transform.p, 0.f,
							radius, contactDist, normalPen);
					}
					else
					{
						assert(rigidType == PxGeometryType::eCAPSULE);

						const PxsCachedTransform& trimeshTransformCache = transformCache[clothCacheRef];

						numContacts = vertCapsuleCollide(
							p,
							rigidTransformCache,
							trimeshTransformCache,
							rigidShape,
							clothShape,
							contactDist,
							contact,
							normalPen
						);
					}
				}

				if (numContacts > 0)
				{
					pairInd0 = rigidId.getInd();
					pairInd1 = PxEncodeClothIndex(femClothId, i);
					contact.w = restDistance[cmIdx];
				}
			}

			const PxU32 mask = __ballot_sync(FULL_MASK, numContacts);

			PxU32 totalCount = __popc(mask);

			PxU32 threadOffset = warpScanExclusive(mask, threadIndexInWarp);

			PxU32 startIndex = 0xFFffFFff;
			if (threadIndexInWarp == 0 && totalCount > 0)
			{
				startIndex = atomicAdd(writer.totalContactCount, totalCount);
			}

			startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

#if 0
			if (startIndex == 0)
				printf("%i %i globalWarpIndex %i numContacts %i totalCount %i startIndex %i inclusiveSum %i \n", threadIndexInWarp, i, globalWarpIndex, numContacts, totalCount, startIndex, inclusiveSum);
#endif
			//first 8 bits for cloth, second 24 bits for triangle index

			if (numContacts > 0)
			{
				const PxU32 index = startIndex + threadOffset;
				writer.writeRigidVsDeformableContact(index, contact, normalPen, make_float4(0.f, 0.f, 0.f, 1.f /*vertex contact*/), pairInd0, pairInd1, rigidMaterialIndex, rigidId);
			}
		}
	}
}


extern "C" __global__
//__launch_bounds__(WARP_SIZE, 16)
void cloth_planeVertContactGenLaunch(
	PxU32 numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxgRigidFilterPair*				filterPairs,
	const PxU32						nbFilterPairs,

	PxgFEMContactWriter					writer				
)
{
	//one block deal with one test
	unsigned int cmIdx = blockIdx.x;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	PxU32 rigidMaterialIndex = 0xFFFFFFFF;
	if (cmIdx < numWorkItems)
	{
		PxgShape clothShape, rigidShape;
		PxU32 clothCacheRef, rigidCacheRef;
		LoadShapePair<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape, clothCacheRef, rigidShape, rigidCacheRef);

		const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];

		const PxU32 femClothId = clothShape.particleOrSoftbodyId;

		const PxgFEMCloth& femCloth = femClothes[femClothId];

		const float4* verts = femCloth.mPosition_InvMass;

		const PxU32 numVerts = femCloth.mNbVerts;

		const PxsCachedTransform& rigidTransformCache = transformCache[rigidCacheRef];

		PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		rigidMaterialIndex = rigidShape.materialIndex;

		float4 contact;
		float4 normalPen;
		PxU64 pairInd0;
		PxU32 pairInd1;

		/*
		const PxU32 numRequiredWarps = (numVerts + WARP_SIZE - 1) / WARP_SIZE;
		const PxU32 numRequiredThreads = numRequiredWarps * WARP_SIZE;
		*/

		const PxU32 numRequiredThreads = (numVerts + 31) & (~31);

		const PxU32 threadIndexInBlock = threadIdx.x + threadIdx.y * WARP_SIZE;

		const PxU32 threadIndexInWarp = threadIdx.x;

#if 0
		if (threadIndexInBlock == 0)
			printf("numRequiredWarps %i numRequiredThreads %i numVerts %i totalThreadInABlock %i\n", numRequiredWarps, numRequiredThreads, numVerts, blockDim.x * blockDim.y);
#endif
		//each thread do collision detection between a triangle and sphere/plane
		for (PxU32 i = threadIndexInBlock; i < numRequiredThreads; i += blockDim.x * blockDim.y)
		{
			//numContacts will be either 0 or 1
			PxU32 numContacts = 0;

			if (i < numVerts)
			{
				const float4 p = verts[i];
				const PxVec3 clothP(p.x, p.y, p.z);

				assert(PxGeometryType::Enum(rigidShape.type) == PxGeometryType::ePLANE);

				numContacts = vertPlaneCollide(clothP, rigidTransformCache.transform, contactDist, normalPen);

				if (numContacts > 0)
				{
					const PxU32 clothMask = PxEncodeClothIndex(femClothId, i);

					//If we find the vert index in the filter pairs, we don't need to generate contacts between the rigid shape and the vertex
					numContacts = find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask) ? 0 : numContacts;

					pairInd0 = rigidId.getInd();
					pairInd1 = PxEncodeClothIndex(femClothId, i);

					const PxVec3 rigidP = clothP - normalPen.w * PxVec3(normalPen.x, normalPen.y, normalPen.z);
					contact = make_float4(rigidP.x, rigidP.y, rigidP.z, restDistance[cmIdx]);
				}
			}

			const PxU32 mask = __ballot_sync(FULL_MASK, numContacts);

			PxU32 totalCount = __popc(mask);

			PxU32 threadOffset = warpScanExclusive(mask, threadIndexInWarp);

			PxU32 startIndex = 0xFFffFFff;
			if (threadIndexInWarp == 0 && totalCount > 0)
			{
				startIndex = atomicAdd(writer.totalContactCount, totalCount);
			}

			startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

#if 0
			if (startIndex == 0)
				printf("%i %i globalWarpIndex %i numContacts %i totalCount %i startIndex %i inclusiveSum %i \n", threadIndexInWarp, i, globalWarpIndex, numContacts, totalCount, startIndex, inclusiveSum);
#endif
			//first 8 bits for cloth, second 24 bits for triangle index

			if (numContacts > 0)
			{
				const PxU32 index = startIndex + threadOffset;
				writer.writeRigidVsDeformableContact(index, contact, normalPen, make_float4(0.f, 0.f, 0.f, 1.f), pairInd0, pairInd1, rigidMaterialIndex, rigidId);
			}
		}
	}
}

//PX_ALIGN_PREFIX(16)
struct ConvexTriangleScratch : public ConvexScratch
{
	PxTransform trimeshToConvexTransform;
	PxTransform convexTransform;

};// PX_ALIGN_SUFFIX(16);
PX_COMPILE_TIME_ASSERT(sizeof(ConvexTriangleScratch) <= 32 * 16 * sizeof(float));


__device__ PxMat33 findRotationMatrixFromZ(const PxVec3& to)
{
	const PxReal e = to.z;
	const PxReal f = PxAbs(e);

	if (f <= 0.9999f)
	{
		// PT: please keep the normal case first for PS3 branch prediction

		// Normal case, to and from are not parallel or anti-parallel
		const PxReal vx = -to.y;
		const PxReal vy = to.x;
		const PxReal h = 1.0f / (1.0f + e); /* optimization by Gottfried Chen */
		const PxReal hvx = h * vx;
		const PxReal hvxy = hvx * vy;

		
		const PxVec3 col0(e + hvx * vx, hvxy, vy);
		const PxVec3 col1(hvxy, e + h * vy*vy, -vx);
		const PxVec3 col2(-vy, vx, e);

		return PxMat33(col0, col1, col2);
	}
	else
	{
		PxVec3 from(0.0f, 0.0f, 1.0f);
		PxVec3 absFrom(0.0f, 1.0f, 0.0f);

		PxVec3 u = absFrom - from;
		PxVec3 v = absFrom - to;

		const PxReal c1 = -(2.0f / u.dot(u));
		const PxReal c2 = -(2.0f / v.dot(v));
		const PxReal c3 = c1 * c2 * (u.dot(v));

		const PxVec3 c1u = u * c1;
		const PxVec3 c2v = v * c2;
		const PxVec3 c3v = v * c3;

		PxReal temp0 = c1u.x;
		PxReal temp1 = c2v.x;
		PxReal temp2 = c3v.x;

		PxVec3 col0 = u * temp2 + v * temp1 + u * temp0;//V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col0.x = col0.x + 1.f;// = V3SetX(col0, FAdd(V3GetX(col0), one));

		temp0 = c1u.y;
		temp1 = c2v.y;
		temp2 = c3v.y;


		PxVec3 col1 = u * temp2 + v * temp1 + u * temp0; //V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col1.y = col1.y + 1.f; //V3SetY(col1, FAdd(V3GetY(col1), one));

		temp0 = c1u.z;
		temp1 = c2v.z;
		temp2 = c3v.z;

		PxVec3 col2 = u * temp2 + v * temp1 + u * temp0;//V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col2.z = col2.z + 1.f;//V3SetZ(col2, FAdd(V3GetZ(col2), one));

		return PxMat33(col0, col1, col2);
	}
}


__device__ bool contain(PxVec3 p, PxVec3 sp1, PxVec3 sp0)
{
	const PxReal eps = PX_EPS_REAL;

	const PxReal tx = p.x; const PxReal ty = p.y;

	const PxReal jy = sp1.y;
	const PxReal iy = sp0.y;

	const PxReal jx = sp1.x;
	const PxReal ix = sp0.x;

	////if p is one of the end point, we will return intersect
	//const bool con0 = (tx > jx) && (ty > jy);
	//const bool con1 = (tx > ix) && (ty > iy);

	//if (con0 || con1)
	//{
	//	return true;
	//}

	const PxU32 yflag0 = jy > ty;
	const PxU32 yflag1 = iy > ty;

	//ML: the only case the ray will intersect this segment is when the p's y is in between two segments y
	if (yflag0 != yflag1)
	{
		//ML: choose ray, which start at p and every points in the ray will have the same y component
		//t1 = (yp - yj)/(yi - yj)
		//qx = xj + t1*(xi-xj)
		//t = qx - xp > 0 for the ray and segment intersection happen
		const PxReal jix = ix - jx;
		const PxReal jiy = iy - jy;
		//const FloatV jtx = FSub(tx, jy);
		const PxReal jty = ty - jy;
		const PxReal part1 = jty * jix;
		//const FloatV part2 = FMul(jx, jiy);
		//const FloatV part3 = FMul(V3Sub(tx, eps), jiy);
		const PxReal part2 = (jx + eps) * jiy;//FMul(FAdd(jx, eps), jiy);
		const PxReal part3 = tx * jiy;//FMul(tx, jiy);

		const bool comp = jiy > 0.f;
		const PxReal tmp = part1 + part2;
		const PxReal comp1 = comp ? tmp : part3;// FSel(comp, tmp, part3);
		const PxReal comp2 = comp ? part3 : tmp; // FSel(comp, part3, tmp);

		if (comp1 >= comp2)
		{
			return true;
		}
	}

	return false;
}

__device__ bool contains(PxVec3* verts, const PxU32 numVerts, const PxVec3& p)
{

	const PxReal tx = p.x;
	const PxReal ty = p.y;

	const PxReal eps = PX_EPS_REAL;

	PxU32 intersectionPoints = 0;

	PxU32 i = 0, j = numVerts - 1;

	for (; i < numVerts; j = i++)
	{
		const PxReal jy = verts[j].y;
		const PxReal iy = verts[i].y;

		const PxReal jx = verts[j].x;
		const PxReal ix = verts[i].x;

		//(verts[i].y > test.y) != (points[j].y > test.y) 
		const PxU32 yflag0 = jy > ty;
		const PxU32 yflag1 = iy > ty;

		//ML: the only case the ray will intersect this segment is when the p's y is in between two segments y
		if (yflag0 != yflag1)
		{

			//ML: choose ray, which start at p and every points in the ray will have the same y component
			//t1 = (yp - yj)/(yi - yj)
			//qx = xj + t1*(xi-xj)
			//t = qx - xp > 0 for the ray and segment intersection happen
			const PxReal jix = ix - jx;
			const PxReal jiy = iy - jy;
			//const FloatV jtx = FSub(tx, jy);
			const PxReal jty = ty - jy;
			const PxReal part1 = jty * jix;
			//const FloatV part2 = FMul(jx, jiy);
			//const FloatV part3 = FMul(V3Sub(tx, eps), jiy);
			const PxReal part2 = (jx + eps) * jiy;
			const PxReal part3 = tx * jiy;

			const bool comp = jiy > 0.f;
			const PxReal tmp = part1 + part2;
			const PxReal comp1 = comp ? tmp : part3;
			const PxReal comp2 = comp ? part3 : tmp;

			if (comp1 >= comp2)
			{
				if (intersectionPoints == 1)
				{
					return false;
				}
				//printf("(%i, %i)\n", i, j);
				intersectionPoints++;
			}
		}
	}
	return intersectionPoints > 0;
}


__device__ bool containsParallel(const PxVec3& start, const PxU32 numVerts, const PxVec3& p)
{
	const PxU32 tI = threadIdx.x;

	const PxReal tx = p.x;
	const PxReal ty = p.y;

	const PxReal eps = PX_EPS_REAL;
	bool intersect = false;
	
	
	PxVec3 endP = shuffle(FULL_MASK, start, tI + 1 == numVerts ? 0 : tI + 1);

	/*if (tI < 3)
	{
		const PxU32 endIndex = (tI + 1 == numVerts ? 0 : tI + 1);
		printf("tI %i (%i , %i) s(%f, %f, %f) e(%f, %f, %f)\n", tI, tI, endIndex, start.x, start.y, start.z, endP.x, endP.y, endP.z);
		
	}*/

	
	const PxReal jy = endP.y;
	const PxReal iy = start.y;

	const PxReal minY = PxMin(jy, iy);
	const PxReal maxY = PxMax(jy, iy);

	//minY == maxY indicates the segment parallel to the ray so it won't have intersection
	if ((minY != maxY) && ty >= minY && ty <= maxY && tI < numVerts)
	{
		const PxReal jx = endP.x;
		const PxReal ix = start.x;
		//ML: choose ray, which start at p and every points in the ray will have the same y component
		//t1 = (yp - yj)/(yi - yj)
		//qx = xj + t1*(xi-xj)
		//t = qx - xp > 0 for the ray and segment intersection happen
		const PxReal jix = ix - jx;
		const PxReal jiy = iy - jy;
		//const FloatV jtx = FSub(tx, jy);
		const PxReal jty = ty - jy;
		const PxReal part1 = jty * jix;
		//const FloatV part2 = FMul(jx, jiy);
		//const FloatV part3 = FMul(V3Sub(tx, eps), jiy);
		const PxReal part2 = (jx + eps) * jiy;
		const PxReal part3 = tx * jiy;

		const bool comp = jiy > 0.f;
		const PxReal tmp = part1 + part2;
		const PxReal comp1 = comp ? tmp : part3;
		const PxReal comp2 = comp ? part3 : tmp;
		if (comp1 >= comp2)
		{
			//const PxU32 endIndex = tI + 1 == numVerts ? 0 : tI + 1;
			//printf("startIndex %i endIndex %i\n", tI, endIndex);
			intersect = true;
		}
	}

	//if numIntersects is odd number, this point contained in the polygon
	PxU32 mask = __ballot_sync(FULL_MASK, intersect);
	const PxU32 numIntersects = __popc(mask);
	return (numIntersects % 2) == 1;
}

__device__ PxReal signed2DTriArea(const PxVec3& a, const PxVec3& b, const PxVec3& c)
{

	const PxVec3 ca = a - c;
	const PxVec3 cb = b - c;

	const PxReal t0 = ca.x * cb.y;// FMul(V3GetX(ca), V3GetY(cb));
	const PxReal t1 = ca.y * cb.x;// FMul(V3GetY(ca), V3GetX(cb));

	return t0 - t1;
}

//pAA is polygon segment point, pBB is triangle segment point
__device__ bool segmentIntersect(const PxVec3& ipA, const PxVec3& ipB, 
	const PxVec3& rpA, const PxVec3& rpB, const PxVec3& rMin, const PxVec3& rMax,
	PxVec3& pAA, PxVec3& pBB)
{
	const PxVec3 iMin = PxVec3(PxMin(ipA.x, ipB.x), PxMin(ipA.y, ipB.y), PxMin(ipA.z, ipB.z));
	const PxVec3 iMax = PxVec3(PxMax(ipA.x, ipB.x), PxMax(ipA.y, ipB.y), PxMax(ipA.z, ipB.z));

	const bool xCon = iMin.x > rMax.x || rMin.x > iMax.x;
	const bool yCon = iMin.y > rMax.y || rMin.y > iMax.y;

	if (xCon || yCon)
		return false;

	PxReal a1 = signed2DTriArea(rpA, rpB, ipA);
	PxReal a2 = signed2DTriArea(rpA, rpB, ipB);

	if ((a1 * a2) < 0.f)//FAllGrtr(zero, FMul(a1, a2)))
	{
		PxReal a3 = signed2DTriArea(ipA, ipB, rpA);
		PxReal a4 = signed2DTriArea(ipA, ipB, rpB);

		if ((a3 * a4) < 0.f)
		{
			//these two segment intersect in 2d
			const PxReal t0 = a1 / (a2 - a1);// FMul(a1, FRecip(FSub(a2, a1)));

			pAA = ipA - t0 * (ipB - ipA); //polygon point

			const PxReal t1 = a3 / (a4 - a3);
			pBB = rpA - t1 * (rpB - rpA); //triangle point
			return true;
		}
	}

	return false;

}

__device__ void addContacts(const PxU32 mask, const PxVec3& contact, const PxVec3& normal, const PxReal pen, 
	volatile float * s_contactsTransposed, PxU32& numContacts)
{
	//each thread's mask might be different. We need every thread aggree on the same mask
	int b = mask & __shfl_xor_sync(FULL_MASK, mask, 16);
	b &= __shfl_xor_sync(FULL_MASK, b, 8);
	b &= __shfl_xor_sync(FULL_MASK, b, 4);
	b &= __shfl_xor_sync(FULL_MASK, b, 2);
	b &= __shfl_xor_sync(FULL_MASK, b, 1);

	PxU32 count = __popc(b);

	int index = warpScanExclusive(b, threadIdx.x) + numContacts;

	if (b & (1 << threadIdx.x) && index < NUM_TMP_CONTACTS_PER_PAIR)
	{
		s_contactsTransposed[8 * index + 0] = contact.x;
		s_contactsTransposed[8 * index + 1] = contact.y;
		s_contactsTransposed[8 * index + 2] = contact.z;
		s_contactsTransposed[8 * index + 3] = pen;

		s_contactsTransposed[8 * index + 4] = normal.x;
		s_contactsTransposed[8 * index + 5] = normal.y;
		s_contactsTransposed[8 * index + 6] = normal.z;
		s_contactsTransposed[8 * index + 7] = 0.f;



		/*s_scratch->contactPen[index] = make_float4(contact.x, contact.y, contact.z, pen);
		s_scratch->normal[index] = normal;*/
	}

	numContacts += count;
}

__device__ void addContacts(const PxU32 mask, const float4& barycentric, const PxVec3& normal, const PxReal pen,
	volatile float * s_contactsTransposed, PxU32& numContacts)
{
	//each thread's mask might be different. We need every thread aggree on the same mask
	int b = mask & __shfl_xor_sync(FULL_MASK, mask, 16);
	b &= __shfl_xor_sync(FULL_MASK, b, 8);
	b &= __shfl_xor_sync(FULL_MASK, b, 4);
	b &= __shfl_xor_sync(FULL_MASK, b, 2);
	b &= __shfl_xor_sync(FULL_MASK, b, 1);

	PxU32 count = __popc(b);

	int index = warpScanExclusive(b, threadIdx.x) + numContacts;

	if (b & (1 << threadIdx.x) && index < NUM_TMP_CONTACTS_PER_PAIR)
	{
		s_contactsTransposed[8 * index + 0] = barycentric.x;
		s_contactsTransposed[8 * index + 1] = barycentric.y;
		s_contactsTransposed[8 * index + 2] = barycentric.z;
		s_contactsTransposed[8 * index + 3] = pen;

		s_contactsTransposed[8 * index + 4] = normal.x;
		s_contactsTransposed[8 * index + 5] = normal.y;
		s_contactsTransposed[8 * index + 6] = normal.z;
		s_contactsTransposed[8 * index + 7] = 0.f;



		/*s_scratch->contactPen[index] = make_float4(contact.x, contact.y, contact.z, pen);
		s_scratch->normal[index] = normal;*/
	}

	numContacts += count;
}

__device__ void addContacts(const PxU32 mask, const float4& barycentric0, const float4& barycentric1, const PxVec3& normal, const PxReal pen,
	volatile float * s_contactsTransposed, PxU32& numContacts)
{
	//each thread's mask might be different. We need every thread aggree on the same mask
	int b = mask & __shfl_xor_sync(FULL_MASK, mask, 16);
	b &= __shfl_xor_sync(FULL_MASK, b, 8);
	b &= __shfl_xor_sync(FULL_MASK, b, 4);
	b &= __shfl_xor_sync(FULL_MASK, b, 2);
	b &= __shfl_xor_sync(FULL_MASK, b, 1);

	PxU32 count = __popc(b);

	int index = warpScanExclusive(b, threadIdx.x) + numContacts;

	if (b & (1 << threadIdx.x) && index < NUM_TMP_CONTACTS_PER_PAIR)
	{
		s_contactsTransposed[12 * index + 0] = barycentric0.x;
		s_contactsTransposed[12 * index + 1] = barycentric0.y;
		s_contactsTransposed[12 * index + 2] = barycentric0.z;
		s_contactsTransposed[12 * index + 3] = pen;


		s_contactsTransposed[12 * index + 4] = barycentric1.x;
		s_contactsTransposed[12 * index + 5] = barycentric1.y;
		s_contactsTransposed[12 * index + 6] = barycentric1.z;
		s_contactsTransposed[12 * index + 7] = 0.f;

		s_contactsTransposed[12 * index + 8] = normal.x;
		s_contactsTransposed[12 * index + 9] = normal.y;
		s_contactsTransposed[12 * index + 10] = normal.z;
		s_contactsTransposed[12 * index + 11] = 0.f;
	}

	numContacts += count;
}

//plane0 is convex, plane1 is triangle, axis is the triangle normal
__device__ static PxU32 polyClip4(const PxPlane plane0, PxVec3 v0, PxU32 nbVerts0, const PxPlane plane1, PxVec3 v1,
	const PxVec3 axis, PxReal contactDist,
	volatile float * s_contactsTransposed
)
{
	PxU32 numContacts = 0;

	const PxU32 tI = threadIdx.x;

	const PxMat33 rot = findRotationMatrixFromZ(axis);

	//transform triangle points to 2d
	PxVec3 triangleP;
	if (threadIdx.x < 3)
		triangleP = rot.transform(v1);

	//transform polygon points to 2d
	PxVec3 polyPoint;
	if (threadIdx.x < nbVerts0)
		polyPoint = rot.transform(v0);

	const PxReal sqContactDist = contactDist * contactDist;

	PxVec3 normal;
	PxVec3 contactPoint;

	PxReal dist = PX_MAX_F32;
	PxU32 mask = 0;

	if (0)
	{
		for (int i = 0; i < 3; i++)
		{
			const PxVec3 v = shuffle(FULL_MASK, triangleP, i);

			if (containsParallel(polyPoint, nbVerts0, v))
				mask |= 1 << i;
		}

		if (mask & (1 << threadIdx.x))
		{
			//v1 is the triangle point
			const PxVec3 projectedP = plane0.project(v1);

			const PxVec3 dir = projectedP - v1;
			const PxReal cosTheta = dir.dot(plane0.n);

			const PxReal sqDist = dir.dot(dir);

			//triangle is on the positive side of the polygon
			if (cosTheta < 0.f)
			{
				if (sqDist > sqContactDist)
				{
					mask &= ~(1 << threadIdx.x); //clear the mask
				}
				else
				{
					dist = PxSqrt(sqDist);
					normal = -dir.getNormalized();
				}

			}
			else
			{
				dist = -PxSqrt(sqDist);
				normal = dir.getNormalized();
			}
		}

		addContacts(mask, v1, normal, dist, s_contactsTransposed, numContacts);
	}


	//if (tI == 0)
	//	printf("triangle numContacts %i\n", numContacts);

	mask = 0;
	dist = PX_MAX_F32;

	for (int j = 0; j < nbVerts0; j++)
	{
		//2d contain
		const PxVec3 v = shuffle(FULL_MASK, polyPoint, j);

		if (containsParallel(triangleP, 3, v))
			mask |= 1 << j;
	}

	if (mask & (1 << threadIdx.x))
	{

		//printf("tI %i v0(%f, %f, %f)\n", tI, v0.x, v0.y, v0.z);
		const PxVec3 tProjPoint = plane1.project(v0);

		const PxVec3 dir = v0 - tProjPoint;
		const PxReal cosTheta = dir.dot(plane0.n);

		const PxReal sqDist = dir.dot(dir);

		//triangle is on the positive side of the polygon
		if (cosTheta < 0.f)
		{

			if (sqDist > sqContactDist)
			{
				mask &= ~(1 << threadIdx.x); //clear the mask
			}
			else
			{
				dist = PxSqrt(sqDist);
				normal = -dir.getNormalized();
				contactPoint = tProjPoint;
			}

		}
		else
		{

			dist = -PxSqrt(sqDist);
			normal = dir.getNormalized();
			contactPoint = tProjPoint;
		}
	}
	addContacts(mask, contactPoint, normal, dist, s_contactsTransposed, numContacts);


	//PxVec3 normal;
	//PxVec3 contactPoint;

	dist = PX_MAX_F32;
	//PxU32 mask = 0;

	PxVec3 pAA, pBB;
	for (PxU32 rStart = 0, rEnd = 2; rStart < 3; rEnd = rStart++)
	{
		const PxVec3 rpA = shuffle(FULL_MASK, triangleP, rStart);
		const PxVec3 rpB = shuffle(FULL_MASK, triangleP, rEnd);

		const PxVec3 rMin = PxVec3(PxMin(rpA.x, rpB.x), PxMin(rpA.y, rpB.y), PxMin(rpA.z, rpA.z));
		const PxVec3 rMax = PxVec3(PxMax(rpA.x, rpB.x), PxMax(rpA.y, rpB.y), PxMax(rpA.z, rpA.z));


		PxVec3 ipB = shuffle(FULL_MASK, polyPoint, tI + 1 == nbVerts0 ? 0 : tI + 1);
		//triangle segment(rpA, rpB), polygon segment(ipA, ipB). pAA intersect point in polgyon. pBB intersect point in triangle
		bool hasContacts = (tI < nbVerts0) ? segmentIntersect(polyPoint, ipB, rpA, rpB, rMin, rMax, pAA, pBB) : false;

		if (hasContacts)
		{
			//printf("has edge contacts \n");
			//rotate back to 3d space
			const PxVec3 pA = rot.transformTranspose(pAA);// M33TrnspsMulV3(rot, pAA);
			contactPoint = rot.transformTranspose(pBB); // M33TrnspsMulV3(rot, pBB);

			const PxVec3 dir = pA - contactPoint;
			const PxReal cosTheta = dir.dot(plane0.n);

			const PxReal sqDist = dir.dot(dir);

			//triangle is on the positive side of the polygon
			if (cosTheta < 0.f)
			{
				if (sqDist > sqContactDist)
					hasContacts = false;
				else
				{
					dist = PxSqrt(sqDist);
					normal = -dir.getNormalized();
				}
			}
			else
			{
				dist = -PxSqrt(sqDist);
				normal = dir.getNormalized();
			}
		}


		const PxU32 mask = __ballot_sync(FULL_MASK, hasContacts);

		int index = warpScanExclusive(mask, threadIdx.x) + numContacts;

		if (hasContacts && index < NUM_TMP_CONTACTS_PER_PAIR)
		{
			s_contactsTransposed[8 * index + 0] = contactPoint.x;
			s_contactsTransposed[8 * index + 1] = contactPoint.y;
			s_contactsTransposed[8 * index + 2] = contactPoint.z;
			s_contactsTransposed[8 * index + 3] = dist;

			s_contactsTransposed[8 * index + 4] = normal.x;
			s_contactsTransposed[8 * index + 5] = normal.y;
			s_contactsTransposed[8 * index + 6] = normal.z;
			s_contactsTransposed[8 * index + 7] = 0.f;

			/*s_scratch->contactPen[index] = make_float4(contactPoint.x, contactPoint.y, contactPoint.z, dist);
			s_scratch->normal[index] = normal;*/
		}

		numContacts += __popc(mask);

	}

	//polyClipVert(plane0, v0, polyPoint, nbVerts0, plane1, v1, triangleP, axis, sqContactDist, s_contactsTransposed, numContacts);

	//polyClipEdge(plane0, polyPoint, nbVerts0, plane1, triangleP, axis, sqContactDist, rot, s_contactsTransposed, numContacts);

	return numContacts;

}
	

static __device__ PxU32 selectPolygonFeatureIndex2(const ConvexScratch* s_scratch, const PxVec3& minNormal, const PxU32 featureIndex, PxU32& faceIndex2)
{
	//If featureIndex is one of the polygon face index, we are done. Otherwise, we need to chose
	//polygon face index
	PxU32 polyFaceIndex = featureIndex;
	faceIndex2 = 0xFFFFFFFF;

	//unsigned mask_featureIndex = __ballot_sync(syncMask, featureIndex == EDGE_FEATURE_IDX || featureIndex == TRI_FEATURE_IDX);
	if (featureIndex == EDGE_FEATURE_IDX || featureIndex == TRI_FEATURE_IDX)
	{
		return selectPolygonFeatureIndex(s_scratch, minNormal, featureIndex, faceIndex2);
	}

	return polyFaceIndex;
}

static __device__ void convexTriangleContactGen(
	ConvexTriangleScratch* s_scratch,
	const PxU32 globalWarpIndex,
	const PxNodeIndex rigidId,
	const PxU32 femClothId,
	const PxU32 triangleId,
	PxgFEMContactWriter& writer,
	PxgShape& rigidShape
)
{
	// SAT
	PxReal separation = -PX_MAX_REAL;
	PxVec3 minNormal(PX_MAX_REAL);
	PxU32 featureIndex = 0xffffffff;

	const PxU32 nbVerts = getNbVerts(s_scratch->nbEdgesNbHullVerticesNbPolygons);
	const PxU32 nbPolygons = getNbPolygons(s_scratch->nbEdgesNbHullVerticesNbPolygons);

	assert(nbVerts <= CONVEX_MAX_VERTICES_POLYGONS);
	assert(nbPolygons <= CONVEX_MAX_VERTICES_POLYGONS);

	for (PxU32 i = threadIdx.x; i < nbVerts; i += WARP_SIZE)
	{
		float4 v = s_scratch->getVertices()[i];
		s_scratch->convexScaledVerts[i] = vertex2Shape(PxVec3(v.x, v.y, v.z), s_scratch->convexScale, s_scratch->convexScaleRot);
	}

	__syncwarp();

	bool isSATPassed = convexMeshSAT<false>(
		s_scratch,
		featureIndex,
		separation,
		minNormal
	);

	const PxReal rest = s_scratch->restDist;

	if (isSATPassed)
	{
		// Select best polygon face
		PxU32 faceIndex2;
		const PxU32 faceIndex1 = selectPolygonFeatureIndex2(s_scratch, minNormal, featureIndex, faceIndex2);

		PxVec3 convexScale = s_scratch->convexScale;
		PxQuat convexScaleRot = s_scratch->convexScaleRot;

		const PxU32* polyDescs = s_scratch->getPolyDescs();
		const float4* planes = s_scratch->getPlanes();

		PxPlane triPlane(s_scratch->triLocVerts[0], s_scratch->triangleLocNormal);

		const float4* PX_RESTRICT vertices = s_scratch->getVertices();
		const PxU8* PX_RESTRICT vertexData8 = s_scratch->getVertexData8();

		PxVec3 verts0(0.f), verts1(0.f);

		if (threadIdx.x < 3)
			verts1 = s_scratch->triLocVerts[threadIdx.x];

		const PxReal contactDist = s_scratch->contactDist;
		const PxTransform convexTransform = s_scratch->convexTransform;

		__syncwarp(); //s_scratch and s_contactsTransposed points to the same shared memory. s_scratch is read above and s_contactsTransposed is written below.

		for (PxU32 faceIndex = faceIndex1; faceIndex != 0xFFFFFFFF;)
		{
			const float4 referencePolygon_plane = planes[faceIndex];
			PxPlane plane0(vertex2ShapeNormalVector(PxVec3(referencePolygon_plane.x, referencePolygon_plane.y, referencePolygon_plane.z),
				convexScale, convexScaleRot), referencePolygon_plane.w);
			plane0.normalize();
			const PxU32 polyDesc = polyDescs[faceIndex];

			{
				const PxU32 vref = getVRef8(polyDesc);

				if (threadIdx.x < getNbVerts(polyDesc))
					verts0 = vertex2Shape(PxLoad3(vertices[vertexData8[vref + threadIdx.x]]), convexScale, convexScaleRot);

				assert(getNbVerts(polyDesc) < 32);

				// WARNING! Contacts re-use same shared memory as intermediate buffers
				volatile float * s_contactsTransposed = reinterpret_cast<volatile float*>(s_scratch);

				PxU32 numContactsGenerated = polyClip4(plane0, verts0, getNbVerts(polyDesc), triPlane, verts1,
					minNormal, contactDist, s_contactsTransposed);

				//Make sure that shared memory writes are visible
				__syncwarp();

				//unsigned mask_numContactsGenerated = __ballot_sync(mask_predicate, numContactsGenerated);
				if (numContactsGenerated)
				{
					PxVec3 a = shuffle(FULL_MASK, verts1, 0);
					PxVec3 b = shuffle(FULL_MASK, verts1, 1);
					PxVec3 c = shuffle(FULL_MASK, verts1, 2);

					PxVec3 pa;
					PxReal sep;
					PxVec3 normal;
					float4 barycentric;
					bool isValid = false;

					if (threadIdx.x < numContactsGenerated)
					{
						pa = PxVec3(s_contactsTransposed[8 * threadIdx.x + 0], s_contactsTransposed[8 * threadIdx.x + 1],
							s_contactsTransposed[8 * threadIdx.x + 2]);
						sep = s_contactsTransposed[8 * threadIdx.x + 3];

						normal = PxVec3(s_contactsTransposed[8 * threadIdx.x + 4], s_contactsTransposed[8 * threadIdx.x + 5],
							s_contactsTransposed[8 * threadIdx.x + 6]);

						isValid = computeBarycentric(a, b, c, pa, barycentric);
					}

					int mask = __ballot_sync(FULL_MASK, isValid);

					numContactsGenerated = __popc(mask);

					if (numContactsGenerated)
					{
						if (numContactsGenerated > 5)
							mask = contactReduce<false, true, 5>(pa, sep, minNormal, mask, 0.f);
						
						PxgOutputIndex indexer(mask, writer.totalContactCount);

						if (mask > 0 && indexer.hasContact())
						{
							const PxU32 ind = indexer.getIndex();


							//need to transform pa/normal to world space
							const PxVec3 worldP = convexTransform.transform(pa);
							const PxVec3 worldNormal = convexTransform.rotate(normal);

							PxU64 pairInd0 = rigidId.getInd();
							PxU32 pairInd1 = PxEncodeClothIndex(femClothId, triangleId);

							writer.writeRigidVsDeformableContact(ind, make_float4(worldP.x, worldP.y, worldP.z, rest), make_float4(worldNormal.x, worldNormal.y, worldNormal.z, sep),
								barycentric, pairInd0, pairInd1, rigidShape.materialIndex, rigidId);
						}
					}
				}

				__syncwarp(); //s_contactsTransposed (shared memory) is read and written in the same loop - syncs between reads and writes required
			}
			faceIndex = faceIndex2;
			faceIndex2 = 0xFFFFFFFF;
		}
	}
}


__device__ static inline void clothConvexCollision(
	const PxU32 globalWarpIndex,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	ConvexTriangleScratch* s_scratch,
	PxgRigidFilterPair* filterPairs,	
	const PxU32 nbFilterPairs,
	PxgFEMContactWriter& writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 triangleIdx = curPair.y;

	PxgShape clothShape, rigidShape;
	PxU32 clothCacheRef, rigidCacheRef;
	LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		clothShape, clothCacheRef, rigidShape, rigidCacheRef);

	PxsCachedTransform clothTransformCache;
	PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);

	PxsCachedTransform rigidTransformCache;
	PxsCachedTransform_ReadWarp(rigidTransformCache, transformCache + rigidCacheRef);

	const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];
	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	const PxgFEMCloth &cloth = femClothes[clothId];

	const uint4* triVertInds = cloth.mTriangleVertexIndices;
	const uint4 triIdx = triVertInds[triangleIdx];

	const PxU32 clothMask0 = PxEncodeClothIndex(clothId, triIdx.x);
	const PxU32 clothMask1 = PxEncodeClothIndex(clothId, triIdx.y);
	const PxU32 clothMask2 = PxEncodeClothIndex(clothId, triIdx.z);

	//If we find all verts in the filter pair list, we don't need to generate contacts between the rigid shape and the triangle
	bool skipTriangle =	find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask0) &&
						find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask1) &&
						find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask2);

	if (skipTriangle)
		return;

	if (threadIdx.x == 0)
	{
		PxTransform clothToConvexTransform(rigidTransformCache.transform.transformInv(clothTransformCache.transform));
		s_scratch->trimeshToConvexTransform = clothToConvexTransform;
		s_scratch->convexTransform = rigidTransformCache.transform;
	}

	__syncwarp();

	if (threadIdx.x == 0)
	{
		// Shapes
		s_scratch->contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		s_scratch->restDist = restDistance[cmIdx];
		s_scratch->convexScaleRot = rigidShape.scale.rotation;
		//s_scratch->trimeshScaleRot = triangleShape.scale.rotation;

		s_scratch->convexScale = rigidShape.scale.scale;
		//s_scratch->trimeshScale = triangleShape.scale.scale;

		const PxU8* convexPtrA = reinterpret_cast<const PxU8 *>(rigidShape.hullOrMeshPtr);
		s_scratch->convexPtrA = convexPtrA;

		const float4 hull0_centerOfMass_f4 = *reinterpret_cast<const float4 *>(convexPtrA);
		const PxVec3 hull0_centerOfMass(hull0_centerOfMass_f4.x, hull0_centerOfMass_f4.y, hull0_centerOfMass_f4.z);

		// Transform CoM into shape space
		PxVec3 shapeSpaceCenterOfMass0 = vertex2Shape(hull0_centerOfMass, rigidShape.scale.scale, rigidShape.scale.rotation);
		s_scratch->convexCenterOfMass = shapeSpaceCenterOfMass0;

		convexPtrA += sizeof(float4);

		const uint4 tmp = *((uint4*)convexPtrA);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;
		s_scratch->nbEdgesNbHullVerticesNbPolygons = polyData0_NbEdgesNbHullVerticesNbPolygons;

		convexPtrA += sizeof(uint4);

		const float4 polyData0_Extents = *((float4*)(convexPtrA));
		PxVec3 convex_extents = PxLoad3(polyData0_Extents);

		const float4* trimeshVerts = cloth.mPosition_InvMass;
		
		uint4 triIndices = triVertInds[triangleIdx];

		float4 triV0_f4 = trimeshVerts[triIndices.x];
		float4 triV1_f4 = trimeshVerts[triIndices.y];
		float4 triV2_f4 = trimeshVerts[triIndices.z];

		const PxVec3 v0 = PxLoad3(triV0_f4);
		const PxVec3 v1 = PxLoad3(triV1_f4);
		const PxVec3 v2 = PxLoad3(triV2_f4);

		PxTransform trimeshToConv = s_scratch->trimeshToConvexTransform;
		PxVec3 triLocVerts0 = trimeshToConv.transform(v0);
		PxVec3 triLocVerts1 = trimeshToConv.transform(v1);
		PxVec3 triLocVerts2 = trimeshToConv.transform(v2);

		s_scratch->triLocVerts[0] = triLocVerts0;
		s_scratch->triLocVerts[1] = triLocVerts1;
		s_scratch->triLocVerts[2] = triLocVerts2;

		PxVec3 triangleLocNormal;
		triangleLocNormal = (triLocVerts1 - triLocVerts0).cross(triLocVerts2 - triLocVerts0).getNormalized();

		s_scratch->triangleLocNormal = triangleLocNormal;		
	}

	__syncwarp();

	convexTriangleContactGen(s_scratch, globalWarpIndex, rigidId, clothId, triangleIdx, writer, rigidShape);
}

extern "C" __global__
void cloth_convexContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,

	PxgRigidFilterPair* clothRigidPairs,
	const PxU32 numClothRigidPairs,

	const PxU32 stackSizeBytes,
	PxU32* PX_RESTRICT stackSizeNeededOnDevice,

	PxgFEMContactWriter writer
)
{
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;

	__shared__ volatile float sharedMem[warpsPerBlock][32 * 16]; // sizeof(ConvexTriangleScratch) is 2000B, allocate 4096B
	volatile float* s_WarpSharedMemory = sharedMem[threadIdx.y];
	
	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);
	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);

	//each warp does collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		const uint4 curPair = pairs[i];

		clothConvexCollision(
			i,
			tolerenceLength,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			femClothes,
			shapeToRigidRemapTable,
			(ConvexTriangleScratch*)s_WarpSharedMemory,
			clothRigidPairs,
			numClothRigidPairs,
			writer
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}

__device__ __forceinline__ static
PxVec3 distancePointSegment(const PxVec3 a, const PxVec3 b, const PxVec3 p)
{
	const PxVec3 ap = p - a;
	const PxVec3 ab = b - a;

	const PxReal nom = ap.dot(ab);
	const PxReal denom = ab.dot(ab);
	const PxReal tValue = PxClamp(nom / denom, 0.f, 1.f);
	const PxReal t = denom > 0.f ? tValue : 0.f;
	return a + ab * t;
}

static __device__ void computeClosestPolygon( const PxVec3& pVertexSpace, const PxPlane& plane, const PxVec3& vert, 
	const PxU32 nbVertices, PxVec3& closestP, PxReal& pen, const PxTransform& transform, const ConvexScratch* s_scratch)
{
	const PxVec3 rpA = shuffle(FULL_MASK, vert, threadIdx.x);
	const PxVec3 rpB = shuffle(FULL_MASK, vert, threadIdx.x + 1 == nbVertices ? 0 : threadIdx.x + 1);

	const PxVec3 dir = rpA - rpB;
	const PxVec3 edgePlaneNormal = (plane.n.cross(dir)).getNormalized();

	//project the point to the edge plane
	bool outside = false;
	PxReal signDist = -PX_MAX_F32;
	if (threadIdx.x < nbVertices)
	{
		signDist = (pVertexSpace - rpA).dot(edgePlaneNormal);
		outside = (signDist > 0.f);
	}

	outside = __ballot_sync(FULL_MASK, outside);

	PxU32 winnerLane;

	if (outside)
	{
		PxReal pToSemSqDist = PX_MAX_F32;

		//point is outside of the edge plane
		if (signDist > 0.f)
		{
			//compute the square distance from the point to the segment
			closestP = distancePointSegment(rpA, rpB, pVertexSpace);

			const PxVec3 dir = closestP - pVertexSpace;
			pToSemSqDist = dir.dot(dir);
		}

		pToSemSqDist = warpReduction<MinOpFloat, PxReal>(FULL_MASK, pToSemSqDist, winnerLane);

		closestP.x = __shfl_sync(FULL_MASK, closestP.x, winnerLane);
		closestP.y = __shfl_sync(FULL_MASK, closestP.y, winnerLane);
		closestP.z = __shfl_sync(FULL_MASK, closestP.z, winnerLane);

		pen = PxSqrt(pToSemSqDist);
	}
	else
	{
		pen = plane.distance(pVertexSpace);
		closestP = pVertexSpace - plane.n * pen;
		//closestP = plane.project(pVertexSpace);
	}
}

static __device__ void closestVertToConvex(const ConvexScratch* s_scratch, const PxVec3& shapeSpaceVert, PxVec3& outContact, 
	PxVec3& outNormal, PxReal& pen, const PxTransform& transform)
{
	const PxVec3 convexScale = s_scratch->convexScale;
	const PxQuat convexScaleRot = s_scratch->convexScaleRot;
	const PxVec3 pVertexSpace = shape2Vertex(shapeSpaceVert, convexScale, convexScaleRot);
	const PxU32 nbPolygons = getNbPolygons(s_scratch->nbEdgesNbHullVerticesNbPolygons);
	const PxU32* polyDescs = s_scratch->getPolyDescs();

	const float4* PX_RESTRICT vertices = s_scratch->getVertices();
	const PxU8* PX_RESTRICT vertexData8 = s_scratch->getVertexData8();
	const float4* PX_RESTRICT planes = s_scratch->getPlanes();

	PxReal maxVal = -PX_MAX_F32;
	PxU32 maxIndex = 0;
	bool outside = false;

	for (PxU32 i = threadIdx.x; i < nbPolygons; i += WARP_SIZE)
	{
		const PxPlane plane = reinterpret_cast<const PxPlane&>(planes[i]);
		const PxReal signDistVertexSpace = plane.distance(pVertexSpace);
		const PxReal scaleCoef = convexScaleRot.rotate(plane.n).multiply(convexScale).magnitude();
		const PxReal signDist = signDistVertexSpace * scaleCoef; // sign handled in vertex space

		outside = outside || signDist > 0.f;

		if (signDist > maxVal)
		{
			maxVal = signDist;
			maxIndex = i;
		}
	}

	const PxU32 mask = __ballot_sync(FULL_MASK, outside);

	PxVec3 closestP;
	PxVec3 normal;

	if (mask != 0)
	{
		//point is outside some of the planes
		PxReal minDist = PX_MAX_F32;

		//all of the threads go into this loop
		for (PxU32 i = 0; i < nbPolygons; i += WARP_SIZE)
		{
			PxU32 idx = i + threadIdx.x;
			bool intersect = false;

			if (idx < nbPolygons)
			{
				const PxPlane& plane = reinterpret_cast<const PxPlane&>(planes[idx]);
				const PxReal signDist = plane.distance(pVertexSpace);

				intersect = signDist > 0.f;
			}

			PxU32 mask = __ballot_sync(FULL_MASK, intersect);

			for (PxU32 j = mask; j; j = clearLowestSetBit(j))
			{
				// polygon id
				const PxU32 idx = i + lowestSetIndex(j);

				const PxPlane& plane = reinterpret_cast<const PxPlane&>(planes[idx]);

				const PxU32 polyDesc = polyDescs[idx];
				const PxU32 vref = getVRef8(polyDesc);

				const PxU32 nbVerts = getNbVerts(polyDesc);
				assert(nbVerts < 32);

				PxVec3 vert;
				if (threadIdx.x < nbVerts)
					vert = PxLoad3(vertices[vertexData8[vref + threadIdx.x]]);

				//a warp of threads, polyClosestP is in vertex space
				PxVec3 polyClosestP;
				PxReal polyDist;
				computeClosestPolygon(pVertexSpace, plane, vert, nbVerts, polyClosestP, polyDist, transform, s_scratch);

				if (polyDist < minDist)
				{
					minDist = polyDist;
					closestP = polyClosestP;
				}
			}
		}

		//normal in vertex space
		normal = pVertexSpace - closestP;

		//maxVal in shape space
		const PxVec3 normalShapeSpace = convexScaleRot.rotate(normal).multiply(convexScale);
		maxVal = normalShapeSpace.magnitude(); // signed distance is positive for points exterior to rigid.
	}
	else
	{
		PxU32 winnerLane;

		//all inside, get the plane with the largest maxVal(maxVal is negative)
		maxVal = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxVal, winnerLane);
		const PxU32 polyFaceIndex = __shfl_sync(FULL_MASK, maxIndex, winnerLane);

		const PxPlane plane = reinterpret_cast<const PxPlane&>(planes[polyFaceIndex]);
		closestP = plane.project(pVertexSpace);
		normal = plane.n;
	}

	if(threadIdx.x == 0)
	{
		// vertex to shape space conversion
		outContact = vertex2Shape(closestP, convexScale, convexScaleRot);
		outNormal = vertex2ShapeNormalVector(PxVec3(normal.x, normal.y, normal.z), convexScale, convexScaleRot);
		outNormal.normalizeSafe();

		// maxVal is already in shape space
		pen = maxVal;
	}
}


static __device__ void clothVertexConvexCollision(
	const uint4 curPair,
	const PxgContactManagerInput* PX_RESTRICT	 cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgFEMCloth* PX_RESTRICT femClothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	ConvexScratch* s_scratch,
	PxgRigidFilterPair* filterPairs,
	const PxU32 nbFilterPairs,
	PxgFEMContactWriter& writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 vertexIdx = curPair.y;

	PxgShape clothShape, rigidShape;
	PxU32 clothCacheRef, rigidCacheRef;
	LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		clothShape, clothCacheRef, rigidShape, rigidCacheRef);

	PxsCachedTransform clothTransformCache;
	PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);

	PxsCachedTransform rigidTransformCache;
	PxsCachedTransform_ReadWarp(rigidTransformCache, transformCache + rigidCacheRef);

	const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];
	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	PxgFEMCloth &cloth = femClothes[clothId];

	const PxU32 clothMask = PxEncodeClothIndex(clothId, vertexIdx);

	//If we find the vert index in the filter pairs, we don't need to generate contacts between the rigid shape and the vertex
	if (find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask))
		return;

	const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
	if (threadIdx.x == 0)
	{
		const PxU8* convexPtrA = reinterpret_cast<const PxU8 *>(rigidShape.hullOrMeshPtr);
		s_scratch->convexPtrA = convexPtrA;

		convexPtrA += sizeof(float4);
		const uint4 tmp = *((uint4*)convexPtrA);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;
		s_scratch->nbEdgesNbHullVerticesNbPolygons = polyData0_NbEdgesNbHullVerticesNbPolygons;

		s_scratch->convexScaleRot = rigidShape.scale.rotation;
		s_scratch->convexScale = rigidShape.scale.scale;
		s_scratch->contactDist = contactDist;
	}

	__syncwarp();

	const float4 p = cloth.mPosition_InvMass[vertexIdx];
	const PxVec3 clothP(p.x, p.y, p.z);

	const PxTransform& transform = rigidTransformCache.transform;
	const PxVec3 shapeSpaceP = transform.transformInv(clothP);

	//contact is in the shape space of the polygon
	PxVec3 shapeSpaceClosestP;
	PxVec3 shapeSpaceNormal;
	PxReal pen;
	closestVertToConvex(s_scratch, shapeSpaceP, shapeSpaceClosestP, shapeSpaceNormal, pen, transform);

	//tranform closestP to world space
	const PxVec3 worldP = transform.transform(shapeSpaceClosestP);

	//compute normal
	const PxVec3 worldNormal = (transform.rotate(shapeSpaceNormal)).getNormalized();

	if (threadIdx.x == 0 && (pen < contactDist))
	{
		PxU64 pairInd0 = rigidId.getInd();
		PxU32 pairInd1 = PxEncodeClothIndex(clothId, vertexIdx);

		float4 normalPen = make_float4(worldNormal.x, worldNormal.y, worldNormal.z, pen);
		const PxReal rest = restDistances[cmIdx];
		float4 contact = make_float4(worldP.x, worldP.y, worldP.z, rest);

		{			
			PxU32 ind = atomicAdd(writer.totalContactCount, 1);
			writer.writeRigidVsDeformableContact(ind, contact, normalPen, make_float4(0.f, 0.f, 0.f, 1.f /*1 in w indicates this is a vertex, not a triangle*/),
				pairInd0, pairInd1, rigidShape.materialIndex, rigidId);
		}
	}
}


extern "C" __global__
void cloth_convexVertexContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgFEMCloth* PX_RESTRICT femClothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,

	PxgRigidFilterPair* clothAttachmentPairs,
	const PxU32 nbClothAttachmentPairs,

	const PxU32 stackSizeBytes,
	PxU32* PX_RESTRICT stackSizeNeededOnDevice,	

	PxgFEMContactWriter writer
)
{
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;

	__shared__ char sSharedMem[sizeof(ConvexScratch) * warpsPerBlock];
	ConvexScratch* sharedMem = reinterpret_cast<ConvexScratch*>(sSharedMem);
	ConvexScratch& s_WarpSharedMemory = sharedMem[threadIdx.y];

	PxU32 globalWarpIdx = warpIndex + blockIdx.x * blockDim.y;

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);
	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);

	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalWarpIdx; i < numPairs; i += gridDim.x * blockDim.y)
	{
		const uint4 curPair = pairs[i];
		clothVertexConvexCollision(
			curPair,
			cmInputs,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			femClothes,
			shapeToRigidRemapTable,
			&s_WarpSharedMemory,
			clothAttachmentPairs,
			nbClothAttachmentPairs,
			writer
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}


#define SUPPORT_SUBDIVISION 0

extern "C" __global__
__launch_bounds__(1024, 1)
void cloth_sdfMeshContactGenLaunch(
	const PxU32									numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistances,
	const PxReal* PX_RESTRICT					restDistances,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	
	PxgRigidFilterPair*							filterPairs,
	const PxU32									nbFilterPairs,

	PxgFEMContactWriter							writer)
{
	__shared__ __align__(16) char sMesh0[sizeof(PxgTriangleMesh)];
	PxgTriangleMesh& triangleMesh = reinterpret_cast<PxgTriangleMesh&>(*sMesh0);

	__shared__ __align__(16) char sSdfTexture[sizeof(SparseSDFTexture)];
	SparseSDFTexture& sdfTexture = reinterpret_cast<SparseSDFTexture&>(*sSdfTexture);

	__shared__ uint2 triangleIndicesS[1024];

#if SUPPORT_SUBDIVISION
	//If a triangle is very big compared to the SDF object that collides against it, then schedule a second collision pass where the triangle gets subdivided
	__shared__ uint2 trianglesToSubdivideS[256]; //This buffer can only hold up to one quarter of the elements as triangleIndicesS because each triangle can produce up to 4 sub-triangles during on-the-fly refinement
	__shared__ PxU32 numTrianglesToSubdivideS;

	if (threadIdx.x == 0)
		numTrianglesToSubdivideS = 0;
#endif


	const PxU32 pairIdx = blockIdx.x;

	if (pairIdx >= numWorkItems)
		return;

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, pairIdx);

	PxgShape shape0;
	PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);

	PxgShape shape1;
	PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);

	assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

	PxgShape* trimeshShape = NULL;
	PxgShape* clothShape = NULL;
	PxU32 trimeshCacheRef;
	PxU32 clothCacheRef;

	if (shape0.particleOrSoftbodyId != 0xffffffff)
	{
		clothShape = &shape0;
		trimeshShape = &shape1;

		clothCacheRef = npWorkItem.transformCacheRef0;
		trimeshCacheRef = npWorkItem.transformCacheRef1;
	}
	else
	{
		clothShape = &shape1;
		trimeshShape = &shape0;

		clothCacheRef = npWorkItem.transformCacheRef1;
		trimeshCacheRef = npWorkItem.transformCacheRef0;
	}


	PxsCachedTransform trimeshTransformCache;
	PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);

	PxsCachedTransform clothTransformCache;
	PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);


	if (threadIdx.x < 32)
	{
		readTriangleMesh(*trimeshShape, triangleMesh);
	}
	__syncthreads();

	if (threadIdx.x == 0)
		sdfTexture.initialize(triangleMesh); //The texture is stored in shared memory - only one threads needs to initialize it

	__syncthreads();


	const PxReal contactDistance = contactDistances[npWorkItem.transformCacheRef0] + contactDistances[npWorkItem.transformCacheRef1];
	const PxReal rest = restDistances[pairIdx];


	PxReal cullScale = contactDistance;
	cullScale /= PxMin(trimeshShape->scale.scale.x, PxMin(trimeshShape->scale.scale.y, trimeshShape->scale.scale.z));

	
	const PxU32 clothId = clothShape->particleOrSoftbodyId;
	const PxgFEMCloth* cloth = &clothes[clothShape->particleOrSoftbodyId];


	const PxTransform& trimeshToWorld = trimeshTransformCache.transform;	
	const PxTransform clothToTrimeshTransform = trimeshToWorld.transformInv(clothTransformCache.transform);

	PxMeshScale unitScale = PxMeshScale();	
	
	//printBounds(sdfTexture, trimeshToWorld, trimeshShape->scale);

	uint2* trianglesToSubdivide;
#if SUPPORT_SUBDIVISION	
	PxU32 maxRefinementLevel = 2; //0 to deactivate subdivision
	PxReal refinementRatioSquared = 512 * 512;	
	trianglesToSubdivide = trianglesToSubdivideS;
#else
	trianglesToSubdivide = NULL;
#endif	

	PxU32 numTrianglesToSubdividePerThread = 0;

	const PxU32 nbClothTriangles = cloth->mNbTriangles;
	for (PxU32 i = 0; i < nbClothTriangles || numTrianglesToSubdividePerThread > 0;)
	{
		PxU32 nbFoundTriangles = findInterestingTrianglesA<32, 1024>(nbClothTriangles, cloth->mTriangleVertexIndices, cloth->mPosition_InvMass, unitScale, trimeshShape->scale,
			cullScale, sdfTexture, clothToTrimeshTransform, i, triangleIndicesS, numTrianglesToSubdividePerThread, trianglesToSubdivide);
		numTrianglesToSubdividePerThread = 0;
		

		bool candidateContact = false;
		PxVec3 pos;
		PxVec3 normal;
		PxReal sep;
		float4 tBarycentric;

		PxU32 ind = threadIdx.x < nbFoundTriangles ? triangleIndicesS[threadIdx.x].x : nbClothTriangles;
		PxU32 subInd = threadIdx.x < nbFoundTriangles ? triangleIndicesS[threadIdx.x].y : 0;

#if SUPPORT_SUBDIVISION	
		bool needsRefinement = false;
#endif

		PxNodeIndex rigidId;

		if (ind < nbClothTriangles)
		{
			rigidId = shapeToRigidRemapTable[trimeshCacheRef];
			
			const uint4 triangle = cloth->mTriangleVertexIndices[ind];
			const PxU32 clothMask0 = PxEncodeClothIndex(clothId, triangle.x);
			const PxU32 clothMask1 = PxEncodeClothIndex(clothId, triangle.y);
			const PxU32 clothMask2 = PxEncodeClothIndex(clothId, triangle.z);

			//If we find all verts in the filter pair list, we don't need to generate contacts between the rigid shape and the triangle
			bool skipTriangle =	find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask0) &&
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask1) &&
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask2);

			if (!skipTriangle)
			{
				//printTriangle(cloth->mPosition_InvMass[tri.x], cloth->mPosition_InvMass[tri.y], cloth->mPosition_InvMass[tri.z]);

				PxVec3 v0, v1, v2;
				getTriangleVertices(triangle, cloth->mPosition_InvMass, unitScale, clothToTrimeshTransform, subInd, v0, v1, v2);
				v0 = shape2Vertex(v0, trimeshShape->scale.scale, trimeshShape->scale.rotation);
				v1 = shape2Vertex(v1, trimeshShape->scale.scale, trimeshShape->scale.rotation);
				v2 = shape2Vertex(v2, trimeshShape->scale.scale, trimeshShape->scale.rotation);

				PxVec3 dir;
				sep = doTriangleSDFCollision(sdfTexture, v0, v1, v2, pos, dir, cullScale);


#if SUPPORT_SUBDIVISION	
				//Check if the triangle needs subdivision
				PxReal triRadius2 = triangleRadiusSquared(v0, v1, v2);
				PxReal sdfRadius2 = sdfRadiusSquared(sdfTexture);

				if (subInd > 0)
				{
					getTriangleVertices(cloth->mTriangleVertexIndices, cloth->mPosition_InvMass, unitScale, clothToTrimeshTransform, ind, 0, v0, v1, v2);
					v0 = shape2Vertex(v0, trimeshShape->scale.scale, trimeshShape->scale.rotation);
					v1 = shape2Vertex(v1, trimeshShape->scale.scale, trimeshShape->scale.rotation);
					v2 = shape2Vertex(v2, trimeshShape->scale.scale, trimeshShape->scale.rotation);
				}
#endif

				computeBarycentric(v0, v1, v2, pos, tBarycentric);

				pos = vertex2Shape(pos, trimeshShape->scale.scale, trimeshShape->scale.rotation);

				if (sep < cullScale)
				{
					//dir.normalize();

					dir = vertex2ShapeNormalVector(dir, trimeshShape->scale.scale, trimeshShape->scale.rotation);

					PxReal m = dir.magnitudeSquared();
					if (m > 0.0f)
					{
						m = 1.0f / PxSqrt(m);
						sep = sep * m;
						dir = dir * m;
					}

					candidateContact = sep < contactDistance;

					if (candidateContact)
					{
						sep = sep;
						normal = dir;

#if SUPPORT_SUBDIVISION	
						if (sep < 0.5f * sep)
							needsRefinement = triRadius2 * refinementRatioSquared > sdfRadius2;
#endif
					}
				}

				/*if (tBarycentric.x > 1.0f - 1e3f ||
					tBarycentric.y > 1.0f - 1e3f ||
					tBarycentric.z > 1.0f - 1e3f)
					candidateContact = false;*/
			}
		}

#if SUPPORT_SUBDIVISION
		//needsRefinement = false;
		PxU32 newBufferCount = addToRefinementBuffer<32, 256>(needsRefinement, ind, subInd, 0, trianglesToSubdivideS, maxRefinementLevel);
		if (threadIdx.x == 0)
			numTrianglesToSubdivideS = newBufferCount;

		__syncthreads();

		numTrianglesToSubdividePerThread = numTrianglesToSubdivideS; 
#endif


		PxU32 contactIndex = globalScanExclusive<32>(candidateContact, writer.totalContactCount);
		
		if (candidateContact)
		{
			normal = trimeshToWorld.rotate(normal);
			pos = trimeshToWorld.transform(pos);			

			PxU64 pairInd0 = rigidId.getInd();
			PxU32 pairInd1 = PxEncodeClothIndex(clothId, ind);

			float4 normalPen = make_float4(normal.x, normal.y, normal.z, sep);
			float4 contact = make_float4(pos.x, pos.y, pos.z, rest);

			//writer.writeContact32(contactIndex, contact, normalPen, tBarycentric, pairInd0, pairInd1, PxU32(rigidId.getInd()));
			writer.writeRigidVsDeformableContact(contactIndex, contact, normalPen, tBarycentric, pairInd0, pairInd1, trimeshShape->materialIndex, rigidId);

			/*uint4 tri = cloth->mTriangleVertexIndices[ind];

			pos = tBarycentric.x*PxLoad3(cloth->mPosition_InvMass[tri.x]) + tBarycentric.y*PxLoad3(cloth->mPosition_InvMass[tri.y]) + tBarycentric.z*PxLoad3(cloth->mPosition_InvMass[tri.z]);
			PxVec3 end = pos + PxAbs(sep) * normal;

			if (sep < rest) 
				printLineWithStartPointGreen(pos, end);
			else
				printLineWithStartPointRed(pos, end);
			printTriangle(cloth->mPosition_InvMass[tri.x], cloth->mPosition_InvMass[tri.y], cloth->mPosition_InvMass[tri.z]);	*/			
		}
	}

	//finishDebugPrint();
}


extern "C" __global__
void cloth_heightfieldVertexContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxgFEMCloth* PX_RESTRICT clothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,

	PxgFEMContactWriter writer,

	const PxU32				stackSizeBytes,
	PxU32* PX_RESTRICT		stackSizeNeededOnDevice
)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const uint4* PX_RESTRICT pairs = reinterpret_cast<uint4*>(stackPtr);
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	//const PxU32 numPairs = *midphasePairsNum;

	//const Pxu32 requiredNumPairs = (numPairs + 31) / 32;
	const PxU32 paddedNumPairs = (numPairs + 31) &(~31);

	//each warp do collision detection between a tetrahedron and a primitive
	for (PxU32 i = globalThreadIdx; i < paddedNumPairs; i += gridDim.x * blockDim.x)
	{
		bool hasContacts = false;
		PxU64 pairInd0;
		PxU32 pairInd1;
		float4 p1_;
		PxVec3 n;
		PxReal pen;
		float4 contact;

		PxU32 rigidMaterialIndex = 0xFFFFFFFF;
		if (i < numPairs)
		{
			const uint4 curPair = pairs[i];

			const PxU32 cmIdx = curPair.x;
			const PxU32 clothVertIndex = curPair.y;
			const PxU32 hfTriangleIdx = curPair.z;

			/*if(clothVertIndex == 1)
				printf("idx %i cmIdx %i clothVertIndex %i triangleIdx %i\n", threadIdx.x, cmIdx, clothVertIndex, meshTriangleIdx);
*/
			const PxgContactManagerInput npWorkItem = cmInputs[cmIdx];

			PxgShape shape0 = gpuShapes[npWorkItem.shapeRef0];
			PxgShape shape1 = gpuShapes[npWorkItem.shapeRef1];

			assert(shape0.type = PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eHEIGHTFIELD);

			/*	PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0.type);
				PxGeometryType::Enum type1 = PxGeometryType::Enum(shape1.type);*/

			PxgShape* heightfieldShape = NULL;
			PxgShape* clothShape = NULL;

			PxU32 heightfieldTransformCacheRef;
			PxU32 clothTransformCacheRef;

			if (shape0.particleOrSoftbodyId != 0xffffffff)
			{
				assert(shape1.particleOrSoftbodyId == 0xffffffff);
				clothShape = &shape0;
				heightfieldShape = &shape1;

				clothTransformCacheRef = npWorkItem.transformCacheRef0;
				heightfieldTransformCacheRef = npWorkItem.transformCacheRef1;
			}
			else
			{
				assert(shape0.particleOrSoftbodyId == 0xffffffff);
				clothShape = &shape1;
				heightfieldShape = &shape0;

				clothTransformCacheRef = npWorkItem.transformCacheRef1;
				heightfieldTransformCacheRef = npWorkItem.transformCacheRef0;
			}
			rigidMaterialIndex = heightfieldShape->materialIndex;

			const PxNodeIndex rigidId = shapeToRigidRemapTable[heightfieldTransformCacheRef];

			PxsCachedTransform heightfieldTransformCache = transformCache[heightfieldTransformCacheRef];
			PxsCachedTransform clothTransformCache = transformCache[clothTransformCacheRef];

			const PxTransform& hfTranform = heightfieldTransformCache.transform;
			
			PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightfieldShape->hullOrMeshPtr);
			const PxU32 nbRows = heightfieldData[0];
			const PxU32 nbCols = heightfieldData[1];
			PxHeightFieldSample* samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);

			PxVec3 a, b, c;
			//vert is in local space of the height field
			getTriangle(a, b, c, NULL, hfTriangleIdx, heightfieldShape->scale, nbRows, nbCols, samples);

			const PxU32 clothId = clothShape->particleOrSoftbodyId;
			PxgFEMCloth* cloth = &clothes[clothShape->particleOrSoftbodyId];

			//world space
			//const float4 p_ = cloth->mPosition_InvMass[clothVertIndex];
			const float4 p0_ = cloth->mPosition_InvMass[clothVertIndex]; //points before integration
			p1_ = cloth->mPosition_InvMass[clothVertIndex];

			const PxVec3 p0(p0_.x, p0_.y, p0_.z);
			const PxVec3 p1(p1_.x, p1_.y, p1_.z);

			const PxTransform clothToHfTransform(hfTranform.transformInv(clothTransformCache.transform));

			const PxVec3 localP0 = clothToHfTransform.transform(p0);// shape2Vertex(meshToWorld.transformInv(p0), trimeshScale.scale, trimeshScale.rotation);
			//const PxVec3 localP1 = shape2Vertex(meshToWorld.transformInv(p1), trimeshScale.scale, trimeshScale.rotation);

			const PxVec3 ab = b - a;
			const PxVec3 ac = c - a;

			const PxVec3 normal = ab.cross(ac);

			//p is outside
			PxVec3 localClosestPt = Gu::closestPtPointTriangle2(localP0, a, b, c, ab, ac);


			//tranform lcoalClosestPt to world space
			PxVec3 closestPt = hfTranform.transform(localClosestPt);////meshToWorld.transform(vertex2Shape(localClosestPt, trimeshScale.scale, trimeshScale.rotation));

			const PxVec3 v = p0 - closestPt;
			const PxReal sqDist = v.dot(v);

			pen = PxSqrt(sqDist);
			n = v / pen;

			const PxVec3 ap1 = p1 - closestPt;
			pen = ap1.dot(n);


			const PxReal cDistance = contactDistance[heightfieldTransformCacheRef] + contactDistance[clothTransformCacheRef];
			//const PxReal sqContactDistance = cDistance * cDistance;

			/*if (clothVertIndex == 1)
				printf("idx %i cmIdx %i clothVertIndex %i triangleIdx %i\n", threadIdx.x, cmIdx, clothVertIndex, meshTriangleIdx);*/

			if ((pen <= cDistance) && normal.dot(n) > 0.f)
			{
				pairInd0 = rigidId.getInd();
				pairInd1 = PxEncodeClothIndex(clothId, clothVertIndex);

				//hasContacts = true;
				contact = make_float4(closestPt.x, closestPt.y, closestPt.z, restDistance[cmIdx]);
			}
		}

		PxgOutputIndex indexer(__ballot_sync(FULL_MASK, hasContacts), writer.totalContactCount);
		if (hasContacts)
		{
			const PxU32 index = indexer.getIndex();
			writer.writeRigidVsDeformableContact32(index, contact, make_float4(n.x, n.y, n.z, pen), make_float4(0.f, 0.f, 0.f, 1.f /*vertex contact*/), pairInd0, pairInd1, rigidMaterialIndex, PxU32(pairInd0));
		}
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
	
}



__device__ static inline void clothParticleCollision(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxgFEMCloth* PX_RESTRICT clothes,

	PxgFEMContactWriter& writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 particleIndex = curPair.y;
	const PxU32 triangleIdx = curPair.z;
	
	//printf("idx %i cmIdx %i particleIndex %i triangleIdx %i\n", threadIdx.x, cmIdx, particleIndex, triangleIdx);

	PxgShape particleShape, clothShape;
	PxU32 particleCacheRef, clothCacheRef;
	LoadShapePair<PxGeometryType::ePARTICLESYSTEM, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		particleShape, particleCacheRef, clothShape, clothCacheRef);

	const PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[clothCacheRef];
	const PxReal restDistance = restDistances[cmIdx];

	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	const PxgFEMCloth& cloth = clothes[clothId];

	const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;
	const PxgParticleSystem& particleSystem = particleSystems[particleSystemId];


	const float4* sortedPose = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);
	const float4* meshVerts = cloth.mPosition_InvMass;

	const uint4 triVertIdx = cloth.mTriangleVertexIndices[triangleIdx];

	const float4 a_ = meshVerts[triVertIdx.x];
	const float4 b_ = meshVerts[triVertIdx.y];
	const float4 c_ = meshVerts[triVertIdx.z];

	const PxVec3 a(a_.x, a_.y, a_.z);
	const PxVec3 b(b_.x, b_.y, b_.z);
	const PxVec3 c(c_.x, c_.y, c_.z);

	const float4 p_ = sortedPose[particleIndex];

	const PxVec3 p(p_.x, p_.y, p_.z);

	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	
	//p is outside
	PxVec3 closestPt = Gu::closestPtPointTriangle2(p, a, b, c, ab, ac);
	const PxVec3 v = p - closestPt;
	const PxReal sqDist = v.dot(v);

	const PxReal sqContactDistance = cDistance * cDistance;

	////if (threadIdx.x == 0)
	//{
	//	/*printf("a(%f, %f, %f)\n", a.x, a.y, a.z);
	//	printf("b(%f, %f, %f)\n", b.x, b.y, b.z);
	//	printf("c(%f, %f, %f)\n", c.x, c.y, c.z);
	//	printf("d(%f, %f, %f)\n", d.x, d.y, d.z);
	//	printf("p(%f, %f, %f)\n", p.x, p.y, p.z);
	//	printf("cloestPt(%f, %f, %f)\n", cloestPt.x, cloestPt.y, cloestPt.z);*/
	//	printf("idx %i sqDist %f sqContactDistance %f particleRestOffset %f\n", threadIdx.x, sqDist, sqContactDistance, particleRestOffset);
	//}


	if (sqDist <= sqContactDistance)
	{

		const PxReal m = PxSqrt(sqDist);
		const PxVec3 n = v / m;
		const PxReal pen = m - restDistance;
		const PxVec3 contact = p - n * restDistance;

		float4 tBarycentric;

		bool intersect = computeBarycentric(a, b, c, closestPt, tBarycentric);

		assert(intersect);


		/*{
			printf("Intersect idx %i contact(%f, %f, %f) closestPt(%f, %f, %f) pen %f\n", threadIdx.x, contact.x, contact.y, contact.z,
				closestPt.x, closestPt.y, closestPt.z, pen);
		}*/

		int32_t index = atomicAdd(writer.totalContactCount, 1);

		//printf("idx %i index %i\n", threadIdx.x, index);

		PxU64 pairInd0 = PxEncodeParticleIndex(particleSystemId, particleIndex);
		PxU32 pairInd1 = PxEncodeClothIndex(clothId, triangleIdx);

		writer.writeContact(index, make_float4(contact.x, contact.y, contact.z, 0.f), make_float4(-n.x, -n.y, -n.z, pen), tBarycentric,
			pairInd0, pairInd1, pairInd0);
	}	
}

extern "C" __global__
void cloth_psContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxgFEMCloth* PX_RESTRICT clothes,

	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,
	PxU32* PX_RESTRICT stackSizeNeededOnDevice,											//output

	PxgFEMContactWriter writer
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

		clothParticleCollision(tolerenceLength, cmInputs,
			curPair, transformCache, contactDistance, restDistances, gpuShapes,
			particleSystems, clothes, writer);
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}


extern "C" __global__
void cloth_meshVertexContactGenLaunch(
	const PxReal									tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT		cmInputs,
	const PxsCachedTransform* PX_RESTRICT			transformCache,
	const PxReal* PX_RESTRICT						contactDistance,
	const PxReal* PX_RESTRICT						restDistance,
	const PxgShape* PX_RESTRICT						gpuShapes,
	PxgFEMCloth* PX_RESTRICT						clothes,
	const PxNodeIndex* PX_RESTRICT					shapeToRigidRemapTable,

	const PxU32										stackSizeBytes,
	PxU8* PX_RESTRICT								stackPtr,
	PxU32* PX_RESTRICT								midphasePairsNum,
	PxU32* PX_RESTRICT								stackSizeNeededOnDevice,									//output

	PxgFEMContactWriter						writer
)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const uint4* PX_RESTRICT pairs = reinterpret_cast<uint4*>(stackPtr);
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);
	const PxU32 paddedNumPairs = (numPairs + 31) &(~31);

	//each warp do collision detection between a triangle and a primitive
	for (PxU32 i = globalThreadIdx; i < paddedNumPairs; i += gridDim.x * blockDim.x)
	{
		bool hasContacts = false;
		PxU64 pairInd0;
		PxU32 pairInd1;
		float4 p1_;
		PxVec3 n;
		PxReal pen;
		float4 contact;

		PxU32 rigidMaterialIndex = 0xFFFFFFFF;
		if (i < numPairs)
		{
			const uint4 curPair = pairs[i];

			const PxU32 cmIdx = curPair.x;
			const PxU32 clothVertIndex = curPair.y;
			const PxU32 meshTriangleIdx = curPair.z;

			const PxgContactManagerInput npWorkItem = cmInputs[cmIdx];

			PxgShape shape0 = gpuShapes[npWorkItem.shapeRef0];
			PxgShape shape1 = gpuShapes[npWorkItem.shapeRef1];

			PxgShape* trimeshShape = NULL;
			PxgShape* clothShape = NULL;

			PxU32 trimeshCacheRef;
			PxU32 clothCacheRef;

			if (shape0.particleOrSoftbodyId != 0xffffffff)
			{
				assert(shape1.particleOrSoftbodyId == 0xffffffff);
				clothShape = &shape0;
				trimeshShape = &shape1;

				clothCacheRef = npWorkItem.transformCacheRef0;
				trimeshCacheRef = npWorkItem.transformCacheRef1;
			}
			else
			{
				assert(shape0.particleOrSoftbodyId == 0xffffffff);
				clothShape = &shape1;
				trimeshShape = &shape0;

				clothCacheRef = npWorkItem.transformCacheRef1;
				trimeshCacheRef = npWorkItem.transformCacheRef0;
			}
			rigidMaterialIndex = trimeshShape->materialIndex;

			const PxNodeIndex rigidId = shapeToRigidRemapTable[trimeshCacheRef];

			PxsCachedTransform trimeshTransformCache = transformCache[trimeshCacheRef];
			PxsCachedTransform clothTransformCache = transformCache[clothCacheRef];

			const PxTransform& trimeshToWorld = trimeshTransformCache.transform;
			const PxMeshScale& trimeshScale = trimeshShape->scale;

			PxU8 * trimeshGeomPtr = reinterpret_cast<PxU8*>(trimeshShape->hullOrMeshPtr);

			const uint4 nbVerts_nbTri_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
			trimeshGeomPtr += sizeof(uint4) + sizeof(const Gu::BV32DataPacked)* nbVerts_nbTri_maxDepth_nbBv32TreeNodes.w;

			const float4* trimeshVerts = reinterpret_cast<const float4 *>(trimeshGeomPtr);

			trimeshGeomPtr += sizeof(float4) * nbVerts_nbTri_maxDepth_nbBv32TreeNodes.x;

			const uint4* trimeshVertIndices = reinterpret_cast<const uint4 *>(trimeshGeomPtr);

			const uint4 triVertInd = trimeshVertIndices[meshTriangleIdx];

			//this is in vertex space
			const PxVec3 a = PxLoad3(trimeshVerts[triVertInd.x]);
			const PxVec3 b = PxLoad3(trimeshVerts[triVertInd.y]);
			const PxVec3 c = PxLoad3(trimeshVerts[triVertInd.z]);

			const PxU32 clothId = clothShape->particleOrSoftbodyId;
			PxgFEMCloth* cloth = &clothes[clothShape->particleOrSoftbodyId];

			//world space
			const float4 p0_ = cloth->mPosition_InvMass[clothVertIndex]; //points before integration
			p1_ = cloth->mPosition_InvMass[clothVertIndex];

			const PxVec3 p0(p0_.x, p0_.y, p0_.z);
			const PxVec3 p1(p1_.x, p1_.y, p1_.z);

			const PxVec3 localP0 = shape2Vertex(trimeshToWorld.transformInv(p0), trimeshScale.scale, trimeshScale.rotation);

			const PxVec3 ab = b - a;
			const PxVec3 ac = c - a;

			const PxVec3 normal = ab.cross(ac);

			//p is outside
			PxVec3 localClosestPt = Gu::closestPtPointTriangle2(localP0, a, b, c, ab, ac);


			//tranform lcoalClosestPt to world space
			PxVec3 closestPt = trimeshToWorld.transform(vertex2Shape(localClosestPt, trimeshScale.scale, trimeshScale.rotation));

			const PxVec3 v = p0 - closestPt;
			const PxReal sqDist = v.dot(v);

			pen = PxSqrt(sqDist);
			n = v / pen;

			const PxVec3 ap1 = p1 - closestPt;
			pen = ap1.dot(n);


			const PxReal cDistance = contactDistance[trimeshCacheRef] + contactDistance[clothCacheRef];

			if ((pen <= cDistance) && normal.dot(n) > 0.f)
			{
				pairInd0 = rigidId.getInd();
				pairInd1 = PxEncodeClothIndex(clothId, clothVertIndex);

				contact = make_float4(closestPt.x, closestPt.y, closestPt.z, restDistance[cmIdx]);
				//hasContacts = true;
			}
		}

		PxgOutputIndex indexer(__ballot_sync(FULL_MASK, hasContacts), writer.totalContactCount);
		if (hasContacts)
		{
			const PxU32 index = indexer.getIndex();

			writer.writeRigidVsDeformableContact32(index, contact, make_float4(n.x, n.y, n.z, pen), make_float4(0.f, 0.f, 0.f, 1.f /*vertex contact*/),
				pairInd0, pairInd1, rigidMaterialIndex, PxU32(pairInd0));
		}
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}

extern "C" __global__
__launch_bounds__(WARP_SIZE*16, 1)
void cloth_boxVertexContactGenLaunch(
	PxU32 numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	const PxNodeIndex* PX_RESTRICT	shapeToRigidRemapTable,

	PxgRigidFilterPair*				filterPairs,
	const PxU32						nbFilterPairs,

	PxgFEMContactWriter		writer
)
{
	//one block deal with one test
	unsigned int cmIdx = blockIdx.x;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	PxU32 rigidMaterialIndex = 0xFFFFFFFF;
	if (cmIdx < numWorkItems)
	{
		PxgShape clothShape, rigidShape;
		PxU32 clothCacheRef, rigidCacheRef;
		LoadShapePair<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape, clothCacheRef, rigidShape, rigidCacheRef);

		const PxVec3 boxHalfExtents = rigidShape.scale.scale;

		const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];

		const PxU32 femClothId = clothShape.particleOrSoftbodyId;

		const PxgFEMCloth& femCloth = femClothes[femClothId];

		const float4* verts = femCloth.mPosition_InvMass;

		const PxU32 numVerts = femCloth.mNbVerts;

		PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		rigidMaterialIndex = rigidShape.materialIndex;

		float4 contact;
		float4 normalPen;
		PxU64 pairInd0;
		PxU32 pairInd1;

		//const PxU32 numRequiredWarps = (numVerts + WARP_SIZE - 1) / WARP_SIZE;
		//const PxU32 numRequiredThreads = numRequiredWarps * WARP_SIZE;

		const PxU32 numRequiredThreads = (numVerts + 31) & (~31);

		const PxU32 threadIndexInBlock = threadIdx.x + threadIdx.y * WARP_SIZE;

		const PxU32 threadIndexInWarp = threadIdx.x;

#if 0
		if (threadIndexInWarp == 0)
			printf("box(%f, %f, %f)\n", boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z);
#endif

		const PxReal sphereRadius = 0.f;

#if 0
		if (threadIndexInBlock == 0)
			printf("numRequiredWarps %i numRequiredThreads %i numVerts %i totalThreadInABlock %i\n", numRequiredWarps, numRequiredThreads, numVerts, blockDim.x * blockDim.y);
#endif

		//each thread do collision detection between a triangle and sphere/plane
		for (PxU32 i = threadIndexInBlock; i < numRequiredThreads; i += blockDim.x * blockDim.y)
		{
			//numContacts will be either 0 or 1
			PxU32 numContacts = 0;

			if (i < numVerts)
			{
				const PxU32 clothMask = PxEncodeClothIndex(femClothId, i);

				//If we find the vert index in the filter pairs, we don't need to generate contacts between the rigid shape and the vertex
				bool skipVertex = find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask);

				if (!skipVertex)
				{
					const float4 p = verts[i];
					const PxVec3 clothP(p.x, p.y, p.z);

					assert(PxGeometryType::Enum(rigidShape.type) == PxGeometryType::eBOX);

					//const PxsCachedTransform& transfCache1 = transformCache[transformCacheRef1];

					numContacts = spherebox(clothP, transformCache[rigidCacheRef].transform, sphereRadius,
						boxHalfExtents, contactDist, normalPen);

					if (numContacts)
					{
						pairInd0 = rigidId.getInd();
						pairInd1 = PxEncodeClothIndex(femClothId, i); //vert id

						const PxVec3 rigidP = clothP - normalPen.w * PxVec3(normalPen.x, normalPen.y, normalPen.z);
						contact = make_float4(rigidP.x, rigidP.y, rigidP.z, restDistance[cmIdx]);

						assert(numContacts == 1);
					}
				}
			}

			const PxU32 mask = __ballot_sync(FULL_MASK, numContacts);

			PxU32 totalCount = __popc(mask);

			PxU32 threadOffset = warpScanExclusive(mask, threadIndexInWarp);

			PxU32 startIndex = 0xFFffFFff;
			if (threadIndexInWarp == 0 && totalCount > 0)
			{
				startIndex = atomicAdd(writer.totalContactCount, totalCount);
			}

			startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

#if 0
			if (startIndex == 0)
				printf("%i %i globalWarpIndex %i numContacts %i totalCount %i startIndex %i inclusiveSum %i \n", threadIndexInWarp, i, globalWarpIndex, numContacts, totalCount, startIndex, inclusiveSum);
#endif
			//first 8 bits for cloth, second 24 bits for triangle index

			if (numContacts > 0)
			{
				const PxU32 index = startIndex + threadOffset;
				writer.writeRigidVsDeformableContact(index, contact, normalPen, make_float4(0.f, 0.f, 0.f, 1.f), pairInd0, pairInd1, rigidMaterialIndex, rigidId);
			}
		}
	}
}


__device__ static inline void clothBoxTriangleCollision(
	const PxU32 globalWarpIndex,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,

	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,
	ConvexTriangleScratch*						s_scratch,
	PxgRigidFilterPair*							filterPairs,
	const PxU32									nbFilterPairs,

	PxgFEMContactWriter&					writer
)
{
	const PxU32 cmIdx = curPair.x;
	const PxU32 triangleIdx = curPair.y;

	PxgShape clothShape, rigidShape;
	PxU32 clothCacheRef, rigidCacheRef;
	LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		clothShape, clothCacheRef, rigidShape, rigidCacheRef);

	PxsCachedTransform clothTransformCache;
	PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);

	PxsCachedTransform rigidTransformCache;
	PxsCachedTransform_ReadWarp(rigidTransformCache, transformCache + rigidCacheRef);

	const PxNodeIndex rigidId = shapeToRigidRemapTable[rigidCacheRef];
	const PxU32 clothId = clothShape.particleOrSoftbodyId;
	
	const PxgFEMCloth &cloth = femClothes[clothId];

	uint4* vertIndices = cloth.mTriangleVertexIndices;

	const uint4 triIdx = vertIndices[triangleIdx];

	const PxU32 clothMask0 = PxEncodeClothIndex(clothId, triIdx.x);
	const PxU32 clothMask1 = PxEncodeClothIndex(clothId, triIdx.y);
	const PxU32 clothMask2 = PxEncodeClothIndex(clothId, triIdx.z);

	//If we find all verts in the filter pair list, we don't need to generate contacts between the rigid shape and the triangle
	bool skipTriangle =	find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask0) &&
						find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask1) &&
						find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask2);

	if (skipTriangle)
		return;

	if (threadIdx.x == 0)
	{
		PxTransform clothToConvexTransform(rigidTransformCache.transform.transformInv(clothTransformCache.transform));
		s_scratch->trimeshToConvexTransform = clothToConvexTransform;
		s_scratch->convexTransform = rigidTransformCache.transform;
	}

	__syncwarp();

	if (threadIdx.x == 0)
	{
		// Shapes
		s_scratch->contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		s_scratch->restDist = restDistance[cmIdx];
		s_scratch->convexScaleRot = rigidShape.scale.rotation;
		//s_scratch->trimeshScaleRot = triangleShape.scale.rotation;

		s_scratch->convexScale = rigidShape.scale.scale;
		//s_scratch->trimeshScale = triangleShape.scale.scale;

		const PxU8* convexPtrA = reinterpret_cast<const PxU8 *>(rigidShape.hullOrMeshPtr);
		s_scratch->convexPtrA = convexPtrA;

		const float4 hull0_centerOfMass_f4 = *reinterpret_cast<const float4 *>(convexPtrA);
		const PxVec3 hull0_centerOfMass(hull0_centerOfMass_f4.x, hull0_centerOfMass_f4.y, hull0_centerOfMass_f4.z);
		// Transform CoM into shape space
		PxVec3 shapeSpaceCenterOfMass0 = vertex2Shape(hull0_centerOfMass, rigidShape.scale.scale, rigidShape.scale.rotation);

		s_scratch->convexCenterOfMass = shapeSpaceCenterOfMass0;


		convexPtrA += sizeof(float4);

		const uint4 tmp = *((uint4*)convexPtrA);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;
		s_scratch->nbEdgesNbHullVerticesNbPolygons = polyData0_NbEdgesNbHullVerticesNbPolygons;

		convexPtrA += sizeof(uint4);

		const float4 polyData0_Extents = *((float4*)(convexPtrA));
		PxVec3 convex_extents = PxLoad3(polyData0_Extents);

		const float4* trimeshVerts = cloth.mPosition_InvMass;
		//s_scratch->trimeshVerts = trimeshVerts;

		const uint4* triVertInds = cloth.mTriangleVertexIndices;
		//s_scratch->trimeshTriIndices = triVertInds;

		uint4 triIndices = triVertInds[triangleIdx];

		float4 triV0_f4 = trimeshVerts[triIndices.x];
		float4 triV1_f4 = trimeshVerts[triIndices.y];
		float4 triV2_f4 = trimeshVerts[triIndices.z];

		PxVec3 triV0 = PxLoad3(triV0_f4);
		PxVec3 triV1 = PxLoad3(triV1_f4);
		PxVec3 triV2 = PxLoad3(triV2_f4);

		// Triangle scaling

		const PxVec3 v0 = triV0;
		const PxVec3 v1 = triV1;
		const PxVec3 v2 = triV2;
		/*v0 = vertex2Shape(triV0, clothShape.scale.scale, clothShape.scale.rotation);
		v1 = vertex2Shape(triV1, clothShape.scale.scale, clothShape.scale.rotation);
		v2 = vertex2Shape(triV2, clothShape.scale.scale, clothShape.scale.rotation);*/

		PxTransform trimeshToConv = s_scratch->trimeshToConvexTransform;
		PxVec3 triLocVerts0 = trimeshToConv.transform(v0);
		PxVec3 triLocVerts1 = trimeshToConv.transform(v1);
		PxVec3 triLocVerts2 = trimeshToConv.transform(v2);

		s_scratch->triLocVerts[0] = triLocVerts0;
		s_scratch->triLocVerts[1] = triLocVerts1;
		s_scratch->triLocVerts[2] = triLocVerts2;

		PxVec3 triangleLocNormal;
		triangleLocNormal = (triLocVerts1 - triLocVerts0).cross(triLocVerts2 - triLocVerts0).getNormalized();

		s_scratch->triangleLocNormal = triangleLocNormal;

	}

	__syncwarp();

	convexTriangleContactGen(s_scratch, globalWarpIndex, rigidId, clothId, triangleIdx, writer, rigidShape);
}

extern "C" __global__
void cloth_boxTriangleContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,

	PxgRigidFilterPair*				clothAttachmentPairs,
	const PxU32						nbClothAttachmentPairs,
	const PxU32				stackSizeBytes,
	PxU32* PX_RESTRICT		stackSizeNeededOnDevice,

	PxgFEMContactWriter		writer
)
{
	const PxU32 warpIndex = threadIdx.y;

	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;

	__shared__ volatile float sharedMem[warpsPerBlock][32 * 16]; // sizeof(ConvexTriangleScratch) is 2000B, allocate 4096B
	volatile float* s_WarpSharedMemory = sharedMem[threadIdx.y];


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

			/*if (globalWarpIdx == 0 && threadIdx.x  == 0)
			{
				printf("numPairs %i\n", numPairs);
			}
	*/

		const uint4 curPair = pairs[i];

		clothBoxTriangleCollision(
			i,
			tolerenceLength,
			cmInputs,
			curPair,
			transformCache,
			contactDistance,
			restDistance,
			gpuShapes,
			femClothes,
			shapeToRigidRemapTable,
			(ConvexTriangleScratch*)s_WarpSharedMemory,
			clothAttachmentPairs,
			nbClothAttachmentPairs,
			writer			
		);
	}

	if (globalWarpIdx == 0 && threadIdx.x == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}



//Only produces correct results if a and b are located on different sides of the plane. One of them is allowed to be exactly on the plane.
PX_FORCE_INLINE __device__ PxVec3 intersectionPoint(const PxPlane& plane, const PxVec3& a, const PxVec3& b)
{
	float distA = PxAbs(plane.distance(a));
	float distB = PxAbs(plane.distance(b));
	float f = distA / (distA + distB);
	return (1.0f - f) * a + f * b;
}

PX_FORCE_INLINE __device__ void insert(PxVec3* arr, int arrCount, int index, const PxVec3& element)
{
	for (int i = arrCount; i > index; --i)
		arr[i] = arr[i - 1];
	arr[index] = element;
}

//Cuts/Trims a polygon loop in 3d using a plane. Points on the normal direction of the plane are removed.
//Returns the new loopCount. The loop vertices are modified in-place. The array called loop must provide enough space.
PX_FORCE_INLINE __device__ int trim(const PxPlane& plane, PxVec3* loop, int loopCount)
{
	if (loopCount < 3)
		return loopCount;

	PxVec3 intersectionA;
	int changeA = -1;
	PxVec3 intersectionB;
	int changeB = -1;

	bool keep = false;

	bool prevOutside = plane.distance(loop[0]) > 0.0f;
	for (int i = 0; i < loopCount; ++i)
	{
		bool outside = plane.distance(loop[(i + 1) % loopCount]) > 0.0f;
		if (outside != prevOutside)
		{
			PxVec3 intersection = intersectionPoint(plane, loop[i], loop[(i + 1) % loopCount]);
			if (changeA < 0)
			{
				changeA = i;
				keep = !prevOutside;
				intersectionA = intersection;
			}
			else
			{
				changeB = i;
				intersectionB = intersection;
			}
		}
		prevOutside = outside;
	}

	if (changeA >= 0 && changeB >= 0)
	{
		int loopIndexer = -1;
		for (int i = 0; i < loopCount; ++i)
		{
			if (keep)
				loop[++loopIndexer] = loop[i];

			if (i == changeA || i == changeB)
			{
				PxVec3 pt = i == changeA ? intersectionA : intersectionB;
				if (loopIndexer == i && !keep)
				{
					insert(loop, loopCount, ++loopIndexer, pt);
					++loopCount;
					++i;
					++changeB;
				}
				else
					loop[++loopIndexer] = pt;

				keep = !keep;
			}
		}
		loopCount = loopIndexer + 1;
	}
	else if (prevOutside)
		loopCount = 0; //If there was no intersection, all points are on the same side of the plane. If all are outside, remove all

	return loopCount;
}

PX_FORCE_INLINE __device__ int trimPolygonByPlanes(PxVec3* poly, int polyLength, const PxPlane& plane0, const PxPlane& plane1, const PxPlane& plane2)
{
	polyLength = trim(plane0, poly, polyLength);
	polyLength = trim(plane1, poly, polyLength);
	polyLength = trim(plane2, poly, polyLength);
	return polyLength;
}

PX_FORCE_INLINE __device__ PxPlane createEdgePlane(const PxVec3& triNormal, const PxVec3& edgeStart, const PxVec3& edgeEnd)
{
	PxVec3 n = (edgeEnd - edgeStart).cross(triNormal).getNormalized();
	return PxPlane(edgeStart, n);
}

//Requires an array with 6 elements
PX_FORCE_INLINE __device__ int trimTriangleByTriangle(PxVec3* poly, int polyLength, const PxVec3& triA, const PxVec3& triB, const PxVec3& triC)
{
	PxVec3 n = (triB - triA).cross(triC - triA);

	polyLength = trim(createEdgePlane(n, triA, triB), poly, polyLength);
	polyLength = trim(createEdgePlane(n, triB, triC), poly, polyLength);
	polyLength = trim(createEdgePlane(n, triC, triA), poly, polyLength);

	return polyLength;
}

//Requires an array with 8 elements
PX_FORCE_INLINE __device__ int trimTriangleByTriangle(PxVec3* poly, int polyLength, const PxVec3& triA, const PxVec3& triB, const PxVec3& triC,
	float maxDistanceLimitPosNormalDir, float maxDistanceLimitNegNormalDir)
{
	PxVec3 n = (triB - triA).cross(triC - triA);
	n.normalizeSafe();

	polyLength = trim(createEdgePlane(n, triA, triB), poly, polyLength);
	polyLength = trim(createEdgePlane(n, triB, triC), poly, polyLength);
	polyLength = trim(createEdgePlane(n, triC, triA), poly, polyLength);

	polyLength = trim(PxPlane(triA + maxDistanceLimitPosNormalDir * n, n), poly, polyLength);
	polyLength = trim(PxPlane(triA - maxDistanceLimitNegNormalDir * n, -n), poly, polyLength);

	return polyLength;
}

//Keep for later improvements: Place triangle side planes to be the half-angle-planes between the triangle and it's adjacent triangles.
//PX_FORCE_INLINE __device__ int trimTriangleByTriangle(PxVec3* poly, int polyLength, const PxVec3& triA, const PxVec3& triB, const PxVec3& triC,
//	const PxVec3& abNeighbor, const PxVec3& bcNeighbor, const PxVec3& caNeighbor, PxU32 neighborFlags, float maxDistanceLimitPosNormalDir, float maxDistanceLimitNegNormalDir)
//{
//
//}

//Only works together with trimTriangleByTriangle methods. For more general triangle edge planes, point to triangle distances must be used.
PX_FORCE_INLINE __device__ PxVec3 getDeepestPointSimplified(PxVec3* poly, int polyLength, const PxPlane& plane, PxReal tolerance, PxReal& minDist)
{
	minDist = 1000000000;
	PxVec3 deepestPoint;
	int count = 0;
	for (int i = 0; i < polyLength; ++i)
	{
		double d = plane.distance(poly[i]);
		if (d < minDist - tolerance)
		{
			minDist = d;
			deepestPoint = poly[i];
			count = 1;
		}
		else if (d < minDist + tolerance)
		{
			deepestPoint += poly[i];
			count++;
		}
	}
	deepestPoint /= count;
	return deepestPoint;
}



struct PxgFemContact
{
	PxVec3 position;
	PxVec3 normal;
	PxReal separation;
	float4 barycentric;
};

PX_FORCE_INLINE __device__ bool staticTriangleVsClothCollision(const PxVec3& localA0, const PxVec3& localB0, const PxVec3& localC0,
	const PxgFEMCloth* cloth, const uint4& clothVertInd, const PxU32 clothTriangleIdx, const PxTransform& clothTransform, PxReal cDistance, PxgFemContact& contact)
{
	const PxVec3 a1 = PxLoad3(cloth->mPosition_InvMass[clothVertInd.x]);
	const PxVec3 b1 = PxLoad3(cloth->mPosition_InvMass[clothVertInd.y]);
	const PxVec3 c1 = PxLoad3(cloth->mPosition_InvMass[clothVertInd.z]);

	PxVec3 convexPoly[8];
	convexPoly[0] = a1;
	convexPoly[1] = b1;
	convexPoly[2] = c1;

	PxVec3 fixNormal = (localB0 - localA0).cross(localC0 - localA0).getNormalized();
	PxPlane staticTrianglePlane(localA0, fixNormal);

	int numPolyCorners = trimTriangleByTriangle(convexPoly, 3, localA0, localB0, localC0, cDistance, cDistance);

	//Work in the plane of the static mesh triangle
	//Trim the cloth triangle such that it lies inside the mesh triangle (it potentially becomes a convex polygon with up to 6 corners during that process)
	//Find the deepest point on that trimmed polygon
	//The deepest point is on a corner. Or on an edge if the edge is parallel to the mesh's triangle plane. Or all points have the same distance if cloth and static mesh triangle are on parallel planes.

	PxReal sep;
	PxVec3 deepestPoint = getDeepestPointSimplified(convexPoly, numPolyCorners, staticTrianglePlane, 0.01f * cDistance, sep);

	bool hasContact = sep < cDistance;	

	if (hasContact)
	{
		computeBarycentric(a1, b1, c1, deepestPoint, contact.barycentric);
		contact.position = clothTransform.transform(deepestPoint);
		contact.normal = clothTransform.rotate(fixNormal);
		contact.separation = sep;
	}
	return hasContact;
}

extern "C" __global__
void cloth_heightfieldContactGenLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT midphasePairsNum,

	const PxU32 stackSizeBytes,
	PxU32* PX_RESTRICT stackSizeNeededOnDevice,

	PxgFEMContactWriter	writer
)
{
	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);

	PxU32 l = ((numPairs + WARP_SIZE - 1) / WARP_SIZE) * WARP_SIZE;
	PxgFemContact c;

	for (PxU32 i = globalThreadIdx; i < l; i += gridDim.x * blockDim.x)
	{
		bool hasContact = false;
		PxU32 cmIdx;
		PxU32 clothId;
		PxU32 clothTriangleIdx;
		PxU32 ref;

		PxU32 rigidMaterialIndex = 0xFFFFFFFF;
		if (i < numPairs) 
		{
			const uint4 curPair = pairs[i];

			cmIdx = curPair.x;
			clothTriangleIdx = curPair.y; //cloth triangleInd index
			const PxU32 hfTriangleIdx = curPair.z;	//heighfield triangle index


			const PxgContactManagerInput npWorkItem = cmInputs[cmIdx];
			PxgShape clothShape = gpuShapes[npWorkItem.shapeRef0];
			PxgShape heightfieldShape = gpuShapes[npWorkItem.shapeRef1];

			rigidMaterialIndex = heightfieldShape.materialIndex;

			ref = npWorkItem.transformCacheRef1;
			PxsCachedTransform heightfieldTransformCache = transformCache[ref];
			PxsCachedTransform clothTransformCache = transformCache[npWorkItem.transformCacheRef0];

			const PxTransform& hfTranform = heightfieldTransformCache.transform;
			const PxTransform& clothTranform = clothTransformCache.transform;

			const PxReal cDistance = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[ref];

			PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightfieldShape.hullOrMeshPtr);
			const PxU32 nbRows = heightfieldData[0];
			const PxU32 nbCols = heightfieldData[1];
			PxHeightFieldSample* samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);

			PxVec3 a0, b0, c0;
			//vert is in local space of the height field
			getTriangle(a0, b0, c0, NULL, hfTriangleIdx, heightfieldShape.scale, nbRows, nbCols, samples);

			const PxTransform hfToClothTransform(clothTranform.transformInv(hfTranform));
			const PxVec3 localA0 = hfToClothTransform.transform(a0);
			const PxVec3 localB0 = hfToClothTransform.transform(b0);
			const PxVec3 localC0 = hfToClothTransform.transform(c0);

			clothId = clothShape.particleOrSoftbodyId;

			const PxgFEMCloth* cloth = &clothes[clothId];
			const uint4 clothVertInd = cloth->mTriangleVertexIndices[clothTriangleIdx];

			hasContact = staticTriangleVsClothCollision(localA0, localB0, localC0, cloth, clothVertInd, clothTriangleIdx,
				clothTransformCache.transform, cDistance, c);
		}

		PxU32 index = globalScanExclusiveSingleWarp(hasContact, writer.totalContactCount);

		if (hasContact)
		{
			PxNodeIndex rigidId = shapeToRigidRemapTable[ref];
			writer.writeRigidVsDeformableContact32(index, make_float4(c.position.x, c.position.y, c.position.z, restDistance[cmIdx]),
				make_float4(c.normal.x, c.normal.y, c.normal.z, c.separation), c.barycentric, rigidId.getInd(), PxEncodeClothIndex(clothId, clothTriangleIdx), rigidMaterialIndex, PxU32(rigidId.getInd()));
		}
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}
}



extern "C" __global__
void cloth_meshContactGenLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				clothes,
	const PxNodeIndex* PX_RESTRICT				shapeToRigidRemapTable,

	const PxU32									stackSizeBytes,
	PxU8* PX_RESTRICT							stackPtr,
	PxU32* PX_RESTRICT							midphasePairsNum,

	PxgRigidFilterPair*							filterPairs,
	const PxU32									nbFilterPairs,

	PxU32* PX_RESTRICT							stackSizeNeededOnDevice,

	PxgFEMContactWriter							writer
)
{
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);

	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	uint4* pairs = reinterpret_cast<uint4*>(stackPtr);
	
	PxU32 l = ((numPairs + WARP_SIZE - 1) / WARP_SIZE) * WARP_SIZE;
	PxgFemContact c;

	for (PxU32 i = globalThreadIdx; i < l; i += gridDim.x * blockDim.x)
	{
		bool hasContact = false;
		PxU32 cmIdx;
		PxU32 clothId;
		PxU32 clothTriangleIdx;
		PxNodeIndex rigidId;

		PxU32 rigidMaterialIndex = 0xFFFFFFFF;
		if (i < numPairs) 
		{
			const uint4 curPair = pairs[i];

			cmIdx = curPair.x;
			clothTriangleIdx = curPair.y; //this will be the cloth triangle index			
			const PxU32 meshTriangleIdx = curPair.z; //this will be the mesh triangle index

			const PxgContactManagerInput npWorkItem = cmInputs[cmIdx];
			PxgShape shape0 = gpuShapes[npWorkItem.shapeRef0];
			PxgShape shape1 = gpuShapes[npWorkItem.shapeRef1];

			const PxReal cDistance = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];


			PxgShape* trimeshShape = NULL;
			PxgShape* clothShape = NULL;
			PxU32 clothCacheRef;
			PxU32 trimeshCacheRef;

			if (shape0.particleOrSoftbodyId != 0xffffffff)
			{
				clothShape = &shape0;
				trimeshShape = &shape1;

				clothCacheRef = npWorkItem.transformCacheRef0;
				trimeshCacheRef = npWorkItem.transformCacheRef1;
			}
			else
			{
				clothShape = &shape1;
				trimeshShape = &shape0;

				clothCacheRef = npWorkItem.transformCacheRef1;
				trimeshCacheRef = npWorkItem.transformCacheRef0;
			}
			rigidMaterialIndex = trimeshShape->materialIndex;

			rigidId = shapeToRigidRemapTable[trimeshCacheRef];

			clothId = clothShape->particleOrSoftbodyId;

			const PxgFEMCloth* cloth = &clothes[clothId];
			const uint4 clothVertInd = cloth->mTriangleVertexIndices[clothTriangleIdx];

			const PxU32 clothMask0 = PxEncodeClothIndex(clothId, clothVertInd.x);
			const PxU32 clothMask1 = PxEncodeClothIndex(clothId, clothVertInd.y);
			const PxU32 clothMask2 = PxEncodeClothIndex(clothId, clothVertInd.z);

			//If we find all verts in the filter pair list, we don't need to generate contacts between the rigid shape and the triangle
			bool skipTriangle =	find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask0) &&
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask1) &&
								find(filterPairs, nbFilterPairs, rigidId.getInd(), clothMask2);

			if (!skipTriangle)
			{
				PxsCachedTransform trimeshTransformCache = transformCache[trimeshCacheRef];
				PxsCachedTransform clothTransformCache = transformCache[clothCacheRef];

				const PxTransform& trimeshToWorld = trimeshTransformCache.transform;
				const PxMeshScale& trimeshScale = trimeshShape->scale;

				PxU8 * trimeshGeomPtr = reinterpret_cast<PxU8*>(trimeshShape->hullOrMeshPtr);

				const uint4 nbVerts_nbTri_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
				trimeshGeomPtr += sizeof(uint4) + sizeof(const Gu::BV32DataPacked)* nbVerts_nbTri_maxDepth_nbBv32TreeNodes.w;

				const float4* trimeshVerts = reinterpret_cast<const float4 *>(trimeshGeomPtr);

				trimeshGeomPtr += sizeof(float4) * nbVerts_nbTri_maxDepth_nbBv32TreeNodes.x;

				const uint4* trimeshVertIndices = reinterpret_cast<const uint4 *>(trimeshGeomPtr);

				//Keep for later improvements
				/*trimeshGeomPtr += sizeof(uint4)* nbVerts_nbTri_maxDepth_nbBv32TreeNodes.y;
				const uint4* PX_RESTRICT trimeshTriAdjacencies = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
				uint4 adjTriangles = trimeshTriAdjacencies[clothTriangleIdx];*/


				const uint4 triVertInd = trimeshVertIndices[meshTriangleIdx];
				const PxVec3 triV0 = PxLoad3(trimeshVerts[triVertInd.x]);
				const PxVec3 triV1 = PxLoad3(trimeshVerts[triVertInd.y]);
				const PxVec3 triV2 = PxLoad3(trimeshVerts[triVertInd.z]);

				const PxTransform trimeshToClothTransform(clothTransformCache.transform.transformInv(trimeshToWorld));
				const PxVec3 localA0 = trimeshToClothTransform.transform(vertex2Shape(triV0, trimeshScale.scale, trimeshScale.rotation));
				const PxVec3 localB0 = trimeshToClothTransform.transform(vertex2Shape(triV1, trimeshScale.scale, trimeshScale.rotation));
				const PxVec3 localC0 = trimeshToClothTransform.transform(vertex2Shape(triV2, trimeshScale.scale, trimeshScale.rotation));


				hasContact = staticTriangleVsClothCollision(localA0, localB0, localC0, cloth, clothVertInd, clothTriangleIdx,
					clothTransformCache.transform, cDistance, c);
			}
		}

		PxU32 index = globalScanExclusiveSingleWarp(hasContact, writer.totalContactCount);

		if (hasContact)
		{
			writer.writeRigidVsDeformableContact32(index, make_float4(c.position.x, c.position.y, c.position.z, restDistance[cmIdx]),
				make_float4(c.normal.x, c.normal.y, c.normal.z, c.separation), c.barycentric, rigidId.getInd(), PxEncodeClothIndex(clothId, clothTriangleIdx), rigidMaterialIndex, PxU32(rigidId.getInd()));
		}
	}

	if (globalThreadIdx == 0)
	{
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
	}	
}
