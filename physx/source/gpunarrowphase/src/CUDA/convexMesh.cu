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

#include <cuda.h>
#include <cuda_runtime.h>
#include <assert.h>

#include "foundation/PxVec3.h"
#include "foundation/PxPlane.h"
#include "foundation/PxTransform.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxGeometry.h"

#include "PxgCommonDefines.h"
#include "utils.cuh"
#include "reduction.cuh"
#include "sphereTriangle.cuh"
#include "convexTriangle.cuh"
#include "capsuleTriangle.cuh"
#include "midphaseAllocate.cuh"

#include "PxgNpKernelIndices.h"
#include "triangleMesh.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels2() {}

// Triangle scaling, triangle verts in shape space
__device__ void getShapeSpaceVerts(PxVec3& v0, PxVec3& v1, PxVec3& v2, const MeshScaling& meshScale, PxU32 triangleIdx, const uint4* PX_RESTRICT trimeshTriIndices, const float4* PX_RESTRICT trimeshVerts)
{
	const uint4 triIndices = trimeshTriIndices[triangleIdx];

	const PxU32 triVertIdx0 = triIndices.x;
	const PxU32 triVertIdx1 = triIndices.y;
	const PxU32 triVertIdx2 = triIndices.z;

	const float4 triV0_f4 = trimeshVerts[triVertIdx0];
	const float4 triV1_f4 = trimeshVerts[triVertIdx1];
	const float4 triV2_f4 = trimeshVerts[triVertIdx2];

	PxVec3 triV0 = PxLoad3(triV0_f4);
	PxVec3 triV1 = PxLoad3(triV1_f4);
	PxVec3 triV2 = PxLoad3(triV2_f4);

	meshScale.getShapeSpaceVert(triV0, triV1, triV2, v0, v1, v2);
}

__device__ void convexTrimeshNarrowphaseCore(
	PxU32 globalWarpIndex,
	PxgShape& convexShape,
	PxgShape& trimeshShape,
	PxU32 transformCacheRef0,
	PxU32 transformCacheRef1,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,							// per CM
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,					// per cvx-tri
	ConvexTriContacts** PX_RESTRICT cvxTriContactsPtr,					// per cvx-tri
	PxReal** PX_RESTRICT cvxTriMaxDepthPtr,								// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,			// per cvx-tri
	PxU32** PX_RESTRICT orderedCvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairPtr,						// per cvx-tri
	const uint4* PX_RESTRICT pairsGPU,
	PxU32* nbPairsGlobal,
	PxU32* nbSecondPassPairs,
	volatile PxU32* s_WarpSharedMemory,
	ConvexTriContact* tempConvexTriContacts,
	const PxU32 tempContactSizeBytes,
	PxU32* pTempContactIndex
)
{
	// 1 warp is doing 1 triangle - convex pair here.

	ConvexMeshScratch* s_scratch = (ConvexMeshScratch*)s_WarpSharedMemory;

	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	__shared__ __align__(16) char sConvexTransformCached[sizeof(PxsCachedTransform) * NP_TRIMESH_WARPS_PER_BLOCK];
	__shared__ __align__(16) char sTrimeshTransformCached[sizeof(PxsCachedTransform) * NP_TRIMESH_WARPS_PER_BLOCK];

	PxsCachedTransform* convexTransformCached = reinterpret_cast<PxsCachedTransform*>(sConvexTransformCached);
	PxsCachedTransform* trimeshTransformCached = reinterpret_cast<PxsCachedTransform*>(sTrimeshTransformCached);


	if (threadIdx.x < sizeof(PxsCachedTransform) / sizeof(PxU32))
	{
		reinterpret_cast<PxU32*>(&convexTransformCached[threadIdx.y])[threadIdx.x] =
			reinterpret_cast<const PxU32*>(transformCache + transformCacheRef0)[threadIdx.x];

		reinterpret_cast<PxU32*>(&trimeshTransformCached[threadIdx.y])[threadIdx.x] =
			reinterpret_cast<const PxU32*>(transformCache + transformCacheRef1)[threadIdx.x];
	}

	ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
	PxU32 convexTriPairOffset = pair.startIndex + testOffset;
	PxU32 convexTriPairOffsetPadded = pair.roundedStartIndex + testOffset;

	__syncwarp();

	if (threadIdx.x < 7)
	{
		reinterpret_cast<PxU32*>(&s_scratch->convexScale)[threadIdx.x] = reinterpret_cast<PxU32*>(&convexShape.scale)[threadIdx.x];
	}


	if (threadIdx.x == 0)
	{
		// convexTransform/trimeshTransform already include bodyTM/relTM
		s_scratch->trimeshToConvexTransform = convexTransformCached[threadIdx.y].transform.transformInv(trimeshTransformCached[threadIdx.y].transform);
		// Shapes
		//s_scratch->contactDist = convexShape.contactOffset + trimeshShape.contactOffset;
		s_scratch->contactDist = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1];

		const PxU8* convexPtrA = reinterpret_cast<const PxU8 *>(convexShape.hullOrMeshPtr);
		s_scratch->convexPtrA = convexPtrA;

		const float4 hull0_centerOfMass_f4 = *reinterpret_cast<const float4 *>(convexPtrA);
		const PxVec3 hull0_centerOfMass(hull0_centerOfMass_f4.x, hull0_centerOfMass_f4.y, hull0_centerOfMass_f4.z);
		// Transform CoM into shape space
		PxVec3 shapeSpaceCenterOfMass0 = vertex2Shape(hull0_centerOfMass, convexShape.scale.scale, convexShape.scale.rotation);

		s_scratch->convexCenterOfMass = shapeSpaceCenterOfMass0;

		convexPtrA += sizeof(float4);

		const uint4 tmp = *((uint4*)convexPtrA);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;
		s_scratch->nbEdgesNbHullVerticesNbPolygons = polyData0_NbEdgesNbHullVerticesNbPolygons;

		convexPtrA += sizeof(uint4);

		// Geometries : Triangle Mesh
		const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

		const uint4* PX_RESTRICT trimeshTriAdjacencies;
		readTriangleMesh(trimeshGeomPtr, s_scratch->trimeshVerts, s_scratch->trimeshTriIndices, trimeshTriAdjacencies, s_scratch->trimeshFaceRemap);
		

		// TODO: shuffle this across threads (reading uint4 seq)
		// later shuffle adjacency work and shuffle back adjNormals
		uint4 triAdjTriIndices = trimeshTriAdjacencies[triangleIdx];

		s_scratch->triAdjTrisIdx[0] = triAdjTriIndices.x;
		s_scratch->triAdjTrisIdx[1] = triAdjTriIndices.y;
		s_scratch->triAdjTrisIdx[2] = triAdjTriIndices.z;
	}

	__syncwarp();

	if (threadIdx.x < 3)
	{
		const PxMeshScale& scale = trimeshShape.scale;
		bool flipNormal = ((scale.scale.x * scale.scale.y * scale.scale.z) < 0.0f);
		PxU32 writeIndex = threadIdx.x;
		if (threadIdx.x > 0 && flipNormal)
			writeIndex = threadIdx.x ^ 3;

		const PxU32 triVertIdx = reinterpret_cast<const PxU32*>(&s_scratch->trimeshTriIndices[triangleIdx])[threadIdx.x];

		const float4 triV_f4 = s_scratch->trimeshVerts[triVertIdx];

		PxVec3 triV = PxLoad3(triV_f4);

		triV = vertex2Shape(triV, trimeshShape.scale.scale, trimeshShape.scale.rotation);

		PxVec3 triLocVerts = s_scratch->trimeshToConvexTransform.transform(triV);

		s_scratch->triLocVerts[writeIndex] = triLocVerts;
	}
	__syncwarp();
	if (threadIdx.x == 0)
	{
		PxVec3 triangleLocNormal;
		triangleLocNormal = (s_scratch->triLocVerts[1] - s_scratch->triLocVerts[0]).cross(s_scratch->triLocVerts[2] - s_scratch->triLocVerts[0]).getNormalized();

		s_scratch->triangleLocNormal = triangleLocNormal;
	}
	__syncwarp();

	const PxU32 remapCpuTriangleIdx = s_scratch->trimeshFaceRemap[triangleIdx];
	convexTriangleContactGen(
		s_WarpSharedMemory, s_scratch, convexTriPairOffset, convexTriPairOffsetPadded, remapCpuTriangleIdx, triangleIdx, globalWarpIndex,
		cvxTriNIPtr, cvxTriContactsPtr, cvxTriMaxDepthPtr, cvxTriIntermPtr, orderedCvxTriIntermPtr, cvxTriSecondPassPairPtr, nbSecondPassPairs,
		tempConvexTriContacts, tempContactSizeBytes/sizeof(ConvexTriContact), pTempContactIndex);
}



__device__ PxU32 sphereTrimeshNarrowphaseCore(
	PxU32 globalThreadIndex,
	const PxgContactManagerInput& npWorkItem,
	const PxgShape& sphereShape,
	const PxgShape& trimeshShape,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	SphereMeshPair* PX_RESTRICT sphereTrimeshPair,					// per CM
	SphereTriNormalAndIndex** PX_RESTRICT sphereTriNIPtr,			// per sphere-tri
	SphereTriContacts** PX_RESTRICT sphereTriContactsPtr,			// per sphere-tri
	PxReal** PX_RESTRICT sphereTriMaxDepthPtr,						// per sphere-tri
	SphereTriIntermediateData** PX_RESTRICT sphereTriIntermPtr,		// per sphere-tri
	PxU32** PX_RESTRICT orderedSphereTriIntermPtr,					// per sphere-tri
	PxU32** PX_RESTRICT sphereTriSecondPassPairPtr,					// per sphere-tri
	PxU32* nbPairsGlobal,
	PxU32* nbSecondPassPairs,
	PxVec3& outContact,
	PxReal& outSep,
	PxU32& outMask 
)
{
	// Pair, stackPtr in convexMeshMidphase kernel
	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	PxsCachedTransform sphereTransformCached = transformCache[npWorkItem.transformCacheRef0];
	PxsCachedTransform trimeshTransformCached = transformCache[npWorkItem.transformCacheRef1];

	SphereMeshPair& pair = sphereTrimeshPair[cmIdx];
	PxU32 sphereTriPairOffset = pair.startIndex + testOffset;
	PxU32 sphereTriPairOffsetPadded = pair.roundedStartIndex + testOffset;


	// Shapes
	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
	const PxQuat trimeshScaleRot = trimeshShape.scale.rotation;

	const PxReal sphereRadius = sphereShape.scale.scale.x;
	const PxVec3 trimeshScale = trimeshShape.scale.scale;

	//Geometries : Sphere
	const PxVec3 sphereCenterInTriMesh = trimeshTransformCached.transform.transformInv(sphereTransformCached.transform.p);

	// Geometries : Triangle Mesh
	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const float4 * trimeshVerts;
	const uint4 * trimeshTriIndices;
	const uint4* PX_RESTRICT trimeshTriAdjacencies;
	const PxU32* PX_RESTRICT trimeshFaceRemap;
	readTriangleMesh(trimeshGeomPtr, trimeshVerts, trimeshTriIndices, trimeshTriAdjacencies, trimeshFaceRemap);


	const PxMeshScale& scale = trimeshShape.scale;
	MeshScaling meshScale(scale.scale, scale.rotation);

	PxVec3 v0, v1, v2;
	getShapeSpaceVerts(v0, v1, v2, meshScale, triangleIdx, trimeshTriIndices, trimeshVerts);

	const PxVec3 n = (v1 - v0).cross(v2 - v0).getNormalized();


	uint4 triAdjTriIndices = trimeshTriAdjacencies[triangleIdx];
	if (meshScale.flipNormal)
		PxSwap(triAdjTriIndices.x, triAdjTriIndices.z);


	const PxU32 remapCpuTriangleIdx = trimeshFaceRemap[triangleIdx];

	const PxReal inflatedRadius = sphereRadius + contactDist;

	const PxReal d = v0.dot(n);
	const PxReal dist0 = sphereCenterInTriMesh.dot(n) - d;


	PxReal separation = PX_MAX_F32;

	PxU32 nbOutputContacts = 0;
	bool delayContacts = false;
	bool edgeContact = false;
	PxVec3 patchNormal(0.f);

	if (dist0 >= 0)
	{
		PxVec3 closestP;
		//mSphereCenter will be in the local space of the triangle mesh
		bool generateContact = false;
		bool faceContact = false;
		
		PxU32 mask;
		const PxReal sqDist = distancePointTriangleSquared(sphereCenterInTriMesh, v0, v1, v2, triAdjTriIndices, closestP, generateContact, faceContact, &mask);

		const PxReal sqInflatedSphereRadius = inflatedRadius * inflatedRadius;
		//sphere overlap with triangles
		if (sqInflatedSphereRadius > sqDist && generateContact)
		{
			//printf("remapCpuTriangleIdx %i sqInflatedSphereRadius %f sqDist %f faceContact %i\n", remapCpuTriangleIdx, sqInflatedSphereRadius, sqDist, PxU32(faceContact));
			patchNormal = n;
			if (!faceContact)
				patchNormal = (sphereCenterInTriMesh - closestP).getNormalized();

			//printf("remapCpuTriangleIdx %i closestP(%f, %f, %f)\n", remapCpuTriangleIdx, closestP.x, closestP.y, closestP.z);

			const PxReal cosTheta = patchNormal.dot(n);
			const PxReal tolerance = 0.996f;//around 5 degree

			//two normal's projection less than 5 degree, generate contacts
			delayContacts = cosTheta <= tolerance;
			if (delayContacts)
			{
				//delay contacts
				//PxU32* PX_RESTRICT sphereTriSecondPairPass = *sphereTriSecondPassPairPtr;
				//const PxU32 startIndex = atomicAdd(nbSecondPassPairs, 1);

				// PT: no need to compute the mask, we got it for free from distancePointTriangleSquared
				//PxU32 mask = getTriangleActiveMask(closestP, v0, v1, v2);

				//sphereTriSecondPairPass[startIndex] = mask | globalThreadIndex;
				outMask = mask | globalThreadIndex;

				// VR: is an edge contact?
				edgeContact = ConvexTriIntermediateData::isEdgeContact(mask);
			}

			nbOutputContacts = 1;
			separation = PxSqrt(sqDist);


			// Generate contacts
			//SphereTriContacts* PX_RESTRICT sphereTriContacts = *sphereTriContactsPtr;

			//float4* contactBuf = reinterpret_cast<float4 *>(sphereTriContacts + sphereTriPairOffset);
			//sphere center in sphere local space
			//contactBuf[0] = make_float4(0.f, 0.f, 0.f, separation);

			//normal = patchNormal;

			outSep = separation;

			outContact = PxVec3(0.f);


		}
	}

	PxReal* PX_RESTRICT sphereTriMaxDepth = *sphereTriMaxDepthPtr;
	sphereTriMaxDepth[sphereTriPairOffset] = separation;

	SphereTriIntermediateData* PX_RESTRICT sphereTriInterm = *sphereTriIntermPtr;
	PxU32* PX_RESTRICT orderedSphereTriInterm = *orderedSphereTriIntermPtr;
	sphereTriInterm[sphereTriPairOffset].gpuTriIndex = triangleIdx;

	// VR: bits 30 and 31 are used for the search flags
	assert(triangleIdx < (1 << 30));

	PxU32 orderedTriangleIdx = triangleIdx;
	if (nbOutputContacts)
		if (!delayContacts) // not dalayed -> face contact
			orderedTriangleIdx |= TriangleSearchFlags::eFACE_CONTACT;
		else if (edgeContact)
			orderedTriangleIdx |= TriangleSearchFlags::eEDGE_CONTACT;

	orderedSphereTriInterm[sphereTriPairOffsetPadded] = orderedTriangleIdx;

	SphereTriNormalAndIndex* PX_RESTRICT sphereTriNI = *sphereTriNIPtr;

	PxU32 delayContactMask = delayContacts ? SphereTriNormalAndIndex::DeferredContactMask : 0;

	//rotate the normal into A space 
	PxVec3 worldNormal = trimeshTransformCached.transform.rotate(-patchNormal);
	sphereTriNI[sphereTriPairOffset].normal = sphereTransformCached.transform.rotateInv(worldNormal);
	sphereTriNI[sphereTriPairOffset].index = delayContactMask + (nbOutputContacts << SphereTriNormalAndIndex::NbContactsShift) + remapCpuTriangleIdx;

	//printf("sphereTriPairOffset %i nbOutputContacts%i \n", sphereTriPairOffset, nbOutputContacts);

	return nbOutputContacts;
}


__device__ PxU32 capsuleTrimeshNarrowphaseCore(
	PxU32 globalThreadIndex,
	const PxgContactManagerInput& npWorkItem,
	const PxgShape& capsuleShape,
	const PxgShape& trimeshShape,
	const uint4 curPair,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	SphereMeshPair* PX_RESTRICT sphereTrimeshPair,					// per CM
	SphereTriNormalAndIndex** PX_RESTRICT sphereTriNIPtr,			// per sphere-tri
	SphereTriContacts** PX_RESTRICT sphereTriContactsPtr,			// per sphere-tri
	PxReal** PX_RESTRICT sphereTriMaxDepthPtr,						// per sphere-tri
	SphereTriIntermediateData** PX_RESTRICT sphereTriIntermPtr,		// per sphere-tri
	PxU32** PX_RESTRICT orderedSphereTriIntermPtr,					// per sphere-tri
	PxU32** PX_RESTRICT sphereTriSecondPassPairPtr,					// per sphere-tri
	PxU32* nbSecondPassPairs,
	PxVec3* contacts,
	PxReal* separations,
	PxU32& outMask
)
{
	// Pair, stackPtr in convexMeshMidphase kernel
	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	PxsCachedTransform capsuleTransformCached = transformCache[npWorkItem.transformCacheRef0];
	PxsCachedTransform trimeshTransformCached = transformCache[npWorkItem.transformCacheRef1];

	SphereMeshPair& pair = sphereTrimeshPair[cmIdx];
	PxU32 sphereTriPairOffset = pair.startIndex + testOffset;
	PxU32 sphereTriPairOffsetPadded = pair.roundedStartIndex + testOffset;


	// Shapes
	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
	const PxQuat trimeshScaleRot = trimeshShape.scale.rotation;

	const PxReal capsuleRadius = capsuleShape.scale.scale.y;
	const PxReal capsuleHalfHeight = capsuleShape.scale.scale.x;
	const PxVec3 trimeshScale = trimeshShape.scale.scale;

	//Geometries : Capsule
	const PxTransform meshToCapsule = capsuleTransformCached.transform.transformInv(trimeshTransformCached.transform);

	//Capsule in triangle mesh space
	const PxVec3 tmp = capsuleTransformCached.transform.q.getBasisVector0() * capsuleHalfHeight;
	const PxVec3 capsuleCenterInMesh = trimeshTransformCached.transform.transformInv(capsuleTransformCached.transform.p);
	const PxVec3 capsuleDirInMesh = trimeshTransformCached.transform.rotateInv(tmp);
	//Geometries : Capsule in mesh local space
	const PxVec3 p0 = capsuleCenterInMesh + capsuleDirInMesh;
	const PxVec3 p1 = capsuleCenterInMesh - capsuleDirInMesh;

	// Geometries : Triangle Mesh
	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const float4 * trimeshVerts;
	const uint4 * trimeshTriIndices;
	const uint4* PX_RESTRICT trimeshTriAdjacencies;
	readTriangleMesh(trimeshGeomPtr, trimeshVerts, trimeshTriIndices, trimeshTriAdjacencies);


	const PxMeshScale& scale = trimeshShape.scale;
	MeshScaling meshScale(scale.scale, scale.rotation);

	PxVec3 v0, v1, v2;
	getShapeSpaceVerts(v0, v1, v2, meshScale, triangleIdx, trimeshTriIndices, trimeshVerts);

	const PxVec3 triNormal = (v1 - v0).cross(v2 - v0).getNormalized();

	uint4 triAdjTrisIdx = trimeshTriAdjacencies[triangleIdx];
	if (meshScale.flipNormal)
		PxSwap(triAdjTrisIdx.x, triAdjTrisIdx.z);

	const PxReal inflatedRadius = capsuleRadius + contactDist;

	//const PxReal d = v0.dot(n);
	//const PxReal dist0 = sphereCenterInTriMesh.dot(n) - d;

	const PxReal dist = (capsuleCenterInMesh - v0).dot(triNormal);

	PxReal separation = PX_MAX_F32;
	PxU32 nbOutputContacts = 0;
	PxVec3 patchNormal(0.f);
	bool delayContacts = false;
	bool edgeContact = false;

	if (dist >= 0)
	{
		//PxVec3 closestP;

		PxReal t, u, v;
		PxReal sqDist = distanceSegmentTriangleSquared(p0, p1, v0, v1, v2, t, u, v);

		const PxReal sqInflatedRadius = inflatedRadius * inflatedRadius;

		//capsule overlap with triangles
		
		if (sqInflatedRadius > sqDist)
		{
// VR: I made this similar to sphere-trimesh
// and I left the old variant for the reference
#if 1
			if (sqDist < FLT_EPSILON)
			{
				//segment intersect with the triangle
				patchNormal = triNormal;
			}
			else
			{
				const PxVec3 pointOnSegment = p0 + (p1 - p0) * t;
				const PxVec3 pointOnTriangle = v0 * (1.f - u - v) + v1 * u + v2 * v;
				patchNormal = (pointOnSegment - pointOnTriangle).getNormalized();

				const PxReal cosTheta = patchNormal.dot(triNormal);
				const PxReal tolerance = 0.996f;//around 5 degree

				//printf("pointOnSegment: (%g %g %g); pointOnTriangle: (%g %g %g); cosTheta: %g\n", pointOnSegment.x, pointOnSegment.y, pointOnSegment.z, pointOnTriangle.x, pointOnTriangle.y, pointOnTriangle.z, cosTheta);

				//two normal's projection less than 5 degree, generate contacts
				delayContacts = cosTheta <= tolerance;
			}

			if (delayContacts)
			{
				PxU32 mask = getTriangleActiveMaskFromNormal(v0, v1, v2, patchNormal);
				outMask = mask | globalThreadIndex;

				//const PxVec3 pn = patchNormal;
				//printf("pn: (%g %g %g); mask: %x\n", pn.x, pn.y, pn.z, mask);

				// is an edge contact?
				edgeContact = ConvexTriIntermediateData::isEdgeContact(mask);
			}
#else
			PxU32 triFlags = 0;

			triFlags |= (triAdjTrisIdx.x != BOUNDARY && (triAdjTrisIdx.x & NONCONVEX_FLAG) ? 0 : 1 << 3);
			triFlags |= (triAdjTrisIdx.y != BOUNDARY && (triAdjTrisIdx.y & NONCONVEX_FLAG) ? 0 : 1 << 4);
			triFlags |= (triAdjTrisIdx.z != BOUNDARY && (triAdjTrisIdx.z & NONCONVEX_FLAG) ? 0 : 1 << 5);

			if (selectNormal(u, v, triFlags))
			{
				patchNormal = triNormal;
			}
			else
			{
				if (sqDist < 1e-6f)
				{
					//segment intersect with the triangle
					patchNormal = triNormal;
				}
				else
				{
					//edge normal
					const PxVec3 pq = p1 - p0;
					const PxVec3 pointOnSegment = p0 + pq * t; // V3ScaleAdd(pq, t, mCapsule.p0);
					const PxReal w = 1.f - (u + v);// FSub(FOne(), FAdd(u, v));
					const PxVec3 pointOnTriangle = v0 * w + v1 * u + v2 * v;// V3ScaleAdd(p0, w, V3ScaleAdd(p1, u, V3Scale(p2, v)));
					patchNormal = (pointOnSegment - pointOnTriangle).getNormalized();// V3Normalize(V3Sub(pointOnSegment, pointOnTriangle));
					delayContacts = true;

				}
			}
#endif

			nbOutputContacts = generateContacts(v0, v1, v2, triNormal, patchNormal, p0, p1, inflatedRadius, contacts, separations);
			
			generateEEContacts(v0, v1, v2, patchNormal, p0, p1, sqInflatedRadius, contacts, separations, nbOutputContacts);

			for (PxU32 i = 0; i < nbOutputContacts; ++i)
			{
				//transform contact back to the capsule space
				contacts[i] = meshToCapsule.transform(contacts[i]);
				const PxReal pen = separations[i];
				separation = PxMin(separation, pen);
			}
		}
	}

	SphereTriIntermediateData* PX_RESTRICT sphereTriInterm = *sphereTriIntermPtr;
	PxU32* PX_RESTRICT orderedSphereTriInterm = *orderedSphereTriIntermPtr;
	sphereTriInterm[sphereTriPairOffset].gpuTriIndex = triangleIdx;

	//if (nbOutputContacts)
	//	printf("triangleIdx: %d; nbOutputContacts: %d; delayContacts %d; edgeContact: %d\n", triangleIdx, nbOutputContacts, delayContacts, edgeContact);

	// VR: bits 30 and 31 are used for the search flags
	assert(triangleIdx < (1 << 30));

	PxU32 orderedTriangleIdx = triangleIdx;
	if (nbOutputContacts)
		if (!delayContacts) // not dalayed -> face contact
			orderedTriangleIdx |= TriangleSearchFlags::eFACE_CONTACT;
		else if (edgeContact)
			orderedTriangleIdx |= TriangleSearchFlags::eEDGE_CONTACT;

	orderedSphereTriInterm[sphereTriPairOffsetPadded] = orderedTriangleIdx;

	/*sphereTriInterm[sphereTriPairOffset].triAdjTrisIds[0] = triAdjTrisIdx.x;
	sphereTriInterm[sphereTriPairOffset].triAdjTrisIds[1] = triAdjTrisIdx.y;
	sphereTriInterm[sphereTriPairOffset].triAdjTrisIds[2] = triAdjTrisIdx.z;*/

	//PxReal* PX_RESTRICT sphereTriMaxDepth = sphereTriMaxDepthPtr[sphereTriPairOffset];
	PxReal* PX_RESTRICT capsuleTriMaxDepth = *sphereTriMaxDepthPtr;
	capsuleTriMaxDepth[sphereTriPairOffset] = separation;

	SphereTriNormalAndIndex* PX_RESTRICT sphereTriNI = *sphereTriNIPtr;

	PxU32 delayContactMask = delayContacts ? SphereTriNormalAndIndex::DeferredContactMask : 0;

	//rotate the normal into A space 
	sphereTriNI[sphereTriPairOffset].normal = meshToCapsule.rotate(-patchNormal);
	sphereTriNI[sphereTriPairOffset].index = delayContactMask + (nbOutputContacts << SphereTriNormalAndIndex::NbContactsShift) + triangleIdx;

	////printf("sphereTriPairOffset %i nbOutputContacts%i \n", sphereTriPairOffset, nbOutputContacts);

	return nbOutputContacts;
}


extern "C" __global__
//__launch_bounds__(NP_TRIMESH_WARPS_PER_BLOCK * WARP_SIZE, 32 / NP_TRIMESH_WARPS_PER_BLOCK)
void convexTrimeshNarrowphase(
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,						// per CM
	
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,				// per cvx-tri
	ConvexTriContacts** PX_RESTRICT cvxTriContactsPtr,				// per cvx-tri
	PxReal** PX_RESTRICT cvxTriMaxDepthPtr,							// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT orderedCvxTriIntermPtr,	// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,					// per cvx-tri
	
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT nbPairsGlobal,
	PxU32* PX_RESTRICT nbPaddedPairsGlobal,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	const PxU32 stackSizeBytes,
	ConvexTriContact* tempConvexTriContacts,
	PxU32* pTempContactIndex,
	PxU32* maxTempMemRequirement,
	PxU32* midPhasePairsNeeded,
	const PxU32 nbContactManagers
)
{
	__shared__ ConvexTriNormalAndIndex* sCvxTriNIPtr;
	__shared__ ConvexTriContacts* sCvxTriContactsPtr;
	__shared__ PxReal*  sCvxTriMaxDepthPtr;
	__shared__ ConvexTriIntermediateData* sCvxTriIntermPtr;
	__shared__ PxU32* sOrderedCvxTriIntermPtr;
	__shared__ uint4*	sPairsGPU;
	__shared__ PxU32*	sCvxTriSecondPassPairsPtr;

	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;

	__shared__ volatile PxU32 sharedMem[warpsPerBlock][WARP_SIZE * 16];
	volatile PxU32* s_WarpSharedMemory = sharedMem[threadIdx.y];

	// we don't need to check overflow here anymore, the midphase checks it.
	const PxU32 nbPairs = *nbPairsGlobal;
	const PxU32 nbPaddedPairs = *nbPaddedPairsGlobal;

	//each block assign the corresponding ptr from the stack memory to sCvxTriNIPtr, sCvxTriContactPtr and sCvxTriMaxDepthPtr
	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		midphaseAllocate(&sCvxTriNIPtr, &sCvxTriContactsPtr, &sCvxTriMaxDepthPtr, &sCvxTriIntermPtr, &sOrderedCvxTriIntermPtr, &sCvxTriSecondPassPairsPtr, &sPairsGPU, stackPtr, nbPairs, nbPaddedPairs);
		if (blockIdx.x == 0)
		{
			atomicMax(maxTempMemRequirement, calculateConvexMeshPairMemRequirement() * (*midPhasePairsNeeded) + calculateAdditionalPadding(nbContactManagers));
		}
	}
	__syncthreads();

	// the first block writes back the pointers to global memory.
	if (threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0)
	{
		*cvxTriNIPtr = sCvxTriNIPtr;
		*cvxTriContactsPtr = sCvxTriContactsPtr;
		*cvxTriMaxDepthPtr = sCvxTriMaxDepthPtr;
		*cvxTriIntermPtr = sCvxTriIntermPtr;
		*orderedCvxTriIntermPtr = sOrderedCvxTriIntermPtr;

		*cvxTriSecondPassPairsPtr = sCvxTriSecondPassPairsPtr;
	}

	// 1 warp deals with 1 convexMeshPair - so 1 convex against 1 mesh triangle.
	for (PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y; globalWarpIndex < nbPairs; globalWarpIndex += gridDim.x * blockDim.y)
	{
		uint4 curPair = sPairsGPU[globalWarpIndex];
		const PxU32 cmIdx = curPair.x;

		PxgContactManagerInput npWorkItem;
		PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

		PxU32 transformCacheRef0 = npWorkItem.transformCacheRef0;
		PxU32 transformCacheRef1 = npWorkItem.transformCacheRef1;
		PxU32 shapeRef0 = npWorkItem.shapeRef0;
		PxU32 shapeRef1 = npWorkItem.shapeRef1;

		__shared__ __align__(16) char sShape0[sizeof(PxgShape) * NP_TRIMESH_WARPS_PER_BLOCK];
		__shared__ __align__(16) char sShape1[sizeof(PxgShape) * NP_TRIMESH_WARPS_PER_BLOCK];

		PxgShape* shape0 = reinterpret_cast<PxgShape*>(sShape0);
		PxgShape* shape1 = reinterpret_cast<PxgShape*>(sShape1);

		if (threadIdx.x < sizeof(PxgShape) / sizeof(PxU32))
		{
			reinterpret_cast<PxU32*>(&shape0[threadIdx.y])[threadIdx.x] = reinterpret_cast<const PxU32*>(gpuShapes + shapeRef0)[threadIdx.x];
			reinterpret_cast<PxU32*>(&shape1[threadIdx.y])[threadIdx.x] = reinterpret_cast<const PxU32*>(gpuShapes + shapeRef1)[threadIdx.x];
		}

		PxgShape* shape = &shape0[threadIdx.y];
		PxgShape* trimeshShape = &shape1[threadIdx.y];

		__syncwarp();

		bool flip = shape0[threadIdx.y].type == PxGeometryType::eTRIANGLEMESH;

		if (flip)
		{
			PxSwap(transformCacheRef0, transformCacheRef1);
			PxSwap(shapeRef0, shapeRef1);

			shape = &shape1[threadIdx.y];
			trimeshShape = &shape0[threadIdx.y];
		}

		convexTrimeshNarrowphaseCore(
			globalWarpIndex,
			*shape,
			*trimeshShape,
			transformCacheRef0,
			transformCacheRef1,
			curPair,
			transformCache,
			contactDistance,
			gpuShapes,

			cvxTrimeshPair,
			&sCvxTriNIPtr,
			&sCvxTriContactsPtr,
			&sCvxTriMaxDepthPtr,
			&sCvxTriIntermPtr,
			&sOrderedCvxTriIntermPtr,
			&sCvxTriSecondPassPairsPtr,

			sPairsGPU,
			nbPairsGlobal,
			nbSecondPassPairs,
			s_WarpSharedMemory,
			tempConvexTriContacts,
			stackSizeBytes,
			pTempContactIndex
		);
	}
}

extern "C" __global__
void sphereTrimeshNarrowphase(
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	SphereMeshPair* PX_RESTRICT sphereTrimeshPair,						// per CM

	SphereTriNormalAndIndex** PX_RESTRICT sphereTriNIPtr,				// per sphere-tri
	SphereTriContacts** PX_RESTRICT sphereTriContactsPtr,				// per sphere-tri
	PxReal** PX_RESTRICT sphereTriMaxDepthPtr,							// per sphere-tri
	SphereTriIntermediateData** PX_RESTRICT sphereTriIntermPtr,			// per sphere-tri
	PxU32** PX_RESTRICT orderedSphereTriIntermPtr,	// per sphere-tri
	PxU32** PX_RESTRICT sphereTriSecondPassPairsPtr,					// per sphere-tri

	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT nbPairsGlobal,
	PxU32* PX_RESTRICT nbPaddedPairsGlobal,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	const PxU32 stackSizeBytes,
	ConvexTriContact* tempConvexTriContacts,
	PxU32* pTempContactIndex,
	PxU32* maxTempMemRequirement,
	PxU32* midphasePairsNeeded,
	const PxU32 nbContactManagers
)
{

	//the pointer is shared memory. However, the memory we are writing to is in global memory
	__shared__ SphereTriNormalAndIndex* sSphereTriNIPtr;
	__shared__ SphereTriContacts* sSphereTriContactsPtr;
	__shared__ PxReal*  sSphereTriMaxDepthPtr;
	__shared__ SphereTriIntermediateData* sSphereTriIntermPtr;
	__shared__ PxU32* sOrderedSphereTriIntermPtr;
	__shared__ uint4*	sPairsGPU;
	__shared__ PxU32*	sSphereTriSecondPassPairsPtr;

	__shared__ char sContacts[sizeof(PxVec3) * 128];
	PxVec3* shContacts = reinterpret_cast<PxVec3*>(sContacts);
	__shared__ PxReal shSeparations[128];

	const PxU32 nbPairs = *nbPairsGlobal;
	const PxU32 nbPaddedPairs = *nbPaddedPairsGlobal;

	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		midphaseAllocate(&sSphereTriNIPtr, &sSphereTriContactsPtr, &sSphereTriMaxDepthPtr, &sSphereTriIntermPtr, &sOrderedSphereTriIntermPtr, &sSphereTriSecondPassPairsPtr, &sPairsGPU, stackPtr, nbPairs, nbPaddedPairs);
		if (blockIdx.x == 0)
		{
			atomicMax(maxTempMemRequirement, calculateConvexMeshPairMemRequirement() * (*midphasePairsNeeded) + calculateAdditionalPadding(nbContactManagers));
		}
	}

	__syncthreads();


	if (threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0)
	{
		*sphereTriNIPtr = sSphereTriNIPtr;
		*sphereTriContactsPtr = sSphereTriContactsPtr;
		*sphereTriMaxDepthPtr = sSphereTriMaxDepthPtr;
		*sphereTriIntermPtr = sSphereTriIntermPtr;
		*orderedSphereTriIntermPtr = sOrderedSphereTriIntermPtr;
		*sphereTriSecondPassPairsPtr = sSphereTriSecondPassPairsPtr;
	}

	//each thread deal with 
	//for (PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x; globalThreadIndex < nbPairs; globalThreadIndex += gridDim.x * blockDim.x)
	for (PxU32 i = 0; i < nbPairs; i += gridDim.x * blockDim.x)
	{
		PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x + i;

		PxU32 nbContacts = 0;
		PxU32 outMask = 0;
		uint4 curPair;

		if (globalThreadIndex < nbPairs)
		{

			//printf("globalThreadIndex %i nbPairs %i\n", globalThreadIndex, nbPairs);
			// Pair, stackPtr in convexMeshMidphase kernel
			curPair = sPairsGPU[globalThreadIndex];

			const PxU32 cmIdx = curPair.x;

			PxgContactManagerInput npWorkItem = cmInputs[cmIdx];


			const PxgShape* shape0 = &gpuShapes[npWorkItem.shapeRef0];
			const PxgShape* trimeshShape = &gpuShapes[npWorkItem.shapeRef1];
			PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0->type);

			
			bool flip = type0 == PxGeometryType::eTRIANGLEMESH;

			if (flip)
			{
				PxSwap(npWorkItem.shapeRef0, npWorkItem.shapeRef1);
				PxSwap(npWorkItem.transformCacheRef0, npWorkItem.transformCacheRef1);

				PxSwap(shape0, trimeshShape);

				type0 = PxGeometryType::Enum(shape0->type);
			}

			if (type0 == PxGeometryType::eSPHERE)
			{
				nbContacts = sphereTrimeshNarrowphaseCore(
					globalThreadIndex,

					npWorkItem,
					*shape0,
					*trimeshShape,
					curPair,
					transformCache,
					contactDistance,
					sphereTrimeshPair,
					&sSphereTriNIPtr,
					&sSphereTriContactsPtr,
					&sSphereTriMaxDepthPtr,
					&sSphereTriIntermPtr,
					&sOrderedSphereTriIntermPtr,
					&sSphereTriSecondPassPairsPtr,
					nbPairsGlobal,
					nbSecondPassPairs,
					shContacts[threadIdx.x*2],
					shSeparations[threadIdx.x*2],
					outMask
				);
			}
			else
			{
				nbContacts = capsuleTrimeshNarrowphaseCore(
					globalThreadIndex,
					npWorkItem,
					*shape0,
					*trimeshShape,
					curPair,
					transformCache,
					contactDistance,
					sphereTrimeshPair,
					&sSphereTriNIPtr,
					&sSphereTriContactsPtr,
					&sSphereTriMaxDepthPtr,
					&sSphereTriIntermPtr,
					&sOrderedSphereTriIntermPtr,
					&sSphereTriSecondPassPairsPtr,
					nbSecondPassPairs,
					&shContacts[threadIdx.x * 2],
					&shSeparations[threadIdx.x * 2],
					outMask
				);

			}
		}

		PxU32 outMaskWriteMask = __ballot_sync(FULL_MASK, outMask);
		PxU32 count = __popc(outMaskWriteMask);
		PxU32 startIndex = 0;

		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		//Now output contacts!!!!

		PxU32 inclusiveSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, nbContacts);

		PxU32 totalContacts = __shfl_sync(FULL_MASK, inclusiveSum, 31);

		if (totalContacts)
		{
			if (count)
			{
				if (threadIndexInWarp == 0)
					startIndex = atomicAdd(nbSecondPassPairs, count);

				startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

				if (outMask)
				{
					PxU32 offset = warpScanExclusive(outMaskWriteMask, threadIndexInWarp);
					sSphereTriSecondPassPairsPtr[startIndex + offset] = outMask;
				}
			}

			if (threadIndexInWarp == 31)
				startIndex = atomicAdd(pTempContactIndex, inclusiveSum);

			startIndex = __shfl_sync(FULL_MASK, startIndex, 31);

			startIndex += inclusiveSum - nbContacts;

			for (PxU32 i = 0, idx = 2*threadIdx.x; i < nbContacts; ++i, idx++)
			{
				tempConvexTriContacts[startIndex + i].contact_sepW = make_float4(shContacts[idx].x, shContacts[idx].y,
					shContacts[idx].z, shSeparations[idx]);
			}

			if (nbContacts)
			{
				PxU32 cmIdx = curPair.x;
				PxU32 testOffset = curPair.z;

				SphereMeshPair& pair = sphereTrimeshPair[cmIdx];
				PxU32 sphereTriPairOffset = pair.startIndex + testOffset;

				sSphereTriContactsPtr[sphereTriPairOffset].index = startIndex;
			}
		}



	}
}
