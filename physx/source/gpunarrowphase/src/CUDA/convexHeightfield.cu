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
#include "foundation/PxTransform.h"
#include "geometry/PxMeshScale.h"

#include "convexNpCommon.h"
#include "cudaNpCommon.h"

#include "PxgPersistentContactManifold.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgNpKernelIndices.h"
#include "PxsTransformCache.h"

#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "nputils.cuh"
#include "midphaseAllocate.cuh"
#include "dataReadWriteHelper.cuh"
#include "convexTriangle.cuh"
#include "heightfieldUtil.cuh"
#include "sphereTriangle.cuh"
#include "sphereCollision.cuh"
#include "capsuleTriangle.cuh"
#include "geometry/PxHeightFieldFlag.h"
#include "geometry/PxGeometry.h"


using namespace physx;

extern "C" __host__ void initNarrowphaseKernels1() {}

////ML: for heightfield, the maximum numbers of adjacent verts will be 6
//#define	HEIGHTFIELD_MAX_ADJACENCY_VERT_INDICE				6
//#define	HEIGHTFIELD_MAX_PROCESS_ADJACENCY_VERT_INDICE		9
#define	HEIGHTFIELD_MAX_VERTS								18	//allow duplication
//
//#define	HEIGHTFIELD_DEBUG									1

struct PxgTriangle
{

public:
	__device__ PX_FORCE_INLINE	PxVec3	denormalizedNormal() const
	{
		return (verts[1] - verts[0]).cross(verts[2] - verts[0]);
	}

	PxVec3 verts[3];
};

__device__ static void getTriangle(PxgTriangle& triLoc, PxU32* adjacencyIndices, const PxU32 triangleIndex, const PxMeshScale& scale, const PxU32 rows, const PxU32 columns, const PxHeightFieldSample* samples)
{

	const PxReal rowScale = scale.scale.x;
	const PxReal heightScale = scale.scale.y;
	const PxReal columnScale = scale.scale.z;
	PxVec3 handedness(1.0f);	// Vector to invert normal coordinates according to the heightfield scales
	bool wrongHanded = false;
	if (columnScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.z = -1.0f;
	}
	if (rowScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.x = -1.0f;
	}

	PxU32 tVertexIndices[3];
	getTriangleVertexIndices(triangleIndex, tVertexIndices[0], tVertexIndices[1 + wrongHanded], tVertexIndices[2 - wrongHanded], columns, samples);


	if (adjacencyIndices)
	{
		getTriangleAdjacencyIndices(triangleIndex, adjacencyIndices[wrongHanded ? 2 : 0], adjacencyIndices[1], adjacencyIndices[wrongHanded ? 0 : 2], rows, columns, samples);
	}

	for (PxU32 vi = 0; vi < 3; vi++)
	{
		const PxVec3 vertex = getVertex(tVertexIndices[vi], columns, samples);
		triLoc.verts[vi] = hf2shapep(vertex, rowScale, heightScale, columnScale);
	}
}



__device__ void buildTriangleInformations(const PxU32 triangleIdx, PxgShape& heightfieldShape, ConvexMeshScratch* s_scratch,
	PxTransform& trimeshToConvexTransform)
{
	const PxU32 * heightfieldGeomPtr = reinterpret_cast<const PxU32 *>(heightfieldShape.hullOrMeshPtr);

	const PxU32 rows = *heightfieldGeomPtr++;
	const PxU32 columns = *heightfieldGeomPtr++;
	const PxHeightFieldSample* samples = reinterpret_cast<const PxHeightFieldSample*>(heightfieldGeomPtr);


	//PxU32 vertexIndices[3];
	PxU32 adjacencyIndices[3];
	PxgTriangle triLocal; // height field local space triangle
	getTriangle(triLocal, adjacencyIndices, triangleIdx, heightfieldShape.scale, rows, columns, samples);


	const PxVec3 triNormal = triLocal.denormalizedNormal();
	const PxVec3 triangleLocNormal = trimeshToConvexTransform.rotate(triNormal).getNormalized();

	if (threadIdx.x < 1)
	{
		s_scratch->triangleLocNormal = triangleLocNormal;
		s_scratch->trimeshToConvexTransform = trimeshToConvexTransform;
		s_scratch->trimeshVerts = NULL;
		//s_scratch->trimeshVerts = trimeshVerts;
	}


	if (threadIdx.x < 3)
	{
		//assign adjacent triangle indices to the scratch memory
		reinterpret_cast<PxU32*>(&s_scratch->triAdjTrisIdx)[threadIdx.x] = adjacencyIndices[threadIdx.x];
		

		//assign triangle vertes in the local space of convex hull to the scratch memory
		s_scratch->triLocVerts[threadIdx.x] = trimeshToConvexTransform.transform(triLocal.verts[threadIdx.x]);
	}
}


__device__ void convexHeightfieldNarrowphaseCore(
	PxU32 globalWarpIndex,
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
	PxU32** PX_RESTRICT cvxTriSecondPassPairPtr,					// per cvx-tri

	const uint4 * PX_RESTRICT pairsGPU,
	PxU32* PX_RESTRICT nbPairsGlobal,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	ConvexTriContact* tempConvexTriContacts,
	const PxU32 tempContactSizeBytes,
	PxU32* pTempContactIndex
	)
{
	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
	const PxU32 wrapIndex = threadIdx.y;

	//for heightfield, each vertex in a triangle has maximum 6 adacent vertexes, which invoke 12 vertexes in total
	//__shared__ PxU32 sTrimeshAdjVerts[warpsPerBlock * HEIGHTFIELD_MAX_VERTS];
	__shared__ volatile PxU32 sharedMem[warpsPerBlock][ WARP_SIZE * 16];
	volatile PxU32* s_WarpSharedMemory = sharedMem[wrapIndex];

	ConvexMeshScratch* s_scratch = (ConvexMeshScratch*)s_WarpSharedMemory;

	// Pair
	uint4 curPair = pairsGPU[globalWarpIndex];

	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

	PxsCachedTransform convexTransformCached, trimeshTransformCached;

	PxsCachedTransform_ReadWarp(convexTransformCached, transformCache + npWorkItem.transformCacheRef0);
	PxsCachedTransform_ReadWarp(trimeshTransformCached, transformCache + npWorkItem.transformCacheRef1);

	PxTransform trimeshToConvexTransform(convexTransformCached.transform.transformInv(trimeshTransformCached.transform));

	PxgShape convexShape;
	PxgShape_ReadWarp(convexShape, gpuShapes + npWorkItem.shapeRef0);

	PxgShape heightfieldShape;
	PxgShape_ReadWarp(heightfieldShape, gpuShapes + npWorkItem.shapeRef1);

	/*const PxU32 startIndex = wrapIndex * HEIGHTFIELD_MAX_VERTS;
	PxU32* startTrimeshAdjVerts = &sTrimeshAdjVerts[startIndex];
	float4* startTrimeshVerts = &sTrimeshVerts[startIndex];
	buildAdjacencyInformations(triangleIdx, heightfieldShape, startTrimeshAdjVerts, startTrimeshVerts, s_scratch, trimeshToConvexTransform);*/
	buildTriangleInformations(triangleIdx, heightfieldShape, s_scratch, trimeshToConvexTransform);

	ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
	PxU32 convexTriPairOffset = pair.startIndex + testOffset;
	PxU32 convexTriPairOffsetPadded = pair.roundedStartIndex + testOffset;
	// Geometries : Convex

	if (threadIdx.x < 7)
	{
		reinterpret_cast<PxU32*>(&s_scratch->convexScale)[threadIdx.x] = reinterpret_cast<PxU32*>(&convexShape.scale)[threadIdx.x];
	}

	if (threadIdx.x == 0)
	{
		// Shapes
		//s_scratch->contactDist = convexShape.contactOffset + heightfieldShape.contactOffset;
		s_scratch->contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
		s_scratch->trimeshToConvexTransform = trimeshToConvexTransform;
		
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

		/*const float4 polyData0_Extents = *((float4*)(convexPtrA));
		PxVec3 convex_extents = toVec3(polyData0_Extents);
		s_scratch->extents = convex_extents;*/
	}
	
	__syncwarp();

	//height field don't have cpu remap triangle index
	convexTriangleContactGen(
		s_WarpSharedMemory, s_scratch, convexTriPairOffset, convexTriPairOffsetPadded, triangleIdx, triangleIdx, globalWarpIndex,
		cvxTriNIPtr, cvxTriContactsPtr,cvxTriMaxDepthPtr, cvxTriIntermPtr, orderedCvxTriIntermPtr, cvxTriSecondPassPairPtr, nbSecondPassPairs,
		tempConvexTriContacts, tempContactSizeBytes / sizeof(ConvexTriContact), pTempContactIndex);
}

extern "C" __global__
__launch_bounds__(NP_TRIMESH_WARPS_PER_BLOCK * WARP_SIZE, 32 / NP_TRIMESH_WARPS_PER_BLOCK)
void convexHeightfieldNarrowphase( 
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,							// per CM

	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,					// per cvx-tri
	ConvexTriContacts** PX_RESTRICT cvxTriContactsPtr,					// per cvx-tri
	PxReal** PX_RESTRICT cvxTriMaxDepthPtr,								// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,			// per cvx-tri
	PxU32** PX_RESTRICT orderedCvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,						// per cvx-tri

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
	__shared__ ConvexTriNormalAndIndex* sCvxTriNIPtr;
	__shared__ ConvexTriContacts* sCvxTriContactsPtr;
	__shared__ PxReal*  sCvxTriMaxDepthPtr;
	__shared__ ConvexTriIntermediateData* sCvxTriIntermPtr;
	__shared__ PxU32* sOrderedCvxTriIntermPtr;
	__shared__ uint4*	sPairsGPU;
	__shared__ PxU32*	sCvxTriSecondPassPairsPtr;

	const PxU32 nbPairs = *nbPairsGlobal;
	const PxU32 nbPaddedPairs = *nbPaddedPairsGlobal;

	//each block assign the corresponding ptr from the stack memory to sCvxTriNIPtr, sCvxTriContactPtr and sCvxTriMaxDepthPtr
	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		midphaseAllocate(&sCvxTriNIPtr, &sCvxTriContactsPtr, &sCvxTriMaxDepthPtr, &sCvxTriIntermPtr, &sOrderedCvxTriIntermPtr, &sCvxTriSecondPassPairsPtr, &sPairsGPU, stackPtr, nbPairs, nbPaddedPairs);
		if (blockIdx.x == 0)
		{
			atomicMax(maxTempMemRequirement, calculateConvexMeshPairMemRequirement() * (*midphasePairsNeeded) + calculateAdditionalPadding(nbContactManagers));
		}
	}
	__syncthreads();

	if (threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0)
	{
		*cvxTriNIPtr = sCvxTriNIPtr;
		*cvxTriContactsPtr = sCvxTriContactsPtr;
		*cvxTriMaxDepthPtr = sCvxTriMaxDepthPtr;
		*cvxTriIntermPtr = sCvxTriIntermPtr;
		*orderedCvxTriIntermPtr = sOrderedCvxTriIntermPtr;
		*cvxTriSecondPassPairsPtr = sCvxTriSecondPassPairsPtr;
	}
	
	for (PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y; globalWarpIndex < nbPairs; globalWarpIndex += gridDim.x * blockDim.y)
	{
		convexHeightfieldNarrowphaseCore(
			globalWarpIndex,

			cmInputs,
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
			tempConvexTriContacts,
			stackSizeBytes,
			pTempContactIndex
			);
	}
}

__device__ PxU32 sphereHeightfieldNarrowphaseCore(
	PxU32 globalThreadIndex,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	SphereMeshPair* PX_RESTRICT	sphereTrimeshPair,						// per CM
	SphereTriNormalAndIndex** PX_RESTRICT sphereTriNIPtr,				// per sphere-tri
	SphereTriContacts** PX_RESTRICT sphereTriContactsPtr,				// per sphere-tri
	PxReal** PX_RESTRICT sphereTriMaxDepthPtr,							// per sphere-tri
	SphereTriIntermediateData** PX_RESTRICT sphereTriIntermPtr,			// per sphere-tri
	PxU32** PX_RESTRICT orderedSphereTriIntermPtr,	// per sphere-tri
	PxU32** PX_RESTRICT sphereTriSecondPassPairPtr,						// per sphere-tri

	const uint4& curPair,
	const PxgContactManagerInput& npWorkItem,
	PxgShape& sphereShape,
	PxU32* PX_RESTRICT nbPairsGlobal,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	PxVec3& outContact,
	PxReal& outSep,
	PxU32& outMask
)
{
	// Pair
	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	PxsCachedTransform sphereTransformCached = transformCache[npWorkItem.transformCacheRef0];
	PxsCachedTransform trimeshTransformCached = transformCache[npWorkItem.transformCacheRef1];

	//PxTransform trimeshToConvexTransform(sphereTransformCached.transform.transformInv(trimeshTransformCached.transform));

	PxgShape heightfieldShape = gpuShapes[npWorkItem.shapeRef1];
	//const PxU32 startIndex = threadIdx.x * HEIGHTFIELD_MAX_VERTS;
	//float4* startTrimeshVerts = &sTrimeshVerts[startIndex];

	//buildTriangleInformations
	const PxU32 * heightfieldGeomPtr = reinterpret_cast<const PxU32 *>(heightfieldShape.hullOrMeshPtr);

	const PxU32 rows = *heightfieldGeomPtr++;
	const PxU32 columns = *heightfieldGeomPtr++;
	const PxHeightFieldSample* samples = reinterpret_cast<const PxHeightFieldSample*>(heightfieldGeomPtr);

	//PxU32 vertexIndices[3];
	uint4 triAdjTriIndices;
	//PxgTriangle triLocal; // height field local space triangle
	PxVec3 triLocV0, triLocV1, triLocV2;
	getTriangle(triLocV0, triLocV1, triLocV2, &triAdjTriIndices, triangleIdx, heightfieldShape.scale, rows, columns, samples);

	//printf("threadIdx.x %i triangleIdx %i\n", threadIdx.x, triangleIdx);


	const PxVec3 triNormal = ((triLocV1 - triLocV0).cross(triLocV2 - triLocV0)).getNormalized();
	//const PxVec3 triNormal = triLocal.denormalizedNormal().getNormalized();
	//const PxVec3 triangleLocNormal = trimeshToConvexTransform.rotate(triNormal).getNormalized();


	ConvexMeshPair& pair = sphereTrimeshPair[cmIdx];
	PxU32 sphereTriPairOffset = pair.startIndex + testOffset;
	PxU32 sphereTriPairOffsetPadded = pair.roundedStartIndex + testOffset;
	

	// Shapes
	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
	const PxReal sphereRadius = sphereShape.scale.scale.x;

	//Geometries : Sphere
	const PxVec3 sphereCenterInTriMesh = trimeshTransformCached.transform.transformInv(sphereTransformCached.transform.p);


	const PxReal inflatedRadius = sphereRadius + contactDist;

	//const PxReal d = triLocal.verts[0].dot(triNormal);
	//const PxReal dist0 = sphereCenterInTriMesh.dot(triNormal) - d;
	const PxReal dist = (sphereCenterInTriMesh - triLocV0).dot(triNormal);

	PxReal separation = PX_MAX_F32;
	PxU32 nbOutputContacts = 0;
	PxVec3 patchNormal(0.f);
	PxVec3 closestP;
	//mSphereCenter will be in the local space of the triangle mesh
	bool generateContact = false;
	bool faceContact = false;
	bool delayContacts = false;
	PxU32 mask;
	PxReal sqDist = distancePointTriangleSquared(sphereCenterInTriMesh, triLocV0, triLocV1, triLocV2, triAdjTriIndices, closestP, generateContact, faceContact, &mask);

	

	
	const PxReal sqInflatedSphereRadius = inflatedRadius * inflatedRadius;
	//sphere overlap with triangles
	if (dist >= 0.f && sqInflatedSphereRadius > sqDist && generateContact)
	{
		//printf("triangleIdx %i sqDist %f\n", triangleIdx, sqDist);
		//printf("triangleIdx %i triNormal(%f, %f, %f)\n", triangleIdx, triNormal.x, triNormal.y, triNormal.z);
		//printf("triangleIdx % i closestP(%f, %f, %f)\n", triangleIdx, closestP.x, closestP.y, closestP.z);


		//printf("triangleIdx %i v0(%f, %f, %f)\n", triangleIdx, triLocal.verts[0].x, triLocal.verts[0].y, triLocal.verts[0].z);
		//printf("triangleIdx %i v1(%f, %f, %f)\n", triangleIdx, triLocal.verts[1].x, triLocal.verts[1].y, triLocal.verts[1].z);
		//printf("triangleIdx %i v2(%f, %f, %f)\n", triangleIdx, triLocal.verts[2].x, triLocal.verts[2].y, triLocal.verts[2].z);

		//printf("remapCpuTriangleIdx %i sqInflatedSphereRadius %f sqDist %f faceContact %i\n", remapCpuTriangleIdx, sqInflatedSphereRadius, sqDist, PxU32(faceContact));

		switch (mask)
		{
		case ConvexTriIntermediateData::eE01:
		{
			PxVec3 triLocV0Adj, triLocV1Adj, triLocV2Adj;
			getTriangle(triLocV0Adj, triLocV1Adj, triLocV2Adj, NULL, triAdjTriIndices.x, heightfieldShape.scale, rows, columns, samples);
			const PxVec3 triNormalAdj = ((triLocV1Adj - triLocV0Adj).cross(triLocV2Adj - triLocV0Adj)).getNormalized();

			generateContact = ((triNormalAdj.dot(triLocV2 - triLocV0Adj)) < 0.f);

			break;
		}
		case ConvexTriIntermediateData::eE12:
		{
			PxVec3 triLocV0Adj, triLocV1Adj, triLocV2Adj;
			getTriangle(triLocV0Adj, triLocV1Adj, triLocV2Adj, NULL, triAdjTriIndices.y, heightfieldShape.scale, rows, columns, samples);
			const PxVec3 triNormalAdj = ((triLocV1Adj - triLocV0Adj).cross(triLocV2Adj - triLocV0Adj)).getNormalized();

			/*printf("%x: triNormalAdj = (%f, %f, %f), triNormal = (%f, %f, %f)\n", triangleIdx, triNormalAdj.x, triNormalAdj.y, triNormalAdj.z, 
				triNormal.x, triNormal.y, triNormal.z);*/

			generateContact = (triNormalAdj.dot(triLocV0 - triLocV0Adj) < 0.f);
			
			break;
		}
		case ConvexTriIntermediateData::eE02:
		{
			PxVec3 triLocV0Adj, triLocV1Adj, triLocV2Adj;
			getTriangle(triLocV0Adj, triLocV1Adj, triLocV2Adj, NULL, triAdjTriIndices.z, heightfieldShape.scale, rows, columns, samples);
			const PxVec3 triNormalAdj = ((triLocV1Adj - triLocV0Adj).cross(triLocV2Adj - triLocV0Adj)).getNormalized();

			generateContact = ((triNormalAdj.dot(triLocV1 - triLocV0Adj)) < 0.f);
			break;
		}
		
		}

		if (generateContact)
		{

			patchNormal = triNormal;
			if (!faceContact)
				patchNormal = (sphereCenterInTriMesh - closestP).getNormalized();

			//const PxU32 value = PxU32(faceContact);
			//printf("triangleIdx %i faceContact %i\n", triangleIdx, value);
			//printf("remapCpuTriangleIdx %i closestP(%f, %f, %f)\n", remapCpuTriangleIdx, closestP.x, closestP.y, closestP.z);

			const PxReal cosTheta = patchNormal.dot(triNormal);
			const PxReal tolerance = 0.996f;//around 5 degree

											//two normal's projection less than 5 degree, generate contacts
			delayContacts = cosTheta <= tolerance;
			if (delayContacts)
			{
				//delay contacts
				/*PxU32* PX_RESTRICT sphereTriSecondPairPass = *sphereTriSecondPassPairPtr;
				const PxU32 startIndex = atomicAdd(nbSecondPassPairs, 1);
				sphereTriSecondPairPass[startIndex] = globalThreadIndex;*/
				outMask = globalThreadIndex | mask;
				//printf("remapCpuTriangleIdx %i gpuTriangleIdx %i delayContacts\n", remapCpuTriangleIdx, triangleIdx);
			}

			//printf("triangleIdx % i delayContacts %i\n", triangleIdx, PxU32(delayContacts));

			nbOutputContacts = 1;
			separation = PxSqrt(sqDist);

			outContact = PxVec3(0.f);
			outSep = separation;
		}

	}

	PxReal* PX_RESTRICT sphereTriMaxDepth = *sphereTriMaxDepthPtr;
	sphereTriMaxDepth[sphereTriPairOffset] = separation;

	SphereTriIntermediateData* PX_RESTRICT sphereTriInterm = *sphereTriIntermPtr;
	PxU32* PX_RESTRICT orderedSphereTriInterm = *orderedSphereTriIntermPtr;
	sphereTriInterm[sphereTriPairOffset].gpuTriIndex = triangleIdx;
	orderedSphereTriInterm[sphereTriPairOffsetPadded] = (nbOutputContacts && !delayContacts) ? (0x80000000 | triangleIdx) : triangleIdx;

	SphereTriNormalAndIndex* PX_RESTRICT sphereTriNI = *sphereTriNIPtr;

	PxU32 delayContactMask = delayContacts ? SphereTriNormalAndIndex::DeferredContactMask : 0;

	//rotate the normal into A space 
	PxVec3 worldNormal = trimeshTransformCached.transform.rotate(-patchNormal);
	sphereTriNI[sphereTriPairOffset].normal = sphereTransformCached.transform.rotateInv(worldNormal);
	sphereTriNI[sphereTriPairOffset].index = delayContactMask + (nbOutputContacts << SphereTriNormalAndIndex::NbContactsShift) + triangleIdx;

	//printf("triangleIdx %i sphereTriPairOffset %i\n", triangleIdx, sphereTriPairOffset);

	return nbOutputContacts;

}


__device__ PxU8 computeTriangleFlags(
	const PxgTriangle& currentTriangle, 
	const PxVec3& triNormal,
	const PxU32* adjTriIndices,
	const PxU16 flags,
	const PxMeshScale& scale, 
	const PxU32 rows, 
	const PxU32 columns, 
	const PxHeightFieldSample* samples
	)
{
	const bool boundaryCollisions = !(flags & PxHeightFieldFlag::eNO_BOUNDARY_EDGES);
	PxU8 triFlags = 0;
	const PxU8 nextInd[] = { 2,0,1 };
	//fill in triangle flag
	for (PxU32 a = 0; a < 3; ++a)
	{
		if (adjTriIndices[a] != 0xFFFFFFFF)
		{
			PxgTriangle adjTri;

			getTriangle(adjTri, NULL, adjTriIndices[a], scale, rows, columns, samples);

			PxVec3 adjNormal = adjTri.denormalizedNormal();
			PxU32 otherIndex = nextInd[a];
			PxReal projD = adjNormal.dot(currentTriangle.verts[otherIndex] - adjTri.verts[0]);
			if (projD < 0.f)
			{
				adjNormal.normalize();

				PxReal proj = adjNormal.dot(triNormal);

				if (proj < 0.997f)
				{
					triFlags |= (1 << (a + 3));
				}
			}
			else if (boundaryCollisions)
			{
				triFlags |= (1 << (a + 3)); //Mark boundary edge active
			}
			else
				triFlags |= (1 << a); //Mark as silhouette edge

		}
	}

	return triFlags;

}


__device__ PxU32 capsuleHeightfieldNarrowphaseCore(
	PxU32 globalThreadIndex,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	SphereMeshPair* PX_RESTRICT	sphereTrimeshPair,						// per CM
	SphereTriNormalAndIndex** PX_RESTRICT sphereTriNIPtr,				// per sphere-tri
	SphereTriContacts** PX_RESTRICT sphereTriContactsPtr,				// per sphere-tri
	PxReal** PX_RESTRICT sphereTriMaxDepthPtr,							// per sphere-tri
	SphereTriIntermediateData** PX_RESTRICT sphereTriIntermPtr,			// per sphere-tri
	PxU32** PX_RESTRICT orderedSphereTriIntermPtr,	// per sphere-tri
	PxU32** PX_RESTRICT sphereTriSecondPassPairPtr,						// per sphere-tri

	const uint4& curPair,
	const PxgContactManagerInput& npWorkItem,
	PxgShape& capsuleShape,
	PxVec3* contacts,
	PxReal* separations
)
{
	// Pair
	const PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;

	PxsCachedTransform capsuleTransformCached = transformCache[npWorkItem.transformCacheRef0];
	PxsCachedTransform trimeshTransformCached = transformCache[npWorkItem.transformCacheRef1];

	PxgShape heightfieldShape = gpuShapes[npWorkItem.shapeRef1];

	//buildTriangleInformations
	const PxU32 * heightfieldGeomPtr = reinterpret_cast<const PxU32 *>(heightfieldShape.hullOrMeshPtr);

	const PxU32 rows = *heightfieldGeomPtr++;
	const PxU32 columns = *heightfieldGeomPtr++;
	const PxHeightFieldSample* samples = reinterpret_cast<const PxHeightFieldSample*>(heightfieldGeomPtr);

	heightfieldGeomPtr += sizeof(PxU32) * rows * columns;
	
	const PxU16 flags = reinterpret_cast<PxU16&>(heightfieldGeomPtr);

	
	PxU32 adjTriIndices[3];
	PxgTriangle currentTriangle; // height field local space triangle
	getTriangle(currentTriangle, adjTriIndices, triangleIdx, heightfieldShape.scale, rows, columns, samples);


	ConvexMeshPair& pair = sphereTrimeshPair[cmIdx];
	PxU32 sphereTriPairOffset = pair.startIndex + testOffset;
	PxU32 sphereTriPairOffsetPadded = pair.roundedStartIndex + testOffset;

	// Shapes
	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
	const PxReal capsuleRadius = capsuleShape.scale.scale.y;
	const PxReal capsuleHalfHeight = capsuleShape.scale.scale.x;


	const PxTransform heighfieldToCapsule = capsuleTransformCached.transform.transformInv(trimeshTransformCached.transform);

	const PxVec3 tmp = capsuleTransformCached.transform.q.getBasisVector0() * capsuleHalfHeight;
	const PxVec3 capsuleCenterInMesh = trimeshTransformCached.transform.transformInv(capsuleTransformCached.transform.p);
	const PxVec3 capsuleDirInMesh = trimeshTransformCached.transform.rotateInv(tmp);
	//Geometries : Capsule in height field local space
	const PxVec3 p0 = capsuleCenterInMesh + tmp;
	const PxVec3 p1 = capsuleCenterInMesh - tmp;


	const PxReal inflatedRadius = capsuleRadius + contactDist;

	const PxVec3 triNormal = currentTriangle.denormalizedNormal().getNormalized();// ((triLocV1 - triLocV0).cross(triLocV2 - triLocV0)).getNormalized();

	//const PxReal d = triLocal.verts[0].dot(triNormal);
	//const PxReal dist0 = sphereCenterInTriMesh.dot(triNormal) - d;
	const PxReal dist = (capsuleCenterInMesh - currentTriangle.verts[0]).dot(triNormal);

	PxU32 nbOutputContacts = 0;
	PxVec3 patchNormal(0.f);
	PxVec3 closestP;
	//mSphereCenter will be in the local space of the triangle mesh

	PxReal t, u, v;
	PxReal sqDist = distanceSegmentTriangleSquared(p0, p1, currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2], t, u, v);


	const PxReal sqInflatedRadius = inflatedRadius * inflatedRadius;
	PxReal separation = PX_MAX_F32;
	bool deferred = false;
	//capsule overlap with triangles
	if (dist >= 0.f && sqInflatedRadius > sqDist)
	{

		const PxU8 triFlags = computeTriangleFlags(currentTriangle, triNormal, adjTriIndices,
			flags, heightfieldShape.scale, rows, columns, samples);
		//printf("triangleIdx %i sqDist %f\n", triangleIdx, sqDist);
		//printf("triangleIdx %i triNormal(%f, %f, %f)\n", triangleIdx, triNormal.x, triNormal.y, triNormal.z);
		//printf("triangleIdx % i closestP(%f, %f, %f)\n", triangleIdx, closestP.x, closestP.y, closestP.z);


		//printf("triangleIdx %i v0(%f, %f, %f)\n", triangleIdx, triLocal.verts[0].x, triLocal.verts[0].y, triLocal.verts[0].z);
		//printf("triangleIdx %i v1(%f, %f, %f)\n", triangleIdx, triLocal.verts[1].x, triLocal.verts[1].y, triLocal.verts[1].z);
		//printf("triangleIdx %i v2(%f, %f, %f)\n", triangleIdx, triLocal.verts[2].x, triLocal.verts[2].y, triLocal.verts[2].z);

		//printf("remapCpuTriangleIdx %i sqInflatedSphereRadius %f sqDist %f faceContact %i\n", remapCpuTriangleIdx, sqInflatedSphereRadius, sqDist, PxU32(faceContact));
		//patchNormal = triNormal;
		if (selectNormal(u, v, triFlags))
		{
			patchNormal = triNormal;
		}
		else
		{
			if (sqDist > 0.f)
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
				const PxVec3 pointOnTriangle = currentTriangle.verts[0] * w + currentTriangle.verts[1] * u + currentTriangle.verts[2] * v;// V3ScaleAdd(p0, w, V3ScaleAdd(p1, u, V3Scale(p2, v)));
				patchNormal = (pointOnSegment - pointOnTriangle).getNormalized();// V3Normalize(V3Sub(pointOnSegment, pointOnTriangle));
				deferred = true;

			}
		}

		//const PxU32 value = PxU32(faceContact);
		//printf("triangleIdx %i faceContact %i\n", triangleIdx, value);
		//printf("remapCpuTriangleIdx %i closestP(%f, %f, %f)\n", remapCpuTriangleIdx, closestP.x, closestP.y, closestP.z)

		nbOutputContacts = generateContacts(currentTriangle.verts[0], currentTriangle.verts[1],
			currentTriangle.verts[2], triNormal, patchNormal, p0, p1, inflatedRadius, contacts, separations);


		generateEEContacts(currentTriangle.verts[0], currentTriangle.verts[1],
			currentTriangle.verts[2], patchNormal, p0, p1, sqInflatedRadius, contacts, separations, nbOutputContacts);

		for (PxU32 i = 0; i < nbOutputContacts; ++i)
		{
			//transform contact back to the capsule space
			contacts[i] = heighfieldToCapsule.transform(contacts[i]);
			const PxReal pen = separations[i];
			separation = PxMin(separation, pen);
		}

	}

	//printf("%i: triangleIndex %i separation %f\n", threadIdx.x, triangleIdx, separation);

	SphereTriIntermediateData* PX_RESTRICT sphereTriInterm = *sphereTriIntermPtr;
	PxU32* PX_RESTRICT orderedSphereTriInterm = *orderedSphereTriIntermPtr;
	sphereTriInterm[sphereTriPairOffset].gpuTriIndex = triangleIdx;
	const PxU32 deferMask = (nbOutputContacts && !deferred) ? 1 << 31 : 0;
	orderedSphereTriInterm[sphereTriPairOffsetPadded] = deferMask | triangleIdx;


	//PxReal* PX_RESTRICT sphereTriMaxDepth = sphereTriMaxDepthPtr[sphereTriPairOffset];
	PxReal* PX_RESTRICT capsuleTriMaxDepth = *sphereTriMaxDepthPtr;
	capsuleTriMaxDepth[sphereTriPairOffset] = separation;

	SphereTriNormalAndIndex* PX_RESTRICT sphereTriNI = *sphereTriNIPtr;

	//rotate the normal into A space 
	sphereTriNI[sphereTriPairOffset].normal = heighfieldToCapsule.rotate(-patchNormal);
	sphereTriNI[sphereTriPairOffset].index = (nbOutputContacts << SphereTriNormalAndIndex::NbContactsShift) + triangleIdx;

	//printf("triangleIdx %i sphereTriPairOffset %i\n", triangleIdx, sphereTriPairOffset);

	return nbOutputContacts;

}

extern "C" __global__
void sphereHeightfieldNarrowphase(
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,							// per CM

	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,					// per cvx-tri
	ConvexTriContacts** PX_RESTRICT cvxTriContactsPtr,					// per cvx-tri
	PxReal** PX_RESTRICT cvxTriMaxDepthPtr,								// per cvx-tri
	SphereTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,			// per cvx-tri
	PxU32** PX_RESTRICT orderedCvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,						// per cvx-tri

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
	__shared__ ConvexTriNormalAndIndex* sCvxTriNIPtr;
	__shared__ ConvexTriContacts* sCvxTriContactsPtr;
	__shared__ PxReal*  sCvxTriMaxDepthPtr;
	__shared__ ConvexTriIntermediateData* sCvxTriIntermPtr;
	__shared__ PxU32* sOrderedTriIntermPtr;
	__shared__ uint4*	sPairsGPU;
	__shared__ PxU32*	sCvxTriSecondPassPairsPtr;

	__shared__ char sShContacts[sizeof(PxVec3) * 128];
	PxVec3* shContacts = reinterpret_cast<PxVec3*>(sShContacts);
	__shared__ PxReal shSeparations[128];

	const PxU32 nbPairs = *nbPairsGlobal;
	const PxU32 nbPaddedPairs = *nbPaddedPairsGlobal;

	//each block assign the corresponding ptr from the stack memory to sCvxTriNIPtr, sCvxTriContactPtr and sCvxTriMaxDepthPtr
	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		midphaseAllocate(
			&sCvxTriNIPtr, 
			&sCvxTriContactsPtr, 
			&sCvxTriMaxDepthPtr, 
			&sCvxTriIntermPtr, 
			&sOrderedTriIntermPtr, 
			&sCvxTriSecondPassPairsPtr, 
			&sPairsGPU, 
			stackPtr, 
			nbPairs,
			nbPaddedPairs);
		if (blockIdx.x == 0)
		{
			atomicMax(maxTempMemRequirement, calculateConvexMeshPairMemRequirement() * (*midphasePairsNeeded) + calculateAdditionalPadding(nbContactManagers));
		}
	}
	__syncthreads();


	if (threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0)
	{
		*cvxTriNIPtr = sCvxTriNIPtr;
		*cvxTriContactsPtr = sCvxTriContactsPtr;
		*cvxTriMaxDepthPtr = sCvxTriMaxDepthPtr;
		*cvxTriIntermPtr = sCvxTriIntermPtr;
		*orderedCvxTriIntermPtr = sOrderedTriIntermPtr;
		*cvxTriSecondPassPairsPtr = sCvxTriSecondPassPairsPtr;
	}


	//for (PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x; globalThreadIndex < nbPairs; globalThreadIndex += gridDim.x * blockDim.x)
	for (PxU32 i = 0; i < nbPairs; i += gridDim.x * blockDim.x)
	{
		PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x + i;

		PxU32 nbContacts = 0;
		PxU32 outMask = 0;
		uint4 curPair;

		if (globalThreadIndex < nbPairs)
		{

			// Pair
			curPair = sPairsGPU[globalThreadIndex];

			PxgContactManagerInput npWorkItem = cmInputs[curPair.x];
			PxgShape shape0 = gpuShapes[npWorkItem.shapeRef0];
			PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0.type);
			if (type0 == PxGeometryType::eSPHERE)
			{
				nbContacts = sphereHeightfieldNarrowphaseCore(
					globalThreadIndex,

					cmInputs,
					transformCache,
					contactDistance,
					gpuShapes,
					cvxTrimeshPair,

					&sCvxTriNIPtr,
					&sCvxTriContactsPtr,
					&sCvxTriMaxDepthPtr,
					&sCvxTriIntermPtr,
					&sOrderedTriIntermPtr,
					&sCvxTriSecondPassPairsPtr,

					curPair,
					npWorkItem,
					shape0,
					nbPairsGlobal,
					nbSecondPassPairs,
					shContacts[threadIdx.x * 2],
					shSeparations[threadIdx.x * 2],
					outMask
				);
			}
			else if (type0 == PxGeometryType::eCAPSULE)
			{
				nbContacts = capsuleHeightfieldNarrowphaseCore(
					globalThreadIndex,

					cmInputs,
					transformCache,
					contactDistance,
					gpuShapes,
					cvxTrimeshPair,

					&sCvxTriNIPtr,
					&sCvxTriContactsPtr,
					&sCvxTriMaxDepthPtr,
					&sCvxTriIntermPtr,
					&sOrderedTriIntermPtr,
					&sCvxTriSecondPassPairsPtr,

					curPair,
					npWorkItem,
					shape0,
					&shContacts[threadIdx.x * 2],
					&shSeparations[threadIdx.x * 2]
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
					sCvxTriSecondPassPairsPtr[startIndex + offset] = outMask;
				}
			}

			if (threadIndexInWarp == 31)
				startIndex = atomicAdd(pTempContactIndex, inclusiveSum);

			startIndex = __shfl_sync(FULL_MASK, startIndex, 31);

			startIndex += inclusiveSum - nbContacts;

			for (PxU32 i = 0, idx = 2 * threadIdx.x; i < nbContacts; ++i, idx++)
			{
				tempConvexTriContacts[startIndex + i].contact_sepW = make_float4(shContacts[idx].x, shContacts[idx].y,
					shContacts[idx].z, shSeparations[idx]);
			}

			if (nbContacts)
			{
				PxU32 cmIdx = curPair.x;
				PxU32 testOffset = curPair.z;

				ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
				PxU32 sphereTriPairOffset = pair.startIndex + testOffset;

				sCvxTriContactsPtr[sphereTriPairOffset].index = startIndex;
			}


		}

	}
}