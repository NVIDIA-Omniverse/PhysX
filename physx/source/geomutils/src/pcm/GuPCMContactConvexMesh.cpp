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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geometry/PxTriangleMesh.h"
#include "geomutils/PxContactBuffer.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuVecTriangle.h"
#include "GuContactMethodImpl.h"
#include "GuPCMShapeConvex.h"
#include "GuConvexUtilsInternal.h"
#include "GuPCMContactConvexCommon.h"
#include "GuPCMContactMeshCallback.h"
#include "GuIntersectionTriangleBox.h"
#include "GuBox.h"
#include "CmMatrix34.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace aos;

namespace
{
struct PCMConvexVsMeshContactGenerationCallback : PCMMeshContactGenerationCallback<PCMConvexVsMeshContactGenerationCallback>
{
	PCMConvexVsMeshContactGenerationCallback& operator=(const PCMConvexVsMeshContactGenerationCallback&);

	PCMConvexVsMeshContactGeneration	mGeneration;
	const BoxPadded&					mBox;

	PCMConvexVsMeshContactGenerationCallback(
		const FloatVArg contactDistance,
		const FloatVArg replaceBreakingThreshold,
		const PxTransformV& convexTransform, 
		const PxTransformV& meshTransform,
		MultiplePersistentContactManifold& multiManifold,
		PxContactBuffer& contactBuffer,
		const PolygonalData& polyData,
		const SupportLocal* polyMap,
		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE>* delayedContacts,
		const FastVertex2ShapeScaling& convexScaling,
		bool idtConvexScale,
		const FastVertex2ShapeScaling& meshScaling,
		const PxU8* extraTriData,
		bool idtMeshScale,
		bool silhouetteEdgesAreActive,
		const BoxPadded& box,
		PxRenderOutput* renderOutput = NULL
	) :
		PCMMeshContactGenerationCallback<PCMConvexVsMeshContactGenerationCallback>(meshScaling, extraTriData, idtMeshScale),
		mGeneration(contactDistance, replaceBreakingThreshold, convexTransform, meshTransform, multiManifold, contactBuffer, polyData, 
			polyMap, delayedContacts, convexScaling, idtConvexScale, silhouetteEdgesAreActive, renderOutput),
		mBox(box)
	{
	}

	PX_FORCE_INLINE PxIntBool doTest(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2)
	{
		// PT: this one is safe because midphase vertices are directly passed to the function
		return intersectTriangleBox(mBox, v0, v1, v2);
	}

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMConvexVsMeshContactGeneration>(cache);
	}
};
}

bool Gu::PCMContactConvexMesh(const PolygonalData& polyData, const SupportLocal* polyMap, const FloatVArg minMargin, const PxBounds3& hullAABB, const PxTriangleMeshGeometry& shapeMesh,
						const PxTransform& transform0, const PxTransform& transform1,
						PxReal contactDistance, PxContactBuffer& contactBuffer,
						const FastVertex2ShapeScaling& convexScaling, const FastVertex2ShapeScaling& meshScaling,
						bool idtConvexScale, bool idtMeshScale, MultiplePersistentContactManifold& multiManifold,
						PxRenderOutput* renderOutput)
{
	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV contactDist = FLoad(contactDistance);
	//Transfer A into the local space of B
	const PxTransformV convexTransform(p0, q0);//box
	const PxTransformV meshTransform(p1, q1);//triangleMesh  
	const PxTransformV curTransform = meshTransform.transformInv(convexTransform);
	
	if(multiManifold.invalidate(curTransform, minMargin))
	{
		const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));
		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		////////////////////
		const TriangleMesh* PX_RESTRICT meshData = _getMeshData(shapeMesh);

		const Matrix34FromTransform world0(transform0);
		const Matrix34FromTransform world1(transform1);
		BoxPadded hullOBB;
		computeHullOBB(hullOBB, hullAABB, contactDistance, world0, world1, meshScaling, idtMeshScale);

		// Setup the collider

		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE> delayedContacts;
		
		const PxU8* PX_RESTRICT extraData = meshData->getExtraTrigData();
		PCMConvexVsMeshContactGenerationCallback blockCallback(
			contactDist, replaceBreakingThreshold, convexTransform, meshTransform, multiManifold, contactBuffer,
			polyData, polyMap, &delayedContacts, convexScaling, idtConvexScale, meshScaling, extraData, idtMeshScale, true,
			hullOBB, renderOutput);

		Midphase::intersectOBB(meshData, hullOBB, blockCallback, true);

		PX_ASSERT(multiManifold.mNumManifolds <= GU_MAX_MANIFOLD_SIZE);

		blockCallback.flushCache();
		//This is very important
		blockCallback.mGeneration.generateLastContacts();
		blockCallback.mGeneration.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);

#if PCM_LOW_LEVEL_DEBUG
		multiManifold.drawManifold(*renderOutput, transform0, transform1);
#endif
	}
	else
	{
		const PxMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.8f));
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, contactDist);
	}

	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, meshTransform);
}

bool Gu::pcmContactConvexMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxConvexMeshGeometry& shapeConvex = checkedCast<PxConvexMeshGeometry>(shape0);
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);

	const ConvexHullData* hullData = _getHullData(shapeConvex);
	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const PxTransformV convexTransform = loadTransformA(transform0);

	const bool idtScaleMesh = shapeMesh.scale.isIdentity();

	FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(shapeMesh.scale);

	FastVertex2ShapeScaling convexScaling;
	PxBounds3 hullAABB;
	PolygonalData polyData;
	const bool idtScaleConvex = getPCMConvexData(shapeConvex, convexScaling, hullAABB, polyData);
	const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);

	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	
	const PxReal toleranceScale = params.mToleranceLength;
	const FloatV minMargin = CalculatePCMConvexMargin(hullData, vScale, toleranceScale, GU_PCM_MESH_MANIFOLD_EPSILON);
	
	const ConvexHullV convexHull(hullData, V3Zero(), vScale, vQuat, idtScaleConvex);

	if(idtScaleConvex)
	{
		SupportLocalImpl<ConvexHullNoScaleV> convexMap(static_cast<const ConvexHullNoScaleV&>(convexHull), convexTransform, convexHull.vertex2Shape, convexHull.shape2Vertex, true);
		return PCMContactConvexMesh(polyData, &convexMap, minMargin, hullAABB, shapeMesh, transform0, transform1, params.mContactDistance, contactBuffer, convexScaling,  
			meshScaling, idtScaleConvex, idtScaleMesh, multiManifold, renderOutput);
	}
	else
	{
		SupportLocalImpl<ConvexHullV> convexMap(convexHull, convexTransform, convexHull.vertex2Shape, convexHull.shape2Vertex, false);
		return PCMContactConvexMesh(polyData, &convexMap, minMargin, hullAABB, shapeMesh, transform0, transform1, params.mContactDistance, contactBuffer, convexScaling,  
			meshScaling, idtScaleConvex, idtScaleMesh, multiManifold, renderOutput);
	}
}

bool Gu::pcmContactBoxMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const PxBoxGeometry& shapeBox = checkedCast<PxBoxGeometry>(shape0);
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);

	const PxBounds3 hullAABB(-shapeBox.halfExtents, shapeBox.halfExtents);  

	const bool idtMeshScale = shapeMesh.scale.isIdentity();

	FastVertex2ShapeScaling meshScaling;
	if(!idtMeshScale)
		meshScaling.init(shapeMesh.scale);

	FastVertex2ShapeScaling idtScaling;

	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);
	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV minMargin = CalculatePCMBoxMargin(boxExtents, toleranceLength, GU_PCM_MESH_MANIFOLD_EPSILON);

	const BoxV boxV(V3Zero(), boxExtents);

	const PxTransformV boxTransform = loadTransformA(transform0);//box

	PolygonalData polyData;
	PCMPolygonalBox polyBox(shapeBox.halfExtents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity = M33Identity();
	SupportLocalImpl<BoxV> boxMap(boxV, boxTransform, identity, identity, true);

	return PCMContactConvexMesh(polyData, &boxMap, minMargin, hullAABB, shapeMesh, transform0, transform1, params.mContactDistance, contactBuffer, idtScaling,  meshScaling, 
		true, idtMeshScale, multiManifold, renderOutput);
}
