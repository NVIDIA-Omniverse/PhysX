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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "CmMatrix34.h"
#include "foundation/PxVec3.h"
#include "CmScaling.h"
#include "foundation/PxMat34.h"
#include "foundation/PxArray.h"
#include "foundation/PxAssert.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxSimpleTypes.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxTriangleMesh.h"
#include "geomutils/PxContactBuffer.h"
#include "GuTriangle.h"
#include "geomutils/PxContactPoint.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxTransform.h"
#include "GuContactMethodImpl.h"
#include "GuContactReduction.h"
#include "GuMidphaseInterface.h"
#include "PxContact.h"

using namespace physx;
using namespace Cm;

using BufferedContactPatch = Gu::BufferedPatch<6, 64>;
using ReducedContactPatch = Gu::TinyContactPatch<6>;

bool Gu::contactPlaneMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);

	// Plane is implicitly <1,0,0> 0 in localspace
	const PxVec3* PX_RESTRICT vertices = shapeMesh.triangleMesh->getVertices();
	const PxTransform meshToPlane0Trafo = transform0.transformInv(transform1);
	const Matrix34FromTransform meshToPlane0 (meshToPlane0Trafo);
	const PxMat33 meshToPlane_rot(meshToPlane0[0], meshToPlane0[1], meshToPlane0[2] );

	const bool idtScale = shapeMesh.scale.isIdentity();
	FastVertex2ShapeScaling meshScaling;
	if(!idtScale)
		meshScaling.init(shapeMesh.scale);

	const PxMat34 meshToPlane(meshToPlane_rot * meshScaling.getVertex2ShapeSkew(), meshToPlane0[3]);

	//convexToPlane = context.mVertex2ShapeSkew[1].getVertex2WorldSkew(convexToPlane);

	const Matrix34FromTransform planeToW(transform0);
	const PxVec3 contactNormal = -planeToW.m.column0;

	BufferedContactPatch patch;
	patch.mRootNormal = contactNormal;

	
	const PxMeshScale& sdfScale = shapeMesh.scale;
	const PxMat33 sdfScaleMat = sdfScale.toMat33();

	PxBounds3 meshBoundsAtWorldScale = shapeMesh.triangleMesh->getLocalBounds();
	meshBoundsAtWorldScale.minimum = sdfScaleMat.transform(meshBoundsAtWorldScale.minimum);
	meshBoundsAtWorldScale.maximum = sdfScaleMat.transform(meshBoundsAtWorldScale.maximum);

	const PxReal radius = 0.5f*(meshBoundsAtWorldScale.maximum - meshBoundsAtWorldScale.minimum).magnitude();
	PxVec3 transformedBoundsCenter = meshToPlane0Trafo.transform(meshBoundsAtWorldScale.getCenter());
	transformedBoundsCenter.x = 0.0f; //In local plane space, this corresponds to a projection of a point onto the plane
	const PxBoxGeometry planeBoxMesh(params.mContactDistance, radius, radius);
	
	const PxU32 MAX_INTERSECTIONS = 1024 * 32;
	PxArray<PxU32> overlappingTriangles;
	overlappingTriangles.resize(MAX_INTERSECTIONS); //TODO: Not ideal, dynamic allocation for every function call
	//PxU32 overlappingTriangles[MAX_INTERSECTIONS]; //TODO: Is this too much memory to allocate on the stack?

	bool overflow = false;
	const PxU32 overlapCount = PxMeshQuery::findOverlapTriangleMesh(planeBoxMesh, PxTransform(transformedBoundsCenter), shapeMesh, meshToPlane0Trafo, overlappingTriangles.begin(), MAX_INTERSECTIONS, 0, overflow);
	PX_ASSERT(!overflow);

	const bool has16BitIndices = shapeMesh.triangleMesh->getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
	const void* PX_RESTRICT tris = shapeMesh.triangleMesh->getTriangles();

	PxBitMap bitmap;
	bitmap.resize(shapeMesh.triangleMesh->getNbVertices(), false); //TODO: Not ideal, dynamic allocation for every function call

	// FIXME: Make use of bvh to reduce number of checks here
	bool contact = false;
	for (PxU32 i = 0; i < overlapCount; ++i)
	{
		const PxU32 triIdx = overlappingTriangles[i];
		Gu::IndexedTriangle32 triIndices;		
		getVertexRefs(triIdx, triIndices.mRef[0], triIndices.mRef[1], triIndices.mRef[2], tris, has16BitIndices);

		for (PxU32 j = 0; j < 3; ++j) 
		{
			const PxU32 vertexIndex = triIndices[j];
			if (bitmap.test(vertexIndex))
				continue;
			bitmap.set(vertexIndex);

			const PxVec3& vertex = vertices[vertexIndex];
			const PxVec3 pointInPlane = meshToPlane.transform(vertex);		//TODO: this multiply could be factored out!
			if (pointInPlane.x <= params.mContactDistance)
			{
				contact = true;
				patch.addContact(TinyContact(contactNormal, pointInPlane.x, planeToW.transform(pointInPlane)));
			}
		}
	}

	if (contact)
	{
		ReducedContactPatch reduced;
		patch.asTinyContactPatch(reduced);
		for (const TinyContact& tiny: reduced)
		{
			PxContactPoint* PX_RESTRICT pt = contactBuffer.contact();
			if(pt != NULL)
			{
				pt->normal				= contactNormal;
				pt->point				= tiny.mPoint;
				pt->separation			= tiny.mSeparation;
				pt->internalFaceIndex1	= PXC_CONTACT_NO_FACE_INDEX;
			}
		}
	}

	return contact;
}
