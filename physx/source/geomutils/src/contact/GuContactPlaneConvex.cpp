// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuConvexMeshData.h"
#include "GuContactMethodImpl.h"
#include "GuConvexMesh.h"
#include "CmScaling.h"
#include "CmMatrix34.h"

using namespace physx;
using namespace Cm;

bool Gu::contactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data
	//const PxPlaneGeometry& shapePlane = checkedCast<PxPlaneGeometry>(shape0);
	const PxConvexMeshGeometry& shapeConvex = checkedCast<PxConvexMeshGeometry>(shape1);

	const ConvexHullData* hullData = _getHullData(shapeConvex);
	const PxVec3* PX_RESTRICT hullVertices = hullData->getHullVertices();
	PxU32 numHullVertices = hullData->mNbHullVertices;
//	PxPrefetch128(hullVertices);

	// Plane is implicitly <1,0,0> 0 in localspace
	const Matrix34FromTransform convexToPlane0 (transform0.transformInv(transform1));
	const PxMat33 convexToPlane_rot(convexToPlane0[0], convexToPlane0[1], convexToPlane0[2] );

	bool idtScale = shapeConvex.scale.isIdentity();
	FastVertex2ShapeScaling convexScaling;	// PT: TODO: remove default ctor
	if(!idtScale)
		convexScaling.init(shapeConvex.scale);

	const PxMat34 convexToPlane(convexToPlane_rot * convexScaling.getVertex2ShapeSkew(), convexToPlane0[3]);

	//convexToPlane = context.mVertex2ShapeSkew[1].getVertex2WorldSkew(convexToPlane);

	const Matrix34FromTransform planeToW(transform0);

	// This is rather brute-force
	
	bool status = false;

	const PxVec3 contactNormal = -planeToW.m.column0;

	while(numHullVertices--)
	{
		const PxVec3& vertex = *hullVertices++;
//		if(numHullVertices)
//			PxPrefetch128(hullVertices);

		const PxVec3 pointInPlane = convexToPlane.transform(vertex);		//TODO: this multiply could be factored out!
		if(pointInPlane.x <= params.mContactDistance)
		{
//			const PxVec3 pointInW = planeToW.transform(pointInPlane);
//			contactBuffer.contact(pointInW, -planeToW.m.column0, pointInPlane.x);
			status = true;
			PxContactPoint* PX_RESTRICT pt = contactBuffer.contact();
			if(pt)
			{
				pt->normal				= contactNormal;
				pt->point				= planeToW.transform(pointInPlane);
				pt->separation			= pointInPlane.x;
				pt->internalFaceIndex1	= PXC_CONTACT_NO_FACE_INDEX;
			}
		}
	}
	return status;
}
