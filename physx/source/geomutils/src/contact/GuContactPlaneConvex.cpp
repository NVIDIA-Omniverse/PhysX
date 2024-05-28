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
