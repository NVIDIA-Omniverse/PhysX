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

#include "common/PxPhysXCommonConfig.h"

#include "geometry/PxConvexCoreGeometry.h"
#include "geometry/PxBoxGeometry.h"

#include "GuConvexSupport.h"

using namespace physx;
using namespace Gu;

PX_CUDA_CALLABLE
// generates contacts between a plane and a convex
PxU32 physx::Gu::generateContacts(const PxPlane& plane0, const ConvexShape& convex1, const PxReal contactDist,
	PxVec3& normal, PxVec3 points[MAX_CONVEX_CONTACTS], PxReal dists[MAX_CONVEX_CONTACTS])
{
	normal = -plane0.n;

	const PxVec3 point1 = convex1.support(normal);
	const PxReal dist = plane0.distance(point1);

	PxU32 numContacts = 0;

	if (dist < contactDist)
	{
		PxVec3 faceNormal, facePoints[Gu::ConvexCore::MAX_FACE_POINTS];
		const PxU32 numPoints = convex1.contactFace(normal, point1, faceNormal, facePoints);
				
		if (numPoints == 0)
		{
			const PxVec3 point = point1 + normal * dist * 0.5f;
			points[numContacts] = point;
			dists[numContacts] = dist;
			++numContacts;
		}

		for (PxU32 i = 0; i < numPoints; ++i)
		{
			const PxVec3 p1 = facePoints[i];
			const PxReal d = plane0.distance(p1);
			points[numContacts] = p1 + normal * d * 0.5f;
			dists[numContacts] = d;
			++numContacts;
		}
	}

	return numContacts;
}
