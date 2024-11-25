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
#include "GuGJKPenetration.h"
#include "GuEPA.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuContactMethodImpl.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactGen.h"
#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"
#include "GuRefGjkEpa.h"

using namespace physx;
using namespace Gu;
using namespace aos;

bool Gu::contactConvexCoreConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	const PxVec3 shift = (transform0.p + transform1.p) * 0.5f;
	const PxTransform pose0(transform0.p - shift, transform0.q);
	const PxTransform pose1(transform1.p - shift, transform1.q);
	const PxReal contactDist = params.mContactDistance;

	ConvexShape convex0; Gu::makeConvexShape(shape0, pose0, convex0);
	ConvexShape convex1; Gu::makeConvexShape(shape1, pose1, convex1);
	PX_ASSERT(convex0.isValid() && convex1.isValid());

	PxVec3 normal, points[Gu::MAX_CONVEX_CONTACTS];
	PxReal dists[Gu::MAX_CONVEX_CONTACTS];
	if (PxU32 count = Gu::generateContacts(convex0, convex1, contactDist, normal, points, dists))
		for (PxU32 i = 0; i < count; ++i)
			contactBuffer.contact(points[i] + shift, normal, dists[i]);

	return contactBuffer.count > 0;
}
