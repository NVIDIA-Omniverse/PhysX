// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
