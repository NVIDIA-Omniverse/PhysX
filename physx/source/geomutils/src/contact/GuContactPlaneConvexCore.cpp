// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "geometry/PxConvexCoreGeometry.h"
#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"

using namespace physx;

bool Gu::contactPlaneConvexCore(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	PxPlane plane0(transform0.p, transform0.q.getBasisVector0());
	Gu::ConvexShape convex1; Gu::makeConvexShape(shape1, transform1, convex1);
	PX_ASSERT(convex1.isValid());

	PxVec3 normal, points[Gu::MAX_CONVEX_CONTACTS];
	PxReal dists[Gu::MAX_CONVEX_CONTACTS];
	if (PxU32 count = Gu::generateContacts(plane0, convex1, params.mContactDistance, normal, points, dists))
		for (PxU32 i = 0; i < count; ++i)
			contactBuffer.contact(points[i], normal, dists[i]);

	return contactBuffer.count > 0;
}
