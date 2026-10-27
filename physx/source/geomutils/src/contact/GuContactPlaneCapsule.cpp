// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "GuInternal.h"
#include "GuSegment.h"

using namespace physx;

bool Gu::contactPlaneCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data
	//const PxPlaneGeometry& shapePlane = checkedCast<PxPlaneGeometry>(shape0);
	const PxCapsuleGeometry& shapeCapsule = checkedCast<PxCapsuleGeometry>(shape1);

	const PxTransform capsuleToPlane = transform0.transformInv(transform1);

	//Capsule in plane space
	Segment segment;
	getCapsuleSegment(capsuleToPlane, shapeCapsule, segment);
	
	const PxVec3 negPlaneNormal = transform0.q.getBasisVector0();
	
	bool contact = false;

	const PxReal separation0 = segment.p0.x - shapeCapsule.radius;
	const PxReal separation1 = segment.p1.x - shapeCapsule.radius;
	if(separation0 <= params.mContactDistance)
	{
		const PxVec3 temp(segment.p0.x - shapeCapsule.radius, segment.p0.y, segment.p0.z);
		const PxVec3 point = transform0.transform(temp);
		contactBuffer.contact(point, -negPlaneNormal, separation0);
		contact = true;
	}

	if(separation1 <= params.mContactDistance)
	{
		const PxVec3 temp(segment.p1.x - shapeCapsule.radius, segment.p1.y, segment.p1.z);
		const PxVec3 point = transform0.transform(temp);
		contactBuffer.contact(point, -negPlaneNormal, separation1);
		contact = true;
	}
	return contact;
}
