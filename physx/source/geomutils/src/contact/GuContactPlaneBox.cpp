// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxUnionCast.h"
#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "CmMatrix34.h"
#include "foundation/PxUtilities.h"

using namespace physx;
using namespace Cm;

bool Gu::contactPlaneBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data      
	//const PxPlaneGeometry& shapePlane = checkedCast<PxPlaneGeometry>(shape0);
	const PxBoxGeometry& shapeBox = checkedCast<PxBoxGeometry>(shape1);
	
	const PxVec3 negPlaneNormal = -transform0.q.getBasisVector0();
	
	const Matrix34FromTransform boxMatrix(transform1);
	const Matrix34FromTransform boxToPlane(transform0.transformInv(transform1));

	PX_ASSERT(contactBuffer.count==0);

	const PxReal contactDistance = params.mContactDistance;
	const PxReal limit = contactDistance - boxToPlane.p.x;
	const PxReal dx = shapeBox.halfExtents.x;
	const PxReal dy = shapeBox.halfExtents.y;
	const PxReal dz = shapeBox.halfExtents.z;
	const PxReal bxdx = boxToPlane.m.column0.x * dx;
	const PxReal bxdy = boxToPlane.m.column1.x * dy;
	const PxReal bxdz = boxToPlane.m.column2.x * dz;

	PxReal depths[8];
	depths[0] =   bxdx + bxdy + bxdz - limit;
	depths[1] =   bxdx + bxdy - bxdz - limit;
	depths[2] =   bxdx - bxdy + bxdz - limit;
	depths[3] =   bxdx - bxdy - bxdz - limit;
	depths[4] = - bxdx + bxdy + bxdz - limit;
	depths[5] = - bxdx + bxdy - bxdz - limit;
	depths[6] = - bxdx - bxdy + bxdz - limit;
	depths[7] = - bxdx - bxdy - bxdz - limit;

	const PxVec3& column0 = boxMatrix.m.column0;
	const PxVec3& column1 = boxMatrix.m.column1;
	const PxVec3& column2 = boxMatrix.m.column2;
	const PxVec3& p = boxMatrix.p;

	const PxVec3 x = column0 * dx;
	const PxVec3 y = column1 * dy;
	const PxVec3 z = column2 * dz;

	if(depths[0] < 0.0f)
		contactBuffer.contact(x + y + z + p, negPlaneNormal, depths[0] + contactDistance);
	if(depths[1] < 0.0f)
		contactBuffer.contact(x + y - z + p, negPlaneNormal, depths[1] + contactDistance);
	if(depths[2] < 0.0f)
		contactBuffer.contact(x - y + z + p, negPlaneNormal, depths[2] + contactDistance);
	if(depths[3] < 0.0f)
		contactBuffer.contact(x - y - z + p, negPlaneNormal, depths[3] + contactDistance);
	if(depths[4] < 0.0f)
		contactBuffer.contact(- x + y + z + p, negPlaneNormal, depths[4] + contactDistance);
	if(depths[5] < 0.0f)
		contactBuffer.contact(- x + y - z + p, negPlaneNormal, depths[5] + contactDistance);
	if(depths[6] < 0.0f)
		contactBuffer.contact(- x - y + z + p, negPlaneNormal, depths[6] + contactDistance);
	if(depths[7] < 0.0f)
		contactBuffer.contact(- x - y - z + p, negPlaneNormal, depths[7] + contactDistance);

	return contactBuffer.count > 0;
}
