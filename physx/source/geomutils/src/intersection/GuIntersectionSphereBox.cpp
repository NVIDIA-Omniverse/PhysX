// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuIntersectionSphereBox.h"
#include "GuSphere.h"
#include "GuBox.h"

using namespace physx;

bool Gu::intersectSphereBox(const Sphere& sphere, const Box& box)
{
	const PxVec3 delta = sphere.center - box.center;
	PxVec3 dRot = box.rot.transformTranspose(delta);	//transform delta into OBB body coords. (use method call!)

	//check if delta is outside AABB - and clip the vector to the AABB.
	bool outside = false;

	if(dRot.x < -box.extents.x)
	{ 
		outside = true; 
		dRot.x = -box.extents.x;
	}
	else if(dRot.x >  box.extents.x)
	{ 
		outside = true; 
		dRot.x = box.extents.x;
	}

	if(dRot.y < -box.extents.y)
	{ 
		outside = true; 
		dRot.y = -box.extents.y;
	}
	else if(dRot.y >  box.extents.y)
	{ 
		outside = true; 
		dRot.y = box.extents.y;
	}

	if(dRot.z < -box.extents.z)
	{ 
		outside = true; 
		dRot.z = -box.extents.z;
	}
	else if(dRot.z >  box.extents.z)
	{ 
		outside = true; 
		dRot.z = box.extents.z;
	}

	if(outside)	//if clipping was done, sphere center is outside of box.
	{
		const PxVec3 clippedDelta = box.rot.transform(dRot);	//get clipped delta back in world coords.

		const PxVec3 clippedVec = delta - clippedDelta;			  //what we clipped away.	
		const PxReal lenSquared = clippedVec.magnitudeSquared();
		const PxReal radius = sphere.radius;
		if(lenSquared > radius * radius)	// PT: objects are defined as closed, so we return 'true' in case of equality
			return false;	//disjoint
	}
	return true;
}
