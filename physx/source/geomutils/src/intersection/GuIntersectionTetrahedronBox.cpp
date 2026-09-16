// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuIntersectionTetrahedronBox.h"
#include "foundation/PxBasicTemplates.h"
#include "GuIntersectionTriangleBox.h"
#include "GuBox.h"

using namespace physx;

namespace physx
{
namespace Gu
{
	bool intersectTetrahedronBox(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxBounds3& box)
	{
		if (box.contains(a) || box.contains(b) || box.contains(c) || box.contains(d))
			return true;

		PxBounds3 tetBox = PxBounds3::empty();
		tetBox.include(a);
		tetBox.include(b);
		tetBox.include(c);
		tetBox.include(d);
		tetBox.fattenFast(1e-6f);

		if (!box.intersects(tetBox))
			return false;

		Gu::BoxPadded boxP;
		boxP.center = box.getCenter();
		boxP.extents = box.getExtents();
		boxP.rot = PxMat33(PxIdentity);
		return intersectTriangleBox(boxP, a, b, c) || intersectTriangleBox(boxP, a, b, d) || intersectTriangleBox(boxP, a, c, d) || intersectTriangleBox(boxP, b, c, d);
	}
}
}
