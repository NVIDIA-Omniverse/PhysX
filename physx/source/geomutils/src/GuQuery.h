// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_QUERY_H
#define GU_QUERY_H

#include "GuBounds.h"
#include "GuBVHTestsSIMD.h"

namespace physx
{
namespace Gu
{
	// PT: TODO: the various V3LoadUs in the base tests like SphereAABBTest could be avoided

	// PT: TODO: check inflation is consistent in all of these. Looks like it's not.

	struct DefaultOBBAABBTest : OBBAABBTest
	{
		PX_FORCE_INLINE DefaultOBBAABBTest(const ShapeData& queryVolume) :
			OBBAABBTest(queryVolume.getPrunerWorldPos(),
						queryVolume.getPrunerWorldRot33(),
						queryVolume.getPrunerBoxGeomExtentsInflated())	{}
	};

	struct DefaultAABBAABBTest : AABBAABBTest
	{
		PX_FORCE_INLINE DefaultAABBAABBTest(const ShapeData& queryVolume) :
			AABBAABBTest(queryVolume.getPrunerInflatedWorldAABB())	{}
	};

	struct DefaultSphereAABBTest : SphereAABBTest
	{
		PX_FORCE_INLINE DefaultSphereAABBTest(const ShapeData& queryVolume) :
			SphereAABBTest(	queryVolume.getGuSphere().center,
							queryVolume.getGuSphere().radius)	{}
	};

	struct DefaultCapsuleAABBTest : CapsuleAABBTest
	{
		PX_FORCE_INLINE DefaultCapsuleAABBTest(const ShapeData& queryVolume, float inflation) :
			CapsuleAABBTest(queryVolume.getGuCapsule().p1,
							queryVolume.getPrunerWorldRot33().column0,
							queryVolume.getCapsuleHalfHeight()*2.0f,
							PxVec3(queryVolume.getGuCapsule().radius*inflation))	{}
	};

}
}

#endif
