// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "foundation/PxBounds3.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxFoundation.h"
#include "extensions/PxBroadPhaseExt.h"

using namespace physx;

PxU32 PxBroadPhaseExt::createRegionsFromWorldBounds(PxBounds3* regions, const PxBounds3& globalBounds, PxU32 nbSubdiv, PxU32 upAxis)
{
	PX_CHECK_MSG(globalBounds.isValid(), "PxBroadPhaseExt::createRegionsFromWorldBounds(): invalid bounds provided!");
	PX_CHECK_MSG(upAxis<3, "PxBroadPhaseExt::createRegionsFromWorldBounds(): invalid up-axis provided!");

	const PxVec3& min = globalBounds.minimum;
	const PxVec3& max = globalBounds.maximum;
	const float dx = (max.x - min.x) / float(nbSubdiv);
	const float dy = (max.y - min.y) / float(nbSubdiv);
	const float dz = (max.z - min.z) / float(nbSubdiv);
	PxU32 nbRegions = 0;
	PxVec3 currentMin, currentMax;
	for(PxU32 j=0;j<nbSubdiv;j++)
	{
		for(PxU32 i=0;i<nbSubdiv;i++)
		{
			if(upAxis==0)
			{
				currentMin = PxVec3(min.x, min.y + dy * float(i),   min.z + dz * float(j));
				currentMax = PxVec3(max.x, min.y + dy * float(i+1), min.z + dz * float(j+1));
			}
			else if(upAxis==1)
			{
				currentMin = PxVec3(min.x + dx * float(i),   min.y, min.z + dz * float(j));
				currentMax = PxVec3(min.x + dx * float(i+1), max.y, min.z + dz * float(j+1));
			}
			else if(upAxis==2)
			{
				currentMin = PxVec3(min.x + dx * float(i),   min.y + dy * float(j), min.z);
				currentMax = PxVec3(min.x + dx * float(i+1), min.y + dy * float(j+1), max.z);
			}

			regions[nbRegions++] = PxBounds3(currentMin, currentMax);
		}
	}
	return nbRegions;
}
