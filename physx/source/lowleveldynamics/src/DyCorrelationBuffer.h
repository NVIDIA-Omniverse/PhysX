// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_CORRELATION_BUFFER_H
#define DY_CORRELATION_BUFFER_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxBounds3.h"
#include "geomutils/PxContactBuffer.h"

#include "PxPhysXConfig.h"
#include "DyFrictionPatch.h"

namespace physx
{
namespace Dy
{
struct CorrelationBuffer
{
	static const PxU32 MAX_FRICTION_PATCHES = 32;
	static const PxU16 LIST_END = 0xffff;

	struct ContactPatchData
	{
		PxBounds3	patchBounds;
		PxU32		boundsPadding;

		PxReal		staticFriction;
		PxReal		dynamicFriction;
		PxReal		restitution;
		PxU16		start;
		PxU16		next;
		PxU8		flags;
		PxU8		count;
	};

	// we can have as many contact patches as contacts, unfortunately
	ContactPatchData	PX_ALIGN(16, contactPatches[PxContactBuffer::MAX_CONTACTS]);

	FrictionPatch		PX_ALIGN(16, frictionPatches[MAX_FRICTION_PATCHES]);
	PxVec3				PX_ALIGN(16, frictionPatchWorldNormal[MAX_FRICTION_PATCHES]);
	PxBounds3			patchBounds[MAX_FRICTION_PATCHES];

	PxU32				frictionPatchContactCounts[MAX_FRICTION_PATCHES];
	PxU32				correlationListHeads[MAX_FRICTION_PATCHES+1];

	// contact IDs are only used to identify auxiliary contact data when velocity
	// targets have been set. 
	PxU16				contactID[MAX_FRICTION_PATCHES][2];

	PxU32				contactPatchCount, frictionPatchCount;
};

bool createContactPatches(CorrelationBuffer& fb, const PxContactPoint* cb, PxU32 contactCount, PxReal normalTolerance);

bool correlatePatches(CorrelationBuffer& fb, 
					  const PxContactPoint* cb,
					  const PxTransform& bodyFrame0,
					  const PxTransform& bodyFrame1,
					  PxReal normalTolerance,
					  PxU32 startContactPatchIndex,
					  PxU32 startFrictionPatchIndex);

void growPatches(CorrelationBuffer& fb,
				 const PxContactPoint* buffer,
				 const PxTransform& bodyFrame0,
				 const PxTransform& bodyFrame1,
				 PxU32 frictionPatchStartIndex,
				 PxReal frictionOffsetThreshold);

// Removes empty friction patches (correlationListHeads == LIST_END) in [startFrictionPatchIndex, frictionPatchCount),
// compacting ALL per-patch arrays together so they stay index-aligned for downstream consumers.
void removeEmptyFrictionPatches(CorrelationBuffer& fb, PxU32 startFrictionPatchIndex);

}

}

#endif
