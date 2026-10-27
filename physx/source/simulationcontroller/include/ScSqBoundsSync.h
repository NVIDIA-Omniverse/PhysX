// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_SQ_BOUNDS_SYNC_H
#define SC_SQ_BOUNDS_SYNC_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxBitMap.h"

#include "PxSceneQuerySystem.h"

namespace physx
{
	class PxBounds3;
	class PxRigidBody;
	class PxShape;

	typedef PxSQPrunerHandle	ScPrunerHandle;

namespace Sc
{
	// PT: TODO: revisit the need for a virtual interface
	struct SqRefFinder
	{
		virtual ScPrunerHandle find(const PxRigidBody* body, const PxShape* shape, PxU32& prunerIndex) = 0;

		virtual ~SqRefFinder() {}
	};

	// PT: TODO: revisit the need for a virtual interface
	struct SqBoundsSync
	{
		virtual void sync(PxU32 prunerIndex, const ScPrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices) = 0;

		virtual ~SqBoundsSync() {}
	};
}
}

#endif
