// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PRUNER_PAYLOAD_H
#define GU_PRUNER_PAYLOAD_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"

namespace physx
{
	class PxBounds3;

namespace Gu
{
	// PT: anonymous payload structure used by the pruners. This is similar in spirit to a userData pointer.
	struct PrunerPayload
	{
		size_t data[2];	// Enough space for two arbitrary pointers

		PX_FORCE_INLINE	bool operator == (const PrunerPayload& other) const
		{
			return (data[0] == other.data[0]) && (data[1] == other.data[1]);
		}
	};

	// PT: pointers to internal data associated with a pruner payload. The lifetime of these pointers
	// is usually limited and they should be used immediately after retrieval.
	struct PrunerPayloadData
	{
		PxBounds3*		mBounds;	// Pointer to internal bounds.
		PxTransform*	mTransform;	// Pointer to internal transform, or NULL.
	};

	// PT: called for each removed payload. Gives users a chance to cleanup their data
	// structures without duplicating the pruner-data to payload mapping on their side.
	struct PrunerPayloadRemovalCallback
	{
						PrunerPayloadRemovalCallback()		{}
		virtual			~PrunerPayloadRemovalCallback()		{}

		virtual void	invoke(PxU32 nbRemoved, const PrunerPayload* removed) = 0;
	};
}
}

#endif
