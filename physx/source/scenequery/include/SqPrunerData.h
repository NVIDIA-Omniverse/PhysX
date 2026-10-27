// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_PRUNER_DATA_H
#define SQ_PRUNER_DATA_H


#include "SqTypedef.h"

// PT: SQ-API LEVEL 2 (Level 1 = SqPruner.h)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

namespace physx
{
namespace Sq
{
	struct PruningIndex
	{
		enum Enum
		{
			eSTATIC		= 0,	// PT: must match PX_SCENE_PRUNER_STATIC
			eDYNAMIC	= 1,	// PT: must match PX_SCENE_PRUNER_DYNAMIC

			eCOUNT		= 2
		};
	};

	PX_FORCE_INLINE PrunerData createPrunerData(PxU32 index, Gu::PrunerHandle h)	{ return PrunerData((h << 1) | index);	}
	PX_FORCE_INLINE PxU32 getPrunerIndex(PrunerData data)							{ return PxU32(data & 1);				}
	PX_FORCE_INLINE Gu::PrunerHandle getPrunerHandle(PrunerData data)				{ return Gu::PrunerHandle(data >> 1);	}
}
}

#endif
