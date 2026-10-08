// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_HEAP_STATS_H
#define PXS_HEAP_STATS_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PxsHeapStats
	{
		enum Enum
		{
			eOTHER = 0,
			eBROADPHASE,
			eNARROWPHASE,
			eSOLVER,
			eARTICULATION,
			eSIMULATION,
			eSIMULATION_ARTICULATION,
			eSIMULATION_PARTICLES,
			eSIMULATION_SOFTBODY,
			eSIMULATION_FEMCLOTH,
			eSHARED_PARTICLES,
			eSHARED_SOFTBODY,
			eSHARED_FEMCLOTH,
			eHEAPSTATS_COUNT
		};

		PxU64 stats[eHEAPSTATS_COUNT];

		PxsHeapStats()
		{
			for (PxU32 i = 0; i < eHEAPSTATS_COUNT; i++)
			{
				stats[i] = 0;
			}
		}
	};
}

#endif
