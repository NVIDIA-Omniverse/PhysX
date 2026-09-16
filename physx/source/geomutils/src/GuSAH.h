// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SAH_H
#define GU_SAH_H


#include "foundation/PxBounds3.h"
#include "CmRadixSort.h"

namespace physx
{
namespace Gu
{
	struct SAH_Buffers
	{
								SAH_Buffers(PxU32 nb_prims);
								~SAH_Buffers();

		bool					split(PxU32& leftCount, PxU32 nb, const PxU32* PX_RESTRICT prims, const PxBounds3* PX_RESTRICT boxes, const PxVec3* PX_RESTRICT centers);

		Cm::RadixSortBuffered	mSorters[3];
		float*					mKeys;
		float*					mCumulativeLower;
		float*					mCumulativeUpper;
		PxU32					mNb;
	};
}
}

#endif
