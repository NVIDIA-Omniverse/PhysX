// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_CONVEX_CONVEX_SHAPE_H
#define PXG_CONVEX_CONVEX_SHAPE_H

#include "geometry/PxMeshScale.h"

#define PXG_MAX_PCM_CONTACTS 4

namespace physx
{
	template<PxU32 byteSize> 
	class FlexiblePad
	{
		PxU8							pad[byteSize];
	};

	template<> 
	class FlexiblePad<0>
	{
	};

	//ML: PxgShape don't need to have contactOffset because we will dma a separated contact offset array later
	struct PX_ALIGN_PREFIX(16) PxgShape
	{
		PxMeshScale						scale;									//28
		PxU32							materialIndex;							//32
		size_t							hullOrMeshPtr;							//36 or 40

		PxU32							type;									//40 or 44
		PxU32							particleOrSoftbodyId;					//44 or 48		

		//FlexiblePad<16 - sizeof(size_t) - sizeof(PxU32)>	pad;			//48
#if !PX_P64_FAMILY
		PxU32    pad0;
#endif
	}
	PX_ALIGN_SUFFIX(16);

	PX_COMPILE_TIME_ASSERT(sizeof(PxgShape) == 48);
}

#endif
