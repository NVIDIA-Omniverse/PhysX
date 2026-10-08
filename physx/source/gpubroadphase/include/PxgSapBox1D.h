// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SAPBOX_1D_H
#define PXG_SAPBOX_1D_H

#include "PxgBroadPhaseCommonDefines.h"

namespace physx
{

class PxgSapBox1D
{
public:

	PX_CUDA_CALLABLE PX_FORCE_INLINE					PxgSapBox1D()	{ mMinMax[0] = PXG_INVALID_BP_HANDLE;  mMinMax[1]=PXG_INVALID_BP_HANDLE; }
	PX_CUDA_CALLABLE PX_FORCE_INLINE					~PxgSapBox1D()	{}

	PX_CUDA_CALLABLE bool	validHandle() const
	{ 
		return (mMinMax[0] != PXG_INVALID_BP_HANDLE) && (mMinMax[1]!=PXG_INVALID_BP_HANDLE); 
	}

	PxU32							mMinMax[2];//mMinMax[0]=min, mMinMax[1]=max
};



}

#endif