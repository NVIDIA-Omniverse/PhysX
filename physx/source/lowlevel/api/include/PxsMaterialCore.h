// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_MATERIAL_CORE_H
#define PXS_MATERIAL_CORE_H

#include "PxMaterial.h"
#include "foundation/PxUtilities.h"
#include "PxsMaterialShared.h"

namespace physx
{

struct PxsMaterialData 
{
	PxReal			dynamicFriction;
	PxReal			staticFriction;
	PxReal			restitution;
	PxReal			damping;
	PxMaterialFlags	flags;
	PxU8			fricCombineMode;	// PxCombineMode::Enum
	PxU8			restCombineMode;	// PxCombineMode::Enum
	PxU8			dampingCombineMode;	// PxCombineMode::Enum

	PxsMaterialData() :
		dynamicFriction	(0.0f),
		staticFriction	(0.0f),
		restitution		(0.0f),
		damping			(0.0f),
		fricCombineMode	(PxCombineMode::eAVERAGE),
		restCombineMode	(PxCombineMode::eAVERAGE),
		dampingCombineMode(PxCombineMode::eAVERAGE)
	{}

	PxsMaterialData(const PxEMPTY) {}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxCombineMode::Enum getFrictionCombineMode()		const	{ return PxCombineMode::Enum(fricCombineMode);	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxCombineMode::Enum getRestitutionCombineMode()	const	{ return PxCombineMode::Enum(restCombineMode);	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxCombineMode::Enum getDampingCombineMode()		const	{ return PxCombineMode::Enum(dampingCombineMode);	}
	PX_FORCE_INLINE void setFrictionCombineMode(PxCombineMode::Enum combineMode)				{ fricCombineMode = PxTo8(combineMode);			}
	PX_FORCE_INLINE void setRestitutionCombineMode(PxCombineMode::Enum combineMode)				{ restCombineMode = PxTo8(combineMode);			}
	PX_FORCE_INLINE void setDampingCombineMode(PxCombineMode::Enum combineMode)					{ dampingCombineMode = PxTo8(combineMode);		}
};

typedef MaterialCoreT<PxsMaterialData, PxMaterial>	PxsMaterialCore;

} //namespace phyxs

#endif
