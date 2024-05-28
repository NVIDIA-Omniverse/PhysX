// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
		flags			(PxMaterialFlag::eIMPROVED_PATCH_FRICTION),
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
