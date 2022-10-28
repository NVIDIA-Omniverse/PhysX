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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_MATERIAL_COMBINER_H
#define PXS_MATERIAL_COMBINER_H

#include "PxsMaterialCore.h"

namespace physx
{
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal combineScalars(PxReal a, PxReal b, PxI32 combineMode)
	{
		switch (combineMode)
		{
			case PxCombineMode::eAVERAGE:
				return 0.5f * (a + b);
			case PxCombineMode::eMIN:
				return PxMin(a,b);
			case PxCombineMode::eMULTIPLY:
				return a * b;
			case PxCombineMode::eMAX:
				return PxMax(a,b);
			default:
				return PxReal(0);
		}   
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal PxsCombineRestitution(const PxsMaterialData& mat0, const PxsMaterialData& mat1)
	{
		return combineScalars(mat0.restitution, mat1.restitution, ((mat0.flags | mat1.flags) & PxMaterialFlag::eCOMPLIANT_CONTACT) ? 
			PxCombineMode::eMIN : PxMax(mat0.getRestitutionCombineMode(), mat1.getRestitutionCombineMode()));
	}

	//ML:: move this function to header file to avoid LHS in Xbox
	//PT: also called by CUDA code now
	PX_CUDA_CALLABLE PX_FORCE_INLINE void PxsCombineIsotropicFriction(const PxsMaterialData& mat0, const PxsMaterialData& mat1, PxReal& dynamicFriction, PxReal& staticFriction, PxU32& flags)
	{
		const PxU32 combineFlags = (mat0.flags | mat1.flags); //& (PxMaterialFlag::eDISABLE_STRONG_FRICTION|PxMaterialFlag::eDISABLE_FRICTION);	//eventually set DisStrongFric flag, lower all others.

		if (!(combineFlags & PxMaterialFlag::eDISABLE_FRICTION))
		{
			const PxI32 fictionCombineMode = PxMax(mat0.getFrictionCombineMode(), mat1.getFrictionCombineMode());
			PxReal dynFriction = 0.0f;
			PxReal staFriction = 0.0f;

			switch (fictionCombineMode)
			{
			case PxCombineMode::eAVERAGE:
				dynFriction = 0.5f * (mat0.dynamicFriction + mat1.dynamicFriction);
				staFriction = 0.5f * (mat0.staticFriction + mat1.staticFriction);
				break;
			case PxCombineMode::eMIN:
				dynFriction = PxMin(mat0.dynamicFriction, mat1.dynamicFriction);
				staFriction = PxMin(mat0.staticFriction, mat1.staticFriction);
				break;
			case PxCombineMode::eMULTIPLY:
				dynFriction = (mat0.dynamicFriction * mat1.dynamicFriction);
				staFriction = (mat0.staticFriction * mat1.staticFriction);
				break;
			case PxCombineMode::eMAX:
				dynFriction = PxMax(mat0.dynamicFriction, mat1.dynamicFriction);
				staFriction = PxMax(mat0.staticFriction, mat1.staticFriction);
				break;
			}   

			//isotropic case
			const PxReal fDynFriction = PxMax(dynFriction, 0.0f);

			// PT: TODO: the two branches aren't actually doing the same thing:
			// - one is ">", the other is ">="
			// - one uses a clamped dynFriction, the other not
#ifdef __CUDACC__
			const PxReal fStaFriction = (staFriction - fDynFriction) > 0 ? staFriction : dynFriction;
#else
			const PxReal fStaFriction = physx::intrinsics::fsel(staFriction - fDynFriction, staFriction, fDynFriction);
#endif
			dynamicFriction = fDynFriction;
			staticFriction = fStaFriction;
			flags = combineFlags;
		}
		else
		{
			flags = combineFlags | PxMaterialFlag::eDISABLE_STRONG_FRICTION;
			dynamicFriction = 0.0f;
			staticFriction = 0.0f;
		}
	}
}

#endif
