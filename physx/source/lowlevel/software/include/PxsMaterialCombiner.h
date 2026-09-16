// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
				return PxMin(a, b);
			case PxCombineMode::eMULTIPLY:
				return a * b;
			case PxCombineMode::eMAX:
				return PxMax(a, b);
			default:
				return 0.0f;
		}
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void PxsCombineMaterials(const PxsMaterialData& mat0Data, const PxsMaterialData& mat1Data,
		PxReal& combinedStaticFriction, PxReal& combinedDynamicFriction, 
		PxReal& combinedRestitution, PxU32& combinedMaterialFlags, PxReal& combinedDamping)
	{
		const PxReal r0 = mat0Data.restitution;
		const PxReal r1 = mat1Data.restitution;
		const bool compliant0 = r0 < 0.0f;
		const bool compliant1 = r1 < 0.0f;
		const bool exactlyOneCompliant = compliant0 ^ compliant1;
		const bool bothCompliant = compliant0 & compliant1;
		const bool compliantAcc0 = !!(mat0Data.flags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING);
		const bool compliantAcc1 = !!(mat1Data.flags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING);
		const bool exactlyOneAccCompliant = compliantAcc0 ^ compliantAcc1;

		// combine restitution
		{
			// For rigid-rigid or compliant-compliant interactions, follow the user's choice of combine mode but make sure it stays negative for multiply.
			// For rigid-compliant interactions, we go with the compliant behavior.
			// For forceCompliant-accelerationCompliant, we go with the accelerationCompliant behavior

			if (bothCompliant && exactlyOneAccCompliant)
			{
				combinedRestitution = compliantAcc0 ? r0 : r1;
			}
			else
			{
				const PxCombineMode::Enum combineMode =
					exactlyOneCompliant ? PxCombineMode::eMIN
										: PxMax(mat0Data.getRestitutionCombineMode(), mat1Data.getRestitutionCombineMode());
				const PxReal flipSign = (bothCompliant && (combineMode == PxCombineMode::eMULTIPLY)) ? -1.0f : 1.0f;
				combinedRestitution = flipSign * combineScalars(r0, r1, combineMode);
			 }
		}

		// combine damping
		{
			// For rigid-rigid or compliant-compliant interactions, follow the user's choice of combine mode.
			// For rigid-compliant interactions, we go with the compliant behavior.
			// For forceCompliant-accelerationCompliant, we go with the accelerationCompliant behavior

			const PxReal d0 = mat0Data.damping;
			const PxReal d1 = mat1Data.damping;

			if (bothCompliant && exactlyOneAccCompliant)
			{
				combinedDamping = compliantAcc0 ? d0 : d1;
			}
			else
			{
				const PxCombineMode::Enum combineMode =
					exactlyOneCompliant ? PxCombineMode::eMAX
										: PxMax(mat0Data.getDampingCombineMode(), mat1Data.getDampingCombineMode());
				combinedDamping = combineScalars(d0, d1, combineMode);
			}
		}

		// combine isotropic friction
		{
			const PxU32 combineFlags = (mat0Data.flags | mat1Data.flags); //& (PxMaterialFlag::eDISABLE_STRONG_FRICTION|PxMaterialFlag::eDISABLE_FRICTION);	//eventually set DisStrongFric flag, lower all others.

			if (!(combineFlags & PxMaterialFlag::eDISABLE_FRICTION))
			{
				const PxI32 fictionCombineMode = PxMax(mat0Data.getFrictionCombineMode(), mat1Data.getFrictionCombineMode());
				PxReal dynFriction = 0.0f;
				PxReal staFriction = 0.0f;

				dynFriction = combineScalars(mat0Data.dynamicFriction, mat1Data.dynamicFriction, fictionCombineMode);
				staFriction = combineScalars(mat0Data.staticFriction, mat1Data.staticFriction, fictionCombineMode);

				//isotropic case
				const PxReal fDynFriction = PxMax(dynFriction, 0.0f);

#if PX_CUDA_COMPILER
				const PxReal fStaFriction = (staFriction - fDynFriction) >= 0 ? staFriction : fDynFriction;
#else
				const PxReal fStaFriction = physx::intrinsics::fsel(staFriction - fDynFriction, staFriction, fDynFriction);
#endif
				combinedDynamicFriction = fDynFriction;
				combinedStaticFriction = fStaFriction;
				combinedMaterialFlags = combineFlags;
			}
			else
			{
				combinedMaterialFlags = combineFlags | PxMaterialFlag::eDISABLE_STRONG_FRICTION;
				combinedDynamicFriction = 0.0f;
				combinedStaticFriction = 0.0f;
			}
		}
	}
}

#endif
