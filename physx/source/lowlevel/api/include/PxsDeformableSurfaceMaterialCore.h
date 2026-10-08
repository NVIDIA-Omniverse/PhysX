// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_DEFORMABLE_SURFACE_MATERIAL_CORE_H
#define PXS_DEFORMABLE_SURFACE_MATERIAL_CORE_H

#include "PxDeformableSurfaceMaterial.h"
#include "PxsMaterialShared.h"

namespace physx
{
	PX_ALIGN_PREFIX(16) 
	struct PxsDeformableSurfaceMaterialData
	{
		PxReal	youngs;					//4
		PxReal	poissons;				//8
		PxReal	dynamicFriction;		//12
		PxReal	thickness;				//16
		PxReal	bendingStiffness;		//20
		PxReal  elasticityDamping;		//24
		PxReal  bendingDamping;			//28
		PxReal	padding[1];				//32, 4 bytes padding to make the total size 32 bytes

		PX_CUDA_CALLABLE PxsDeformableSurfaceMaterialData()
		: youngs(1.e+6f)
		, poissons(0.45f)
		, dynamicFriction(0.0f)
		, thickness(0.0f)
		, bendingStiffness(0.0f)
		, elasticityDamping(0.0f)
		, bendingDamping(0.0f)
		{}

		PxsDeformableSurfaceMaterialData(const PxEMPTY) {}
	}
	PX_ALIGN_SUFFIX(16);

typedef MaterialCoreT<PxsDeformableSurfaceMaterialData, PxDeformableSurfaceMaterial>	PxsDeformableSurfaceMaterialCore;

} //namespace phyxs

#endif // PXS_DEFORMABLE_SURFACE_MATERIAL_CORE_H
