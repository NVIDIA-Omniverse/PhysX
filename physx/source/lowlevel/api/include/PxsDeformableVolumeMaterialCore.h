// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_DEFORMABLE_VOLUME_MATERIAL_CORE_H
#define PXS_DEFORMABLE_VOLUME_MATERIAL_CORE_H

#include "PxDeformableVolumeMaterial.h"
#include "PxsMaterialShared.h"

namespace physx
{

PX_FORCE_INLINE PX_CUDA_CALLABLE PxU16 toUniformU16(PxReal f)
{
	f = PxClamp(f, 0.0f, 1.0f);
	return PxU16(f * 65535.0f);
}

PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal toUniformReal(PxU16 v)
{
	return PxReal(v) * (1.0f / 65535.0f);
}


PX_ALIGN_PREFIX(16) struct PxsDeformableVolumeMaterialData
{
	PxReal	youngs;					//4
	PxReal	poissons;				//8
	PxReal	dynamicFriction;		//12
	PxReal	elasticityDamping;		//16
	PxU16	materialModel;			//18
	PxU16	_pad;					//20
	PxReal	deformThreshold;		//24
	PxReal	deformLowLimitRatio;	//28
	PxReal	deformHighLimitRatio;	//32

	PX_CUDA_CALLABLE PxsDeformableVolumeMaterialData() :
		youngs				(1.e+6f),
		poissons			(0.45f),
		dynamicFriction		(0.0f),
		elasticityDamping	(0.0f),
		materialModel		(PxDeformableVolumeMaterialModel::eCO_ROTATIONAL),
		_pad				(0),
		deformThreshold		(PX_MAX_F32),
		deformLowLimitRatio	(1.0f),
		deformHighLimitRatio(1.0f)
	{}

	PxsDeformableVolumeMaterialData(const PxEMPTY) {}

}PX_ALIGN_SUFFIX(16);

typedef MaterialCoreT<PxsDeformableVolumeMaterialData, PxDeformableVolumeMaterial>	PxsDeformableVolumeMaterialCore;

} //namespace phyxs

#endif // PXS_DEFORMABLE_VOLUME_MATERIAL_CORE_H
