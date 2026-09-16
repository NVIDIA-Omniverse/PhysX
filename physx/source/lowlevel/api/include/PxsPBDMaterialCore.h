// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_PBD_MATERIAL_CORE_H
#define PXS_PBD_MATERIAL_CORE_H

#include "PxParticleGpu.h"
#include "PxsMaterialShared.h"

namespace physx
{
	struct PxsPBDMaterialData : public PxsParticleMaterialData
	{
		PxsPBDMaterialData()				{}	// PT: TODO: ctor leaves things uninitialized, is that by design?
		PxsPBDMaterialData(const PxEMPTY)	{}

		PxU32		flags;					//24
		PxReal		viscosity;				//28
		PxReal		vorticityConfinement;	//32
		PxReal		surfaceTension;			//36
		PxReal		cohesion;				//40
		PxReal		lift;					//44
		PxReal		drag;					//48
		PxReal		cflCoefficient;			//52
		PxReal		particleFrictionScale;	//56
		PxReal		particleAdhesionScale;	//60
	};

	typedef MaterialCoreT<PxsPBDMaterialData, PxPBDMaterial>		PxsPBDMaterialCore;

} //namespace phyxs

#endif
