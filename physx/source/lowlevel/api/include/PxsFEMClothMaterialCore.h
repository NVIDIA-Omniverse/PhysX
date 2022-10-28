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

#ifndef PXS_FEM_CLOTH_MATERIAL_CORE_H
#define PXS_FEM_CLOTH_MATERIAL_CORE_H

#include "PxFEMClothMaterial.h"
#include "PxsMaterialShared.h"

namespace physx
{
	PX_ALIGN_PREFIX(16) struct PxsFEMClothMaterialAuxData
	{
		// derived quantities (computed internally)
		float	lambda;					//4
		float	mu;						//8
		float	padding[2];				//16
	}PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16) struct PxsFEMClothMaterialData
	{
		PxReal	youngs;					//4
		PxReal	poissons;				//8
		PxReal	dynamicFriction;		//12
		PxReal	thickness;				//16
		PxReal	elasticityDamping;		//20
		PxReal	bendingDamping;			//24
		PxReal	padding[2];				//32
	
		PX_CUDA_CALLABLE PxsFEMClothMaterialData() :
			youngs				(1.e+6f),
			poissons			(0.45f),
			dynamicFriction		(0.0f),
			thickness			(0.0f),
			elasticityDamping	(0.0f),
			bendingDamping		(0.0f)
		{}

		PxsFEMClothMaterialData(const PxEMPTY) {}

	}PX_ALIGN_SUFFIX(16);

	typedef MaterialCoreT<PxsFEMClothMaterialData, PxFEMClothMaterial>	PxsFEMClothMaterialCore;

} //namespace phyxs

#endif
