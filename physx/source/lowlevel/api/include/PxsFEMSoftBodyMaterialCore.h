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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_FEM_MATERIAL_CORE_H
#define PXS_FEM_MATERIAL_CORE_H

#include "PxFEMSoftBodyMaterial.h"
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


	PX_ALIGN_PREFIX(16) struct PxsFEMSoftBodyMaterialData
	{
		PxReal	youngs;					//4
		PxReal	poissons;				//8
		PxReal	dynamicFriction;		//12
		PxReal	damping;				//16
		PxU16	dampingScale;			//20, known to be in the range of 0...1. Mapped to integer range 0...65535
		PxU16	materialModel;          //22
		PxReal	deformThreshold;		//24
		PxReal	deformLowLimitRatio;	//28
		PxReal	deformHighLimitRatio;	//32

		PX_CUDA_CALLABLE PxsFEMSoftBodyMaterialData() :
			youngs				(1.e+6f),
			poissons			(0.45f),
			dynamicFriction		(0.0f),
			damping				(0.0f),
			//dampingScale        (0),
			materialModel       (PxFEMSoftBodyMaterialModel::eCO_ROTATIONAL),
			deformThreshold		(PX_MAX_F32),
			deformLowLimitRatio	(1.0f),
			deformHighLimitRatio(1.0f)
		{}

		PxsFEMSoftBodyMaterialData(const PxEMPTY) {}

	}PX_ALIGN_SUFFIX(16);

	typedef MaterialCoreT<PxsFEMSoftBodyMaterialData, PxFEMSoftBodyMaterial>	PxsFEMSoftBodyMaterialCore;

} //namespace phyxs

#endif
