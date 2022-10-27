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

#ifndef PXS_MPM_MATERIAL_CORE_H
#define PXS_MPM_MATERIAL_CORE_H

#include "PxParticleGpu.h"
#include "PxsMaterialShared.h"
#include "PxMPMMaterial.h"

namespace physx
{
	struct PxsMPMMaterialData : public PxsParticleMaterialData
	{
		PxsMPMMaterialData()				{}	// PT: TODO: ctor leaves things uninitialized, is that by design?
		PxsMPMMaterialData(const PxEMPTY)	{}


		PxIntBool	isPlastic;						//24
		PxReal		youngsModulus;					//28
		PxReal		poissonsRatio;					//32

		PxReal		hardening; //snow				//36
		PxReal		criticalCompression; //snow		//40
		PxReal		criticalStretch; //snow			//44
		
		//Only used when damage tracking is activated in the cutting flags
		PxReal		tensileDamageSensitivity;		//48
		PxReal		compressiveDamageSensitivity;	//52
		PxReal		attractiveForceResidual;		//56

		PxReal		sandFrictionAngle; //sand		//60
		PxReal		yieldStress; //von Mises		//64

		PxMPMMaterialModel::Enum materialModel;		//68	
		PxMPMCuttingFlags cuttingFlags;				//72
		PxReal		density;						//76;

		PxReal		stretchAndShearDamping;			//80;
		PxReal		rotationalDamping;				//84;
	};

	typedef MaterialCoreT<PxsMPMMaterialData, PxMPMMaterial>	PxsMPMMaterialCore;

} //namespace phyxs

#endif
