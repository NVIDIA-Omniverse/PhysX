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

#ifndef PXS_PBD_MATERIAL_CORE_H
#define PXS_PBD_MATERIAL_CORE_H

#include "PxParticleGpu.h"
#include "PxsMaterialShared.h"

namespace physx
{
	//Technically, this doesn't need the PxsParticleMaterialData types, but this allows us
	//to have a common base and opens the scope for rigid-particle interactions.
	struct PxsCustomMaterialData : public PxsParticleMaterialData
	{	
		PxsCustomMaterialData()					{}	// PT: TODO: ctor leaves things uninitialized, is that by design?
		PxsCustomMaterialData(const PxEMPTY)	{}

		void*		userData;				//24
	};

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
	typedef MaterialCoreT<PxsCustomMaterialData, PxCustomMaterial>	PxsCustomMaterialCore;

} //namespace phyxs

#endif
