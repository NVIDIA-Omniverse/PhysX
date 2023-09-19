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

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "OmniPvdChunkAlloc.h"

namespace physx
{
	class PxScene;
	class PxBase;
	class NpScene;
	class PxActor;
	class PxShape;
	class PxMaterial;

	class PxFEMClothMaterial;
	class PxFEMMaterial;
	class PxFEMSoftBodyMaterial;
	class PxFLIPMaterial;
	class PxMPMMaterial;
	class PxParticleMaterial;
	class PxPBDMaterial;

	struct OmniPvdPxCoreRegistrationData;
}

void streamActorName(const physx::PxActor & a, const char* name);
void streamSceneName(const physx::PxScene & s, const char* name);

void streamShapeMaterials(const physx::PxShape&, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxFEMClothMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxFEMMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxFEMSoftBodyMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxFLIPMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxMPMMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxParticleMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials);


enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
};

class OmniPvdWriter;
class OmniPvdPxScene;

class OmniPvdPxSampler : public physx::PxUserAllocated
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	//enables sampling: 
	void startSampling();
	bool isSampling();
	//sets destination: 
	void setOmniPvdWriter(OmniPvdWriter* omniPvdWriter);	

	// writes all contacts to the stream
	void streamSceneContacts(physx::NpScene& scene);

	// call at the end of a simulation step: 
	void sampleScene(physx::NpScene* scene);

	static OmniPvdPxSampler* getInstance();

	void onObjectAdd(const physx::PxBase& object);
	void onObjectRemove(const physx::PxBase& object);


private:
	OmniPvdPxScene* getSampledScene(physx::NpScene* scene);
};


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData();
OmniPvdWriter* NpOmniPvdGetWriter();

}

#endif

#endif
