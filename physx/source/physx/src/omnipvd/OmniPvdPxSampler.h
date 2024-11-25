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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxErrorCallback.h"
#include "OmniPvdChunkAlloc.h"

namespace physx
{
	class PxScene;
	class PxBase;
	class NpScene;
	class PxActor;
	class PxShape;
	class PxMaterial;

	class PxArticulationReducedCoordinate;
	class PxRigidDynamic;

	class PxDeformableMaterial;
	class PxDeformableSurfaceMaterial;
	class PxDeformableVolumeMaterial;
	class PxPBDMaterial;
	class PxDiffuseParticleParams;

	struct OmniPvdPxCoreRegistrationData;

	class NpOmniPvd;
}

void streamActorName(const physx::PxActor & a, const char* name);
void streamSceneName(const physx::PxScene & s, const char* name);
void streamArticulationName(const physx::PxArticulationReducedCoordinate & art, const char* name);

void streamShapeMaterials(const physx::PxShape&, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials);

void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableSurfaceMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableVolumeMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials);

void streamDiffuseParticleParamsAttributes(const physx::PxDiffuseParticleParams& diffuseParams);

enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
};

class OmniPvdWriter;

namespace physx
{

class NpOmniPvdSceneClient : public physx::PxUserAllocated
{
public:
	NpOmniPvdSceneClient(physx::PxScene& scene);
	~NpOmniPvdSceneClient();	

	////////////////////////////////////////////////////////////////////////////////
	// Regarding the frame sampling strategy, the OVD frames start at (1:odd) with the first
	// one being a pre-Sim frame, for the setup calls done on the NpScene, in the constructor
	// as well as any user set operations once the scene was created, but not yet simulated.
	// 
	// After the first simulate call, the second frame (2:even), considers all the data recorded
	// up until the end of fetchresults as post-Sim.
	// 
	// Once fetchresults has exited, all the subsequent data is considered as pre-Sim data (odd frames)
	// 
	// Similarly for any subsequent simulate call, the data is considered post-Sim (evem frames)
	// 
	// A diagram of how this is layed out
	// 
	//  NpScene::NpScene()
	//    [pre-Sim data]  : frame 1   (odd frame)
	//  NpScene::simulate()
	//    [post-Sim data] : frame 2   (even frame)
	//  NpScene::fetchresults()
	//    [pre-Sim data]  : frame n+1 (odd frame)
	//  NpScene::simulate()
	//    [post-Sim data] : frame n+2 (even frame)
	//  NpScene::fetchresults()
	// 
	////////////////////////////////////////////////////////////////////////////////

	void startFirstFrame(OmniPvdWriter& pvdWriter);
	void incrementFrame(OmniPvdWriter& pvdWriter, bool recordProfileFrame = false); // stopFrame (frameID), then startFrame (frameID + 1)
	void stopLastFrame(OmniPvdWriter& pvdWriter);
	
	void addRigidDynamicForceReset(const physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicTorqueReset(const physx::PxRigidDynamic* rigidDynamic);
	
	void addArticulationLinksForceReset(const physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationLinksTorqueReset(const physx::PxArticulationReducedCoordinate* articulation);
	
	void addArticulationJointsForceReset(const physx::PxArticulationReducedCoordinate* articulation);
	
	void resetForces();

private:
	physx::PxScene& mScene;
	physx::PxU64 mFrameId;

	physx::PxArray<const PxRigidDynamic*> mRigidDynamicForceSets;
	physx::PxArray<const PxRigidDynamic*> mRigidDynamicTorqueSets;

	physx::PxArray<const PxArticulationReducedCoordinate*> mArticulationLinksForceSets;
	physx::PxArray<const PxArticulationReducedCoordinate*> mArticulationLinksTorqueSets;

	physx::PxArray<const PxArticulationReducedCoordinate*> mArticulationJointsForceSets;
};

}

class OmniPvdPxSampler : public physx::PxUserAllocated, public physx::PxErrorCallback
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	bool startSampling();
	bool isSampling();
	void setOmniPvdInstance(physx::NpOmniPvd* omniPvdIntance);

	// writes all contacts to the stream
	void streamSceneContacts(physx::NpScene& scene);

	static OmniPvdPxSampler* getInstance();

	void onObjectAdd(const physx::PxBase& object);
	void onObjectRemove(const physx::PxBase& object);
	
	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) PX_OVERRIDE;

};


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData();
NpOmniPvd* NpOmniPvdGetInstance();

}

#endif

#endif
