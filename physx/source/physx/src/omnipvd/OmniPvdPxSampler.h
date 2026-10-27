// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
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
	class PxArticulationJointReducedCoordinate;
	class PxArticulationLink;
	class PxRigidDynamic;
	class PxArticulationMimicJoint;

	class PxDeformableBody;
	class PxDeformableMaterial;
	class PxDeformableSurface;
	class PxDeformableSurfaceMaterial;
	class PxDeformableVolume;
	class PxDeformableVolumeMaterial;
	class PxPBDMaterial;
	class PxDiffuseParticleParams;
	class PxParticleBuffer;

	struct OmniPvdPxCoreRegistrationData;

	class NpOmniPvd;
}

void streamActorName(const physx::PxActor & a, const char* name);
void streamSceneName(const physx::PxScene & s, const char* name);
void streamArticulationName(const physx::PxArticulationReducedCoordinate & art, const char* name);
void streamArticulationJointName(const physx::PxArticulationJointReducedCoordinate& joint, const char* name);
void streamParticleBufferName(const physx::PxParticleBuffer& pb, const char* name);

void streamShapeMaterials(const physx::PxShape&, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials);

void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableSurfaceMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxDeformableVolumeMaterial* const * mats, physx::PxU32 nbrMaterials);
void streamShapeMaterials(const physx::PxShape&, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials);

void streamDiffuseParticleParamsAttributes(const physx::PxDiffuseParticleParams& diffuseParams);

void streamArticulationMimicJoint(const physx::PxArticulationMimicJoint& mj);

void streamShapeUpdateGeometry(const physx::PxShape& shape);

void streamDeformableVolumeAttributes(const physx::PxDeformableVolume& dv);
void streamDeformableSurfaceAttributes(const physx::PxDeformableSurface& ds);

enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
	eOmniPvdTetraMesh   = 3,
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
	
	void addRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicForceReset(const physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicTorqueReset(const physx::PxRigidDynamic* rigidDynamic);
	void removeRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic);
	
	void addArticulationFromLinkFlagChangeReset(const physx::PxArticulationLink* link);
	void addArticulationLinksForceReset(const physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationLinksTorqueReset(const physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationJointsForceReset(const physx::PxArticulationReducedCoordinate* articulation);
	void removeArticulationReset(const physx::PxArticulationReducedCoordinate* articulation);
	
	void resetForces();

private:
	physx::PxScene& mScene;
	physx::PxU64 mFrameId;

	physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicForce;
	physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicTorque;

	physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksForce;
	physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksTorque;
	physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationJointsForce;
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
	static OmniPvdPxSampler* getSamplingInstance();

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
