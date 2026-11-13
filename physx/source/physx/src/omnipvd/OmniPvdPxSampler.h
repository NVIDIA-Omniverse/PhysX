// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxErrorCallback.h"
#include "common/PxPhysXCommonConfig.h"
#include "OmniPvdChunkAlloc.h"
#include "omnipvd/PxOmniPvd.h"

namespace physx
{
	class PxScene;
	class PxBase;
	class NpScene;
	class PxActor;
	class PxShape;
	class PxConstraint;
	class PxConstraintConnector;
	class PxMaterial;

	class PxArticulationReducedCoordinate;
	class PxArticulationJointReducedCoordinate;
	class PxArticulationLink;
	class PxRigidDynamic;
	class PxRigidActor;
	class PxRigidBody;
	class PxArticulationMimicJoint;

	namespace Sc { class BodyCore; }

	class PxDeformableBody;
	class PxDeformableSurface;
	class PxDeformableSurfaceMaterial;
	class PxDeformableVolume;
	class PxDeformableVolumeMaterial;
	class PxPBDMaterial;
	class PxDiffuseParticleParams;
	class PxParticleBuffer;
	class PxPBDParticleSystem;

	struct OmniPvdPxCoreRegistrationData;

	class NpOmniPvd;
}

class OmniPvdWriter;

void streamActorName(const physx::PxActor & a, const char* name);
void streamSceneName(const physx::PxScene & s, const char* name);
void streamArticulationName(const physx::PxArticulationReducedCoordinate & art, const char* name);
void streamArticulationJointName(const physx::PxArticulationJointReducedCoordinate& joint, const char* name);
void streamParticleBufferName(const physx::PxParticleBuffer& pb, const char* name);
// Explicit forms used by the full-state snapshot (caller already holds a write scope).
void streamSceneName(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxScene& s, const char* name);
void streamArticulationName(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationReducedCoordinate& art, const char* name);
void streamArticulationJointName(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationJointReducedCoordinate& joint, const char* name);

void streamShapeMaterials(const physx::PxShape&, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials);

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
	void resetFrameId(); // rewind mFrameId to 1 (odd = pre-sim) for a snapshot onto a fresh stream
	
	void addRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicForceReset(const physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicTorqueReset(const physx::PxRigidDynamic* rigidDynamic);
	void removeRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic);
	
	void addArticulationFromLinkFlagChangeReset(const physx::PxArticulationLink* link);
	void addArticulationLinksForceReset(const physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationLinksTorqueReset(const physx::PxArticulationReducedCoordinate* articulation);
	void removeArticulationReset(const physx::PxArticulationReducedCoordinate* articulation);
	
	void resetForces();
	bool areContactsValid() const { return mContactsValid; }
	void setContactsValid(bool valid) { mContactsValid = valid; }
	bool contactArraysCaptured() const { return mContactArraysCaptured; }
	void setContactArraysCaptured(bool captured) { mContactArraysCaptured = captured; }
	physx::PxArray<physx::PxU32>& getGpuSnapshotIndices() { return mGpuSnapshotIndices; }

private:
	physx::PxScene& mScene;
	physx::PxU64 mFrameId;
	bool mFrameOpen;
	bool mContactsValid;
	bool mContactArraysCaptured;
	physx::PxArray<physx::PxU32> mGpuSnapshotIndices;

	physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicForce;
	physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicTorque;

	physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksForce;
	physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksTorque;
};

}

class OmniPvdPxSampler : public physx::PxUserAllocated, public physx::PxErrorCallback
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	// Stops sampling: flips the sampling flag off so subsequent SDK object
	// add/remove notifications and per-frame writes are suppressed. Calling it twice is
	// harmless. It is required before starting another recording, but optional for final
	// teardown: PxPhysics destruction emits final remove notifications before deleting the
	// sampler. A later snapshotAll() (driven by startSampling()) records the current state again.
	// Retain particle mirrors on stop; release excluded mirrors at the next snapshot and all on buffer teardown.
	// Primary-only snapshots also release diffuse mirrors.
	bool stopSampling();
	bool isSampling();
	bool isTakingSnapshot() const;
	// Emits a full snapshot of the current live simulation state to the currently
	// bound write stream, in referenced-before-referrer order: registers the full
	// schema + singletons, then walks shared resources -> shapes -> actors /
	// articulations / aggregates -> deformables / particles -> per-scene state. Called
	// from NpOmniPvd::startSampling() so the bound stream is self-contained (a full
	// state to attach to, followed by the changes after it). The writer's handle counters
	// were zeroed by OmniPvdWriter::setWriteStream when the stream was bound, so the schema
	// registration re-mints identical handles. Ensures sampling is on (emission flows via
	// onObjectAdd). Returns false if there is no sampler.
	bool snapshotAll();
	bool streamCaptureCatalog(bool initialize);
	static void getCaptureObjectIdentity(const physx::PxBase& object, const physx::PxScene*& scene,
		const physx::PxBase*& owner, const char*& name);
	bool setCaptureDesc(const physx::PxOmniPvdFilterDesc& desc);
	physx::PxU32 validateCaptureDesc(const physx::PxOmniPvdFilterDesc& desc) const;
	static void buildCaptureObjectCatalog(physx::PxHashMap<physx::PxU64, const physx::PxBase*>& catalog);
	void resetCaptureObjects(OmniPvdWriter& writer);
	void clearRetiredCaptureHandles();
	bool isCaptureHandleRetired(physx::PxU64 handle) const;
	bool isObjectRecorded(const physx::PxBase& object) const;
	bool isCapturingAllObjects() const;
	void prepareSceneJointState(const physx::PxScene& scene);
	void updateConstraintOmniPvdProperties(const physx::PxConstraint& constraint);
	// Skip connector updates during NpPhysics::createConstraint(): its owner is still initializing.
	void beginConstraintCreation(const physx::PxConstraintConnector& connector);
	void endConstraintCreation(const physx::PxConstraintConnector& connector);
	bool isJointStateActor(const physx::PxActor& actor) const;
	void storeJointActorPose(const physx::PxActor& actor, const physx::PxTransform& pose);
	void storeJointActorVelocity(const physx::PxActor& actor, const physx::PxVec3& velocity, bool angular);
	bool getJointActorPose(const physx::PxRigidActor& actor, physx::PxTransform& pose) const;
	bool getJointActorVelocity(const physx::PxRigidActor& actor, physx::PxVec3& linear, physx::PxVec3& angular) const;
	void streamArticulationReactions(OmniPvdWriter* writer, const physx::OmniPvdPxCoreRegistrationData* registrationData,
		const physx::PxArticulationReducedCoordinate& articulation);
#if PX_SUPPORT_GPU_PHYSX
	void beginSceneGpuReadback(const physx::PxScene& scene);
	bool streamDeformableState(OmniPvdWriter* writer, const physx::OmniPvdPxCoreRegistrationData* registrationData,
		const physx::PxDeformableBody& body);
	bool streamParticleBufferState(OmniPvdWriter* writer, const physx::OmniPvdPxCoreRegistrationData* registrationData,
		const physx::PxParticleBuffer& buffer);
	bool streamParticleSystemState(OmniPvdWriter* writer, const physx::OmniPvdPxCoreRegistrationData* registrationData,
		const physx::PxPBDParticleSystem& system);
#endif
	void setOmniPvdInstance(physx::NpOmniPvd* omniPvdIntance);

	// writes all contacts to the stream
	void streamSceneContacts(physx::NpScene& scene);

	static OmniPvdPxSampler* getInstance();
	static OmniPvdPxSampler* getSamplingInstance();

	void onObjectAdd(const physx::PxBase& object);
	// Batch variant: emits the object under a caller-owned write scope (the caller holds the
	// exclusive writer lock and provides the bound writer + registration data). snapshotAll()
	// uses this so a whole batch of objects shares one lock acquisition instead of one per object.
	// The single-object onObjectAdd(object) above opens exactly one scope and forwards here.
	void onObjectAdd(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBase& object);
	void onObjectRemove(const physx::PxBase& object);
	void onConstraintActorsChange(const physx::PxConstraint& constraint);
#if PX_SUPPORT_GPU_PHYSX
	void onParticlePhaseChange(const physx::PxPBDParticleSystem& system);
#endif
	
	void reportAlreadySamplingError();
	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) PX_OVERRIDE;
};


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData();
NpOmniPvd* NpOmniPvdGetInstance();

// True only while a recording is active. stopSampling() clears the flag; PxPhysics teardown
// deletes the sampler after emitting final remove notifications, which also makes this false. The
// OVD write macros gate on this so that no PhysX operation writes OVD data when not sampling. Used
// only inside PhysX core; the extensions write path gates on PxOmniPvd::isSampling() instead.
bool NpOmniPvdSampling();
bool NpOmniPvdRecordSecondaryData();
bool NpOmniPvdCaptureActor(const PxActor& actor);
bool NpOmniPvdCaptureActorState(const PxActor& actor);
bool NpOmniPvdCaptureArticulation(const PxArticulationReducedCoordinate& articulation);
bool NpOmniPvdCaptureScene(const PxScene& scene);
bool NpOmniPvdCaptureIsSelective();
void NpOmniPvdCaptureDependency(const void* object);
void NpOmniPvdCaptureRetireHandle(PxU64 handle);
bool NpOmniPvdShouldRecordCreation(const char* className, PxU64 objectHandle);
bool NpOmniPvdShouldRecordDestruction(const char* className, PxU64 objectHandle);
bool NpOmniPvdShouldRecordValue(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle);
bool NpOmniPvdShouldRecordAddition(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle);
bool NpOmniPvdShouldRecordRemoval(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle);

}

#endif

#endif
