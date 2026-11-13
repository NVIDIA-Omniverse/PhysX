// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#if PX_SUPPORT_OMNI_PVD
#include <stdio.h>

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxString.h"
#include "common/PxProfileZone.h"

#include "ScIterators.h"

#include "omnipvd/NpOmniPvd.h"
#include "OmniPvdPxSampler.h"
#include "OmniPvdWriteStream.h"
#include "NpOmniPvdSetData.h"

#include "NpOmniPvdMetaData.h"


#include "NpPhysics.h"
#include "NpFactory.h"
#include "NpScene.h"
#include "NpAggregate.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpArticulationMimicJoint.h"
#include "NpPBDParticleSystem.h"
#include "NpParticleBuffer.h"
#include "PxPBDMaterial.h"
#include "NpDeformableVolume.h"
#include "NpDeformableSurface.h"
#include "PxDeformableVolume.h"
#include "PxDeformableSurface.h"
#include "PxDeformableBody.h"
#include "PxDeformableSurfaceMaterial.h"
#include "NpShape.h"
#include "NpConstraint.h"
#include "PxConstraintDesc.h"
#if PX_SUPPORT_GPU_PHYSX
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxsSimulationController.h"
#include "ScBodySim.h"
#endif

using namespace physx;

// Forward declaration - defined later in this file
void streamTetMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTetrahedronMesh& mesh);

class OmniPvdStreamContainer
{
public:
	OmniPvdStreamContainer();
	~OmniPvdStreamContainer();
	bool initOmniPvd();
	void registerClasses();
	bool dataWasWrittenSuccessfully();
	void setOmniPvdInstance(NpOmniPvd* omniPvdInstance);

	NpOmniPvd* mOmniPvdInstance;
	physx::PxMutex mMutex;
	OmniPvdPxCoreRegistrationData mRegistrationData;
};

struct OmniPvdJointActorState
{
	enum { ePOSE = 1, eLINEAR_VELOCITY = 2, eANGULAR_VELOCITY = 4 };
	OmniPvdJointActorState() : mScene(NULL), mPose(PxIdentity),
		mLinearVelocity(0.0f), mAngularVelocity(0.0f), mValid(0) {}
	OmniPvdJointActorState(const PxScene& scene) : OmniPvdJointActorState() { mScene = &scene; }
	const PxScene* mScene;
	PxTransform mPose;
	PxVec3 mLinearVelocity;
	PxVec3 mAngularVelocity;
	PxU8 mValid;
};

class OmniPvdSamplerInternals : public physx::PxUserAllocated
{
public:
OmniPvdStreamContainer mPvdStream;
bool addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum); // Returns true if the Geom was not yet seen and added
physx::PxMutex mSampleMutex;
bool mIsSampling;

physx::PxMutex mSharedGeomsMutex;
physx::PxHashMap<const void*, OmniPvdSharedMeshEnum> mSharedMeshesMap;
bool mRecordSecondaryData;
PxHashSet<const PxScene*> mCaptureSceneRoots;
PxHashSet<const PxActor*> mCaptureActorRoots;
PxHashSet<const PxArticulationReducedCoordinate*> mCaptureArticulationRoots;
PxHashSet<const PxBase*> mCaptureObjectRoots;
// Leaf lock: bookkeeping only; never hold across writer acquisition, callbacks, factory walks or CUDA.
physx::PxMutex mCaptureStateMutex;
PxHashSet<const PxScene*> mCaptureContextScenes;
PxHashMap<const PxBase*, const PxScene*> mCaptureRootScenes;
#if PX_SUPPORT_GPU_PHYSX
PxHashMap<const PxParticleBuffer*, const PxPBDParticleSystem*> mCaptureBufferOwners;
#endif
bool mCaptureContextScenesDirty;
bool mCaptureAllObjects;
bool mTakingSnapshot;
PxU32 mDependencyDepth;
PxHashSet<PxU64> mDependencies;
PxHashSet<PxU64> mRecordedObjects;
PxHashSet<PxU64> mRetiredControlHandles;
PxU64 mRetiredHandleSession;
physx::PxMutex mRetiredHandleMutex;
PxHashSet<PxU64> mStreamedObjects;
PxHashSet<const PxActor*> mContactActors;
PxHashMap<const PxActor*, const PxScene*> mContactActorScenes;
PxHashSet<const PxActor*> mRequiredContextActors;
PxHashSet<const PxConstraintConnector*> mCreatingConnectors;
PxArray<const PxActor*> mContextActorScratch;
PxHashMap<const PxArticulationReducedCoordinate*, PxArticulationCache*> mReactionCaches;
PxArray<PxU32> mArticulationDofStarts;
PxHashMap<const PxActor*, OmniPvdJointActorState> mJointActorStates;
PxArray<const PxActor*> mJointActorScratch;
#if PX_SUPPORT_GPU_PHYSX
PxArray<PxVec4> mGpuReadbackScratch;
PxHashSet<const PxScene*> mSyncedParticleScenes;
#endif
};
OmniPvdSamplerInternals * samplerInternals = NULL;
static thread_local bool suppressCaptureError = false;

template<typename T, typename U>
static bool captureContains(const PxHashSet<T>& objects, const U& object)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return objects.contains(object);
}

template<typename T, typename U>
static bool captureInsert(PxHashSet<T>& objects, const U& object)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return objects.insert(object);
}

template<typename T, typename U>
static bool captureErase(PxHashSet<T>& objects, const U& object)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return objects.erase(object);
}

template<typename T>
static void captureClear(PxHashSet<T>& objects)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	objects.clear();
}

static bool captureAllObjects()
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return samplerInternals->mCaptureAllObjects;
}

static bool captureSecondaryData()
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return samplerInternals->mRecordSecondaryData;
}

// The writer protects cache use and retirement. The state lock protects the map itself.
static PxArticulationCache* captureReactionCache(const PxArticulationReducedCoordinate& articulation)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	const PxHashMap<const PxArticulationReducedCoordinate*, PxArticulationCache*>::Entry* entry =
		samplerInternals->mReactionCaches.find(&articulation);
	return entry ? entry->second : NULL;
}

static void releaseCaptureReactionCache(const PxArticulationReducedCoordinate& articulation)
{
	PxArticulationCache* cache = NULL;
	{
		physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
		if (const PxHashMap<const PxArticulationReducedCoordinate*, PxArticulationCache*>::Entry* entry =
			samplerInternals->mReactionCaches.find(&articulation))
		{
			cache = entry->second;
			samplerInternals->mReactionCaches.erase(&articulation);
		}
	}
	if (cache) cache->release();
}

static bool captureIsSelective()
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	return !samplerInternals->mCaptureAllObjects ||
		!samplerInternals->mRecordSecondaryData;
}

static bool captureArticulation(const PxArticulationReducedCoordinate& articulation)
{
	return captureAllObjects() ||
		captureContains(samplerInternals->mCaptureArticulationRoots, &articulation) ||
		captureContains(samplerInternals->mCaptureSceneRoots,
			static_cast<const PxScene*>(static_cast<const NpArticulationReducedCoordinate&>(articulation).getNpScene()));
}

#if PX_SUPPORT_GPU_PHYSX
static const PxParticleBuffer* captureAsParticleBuffer(const PxBase& object)
{
	// PxBase::is<PxParticleBuffer>() checks the concrete type, excluding diffuse buffers.
	if (object.getConcreteType() == PxConcreteType::ePARTICLE_BUFFER ||
		object.getConcreteType() == PxConcreteType::ePARTICLE_DIFFUSE_BUFFER)
		return static_cast<const PxParticleBuffer*>(&object);
	return NULL;
}

static const NpPBDParticleSystem* captureParticleBufferOwner(const PxParticleBuffer& buffer)
{
	if (buffer.getConcreteType() == PxConcreteType::ePARTICLE_DIFFUSE_BUFFER)
		return static_cast<const NpParticleAndDiffuseBuffer&>(buffer).mParticleSystem;
	return static_cast<const NpParticleBuffer&>(buffer).mParticleSystem;
}
#endif

static bool captureActor(const PxActor& actor)
{
	if (captureAllObjects() || captureContains(samplerInternals->mCaptureActorRoots, &actor)) return true;
	if (actor.getConcreteType() == PxConcreteType::eARTICULATION_LINK &&
		captureArticulation(static_cast<const PxArticulationLink&>(actor).getArticulation())) return true;
	return captureContains(samplerInternals->mCaptureSceneRoots,
		static_cast<const PxScene*>(NpActor::getNpSceneFromActor(actor)));
}

static void invalidateCaptureScenes()
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	samplerInternals->mCaptureContextScenesDirty = true;
}

static void updateCaptureMembership(const char* className, const char* attributeName,
	PxU64 objectHandle, PxU64 valueHandle, bool added)
{
	if (samplerInternals->mTakingSnapshot) return;
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	if (!Pxstrcmp(className, "PxScene") && (!Pxstrcmp(attributeName, "actors") || !Pxstrcmp(attributeName, "articulations")))
	{
		const PxScene* scene = added ? reinterpret_cast<const PxScene*>(objectHandle) : NULL;
		if (const PxHashMap<const PxBase*, const PxScene*>::Entry* entry =
			samplerInternals->mCaptureRootScenes.find(reinterpret_cast<const PxBase*>(valueHandle)))
			samplerInternals->mCaptureRootScenes[entry->first] = scene;
		if (const PxHashMap<const PxActor*, const PxScene*>::Entry* entry =
			samplerInternals->mContactActorScenes.find(reinterpret_cast<const PxActor*>(valueHandle)))
			samplerInternals->mContactActorScenes[entry->first] = scene;
#if PX_SUPPORT_GPU_PHYSX
		for (PxHashMap<const PxParticleBuffer*, const PxPBDParticleSystem*>::Iterator it =
			samplerInternals->mCaptureBufferOwners.getIterator(); !it.done(); ++it)
			if (reinterpret_cast<PxU64>(it->second) == valueHandle) samplerInternals->mCaptureRootScenes[it->first] = scene;
#endif
		samplerInternals->mCaptureContextScenesDirty = true;
	}
#if PX_SUPPORT_GPU_PHYSX
	else if (!Pxstrcmp(className, "PxPBDParticleSystem") && !Pxstrcmp(attributeName, "particleBuffers"))
	{
		const PxParticleBuffer* buffer = reinterpret_cast<const PxParticleBuffer*>(valueHandle);
		if (samplerInternals->mCaptureBufferOwners.find(buffer))
		{
			const PxPBDParticleSystem* system = added ? reinterpret_cast<const PxPBDParticleSystem*>(objectHandle) : NULL;
			samplerInternals->mCaptureBufferOwners[buffer] = system;
			// This notification runs on the thread editing this system, not a foreign root.
			samplerInternals->mCaptureRootScenes[buffer] = system ? static_cast<const NpPBDParticleSystem*>(system)->getNpScene() : NULL;
			samplerInternals->mCaptureContextScenesDirty = true;
		}
	}
#endif
}

static void forgetCaptureOwnership(PxU64 objectHandle, bool scene)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	samplerInternals->mCaptureRootScenes.erase(reinterpret_cast<const PxBase*>(objectHandle));
	samplerInternals->mContactActorScenes.erase(reinterpret_cast<const PxActor*>(objectHandle));
	if (scene)
	{
		for (PxHashMap<const PxActor*, const PxScene*>::Iterator it = samplerInternals->mContactActorScenes.getIterator(); !it.done(); ++it)
			if (reinterpret_cast<PxU64>(it->second) == objectHandle) samplerInternals->mContactActorScenes[it->first] = NULL;
		for (PxHashMap<const PxBase*, const PxScene*>::Iterator it = samplerInternals->mCaptureRootScenes.getIterator(); !it.done(); ++it)
			if (reinterpret_cast<PxU64>(it->second) == objectHandle) samplerInternals->mCaptureRootScenes[it->first] = NULL;
	}
#if PX_SUPPORT_GPU_PHYSX
	samplerInternals->mCaptureBufferOwners.erase(reinterpret_cast<const PxParticleBuffer*>(objectHandle));
	for (PxHashMap<const PxParticleBuffer*, const PxPBDParticleSystem*>::Iterator it =
		samplerInternals->mCaptureBufferOwners.getIterator(); !it.done(); ++it)
		if (reinterpret_cast<PxU64>(it->second) == objectHandle)
		{
			// System release detaches buffers without individual membership removals.
			samplerInternals->mCaptureRootScenes[it->first] = NULL;
			samplerInternals->mCaptureBufferOwners[it->first] = NULL;
		}
#endif
	samplerInternals->mCaptureContextScenesDirty = true;
}

static bool captureScene(const PxScene& scene)
{
	physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
	if (captureAllObjects() || captureContains(samplerInternals->mCaptureSceneRoots, &scene)) return true;
	if (!samplerInternals->mCaptureContextScenesDirty) return samplerInternals->mCaptureContextScenes.contains(&scene);
	// Membership notifications maintain these identities under the same state lock.
	PxHashSet<const PxScene*>& scenes = samplerInternals->mCaptureContextScenes;
	scenes.clear();
	for (PxHashMap<const PxBase*, const PxScene*>::Iterator it = samplerInternals->mCaptureRootScenes.getIterator(); !it.done(); ++it)
		if (it->second) scenes.insert(it->second);
	samplerInternals->mCaptureContextScenesDirty = false;
	return scenes.contains(&scene);
}

static bool captureConstraint(const PxConstraint& constraint)
{
	PxRigidActor* first = NULL;
	PxRigidActor* second = NULL;
	constraint.getActors(first, second);
	return (first && captureActor(*first)) || (second && captureActor(*second));
}

static bool captureBase(const PxBase& object)
{
	if (captureAllObjects()) return true;
	if (captureContains(samplerInternals->mCaptureObjectRoots, &object)) return true;
	if (const PxActor* actor = object.is<PxActor>()) return captureActor(*actor);
	if (const PxArticulationReducedCoordinate* articulation = object.is<PxArticulationReducedCoordinate>()) return captureArticulation(*articulation);
	if (const PxArticulationJointReducedCoordinate* joint = object.is<PxArticulationJointReducedCoordinate>())
		return captureArticulation(joint->getChildArticulationLink().getArticulation());
	if (object.getConcreteType() == PxConcreteType::eARTICULATION_MIMIC_JOINT)
		return captureArticulation(static_cast<const PxArticulationMimicJoint&>(object).getArticulation());
	if (const PxConstraint* constraint = object.is<PxConstraint>()) return captureConstraint(*constraint);
	if (const PxAggregate* aggregate = object.is<PxAggregate>())
	{
		if (captureContains(samplerInternals->mCaptureSceneRoots,
			static_cast<const PxScene*>(static_cast<const NpAggregate*>(aggregate)->getNpScene()))) return true;
		for (PxU32 i = 0; i < aggregate->getNbActors(); ++i)
		{
			PxActor* actor = NULL;
			aggregate->getActors(&actor, 1, i);
			if (actor && captureActor(*actor)) return true;
		}
		return false;
	}
#if PX_SUPPORT_GPU_PHYSX
	if (const PxParticleBuffer* buffer = captureAsParticleBuffer(object))
	{
		const NpPBDParticleSystem* owner = captureParticleBufferOwner(*buffer);
		return captureAllObjects() || (owner && captureActor(*owner));
	}
#endif
	return captureContains(samplerInternals->mDependencies, reinterpret_cast<PxU64>(&object)) ||
		captureAllObjects();
}

static bool captureClass(const char* className, PxU64 handle)
{
	if (!captureIsSelective()) return true;
	if (!Pxstrcmp(className, "PxPhysics") || !Pxstrcmp(className, "PxOmniPvdMetaData")) return true;
	if (!Pxstrcmp(className, "PxScene")) return captureScene(*reinterpret_cast<const PxScene*>(handle));
	if (!Pxstrcmp(className, "PxActor") || !Pxstrcmp(className, "PxRigidActor") || !Pxstrcmp(className, "PxRigidBody") ||
		!Pxstrcmp(className, "PxRigidDynamic") || !Pxstrcmp(className, "PxRigidStatic") || !Pxstrcmp(className, "PxArticulationLink") ||
		!Pxstrcmp(className, "PxDeformableBody") || !Pxstrcmp(className, "PxDeformableSurface") ||
		!Pxstrcmp(className, "PxDeformableVolume") || !Pxstrcmp(className, "PxPBDParticleSystem"))
	{
		const PxActor* actor = reinterpret_cast<const PxActor*>(handle);
		return captureActor(*actor) || captureContains(samplerInternals->mContactActors, actor);
	}
	if (!Pxstrcmp(className, "PxArticulationReducedCoordinate") || !Pxstrcmp(className, "PxArticulationJointReducedCoordinate") ||
		!Pxstrcmp(className, "PxArticulationMimicJoint") || !Pxstrcmp(className, "PxAggregate") || !Pxstrcmp(className, "PxConstraint"))
		return captureBase(*reinterpret_cast<const PxBase*>(handle)) || captureContains(samplerInternals->mDependencies, handle);
	if (!Pxstrcmp(className, "PxParticleBuffer") || !Pxstrcmp(className, "PxParticleAndDiffuseBuffer"))
		return captureBase(*reinterpret_cast<const PxBase*>(handle));
	return captureContains(samplerInternals->mDependencies, handle) || samplerInternals->mDependencyDepth ||
		captureAllObjects();
}

static bool captureIdentityAttribute(const char* attribute)
{
	return !Pxstrcmp(attribute, "name") || !Pxstrcmp(attribute, "type") || !Pxstrcmp(attribute, "concreteTypeName") ||
		!Pxstrcmp(attribute, "parentLink") || !Pxstrcmp(attribute, "childLink") || !Pxstrcmp(attribute, "inboundJoint") ||
		!Pxstrcmp(attribute, "articulation") || !Pxstrcmp(attribute, "jointA") || !Pxstrcmp(attribute, "jointB") ||
		!Pxstrcmp(attribute, "simulationMesh") || !Pxstrcmp(attribute, "collisionMesh") ||
		!Pxstrcmp(attribute, "scene") || !Pxstrcmp(attribute, "elapsedTime");
}

static bool captureMotionAttribute(const char* attribute)
{
	if (!Pxstrcmp(attribute, "linearVelocity") || !Pxstrcmp(attribute, "angularVelocity") || !Pxstrcmp(attribute, "jointVelocity") ||
		!Pxstrcmp(attribute, "linearAcceleration") || !Pxstrcmp(attribute, "angularAcceleration") || !Pxstrcmp(attribute, "jointAcceleration") || !Pxstrcmp(attribute, "velocities")) return true;
	if (!Pxstrcmp(attribute, "globalPose") || !Pxstrcmp(attribute, "localPose") || !Pxstrcmp(attribute, "worldBounds") ||
		!Pxstrcmp(attribute, "parentTranslation") || !Pxstrcmp(attribute, "parentRotation") || !Pxstrcmp(attribute, "childTranslation") ||
		!Pxstrcmp(attribute, "childRotation") || !Pxstrcmp(attribute, "jointPosition") || !Pxstrcmp(attribute, "positions") ||
		!Pxstrcmp(attribute, "collisionPositions") || !Pxstrcmp(attribute, "simulationPositions") ||
		!Pxstrcmp(attribute, "positionInvMasses")) return true;
	return false;
}

static bool capturePrimaryAttribute(const char* className, const char* attribute)
{
	if (captureMotionAttribute(attribute)) return true;
	if (!Pxstrcmp(className, "PxParticleBuffer") && !Pxstrcmp(attribute, "phases")) return true;
	if (!Pxstrcmp(className, "PxShape") || !Pxstrcmp(className, "PxBaseMaterial") || !Pxstrncmp(className, "PxMaterial", 10) ||
		!Pxstrcmp(className, "PxPBDMaterial") || !Pxstrcmp(className, "PxDeformableSurfaceMaterial") ||
		!Pxstrcmp(className, "PxDeformableVolumeMaterial") || !Pxstrncmp(className, "PxConvex", 8) ||
		!Pxstrncmp(className, "PxTriangleMesh", 14) || !Pxstrncmp(className, "PxTetrahedronMesh", 17) ||
		!Pxstrncmp(className, "PxHeightField", 13) || !Pxstrcmp(className, "PxDeformableVolumeMesh") ||
		!Pxstrcmp(className, "PxSphereGeometry") || !Pxstrcmp(className, "PxBoxGeometry") ||
		!Pxstrcmp(className, "PxCapsuleGeometry") || !Pxstrcmp(className, "PxCustomGeometry"))
		return true;
	return false;
}

static void ensureCaptureObject(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxBase& object);
static void ensureCaptureContext(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxActor& actor);
static void refreshCaptureContextActors(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxScene& scene);
static void retireCaptureContextActors(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxScene& scene);
static void ensureCaptureShape(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxShape& shape);
static void ensureCaptureDependencies(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxBase& object);

static bool captureActorStateOnDevice(const PxActor& actor)
{
	NpScene* scene = NpActor::getNpSceneFromActor(actor);
	if (!scene || !scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) ||
		!scene->isDirectGPUAPIInitialized()) return false;
#if PX_SUPPORT_GPU_PHYSX
	if (actor.getType() == PxActorType::eRIGID_DYNAMIC)
	{
		const Sc::BodySim* sim = static_cast<const NpRigidDynamic&>(actor).getCore().getSim();
		// Before the first upload, the host fields describe this body and a reused device slot does not.
		if (sim && (sim->getLowLevelBody().mInternalFlags & PxsRigidBody::eFIRST_BODY_COPY_GPU)) return false;
	}
	if (actor.getType() == PxActorType::eARTICULATION_LINK)
	{
		const NpArticulationReducedCoordinate& articulation = static_cast<const NpArticulationReducedCoordinate&>(
			static_cast<const PxArticulationLink&>(actor).getArticulation());
		const PxU32 index = articulation.getCore().getGpuArticulationIndex();
		// Before the first upload, the host link and joint state describes this articulation.
		if (index == 0xffffffffu || scene->getScScene().getSimulationController()->isArticulationUploadPending(index)) return false;
	}
#endif
	return actor.getType() != PxActorType::eRIGID_STATIC;
}

static bool captureActorUsesDirectGPU(const PxActor& actor)
{
	return captureIsSelective() && captureActorStateOnDevice(actor);
}

namespace physx
{
NpOmniPvdSceneClient::NpOmniPvdSceneClient(physx::PxScene& scene) :
	mScene(scene), mFrameId(1), mFrameOpen(false), mContactsValid(false), mContactArraysCaptured(false)
{
}

NpOmniPvdSceneClient::~NpOmniPvdSceneClient()
{
}

void NpOmniPvdSceneClient::startFirstFrame(OmniPvdWriter& pvdWriter)
{
	if (mFrameOpen || !NpOmniPvdCaptureScene(mScene)) return;
	pvdWriter.startFrame((OmniPvdContextHandle)(&mScene), mFrameId);
	mFrameOpen = true;
}

void NpOmniPvdSceneClient::resetFrameId()
{
	// Rewind to the first (odd = pre-sim) frame so a snapshot onto a freshly-bound
	// stream opens at frame 1, matching a fresh recording, regardless of how far
	// this scene's frame counter has already advanced on the previous stream.
	mFrameId = 1;
	mFrameOpen = false;
}

void NpOmniPvdSceneClient::incrementFrame(OmniPvdWriter& pvdWriter, bool recordProfileFrame)
{
	const bool recorded = !captureIsSelective() || captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&mScene));
	stopLastFrame(pvdWriter);
	mFrameId++;
	if (recorded) pvdWriter.startFrame((OmniPvdContextHandle)(&mScene), mFrameId);
	mFrameOpen = recorded;
	if (recorded && recordProfileFrame)
	{
		PX_PROFILE_FRAME("PVD", PxU64(&mScene));
	}
}

void NpOmniPvdSceneClient::stopLastFrame(OmniPvdWriter& pvdWriter)
{
	if (!mFrameOpen) return;
	pvdWriter.stopFrame((OmniPvdContextHandle)(&mScene), mFrameId);
	mFrameOpen = false;
}

void NpOmniPvdSceneClient::addRigidDynamicForceReset(const physx::PxRigidDynamic* rigidDynamic)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureActor(*rigidDynamic)) return;
	mResetRigidDynamicForce.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::addRigidDynamicTorqueReset(const physx::PxRigidDynamic* rigidDynamic)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureActor(*rigidDynamic)) return;
	mResetRigidDynamicTorque.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::addRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureActor(*rigidDynamic)) return;
	mResetRigidDynamicForce.insert(rigidDynamic);
	mResetRigidDynamicTorque.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::removeRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic)
{
	mResetRigidDynamicForce.erase(rigidDynamic);
	mResetRigidDynamicTorque.erase(rigidDynamic);
	PxVec3 zeroForce(0.0f, 0.0f, 0.0f);
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, force, *rigidDynamic, zeroForce);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, torque, *rigidDynamic, zeroForce);
	OMNI_PVD_WRITE_SCOPE_END
}

void NpOmniPvdSceneClient::addArticulationLinksForceReset(const PxArticulationReducedCoordinate* articulation)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureArticulation(*articulation)) return;
	mResetArticulationLinksForce.insert(articulation);
}

void NpOmniPvdSceneClient::addArticulationLinksTorqueReset(const PxArticulationReducedCoordinate* articulation)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureArticulation(*articulation)) return;
	mResetArticulationLinksTorque.insert(articulation);
}

void NpOmniPvdSceneClient::addArticulationFromLinkFlagChangeReset(const physx::PxArticulationLink* link)
{
	if (!NpOmniPvdRecordSecondaryData() || !NpOmniPvdCaptureActor(*link)) return;
	PxArticulationReducedCoordinate& arti = link->getArticulation();
	{
		mResetArticulationLinksForce.insert(&arti);
		mResetArticulationLinksTorque.insert(&arti);
	}
}

#define SET_RIGID_BODY_ATTRIBS(resetRigidDynamic, rigiBodyAttribute, attribVal) \
{ \
	for(PxHashSet<const PxRigidDynamic*>::Iterator iter = resetRigidDynamic.getIterator(); !iter.done(); ++iter) \
	{ \
		const PxRigidDynamic* rdyn = *iter; \
		if (!(rdyn->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)) \
		{ \
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigiBodyAttribute, *rdyn, attribVal); \
		} \
	} \
	resetRigidDynamic.clear(); \
}

#define SET_ARTICULATION_LINK_ATTRIBS(articulationHash, linkAttribute, attribVal) \
{ \
	for(PxHashSet<const PxArticulationReducedCoordinate*>::Iterator iter = articulationHash.getIterator(); !iter.done(); ++iter) \
	{ \
		const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(*iter);	\
		const PxU32 nbLinks = npArticulation->getNbLinks(); \
		const NpArticulationLink* const * npLinks = npArticulation->getLinks(); \
		for(PxU32 linkId = 0; linkId < nbLinks; linkId++) \
		{ \
			const PxRigidBody* pxBody = static_cast<const PxRigidBody*>(npLinks[linkId]); \
			if (!(pxBody->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)) \
			{ \
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linkAttribute, *pxBody, attribVal); \
			} \
		} \
	} \
	articulationHash.clear();\
}

#define SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(pxArticulation, linkAttribute, attribVal) \
{ \
	const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(pxArticulation);	\
	const PxU32 nbLinks = npArticulation->getNbLinks(); \
	const NpArticulationLink* const * npLinks = npArticulation->getLinks(); \
	for(PxU32 linkId = 0; linkId < nbLinks; linkId++) \
	{ \
		const PxRigidBody* pxBody = static_cast<const PxRigidBody*>(npLinks[linkId]); \
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linkAttribute, *pxBody, attribVal); \
	} \
}

void setSingleArticulationJointForces(const PxArticulationReducedCoordinate* pxArticulation, OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData,  const PxReal* dofForces)
{
	const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(pxArticulation);
	const PxU32 nbLinks = npArticulation->getNbLinks();
	const NpArticulationLink* const * npLinks = npArticulation->getLinks();
	for(PxU32 linkId = 0; linkId < nbLinks; linkId++)
	{
		const NpArticulationLink* npLink = npLinks[linkId];
		PxArticulationJointReducedCoordinate* pxJoint = npLink->getInboundJoint();
		if (pxJoint)
		{
			const PxU32 nbrDofs = npLink->getInboundJointDof();
			if (nbrDofs > 0)
			{
				OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointForce, *pxJoint, dofForces, PxArticulationAxis::eCOUNT);
			}
		}
	}
}

void NpOmniPvdSceneClient::removeArticulationReset(const PxArticulationReducedCoordinate* articulation)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		PxVec3 zeroForce(0.0f, 0.0f, 0.0f);
		const PxReal dofZeroForces[PxArticulationAxis::eCOUNT] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
		SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(articulation, force, zeroForce)
		SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(articulation, torque, zeroForce)
		setSingleArticulationJointForces(articulation, pvdWriter, pvdRegData, dofZeroForces);
	OMNI_PVD_WRITE_SCOPE_END

	mResetArticulationLinksForce.erase(articulation);
	mResetArticulationLinksTorque.erase(articulation);
}

void NpOmniPvdSceneClient::resetForces()
{
	if ( (mResetRigidDynamicForce.size() > 0) || (mResetRigidDynamicTorque.size() > 0) ||
		 (mResetArticulationLinksForce.size() > 0) || (mResetArticulationLinksTorque.size() > 0)
	   )
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

		PxVec3 zeroForce(0.0f, 0.0f, 0.0f);

		// RigidDynamic
		SET_RIGID_BODY_ATTRIBS(mResetRigidDynamicForce, force, zeroForce)
		SET_RIGID_BODY_ATTRIBS(mResetRigidDynamicTorque, torque, zeroForce)

		// Articulations
		SET_ARTICULATION_LINK_ATTRIBS(mResetArticulationLinksForce, force, zeroForce)
		SET_ARTICULATION_LINK_ATTRIBS(mResetArticulationLinksTorque, torque, zeroForce)

		OMNI_PVD_WRITE_SCOPE_END
	}
}

}

OmniPvdStreamContainer::OmniPvdStreamContainer()
{
	physx::PxMutex::ScopedLock myLock(mMutex);
	mOmniPvdInstance = NULL;
}

OmniPvdStreamContainer::~OmniPvdStreamContainer()
{
}

void OmniPvdStreamContainer::setOmniPvdInstance(NpOmniPvd* omniPvdInstance)
{
	mOmniPvdInstance = omniPvdInstance;
}

bool OmniPvdStreamContainer::initOmniPvd()
{
	physx::PxMutex::ScopedLock myLock(mMutex);

	registerClasses();

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OmniPvdObjectHandle metaDataInstanceHandle = reinterpret_cast<OmniPvdObjectHandle>(&mOmniPvdInstance->mMetaData);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, metaDataInstanceHandle);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionMajor,  metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionMajor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionMinor,  metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionMinor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionBugfix, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionBugfix);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, ovdIntegrationVersionMajor, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.ovdIntegrationVersionMajor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, ovdIntegrationVersionMinor, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.ovdIntegrationVersionMinor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, recordSecondaryData, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.recordSecondaryData);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, captureAllObjects, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.captureAllObjects);

	PxPhysics& physicsRef = static_cast<PxPhysics&>(NpPhysics::getInstance());
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, physicsRef);
	const physx::PxTolerancesScale& tolScale = physicsRef.getTolerancesScale();
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tolerancesScale, physicsRef, tolScale);

	OMNI_PVD_WRITE_SCOPE_END

	return dataWasWrittenSuccessfully();
}

void OmniPvdStreamContainer::registerClasses()
{
	// Write the full class/attribute schema to the bound stream. startSampling() calls this once
	// per snapshot (through initOmniPvd) so every recording starts with its own schema block. The
	// writer's handle counters were zeroed by OmniPvdWriter::setWriteStream when the stream was
	// bound, so registerData() re-mints the same class/attribute handle sequence (registration is
	// deterministic), keeping the cached handles in mRegistrationData consistent.
	PxOmniPvd::ScopedExclusiveWriter writeLock(mOmniPvdInstance);
	OmniPvdWriter* writer = writeLock.getWriter();
	if (writer)
	{
		mRegistrationData.registerData(*writer);
	}
}

bool OmniPvdStreamContainer::dataWasWrittenSuccessfully()
{
	bool dataWasWrittenOk = false;
	PxOmniPvd::ScopedExclusiveWriter writeLock(mOmniPvdInstance);
	OmniPvdWriter* writer = writeLock.getWriter();
	if (writer)
	{
		uint32_t statusFlags = writer->getStatus();
		if (!(statusFlags & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE))
		{
			dataWasWrittenOk = true;
		}
	}
	return dataWasWrittenOk;
}


int streamStringLength(const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	#define OMNI_PVD_MAX_STRING_LENGTH 2048
	if (NpPhysics::getInstance().mOmniPvdSampler == NULL)
	{
		return 0;
	}
	if (name == NULL)
	{
		return 0;
	}
	int len = static_cast<int>(strnlen(name, OMNI_PVD_MAX_STRING_LENGTH));
	if (len > 0)
	{
		return len;
	}
	else
	{
		return 0;
	}
#else
	return 0;
#endif
}

// Explicit form for callers that already hold a write scope (snapshot / streamActorAttributes):
// it reuses the caller's writer instead of opening a nested one.
void streamActorName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxActor& a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, name, a, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

// Standalone form for the public setName() API path, which holds no write scope of its own.
void streamActorName(const physx::PxActor & a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamActorName(pvdWriter, pvdRegData, a, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

// Explicit forms for callers that already hold a write scope (the full-state snapshot emits the
// scene / articulation / joint name inside its scope). The standalone setName() API path uses the
// non-explicit overloads below, which open their own scope and delegate here.
void streamSceneName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxScene& s, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, name, s, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamSceneName(const physx::PxScene & s, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamSceneName(pvdWriter, pvdRegData, s, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamArticulationName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationReducedCoordinate& art, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, name, art, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamArticulationName(const physx::PxArticulationReducedCoordinate & art, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationName(pvdWriter, pvdRegData, art, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamArticulationJointName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationJointReducedCoordinate& joint, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, name, joint, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamArticulationJointName(const physx::PxArticulationJointReducedCoordinate& joint, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationJointName(pvdWriter, pvdRegData, joint, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

// Explicit form for callers that already hold a write scope (snapshot / streamParticleBufferAttributes):
// it reuses the caller's writer instead of opening a nested one.
void streamParticleBufferName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, name, pb, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

// Standalone form for the public setName() API path, which holds no write scope of its own.
void streamParticleBufferName(const physx::PxParticleBuffer& pb, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamParticleBufferName(pvdWriter, pvdRegData, pb, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamSphereGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxSphereGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, radius, g, g.radius);
}

void streamCapsuleGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxCapsuleGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, halfHeight, g, g.halfHeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, radius, g, g.radius);
}

void streamBoxGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBoxGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, halfExtents, g, g.halfExtents);
}

void streamPlaneGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPlaneGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, g);
}

void streamCustomGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxCustomGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, callbacks, g, g.callbacks);
}

void streamConvexCore(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexCoreGeometry& g)
{
		switch (g.getCoreType())
		{
			case PxConvexCore::ePOINT:
			{
				const PxConvexCore::Point& c = g.getCore<PxConvexCore::Point>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCorePoint, OmniPvdObjectHandle(&c));
			}
			break;
			case PxConvexCore::eSEGMENT:
			{
				const PxConvexCore::Segment& c = g.getCore<PxConvexCore::Segment>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreSegment, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreSegment, length, OmniPvdObjectHandle(&c), c.length);
			}
			break;
			case PxConvexCore::eBOX:
			{
				const PxConvexCore::Box& c = g.getCore<PxConvexCore::Box>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreBox, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreBox, extents, OmniPvdObjectHandle(&c), c.extents);
			}
			break;
			case PxConvexCore::eELLIPSOID:
			{
				const PxConvexCore::Ellipsoid& c = g.getCore<PxConvexCore::Ellipsoid>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreEllipsoid, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreEllipsoid, radii, OmniPvdObjectHandle(&c), c.radii);
			}
			break;
			case PxConvexCore::eCYLINDER:
			{
				const PxConvexCore::Cylinder& c = g.getCore<PxConvexCore::Cylinder>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, height, OmniPvdObjectHandle(&c), c.height);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, radius, OmniPvdObjectHandle(&c), c.radius);
			}
			break;
			case PxConvexCore::eCONE:
			{
				const PxConvexCore::Cone& c = g.getCore<PxConvexCore::Cone>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, height, OmniPvdObjectHandle(&c), c.height);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, radius, OmniPvdObjectHandle(&c), c.radius);
			}
			break;
			default:
				break;
		}
}

void streamConvexCoreGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexCoreGeometry& g)
{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, g);
		streamConvexCore(pvdWriter, pvdRegData, g);
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, core, g, g.getCoreData());
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, margin, g, g.getMargin());
}

void streamConvexMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexMesh& mesh)
{
	PX_COMPILE_TIME_ASSERT(sizeof(PxVec3) == 3 * sizeof(PxReal));

	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdConvexMesh))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, mesh);


		const PxU32 nbPolys = mesh.getNbPolygons();
		const PxU8* polygons = mesh.getIndexBuffer();
		const PxVec3* verts = mesh.getVertices();
		const PxU32 nbrVerts = mesh.getNbVertices();

		PxU32 totalTris = 0;
		for (PxU32 i = 0; i < nbPolys; i++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(i, data);
			totalTris += data.mNbVerts - 2;
		}

		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(totalTris * 3), "tmpIndices");

		PxU32 triIndex = 0;
		for (PxU32 p = 0; p < nbPolys; p++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(p, data);
			PxU32 nbTris = data.mNbVerts - 2;
			const PxU32 vref0 = polygons[data.mIndexBase + 0 + 0];
			for (PxU32 t = 0; t < nbTris; t++)
			{
				const PxU32 vref1 = polygons[data.mIndexBase + t + 1];
				const PxU32 vref2 = polygons[data.mIndexBase + t + 2];
				tmpIndices[triIndex + 0] = vref0;
				tmpIndices[triIndex + 1] = vref1;
				tmpIndices[triIndex + 2] = vref2;
				triIndex += 3;
			}
		}

		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, verts, mesh, reinterpret_cast<const PxReal*>(verts), 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, tris, mesh, tmpIndices, 3 * totalTris);
		PX_FREE(tmpIndices);
	}
}

void streamConvexMeshGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexMeshGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, scale, g, g.scale.scale);
	streamConvexMesh(pvdWriter, pvdRegData, *g.convexMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, convexMesh, g, g.convexMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, meshFlags, g, g.meshFlags);
}

void streamHeightField(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxHeightField& hf)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&hf, OmniPvdSharedMeshEnum::eOmniPvdHeightField))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);

		const PxU32 nbCols = hf.getNbColumns();
		const PxU32 nbRows = hf.getNbRows();
		const PxU32 nbVerts = nbRows * nbCols;
		const PxU32 nbFaces = (nbCols - 1) * (nbRows - 1) * 2;
		physx::PxHeightFieldSample* sampleBuffer = (physx::PxHeightFieldSample*)PX_ALLOC(sizeof(physx::PxHeightFieldSample)*(nbVerts), "sampleBuffer");
		hf.saveCells(sampleBuffer, nbVerts * sizeof(physx::PxHeightFieldSample));
		//TODO: are the copies necessary?
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbVerts * 3), "tmpVerts");
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbFaces * 3), "tmpIndices");
		for (PxU32 i = 0; i < nbRows; i++)
		{
			for (PxU32 j = 0; j < nbCols; j++)
			{
				const float x = PxReal(i);// *rs;
				const float y = PxReal(sampleBuffer[j + (i*nbCols)].height);// *hs;
				const float z = PxReal(j);// *cs;
				const PxU32 vertexIndex = 3 * (i * nbCols + j);
				float* vert = &tmpVerts[vertexIndex];
				vert[0] = x;
				vert[1] = y;
				vert[2] = z;
			}
		}
		for (PxU32 i = 0; i < (nbCols - 1); ++i)
		{
			for (PxU32 j = 0; j < (nbRows - 1); ++j)
			{
				PxU8 tessFlag = sampleBuffer[i + j * nbCols].tessFlag();
				PxU32 i0 = j * nbCols + i;
				PxU32 i1 = j * nbCols + i + 1;
				PxU32 i2 = (j + 1) * nbCols + i;
				PxU32 i3 = (j + 1) * nbCols + i + 1;
				// i2---i3
				// |    |
				// |    |
				// i0---i1
				// this is really a corner vertex index, not triangle index
				PxU32 mat0 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2);
				PxU32 mat1 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2 + 1);
				bool hole0 = (mat0 == PxHeightFieldMaterial::eHOLE);
				bool hole1 = (mat1 == PxHeightFieldMaterial::eHOLE);
				// first triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 0] = hole0 ? i0 : i2; // duplicate i0 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 1] = i0;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 2] = tessFlag ? i3 : i1;
				// second triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 3] = hole1 ? i1 : i3; // duplicate i1 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 4] = tessFlag ? i0 : i2;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 5] = i1;
			}
		}
		PX_FREE(sampleBuffer);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, verts, hf, tmpVerts, 3 * nbVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, tris, hf, tmpIndices, 3 * nbFaces);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamHeightFieldGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxHeightFieldGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, g);
	streamHeightField(pvdWriter, pvdRegData, *g.heightField);

	PxVec3 vertScale(g.rowScale, g.heightScale, g.columnScale);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, scale, g, vertScale);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, heightField, g, g.heightField);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, meshFlags, g, g.heightFieldFlags);
}

void streamActorAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxActor& actor, const bool supportStandaloneBounds)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, flags, actor, actor.getActorFlags());
	streamActorName(pvdWriter, pvdRegData, actor, actor.getName());
	// Should we stream the worldBounds if the actor is not part of a Scene yet?
	if (!captureActorUsesDirectGPU(actor) && (supportStandaloneBounds || actor.getScene()))
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, actor, actor.getWorldBounds())
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, dominance, actor, actor.getDominanceGroup())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, ownerClient, actor, actor.getOwnerClient())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, environmentID, actor, actor.getEnvironmentID());
}

void streamRigidActorAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const PxRigidActor &ra)
{
	streamActorAttributes(pvdWriter, pvdRegData, ra, true);

	if (!captureActorUsesDirectGPU(ra))
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, ra, ra.getGlobalPose())
	}

	// Stream shapes too
	const int nbrShapes = NpOmniPvdSampling() ? ra.getNbShapes() : 0;
	for (int s = 0; s < nbrShapes; s++)
	{
		PxShape* shape[1];
		ra.getShapes(shape, 1, s);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, shapes, ra, *shape[0])
	}
}

void streamRigidBodyAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidBody& rigidBody)
{
	streamRigidActorAttributes(pvdWriter, pvdRegData, rigidBody);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, cMassLocalPose, rigidBody, rigidBody.getCMassLocalPose());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, mass, rigidBody, rigidBody.getMass());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, massSpaceInertiaTensor, rigidBody, rigidBody.getMassSpaceInertiaTensor());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearDamping, rigidBody, rigidBody.getLinearDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularDamping, rigidBody, rigidBody.getAngularDamping());
	if (!captureActorUsesDirectGPU(rigidBody))
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, rigidBody, rigidBody.getLinearVelocity());
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, rigidBody, rigidBody.getAngularVelocity());
		const PxScene* scene = rigidBody.getScene();
		if (samplerInternals->mTakingSnapshot && scene && !(scene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
			(scene->getFlags() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS) && captureActor(rigidBody))
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearAcceleration, rigidBody, rigidBody.getLinearAcceleration());
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularAcceleration, rigidBody, rigidBody.getAngularAcceleration());
		}
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxLinearVelocity, rigidBody, rigidBody.getMaxLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxAngularVelocity, rigidBody, rigidBody.getMaxAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, rigidBody, rigidBody.getRigidBodyFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, minCCDAdvanceCoefficient, rigidBody, rigidBody.getMinCCDAdvanceCoefficient());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxDepenetrationVelocity, rigidBody, rigidBody.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxContactImpulse, rigidBody, rigidBody.getMaxContactImpulse());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, contactSlopCoefficient, rigidBody, rigidBody.getContactSlopCoefficient());
}

void streamRigidDynamicAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidDynamic& rd)
{
	streamRigidBodyAttributes(pvdWriter, pvdRegData, rd);

	if (rd.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, rd, rd.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, rd, true);
	}

	// Getters don't issue warnings, just return values
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, sleepThreshold, rd, rd.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, stabilizationThreshold, rd, rd.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rigidDynamicLockFlags, rd, rd.getRigidDynamicLockFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, wakeCounter, rd, rd.getWakeCounter());

	PxU32 positionIters, velocityIters; rd.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, positionIterations, rd, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, velocityIterations, rd, velocityIters);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, contactReportThreshold, rd, rd.getContactReportThreshold());
	PxTransform target;
	if(rd.getKinematicTarget(target))
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, kinematicTarget, rd, target);
	}
}

void streamRigidDynamic(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidDynamic& rd)
{
	const PxActor& a = rd;
	PX_ASSERT(&a == &rd);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rd);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_DYNAMIC);

	streamRigidDynamicAttributes(pvdWriter, pvdRegData, rd);
}

void streamRigidStatic(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidStatic& rs)
{
	const PxActor& a = rs;
	PX_ASSERT(&a == &rs);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidStatic, rs);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_STATIC);

	streamRigidActorAttributes(pvdWriter, pvdRegData, rs);
}

#if PX_SUPPORT_GPU_PHYSX

void streamPBDParticleSystemAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDParticleSystem& ps)
{
	streamActorAttributes(pvdWriter, pvdRegData, ps, false);
	const NpPBDParticleSystem& npPs = static_cast<const NpPBDParticleSystem&>(ps);

	PxU32 positionIters, velocityIters; ps.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, positionIterations, ps, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, velocityIterations, ps, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, simulationFilterData, ps, ps.getSimulationFilterData());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleFlags, ps, ps.getParticleFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxDepenetrationVelocity, ps, ps.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxVelocity, ps, ps.getMaxVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, restOffset, ps, ps.getRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, contactOffset, ps, ps.getContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleContactOffset, ps, ps.getParticleContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, solidRestOffset, ps, ps.getSolidRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleLockFlags, ps, ps.getParticleLockFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidRestOffset, ps, ps.getFluidRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, wind, ps, ps.getWind());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidBoundaryDensityScale, ps, ps.getFluidBoundaryDensityScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeX, ps, npPs.getCore().getGridSizeX());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeY, ps, npPs.getCore().getGridSizeY());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeZ, ps, npPs.getCore().getGridSizeZ());

	const PxArray<NpParticleBuffer*>& particleBuffers = npPs.mParticleBuffers;
	for (PxParticleBuffer* pb : particleBuffers)
	{
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, ps, *pb);
	}

	const PxArray<NpParticleAndDiffuseBuffer*>& particleDiffuseBuffers = npPs.mParticleDiffuseBuffers;
	for (PxParticleBuffer* pb : particleDiffuseBuffers)
	{
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, ps, *pb);
	}
}

void streamPBDParticleSystem(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDParticleSystem& ps)
{
	const PxActor& a = ps;
	PX_ASSERT(&a == &ps);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, ps);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::ePBD_PARTICLESYSTEM);

	streamPBDParticleSystemAttributes(pvdWriter, pvdRegData, ps);
}

void streamParticleBufferAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, maxParticles, pb, pb.getMaxParticles());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, flatListStartIndex, pb, pb.getFlatListStartIndex());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, uniqueId, pb, pb.getUniqueId());
	streamParticleBufferName(pvdWriter, pvdRegData, pb, pb.getName());
}

void streamParticleBuffer(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, pb);
	streamParticleBufferAttributes(pvdWriter, pvdRegData, pb);
}

void streamDiffuseParticleParamsAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDiffuseParticleParams& diffuseParams)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, threshold, diffuseParams, diffuseParams.threshold);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, lifetime, diffuseParams, diffuseParams.lifetime);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, airDrag, diffuseParams, diffuseParams.airDrag);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, bubbleDrag, diffuseParams, diffuseParams.bubbleDrag);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, buoyancy, diffuseParams, diffuseParams.buoyancy);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, kineticEnergyWeight, diffuseParams, diffuseParams.kineticEnergyWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, pressureWeight, diffuseParams, diffuseParams.pressureWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, divergenceWeight, diffuseParams, diffuseParams.divergenceWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, collisionDecay, diffuseParams, diffuseParams.collisionDecay);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, useAccurateVelocity, diffuseParams, diffuseParams.useAccurateVelocity);
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDiffuseParticleParamsAttributes(const physx::PxDiffuseParticleParams& diffuseParams)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDiffuseParticleParamsAttributes(pvdWriter, pvdRegData, diffuseParams);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamParticleAndDiffuseBufferAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleAndDiffuseBuffer& pb)
{
	streamParticleBufferAttributes(pvdWriter, pvdRegData, pb);
	if (NpOmniPvdRecordSecondaryData())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, maxDiffuseParticles, pb, pb.getMaxDiffuseParticles());
		const PxDiffuseParticleParams& diffuseParams = static_cast<const NpParticleAndDiffuseBuffer&>(pb).getDiffuseParticleParamsRef();
		streamDiffuseParticleParamsAttributes(pvdWriter, pvdRegData, diffuseParams);
	}
}

void streamParticleAndDiffuseBuffer(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleAndDiffuseBuffer& pb)
{
	const PxDiffuseParticleParams& diffuseParams = static_cast<const NpParticleAndDiffuseBuffer&>(pb).getDiffuseParticleParamsRef();
	if (NpOmniPvdRecordSecondaryData())
	{
		captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(&diffuseParams));
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, diffuseParams);
	}
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, pb);
		streamParticleAndDiffuseBufferAttributes(pvdWriter, pvdRegData, pb);
	}

	//add PxDiffuseParticleParams
	if (NpOmniPvdRecordSecondaryData())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, diffuseParticleParams, pb, &diffuseParams);
	}
}

void streamDeformableBodyAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableBody& db)
{
	streamActorAttributes(pvdWriter, pvdRegData, db, false);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, deformableBodyFlags, db, db.getDeformableBodyFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, linearDamping, db, db.getLinearDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxLinearVelocity, db, db.getMaxLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxDepenetrationVelocity, db, db.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, selfCollisionFilterDistance, db, db.getSelfCollisionFilterDistance());
	PxU32 positionIters, velocityIters; db.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, solverIterationCount, db, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, sleepThreshold, db, db.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingThreshold, db, db.getSettlingThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingDamping, db, db.getSettlingDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, wakeCounter, db, db.getWakeCounter());
	if (db.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, isSleeping, db, db.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, isSleeping, db, true);
	}
}

void streamDeformableVolumeAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolume& dv)
{
	if (captureIsSelective())
	{
		if (!captureBase(dv)) return;
		ensureCaptureDependencies(pvdWriter, pvdRegData, dv);
	}
	streamDeformableBodyAttributes(pvdWriter, pvdRegData, dv);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, selfCollisionStressTolerance, dv, dv.getSelfCollisionStressTolerance());

	// Collision shape is added in NpDeformableVolume::attachShape.
	// Simulation mesh shape is added here (deferred) because the tet mesh object
	// may not exist in the OVD stream yet when attachSimulationMesh fires.
	const PxShape* shape = static_cast<const NpDeformableVolume&>(dv).getShapeInternal();
	if (shape)
	{
		const PxTetrahedronMesh* simMesh = dv.getSimulationMesh();
		if (simMesh)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, simulationMesh, dv, simMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, nbSimulationMeshVertices, dv, simMesh->getNbVertices());

			// Stream the simulation tet mesh once. It doesn't go through onObjectAdd
			// since it's created internally as part of PxDeformableVolumeMesh.
			if (samplerInternals->addSharedMeshIfNotSeen(simMesh, OmniPvdSharedMeshEnum::eOmniPvdTetraMesh))
			{
				streamTetMesh(pvdWriter, pvdRegData, *simMesh);
				const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());
				OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, *simMesh);
				OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, simulationMeshShapes, dv, *simMesh);
			}
		}

		const PxTetrahedronMesh* collMesh = dv.getCollisionMesh();
		if (collMesh)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, collisionMesh, dv, collMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, nbCollisionMeshVertices, dv, collMesh->getNbVertices());
		}
	}
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDeformableVolumeAttributes(const physx::PxDeformableVolume& dv)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDeformableVolumeAttributes(pvdWriter, pvdRegData, dv);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamDeformableVolume(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolume& dv)
{
	const PxActor& a = dv;
	PX_ASSERT(&a == &dv);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, dv);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eDEFORMABLE_VOLUME);

	streamDeformableVolumeAttributes(pvdWriter, pvdRegData, dv);
}

void streamDeformableSurfaceAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurface& ds)
{
	if (captureIsSelective())
	{
		if (!captureBase(ds)) return;
		ensureCaptureDependencies(pvdWriter, pvdRegData, ds);
	}
	streamDeformableBodyAttributes(pvdWriter, pvdRegData, ds);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbCollisionPairUpdatesPerTimestep, ds, ds.getNbCollisionPairUpdatesPerTimestep());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbCollisionSubsteps, ds, ds.getNbCollisionSubsteps());

	// Shape is now added in NpDeformableSurface::attachShape.
	// Here we only stream vertex count for reference.
	const PxShape* shape = static_cast<const NpDeformableSurface&>(ds).getShapeInternal();
	if (shape)
	{
		const PxGeometry& geom = shape->getGeometry();
		if (geom.getType() == PxGeometryType::eTRIANGLEMESH)
		{
			const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
			if (triGeom.triangleMesh)
			{
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, collisionMesh, ds, triGeom.triangleMesh);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbVertices, ds, triGeom.triangleMesh->getNbVertices());
			}
		}
	}
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDeformableSurfaceAttributes(const physx::PxDeformableSurface& ds)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDeformableSurfaceAttributes(pvdWriter, pvdRegData, ds);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamDeformableSurface(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurface& ds)
{
	const PxActor& a = ds;
	PX_ASSERT(&a == &ds);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, ds);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eDEFORMABLE_SURFACE);

	streamDeformableSurfaceAttributes(pvdWriter, pvdRegData, ds);
}

static bool reportGpuCaptureFailure()
{
	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
		"OmniPVD GPU payload readback failed; recording stopped.");
	OmniPvdPxSampler::getInstance()->stopSampling();
	return false;
}

static bool readDeformableBuffer(const PxDeformableBody& body, const PxVec4* deviceData, PxU32 count)
{
	if (!count) return true;
	if (!deviceData || count > PX_MAX_U32 / sizeof(PxVec4)) return reportGpuCaptureFailure();
	PxCudaContextManager* context = body.getCudaContextManager();
	if (!context || !context->contextIsValid()) return reportGpuCaptureFailure();
	samplerInternals->mGpuReadbackScratch.resize(count);
	const PxU32 bytes = count * sizeof(PxVec4);
	PxScopedCudaLock lock(*context);
	if (context->getCudaContext()->memcpyDtoH(samplerInternals->mGpuReadbackScratch.begin(),
		reinterpret_cast<CUdeviceptr>(deviceData), bytes)) return reportGpuCaptureFailure();
	if (NpScene* scene = NpActor::getNpSceneFromActor(body))
		scene->getScScene().getSimulationController()->recordOmniPvdReadbackBytes(bytes);
	return true;
}

static void streamDeformableBounds(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registration,
	const PxDeformableBody& body, PxU32 count)
{
	if (!count) return;
	PxBounds3 bounds = PxBounds3::empty();
	for (PxU32 i = 0; i < count; ++i)
	{
		const PxVec3 position = samplerInternals->mGpuReadbackScratch[i].getXYZ();
		// Raw invalid coordinates remain recorded. They cannot define new bounds.
		if (!position.isFinite()) return;
		bounds.include(position);
	}
	bounds.scaleFast(1.01f);
	OMNI_PVD_SET_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds,
		static_cast<const PxActor&>(body), bounds);
}

bool OmniPvdPxSampler::streamDeformableState(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registration,
	const PxDeformableBody& body)
{
	if (!isSampling()) return false;
	if (!captureActor(body)) return true;
	const bool positions = NpOmniPvdSampling();
	const bool velocities = NpOmniPvdSampling();
	const bool legacy = !captureIsSelective();
	if (body.getConcreteType() == PxConcreteType::eDEFORMABLE_VOLUME)
	{
		const NpDeformableVolume& volume = static_cast<const NpDeformableVolume&>(body);
		if (!samplerInternals->mTakingSnapshot) streamDeformableVolumeAttributes(writer, registration, volume);
		if (!volume.getShapeInternal()) return true;
		const Dy::DeformableVolumeCore& core = volume.getCore().getCore();
		const PxTetrahedronMesh* collision = volume.getCollisionMesh();
		const PxTetrahedronMesh* simulation = volume.getSimulationMesh();
		if (collision && positions)
		{
			const PxU32 count = collision->getNbVertices();
			if (!readDeformableBuffer(body, core.positionInvMass, count)) return false;
			const PxReal* data = reinterpret_cast<const PxReal*>(samplerInternals->mGpuReadbackScratch.begin());
			if (!legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, collisionPositions, volume, data, count * 4); }
			if (legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, positions, *collision, data, count * 4); }
			streamDeformableBounds(writer, registration, body, count);
		}
		if (simulation)
		{
			const PxU32 count = simulation->getNbVertices();
			if (positions)
			{
				if (!readDeformableBuffer(body, core.simPositionInvMass, count)) return false;
				const PxReal* data = reinterpret_cast<const PxReal*>(samplerInternals->mGpuReadbackScratch.begin());
				if (!legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, simulationPositions, volume, data, count * 4); }
				if (legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, positions, *simulation, data, count * 4); }
			}
			if (velocities)
			{
				if (!readDeformableBuffer(body, core.simVelocity, count)) return false;
				const PxReal* data = reinterpret_cast<const PxReal*>(samplerInternals->mGpuReadbackScratch.begin());
				if (!legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, velocities, volume, data, count * 4); }
				if (legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, velocities, *simulation, data, count * 4); }
			}
		}
	}
	else
	{
		const NpDeformableSurface& surface = static_cast<const NpDeformableSurface&>(body);
		if (!samplerInternals->mTakingSnapshot) streamDeformableSurfaceAttributes(writer, registration, surface);
		const NpShape* shape = surface.getShapeInternal();
		if (!shape) return true;
		const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry()).triangleMesh;
		if (!mesh) return true;
		const PxU32 count = mesh->getNbVertices();
		const Dy::DeformableSurfaceCore& core = surface.getCore().getCore();
		if (positions)
		{
			if (!readDeformableBuffer(body, core.positionInvMass, count)) return false;
			const PxReal* data = reinterpret_cast<const PxReal*>(samplerInternals->mGpuReadbackScratch.begin());
			if (!legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, positions, surface, data, count * 4); }
			if (legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, positions, *mesh, data, count * 4); }
			streamDeformableBounds(writer, registration, body, count);
		}
		if (velocities)
		{
			if (!readDeformableBuffer(body, core.velocity, count)) return false;
			const PxReal* data = reinterpret_cast<const PxReal*>(samplerInternals->mGpuReadbackScratch.begin());
			if (!legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, velocities, surface, data, count * 4); }
			if (legacy) { OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, velocities, *mesh, data, count * 4); }
		}
	}
	return true;
}

static PxU32 particleReadFlags(const PxParticleBuffer& buffer)
{
	PxU32 flags = PxsParticleBuffer::ePOSITION | PxsParticleBuffer::eVELOCITY | PxsParticleBuffer::ePHASE;
	if (NpOmniPvdRecordSecondaryData() && buffer.is<PxParticleAndDiffuseBuffer>())
		flags |= PxsParticleBuffer::eDIFFUSE_POSITION | PxsParticleBuffer::eDIFFUSE_VELOCITY;
	return flags;
}

static PxsParticleBuffer& particleBufferCore(const PxParticleBuffer& buffer, NpPBDParticleSystem*& system)
{
	if (buffer.is<PxParticleAndDiffuseBuffer>())
	{
		const NpParticleAndDiffuseBuffer& npBuffer = static_cast<const NpParticleAndDiffuseBuffer&>(buffer);
		system = npBuffer.mParticleSystem;
		return *npBuffer.mGpuBuffer;
	}
	const NpParticleBuffer& npBuffer = static_cast<const NpParticleBuffer&>(buffer);
	system = npBuffer.mParticleSystem;
	return *npBuffer.mGpuBuffer;
}

static void trimParticleMirror(const PxParticleBuffer& buffer)
{
	NpPBDParticleSystem* system;
	PxsParticleBuffer& core = particleBufferCore(buffer, system);
	core.trimOmniPvdData(captureBase(buffer) ? particleReadFlags(buffer) : 0);
	core.setOmniPvdPhasesChanged(true);
}

bool OmniPvdPxSampler::streamParticleBufferState(OmniPvdWriter* writer,
	const OmniPvdPxCoreRegistrationData* registration, const PxParticleBuffer& buffer)
{
	if (!isSampling()) return false;
	if (!captureBase(buffer)) return true;
	PxU32 flags = particleReadFlags(buffer);
	NpPBDParticleSystem* system;
	PxsParticleBuffer* gpuBuffer = &particleBufferCore(buffer, system);
	if (!samplerInternals->mTakingSnapshot && !gpuBuffer->getOmniPvdPhasesChanged())
		flags &= ~PxU32(PxsParticleBuffer::ePHASE);
	const PxParticleAndDiffuseBuffer* diffuse = buffer.is<PxParticleAndDiffuseBuffer>();
	NpScene* scene = system ? system->getNpScene() : NULL;
	if (scene && captureInsert(samplerInternals->mSyncedParticleScenes, scene))
		scene->getScScene().getSimulationController()->syncParticleData();
	const PxU32 count = gpuBuffer->getNbActiveParticles();
	if (count > PX_MAX_U32 / sizeof(PxVec4)) return reportGpuCaptureFailure();

	PxU64 bytes = 0;
	const bool success = gpuBuffer->readOmniPvdData(flags, bytes);
	if (scene) scene->getScScene().getSimulationController()->recordOmniPvdReadbackBytes(bytes);
	if (!success) return reportGpuCaptureFailure();
	if (flags & PxsParticleBuffer::ePOSITION)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, positionInvMasses, buffer,
			reinterpret_cast<const PxReal*>(gpuBuffer->getPositionInvMassesH()), count * 4);
	}
	if (flags & PxsParticleBuffer::eVELOCITY)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, velocities, buffer,
			reinterpret_cast<const PxReal*>(gpuBuffer->getVelocitiesH()), count * 4);
	}
	if (flags & PxsParticleBuffer::ePHASE)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, phases, buffer, gpuBuffer->getPhasesH(), count);
		gpuBuffer->setOmniPvdPhasesChanged(false);
	}
	OMNI_PVD_SET_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, flatListStartIndex, buffer, gpuBuffer->getFlatListStartIndex());
	if (diffuse)
	{
		const PxsParticleAndDiffuseBuffer& gpuDiffuse = static_cast<const PxsParticleAndDiffuseBuffer&>(*gpuBuffer);
		const PxU32 diffuseCount = gpuDiffuse.getOmniPvdDiffuseCount();
		if (flags & PxsParticleBuffer::eDIFFUSE_POSITION)
		{
			OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, diffusePositionLifeTime, *diffuse,
				reinterpret_cast<const PxReal*>(gpuDiffuse.getDiffusePositionLifeTimeH()), diffuseCount * 4);
		}
		if (flags & PxsParticleBuffer::eDIFFUSE_VELOCITY)
		{
			OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, diffuseVelocities, *diffuse,
				reinterpret_cast<const PxReal*>(gpuDiffuse.getDiffuseVelocitiesH()), diffuseCount * 4);
		}
	}
	return true;
}

bool OmniPvdPxSampler::streamParticleSystemState(OmniPvdWriter* writer,
	const OmniPvdPxCoreRegistrationData* registration, const PxPBDParticleSystem& system)
{
	const NpPBDParticleSystem& npSystem = static_cast<const NpPBDParticleSystem&>(system);
	for (PxU32 i = 0; i < npSystem.mParticleBuffers.size(); ++i)
		if (!streamParticleBufferState(writer, registration, *npSystem.mParticleBuffers[i])) return false;
	for (PxU32 i = 0; i < npSystem.mParticleDiffuseBuffers.size(); ++i)
		if (!streamParticleBufferState(writer, registration, *npSystem.mParticleDiffuseBuffers[i])) return false;
	if (captureActor(system) && system.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(writer, registration, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds,
			static_cast<const PxActor&>(system), system.getWorldBounds());
	}
	return true;
}

void OmniPvdPxSampler::beginSceneGpuReadback(const PxScene& scene)
{
	captureErase(samplerInternals->mSyncedParticleScenes, &scene);
}

#endif

void streamArticulationJoint(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationJointReducedCoordinate& jointRef)
{
	const PxU32 degreesOfFreedom = PxArticulationAxis::eCOUNT;

	// make sure size matches the size used in the PVD description
	PX_ASSERT(sizeof(PxArticulationMotion::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());
	PX_ASSERT(sizeof(PxArticulationDriveType::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());

	PxArticulationJointType::Enum jointType = jointRef.getJointType();
	const PxArticulationLink* parentPxLinkPtr = &jointRef.getParentArticulationLink();
	const PxArticulationLink* childPxLinkPtr = &jointRef.getChildArticulationLink();
	PxArticulationMotion::Enum motions[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		motions[ax] = jointRef.getMotion(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal armatures[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		armatures[ax] = jointRef.getArmature(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal positions[degreesOfFreedom] = {};
	for (PxU32 ax = 0; NpOmniPvdSampling() && !captureActorUsesDirectGPU(jointRef.getChildArticulationLink()) && ax < degreesOfFreedom; ++ax)
		positions[ax] = jointRef.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal velocitys[degreesOfFreedom] = {};
	for (PxU32 ax = 0; NpOmniPvdSampling() && !captureActorUsesDirectGPU(jointRef.getChildArticulationLink()) && ax < degreesOfFreedom; ++ax)
		velocitys[ax] = jointRef.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	const char* concreteTypeName = jointRef.getConcreteTypeName();
	PxU32 concreteTypeNameLen = PxU32(strnlen(concreteTypeName, OMNI_PVD_MAX_STRING_LENGTH)) + 1;
	PxReal lowlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		lowlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).low;
	PxReal highlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		highlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).high;
	PxReal stiffnesss[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		stiffnesss[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).stiffness;
	PxReal dampings[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		dampings[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).damping;
	PxReal maxefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxefforts[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.maxEffort;
	PxReal maxactuatorvelocities[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxactuatorvelocities[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.maxActuatorVelocity;
	PxReal velocitydependentresistances[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		velocitydependentresistances[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.velocityDependentResistance;
	PxReal speedeffortgradients[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		speedeffortgradients[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.speedEffortGradient;
	PxArticulationDriveType::Enum drivetypes[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetypes[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).driveType;
	PxReal drivetargets[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetargets[ax] = jointRef.getDriveTarget(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal drivevelocitys[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivevelocitys[ax] = jointRef.getDriveVelocity(static_cast<PxArticulationAxis::Enum>(ax));

	PxReal staticfrictionefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		staticfrictionefforts[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).staticFrictionEffort;
	PxReal dynamicfrictionefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		dynamicfrictionefforts[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).dynamicFrictionEffort;
	PxReal viscousFrictionCoefficients[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		viscousFrictionCoefficients[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).viscousFrictionCoefficient;
	PxReal maxJointDofV[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxJointDofV[ax] = jointRef.getMaxJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	// Current parent/child frames: normally streamed via the setParentPose/setChildPose
	// hooks, but emitted here too so the create-time stream (and a full-state snapshot)
	// carries the current joint frames without depending on a later setter call.
	const physx::PxTransform jointParentPose = jointRef.getParentPose();
	const physx::PxTransform jointChildPose = jointRef.getChildPose();

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointRef);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, type, jointRef, jointType);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentLink, jointRef, parentPxLinkPtr);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childLink, jointRef, childPxLinkPtr);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentTranslation, jointRef, jointParentPose.p);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentRotation, jointRef, jointParentPose.q);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childTranslation, jointRef, jointChildPose.p);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childRotation, jointRef, jointChildPose.q);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, motion, jointRef, motions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, armature, jointRef, armatures, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, staticFrictionEffort,
								jointRef, staticfrictionefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, dynamicFrictionEffort, jointRef, dynamicfrictionefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
								viscousFrictionCoefficient, jointRef, viscousFrictionCoefficients, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
								maxJointDofVelocity, jointRef, maxJointDofV, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, jointRef, positions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, jointRef, velocitys, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, concreteTypeName, jointRef, concreteTypeName, concreteTypeNameLen);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitLow, jointRef, lowlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitHigh, jointRef, highlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveStiffness, jointRef, stiffnesss, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveDamping, jointRef, dampings, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxEffort, jointRef, maxefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxActuatorVelocity, jointRef, maxactuatorvelocities, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocityDependentResistance, jointRef, velocitydependentresistances, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveSpeedEffortGradient, jointRef, speedeffortgradients, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveType, jointRef, drivetypes, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, jointRef, drivetargets, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, jointRef, drivevelocitys, degreesOfFreedom);
}

void streamArticulationLink(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationLink& al)
{
	const PxActor& a = al;
	PX_ASSERT(&a == &al);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, al);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eARTICULATION_LINK);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, articulation, al, &al.getArticulation());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, CFMScale, al, al.getCfmScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJointDOF, al, al.getInboundJointDof());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJoint, al, al.getInboundJoint());

	streamRigidBodyAttributes(pvdWriter, pvdRegData, al);
}

void streamArticulationMimicJoint(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationMimicJoint& mj)
{
	const NpArticulationMimicJoint& np = static_cast<const NpArticulationMimicJoint&>(mj);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, mj);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, articulation, mj, &mj.getArticulation());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, jointA, mj, &np.getJointA());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, jointB, mj, &np.getJointB());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, axisA, mj, np.getAxisA());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, axisB, mj, np.getAxisB());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, gearRatio, mj, mj.getGearRatio());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, offset, mj, mj.getOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, naturalFrequency, mj, mj.getNaturalFrequency());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, dampingRatio, mj, mj.getDampingRatio());
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamArticulationMimicJoint(const physx::PxArticulationMimicJoint& mj)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationMimicJoint(pvdWriter, pvdRegData, mj);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamArticulation(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationReducedCoordinate& art)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
	PxU32 solverIterations[2]; art.getSolverIterationCounts(solverIterations[0], solverIterations[1]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, positionIterations, art, solverIterations[0]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, velocityIterations, art, solverIterations[1]);

	const NpScene* npScene = static_cast<const NpScene*>(art.getScene());

	if (npScene)
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, art, art.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, art, false);
	}

	// Getters don't issue warnings, just return values
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, sleepThreshold, art, art.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, stabilizationThreshold, art, art.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, wakeCounter, art, art.getWakeCounter());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, art, art.getWorldBounds());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, articulationFlags, art, art.getArticulationFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, dofs, art, art.getDofs());
}

void streamAggregate(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxAggregate& agg)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
	PxU32 actorCount = agg.getNbActors();
	for (PxU32 i = 0; i < actorCount; ++i)
	{
		PxActor* a; agg.getActors(&a, 1, i);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, actors, agg, *a);
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, selfCollision, agg, agg.getSelfCollision());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, environmentID, agg, agg.getEnvironmentID());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, maxNbShapes, agg, agg.getMaxNbShapes());
	PxScene* scene = static_cast<const NpAggregate&>(agg).getNpScene();  // because PxAggregate::getScene() is not marked const
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, scene, agg, scene);
}

void streamPBDMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, friction, m, m.getFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, damping, m, m.getDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, adhesion, m, m.getAdhesion());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, gravityScale, m, m.getGravityScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, adhesionRadiusScale, m, m.getAdhesionRadiusScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, viscosity, m, m.getViscosity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, vorticityConfinement, m, m.getVorticityConfinement());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, surfaceTension, m, m.getSurfaceTension());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, cohesion, m, m.getCohesion());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, lift, m, m.getLift());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, drag, m, m.getDrag());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, CFLCoefficient, m, m.getCFLCoefficient());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, particleFrictionScale, m, m.getParticleFrictionScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, particleAdhesionScale, m, m.getParticleAdhesionScale());
}

void streamFEMClothMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurfaceMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, youngsModulus, m, m.getYoungsModulus());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, poissons, m, m.getPoissons());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, elasticityDamping, m, m.getElasticityDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, thickness, m, m.getThickness());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, bendingStiffness, m, m.getBendingStiffness());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, bendingDamping, m, m.getBendingDamping());
}

void streamFEMSoBoMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolumeMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, youngsModulus, m, m.getYoungsModulus());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, poissons, m, m.getPoissons());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, elasticityDamping, m, m.getElasticityDamping());
}

void streamMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, flags, m, m.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, frictionCombineMode, m, m.getFrictionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitutionCombineMode, m, m.getRestitutionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, staticFriction, m, m.getStaticFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitution, m, m.getRestitution());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, damping, m, m.getDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dampingCombineMode, m, m.getDampingCombineMode());
}

// Explicit form (the caller already holds a write scope), used by the full-state snapshot's streamShape.
void streamShapeMaterials(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxShape& shape, physx::PxMaterial* const* mats, physx::PxU32 nbrMaterials)
{
	if (!NpOmniPvdSampling() || !captureBase(shape)) return;
	if (captureIsSelective())
		for (PxU32 i = 0; i < nbrMaterials; ++i)
		{
			captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(mats[i]));
			ensureCaptureObject(pvdWriter, pvdRegData, *mats[i]);
		}
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, materials, shape, mats, nbrMaterials);
}

// Scope-opening form, used live by NpShape::setMaterialsInternal (template instantiation for PxMaterial).
void streamShapeMaterials(const physx::PxShape& shape, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamShapeMaterials(pvdWriter, pvdRegData, shape, mats, nbrMaterials);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxDeformableSurfaceMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxDeformableVolumeMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}


void streamShape(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxShape& shape)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, isExclusive, shape, shape.isExclusive());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, geom, shape, &shape.getGeometry());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, contactOffset, shape, shape.getContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, restOffset, shape, shape.getRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, densityForFluid, shape, shape.getDensityForFluid());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, torsionalPatchRadius, shape, shape.getTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, minTorsionalPatchRadius, shape, shape.getMinTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shapeFlags, shape, shape.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, simulationFilterData, shape, shape.getSimulationFilterData());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, queryFilterData, shape, shape.getQueryFilterData());

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, localPose, shape, shape.getLocalPose());


	// Stream shape materials. Try deformable-specific getters first to get the
	// correct pointer values (multiple inheritance can adjust PxMaterial* pointers
	// away from the original PxDeformable*Material* address used in OVD registration).
	const int nbrMaterials = shape.getNbMaterials();
	void* tmpMaterials = PX_ALLOC(sizeof(void*) * nbrMaterials, "tmpMaterials");
	// Use internal shape core flags to determine the actual material type.
	// All public getMaterials overloads return materials regardless of type,
	// but pointer values differ due to multiple inheritance - we must use
	// the getter matching the registered OVD material type.
	const NpShape& npShape = static_cast<const NpShape&>(shape);
	const PxShapeCoreFlags coreFlags = npShape.getCore().mShapeCoreFlags;
	if (coreFlags & PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE)
	{
		physx::PxU32 nbrMats = shape.getDeformableVolumeMaterials(reinterpret_cast<PxDeformableVolumeMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}
	else if (coreFlags & PxShapeCoreFlag::eDEFORMABLE_SURFACE_SHAPE)
	{
		physx::PxU32 nbrMats = shape.getDeformableSurfaceMaterials(reinterpret_cast<PxDeformableSurfaceMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}
	else
	{
		physx::PxU32 nbrMats = shape.getMaterials(reinterpret_cast<PxMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}

	PX_FREE(tmpMaterials);
}

void streamBVH(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBVH& bvh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
}

void streamDeVoMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolumeMesh& mesh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, mesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, collisionMesh, mesh, mesh.getCollisionMesh());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, simulationMesh, mesh, mesh.getSimulationMesh());
}

void streamTetMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTetrahedronMesh& mesh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, mesh);
	if (!NpOmniPvdSampling()) return;
	//this gets done at the bottom now
	const PxU32 tetrahedronCount = mesh.getNbTetrahedrons();
	const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & physx::PxTetrahedronMeshFlag::e16_BIT_INDICES;
	const void* indexBuffer = mesh.getTetrahedrons();
	const PxVec3* vertexBuffer = mesh.getVertices();
	const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
	const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
	//TODO: not needed to copy this
	const PxU32 nbrVerts = mesh.getNbVertices();
	const PxU32 nbrTets = mesh.getNbTetrahedrons();
	float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
	PxU32 vertIndex = 0;
	for (PxU32 v = 0; v < nbrVerts; v++)
	{
		tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
		tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
		tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
		vertIndex += 3;
	}
	PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTets * 4), "tmpIndices");
	const PxU32 totalIndexCount = tetrahedronCount * 4;
	if (has16BitIndices)
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = shortIndices[i];
		}
	}
	else
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = intIndices[i];
		}
	}
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tets, mesh, tmpIndices, 4 * nbrTets);
	PX_FREE(tmpVerts);
	PX_FREE(tmpIndices);
}

void streamTriMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTriangleMesh& mesh)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdTriMesh))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, mesh);
		if (!NpOmniPvdSampling()) return;
		//this gets done at the bottom now
		const PxU32 triangleCount = mesh.getNbTriangles();
		const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
		const void* indexBuffer = mesh.getTriangles();
		const PxVec3* vertexBuffer = mesh.getVertices();
		const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
		const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
		//TODO: not needed to copy this
		const PxU32 nbrVerts = mesh.getNbVertices();
		const PxU32 nbrTris = mesh.getNbTriangles();
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		PxU32 vertIndex = 0;
		for (PxU32 v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
			tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
			tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
			vertIndex += 3;
		}
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTris * 3), "tmpIndices");
		const PxU32 totalIndexCount = triangleCount * 3;
		if (has16BitIndices)
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = shortIndices[i];
			}
		}
		else
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = intIndices[i];
			}
		}
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, tris, mesh, tmpIndices, 3 * nbrTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamTriMeshGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTriangleMeshGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, scale, g, g.scale.scale);
	streamTriMesh(pvdWriter, pvdRegData, *g.triangleMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, triangleMesh, g, g.triangleMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, meshFlags, g, g.meshFlags);
}

void OmniPvdPxSampler::streamSceneContacts(physx::NpScene& scene)
{
	NpOmniPvdSceneClient& client = scene.getSceneOvdClientInternal();
	const bool captured = NpOmniPvdCaptureScene(scene);
	if (!captured)
	{
		// Clear former contact arrays and counterparts after the last selected object leaves.
		bool update = client.contactArraysCaptured() &&
			captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(static_cast<const PxScene*>(&scene)));
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		for (PxHashMap<const PxActor*, const PxScene*>::Iterator it = samplerInternals->mContactActorScenes.getIterator(); !update && !it.done(); ++it)
			if (it->second == &scene) update = true;
		if (!update) return;
	}
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	if (captureIsSelective()) refreshCaptureContextActors(pvdWriter, pvdRegData, scene);
	PxsContactManagerOutputIterator outputIter;
	Sc::ContactIterator contactIter;
	const bool contactsValid = captured && NpOmniPvdRecordSecondaryData() && scene.areOmniPvdContactsValid();
	if (contactsValid)
	{
		scene.getScScene().initContactsIterator(contactIter, outputIter);
		contactIter.enableOmniPvdReadback(*scene.getScScene().getSimulationController());
	}
	Sc::ContactIterator::Pair* pair;
	PxU32 pairCount = 0;
	PxArray<const PxActor*> pairsActors;
	PxArray<PxU32> pairsContactCounts;
	PxArray<PxVec3> pairsContactPoints;
	PxArray<PxVec3> pairsContactNormals;
	PxArray<PxReal> pairsContactSeparations;
	PxArray<const PxShape*> pairsContactShapes;
	PxArray<PxU32> pairsContactFacesIndices;
	PxArray<PxReal> pairsContactImpulses;
	PxArray<PxU32> pairsFrictionAnchorCounts;
	PxArray<PxVec3> pairsFrictionAnchorPositions;
	PxArray<PxVec3> pairsFrictionAnchorNormals;
	PxArray<PxVec3> pairsFrictionAnchorImpulses;

	while (contactsValid && (pair = contactIter.getNextPair()) != NULL)
	{
		if (captureIsSelective() && !captureActor(*pair->getActor0()) && !captureActor(*pair->getActor1())) continue;
		PxU32 pairContactCount = 0;
		PxU32 pairFrictionAnchorCount = 0;
		Sc::Contact* contact = NULL;
		Sc::FrictionAnchor* anchor = NULL;
		bool firstContact = true;
		while ((contact = pair->getNextContact()) != NULL)
		{
			if (firstContact) {
				if (captureIsSelective())
				{
					samplerInternals->mRequiredContextActors.insert(pair->getActor0());
					samplerInternals->mRequiredContextActors.insert(pair->getActor1());
					ensureCaptureContext(pvdWriter, pvdRegData, *pair->getActor0());
					ensureCaptureContext(pvdWriter, pvdRegData, *pair->getActor1());
				}
				pairsActors.pushBack(pair->getActor0());
				pairsActors.pushBack(pair->getActor1());
				++pairCount;
				firstContact = false;
			}
			++pairContactCount;
			if (captureIsSelective())
			{
				ensureCaptureShape(pvdWriter, pvdRegData, *contact->shape0);
				ensureCaptureShape(pvdWriter, pvdRegData, *contact->shape1);
			}
			pairsContactPoints.pushBack(contact->point);
			pairsContactNormals.pushBack(contact->normal);
			pairsContactSeparations.pushBack(contact->separation);
			pairsContactShapes.pushBack(contact->shape0);
			pairsContactShapes.pushBack(contact->shape1);
			pairsContactFacesIndices.pushBack(contact->faceIndex0);
			pairsContactFacesIndices.pushBack(contact->faceIndex1);
			pairsContactImpulses.pushBack(contact->normalForce);
		}
		if (pairContactCount)
		{
			pairsContactCounts.pushBack(pairContactCount);
		}
		while ((anchor = pair->getNextFrictionAnchor()) != NULL)
		{
			++pairFrictionAnchorCount;
			pairsFrictionAnchorPositions.pushBack(anchor->point);
			pairsFrictionAnchorNormals.pushBack(anchor->normal);
			pairsFrictionAnchorImpulses.pushBack(anchor->impulse);
		}
		if (pairContactCount)
		{
			pairsFrictionAnchorCounts.pushBack(pairFrictionAnchorCount);
		}

	}

	if (contactIter.hadOmniPvdReadbackError())
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "OmniPVD contact readback failed; recording stopped.");
		stopSampling();
		return;
	}

	// Scan sleeping pairs only for object selections; selected scenes already include all actors.
	if (contactsValid && captureIsSelective() &&
		!captureContains(samplerInternals->mCaptureSceneRoots, static_cast<const PxScene*>(&scene)))
	{
		Sc::Scene& scScene = scene.getScScene();
		Sc::ElementSimInteraction** overlaps = scScene.getInteractions(Sc::InteractionType::eOVERLAP);
		Sc::SleepingTouchIterator sleeping(overlaps,
			overlaps ? overlaps + scScene.getNbInteractions(Sc::InteractionType::eOVERLAP) : NULL);
		PxActor* actor0 = NULL;
		PxActor* actor1 = NULL;
		while (sleeping.getNextPair(actor0, actor1))
		{
			if (!captureActor(*actor0) && !captureActor(*actor1)) continue;
			samplerInternals->mRequiredContextActors.insert(actor0);
			samplerInternals->mRequiredContextActors.insert(actor1);
			ensureCaptureContext(pvdWriter, pvdRegData, *actor0);
			ensureCaptureContext(pvdWriter, pvdRegData, *actor1);
		}
	}

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairCount, scene, pairCount);
	const PxU32 actorCount = pairsActors.size();
	const PxActor** actors = actorCount ? pairsActors.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsActors, scene, actors, actorCount);
	PxU32 nbContactCount = pairsContactCounts.size();
	PxU32* contactCounts = nbContactCount ? pairsContactCounts.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactCounts, scene, contactCounts, nbContactCount);
	PxU32 contactPointFloatCount = pairsContactPoints.size() * 3;
	PxReal* contactPoints = contactPointFloatCount ? &pairsContactPoints[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactPoints, scene, contactPoints, contactPointFloatCount);
	PxU32 contactNormalFloatCount = pairsContactNormals.size() * 3;
	PxReal* contactNormals = contactNormalFloatCount ? &pairsContactNormals[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactNormals, scene, contactNormals, contactNormalFloatCount);
	PxU32 contactSeparationCount = pairsContactSeparations.size();
	PxReal* contactSeparations = contactSeparationCount ? pairsContactSeparations.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactSeparations, scene, contactSeparations, contactSeparationCount);
	PxU32 contactShapeCount = pairsContactShapes.size();
	const PxShape** contactShapes = contactShapeCount ? pairsContactShapes.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactShapes, scene, contactShapes, contactShapeCount);
	PxU32 contactFacesIndexCount = pairsContactFacesIndices.size();
	PxU32* contactFacesIndices = contactFacesIndexCount ? pairsContactFacesIndices.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactFacesIndices, scene, contactFacesIndices, contactFacesIndexCount);
	PxU32 contactImpulseCount = pairsContactImpulses.size();
	PxReal* contactImpulses = contactImpulseCount ? pairsContactImpulses.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactImpulses, scene, contactImpulses, contactImpulseCount);
	PxU32 nbFrictionAnchorCount = pairsFrictionAnchorCounts.size();
	PxU32* frictionAnchorCounts = nbFrictionAnchorCount ? pairsFrictionAnchorCounts.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorCounts, scene, frictionAnchorCounts, nbFrictionAnchorCount);
	PxU32 frictionAnchorPositionFloatCount = pairsFrictionAnchorPositions.size() * 3;
	PxReal* frictionAnchorPositions = frictionAnchorPositionFloatCount ? &pairsFrictionAnchorPositions[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorPositions, scene, frictionAnchorPositions, frictionAnchorPositionFloatCount);
	PxU32 frictionAnchorNormalFloatCount = pairsFrictionAnchorNormals.size() * 3;
	PxReal* frictionAnchorNormals = frictionAnchorNormalFloatCount ? &pairsFrictionAnchorNormals[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorNormals, scene, frictionAnchorNormals, frictionAnchorNormalFloatCount);
	PxU32 frictionAnchorImpulseFloatCount = pairsFrictionAnchorImpulses.size() * 3;
	PxReal* frictionAnchorImpulses = frictionAnchorImpulseFloatCount ? &pairsFrictionAnchorImpulses[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorImpulses, scene, frictionAnchorImpulses, frictionAnchorImpulseFloatCount);

	client.setContactArraysCaptured(captured);
	if (captureIsSelective()) retireCaptureContextActors(pvdWriter, pvdRegData, scene);
	OMNI_PVD_WRITE_SCOPE_END

}

OmniPvdPxSampler::OmniPvdPxSampler()
{
	samplerInternals = PX_NEW(OmniPvdSamplerInternals)();

	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	samplerInternals->mIsSampling = false;
	samplerInternals->mRecordSecondaryData = true;
	samplerInternals->mCaptureAllObjects = true;
	samplerInternals->mCaptureContextScenesDirty = true;
	samplerInternals->mTakingSnapshot = false;
	samplerInternals->mDependencyDepth = 0;
	samplerInternals->mRetiredHandleSession = 0;
}

OmniPvdPxSampler::~OmniPvdPxSampler()
{
	PxOmniPvd::ScopedExclusiveWriter writerLock(NpOmniPvdGetInstance());
	for (;;)
	{
		const PxArticulationReducedCoordinate* articulation = NULL;
		{
			physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
			PxHashMap<const PxArticulationReducedCoordinate*, PxArticulationCache*>::Iterator it =
				samplerInternals->mReactionCaches.getIterator();
			if (!it.done()) articulation = it->first;
		}
		if (!articulation) break;
		releaseCaptureReactionCache(*articulation);
	}
	PX_DELETE(samplerInternals);
}

bool OmniPvdPxSampler::stopSampling()
{
	if (!samplerInternals) return false;
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		samplerInternals->mIsSampling = false;
	}
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		samplerInternals->mJointActorStates.clear();
	}
	return true;
}

bool OmniPvdPxSampler::isTakingSnapshot() const
{
	return samplerInternals && samplerInternals->mTakingSnapshot;
}

bool OmniPvdPxSampler::isSampling()
{
	if (!samplerInternals) return false;
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	return samplerInternals->mIsSampling;
}

void OmniPvdPxSampler::buildCaptureObjectCatalog(PxHashMap<PxU64, const PxBase*>& catalog)
{
	NpFactory& factory = NpFactory::getInstance();
#define OMNI_PVD_CATALOG_OBJECTS(Owner, Type, countMethod, getMethod) \
	{ Type* batch[64]; \
	for (PxU32 offset = 0; offset < Owner.countMethod(); offset += 64) \
	{ const PxU32 count = Owner.getMethod(batch, 64, offset); \
	for (PxU32 i = 0; i < count; ++i) \
	{ const PxBase* object = batch[i]; catalog.insert(reinterpret_cast<PxU64>(object), object); } } }
	OMNI_PVD_CATALOG_OBJECTS(factory, PxRigidStatic, getNbRigidStatics, getRigidStatics)
	OMNI_PVD_CATALOG_OBJECTS(factory, PxRigidDynamic, getNbRigidDynamics, getRigidDynamics)
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_CATALOG_OBJECTS(factory, PxActor, getNbDeformableSurfaces, getDeformableSurfaces)
	OMNI_PVD_CATALOG_OBJECTS(factory, PxActor, getNbDeformableVolumes, getDeformableVolumes)
	OMNI_PVD_CATALOG_OBJECTS(factory, PxActor, getNbPBDParticleSystems, getPBDParticleSystems)
	OMNI_PVD_CATALOG_OBJECTS(factory, PxParticleBuffer, getNbParticleBuffers, getParticleBuffers)
#endif
	OMNI_PVD_CATALOG_OBJECTS(factory, PxArticulationReducedCoordinate, getNbArticulations, getArticulations)
#undef OMNI_PVD_CATALOG_OBJECTS
}

void OmniPvdPxSampler::getCaptureObjectIdentity(const PxBase& object,
	const PxScene*& scene, const PxBase*& owner, const char*& name)
{
	scene = NULL; owner = NULL; name = NULL;
	if (const PxActor* actor = object.is<PxActor>())
	{
		scene = actor->getScene(); name = actor->getName();
	}
	else if (const PxArticulationReducedCoordinate* articulation = object.is<PxArticulationReducedCoordinate>())
	{
		scene = articulation->getScene(); name = articulation->getName();
	}
#if PX_SUPPORT_GPU_PHYSX
	else if (const PxParticleBuffer* buffer = captureAsParticleBuffer(object))
	{
		name = buffer->getName();
		if (const NpPBDParticleSystem* system = captureParticleBufferOwner(*buffer))
		{
			owner = system; scene = system->getScene();
		}
	}
#endif
}

bool OmniPvdPxSampler::streamCaptureCatalog(bool initialize)
{
	NpOmniPvd& omniPvd = *samplerInternals->mPvdStream.mOmniPvdInstance;
	if (initialize) samplerInternals->mPvdStream.registerClasses();
	PxOmniPvd::ScopedExclusiveWriter lock(&omniPvd);
	OmniPvdWriter* writer = lock.getWriter();
	if (!writer) return false;
	const OmniPvdPxCoreRegistrationData& registration = samplerInternals->mPvdStream.mRegistrationData;
	if (initialize)
	{
		// Identity metadata is the only world-context object in a discovery-only session.
		// Direct generated setters intentionally bypass the disabled simulation-data hooks.
		captureClear(samplerInternals->mRecordedObjects);
		const OmniPvdObjectHandle metadata = reinterpret_cast<OmniPvdObjectHandle>(&omniPvd.mMetaData);
		captureInsert(samplerInternals->mRecordedObjects, metadata);
		registration.pvdPxOmniPvdMetaData.createInstance(*writer, OMNI_PVD_CONTEXT_HANDLE, metadata);
#define OMNI_PVD_CATALOG_METADATA(attribute) registration.pvdPxOmniPvdMetaData.set_##attribute##_(*writer, OMNI_PVD_CONTEXT_HANDLE, metadata, omniPvd.mMetaData.attribute)
		OMNI_PVD_CATALOG_METADATA(physxVersionMajor);
		OMNI_PVD_CATALOG_METADATA(physxVersionMinor);
		OMNI_PVD_CATALOG_METADATA(physxVersionBugfix);
		OMNI_PVD_CATALOG_METADATA(ovdIntegrationVersionMajor);
		OMNI_PVD_CATALOG_METADATA(ovdIntegrationVersionMinor);
#undef OMNI_PVD_CATALOG_METADATA
	}
	++omniPvd.mDiscoveryGeneration;
	for (PxU32 i = 0; i < omniPvd.mDiscoveryRows.size(); ++i)
		writer->destroyObject(OMNI_PVD_CAPTURE_CATALOG_CONTEXT, reinterpret_cast<OmniPvdObjectHandle>(&omniPvd.mDiscoveryRows[i]));
	omniPvd.mDiscoveryRows.clear();
	NpPhysics& physics = NpPhysics::getInstance();
	PxHashMap<PxU64, const PxBase*> catalog;
	buildCaptureObjectCatalog(catalog);
	// Each catalog object emits at most itself, its scene and its owner. Reserve before taking addresses.
	const PxU64 maxRows = PxU64(physics.getNbScenes()) + PxU64(catalog.size()) * 3;
	if (maxRows > PX_MAX_U32)
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "OmniPVD capture catalog exceeds the supported row count.");
		return false;
	}
	omniPvd.mDiscoveryRows.reserve(PxU32(maxRows));
	PxHashSet<PxU64> emitted;
	const OmniPvdPxCoreRegistrationData::PvdPxOmniPvdCaptureEntry& entry = registration.pvdPxOmniPvdCaptureEntry;
	const auto emit = [&](PxU64 target, const char* type, const char* name, PxU64 owner, PxU64 scene, bool canSelect)
	{
		if (!emitted.insert(target)) return;
		omniPvd.mDiscoveryRows.pushBack(0);
		const OmniPvdObjectHandle row = reinterpret_cast<OmniPvdObjectHandle>(&omniPvd.mDiscoveryRows.back());
		entry.createInstance(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row);
		entry.set_targetHandle_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, target);
		entry.set_typeName_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, type, PxU32(strnlen(type, OMNI_PVD_MAX_STRING_LENGTH)));
		PxU32 length = 0;
		if (name) while (length < 2048 && name[length]) ++length;
		entry.set_name_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, name ? name : "", length);
		entry.set_ownerHandle_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, owner);
		entry.set_sceneHandle_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, scene);
		entry.set_canSelect_(*writer, OMNI_PVD_CAPTURE_CATALOG_CONTEXT, row, canSelect);
	};
	for (PxU32 i = 0; i < physics.getNbScenes(); ++i)
	{
		PxScene* scene = NULL; physics.getScenes(&scene, 1, i);
		if (omniPvd.isCaptureDiscoveryScene(*scene))
			emit(reinterpret_cast<PxU64>(scene), "PxScene", scene->getName(), 0, reinterpret_cast<PxU64>(scene), true);
	}
	for (PxHashMap<PxU64, const PxBase*>::Iterator it = catalog.getIterator(); !it.done(); ++it)
	{
		const PxBase& object = *it->second;
		if (!omniPvd.isCaptureDiscoveryObject(object)) continue;
		const PxScene* scene; const PxBase* owner; const char* name;
		getCaptureObjectIdentity(object, scene, owner, name);
		const PxU64 sceneHandle = reinterpret_cast<PxU64>(scene);
		if (scene) emit(sceneHandle, "PxScene", scene->getName(), 0, sceneHandle, omniPvd.isCaptureDiscoveryScene(*scene));
		if (owner)
		{
			const PxScene* ownerScene; const PxBase* parent; const char* ownerName;
			getCaptureObjectIdentity(*owner, ownerScene, parent, ownerName);
			emit(reinterpret_cast<PxU64>(owner), owner->getConcreteTypeName(), ownerName,
				reinterpret_cast<PxU64>(ownerScene), reinterpret_cast<PxU64>(ownerScene), omniPvd.isCaptureDiscoveryObject(*owner));
		}
		emit(it->first, object.getConcreteTypeName(), name, owner ? reinterpret_cast<PxU64>(owner) : sceneHandle, sceneHandle, true);
	}
	// Catalog generations must not change PvdDom's recording timeline.
	registration.pvdPxOmniPvdMetaData.set_captureCatalogGeneration_(*writer, OMNI_PVD_CONTEXT_HANDLE,
		reinterpret_cast<OmniPvdObjectHandle>(&omniPvd.mMetaData), omniPvd.mDiscoveryGeneration);
	return !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
}

PxU32 OmniPvdPxSampler::validateCaptureDesc(const PxOmniPvdFilterDesc& desc) const
{
	if (!desc.isValid()) return 2;
	PxPhysics& physics = NpPhysics::getInstance();
	PxHashSet<const PxScene*> scenes;
	for (PxU32 i = 0; i < desc.nbScenes; ++i) scenes.insert(desc.scenes[i]);
	if (scenes.size())
	{
		PxU32 owned = 0;
		for (PxU32 j = 0; j < physics.getNbScenes(); ++j)
		{
			PxScene* scene = NULL;
			physics.getScenes(&scene, 1, j);
			owned += scenes.contains(scene);
		}
		if (owned != scenes.size()) return 2;
	}
	if (desc.nbObjects)
	{
		PxHashMap<PxU64, const PxBase*> catalog;
		buildCaptureObjectCatalog(catalog);
		for (PxU32 i = 0; i < desc.nbObjects; ++i)
		{
			if (!catalog.find(reinterpret_cast<PxU64>(desc.objects[i]))) return 2;
		}
	}
	return 0;
}

bool OmniPvdPxSampler::setCaptureDesc(const PxOmniPvdFilterDesc& desc)
{
	PxOmniPvd::ScopedExclusiveWriter writerLock(NpOmniPvdGetInstance());
	if (isSampling() || validateCaptureDesc(desc)) return false;
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	samplerInternals->mCaptureSceneRoots.clear();
	samplerInternals->mCaptureActorRoots.clear();
	samplerInternals->mCaptureArticulationRoots.clear();
	samplerInternals->mCaptureObjectRoots.clear();
	samplerInternals->mCaptureRootScenes.clear();
#if PX_SUPPORT_GPU_PHYSX
	samplerInternals->mCaptureBufferOwners.clear();
#endif
	for (PxU32 i = 0; i < desc.nbScenes; ++i) samplerInternals->mCaptureSceneRoots.insert(desc.scenes[i]);
	for (PxU32 i = 0; i < desc.nbObjects; ++i)
	{
		const PxBase& object = *desc.objects[i];
		// startSampling requires caller synchronization across all scenes.
		if (const PxActor* actor = object.is<PxActor>())
		{
			samplerInternals->mCaptureActorRoots.insert(actor);
			samplerInternals->mCaptureRootScenes[&object] = NpActor::getNpSceneFromActor(*actor);
		}
		else if (const PxArticulationReducedCoordinate* articulation = object.is<PxArticulationReducedCoordinate>())
		{
			samplerInternals->mCaptureArticulationRoots.insert(articulation);
			samplerInternals->mCaptureRootScenes[&object] = static_cast<const NpArticulationReducedCoordinate*>(articulation)->getNpScene();
		}
		else
		{
			samplerInternals->mCaptureObjectRoots.insert(&object);
#if PX_SUPPORT_GPU_PHYSX
			if (const PxParticleBuffer* buffer = captureAsParticleBuffer(object))
			{
				const NpPBDParticleSystem* system = captureParticleBufferOwner(*buffer);
				samplerInternals->mCaptureBufferOwners[buffer] = system;
				samplerInternals->mCaptureRootScenes[&object] = system ? system->getNpScene() : NULL;
			}
#endif
		}
	}
	samplerInternals->mRecordSecondaryData = desc.recordSecondaryData;
	samplerInternals->mCaptureAllObjects = !desc.nbScenes && !desc.nbObjects;
	samplerInternals->mCaptureContextScenesDirty = true;
	return true;
}

bool OmniPvdPxSampler::isCapturingAllObjects() const { return captureAllObjects(); }
bool OmniPvdPxSampler::isObjectRecorded(const PxBase& object) const { return captureBase(object); }

void OmniPvdPxSampler::prepareSceneJointState(const PxScene& scene)
{
	PxOmniPvd::ScopedExclusiveWriter writerLock(NpOmniPvdGetInstance());
	PxArray<const PxActor*>& scratch = samplerInternals->mJointActorScratch;
	scratch.clear();
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		for (PxHashMap<const PxActor*, OmniPvdJointActorState>::Iterator it =
			samplerInternals->mJointActorStates.getIterator(); !it.done(); ++it)
			if (it->second.mScene == &scene) scratch.pushBack(it->first);
		for (PxU32 i = 0; i < scratch.size(); ++i) samplerInternals->mJointActorStates.erase(scratch[i]);
	}
	if (!isSampling() || !(scene.getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)) return;
	if (!captureIsSelective() && !samplerInternals->mTakingSnapshot) return;
	PxConstraint* constraints[64];
	for (PxU32 offset = 0; offset < scene.getNbConstraints(); offset += 64)
	{
		const PxU32 count = scene.getConstraints(constraints, 64, offset);
		for (PxU32 i = 0; i < count; ++i)
		{
			if (!captureBase(*constraints[i])) continue;
			PxConstraintConnector* connector = static_cast<const NpConstraint*>(constraints[i])->getCore().getPxConnector();
			if (!connector) continue;
			PxRigidActor* actors[2] = {};
			constraints[i]->getActors(actors[0], actors[1]);
			for (PxU32 endpoint = 0; endpoint < 2; ++endpoint)
				if (actors[endpoint] && actors[endpoint]->getType() != PxActorType::eRIGID_STATIC &&
					NpActor::getNpSceneFromActor(*actors[endpoint]) == &scene)
				{
					physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
					samplerInternals->mJointActorStates[actors[endpoint]] = OmniPvdJointActorState(scene);
				}
		}
	}
}

bool OmniPvdPxSampler::isJointStateActor(const PxActor& actor) const
{
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	return samplerInternals->mJointActorStates.find(&actor) != NULL;
}

void OmniPvdPxSampler::storeJointActorPose(const PxActor& actor, const PxTransform& pose)
{
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	if (samplerInternals->mJointActorStates.find(&actor))
	{
		OmniPvdJointActorState& state = samplerInternals->mJointActorStates[&actor];
		state.mPose = pose;
		state.mValid |= OmniPvdJointActorState::ePOSE;
	}
}

void OmniPvdPxSampler::storeJointActorVelocity(const PxActor& actor, const PxVec3& velocity, bool angular)
{
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	if (samplerInternals->mJointActorStates.find(&actor))
	{
		OmniPvdJointActorState& state = samplerInternals->mJointActorStates[&actor];
		if (angular) state.mAngularVelocity = velocity;
		else state.mLinearVelocity = velocity;
		state.mValid |= static_cast<PxU8>(angular ? OmniPvdJointActorState::eANGULAR_VELOCITY : OmniPvdJointActorState::eLINEAR_VELOCITY);
	}
}

bool OmniPvdPxSampler::getJointActorPose(const PxRigidActor& actor, PxTransform& pose) const
{
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	const PxHashMap<const PxActor*, OmniPvdJointActorState>::Entry* entry = samplerInternals->mJointActorStates.find(&actor);
	if (!entry || entry->second.mScene != NpActor::getNpSceneFromActor(actor) ||
		!(entry->second.mValid & OmniPvdJointActorState::ePOSE)) return false;
	pose = entry->second.mPose;
	return true;
}

bool OmniPvdPxSampler::getJointActorVelocity(const PxRigidActor& actor, PxVec3& linear, PxVec3& angular) const
{
	physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
	const PxHashMap<const PxActor*, OmniPvdJointActorState>::Entry* entry = samplerInternals->mJointActorStates.find(&actor);
	const PxU8 velocityFlags = OmniPvdJointActorState::eLINEAR_VELOCITY | OmniPvdJointActorState::eANGULAR_VELOCITY;
	if (!entry || entry->second.mScene != NpActor::getNpSceneFromActor(actor) ||
		(entry->second.mValid & velocityFlags) != velocityFlags) return false;
	linear = entry->second.mLinearVelocity;
	angular = entry->second.mAngularVelocity;
	return true;
}

void OmniPvdPxSampler::resetCaptureObjects(OmniPvdWriter& writer)
{
	// The caller holds the writer. Stop frames before clearing their recorded membership,
	// then detach bookkeeping under the state lock. Stream writes and cache releases stay
	// outside that leaf lock.
	NpPhysics& physics = NpPhysics::getInstance();
	for (PxU32 i = 0; i < physics.getNbScenes(); ++i)
	{
		PxScene* scene = NULL;
		physics.getScenes(&scene, 1, i);
		if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(scene)))
			static_cast<NpScene*>(scene)->getSceneOvdClientInternal().stopLastFrame(writer);
	}
	PxArray<PxU64> recorded;
	PxArray<PxArticulationCache*> caches;
	{
		PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		recorded.reserve(samplerInternals->mRecordedObjects.size());
		for (PxHashSet<PxU64>::Iterator it = samplerInternals->mRecordedObjects.getIterator(); !it.done(); ++it)
			recorded.pushBack(*it);
		caches.reserve(samplerInternals->mReactionCaches.size());
		for (PxHashMap<const PxArticulationReducedCoordinate*, PxArticulationCache*>::Iterator it =
			samplerInternals->mReactionCaches.getIterator(); !it.done(); ++it) caches.pushBack(it->second);
		samplerInternals->mReactionCaches.clear();
		samplerInternals->mRecordedObjects.clear();
		samplerInternals->mStreamedObjects.clear();
		samplerInternals->mDependencies.clear();
		samplerInternals->mContactActors.clear();
		samplerInternals->mContactActorScenes.clear();
		samplerInternals->mJointActorStates.clear();
#if PX_SUPPORT_GPU_PHYSX
		samplerInternals->mSyncedParticleScenes.clear();
#endif
	}
	for (PxU32 i = 0; i < recorded.size(); ++i)
		writer.destroyObject(OMNI_PVD_CONTEXT_HANDLE, recorded[i]);
	for (PxU32 i = 0; i < caches.size(); ++i) caches[i]->release();
}

void OmniPvdPxSampler::clearRetiredCaptureHandles()
{
	NpOmniPvd* omniPvd = samplerInternals->mPvdStream.mOmniPvdInstance;
	const PxU64 session = omniPvd && omniPvd->mWriter && omniPvd->mWriter->getWriteStream() ?
		omniPvd->mWriter->getWriteStream()->getCaptureSessionId() : 0;
	PxMutex::ScopedLock lock(samplerInternals->mRetiredHandleMutex);
	if (session != samplerInternals->mRetiredHandleSession)
	{
		samplerInternals->mRetiredControlHandles.clear();
		samplerInternals->mRetiredHandleSession = session;
	}
}
bool OmniPvdPxSampler::isCaptureHandleRetired(PxU64 handle) const
{
	PxMutex::ScopedLock lock(samplerInternals->mRetiredHandleMutex);
	return samplerInternals->mRetiredControlHandles.contains(handle);
}

void OmniPvdPxSampler::updateConstraintOmniPvdProperties(const PxConstraint& constraint)
{
	if (!isObjectRecorded(constraint)) return;
	PxConstraintConnector* connector = static_cast<const NpConstraint&>(constraint).getCore().getPxConnector();
	if (!connector || captureContains(samplerInternals->mCreatingConnectors, connector)) return;
	PxConstraintOmniPvdState state;
	PxRigidActor* actors[2]; constraint.getActors(actors[0], actors[1]);
	for (PxU32 i = 0; i < 2; ++i)
	{
		const PxRigidActor* actor = actors[i];
		const PxRigidBody* body = actor ? actor->is<PxRigidBody>() : NULL;
		const bool onDevice = body && captureActorStateOnDevice(*actor);
		const bool devicePose = onDevice && getJointActorPose(*actor, state.actorGlobalPose[i]);
		// Selective captures and full snapshots read DirectGPU endpoints from the device, not stale host state.
		if (devicePose || (onDevice && captureIsSelective()))
		{
			state.actorPoseValid[i] = devicePose;
			state.actorVelocityValid[i] = getJointActorVelocity(*actor, state.actorLinearVelocity[i], state.actorAngularVelocity[i]);
		}
		else
		{
			if (actor) state.actorGlobalPose[i] = actor->getGlobalPose();
			if (body)
			{
				state.actorLinearVelocity[i] = body->getLinearVelocity();
				state.actorAngularVelocity[i] = body->getAngularVelocity();
			}
			state.actorPoseValid[i] = state.actorVelocityValid[i] = true;
		}
	}
	connector->updateOmniPvdProperties(state);
}

void OmniPvdPxSampler::beginConstraintCreation(const PxConstraintConnector& connector)
{
	captureInsert(samplerInternals->mCreatingConnectors, &connector);
}

void OmniPvdPxSampler::endConstraintCreation(const PxConstraintConnector& connector)
{
	captureErase(samplerInternals->mCreatingConnectors, &connector);
}

void OmniPvdPxSampler::streamArticulationReactions(OmniPvdWriter* writer,
	const OmniPvdPxCoreRegistrationData* registrationData, const PxArticulationReducedCoordinate& articulation)
{
	if (!captureArticulation(articulation)) return;
	const NpArticulationReducedCoordinate& npArticulation = static_cast<const NpArticulationReducedCoordinate&>(articulation);
	const NpScene* scene = npArticulation.getNpScene();
	if (!scene || scene->getScScene().getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API)) return;
	const bool reactions = NpOmniPvdRecordSecondaryData();
	const bool accelerations = NpOmniPvdSampling() &&
		scene->getScScene().getFlags().isSet(PxSceneFlag::eENABLE_BODY_ACCELERATIONS);
	if (!reactions && !accelerations) return;
	PxArticulationCache* cache = captureReactionCache(articulation);
	if (cache && cache->version != npArticulation.mCacheVersion)
	{
		releaseCaptureReactionCache(articulation);
		cache = NULL;
	}
	if (!cache)
	{
		cache = articulation.createCache();
		if (!cache) { stopSampling(); return; }
		physx::PxMutex::ScopedLock lock(samplerInternals->mCaptureStateMutex);
		samplerInternals->mReactionCaches.insert(&articulation, cache);
	}
	PxArticulationCacheFlags cacheFlags;
	if (reactions) cacheFlags |= PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE;
	if (accelerations) cacheFlags |= PxArticulationCacheFlag::eACCELERATION;
	articulation.copyInternalStateToCache(*cache, cacheFlags);
	const NpArticulationLink* const* links = npArticulation.getLinks();
	const PxU32 linkCount = npArticulation.getNbLinks();
	PxArray<PxU32>& dofStarts = samplerInternals->mArticulationDofStarts;
	if (accelerations)
	{
		dofStarts.resize(linkCount);
		PxMemZero(dofStarts.begin(), linkCount * sizeof(PxU32));
		for (PxU32 i = 1; i < linkCount; ++i)
			dofStarts[links[i]->getLinkIndex()] = links[i]->getInboundJointDof();
		PxU32 dofOffset = 0;
		for (PxU32 i = 1; i < linkCount; ++i)
		{
			const PxU32 dofCount = dofStarts[i];
			dofStarts[i] = dofOffset;
			dofOffset += dofCount;
		}
	}
	for (PxU32 i = 0; i < linkCount; ++i)
	{
		const PxArticulationJointReducedCoordinate* joint = links[i]->getInboundJoint();
		if (joint && reactions)
		{
			const PxSpatialForce& force = cache->linkIncomingJointForce[links[i]->getLinkIndex()];
			const OmniPvdJointReaction reaction = { force.force, force.torque };
			OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
				incomingJointForce, *joint, reaction);
		}
		if (joint && accelerations)
		{
			PxU32 dofOffset = dofStarts[links[i]->getLinkIndex()];
			PxReal values[PxArticulationAxis::eCOUNT] = {};
			for (PxU32 axis = 0; axis < PxArticulationAxis::eCOUNT; ++axis)
				if (joint->getMotion(static_cast<PxArticulationAxis::Enum>(axis)) != PxArticulationMotion::eLOCKED)
					values[axis] = cache->jointAcceleration[dofOffset++];
			OMNI_PVD_SET_ARRAY_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
				jointAcceleration, *joint, values, PxArticulationAxis::eCOUNT);
		}
	}
}

static const physx::PxU32 gOvdSnapshotBatch = 64;

// Re-emit a resource list using the snapshot's existing exclusive writer and registration data.
#define OVD_SNAPSHOT_PHYSICS_LIST(PxType, GetNbFn, GetFn)                                   \
	{                                                                                       \
		const physx::PxU32 _nb = physics.GetNbFn();                                         \
		PxType* _batch[gOvdSnapshotBatch];                                                  \
		for (physx::PxU32 _off = 0; _off < _nb; _off += gOvdSnapshotBatch)                  \
		{                                                                                   \
			const physx::PxU32 _got = physics.GetFn(_batch, gOvdSnapshotBatch, _off);       \
			for (physx::PxU32 _i = 0; _i < _got; ++_i)                                      \
				onObjectAdd(pvdWriter, pvdRegData, *_batch[_i]);                            \
		}                                                                                   \
	}

bool OmniPvdPxSampler::snapshotAll()
{
	if (!samplerInternals) return false;
	PxOmniPvd::ScopedExclusiveWriter snapshotLock(NpOmniPvdGetInstance());
	captureClear(samplerInternals->mRecordedObjects);
	captureClear(samplerInternals->mStreamedObjects);
	captureClear(samplerInternals->mDependencies);
	captureClear(samplerInternals->mContactActors);
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		samplerInternals->mContactActorScenes.clear();
	}
	samplerInternals->mRequiredContextActors.clear();
	samplerInternals->mContextActorScratch.clear();
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		samplerInternals->mJointActorStates.clear();
	}
#if PX_SUPPORT_GPU_PHYSX
	captureClear(samplerInternals->mSyncedParticleScenes);
#endif
	samplerInternals->mTakingSnapshot = true;

	// Ensure sampling is on: emission flows through onObjectAdd(), which early-returns
	// when sampling is off.
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		samplerInternals->mIsSampling = true;
	}

	physx::NpPhysics& npPhysics = physx::NpPhysics::getInstance();
	physx::PxPhysics& physics = static_cast<physx::PxPhysics&>(npPhysics);

	// (0) Register the full class/attribute schema on the bound stream, then emit the
	//     PxOmniPvdMetaData + PxPhysics singletons. initOmniPvd() does both, in that order.
	samplerInternals->mPvdStream.initOmniPvd();

	// (0c) Clear the cross-stream shared-mesh dedup so mesh vertex/index data re-emits.
	{
		physx::PxMutex::ScopedLock geomLock(samplerInternals->mSharedGeomsMutex);
		samplerInternals->mSharedMeshesMap.clear();
	}

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	// (1) Shared leaf resources first (materials + meshes), since shapes reference them.
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxMaterial,         getNbMaterials,         getMaterials)
#if PX_SUPPORT_GPU_PHYSX
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxDeformableSurfaceMaterial, getNbDeformableSurfaceMaterials, getDeformableSurfaceMaterials)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxDeformableVolumeMaterial,  getNbDeformableVolumeMaterials,  getDeformableVolumeMaterials)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxPBDMaterial,      getNbPBDMaterials,      getPBDMaterials)
#endif
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxConvexMesh,       getNbConvexMeshes,      getConvexMeshes)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxTriangleMesh,     getNbTriangleMeshes,    getTriangleMeshes)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxHeightField,      getNbHeightFields,      getHeightFields)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxTetrahedronMesh,  getNbTetrahedronMeshes, getTetrahedronMeshes)
	// Deformable volume meshes: the cooked wrapper bundling the collision + simulation tetrahedron
	// meshes emitted just above. There is no PxPhysics-level getter, so enumerate them from the
	// factory (which tracks them); emit after the tetrahedron meshes so the wrapper's mesh
	// references resolve. Deformable volumes are GPU-only, so the factory getters live behind
	// PX_SUPPORT_GPU_PHYSX.
#if PX_SUPPORT_GPU_PHYSX
	{
		physx::NpFactory& dvmFactory = physx::NpFactory::getInstance();
		const physx::PxU32 nbDvm = dvmFactory.getNbDeformableVolumeMeshes();
		physx::PxDeformableVolumeMesh* dvmBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbDvm; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = dvmFactory.getDeformableVolumeMeshes(dvmBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				onObjectAdd(pvdWriter, pvdRegData, *dvmBatch[i]);
		}
	}
#endif
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxBVH,              getNbBVHs,              getBVHs)

	// (2) Shapes (global, de-duplicated set), referenced by actors below.
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxShape,            getNbShapes,            getShapes)

	// Emit factory identities before scene membership; recorded handles prevent duplicates.
	{
		physx::NpFactory& factory = physx::NpFactory::getInstance();
#if PX_SUPPORT_GPU_PHYSX
		// Particle buffers before the particle systems, so the systems' particleBuffers edges resolve.
		const physx::PxU32 nbBufs = factory.getNbParticleBuffers();
		physx::PxParticleBuffer* bufBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbBufs; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getParticleBuffers(bufBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (bufBatch[i]) trimParticleMirror(*bufBatch[i]);
		}
		// Creating an owner can record its sibling buffers, so prepare every mirror first.
		for (physx::PxU32 off = 0; off < nbBufs; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getParticleBuffers(bufBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (bufBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *bufBatch[i]);
		}
#endif
		// Actors, enumerated per factory tracking set, exactly as from-start recording emits each:
		// onObjectAdd creates the object with its shapes / particleBuffers edges. Deformables additionally
		// need the PxDeformableBody.shapes edge re-emitted here (otherwise emitted only at attachShape; the
		// viewer reaches the collision/sim mesh through it).

		// Rigid statics, then rigid dynamics (each tracked in its own factory set).
		physx::PxRigidStatic* rigidStaticBatch[gOvdSnapshotBatch];
		const physx::PxU32 nbRigidStatics = factory.getNbRigidStatics();
		for (physx::PxU32 off = 0; off < nbRigidStatics; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getRigidStatics(rigidStaticBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (rigidStaticBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *rigidStaticBatch[i]);
		}
		physx::PxRigidDynamic* rigidDynamicBatch[gOvdSnapshotBatch];
		const physx::PxU32 nbRigidDynamics = factory.getNbRigidDynamics();
		for (physx::PxU32 off = 0; off < nbRigidDynamics; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getRigidDynamics(rigidDynamicBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				if (!rigidDynamicBatch[i])
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *rigidDynamicBatch[i]);
			}
		}
#if PX_SUPPORT_GPU_PHYSX
		physx::PxActor* actorBatch[gOvdSnapshotBatch]; // shared batch buffer for the GPU actor types below
		// Deformable surfaces, with the PxDeformableBody.shapes edge re-emitted.
		const physx::PxU32 nbDefSurf = factory.getNbDeformableSurfaces();
		for (physx::PxU32 off = 0; off < nbDefSurf; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getDeformableSurfaces(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxActor* a = actorBatch[i];
				if (!a)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *a);
				if (physx::PxShape* sh = static_cast<physx::PxDeformableSurface*>(a)->getShape())
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<physx::PxDeformableBody&>(*a), *sh)
				}
			}
		}
		// Deformable volumes, with the PxDeformableBody.shapes edge re-emitted.
		const physx::PxU32 nbDefVol = factory.getNbDeformableVolumes();
		for (physx::PxU32 off = 0; off < nbDefVol; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getDeformableVolumes(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxActor* a = actorBatch[i];
				if (!a)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *a);
				if (physx::PxShape* sh = static_cast<physx::PxDeformableVolume*>(a)->getShape())
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<physx::PxDeformableBody&>(*a), *sh)
				}
			}
		}
		// PBD particle systems.
		const physx::PxU32 nbPS = factory.getNbPBDParticleSystems();
		for (physx::PxU32 off = 0; off < nbPS; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getPBDParticleSystems(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (actorBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *actorBatch[i]);
		}
#endif
		// Articulations, with their links, then inbound joints, then mimic joints (referenced-before-referrer).
		const physx::PxU32 nbArts = factory.getNbArticulations();
		physx::PxArticulationReducedCoordinate* artBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbArts; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getArticulations(artBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxArticulationReducedCoordinate* art = artBatch[i];
				if (!art)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *art);
				streamArticulationName(pvdWriter, pvdRegData, *art, art->getName()); // re-emit the name (set live only via setName, not at create)
				const physx::PxU32 nbLinks = art->getNbLinks();
				physx::PxArticulationLink* linkBatch[gOvdSnapshotBatch];
				for (physx::PxU32 lo = 0; lo < nbLinks; lo += gOvdSnapshotBatch)
				{
					const physx::PxU32 gotL = art->getLinks(linkBatch, gOvdSnapshotBatch, lo);
					for (physx::PxU32 l = 0; l < gotL; ++l)
					{
						onObjectAdd(pvdWriter, pvdRegData, *linkBatch[l]);     // creates the link and adds it to PxArticulationReducedCoordinate.links
					}
					for (physx::PxU32 l = 0; l < gotL; ++l)
					{
						physx::PxArticulationJointReducedCoordinate* joint = linkBatch[l]->getInboundJoint();
						if (joint)
						{
							onObjectAdd(pvdWriter, pvdRegData, *joint);
							streamArticulationJointName(pvdWriter, pvdRegData, *joint, joint->getName()); // re-emit the name (set live only via setName)
						}
					}
				}
				const physx::PxU32 nbMimic = art->getNbMimicJoints();
				physx::PxArticulationMimicJoint* mimicBatch[gOvdSnapshotBatch];
				for (physx::PxU32 mo = 0; mo < nbMimic; mo += gOvdSnapshotBatch)
				{
					const physx::PxU32 gotM = art->getMimicJoints(mimicBatch, gOvdSnapshotBatch, mo);
					for (physx::PxU32 m = 0; m < gotM; ++m)
						streamArticulationMimicJoint(pvdWriter, pvdRegData, *mimicBatch[m]);
				}
				streamArticulationReactions(pvdWriter, pvdRegData, *art);
			}
		}
		// Aggregates.
		const physx::PxU32 nbAggs = factory.getNbAggregates();
		physx::PxAggregate* aggBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbAggs; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getAggregates(aggBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (aggBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *aggBatch[i]);
		}
	}
	OVD_SNAPSHOT_PHYSICS_LIST(PxConstraint, getNbConstraints, getConstraints)
	OMNI_PVD_WRITE_SCOPE_END

	// (4) Per scene: the scene object and its membership edges (which actors / articulations /
	//     aggregates belong to it), plus the direct-GPU-API retained body / link forces that live in
	//     device state. The objects were already created in (3); from-start recording likewise emits
	//     these memberships at add time, separately from object creation.
	const physx::PxU32 nbScenes = physics.getNbScenes();
	for (physx::PxU32 si = 0; si < nbScenes; ++si)
	{
		physx::PxScene* pxScene = NULL;
		if (physics.getScenes(&pxScene, 1, si) != 1 || pxScene == NULL)
			continue;
		physx::NpScene* npScene = static_cast<physx::NpScene*>(pxScene);
		npScene->getSceneOvdClientInternal().resetFrameId();
		if (!captureScene(*pxScene)) continue;

		// The PxScene object itself + its membership references, opening a clean
		// pre-sim frame 1 on the new stream.
		npScene->emitSceneStateToOmniPvd();
		npScene->emitSceneObjectListsToOmniPvd();
		npScene->streamRigidDynamicGPUForcesToOmniPvd(); // GPU direct-API retained forces (device-side state)
		npScene->streamArticulationGPUForcesToOmniPvd(); // GPU direct-API articulation link forces (device-side state)
		streamSceneContacts(*npScene);
		if (!npScene->streamGPUStateToOmniPvd(true)) break;
	}

	// If any write failed while emitting the snapshot (e.g. the late-attach reader went
	// away mid-snapshot, or a send timed out), report failure and clear the sampling flag
	// (set isSampling() back to false) so the caller can rebind a fresh stream and start again.
	const bool snapshotOk = samplerInternals->mPvdStream.dataWasWrittenSuccessfully() && isSampling();
	samplerInternals->mTakingSnapshot = false;
	if (!snapshotOk)
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		samplerInternals->mIsSampling = false;
	}
	return snapshotOk;
}

#undef OVD_SNAPSHOT_PHYSICS_LIST

void OmniPvdPxSampler::setOmniPvdInstance(physx::NpOmniPvd* omniPvdInstance)
{
	samplerInternals->mPvdStream.setOmniPvdInstance(omniPvdInstance);
}

void createGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxGeometry & pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		streamSphereGeometry(pvdWriter, pvdRegData, (const physx::PxSphereGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		streamCapsuleGeometry(pvdWriter, pvdRegData, (const physx::PxCapsuleGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		streamBoxGeometry(pvdWriter, pvdRegData, (const physx::PxBoxGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		streamTriMeshGeometry(pvdWriter, pvdRegData, (const physx::PxTriangleMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eTETRAHEDRONMESH:
	{
		const physx::PxTetrahedronMeshGeometry& g = static_cast<const physx::PxTetrahedronMeshGeometry&>(pxGeom);
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, g);
		if (g.tetrahedronMesh)
		{
			streamTetMesh(pvdWriter, pvdRegData, *g.tetrahedronMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, tetrahedronMesh, g, g.tetrahedronMesh);
		}
	}
	break;
	case physx::PxGeometryType::eCONVEXCORE:
	{
		streamConvexCoreGeometry(pvdWriter, pvdRegData, (const physx::PxConvexCoreGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		streamConvexMeshGeometry(pvdWriter, pvdRegData, (const physx::PxConvexMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		streamHeightFieldGeometry(pvdWriter, pvdRegData, (const physx::PxHeightFieldGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		streamPlaneGeometry(pvdWriter, pvdRegData, (const physx::PxPlaneGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		streamCustomGeometry(pvdWriter, pvdRegData, (const physx::PxCustomGeometry &)pxGeom);
	}
	break;
	default:
	break;
	}
}

void destroyGeometry(const physx::PxGeometry& pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, static_cast<const PxSphereGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, static_cast<const PxCapsuleGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, static_cast<const PxBoxGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, static_cast<const PxTriangleMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eTETRAHEDRONMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, static_cast<const PxTetrahedronMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, static_cast<const PxConvexMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, static_cast<const PxHeightFieldGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, static_cast<const PxPlaneGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, static_cast<const PxCustomGeometry&>(pxGeom));
	}
	break;
	default:
	break;
	}
}

void streamShapeUpdateGeometry(const physx::PxShape& shape)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	if ((captureIsSelective() && !captureBase(shape))) return;
	if (!samplerInternals->mTakingSnapshot) NpOmniPvdGetInstance()->notifyObjectCapture(shape);
	destroyGeometry(shape.getGeometry());
	++samplerInternals->mDependencyDepth;
	createGeometry(pvdWriter, pvdRegData, shape.getGeometry());
	--samplerInternals->mDependencyDepth;
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, geom, shape, &shape.getGeometry());
	OMNI_PVD_WRITE_SCOPE_END
}

static void ensureCaptureObject(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxBase& object)
{
	OmniPvdPxSampler::getInstance()->onObjectAdd(writer, registrationData, object);
}

#if PX_SUPPORT_GPU_PHYSX
static void ensureCaptureParticleMaterials(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const PxPBDParticleSystem& system)
{
	PxParticleMaterial* materials[64];
	for (PxU32 offset = 0; offset < system.getNbParticleMaterials(); offset += 64)
	{
		const PxU32 count = system.getParticleMaterials(materials, 64, offset);
		for (PxU32 i = 0; i < count; ++i)
		{
			captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(materials[i]));
			ensureCaptureObject(writer, registrationData, *materials[i]);
		}
	}
}
#endif

static void streamCaptureContextScene(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxActor& actor)
{
	if (const PxScene* scene = NpActor::getNpSceneFromActor(actor))
		// Contacts can admit context actors after the snapshot has emitted the scene's actor list.
		if (!samplerInternals->mTakingSnapshot || captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(scene)))
		{
			OMNI_PVD_ADD_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, *scene, actor);
		}
}

static void ensureCaptureContext(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxActor& actor)
{
	if (captureActor(actor)) { ensureCaptureObject(writer, registrationData, actor); return; }
#if PX_SUPPORT_GPU_PHYSX
	if (const PxPBDParticleSystem* system = actor.is<PxPBDParticleSystem>())
		ensureCaptureParticleMaterials(writer, registrationData, *system);
#endif
	bool admitted;
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		admitted = samplerInternals->mContactActors.insert(&actor);
		samplerInternals->mContactActorScenes[&actor] = NpActor::getNpSceneFromActor(actor);
	}
	if (admitted) invalidateCaptureScenes();
	captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(&actor));
	if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&actor)) && !admitted)
	{
		// Scene list passes omit links; add a context link once its scene has been recorded.
		if (samplerInternals->mTakingSnapshot && actor.is<PxArticulationLink>())
			streamCaptureContextScene(writer, registrationData, actor);
		return;
	}
	if (actor.is<PxRigidActor>()) ensureCaptureDependencies(writer, registrationData, actor);
	if (const PxRigidDynamic* dynamic = actor.is<PxRigidDynamic>())
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, *dynamic);
	}
	else if (const PxRigidStatic* rigidStatic = actor.is<PxRigidStatic>())
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxRigidStatic, *rigidStatic);
	}
	else if (const PxArticulationLink* link = actor.is<PxArticulationLink>())
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, *link);
	}
#if PX_SUPPORT_GPU_PHYSX
	else if (const PxPBDParticleSystem* system = actor.is<PxPBDParticleSystem>())
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, *system);
	}
#endif
	else
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxActor, actor);
	}
	OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, actor, actor.getType());
	streamActorName(writer, registrationData, actor, actor.getName());
	streamCaptureContextScene(writer, registrationData, actor);

	// Admit only this endpoint, without expanding to the articulation or its other links.
	if (const PxRigidDynamic* dynamic = actor.is<PxRigidDynamic>())
		streamRigidDynamicAttributes(writer, registrationData, *dynamic);
	else if (const PxRigidBody* body = actor.is<PxRigidBody>())
		streamRigidBodyAttributes(writer, registrationData, *body);
	else if (const PxRigidActor* rigid = actor.is<PxRigidActor>())
		streamRigidActorAttributes(writer, registrationData, *rigid);
}

static void refreshCaptureContextActors(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxScene& scene)
{
	PxHashSet<const PxActor*>& required = samplerInternals->mRequiredContextActors;
	required.clear();
	PxConstraint* constraints[64];
	for (PxU32 offset = 0; offset < scene.getNbConstraints(); offset += 64)
	{
		const PxU32 count = scene.getConstraints(constraints, 64, offset);
		for (PxU32 i = 0; i < count; ++i)
		{
			if (!captureConstraint(*constraints[i])) continue;
			PxRigidActor* actors[2] = {};
			constraints[i]->getActors(actors[0], actors[1]);
			for (PxU32 endpoint = 0; endpoint < 2; ++endpoint)
				if (actors[endpoint])
				{
					required.insert(actors[endpoint]);
					ensureCaptureContext(writer, registrationData, *actors[endpoint]);
				}
		}
	}
}

static void retireCaptureContextActors(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxScene& scene)
{
	PxArray<const PxActor*>& retired = samplerInternals->mContextActorScratch;
	retired.clear();
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		for (PxHashMap<const PxActor*, const PxScene*>::Iterator it = samplerInternals->mContactActorScenes.getIterator(); !it.done(); ++it)
			if (it->second == &scene && it->first->is<PxRigidActor>() && !captureActor(*it->first) &&
				!samplerInternals->mRequiredContextActors.contains(it->first)) retired.pushBack(it->first);
	}
	for (PxU32 i = 0; i < retired.size(); ++i)
	{
		const PxActor& actor = *retired[i];
		// A constraint with an endpoint outside every scene is absent from the scene's constraint list.
		const PxRigidActor& rigid = static_cast<const PxRigidActor&>(actor);
		PxConstraint* constraints[64];
		for (PxU32 offset = 0; offset < rigid.getNbConstraints(); offset += 64)
		{
			const PxU32 count = rigid.getConstraints(constraints, 64, offset);
			for (PxU32 j = 0; j < count; ++j)
				if (!constraints[j]->getScene() && captureConstraint(*constraints[j]) &&
					captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(constraints[j])))
					samplerInternals->mRequiredContextActors.insert(&actor);
		}
		if (samplerInternals->mRequiredContextActors.contains(&actor)) continue;
		// A joint to a selected actor that is in no scene admits this actor without recording its scene.
		if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&scene)))
			registrationData->pvdPxScene.removeFrom_actors_(*writer, OMNI_PVD_CONTEXT_HANDLE, scene, actor);
		const PxPhysics& physics = NpPhysics::getInstance();
		// These physics lists contain only previously selected actors; context-only removals are ignored.
		if (const PxRigidDynamic* dynamic = actor.is<PxRigidDynamic>())
		{
			OMNI_PVD_REMOVE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, *dynamic);
		}
		else if (const PxRigidStatic* rigidStatic = actor.is<PxRigidStatic>())
		{
			OMNI_PVD_REMOVE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, *rigidStatic);
		}
		OMNI_PVD_DESTROY_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxActor, actor);
	}
	// Refresh motion only: re-adding shapes would restart their recorded lifetimes.
	for (PxHashSet<const PxActor*>::Iterator it = samplerInternals->mRequiredContextActors.getIterator(); !it.done(); ++it)
	{
		const PxActor& actor = **it;
		const PxRigidBody* body = actor.is<PxRigidBody>();
		if (!body || captureActor(actor) || captureActorUsesDirectGPU(actor)) continue;
		OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, *body, body->getGlobalPose());
		OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, *body, body->getWorldBounds());
		OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, *body, body->getLinearVelocity());
		OMNI_PVD_SET_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, *body, body->getAngularVelocity());
	}
}

static void ensureCaptureShape(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxShape& shape)
{
	captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(&shape));
	ensureCaptureObject(writer, registrationData, shape);
}

static void ensureCaptureShapeMaterials(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxShape& shape)
{
	const PxShapeCoreFlags flags = static_cast<const NpShape&>(shape).getCore().mShapeCoreFlags;
	for (PxU32 offset = 0; offset < shape.getNbMaterials(); offset += 64)
	{
#define OMNI_PVD_CAPTURE_MATERIALS(Type, getter) \
		{ \
			Type* materials[64]; \
			const PxU32 count = shape.getter(materials, 64, offset); \
			for (PxU32 i = 0; i < count; ++i) \
			{ \
				captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(materials[i])); \
				ensureCaptureObject(writer, registrationData, *materials[i]); \
			} \
		}
		if (flags & PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE)
			OMNI_PVD_CAPTURE_MATERIALS(PxDeformableVolumeMaterial, getDeformableVolumeMaterials)
		else if (flags & PxShapeCoreFlag::eDEFORMABLE_SURFACE_SHAPE)
			OMNI_PVD_CAPTURE_MATERIALS(PxDeformableSurfaceMaterial, getDeformableSurfaceMaterials)
		else OMNI_PVD_CAPTURE_MATERIALS(PxMaterial, getMaterials)
#undef OMNI_PVD_CAPTURE_MATERIALS
	}
}

static void ensureCaptureDependencies(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const PxBase& object)
{
	if (const PxRigidActor* actor = object.is<PxRigidActor>())
	{
		PxShape* shapes[64];
		for (PxU32 offset = 0; offset < actor->getNbShapes(); offset += 64)
		{
			const PxU32 count = actor->getShapes(shapes, 64, offset);
			for (PxU32 i = 0; i < count; ++i) ensureCaptureShape(writer, registrationData, *shapes[i]);
		}
	}
	if (const PxShape* shape = object.is<PxShape>())
	{
		ensureCaptureShapeMaterials(writer, registrationData, *shape);
		if (shape->getGeometry().getType() == PxGeometryType::eHEIGHTFIELD)
		{
			const PxHeightField* mesh = static_cast<const PxHeightFieldGeometry&>(shape->getGeometry()).heightField;
			if (mesh)
			{
				captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(mesh));
				ensureCaptureObject(writer, registrationData, *mesh);
			}
		}
	}
	if (const PxConstraint* constraint = object.is<PxConstraint>())
	{
		PxRigidActor* first = NULL;
		PxRigidActor* second = NULL;
		constraint->getActors(first, second);
		if (first) ensureCaptureContext(writer, registrationData, *first);
		if (second) ensureCaptureContext(writer, registrationData, *second);
	}
	if (object.getConcreteType() == PxConcreteType::eDEFORMABLE_VOLUME_MESH)
	{
		const PxDeformableVolumeMesh* mesh = static_cast<const PxDeformableVolumeMesh*>(&object);
		const PxTetrahedronMesh* children[] = { mesh->getCollisionMesh(), mesh->getSimulationMesh() };
		for (PxU32 i = 0; i < 2; ++i)
			if (children[i])
			{
				captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(children[i]));
				ensureCaptureObject(writer, registrationData, *children[i]);
			}
	}
#if PX_SUPPORT_GPU_PHYSX
	if (const PxParticleBuffer* buffer = captureAsParticleBuffer(object))
		if (const NpPBDParticleSystem* owner = captureParticleBufferOwner(*buffer))
			ensureCaptureContext(writer, registrationData, *owner);
	if (const PxDeformableVolume* volume = object.is<PxDeformableVolume>())
	{
		const PxShape* shape = static_cast<const NpDeformableVolume*>(volume)->getShapeInternal();
		const PxTetrahedronMesh* meshes[] = { shape ? volume->getCollisionMesh() : NULL, volume->getSimulationMesh() };
		for (PxU32 i = 0; i < 2; ++i)
			if (meshes[i])
			{
				captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(meshes[i]));
				ensureCaptureObject(writer, registrationData, *meshes[i]);
			}
		if (shape) ensureCaptureShape(writer, registrationData, *shape);
		if (const PxTetrahedronMesh* mesh = volume->getSimulationMesh())
		{
			captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(mesh));
			ensureCaptureObject(writer, registrationData, *mesh);
		}
	}
	if (const PxDeformableSurface* surface = object.is<PxDeformableSurface>())
	{
		const PxShape* shape = static_cast<const NpDeformableSurface*>(surface)->getShapeInternal();
		if (shape && shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH)
			if (const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry()).triangleMesh)
			{
				captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(mesh));
				ensureCaptureObject(writer, registrationData, *mesh);
			}
		if (shape)
			ensureCaptureShape(writer, registrationData, *shape);
	}
	if (const PxPBDParticleSystem* system = object.is<PxPBDParticleSystem>())
	{
		ensureCaptureParticleMaterials(writer, registrationData, *system);
		const NpPBDParticleSystem& npSystem = static_cast<const NpPBDParticleSystem&>(*system);
		for (PxU32 i = 0; i < npSystem.mParticleBuffers.size(); ++i)
		{
			if (!captureBase(*npSystem.mParticleBuffers[i])) continue;
			captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(npSystem.mParticleBuffers[i]));
			ensureCaptureObject(writer, registrationData, *npSystem.mParticleBuffers[i]);
		}
		for (PxU32 i = 0; i < npSystem.mParticleDiffuseBuffers.size(); ++i)
		{
			if (!captureBase(*npSystem.mParticleDiffuseBuffers[i])) continue;
			captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(npSystem.mParticleDiffuseBuffers[i]));
			ensureCaptureObject(writer, registrationData, *npSystem.mParticleDiffuseBuffers[i]);
		}
	}
#endif
	if (const PxArticulationReducedCoordinate* articulation = object.is<PxArticulationReducedCoordinate>())
	{
		OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, *articulation);
		const NpArticulationReducedCoordinate& npArticulation = static_cast<const NpArticulationReducedCoordinate&>(*articulation);
		const NpArticulationLink* const* links = npArticulation.getLinks();
		const PxU32 count = articulation->getNbLinks();
		for (PxU32 i = 0; i < count; ++i)
		{
			OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, static_cast<const PxArticulationLink&>(*links[i]));
		}
		for (PxU32 i = 0; i < count; ++i)
			if (const PxArticulationJointReducedCoordinate* joint = links[i]->getInboundJoint())
			{
				OMNI_PVD_CREATE_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, *joint);
			}
		for (PxU32 i = 0; i < count; ++i) ensureCaptureObject(writer, registrationData, *links[i]);
		for (PxU32 i = 0; i < count; ++i)
			if (const PxArticulationJointReducedCoordinate* joint = links[i]->getInboundJoint()) ensureCaptureObject(writer, registrationData, *joint);
	}
}

static void refreshCaptureConstraintDependencies(OmniPvdWriter* writer,
	const OmniPvdPxCoreRegistrationData* registrationData, const PxConstraint& constraint)
{
	const bool streamed = captureContains(samplerInternals->mStreamedObjects, reinterpret_cast<PxU64>(&constraint));
	ensureCaptureDependencies(writer, registrationData, constraint);
	ensureCaptureObject(writer, registrationData, constraint);
	if (streamed && !samplerInternals->mTakingSnapshot)
		NpOmniPvdGetInstance()->notifyObjectCapture(constraint);
}

void OmniPvdPxSampler::onConstraintActorsChange(const PxConstraint& constraint)
{
	if (!isSampling()) return;
	invalidateCaptureScenes();
	if (!captureIsSelective()) return;
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	const bool recorded = captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&constraint));
	if (captureBase(constraint))
	{
		refreshCaptureConstraintDependencies(pvdWriter, pvdRegData, constraint);
		if (!recorded)
			if (const PxScene* scene = constraint.getScene())
				if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(scene)))
					pvdRegData->pvdPxScene.addTo_constraints_(*pvdWriter, OMNI_PVD_CONTEXT_HANDLE, *scene, constraint);
	}
	else if (recorded)
	{
		// Retire the extension joint first, before its core identity or old counterparts.
		NpOmniPvdGetInstance()->notifyObjectCapture(constraint);
		if (const PxScene* scene = constraint.getScene())
		{
			OMNI_PVD_REMOVE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, constraints, *scene, constraint);
		}
		OMNI_PVD_DESTROY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConstraint, constraint);
	}
	OMNI_PVD_WRITE_SCOPE_END
}

#if PX_SUPPORT_GPU_PHYSX
void OmniPvdPxSampler::onParticlePhaseChange(const PxPBDParticleSystem& system)
{
	if (!isSampling() || !captureIsSelective()) return;
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	bool selected = captureActor(system);
	if (!selected)
	{
		// A selected buffer needs its current owner's materials without selecting sibling buffers.
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		for (PxHashMap<const PxParticleBuffer*, const PxPBDParticleSystem*>::Iterator it =
			samplerInternals->mCaptureBufferOwners.getIterator(); !it.done(); ++it)
			if (it->second == &system) { selected = true; break; }
	}
	if (selected) ensureCaptureParticleMaterials(pvdWriter, pvdRegData, system);
	OMNI_PVD_WRITE_SCOPE_END
}
#endif

void OmniPvdPxSampler::onObjectAdd(const physx::PxBase& object)
{
	if (!isSampling()) return;

	// A single live add opens exactly one write scope (one lock acquisition) and forwards to the
	// batch variant below, which does the actual emission with the bound writer + registration data.
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	onObjectAdd(pvdWriter, pvdRegData, object);
	OMNI_PVD_WRITE_SCOPE_END
}

void OmniPvdPxSampler::onObjectAdd(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBase& object)
{
	if (captureIsSelective())
	{
		if (!captureBase(object)) return;
		if (const PxActor* actor = object.is<PxActor>())
		{
			physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
			samplerInternals->mContactActors.erase(actor);
			samplerInternals->mContactActorScenes.erase(actor);
		}
		if (!captureInsert(samplerInternals->mStreamedObjects, reinterpret_cast<PxU64>(&object))) return;
		ensureCaptureDependencies(pvdWriter, pvdRegData, object);
	}
	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			streamHeightField(pvdWriter, pvdRegData, hf);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			streamConvexMesh(pvdWriter, pvdRegData, cm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			streamTriMesh(pvdWriter, pvdRegData, m);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			streamTetMesh(pvdWriter, pvdRegData, tm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MESH:
		{
			const PxDeformableVolumeMesh& dm = static_cast<const PxDeformableVolumeMesh&>(object);
			streamDeVoMesh(pvdWriter, pvdRegData, dm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMeshes, physics, dm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			streamBVH(pvdWriter, pvdRegData, bvh);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			if (!samplerInternals->mTakingSnapshot) NpOmniPvdGetInstance()->notifyObjectCapture(shape);
			++samplerInternals->mDependencyDepth;
			createGeometry(pvdWriter, pvdRegData, shape.getGeometry());
			--samplerInternals->mDependencyDepth;
			streamShape(pvdWriter, pvdRegData, shape);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
		}
		break;
		case physx::PxConcreteType::eCONSTRAINT:
		{
			const PxConstraint& constraint = static_cast<const PxConstraint&>(object);
			if (!captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&constraint)))
			{
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConstraint, constraint);
			}
			if (!samplerInternals->mTakingSnapshot) NpOmniPvdGetInstance()->notifyObjectCapture(constraint);
			break;
		}
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			streamMaterial(pvdWriter, pvdRegData, mat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE_MATERIAL:
		{
			const PxDeformableSurfaceMaterial& dsMat = static_cast<const PxDeformableSurfaceMaterial&>(object);
			streamFEMClothMaterial(pvdWriter, pvdRegData, dsMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaceMaterials, physics, dsMat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MATERIAL:
		{
			const PxDeformableVolumeMaterial& sbMat = static_cast<const PxDeformableVolumeMaterial&>(object);
			streamFEMSoBoMaterial(pvdWriter, pvdRegData, sbMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMaterials, physics, sbMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			streamPBDMaterial(pvdWriter, pvdRegData, pbdhMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			streamAggregate(pvdWriter, pvdRegData, agg);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			streamArticulation(pvdWriter, pvdRegData, art);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			streamArticulationLink(pvdWriter, pvdRegData, artLink);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, links, artLink.getArticulation(), artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			streamArticulationJoint(pvdWriter, pvdRegData, artJoint);
			break;
		}
		case physx::PxConcreteType::eARTICULATION_MIMIC_JOINT:
		{
			// this is added in NpScene::addArticulationMimicJointInternal
			break;
		}

		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			streamRigidDynamic(pvdWriter, pvdRegData, rd);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			streamRigidStatic(pvdWriter, pvdRegData, rs);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
		}
		break;

	#if PX_SUPPORT_GPU_PHYSX

		case physx::PxConcreteType::eDEFORMABLE_VOLUME:
		{
			const PxDeformableVolume& dv = static_cast<const PxDeformableVolume&>(object);
			streamDeformableVolume(pvdWriter, pvdRegData, dv);
			if (samplerInternals->mTakingSnapshot) streamDeformableState(pvdWriter, pvdRegData, dv);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumes, physics, dv);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE:
		{
			const PxDeformableSurface& ds = static_cast<const PxDeformableSurface&>(object);
			streamDeformableSurface(pvdWriter, pvdRegData, ds);
			if (samplerInternals->mTakingSnapshot) streamDeformableState(pvdWriter, pvdRegData, ds);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaces, physics, ds);
		}
		break;

		case physx::PxConcreteType::ePBD_PARTICLESYSTEM:
		{
			const PxPBDParticleSystem& ps = static_cast<const PxPBDParticleSystem&>(object);
			streamPBDParticleSystem(pvdWriter, pvdRegData, ps);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, pbdParticleSystems, physics, ps);
		}
		break;
		case physx::PxConcreteType::ePARTICLE_BUFFER:
		{
			const PxParticleBuffer& pb = static_cast<const PxParticleBuffer&>(object);
			streamParticleBuffer(pvdWriter, pvdRegData, pb);
			if (samplerInternals->mTakingSnapshot) streamParticleBufferState(pvdWriter, pvdRegData, pb);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			if (captureIsSelective())
			{
				if (const NpPBDParticleSystem* owner = captureParticleBufferOwner(pb))
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, *owner, pb);
				}
			}
		}
		break;
		case physx::PxConcreteType::ePARTICLE_DIFFUSE_BUFFER:
		{
			const PxParticleAndDiffuseBuffer& pb = static_cast<const PxParticleAndDiffuseBuffer&>(object);
			streamParticleAndDiffuseBuffer(pvdWriter, pvdRegData, pb);
			if (samplerInternals->mTakingSnapshot) streamParticleBufferState(pvdWriter, pvdRegData, pb);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			if (captureIsSelective())
			{
				if (const NpPBDParticleSystem* owner = captureParticleBufferOwner(pb))
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, *owner, pb);
				}
			}
		}
		break;

#endif
	}
}

void OmniPvdPxSampler::onObjectRemove(const physx::PxBase& object)
{
	// Retirement must remain serialized after sampling stops. The stream is not inspected.
	PxOmniPvd::ScopedExclusiveWriter writerLock(NpOmniPvdGetInstance());
	NpOmniPvdCaptureRetireHandle(reinterpret_cast<PxU64>(&object));
	const PxBase* serializable = NULL;
	if (const PxConstraint* constraint = object.is<PxConstraint>())
		if (PxConstraintConnector* connector = static_cast<const NpConstraint*>(constraint)->getCore().getPxConnector())
			serializable = connector->getSerializable();
	if (serializable) NpOmniPvdCaptureRetireHandle(reinterpret_cast<PxU64>(serializable));
	forgetCaptureOwnership(reinterpret_cast<PxU64>(&object), false);
	{
		physx::PxMutex::ScopedLock sceneLock(samplerInternals->mCaptureStateMutex);
		if (const PxActor* actor = object.is<PxActor>()) samplerInternals->mJointActorStates.erase(actor);
		captureErase(samplerInternals->mCaptureObjectRoots, &object);
		samplerInternals->mCaptureContextScenesDirty = true;
	}
	if (const PxArticulationReducedCoordinate* articulation = object.is<PxArticulationReducedCoordinate>())
		releaseCaptureReactionCache(*articulation);
	if (!isSampling()) return;

	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tm);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MESH:
		{
			const PxDeformableVolumeMesh& dm = static_cast<const PxDeformableVolumeMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMeshes, physics, dm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, dm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
			destroyGeometry(shape.getGeometry());
		}
		break;
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, mat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE_MATERIAL:
		{
			const PxDeformableSurfaceMaterial& dsMat = static_cast<const PxDeformableSurfaceMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaceMaterials, physics, dsMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, dsMat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MATERIAL:
		{
			const PxDeformableVolumeMaterial& sbMat = static_cast<const PxDeformableVolumeMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMaterials, physics, sbMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, sbMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
		}
		break;
		case physx::PxConcreteType::eCONSTRAINT:
		{
			const PxConstraint& constraint = static_cast<const PxConstraint&>(object);
			if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&constraint)))
			{
				OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConstraint, constraint);
			}
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, artJoint);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_MIMIC_JOINT:
		{
			const PxArticulationMimicJoint& artMimicJoint = static_cast<const PxArticulationMimicJoint&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, artMimicJoint);
		}
		break;

		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rs);
		}
		break;
		case physx::PxConcreteType::ePBD_PARTICLESYSTEM:
		{
			const PxPBDParticleSystem& ps = static_cast<const PxPBDParticleSystem&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, pbdParticleSystems, physics, ps);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, ps);
		}
		break;
#if PX_SUPPORT_GPU_PHYSX

		case physx::PxConcreteType::eDEFORMABLE_VOLUME:
		{
			const PxDeformableVolume& dv = static_cast<const PxDeformableVolume&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumes, physics, dv);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, dv);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE:
		{
			const PxDeformableSurface& ds = static_cast<const PxDeformableSurface&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaces, physics, ds);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, ds);
		}
		break;

		case physx::PxConcreteType::ePARTICLE_BUFFER:
		{
			const PxParticleBuffer& pb = static_cast<const PxParticleBuffer&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, pb);
		}
		break;
		case physx::PxConcreteType::ePARTICLE_DIFFUSE_BUFFER:
		{
			//need to remove PxDiffuseParticleParams before releasing the low level object
			const PxParticleAndDiffuseBuffer& pb = static_cast<const PxParticleAndDiffuseBuffer&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, pb);
		}
		break;

#endif
	}
}

// Returns true if the Geom was not yet seen and added
bool OmniPvdSamplerInternals::addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum)
{
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSharedGeomsMutex);
	const physx::PxHashMap<const void*, OmniPvdSharedMeshEnum>::Entry* entry = samplerInternals->mSharedMeshesMap.find(geom);
	if (entry)
	{
		return false;
	}
	else
	{
		samplerInternals->mSharedMeshesMap[geom] = geomEnum;
		return true;
	}
}

void OmniPvdPxSampler::reportAlreadySamplingError()
{
	const bool previous = suppressCaptureError;
	suppressCaptureError = true;
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
		"PxOmniPvd::startSampling(): already sampling. Call stopSampling() before starting a new recording.");
	suppressCaptureError = previous;
}

void OmniPvdPxSampler::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
	if (suppressCaptureError) return;
	OMNI_PVD_WRITE_SCOPE_BEGIN(writer, registrationData)

	// The write scope above yields a writer only while sampling (its NpOmniPvdSampling() gate), so
	// nothing below runs unless a recording is active; the schema is then registered on the bound
	// stream and the pvdPxErrorCode class handle is valid. The handle is generated by the
	// OMNI_PVD_ENUM_BEGIN(PxErrorCode) macro in OmniPvdTypes.h; to add new message types, add a new
	// enum there so the type (code) parameter can be indexed in the class data (see OmniPvdOvdParser.cpp).
	const OmniPvdClassHandle handle = registrationData->pvdPxErrorCode.classHandle;

	writer->recordMessage(OMNI_PVD_CONTEXT_HANDLE, message, file, line, code, handle);
	OMNI_PVD_WRITE_SCOPE_END
}

///////////////////////////////////////////////////////////////////////////////

OmniPvdPxSampler* OmniPvdPxSampler::getInstance()
{
	PX_ASSERT(&physx::NpPhysics::getInstance() != NULL);
	return &physx::NpPhysics::getInstance() ? physx::NpPhysics::getInstance().mOmniPvdSampler : NULL;
}

OmniPvdPxSampler* OmniPvdPxSampler::getSamplingInstance()
{
	OmniPvdPxSampler* sampler = getInstance();
	if (sampler)
	{
		return sampler->isSampling() ? sampler : NULL;
	}
	return NULL;
}


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData()
{
	if (samplerInternals)
	{
		return &samplerInternals->mPvdStream.mRegistrationData;
	}
	else
	{
		return NULL;
	}
}

physx::NpOmniPvd* NpOmniPvdGetInstance()
{
	if (samplerInternals)
	{
		return samplerInternals->mPvdStream.mOmniPvdInstance;
	}
	else
	{
		return NULL;
	}
}

bool NpOmniPvdSampling()
{
	OmniPvdPxSampler* sampler = OmniPvdPxSampler::getInstance();
	return sampler ? sampler->isSampling() : false;
}

bool NpOmniPvdRecordSecondaryData()
{
	return NpOmniPvdSampling() && captureSecondaryData();
}
bool NpOmniPvdCaptureActor(const PxActor& actor) { return NpOmniPvdSampling() && captureActor(actor); }
bool NpOmniPvdCaptureActorState(const PxActor& actor)
{
	return NpOmniPvdSampling() && (captureActor(actor) || captureContains(samplerInternals->mContactActors, &actor));
}
bool NpOmniPvdCaptureArticulation(const PxArticulationReducedCoordinate& articulation)
{
	return NpOmniPvdSampling() && captureArticulation(articulation);
}
bool NpOmniPvdCaptureScene(const PxScene& scene) { return NpOmniPvdSampling() && captureScene(scene); }
bool NpOmniPvdCaptureIsSelective() { return NpOmniPvdSampling() && captureIsSelective(); }
void NpOmniPvdCaptureDependency(const void* object) { captureInsert(samplerInternals->mDependencies, reinterpret_cast<PxU64>(object)); }

bool NpOmniPvdShouldRecordCreation(const char* className, PxU64 objectHandle)
{
	if (!NpOmniPvdSampling() || !captureClass(className, objectHandle)) return false;
	if (samplerInternals->mDependencyDepth) captureInsert(samplerInternals->mDependencies, objectHandle);
	const bool first = captureInsert(samplerInternals->mRecordedObjects, objectHandle);
	return !captureIsSelective() || first;
}

void NpOmniPvdCaptureRetireHandle(PxU64 handle)
{
	NpOmniPvd* omniPvd = samplerInternals ? samplerInternals->mPvdStream.mOmniPvdInstance : NULL;
	if (!omniPvd || (!NpOmniPvdSampling() && !omniPvd->isCaptureDiscoveryActive())) return;
	PxMutex::ScopedLock lock(samplerInternals->mRetiredHandleMutex);
	// Scope roots must retire even on streams without a TCP connection identity.
	omniPvd->mDiscoveryScenes.erase(handle);
	omniPvd->mDiscoveryObjects.erase(handle);
	// Keep connection history across a recording restart, but stop growing it while inactive.
	if (samplerInternals->mRetiredHandleSession)
		samplerInternals->mRetiredControlHandles.insert(handle);
}

bool NpOmniPvdShouldRecordDestruction(const char* className, PxU64 objectHandle)
{
	if (!NpOmniPvdSampling()) return false;
	forgetCaptureOwnership(objectHandle, !Pxstrcmp(className, "PxScene"));
	const bool recorded = captureErase(samplerInternals->mRecordedObjects, objectHandle);
	captureErase(samplerInternals->mDependencies, objectHandle);
	captureErase(samplerInternals->mStreamedObjects, objectHandle);
	if (!Pxstrcmp(className, "PxArticulationReducedCoordinate"))
		releaseCaptureReactionCache(*reinterpret_cast<const PxArticulationReducedCoordinate*>(objectHandle));
	physx::PxMutex::ScopedLock sceneLock(samplerInternals->mCaptureStateMutex);
	if (!Pxstrcmp(className, "PxScene"))
	{
		captureErase(samplerInternals->mCaptureSceneRoots, reinterpret_cast<const PxScene*>(objectHandle));
	}
	if (!Pxstrcmp(className, "PxActor") || !Pxstrcmp(className, "PxRigidDynamic") || !Pxstrcmp(className, "PxRigidStatic") || !Pxstrcmp(className, "PxArticulationLink") ||
		!Pxstrcmp(className, "PxPBDParticleSystem") || !Pxstrcmp(className, "PxDeformableVolume") || !Pxstrcmp(className, "PxDeformableSurface"))
	{
		const PxActor* actor = reinterpret_cast<const PxActor*>(objectHandle);
		captureErase(samplerInternals->mCaptureActorRoots, actor);
		captureErase(samplerInternals->mContactActors, actor);
	}
	if (!Pxstrcmp(className, "PxArticulationReducedCoordinate"))
	{
		const PxArticulationReducedCoordinate* articulation = reinterpret_cast<const PxArticulationReducedCoordinate*>(objectHandle);
		captureErase(samplerInternals->mCaptureArticulationRoots, articulation);
	}
	samplerInternals->mCaptureContextScenesDirty = true;
	return !captureIsSelective() || recorded;
}

bool NpOmniPvdShouldRecordValue(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle)
{
	if (!NpOmniPvdSampling()) return false;
	if (!captureIsSelective()) return true;
	if (!captureClass(className, objectHandle))
	{
		return !Pxstrcmp(className, "PxScene") && (!Pxstrcmp(attributeName, "pairCount") || !Pxstrncmp(attributeName, "pairs", 5)) &&
			captureSecondaryData() && captureContains(samplerInternals->mRecordedObjects, objectHandle);
	}
	if (!Pxstrcmp(className, "PxOmniPvdMetaData") || !Pxstrcmp(className, "PxPhysics")) return true;
	if (!captureContains(samplerInternals->mRecordedObjects, objectHandle))
	{
		if (!Pxstrcmp(className, "PxScene"))
			reinterpret_cast<NpScene*>(objectHandle)->emitSceneStateToOmniPvd();
		else if (!Pxstrcmp(className, "PxActor") || !Pxstrcmp(className, "PxRigidActor") || !Pxstrcmp(className, "PxRigidBody") ||
			!Pxstrcmp(className, "PxRigidDynamic") || !Pxstrcmp(className, "PxRigidStatic") || !Pxstrcmp(className, "PxArticulationLink"))
			ensureCaptureObject(writer, registrationData, *reinterpret_cast<const PxBase*>(objectHandle));
		if (!captureContains(samplerInternals->mRecordedObjects, objectHandle)) return false;
	}
	if (captureIdentityAttribute(attributeName)) return true;
	const PxActor* actor = reinterpret_cast<const PxActor*>(objectHandle);
	if (captureContains(samplerInternals->mContactActors, actor) && !captureActor(*actor))
	{
		return captureMotionAttribute(attributeName);
	}
	return captureSecondaryData() || capturePrimaryAttribute(className, attributeName);
}

bool NpOmniPvdShouldRecordAddition(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle)
{
	updateCaptureMembership(className, attributeName, objectHandle, valueHandle, true);
	if (!NpOmniPvdSampling()) return false;
	if (!captureIsSelective())
	{
		if (!Pxstrcmp(className, "PxScene") && !Pxstrcmp(attributeName, "constraints") &&
			!captureContains(samplerInternals->mRecordedObjects, valueHandle))
			ensureCaptureObject(writer, registrationData, *reinterpret_cast<const PxBase*>(valueHandle));
		return true;
	}
	if (!Pxstrcmp(className, "PxPhysics") && !Pxstrcmp(attributeName, "scenes"))
	{
		NpScene& scene = *reinterpret_cast<NpScene*>(valueHandle);
		if (!captureScene(scene)) return false;
		if (!captureContains(samplerInternals->mRecordedObjects, valueHandle)) scene.emitSceneStateToOmniPvd();
	}
	else if (!Pxstrcmp(attributeName, "shapes"))
	{
		if (!captureClass(className, objectHandle)) return false;
		captureInsert(samplerInternals->mDependencies, valueHandle);
		ensureCaptureObject(writer, registrationData, *reinterpret_cast<const PxBase*>(valueHandle));
	}
	else if (!Pxstrcmp(attributeName, "particleBuffers"))
	{
		// Refresh a selected buffer's owner dependencies even if that buffer was already streamed.
		const PxBase& buffer = *reinterpret_cast<const PxBase*>(valueHandle);
		if (captureBase(buffer)) ensureCaptureDependencies(writer, registrationData, buffer);
		if (!captureClass(className, objectHandle)) return false;
		captureInsert(samplerInternals->mDependencies, valueHandle);
		ensureCaptureObject(writer, registrationData, *reinterpret_cast<const PxBase*>(valueHandle));
	}
	else
	{
		const PxBase& target = *reinterpret_cast<const PxBase*>(valueHandle);
		if (!captureBase(target) && !captureContains(samplerInternals->mDependencies, valueHandle)) return false;
		if (const PxConstraint* constraint = target.is<PxConstraint>())
			refreshCaptureConstraintDependencies(writer, registrationData, *constraint);
		else ensureCaptureObject(writer, registrationData, target);
		if (!Pxstrcmp(className, "PxAggregate") && !Pxstrcmp(attributeName, "actors") &&
			!captureContains(samplerInternals->mRecordedObjects, objectHandle))
		{
			const PxAggregate& aggregate = *reinterpret_cast<const PxAggregate*>(objectHandle);
			if (captureBase(aggregate))
			{
				ensureCaptureObject(writer, registrationData, aggregate);
				if (const PxScene* scene = static_cast<const NpAggregate&>(aggregate).getNpScene())
					if (captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(scene)))
					{
						OMNI_PVD_ADD_EXPLICIT(writer, registrationData, OMNI_PVD_CONTEXT_HANDLE, PxScene, aggregates, *scene, aggregate);
					}
				return false;
			}
		}
	}
	if (!captureClass(className, objectHandle)) return false;
	if (!Pxstrcmp(className, "PxScene") && !captureContains(samplerInternals->mRecordedObjects, objectHandle))
		reinterpret_cast<NpScene*>(objectHandle)->emitSceneStateToOmniPvd();
	return captureContains(samplerInternals->mRecordedObjects, objectHandle) && captureContains(samplerInternals->mRecordedObjects, valueHandle);
}

static void forgetCaptureStreamedArticulation(const PxArticulationReducedCoordinate& articulation)
{
	captureErase(samplerInternals->mStreamedObjects, reinterpret_cast<PxU64>(&articulation));
	const NpArticulationReducedCoordinate& npArticulation = static_cast<const NpArticulationReducedCoordinate&>(articulation);
	const NpArticulationLink* const* links = npArticulation.getLinks();
	for (PxU32 i = 0; i < articulation.getNbLinks(); ++i)
	{
		captureErase(samplerInternals->mStreamedObjects, reinterpret_cast<PxU64>(links[i]));
		if (const PxArticulationJointReducedCoordinate* joint = links[i]->getInboundJoint())
			captureErase(samplerInternals->mStreamedObjects, reinterpret_cast<PxU64>(joint));
	}
}

// Remove rigid context dependencies until readmission; preserve selected particle-buffer owners.
static bool forgetCaptureContextActor(const PxActor& actor)
{
	{
		physx::PxMutex::ScopedLock stateLock(samplerInternals->mCaptureStateMutex);
		samplerInternals->mContactActorScenes.erase(&actor);
		if (!samplerInternals->mContactActors.erase(&actor)) return false;
	}
	if (actor.is<PxRigidActor>()) captureErase(samplerInternals->mDependencies, reinterpret_cast<PxU64>(&actor));
	return true;
}

bool NpOmniPvdShouldRecordRemoval(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle)
{
	updateCaptureMembership(className, attributeName, objectHandle, valueHandle, false);
	if (!NpOmniPvdSampling()) return false;
	if (!captureIsSelective()) return true;
	// Keep identities, but resend current attributes and dependencies when membership changes.
	if (!Pxstrcmp(className, "PxScene") && !Pxstrcmp(attributeName, "articulations"))
	{
		const PxArticulationReducedCoordinate& articulation = *reinterpret_cast<const PxArticulationReducedCoordinate*>(valueHandle);
		const NpArticulationLink* const* links = static_cast<const NpArticulationReducedCoordinate&>(articulation).getLinks();
		const bool sceneRecorded = captureContains(samplerInternals->mRecordedObjects, objectHandle);
		for (PxU32 i = 0; i < articulation.getNbLinks(); ++i)
		{
			const PxActor& actor = *links[i];
			if (forgetCaptureContextActor(actor) && sceneRecorded &&
				captureContains(samplerInternals->mRecordedObjects, reinterpret_cast<PxU64>(&actor)))
			{
				// Remove context-link scene entries here; retire the articulation only once below.
				registrationData->pvdPxScene.removeFrom_actors_(*writer, OMNI_PVD_CONTEXT_HANDLE,
					*reinterpret_cast<const PxScene*>(objectHandle), actor);
			}
		}
		forgetCaptureStreamedArticulation(articulation);
	}
	else if ((!Pxstrcmp(className, "PxScene") || !Pxstrcmp(className, "PxAggregate")) && !Pxstrcmp(attributeName, "actors"))
	{
		const PxActor& actor = *reinterpret_cast<const PxActor*>(valueHandle);
		if (!Pxstrcmp(className, "PxScene")) forgetCaptureContextActor(actor);
		if (const PxArticulationLink* link = actor.is<PxArticulationLink>())
			forgetCaptureStreamedArticulation(link->getArticulation());
		else captureErase(samplerInternals->mStreamedObjects, valueHandle);
	}
	else if (!Pxstrcmp(className, "PxScene") && !Pxstrcmp(attributeName, "aggregates"))
		captureErase(samplerInternals->mStreamedObjects, valueHandle);
	return captureContains(samplerInternals->mRecordedObjects, objectHandle) && captureContains(samplerInternals->mRecordedObjects, valueHandle);
}

}

#endif
