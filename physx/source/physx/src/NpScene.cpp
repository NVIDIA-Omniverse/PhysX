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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "NpScene.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpAggregate.h"
#include "PxConstraint.h"
#include "PxSceneDesc.h"
#include "PxDirectGPUAPI.h"
#include "ScScene.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#if PX_SUPPORT_GPU_PHYSX
	#include "NpPBDParticleSystem.h"
	#include "NpDeformableSurface.h"
	#include "NpDeformableVolume.h"
	#include "NpDeformableAttachment.h"
	#include "NpDeformableElementFilter.h"
	#include "cudamanager/PxCudaContextManager.h"
	#include "cudamanager/PxCudaContext.h"
#endif
#include "ScArticulationSim.h"
#include "ScArticulationTendonSim.h"
#include "CmCollection.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"
#include "BpBroadPhase.h"
#include "BpAABBManagerBase.h"
#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;

// enable thread checks in all debug builds
#if PX_DEBUG || PX_CHECKED
	#define NP_ENABLE_THREAD_CHECKS 1
#else
	#define NP_ENABLE_THREAD_CHECKS 0
#endif

using namespace Sq;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_PVD
	#define CREATE_PVD_INSTANCE(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.createPVDInstance", mScene.getContextId());\
			mScenePvdClient.createPvdInstance(obj); \
		} \
	}
	#define RELEASE_PVD_INSTANCE(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.releasePVDInstance", mScene.getContextId());\
			mScenePvdClient.releasePvdInstance(obj); \
		} \
	}
	#define UPDATE_PVD_PROPERTIES(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.updatePVDProperties", mScene.getContextId());\
			mScenePvdClient.updatePvdProperties(obj); \
		} \
	}
	#define PVD_ORIGIN_SHIFT(shift) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.originShift", mScene.getContextId());\
			mScenePvdClient.originShift(shift); \
		} \
	}
#else
	#define CREATE_PVD_INSTANCE(obj) {}
	#define RELEASE_PVD_INSTANCE(obj) {}
	#define UPDATE_PVD_PROPERTIES(obj) {}
	#define PVD_ORIGIN_SHIFT(shift){}
#endif

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE bool removeFromSceneCheck(NpScene* npScene, PxScene* scene, const char* name)
{
	if(scene == static_cast<PxScene*>(npScene))
		return true;
	else
		return PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "%s not assigned to scene or assigned to another scene. Call will be ignored!", name);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_OMNI_PVD
static void SleepingStateChanged(PxRigidDynamic& actor, bool sleeping)
{
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, actor, sleeping)
}
#endif

NpScene::NpScene(const PxSceneDesc& desc, NpPhysics& physics) :
	mNpSQ					(desc,
#if PX_SUPPORT_PVD
		&mScenePvdClient,
#else
		NULL,
#endif
		getContextId()),
	mSceneQueriesStaticPrunerUpdate	(getContextId(), 0, "NpScene.sceneQueriesStaticPrunerUpdate"),
	mSceneQueriesDynamicPrunerUpdate(getContextId(), 0, "NpScene.sceneQueriesDynamicPrunerUpdate"),
	mRigidDynamics				("sceneRigidDynamics"),
	mRigidStatics				("sceneRigidStatics"),
	mArticulations				("sceneArticulations"),
	mAggregates					("sceneAggregates"),
	mSanityBounds				(desc.sanityBounds),
	mNbClients					(1),			//we always have the default client.
	mSceneCompletion			(getContextId(), mPhysicsDone),
	mCollisionCompletion		(getContextId(), mCollisionDone),
	mSceneQueriesCompletion		(getContextId(), mSceneQueriesDone),
	mSceneExecution				(getContextId(), 0, "NpScene.execution"),
	mSceneCollide				(getContextId(), 0, "NpScene.collide"),
	mSceneAdvance				(getContextId(), 0, "NpScene.solve"),
	mStaticBuildStepHandle		(NULL),
	mDynamicBuildStepHandle		(NULL),
	mControllingSimulation		(false),
	mIsAPIReadForbidden			(false),
	mIsAPIWriteForbidden		(false),
	mSimThreadStackSize			(0),
	mConcurrentWriteCount		(0),
	mConcurrentReadCount		(0),
	mConcurrentErrorCount		(0),	
	mCurrentWriter				(0),
	mSQUpdateRunning			(false),
	mBetweenFetchResults		(false),
	mBuildFrozenActors			(false),
	mCorruptedState				(false),
	mScene						(desc, getContextId()),
	mDirectGPUAPI				(NULL),
#if PX_SUPPORT_PVD
	mScenePvdClient				(*this),
#endif
#if PX_SUPPORT_OMNI_PVD
	mSceneOvdClient				(*this),
#endif
	mWakeCounterResetValue		(desc.wakeCounterResetValue),
	mPhysics					(physics),
	mName						(NULL)
{
	mGpuDynamicsConfig = desc.gpuDynamicsConfig;
	mSceneQueriesStaticPrunerUpdate.setObject(this);
	mSceneQueriesDynamicPrunerUpdate.setObject(this);

	mPrunerType[0] = desc.staticStructure;
	mPrunerType[1] = desc.dynamicStructure;

	mSceneExecution.setObject(this);
	mSceneCollide.setObject(this);
	mSceneAdvance.setObject(this);

	mTaskManager = mScene.getTaskManagerPtr();
	mCudaContextManager = mScene.getCudaContextManager();

	mThreadReadWriteDepth = PxTlsAlloc();

#if PX_SUPPORT_OMNI_PVD
	createInOmniPVD(desc);
	OmniPvdPxSampler* sampler = NpPhysics::getInstance().mOmniPvdSampler;
	if (sampler) 
	{
		mScene.mOnSleepingStateChanged = SleepingStateChanged;
		PxsSimulationController* sc = getSimulationController();
		if (sc) 
		{
			// pdeheras : should be depending on OVD readback flags
			sc->setEnableOVDReadback(true);
			sc->setEnableOVDCollisionReadback(true);
		}
	}
#endif
}

NpScene::~NpScene()
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_DESTROY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, this->mGpuDynamicsConfig)
	OMNI_PVD_DESTROY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, static_cast<PxScene &>(*this))
	getSceneOvdClientInternal().stopLastFrame(*pvdWriter);
	OMNI_PVD_WRITE_SCOPE_END
#endif

	// PT: we need to do that one first, now that we don't release the objects anymore. Otherwise we end up with a sequence like:
	// - actor is part of an aggregate, and part of a scene
	// - actor gets removed from the scene. This does *not* remove it from the aggregate.
	// - aggregate gets removed from the scene, sees that one contained actor ain't in the scene => we get a warning message
	PxU32 aggregateCount = mAggregates.size();
	while(aggregateCount--)
		removeAggregate(*mAggregates.getEntries()[aggregateCount], false);

	PxU32 rigidDynamicCount = mRigidDynamics.size();
	while(rigidDynamicCount--)
		removeRigidDynamic(*mRigidDynamics[rigidDynamicCount], false, true);

	PxU32 rigidStaticCount = mRigidStatics.size();
	while(rigidStaticCount--)
		removeRigidStatic(*mRigidStatics[rigidStaticCount], false, true);

	PxU32 articCount = mArticulations.size();
	while(articCount--)
		removeArticulation(*mArticulations.getEntries()[articCount], false);

#if PX_SUPPORT_GPU_PHYSX
	PxU32 particleCount = mPBDParticleSystems.size();
	while(particleCount--)
		removeParticleSystem(*mPBDParticleSystems.getEntries()[particleCount], false);

	PxU32 deformableSurfaceCount = mDeformableSurfaces.size();
	while (deformableSurfaceCount--)
		removeDeformableSurface(*mDeformableSurfaces.getEntries()[deformableSurfaceCount], false);

	PxU32 deformableVolumeCount = mDeformableVolumes.size();
	while(deformableVolumeCount--)
		removeDeformableVolume(*mDeformableVolumes.getEntries()[deformableVolumeCount], false);
#endif
	bool unlock = mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK;

#if PX_SUPPORT_PVD
	mNpSQ.getSingleSqCollector().release();
#endif

#if PX_SUPPORT_PVD
	mScenePvdClient.releasePvdInstance();
#endif
	mScene.release();

	PX_DELETE(mDirectGPUAPI);

	// unlock the lock taken in release(), must unlock before 
	// mRWLock is destroyed otherwise behavior is undefined
	if (unlock)
		unlockWrite();

	PxTlsFree(mThreadReadWriteDepth);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::release()
{
	// need to acquire lock for release, note this is unlocked in the destructor
	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
		lockWrite(PX_FL);

	// It will be hard to do a write check here since all object release calls in the scene destructor do it and would mess
	// up the test. If we really want it on scene destruction as well, we need to either have internal and external release
	// calls or come up with a different approach (for example using thread ID as detector variable).

	if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::release(): Scene is still being simulated! PxScene::fetchResults() is called implicitly.");
		
		if(getSimulationStage() == Sc::SimulationStage::eCOLLIDE)
			fetchCollision(true);

		if(getSimulationStage() == Sc::SimulationStage::eFETCHCOLLIDE)  // need to call getSimulationStage() again beacause fetchCollision() might change the value.
		{
			// this is for split sim
			advance(NULL);
		}

		fetchResults(true, NULL);
	}
	NpPhysics::getInstance().releaseSceneInternal(*this);
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::loadFromDesc(const PxSceneDesc& desc)
{
	if (desc.limits.maxNbBodies)
		mRigidDynamics.reserve(desc.limits.maxNbBodies);

	if (desc.limits.maxNbActors)
		mRigidStatics.reserve(desc.limits.maxNbActors);	// to be consistent with code below (but to match previous interpretation 
														// it would rather be desc.limits.maxNbActors - desc.limits.maxNbBodies)

	mScene.preAllocate(desc.limits.maxNbActors, desc.limits.maxNbBodies, desc.limits.maxNbStaticShapes, desc.limits.maxNbDynamicShapes);

	userData = desc.userData;

	return true;
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setGravity(const PxVec3& g)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setGravity() not allowed while simulation is running. Call will be ignored.")

	mScene.setGravity(g);

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, gravity, static_cast<PxScene&>(*this), g)

	updatePvdProperties();
}

PxVec3 NpScene::getGravity() const
{
	NP_READ_CHECK(this);
	return mScene.getGravity();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setBounceThresholdVelocity(const PxReal t)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((t>0.0f), "PxScene::setBounceThresholdVelocity(): threshold value has to be in (0, PX_MAX_F32)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setBounceThresholdVelocity() not allowed while simulation is running. Call will be ignored.")

	mScene.setBounceThresholdVelocity(t);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, bounceThresholdVelocity, static_cast<PxScene&>(*this), t)
}

PxReal NpScene::getBounceThresholdVelocity() const
{
	NP_READ_CHECK(this)
	return mScene.getBounceThresholdVelocity();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setLimits(const PxSceneLimits& limits)
{
	NP_WRITE_CHECK(this);

	if (limits.maxNbBodies)
		mRigidDynamics.reserve(limits.maxNbBodies);

	if (limits.maxNbActors)
		mRigidStatics.reserve(limits.maxNbActors);	// to be consistent with code below (but to match previous interpretation 
													// it would rather be desc.limits.maxNbActors - desc.limits.maxNbBodies)

	mScene.preAllocate(limits.maxNbActors, limits.maxNbBodies, limits.maxNbStaticShapes, limits.maxNbDynamicShapes);
	mScene.setLimits(limits);

	// PT: TODO: there is no guarantee that all simulation shapes will be SQ shapes so this is wrong
	getSQAPI().preallocate(PX_SCENE_PRUNER_STATIC, limits.maxNbStaticShapes);
	getSQAPI().preallocate(PX_SCENE_PRUNER_DYNAMIC, limits.maxNbDynamicShapes);

	updatePvdProperties();

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbActors, static_cast<PxScene&>(*this), limits.maxNbActors)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbBodies, static_cast<PxScene&>(*this), limits.maxNbBodies)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbStaticShapes, static_cast<PxScene&>(*this), limits.maxNbStaticShapes)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbDynamicShapes, static_cast<PxScene&>(*this), limits.maxNbDynamicShapes)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbAggregates, static_cast<PxScene&>(*this), limits.maxNbAggregates)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbConstraints, static_cast<PxScene&>(*this), limits.maxNbConstraints)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbRegions, static_cast<PxScene&>(*this), limits.maxNbRegions)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbBroadPhaseOverlaps, static_cast<PxScene&>(*this), limits.maxNbBroadPhaseOverlaps)
	OMNI_PVD_WRITE_SCOPE_END
}

//////////////////////////////////////////////////////////////////////////

PxSceneLimits NpScene::getLimits() const
{
	NP_READ_CHECK(this);

	return mScene.getLimits();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setFlag(PxSceneFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(this);
	// this call supports mutable flags only
	PX_CHECK_AND_RETURN(PxSceneFlags(flag) & PxSceneFlags(PxSceneFlag::eMUTABLE_FLAGS), "PxScene::setFlag: This flag is not mutable - you can only set it once in PxSceneDesc at startup!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setFlag() not allowed while simulation is running. Call will be ignored.")

	PxSceneFlags currentFlags = mScene.getFlags();

	if(value)
		currentFlags |= flag;
	else
		currentFlags &= ~PxSceneFlags(flag);

	mScene.setFlags(currentFlags);
	const bool pcm = (currentFlags & PxSceneFlag::eENABLE_PCM);
	mScene.setPCM(pcm);
	const bool contactCache = !(currentFlags & PxSceneFlag::eDISABLE_CONTACT_CACHE);
	mScene.setContactCache(contactCache);
	updatePvdProperties();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, flags, static_cast<PxScene&>(*this), getFlags())
}

PxSceneFlags NpScene::getFlags() const
{
	NP_READ_CHECK(this);
	return mScene.getFlags();
}

void NpScene::setName(const char* name)
{
	mName = name;
#if PX_SUPPORT_OMNI_PVD
	PxScene & s = *this;
	streamSceneName(s, mName);
#endif
}

const char*	NpScene::getName() const 
{
	return mName;
}

///////////////////////////////////////////////////////////////////////////////

template<class actorT>
static PX_NOINLINE bool doRigidActorChecks(const actorT& actor, const PruningStructure* ps, const NpScene* scene)
{
	if(!ps && actor.getShapeManager().getPruningStructure())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActors(): actor is in a pruning structure and cannot be added to a scene directly, use addActors(const PxPruningStructure& )");

	if(actor.getNpScene())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActors(): Actor already assigned to a scene. Call will be ignored!");

#if PX_CHECKED
	if(!actor.checkConstraintValidity())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActors(): actor has invalid constraint and may not be added to scene");

	scene->checkPositionSanity(actor, actor.getGlobalPose(), "PxScene::addActors");
#else
	PX_UNUSED(scene);
#endif
	return true;
}

// PT: make sure we always add to array and set the array index properly / at the same time
template<class T>
static PX_FORCE_INLINE void addRigidActorToArray(T& a, PxArray<T*>& rigidActors, Cm::IDPool& idPool)
{
	a.setRigidActorArrayIndex(rigidActors.size());
	rigidActors.pushBack(&a);

	a.setRigidActorSceneIndex(idPool.getNewID());
}

bool NpScene::addActor(PxActor& actor, const PxBVH* bvh)
{
	PX_PROFILE_ZONE("API.addActor", getContextId());

	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::addActor() not allowed while simulation is running. Call will be ignored.", false)

	PX_SIMD_GUARD;

	NpScene* scene = NpActor::getFromPxActor(actor).getNpScene();
	if (scene)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActor(): Actor already assigned to a scene. Call will be ignored!");

	return addActorInternal(actor, bvh);	
}

bool NpScene::addActorInternal(PxActor& actor, const PxBVH* bvh)
{
	if(bvh)
	{
		PxRigidActor* ra = &static_cast<PxRigidActor&>(actor);
		if(!ra || bvh->getNbBounds() == 0 || bvh->getNbBounds() > ra->getNbShapes())
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidActor::setBVH: BVH is empty or does not match shapes in the actor.");
	}	

	const PxType type = actor.getConcreteType();
	switch (type)
	{
		case (PxConcreteType::eRIGID_STATIC):
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			if (!doRigidActorChecks(npStatic, NULL, this))
				return false;

			return addRigidStatic(npStatic, static_cast<const BVH*>(bvh));
		}
		case (PxConcreteType::eRIGID_DYNAMIC):
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			if (!doRigidActorChecks(npDynamic, NULL, this))
				return false;
	
			return addRigidDynamic(npDynamic, static_cast<const BVH*>(bvh));
		}
		case (PxConcreteType::eARTICULATION_LINK):
		{
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::addActor(): Individual articulation links can not be added to the scene");
		}
#if PX_SUPPORT_GPU_PHYSX
		case (PxConcreteType::eDEFORMABLE_SURFACE):
		{
			return addDeformableSurface(static_cast<PxDeformableSurface&>(actor));
		}
		case (PxConcreteType::eDEFORMABLE_VOLUME):
		{
			return addDeformableVolume(static_cast<PxDeformableVolume&>(actor));
		}
		case (PxConcreteType::ePBD_PARTICLESYSTEM):
		{
			return addParticleSystem(static_cast<PxPBDParticleSystem&>(actor));
		}
#endif
		default:
			PX_ASSERT(false); 	// should not happen
			return false;
	}
}

static void updateScStateAndSetupSq(NpScene* scene, PxSceneQuerySystem& sqManager, NpActor& npActor, const PxRigidActor& actor, NpShapeManager& shapeManager, bool actorDynamic, const PxBounds3* bounds, const PruningStructure* ps)
{
	npActor.setNpScene(scene);
	NpShape*const * shapes = shapeManager.getShapes();
	PxU32 nbShapes = shapeManager.getNbShapes();

	for(PxU32 i=0;i<nbShapes;i++)
		shapes[i]->setSceneIfExclusive(scene);

	shapeManager.setupAllSceneQuery(sqManager, npActor, actor, ps, bounds, actorDynamic);
}

bool NpScene::addActors(PxActor*const* actors, PxU32 nbActors)
{
	return addActorsInternal(actors, nbActors, NULL);
}

bool NpScene::addActors(const PxPruningStructure& ps)
{
	const PruningStructure& prunerStructure = static_cast<const PruningStructure&>(ps);
	if(!prunerStructure.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::addActors(): Provided pruning structure is not valid.");

	return addActorsInternal(prunerStructure.getActors(), prunerStructure.getNbActors(), &prunerStructure);
}

/////////////

bool NpScene::addActorsInternal(PxActor*const* PX_RESTRICT actors, PxU32 nbActors, const PruningStructure* ps)
{
	NP_WRITE_CHECK(this);
	PX_PROFILE_ZONE("API.addActors", getContextId());

	PX_SIMD_GUARD;

	if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActors() not allowed while simulation is running. Call will be ignored.");

	Sc::Scene& scScene = mScene;
	PxU32 actorsDone;

	Sc::BatchInsertionState scState;
	scScene.startBatchInsertion(scState);

	scState.staticActorOffset		= ptrdiff_t(NpRigidStatic::getCoreOffset());
	scState.staticShapeTableOffset	= ptrdiff_t(NpRigidStatic::getNpShapeManagerOffset() + NpShapeManager::getShapeTableOffset());
	scState.dynamicActorOffset		= ptrdiff_t(NpRigidDynamic::getCoreOffset());
	scState.dynamicShapeTableOffset = ptrdiff_t(NpRigidDynamic::getNpShapeManagerOffset() + NpShapeManager::getShapeTableOffset());
	scState.shapeOffset				= ptrdiff_t(NpShape::getCoreOffset());

	PxInlineArray<PxBounds3, 8> shapeBounds;
	for(actorsDone=0; actorsDone<nbActors; actorsDone++)
	{
		if(actorsDone+1<nbActors)
			PxPrefetch(actors[actorsDone+1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		const PxType type = actors[actorsDone]->getConcreteType();
		if(type == PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic& a = *static_cast<NpRigidStatic*>(actors[actorsDone]);
			if(!doRigidActorChecks(a, ps, this))
				break;

			if(!(a.getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)))
			{
				shapeBounds.resizeUninitialized(a.NpRigidStatic::getNbShapes()+1);	// PT: +1 for safe reads in addPrunerData/inflateBounds
				scScene.addStatic(&a, scState, shapeBounds.begin());
				// PT: must call this one before doing SQ calls
				addRigidActorToArray(a, mRigidStatics, mRigidActorIndexPool);
				updateScStateAndSetupSq(this, getSQAPI(), a, a, a.getShapeManager(), false, shapeBounds.begin(), ps);
				a.addConstraintsToScene();
			}
			else
				addRigidStatic(a, NULL, ps);
		}
		else if(type == PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic& a = *static_cast<NpRigidDynamic*>(actors[actorsDone]);
			if(!doRigidActorChecks(a, ps, this))
				break;

			if(!(a.getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)))
			{
				shapeBounds.resizeUninitialized(a.NpRigidDynamic::getNbShapes()+1);	// PT: +1 for safe reads in addPrunerData/inflateBounds
				scScene.addBody(&a, scState, shapeBounds.begin(), false);
				// PT: must call this one before doing SQ calls
				addRigidActorToArray(a, mRigidDynamics, mRigidActorIndexPool);
				updateScStateAndSetupSq(this, getSQAPI(), a, a, a.getShapeManager(), true, shapeBounds.begin(), ps);
				a.addConstraintsToScene();
			}
			else
				addRigidDynamic(a, NULL, ps);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "PxScene::addActors(): Batch addition is not permitted for this actor type, aborting at index %u!", actorsDone);
			break;
		}
	}
	// merge sq PrunerStructure
	if(ps)
	{
		getSQAPI().merge(*ps);
	}

	scScene.finishBatchInsertion(scState);

	// if we failed, still complete everything for the successful inserted actors before backing out	
#if PX_SUPPORT_PVD
	for(PxU32 i=0;i<actorsDone;i++)
	{
		if ((actors[i]->getConcreteType()==PxConcreteType::eRIGID_STATIC) && (!(static_cast<NpRigidStatic*>(actors[i])->getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))))
			mScenePvdClient.addStaticAndShapesToPvd(*static_cast<NpRigidStatic*>(actors[i]));
		else if ((actors[i]->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC) && (!(static_cast<NpRigidDynamic*>(actors[i])->getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))))
			mScenePvdClient.addBodyAndShapesToPvd(*static_cast<NpRigidDynamic*>(actors[i]));
	}
#endif

	if(actorsDone<nbActors)	// Everything is consistent up to the failure point, so just use removeActor to back out gracefully if necessary
	{
		for(PxU32 j=0;j<actorsDone;j++)
			removeActorInternal(*actors[j], false, true);
		return false;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

template<typename T>
static PX_FORCE_INLINE void removeFromRigidActorListT(T& rigidActor, PxArray<T*>& rigidActorList, Cm::IDPool& idPool, PxArray<NpScene::Acceleration>* accels)
{
	const PxU32 index = rigidActor.getRigidActorArrayIndex();
	PX_ASSERT(index != 0xFFFFFFFF);
	PX_ASSERT(index < rigidActorList.size());

	const PxU32 size = rigidActorList.size() - 1;
	rigidActorList.replaceWithLast(index);
	if(accels && index < accels->size())
		accels->replaceWithLast(index);
	if(size && size != index)
	{
		T& swappedActor = *rigidActorList[index];
		swappedActor.setRigidActorArrayIndex(index);
	}

	idPool.freeID(rigidActor.getRigidActorSceneIndex());
	rigidActor.setRigidActorSceneIndex(NP_UNUSED_BASE_INDEX);
}

void NpScene::removeFromRigidDynamicList(NpRigidDynamic& rigidDynamic)
{
	removeFromRigidActorListT(rigidDynamic, mRigidDynamics, mRigidActorIndexPool, &mRigidDynamicsAccelerations);
#if PX_SUPPORT_OMNI_PVD
	if (getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
	{
		getSceneOvdClientInternal().removeRigidDynamicReset(&rigidDynamic);
	}
#endif
}

void NpScene::removeFromRigidStaticList(NpRigidStatic& rigidStatic)
{
	removeFromRigidActorListT(rigidStatic, mRigidStatics, mRigidActorIndexPool, NULL);
}

///////////////////////////////////////////////////////////////////////////////

template<class ActorT>
static void removeActorT(NpScene* npScene, ActorT& actor, PxArray<ActorT*>& actors, bool wakeOnLostTouch, PxArray<NpScene::Acceleration>* accels)
{
	const PxActorFlags actorFlags = actor.getCore().getActorFlags();

	if(actor.getShapeManager().getNbShapes())
		PxPrefetch(actor.getShapeManager().getShapes()[0],sizeof(NpShape));
	PxPrefetch(actors[actors.size()-1],sizeof(ActorT));

	const bool noSim = actorFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);
	if (!noSim)
		actor.removeConstraintsFromScene();

	actor.getShapeManager().teardownAllSceneQuery(npScene->getSQAPI(), actor);

	npScene->scRemoveActor(actor, wakeOnLostTouch, noSim);
	removeFromRigidActorListT(actor, actors, npScene->mRigidActorIndexPool, accels);

	OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene &>(*npScene), static_cast<PxActor &>(actor))
}

void NpScene::removeActors(PxActor*const* PX_RESTRICT actors, PxU32 nbActors, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeActors", getContextId());
	NP_WRITE_CHECK(this);
	
	Sc::Scene& scScene = mScene;
	// resize the bitmap so it does not allocate each remove actor call
	scScene.resizeReleasedBodyIDMaps(mRigidDynamics.size() + mRigidStatics.size(), nbActors);
	Sc::BatchRemoveState removeState;
	scScene.setBatchRemove(&removeState);

	for(PxU32 actorsDone=0; actorsDone<nbActors; actorsDone++)
	{
		if(actorsDone+1<nbActors)
			PxPrefetch(actors[actorsDone+1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		PxType type = actors[actorsDone]->getConcreteType();
		if(!removeFromSceneCheck(this, actors[actorsDone]->getScene(), "PxScene::removeActors(): Actor"))
			break;
					
		removeState.bufferedShapes.clear();
		removeState.removedShapes.clear();		

		if(type == PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic& actor = *static_cast<NpRigidStatic*>(actors[actorsDone]);
			removeActorT(this, actor, mRigidStatics, wakeOnLostTouch, NULL);
		}
		else if(type == PxConcreteType::eRIGID_DYNAMIC)
		{			
			NpRigidDynamic& actor = *static_cast<NpRigidDynamic*>(actors[actorsDone]);	
			removeActorT(this, actor, mRigidDynamics, wakeOnLostTouch, &mRigidDynamicsAccelerations);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "PxScene::removeActor(): Batch removal is not supported for this actor type, aborting at index %u!", actorsDone);
			break;
		}
	}	

	scScene.setBatchRemove(NULL);
}

void NpScene::removeActor(PxActor& actor, bool wakeOnLostTouch)
{
	if(0)	// PT: repro for PX-1999
	{
		PxActor* toRemove = &actor;
		removeActors(&toRemove, 1, wakeOnLostTouch);
		return;
	}

	PX_PROFILE_ZONE("API.removeActor", getContextId());
	NP_WRITE_CHECK(this);

	if(removeFromSceneCheck(this, actor.getScene(), "PxScene::removeActor(): Actor"))
		removeActorInternal(actor, wakeOnLostTouch, true);
}

void NpScene::removeActorInternal(PxActor& actor, bool wakeOnLostTouch, bool removeFromAggregate)
{
	switch(actor.getType())
	{
		case PxActorType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			removeRigidStatic(npStatic, wakeOnLostTouch, removeFromAggregate);
		}
		break;

		case PxActorType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			removeRigidDynamic(npDynamic, wakeOnLostTouch, removeFromAggregate);
		}
		break;

		case PxActorType::eARTICULATION_LINK:
		{
			outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::removeActor(): Individual articulation links can not be removed from the scene");
		}
		break;

#if PX_SUPPORT_GPU_PHYSX
		case PxActorType::eDEFORMABLE_SURFACE:
		{
			NpDeformableSurface& npDeformableSurface = static_cast<NpDeformableSurface&>(actor);
			removeDeformableSurface(npDeformableSurface, wakeOnLostTouch);
		}
		break;
		case PxActorType::eDEFORMABLE_VOLUME:
		{
			NpDeformableVolume& npDeformableVolume = static_cast<NpDeformableVolume&>(actor);
			removeDeformableVolume(npDeformableVolume, wakeOnLostTouch);
		}
		break;
		case PxActorType::ePBD_PARTICLESYSTEM:
		{
			PxPBDParticleSystem& npParticleSystem = static_cast<PxPBDParticleSystem&>(actor);
			removeParticleSystem(npParticleSystem, wakeOnLostTouch);
		}
		break;
#endif
		default:
			PX_ASSERT(0);
	}
}

///////////////////////////////////////////////////////////////////////////////

template<class T>
static PX_FORCE_INLINE bool addRigidActorT(T& rigidActor, PxArray<T*>& rigidActorList, NpScene* scene, const BVH* bvh, const PruningStructure* ps)
{
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(scene, "PxScene::addActor() not allowed while simulation is running. Call will be ignored.", false)

	const bool isNoSimActor = rigidActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);

	PxBounds3 bounds[8+1];	// PT: +1 for safe reads in addPrunerData/inflateBounds
	const bool canReuseBounds = !isNoSimActor && rigidActor.getShapeManager().getNbShapes()<=8;
	PxBounds3* uninflatedBounds = canReuseBounds ? bounds : NULL;

	scene->scAddActor(rigidActor, isNoSimActor, uninflatedBounds, bvh);

	// PT: must call this one before doing SQ calls
	addRigidActorToArray(rigidActor, rigidActorList, scene->mRigidActorIndexPool);

	// PT: SQ_CODEPATH1
	rigidActor.getShapeManager().setupAllSceneQuery(scene->getSQAPI(), rigidActor, ps, uninflatedBounds, bvh);
	if(!isNoSimActor)
		rigidActor.addConstraintsToScene();

#if PX_SUPPORT_GPU_PHYSX
	rigidActor.addAttachments(rigidActor);
	rigidActor.addElementFilters(rigidActor);
#endif

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, static_cast<PxActor &>(rigidActor), rigidActor.getWorldBounds())
	OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene &>(*scene), static_cast<PxActor &>(rigidActor))

	return true;
}

bool NpScene::addRigidStatic(NpRigidStatic& actor, const BVH* bvh, const PruningStructure* ps)
{
	return addRigidActorT(actor, mRigidStatics, this, bvh, ps);
}

bool NpScene::addRigidDynamic(NpRigidDynamic& body, const BVH* bvh, const PruningStructure* ps)
{
	return addRigidActorT(body, mRigidDynamics, this, bvh, ps);
}

///////////////////////////////////////////////////////////////////////////////

template<class T>
static PX_FORCE_INLINE void removeRigidActorT(T& rigidActor, NpScene* scene, bool wakeOnLostTouch, bool removeFromAggregate)
{
	PX_ASSERT(rigidActor.getNpScene() == scene);
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(scene, "PxScene::removeActor() not allowed while simulation is running. Call will be ignored.")

	const bool isNoSimActor = rigidActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);

	if(removeFromAggregate)
	{
		PxU32 index = 0xffffffff;
		NpAggregate* aggregate = rigidActor.getNpAggregate(index);
		if(aggregate)
		{
			aggregate->removeActorAndReinsert(rigidActor, false);
			PX_ASSERT(!rigidActor.getAggregate());
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	rigidActor.removeAttachments(rigidActor, false);
	rigidActor.removeElementFilters(rigidActor, false);
#endif

	rigidActor.getShapeManager().teardownAllSceneQuery(scene->getSQAPI(), rigidActor);
	if(!isNoSimActor)
		rigidActor.removeConstraintsFromScene();

	scene->scRemoveActor(rigidActor, wakeOnLostTouch, isNoSimActor);
	scene->removeFromRigidActorList(rigidActor);

	OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene &>(*scene), static_cast<PxActor &>(rigidActor))
}

void NpScene::removeRigidStatic(NpRigidStatic& actor, bool wakeOnLostTouch, bool removeFromAggregate)
{
	removeRigidActorT(actor, this, wakeOnLostTouch, removeFromAggregate);
}

void NpScene::removeRigidDynamic(NpRigidDynamic& body, bool wakeOnLostTouch, bool removeFromAggregate)
{
	removeRigidActorT(body, this, wakeOnLostTouch, removeFromAggregate);
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::addArticulation(PxArticulationReducedCoordinate& articulation)
{
	PX_PROFILE_ZONE("API.addArticulation", getContextId());
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN_VAL(articulation.getNbLinks()>0, "PxScene::addArticulation: Empty articulations may not be added to a scene.", false);

	NpArticulationReducedCoordinate& npa = static_cast<NpArticulationReducedCoordinate&>(articulation);
	// check that any tendons are not empty
#if PX_CHECKED
	for(PxU32 i = 0u; i < articulation.getNbFixedTendons(); ++i)
	{
		PX_CHECK_AND_RETURN_VAL(npa.getFixedTendon(i)->getNbTendonJoints() > 0u, "PxScene::addArticulation: Articulations with empty fixed tendons may not be added to a scene.", false)
	}
	for(PxU32 i = 0u; i < articulation.getNbSpatialTendons(); ++i)
	{
		PX_CHECK_AND_RETURN_VAL(npa.getSpatialTendon(i)->getNbAttachments() > 0u, "PxScene::addArticulation: Articulations with empty spatial tendons may not be added to a scene.", false)
	}
	for(PxU32 i = 0u; i < articulation.getNbMimicJoints(); i++)
	{
		PxArticulationMimicJoint* mimicJoint = NULL;
		articulation.getMimicJoints(&mimicJoint, 1, i);
		if(mimicJoint->getJointA().getMotion(mimicJoint->getAxisA()) == PxArticulationMotion::eLOCKED || mimicJoint->getJointB().getMotion(mimicJoint->getAxisB()) == PxArticulationMotion::eLOCKED)
		{
			PX_CHECK_AND_RETURN_VAL(false, "PxScene::addArticulation: Mimic joints must couple non-locked joints.", false)
		}
	}
#endif

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::addArticulation() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD;

	if(getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS && articulation.getConcreteType() != PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addArticulation(): Only Reduced coordinate articulations are currently supported when PxSceneFlag::eENABLE_GPU_DYNAMICS is set!");

	if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE && articulation.getConcreteType() == PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addArticulation(): this call is not allowed while the simulation is running. Call will be ignored!");

	if(!npa.getNpScene())
		return addArticulationInternal(articulation);
	else
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addArticulation(): Articulation already assigned to a scene. Call will be ignored!");
}

static void checkArticulationLink(NpScene* scene, NpArticulationLink* link)
{
#if PX_CHECKED
	scene->checkPositionSanity(*link, link->getGlobalPose(), "PxScene::addArticulation or PxScene::addAggregate");
#else
	PX_UNUSED(scene);
#endif
	if(link->getMass()==0.0f)
	{
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxScene::addArticulation(): Articulation link with zero mass added to scene; defaulting mass to 1");
		link->setMass(1.0f);
	}

	const PxVec3 inertia0 = link->getMassSpaceInertiaTensor();
	if(inertia0.x == 0.0f || inertia0.y == 0.0f || inertia0.z == 0.0f)
	{
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxScene::addArticulation(): Articulation link with zero moment of inertia added to scene; defaulting inertia to (1,1,1)");
		link->setMassSpaceInertiaTensor(PxVec3(1.0f, 1.0f, 1.0f));
	}
}

bool NpScene::addSpatialTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim)
{
	const PxU32 nbTendons = npaRC->getNbSpatialTendons();

	PxU32 maxAttachments = 0;
	for (PxU32 i = 0; i < nbTendons; ++i)
	{
		NpArticulationSpatialTendon* tendon = npaRC->getSpatialTendon(i);

		const PxU32 numAttachments = tendon->getNbAttachments();

		maxAttachments = PxMax(numAttachments, maxAttachments);
	}

	PxU32 stackSize = 1;
	// Add spatial tendons
	PX_ALLOCA(attachmentStack, NpArticulationAttachment*, maxAttachments);

	for (PxU32 i = 0; i < nbTendons; ++i)
	{
		NpArticulationSpatialTendon* tendon = npaRC->getSpatialTendon(i);

		scAddArticulationSpatialTendon(*tendon);

		//add tendon sim to articulation sim
		Sc::ArticulationSpatialTendonSim* tendonSim = tendon->getTendonCore().getSim();
		scArtSim->addTendon(tendonSim);

		const PxU32 numAttachments = tendon->getNbAttachments();

		// Np check on addArticulation does not allow empty tendons, but assert here.
		PX_ASSERT(numAttachments);

		NpArticulationAttachment* attachment = tendon->getAttachment(0);

		NpArticulationLink* pLink = static_cast<NpArticulationLink*>(attachment->mLink);

		Sc::ArticulationAttachmentCore& lcore = attachment->getCore();
		lcore.mLLLinkIndex = pLink->getLinkIndex();

		tendonSim->addAttachment(lcore);

		attachmentStack[0] = attachment;
		PxU32 curAttachment = 0;
		stackSize = 1;
		while (curAttachment < (numAttachments - 1))
		{
			PX_ASSERT(curAttachment < stackSize);
			NpArticulationAttachment* p = attachmentStack[curAttachment];

			const PxU32 numChildrens = p->getNumChildren();

			NpArticulationAttachment*const* children = p->getChildren();

			for (PxU32 j = 0; j < numChildrens; j++)
			{
				NpArticulationAttachment* child = children[j];

				NpArticulationLink* cLink = static_cast<NpArticulationLink*>(child->mLink);

				Sc::ArticulationAttachmentCore& cCore = child->getCore();
				cCore.mLLLinkIndex = cLink->getLinkIndex();

				tendonSim->addAttachment(cCore);

				attachmentStack[stackSize] = child;
				stackSize++;
			}

			curAttachment++;
		}
	}
	return true;
}

bool NpScene::addFixedTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim)
{
	const PxU32 nbFixedTendons = npaRC->getNbFixedTendons();

	PxU32 maxTendonJoints = 0;
	for (PxU32 i = 0; i < nbFixedTendons; ++i)
	{
		NpArticulationFixedTendon* tendon = npaRC->getFixedTendon(i);

		const PxU32 numTendonJoints = tendon->getNbTendonJoints();

		maxTendonJoints = PxMax(numTendonJoints, maxTendonJoints);
	}

	PxU32 stackSize = 1;
	// Add fixed tendon joint
	PX_ALLOCA(tendonJointStack, NpArticulationTendonJoint*, maxTendonJoints);

	for (PxU32 i = 0; i < nbFixedTendons; ++i)
	{
		NpArticulationFixedTendon* tendon = npaRC->getFixedTendon(i);

		//addTendon(npaRC->getImpl(), *tendon);
		scAddArticulationFixedTendon(*tendon);

		//add tendon sim to articulation sim
		Sc::ArticulationFixedTendonSim* tendonSim = tendon->getTendonCore().getSim();
		scArtSim->addTendon(tendonSim);

		const PxU32 numTendonJoints = tendon->getNbTendonJoints();

		// Np check on addArticulation does not allow empty tendons, but assert here.
		PX_ASSERT(numTendonJoints);

		NpArticulationTendonJoint* tendonJoint = tendon->getTendonJoint(0);

		NpArticulationLink* pLink = static_cast<NpArticulationLink*>(tendonJoint->mLink);

		Sc::ArticulationTendonJointCore& lcore = tendonJoint->getCore();
		lcore.mLLLinkIndex = pLink->getLinkIndex();

		//add parent joint
		tendonSim->addTendonJoint(lcore);

		tendonJointStack[0] = tendonJoint;

		PxU32 curTendonJoint = 0;
		stackSize = 1;
		while (curTendonJoint < (numTendonJoints - 1))
		{
			PX_ASSERT(curTendonJoint < stackSize);
			NpArticulationTendonJoint* p = tendonJointStack[curTendonJoint];

			const PxU32 numChildrens = p->getNumChildren();

			NpArticulationTendonJoint*const* children = p->getChildren();

			for (PxU32 j = 0; j < numChildrens; j++)
			{
				NpArticulationTendonJoint* child = children[j];

				NpArticulationLink* cLink = static_cast<NpArticulationLink*>(child->mLink);

				Sc::ArticulationTendonJointCore& cCore = child->getCore();
				cCore.mLLLinkIndex = cLink->getLinkIndex();

				tendonSim->addTendonJoint(cCore);

				tendonJointStack[stackSize] = child;
				stackSize++;
			}

			curTendonJoint++;
		}
	}
	return true;
}

bool NpScene::addArticulationMimicJointInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim)
{
	const PxU32 nbMimicJoints = npaRC->getNbMimicJoints();

	for (PxU32 i = 0; i < nbMimicJoints; ++i)
	{
		NpArticulationMimicJoint* mimicJoint = npaRC->getMimicJoint(i);

		scAddArticulationMimicJoint(*mimicJoint);

		//add mimic joint sim to articulation sim
		Sc::ArticulationMimicJointSim* mimicJointSim = mimicJoint->getMimicJointCore().getSim();
		scArtSim->addMimicJoint(mimicJointSim, mimicJoint->getLinkA()->getLinkIndex(), mimicJoint->getLinkB()->getLinkIndex());
	}
	return true;
}



bool NpScene::addArticulationInternal(PxArticulationReducedCoordinate& npa)
{
	// Add root link first
	const PxU32 nbLinks = npa.getNbLinks();
	PX_ASSERT(nbLinks > 0);
	NpArticulationReducedCoordinate& npaRC = static_cast<NpArticulationReducedCoordinate&>(npa);
	NpArticulationLink* rootLink = static_cast<NpArticulationLink*>(npaRC.getRoot());

	checkArticulationLink(this, rootLink);

	bool linkTriggersWakeUp = !rootLink->scCheckSleepReadinessBesidesWakeCounter();
	
	addArticulationLinkBody(*rootLink);

	// Add articulation
	scAddArticulation(npaRC);

	if (npaRC.mTopologyChanged)
	{
		//increase cache version
		npaRC.mCacheVersion++;
		npaRC.mTopologyChanged = false;
	}

	Sc::ArticulationCore& scArtCore = npaRC.getCore();
	Sc::ArticulationSim* scArtSim = scArtCore.getSim();

	PxU32 handle = scArtSim->findBodyIndex(*rootLink->getCore().getSim());
	rootLink->setLLIndex(handle);
	
	rootLink->setInboundJointDof(0);

	addArticulationLinkConstraint(*rootLink);
	
	// Add links & joints
	PX_ALLOCA(linkStack, NpArticulationLink*, nbLinks);
	linkStack[0] = rootLink;

	PxU32 curLink = 0;
	PxU32 stackSize = 1;
	while(curLink < (nbLinks-1))
	{
		PX_ASSERT(curLink < stackSize);
		NpArticulationLink* l = linkStack[curLink];
		NpArticulationLink*const* children = l->getChildren();

		for(PxU32 i=0; i < l->getNbChildren(); i++)
		{
			NpArticulationLink* child = children[i];

			NpArticulationJointReducedCoordinate* joint = static_cast<NpArticulationJointReducedCoordinate*>(child->getInboundJoint());
			Sc::ArticulationJointCore& jCore = joint->getCore();

			jCore.getCore().jCalcUpdateFrames = true;

			checkArticulationLink(this, child);

			linkTriggersWakeUp = linkTriggersWakeUp || (!child->scCheckSleepReadinessBesidesWakeCounter());

			addArticulationLink(*child);  // Adds joint too
			
			//child->setInboundJointDof(scArtSim->getDof(child->getLinkIndex()));

			linkStack[stackSize] = child;
			stackSize++;
		}

		curLink++;
	}

	//create low-level tendons
	addSpatialTendonInternal(&npaRC, scArtSim);

	addFixedTendonInternal(&npaRC, scArtSim);

	addArticulationMimicJointInternal(&npaRC, scArtSim);

	scArtSim->createLLStructure();
	
	if ((scArtCore.getWakeCounter() == 0.0f) && linkTriggersWakeUp)
	{
		//The articulation needs to wake up, if one of the links triggers activation.
		npaRC.wakeUpInternal(true, false);
	}

	mArticulations.insert(&npa);

	//add loop joints
	
	if(scArtCore.getArticulationFlags() & PxArticulationFlag::eFIX_BASE)
		rootLink->setFixedBaseLink(true);

	//This method will prepare link data for the gpu 
	mScene.addArticulationSimControl(scArtCore);
	const PxU32 maxLinks = mScene.getMaxArticulationLinks();
	if (maxLinks < nbLinks)
		mScene.setMaxArticulationLinks(nbLinks);

	for (PxU32 i = 0; i < npaRC.mLoopJoints.size(); ++i)
	{
		Sc::ConstraintSim* cSim = npaRC.mLoopJoints[i]->getCore().getSim();
		scArtSim->addLoopConstraint(cSim);
	}

	scArtSim->initializeConfiguration(); 
	
	npaRC.updateKinematicInternal(PxArticulationKinematicFlag::ePOSITION | PxArticulationKinematicFlag::eVELOCITY);

	if (scArtSim)
	{
		//scArtSim->checkResize();

		linkStack[0] = rootLink;
		curLink = 0;
		stackSize = 1;

		while (curLink < (nbLinks - 1))
		{
			PX_ASSERT(curLink < stackSize);
			NpArticulationLink* l = linkStack[curLink];
			NpArticulationLink*const* children = l->getChildren();

			for (PxU32 i = 0; i < l->getNbChildren(); i++)
			{
				NpArticulationLink* child = children[i];

				child->setInboundJointDof(scArtSim->getDof(child->getLinkIndex()));

				NpArticulationJointReducedCoordinate* joint = static_cast<NpArticulationJointReducedCoordinate*>(child->getInboundJoint());
				
				PxArticulationJointType::Enum jointType = joint->getJointType();

				if (jointType == PxArticulationJointType::eUNDEFINED)
				{
#if PX_CHECKED
					outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxScene::addArticulation(): The application need to set joint type. defaulting joint type to eFix");
#endif
					joint->scSetJointType(PxArticulationJointType::eFIX);
					child->setInboundJointDof(0);
				}

				if (jointType != PxArticulationJointType::eFIX)
				{

					PxArticulationMotion::Enum motionX = joint->getMotion(PxArticulationAxis::eX);
					PxArticulationMotion::Enum motionY = joint->getMotion(PxArticulationAxis::eY);
					PxArticulationMotion::Enum motionZ = joint->getMotion(PxArticulationAxis::eZ);

					PxArticulationMotion::Enum motionSwing1 = joint->getMotion(PxArticulationAxis::eSWING1);
					PxArticulationMotion::Enum motionSwing2 = joint->getMotion(PxArticulationAxis::eSWING2);
					PxArticulationMotion::Enum motionTwist = joint->getMotion(PxArticulationAxis::eTWIST);

					//PxArticulationMotion::eLOCKED is 0 
					if (!(motionX | motionY | motionZ | motionSwing1 | motionSwing2 | motionTwist))
					{
						//if all axis are locked, which means the user doesn't set the motion. In this case, we should change the joint type to be
						//fix to avoid crash in the solver
#if PX_CHECKED
						outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxScene::addArticulation(): Encountered a joint with all motions fixed. Switching joint type to eFix");
#endif
						joint->scSetJointType(PxArticulationJointType::eFIX);
						child->setInboundJointDof(0);
					}
				}


				linkStack[stackSize] = child;
				stackSize++;
			}

			curLink++;
		}
	}

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, articulations, static_cast<PxScene &>(*this), static_cast<PxArticulationReducedCoordinate&>(npa));
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, dofs, static_cast<PxArticulationReducedCoordinate&>(npa), npa.getDofs());
	OMNI_PVD_WRITE_SCOPE_END

	return true;
}

void NpScene::removeArticulation(PxArticulationReducedCoordinate& articulation, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeArticulation", getContextId());
	NP_WRITE_CHECK(this);
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::removeArticulation() not allowed while simulation is running. Call will be ignored.")

	if(removeFromSceneCheck(this, articulation.getScene(), "PxScene::removeArticulation(): Articulation"))
		removeArticulationInternal(articulation, wakeOnLostTouch, true);
}

void NpScene::removeArticulationInternal(PxArticulationReducedCoordinate& pxa, bool wakeOnLostTouch, bool removeFromAggregate)
{
	NpArticulationReducedCoordinate& npArticulation = static_cast<NpArticulationReducedCoordinate&>(pxa);

	PxU32 nbLinks = npArticulation.getNbLinks();
	PX_ASSERT(nbLinks > 0);

	if(removeFromAggregate && npArticulation.getAggregate())
	{
		static_cast<NpAggregate*>(npArticulation.getAggregate())->removeArticulationAndReinsert(npArticulation, false);
		PX_ASSERT(!npArticulation.getAggregate());
	}

#if PX_SUPPORT_OMNI_PVD
	if (getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
	{
		getSceneOvdClientInternal().removeArticulationReset(&pxa);
	}
#endif


	//!!!AL
	// Inefficient. We might want to introduce a LL method to kill the whole LL articulation together with all joints in one go, then
	// the order of removing the links/joints does not matter anymore.

	// Remove links & joints
	PX_ALLOCA(linkStack, NpArticulationLink*, nbLinks);
	linkStack[0] = npArticulation.getLinks()[0];
	PxU32 curLink = 0, stackSize = 1;

	while(curLink < (nbLinks-1))
	{
		PX_ASSERT(curLink < stackSize);
		NpArticulationLink* l = linkStack[curLink];
		NpArticulationLink*const* children = l->getChildren();

		for(PxU32 i=0; i < l->getNbChildren(); i++)
		{
			linkStack[stackSize] = children[i];
			stackSize++;
		}

		curLink++;
	}

	PxRigidBodyFlags flag;
	for(PxI32 j=PxI32(nbLinks); j-- > 0; )
	{
		flag |= linkStack[j]->getCore().getCore().mFlags;
		removeArticulationLink(*linkStack[j], wakeOnLostTouch);
	}

	// Remove tendons (RC checked in method)
	removeArticulationTendons(npArticulation);

	// Remove mimic joints.
	removeArticulationMimicJoints(npArticulation);

	if (flag & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		PxNodeIndex index = npArticulation.getCore().getIslandNodeIndex();
		if (index.isValid())
			mScene.resetSpeculativeCCDArticulationLink(index.index());
	}

	// Remove articulation
	scRemoveArticulation(npArticulation);
	removeFromArticulationList(npArticulation);
	
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_REMOVE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, articulations, static_cast<PxScene &>(*this), pxa)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, dofs, pxa, pxa.getDofs());
	OMNI_PVD_WRITE_SCOPE_END
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::addDeformableSurface(PxDeformableSurface& deformableSurface)
{
	if (!(getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS))
		return PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
			"PxScene::addActor(): Deformable surfaces can only be simulated by GPU-accelerated scenes!");

#if PX_SUPPORT_GPU_PHYSX
	if (mDeformableSurfaces.size() == PX_MAX_NB_DEFORMABLE_SURFACE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__,
			"PxScene::addActor(): Deformable surface exceeds maximum number of instances per scene (PX_MAX_NB_DEFORMABLE_SURFACE)!");

	// Add deformable surface
	NpDeformableSurface& npSurface = static_cast<NpDeformableSurface&>(deformableSurface);
	scAddDeformableSurface(this, npSurface);

	NpShape* npShape = static_cast<NpShape*>(npSurface.getShape());
	Sc::ShapeCore* shapeCore = &npShape->getCore();
	npSurface.getCore().attachShapeCore(shapeCore);

	mDeformableSurfaces.insert(&deformableSurface);

	//for gpu deformable surface
	mScene.addDeformableSurfaceSimControl(npSurface.getCore());

	npSurface.addAttachments(deformableSurface);
	npSurface.addElementFilters(deformableSurface);

	return true;
#else
	PX_UNUSED(deformableSurface);
	return false;
#endif
}

void NpScene::removeDeformableSurface(PxDeformableSurface& deformableSurface, bool /*wakeOnLostTouch*/)
{
#if PX_SUPPORT_GPU_PHYSX
	NpDeformableSurface& npSurface = reinterpret_cast<NpDeformableSurface&>(deformableSurface);

	npSurface.removeAttachments(deformableSurface, false);
	npSurface.removeElementFilters(deformableSurface, false);

	scRemoveDeformableSurface(npSurface);
	removeFromDeformableSurfaceList(deformableSurface);
#else
	PX_UNUSED(deformableSurface);
#endif
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::addDeformableVolume(PxDeformableVolume& deformableVolume)
{
	if (!(getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS))
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__,
			"PxScene::addActor(): Deformable volumes can only be simulated by GPU-accelerated scenes!");

#if PX_SUPPORT_GPU_PHYSX
	if (!deformableVolume.getSimulationMesh())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__,
			"PxScene::addActor(): Deformable volume does not have simulation mesh, will not be added to scene!");

	if (mDeformableVolumes.size() == PX_MAX_NB_DEFORMABLE_VOLUME)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__,
			"PxScene::addActor(): Deformable volume exceeds maximum number of deformable volumes per scene (PX_MAX_NB_DEFORMABLE_VOLUME)!");

	NpDeformableVolume& npVolume = static_cast<NpDeformableVolume&>(deformableVolume);
	scAddDeformableVolume(npVolume);

	NpShape* npShape = static_cast<NpShape*>(npVolume.getShape());
	Sc::ShapeCore* shapeCore = &npShape->getCore();
	npVolume.getCore().attachShapeCore(shapeCore);
	npVolume.getCore().attachSimulationMesh(deformableVolume.getSimulationMesh(), deformableVolume.getDeformableVolumeAuxData());

	mDeformableVolumes.insert(&deformableVolume);

	//for gpu deformable volume
	mScene.addDeformableVolumeSimControl(npVolume.getCore());

	npVolume.addAttachments(deformableVolume);
	npVolume.addElementFilters(deformableVolume);

	return true;
#else
	PX_UNUSED(deformableVolume);
	return false;
#endif
}

void NpScene::removeDeformableVolume(PxDeformableVolume& deformableVolume, bool /*wakeOnLostTouch*/)
{
#if PX_SUPPORT_GPU_PHYSX
	NpDeformableVolume& npVolume = reinterpret_cast<NpDeformableVolume&>(deformableVolume);

	npVolume.removeAttachments(deformableVolume, false);
	npVolume.removeElementFilters(deformableVolume, false);

	scRemoveDeformableVolume(npVolume);
	removeFromDeformableVolumeList(deformableVolume);
#else
	PX_UNUSED(deformableVolume);
#endif
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbDeformableSurfaces() const
{
#if PX_SUPPORT_GPU_PHYSX
	NP_READ_CHECK(this);
	return mDeformableSurfaces.size();
#else
	return 0;
#endif
}

PxU32 NpScene::getDeformableSurfaces(PxDeformableSurface** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
#if PX_SUPPORT_GPU_PHYSX
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mDeformableSurfaces.getEntries(), mDeformableSurfaces.size());
#else
	PX_UNUSED(userBuffer);
	PX_UNUSED(bufferSize);
	PX_UNUSED(startIndex);
	return 0;
#endif
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbDeformableVolumes() const
{
#if PX_SUPPORT_GPU_PHYSX
	NP_READ_CHECK(this);
	return mDeformableVolumes.size();
#else
	return 0;
#endif
}

PxU32 NpScene::getDeformableVolumes(PxDeformableVolume** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
#if PX_SUPPORT_GPU_PHYSX
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mDeformableVolumes.getEntries(), mDeformableVolumes.size());
#else
	PX_UNUSED(userBuffer);
	PX_UNUSED(bufferSize);
	PX_UNUSED(startIndex);
	return 0;
#endif
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::addParticleSystem(PxPBDParticleSystem& particleSystem)
{
	if (!mScene.isUsingGpuDynamicsAndBp())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addActor(): Particle systems only currently supported with GPU-accelerated scenes!");

#if PX_SUPPORT_GPU_PHYSX
	NpPBDParticleSystem& npPS = static_cast<NpPBDParticleSystem&>(particleSystem);
	scAddParticleSystem(npPS);
	mPBDParticleSystems.insert(&particleSystem);
	mScene.addParticleSystemSimControl(npPS.getCore());

#if PX_SUPPORT_OMNI_PVD
	OmniPvdPxSampler* omniPvdSampler = NpPhysics::getInstance().mOmniPvdSampler;
	if (omniPvdSampler && omniPvdSampler->isSampling()) 
	{
		npPS.getCore().getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eENABLE_GPU_DATA_SYNC;
	}
	OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene&>(*this), static_cast<PxActor&>(particleSystem));
#endif // PX_SUPPORT_OMNI_PVD

	return true;
#else
	PX_UNUSED(particleSystem);
	return false;
#endif
}

void NpScene::removeParticleSystem(PxPBDParticleSystem& particleSystem, bool /*wakeOnLostTouch*/)
{
#if PX_SUPPORT_GPU_PHYSX
	// Remove particle system
	NpPBDParticleSystem& npPS = reinterpret_cast<NpPBDParticleSystem&>(particleSystem);
	scRemoveParticleSystem(npPS);
	removeFromParticleSystemList(particleSystem);
    OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene&>(*this), static_cast<PxActor&>(particleSystem));
#else
	PX_UNUSED(particleSystem);
#endif
}

PxU32 NpScene::getNbPBDParticleSystems() const
{
	NP_READ_CHECK(this);
#if PX_SUPPORT_GPU_PHYSX
	return mPBDParticleSystems.size();
#else
	return 0;
#endif
}

PxU32 NpScene::getPBDParticleSystems(PxPBDParticleSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);

#if PX_SUPPORT_GPU_PHYSX
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mPBDParticleSystems.getEntries(), mPBDParticleSystems.size());
#else
	PX_UNUSED(userBuffer);
	PX_UNUSED(bufferSize);
	PX_UNUSED(startIndex);
	return 0;
#endif
}

PxU32 NpScene::getNbParticleSystems(PxParticleSolverType::Enum type) const
{
	PX_UNUSED(type);
	return getNbPBDParticleSystems();
}

PxU32 NpScene::getParticleSystems(PxParticleSolverType::Enum type, PxPBDParticleSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PX_UNUSED(type);
	return getPBDParticleSystems(userBuffer, bufferSize, startIndex);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addArticulationLinkBody(NpArticulationLink& link)
{
	scAddActor(link, false, NULL, NULL);

	link.setRigidActorSceneIndex(mRigidActorIndexPool.getNewID());

	link.getShapeManager().setupAllSceneQuery(getSQAPI(), link, NULL);
}

void NpScene::addArticulationLinkConstraint(NpArticulationLink& link)
{
	NpArticulationJointReducedCoordinate* j = static_cast<NpArticulationJointReducedCoordinate*>(link.getInboundJoint());
	if (j)
	{
		scAddArticulationJoint(*j);
	}

	link.addConstraintsToScene();
}

void NpScene::addArticulationLink(NpArticulationLink& link)
{
	Sc::ArticulationCore& scArtCore = static_cast<NpArticulationReducedCoordinate&>(link.getArticulation()).getCore();
	Sc::ArticulationSim* scArtSim = scArtCore.getSim();

	Sc::ArticulationSimDirtyFlags dirtyFlags = scArtSim->getDirtyFlag();

	addArticulationLinkBody(link);
	addArticulationLinkConstraint(link);
	
	if (scArtSim)
	{
		PxU32 cHandle = scArtSim->findBodyIndex(*link.getCore().getSim());
		link.setLLIndex(cHandle);
		
		NpArticulationJointReducedCoordinate* j = static_cast<NpArticulationJointReducedCoordinate*>(link.getInboundJoint());

		j->getCore().setLLIndex(cHandle);
		
		const bool isDirty = (dirtyFlags & Sc::ArticulationSimDirtyFlag::eUPDATE);
		if (j && (!isDirty))
		{
			getScScene().addDirtyArticulationSim(scArtSim);
		}
	}
}

void NpScene::removeArticulationLink(NpArticulationLink& link, bool wakeOnLostTouch)
{
	NpArticulationJointReducedCoordinate* j = static_cast<NpArticulationJointReducedCoordinate*>(link.getInboundJoint());

	link.removeConstraintsFromScene();
	link.getShapeManager().teardownAllSceneQuery(getSQAPI(), link);

	Sc::ArticulationCore& scArtCore = static_cast<NpArticulationReducedCoordinate&>(link.getArticulation()).getCore();
	Sc::ArticulationSim* scArtSim = scArtCore.getSim();

	Sc::ArticulationSimDirtyFlags dirtyFlags = scArtSim->getDirtyFlag();

	if (j)
	{
		const bool isDirty = (dirtyFlags & Sc::ArticulationSimDirtyFlag::eUPDATE);
		if (!isDirty)
		{
			getScScene().addDirtyArticulationSim(scArtSim);
		}
		const PxU32 linkIndex = link.getLinkIndex();
		scArtSim->copyJointStatus(linkIndex);
		scRemoveArticulationJoint(*j);
	}

	scRemoveActor(link, wakeOnLostTouch, false);

	mRigidActorIndexPool.freeID(link.getRigidActorSceneIndex());
	link.setRigidActorSceneIndex(NP_UNUSED_BASE_INDEX);
}

////////////////////////////////////////////////////////////////////////////////
void NpScene::addArticulationAttachment(NpArticulationAttachment& attachment)
{
	Sc::ArticulationSpatialTendonCore& tendonCore = attachment.getTendon().getTendonCore();
	Sc::ArticulationSpatialTendonSim* sim = tendonCore.getSim();

	if (sim)
	{
		Sc::ArticulationAttachmentCore& attachmentCore = attachment.getCore();
		attachmentCore.mLLLinkIndex = attachment.mLink->getLinkIndex();
		sim->addAttachment(attachmentCore);
	}
}

void NpScene::removeArticulationAttachment(NpArticulationAttachment& attachment)
{
	Sc::ArticulationSpatialTendonCore& tendonCore = attachment.getTendon().getTendonCore();
	Sc::ArticulationSpatialTendonSim* sim = tendonCore.getSim();

	if (sim)
	{
		Sc::ArticulationAttachmentCore& attachmentCore = attachment.getCore();
		sim->removeAttachment(attachmentCore);
	}
}

////////////////////////////////////////////////////////////////////////////////////

void NpScene::addArticulationTendonJoint(NpArticulationTendonJoint& tendonJoint)
{
	Sc::ArticulationFixedTendonCore& tendonCore = tendonJoint.getTendon().getTendonCore();
	Sc::ArticulationFixedTendonSim* sim = tendonCore.getSim();

	if (sim)
	{
		Sc::ArticulationTendonJointCore& jointCore = tendonJoint.getCore();
		jointCore.mLLLinkIndex = tendonJoint.mLink->getLinkIndex();
		sim->addTendonJoint(jointCore);
	}
}

void NpScene::removeArticulationTendonJoint(NpArticulationTendonJoint& joint)
{
	Sc::ArticulationFixedTendonCore& tendonCore = joint.getTendon().getTendonCore();
	Sc::ArticulationFixedTendonSim* sim = tendonCore.getSim();

	if (sim)
	{
		Sc::ArticulationTendonJointCore& jointCore = joint.getCore();
		sim->removeTendonJoint(jointCore);
	}
}

void NpScene::removeArticulationTendons(PxArticulationReducedCoordinate& articulation)
{
	NpArticulationReducedCoordinate* npaRC = static_cast<NpArticulationReducedCoordinate*>(&articulation);

	// Remove spatial tendons
	const PxU32 nbSpatialTendons = npaRC->getNbSpatialTendons();

	for(PxU32 i = 0; i < nbSpatialTendons; i++)
	{
		NpArticulationSpatialTendon* tendon = npaRC->getSpatialTendon(i);

		npaRC->removeSpatialTendonInternal(tendon);
	}

	//Remove fixed tendons
	const PxU32 nbFixedTendons = npaRC->getNbFixedTendons();

	for(PxU32 i = 0; i < nbFixedTendons; i++)
	{
		NpArticulationFixedTendon* tendon = npaRC->getFixedTendon(i);
		npaRC->removeFixedTendonInternal(tendon);
	}
}

void NpScene::removeArticulationMimicJoints(PxArticulationReducedCoordinate& articulation)
{
	NpArticulationReducedCoordinate* npaRC = static_cast<NpArticulationReducedCoordinate*>(&articulation);

	const PxU32 nbMimicJoints = npaRC->getNbMimicJoints();

	for (PxU32 i = 0; i < nbMimicJoints; i++)
	{
		NpArticulationMimicJoint* mimicJoint = npaRC->getMimicJoint(i);
		npaRC->removeMimicJointInternal(mimicJoint);
	}
}


///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddAggregate(NpAggregate& agg)
{
	PX_ASSERT(!isAPIWriteForbidden());

	agg.setNpScene(this);

	const PxU32 aggregateID = mScene.createAggregate(&agg, agg.getMaxNbShapesFast(), agg.getFilterHint(), agg.getEnvID());
	agg.setAggregateID(aggregateID);
#if PX_SUPPORT_PVD
	//Sending pvd events after all aggregates's actors are inserted into scene
	mScenePvdClient.createPvdInstance(&agg);
#endif
}

void NpScene::scRemoveAggregate(NpAggregate& agg)
{
	PX_ASSERT(!isAPIWriteForbidden());

	mScene.deleteAggregate(agg.getAggregateID());
	agg.setNpScene(NULL);
#if PX_SUPPORT_PVD
	mScenePvdClient.releasePvdInstance(&agg);
#endif
}

bool NpScene::addAggregate(PxAggregate& aggregate)
{
	PX_PROFILE_ZONE("API.addAggregate", getContextId());
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::addAggregate() not allowed while simulation is running. Call will be ignored.", false)

	PX_SIMD_GUARD;

	NpAggregate& np = static_cast<NpAggregate&>(aggregate);

#if PX_CHECKED
	{
		const PxU32 nb = np.getCurrentSizeFast();
		for(PxU32 i=0;i<nb;i++)
		{
			PxRigidStatic* a = np.getActorFast(i)->is<PxRigidStatic>();
			if(a && !static_cast<NpRigidStatic*>(a)->checkConstraintValidity())
				return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addAggregate(): Aggregate contains an actor with an invalid constraint!");
		}
	}
#endif

	if(mScene.isUsingGpuDynamicsOrBp() && np.getMaxNbShapesFast() == PX_MAX_U32)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addAggregate(): Aggregates cannot be added to GPU scene unless you provide a maxNbShapes!");

	if(np.getNpScene())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::addAggregate(): Aggregate already assigned to a scene. Call will be ignored!");

	scAddAggregate(np);

	np.addToScene(*this);

	mAggregates.insert(&aggregate);

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, aggregates, static_cast<PxScene&>(*this), aggregate);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, scene, aggregate, static_cast<PxScene const*>(this));
	OMNI_PVD_WRITE_SCOPE_END

	return true;
}

void NpScene::removeAggregate(PxAggregate& aggregate, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeAggregate", getContextId());
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::removeAggregate() not allowed while simulation is running. Call will be ignored.")

	if(!removeFromSceneCheck(this, aggregate.getScene(), "PxScene::removeAggregate(): Aggregate"))
		return;

	NpAggregate& np = static_cast<NpAggregate&>(aggregate);
	if(np.getScene()!=this)
		return;

	const PxU32 nb = np.getCurrentSizeFast();
	for(PxU32 j=0;j<nb;j++)
	{
		PxActor* a = np.getActorFast(j);
		PX_ASSERT(a);

		if (a->getType() != PxActorType::eARTICULATION_LINK)
		{
			NpActor& sc = NpActor::getFromPxActor(*a);

			np.scRemoveActor(sc, false);  // This is only here to make sure the aggregateID gets set to invalid

			removeActorInternal(*a, wakeOnLostTouch, false);
		}
		else if (a->getScene())
		{
			NpArticulationLink& al = static_cast<NpArticulationLink&>(*a);
			NpArticulationReducedCoordinate& npArt = static_cast<NpArticulationReducedCoordinate&>(al.getRoot());
			NpArticulationLink* const* links = npArt.getLinks();

			for(PxU32 i=0; i < npArt.getNbLinks(); i++)
			{
				np.scRemoveActor(*links[i], false);  // This is only here to make sure the aggregateID gets set to invalid
			}

			removeArticulationInternal(npArt, wakeOnLostTouch, false);
		}
	}

	scRemoveAggregate(np);

	removeFromAggregateList(aggregate);

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_REMOVE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, aggregates, static_cast<PxScene&>(*this), aggregate);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, scene, aggregate, static_cast<PxScene const*>(NULL));
	OMNI_PVD_WRITE_SCOPE_END
}

PxU32 NpScene::getNbAggregates() const
{
	NP_READ_CHECK(this);
	return mAggregates.size();
}

PxU32 NpScene::getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mAggregates.getEntries(), mAggregates.size());
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::scSwitchRigidToNoSim(NpActor& r)
{
	PX_ASSERT(!isAPIWriteForbidden());

	if(r.getNpScene())
	{
		PxInlineArray<const Sc::ShapeCore*, 64> scShapes;

		const NpType::Enum npType = r.getNpType();
		if(npType==NpType::eRIGID_STATIC)
			getScScene().removeStatic(static_cast<NpRigidStatic&>(r).getCore(), scShapes, true);
		else if(npType==NpType::eBODY)
			getScScene().removeBody(static_cast<NpRigidDynamic&>(r).getCore(), scShapes, true);
		else if(npType==NpType::eBODY_FROM_ARTICULATION_LINK)
			getScScene().removeBody(static_cast<NpArticulationLink&>(r).getCore(), scShapes, true);
		else PX_ASSERT(0);
	}
}

void NpScene::scSwitchRigidFromNoSim(NpActor& r)
{
	PX_ASSERT(!isAPIWriteForbidden());

	if(r.getNpScene())
	{
		NpShape* const* shapes;
		const size_t shapePtrOffset = NpShape::getCoreOffset();
		PxU32 nbShapes;
		{
			bool isCompound;
			const NpType::Enum npType = r.getNpType();
			if(npType==NpType::eRIGID_STATIC)
			{
				NpRigidStatic& np = static_cast<NpRigidStatic&>(r);
				nbShapes = NpRigidStaticGetShapes(np, shapes);
				getScScene().addStatic(np.getCore(), shapes, nbShapes, shapePtrOffset, NULL);
			}
			else if(npType==NpType::eBODY)
			{
				NpRigidDynamic& np = static_cast<NpRigidDynamic&>(r);
				nbShapes = NpRigidDynamicGetShapes(np, shapes, &isCompound);
				getScScene().addBody(np.getCore(), shapes, nbShapes, shapePtrOffset, NULL, isCompound);
			}
			else if(npType==NpType::eBODY_FROM_ARTICULATION_LINK)
			{
				NpArticulationLink& np = static_cast<NpArticulationLink&>(r);
				nbShapes = NpArticulationGetShapes(np, shapes, &isCompound);
				getScScene().addBody(np.getCore(), shapes, nbShapes, shapePtrOffset, NULL, isCompound);
			}
			else
			{
				nbShapes = 0;
				shapes = NULL;
				isCompound = false;
				PX_ASSERT(0);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::addCollection(const PxCollection& collection)
{
	PX_PROFILE_ZONE("API.addCollection", getContextId());
	const Cm::Collection& col = static_cast<const Cm::Collection&>(collection);

	PxU32 nb = col.internalGetNbObjects();
#if PX_CHECKED
	for(PxU32 i=0;i<nb;i++)
	{
		PxRigidStatic* a = col.internalGetObject(i)->is<PxRigidStatic>();
		if(a && !static_cast<NpRigidStatic*>(a)->checkConstraintValidity())
			return outputError<PxErrorCode::eINVALID_OPERATION>( __LINE__, "PxScene::addCollection(): collection contains an actor with an invalid constraint!");
	}	
#endif

	PxArray<PxActor*> actorsToInsert;
	actorsToInsert.reserve(nb);

	struct Local
	{
		static void addActorIfNeeded(PxActor* actor, PxArray<PxActor*>& actorArray)
		{
			if(actor->getAggregate())
				return;	// The actor will be added when the aggregate is added
			actorArray.pushBack(actor);			
		}
	};

	for(PxU32 i=0;i<nb;i++)
	{
		PxBase* s = col.internalGetObject(i);
		const PxType serialType = s->getConcreteType();

		//NpArticulationLink, NpArticulationJoint are added with the NpArticulation
		//Actors and Articulations that are members of an Aggregate are added with the NpAggregate

		if(serialType==PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* np = static_cast<NpRigidDynamic*>(s);
			// if pruner structure exists for the actor, actor will be added with the pruner structure
			if(!np->getShapeManager().getPruningStructure())
				Local::addActorIfNeeded(np, actorsToInsert);
		}
		else if(serialType==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* np = static_cast<NpRigidStatic*>(s);
			// if pruner structure exists for the actor, actor will be added with the pruner structure
			if(!np->getShapeManager().getPruningStructure())
				Local::addActorIfNeeded(np, actorsToInsert);
		}
		else if(serialType==PxConcreteType::eSHAPE)
		{			
		}
		else if (serialType == PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
		{
			NpArticulationReducedCoordinate* np = static_cast<NpArticulationReducedCoordinate*>(s);
			if (!np->getAggregate()) // The actor will be added when the aggregate is added
			{
				addArticulation(static_cast<PxArticulationReducedCoordinate&>(*np));
			}
		}
		else if(serialType==PxConcreteType::eAGGREGATE)
		{
			NpAggregate* np = static_cast<NpAggregate*>(s);
			addAggregate(*np);
		}
		else if(serialType == PxConcreteType::ePRUNING_STRUCTURE)
		{
			PxPruningStructure* ps = static_cast<PxPruningStructure*>(s);
			addActors(*ps);
		}
	}

	if(!actorsToInsert.empty())
		addActorsInternal(&actorsToInsert[0], actorsToInsert.size(), NULL);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbActors(PxActorTypeFlags types) const
{
	NP_READ_CHECK(this);
	PxU32 nbActors = 0;

	if(types & PxActorTypeFlag::eRIGID_STATIC)
		nbActors += mRigidStatics.size();

	if(types & PxActorTypeFlag::eRIGID_DYNAMIC)
		nbActors += mRigidDynamics.size();

	return nbActors;
}

static PxU32 getArrayOfPointers_RigidActors(PxActor** PX_RESTRICT userBuffer, PxU32 bufferSize, PxU32 startIndex,
											NpRigidStatic*const* PX_RESTRICT src0, PxU32 size0,
											NpRigidDynamic*const* PX_RESTRICT src1, PxU32 size1)
{
	// PT: we run the same code as getArrayOfPointers but with a virtual array containing both static & dynamic actors.
	const PxU32 size = size0 + size1;

	const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
	const PxU32 writeCount = PxMin(remainder, bufferSize);
	for(PxU32 i=0;i<writeCount;i++)
	{
		const PxU32 index = startIndex+i;

		PX_ASSERT(index<size);
		if(index<size0)
			userBuffer[i] = src0[index];
		else
			userBuffer[i] = src1[index-size0];
	}
	return writeCount;
}

PxU32 NpScene::getActors(PxActorTypeFlags types, PxActor** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);

	const bool wantsStatic = types & PxActorTypeFlag::eRIGID_STATIC;
	const bool wantsDynamic = types & PxActorTypeFlag::eRIGID_DYNAMIC;

	if(wantsStatic && !wantsDynamic)
		return Cm::getArrayOfPointers(buffer, bufferSize, startIndex, mRigidStatics.begin(), mRigidStatics.size());

	if(!wantsStatic && wantsDynamic)
		return Cm::getArrayOfPointers(buffer, bufferSize, startIndex, mRigidDynamics.begin(), mRigidDynamics.size());

	if(wantsStatic && wantsDynamic)
	{
		return getArrayOfPointers_RigidActors(buffer, bufferSize, startIndex,
											mRigidStatics.begin(), mRigidStatics.size(),
											mRigidDynamics.begin(), mRigidDynamics.size());
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////

PxActor** NpScene::getActiveActors(PxU32& nbActorsOut)
{
	NP_READ_CHECK(this);

	if(!isAPIWriteForbidden())
		return mScene.getActiveActors(nbActorsOut);
	else
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::getActiveActors() not allowed while simulation is running. Call will be ignored.");
		nbActorsOut = 0;
		return NULL;
	}
}

PxActor** NpScene::getFrozenActors(PxU32& nbActorsOut)
{
	NP_READ_CHECK(this);

	if(!isAPIWriteForbidden())
		return mScene.getFrozenActors(nbActorsOut);
	else
	{
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxScene::getFrozenActors() not allowed while simulation is running. Call will be ignored.");
		nbActorsOut = 0;
		return NULL;
	}
}

void NpScene::setFrozenActorFlag(const bool buildFrozenActors)
{
#if PX_CHECKED
	PxSceneFlags combinedFlag(PxSceneFlag::eENABLE_ACTIVE_ACTORS | PxSceneFlag::eENABLE_STABILIZATION);

	PX_CHECK_AND_RETURN((getFlags() & combinedFlag)== combinedFlag,
		"NpScene::setFrozenActorFlag: Cannot raise BuildFrozenActors if PxSceneFlag::eENABLE_STABILIZATION and PxSceneFlag::eENABLE_ACTIVE_ACTORS is not raised!");
#endif
	mBuildFrozenActors = buildFrozenActors;
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbArticulations() const
{
	NP_READ_CHECK(this);
	return mArticulations.size();
}

PxU32 NpScene::getArticulations(PxArticulationReducedCoordinate** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mArticulations.getEntries(), mArticulations.size());
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbConstraints() const
{
	NP_READ_CHECK(this);
	return mScene.getNbConstraints();
}

static PX_FORCE_INLINE PxU32 getArrayOfPointers(PxConstraint** PX_RESTRICT userBuffer, PxU32 bufferSize, PxU32 startIndex, Sc::ConstraintCore*const* PX_RESTRICT src, PxU32 size)
{
	const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
	const PxU32 writeCount = PxMin(remainder, bufferSize);
	src += startIndex;
	for(PxU32 i=0;i<writeCount;i++)
	{
		PxConstraint* pxc = src[i]->getPxConstraint();
		userBuffer[i] = static_cast<PxConstraint*>(pxc);
	}
	return writeCount;
}

PxU32 NpScene::getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return ::getArrayOfPointers(userBuffer, bufferSize, startIndex, mScene.getConstraints(), mScene.getNbConstraints());
}

///////////////////////////////////////////////////////////////////////////////

const PxRenderBuffer& NpScene::getRenderBuffer()
{
	if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE) 
	{
		// will be reading the Sc::Scene renderable which is getting written 
		// during the sim, hence, avoid call while simulation is running.
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::getRenderBuffer() not allowed while simulation is running. Call will be ignored.");
	}

	return mRenderBuffer;
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::getSimulationStatistics(PxSimulationStatistics& s) const
{
	NP_READ_CHECK(this);

	if (getSimulationStage() == Sc::SimulationStage::eCOMPLETE)
	{
#if PX_ENABLE_SIM_STATS
		mScene.getStats(s);
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
		PX_UNUSED(s);
#endif
	}
	else
	{
		//will be reading data that is getting written during the sim, hence, avoid call while simulation is running.
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::getSimulationStatistics() not allowed while simulation is running. Call will be ignored.");
	}
}

///////////////////////////////////////////////////////////////////////////////

PxClientID NpScene::createClient()
{
	NP_WRITE_CHECK(this);

	// PT: mNbClients starts at 1, 0 reserved for PX_DEFAULT_CLIENT
	const PxClientID clientID = PxClientID(mNbClients);
	mNbClients++;
	return clientID;
}

///////////////////////////////////////////////////////////////////////////////

PxSolverType::Enum NpScene::getSolverType() const
{
	NP_READ_CHECK(this);
	return mScene.getSolverType();
}

//FrictionModel 

PxFrictionType::Enum NpScene::getFrictionType() const
{
	NP_READ_CHECK(this);
	return PxFrictionType::ePATCH;
}

///////////////////////////////////////////////////////////////////////////////

// Callbacks

void NpScene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setSimulationEventCallback() not allowed while simulation is running. Call will be ignored.")

	mScene.setSimulationEventCallback(callback);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, hasSimulationEventCallback, static_cast<PxScene&>(*this), callback ? true : false)
}

PxSimulationEventCallback* NpScene::getSimulationEventCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getSimulationEventCallback();
}

void NpScene::setContactModifyCallback(PxContactModifyCallback* callback)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setContactModifyCallback() not allowed while simulation is running. Call will be ignored.")

	mScene.setContactModifyCallback(callback);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, hasContactModifyCallback, static_cast<PxScene&>(*this), callback ? true : false)
}

PxContactModifyCallback* NpScene::getContactModifyCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getContactModifyCallback();
}

void NpScene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setCCDContactModifyCallback() not allowed while simulation is running. Call will be ignored.")

	mScene.setCCDContactModifyCallback(callback);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, hasCCDContactModifyCallback, static_cast<PxScene&>(*this), callback ? true : false)
}

PxCCDContactModifyCallback* NpScene::getCCDContactModifyCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDContactModifyCallback();
}

void NpScene::setBroadPhaseCallback(PxBroadPhaseCallback* callback)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setBroadPhaseCallback() not allowed while simulation is running. Call will be ignored.")

	mScene.getBroadphaseManager().setBroadPhaseCallback(callback);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, hasBroadPhaseCallback, static_cast<PxScene&>(*this), callback ? true : false)
}

PxBroadPhaseCallback* NpScene::getBroadPhaseCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getBroadphaseManager().getBroadPhaseCallback();
}

void NpScene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((ccdMaxPasses!=0), "PxScene::setCCDMaxPasses(): ccd max passes cannot be zero!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setCCDMaxPasses() not allowed while simulation is running. Call will be ignored.")

	mScene.setCCDMaxPasses(ccdMaxPasses);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdMaxPasses, static_cast<PxScene&>(*this), ccdMaxPasses)
}

PxU32 NpScene::getCCDMaxPasses() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDMaxPasses();
}

void NpScene::setCCDMaxSeparation(const PxReal separation)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((separation>=0.0f), "PxScene::setCCDMaxSeparation(): separation value has to be in [0, PX_MAX_F32)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setCCDMaxSeparation() not allowed while simulation is running. Call will be ignored.")

	mScene.setCCDMaxSeparation(separation);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdMaxSeparation, static_cast<PxScene&>(*this), separation)
}

PxReal NpScene::getCCDMaxSeparation() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDMaxSeparation();
}

void NpScene::setCCDThreshold(const PxReal t)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((t>0.0f), "PxScene::setCCDThreshold(): threshold value has to be in [eps, PX_MAX_F32)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setCCDThreshold() not allowed while simulation is running. Call will be ignored.")

	mScene.setCCDThreshold(t);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdThreshold, static_cast<PxScene&>(*this), t)
}

PxReal NpScene::getCCDThreshold() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDThreshold();
}

PxBroadPhaseType::Enum NpScene::getBroadPhaseType() const
{
	NP_READ_CHECK(this);
	const Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();
	return bp->getType();
}

bool NpScene::getBroadPhaseCaps(PxBroadPhaseCaps& caps) const
{
	NP_READ_CHECK(this);
	const Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();
	bp->getCaps(caps);
	return true;
}

PxU32 NpScene::getNbBroadPhaseRegions() const
{
	NP_READ_CHECK(this);
	const Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();
	return bp->getNbRegions();
}

PxU32 NpScene::getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	const Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();
	return bp->getRegions(userBuffer, bufferSize, startIndex);
}

PxU32 NpScene::addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	PX_PROFILE_ZONE("BroadPhase.addBroadPhaseRegion", getContextId());

	NP_WRITE_CHECK(this);
	PX_CHECK_MSG(region.mBounds.isValid(), "PxScene::addBroadPhaseRegion(): invalid bounds provided!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::addBroadPhaseRegion() not allowed while simulation is running. Call will be ignored.", 0xffffffff)

	if(region.mBounds.isEmpty())
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::addBroadPhaseRegion(): region bounds are empty. Call will be ignored.");
		return 0xffffffff;
	}

	Bp::AABBManagerBase* aabbManager = mScene.getAABBManager();
	Bp::BroadPhase* bp = aabbManager->getBroadPhase();
	return bp->addRegion(region, populateRegion, aabbManager->getBoundsArray().begin(), aabbManager->getContactDistances());
}

bool NpScene::removeBroadPhaseRegion(PxU32 handle)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::removeBroadPhaseRegion() not allowed while simulation is running. Call will be ignored.", false)

	Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();
	return bp->removeRegion(handle);
}

///////////////////////////////////////////////////////////////////////////////

// Filtering
void NpScene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((	((dataSize == 0) && (data == NULL)) ||
							((dataSize > 0) && (data != NULL)) ), "PxScene::setFilterShaderData(): data pointer must not be NULL unless the specified data size is 0 too and vice versa.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setFilterShaderData() not allowed while simulation is running. Call will be ignored.")

	mScene.setFilterShaderData(data, dataSize);
	updatePvdProperties();
}

const void*	NpScene::getFilterShaderData() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShaderDataFast();
}

PxU32 NpScene::getFilterShaderDataSize() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShaderDataSizeFast();
}

PxSimulationFilterShader NpScene::getFilterShader() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShaderFast();
}

PxSimulationFilterCallback*	NpScene::getFilterCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterCallbackFast();
}

bool NpScene::resetFiltering(PxActor& actor)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN_VAL(NpActor::getNpSceneFromActor(actor) && (NpActor::getNpSceneFromActor(actor) == this), "PxScene::resetFiltering(): Actor must be in a scene.", false);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::resetFiltering() not allowed while simulation is running. Call will be ignored.", false)

	bool status;
	switch(actor.getConcreteType())
	{
		case PxConcreteType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			status = npStatic.NpRigidStaticT::resetFiltering_(npStatic, npStatic.getCore(), NULL, 0);
		}
		break;

		case PxConcreteType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			status = npDynamic.resetFiltering_(npDynamic, npDynamic.getCore(), NULL, 0);
			if(status)
				npDynamic.wakeUpInternal();
		}
		break;

		case PxConcreteType::eARTICULATION_LINK:
		{
			NpArticulationLink& npLink = static_cast<NpArticulationLink&>(actor);
			status = npLink.resetFiltering_(npLink, npLink.getCore(), NULL, 0);
			if(status)
			{
				NpArticulationReducedCoordinate& npArticulation = static_cast<NpArticulationReducedCoordinate&>(npLink.getRoot());
				npArticulation.wakeUpInternal(false, true);
			}
		}
		break;

		default:
			status = outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::resetFiltering(): only PxRigidActor supports this operation!");
	}
	return status;
}

bool NpScene::resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN_VAL(NpActor::getNpSceneFromActor(actor) && (NpActor::getNpSceneFromActor(actor) == this), "PxScene::resetFiltering(): Actor must be in a scene.", false);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::resetFiltering() not allowed while simulation is running. Call will be ignored.", false)

	PX_SIMD_GUARD;

	bool status = false;
	switch(actor.getConcreteType())
	{
		case PxConcreteType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			status = npStatic.NpRigidStaticT::resetFiltering_(npStatic, npStatic.getCore(), shapes, shapeCount);
		}
		break;

		case PxConcreteType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			status = npDynamic.resetFiltering_(npDynamic, npDynamic.getCore(), shapes, shapeCount);
			if(status)
				npDynamic.wakeUpInternal();
		}
		break;

		case PxConcreteType::eARTICULATION_LINK:
		{
			NpArticulationLink& npLink = static_cast<NpArticulationLink&>(actor);
			status = npLink.resetFiltering_(npLink, npLink.getCore(), shapes, shapeCount);
			if(status)
			{
				NpArticulationReducedCoordinate& impl = static_cast<NpArticulationReducedCoordinate&>(npLink.getRoot());
				impl.wakeUpInternal(false, true);
			}
		}
		break;
	}
	return status;
}

PxPairFilteringMode::Enum NpScene::getKinematicKinematicFilteringMode() const
{
	NP_READ_CHECK(this);
	return mScene.getKineKineFilteringMode();
}

PxPairFilteringMode::Enum NpScene::getStaticKinematicFilteringMode() const
{
	NP_READ_CHECK(this);
	return mScene.getStaticKineFilteringMode();
}

///////////////////////////////////////////////////////////////////////////////

PxPhysics& NpScene::getPhysics()
{
	return mPhysics;
}

void NpScene::updateConstants(const PxArray<NpConstraint*>& constraints)
{
	PxsSimulationController* simController = mScene.getSimulationController();
	PX_ASSERT(simController);

	PxU32 nbConstraints = constraints.size();
	NpConstraint*const* currentConstraint = constraints.begin();
	while(nbConstraints--)
	{
		(*currentConstraint)->updateConstants(*simController);
		currentConstraint++;
	}
}

void NpScene::updateDirtyShaders()
{
	PX_PROFILE_ZONE("Sim.updateDirtyShaders", getContextId());
	// this should continue to be done in the Np layer even after SC has taken over
	// all vital simulation functions, because it needs to complete before simulate()
	// returns to the application

#ifdef NEW_DIRTY_SHADERS_CODE
	if(1)
	{
		updateConstants(mAlwaysUpdatedConstraints);
		updateConstants(mDirtyConstraints);
		mDirtyConstraints.clear();
	}
	else
#endif
	{
		// However, the implementation needs fixing so that it does work proportional to
		// the number of dirty shaders

		PxsSimulationController* simController = mScene.getSimulationController();
		PX_ASSERT(simController);
		const PxU32 nbConstraints = mScene.getNbConstraints();
		Sc::ConstraintCore*const* constraints = mScene.getConstraints();
		for(PxU32 i=0;i<nbConstraints;i++)
		{
			PxConstraint* pxc = constraints[i]->getPxConstraint();
			static_cast<NpConstraint*>(pxc)->updateConstants(*simController);
		}
	}
}

// PT: TODO
// - classes like PxsMaterialManager are already typedef of templated types so maybe we don't need them here

template<class NpMaterialT, class MaterialManagerT, class MaterialCoreT>
static void updateLowLevelMaterials(NpPhysics& physics, MaterialManagerT& pxsMaterialManager, PxArray<NpScene::MaterialEvent>& materialBuffer, PxvNphaseImplementationContext* context)
{
	NpMaterialT** masterMaterial = NpMaterialAccessor<NpMaterialT>::getMaterialManager(physics).getMaterials();

	//sync all the material events
	const PxU32 size = materialBuffer.size();
	for(PxU32 i=0; i<size; i++)
	{
		const NpScene::MaterialEvent& event = materialBuffer[i];
		const NpMaterialT* masMat = masterMaterial[event.mHandle];
		switch(event.mType)
		{
		case NpScene::MATERIAL_ADD:
			if(masMat)
			{
				MaterialCoreT* materialCore = &masterMaterial[event.mHandle]->mMaterial;
				pxsMaterialManager.setMaterial(materialCore);
				context->registerMaterial(*materialCore);
			}
			break;
		case NpScene::MATERIAL_UPDATE:
			if(masMat)
			{
				MaterialCoreT* materialCore = &masterMaterial[event.mHandle]->mMaterial;
				pxsMaterialManager.updateMaterial(materialCore);
				context->updateMaterial(*materialCore);
			}
			break;
		case NpScene::MATERIAL_REMOVE:
			if (event.mHandle < pxsMaterialManager.getMaxSize())	// materials might get added and then removed again immediately. However, the add does not get processed (see case MATERIAL_ADD above),
			{														// so the remove might end up reading out of bounds memory unless checked.
				MaterialCoreT* materialCore = pxsMaterialManager.getMaterial(event.mHandle);
				if (materialCore->mMaterialIndex == event.mHandle)
				{
					context->unregisterMaterial(*materialCore);
					pxsMaterialManager.removeMaterial(materialCore);
				}
			}
			break;
		};
	}

	materialBuffer.resize(0);
}

void NpScene::syncMaterialEvents()
{
	//
	// Materials are added/updated/removed on a PxPhysics level, thus these operations
	// can run while a scene is simulating. The operations get buffered and applied here at
	// the beginning of a simulation step. A lock prevents clashes for:
	// - modifications to the buffered material events list while processing it in here
	// - modifications to the master material manager while being accessed in here
	//

	PxMutex::ScopedLock lock(mPhysics.getSceneAndMaterialMutex());

	PxvNphaseImplementationContext* context = mScene.getLowLevelContext()->getNphaseImplementationContext();
	updateLowLevelMaterials<NpMaterial, PxsMaterialManager, PxsMaterialCore>(mPhysics, mScene.getMaterialManager(), mSceneMaterialBuffer, context);

#if PX_SUPPORT_GPU_PHYSX
	updateLowLevelMaterials<NpDeformableSurfaceMaterial, PxsDeformableSurfaceMaterialManager, PxsDeformableSurfaceMaterialCore>(mPhysics, mScene.getDeformableSurfaceMaterialManager(), mSceneDeformableSurfaceMaterialBuffer, context);
	updateLowLevelMaterials<NpDeformableVolumeMaterial, PxsDeformableVolumeMaterialManager, PxsDeformableVolumeMaterialCore>(mPhysics, mScene.getDeformableVolumeMaterialManager(), mSceneDeformableVolumeMaterialBuffer, context);
	updateLowLevelMaterials<NpPBDMaterial, PxsPBDMaterialManager, PxsPBDMaterialCore>(mPhysics, mScene.getPBDMaterialManager(), mScenePBDMaterialBuffer, context);
#endif
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::simulateOrCollide(PxReal elapsedTime, PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation, const char* invalidCallMsg, Sc::SimulationStage::Enum simStage)
{
	PX_SIMD_GUARD;

	{
		// write guard must end before simulation kicks off worker threads
		// otherwise the simulation callbacks could overlap with this function
		// and perform API reads,triggering an error
		NP_WRITE_CHECK(this);

		NP_CHECK_SCENE_CORRUPTION;

		PX_PROFILE_START_CROSSTHREAD("Basic.simulate", getContextId());

		if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
		{
			//fetchResult doesn't get called
			return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, invalidCallMsg);
		}

		if (!checkGpuErrorsPreSim(true))
			return false;

		PX_CHECK_AND_RETURN_VAL(elapsedTime > 0, "PxScene::collide/simulate: The elapsed time must be positive!", false);

		PX_CHECK_AND_RETURN_VAL((size_t(scratchBlock)&15) == 0, "PxScene::simulate: scratch block must be 16-byte aligned!", false);
	
		PX_CHECK_AND_RETURN_VAL((scratchBlockSize&16383) == 0, "PxScene::simulate: scratch block size must be a multiple of 16K", false);
	
#if PX_SUPPORT_PVD		
		//signal the frame is starting.	
		mScenePvdClient.frameStart(elapsedTime);
#endif

#if PX_SUPPORT_OMNI_PVD		
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			getSceneOvdClientInternal().incrementFrame(*pvdWriter);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, elapsedTime, static_cast<PxScene&>(*this), elapsedTime)
		OMNI_PVD_WRITE_SCOPE_END
#endif
	
#if PX_ENABLE_DEBUG_VISUALIZATION
		visualize();
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

		updateDirtyShaders();

#if PX_SUPPORT_PVD
		mScenePvdClient.updateJoints();			
#endif

		mScene.setScratchBlock(scratchBlock, scratchBlockSize);

		mElapsedTime = elapsedTime;
		if (simStage == Sc::SimulationStage::eCOLLIDE)
			mScene.setElapsedTime(elapsedTime);

		mControllingSimulation = controlSimulation;

		syncMaterialEvents();

		setSimulationStage(simStage);
		setAPIWriteToForbidden();
		setAPIReadToForbidden();
		mScene.setCollisionPhaseToActive();
	}

	{
		PX_PROFILE_ZONE("Sim.taskFrameworkSetup", getContextId());

		if (controlSimulation)
		{
			{
				PX_PROFILE_ZONE("Sim.resetDependencies", getContextId());
				// Only reset dependencies, etc if we own the TaskManager. Will be false
				// when an NpScene is controlled by an APEX scene.
				mTaskManager->resetDependencies();
			}
			mTaskManager->startSimulation();
		}

		if (simStage == Sc::SimulationStage::eCOLLIDE)
		{
			mCollisionCompletion.setContinuation(*mTaskManager, completionTask);
			mSceneCollide.setContinuation(&mCollisionCompletion);
			//Initialize scene completion task
			mSceneCompletion.setContinuation(*mTaskManager, NULL);

			mCollisionCompletion.removeReference();
			mSceneCollide.removeReference();
		}
		else
		{
			mSceneCompletion.setContinuation(*mTaskManager, completionTask);
			mSceneExecution.setContinuation(*mTaskManager, &mSceneCompletion);

			mSceneCompletion.removeReference();
			mSceneExecution.removeReference();
		}
	}
	return true;
}

bool NpScene::simulate(PxReal elapsedTime, PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation)
{
	return simulateOrCollide(	elapsedTime, completionTask, scratchBlock, scratchBlockSize, controlSimulation, 
								"PxScene::simulate: Simulation is still processing last simulate call, you should call fetchResults()!", Sc::SimulationStage::eADVANCE);
}

bool NpScene::advance(PxBaseTask* completionTask)
{
	NP_WRITE_CHECK(this);

	NP_CHECK_SCENE_CORRUPTION;

	if (!checkGpuErrorsPreSim(false))
		return false;

	//issue error if advance() doesn't get called between fetchCollision() and fetchResult()
	if(getSimulationStage() != Sc::SimulationStage::eFETCHCOLLIDE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::advance: advance() called illegally! advance() needed to be called after fetchCollision() and before fetchResult()!!");

	//if mSimulateStage == eFETCHCOLLIDE, which means collide() has been kicked off and finished running, we can run advance() safely
	{
		//change the mSimulateStaget to eADVANCE to indicate the next stage to run is fetchResult()
		setSimulationStage(Sc::SimulationStage::eADVANCE);
		setAPIReadToForbidden();

		{
			PX_PROFILE_ZONE("Sim.taskFrameworkSetup", getContextId());

			mSceneCompletion.setDependent(completionTask);
			mSceneAdvance.setContinuation(*mTaskManager, &mSceneCompletion);
			mSceneCompletion.removeReference();
			mSceneAdvance.removeReference();
		}
	}
	return true;
}

bool NpScene::collide(PxReal elapsedTime, PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation)
{
	return simulateOrCollide(	elapsedTime, 
								completionTask,
								scratchBlock,
								scratchBlockSize,
								controlSimulation,
								"PxScene::collide: collide() called illegally! If it isn't the first frame, collide() needed to be called between fetchResults() and fetchCollision(). Otherwise, collide() needed to be called before fetchCollision()", 
								Sc::SimulationStage::eCOLLIDE);
}

bool NpScene::checkCollisionInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkCollision", getContextId());
	return mCollisionDone.wait(block ? PxSync::waitForever : 0);
}

bool NpScene::fetchCollision(bool block)
{
	if (mCorruptedState)
		return true;

	if(getSimulationStage() != Sc::SimulationStage::eCOLLIDE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchCollision: fetchCollision() should be called after collide() and before advance()!");

	//if collision isn't finish running (and block is false), then return false
	if(!checkCollisionInternal(block))
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if (!checkSceneStateAndCudaErrors(true))
		return true;
#endif

	// take write check *after* collision() finished, otherwise 
	// we will block fetchCollision() from using the API
	NP_WRITE_CHECK_NOREENTRY(this);

	setSimulationStage(Sc::SimulationStage::eFETCHCOLLIDE);
	setAPIReadToAllowed();

	return true;
}

bool NpScene::checkSceneStateAndCudaErrors(bool isCollide /*= false*/)
{
	if (mCorruptedState) // silent because this case means we already reported errors in previous fetch* calls.
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if (mCudaContextManager && mScene.isUsingGpuDynamicsOrBp())
	{
		PxCUresult res = mCudaContextManager->getCudaContext()->getLastError();
		if (res)
		{
			if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "PhysX Internal CUDA error. Simulation cannot continue! Error code %i!\n", PxI32(res));

			Sc::SimulationStage::Enum compareStage = isCollide ? Sc::SimulationStage::eCOLLIDE : Sc::SimulationStage::eADVANCE;
			if (mCudaContextManager->getCudaContext()->isInAbortMode() && (getSimulationStage() == compareStage))
			{
				outputError<PxErrorCode::eABORT>(__LINE__, "PhysX failed to allocate GPU memory - aborting simulation.");
				mCorruptedState = true;
			}
		
			setSimulationStage(Sc::SimulationStage::eCOMPLETE);
			setAPIReadToAllowed();
			setAPIWriteToAllowed();
			if (isCollide)
				mCollisionDone.reset();
			else
				mPhysicsDone.reset();
			return false;
		}
	}
#else
	PX_UNUSED(isCollide);
#endif

	return true;
}

bool NpScene::checkGpuErrorsPreSim(bool isCollide /* = false */)
{
#if PX_SUPPORT_GPU_PHYSX
	if (mScene.isUsingGpuDynamicsOrBp())
	{
		// AD: order of these 2 matters because we always report an error if we're in the skip state.
		const bool abortMode = mCudaContextManager->getCudaContext()->isInAbortMode();
		if (abortMode)
		{
			if (isCollide)
				return outputError<PxErrorCode::eABORT>(__LINE__, "PhysX cannot start GPU simulation because the PxCudaContextManager is still in out-of-memory state!\n");
			else
				return outputError<PxErrorCode::eABORT>(__LINE__, "PhysX cannot advance GPU simulation because the PxCudaContextManager is still in out-of-memory state!\n");
		}

		const PxCUresult lastError = mCudaContextManager->getCudaContext()->getLastError();
		if (lastError)
		{
			if (isCollide)
				return PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "PhysX cannot start GPU simulation because of previous CUDA errors! Error code %i!\n", PxI32(lastError));
			else
				return PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "PhysX cannot advance GPU simulation because of previous CUDA errors! Error code %i!\n", PxI32(lastError));
		}
	}
#else
	PX_UNUSED(isCollide);
#endif

	return true;
}

void NpScene::flushSimulation(bool sendPendingReports)
{
	PX_PROFILE_ZONE("API.flushSimulation", getContextId());
	NP_WRITE_CHECK_NOREENTRY(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::flushSimulation(): This call is not allowed while the simulation is running. Call will be ignored")

	PX_SIMD_GUARD;

	mScene.flush(sendPendingReports);
	getSQAPI().flushMemory();

	//!!! TODO: Shrink all NpObject lists?
}

/*
Replaces finishRun() with the addition of appropriate thread sync(pulled out of PhysicsThread())

Note: this function can be called from the application thread or the physics thread, depending on the
scene flags.
*/
void NpScene::executeScene(PxBaseTask* continuation)
{
	mScene.simulate(mElapsedTime, continuation);
}

void NpScene::executeCollide(PxBaseTask* continuation)
{
	mScene.collide(mElapsedTime, continuation);
}

void NpScene::executeAdvance(PxBaseTask* continuation)
{
	mScene.advance(mElapsedTime, continuation);
}

///////////////////////////////////////////////////////////////////////////////

// all of these rely on NpPhysics::mSceneAndMaterialMutex being locked when called to
// prevent clashes with syncMaterialEvents()
#define IMPLEMENT_MATERIAL(MaterialType, CoreType, BufferName)						\
void NpScene::addMaterial(const MaterialType& mat)									\
{																					\
	const CoreType& material = mat.mMaterial;										\
	BufferName.pushBack(MaterialEvent(material.mMaterialIndex, MATERIAL_ADD));		\
	CREATE_PVD_INSTANCE(&material)													\
}																					\
																					\
void NpScene::updateMaterial(const MaterialType& mat)								\
{																					\
	const CoreType& material = mat.mMaterial;										\
	BufferName.pushBack(MaterialEvent(material.mMaterialIndex, MATERIAL_UPDATE));	\
	UPDATE_PVD_PROPERTIES(&material)												\
}																					\
																					\
void NpScene::removeMaterial(const MaterialType& mat)								\
{																					\
	const CoreType& material = mat.mMaterial;										\
	if(material.mMaterialIndex == MATERIAL_INVALID_HANDLE)							\
		return;																		\
	BufferName.pushBack(MaterialEvent(material.mMaterialIndex, MATERIAL_REMOVE));	\
	RELEASE_PVD_INSTANCE(&material);												\
}

IMPLEMENT_MATERIAL(NpMaterial, PxsMaterialCore, mSceneMaterialBuffer)

#if PX_SUPPORT_GPU_PHYSX
	IMPLEMENT_MATERIAL(NpDeformableSurfaceMaterial, PxsDeformableSurfaceMaterialCore, mSceneDeformableSurfaceMaterialBuffer)
	IMPLEMENT_MATERIAL(NpDeformableVolumeMaterial, PxsDeformableVolumeMaterialCore, mSceneDeformableVolumeMaterialBuffer)
	IMPLEMENT_MATERIAL(NpPBDMaterial, PxsPBDMaterialCore, mScenePBDMaterialBuffer)
#endif

///////////////////////////////////////////////////////////////////////////////

void NpScene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((group1 < PX_MAX_DOMINANCE_GROUP && group2 < PX_MAX_DOMINANCE_GROUP), 
		"PxScene::setDominanceGroupPair: invalid params! Groups must be <= 31!");
	//can't change matrix diagonal 
	PX_CHECK_AND_RETURN(group1 != group2, "PxScene::setDominanceGroupPair: invalid params! Groups must be unequal! Can't change matrix diagonal!");
	PX_CHECK_AND_RETURN(
		((dominance.dominance0) == 1.0f && (dominance.dominance1 == 1.0f))
		||	((dominance.dominance0) == 1.0f && (dominance.dominance1 == 0.0f))
		||	((dominance.dominance0) == 0.0f && (dominance.dominance1 == 1.0f))
		, "PxScene::setDominanceGroupPair: invalid params! dominance must be one of (1,1), (1,0), or (0,1)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setDominanceGroupPair() not allowed while simulation is running. Call will be ignored.")

	mScene.setDominanceGroupPair(group1, group2, dominance);
	updatePvdProperties();
}

PxDominanceGroupPair NpScene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	NP_READ_CHECK(this);
	PX_CHECK_AND_RETURN_VAL((group1 < PX_MAX_DOMINANCE_GROUP && group2 < PX_MAX_DOMINANCE_GROUP), 
		"PxScene::getDominanceGroupPair: invalid params! Groups must be <= 31!", PxDominanceGroupPair(PxU8(1u), PxU8(1u)));
	return mScene.getDominanceGroupPair(group1, group2);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX && !PX_PUBLIC_RELEASE
void NpScene::updatePhysXIndicator()
{
	PxIntBool isGpu = mScene.isUsingGpuDynamicsOrBp();

	mPhysXIndicator.setIsGpu(isGpu != 0);
}
#endif

///////////////////////////////////////////////////////////////////////////////

void NpScene::setSolverBatchSize(PxU32 solverBatchSize)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setSolverBatchSize() not allowed while simulation is running. Call will be ignored.")

	mScene.setSolverBatchSize(solverBatchSize);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, solverBatchSize, static_cast<PxScene&>(*this), solverBatchSize)
}

PxU32 NpScene::getSolverBatchSize() const
{
	NP_READ_CHECK(this);
	// get from our local copy
	return mScene.getSolverBatchSize();
}

void NpScene::setSolverArticulationBatchSize(PxU32 solverBatchSize)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setSolverArticulationBatchSize() not allowed while simulation is running. Call will be ignored.")

	mScene.setSolverArticBatchSize(solverBatchSize);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, solverArticulationBatchSize, static_cast<PxScene&>(*this), solverBatchSize)
}

PxU32 NpScene::getSolverArticulationBatchSize() const
{
	NP_READ_CHECK(this);
	// get from our local copy
	return mScene.getSolverArticBatchSize();
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(value), "PxScene::setVisualizationParameter: value is not valid.", false);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(this, "PxScene::setVisualizationParameter() not allowed while simulation is running. Call will be ignored.", false)

	if (param >= PxVisualizationParameter::eNUM_VALUES)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "setVisualizationParameter: parameter out of range.");
	else if (value < 0.0f)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "setVisualizationParameter: value must be larger or equal to 0.");
	else
	{
		mScene.setVisualizationParameter(param, value);
		return true;
	}
}

PxReal NpScene::getVisualizationParameter(PxVisualizationParameter::Enum param) const
{
	if (param < PxVisualizationParameter::eNUM_VALUES)
		return mScene.getVisualizationParameter(param);
	else
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "getVisualizationParameter: param is not an enum.");

	return 0.0f;
}

void NpScene::setVisualizationCullingBox(const PxBounds3& box)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_MSG(box.isValid(), "PxScene::setVisualizationCullingBox(): invalid bounds provided!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setVisualizationCullingBox() not allowed while simulation is running. Call will be ignored.")

	mScene.setVisualizationCullingBox(box);
}

PxBounds3 NpScene::getVisualizationCullingBox() const
{
	NP_READ_CHECK(this);
	const PxBounds3& bounds = mScene.getVisualizationCullingBox();
	PX_ASSERT(bounds.isValid());
	return bounds;
}

void NpScene::setNbContactDataBlocks(PxU32 numBlocks)
{
	PX_CHECK_AND_RETURN((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::setNbContactDataBlock: This call is not allowed while the simulation is running. Call will be ignored!");
	
	mScene.setNbContactDataBlocks(numBlocks);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, nbContactDataBlocks, static_cast<PxScene&>(*this), numBlocks)
}

PxU32 NpScene::getNbContactDataBlocksUsed() const
{
	PX_CHECK_AND_RETURN_VAL((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::getNbContactDataBlocksUsed: This call is not allowed while the simulation is running. Returning 0.", 0);
	
	return mScene.getNbContactDataBlocksUsed();
}

PxU32 NpScene::getMaxNbContactDataBlocksUsed() const
{
	PX_CHECK_AND_RETURN_VAL((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::getMaxNbContactDataBlocksUsed: This call is not allowed while the simulation is running. Returning 0.", 0);
	
	return mScene.getMaxNbContactDataBlocksUsed();
}

PxU32 NpScene::getTimestamp() const
{
	return mScene.getTimeStamp();
}

PxCpuDispatcher* NpScene::getCpuDispatcher() const
{
	return mTaskManager->getCpuDispatcher();
}

void NpScene::setMaxBiasCoefficient(const PxReal coeff)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((coeff>=0.0f), "PxScene::setMaxBiasCoefficient(): coefficient has to be in [0, PX_MAX_F32]!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setMaxBiasCoefficient() not allowed while simulation is running. Call will be ignored.")

	mScene.setMaxBiasCoefficient(coeff);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, maxBiasCoefficient, static_cast<PxScene&>(*this), coeff)
}

PxReal NpScene::getMaxBiasCoefficient() const
{
	NP_READ_CHECK(this);
	return mScene.getMaxBiasCoefficient();
}

void NpScene::setFrictionOffsetThreshold(const PxReal t)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((t>=0.0f), "PxScene::setFrictionOffsetThreshold(): threshold value has to be in [0, PX_MAX_F32)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setFrictionOffsetThreshold() not allowed while simulation is running. Call will be ignored.")

	mScene.setFrictionOffsetThreshold(t);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, frictionOffsetThreshold, static_cast<PxScene&>(*this), t)
}

PxReal NpScene::getFrictionOffsetThreshold() const
{
	NP_READ_CHECK(this);
	return mScene.getFrictionOffsetThreshold();
}

void NpScene::setFrictionCorrelationDistance(const PxReal t)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((t >= 0.0f), "PxScene::setFrictionCorrelationDistance(): threshold value has to be in [0, PX_MAX_F32)!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::setFrictionCorrelationDistance() not allowed while simulation is running. Call will be ignored.")

	mScene.setFrictionCorrelationDistance(t);
	updatePvdProperties();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxScene, frictionCorrelationDistance, static_cast<PxScene&>(*this), t)
}

PxReal NpScene::getFrictionCorrelationDistance() const
{
	NP_READ_CHECK(this);
	return mScene.getFrictionCorrelationDistance();
}

PxU32 NpScene::getContactReportStreamBufferSize() const
{
	NP_READ_CHECK(this);
	return mScene.getDefaultContactReportStreamBufferSize();
}

#if PX_CHECKED
void NpScene::checkPositionSanity(const PxRigidActor& a, const PxTransform& pose, const char* fnName) const
{
	if(!mSanityBounds.contains(pose.p))
		PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "%s: actor pose for %lp is outside sanity bounds\n", fnName, &a);
}
#endif

namespace 
{
	struct ThreadReadWriteCount
	{
		ThreadReadWriteCount(const size_t data)
			:	readDepth(data & 0xFF),
				writeDepth((data >> 8) & 0xFF),
				readLockDepth((data >> 16) & 0xFF),
				writeLockDepth((data >> 24) & 0xFF)
		{
			
		}

		size_t getData() const { return size_t(writeLockDepth) << 24 |  size_t(readLockDepth) << 16 | size_t(writeDepth) << 8 | size_t(readDepth); }

		PxU8 readDepth;			// depth of re-entrant reads
		PxU8 writeDepth;		// depth of re-entrant writes 

		PxU8 readLockDepth;		// depth of read-locks
		PxU8 writeLockDepth;	// depth of write-locks
	};
}

#if NP_ENABLE_THREAD_CHECKS

NpScene::StartWriteResult::Enum NpScene::startWrite(bool allowReentry)
{ 
	PX_COMPILE_TIME_ASSERT(sizeof(ThreadReadWriteCount) == 4);

	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
	{
		ThreadReadWriteCount localCounts(PxTlsGetValue(mThreadReadWriteDepth));

		if (mBetweenFetchResults)
			return StartWriteResult::eIN_FETCHRESULTS;

		// ensure we already have the write lock
		return localCounts.writeLockDepth > 0 ? StartWriteResult::eOK : StartWriteResult::eNO_LOCK;
	}
	
	{
		ThreadReadWriteCount localCounts(PxTlsGetValue(mThreadReadWriteDepth));
		StartWriteResult::Enum result;

		if (mBetweenFetchResults)
			result = StartWriteResult::eIN_FETCHRESULTS;

		// check we are the only thread reading (allows read->write order on a single thread) and no other threads are writing
		else if (mConcurrentReadCount != localCounts.readDepth || mConcurrentWriteCount != localCounts.writeDepth)
			result = StartWriteResult::eRACE_DETECTED;
		
		else
			result = StartWriteResult::eOK;

		// increment shared write counter
		PxAtomicIncrement(&mConcurrentWriteCount);

		// in the normal case (re-entry is allowed) then we simply increment
		// the writeDepth by 1, otherwise (re-entry is not allowed) increment
		// by 2 to force subsequent writes to fail by creating a mismatch between
		// the concurrent write counter and the local counter, any value > 1 will do
		localCounts.writeDepth += allowReentry ? 1 : 2;
		PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

		if (result != StartWriteResult::eOK)
			PxAtomicIncrement(&mConcurrentErrorCount);

		return result;
	}
}

void NpScene::stopWrite(bool allowReentry) 
{ 
	if (!(mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
	{
		PxAtomicDecrement(&mConcurrentWriteCount);

		// decrement depth of writes for this thread
		ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));

		// see comment in startWrite()
		if (allowReentry)
			localCounts.writeDepth--;
		else
			localCounts.writeDepth-=2;

		PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());
	}
}

bool NpScene::startRead() const 
{ 
	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
	{
		ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));

		// ensure we already have the write or read lock
		return localCounts.writeLockDepth > 0 || localCounts.readLockDepth > 0;
	}
	else
	{
		PxAtomicIncrement(&mConcurrentReadCount);

		// update current threads read depth
		ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
		localCounts.readDepth++;
		PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

		// success if the current thread is already performing a write (API re-entry) or no writes are in progress
		const bool success = (localCounts.writeDepth > 0 || mConcurrentWriteCount == 0); 

		if (!success)
			PxAtomicIncrement(&mConcurrentErrorCount);

		return success;
	}
} 

void NpScene::stopRead() const 
{
	if (!(mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
	{
		PxAtomicDecrement(&mConcurrentReadCount); 

		// update local threads read depth
		ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
		localCounts.readDepth--;
		PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());
	}
}

#else 

NpScene::StartWriteResult::Enum NpScene::startWrite(bool) { PX_ASSERT(0); return NpScene::StartWriteResult::eOK; }
void NpScene::stopWrite(bool) {}

bool NpScene::startRead() const { PX_ASSERT(0); return false; }
void NpScene::stopRead() const {}

#endif // NP_ENABLE_THREAD_CHECKS

void NpScene::lockRead(const char* /*file*/, PxU32 /*line*/)
{
	// increment this threads read depth
	ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
	localCounts.readLockDepth++;
	PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// if we are the current writer then increment the reader count but don't actually lock (allow reading from threads with write ownership)
	if(localCounts.readLockDepth == 1)
		mRWLock.lockReader(mCurrentWriter != PxThread::getId());
}

void NpScene::unlockRead()
{
	// increment this threads read depth
	ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
	if(localCounts.readLockDepth < 1)
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::unlockRead() called without matching call to PxScene::lockRead(), behaviour will be undefined.");
		return;
	}
	localCounts.readLockDepth--;
	PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// only unlock on last read
	if(localCounts.readLockDepth == 0)
		mRWLock.unlockReader();
}

void NpScene::lockWrite(const char* file, PxU32 line)
{
	// increment this threads write depth
	ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
	if (localCounts.writeLockDepth == 0 && localCounts.readLockDepth > 0)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, file?file:__FILE__, file?int(line):__LINE__, "PxScene::lockWrite() detected after a PxScene::lockRead(), lock upgrading is not supported, behaviour will be undefined.");
		return;
	}
	localCounts.writeLockDepth++;
	PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// only lock on first call
	if (localCounts.writeLockDepth == 1)
		mRWLock.lockWriter();

	PX_ASSERT(mCurrentWriter == 0 || mCurrentWriter == PxThread::getId());

	// set ourselves as the current writer
	mCurrentWriter = PxThread::getId();
}

void NpScene::unlockWrite()
{
	// increment this thread's write depth
	ThreadReadWriteCount localCounts (PxTlsGetValue(mThreadReadWriteDepth));
	if (localCounts.writeLockDepth < 1)
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::unlockWrite() called without matching call to PxScene::lockWrite(), behaviour will be undefined.");
		return;
	}
	localCounts.writeLockDepth--;
	PxTlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	PX_ASSERT(mCurrentWriter == PxThread::getId());

	if (localCounts.writeLockDepth == 0)
	{
		mCurrentWriter = 0;	
		mRWLock.unlockWriter();
	}
}

PxReal NpScene::getWakeCounterResetValue() const
{
	NP_READ_CHECK(this);

	return getWakeCounterResetValueInternal();
}

static PX_FORCE_INLINE void shiftRigidActor(PxRigidActor* a, const PxVec3& shift)
{
	const PxType t = a->getConcreteType();
	if(t==PxConcreteType::eRIGID_DYNAMIC)
	{
		static_cast<NpRigidDynamic*>(a)->getCore().onOriginShift(shift);
	}
	else if(t==PxConcreteType::eRIGID_STATIC)
	{
		static_cast<NpRigidStatic*>(a)->getCore().onOriginShift(shift);
	}
	else
	{
		PX_ASSERT(t == PxConcreteType::eARTICULATION_LINK);
		static_cast<NpArticulationLink*>(a)->getCore().onOriginShift(shift);
	}
}

template<typename T>
static void shiftRigidActors(PxArray<T*>& rigidActorList, const PxVec3& shift)
{
	const PxU32 prefetchLookAhead = 4;
	PxU32 rigidCount = rigidActorList.size();
	T*const* rigidActors = rigidActorList.begin();
	PxU32 batchIterCount = rigidCount / prefetchLookAhead;
	
	PxU32 idx = 0;
	for(PxU32 i=0; i < batchIterCount; i++)
	{
		// prefetch elements for next batch
		if (i < (batchIterCount-1))
		{
			PxPrefetchLine(rigidActors[idx + prefetchLookAhead]);
			PxPrefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead]) + 128);  
			PxPrefetchLine(rigidActors[idx + prefetchLookAhead + 1]);
			PxPrefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 1]) + 128);
			PxPrefetchLine(rigidActors[idx + prefetchLookAhead + 2]);
			PxPrefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 2]) + 128);
			PxPrefetchLine(rigidActors[idx + prefetchLookAhead + 3]);
			PxPrefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 3]) + 128);
		}
		else
		{
			for(PxU32 k=(idx + prefetchLookAhead); k < rigidCount; k++)
			{
				PxPrefetchLine(rigidActors[k]);
				PxPrefetchLine(reinterpret_cast<PxU8*>(rigidActors[k]) + 128);
			}
		}

		for(PxU32 j=idx; j < (idx + prefetchLookAhead); j++)
		{
			shiftRigidActor(rigidActors[j], shift);
		}

		idx += prefetchLookAhead;
	}
	// process remaining objects
	for(PxU32 i=idx; i < rigidCount; i++)
	{
		shiftRigidActor(rigidActors[i], shift);
	}
}

void NpScene::shiftOrigin(const PxVec3& shift)
{
	PX_PROFILE_ZONE("API.shiftOrigin", getContextId());
	NP_WRITE_CHECK(this);

	if(getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
	{
		NP_API_READ_WRITE_ERROR_MSG(
			"shiftOrigin() not allowed when direct-GPU API is used.");
	}

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::shiftOrigin() not allowed while simulation is running. Call will be ignored.")
	
	PX_SIMD_GUARD;

	shiftRigidActors(mRigidDynamics, shift);	// PT: TODO: we don't need to re-test the type all the time in shiftRigidActors...
	shiftRigidActors(mRigidStatics, shift);

	PxArticulationReducedCoordinate*const* articulations = mArticulations.getEntries();
	for(PxU32 i=0; i < mArticulations.size(); i++)
	{
		PxArticulationReducedCoordinate* np = (articulations[i]);
		
		NpArticulationLink*const* links = static_cast<NpArticulationReducedCoordinate*>(np)->getLinks();

		for(PxU32 j=0; j < np->getNbLinks(); j++)
		{
			shiftRigidActor(links[j], shift);
		}
	}

	mScene.shiftOrigin(shift);
	PVD_ORIGIN_SHIFT(shift);

	// shift scene query related data structures
	getSQAPI().shiftOrigin(shift);

#if PX_ENABLE_DEBUG_VISUALIZATION
	// debug visualization
	mRenderBuffer.shift(-shift);
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
}

PxPvdSceneClient* NpScene::getScenePvdClient()
{
#if PX_SUPPORT_PVD
	return &mScenePvdClient;
#else
	return NULL;
#endif
}

void NpScene::setDeformableSurfaceGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)
{
	mScene.setDeformableSurfaceGpuPostSolveCallback(postSolveCallback);
}

void NpScene::setDeformableVolumeGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)
{
	mScene.setDeformableVolumeGpuPostSolveCallback(postSolveCallback);
}

// PT: DIRECTGPU: deprecated
void NpScene::copySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent)
{
	PX_CHECK_SCENE_API_READ_FORBIDDEN(this, "PxScene::copySoftBodyData() not allowed while simulation is running. Call will be ignored.");
	
	//if ((mScene.getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && mScene.isUsingGpuRigidBodies())
		mScene.getSimulationController()->copySoftBodyDataDEPRECATED(data, dataSizes, softBodyIndices, flag, nbCopySoftBodies, maxSize, copyEvent);
}

// PT: DIRECTGPU: deprecated
void NpScene::applySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent)
{
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::applySoftBodyData() not allowed while simulation is running. Call will be ignored.");
	
	//if ((mScene.getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && mScene.isUsingGpuRigidBodies())
		mScene.getSimulationController()->applySoftBodyDataDEPRECATED(data, dataSizes, softBodyIndices, flag, nbUpdatedSoftBodies, maxSize, applyEvent, signalEvent);
}

// PT: DIRECTGPU: deprecated
void NpScene::applyParticleBufferData(const PxU32* indices, const PxGpuParticleBufferIndexPair* indexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, CUevent waitEvent, CUevent signalEvent)
{
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(this, "PxScene::applyParticleBufferData() not allowed while simulation is running. Call will be ignored.");

	if (!indices || !flags)
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::applyParticleBufferData, indices and/or flags has to be valid pointer.");
		return;
	}

	if (!isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::applyParticleBufferData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");
		return;
	}

	mScene.getSimulationController()->applyParticleBufferDataDEPRECATED(indices, indexPairs, flags, nbUpdatedBuffers, waitEvent, signalEvent);
}

PxDirectGPUAPI& NpScene::getDirectGPUAPI()
{
#if PX_SUPPORT_GPU_PHYSX
	if (!mDirectGPUAPI)
		mDirectGPUAPI = PX_NEW(NpDirectGPUAPI)(*this);

	return *mDirectGPUAPI;
#else
	PxDirectGPUAPI* p = NULL;
	return *p;
#endif
}

PxsSimulationController* NpScene::getSimulationController()
{
	return mScene.getSimulationController();
}

void NpScene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	NP_WRITE_CHECK(this);

	mScene.setActiveActors(actors, nbActors);
}

void NpScene::frameEnd()
{
#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
}

///////////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE PxU32 getShapes(NpRigidStatic& rigid, NpShape* const *& shapes)
{
	return NpRigidStaticGetShapes(rigid, shapes);
}

PX_FORCE_INLINE PxU32 getShapes(NpRigidDynamic& rigid, NpShape* const *& shapes)
{
	return NpRigidDynamicGetShapes(rigid, shapes);
}

PX_FORCE_INLINE PxU32 getShapes(NpArticulationLink& rigid, NpShape* const *& shapes)
{
	return NpArticulationGetShapes(rigid, shapes);
}

PX_FORCE_INLINE NpShape* getShape(NpShape* const* shapeArray, const PxU32 i)
{
	return shapeArray[i];
}

PX_FORCE_INLINE NpShape* getShape(Sc::ShapeCore* const* shapeArray, const PxU32 i)
{
	return static_cast<NpShape*>(shapeArray[i]->getPxShape());
}

template<class T>
PX_FORCE_INLINE static void addActorShapes(T* const* shapeArray, const PxU32 nbShapes, PxActor* pxActor, NpScene* scScene)
{
	PX_ASSERT(pxActor);
	PX_ASSERT(scScene);
	PX_ASSERT((0==nbShapes) || shapeArray);
	for (PxU32 i = 0; i < nbShapes; i++)
	{
		NpShape* npShape = getShape(shapeArray, i);
		PX_ASSERT(npShape);
		npShape->setSceneIfExclusive(scScene);
#if PX_SUPPORT_PVD
		scScene->getScenePvdClientInternal().createPvdInstance(npShape, *pxActor);
#else
		PX_UNUSED(pxActor);
#endif
	}
}

template<class T>
PX_FORCE_INLINE static void removeActorShapes(T* const* shapeArray, const PxU32 nbShapes, PxActor* pxActor, NpScene* scScene)
{
	PX_ASSERT(pxActor);
	PX_ASSERT(scScene);
	PX_ASSERT((0 == nbShapes) || shapeArray);
	for (PxU32 i = 0; i < nbShapes; i++)
	{
		NpShape* npShape = getShape(shapeArray, i);
		PX_ASSERT(npShape);
#if PX_SUPPORT_PVD
		scScene->getScenePvdClientInternal().releasePvdInstance(npShape, *pxActor);
#else
		PX_UNUSED(pxActor);
		PX_UNUSED(scScene);
#endif
		npShape->setSceneIfExclusive(NULL);
	}
}

static void addSimActorToScScene(Sc::Scene& s, NpRigidStatic& staticObject, NpShape* const* npShapes, PxU32 nbShapes, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	PX_UNUSED(bvh);
	const size_t shapePtrOffset = NpShape::getCoreOffset();
	s.addStatic(staticObject.getCore(), npShapes, nbShapes, shapePtrOffset, uninflatedBounds);
}

template <class T>
static void addSimActorToScSceneT(Sc::Scene& s, NpRigidBodyTemplate<T>& dynamicObject, NpShape* const* npShapes, PxU32 nbShapes, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	const bool isCompound = bvh ? true : false;
	const size_t shapePtrOffset = NpShape::getCoreOffset();
	s.addBody(dynamicObject.getCore(), npShapes, nbShapes, shapePtrOffset, uninflatedBounds, isCompound);
}

static void addSimActorToScScene(Sc::Scene& s, NpRigidDynamic& dynamicObject, NpShape* const* npShapes, PxU32 nbShapes, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	addSimActorToScSceneT<PxRigidDynamic>(s, dynamicObject, npShapes, nbShapes, uninflatedBounds, bvh);
}

static void addSimActorToScScene(Sc::Scene& s, NpArticulationLink& dynamicObject, NpShape* const* npShapes, PxU32 nbShapes, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	addSimActorToScSceneT<PxArticulationLink>(s, dynamicObject, npShapes, nbShapes, uninflatedBounds, bvh);
}

PX_FORCE_INLINE static void removeSimActorFromScScene(Sc::Scene& s, NpRigidStatic& staticObject, PxInlineArray<const Sc::ShapeCore*, 64>& scBatchRemovedShapes, bool wakeOnLostTouch)
{
	s.removeStatic(staticObject.getCore(), scBatchRemovedShapes, wakeOnLostTouch);
}

template <class T>
PX_FORCE_INLINE static void removeSimActorFromScSceneT(Sc::Scene& s, NpRigidBodyTemplate<T>& dynamicObject, PxInlineArray<const Sc::ShapeCore*, 64>& scBatchRemovedShapes, bool wakeOnLostTouch)
{
	s.removeBody(dynamicObject.getCore(), scBatchRemovedShapes, wakeOnLostTouch);
}

PX_FORCE_INLINE static void removeSimActorFromScScene(Sc::Scene& s, NpRigidDynamic& dynamicObject, PxInlineArray<const Sc::ShapeCore*, 64>& scBatchRemovedShapes, bool wakeOnLostTouch)
{
	removeSimActorFromScSceneT<PxRigidDynamic>(s, dynamicObject, scBatchRemovedShapes, wakeOnLostTouch);
}

PX_FORCE_INLINE static void removeSimActorFromScScene(Sc::Scene& s, NpArticulationLink& dynamicObject, PxInlineArray<const Sc::ShapeCore*, 64>& scBatchRemovedShapes, bool wakeOnLostTouch)
{
	removeSimActorFromScSceneT<PxArticulationLink>(s, dynamicObject, scBatchRemovedShapes, wakeOnLostTouch);
}

template <class T>
PX_FORCE_INLINE static void addSimActor(Sc::Scene& s, T& object, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	NpShape* const* npShapes = NULL;
	const PxU32 nbShapes = getShapes(object, npShapes);
	PX_ASSERT((0 == nbShapes) || npShapes);

	addSimActorToScScene(s, object, npShapes, nbShapes, uninflatedBounds, bvh);

	NpScene* scScene = object.getNpScene();
	addActorShapes(npShapes, nbShapes, &object, scScene);
}

template <class T>
PX_FORCE_INLINE static void removeSimActor(Sc::Scene& s, T& object, bool wakeOnLostTouch)
{
	NpScene* scScene = object.getNpScene();

	PxInlineArray<const Sc::ShapeCore*, 64> localShapes;
	PxInlineArray<const Sc::ShapeCore*, 64>& scBatchRemovedShapes = s.getBatchRemove() ? s.getBatchRemove()->removedShapes : localShapes;
	removeSimActorFromScScene(s, object, scBatchRemovedShapes, wakeOnLostTouch);
	Sc::ShapeCore* const* scShapes = const_cast<Sc::ShapeCore*const*>(scBatchRemovedShapes.begin());
	const PxU32 nbShapes = scBatchRemovedShapes.size();
	PX_ASSERT((0 == nbShapes) || scShapes);

	removeActorShapes(scShapes, nbShapes, &object, scScene);
}

template <const bool add, class T>
PX_FORCE_INLINE static void addOrRemoveNonSimActor(T& rigid)
{
	NpShape* const* npShapes = NULL;
	const PxU32 nbShapes = getShapes(rigid, npShapes);
	PX_ASSERT((0 == nbShapes) || npShapes);
	NpScene* scScene = rigid.getNpScene();
	PX_ASSERT(scScene);
	if(add)
		addActorShapes(npShapes, nbShapes, &rigid, scScene);
	else
		removeActorShapes(npShapes, nbShapes, &rigid, scScene);
}

template <typename T>struct ScSceneFns {};

#if PX_SUPPORT_GPU_PHYSX
template<> struct ScSceneFns<NpDeformableSurface>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpDeformableSurface& v, PxBounds3*, const Gu::BVH*, bool)
	{
		s.addDeformableSurface(v.getCore());
	}

	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpDeformableSurface& v, bool /*wakeOnLostTouch*/)
	{
		s.removeDeformableSurface(v.getCore());
	}
};

template<> struct ScSceneFns<NpDeformableVolume>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpDeformableVolume& v, PxBounds3*, const BVH*, bool)
	{
		s.addDeformableVolume(v.getCore());
	}

	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpDeformableVolume& v, bool /*wakeOnLostTouch*/)
	{
		s.removeDeformableVolume(v.getCore());
	}
};

template<> struct ScSceneFns<NpPBDParticleSystem>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpPBDParticleSystem& v, PxBounds3*, const BVH*, bool)
	{
		s.addParticleSystem(v.getCore());
	}

	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpPBDParticleSystem& v, bool /*wakeOnLostTouch*/)
	{
		s.removeParticleSystem(v.getCore());
	}
};

#endif

template<> struct ScSceneFns<NpArticulationReducedCoordinate>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationReducedCoordinate& v, PxBounds3*, const BVH*, bool)
	{ 
		s.addArticulation(v.getCore(), v.getRoot()->getCore());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationReducedCoordinate& v, bool /*wakeOnLostTouch*/)
	{
		s.removeArticulation(v.getCore());
	}
};

template<> struct ScSceneFns<NpArticulationJointReducedCoordinate>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationJointReducedCoordinate& v, PxBounds3*, const BVH*, bool)
	{ 
		s.addArticulationJoint(v.getCore(), v.getParent().getCore(), v.getChild().getCore());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationJointReducedCoordinate& v, bool /*wakeOnLostTouch*/)
	{
		s.removeArticulationJoint(v.getCore()); 
	}
};

template<> struct ScSceneFns<NpArticulationSpatialTendon>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationSpatialTendon& v, PxBounds3*, const BVH*, bool)
	{
		s.addArticulationTendon(v.getTendonCore());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationSpatialTendon& v, bool /*wakeOnLostTouch*/)
	{
		PX_UNUSED(s);
		Sc::Scene::removeArticulationTendon(v.getTendonCore());
	}
};

template<> struct ScSceneFns<NpArticulationFixedTendon>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationFixedTendon& v, PxBounds3*, const BVH*, bool)
	{
		s.addArticulationTendon(v.getTendonCore());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationFixedTendon& v, bool /*wakeOnLostTouch*/)
	{
		PX_UNUSED(s);
		Sc::Scene::removeArticulationTendon(v.getTendonCore());
	}
};

template<> struct ScSceneFns<NpArticulationMimicJoint>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationMimicJoint& v, PxBounds3*, const BVH*, bool)
	{
		s.addArticulationMimicJoint(v.getMimicJointCore());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationMimicJoint& v, bool /*wakeOnLostTouch*/)
	{
		PX_UNUSED(s);
		Sc::Scene::removeArticulationMimicJoint(v.getMimicJointCore());
	}
};

template<> struct ScSceneFns<NpConstraint>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpConstraint& v, PxBounds3*, const BVH*, bool)
	{ 
		PxRigidActor* a0, * a1;
		v.getActors(a0, a1);
		NpActor* sc0 = NpActor::getNpActor(a0);
		NpActor* sc1 = NpActor::getNpActor(a1);

		PX_ASSERT((!sc0) || (!(sc0->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))));
		PX_ASSERT((!sc1) || (!(sc1->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))));
		
		s.addConstraint(v.getCore(), sc0 ? &sc0->getScRigidCore() : NULL, sc1 ? &sc1->getScRigidCore() : NULL);
		OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxScene, constraints, static_cast<PxScene &>(*v.getNpScene()), static_cast<PxConstraint &>(v))
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpConstraint& v, bool /*wakeOnLostTouch*/)
	{
		s.removeConstraint(v.getCore());
		OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxScene, constraints, static_cast<PxScene &>(*v.getNpScene()), static_cast<PxConstraint &>(v))
	}
};

template<> struct ScSceneFns<NpRigidStatic>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpRigidStatic& v, PxBounds3* uninflatedBounds, const BVH* bvh, bool noSim)
	{
		PX_ASSERT(v.getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)==noSim);

		if(!noSim)
			addSimActor(s, v, uninflatedBounds, bvh);
		else
			addOrRemoveNonSimActor<true>(v);
	}

	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpRigidStatic& v, bool wakeOnLostTouch)
	{		
		if(!v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
			removeSimActor(s, v, wakeOnLostTouch);
		else
			addOrRemoveNonSimActor<false>(v);
	}
};

template<> struct ScSceneFns<NpRigidDynamic>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpRigidDynamic& v, PxBounds3* uninflatedBounds, const BVH* bvh, bool noSim)
	{
		PX_ASSERT(v.getCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)==noSim);

		if(!noSim)
			addSimActor(s, v, uninflatedBounds, bvh);
		else
			addOrRemoveNonSimActor<true>(v);
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpRigidDynamic& v, bool wakeOnLostTouch)	
	{
		if(!v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
			removeSimActor(s, v, wakeOnLostTouch);
		else
			addOrRemoveNonSimActor<false>(v);
	}
};

template<> struct ScSceneFns<NpArticulationLink>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, NpArticulationLink& v, PxBounds3* uninflatedBounds, const BVH* bvh, bool noSim)
	{
		PX_UNUSED(noSim);
		PX_ASSERT(!noSim);	// PT: the flag isn't supported on NpArticulationLink
		PX_ASSERT(!v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION));

		//if(!noSim)
			addSimActor(s, v, uninflatedBounds, bvh);
		//else
		//	addOrRemoveNonSimActor<true>(v);
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, NpArticulationLink& v, bool wakeOnLostTouch)	
	{
		PX_ASSERT(!v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION));
		//if(!v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
			removeSimActor(s, v, wakeOnLostTouch);
		//else
		//	addOrRemoveNonSimActor<false>(v);
	}
};

///////////////////////////////////////////////////////////////////////////////
#if PX_SUPPORT_PVD
template<typename T> struct PvdFns 
{	
	// PT: in the following functions, checkPvdDebugFlag() is done by the callers to save time when functions are called from a loop.

	static void createInstance(NpScene& scene, Vd::PvdSceneClient& d, T* v)
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", scene.getScScene().getContextId());
		PX_UNUSED(scene);
		d.createPvdInstance(v);
	}

	static void updateInstance(NpScene& scene, Vd::PvdSceneClient& d, T* v) 
	{ 
		PX_UNUSED(scene);
		{ 
			PX_PROFILE_ZONE("PVD.updatePVDProperties", scene.getScScene().getContextId());
			d.updatePvdProperties(v); 
		} 
	}

	static void releaseInstance(NpScene& scene, Vd::PvdSceneClient& d, T* v) 
	{ 
		PX_UNUSED(scene);
		PX_PROFILE_ZONE("PVD.releasePVDInstance", scene.getScScene().getContextId());
		d.releasePvdInstance(v); 
	}
};
#endif

///////////////////////////////////////////////////////////////////////////////

template<typename T>
static void add(NpScene* npScene, T& v, PxBounds3* uninflatedBounds=NULL, const BVH* bvh=NULL, bool noSim=false)
{
	PX_ASSERT(!npScene->isAPIWriteForbidden());

	v.setNpScene(npScene);

	ScSceneFns<T>::insert(npScene->getScScene(), v, uninflatedBounds, bvh, noSim);
#if PX_SUPPORT_PVD
	Vd::PvdSceneClient& pvdClient = npScene->getScenePvdClientInternal();
	if(pvdClient.checkPvdDebugFlag())
		PvdFns<T>::createInstance(*npScene, pvdClient, &v);
#endif
}

template<typename T>
static void remove(NpScene* npScene, T& v, bool wakeOnLostTouch=false)
{
	PX_ASSERT(!npScene->isAPIWriteForbidden());

	ScSceneFns<T>::remove(npScene->getScScene(), v, wakeOnLostTouch);
#if PX_SUPPORT_PVD
	Vd::PvdSceneClient& pvdClient = npScene->getScenePvdClientInternal();
	if(pvdClient.checkPvdDebugFlag())	
	   PvdFns<T>::releaseInstance(*npScene, pvdClient, &v);
#endif
	v.setNpScene(NULL);
}

template<class T>
static void removeRigidNoSimT(NpScene* npScene, T& v)
{
	PX_ASSERT(!npScene->isAPIWriteForbidden());

	PX_ASSERT(v.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION));

	addOrRemoveNonSimActor<false>(v);

#if PX_SUPPORT_PVD
	Vd::PvdSceneClient& pvdClient = npScene->getScenePvdClientInternal();
	if(pvdClient.checkPvdDebugFlag())
		PvdFns<T>::releaseInstance(*npScene, pvdClient, &v);
#else
	PX_UNUSED(npScene);
#endif
	v.setNpScene(NULL);
}

template<class T>
static PX_FORCE_INLINE void addActorT(NpScene* npScene, T& actor, bool noSim, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	PX_ASSERT(!npScene->isAPIWriteForbidden());
	PX_PROFILE_ZONE("API.addActorToSim", npScene->getScScene().getContextId());
	if(!noSim)
	{
		// PT: TODO: this codepath re-tests the sim flag and actually supports both cases!!!
		add<T>(npScene, actor, uninflatedBounds, bvh, noSim);
	}
	else
	{
		PX_ASSERT(actor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION));
		actor.setNpScene(npScene);

#if PX_SUPPORT_PVD
		Vd::PvdSceneClient& pvdClient = npScene->getScenePvdClientInternal();
		if(pvdClient.checkPvdDebugFlag())
			PvdFns<T>::createInstance(*npScene, pvdClient, &actor);		
#endif

		OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxScene, actors, static_cast<PxScene &>(*npScene), static_cast<PxActor &>(actor))

		addOrRemoveNonSimActor<true>(actor);
	}
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddActor(NpRigidStatic& rigidStatic, bool noSim, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	addActorT(this, rigidStatic, noSim, uninflatedBounds, bvh);
}

void NpScene::scAddActor(NpRigidDynamic& body, bool noSim, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	addActorT(this, body, noSim, uninflatedBounds, bvh);
}

void NpScene::scAddActor(NpArticulationLink& body, bool noSim, PxBounds3* uninflatedBounds, const BVH* bvh)
{
	addActorT(this, body, noSim, uninflatedBounds, bvh);
}

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: refactor scRemoveActor for NpRigidStatic & NpRigidDynamic
void NpScene::scRemoveActor(NpRigidStatic& rigidStatic, bool wakeOnLostTouch, bool noSim)
{
	PX_ASSERT(!isAPIWriteForbidden());

	PX_PROFILE_ZONE("API.removeActorFromSim", getScScene().getContextId());

	if(!noSim)
		remove<NpRigidStatic>(this, rigidStatic, wakeOnLostTouch);
	else
		removeRigidNoSimT(this, rigidStatic);
}

void NpScene::scRemoveActor(NpRigidDynamic& body, bool wakeOnLostTouch, bool noSim)
{
	PX_ASSERT(!isAPIWriteForbidden());

	PX_PROFILE_ZONE("API.removeActorFromSim", getScScene().getContextId());

	if(!noSim)
		remove<NpRigidDynamic>(this, body, wakeOnLostTouch);
	else
		removeRigidNoSimT(this, body);
}

void NpScene::scRemoveActor(NpArticulationLink& body, bool wakeOnLostTouch, bool noSim)
{
	PX_ASSERT(!noSim);
	PX_UNUSED(noSim);
	PX_ASSERT(!isAPIWriteForbidden());

	PX_PROFILE_ZONE("API.removeActorFromSim", getScScene().getContextId());

	remove<NpArticulationLink>(this, body, wakeOnLostTouch);
}

///////////////////////////////////////////////////////////////////////////////

#ifdef NEW_DIRTY_SHADERS_CODE
void NpScene::addDirtyConstraint(NpConstraint* constraint)
{
	PX_ASSERT(!constraint->isDirty());

	// PT: lock needed because PxConstraint::markDirty() can be called from multiple threads.
	// PT: TODO: consider optimizing this
	PxMutex::ScopedLock lock(mDirtyConstraintsLock);
	mDirtyConstraints.pushBack(constraint);
}
#endif

void NpScene::addToConstraintList(PxConstraint& constraint)
{
	NpConstraint& npConstraint = static_cast<NpConstraint&>(constraint);

	// AD: cannot call getFlags() directly because that clears out the GPU_COMPATIBLE flag!
	bool gpuCompatible = npConstraint.getCore().getFlags() & PxConstraintFlag::eGPU_COMPATIBLE;
	bool useDirectApi = getFlagsFast() & PxSceneFlag::eENABLE_DIRECT_GPU_API;
	bool opAllowed = gpuCompatible || !useDirectApi;
	if (!opAllowed)
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "It is not allowed to add a non-GPU compatible PxConstraint to a scene that is configured for direct-GPU access. Please use a D6 joint instead.");
		return;
	}

	add<NpConstraint>(this, npConstraint);

#ifdef NEW_DIRTY_SHADERS_CODE
	if(npConstraint.getCore().getFlags() & PxConstraintFlag::eALWAYS_UPDATE)
		mAlwaysUpdatedConstraints.pushBack(&npConstraint);
	else
	{
		// PT: mark all new constraints dirty to make sure their data is copied at least once
		mDirtyConstraints.pushBack(&npConstraint);
		npConstraint.getCore().setDirty();
	}
#endif
}

void NpScene::removeFromConstraintList(PxConstraint& constraint)
{
	PX_ASSERT(!isAPIWriteForbidden());

	NpConstraint& npConstraint = static_cast<NpConstraint&>(constraint);

#ifdef NEW_DIRTY_SHADERS_CODE
	// PT: TODO: consider optimizing this
	{
		if(npConstraint.getCore().isDirty())
			mDirtyConstraints.findAndReplaceWithLast(&npConstraint);

		if(npConstraint.getCore().getFlags() & PxConstraintFlag::eALWAYS_UPDATE)
			mAlwaysUpdatedConstraints.findAndReplaceWithLast(&npConstraint);
	}
#endif

	mScene.removeConstraint(npConstraint.getCore());

	// Release pvd constraint immediately since delayed removal with already released ext::joints does not work, can't call callback.
	RELEASE_PVD_INSTANCE(&npConstraint)

	npConstraint.setNpScene(NULL);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX

void NpScene::scAddDeformableSurface(NpScene* npScene, NpDeformableSurface& npSurface)
{
	add<NpDeformableSurface>(npScene, npSurface, NULL, NULL);
}

void NpScene::scRemoveDeformableSurface(NpDeformableSurface& npSurface)
{
	mScene.removeDeformableSurfaceSimControl(npSurface.getCore());
	remove<NpDeformableSurface>(this, npSurface, false);
}

////////////////////////////////////////////////////////////////////////////////

void NpScene::scAddDeformableVolume(NpDeformableVolume& npVolume)
{
	add<NpDeformableVolume>(this, npVolume);
}

void NpScene::scRemoveDeformableVolume(NpDeformableVolume& npVolume)
{
	mScene.removeDeformableVolumeSimControl(npVolume.getCore());
	remove<NpDeformableVolume>(this, npVolume);
}

////////////////////////////////////////////////////////////////////////////////

void NpScene::scAddParticleSystem(NpPBDParticleSystem& particleSystem)
{
	add<NpPBDParticleSystem>(this, particleSystem);
}

void NpScene::scRemoveParticleSystem(NpPBDParticleSystem& particleSystem)
{
	mScene.removeParticleSystemSimControl(particleSystem.getCore());
	remove<NpPBDParticleSystem>(this, particleSystem);
}

////////////////////////////////////////////////////////////////////////////////

void NpScene::addToAttachmentList(PxDeformableAttachment& attachment)
{
	NpDeformableAttachment& npAttachment = static_cast<NpDeformableAttachment&>(attachment);

	//TODO: Attachment does not have a sc layer implementation but could potentially benefit from implementing one to unify the disparate pieces of low level code.
	//add<NpDeformableAttachment>(this, npAttachment);

	npAttachment.addAttachment();
	npAttachment.setNpScene(npAttachment.getSceneFromActors());
}

void NpScene::removeFromAttachmentList(PxDeformableAttachment& attachment)
{
	NpDeformableAttachment& npAttachment = static_cast<NpDeformableAttachment&>(attachment);

	//TODO: Attachment does not have a sc layer implementation but could potentially benefit from implementing one to unify the disparate pieces of low level code.
	//remove<NpDeformableAttachment>(this, npAttachment);

	npAttachment.removeAttachment();
	npAttachment.setNpScene(NULL);
}

void NpScene::addToElementFilterList(PxDeformableElementFilter& elementFilter)
{
	NpDeformableElementFilter& npElementFilter = static_cast<NpDeformableElementFilter&>(elementFilter);

	//TODO: Element filter does not have a sc layer implementation but could potentially benefit from implementing one to unify the disparate pieces of low level code.
	//add<NpDeformableElementFilter>(this, npElementFilter);

	npElementFilter.addElementFilter();
	npElementFilter.setNpScene(npElementFilter.getSceneFromActors());
}

void NpScene::removeFromElementFilterList(PxDeformableElementFilter& elementFilter)
{
	NpDeformableElementFilter& npElementFilter = static_cast<NpDeformableElementFilter&>(elementFilter);

	//TODO: Element filter does not have a sc layer implementation but could potentially benefit from implementing one to unify the disparate pieces of low level code.
	//remove<NpDeformableElementFilter>(this, npElementFilter);

	npElementFilter.removeElementFilter();
	npElementFilter.setNpScene(NULL);
}
#endif

///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddArticulation(NpArticulationReducedCoordinate& articulation)
{
	add<NpArticulationReducedCoordinate>(this, articulation);
}

void NpScene::scRemoveArticulation(NpArticulationReducedCoordinate& articulation)
{
	mScene.removeArticulationSimControl(articulation.getCore());
	remove<NpArticulationReducedCoordinate>(this, articulation);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddArticulationJoint(NpArticulationJointReducedCoordinate& joint)
{
	add<NpArticulationJointReducedCoordinate>(this, joint);
}

void NpScene::scRemoveArticulationJoint(NpArticulationJointReducedCoordinate& joint)
{
	remove<NpArticulationJointReducedCoordinate>(this, joint);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddArticulationSpatialTendon(NpArticulationSpatialTendon& tendon)
{
	add<NpArticulationSpatialTendon>(this, tendon);
}

void NpScene::scRemoveArticulationSpatialTendon(NpArticulationSpatialTendon& tendon)
{
	remove<NpArticulationSpatialTendon>(this, tendon);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::scAddArticulationFixedTendon(NpArticulationFixedTendon& tendon)
{
	add<NpArticulationFixedTendon>(this, tendon);
}

void NpScene::scRemoveArticulationFixedTendon(NpArticulationFixedTendon& tendon)
{
	remove<NpArticulationFixedTendon>(this, tendon);
}

void NpScene::scAddArticulationMimicJoint(NpArticulationMimicJoint& mimicJoint)
{
	add<NpArticulationMimicJoint>(this, mimicJoint);
}
void NpScene::scRemoveArticulationMimicJoint(NpArticulationMimicJoint& mimicJoint)
{
	remove<NpArticulationMimicJoint>(this, mimicJoint);
}

#if PX_SUPPORT_OMNI_PVD
void NpScene::createInOmniPVD(const PxSceneDesc& desc)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, static_cast<PxScene &>(*this))

	getSceneOvdClientInternal().startFirstFrame(*pvdWriter); // Needs to have the PxScene pointer object set before the first startFrame

	// Create the PxGpuDynamicsMemoryConfig object
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, desc.gpuDynamicsConfig)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, tempBufferCapacity, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.tempBufferCapacity)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, maxRigidContactCount, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.maxRigidContactCount)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, maxRigidPatchCount, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.maxRigidPatchCount)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, heapCapacity, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.heapCapacity)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, foundLostPairsCapacity, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.foundLostPairsCapacity)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, foundLostAggregatePairsCapacity, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.foundLostAggregatePairsCapacity)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, totalAggregatePairsCapacity, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.totalAggregatePairsCapacity)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, maxDeformableSurfaceContacts, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.maxDeformableSurfaceContacts)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, maxDeformableVolumeContacts, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.maxDeformableVolumeContacts)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, maxParticleContacts, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.maxParticleContacts)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxGpuDynamicsMemoryConfig, collisionStackSize, desc.gpuDynamicsConfig, desc.gpuDynamicsConfig.collisionStackSize)


	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, gravity, static_cast<PxScene &>(*this), getGravity())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, flags,	static_cast<PxScene&>(*this), getFlags())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, frictionType,	static_cast<PxScene&>(*this), getFrictionType())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, broadPhaseType,	static_cast<PxScene&>(*this), getBroadPhaseType())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, kineKineFilteringMode,	static_cast<PxScene&>(*this), getKinematicKinematicFilteringMode())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, staticKineFilteringMode,	static_cast<PxScene&>(*this), getStaticKinematicFilteringMode())

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, solverType, static_cast<PxScene&>(*this), getSolverType())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, bounceThresholdVelocity, static_cast<PxScene&>(*this), getBounceThresholdVelocity())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, frictionOffsetThreshold, static_cast<PxScene&>(*this), getFrictionOffsetThreshold())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, frictionCorrelationDistance, static_cast<PxScene&>(*this), getFrictionCorrelationDistance())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, solverBatchSize, static_cast<PxScene&>(*this), getSolverBatchSize())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, solverArticulationBatchSize, static_cast<PxScene&>(*this), getSolverArticulationBatchSize())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, nbContactDataBlocks, static_cast<PxScene&>(*this), getNbContactDataBlocksUsed())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, maxNbContactDataBlocks, static_cast<PxScene&>(*this), getMaxNbContactDataBlocksUsed())//naming problem of functions
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, maxBiasCoefficient, static_cast<PxScene&>(*this), getMaxBiasCoefficient())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, contactReportStreamBufferSize, static_cast<PxScene&>(*this), getContactReportStreamBufferSize())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdMaxPasses, static_cast<PxScene&>(*this), getCCDMaxPasses())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdThreshold, static_cast<PxScene&>(*this), getCCDThreshold())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, ccdMaxSeparation, static_cast<PxScene&>(*this), getCCDMaxSeparation())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, wakeCounterResetValue, static_cast<PxScene&>(*this), getWakeCounterResetValue())
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbActors, static_cast<PxScene&>(*this), desc.limits.maxNbActors)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbBodies, static_cast<PxScene&>(*this), desc.limits.maxNbBodies)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbStaticShapes, static_cast<PxScene&>(*this), desc.limits.maxNbStaticShapes)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbDynamicShapes, static_cast<PxScene&>(*this), desc.limits.maxNbDynamicShapes)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbAggregates, static_cast<PxScene&>(*this), desc.limits.maxNbAggregates)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbConstraints, static_cast<PxScene&>(*this), desc.limits.maxNbConstraints)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbRegions, static_cast<PxScene&>(*this), desc.limits.maxNbRegions)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, limitsMaxNbBroadPhaseOverlaps, static_cast<PxScene&>(*this), desc.limits.maxNbBroadPhaseOverlaps)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasCPUDispatcher, static_cast<PxScene&>(*this), getCpuDispatcher() ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasCUDAContextManager, static_cast<PxScene&>(*this), getCudaContextManager()  ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasSimulationEventCallback, static_cast<PxScene&>(*this), getSimulationEventCallback() ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasContactModifyCallback, static_cast<PxScene&>(*this), getContactModifyCallback() ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasCCDContactModifyCallback, static_cast<PxScene&>(*this), getCCDContactModifyCallback() ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasBroadPhaseCallback, static_cast<PxScene&>(*this), getBroadPhaseCallback() ? true : false)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, hasFilterCallback, static_cast<PxScene&>(*this), getFilterCallback() ? true : false)
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, sanityBounds, static_cast<PxScene&>(*this), desc.sanityBounds)

	// Point to the PxGpuDynamicsMemoryConfig object
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, gpuDynamicsConfig, static_cast<PxScene&>(*this), static_cast<PxGpuDynamicsMemoryConfig const*>(&this->mGpuDynamicsConfig))
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, gpuMaxNumPartitions, static_cast<PxScene&>(*this), desc.gpuMaxNumPartitions)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, gpuMaxNumStaticPartitions, static_cast<PxScene&>(*this), desc.gpuMaxNumStaticPartitions)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, gpuComputeVersion, static_cast<PxScene&>(*this), desc.gpuComputeVersion)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, contactPairSlabSize, static_cast<PxScene&>(*this), desc.contactPairSlabSize)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, tolerancesScale, static_cast<PxScene&>(*this), desc.getTolerancesScale())

	OMNI_PVD_WRITE_SCOPE_END
}

#endif
