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

#include "common/PxProfileZone.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScArticulationSim.h"
#include "ScScene.h"

using namespace physx;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

void BodySim::addToSpeculativeCCDMap()
{
	if(mNodeIndex.isValid())
	{
		if(isArticulationLink())
			mScene.setSpeculativeCCDArticulationLink(mNodeIndex.index());
		else
			mScene.setSpeculativeCCDRigidBody(mNodeIndex.index());
	}
}

void BodySim::removeFromSpeculativeCCDMap()
{
	if(mNodeIndex.isValid())
	{
		if(isArticulationLink())
			mScene.resetSpeculativeCCDArticulationLink(mNodeIndex.index());
		else
			mScene.resetSpeculativeCCDRigidBody(mNodeIndex.index());
	}
}

// PT: TODO: consider using a non-member function for this one
void BodySim::updateContactDistance(PxReal* contactDistance, PxReal dt, const Bp::BoundsArray& boundsArray)
{
	const PxsRigidBody& llBody = getLowLevelBody();

	const PxRigidBodyFlags flags = llBody.getCore().mFlags;

	// PT: TODO: no need to test eENABLE_SPECULATIVE_CCD if we parsed mSpeculativeCCDRigidBodyBitMap initially 
	if((flags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) && !(llBody.mInternalFlags & PxsRigidBody::eFROZEN))
	{
		// PT: if both CCD flags are enabled we're in "hybrid mode" and we only use speculative contacts for the angular part
		const PxReal linearInflation = (flags & PxRigidBodyFlag::eENABLE_CCD) ? 0.0f : llBody.getLinearVelocity().magnitude() * dt;

		const float angVelMagTimesDt = llBody.getAngularVelocity().magnitude() * dt;

		PxU32 nbElems = getNbElements();
		ElementSim** elems = getElements();
		while(nbElems--)
		{
			ShapeSim* current = static_cast<ShapeSim*>(*elems++);

			const PxU32 index = current->getElementID();

			const PxBounds3& bounds = boundsArray.getBounds(index);

			const PxReal radius = bounds.getExtents().magnitude();

			//Heuristic for angular velocity...
			const PxReal angularInflation = angVelMagTimesDt * radius;

			contactDistance[index] = linearInflation + current->getContactOffset() + angularInflation;
		}
	}
}

void Sc::ArticulationSim::updateContactDistance(PxReal* contactDistance, PxReal dt, const Bp::BoundsArray& boundsArray)
{
	const PxU32 size = mBodies.size();
	for(PxU32 i=0; i<size; i++)
		mBodies[i]->updateContactDistance(contactDistance, dt, boundsArray);
}

namespace
{
class SpeculativeCCDBaseTask : public Cm::Task
{
	PX_NOCOPY(SpeculativeCCDBaseTask)
	public:
	const Bp::BoundsArray&	mBoundsArray;
	float*					mContactDistances;
	const float				mDt;

	SpeculativeCCDBaseTask(PxU64 contextID, const Bp::BoundsArray& boundsArray, PxReal* contactDistances, PxReal dt) :
		Cm::Task			(contextID),
		mBoundsArray		(boundsArray),
		mContactDistances	(contactDistances),
		mDt					(dt)
	{}
};

class SpeculativeCCDContactDistanceUpdateTask : public SpeculativeCCDBaseTask
{
public:
	static const PxU32 MaxBodies = 128;
	BodySim*	mBodySims[MaxBodies];
	PxU32		mNbBodies;

	SpeculativeCCDContactDistanceUpdateTask(PxU64 contextID, PxReal* contactDistances, PxReal dt, const Bp::BoundsArray& boundsArray) :
		SpeculativeCCDBaseTask	(contextID, boundsArray, contactDistances, dt),
		mNbBodies				(0)
	{}

	virtual void runInternal()
	{
		const PxU32 nb = mNbBodies;
		for(PxU32 i=0; i<nb; i++)
			mBodySims[i]->updateContactDistance(mContactDistances, mDt, mBoundsArray);
	}

	virtual const char* getName() const { return "SpeculativeCCDContactDistanceUpdateTask"; }

private:
	PX_NOCOPY(SpeculativeCCDContactDistanceUpdateTask)
};

class SpeculativeCCDContactDistanceArticulationUpdateTask : public SpeculativeCCDBaseTask
{
public:
	ArticulationSim* mArticulation;

	SpeculativeCCDContactDistanceArticulationUpdateTask(PxU64 contextID, PxReal* contactDistances, PxReal dt, const Bp::BoundsArray& boundsArray, ArticulationSim* sim) :
		SpeculativeCCDBaseTask	(contextID, boundsArray, contactDistances, dt),
		mArticulation			(sim)
	{}

	virtual void runInternal()
	{
		mArticulation->updateContactDistance(mContactDistances, mDt, mBoundsArray);
	}

	virtual const char* getName() const { return "SpeculativeCCDContactDistanceArticulationUpdateTask"; }

private:
	PX_NOCOPY(SpeculativeCCDContactDistanceArticulationUpdateTask)
};
}

static SpeculativeCCDContactDistanceUpdateTask* createCCDTask(Cm::FlushPool& pool, PxU64 contextID, PxReal* contactDistances, PxReal dt, const Bp::BoundsArray& boundsArray)
{
	return PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceUpdateTask)), SpeculativeCCDContactDistanceUpdateTask)(contextID, contactDistances, dt, boundsArray);
}

void Sc::Scene::updateContactDistances(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.updateContactDistances", mContextId);
	
	Cm::FlushPool& pool = mLLContext->getTaskPool();
	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();
	bool hasContactDistanceChanged = mHasContactDistanceChanged;

	// PT: TODO: it is quite unfortunate that we cannot shortcut parsing the bitmaps. Consider switching to arrays.
	// We remove sleeping bodies from the map but we never shrink it....

	// PT: TODO: why do we need to involve the island manager here?
	// PT: TODO: why do we do that on sleeping bodies? Why don't we use mActiveBodies?
	// PxArray<BodyCore*>			mActiveBodies;				// Sorted: kinematic before dynamic
	// ===> because we remove bodies from the bitmap in BodySim::deactivate()

	//calculate contact distance for speculative CCD shapes
	if(1)
	{
		PxBitMap::Iterator speculativeCCDIter(mSpeculativeCCDRigidBodyBitMap);

		SpeculativeCCDContactDistanceUpdateTask* ccdTask = createCCDTask(pool, mContextId, mContactDistance->begin(), mDt, *mBoundsArray);

		PxBitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

		const size_t bodyOffset = PX_OFFSET_OF_RT(BodySim, getLowLevelBody());

		//printf("\n");
		//PxU32 count = 0;

		PxU32 nbBodies = 0;
		PxU32 index;
		while((index = speculativeCCDIter.getNext()) != PxBitMap::Iterator::DONE)
		{
			PxsRigidBody* rigidBody = islandSim.getRigidBody(PxNodeIndex(index));
			BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(rigidBody)-bodyOffset);
			if(bodySim)
			{
				//printf("%d\n", bodySim->getActiveListIndex());
				//printf("%d: %d\n", count++, bodySim->isActive());

				hasContactDistanceChanged = true;
				ccdTask->mBodySims[nbBodies++] = bodySim;

				// PT: ### changedMap pattern #1
				// PT: TODO: isn't there a problem here? The task function will only touch the shapes whose body has the
				// speculative flag and isn't frozen, but here we mark all shapes as changed no matter what.
				//
				// Also we test some bodySim data and one bit of each ShapeSim here, not great.
				PxU32 nbElems = bodySim->getNbElements();
				ElementSim** elems = bodySim->getElements();
				while(nbElems--)
				{
					ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
					if(sim->getFlags() & PxShapeFlag::eSIMULATION_SHAPE)
						changedMap.growAndSet(sim->getElementID());
				}

				// PT: TODO: better load balancing?
				if(nbBodies == SpeculativeCCDContactDistanceUpdateTask::MaxBodies)
				{
					ccdTask->mNbBodies = nbBodies;
					nbBodies = 0;
					startTask(ccdTask, continuation);

					if(continuation)
						ccdTask = createCCDTask(pool, mContextId, mContactDistance->begin(), mDt, *mBoundsArray);
					else
						ccdTask->mNbBodies = 0;	// PT: no need to create a new task in single-threaded mode
				}
			}
		}

		if(nbBodies)
		{
			ccdTask->mNbBodies = nbBodies;
			startTask(ccdTask, continuation);
		}
	}
/*	else
	{
		// PT: codepath without mSpeculativeCCDRigidBodyBitMap
		PxU32 nb = mActiveBodies.size();
		BodyCore** bodies = mActiveBodies.begin();

		while(nb--)
		{
			const BodyCore* current = *bodies++;
			BodySim* bodySim = current->getSim();
			if(bodySim)
			{
				...
			}
		}
	}*/

	//calculate contact distance for articulation links
	{
		PxBitMap::Iterator articulateCCDIter(mSpeculativeCDDArticulationBitMap);
		PxU32 index;
		while((index = articulateCCDIter.getNext()) != PxBitMap::Iterator::DONE)
		{
			ArticulationSim* articulationSim = islandSim.getArticulationSim(PxNodeIndex(index));
			if(articulationSim)
			{
				hasContactDistanceChanged = true;
				if(continuation)
				{
					SpeculativeCCDContactDistanceArticulationUpdateTask* articulationUpdateTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceArticulationUpdateTask)), SpeculativeCCDContactDistanceArticulationUpdateTask)(mContextId, mContactDistance->begin(), mDt, *mBoundsArray, articulationSim);
					articulationUpdateTask->setContinuation(continuation);
					articulationUpdateTask->removeReference();
				}
				else
				{
					articulationSim->updateContactDistance(mContactDistance->begin(), mDt, *mBoundsArray);
				}
			}
		}
	}

	mHasContactDistanceChanged = hasContactDistanceChanged;
}

///////////////////////////////////////////////////////////////////////////////

#include "ScNPhaseCore.h"
#include "ScShapeInteraction.h"
#include "PxsCCD.h"
#include "PxsSimulationController.h"
#include "CmTransformUtils.h"

void Sc::Scene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	mCCDContext->setCCDContactModifyCallback(callback);
}

PxCCDContactModifyCallback* Sc::Scene::getCCDContactModifyCallback() const
{
	return mCCDContext->getCCDContactModifyCallback();
}

void Sc::Scene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	mCCDContext->setCCDMaxPasses(ccdMaxPasses);
}

PxU32 Sc::Scene::getCCDMaxPasses() const
{
	return mCCDContext->getCCDMaxPasses();
}

void Sc::Scene::setCCDThreshold(PxReal t)
{
	mCCDContext->setCCDThreshold(t);
}

PxReal Sc::Scene::getCCDThreshold() const
{
	return mCCDContext->getCCDThreshold();
}

void Sc::Scene::collectPostSolverVelocitiesBeforeCCD()
{
	if(mContactReportsNeedPostSolverVelocity)
	{
		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if(i < (nbActorPairs - 1))
				PxPrefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];

			ContactStreamManager& cs = aPair->getContactStreamManager();

			PxU32 streamManagerFlag = cs.getFlags();
			if(streamManagerFlag & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;

			PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
			
			if(i + 1 < nbActorPairs)
				PxPrefetch(&(actorPairs[i+1]->getContactStreamManager()));

			if(!cs.extraDataSize)
				continue;
			else if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
				cs.setContactReportPostSolverVelocity(stream, aPair->getActorA(), aPair->getActorB());
		}
	}
}

void Sc::Scene::updateCCDMultiPass(PxBaseTask* parentContinuation)
{
	getCcdBodies().forceSize_Unsafe(mSimulationControllerCallback->getNbCcdBodies());
	
	// second run of the broadphase for making sure objects we have integrated did not tunnel.
	if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
	{
		if(mContactReportsNeedPostSolverVelocity)
		{
			// the CCD code will overwrite the post solver body velocities, hence, we need to extract the info
			// first if any CCD enabled pair requested it.
			collectPostSolverVelocitiesBeforeCCD();
		}

		//We use 2 CCD task chains to be able to chain together an arbitrary number of ccd passes
		if(mPostCCDPass.size() != 2)
		{
			mPostCCDPass.clear();
			mUpdateCCDSinglePass.clear();
			mCCDBroadPhase.clear();
			mCCDBroadPhaseAABB.clear();
			mPostCCDPass.reserve(2);
			mUpdateCCDSinglePass.reserve(2);
			mUpdateCCDSinglePass2.reserve(2);
			mUpdateCCDSinglePass3.reserve(2);
			mCCDBroadPhase.reserve(2);
			mCCDBroadPhaseAABB.reserve(2);
			for (int j = 0; j < 2; j++)
			{
				mPostCCDPass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::postCCDPass>(mContextId, this, "ScScene.postCCDPass"));
				mUpdateCCDSinglePass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePass>(mContextId, this, "ScScene.updateCCDSinglePass"));
				mUpdateCCDSinglePass2.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage2>(mContextId, this, "ScScene.updateCCDSinglePassStage2"));
				mUpdateCCDSinglePass3.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage3>(mContextId, this, "ScScene.updateCCDSinglePassStage3"));
				mCCDBroadPhase.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhase>(mContextId, this, "ScScene.ccdBroadPhase"));
				mCCDBroadPhaseAABB.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhaseAABB>(mContextId, this, "ScScene.ccdBroadPhaseAABB"));
			}
		}

		//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
		mLLContext->resetThreadContexts();

		mCCDContext->updateCCDBegin();

		mCCDBroadPhase[0].setContinuation(parentContinuation);
		mCCDBroadPhaseAABB[0].setContinuation(&mCCDBroadPhase[0]);
		mCCDBroadPhase[0].removeReference();
		mCCDBroadPhaseAABB[0].removeReference();
	}
}

namespace
{
class UpdateCCDBoundsTask : public Cm::Task
{
	Bp::BoundsArray*	mBoundArray;
	PxsTransformCache*	mTransformCache;
	BodySim**			mBodySims;
	PxU32				mNbToProcess;
	PxI32*				mNumFastMovingShapes;

public:

	static const PxU32 MaxPerTask = 256;

	UpdateCCDBoundsTask(PxU64 contextID, Bp::BoundsArray* boundsArray, PxsTransformCache* transformCache, BodySim** bodySims, PxU32 nbToProcess, PxI32* numFastMovingShapes) :
		Cm::Task			(contextID),
		mBoundArray			(boundsArray),
		mTransformCache		(transformCache),
		mBodySims			(bodySims), 
		mNbToProcess		(nbToProcess),
		mNumFastMovingShapes(numFastMovingShapes)
	{
	}

	virtual const char* getName() const { return "UpdateCCDBoundsTask";}

	PxIntBool	updateSweptBounds(ShapeSim* sim, BodySim* body)
	{
		PX_ASSERT(body==sim->getBodySim());

		const PxU32 elementID = sim->getElementID();

		const ShapeCore& shapeCore = sim->getCore();
		const PxTransform& endPose = mTransformCache->getTransformCache(elementID).transform;

		const PxGeometry& shapeGeom = shapeCore.getGeometry();

		const PxsRigidBody& rigidBody = body->getLowLevelBody();
		const PxsBodyCore& bodyCore = body->getBodyCore().getCore();
		PX_ALIGN(16, PxTransform shape2World);
		Cm::getDynamicGlobalPoseAligned(rigidBody.mLastTransform, shapeCore.getShape2Actor(), bodyCore.getBody2Actor(), shape2World);

		const float ccdThreshold = computeCCDThreshold(shapeGeom);
		PxBounds3 bounds = Gu::computeBounds(shapeGeom, endPose);
		PxIntBool isFastMoving;
		if(1)
		{
			// PT: this alternative implementation avoids computing the start bounds for slow moving objects.
			isFastMoving = (shape2World.p - endPose.p).magnitudeSquared() >= ccdThreshold * ccdThreshold ? 1 : 0;
			if(isFastMoving)
			{
				const PxBounds3 startBounds = Gu::computeBounds(shapeGeom, shape2World);
				bounds.include(startBounds);
			}
		}
		else
		{
			const PxBounds3 startBounds = Gu::computeBounds(shapeGeom, shape2World);

			isFastMoving = (startBounds.getCenter() - bounds.getCenter()).magnitudeSquared() >= ccdThreshold * ccdThreshold ? 1 : 0;

			if(isFastMoving)
				bounds.include(startBounds);
		}

		PX_ASSERT(bounds.minimum.x <= bounds.maximum.x
			&&	  bounds.minimum.y <= bounds.maximum.y
			&&	  bounds.minimum.z <= bounds.maximum.z);

		mBoundArray->setBounds(bounds, elementID);

		return isFastMoving;
	}

	virtual void runInternal()
	{
		PxU32 activeShapes = 0;
		const PxU32 nb = mNbToProcess;
		for(PxU32 i=0; i<nb; i++)
		{
			PxU32 isFastMoving = 0;
			BodySim& bodySim = *mBodySims[i];

			PxU32 nbElems = bodySim.getNbElements();
			ElementSim** elems = bodySim.getElements();
			while(nbElems--)
			{
				ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
				if(sim->getFlags() & PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
				{
					const PxIntBool fastMovingShape = updateSweptBounds(sim, &bodySim);
					activeShapes += fastMovingShape;

					isFastMoving = isFastMoving | fastMovingShape;
				}
			}

			bodySim.getLowLevelBody().getCore().isFastMoving = isFastMoving!=0;
		}

		PxAtomicAdd(mNumFastMovingShapes, PxI32(activeShapes));
	}
};
}

void Sc::Scene::ccdBroadPhaseAABB(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Sim.ccdBroadPhaseComplete", mContextId);
	PX_PROFILE_ZONE("Sim.ccdBroadPhaseAABB", mContextId);
	PX_UNUSED(continuation);

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mNumFastMovingShapes = 0;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if(currentPass == 0 || mCCDContext->getNumSweepHits())
	{
		PxsTransformCache& transformCache = getLowLevelContext()->getTransformCache();
		for(PxU32 i = 0; i < mCcdBodies.size(); i+= UpdateCCDBoundsTask::MaxPerTask)
		{
			const PxU32 nbToProcess = PxMin(UpdateCCDBoundsTask::MaxPerTask, mCcdBodies.size() - i);
			UpdateCCDBoundsTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateCCDBoundsTask)), UpdateCCDBoundsTask)(mContextId, mBoundsArray, &transformCache, &mCcdBodies[i], nbToProcess, &mNumFastMovingShapes);
			task->setContinuation(continuation);
			task->removeReference();
		}
	}
}

void Sc::Scene::ccdBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.ccdBroadPhase", mContextId);

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	const PxU32 ccdMaxPasses = mCCDContext->getCCDMaxPasses();
	mCCDPass = currentPass+1;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if( (currentPass == 0 || mCCDContext->getNumSweepHits()) && mNumFastMovingShapes != 0)
	{
		const PxU32 currIndex = currentPass & 1;
		const PxU32 nextIndex = 1 - currIndex;
		//Initialize the CCD task chain unless this is the final pass
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].setContinuation(continuation);
			mCCDBroadPhaseAABB[nextIndex].setContinuation(&mCCDBroadPhase[nextIndex]);
		}
		mPostCCDPass[currIndex].setContinuation(currentPass == ccdMaxPasses-1 ? continuation : &mCCDBroadPhaseAABB[nextIndex]);
		mUpdateCCDSinglePass3[currIndex].setContinuation(&mPostCCDPass[currIndex]);
		mUpdateCCDSinglePass2[currIndex].setContinuation(&mUpdateCCDSinglePass3[currIndex]);
		mUpdateCCDSinglePass[currIndex].setContinuation(&mUpdateCCDSinglePass2[currIndex]);

		//Do the actual broad phase
		PxBaseTask* continuationTask = &mUpdateCCDSinglePass[currIndex];
//		const PxU32 numCpuTasks = continuationTask->getTaskManager()->getCpuDispatcher()->getWorkerCount();

		mCCDBp = true;

		mBpSecondPass.setContinuation(continuationTask);
		mBpFirstPass.setContinuation(&mBpSecondPass);

		mBpSecondPass.removeReference();
		mBpFirstPass.removeReference();
		
		//mAABBManager->updateAABBsAndBP(numCpuTasks, mLLContext->getTaskPool(), &mLLContext->getScratchAllocator(), false, continuationTask, NULL);

		//Allow the CCD task chain to continue
		mPostCCDPass[currIndex].removeReference();
		mUpdateCCDSinglePass3[currIndex].removeReference();
		mUpdateCCDSinglePass2[currIndex].removeReference();
		mUpdateCCDSinglePass[currIndex].removeReference();
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].removeReference();
			mCCDBroadPhaseAABB[nextIndex].removeReference();
		}
	}
	else if (currentPass == 0)
	{
		PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", mContextId);
		mCCDContext->resetContactManagers();
	}
}

void Sc::Scene::updateCCDSinglePass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePass", mContextId);
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	mAABBManager->postBroadPhase(NULL, *getFlushPool());
	finishBroadPhase(continuation);
	
	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	if(currentPass == 1)		// reset the handle map so we only update CCD objects from here on
	{
		PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
		//changedAABBMgrActorHandles.clear();
		for(PxU32 i = 0; i < mCcdBodies.size();i++)
		{
			// PT: ### changedMap pattern #1
			PxU32 nbElems = mCcdBodies[i]->getNbElements();
			ElementSim** elems = mCcdBodies[i]->getElements();
			while(nbElems--)
			{
				ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
				if(sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
					changedAABBMgrActorHandles.growAndSet(sim->getElementID());
			}
		}
	}
}

void Sc::Scene::updateCCDSinglePassStage2(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage2", mContextId);
	postBroadPhaseStage2(continuation);
}

void Sc::Scene::updateCCDSinglePassStage3(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage3", mContextId);
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	finishBroadPhaseStage2(currentPass);
	PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", mContextId);

	//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
	mLLContext->resetThreadContexts();

	mCCDContext->updateCCD(mDt, continuation, mSimpleIslandManager->getAccurateIslandSim(), (mPublicFlags & PxSceneFlag::eDISABLE_CCD_RESWEEP), mNumFastMovingShapes);
}

static PX_FORCE_INLINE Sc::ShapeInteraction* getSI(PxvContactManagerTouchEvent& evt)
{
	return reinterpret_cast<Sc::ShapeInteraction*>(evt.getCMTouchEventUserData());
}

void Sc::Scene::postCCDPass(PxBaseTask* /*continuation*/)
{
	// - Performs sleep check
	// - Updates touch flags

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	PX_ASSERT(currentPass > 0); // to make sure changes to the CCD pass counting get noticed. For contact reports, 0 means discrete collision phase.

	int newTouchCount, lostTouchCount, ccdTouchCount;
	mLLContext->getManagerTouchEventCount(&newTouchCount, &lostTouchCount, &ccdTouchCount);
	PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
	PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);
	PX_ALLOCA(ccdTouches, PxvContactManagerTouchEvent, ccdTouchCount);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	// Note: For contact notifications it is important that the new touch pairs get processed before the lost touch pairs.
	//       This allows to know for sure if a pair of actors lost all touch (see eACTOR_PAIR_LOST_TOUCH).
	mLLContext->fillManagerTouchEvents(newTouches, newTouchCount, lostTouches, lostTouchCount, ccdTouches, ccdTouchCount);
	for(PxI32 i=0; i<newTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(newTouches[i]);
		PX_ASSERT(si);
		mNPhaseCore->managerNewTouch(*si);
		si->managerNewTouch(currentPass, true, outputs);
		if (!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
		{
			mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex(), IG::Edge::eCONTACT_MANAGER);
		}
	}
	for(PxI32 i=0; i<lostTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(lostTouches[i]);
		PX_ASSERT(si);
		if (si->managerLostTouch(currentPass, true, outputs) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getShape0().getActor(), si->getShape1().getActor());

		mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
	}
	for(PxI32 i=0; i<ccdTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(ccdTouches[i]);
		PX_ASSERT(si);
		si->sendCCDRetouch(currentPass, outputs);
	}
	checkForceThresholdContactEvents(currentPass);
	{
		PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

		for (PxU32 i = 0, s = mCcdBodies.size(); i < s; i++)
		{
			BodySim*const body = mCcdBodies[i];
			if(i+8 < s)
				PxPrefetch(mCcdBodies[i+8], 512);

			PX_ASSERT(body->getBody2World().p.isFinite());
			PX_ASSERT(body->getBody2World().q.isFinite());

			body->updateCached(&changedAABBMgrActorHandles);
		}

		ArticulationCore* const* articList = mArticulations.getEntries();
		for(PxU32 i=0;i<mArticulations.size();i++)
			articList[i]->getSim()->updateCached(&changedAABBMgrActorHandles);
	}
}
