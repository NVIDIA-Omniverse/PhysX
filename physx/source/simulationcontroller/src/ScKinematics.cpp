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

#include "common/PxProfileZone.h"
#include "ScScene.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "PxsSimulationController.h"
#include "BpAABBManagerBase.h"

using namespace physx;
using namespace Sc;

//PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: consider using a non-member function for this one
void BodySim::calculateKinematicVelocity(PxReal oneOverDt)
{
	PX_ASSERT(isKinematic());
	
	/*------------------------------------------------\
	| kinematic bodies are moved directly by the user and are not influenced by external forces
	| we simply determine the distance moved since the last simulation frame and 
	| assign the appropriate delta to the velocity. This vel will be used to shove dynamic
	| objects in the solver.
	| We have to do this like so in a delayed way, because when the user sets the target pos the dt is not
	| yet known.
	\------------------------------------------------*/
	PX_ASSERT(isActive());

	BodyCore& core = getBodyCore();

	if (readInternalFlag(BF_KINEMATIC_MOVED))
	{
		clearInternalFlag(InternalFlags(BF_KINEMATIC_SETTLING | BF_KINEMATIC_SETTLING_2));
		const SimStateData* kData = getSimStateData(true);
		PX_ASSERT(kData);
		PX_ASSERT(kData->isKine());
		PX_ASSERT(kData->getKinematicData()->targetValid);
		PxVec3 linVelLL, angVelLL;
		const PxTransform targetPose = kData->getKinematicData()->targetPose;
		const PxTransform& currBody2World = getBody2World();

		//the kinematic target pose is now the target of the body (CoM) and not the actor.

		PxVec3 deltaPos = targetPose.p;
		deltaPos -= currBody2World.p;
		linVelLL = deltaPos * oneOverDt;

		PxQuat q = targetPose.q * currBody2World.q.getConjugate();

		if (q.w < 0)	//shortest angle.
			q = -q;

		PxReal angle;
 		PxVec3 axis;
		q.toRadiansAndUnitAxis(angle, axis);
		angVelLL = axis * angle * oneOverDt;

		core.getCore().linearVelocity = linVelLL;
		core.getCore().angularVelocity = angVelLL;

		// Moving a kinematic should trigger a wakeUp call on a higher level.
		PX_ASSERT(core.getWakeCounter()>0);
		PX_ASSERT(isActive());
		
	}
	else if (!readInternalFlag(BF_KINEMATIC_SURFACE_VELOCITY))
	{
		core.setLinearVelocity(PxVec3(0.0f), true);
		core.setAngularVelocity(PxVec3(0.0f), true);
	}
}

namespace
{
class ScKinematicUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*	mKinematics;
	const PxU32			mNbKinematics;
	const PxReal		mOneOverDt;

	PX_NOCOPY(ScKinematicUpdateTask)
public:

	static const PxU32 NbKinematicsPerTask = 1024;
	
	ScKinematicUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxReal oneOverDt, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mOneOverDt(oneOverDt)
	{
	}

	virtual void runInternal()
	{
		Sc::BodyCore*const*	kinematics = mKinematics;
		PxU32 nb = mNbKinematics;
		const float oneOverDt = mOneOverDt;

		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->calculateKinematicVelocity(oneOverDt);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicUpdateTask";
	}
};
}

void Sc::Scene::kinematicsSetup(PxBaseTask* continuation)
{
	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	if(!nbKinematics)
		return;

	BodyCore*const* kinematics = getActiveKinematicBodies();

	// PT: create a copy of active bodies for the taks to operate on while the main array is
	// potentially resized by operations running in parallel.
	if(mActiveKinematicsCopyCapacity<nbKinematics)
	{
		PX_FREE(mActiveKinematicsCopy);
		mActiveKinematicsCopy = PX_ALLOCATE(BodyCore*, nbKinematics, "Sc::Scene::mActiveKinematicsCopy");
		mActiveKinematicsCopyCapacity = nbKinematics;
	}
	PxMemCopy(mActiveKinematicsCopy, kinematics, nbKinematics*sizeof(BodyCore*));
	kinematics = mActiveKinematicsCopy;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	// PT: TASK-CREATION TAG
	// PT: TODO: better load balancing? This will be single threaded for less than 1K kinematics
	for(PxU32 i = 0; i < nbKinematics; i += ScKinematicUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicUpdateTask)), ScKinematicUpdateTask)
			(kinematics + i, PxMin(ScKinematicUpdateTask::NbKinematicsPerTask, nbKinematics - i), mOneOverDt, mContextId);

		task->setContinuation(continuation);
		task->removeReference();
	}

	if((mPublicFlags & PxSceneFlag::eENABLE_GPU_DYNAMICS))
	{
		// PT: running this serially for now because it's unsafe: mNPhaseCore->updateDirtyInteractions() (called after this)
		// can also call mSimulationController.updateDynamic() via BodySim::internalWakeUpBase
		PxU32 nb = nbKinematics;
		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			Sc::BodySim* bodySim = b->getSim();
			PX_ASSERT(!bodySim->getArticulation());
			mSimulationController->updateDynamic(NULL, bodySim->getNodeIndex());
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: consider using a non-member function for this one
void BodySim::updateKinematicPose()
{
	/*------------------------------------------------\
	| kinematic bodies are moved directly by the user and are not influenced by external forces
	| we simply determine the distance moved since the last simulation frame and 
	| assign the appropriate delta to the velocity. This vel will be used to shove dynamic
	| objects in the solver.
	| We have to do this like so in a delayed way, because when the user sets the target pos the dt is not
	| yet known.
	\------------------------------------------------*/

	PX_ASSERT(isKinematic());
	PX_ASSERT(isActive());

	if(readInternalFlag(BF_KINEMATIC_MOVED))
	{
		clearInternalFlag(InternalFlags(BF_KINEMATIC_SETTLING | BF_KINEMATIC_SETTLING_2));
		const SimStateData* kData = getSimStateData(true);
		PX_ASSERT(kData);
		PX_ASSERT(kData->isKine());
		PX_ASSERT(kData->getKinematicData()->targetValid);
		
		const PxTransform targetPose = kData->getKinematicData()->targetPose;
		getBodyCore().getCore().body2World = targetPose;
	}
}

namespace
{
class ScKinematicPoseUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*	mKinematics;
	const PxU32			mNbKinematics;

	PX_NOCOPY(ScKinematicPoseUpdateTask)
public:
	static const PxU32 NbKinematicsPerTask = 1024;

	ScKinematicPoseUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics)
	{
	}

	virtual void runInternal()
	{
		const PxU32 nb = mNbKinematics;

		for(PxU32 a=0; a<nb; ++a)
		{
			if ((a + 16) < nb)
			{
				PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 16]));

				if ((a + 4) < nb)
				{
					PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 4])->getSim());
					PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 4])->getSim()->getSimStateData_Unchecked());
				}
			}
			Sc::BodyCore* b = static_cast<Sc::BodyCore* const>(mKinematics[a]);
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());
			b->getSim()->updateKinematicPose();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScKinematicPoseUpdateTask";
	}
};
}

void Sc::Scene::integrateKinematicPose()
{
	PX_PROFILE_ZONE("Sim.integrateKinematicPose", mContextId);

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	// PT: TASK-CREATION TAG
	for(PxU32 i=0; i<nbKinematics; i+= ScKinematicPoseUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicPoseUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicPoseUpdateTask)), ScKinematicPoseUpdateTask)
			(kinematics + i, PxMin(nbKinematics - i, ScKinematicPoseUpdateTask::NbKinematicsPerTask), mContextId);
		task->setContinuation(&mAfterIntegration);
		task->removeReference();
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
class ScKinematicShapeUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	const PxU32				mNbKinematics;
	PxsTransformCache&		mCache;
	Bp::BoundsArray&		mBoundsArray;

	PX_NOCOPY(ScKinematicShapeUpdateTask)
public:
	static const PxU32 NbKinematicsShapesPerTask = 1024;

	ScKinematicShapeUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxsTransformCache& cache, Bp::BoundsArray& boundsArray, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mCache(cache), mBoundsArray(boundsArray)
	{
	}

	virtual void runInternal()
	{
		const PxU32 nb = mNbKinematics;
		for(PxU32 a=0; a<nb; ++a)
		{
			Sc::BodyCore* b = static_cast<Sc::BodyCore*>(mKinematics[a]);
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->updateCached(mCache, mBoundsArray);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicShapeUpdateTask";
	}
};
}

void Sc::Scene::updateKinematicCached(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateKinematicCached", mContextId);

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	if(!nbKinematics)
		return;

	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();
	
	PxU32 startIndex = 0;
	PxU32 nbShapes = 0;

	{
		PX_PROFILE_ZONE("ShapeUpdate", mContextId);

		// PT: TASK-CREATION TAG
		for(PxU32 i=0; i<nbKinematics; i++)
		{
			Sc::BodySim* sim = static_cast<Sc::BodyCore*>(kinematics[i])->getSim();
			PX_ASSERT(sim->isKinematic());
			PX_ASSERT(sim->isActive());

			nbShapes += sim->getNbShapes();

			if (nbShapes >= ScKinematicShapeUpdateTask::NbKinematicsShapesPerTask)
			{
				ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
					(kinematics + startIndex, (i + 1) - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

				task->setContinuation(continuation);
				task->removeReference();
				startIndex = i + 1;
				nbShapes = 0;
			}
		}

		if(nbShapes)
		{
			ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
				(kinematics + startIndex, nbKinematics - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	{
		PxBitMapPinned& changedAABBMap = mAABBManager->getChangedAABBMgActorHandleMap();
		mLLContext->getTransformCache().setChangedState();
		mBoundsArray->setChangedState();
		for (PxU32 i = 0; i < nbKinematics; ++i)
		{
			Sc::BodySim* bodySim = static_cast<Sc::BodyCore*>(kinematics[i])->getSim();

			if ((i+16) < nbKinematics)
			{
				PxPrefetchLine(kinematics[i + 16]);
				if ((i + 8) < nbKinematics)
				{
					PxPrefetchLine(kinematics[i + 8]->getSim());
				}
			}

			// PT: ### changedMap pattern #1
			PxU32 nbElems = bodySim->getNbElements();
			Sc::ElementSim** elems = bodySim->getElements();
			while (nbElems--)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
				//KS - TODO - can we parallelize this? The problem with parallelizing is that it's a bit operation,
				//so we would either need to use atomic operations or have some high-level concept that guarantees 
				//that threads don't write to the same word in the map simultaneously
				if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
					changedAABBMap.set(sim->getElementID());
			}

			mSimulationController->updateDynamic(NULL, bodySim->getNodeIndex());
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: consider using a non-member function for this one
bool BodySim::deactivateKinematic()
{
	BodyCore& core = getBodyCore();
	if(readInternalFlag(BF_KINEMATIC_SETTLING_2))
	{
		clearInternalFlag(BF_KINEMATIC_SETTLING_2);
		core.setWakeCounterFromSim(0);	// For sleeping objects the wake counter must be 0. This needs to hold for kinematics too.
		notifyReadyForSleeping();
		notifyPutToSleep();
		setActive(false);
		return true;
	}
	else if (readInternalFlag(BF_KINEMATIC_SETTLING))
	{
		clearInternalFlag(BF_KINEMATIC_SETTLING);
		raiseInternalFlag(BF_KINEMATIC_SETTLING_2);
	}
	else if (!readInternalFlag(BF_KINEMATIC_SURFACE_VELOCITY))
	{
		clearInternalFlag(BF_KINEMATIC_MOVED);
		raiseInternalFlag(BF_KINEMATIC_SETTLING);
	}
	return false;
}

// PT: called during fetchResults()
void Sc::Scene::postCallbacksPreSyncKinematics()
{
	PX_PROFILE_ZONE("Sim.postCallbacksPreSyncKinematics", mContextId);

	// Put/prepare kinematics to/for sleep and invalidate target pose
	// note: this needs to get done after the contact callbacks because
	//       the target might get read there.
	//
	PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	//KS - this method must run over the kinematic actors in reverse.
	while(nbKinematics--)
	{
		if(nbKinematics > 16)
		{
			PxPrefetchLine(static_cast<BodyCore*>(kinematics[nbKinematics-16]));
		}
		if (nbKinematics > 4)
		{
			PxPrefetchLine((static_cast<BodyCore*>(kinematics[nbKinematics - 4]))->getSim());
			PxPrefetchLine((static_cast<BodyCore*>(kinematics[nbKinematics - 4]))->getSim()->getSimStateData_Unchecked());
		}

		BodyCore* b = static_cast<BodyCore*>(kinematics[nbKinematics]);
		//kinematics++;
		PX_ASSERT(b->getSim()->isKinematic());
		PX_ASSERT(b->getSim()->isActive());

		b->invalidateKinematicTarget();
		b->getSim()->deactivateKinematic();
	}
}
