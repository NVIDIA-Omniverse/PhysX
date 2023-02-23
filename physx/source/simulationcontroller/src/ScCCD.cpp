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
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScArticulationSim.h"
#include "ScScene.h"

using namespace physx;
using namespace Sc;

// PT: TODO: consider using a non-member function for this one
void BodySim::updateContactDistance(PxReal* contactDistance, const PxReal dt, const Bp::BoundsArray& boundsArray)
{
	const PxsRigidBody& llBody = getLowLevelBody();

	const PxRigidBodyFlags flags = llBody.getCore().mFlags;

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

// PT: TODO: merge this with task code
void Sc::ArticulationSim::updateContactDistance(PxReal* contactDistance, const PxReal dt, const Bp::BoundsArray& boundsArray)
{
	const PxU32 size = mBodies.size();
	for(PxU32 i=0; i<size; i++)
		mBodies[i]->updateContactDistance(contactDistance, dt, boundsArray);
}

class SpeculativeCCDBaseTask : public Cm::Task
{
	PX_NOCOPY(SpeculativeCCDBaseTask)
	public:
	const Bp::BoundsArray&	mBoundsArray;
	PxReal*					mContactDistances;
	const PxReal			mDt;

	SpeculativeCCDBaseTask(PxU64 contextID, const Bp::BoundsArray& boundsArray, PxReal* contactDistances, const PxReal dt) :
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
	BodySim* mBodySims[MaxBodies];
	PxU32 mNbBodies;

	SpeculativeCCDContactDistanceUpdateTask(PxU64 contextID, PxReal* contactDistances, const PxReal dt, const Bp::BoundsArray& boundsArray) :
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

	SpeculativeCCDContactDistanceArticulationUpdateTask(PxU64 contextID, PxReal* contactDistances, const PxReal dt, const Bp::BoundsArray& boundsArray, ArticulationSim* sim) :
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

static PX_FORCE_INLINE	void startTask(Cm::Task* task, PxBaseTask* continuation)
{
	task->setContinuation(continuation);
	task->removeReference();
}

void Sc::Scene::updateContactDistances(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.updateContactDistances", getContextId());
	
	Cm::FlushPool& pool = mLLContext->getTaskPool();
	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();
	PxU32 index;
	bool hasContactDistanceChanged = mHasContactDistanceChanged;

	//calculate contact distance for speculative CCD shapes
	{
		PxBitMap::Iterator speculativeCCDIter(mSpeculativeCCDRigidBodyBitMap);

		SpeculativeCCDContactDistanceUpdateTask* ccdTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceUpdateTask)), SpeculativeCCDContactDistanceUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray);

		PxBitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

		const size_t bodyOffset = PX_OFFSET_OF_RT(BodySim, getLowLevelBody());

		while((index = speculativeCCDIter.getNext()) != PxBitMap::Iterator::DONE)
		{
			PxsRigidBody* rigidBody = islandSim.getRigidBody(PxNodeIndex(index));
			BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(rigidBody)-bodyOffset);
			if (bodySim)
			{
				hasContactDistanceChanged = true;
				ccdTask->mBodySims[ccdTask->mNbBodies++] = bodySim;

				// PT: ### changedMap pattern #1
				// PT: TODO: isn't there a problem here? The task function will only touch the shapes whose body has the
				// speculative flag and isn't frozen, but here we mark all shapes as changed no matter what.
				PxU32 nbElems = bodySim->getNbElements();
				ElementSim** elems = bodySim->getElements();
				while(nbElems--)
				{
					ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
					if(sim->getFlags() & PxShapeFlag::eSIMULATION_SHAPE)
						changedMap.growAndSet(sim->getElementID());
				}

				if(ccdTask->mNbBodies == SpeculativeCCDContactDistanceUpdateTask::MaxBodies)
				{
					startTask(ccdTask, continuation);
					ccdTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceUpdateTask)), SpeculativeCCDContactDistanceUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray);
				}
			}
		}

		if(ccdTask->mNbBodies)
			startTask(ccdTask, continuation);
	}

	//calculate contact distance for articulation links
	{
		SpeculativeCCDContactDistanceArticulationUpdateTask* articulationUpdateTask = NULL;

		PxBitMap::Iterator articulateCCDIter(mSpeculativeCDDArticulationBitMap);
		while((index = articulateCCDIter.getNext()) != PxBitMap::Iterator::DONE)
		{
			ArticulationSim* articulationSim = islandSim.getArticulationSim(PxNodeIndex(index));
			if(articulationSim)
			{
				hasContactDistanceChanged = true;
				articulationUpdateTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceArticulationUpdateTask)), SpeculativeCCDContactDistanceArticulationUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray, articulationSim);
				startTask(articulationUpdateTask, continuation);
			}
		}
	}

	mHasContactDistanceChanged = hasContactDistanceChanged;
}
