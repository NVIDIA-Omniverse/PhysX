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

#include "ScArticulationSim.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointSim.h"
#include "ScArticulationJointCore.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"
#include "ScArticulationTendonSim.h"
#include "ScArticulationSensorSim.h"
#include "ScArticulationSensor.h"
#include "ScScene.h"

#include "DyConstraint.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsContext.h"
#include "CmSpatialVector.h"
#include "foundation/PxVecMath.h"
#include "PxsSimpleIslandManager.h"
#include "ScShapeSim.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace physx::Dy;

Sc::ArticulationSim::ArticulationSim(ArticulationCore& core, Scene& scene, BodyCore& root) : 
	mLLArticulation	(NULL),
	mScene			(scene),
	mCore			(core),
	mLinks			("ScArticulationSim::links"),
	mBodies			("ScArticulationSim::bodies"),
	mJoints			("ScArticulationSim::joints"),
	mMaxDepth		(0),
	mIsLLArticultionInitialized(false)
{
	mLinks.reserve(16);
	mJoints.reserve(16);
	mBodies.reserve(16);

	mLLArticulation = mScene.createLLArticulation(this);
	
	mIslandNodeIndex = scene.getSimpleIslandManager()->addArticulation(this, mLLArticulation, false);

	if(!mLLArticulation)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Articulation: could not allocate low-level resources.");
		return;
	}

	PX_ASSERT(root.getSim());

	addBody(*root.getSim(), NULL, NULL);

	mCore.setSim(this);

	mLLArticulation->setDyContext(mScene.getDynamicsContext());
	mLLArticulation->getSolverDesc().initData(&core.getCore(), NULL);

	//mLLArticulation->onUpdateSolverDesc();
}

Sc::ArticulationSim::~ArticulationSim()
{
	if (!mLLArticulation)
		return;

	mScene.destroyLLArticulation(*mLLArticulation);

	mScene.getSimpleIslandManager()->removeNode(mIslandNodeIndex);

	mCore.setSim(NULL);
}

PxU32 Sc::ArticulationSim::findBodyIndex(BodySim& body) const
{
	for(PxU32 i=0; i<mBodies.size(); i++)
	{
		if(mBodies[i]==&body)
			return i;
	}
	PX_ASSERT(0);
	return 0x80000000;
}

void Sc::ArticulationSim::addLoopConstraint(ConstraintSim* constraintSim)
{
	const PxU32 size = mLoopConstraints.size();
	if (size < mLoopConstraints.size())
		mLoopConstraints.reserve(size*2 + 1);
	
	BodySim* bodySim0 = constraintSim->getBody(0);
	BodySim* bodySim1 = constraintSim->getBody(1);

	ArticulationLoopConstraint lConstraint;
	if (bodySim0)
		lConstraint.linkIndex0 = findBodyIndex(*bodySim0);
	else
		lConstraint.linkIndex0 = 0x80000000;

	if(bodySim1)
		lConstraint.linkIndex1 = findBodyIndex(*bodySim1);
	else
		lConstraint.linkIndex1 = 0x80000000;

	lConstraint.constraint = &constraintSim->getLowLevelConstraint();

	mLoopConstraints.pushBack(lConstraint);
}

void Sc::ArticulationSim::removeLoopConstraint(ConstraintSim* constraintSim)
{
	Dy::Constraint* constraint = &constraintSim->getLowLevelConstraint();

	const PxU32 size = mLoopConstraints.size();
	PxU32 index = 0;
	while (index < size && mLoopConstraints[index].constraint != constraint)
		++index;

	if (index != size)
		mLoopConstraints.replaceWithLast(index);
}

void Sc::ArticulationSim::updateCached(PxBitMapPinned* shapeChangedMap)
{
	for(PxU32 i=0; i<mBodies.size(); i++)
		mBodies[i]->updateCached(shapeChangedMap);
}

void Sc::ArticulationSim::markShapesUpdated(PxBitMapPinned* shapeChangedMap)
{
	for (PxU32 a = 0; a < mBodies.size(); ++a)
	{
		PxU32 nbElems = mBodies[a]->getNbElements();
		ElementSim** elems = mBodies[a]->getElements();
		while (nbElems--)
		{
			ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
			if (sim->isInBroadPhase())
				shapeChangedMap->growAndSet(sim->getElementID());
		}
	}
}

void Sc::ArticulationSim::addBody(BodySim& body, BodySim* parent, ArticulationJointSim* joint)
{
	mBodies.pushBack(&body);
	mJoints.pushBack(joint);
	mLLArticulation->addBody();

	const PxU32 index = mLinks.size();

	PX_ASSERT((((index==0) && (joint == 0)) && (parent == 0)) ||
			  (((index!=0) && joint) && (parent && (parent->getArticulation() == this))));

	ArticulationLink& link = mLinks.insert();

	link.bodyCore	= &body.getBodyCore().getCore();
	link.children	= 0;
	link.mPathToRootStartIndex = 0;
	link.mPathToRootCount = 0;
	link.mChildrenStartIndex = 0xffffffff;
	link.mNumChildren = 0;
	bool shouldSleep;
	bool currentlyAsleep;
	const bool bodyReadyForSleep = body.checkSleepReadinessBesidesWakeCounter();
	const PxReal wakeCounter = getCore().getWakeCounter();

	if(parent)
	{
		currentlyAsleep = !mBodies[0]->isActive();
		shouldSleep = currentlyAsleep && bodyReadyForSleep;

		PxU32 parentIndex = findBodyIndex(*parent);
		link.parent = parentIndex;
		ArticulationLink& parentLink = mLinks[parentIndex];
		link.pathToRoot = parentLink.pathToRoot | ArticulationBitField(1)<<index;
		link.inboundJoint = &joint->getCore().getCore();
		parentLink.children |= ArticulationBitField(1)<<index;
		
		if (parentLink.mChildrenStartIndex == 0xffffffff)
			parentLink.mChildrenStartIndex = index;

		parentLink.mNumChildren++;

	}
	else
	{
		currentlyAsleep = (wakeCounter == 0.0f);
		shouldSleep = currentlyAsleep && bodyReadyForSleep;

		link.parent = DY_ARTICULATION_LINK_NONE;
		link.pathToRoot = 1;
		link.inboundJoint = NULL;
	}
	

	if(currentlyAsleep && !shouldSleep)
	{
		for(PxU32 i=0; i < (mBodies.size() - 1); i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}

	body.setArticulation(this, wakeCounter, shouldSleep, index);
}

void Sc::ArticulationSim::removeBody(BodySim& body)
{
	for (PxU32 i = 0; i < mBodies.size(); ++i)
	{
		if (mBodies[i] == &body)
		{
			mBodies.replaceWithLast(i);
			mJoints.replaceWithLast(i);
			break;
		}
	}
	
}


void Sc::ArticulationSim::addTendon(ArticulationSpatialTendonSim* tendonSim)
{
	tendonSim->mArtiSim = this;

	const PxU32 index = mSpatialTendons.size();
	Dy::ArticulationSpatialTendon& llTendon = tendonSim->mLLTendon;
	llTendon.setTendonIndex(index);
	mSpatialTendons.pushBack(&llTendon);

	//mSpatialTendons.pushBack(&tendonSim->mLLTendon);
}

void Sc::ArticulationSim::addTendon(ArticulationFixedTendonSim* tendonSim)
{
	tendonSim->mArtiSim = this;
	
	const PxU32 index = mFixedTendons.size();
	Dy::ArticulationFixedTendon& llTendon = tendonSim->mLLTendon;
	llTendon.setTendonIndex(index);
	mFixedTendons.pushBack(&llTendon);
}


void Sc::ArticulationSim::addSensor(ArticulationSensorSim* sensorSim, const PxU32 linkID)
{
	const PxU32 index = mSensors.size();
	sensorSim->setLowLevelIndex(index);
	sensorSim->mArticulationSim = this;

	Dy::ArticulationSensor& llSensor = sensorSim->getLLSensor();
	llSensor.mLinkID = PxU16(linkID);
	mSensors.pushBack(&llSensor);
	mSensorForces.insert();
	mSensorForces.back().force = PxVec3(0.f);
	mSensorForces.back().torque = PxVec3(0.f);
}

void Sc::ArticulationSim::createLLStructure()
{
	if(!mBodies.size())
		return;

	mLLArticulation->setupLinks(mLinks.size(), const_cast<Dy::ArticulationLink*>(mLinks.begin()));

	mLLArticulation->assignTendons(mSpatialTendons.size(), const_cast<Dy::ArticulationSpatialTendon**>(mSpatialTendons.begin()));

	mLLArticulation->assignTendons(mFixedTendons.size(), const_cast<Dy::ArticulationFixedTendon**>(mFixedTendons.begin()));
	
	mLLArticulation->assignSensors(mSensors.size(), const_cast<Dy::ArticulationSensor**>(mSensors.begin()), const_cast<PxSpatialForce*>(mSensorForces.begin()));

	mIsLLArticultionInitialized = true;

}

void Sc::ArticulationSim::initializeConfiguration()
{
	Dy::ArticulationData& data = mLLArticulation->getArticulationData();
	mLLArticulation->jcalc(data);

	Dy::ArticulationLink* links = data.getLinks();
	Dy::ArticulationJointCoreData* jointData = data.getJointData();
	const PxU32 linkCount = data.getLinkCount();

	PxReal* jointVelocites = data.getJointVelocities();
	PxReal* jointPositions = data.getJointPositions();
	
	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	{
		Dy::ArticulationLink& link = links[linkID];

		Dy::ArticulationJointCore* joint = link.inboundJoint;
		Dy::ArticulationJointCoreData& jointDatum = jointData[linkID];

		PxReal* jPositions = &jointPositions[jointDatum.jointOffset];
		PxReal* jVelocites = &jointVelocites[jointDatum.jointOffset];
		for (PxU8 i = 0; i < jointDatum.dof; ++i)
		{
			const PxU32 dofId = joint->dofIds[i];
			jPositions[i] = joint->jointPos[dofId];
			jVelocites[i] = joint->jointVel[dofId];
		}
	}

	mLLArticulation->initPathToRoot();
}

void Sc::ArticulationSim::updateKinematic(PxArticulationKinematicFlags flags)
{
	Dy::ArticulationData& data = mLLArticulation->getArticulationData();

	if ((flags & PxArticulationKinematicFlag::ePOSITION))
	{
		mLLArticulation->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS);
		mLLArticulation->teleportLinks(data);
	}

	if ((flags & PxArticulationKinematicFlag::ePOSITION) ||
		(flags & PxArticulationKinematicFlag::eVELOCITY))
	{
		mLLArticulation->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES);
		mLLArticulation->computeLinkVelocities(data);
	}
}

void Sc::ArticulationSim::copyJointStatus(const PxU32 linkID)
{
	Dy::ArticulationData& data = mLLArticulation->getArticulationData();
	Dy::ArticulationLink* links = data.getLinks();
	Dy::ArticulationJointCoreData* jointData = data.getJointData();

	Dy::ArticulationLink& link = links[linkID];
	Dy::ArticulationJointCore* joint = link.inboundJoint;
	Dy::ArticulationJointCoreData& jointDatum = jointData[linkID];

	PxReal* jointVelocites = data.getJointVelocities();
	PxReal* jointPositions = data.getJointPositions();
	
	PxReal* jVelocities = &jointVelocites[jointDatum.jointOffset];
	PxReal* jPositions = &jointPositions[jointDatum.jointOffset];

	for(PxU8 i = 0; i < jointDatum.dof; ++i)
	{
		const PxU32 dofId = joint->dofIds[i];
		joint->jointPos[dofId] = jPositions[i];
		joint->jointVel[dofId] = jVelocities[i];
	}

}

void Sc::ArticulationSim::updateCCDLinks(PxArray<BodySim*>& sims)
{
	
	for (PxU32 a = 0; a < mBodies.size(); ++a)
	{
		if (mBodies[a]->getLowLevelBody().getCore().mFlags & PxRigidBodyFlag::eENABLE_CCD)
		{
			sims.pushBack(mBodies[a]);
		}
	}
}

void Sc::ArticulationSim::putToSleep()
{
	for (PxU32 i = 0; i < mLinks.size(); i++)
	{
		BodySim* bodySim = mBodies[i];
		PxsRigidBody& rigid = bodySim->getLowLevelBody();
		
		PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();

		//rigid.setPose(rigid.getLastCCDTransform());

		//KS - the IG deactivates bodies in parallel with the solver. It appears that under certain circumstances, the solver's integration (which performs
		//sleep checks) could decide that the body is no longer a candidate for sleeping on the same frame that the island gen decides to deactivate the island
		//that the body is contained in. This is a rare occurrence but the behavior we want to emulate is that of IG running before solver so we should therefore
		//permit the IG to make the authoritative decision over whether the body should be active or inactive.
		bodyCore.wakeCounter = 0.0f;
		bodyCore.linearVelocity = PxVec3(0.0f);
		bodyCore.angularVelocity = PxVec3(0.0f);

		rigid.clearAllFrameFlags();

		//Force update
	}
	mScene.getSimulationController()->updateArticulation(mLLArticulation, mIslandNodeIndex);
}

void Sc::ArticulationSim::sleepCheck(PxReal dt)
{
	if(!mBodies.size())
		return;

#if PX_CHECKED
	{
		PxReal maxTimer = 0.0f, minTimer = PX_MAX_F32;
		bool allActive = true, noneActive = true;
		for(PxU32 i=0;i<mLinks.size();i++)
		{
			PxReal timer = mBodies[i]->getBodyCore().getWakeCounter();
			maxTimer = PxMax(maxTimer, timer);
			minTimer = PxMin(minTimer, timer);
			bool active = mBodies[i]->isActive();
			allActive &= active;
			noneActive &= !active;
		}
		// either all links are asleep, or no links are asleep
		PX_ASSERT(maxTimer==0 || minTimer!=0);
		PX_ASSERT(allActive || noneActive);
	}

#endif

	if(!mBodies[0]->isActive())
		return;

	const PxReal sleepThreshold = getCore().getCore().sleepThreshold;

	PxReal maxTimer = 0.0f , minTimer = PX_MAX_F32;

	for(PxU32 i=0;i<mLinks.size();i++)
	{
		const Cm::SpatialVector& motionVelocity = mLLArticulation->getMotionVelocity(i);
		PxReal timer = mBodies[i]->updateWakeCounter(dt, sleepThreshold, motionVelocity);
		maxTimer = PxMax(maxTimer, timer);
		minTimer = PxMin(minTimer, timer);
	}

	mCore.setWakeCounterInternal(maxTimer);

	if(maxTimer != 0.0f)
	{
		if(minTimer == 0.0f)
		{
			// make sure nothing goes to sleep unless everything does
			for(PxU32 i=0;i<mLinks.size();i++)
				mBodies[i]->getBodyCore().setWakeCounterFromSim(PxMax(1e-6f, mBodies[i]->getBodyCore().getWakeCounter()));
		}
		return;
	}

	for(PxU32 i=0;i<mLinks.size();i++)
	{
		mBodies[i]->notifyReadyForSleeping();
		mBodies[i]->resetSleepFilter();
	}

	mScene.getSimpleIslandManager()->deactivateNode(mIslandNodeIndex);
}

bool Sc::ArticulationSim::isSleeping() const
{
	return (mBodies.size() > 0) ? (!mBodies[0]->isActive()) : true;
}

void Sc::ArticulationSim::internalWakeUp(PxReal wakeCounter)
{
	if(mCore.getWakeCounter() < wakeCounter)
	{
		mCore.setWakeCounterInternal(wakeCounter);
		for(PxU32 i=0;i<mBodies.size();i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}
}

void Sc::ArticulationSim::setActive(const bool b, const PxU32 infoFlag)
{
	const PxReal wakeCounter = mCore.getWakeCounter();
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i+1],0);
			PxPrefetchLine(mBodies[i+1],128);
		}
		//KS - force in the wake counter from the articulation to its links. This is required because
		//GPU articulation simulation does not DMA back wake counters for each link - it just brings back a global wake counter
		mBodies[i]->getBodyCore().setWakeCounterFromSim(wakeCounter);
		mBodies[i]->setActive(b, infoFlag);
	}
}

void Sc::ArticulationSim::updateForces(PxReal dt, bool notify)
{
	PxU32 count = 0;

	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i+1],128);
			PxPrefetchLine(mBodies[i+1],256);
		}

		mBodies[i]->updateForces(dt, NULL, NULL, count, &mLLArticulation->getSolverDesc().acceleration[i]);
	}
	if(notify)
		mScene.getSimulationController()->updateArticulationExtAccel(mLLArticulation, mIslandNodeIndex);
}

void Sc::ArticulationSim::clearAcceleration(PxReal dt)
{
	PxU32 count = 0;

	for (PxU32 i = 0; i < mBodies.size(); i++)
	{
		if (i + 1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i + 1], 128);
			PxPrefetchLine(mBodies[i + 1], 256);
		}


		const bool accDirty = mBodies[i]->readVelocityModFlag(VMF_ACC_DIRTY);

		if (!accDirty)
		{
			mLLArticulation->getSolverDesc().acceleration[i].linear = PxVec3(0.f);
			mLLArticulation->getSolverDesc().acceleration[i].angular = PxVec3(0.f);
		}
		else
			mBodies[i]->updateForces(dt, NULL, NULL, count, &mLLArticulation->getSolverDesc().acceleration[i]);

	}

	mScene.getSimulationController()->updateArticulationExtAccel(mLLArticulation, mIslandNodeIndex);
}

void Sc::ArticulationSim::saveLastCCDTransform()
{
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i+1],128);
			PxPrefetchLine(mBodies[i+1],256);
		}
		mBodies[i]->getLowLevelBody().saveLastCCDTransform();
	}
}

void Sc::ArticulationSim::setKinematicLink(const bool value)
{
	const PxU32 linkCount = mLinks.size();

	if (linkCount > 0)
	{
		mLinks[0].bodyCore->kinematicLink = PxU8(value);
	}
}

PxU32 Sc::ArticulationSim::getDofs() const
{
	return mLLArticulation->getDofs();
}

PxU32 Sc::ArticulationSim::getDof(const PxU32 linkID) const
{
	return mLLArticulation->getDof(linkID);
}

PX_COMPILE_TIME_ASSERT(sizeof(Cm::SpatialVector)==sizeof(PxSpatialForce));
PxArticulationCache* Sc::ArticulationSim::createCache()
{
	const PxU32 totalSize = getCacheDataSize() + sizeof(PxArticulationCache);

	const PxU32 totalDofs = mLLArticulation->getDofs();
	const PxU32 linkCount = mLinks.size();
	const PxU32 sensorCount = mSensors.size();

	PxU8* tCache = reinterpret_cast<PxU8*>(PX_ALLOC(totalSize, "Articulation cache"));
	PxMemZero(tCache, totalSize);

	PxArticulationCache* cache = reinterpret_cast<PxArticulationCache*>(tCache);

	PxU32 offset = sizeof(PxArticulationCache);
	cache->externalForces = reinterpret_cast<PxSpatialForce*>(tCache + offset);
	offset += sizeof(PxSpatialForce) * linkCount;
	cache->sensorForces = reinterpret_cast<PxSpatialForce*>(tCache + offset);
	offset += sizeof(PxSpatialForce) * sensorCount;
	
	cache->denseJacobian = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * (6 + totalDofs) * (linkCount * 6);				//size of dense jacobian assuming free floating base link.

	cache->massMatrix = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs * totalDofs;

	cache->jointVelocity = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs;

	cache->jointAcceleration = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs;

	cache->jointPosition = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs;

	cache->jointForce = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs;

	cache->jointSolverForces = reinterpret_cast<PxReal*>(tCache + offset);
	offset += sizeof(PxReal) * totalDofs;

	cache->linkVelocity = reinterpret_cast<PxSpatialVelocity*>(tCache + offset);
	offset += sizeof(PxSpatialVelocity) * linkCount;

	cache->linkAcceleration = reinterpret_cast<PxSpatialVelocity*>(tCache + offset);
	offset += sizeof(PxSpatialVelocity) * linkCount;

	cache->rootLinkData = reinterpret_cast<PxArticulationRootLinkData*>(tCache + offset);

	cache->coefficientMatrix = NULL;
	cache->lambda =NULL;

	const PxU32 scratchMemorySize = getScratchMemorySize();
	void* scratchMemory = PX_ALLOC(scratchMemorySize, "Cache scratch memory");
	cache->scratchMemory = scratchMemory;

	PxcScratchAllocator* sa = PX_NEW(PxcScratchAllocator);
	sa->setBlock(scratchMemory, scratchMemorySize);
	cache->scratchAllocator = sa;

	return cache;
}

PxU32 Sc::ArticulationSim::getCacheDataSize() const
{
	const PxU32 totalDofs = mLLArticulation->getDofs();
	const PxU32 linkCount = mLinks.size();
	const PxU32 sensorCount = mSensors.size();
	const PxU32 totalSize =
		sizeof(PxSpatialForce) * linkCount							//external force
		+ sizeof(PxSpatialForce) * sensorCount						//sensors
		+ sizeof(PxReal) * (6 + totalDofs) * (linkCount * 6)		//Free floating base dofs = 6 + totalDofs, and each link (incl. base) velocity has 6 elements
		+ sizeof(PxReal) * totalDofs * totalDofs					//mass matrix
		+ sizeof(PxReal) * totalDofs * 5							//jointVelocity, jointAcceleration, jointPosition, joint force, joint constraint force
		+ sizeof(PxSpatialVelocity) * linkCount * 2					//link velocity, link acceleration
		+ sizeof(PxArticulationRootLinkData);						//root link data
	
	return totalSize;
}

PxU32 Sc::ArticulationSim::getScratchMemorySize() const
{
	const PxU32 totalDofs = mLLArticulation->getDofs();
	const PxU32 linkCount = mLinks.size();

	PxU32 totalSize =
		sizeof(Cm::SpatialVectorF) * linkCount * 5				//motionVelocity, motionAccelerations, coriolisVectors, spatialZAVectors, externalAccels;
		+ sizeof(Dy::SpatialMatrix) * linkCount					//compositeSpatialInertias;
		+ sizeof(PxReal) * totalDofs * 5;						//jointVelocity, jointAcceleration, jointForces, jointPositions, jointFrictionForces

	totalSize = (totalSize+15)&~15;

	return totalSize;
}

void Sc::ArticulationSim::zeroCache(PxArticulationCache& cache) const
{
	const PxU32 cacheDataSize = getCacheDataSize();

	PxMemZero(cache.externalForces, cacheDataSize);
}

//copy external data to internal data
bool  Sc::ArticulationSim::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	//checkResize();
	bool shouldWake = false;
	if (mLLArticulation->applyCache(cache, flag, shouldWake))
	{
		mScene.getSimulationController()->updateArticulation(mLLArticulation, mIslandNodeIndex);
	}
	return shouldWake;
}

//copy internal data to external data
void Sc::ArticulationSim::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	mLLArticulation->copyInternalStateToCache(cache, flag);
}


void Sc::ArticulationSim::packJointData(const PxReal* maximum, PxReal* reduced) const
{
	mLLArticulation->packJointData(maximum, reduced);
}

void Sc::ArticulationSim::unpackJointData(const PxReal* reduced, PxReal* maximum) const
{
	mLLArticulation->unpackJointData(reduced, maximum);
}

void Sc::ArticulationSim::commonInit()
{
	mLLArticulation->initializeCommonData();
}

void Sc::ArticulationSim::computeGeneralizedGravityForce(PxArticulationCache& cache)
{
	mLLArticulation->getGeneralizedGravityForce(mScene.getGravityFast(), cache);
}

void Sc::ArticulationSim::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache)
{
	mLLArticulation->getCoriolisAndCentrifugalForce(cache);
}

void Sc::ArticulationSim::computeGeneralizedExternalForce(PxArticulationCache& cache)
{
	mLLArticulation->getGeneralizedExternalForce(cache);
}

void Sc::ArticulationSim::computeJointAcceleration(PxArticulationCache& cache)
{
	mLLArticulation->getJointAcceleration(mScene.getGravityFast(), cache);
}

void Sc::ArticulationSim::computeJointForce(PxArticulationCache& cache)
{
	mLLArticulation->getJointForce(cache);
}

void Sc::ArticulationSim::computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols)
{
	mLLArticulation->getDenseJacobian(cache, nRows, nCols);
}

void Sc::ArticulationSim::computeCoefficientMatrix(PxArticulationCache& cache)
{
	mLLArticulation->getCoefficientMatrixWithLoopJoints(mLoopConstraints.begin(), mLoopConstraints.size(), cache);
}

bool Sc::ArticulationSim::computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState,
	const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter)
{
	const PxReal invLengthScale = 1.f / mScene.getLengthScale();
	return mLLArticulation->getLambda(mLoopConstraints.begin(), mLoopConstraints.size(), cache, initialState, jointTorque, gravity, maxIter, invLengthScale);
}

void Sc::ArticulationSim::computeGeneralizedMassMatrix(PxArticulationCache& cache)
{
	mLLArticulation->getGeneralizedMassMatrixCRB(cache);

	/*const PxU32 totalDofs = mLLArticulation->getDofs();

	PxReal* massMatrix = reinterpret_cast<PxReal*>(PX_ALLOC(sizeof(PxReal) * totalDofs * totalDofs, "MassMatrix"));
	PxMemCopy(massMatrix, cache.massMatrix, sizeof(PxReal)*totalDofs * totalDofs);

	mLLArticulation->getGeneralizedMassMatrix(cache);

	PxReal* massMatrix1 = cache.massMatrix;
	for (PxU32 i = 0; i < totalDofs; ++i)
	{
		PxReal* row = &massMatrix1[i * totalDofs];

		for (PxU32 j = 0; j < totalDofs; ++j)
		{
			const PxReal dif = row[j] - massMatrix[j*totalDofs + i];
			PX_ASSERT (PxAbs(dif) < 2e-4f)
		}
	}

	PX_FREE(massMatrix);*/
}

PxU32 Sc::ArticulationSim::getCoefficientMatrixSize() const
{
	const PxU32 size = mLoopConstraints.size();
	const PxU32 totalDofs = mLLArticulation->getDofs();
	return size * totalDofs;
}

void Sc::ArticulationSim::setRootLinearVelocity(const PxVec3& velocity)
{
	mLLArticulation->setRootLinearVelocity(velocity);
}

void Sc::ArticulationSim::setRootAngularVelocity(const PxVec3& velocity)
{
	mLLArticulation->setRootAngularVelocity(velocity);
}

PxSpatialVelocity Sc::ArticulationSim::getLinkVelocity(const PxU32 linkId) const
{
	Cm::SpatialVector vel = mLLArticulation->getLinkScalarVelocity(linkId);
	return reinterpret_cast<PxSpatialVelocity&>(vel);
}

PxSpatialVelocity Sc::ArticulationSim::getLinkAcceleration(const PxU32 linkId) const
{
	Cm::SpatialVector accel = mLLArticulation->getMotionAcceleration(linkId);
	return reinterpret_cast<PxSpatialVelocity&>(accel);
}

// This method allows user teleport the root links and the articulation
//system update all other links pose
void Sc::ArticulationSim::setGlobalPose()
{
	mLLArticulation->teleportRootLink();
}

void Sc::ArticulationSim::setJointDirty(Dy::ArticulationJointCore& jointCore)
{
	PX_UNUSED(jointCore);
	mScene.getSimulationController()->updateArticulationJoint(mLLArticulation, mIslandNodeIndex);
}

void Sc::ArticulationSim::setArticulationDirty(PxU32 flag)
{
	Dy::FeatherstoneArticulation* featherstoneArtic = static_cast<Dy::FeatherstoneArticulation*>(mLLArticulation);
	featherstoneArtic->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::Enum(flag));
	mScene.getSimulationController()->updateArticulation(mLLArticulation, mIslandNodeIndex);
}

void Sc::ArticulationSim::debugCheckWakeCounterOfLinks(PxReal wakeCounter) const
{
	PX_UNUSED(wakeCounter);

#ifdef _DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == wakeCounter);
	}
#endif
}

void Sc::ArticulationSim::debugCheckSleepStateOfLinks(bool isSleeping) const
{
	PX_UNUSED(isSleeping);

#ifdef _DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		if (isSleeping)
		{
			PX_ASSERT(!mBodies[i]->isActive());
			PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == 0.0f);
			PX_ASSERT(mBodies[i]->checkSleepReadinessBesidesWakeCounter());
		}
		else
			PX_ASSERT(mBodies[i]->isActive());
	}
#endif
}

PxU32 Sc::ArticulationSim::getRootActorIndex() const
{
	return mBodies[0]->getActorID();
}

const PxSpatialForce& Sc::ArticulationSim::getSensorForce(const PxU32 lowLevelIndex) const
{
	return mSensorForces[lowLevelIndex];
}
