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

#include "ScBodySim.h"
#include "ScStaticSim.h"
#include "ScConstraintCore.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"
#include "ScElementSimInteraction.h"

using namespace physx;
using namespace Sc;

static ConstraintInteraction* createInteraction(ConstraintSim* sim, RigidCore* r0, RigidCore* r1, Scene& scene)
{
	return scene.getConstraintInteractionPool()->construct(	sim, 
															r0 ? *r0->getSim() : scene.getStaticAnchor(), 
															r1 ? *r1->getSim() : scene.getStaticAnchor());
}

static void releaseInteraction(ConstraintInteraction* interaction, const ConstraintSim* sim, Scene& scene)
{
	if(!sim->isBroken())
		interaction->destroy();

	scene.getConstraintInteractionPool()->destroy(interaction);
}

Sc::ConstraintSim::ConstraintSim(ConstraintCore& core, RigidCore* r0, RigidCore* r1, Scene& scene) :
	mScene		(scene),
	mCore		(core),
	mInteraction(NULL),
	mFlags		(0)
{
	mBodies[0] = (r0 && (r0->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r0->getSim()) : 0;
	mBodies[1] = (r1 && (r1->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r1->getSim()) : 0;
	
	const PxU32 id = scene.getConstraintIDTracker().createID();

	mLowLevelConstraint.index = id;
	PxPinnedArray<Dy::ConstraintWriteback>& writeBackPool = scene.getDynamicsContext()->getConstraintWriteBackPool();
	if(id >= writeBackPool.capacity())
		writeBackPool.reserve(writeBackPool.capacity() * 2);

	writeBackPool.resize(PxMax(writeBackPool.size(), id + 1));
	writeBackPool[id].initialize();

	if(!createLLConstraint())
		return;

	PxReal linBreakForce, angBreakForce;
	core.getBreakForce(linBreakForce, angBreakForce);
	if ((linBreakForce < PX_MAX_F32) || (angBreakForce < PX_MAX_F32))
		setFlag(eBREAKABLE);

	core.setSim(this);

	mInteraction = createInteraction(this, r0, r1, scene);

	PX_ASSERT(!mInteraction->isRegistered());  // constraint interactions must not register in the scene, there is a list of Sc::ConstraintSim instead
}

Sc::ConstraintSim::~ConstraintSim()
{
	PX_ASSERT(mInteraction);  // This is fine now, a body which gets removed from the scene removes all constraints automatically
	PX_ASSERT(!mInteraction->isRegistered());  // constraint interactions must not register in the scene, there is a list of Sc::ConstraintSim instead

	releaseInteraction(mInteraction, this, mScene);

	mScene.getConstraintIDTracker().releaseID(mLowLevelConstraint.index);
	destroyLLConstraint();

	mCore.setSim(NULL);
}

static PX_FORCE_INLINE void setLLBodies(Dy::Constraint& c, BodySim* b0, BodySim* b1)
{
	PxsRigidBody* body0 = b0 ? &b0->getLowLevelBody() : NULL;
	PxsRigidBody* body1 = b1 ? &b1->getLowLevelBody() : NULL;

	c.body0 = body0;
	c.body1 = body1;

	c.bodyCore0 = body0 ? &body0->getCore() : NULL;
	c.bodyCore1 = body1 ? &body1->getCore() : NULL;
}

bool Sc::ConstraintSim::createLLConstraint()
{
	ConstraintCore& core = getCore();
	const PxU32 constantBlockSize = core.getConstantBlockSize();

	void* constantBlock = mScene.allocateConstraintBlock(constantBlockSize);
	if(!constantBlock)
		return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Constraint: could not allocate low-level resources.");

	//Ensure the constant block isn't just random data because some functions may attempt to use it before it is
	//setup.  Specifically pvd visualization of joints
	//-CN

	PxMemZero(constantBlock, constantBlockSize);

	Dy::Constraint& llc = mLowLevelConstraint;
	core.getBreakForce(llc.linBreakForce, llc.angBreakForce);
	llc.flags					= core.getFlags();
	llc.constantBlockSize		= PxU16(constantBlockSize);

	llc.solverPrep				= core.getSolverPrep();
	llc.constantBlock			= constantBlock;
	llc.minResponseThreshold	= core.getMinResponseThreshold();

	//llc.index = mLowLevelConstraint.index;
	setLLBodies(llc, mBodies[0], mBodies[1]);

	return true;
}

void Sc::ConstraintSim::destroyLLConstraint()
{
	if(mLowLevelConstraint.constantBlock)
		mScene.deallocateConstraintBlock(mLowLevelConstraint.constantBlock, mLowLevelConstraint.constantBlockSize);
}

void Sc::ConstraintSim::setBodies(RigidCore* r0, RigidCore* r1)
{
	PX_ASSERT(mInteraction);

	releaseInteraction(mInteraction, this, mScene);

	BodySim* b0 = (r0 && (r0->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r0->getSim()) : 0;
	BodySim* b1 = (r1 && (r1->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r1->getSim()) : 0;

	setLLBodies(mLowLevelConstraint, b0, b1);

	mBodies[0] = b0;
	mBodies[1] = b1;

	mInteraction = createInteraction(this, r0, r1, mScene);
}

void Sc::ConstraintSim::getForce(PxVec3& lin, PxVec3& ang)
{
	const PxReal recipDt = mScene.getOneOverDt();
	Dy::ConstraintWriteback& solverOutput= mScene.getDynamicsContext()->getConstraintWriteBackPool()[mLowLevelConstraint.index];
	lin = solverOutput.linearImpulse * recipDt;
	ang = solverOutput.angularImpulse * recipDt;
}

void Sc::ConstraintSim::setBreakForceLL(PxReal linear, PxReal angular)
{
	PxU8 wasBreakable = readFlag(eBREAKABLE);
	PxU8 isBreakable;
	if ((linear < PX_MAX_F32) || (angular < PX_MAX_F32))
		isBreakable = eBREAKABLE;
	else
		isBreakable = 0;

	if (isBreakable != wasBreakable)
	{
		if (isBreakable)
		{
			PX_ASSERT(!readFlag(eCHECK_MAX_FORCE_EXCEEDED));
			setFlag(eBREAKABLE);
			if (mInteraction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
				mScene.addActiveBreakableConstraint(this, mInteraction);
		}
		else
		{
			if (readFlag(eCHECK_MAX_FORCE_EXCEEDED))
				mScene.removeActiveBreakableConstraint(this);
			clearFlag(eBREAKABLE);
		}
	}

	mLowLevelConstraint.linBreakForce = linear;
	mLowLevelConstraint.angBreakForce = angular;
}

void Sc::ConstraintSim::postFlagChange(PxConstraintFlags /*oldFlags*/, PxConstraintFlags newFlags)
{
	mLowLevelConstraint.flags = newFlags;
}
