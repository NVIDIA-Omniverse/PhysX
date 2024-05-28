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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScSoftBodySim.h"
#include "ScSoftBodyCore.h"
#include "ScScene.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace physx::Dy; 

Sc::SoftBodySim::SoftBodySim(SoftBodyCore& core, Scene& scene) :
	ActorSim(scene, core),
	mShapeSim(*this)
{
	mLLSoftBody = scene.createLLSoftBody(this);

	mNodeIndex = scene.getSimpleIslandManager()->addSoftBody(mLLSoftBody, false);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	mLLSoftBody->setElementId(mShapeSim.getElementID());
}

Sc::SoftBodySim::~SoftBodySim()
{
	if (!mLLSoftBody)
		return;

	mScene.destroyLLSoftBody(*mLLSoftBody);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

void Sc::SoftBodySim::updateBounds()
{
	mShapeSim.updateBounds();
}

void Sc::SoftBodySim::updateBoundsInAABBMgr()
{
	mShapeSim.updateBoundsInAABBMgr();
}

PxBounds3 Sc::SoftBodySim::getBounds() const
{
	return mShapeSim.getBounds();
}

bool Sc::SoftBodySim::isSleeping() const
{
	IG::IslandSim& sim = mScene.getSimpleIslandManager()->getAccurateIslandSim();
	return sim.getActiveNodeIndex(mNodeIndex) == PX_INVALID_NODE;
}

void Sc::SoftBodySim::onSetWakeCounter()
{
	getScene().getSimulationController()->setSoftBodyWakeCounter(mLLSoftBody);
	if (mLLSoftBody->getCore().wakeCounter > 0.f)
		getScene().getSimpleIslandManager()->activateNode(mNodeIndex);
	else
		getScene().getSimpleIslandManager()->deactivateNode(mNodeIndex);
}

void Sc::SoftBodySim::attachShapeCore(ShapeCore* core)
{
	mShapeSim.attachShapeCore(core);

	PxsShapeCore* shapeCore = const_cast<PxsShapeCore*>(&core->getCore());
	mLLSoftBody->setShapeCore(shapeCore);
}

void Sc::SoftBodySim::attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxSoftBodyAuxData* simulationState)
{
	mLLSoftBody->setSimShapeCore(simulationMesh, simulationState);
}

PxTetrahedronMesh* Sc::SoftBodySim::getSimulationMesh()
{
	return mLLSoftBody->getSimulationMesh();
}

PxSoftBodyAuxData* Sc::SoftBodySim::getSoftBodyAuxData()
{
	return mLLSoftBody->getSoftBodyAuxData();
}

PxTetrahedronMesh* Sc::SoftBodySim::getCollisionMesh()
{
	return mLLSoftBody->getCollisionMesh();
}

void Sc::SoftBodySim::enableSelfCollision()
{
	if (isActive())
	{
		getScene().getSimulationController()->activateSoftbodySelfCollision(mLLSoftBody);
	}
}

void Sc::SoftBodySim::disableSelfCollision()
{
	if (isActive())
	{
		getScene().getSimulationController()->deactivateSoftbodySelfCollision(mLLSoftBody);
	}
}

/*void Sc::SoftBodySim::activate()
{
	// Activate body
	//{
	//	PX_ASSERT((!isKinematic()) || notInScene() || readInternalFlag(InternalFlags(BF_KINEMATIC_MOVED | BF_KINEMATIC_SURFACE_VELOCITY)));	// kinematics should only get activated when a target is set.
	//																																		// exception: object gets newly added, then the state change will happen later
	//	if (!isArticulationLink())
	//	{
	//		mLLBody.mInternalFlags &= (~PxsRigidBody::eFROZEN);
	//		// Put in list of activated bodies. The list gets cleared at the end of a sim step after the sleep callbacks have been fired.
	//		getScene().onBodyWakeUp(this);
	//	}

	//	BodyCore& core = getBodyCore();
	//	if (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
	//	{
	//		PX_ASSERT(!getScene().isInPosePreviewList(*this));
	//		getScene().addToPosePreviewList(*this);
	//	}
	//	createSqBounds();
	//}

	activateInteractions(*this);
}

void Sc::SoftBodySim::deactivate()
{
	deactivateInteractions(*this);

	// Deactivate body
	//{
	//	PX_ASSERT((!isKinematic()) || notInScene() || !readInternalFlag(BF_KINEMATIC_MOVED));	// kinematics should only get deactivated when no target is set.
	//																							// exception: object gets newly added, then the state change will happen later
	//	BodyCore& core = getBodyCore();
	//	if (!readInternalFlag(BF_ON_DEATHROW))
	//	{
	//		// Set velocity to 0.
	//		// Note: this is also fine if the method gets called because the user puts something to sleep (this behavior is documented in the API)
	//		PX_ASSERT(core.getWakeCounter() == 0.0f);
	//		const PxVec3 zero(0.0f);
	//		core.setLinearVelocityInternal(zero);
	//		core.setAngularVelocityInternal(zero);

	//		setForcesToDefaults(!(mLLBody.mInternalFlags & PxsRigidBody::eDISABLE_GRAVITY));
	//	}

	//	if (!isArticulationLink())  // Articulations have their own sleep logic.
	//		getScene().onBodySleep(this);

	//	if (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
	//	{
	//		PX_ASSERT(getScene().isInPosePreviewList(*this));
	//		getScene().removeFromPosePreviewList(*this);
	//	}
	//	destroySqBounds();
	//}
}*/

PxU32 Sc::SoftBodySim::getGpuSoftBodyIndex() const
{
	return mLLSoftBody->getGpuSoftBodyIndex();
}

#endif //PX_SUPPORT_GPU_PHYSX
