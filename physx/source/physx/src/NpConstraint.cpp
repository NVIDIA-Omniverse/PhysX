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

#include "PxConstraint.h"
#include "NpConstraint.h"
#include "NpPhysics.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxConstraintFlags scGetFlags(const ConstraintCore& core)
{
	return core.getFlags() & (~(PxConstraintFlag::eGPU_COMPATIBLE));
}

static NpScene* getSceneFromActors(const PxRigidActor* actor0, const PxRigidActor* actor1)
{
	NpScene* s0 = NULL;
	NpScene* s1 = NULL;

	if(actor0 && (!(actor0->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))))
		s0 = static_cast<NpScene*>(actor0->getScene());
	if(actor1 && (!(actor1->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))))
		s1 = static_cast<NpScene*>(actor1->getScene());

#if PX_CHECKED
	if ((s0 && s1) && (s0 != s1))
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Adding constraint to scene: Actors belong to different scenes, undefined behavior expected!");
#endif

	if ((!actor0 || s0) && (!actor1 || s1))
		return s0 ? s0 : s1;
	else
		return NULL;
}

void NpConstraint::setConstraintFunctions(PxConstraintConnector& n, const PxConstraintShaderTable& shaders)
{
	mCore.setConstraintFunctions(n, shaders);
	
	//update mConnectorArray, since mActor0 or mActor1 should be in external reference
	bool bNeedUpdate = false;
	if(mActor0)
	{
		NpActor& npActor = NpActor::getFromPxActor(*mActor0);
		if(npActor.findConnector(NpConnectorType::eConstraint, this) == 0xffffffff)
		{
			bNeedUpdate = true;	
			npActor.addConnector(NpConnectorType::eConstraint, this, "PxConstraint: Add to rigid actor 0: Constraint already added");
		}
	}

	if(mActor1)
	{
		NpActor& npActor = NpActor::getFromPxActor(*mActor1);
		if(npActor.findConnector(NpConnectorType::eConstraint, this) == 0xffffffff)
		{
			bNeedUpdate = true;	
			npActor.addConnector(NpConnectorType::eConstraint, this, "PxConstraint: Add to rigid actor 1: Constraint already added");
		}
	}

	if(bNeedUpdate)
	{
		NpScene* newScene = ::getSceneFromActors(mActor0, mActor1);
		NpScene* oldScene = getNpScene();

		if (oldScene != newScene)
		{
			if(oldScene)
				oldScene->removeFromConstraintList(*this);

			if(newScene)
				newScene->addToConstraintList(*this);
		}
	}
}

void NpConstraint::addConnectors(PxRigidActor* actor0, PxRigidActor* actor1)
{
	if(actor0)
		NpActor::getFromPxActor(*actor0).addConnector(NpConnectorType::eConstraint, this, "PxConstraint: Add to rigid actor 0: Constraint already added");
	if(actor1)
		NpActor::getFromPxActor(*actor1).addConnector(NpConnectorType::eConstraint, this, "PxConstraint: Add to rigid actor 1: Constraint already added");
}

void NpConstraint::removeConnectors(const char* errorMsg0, const char* errorMsg1)
{
	if(mActor0)
		NpActor::getFromPxActor(*mActor0).removeConnector(*mActor0, NpConnectorType::eConstraint, this, errorMsg0);
	if(mActor1)
		NpActor::getFromPxActor(*mActor1).removeConnector(*mActor1, NpConnectorType::eConstraint, this, errorMsg1);
}

NpConstraint::NpConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize) :
	PxConstraint(PxConcreteType::eCONSTRAINT, PxBaseFlag::eOWNS_MEMORY),
	NpBase		(NpType::eCONSTRAINT),
	mActor0		(actor0),
	mActor1		(actor1),
	mCore		(connector, shaders, dataSize)
{
	scSetFlags(shaders.flag);

	addConnectors(actor0, actor1);

	NpScene* s = ::getSceneFromActors(actor0, actor1);
	if (s)
	{
		if(s->isAPIWriteForbidden())
		{
			outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxConstraint creation not allowed while simulation is running. Call will be ignored.");
			return;
		}

		s->addToConstraintList(*this);
	}
}

NpConstraint::~NpConstraint()
{
	if(getBaseFlags()&PxBaseFlag::eOWNS_MEMORY)
		mCore.getPxConnector()->onConstraintRelease();

	NpFactory::getInstance().onConstraintRelease(this);
}

static const char* gRemoveConnectorMsg = "PxConstraint::release(): internal error, mConnectorArray not created.";

void NpConstraint::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::release() not allowed while simulation is running. Call will be ignored.")

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, NULL);

	removeConnectors(gRemoveConnectorMsg, gRemoveConnectorMsg);

	if(npScene)
		npScene->removeFromConstraintList(*this);

	NpDestroyConstraint(this);
}

// PX_SERIALIZATION
void NpConstraint::resolveReferences(PxDeserializationContext& context)
{	
	context.translatePxBase(mActor0);
	context.translatePxBase(mActor1);
}

NpConstraint* NpConstraint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpConstraint* obj = PX_PLACEMENT_NEW(address, NpConstraint(PxBaseFlags(0)));
	address += sizeof(NpConstraint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
// ~PX_SERIALIZATION

PxScene* NpConstraint::getScene() const
{
	return getNpScene();
}

void NpConstraint::getActors(PxRigidActor*& actor0, PxRigidActor*& actor1) const
{
	NP_READ_CHECK(getNpScene());
	actor0 = mActor0;
	actor1 = mActor1;
}

static PX_INLINE void scSetBodies(ConstraintCore& core, NpActor* r0, NpActor* r1)
{
	Sc::RigidCore* scR0 = r0 ? &r0->getScRigidCore() : NULL;
	Sc::RigidCore* scR1 = r1 ? &r1->getScRigidCore() : NULL;
	core.setBodies(scR0, scR1);
}

void NpConstraint::setActors(PxRigidActor* actor0, PxRigidActor* actor1)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN((actor0 && actor0->getConcreteType()!=PxConcreteType::eRIGID_STATIC) || (actor1 && actor1->getConcreteType()!=PxConcreteType::eRIGID_STATIC), "PxConstraint: at least one actor must be non-static");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::setActors() not allowed while simulation is running. Call will be ignored.")

	if(mActor0 == actor0 && mActor1 == actor1)
		return;

	removeConnectors(	"PxConstraint: Add to rigid actor 0: Constraint already added",
						"PxConstraint: Add to rigid actor 1: Constraint already added");

	addConnectors(actor0, actor1);

	mActor0 = actor0;
	mActor1 = actor1;

	NpScene* newScene = ::getSceneFromActors(actor0, actor1);
	NpScene* oldScene = getNpScene();

	// PT: bypassing the calls to removeFromConstraintList / addToConstraintList creates issues like PX-2363, where
	// various internal structures are not properly updated. Always going through the slower codepath fixes them.
//	if(oldScene != newScene)
	{
		if(oldScene)
			oldScene->removeFromConstraintList(*this);

		scSetBodies(mCore, NpActor::getNpActor(actor0), NpActor::getNpActor(actor1));

		if(newScene)
			newScene->addToConstraintList(*this);
	}
//	else
//		scSetBodies(mCore, NpActor::getNpActor(actor0), NpActor::getNpActor(actor1));

	UPDATE_PVD_PROPERTY
}

PxConstraintFlags NpConstraint::getFlags() const
{
	NP_READ_CHECK(getNpScene());

	return scGetFlags(mCore);
}

void NpConstraint::setFlags(PxConstraintFlags flags)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(!(flags & PxConstraintFlag::eBROKEN), "PxConstraintFlag::eBROKEN is a read only flag");
	PX_CHECK_AND_RETURN(!(flags & PxConstraintFlag::eGPU_COMPATIBLE), "PxConstraintFlag::eGPU_COMPATIBLE is an internal flag and is illegal to set via the API");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::setFlags() not allowed while simulation is running. Call will be ignored.")

	scSetFlags(flags);
}

void NpConstraint::setFlag(PxConstraintFlag::Enum flag, bool value)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(flag != PxConstraintFlag::eBROKEN, "PxConstraintFlag::eBROKEN is a read only flag");
	PX_CHECK_AND_RETURN(flag != PxConstraintFlag::eGPU_COMPATIBLE, "PxConstraintFlag::eGPU_COMPATIBLE is an internal flag and is illegal to set via the API");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::setFlag() not allowed while simulation is running. Call will be ignored.")

	const PxConstraintFlags f = scGetFlags(mCore);
	scSetFlags(value ? f|flag : f&~flag);
}

void NpConstraint::getForce(PxVec3& linear, PxVec3& angular) const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE(getNpScene(), "PxConstraint::getForce() not allowed while simulation is running (except during PxScene::collide()).");

	mCore.getForce(linear, angular);
}

void NpConstraint::markDirty()
{
#ifdef NEW_DIRTY_SHADERS_CODE
	if(mCore.getFlags() & PxConstraintFlag::eALWAYS_UPDATE)
		return;

	if(!mCore.isDirty())
	{
		NpScene* npScene = getNpScene();
		if(npScene)
			npScene->addDirtyConstraint(this);
		mCore.setDirty();
	}
#else
	mCore.setDirty();
#endif
}

void NpConstraint::updateConstants(PxsSimulationController& simController)
{
	if(!mCore.isDirty() && !(mCore.getFlags() & PxConstraintFlag::eALWAYS_UPDATE))
		return;

	PX_ASSERT(!isAPIWriteForbidden());

	Sc::ConstraintSim* sim = mCore.getSim();
	if(sim)
	{
		Dy::Constraint& LLC = sim->getLowLevelConstraint();
		PxMemCopy(LLC.constantBlock, mCore.getPxConnector()->prepareData(), LLC.constantBlockSize);
		simController.updateJoint(sim->getInteraction()->getEdgeIndex(), &LLC);
	}

	mCore.clearDirty();

#if PX_SUPPORT_PVD
	NpScene* npScene = getNpScene();
	//Changed to use the visual scenes update system which respects
	//the debugger's connection type flag.
	if(npScene)
		npScene->getScenePvdClientInternal().updatePvdProperties(this);
#endif
}

void NpConstraint::setBreakForce(PxReal linear, PxReal angular)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::setBreakForce() not allowed while simulation is running. Call will be ignored.")

	mCore.setBreakForce(linear, angular);
	markDirty();
	UPDATE_PVD_PROPERTY
}

void NpConstraint::getBreakForce(PxReal& linear, PxReal& angular) const
{
	NP_READ_CHECK(getNpScene());
	mCore.getBreakForce(linear, angular);
}

void NpConstraint::setMinResponseThreshold(PxReal threshold)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(threshold) && threshold>=0, "PxConstraint::setMinResponseThreshold: threshold must be non-negative");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxConstraint::setMinResponseThreshold() not allowed while simulation is running. Call will be ignored.")

	mCore.setMinResponseThreshold(threshold);
	UPDATE_PVD_PROPERTY
}

PxReal NpConstraint::getMinResponseThreshold() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getMinResponseThreshold();
}

bool NpConstraint::isValid() const
{
	NP_READ_CHECK(getNpScene());
	const bool isValid0 = mActor0 && mActor0->getConcreteType()!=PxConcreteType::eRIGID_STATIC;
	const bool isValid1 = mActor1 && mActor1->getConcreteType()!=PxConcreteType::eRIGID_STATIC;
	return isValid0 || isValid1;
}

void* NpConstraint::getExternalReference(PxU32& typeID)
{
	NP_READ_CHECK(getNpScene());
	return mCore.getPxConnector()->getExternalReference(typeID);
}

void NpConstraint::comShift(PxRigidActor* actor)
{
	PX_ASSERT(actor == mActor0 || actor == mActor1);
	PxConstraintConnector* connector = mCore.getPxConnector();
	if(actor == mActor0)
		connector->onComShift(0);
	if(actor == mActor1)
		connector->onComShift(1);
}

void NpConstraint::actorDeleted(PxRigidActor* actor)
{
	// the actor cannot be deleted without also removing it from the scene,
	// which means that the joint will also have been removed from the scene,
	// so we can just reset the actor here.
	PX_ASSERT(actor == mActor0 || actor == mActor1);

	if(actor == mActor0)
		mActor0 = NULL;
	else
		mActor1 = NULL;
}

NpScene* NpConstraint::getSceneFromActors() const
{
	return ::getSceneFromActors(mActor0, mActor1);
}
