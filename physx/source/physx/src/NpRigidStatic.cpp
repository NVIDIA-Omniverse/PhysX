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

#include "NpRigidStatic.h"
#include "NpRigidActorTemplateInternal.h"
#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;

NpRigidStatic::NpRigidStatic(const PxTransform& pose) :
	NpRigidStaticT	(PxConcreteType::eRIGID_STATIC, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eRIGID_STATIC),
	mCore			(pose)
{
}

NpRigidStatic::~NpRigidStatic()
{
}

// PX_SERIALIZATION
void NpRigidStatic::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpRigidStaticT::requiresObjects(c);	
}

NpRigidStatic* NpRigidStatic::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpRigidStatic* obj = PX_PLACEMENT_NEW(address, NpRigidStatic(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpRigidStatic);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpRigidStatic::release()
{
	if(releaseRigidActorT<PxRigidStatic>(*this))
	{
		PX_ASSERT(!isAPIWriteForbidden());  // the code above should return false in that case
		NpDestroyRigidActor(this);
	}
}

void NpRigidStatic::setGlobalPose(const PxTransform& pose, bool /*wake*/)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidStatic::setGlobalPose: pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidStatic::setGlobalPose() not allowed while simulation is running. Call will be ignored.")

#if PX_CHECKED
	if(npScene)
		npScene->checkPositionSanity(*this, pose, "PxRigidStatic::setGlobalPose");
#endif

	const PxTransform newPose = pose.getNormalized();	//AM: added to fix 1461 where users read and write orientations for no reason.

	mCore.setActor2World(newPose);
	UPDATE_PVD_PROPERTY
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, translation, *static_cast<PxRigidActor*>(this), newPose.p);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, rotation, *static_cast<PxRigidActor*>(this), newPose.q);
	OMNI_PVD_WRITE_SCOPE_END

	if(npScene)
		mShapeManager.markActorForSQUpdate(npScene->getSQAPI(), *this);

	// invalidate the pruning structure if the actor bounds changed
	if(mShapeManager.getPruningStructure())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxRigidStatic::setGlobalPose: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	updateShaderComs();
}

PxTransform NpRigidStatic::getGlobalPose() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getActor2World();
}

PxU32 physx::NpRigidStaticGetShapes(NpRigidStatic& rigid, NpShape* const *&shapes)
{
	NpShapeManager& sm = rigid.getShapeManager();
	shapes = sm.getShapes();
	return sm.getNbShapes();
}

void NpRigidStatic::switchToNoSim()
{
	NpActor::scSwitchToNoSim();
}

void NpRigidStatic::switchFromNoSim()
{
	NpActor::scSwitchFromNoSim();
}

#if PX_CHECKED
bool NpRigidStatic::checkConstraintValidity() const
{
	// Perhaps NpConnectorConstIterator would be worth it...
	NpConnectorIterator iter = (const_cast<NpRigidStatic*>(this))->getConnectorIterator(NpConnectorType::eConstraint); 
	while (PxBase* ser = iter.getNext())
	{
		NpConstraint* c = static_cast<NpConstraint*>(ser);
		if(!c->NpConstraint::isValid())
			return false;
	}
	return true;
}
#endif


