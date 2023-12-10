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

#include "NpArticulationReducedCoordinate.h"
#include "NpRigidActorTemplateInternal.h"

#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;
using namespace Cm;

// PX_SERIALIZATION
void NpArticulationLink::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpArticulationLinkT::requiresObjects(c);
	
	if(mInboundJoint)
		c.process(*mInboundJoint);
}

void NpArticulationLink::exportExtraData(PxSerializationContext& stream)
{
	NpArticulationLinkT::exportExtraData(stream);
	exportInlineArray(mChildLinks, stream);
}

void NpArticulationLink::importExtraData(PxDeserializationContext& context)
{
	NpArticulationLinkT::importExtraData(context);
	importInlineArray(mChildLinks, context);
}

void NpArticulationLink::resolveReferences(PxDeserializationContext& context)
{	
    context.translatePxBase(mRoot);
    context.translatePxBase(mInboundJoint);
    context.translatePxBase(mParent);
       
    NpArticulationLinkT::resolveReferences(context);

    const PxU32 nbLinks = mChildLinks.size();
    for(PxU32 i=0;i<nbLinks;i++)
        context.translatePxBase(mChildLinks[i]);
}

NpArticulationLink* NpArticulationLink::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationLink* obj = PX_PLACEMENT_NEW(address, NpArticulationLink(PxBaseFlags(0)));
	address += sizeof(NpArticulationLink);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationLink::NpArticulationLink(const PxTransform& bodyPose, PxArticulationReducedCoordinate& root, NpArticulationLink* parent) :
	NpArticulationLinkT	(PxConcreteType::eARTICULATION_LINK, PxBaseFlag::eOWNS_MEMORY, PxActorType::eARTICULATION_LINK, NpType::eBODY_FROM_ARTICULATION_LINK, bodyPose),
	mRoot				(&root),
	mInboundJoint		(NULL),
	mParent				(parent),
	mLLIndex			(0xffffffff),
	mInboundJointDof	(0xffffffff)
{
	if (parent)
		parent->addToChildList(*this);
}

NpArticulationLink::~NpArticulationLink()
{
}

void NpArticulationLink::releaseInternal()
{
	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, userData);

	NpArticulationReducedCoordinate* npArticulation = static_cast<NpArticulationReducedCoordinate*>(mRoot);
	npArticulation->removeLinkFromList(*this);

	if (mParent)
		mParent->removeFromChildList(*this);

	if (mInboundJoint)
		mInboundJoint->release();

	//Remove constraints, aggregates, scene, shapes. 
	removeRigidActorT<PxArticulationLink>(*this);

	PX_ASSERT(!isAPIWriteForbidden());
	NpDestroyArticulationLink(this);
}

void NpArticulationLink::release()
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::release() not allowed while the articulation link is in a scene. Call will be ignored.");
		return;
	}

	//! this function doesn't get called when the articulation root is released
	// therefore, put deregistration code etc. into dtor, not here

	if (mChildLinks.empty())
	{
		releaseInternal();
	}
	else
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::release(): Only leaf articulation links can be released. Call will be ignored.");
	}
}

PxTransform NpArticulationLink::getGlobalPose() const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationLink::getGlobalPose() not allowed while simulation is running (except during PxScene::collide()).", PxTransform(PxIdentity));

	// PT:: tag: scalar transform*transform
	return mCore.getBody2World() * mCore.getBody2Actor().getInverse();
}

bool NpArticulationLink::attachShape(PxShape& shape)
{
	static_cast<NpArticulationReducedCoordinate*>(mRoot)->incrementShapeCount();
	return NpRigidActorTemplate::attachShape(shape);
}

void NpArticulationLink::detachShape(PxShape& shape, bool wakeOnLostTouch)
{
	static_cast<NpArticulationReducedCoordinate*>(mRoot)->decrementShapeCount();
	NpRigidActorTemplate::detachShape(shape, wakeOnLostTouch);
}

PxArticulationReducedCoordinate& NpArticulationLink::getArticulation() const
{
	NP_READ_CHECK(getNpScene());
	return *mRoot;
}

PxArticulationJointReducedCoordinate* NpArticulationLink::getInboundJoint() const
{
	NP_READ_CHECK(getNpScene());
	return mInboundJoint;
}

PxU32 NpArticulationLink::getInboundJointDof() const
{
	NP_READ_CHECK(getNpScene());

	return getNpScene() ? mInboundJointDof : 0xffffffffu;
}

PxU32 NpArticulationLink::getNbChildren() const
{
	NP_READ_CHECK(getNpScene());
	return mChildLinks.size();
}

PxU32 NpArticulationLink::getChildren(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mChildLinks.begin(), mChildLinks.size());
}

PxU32 NpArticulationLink::getLinkIndex() const
{
	NP_READ_CHECK(getNpScene());
	return getNpScene() ? mLLIndex : 0xffffffffu;
}

void NpArticulationLink::setCMassLocalPose(const PxTransform& pose)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxArticulationLink::setCMassLocalPose: invalid parameter");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationLink::setCMassLocalPose() not allowed while simulation is running. Call will be ignored.")

	const PxTransform p = pose.getNormalized();
	const PxTransform oldpose = mCore.getBody2Actor();
	const PxTransform comShift = p.transformInv(oldpose);

	NpArticulationLinkT::setCMassLocalPoseInternal(p);

	if(mInboundJoint)
	{
		NpArticulationJointReducedCoordinate* j =static_cast<NpArticulationJointReducedCoordinate*>(mInboundJoint);
		// PT:: tag: scalar transform*transform
		j->scSetChildPose(comShift.transform(j->getCore().getChildPose()));
	}

	for(PxU32 i=0; i<mChildLinks.size(); i++)
	{
		NpArticulationJointReducedCoordinate* j = static_cast<NpArticulationJointReducedCoordinate*>(mChildLinks[i]->getInboundJoint());
		// PT:: tag: scalar transform*transform
		j->scSetParentPose(comShift.transform(j->getCore().getParentPose()));
	}
}

void NpArticulationLink::addForce(const PxVec3& force, PxForceMode::Enum mode, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(force.isFinite(), "PxArticulationLink::addForce: force is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxArticulationLink::addForce: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxArticulationLink::addForce() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::addForce(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	addSpatialForce(&force, NULL, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!force.isZero()), autowake);
}

void NpArticulationLink::addTorque(const PxVec3& torque, PxForceMode::Enum mode, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxArticulationLink::addTorque: force is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxArticulationLink::addTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxArticulationLink::addTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::addTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	addSpatialForce(NULL, &torque, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!torque.isZero()), autowake);
}

void NpArticulationLink::setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxArticulationLink::setForceAndTorque: torque is not valid.");
	PX_CHECK_AND_RETURN(force.isFinite(), "PxArticulationLink::setForceAndTorque: force is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxArticulationLink::addTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxArticulationLink::setForceAndTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::setForceAndTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	setSpatialForce(&force, &torque, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!torque.isZero()), true);
}

void NpArticulationLink::clearForce(PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxArticulationLink::clearForce: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxArticulationLink::clearForce() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::clearForce(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	clearSpatialForce(mode, true, false);
}

void NpArticulationLink::clearTorque(PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxArticulationLink::clearTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxArticulationLink::clearTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::clearTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	clearSpatialForce(mode, false, true);
}

void NpArticulationLink::setCfmScale(const PxReal cfmScale) 
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(cfmScale >= 0.f  && cfmScale <= 1.f, "PxArticulationLink::setCfmScale: cfm is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationLink::setCfmScale() not allowed while simulation is running. Call will be ignored.")

	mCore.getCore().cfmScale = cfmScale;
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, CFMScale, static_cast<PxArticulationLink&>(*this), cfmScale); // @@@
}

PxReal NpArticulationLink::getCfmScale() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getCore().cfmScale;
}

void NpArticulationLink::setGlobalPoseInternal(const PxTransform& pose, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxArticulationLink::setGlobalPose: pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationLink::setGlobalPose() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationLink::setGlobalPose(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

#if PX_CHECKED
	if (npScene)
		npScene->checkPositionSanity(*this, pose, "PxArticulationLink::setGlobalPose");
#endif

	const PxTransform newPose = pose.getNormalized();	//AM: added to fix 1461 where users read and write orientations for no reason.

	// PT:: tag: scalar transform*transform
	const PxTransform body2World = newPose * mCore.getBody2Actor();
	scSetBody2World(body2World);

	if (npScene && autowake)
		static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal(false, true);

	if (npScene)
		static_cast<NpArticulationReducedCoordinate*>(mRoot)->setGlobalPose();
}

void NpArticulationLink::setInboundJointDof(const PxU32 index)
{
	mInboundJointDof = index;
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJointDOF, static_cast<PxArticulationLink&>(*this), mInboundJointDof);
}

void NpArticulationLink::setFixedBaseLink(bool value)
{
	NP_WRITE_CHECK(getNpScene());

	mCore.setFixedBaseLink(value);
}

PxU32 physx::NpArticulationGetShapes(NpArticulationLink& actor, NpShape* const*& shapes, bool* isCompound)
{
	NpShapeManager& sm = actor.getShapeManager();
	shapes = sm.getShapes();
	if (isCompound)
		*isCompound = sm.isSqCompound();
	return sm.getNbShapes();
}

