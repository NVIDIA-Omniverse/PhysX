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

#include "NpArticulationTendon.h"
#include "NpArticulationLink.h"
#include "NpArticulationReducedCoordinate.h"
#include "ScArticulationTendonSim.h"
#include "CmUtils.h"

using namespace physx;

// PX_SERIALIZATION
void NpArticulationAttachment::requiresObjects(PxProcessPxBaseCallback& c)
{
	// Collect articulation links
	const PxU32 nbChildren = mChildren.size();
	for (PxU32 i = 0; i < nbChildren; i++)
		c.process(*mChildren[i]);
}

void NpArticulationAttachment::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mChildren, stream);
}

void NpArticulationAttachment::importExtraData(PxDeserializationContext& context)
{
	Cm::importInlineArray(mChildren, context);
}

void NpArticulationAttachment::resolveReferences(PxDeserializationContext& context)
{
	context.translatePxBase(mLink);
	context.translatePxBase(mParent);

	const PxU32 nbChildren = mChildren.size();
	for (PxU32 i = 0; i < nbChildren; i++)
	{
		NpArticulationAttachment*& attachment = mChildren[i];
		context.translatePxBase(attachment);
	}

	context.translatePxBase(mTendon);
	mCore.mParent = mParent ? &static_cast<NpArticulationAttachment*>(mParent)->mCore : NULL;
}

NpArticulationAttachment* NpArticulationAttachment::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationAttachment* obj = PX_PLACEMENT_NEW(address, NpArticulationAttachment(PxBaseFlags(0)));
	address += sizeof(NpArticulationAttachment);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// ~PX_SERIALIZATION

NpArticulationAttachment::NpArticulationAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link):
	PxArticulationAttachment(PxConcreteType::eARTICULATION_ATTACHMENT, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_ATTACHMENT),
	mLink(link), mParent(parent)
{
	NpArticulationAttachment* npParent = static_cast<NpArticulationAttachment*>(parent);

	mCore.mRelativeOffset = link->getCMassLocalPose().transform(relativeOffset);
	mCore.mParent = npParent ? &npParent->getCore() : NULL;
	mCore.mLowLimit = PX_MAX_F32;
	mCore.mHighLimit = -PX_MAX_F32;
	mCore.mRestLength = 0.f;
	mCore.mCoefficient = coefficient;
	mCore.mTendonSim = NULL;
	mCore.mAttachmentIndex = 0xffffffff;
}

NpArticulationAttachment::~NpArticulationAttachment()
{
}

void NpArticulationAttachment::setRestLength(const PxReal restLength)
{
	PX_CHECK_AND_RETURN(PxIsFinite(restLength), "PxArticulationAttachment::setRestLength(): restLength must have valid value.");
	PX_CHECK_AND_RETURN(isLeaf(), "PxArticulationAttachment::setRestLength(): Setting rest length on a non-leaf attachment has no effect.");

    NpScene* npScene = mTendon ? mTendon->getNpScene() : NULL;
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationAttachment::setRestLength(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	mCore.mRestLength = restLength;

	if (mCore.mTendonSim)
		mCore.mTendonSim->setAttachmentRestLength(mCore, restLength);
}

PxReal NpArticulationAttachment::getRestLength() const
{
	return mCore.mRestLength;
}

void NpArticulationAttachment::setLimitParameters(const PxArticulationTendonLimit& parameter)
{
	PX_CHECK_AND_RETURN(PxIsFinite(parameter.lowLimit) && PxIsFinite(parameter.highLimit) && (parameter.lowLimit <= parameter.highLimit),
						"NpArticulationAttachment::setLimitParameters(): lowLimit and highLimit must have valid values and lowLimit must be less than highLimit!");
	PX_CHECK_AND_RETURN(isLeaf(), "PxArticulationAttachment::setLimitParameters(): Setting limits on a non-leaf attachment has no effect.");

    NpScene* npScene = mTendon ? mTendon->getNpScene() : NULL;
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationAttachment::setLimitParameters(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	mCore.mLowLimit = parameter.lowLimit;
	mCore.mHighLimit = parameter.highLimit;

	if (mCore.mTendonSim)
		mCore.mTendonSim->setAttachmentLimits(mCore, parameter.lowLimit, parameter.highLimit);
}

PxArticulationTendonLimit NpArticulationAttachment::getLimitParameters()const 
{
	PxArticulationTendonLimit parameter;
	parameter.lowLimit = mCore.mLowLimit;
	parameter.highLimit = mCore.mHighLimit;

	return parameter;
}

void NpArticulationAttachment::setRelativeOffset(const PxVec3& offset)
{
    NpScene* npScene = mTendon ? mTendon->getNpScene() : NULL;
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationAttachment::setRelativeOffset(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	mCore.mRelativeOffset = offset;

	if (mCore.mTendonSim)
		mCore.mTendonSim->setAttachmentRelativeOffset(mCore, offset);
}

PxVec3 NpArticulationAttachment::getRelativeOffset() const 
{ 
	return mCore.mRelativeOffset; 
}

void NpArticulationAttachment::setCoefficient(const PxReal coefficient)
{
	PX_CHECK_AND_RETURN(PxIsFinite(coefficient), "PxArticulationAttachment::setCoefficient :: Error: NaN or Inf joint coefficient provided!");

    NpScene* npScene = mTendon ? mTendon->getNpScene() : NULL;
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationAttachment::setCoefficient(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	mCore.mCoefficient = coefficient;

	if (mCore.mTendonSim)
		mCore.mTendonSim->setAttachmentCoefficient(mCore, coefficient);
}

PxReal NpArticulationAttachment::getCoefficient() const
{
	return mCore.mCoefficient;
}

PxArticulationSpatialTendon* NpArticulationAttachment::getTendon() const
{
	return mTendon;
}

void NpArticulationAttachment::release()
{
	if (mTendon->getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationAttachment::release() not allowed while the articulation is in the scene. Call will be ignored.");
		return;
	}

	PX_CHECK_AND_RETURN(getNumChildren() == 0, "PxArticulationAttachment:release() can only release leaf attachments, i.e. attachments with zero children.");

	NpArticulationAttachmentArray& attachments = mTendon->getAttachments();

	NpArticulationAttachment* npParentAttachment = static_cast<NpArticulationAttachment*>(mParent);

	//remove this attachment from the parent
	if (npParentAttachment)
		npParentAttachment->removeChild(this);

	attachments.back()->mHandle = mHandle;
	attachments.replaceWithLast(mHandle);
	this->~NpArticulationAttachment();

	if (mBaseFlags & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE_THIS;
}

void NpArticulationAttachment::removeChild(NpArticulationAttachment* child)
{
	const PxU32 size = mChildren.size();

	PxU32 index = 0;
	for (PxU32 i = 0; i < size; ++i)
	{
		NpArticulationAttachment* otherChild = mChildren[i];
		if (otherChild == child)
		{
			index = i;
			break;
		}
	}
	const PxU32 lastIndex = size - 1;
	mChildren[index] = mChildren[lastIndex];
	mChildren.forceSize_Unsafe(lastIndex);
}

// PX_SERIALIZATION

void NpArticulationSpatialTendon::requiresObjects(PxProcessPxBaseCallback& c)
{
	const PxU32 nbAttachments = mAttachments.size();
	for (PxU32 i = 0; i < nbAttachments; i++)
		c.process(*mAttachments[i]);
}

void NpArticulationSpatialTendon::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mAttachments, stream);
}

void NpArticulationSpatialTendon::importExtraData(PxDeserializationContext& context)
{
	Cm::importInlineArray(mAttachments, context);
}

void NpArticulationSpatialTendon::resolveReferences(PxDeserializationContext& context)
{	
	const PxU32 nbAttachments = mAttachments.size();
	for (PxU32 i = 0; i < nbAttachments; i++)
	{
		NpArticulationAttachment*& attachment = mAttachments[i];
		context.translatePxBase(attachment);
	}

	context.translatePxBase(mArticulation);
}

NpArticulationSpatialTendon* NpArticulationSpatialTendon::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationSpatialTendon* obj = PX_PLACEMENT_NEW(address, NpArticulationSpatialTendon(PxBaseFlags(0)));
	address += sizeof(NpArticulationSpatialTendon);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationSpatialTendon::NpArticulationSpatialTendon(NpArticulationReducedCoordinate* articulation) :
	PxArticulationSpatialTendon(PxConcreteType::eARTICULATION_SPATIAL_TENDON, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_SPATIAL_TENDON), mArticulation(articulation)
{
	mLLIndex = 0xffffffff;
	mHandle = 0xffffffff;
}

NpArticulationSpatialTendon::~NpArticulationSpatialTendon()
{
	for (PxU32 i = 0; i < mAttachments.size(); ++i)
	{
		if (mAttachments[i])
		{
			mAttachments[i]->~NpArticulationAttachment();
			if(mAttachments[i]->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE(mAttachments[i]);
		}
	}
}

PxArticulationAttachment* NpArticulationSpatialTendon::createAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSpatialTendon::createAttachment() not allowed while the articulation is in the scene. Call will be ignored.");
		return NULL;
	}
	PX_CHECK_AND_RETURN_NULL(link, "PxArticulationSpatialTendon::createAttachment: Null pointer link provided. Need valid link.");
	PX_CHECK_AND_RETURN_NULL(&link->getArticulation() == getArticulation(), "PxArticulationSpatialTendon::createAttachment: Link from another articulation provided. Need valid link from same articulation.");

#if PX_CHECKED
	if(parent)
		PX_CHECK_AND_RETURN_NULL(parent->getTendon() == this, "PxArticulationSpatialTendon::createAttachment: Parent attachment from another tendon provided. Need valid parent from same tendon.");
#endif

	void* npAttachmentMem = PX_ALLOC(sizeof(NpArticulationAttachment), "NpArticulationAttachment");
	PxMarkSerializedMemory(npAttachmentMem, sizeof(NpArticulationAttachment));
	NpArticulationAttachment* npAttachment = PX_PLACEMENT_NEW(npAttachmentMem, NpArticulationAttachment)(parent, coefficient, relativeOffset, link);

	if (npAttachment)
	{
		npAttachment->setTendon(this);

		NpArticulationAttachment* parentAttachment = static_cast<NpArticulationAttachment*>(parent);

		ArticulationAttachmentHandle handle = mAttachments.size();
		npAttachment->mHandle = handle;

		mAttachments.pushBack(npAttachment);

		if (parentAttachment)
		{
			parentAttachment->mChildren.pushBack(npAttachment);
			npAttachment->mParent = parent;
		}
		else
		{
			npAttachment->mParent = NULL;
		}
	}
	
	return npAttachment;
}

PxU32 NpArticulationSpatialTendon::getAttachments(PxArticulationAttachment** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(mArticulation->getNpScene());

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mAttachments.begin(), mAttachments.size());
}

void NpArticulationSpatialTendon::setStiffness(const PxReal stiffness)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness) && stiffness >= 0.0f, "PxArticulationTendon::setStiffness: spring coefficient must be >= 0!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationTendon::setStiffness() not allowed while simulation is running. Call will be ignored.")

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSpatialTendon::setStiffness(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setStiffness(stiffness);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationSpatialTendon::getStiffness() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getStiffness();
}

void NpArticulationSpatialTendon::setDamping(const PxReal damping)
{
	PX_CHECK_AND_RETURN(PxIsFinite(damping) && damping >= 0.0f, "PxArticulationTendon::setDamping: damping coefficient must be >= 0!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSpatialTendon::setDamping(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setDamping(damping);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationSpatialTendon::getDamping() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getDamping();
}

void  NpArticulationSpatialTendon::setLimitStiffness(const PxReal stiffness)
{
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness) && stiffness >= 0.0f, "PxArticulationTendon::setLimitStiffness: stiffness must be >= 0!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSpatialTendon::setLimitStiffness(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setLimitStiffness(stiffness);
	UPDATE_PVD_PROPERTY
}

PxReal	NpArticulationSpatialTendon::getLimitStiffness() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getLimitStiffness();
}

void NpArticulationSpatialTendon::setOffset(const PxReal offset, bool autowake)
{
	PX_CHECK_AND_RETURN(PxIsFinite(offset), "PxArticulationTendon::setOffset(): invalid value provided!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSpatialTendon::setOffset(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());

	if (autowake && getNpScene())
		mArticulation->autoWakeInternal();

	mCore.setOffset(offset);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationSpatialTendon::getOffset() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getOffset();
}

PxArticulationReducedCoordinate* physx::NpArticulationSpatialTendon::getArticulation() const
{
	return mArticulation;
}

void NpArticulationSpatialTendon::release()
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
			"PxArticulationSpatialTendon::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	PxArray<NpArticulationSpatialTendon*>& spatialTendons = mArticulation->getSpatialTendons();

	spatialTendons.back()->setHandle(mHandle);
	spatialTendons.replaceWithLast(mHandle);
	this->~NpArticulationSpatialTendon();

	if (mBaseFlags & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE_THIS;
}

NpArticulationAttachment* NpArticulationSpatialTendon::getAttachment(const PxU32 index)
{
	return mAttachments[index];
}

PxU32 NpArticulationSpatialTendon::getNbAttachments() const
{
	return mAttachments.size();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PX_SERIALIZATION
void NpArticulationTendonJoint::requiresObjects(PxProcessPxBaseCallback& c)
{
	// Collect articulation links
	const PxU32 nbChildren = mChildren.size();
	for (PxU32 i = 0; i < nbChildren; i++)
		c.process(*mChildren[i]);
}

void NpArticulationTendonJoint::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mChildren, stream);
}

void NpArticulationTendonJoint::importExtraData(PxDeserializationContext& context)
{
	Cm::importInlineArray(mChildren, context);
}

void NpArticulationTendonJoint::resolveReferences(PxDeserializationContext& context)
{
	context.translatePxBase(mLink);
	context.translatePxBase(mParent);

	const PxU32 nbChildren = mChildren.size();
	for (PxU32 i = 0; i < nbChildren; i++)
	{
		NpArticulationTendonJoint*& tendonJoint = mChildren[i];
		context.translatePxBase(tendonJoint);
	}
		
	context.translatePxBase(mTendon);
	mCore.mParent = mParent ? &static_cast<NpArticulationTendonJoint*>(mParent)->mCore : NULL;
}

NpArticulationTendonJoint* NpArticulationTendonJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationTendonJoint* obj = PX_PLACEMENT_NEW(address, NpArticulationTendonJoint(PxBaseFlags(0)));
	address += sizeof(NpArticulationTendonJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// ~PX_SERIALIZATION

NpArticulationTendonJoint::NpArticulationTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, 
	const PxReal coefficient, const PxReal recipCoefficient, PxArticulationLink* link) :
	PxArticulationTendonJoint(PxConcreteType::eARTICULATION_TENDON_JOINT, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_TENDON_JOINT)
{
	NpArticulationTendonJoint* npParent = static_cast<NpArticulationTendonJoint*>(parent);
	mCore.mParent = parent ? &npParent->getCore() : NULL;
	mCore.mLLTendonJointIndex = 0xffffffff;
	mCore.coefficient = coefficient;
	mCore.recipCoefficient = recipCoefficient;
	mCore.axis = axis;
	mCore.mTendonSim = NULL;

	mLink = link;
	mParent = parent;

	mTendon = NULL;
	mHandle = 0xffffffff;
}

PxArticulationFixedTendon* physx::NpArticulationTendonJoint::getTendon() const
{
	return mTendon;
}

void NpArticulationTendonJoint::setCoefficient(const PxArticulationAxis::Enum axis, const PxReal coefficient, const PxReal recipCoefficient)
{
	PX_CHECK_AND_RETURN(PxIsFinite(coefficient), "PxArticulationTendonJoint::setCoefficient :: Error: NaN or Inf joint coefficient provided!");
	PX_CHECK_AND_RETURN(PxIsFinite(recipCoefficient), "PxArticulationTendonJoint::setCoefficient :: Error: NaN or Inf joint recipCoefficient provided!");

    NpScene* npScene = mTendon ? mTendon->getNpScene() : NULL;
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationTendonJoint::setCoefficient(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}
	
	mCore.coefficient = coefficient;
	mCore.recipCoefficient = recipCoefficient;

	if (mCore.mTendonSim)
	{
		mCore.mTendonSim->setTendonJointCoefficient(mCore, axis, coefficient, recipCoefficient);
	}
}

void NpArticulationTendonJoint::getCoefficient(PxArticulationAxis::Enum& axis, PxReal& coefficient, PxReal& recipCoefficient) const
{
	mCore.getCoefficient(axis, coefficient, recipCoefficient);
}

void NpArticulationTendonJoint::release()
{
	if (mTendon->getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationTendonJoint::release() not allowed while the articulation is in the scene. Call will be ignored.");
		return;
	}

	PX_CHECK_AND_RETURN(getNumChildren() == 0, "PxArticulationTendonJoint::release() can only release leaf tendon joints, i.e. joints with zero children.");

	NpArticulationTendonJointArray& tendonJoints = mTendon->getTendonJoints();

	//remove this joint from the parent
	NpArticulationTendonJoint* npParentJoint = static_cast<NpArticulationTendonJoint*>(mParent);
	if (npParentJoint)
		npParentJoint->removeChild(this);

	tendonJoints.back()->mHandle = mHandle;
	tendonJoints.replaceWithLast(mHandle);
	this->~NpArticulationTendonJoint();

	if (mBaseFlags & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE_THIS;
}

// PX_SERIALIZATION

void NpArticulationFixedTendon::requiresObjects(PxProcessPxBaseCallback& c)
{
	const PxU32 nbTendonJoints = mTendonJoints.size();
	for (PxU32 i = 0; i < nbTendonJoints; i++)
		c.process(*mTendonJoints[i]);
}

void NpArticulationFixedTendon::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mTendonJoints, stream);
}

void NpArticulationFixedTendon::importExtraData(PxDeserializationContext& context)
{
	Cm::importInlineArray(mTendonJoints, context);
}

void NpArticulationFixedTendon::resolveReferences(PxDeserializationContext& context)
{	
	const PxU32 nbTendonJoints = mTendonJoints.size();
	for (PxU32 i = 0; i < nbTendonJoints; i++)
	{
		NpArticulationTendonJoint*& tendonJoint = mTendonJoints[i];
		context.translatePxBase(tendonJoint);
	}

	context.translatePxBase(mArticulation);
}

NpArticulationFixedTendon* NpArticulationFixedTendon::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationFixedTendon* obj = PX_PLACEMENT_NEW(address, NpArticulationFixedTendon(PxBaseFlags(0)));
	address += sizeof(NpArticulationFixedTendon);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationFixedTendon::NpArticulationFixedTendon(NpArticulationReducedCoordinate* articulation) :
	PxArticulationFixedTendon(PxConcreteType::eARTICULATION_FIXED_TENDON, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_FIXED_TENDON), mArticulation(articulation)
{
	mLLIndex = 0xffffffff;
	mHandle = 0xffffffff;
}

NpArticulationFixedTendon::~NpArticulationFixedTendon()
{
	for (PxU32 i = 0; i < mTendonJoints.size(); ++i)
	{
		if (mTendonJoints[i])
		{
			mTendonJoints[i]->~NpArticulationTendonJoint();
			if (mTendonJoints[i]->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
			{
				PX_FREE(mTendonJoints[i]);
			}
		}
	}
}

void NpArticulationFixedTendon::release()
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	PxArray<NpArticulationFixedTendon*>& fixedTendons = mArticulation->getFixedTendons();

	fixedTendons.back()->setHandle(mHandle);
	fixedTendons.replaceWithLast(mHandle);
	this->~NpArticulationFixedTendon();

	if (mBaseFlags & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE_THIS;
}

PxArticulationTendonJoint* NpArticulationFixedTendon::createTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, const PxReal coefficient, const PxReal recipCoefficient, PxArticulationLink* link)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::createTendonJoint() not allowed while the articulation is in the scene. Call will be ignored.");
		return NULL;
	}
	PX_CHECK_AND_RETURN_NULL(link, "PxArticulationFixedTendon::createTendonJoint: Null pointer link provided. Need valid link.");
	PX_CHECK_AND_RETURN_NULL(&link->getArticulation() == getArticulation(), "PxArticulationFixedTendon::createTendonJoint: Link from another articulation provided. Need valid link from same articulation.");

#if PX_CHECKED
	if (parent)
	{
		PX_CHECK_AND_RETURN_NULL(parent->getTendon() == this, "PxArticulationFixedTendon::createTendonJoint: Parent tendon joint from another tendon provided. Need valid parent from same tendon.");
		PX_CHECK_AND_RETURN_NULL(parent->getLink() != link, "PxArticulationFixedTendon::createTendonJoint :: Error: Parent link and child link are the same link!");
		PX_CHECK_AND_RETURN_NULL(parent->getLink()== (&link->getInboundJoint()->getParentArticulationLink()), "PxArticulationFixedTendon::createTendonJoint :: Error: Link referenced by parent tendon joint must be the parent of the child link!");
	}
#endif

	void* npTendonJointtMem = PX_ALLOC(sizeof(NpArticulationTendonJoint), "NpArticulationTendonJoint");
	PxMarkSerializedMemory(npTendonJointtMem, sizeof(NpArticulationTendonJoint));
	NpArticulationTendonJoint* npTendonJoint = PX_PLACEMENT_NEW(npTendonJointtMem, NpArticulationTendonJoint)(parent, axis, coefficient, recipCoefficient, link);

	if (npTendonJoint)
	{
		NpArticulationTendonJoint* parentTendonJoint = static_cast<NpArticulationTendonJoint*>(parent);

		npTendonJoint->setTendon(this);
		if (parentTendonJoint)
		{
			parentTendonJoint->mChildren.pushBack(npTendonJoint);
			npTendonJoint->mParent = parent;
		}
		else
		{
			npTendonJoint->mParent = NULL;
		}
		npTendonJoint->mHandle = mTendonJoints.size();
		mTendonJoints.pushBack(npTendonJoint);
	}

	return npTendonJoint;
}

PxU32 NpArticulationFixedTendon::getTendonJoints(PxArticulationTendonJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(mArticulation->getNpScene());

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mTendonJoints.begin(), mTendonJoints.size());
}

void NpArticulationFixedTendon::setStiffness(const PxReal stiffness)
{
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness) && stiffness >= 0.0f, "PxArticulationTendon::setStiffness: spring coefficient must be >= 0!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setStiffness(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setStiffness(stiffness);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationFixedTendon::getStiffness() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getStiffness();
}

void NpArticulationFixedTendon::setDamping(const PxReal damping)
{
	PX_CHECK_AND_RETURN(PxIsFinite(damping) && damping >= 0.0f, "PxArticulationTendon::setDamping: damping coefficient must be >= 0!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setDamping(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setDamping(damping);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationFixedTendon::getDamping() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getDamping();
}

void NpArticulationFixedTendon::setLimitStiffness(const PxReal stiffness)
{
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness) && stiffness >=0.f , "PxArticulationTendon::setLimitStiffness: stiffness must have valid value!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setLimitStiffness(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setLimitStiffness(stiffness);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationFixedTendon::getLimitStiffness() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getLimitStiffness();
}

void NpArticulationFixedTendon::setRestLength(const PxReal restLength)
{
	PX_CHECK_AND_RETURN(PxIsFinite(restLength) , "PxArticulationTendon::setRestLength: restLength must have valid value!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setRestLength(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setSpringRestLength(restLength);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationFixedTendon::getRestLength() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getSpringRestLength();
}

void NpArticulationFixedTendon::setLimitParameters(const PxArticulationTendonLimit& parameter)
{
	PX_CHECK_AND_RETURN(PxIsFinite(parameter.lowLimit) && PxIsFinite(parameter.highLimit) && (parameter.lowLimit <= parameter.highLimit), "PxArticulationFixedTendon::setLimitParameters: lowLimit and highLimit must have valid values and lowLimit must be less than highLimit!");

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setLimitParameters(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setLimitRange(parameter.lowLimit, parameter.highLimit);
	UPDATE_PVD_PROPERTY
}

PxArticulationTendonLimit NpArticulationFixedTendon::getLimitParameters() const
{
	NP_READ_CHECK(getNpScene());

	PxArticulationTendonLimit parameter;
	mCore.getLimitRange(parameter.lowLimit, parameter.highLimit);
	return parameter;
}

NpArticulationTendonJoint* NpArticulationFixedTendon::getTendonJoint(const PxU32 index)
{
	return mTendonJoints[index];
}

PxU32 NpArticulationFixedTendon::getNbTendonJoints() const
{
	return mTendonJoints.size();
}

void  NpArticulationFixedTendon::setOffset(const PxReal offset, bool autowake)
{
	PX_CHECK_AND_RETURN(PxIsFinite(offset), "PxArticulationTendon::setOffset(): invalid value provided!");

	PX_ASSERT(!isAPIWriteForbidden());

	if (getNpScene() && (getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && getNpScene()->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationFixedTendon::setOffset(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	if (autowake && getNpScene())
		mArticulation->autoWakeInternal();

	mCore.setOffset(offset);
	UPDATE_PVD_PROPERTY
}

PxReal NpArticulationFixedTendon::getOffset() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getOffset();
}

PxArticulationReducedCoordinate* physx::NpArticulationFixedTendon::getArticulation() const
{
	return mArticulation;
}

void NpArticulationTendonJoint::removeChild(NpArticulationTendonJoint* child)
{
	for(PxU32 i = 0; i < mChildren.size(); ++i)
	{
		if(mChildren[i] == child)
		{
			mChildren.replaceWithLast(i);
			break;
		}
	}
}

