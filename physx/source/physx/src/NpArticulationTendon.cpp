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

#include "NpArticulationTendon.h"
#include "NpArticulationLink.h"
#include "NpArticulationReducedCoordinate.h"
#include "ScArticulationTendonSim.h"
#include "CmUtils.h"

using namespace physx;

NpArticulationAttachment::NpArticulationAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link):
	mLink(link), mParent(parent)
{
	mChildren.reserve(64);

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
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationAttachment::release() not allowed while the articulation is in the scene. Call will be ignored.");
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

NpArticulationSpatialTendon::NpArticulationSpatialTendon(NpArticulationReducedCoordinate* articulation) :
	PxArticulationSpatialTendon(PxConcreteType::eARTICULATION_SPATIAL_TENDON, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_TENDON), mArticulation(articulation)
{
	mAttachments.reserve(50);
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
			PX_FREE(mAttachments[i]);
		}
	}
}

PxArticulationAttachment* NpArticulationSpatialTendon::createAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationSpatialTendon::createAttachment() not allowed while the articulation is in the scene. Call will be ignored.");
		return NULL;
	}
	PX_CHECK_AND_RETURN_NULL(link, "PxArticulationSpatialTendon::createAttachment: Null pointer link provided. Need valid link.");
	PX_CHECK_AND_RETURN_NULL(&link->getArticulation() == getArticulation(), "PxArticulationSpatialTendon::createAttachment: Link from another articulation provided. Need valid link from same articulation.");

#if PX_CHECKED
	if(parent)
		PX_CHECK_AND_RETURN_NULL(parent->getTendon() == this, "PxArticulationSpatialTendon::createAttachment: Parent attachment from another tendon provided. Need valid parent from same tendon.");
#endif

	NpArticulationAttachment* npAttachment = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(NpArticulationAttachment), "NpArticulationAttachment"), NpArticulationAttachment)(parent, coefficient, relativeOffset, link);

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

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setLimitStiffness(stiffness);
	UPDATE_PVD_PROPERTY
}

PxReal	NpArticulationSpatialTendon::getLimitStiffness() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getLimitStiffness();
}

void  NpArticulationSpatialTendon::setOffset(const PxReal offset, bool autowake)
{
	PX_CHECK_AND_RETURN(PxIsFinite(offset), "PxArticulationTendon::setOffset(): invalid value provided!");

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
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
			"PxArticulationSpatialTendon::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	PxArray<NpArticulationSpatialTendon*>& spatialTendons = mArticulation->getSpatialTendons();

	spatialTendons.back()->setHandle(mHandle);
	spatialTendons.replaceWithLast(mHandle);
	this->~NpArticulationSpatialTendon();

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

NpArticulationTendonJoint::NpArticulationTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, 
	const PxReal coefficient, const PxReal recipCoefficient, PxArticulationLink* link) :
	PxArticulationTendonJoint()
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
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationTendonJoint::release() not allowed while the articulation is in the scene. Call will be ignored.");
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

	PX_FREE_THIS;
}

NpArticulationFixedTendon::NpArticulationFixedTendon(NpArticulationReducedCoordinate* articulation) :
	PxArticulationFixedTendon(PxConcreteType::eARTICULATION_FIXED_TENDON, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_TENDON), mArticulation(articulation)
{
	mTendonJoints.reserve(50);
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
			PX_FREE(mTendonJoints[i]);
		}
	}
}

void NpArticulationFixedTendon::release()
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationFixedTendon::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	PxArray<NpArticulationFixedTendon*>& fixedTendons = mArticulation->getFixedTendons();

	fixedTendons.back()->setHandle(mHandle);
	fixedTendons.replaceWithLast(mHandle);
	this->~NpArticulationFixedTendon();

	PX_FREE_THIS;
}

PxArticulationTendonJoint* NpArticulationFixedTendon::createTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, const PxReal coefficient, const PxReal recipCoefficient, PxArticulationLink* link)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationFixedTendon::createTendonJoint() not allowed while the articulation is in the scene. Call will be ignored.");
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

	NpArticulationTendonJoint* npTendonJoint = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(NpArticulationTendonJoint), "NpArticulationTendonJoint"), NpArticulationTendonJoint)(parent, axis, coefficient, recipCoefficient, link);

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

