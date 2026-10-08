// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpArticulationMimicJoint.h"
#include "PxArticulationLink.h"
#include "NpArticulationLink.h"
#include "NpArticulationReducedCoordinate.h"
#include "ScArticulationMimicJointSim.h"

namespace physx
{

// PX_SERIALIZATION

void NpArticulationMimicJoint::resolveReferences(PxDeserializationContext& context)
{
	context.translatePxBase(mLinkA);
	context.translatePxBase(mLinkB);
}

NpArticulationMimicJoint* NpArticulationMimicJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationMimicJoint* obj = PX_PLACEMENT_NEW(address, NpArticulationMimicJoint(PxBaseFlags(0)));
	address += sizeof(NpArticulationMimicJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationMimicJoint::NpArticulationMimicJoint(
	const PxArticulationJointReducedCoordinate& jointA, PxArticulationAxis::Enum axisA, 
	const PxArticulationJointReducedCoordinate& jointB, PxArticulationAxis::Enum axisB, 
	PxReal gearRatio, PxReal offset,
	PxReal naturalFrequency, PxReal dampingRatio) :
	PxArticulationMimicJoint(PxConcreteType::eARTICULATION_MIMIC_JOINT, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_MIMIC_JOINT)
{
	mLinkA = &jointA.getChildArticulationLink();
	mLinkB = &jointB.getChildArticulationLink();
	mCore.mAxisA = axisA;
	mCore.mAxisB = axisB;
	mCore.mGearRatio = gearRatio;
	mCore.mOffset = offset;
	mCore.mNaturalFrequency = naturalFrequency;
	mCore.mDampingRatio = dampingRatio;
}

void NpArticulationMimicJoint::release()
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "NpArticulationMimicJoint::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	NpArticulationReducedCoordinate* articulation = static_cast<NpArticulationReducedCoordinate*>(&mLinkA->getArticulation());
	
	PxArray<NpArticulationMimicJoint*>& mimicJoints = articulation->getMimicJoints();

	PX_CHECK_AND_RETURN(mHandle < mimicJoints.size() && mimicJoints[mHandle] == this,
		"PxArticulationMimicJoint::release() attempt to release mimic joint that is not part of this articulation.");

	mimicJoints.back()->setHandle(mHandle);
	mimicJoints.replaceWithLast(mHandle);

	NpDestroyArticulationMimicJoint(this);
}

PxArticulationReducedCoordinate& NpArticulationMimicJoint::getArticulation() const
{
	return mLinkA->getArticulation();
}

void NpArticulationMimicJoint::setGearRatio(const PxReal gearRatio)
{
	mCore.mGearRatio = gearRatio;

	if (mCore.getSim())
	{
		mCore.getSim()->setGearRatio(gearRatio);
	}
}

void NpArticulationMimicJoint::setOffset(const PxReal offset)
{
	mCore.mOffset = offset;

	if (mCore.getSim())
	{
		mCore.getSim()->setOffset(offset);
	}
}

void NpArticulationMimicJoint::setNaturalFrequency(const PxReal naturalFrequency)
{
	mCore.mNaturalFrequency = naturalFrequency;

	if (mCore.getSim())
	{
		mCore.getSim()->setNaturalFrequency(naturalFrequency);
	}
}

void NpArticulationMimicJoint::setDampingRatio(const PxReal dampingRatio)
{
	mCore.mDampingRatio = dampingRatio;

	if (mCore.getSim())
	{
		mCore.getSim()->setDampingRatio(dampingRatio);
	}
}

PxReal NpArticulationMimicJoint::getGearRatio() const
{
	return mCore.mGearRatio;
}

PxReal NpArticulationMimicJoint::getOffset() const
{
	return mCore.mOffset;
}

PxReal NpArticulationMimicJoint::getNaturalFrequency() const
{
	return mCore.mNaturalFrequency;
}

PxReal NpArticulationMimicJoint::getDampingRatio() const
{
	return mCore.mDampingRatio;
}

PxArticulationJointReducedCoordinate& NpArticulationMimicJoint::getJointA() const 
{
	return *mLinkA->getInboundJoint();
}

PxArticulationJointReducedCoordinate& NpArticulationMimicJoint::getJointB() const 
{
	return *mLinkB->getInboundJoint();
}

PxArticulationAxis::Enum NpArticulationMimicJoint::getAxisA() const
{
	return PxArticulationAxis::Enum(mCore.mAxisA);
}

PxArticulationAxis::Enum NpArticulationMimicJoint::getAxisB() const
{
	return PxArticulationAxis::Enum(mCore.mAxisB);
}

}

