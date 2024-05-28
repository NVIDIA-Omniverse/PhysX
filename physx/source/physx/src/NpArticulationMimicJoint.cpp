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

NpArticulationMimicJoint::NpArticulationMimicJoint(const PxArticulationJointReducedCoordinate& jointA, PxArticulationAxis::Enum axisA, const PxArticulationJointReducedCoordinate& jointB, PxArticulationAxis::Enum axisB, PxReal gearRatio, PxReal offset) : 
	PxArticulationMimicJoint(PxConcreteType::eARTICULATION_MIMIC_JOINT, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_MIMIC_JOINT)
{
	mLinkA = &jointA.getChildArticulationLink();
	mLinkB = &jointB.getChildArticulationLink();
	mCore.mAxisA = axisA;
	mCore.mAxisB = axisB;
	mCore.mGearRatio = gearRatio;
	mCore.mOffset = offset;
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

PxReal NpArticulationMimicJoint::getGearRatio() const
{
	return mCore.mGearRatio;
}

PxReal NpArticulationMimicJoint::getOffset() const
{
	return mCore.mOffset;
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

