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

#include "NpArticulationSensor.h"
#include "PxArticulationLink.h"
#include "NpArticulationLink.h"
#include "ScArticulationSensorSim.h"
#include "NpArticulationReducedCoordinate.h"

using namespace physx;

namespace physx
{

// PX_SERIALIZATION

void NpArticulationSensor::resolveReferences(PxDeserializationContext& context)
{	
	context.translatePxBase(mLink);
}

NpArticulationSensor* NpArticulationSensor::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationSensor* obj = PX_PLACEMENT_NEW(address, NpArticulationSensor(PxBaseFlags(0)));
	address += sizeof(NpArticulationSensor);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationSensor::NpArticulationSensor(PxArticulationLink* link, const PxTransform& relativePose) : 
	PxArticulationSensor(PxConcreteType::eARTICULATION_SENSOR, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_SENSOR)
{
	mLink = link;
	mCore.mRelativePose = relativePose;
	mCore.mSim = NULL;
	mCore.mFlags = 0;
}

void NpArticulationSensor::release()
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationSensor::release() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	NpArticulationReducedCoordinate* articulation = static_cast<NpArticulationReducedCoordinate*>(&mLink->getArticulation());
	
	PxArray<NpArticulationSensor*>& sensors = articulation->getSensors();

	PX_CHECK_AND_RETURN(mHandle < sensors.size() && sensors[mHandle] == this,
		"PxArticulationSensor::release() attempt to release sensor that is not part of this articulation.");

	sensors.back()->setHandle(mHandle);
	sensors.replaceWithLast(mHandle);
	this->~NpArticulationSensor();

	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE_THIS;
}

PxSpatialForce NpArticulationSensor::getForces() const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationSensor::getForces() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxSpatialForce());

	if (mCore.getSim())
		return mCore.getSim()->getForces();
	PxSpatialForce zero;
	zero.force = PxVec3(0.f);
	zero.torque = PxVec3(0.f);
	return zero;
}

PxTransform NpArticulationSensor::getRelativePose() const
{
	return mCore.mRelativePose;
}

void NpArticulationSensor::setRelativePose(const PxTransform& pose)
{
	mCore.mRelativePose = pose;

	if (mCore.getSim())
	{
		mCore.getSim()->setRelativePose(pose);
	}
}

PxArticulationLink* NpArticulationSensor::getLink() const
{
	return mLink;
}

PxU32 NpArticulationSensor::getIndex() const
{
	NP_READ_CHECK(getNpScene());
	if(mCore.getSim())
		return mCore.getSim()->getLowLevelIndex();
	return 0xFFFFFFFFu;
}

PxArticulationReducedCoordinate* NpArticulationSensor::getArticulation() const
{
	return &mLink->getArticulation();
}

PxArticulationSensorFlags NpArticulationSensor::getFlags() const
{
	return PxArticulationSensorFlags(PxU8(mCore.mFlags));
}

void NpArticulationSensor::setFlag(PxArticulationSensorFlag::Enum flag, bool enabled)
{

	if(enabled)
		mCore.mFlags |= flag;
	else
		mCore.mFlags &= (~flag);

	if (mCore.getSim())
	{
		mCore.getSim()->setFlag(mCore.mFlags);
	}
}

}

