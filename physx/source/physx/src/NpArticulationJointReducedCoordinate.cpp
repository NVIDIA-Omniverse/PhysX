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

#include "NpCheck.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationReducedCoordinate.h"

#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;

//PX_SERIALIZATION

NpArticulationJointReducedCoordinate* NpArticulationJointReducedCoordinate::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationJointReducedCoordinate* obj = PX_PLACEMENT_NEW(address, NpArticulationJointReducedCoordinate(PxBaseFlags(0)));
	address += sizeof(NpArticulationJointReducedCoordinate);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void NpArticulationJointReducedCoordinate::resolveReferences(PxDeserializationContext& context)
{
	//mImpl.resolveReferences(context, *this);
	context.translatePxBase(mParent);
	context.translatePxBase(mChild);
	mCore.setRoot(this);
	NpArticulationReducedCoordinate* articulation = static_cast<NpArticulationReducedCoordinate*>(&mParent->getRoot());
	mCore.setArticulation(&articulation->getCore());
}

//~PX_SERIALIZATION

NpArticulationJointReducedCoordinate::NpArticulationJointReducedCoordinate(NpArticulationLink& parent, const PxTransform& parentFrame,
	NpArticulationLink& child, const PxTransform& childFrame) 
: PxArticulationJointReducedCoordinate(PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE, PxBaseFlag::eOWNS_MEMORY),
	NpBase(NpType::eARTICULATION_JOINT),
	mCore(parentFrame, childFrame),
	mParent(&parent),
	mChild(&child)
{
	NpArticulationReducedCoordinate* articulation = static_cast<NpArticulationReducedCoordinate*>(&parent.getRoot());
	mCore.setArticulation(&articulation->getCore());
	mCore.setRoot(this);
}

NpArticulationJointReducedCoordinate::~NpArticulationJointReducedCoordinate()
{
}

void NpArticulationJointReducedCoordinate::setJointType(PxArticulationJointType::Enum jointType)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setJointType() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}
	PX_CHECK_AND_RETURN(jointType != PxArticulationJointType::eUNDEFINED, "PxArticulationJointReducedCoordinate::setJointType valid joint type(ePRISMATIC, eREVOLUTE, eREVOLUTE_UNWRAPPED, eSPHERICAL, eFIX) need to be set");

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, type, static_cast<PxArticulationJointReducedCoordinate&>(*this), jointType)

	scSetJointType(jointType);
}

PxArticulationJointType::Enum NpArticulationJointReducedCoordinate::getJointType() const
{
	return mCore.getJointType();
}

#if PX_CHECKED
bool NpArticulationJointReducedCoordinate::isValidMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
{
	bool valid = true;
	switch (mCore.getJointType())
	{
	case PxArticulationJointType::ePRISMATIC:
	{
		if (axis < PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
			valid = false;
		else if(motion != PxArticulationMotion::eLOCKED)
		{
			//Check to ensure that we only have zero DOFs already active...
			for (PxU32 i = PxArticulationAxis::eX; i <= PxArticulationAxis::eZ; i++)
			{
				if(i != PxU32(axis) && mCore.getMotion(PxArticulationAxis::Enum(i)) != PxArticulationMotion::eLOCKED)
					valid = false;
			}
		}
		break;
	}
	case PxArticulationJointType::eREVOLUTE:
	case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
	{
		if (axis >= PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
			valid = false;
		else if (motion != PxArticulationMotion::eLOCKED)
		{
			for (PxU32 i = PxArticulationAxis::eTWIST; i < PxArticulationAxis::eX; i++)
			{
				if (i != PxU32(axis) && mCore.getMotion(PxArticulationAxis::Enum(i)) != PxArticulationMotion::eLOCKED)
					valid = false;
			}
		}
		break;
	}
	case PxArticulationJointType::eSPHERICAL:
	{
		if (axis >= PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
			valid = false;
		break;
	}
	case PxArticulationJointType::eFIX:
	{
		if (motion != PxArticulationMotion::eLOCKED)
			valid = false;
		break;
	}
	case PxArticulationJointType::eUNDEFINED:
	{
		valid = false;
		break;
	}
	default:
		break;
	}

	return valid;
}
#endif

void NpArticulationJointReducedCoordinate::setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setMotion() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eUNDEFINED, "PxArticulationJointReducedCoordinate::setMotion valid joint type(ePRISMATIC, eREVOLUTE, eREVOUTE_UNWRAPPED, eSPHERICAL or eFIX) has to be set before setMotion");
	PX_CHECK_AND_RETURN(isValidMotion(axis, motion), "PxArticulationJointReducedCoordinate::setMotion illegal configuration for the joint type that is set.");

	scSetMotion(axis, motion);

#if PX_SUPPORT_OMNI_PVD
	PxArticulationMotion::Enum motions[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		motions[ax] = mCore.getMotion(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, motion, static_cast<PxArticulationJointReducedCoordinate&>(*this), motions, PxArticulationAxis::eCOUNT);
#endif

	static_cast<NpArticulationReducedCoordinate*>(&getChild().getArticulation())->mTopologyChanged = true;
}

PxArticulationMotion::Enum NpArticulationJointReducedCoordinate::getMotion(PxArticulationAxis::Enum axis) const
{
	return mCore.getMotion(axis);
}

void NpArticulationJointReducedCoordinate::setFrictionCoefficient(const PxReal coefficient)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setFrictionCoefficient() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, frictionCoefficient, static_cast<PxArticulationJointReducedCoordinate&>(*this), coefficient);

	scSetFrictionCoefficient(coefficient);
}

PxReal NpArticulationJointReducedCoordinate::getFrictionCoefficient() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getFrictionCoefficient();
}

void NpArticulationJointReducedCoordinate::setMaxJointVelocity(const PxReal maxJointV)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setMaxJointVelocity() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, maxJointVelocity, static_cast<PxArticulationJointReducedCoordinate&>(*this), maxJointV);

	scSetMaxJointVelocity(maxJointV);
}

PxReal NpArticulationJointReducedCoordinate::getMaxJointVelocity() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getMaxJointVelocity();
}

void NpArticulationJointReducedCoordinate::setLimitParams(PxArticulationAxis::Enum axis, const PxArticulationLimit& pair)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(PxIsFinite(pair.low) && PxIsFinite(pair.high) && pair.low <= pair.high, "PxArticulationJointReducedCoordinate::setLimitParams(): Invalid limit parameters; lowLimit must be <= highLimit.");
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eSPHERICAL || (PxAbs(pair.low) <= PxPi && PxAbs(pair.high) <= PxPi), "PxArticulationJointReducedCoordinate::setLimitParams() only supports limit angles in range [-Pi, Pi] for joints of type PxArticulationJointType::eSPHERICAL");		
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eREVOLUTE || (PxAbs(pair.low) <= 2.0f*PxPi && PxAbs(pair.high) <= 2.0f*PxPi), "PxArticulationJointReducedCoordinate::setLimitParams() only supports limit angles in range [-2Pi, 2Pi] for joints of type PxArticulationJointType::eREVOLUTE");		

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setLimitParams() not allowed while simulation is running. Call will be ignored.")

	scSetLimit(axis, pair);

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxArticulationJointReducedCoordinate& joint = *this;
	PxReal limits[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		limits[ax] = mCore.getLimit(static_cast<PxArticulationAxis::Enum>(ax)).low;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitLow, joint, limits, PxArticulationAxis::eCOUNT);
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		limits[ax] = mCore.getLimit(static_cast<PxArticulationAxis::Enum>(ax)).high;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitHigh, joint, limits, PxArticulationAxis::eCOUNT);

	OMNI_PVD_WRITE_SCOPE_END
#endif
}

PxArticulationLimit NpArticulationJointReducedCoordinate::getLimitParams(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getLimit(axis);
}

void NpArticulationJointReducedCoordinate::setDriveParams(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setDriveParams() not allowed while simulation is running. Call will be ignored.")

	scSetDrive(axis, drive);

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxArticulationJointReducedCoordinate& joint = *this;
	PxReal stiffnesss[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		stiffnesss[ax] = mCore.getDrive(static_cast<PxArticulationAxis::Enum>(ax)).stiffness;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveStiffness, joint, stiffnesss, PxArticulationAxis::eCOUNT);
	PxReal dampings[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		dampings[ax] = mCore.getDrive(static_cast<PxArticulationAxis::Enum>(ax)).damping;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveDamping, joint, dampings, PxArticulationAxis::eCOUNT);
	PxReal maxforces[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		maxforces[ax] = mCore.getDrive(static_cast<PxArticulationAxis::Enum>(ax)).maxForce;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxForce, joint, maxforces, PxArticulationAxis::eCOUNT);
	PxArticulationDriveType::Enum drivetypes[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		drivetypes[ax] = mCore.getDrive(static_cast<PxArticulationAxis::Enum>(ax)).driveType;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveType, joint, drivetypes, PxArticulationAxis::eCOUNT);

	OMNI_PVD_WRITE_SCOPE_END
#endif
}

PxArticulationDrive NpArticulationJointReducedCoordinate::getDriveParams(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getDrive(axis);
}

void NpArticulationJointReducedCoordinate::setDriveTarget(PxArticulationAxis::Enum axis, const PxReal target, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eSPHERICAL || PxAbs(target) <= PxPi, "PxArticulationJointReducedCoordinate::setDriveTarget() only supports target angle in range [-Pi, Pi] for joints of type PxArticulationJointType::eSPHERICAL");		
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eREVOLUTE || PxAbs(target) <= 2.0f*PxPi, "PxArticulationJointReducedCoordinate::setDriveTarget() only supports target angle in range [-2Pi, 2Pi] for joints of type PxArticulationJointType::eREVOLUTE");		

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationJointReducedCoordinate::setDriveTarget() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setDriveTarget(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	if (autowake && npScene)
	{
		NpArticulationReducedCoordinate* npArticulation = static_cast<NpArticulationReducedCoordinate*>(&mParent->getArticulation());
		npArticulation->autoWakeInternal();
	}

	scSetDriveTarget(axis, target);

#if PX_SUPPORT_OMNI_PVD
	PxReal targets[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		targets[ax] = mCore.getTargetP(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, static_cast<PxArticulationJointReducedCoordinate&>(*this), targets, PxArticulationAxis::eCOUNT);
#endif
}

void NpArticulationJointReducedCoordinate::setDriveVelocity(PxArticulationAxis::Enum axis, const PxReal targetVel, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationJointReducedCoordinate::setDriveVelocity() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setDriveVelocity(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	if (autowake && npScene)
	{
		NpArticulationReducedCoordinate* npArticulation = static_cast<NpArticulationReducedCoordinate*>(&mParent->getArticulation());
		npArticulation->autoWakeInternal();
	}

	scSetDriveVelocity(axis, targetVel);

#if PX_SUPPORT_OMNI_PVD
	PxReal velocities[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		velocities[ax] = mCore.getTargetV(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, static_cast<PxArticulationJointReducedCoordinate&>(*this), velocities, PxArticulationAxis::eCOUNT);
#endif
}

PxReal NpArticulationJointReducedCoordinate::getDriveTarget(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getTargetP(axis);
}

PxReal NpArticulationJointReducedCoordinate::getDriveVelocity(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getTargetV(axis);
}

void NpArticulationJointReducedCoordinate::setArmature(PxArticulationAxis::Enum axis, const PxReal armature)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setArmature() not allowed while simulation is running. Call will be ignored.")

	scSetArmature(axis, armature);

#if PX_SUPPORT_OMNI_PVD
	PxReal armatures[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		armatures[ax] = mCore.getArmature(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, armature, static_cast<PxArticulationJointReducedCoordinate&>(*this), armatures, PxArticulationAxis::eCOUNT);
#endif
}

PxReal	NpArticulationJointReducedCoordinate::getArmature(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getArmature(axis);
}

PxTransform	NpArticulationJointReducedCoordinate::getParentPose() const
{
	NP_READ_CHECK(getNpScene());
	// PT:: tag: scalar transform*transform
	return mParent->getCMassLocalPose().transform(mCore.getParentPose());
}

void NpArticulationJointReducedCoordinate::setParentPose(const PxTransform& t)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(t.isSane(), "PxArticulationJointReducedCoordinate::setParentPose: Input pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setParentPose() not allowed while simulation is running. Call will be ignored.")

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxArticulationJointReducedCoordinate& joint = *this;
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentTranslation, joint, t.p)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentRotation, joint, t.q)

	OMNI_PVD_WRITE_SCOPE_END
#endif

	if (mParent == NULL)
		return;

	scSetParentPose(mParent->getCMassLocalPose().transformInv(t.getNormalized()));
}

PxTransform NpArticulationJointReducedCoordinate::getChildPose() const
{
	NP_READ_CHECK(getNpScene());

	// PT:: tag: scalar transform*transform
	return mChild->getCMassLocalPose().transform(mCore.getChildPose());
}

void NpArticulationJointReducedCoordinate::setChildPose(const PxTransform& t)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(t.isSane(), "PxArticulationJointReducedCoordinate::setChildPose: Input pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationJointReducedCoordinate::setChildPose() not allowed while simulation is running. Call will be ignored.")

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxArticulationJointReducedCoordinate& joint = *this;
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childTranslation, joint, t.p)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childRotation, joint, t.q)

	OMNI_PVD_WRITE_SCOPE_END
#endif

	scSetChildPose(mChild->getCMassLocalPose().transformInv(t.getNormalized()));
}

void NpArticulationJointReducedCoordinate::setJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(jointPos), "PxArticulationJointReducedCoordinate::setJointPosition: jointPos is not valid.");
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eSPHERICAL || PxAbs(jointPos) <= PxPi, "PxArticulationJointReducedCoordinate::setJointPosition() only supports jointPos in range [-Pi, Pi] for joints of type PxArticulationJointType::eSPHERICAL");	
	PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eREVOLUTE || PxAbs(jointPos) <= 2.0f*PxPi, "PxArticulationJointReducedCoordinate::setJointPosition() only supports jointPos in range [-2Pi, 2Pi] for joints of type PxArticulationJointType::eREVOLUTE");	

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationJointReducedCoordinate::setJointPosition() not allowed while simulation is running. Call will be ignored.");

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setJointPosition(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	scSetJointPosition(axis, jointPos);

#if PX_SUPPORT_OMNI_PVD
	PxReal positions[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		positions[ax] = mCore.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, static_cast<PxArticulationJointReducedCoordinate&>(*this), positions, PxArticulationAxis::eCOUNT);
#endif
}

PxReal NpArticulationJointReducedCoordinate::getJointPosition(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationJointReducedCoordinate::getJointPosition() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", 0.f);

	return mCore.getJointPosition(axis);
}

void NpArticulationJointReducedCoordinate::setJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(jointVel), "PxArticulationJointReducedCoordinate::setJointVelocity: jointVel is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationJointReducedCoordinate::setJointVelocity() not allowed while simulation is running. Call will be ignored.");

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxArticulationJointReducedCoordinate::setJointVelocity(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
	}

	scSetJointVelocity(axis, jointVel);

#if PX_SUPPORT_OMNI_PVD
	PxReal velocities[PxArticulationAxis::eCOUNT];
	for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
		velocities[ax] = mCore.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, static_cast<PxArticulationJointReducedCoordinate&>(*this), velocities, PxArticulationAxis::eCOUNT);
#endif
}

PxReal NpArticulationJointReducedCoordinate::getJointVelocity(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationJointReducedCoordinate::getJointVelocity() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", 0.f);

	return mCore.getJointVelocity(axis);
}

void NpArticulationJointReducedCoordinate::release()
{
	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, NULL);

	if(getNpScene())
		getNpScene()->scRemoveArticulationJoint(*this);

	PX_ASSERT(!isAPIWriteForbidden());
	NpDestroyArticulationJoint(mCore.getRoot());
}
