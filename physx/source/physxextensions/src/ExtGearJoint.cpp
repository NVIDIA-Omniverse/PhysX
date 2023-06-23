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

#include "ExtGearJoint.h"
#include "ExtConstraintHelper.h"
#include "extensions/PxRevoluteJoint.h"
#include "PxArticulationJointReducedCoordinate.h"
//#include <stdio.h>

using namespace physx;
using namespace Ext;

PX_IMPLEMENT_OUTPUT_ERROR

GearJoint::GearJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	GearJointT(PxJointConcreteType::eGEAR, actor0, localFrame0, actor1, localFrame1, "GearJointData")
{
	GearJointData* data = static_cast<GearJointData*>(mData);
	data->hingeJoint0 = NULL;
	data->hingeJoint1 = NULL;
	data->gearRatio = 0.0f;
	data->error = 0.0f;

	resetError();
}

bool GearJoint::setHinges(const PxBase* hinge0, const PxBase* hinge1)
{
	GearJointData* data = static_cast<GearJointData*>(mData);

	if(!hinge0 || !hinge1)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxGearJoint::setHinges: cannot pass null pointers to this function.");

	const PxType type0 = hinge0->getConcreteType();
	if(type0 == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
	{
		const PxArticulationJointReducedCoordinate* joint0 = static_cast<const PxArticulationJointReducedCoordinate*>(hinge0);
		const PxArticulationJointType::Enum artiJointType = joint0->getJointType();
		if(artiJointType != PxArticulationJointType::eREVOLUTE && artiJointType != PxArticulationJointType::eREVOLUTE_UNWRAPPED)
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxGearJoint::setHinges: passed joint must be either a revolute joint.");
	}
	else
	{
		if(type0 != PxJointConcreteType::eREVOLUTE && type0 != PxJointConcreteType::eD6)
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxGearJoint::setHinges: passed joint must be either a revolute joint or a D6 joint.");
	}
		
	const PxType type1 = hinge1->getConcreteType();
	if(type1 == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
	{
		const PxArticulationJointReducedCoordinate* joint1 = static_cast<const PxArticulationJointReducedCoordinate*>(hinge1);
		const PxArticulationJointType::Enum artiJointType = joint1->getJointType();
		if(artiJointType != PxArticulationJointType::eREVOLUTE && artiJointType != PxArticulationJointType::eREVOLUTE_UNWRAPPED)
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxGearJoint::setHinges: passed joint must be either a revolute joint.");
	}
	else
	{
		if(type1 != PxJointConcreteType::eREVOLUTE && type1 != PxJointConcreteType::eD6)
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxGearJoint::setHinges: passed joint must be either a revolute joint or a D6 joint.");
	}

	data->hingeJoint0 = hinge0;
	data->hingeJoint1 = hinge1;
	resetError();
	markDirty();

#if PX_SUPPORT_OMNI_PVD
	const PxBase* joints[] ={ hinge0, hinge1 };
	PxU32 jointsLength = sizeof(joints);
	OMNI_PVD_SETB(PxGearJoint, hinges, static_cast<PxGearJoint&>(*this), joints, jointsLength)
#endif
		
	return true;
}

void GearJoint::getHinges(const PxBase*& hinge0, const PxBase*& hinge1) const
{
	const GearJointData* data = static_cast<const GearJointData*>(mData);
	hinge0 = data->hingeJoint0;
	hinge1 = data->hingeJoint1;
}

void GearJoint::setGearRatio(float ratio)
{
	GearJointData* data = static_cast<GearJointData*>(mData);
	data->gearRatio = ratio;
	resetError();
	markDirty();

	OMNI_PVD_SET(PxGearJoint, ratio, static_cast<PxGearJoint&>(*this), ratio)
}

float GearJoint::getGearRatio() const
{
	const GearJointData* data = static_cast<const GearJointData*>(mData);
	return data->gearRatio;
}

static float angleDiff(float angle0, float angle1)
{
	const float diff = fmodf(angle1 - angle0 + PxPi, PxTwoPi) - PxPi;
	return diff < -PxPi ? diff + PxTwoPi : diff;
}

static void getAngleAndSign(float& angle, float& sign, const PxBase* dataHingeJoint, PxRigidActor* gearActor0, PxRigidActor* gearActor1)
{
	PxRigidActor* hingeActor0;
	PxRigidActor* hingeActor1;
	const PxType type = dataHingeJoint->getConcreteType();

	if(type == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
	{
		const PxArticulationJointReducedCoordinate* artiHingeJoint = static_cast<const PxArticulationJointReducedCoordinate*>(dataHingeJoint);

		hingeActor0 = &artiHingeJoint->getParentArticulationLink();
		hingeActor1 = &artiHingeJoint->getChildArticulationLink();
		
		angle = artiHingeJoint->getJointPosition(PxArticulationAxis::eTWIST);
	}
	else
	{
		const PxJoint* hingeJoint = static_cast<const PxJoint*>(dataHingeJoint);

		hingeJoint->getActors(hingeActor0, hingeActor1);

		if(type == PxJointConcreteType::eREVOLUTE)
			angle = static_cast<const PxRevoluteJoint*>(hingeJoint)->getAngle();
		else if(type == PxJointConcreteType::eD6)
			angle = static_cast<const PxD6Joint*>(hingeJoint)->getTwistAngle();
	}

	if(gearActor0 == hingeActor0 || gearActor1 == hingeActor0)
		sign = -1.0f;
	else if(gearActor0 == hingeActor1 || gearActor1 == hingeActor1)
		sign = 1.0f;
	else
		PX_ASSERT(0);
}

void GearJoint::updateError()
{
	GearJointData* data = static_cast<GearJointData*>(mData);

	if(!data->hingeJoint0 || !data->hingeJoint1)
		return;

	PxRigidActor* gearActor0;
	PxRigidActor* gearActor1;
	getActors(gearActor0, gearActor1);

	float Angle0 = 0.0f;
	float Sign0 = 0.0f;
	getAngleAndSign(Angle0, Sign0, data->hingeJoint0, gearActor0, gearActor1);

	float Angle1 = 0.0f;
	float Sign1 = 0.0f;
	getAngleAndSign(Angle1, Sign1, data->hingeJoint1, gearActor0, gearActor1);
	Angle1 = -Angle1;

	if(!mInitDone)
	{
		mInitDone = true;
		mPersistentAngle0 = Angle0;
		mPersistentAngle1 = Angle1;
	}

	const float travelThisFrame0 = angleDiff(Angle0, mPersistentAngle0);
	const float travelThisFrame1 = angleDiff(Angle1, mPersistentAngle1);
	mVirtualAngle0 += travelThisFrame0;
	mVirtualAngle1 += travelThisFrame1;

//	printf("travelThisFrame0: %f\n", travelThisFrame0);
//	printf("travelThisFrame1: %f\n", travelThisFrame1);
//	printf("ratio: %f\n", travelThisFrame1/travelThisFrame0);
	mPersistentAngle0 = Angle0;
	mPersistentAngle1 = Angle1;

	const float error = Sign0*mVirtualAngle0*data->gearRatio - Sign1*mVirtualAngle1;
//	printf("error: %f\n", error);

	data->error = error;
	markDirty();
}

void GearJoint::resetError()
{
	mVirtualAngle0 = mVirtualAngle1 = 0.0f;
	mPersistentAngle0 = mPersistentAngle1 = 0.0f;
	mInitDone = false;
}

static const bool gVizJointFrames = true;
static const bool gVizGearAxes = false;

static void GearJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
	{
		const GearJointData& data = *reinterpret_cast<const GearJointData*>(constantBlock);

		// Visualize joint frames
		PxTransform32 cA2w, cB2w;
		joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
		if(gVizJointFrames)
			viz.visualizeJointFrames(cA2w, cB2w);

		if(gVizGearAxes)
		{
			const PxVec3 gearAxis0 = cA2w.rotate(PxVec3(1.0f, 0.0f, 0.0f)).getNormalized();
			const PxVec3 gearAxis1 = cB2w.rotate(PxVec3(1.0f, 0.0f, 0.0f)).getNormalized();
			viz.visualizeLine(body0Transform.p+gearAxis0, body0Transform.p, 0xff0000ff);
			viz.visualizeLine(body1Transform.p+gearAxis1, body1Transform.p, 0xff0000ff);
		}
	}
}

//TAG:solverprepshader
static PxU32 GearJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const GearJointData& data = *reinterpret_cast<const GearJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	cA2wOut = cB2w.p;
	cB2wOut = cB2w.p;

	const PxVec3 gearAxis0 = cA2w.q.getBasisVector0();
	const PxVec3 gearAxis1 = cB2w.q.getBasisVector0();

	Px1DConstraint& con = constraints[0];
	con.linear0 = PxVec3(0.0f);
	con.linear1 = PxVec3(0.0f);
	con.angular0 = gearAxis0*data.gearRatio;
	con.angular1 = -gearAxis1;
	con.geometricError = -data.error;
	con.minImpulse = -PX_MAX_F32;
	con.maxImpulse = PX_MAX_F32;
	con.velocityTarget = 0.f;
	con.forInternalUse = 0.f;
	con.solveHint = 0;
	con.flags = Px1DConstraintFlag::eOUTPUT_FORCE|Px1DConstraintFlag::eANGULAR_CONSTRAINT;
	con.mods.bounce.restitution = 0.0f;
	con.mods.bounce.velocityThreshold = 0.0f;
	return 1;
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gGearJointShaders = { GearJointSolverPrep, GearJointVisualize, PxConstraintFlag::eALWAYS_UPDATE };

PxConstraintSolverPrep GearJoint::getPrep()	const	{ return gGearJointShaders.solverPrep;  }

PxGearJoint* physx::PxGearJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxGearJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxGearJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxGearJointCreate: at least one actor must be dynamic");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxGearJointCreate: actors must be different");

	return createJointT<GearJoint, GearJointData>(physics, actor0, localFrame0, actor1, localFrame1, gGearJointShaders);
}

// PX_SERIALIZATION
void GearJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gGearJointShaders);

	GearJointData* data = static_cast<GearJointData*>(mData);
	context.translatePxBase(data->hingeJoint0);
	context.translatePxBase(data->hingeJoint1);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

template<>
void physx::Ext::omniPvdInitJoint<GearJoint>(GearJoint* joint)
{
	PxGearJoint& j = static_cast<PxGearJoint&>(*joint);
	OMNI_PVD_CREATE(PxGearJoint, j);
	omniPvdSetBaseJointParams(static_cast<PxJoint&>(*joint), PxJointConcreteType::eGEAR);
	OMNI_PVD_SET(PxGearJoint, ratio, j , joint->getGearRatio())
}

#endif
