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

#include "ExtRackAndPinionJoint.h"
#include "ExtConstraintHelper.h"
#include "extensions/PxRevoluteJoint.h"
#include "extensions/PxPrismaticJoint.h"
#include "PxArticulationJointReducedCoordinate.h"
//#include <stdio.h>

#include "omnipvd/ExtOmniPvdSetData.h"

using namespace physx;
using namespace Ext;

PX_IMPLEMENT_OUTPUT_ERROR

RackAndPinionJoint::RackAndPinionJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	RackAndPinionJointT(PxJointConcreteType::eRACK_AND_PINION, actor0, localFrame0, actor1, localFrame1, "RackAndPinionJointData")
{
	RackAndPinionJointData* data = static_cast<RackAndPinionJointData*>(mData);
	data->hingeJoint = NULL;
	data->prismaticJoint = NULL;
	data->ratio = 1.0f;
	data->px = 0.0f;
	data->vangle = 0.0f;

	resetError();
}

void RackAndPinionJoint::setRatio(float ratio)
{
	RackAndPinionJointData* data = reinterpret_cast<RackAndPinionJointData*>(mData);
	data->ratio = ratio;
	resetError();
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRackAndPinionJoint, ratio, static_cast<PxRackAndPinionJoint&>(*this), ratio)
}

float RackAndPinionJoint::getRatio() const
{
	RackAndPinionJointData* data = reinterpret_cast<RackAndPinionJointData*>(mData);
	return data->ratio;
}

bool RackAndPinionJoint::setData(PxU32 nbRackTeeth, PxU32 nbPinionTeeth, float rackLength)
{
	if(!nbRackTeeth)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setData: nbRackTeeth cannot be zero.");

	if(!nbPinionTeeth)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setData: nbPinionTeeth cannot be zero.");

	if(rackLength<=0.0f)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setData: rackLength must be positive.");

	RackAndPinionJointData* data = reinterpret_cast<RackAndPinionJointData*>(mData);
	data->ratio = (PxTwoPi*nbRackTeeth)/(rackLength*nbPinionTeeth);
	resetError();
	markDirty();
	return true;
}

bool RackAndPinionJoint::setJoints(const PxBase* hinge, const PxBase* prismatic)
{
	RackAndPinionJointData* data = static_cast<RackAndPinionJointData*>(mData);

	if(hinge)
	{
		const PxType type0 = hinge->getConcreteType();
		if(type0 == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
			const PxArticulationJointReducedCoordinate* joint0 = static_cast<const PxArticulationJointReducedCoordinate*>(hinge);
			const PxArticulationJointType::Enum artiJointType0 = joint0->getJointType();
			if(artiJointType0 != PxArticulationJointType::eREVOLUTE && artiJointType0 != PxArticulationJointType::eREVOLUTE_UNWRAPPED)
				return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setJoints: passed joint must be a revolute joint.");
		}
		else
		{
			if(type0 != PxJointConcreteType::eREVOLUTE && type0 != PxJointConcreteType::eD6)
				return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setJoints: passed hinge joint must be either a revolute joint or a D6 joint.");
		}
	}

	if(prismatic)
	{
		const PxType type1 = prismatic->getConcreteType();
		if(type1 == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
			const PxArticulationJointReducedCoordinate* joint1 = static_cast<const PxArticulationJointReducedCoordinate*>(prismatic);
			const PxArticulationJointType::Enum artiJointType1 = joint1->getJointType();
			if(artiJointType1 != PxArticulationJointType::ePRISMATIC)
				return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setJoints: passed joint must be a prismatic joint.");
		}
		else
		{
			if(type1 != PxJointConcreteType::ePRISMATIC && type1 != PxJointConcreteType::eD6)
				return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRackAndPinionJoint::setJoints: passed prismatic joint must be either a prismatic joint or a D6 joint.");
		}
	}

	data->hingeJoint = hinge;
	data->prismaticJoint = prismatic;
	resetError();
	markDirty();

#if PX_SUPPORT_OMNI_PVD
	const PxBase* joints[] = { hinge, prismatic };
	PxU32 jointCount = sizeof(joints) / sizeof(joints[0]);
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxRackAndPinionJoint, joints, static_cast<PxRackAndPinionJoint&>(*this), joints, jointCount)
#endif
		
	return true;
}

void RackAndPinionJoint::getJoints(const PxBase*& hinge, const PxBase*& prismatic) const
{
	const RackAndPinionJointData* data = static_cast<const RackAndPinionJointData*>(mData);
	hinge = data->hingeJoint;
	prismatic = data->prismaticJoint;
}

static float angleDiff(float angle0, float angle1)
{
	const float diff = fmodf( angle1 - angle0 + PxPi, PxTwoPi) - PxPi;
	return diff < -PxPi ? diff + PxTwoPi : diff;
}

void RackAndPinionJoint::updateError()
{
	RackAndPinionJointData* data = static_cast<RackAndPinionJointData*>(mData);

	if(!data->hingeJoint || !data->prismaticJoint)
		return;

	PxRigidActor* rackActor0;
	PxRigidActor* rackActor1;
	getActors(rackActor0, rackActor1);

	float Angle0 = 0.0f;
	float Sign0 = 0.0f;
	{
		PxRigidActor* hingeActor0;
		PxRigidActor* hingeActor1;

		const PxType type = data->hingeJoint->getConcreteType();
		if(type == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
			const PxArticulationJointReducedCoordinate* artiHingeJoint = static_cast<const PxArticulationJointReducedCoordinate*>(data->hingeJoint);

			hingeActor0 = &artiHingeJoint->getParentArticulationLink();
			hingeActor1 = &artiHingeJoint->getChildArticulationLink();

			Angle0 = artiHingeJoint->getJointPosition(PxArticulationAxis::eTWIST);
		}
		else
		{
			const PxJoint* hingeJoint = static_cast<const PxJoint*>(data->hingeJoint);

			hingeJoint->getActors(hingeActor0, hingeActor1);

			if(type == PxJointConcreteType::eREVOLUTE)
				Angle0 = static_cast<const PxRevoluteJoint*>(hingeJoint)->getAngle();
			else if (type == PxJointConcreteType::eD6)
				Angle0 = static_cast<const PxD6Joint*>(hingeJoint)->getTwistAngle();
		}

		if(rackActor0 == hingeActor0 || rackActor1 == hingeActor0)
			Sign0 = -1.0f;
		else if (rackActor0 == hingeActor1 || rackActor1 == hingeActor1)
			Sign0 = 1.0f;
		else
			PX_ASSERT(0);
	}

	if(!mInitDone)
	{
		mInitDone = true;
		mPersistentAngle0 = Angle0;
	}

	const float travelThisFrame0 = angleDiff(Angle0, mPersistentAngle0);
	mVirtualAngle0 += travelThisFrame0;
//	printf("mVirtualAngle0: %f\n", mVirtualAngle0);

	mPersistentAngle0 = Angle0;

	float px = 0.0f;
	float Sign1 = 0.0f;
	{
		PxRigidActor* prismaticActor0;
		PxRigidActor* prismaticActor1;

		const PxType type = data->prismaticJoint->getConcreteType();
		if(type == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
			const PxArticulationJointReducedCoordinate* artiPrismaticJoint = static_cast<const PxArticulationJointReducedCoordinate*>(data->prismaticJoint);

			prismaticActor0 = &artiPrismaticJoint->getParentArticulationLink();
			prismaticActor1 = &artiPrismaticJoint->getChildArticulationLink();

			px = artiPrismaticJoint->getJointPosition(PxArticulationAxis::eX);
		}
		else
		{
			const PxJoint* prismaticJoint = static_cast<const PxJoint*>(data->prismaticJoint);

			prismaticJoint->getActors(prismaticActor0, prismaticActor1);

			if(type==PxJointConcreteType::ePRISMATIC)
				px = static_cast<const PxPrismaticJoint*>(prismaticJoint)->getPosition();
			else if(type==PxJointConcreteType::eD6)
				px = prismaticJoint->getRelativeTransform().p.x;
		}

		if(rackActor0 == prismaticActor0 || rackActor1 == prismaticActor0)
			Sign1 = -1.0f;
		else if(rackActor0 == prismaticActor1 || rackActor1 == prismaticActor1)
			Sign1 = 1.0f;
		else
			PX_ASSERT(0);
	}

//	printf("px: %f\n", px);
	data->px = Sign1*px;
	data->vangle = Sign0*mVirtualAngle0;
	markDirty();
}

void RackAndPinionJoint::resetError()
{
	mVirtualAngle0 = 0.0f;
	mPersistentAngle0 = 0.0f;
	mInitDone = false;
}

static void RackAndPinionJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
	{
		const RackAndPinionJointData& data = *reinterpret_cast<const RackAndPinionJointData*>(constantBlock);

		// Visualize joint frames
		PxTransform32 cA2w, cB2w;
		joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
		viz.visualizeJointFrames(cA2w, cB2w);
	}

	if(0)
	{
		const RackAndPinionJointData& data = *reinterpret_cast<const RackAndPinionJointData*>(constantBlock);

		if(0)
		{
			PxTransform32 cA2w, cB2w;
			joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);

			const PxVec3 gearAxis0 = cA2w.q.getBasisVector0();
			const PxVec3 rackPrismaticAxis = cB2w.q.getBasisVector0();
			viz.visualizeLine(cA2w.p, cA2w.p + gearAxis0, 0xff0000ff);
			viz.visualizeLine(cB2w.p, cB2w.p + rackPrismaticAxis, 0xff0000ff);
		}
	}
}

//TAG:solverprepshader
static PxU32 RackAndPinionJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const RackAndPinionJointData& data = *reinterpret_cast<const RackAndPinionJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	cA2wOut = cB2w.p;
	cB2wOut = cB2w.p;

	const PxVec3 gearAxis = cA2w.q.getBasisVector0();
	const PxVec3 rackPrismaticAxis = cB2w.q.getBasisVector0();

	// PT: this optional bit of code tries to fix the ratio for cases where the "same" rack is moved e.g. above or below a gear.
	// In that case the rack would move in one direction or another depending on its position compared to the gear, and to avoid
	// having to use a negative ratio in one of these cases this code tries to compute the proper sign and handle both cases the
	// same way from the user's perspective. This created unexpected issues in ill-defined cases where e.g. the gear and the rack
	// completely overlap, and we end up with a +0 or -0 for "dp" in the code below. So now this code disables itself in these
	// cases but it would probably be better to disable it entirely. We don't do it though since it could break existing scenes.
	// We might want to revisit these decisions at some point.
	float Coeff = 1.0f;
	const float epsilon = 0.001f;
	const PxVec3 delta = cB2w.p - cA2w.p;
	if(delta.magnitudeSquared()>epsilon*epsilon)
	{
		const PxVec3 velocity = gearAxis.cross(delta);
		if(velocity.magnitudeSquared()>epsilon*epsilon)
		{
			const float dp = velocity.dot(rackPrismaticAxis);
			Coeff = fabsf(dp)>epsilon ? PxSign(dp) : 1.0f;
		}
	}

	Px1DConstraint& con = constraints[0];
	con.linear0 = PxVec3(0.0f);
	con.linear1 = rackPrismaticAxis * data.ratio*Coeff;
	con.angular0 = gearAxis;
	con.angular1 = PxVec3(0.0f);
	con.geometricError = -Coeff*data.px*data.ratio - data.vangle;
	con.minImpulse = -PX_MAX_F32;
	con.maxImpulse = PX_MAX_F32;
	con.velocityTarget = 0.f;
	con.solveHint = 0;
	con.flags = Px1DConstraintFlag::eOUTPUT_FORCE|Px1DConstraintFlag::eANGULAR_CONSTRAINT;
	con.mods.bounce.restitution = 0.0f;
	con.mods.bounce.velocityThreshold = 0.0f;
	return 1;
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gRackAndPinionJointShaders = { RackAndPinionJointSolverPrep, RackAndPinionJointVisualize, PxConstraintFlag::eALWAYS_UPDATE };

PxConstraintSolverPrep RackAndPinionJoint::getPrep()	const	{ return gRackAndPinionJointShaders.solverPrep;  }

PxRackAndPinionJoint* physx::PxRackAndPinionJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxRackAndPinionJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxRackAndPinionJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxRackAndPinionJointCreate: at least one actor must be dynamic");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxRackAndPinionJointCreate: actors must be different");

	return createJointT<RackAndPinionJoint, RackAndPinionJointData>(physics, actor0, localFrame0, actor1, localFrame1, gRackAndPinionJointShaders);
}

// PX_SERIALIZATION
void RackAndPinionJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gRackAndPinionJointShaders);

	RackAndPinionJointData* data = static_cast<RackAndPinionJointData*>(mData);
	context.translatePxBase(data->hingeJoint);
	context.translatePxBase(data->prismaticJoint);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

template<>
void physx::Ext::omniPvdInitJoint<RackAndPinionJoint>(RackAndPinionJoint& joint)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxRackAndPinionJoint& j = static_cast<PxRackAndPinionJoint&>(joint);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRackAndPinionJoint, j);
	omniPvdSetBaseJointParams(static_cast<PxJoint&>(joint), PxJointConcreteType::eRACK_AND_PINION);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRackAndPinionJoint, ratio, j, joint.getRatio())

	OMNI_PVD_WRITE_SCOPE_END
}

#endif
