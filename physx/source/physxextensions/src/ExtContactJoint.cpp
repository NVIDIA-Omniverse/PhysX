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

#include "ExtContactJoint.h"

#include "omnipvd/ExtOmniPvdSetData.h"

using namespace physx;
using namespace Ext;

ContactJoint::ContactJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	ContactJointT(PxJointConcreteType::eCONTACT, actor0, localFrame0, actor1, localFrame1, "ContactJointData")
{
	PX_UNUSED(scale);

	ContactJointData* data = static_cast<ContactJointData*>(mData);

	data->contact = PxVec3(0.f);
	data->normal = PxVec3(0.f);
	data->penetration = 0.f;
	data->restitution = 0.f;
	data->bounceThreshold = 0.f;
}

PxVec3 ContactJoint::getContact() const
{
	return data().contact;
}

void ContactJoint::setContact(const PxVec3& contact)
{
	PX_CHECK_AND_RETURN(contact.isFinite(), "PxContactJoint::setContact: invalid parameter");
	data().contact = contact;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, point, static_cast<PxContactJoint&>(*this), getContact())
}

PxVec3 ContactJoint::getContactNormal() const
{
	return data().normal;
}

void ContactJoint::setContactNormal(const PxVec3& normal)
{
	PX_CHECK_AND_RETURN(normal.isFinite(), "PxContactJoint::setContactNormal: invalid parameter");
	data().normal = normal;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, normal, static_cast<PxContactJoint&>(*this), getContactNormal())
}

PxReal ContactJoint::getPenetration() const
{
	return data().penetration;
}

void ContactJoint::setPenetration(PxReal penetration)
{
	PX_CHECK_AND_RETURN(PxIsFinite(penetration), "ContactJoint::setPenetration: invalid parameter");
	data().penetration = penetration;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, penetration, static_cast<PxContactJoint&>(*this), getPenetration())
}

PxReal ContactJoint::getRestitution() const
{
	return data().restitution;
}

void ContactJoint::setRestitution(const PxReal restitution)
{
	PX_CHECK_AND_RETURN(PxIsFinite(restitution) && restitution >= 0.f && restitution <= 1.f, "ContactJoint::setRestitution: invalid parameter");
	data().restitution = restitution;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, restitution, static_cast<PxContactJoint&>(*this), getRestitution())
}

PxReal ContactJoint::getBounceThreshold() const
{
	return data().bounceThreshold;
}

void ContactJoint::setBounceThreshold(const PxReal bounceThreshold)
{
	PX_CHECK_AND_RETURN(PxIsFinite(bounceThreshold) && bounceThreshold > 0.f, "ContactJoint::setBounceThreshold: invalid parameter");
	data().bounceThreshold = bounceThreshold;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, bounceThreshold, static_cast<PxContactJoint&>(*this), getBounceThreshold())
}

void ContactJoint::computeJacobians(PxJacobianRow* jacobian) const
{
	const PxVec3 cp = data().contact;
	const PxVec3 normal = data().normal;

	PxRigidActor* actor0, *actor1;
	this->getActors(actor0, actor1);

	PxVec3 raXn(0.f), rbXn(0.f);

	if (actor0 && actor0->is<PxRigidBody>())
	{
		PxRigidBody* dyn = actor0->is<PxRigidBody>();
		PxTransform cmassPose = dyn->getGlobalPose() * dyn->getCMassLocalPose();
		raXn = (cp - cmassPose.p).cross(normal);
	}

	if (actor1 && actor1->is<PxRigidBody>())
	{
		PxRigidBody* dyn = actor1->is<PxRigidBody>();
		PxTransform cmassPose = dyn->getGlobalPose() * dyn->getCMassLocalPose();
		rbXn = (cp - cmassPose.p).cross(normal);
	}

	jacobian->linear0  = normal;
	jacobian->angular0 = raXn;
	jacobian->linear1 = -normal;
	jacobian->angular1 = -rbXn;
	
}
PxU32 ContactJoint::getNbJacobianRows() const
{
	return 1;
}

static void ContactJointVisualize(PxConstraintVisualizer& /*viz*/, const void* /*constantBlock*/, const PxTransform& /*body0Transform*/, const PxTransform& /*body1Transform*/, PxU32 /*flags*/)
{
	//TODO
}

//TAG:solverprepshader
static PxU32 ContactJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& /*invMassScale*/,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const ContactJointData& data = *reinterpret_cast<const ContactJointData*>(constantBlock);

	const PxVec3& contact = data.contact;
	const PxVec3& normal = data.normal;

	cA2wOut = contact;
	cB2wOut = contact;

	const PxVec3 ra = contact - bA2w.p;
	const PxVec3 rb = contact - bB2w.p;

	body0WorldOffset = PxVec3(0.f);

	Px1DConstraint& con = constraints[0];
	con.linear0 = normal;
	con.linear1 = normal;
	con.angular0 = ra.cross(normal);
	con.angular1 = rb.cross(normal);

	con.geometricError = data.penetration;
	con.minImpulse = 0.f;
	con.maxImpulse = PX_MAX_F32;

	con.velocityTarget = 0.f;
	con.solveHint = 0;
	con.flags = Px1DConstraintFlag::eOUTPUT_FORCE;
	con.mods.bounce.restitution = data.restitution;
	con.mods.bounce.velocityThreshold = data.bounceThreshold;

	return 1;
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gContactJointShaders = { ContactJointSolverPrep, ContactJointVisualize, PxConstraintFlag::Enum(0) };

PxConstraintSolverPrep ContactJoint::getPrep()	const	{ return gContactJointShaders.solverPrep; }

PxContactJoint* physx::PxContactJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxContactJointCreate: local frame 0 is not a valid transform");
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxContactJointCreate: local frame 1 is not a valid transform");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxContactJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxContactJointCreate: at least one actor must be dynamic");

	return createJointT<ContactJoint, ContactJointData>(physics, actor0, localFrame0, actor1, localFrame1, gContactJointShaders);
}

// PX_SERIALIZATION
void ContactJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gContactJointShaders);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

template<>
void physx::Ext::omniPvdInitJoint<ContactJoint>(ContactJoint& joint)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxContactJoint& j = static_cast<PxContactJoint&>(joint);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, j);
	omniPvdSetBaseJointParams(static_cast<PxJoint&>(joint), PxJointConcreteType::eCONTACT);
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, point, j, joint.getContact())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, normal, j, joint.getContactNormal())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, penetration, j, joint.getPenetration())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, restitution, j, joint.getRestitution())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxContactJoint, bounceThreshold, j, joint.getBounceThreshold())

	OMNI_PVD_WRITE_SCOPE_END
}

#endif
