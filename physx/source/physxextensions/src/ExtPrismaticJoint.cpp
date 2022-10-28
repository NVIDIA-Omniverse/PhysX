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

#include "ExtPrismaticJoint.h"
#include "ExtConstraintHelper.h"

using namespace physx;
using namespace Ext;

PrismaticJoint::PrismaticJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	PrismaticJointT(PxJointConcreteType::ePRISMATIC, actor0, localFrame0, actor1, localFrame1, "PrismaticJointData")
{
	PrismaticJointData* data = static_cast<PrismaticJointData*>(mData);

	data->limit							= PxJointLinearLimitPair(scale);
	data->projectionLinearTolerance		= 1e10f;
	data->projectionAngularTolerance	= PxPi;
	data->jointFlags					= PxPrismaticJointFlags();
}

void PrismaticJoint::setProjectionAngularTolerance(PxReal tolerance)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0 && tolerance <= PxPi, "PxPrismaticJoint::setProjectionAngularTolerance: invalid parameter");
	data().projectionAngularTolerance = tolerance;	
	markDirty();	
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(joint, prismaticProjectionAngularTolerance, static_cast<PxJoint&>(*this), tolerance)
#endif
}

PxReal PrismaticJoint::getProjectionAngularTolerance() const
{ 
	return data().projectionAngularTolerance; 
}

void PrismaticJoint::setProjectionLinearTolerance(PxReal tolerance) 
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0, "PxPrismaticJoint::setProjectionLinearTolerance: invalid parameter");
	data().projectionLinearTolerance = tolerance;	
	markDirty(); 
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(joint, prismaticProjectionLinearTolerance, static_cast<PxJoint&>(*this), tolerance)
#endif
}

PxReal PrismaticJoint::getProjectionLinearTolerance() const	
{ 
	return data().projectionLinearTolerance;		
}

PxPrismaticJointFlags PrismaticJoint::getPrismaticJointFlags(void) const
{ 
	return data().jointFlags;		
}

void PrismaticJoint::setPrismaticJointFlags(PxPrismaticJointFlags flags)
{ 
	data().jointFlags = flags; markDirty();	
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(joint, prismaticJointFlags, static_cast<PxJoint&>(*this), flags)
#endif
}

void PrismaticJoint::setPrismaticJointFlag(PxPrismaticJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(joint, prismaticJointFlags, static_cast<PxJoint&>(*this), getPrismaticJointFlags())
#endif
}

PxJointLinearLimitPair PrismaticJoint::getLimit() const
{ 
	return data().limit;	
}

void PrismaticJoint::setLimit(const PxJointLinearLimitPair& limit)
{ 
	PX_CHECK_AND_RETURN(limit.isValid(), "PxPrismaticJoint::setLimit: invalid parameter");
	data().limit = limit;
	markDirty();
#if PX_SUPPORT_OMNI_PVD
	PxJoint& j = static_cast<PxJoint&>(*this);
	OMNI_PVD_SET(joint, prismaticLimitLower, j, limit.lower)
	OMNI_PVD_SET(joint, prismaticLimitUpper, j, limit.upper)
	OMNI_PVD_SET(joint, prismaticLimitRestitution, j, limit.restitution)
	OMNI_PVD_SET(joint, prismaticLimitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET(joint, prismaticLimitStiffness, j, limit.stiffness)
	OMNI_PVD_SET(joint, prismaticLimitDamping, j, limit.damping)
	OMNI_PVD_SET(joint, prismaticLimitContactDistance, j, limit.contactDistance_deprecated)
#endif
}

static void PrismaticJointProject(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
	const PrismaticJointData& data = *reinterpret_cast<const PrismaticJointData*>(constantBlock);

	PxTransform cA2w, cB2w, cB2cA, projected;
	joint::computeDerived(data, bodyAToWorld, bodyBToWorld, cA2w, cB2w, cB2cA);

	const PxVec3 v(0.0f, cB2cA.p.y, cB2cA.p.z);
	bool linearTrunc, angularTrunc;
	projected.p = joint::truncateLinear(v, data.projectionLinearTolerance, linearTrunc);
	projected.q = joint::truncateAngular(cB2cA.q, PxSin(data.projectionAngularTolerance/2), PxCos(data.projectionAngularTolerance/2), angularTrunc);
	
	if(linearTrunc || angularTrunc)
	{
		projected.p.x = cB2cA.p.x;
		joint::projectTransforms(bodyAToWorld, bodyBToWorld, cA2w, cB2w, projected, data, projectToA);
	}
}

static void PrismaticJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const PrismaticJointData& data = *reinterpret_cast<const PrismaticJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if((flags & PxConstraintVisualizationFlag::eLIMITS) && (data.jointFlags & PxPrismaticJointFlag::eLIMIT_ENABLED))
	{
		const PxVec3 bOriginInA = cA2w.transformInv(cB2w.p);
		const PxReal ordinate = bOriginInA.x;

		const PxReal pad = data.limit.isSoft() ? 0.0f : data.limit.contactDistance_deprecated;
		viz.visualizeLinearLimit(cA2w, cB2w, data.limit.lower, ordinate < data.limit.lower + pad);
		viz.visualizeLinearLimit(cA2w, cB2w, data.limit.upper, ordinate > data.limit.upper - pad);
	}
}

//TAG:solverprepshader
static PxU32 PrismaticJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const PrismaticJointData& data = *reinterpret_cast<const PrismaticJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	if (cA2w.q.dot(cB2w.q)<0.0f)	// minimum dist quat (equiv to flipping cB2bB.q, which we don't use anywhere)
		cB2w.q = -cB2w.q;

	const bool limitEnabled = data.jointFlags & PxPrismaticJointFlag::eLIMIT_ENABLED;
	const PxJointLinearLimitPair& limit = data.limit;
	const bool limitIsLocked = limitEnabled && limit.lower >= limit.upper;

	const PxVec3 bOriginInA = cA2w.transformInv(cB2w.p);
	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, bOriginInA, limitIsLocked ? 7ul : 6ul, 7ul, ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	if(limitEnabled && !limitIsLocked)
	{
		const PxVec3 axis = cA2w.q.getBasisVector0();	// PT: TODO: this has already been computed as part of the quat-to-matrix transform within prepareLockedAxes

		const PxReal ordinate = bOriginInA.x;

		ch.linearLimit(axis, ordinate, limit.upper, limit);
		ch.linearLimit(-axis, -ordinate, -limit.lower, limit);
	}

	return ch.getCount();
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gPrismaticJointShaders = { PrismaticJointSolverPrep, PrismaticJointProject, PrismaticJointVisualize, PxConstraintFlag::Enum(0) };

PxConstraintSolverPrep PrismaticJoint::getPrep()	const	{ return gPrismaticJointShaders.solverPrep; }

PxPrismaticJoint* physx::PxPrismaticJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxPrismaticJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxPrismaticJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxPrismaticJointCreate: at least one actor must be dynamic");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxPrismaticJointCreate: actors must be different");

	return createJointT<PrismaticJoint, PrismaticJointData>(physics, actor0, localFrame0, actor1, localFrame1, gPrismaticJointShaders);
}

// PX_SERIALIZATION
void PrismaticJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gPrismaticJointShaders);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

void PrismaticJoint::updateOmniPvdProperties() const
{
	const PxJoint& j = static_cast<const PxJoint&>(*this);
	OMNI_PVD_SET(joint, prismaticPosition, j, getPosition())
	OMNI_PVD_SET(joint, prismaticVelocity, j, getVelocity())
}

template<>
void physx::Ext::omniPvdInitJoint<PrismaticJoint>(PrismaticJoint* joint)
{
	PxJoint& j = static_cast<PxJoint&>(*joint);
	OMNI_PVD_SET(joint, type, j, PxJointConcreteType::ePRISMATIC)
	OMNI_PVD_SET(joint, prismaticProjectionLinearTolerance, j, joint->getProjectionLinearTolerance())
	OMNI_PVD_SET(joint, prismaticProjectionAngularTolerance, j, joint->getProjectionAngularTolerance())

	PxJointLinearLimitPair limit = joint->getLimit();
	OMNI_PVD_SET(joint, prismaticLimitLower, j, limit.lower)
	OMNI_PVD_SET(joint, prismaticLimitUpper, j, limit.upper)
	OMNI_PVD_SET(joint, prismaticLimitRestitution, j, limit.restitution)
	OMNI_PVD_SET(joint, prismaticLimitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET(joint, prismaticLimitStiffness, j, limit.stiffness)
	OMNI_PVD_SET(joint, prismaticLimitDamping, j, limit.damping)
	OMNI_PVD_SET(joint, prismaticLimitContactDistance, j, limit.contactDistance_deprecated)

	OMNI_PVD_SET(joint, prismaticPosition, j, joint->getPosition())
	OMNI_PVD_SET(joint, prismaticVelocity, j, joint->getVelocity())
}

#endif
