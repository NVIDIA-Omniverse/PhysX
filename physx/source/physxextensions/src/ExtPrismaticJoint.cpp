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

#include "ExtPrismaticJoint.h"
#include "ExtConstraintHelper.h"

#include "omnipvd/ExtOmniPvdSetData.h"

using namespace physx;
using namespace Ext;

PrismaticJoint::PrismaticJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	PrismaticJointT(PxJointConcreteType::ePRISMATIC, actor0, localFrame0, actor1, localFrame1, "PrismaticJointData")
{
	PrismaticJointData* data = static_cast<PrismaticJointData*>(mData);

	data->limit			= PxJointLinearLimitPair(scale);
	data->jointFlags	= PxPrismaticJointFlags();
}

PxPrismaticJointFlags PrismaticJoint::getPrismaticJointFlags() const
{ 
	return data().jointFlags;		
}

void PrismaticJoint::setPrismaticJointFlags(PxPrismaticJointFlags flags)
{ 
	data().jointFlags = flags; markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, jointFlags, static_cast<PxPrismaticJoint&>(*this), flags)
}

void PrismaticJoint::setPrismaticJointFlag(PxPrismaticJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, jointFlags, static_cast<PxPrismaticJoint&>(*this), getPrismaticJointFlags())
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
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxPrismaticJoint& j = static_cast<PxPrismaticJoint&>(*this);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitLower, j, limit.lower)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitUpper, j, limit.upper)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitRestitution, j, limit.restitution)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitStiffness, j, limit.stiffness)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitDamping, j, limit.damping)

	OMNI_PVD_WRITE_SCOPE_END
#endif
}

static void PrismaticJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const PrismaticJointData& data = *reinterpret_cast<const PrismaticJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if((flags & PxConstraintVisualizationFlag::eLIMITS) && (data.jointFlags & PxPrismaticJointFlag::eLIMIT_ENABLED))
	{
		viz.visualizeLinearLimit(cA2w, cB2w, data.limit.lower);
		viz.visualizeLinearLimit(cA2w, cB2w, data.limit.upper);
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

	PxTransform32 cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	joint::applyNeighborhoodOperator(cA2w, cB2w);

	const bool limitEnabled = data.jointFlags & PxPrismaticJointFlag::eLIMIT_ENABLED;
	const PxJointLinearLimitPair& limit = data.limit;
	const bool limitIsLocked = limitEnabled && limit.lower >= limit.upper;

	const PxVec3 bOriginInA = cA2w.transformInv(cB2w.p);

	PxVec3 ra, rb, axis;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, bOriginInA, limitIsLocked ? 7ul : 6ul, 7ul, ra, rb, &axis);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	if(limitEnabled && !limitIsLocked)
	{
		const PxReal ordinate = bOriginInA.x;

		ch.linearLimit(axis, ordinate, limit.upper, limit);
		ch.linearLimit(-axis, -ordinate, -limit.lower, limit);
	}

	return ch.getCount();
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gPrismaticJointShaders = { PrismaticJointSolverPrep, PrismaticJointVisualize, PxConstraintFlag::Enum(0) };

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
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	const PxPrismaticJoint& j = static_cast<const PxPrismaticJoint&>(*this);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, position, j, getPosition())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, velocity, j, getVelocity())

	OMNI_PVD_WRITE_SCOPE_END
}

template<>
void physx::Ext::omniPvdInitJoint<PrismaticJoint>(PrismaticJoint& joint)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	PxPrismaticJoint& j = static_cast<PxPrismaticJoint&>(joint);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, j);
	omniPvdSetBaseJointParams(static_cast<PxJoint&>(joint), PxJointConcreteType::ePRISMATIC);

	PxJointLinearLimitPair limit = joint.getLimit();
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitLower, j, limit.lower)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitUpper, j, limit.upper)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitRestitution, j, limit.restitution)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitStiffness, j, limit.stiffness)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, limitDamping, j, limit.damping)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, position, j, joint.getPosition())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPrismaticJoint, velocity, j, joint.getVelocity())

	OMNI_PVD_WRITE_SCOPE_END
}

#endif
