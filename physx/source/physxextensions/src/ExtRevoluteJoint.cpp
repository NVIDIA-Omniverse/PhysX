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

#include "ExtRevoluteJoint.h"
#include "ExtConstraintHelper.h"

using namespace physx;
using namespace Ext;

RevoluteJoint::RevoluteJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0,  PxRigidActor* actor1, const PxTransform& localFrame1) :
	RevoluteJointT(PxJointConcreteType::eREVOLUTE, actor0, localFrame0, actor1, localFrame1, "RevoluteJointData")
{
	RevoluteJointData* data = static_cast<RevoluteJointData*>(mData);

	data->driveForceLimit	= PX_MAX_F32;
	data->driveVelocity		= 0.0f;
	data->driveGearRatio	= 1.0f;
	data->limit				= PxJointAngularLimitPair(-PxPi/2, PxPi/2);
	data->jointFlags		= PxRevoluteJointFlags();
}

PxReal RevoluteJoint::getAngle() const
{
	return getTwistAngle_Internal();
}

PxReal RevoluteJoint::getVelocity() const
{
	return getRelativeAngularVelocity().magnitude();
}

PxJointAngularLimitPair RevoluteJoint::getLimit()	const
{ 
	return data().limit;	
}

void RevoluteJoint::setLimit(const PxJointAngularLimitPair& limit)
{ 
	PX_CHECK_AND_RETURN(limit.isValid(), "PxRevoluteJoint::setLimit: limit invalid");
	PX_CHECK_AND_RETURN(limit.lower>-PxTwoPi && limit.upper<PxTwoPi , "PxRevoluteJoint::twist limit must be strictly between -2*PI and 2*PI");

	data().limit = limit; 
	markDirty();	

#if PX_SUPPORT_OMNI_PVD
	PxRevoluteJoint& j = static_cast<PxRevoluteJoint&>(*this);
	OMNI_PVD_SET(PxRevoluteJoint, limitLower, j, limit.lower)
	OMNI_PVD_SET(PxRevoluteJoint, limitUpper, j, limit.upper)
	OMNI_PVD_SET(PxRevoluteJoint, limitRestitution, j, limit.restitution)
	OMNI_PVD_SET(PxRevoluteJoint, limitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET(PxRevoluteJoint, limitStiffness, j, limit.stiffness)
	OMNI_PVD_SET(PxRevoluteJoint, limitDamping, j, limit.damping)
#endif
}

PxReal RevoluteJoint::getDriveVelocity() const
{ 
	return data().driveVelocity;
}

void RevoluteJoint::setDriveVelocity(PxReal velocity, bool autowake)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(velocity), "PxRevoluteJoint::setDriveVelocity: invalid parameter");
	data().driveVelocity = velocity; 
	if(autowake)
		wakeUpActors();
	markDirty();

	OMNI_PVD_SET(PxRevoluteJoint, driveVelocity, static_cast<PxRevoluteJoint&>(*this), velocity)
}

PxReal RevoluteJoint::getDriveForceLimit() const
{ 
	return data().driveForceLimit;	
}

void RevoluteJoint::setDriveForceLimit(PxReal forceLimit)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(forceLimit), "PxRevoluteJoint::setDriveForceLimit: invalid parameter");
	data().driveForceLimit = forceLimit; 
	markDirty();

	OMNI_PVD_SET(PxRevoluteJoint, driveForceLimit, static_cast<PxRevoluteJoint&>(*this), forceLimit)
}

PxReal RevoluteJoint::getDriveGearRatio() const
{ 
	return data().driveGearRatio;	
}

void RevoluteJoint::setDriveGearRatio(PxReal gearRatio)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(gearRatio) && gearRatio>0, "PxRevoluteJoint::setDriveGearRatio: invalid parameter");
	data().driveGearRatio = gearRatio; 
	markDirty();

	OMNI_PVD_SET(PxRevoluteJoint, driveGearRatio, static_cast<PxRevoluteJoint&>(*this), gearRatio)
}

PxRevoluteJointFlags RevoluteJoint::getRevoluteJointFlags(void)	const
{ 
	return data().jointFlags; 
}

void RevoluteJoint::setRevoluteJointFlags(PxRevoluteJointFlags flags)
{ 
	data().jointFlags = flags;

	OMNI_PVD_SET(PxRevoluteJoint, jointFlags, static_cast<PxRevoluteJoint&>(*this), flags)
}

void RevoluteJoint::setRevoluteJointFlag(PxRevoluteJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();

	OMNI_PVD_SET(PxRevoluteJoint, jointFlags, static_cast<PxRevoluteJoint&>(*this), getRevoluteJointFlags())
}

static PxQuat computeTwist(const PxTransform& cA2w, const PxTransform& cB2w)
{
	// PT: following code is the same as this part of the "getAngle" function:
	//	const PxQuat q = getRelativeTransform().q;
	//	PxQuat swing, twist;
	//	PxSeparateSwingTwist(q, swing, twist);
	// But it's done a little bit more efficiently since we don't need the swing quat.

	// PT: rotation part of "const PxTransform cB2cA = cA2w.transformInv(cB2w);"
	const PxQuat cB2cAq = cA2w.q.getConjugate() * cB2w.q;

	// PT: twist part of "PxSeparateSwingTwist(cB2cAq,swing,twist)" (more or less)
	return PxQuat(cB2cAq.x, 0.0f, 0.0f, cB2cAq.w);
}

// PT: this version is similar to the "getAngle" function, but the twist is computed slightly differently.
static PX_FORCE_INLINE PxReal computePhi(const PxTransform& cA2w, const PxTransform& cB2w)
{
	PxQuat twist = computeTwist(cA2w, cB2w);
	twist.normalize();

	PxReal angle = twist.getAngle();
	if(twist.x<0.0f)
		angle = -angle;
	return angle;
}

static void RevoluteJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const RevoluteJointData& data = *reinterpret_cast<const RevoluteJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if((data.jointFlags & PxRevoluteJointFlag::eLIMIT_ENABLED) && (flags & PxConstraintVisualizationFlag::eLIMITS))
		viz.visualizeAngularLimit(cA2w, data.limit.lower, data.limit.upper);
}

//TAG:solverprepshader
static PxU32 RevoluteJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool useExtendedLimits,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const RevoluteJointData& data = *reinterpret_cast<const RevoluteJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	const PxJointAngularLimitPair& limit = data.limit;

	const bool limitEnabled = data.jointFlags & PxRevoluteJointFlag::eLIMIT_ENABLED;
	const bool limitIsLocked = limitEnabled && limit.lower >= limit.upper;

	// PT: it is a mistake to use the neighborhood operator since it
	// prevents us from using the quat's double-cover feature.
	if(!useExtendedLimits)
		joint::applyNeighborhoodOperator(cA2w, cB2w);

	PxVec3 ra, rb, axis;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, PxU32(limitIsLocked ? 7 : 6), ra, rb, &axis);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	if(limitIsLocked)
		return ch.getCount();

	if(data.jointFlags & PxRevoluteJointFlag::eDRIVE_ENABLED)
	{
		Px1DConstraint* c = ch.getConstraintRow();

		c->solveHint		= PxConstraintSolveHint::eNONE;
		c->linear0			= PxVec3(0.0f);
		c->angular0			= -axis;
		c->linear1			= PxVec3(0.0f);
		c->angular1			= -axis * data.driveGearRatio;
		c->velocityTarget	= data.driveVelocity;
		c->minImpulse		= -data.driveForceLimit;
		c->maxImpulse		= data.driveForceLimit;
		c->flags |= Px1DConstraintFlag::eANGULAR_CONSTRAINT;
		if(data.jointFlags & PxRevoluteJointFlag::eDRIVE_FREESPIN)
		{
			if(data.driveVelocity > 0.0f)
				c->minImpulse = 0.0f;
			if(data.driveVelocity < 0.0f)
				c->maxImpulse = 0.0f;
		}
		c->flags |= Px1DConstraintFlag::eHAS_DRIVE_LIMIT;
	}

	if(limitEnabled)
	{
		const PxReal phi = computePhi(cA2w, cB2w);
		ch.anglePair(phi, data.limit.lower, data.limit.upper, axis, limit);
	}

	return ch.getCount();
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gRevoluteJointShaders = { RevoluteJointSolverPrep, RevoluteJointVisualize, PxConstraintFlag::Enum(0) };

PxConstraintSolverPrep RevoluteJoint::getPrep()	const	{ return gRevoluteJointShaders.solverPrep; }

// PT: for tests / benchmarks
PxConstraintSolverPrep getRevoluteJointPrep()	{ return gRevoluteJointShaders.solverPrep; }

PxRevoluteJoint* physx::PxRevoluteJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxRevoluteJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxRevoluteJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxRevoluteJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxRevoluteJointCreate: at least one actor must be dynamic");

	return createJointT<RevoluteJoint, RevoluteJointData>(physics, actor0, localFrame0, actor1, localFrame1, gRevoluteJointShaders);
}

// PX_SERIALIZATION
void RevoluteJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gRevoluteJointShaders);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

void RevoluteJoint::updateOmniPvdProperties() const
{
	const PxRevoluteJoint& j = static_cast<const PxRevoluteJoint&>(*this);
	OMNI_PVD_SET(PxRevoluteJoint, angle, j, getAngle())
	OMNI_PVD_SET(PxRevoluteJoint, velocity, j, getVelocity())
}

template<>
void physx::Ext::omniPvdInitJoint<RevoluteJoint>(RevoluteJoint* joint)
{
	PxRevoluteJoint& j = static_cast<PxRevoluteJoint&>(*joint);
	OMNI_PVD_CREATE(PxRevoluteJoint, j);
	omniPvdSetBaseJointParams(static_cast<PxJoint&>(*joint), PxJointConcreteType::eREVOLUTE);
	
	PxJointAngularLimitPair limit = joint->getLimit();
	OMNI_PVD_SET(PxRevoluteJoint, limitLower, j, limit.lower)
	OMNI_PVD_SET(PxRevoluteJoint, limitUpper, j, limit.upper)
	OMNI_PVD_SET(PxRevoluteJoint, limitRestitution, j, limit.restitution)
	OMNI_PVD_SET(PxRevoluteJoint, limitBounceThreshold, j, limit.bounceThreshold)
	OMNI_PVD_SET(PxRevoluteJoint, limitStiffness, j, limit.stiffness)
	OMNI_PVD_SET(PxRevoluteJoint, limitDamping, j, limit.damping)

	OMNI_PVD_SET(PxRevoluteJoint, driveVelocity, j, joint->getDriveVelocity())
	OMNI_PVD_SET(PxRevoluteJoint, driveForceLimit, j, joint->getDriveForceLimit())
	OMNI_PVD_SET(PxRevoluteJoint, driveGearRatio, j, joint->getDriveGearRatio())
	OMNI_PVD_SET(PxRevoluteJoint, jointFlags, j, joint->getRevoluteJointFlags())

	OMNI_PVD_SET(PxRevoluteJoint, angle, j, joint->getAngle())
	OMNI_PVD_SET(PxRevoluteJoint, velocity, j, joint->getVelocity())
}

#endif
