// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScConstraintCore.h"
#include "ScPhysics.h"
#include "ScConstraintSim.h"

using namespace physx;

Sc::ConstraintCore::ConstraintCore(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize) :
	mFlags					(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES),
	mIsDirty				(1),
	mAppliedForce			(PxVec3(0.0f)),
	mAppliedTorque			(PxVec3(0.0f)),
	mConnector				(&connector),
	mSolverPrep				(shaders.solverPrep),
	mVisualize				(shaders.visualize),
	mDataSize				(dataSize),
	mLinearBreakForce		(PX_MAX_F32),
	mAngularBreakForce		(PX_MAX_F32),
	mMinResponseThreshold	(0.0f),
	mSim					(NULL)
{
}

void Sc::ConstraintCore::setFlags(PxConstraintFlags flags)
{
	PxConstraintFlags old = mFlags;
	flags = flags | (old & PxConstraintFlag::eGPU_COMPATIBLE);  // eGPU_COMPATIBLE is for internal use only and should keep its initial state
	if(flags != old)
	{
		mFlags = flags;
		if(mSim)
			mSim->postFlagChange(old, flags);
	}
}

void Sc::ConstraintCore::getForce(PxVec3& force, PxVec3& torque) const
{
	if(!mSim)
	{
		force = PxVec3(0.0f);
		torque = PxVec3(0.0f);
	}
	else
		mSim->getForce(force, torque);
}

void Sc::ConstraintCore::setBodies(RigidCore* r0v, RigidCore* r1v)
{
	if(mSim)
		mSim->setBodies(r0v, r1v);
}

void Sc::ConstraintCore::setBreakForce(PxReal linear, PxReal angular)
{
	mLinearBreakForce = linear;
	mAngularBreakForce = angular;

	if(mSim)
		mSim->setBreakForceLL(linear, angular);
}

void Sc::ConstraintCore::setMinResponseThreshold(PxReal threshold)
{
	mMinResponseThreshold = threshold;

	if(mSim)
		mSim->setMinResponseThresholdLL(threshold);
}

PxConstraint* Sc::ConstraintCore::getPxConstraint()
{
	return gOffsetTable.convertScConstraint2Px(this);
}

const PxConstraint* Sc::ConstraintCore::getPxConstraint() const
{
	return gOffsetTable.convertScConstraint2Px(this);
}

void Sc::ConstraintCore::breakApart()
{
	// TODO: probably want to do something with the interaction here
	// as well as remove the constraint from LL.

	mFlags |= PxConstraintFlag::eBROKEN;
}

