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

#include "PulleyJoint.h"
#include <assert.h>
#include "PxConstraint.h"

using namespace physx;

//TAG:solverprepshader
static PxU32 solverPrep(Px1DConstraint* constraints,
						PxVec3p& body0WorldOffset,
						PxU32 maxConstraints,
						PxConstraintInvMassScale&,
						const void* constantBlock,
						const PxTransform& bA2w,
						const PxTransform& bB2w,
						bool /*useExtendedLimits*/,
						PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{		
	PX_UNUSED(maxConstraints);

	const PulleyJoint::PulleyJointData& data = *reinterpret_cast<const PulleyJoint::PulleyJointData*>(constantBlock);

	PxTransform cA2w = bA2w.transform(data.c2b[0]);
	PxTransform cB2w = bB2w.transform(data.c2b[1]);

	cA2wOut = cA2w.p;
	cB2wOut = cB2w.p;

	body0WorldOffset = cB2w.p - bA2w.p;

	PxVec3 directionA = data.attachment0 - cA2w.p;
	PxReal distanceA = directionA.normalize();

	PxVec3 directionB = data.attachment1 - cB2w.p;
	PxReal distanceB = directionB.normalize();

	directionB *= data.ratio;

	PxReal totalDistance = distanceA + distanceB;

	// compute geometric error:
	PxReal geometricError = (data.distance - totalDistance); 

	Px1DConstraint *c = constraints;
	// constraint is breakable, so we need to output forces
	c->flags = Px1DConstraintFlag::eOUTPUT_FORCE;

	if (geometricError < 0.0f) 
	{
		c->maxImpulse = PX_MAX_F32;
		c->minImpulse = 0;
		c->geometricError = geometricError; 
	}
	else if(geometricError > 0.0f)
	{
		c->maxImpulse = 0;
		c->minImpulse = -PX_MAX_F32;
		c->geometricError = geometricError; 
	}

	c->linear0 = directionA;	c->angular0 = (cA2w.p - bA2w.p).cross(c->linear0);		
	c->linear1 = -directionB;	c->angular1 = (cB2w.p - bB2w.p).cross(c->linear1);		

	return 1;
}

static void visualize(	PxConstraintVisualizer&	viz,
						const void*				constantBlock,
						const PxTransform&		body0Transform,
						const PxTransform&		body1Transform,
						PxU32					flags)
{
	PX_UNUSED(flags);
	const PulleyJoint::PulleyJointData& data = *reinterpret_cast<const PulleyJoint::PulleyJointData*>(constantBlock);

	PxTransform cA2w = body0Transform * data.c2b[0];
	PxTransform cB2w = body1Transform * data.c2b[1];
	viz.visualizeJointFrames(cA2w, cB2w);
	viz.visualizeJointFrames(PxTransform(data.attachment0), PxTransform(data.attachment1));
}

static PxConstraintShaderTable sShaderTable = { solverPrep, visualize, PxConstraintFlag::Enum(0) };

PxConstraintSolverPrep PulleyJoint::getPrep() const { return solverPrep; }

PulleyJoint::PulleyJoint(PxPhysics& physics, PxRigidBody& body0, const PxTransform& localFrame0, const PxVec3& attachment0,
											 PxRigidBody& body1, const PxTransform& localFrame1, const PxVec3& attachment1)
{
	mConstraint = physics.createConstraint(&body0, &body1, *this, sShaderTable, sizeof(PulleyJointData));

	mBody[0] = &body0;
	mBody[1] = &body1;

	// keep these around in case the CoM gets relocated
	mLocalPose[0] = localFrame0.getNormalized();
	mLocalPose[1] = localFrame1.getNormalized();

	// the data which will be fed to the joint solver and projection shaders
	mData.attachment0 = attachment0;
	mData.attachment1 = attachment1;
	mData.distance = 1.0f;
	mData.ratio = 1.0f;
	mData.c2b[0] = body0.getCMassLocalPose().transformInv(mLocalPose[0]);
	mData.c2b[1] = body1.getCMassLocalPose().transformInv(mLocalPose[1]);
}

void PulleyJoint::release()
{
	mConstraint->release();
}

///////////////////////////////////////////// attribute accessors and mutators

void PulleyJoint::setAttachment0(const PxVec3& pos)
{
	mData.attachment0 = pos;
	mConstraint->markDirty();
}

PxVec3 PulleyJoint::getAttachment0() const
{
	return mData.attachment0;
}

void PulleyJoint::setAttachment1(const PxVec3& pos)
{
	mData.attachment1 = pos;
	mConstraint->markDirty();
}

PxVec3 PulleyJoint::getAttachment1() const
{
	return mData.attachment1;
}

void PulleyJoint::setDistance(float totalDistance)
{
	mData.distance = totalDistance;
	mConstraint->markDirty();
}

float PulleyJoint::getDistance() const
{
	return mData.distance;
}
	
void PulleyJoint::setRatio(float ratio)
{
	mData.ratio = ratio;
	mConstraint->markDirty();
}

float PulleyJoint::getRatio() const
{
	return mData.ratio;
}

///////////////////////////////////////////// PxConstraintConnector methods

void* PulleyJoint::prepareData()
{
	return &mData;
}

void PulleyJoint::onConstraintRelease()
{
	delete this;
}

void PulleyJoint::onComShift(PxU32 actor)
{
	mData.c2b[actor] = mBody[actor]->getCMassLocalPose().transformInv(mLocalPose[actor]); 
	mConstraint->markDirty();
}

void PulleyJoint::onOriginShift(const PxVec3& shift)
{
	mData.attachment0 -= shift;
	mData.attachment1 -= shift;
}

void* PulleyJoint::getExternalReference(PxU32& typeID)
{
	typeID = TYPE_ID;
	return this;
}

