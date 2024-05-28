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

#ifndef PULLEY_JOINT_H
#define PULLEY_JOINT_H

#include "PxPhysicsAPI.h"

// a pulley joint constrains two actors such that the sum of their distances from their respective anchor points at their attachment points 
// is a fixed value (the parameter 'distance'). Only dynamic actors are supported.
//
// The constraint equation is as follows:
//
// |anchor0 - attachment0| + |anchor1 - attachment1| * ratio = distance
// 
// where 'ratio' provides mechanical advantage.
//
// The above equation results in a singularity when the anchor point is coincident with the attachment point; for simplicity
// the constraint does not attempt to handle this case robustly.

class PulleyJoint : public physx::PxConstraintConnector
{
public:

	static const physx::PxU32 TYPE_ID = physx::PxConcreteType::eFIRST_USER_EXTENSION;

	PulleyJoint(physx::PxPhysics& physics, 
				physx::PxRigidBody& body0, const physx::PxTransform& localFrame0, const physx::PxVec3& attachment0,
			    physx::PxRigidBody& body1, const physx::PxTransform& localFrame1, const physx::PxVec3& attachment1);

	void release();

	// attribute accessor and mutators

	void			setAttachment0(const physx::PxVec3& pos);
	physx::PxVec3	getAttachment0() const;

	void			setAttachment1(const physx::PxVec3& pos);
	physx::PxVec3	getAttachment1() const;

	void			setDistance(physx::PxReal totalDistance);
	physx::PxReal	getDistance() const;
	
	void			setRatio(physx::PxReal ratio);
	physx::PxReal	getRatio() const;

	// PxConstraintConnector boilerplate

	void*			prepareData();
	void			onConstraintRelease();
	void			onComShift(physx::PxU32 actor);
	void			onOriginShift(const physx::PxVec3& shift);
	void*			getExternalReference(physx::PxU32& typeID);

	bool			updatePvdProperties(physx::pvdsdk::PvdDataStream&,
										const physx::PxConstraint*,
										physx::PxPvdUpdateType::Enum) const { return true; }
	void			updateOmniPvdProperties() const { }
	physx::PxBase*	getSerializable() { return NULL; }

	virtual physx::PxConstraintSolverPrep getPrep() const;

	virtual const void* getConstantBlock() const { return &mData; }

	struct PulleyJointData
	{
		physx::PxTransform c2b[2];
	
		physx::PxVec3 attachment0;
		physx::PxVec3 attachment1;

		physx::PxReal distance;
		physx::PxReal ratio;
		physx::PxReal tolerance;
	};

	physx::PxRigidBody*		mBody[2];
	physx::PxTransform		mLocalPose[2];

	physx::PxConstraint*	mConstraint;
	PulleyJointData			mData;

	~PulleyJoint() {}
};

#endif
