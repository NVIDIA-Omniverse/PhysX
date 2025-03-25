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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_D6_JOINT_DATA_H
#define PXG_D6_JOINT_DATA_H

#include "PxConstraintDesc.h"
#include "PxgD6JointLimit.h"
#include "AlignedTransform.h"
#include "PxgSolverConstraintDesc.h"

namespace physx
{
	struct PxgJointData
	{
		PxgConstraintInvMassScale	invMassScale;
		PxTransform32				c2b[2];
	};

	struct PxgD6JointData : public PxgJointData
	{
	
	public:

		PX_CUDA_CALLABLE PxgD6JointData(){}

		PxgD6Motion::Enum			motion[6];
		PxgJointLinearLimit			distanceLimit;
		PxgJointLinearLimitPair		linearLimitX;
		PxgJointLinearLimitPair		linearLimitY;
		PxgJointLinearLimitPair		linearLimitZ;
		PxgJointAngularLimitPair	twistLimit;
		PxgJointLimitCone			swingLimit;
		PxgJointLimitPyramid		pyramidSwingLimit;
		
		PxgD6JointDrive				drive[PxgD6Drive::eCOUNT];

		PxTransform					drivePosition;
		PxVec3						driveLinearVelocity;
		PxVec3						driveAngularVelocity;

		// derived quantities

		PxU32						locked;		// bitmap of locked DOFs
		PxU32						limited;	// bitmap of limited DOFs
		PxU32						driving;	// bitmap of active drives (implies driven DOFs not locked)

		PxReal						distanceMinDist;	// distance limit minimum distance to get a good direction

		// PT: the PxD6Motion values are now shared for both kind of linear limits, so we need
		// an extra bool to know which one(s) should be actually used.
		bool						mUseDistanceLimit;
		bool						mUseNewLinearLimits;

		// PT: the swing limits can now be a cone or a pyramid, so we need
		// an extra bool to know which one(s) should be actually used.
		bool						mUseConeLimit;
		bool						mUsePyramidLimits;

		//Please don't add fields above this line since the layout must match D6JointData

		// forestall compiler complaints about not being able to generate a constructor
	private:
		PxgD6JointData(const PxgJointLinearLimit& distance,
			const PxgJointLinearLimitPair& linearX,
			const PxgJointLinearLimitPair& linearY,
			const PxgJointLinearLimitPair& linearZ,
			const PxgJointAngularLimitPair& twist,
			const PxgJointLimitCone& swing,
			const PxgJointLimitPyramid& pyramid):
			distanceLimit(distance),
			linearLimitX(linearX),
			linearLimitY(linearY),
			linearLimitZ(linearZ),
			twistLimit(twist),
			swingLimit(swing),
			pyramidSwingLimit(pyramid),
			mUseDistanceLimit(false),
			mUseNewLinearLimits(false),
			mUseConeLimit(false),
			mUsePyramidLimits(false)
		{}

	};
	PX_COMPILE_TIME_ASSERT(sizeof(PxgD6JointData) <= 512);
}
#endif