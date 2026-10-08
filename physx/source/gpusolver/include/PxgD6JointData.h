// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

		static constexpr PxU32 sDriveEntryCapacity = 6;

		PxgD6Motion::Enum			motion[6];
		PxgJointLinearLimit			distanceLimit;
		PxgJointLinearLimitPair		linearLimitX;
		PxgJointLinearLimitPair		linearLimitY;
		PxgJointLinearLimitPair		linearLimitZ;
		PxgJointAngularLimitPair	twistLimit;
		PxgJointLimitCone			swingLimit;
		PxgJointLimitPyramid		pyramidSwingLimit;
		
		PxgD6JointDrive				drive[sDriveEntryCapacity];

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

		PxU8						angularDriveConfig;  // stores the angular drive config (PxD6AngularDriveConfig::Enum)

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
			mUsePyramidLimits(false),
			angularDriveConfig(0)
		{}

	};
	PX_COMPILE_TIME_ASSERT(sizeof(PxgD6JointData) <= 512);
}
#endif