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

#ifndef PXG_D6_JOINT_H
#define PXG_D6_JOINT_H

#include "PxConstraintDesc.h"
#include "PxgD6JointLimit.h"

namespace physx
{
	struct PxgD6Drive
	{
		enum Enum
		{
			eX			= 0,		//!< drive along the X-axis
			eY			= 1,		//!< drive along the Y-axis
			eZ			= 2,		//!< drive along the Z-axis
			eSWING		= 3,		//!< drive of displacement from the X-axis
			eTWIST		= 4,		//!< drive of the displacement around the X-axis
			eSLERP		= 5,		//!< drive of all three angular degrees along a SLERP-path
			eCOUNT		= 6
		};
	};

	struct PxgD6Motion
	{
		enum Enum
		{
			eLOCKED,	//!< The DOF is locked, it does not allow relative motion.
			eLIMITED,	//!< The DOF is limited, it only allows motion within a specific range.
			eFREE,		//!< The DOF is free and has its full range of motion.
			eFORCE_DWORD = 0x7fffffff
		};
	};

	struct PxgD6Axis
	{
		enum Enum
		{
			eX      = 0,	//!< motion along the X axis
			eY      = 1,	//!< motion along the Y axis
			eZ      = 2,	//!< motion along the Z axis
			eTWIST  = 3,	//!< motion around the X axis
			eSWING1 = 4,	//!< motion around the Y axis
			eSWING2 = 5,	//!< motion around the Z axis
			eCOUNT	= 6
		};
	};

	struct PxgD6JointDriveFlag
	{
		PX_CUDA_CALLABLE PxgD6JointDriveFlag(){}

		enum Enum
		{
			// IMPORTANT: the enum values need to match the ones in PxD6JointDriveFlag. Unfortunately, the GPU
			//            version just copy pasted all the D6 logic. Testing with a compile time assert would
			//            create a bit of a mess with our code hierarchy (on CPU, joints are a concept known
			//            to the PhysXExtensions library only)

			eACCELERATION	= (1 << 0),	//!< drive spring is for the acceleration at the joint (rather than the force) 
			eOUTPUT_FORCE	= (1 << 1)	// see PxD6JointDriveFlag::eOUTPUT_FORCE
		};
	};
	typedef PxFlags<PxgD6JointDriveFlag::Enum, PxU32> PxgD6JointDriveFlags;

	class PxgSpring
	{
	public:

		PxReal					stiffness;			//!< the spring strength of the drive: that is, the force proportional to the position error
		PxReal					damping;			//!< the damping strength of the drive: that is, the force proportional to the velocity error

		PX_CUDA_CALLABLE PxgSpring(PxReal stiffness_, PxReal damping_): stiffness(stiffness_), damping(damping_) {}
	};

	class PxgD6JointDrive : public PxgSpring
	{

	public:
		PxReal						forceLimit;			//!< the force limit of the drive - may be an impulse or a force depending on PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
		PxgD6JointDriveFlags		flags;				//!< the joint drive flags 


		/**
		\brief default constructor for PxD6JointDrive.
		*/

		PX_CUDA_CALLABLE PxgD6JointDrive(): PxgSpring(0,0), forceLimit(PX_MAX_F32), flags(0) {}

		/**
		\brief constructor a PxD6JointDrive.

		\param[in] driveStiffness the stiffness of the drive spring.
		\param[in] driveDamping the damping of the drive spring
		\param[in] driveForceLimit the maximum impulse or force that can be exerted by the drive
		\param[in] isAcceleration whether the drive is an acceleration drive or a force drive
		*/


		PX_CUDA_CALLABLE PxgD6JointDrive(PxReal driveStiffness, PxReal driveDamping, PxReal driveForceLimit, bool isAcceleration = false)
		: PxgSpring(driveStiffness, driveDamping)
		, forceLimit(driveForceLimit)
		, flags(isAcceleration?(PxU32)PxgD6JointDriveFlag::eACCELERATION : 0) 
		{}


		/** 
		\brief returns true if the drive is valid
		*/

		bool isValid() const
		{
			return PxIsFinite(stiffness) && stiffness>=0 &&
				   PxIsFinite(damping) && damping >=0 &&
				   PxIsFinite(forceLimit) && forceLimit >=0;
		}
	};
}

#endif