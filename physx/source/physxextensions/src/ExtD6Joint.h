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

#ifndef EXT_D6_JOINT_H
#define EXT_D6_JOINT_H

#include "extensions/PxD6Joint.h"

#include "ExtJoint.h"

namespace physx
{
struct PxD6JointGeneratedValues;
namespace Ext
{
	struct D6JointData : public JointData
	{
		PxD6Motion::Enum		motion[6]; 
		PxJointLinearLimit		distanceLimit;
		PxJointLinearLimitPair	linearLimitX;
		PxJointLinearLimitPair	linearLimitY;
		PxJointLinearLimitPair	linearLimitZ;
		PxJointAngularLimitPair	twistLimit;
		PxJointLimitCone		swingLimit;
		PxJointLimitPyramid		pyramidSwingLimit;

		PxD6JointDrive			drive[PxD6Drive::eCOUNT];

		PxTransform				drivePosition;
		PxVec3					driveLinearVelocity;
		PxVec3					driveAngularVelocity;

		// derived quantities

		PxU32					locked;		// bitmap of locked DOFs
		PxU32					limited;	// bitmap of limited DOFs
		PxU32					driving;	// bitmap of active drives (implies driven DOFs not locked)

		PxReal					distanceMinDist;	// distance limit minimum distance to get a good direction

		// PT: the PxD6Motion values are now shared for both kind of linear limits, so we need
		// an extra bool to know which one(s) should be actually used.
		bool					mUseDistanceLimit;
		bool					mUseNewLinearLimits;

		// PT: the swing limits can now be a cone or a pyramid, so we need
		// an extra bool to know which one(s) should be actually used.
		bool					mUseConeLimit;
		bool					mUsePyramidLimits;

	private:
		D6JointData(const PxJointLinearLimit& distance,
					const PxJointLinearLimitPair& linearX,
					const PxJointLinearLimitPair& linearY,
					const PxJointLinearLimitPair& linearZ,
					const PxJointAngularLimitPair& twist,
					const PxJointLimitCone& swing,
					const PxJointLimitPyramid& pyramid) :
		distanceLimit		(distance),
		linearLimitX		(linearX),
		linearLimitY		(linearY),
		linearLimitZ		(linearZ),
		twistLimit			(twist),
		swingLimit			(swing),
		pyramidSwingLimit	(pyramid),
		mUseDistanceLimit	(false),
		mUseNewLinearLimits	(false),
		mUseConeLimit		(false),
		mUsePyramidLimits	(false)
		{}
	};

	typedef JointT<PxD6Joint, D6JointData, PxD6JointGeneratedValues> D6JointT;
    
    class D6Joint : public D6JointT
	{
	public:
// PX_SERIALIZATION
										D6Joint(PxBaseFlags baseFlags) : D6JointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	D6Joint*				createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<D6Joint>(address, context);	}
		static	void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
										D6Joint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxD6Joint
		virtual	void					setMotion(PxD6Axis::Enum index, PxD6Motion::Enum t)	PX_OVERRIDE;
		virtual	PxD6Motion::Enum		getMotion(PxD6Axis::Enum index)	const	PX_OVERRIDE;
		virtual	PxReal					getTwistAngle()		const	PX_OVERRIDE;
		virtual	PxReal					getSwingYAngle()	const	PX_OVERRIDE;
		virtual	PxReal					getSwingZAngle()	const	PX_OVERRIDE;
		virtual	void					setDistanceLimit(const PxJointLinearLimit& l)	PX_OVERRIDE;
		virtual	PxJointLinearLimit		getDistanceLimit()	const	PX_OVERRIDE;
		virtual void					setLinearLimit(PxD6Axis::Enum axis, const PxJointLinearLimitPair& limit)	PX_OVERRIDE;
		virtual PxJointLinearLimitPair	getLinearLimit(PxD6Axis::Enum axis)	const	PX_OVERRIDE;
		virtual	void					setTwistLimit(const PxJointAngularLimitPair& l)	PX_OVERRIDE;
		virtual	PxJointAngularLimitPair	getTwistLimit()	const	PX_OVERRIDE;
		virtual	void					setSwingLimit(const PxJointLimitCone& l)	PX_OVERRIDE;
		virtual	PxJointLimitCone		getSwingLimit()	const	PX_OVERRIDE;
		virtual	void					setPyramidSwingLimit(const PxJointLimitPyramid& limit)	PX_OVERRIDE;
		virtual	PxJointLimitPyramid		getPyramidSwingLimit()	const	PX_OVERRIDE;
		virtual	void					setDrive(PxD6Drive::Enum index, const PxD6JointDrive& d)	PX_OVERRIDE;
		virtual	PxD6JointDrive			getDrive(PxD6Drive::Enum index)	const	PX_OVERRIDE;
		virtual	void					setDrivePosition(const PxTransform& pose, bool autowake = true)	PX_OVERRIDE;
		virtual	PxTransform				getDrivePosition()	const	PX_OVERRIDE;
		virtual	void					setDriveVelocity(const PxVec3& linear, const PxVec3& angular, bool autowake = true)	PX_OVERRIDE;
		virtual	void					getDriveVelocity(PxVec3& linear, PxVec3& angular)	const	PX_OVERRIDE;						
		//~PxD6Joint

		// PxConstraintConnector
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
		virtual	void*					prepareData()	PX_OVERRIDE;
#if PX_SUPPORT_OMNI_PVD
		virtual void updateOmniPvdProperties() const PX_OVERRIDE;
#endif
		//~PxConstraintConnector

	private:
		bool active(const PxD6Drive::Enum index) const
		{
			const PxD6JointDrive& d = data().drive[index];
			return d.stiffness!=0 || d.damping != 0;
		}

		bool	mRecomputeMotion;
		bool	mPadding[3];	// PT: padding from prev bool
	};

} // namespace Ext

} // namespace physx

#endif
