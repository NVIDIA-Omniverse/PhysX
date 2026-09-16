// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_REVOLUTE_JOINT_H
#define EXT_REVOLUTE_JOINT_H

#include "extensions/PxRevoluteJoint.h"

#include "ExtJoint.h"
#include "foundation/PxIntrinsics.h"
#include "CmUtils.h"

namespace physx
{
struct PxRevoluteJointGeneratedValues;

namespace Ext
{
	struct RevoluteJointData : public JointData
	{
		PxReal					driveVelocity;
		PxReal					driveForceLimit;
		PxReal					driveGearRatio;

		PxJointAngularLimitPair	limit;
							
		PxRevoluteJointFlags	jointFlags;
//	private:	// PT: must be public for a benchmark
		RevoluteJointData(const PxJointAngularLimitPair& pair) : limit(pair)	{}
	};

    typedef JointT<PxRevoluteJoint, RevoluteJointData, PxRevoluteJointGeneratedValues> RevoluteJointT;
    
	class RevoluteJoint : public RevoluteJointT
	{
	public:
// PX_SERIALIZATION
										RevoluteJoint(PxBaseFlags baseFlags) : RevoluteJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	RevoluteJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<RevoluteJoint>(address, context);	}
//~PX_SERIALIZATION
										RevoluteJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxRevoluteJoint
		virtual	PxReal					getAngle() const	PX_OVERRIDE;
		virtual	PxReal					getVelocity() const	PX_OVERRIDE;
		virtual	void					setLimit(const PxJointAngularLimitPair& limit)	PX_OVERRIDE;
		virtual	PxJointAngularLimitPair	getLimit()	const	PX_OVERRIDE;
		virtual	void					setDriveVelocity(PxReal velocity, bool autowake = true)	PX_OVERRIDE;
		virtual	PxReal					getDriveVelocity() const	PX_OVERRIDE;
		virtual	void					setDriveForceLimit(PxReal forceLimit)	PX_OVERRIDE;
		virtual	PxReal					getDriveForceLimit() const	PX_OVERRIDE;
		virtual	void					setDriveGearRatio(PxReal gearRatio)	PX_OVERRIDE;
		virtual	PxReal					getDriveGearRatio() const	PX_OVERRIDE;
		virtual	void					setRevoluteJointFlags(PxRevoluteJointFlags flags)	PX_OVERRIDE;
		virtual	void					setRevoluteJointFlag(PxRevoluteJointFlag::Enum flag, bool value)	PX_OVERRIDE;
		virtual	PxRevoluteJointFlags	getRevoluteJointFlags()	const	PX_OVERRIDE;
		//~PxRevoluteJoint
	
		// PxConstraintConnector
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
#if PX_SUPPORT_OMNI_PVD
		virtual void updateOmniPvdProperties() const PX_OVERRIDE;
#endif
		//~PxConstraintConnector
	};

} // namespace Ext

} // namespace physx

#endif
